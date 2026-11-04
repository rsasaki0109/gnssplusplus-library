#!/usr/bin/env node
// Runs the browser app's demo pipeline against the WebAssembly build in Node,
// through the same engine and command builders the web GUI uses:
//
//   node wasm/test/smoke.mjs build-wasm/wasm/dist
//
// RTCM3 -> RINEX (gnss_convert), RINEX info, visibility (gnss_visibility),
// kinematic RTK (gnss_solve), SPP and static PPP on the repository fixtures,
// with accuracy checks against the fixture reference trajectory.

import assert from 'node:assert/strict';
import { readFileSync } from 'node:fs';
import { join, resolve } from 'node:path';
import { fileURLToPath, pathToFileURL } from 'node:url';

import { runTool } from '../../web/js/engine.js';
import {
  buildConvertCommand, buildInfoCommand, buildPostCommand, buildVisibilityCommand,
} from '../../web/js/commands.js';
import { parseSolution, parseVisibilityCsv, Q } from '../../web/js/formats.js';
import { errorSummary, isFixQuality, referenceErrors } from '../../web/js/analysis.js';

const ROOT = fileURLToPath(new URL('../../', import.meta.url));

const FIXTURES = {
  'rover.rtcm': 'tests/fixtures/realtime_stress/rover.rtcm',
  'base.rtcm': 'tests/fixtures/realtime_stress/base.rtcm',
  'reference.csv': 'tests/fixtures/realtime_stress/reference.csv',
  'synthetic_ppp.obs': 'demo/fixtures/synthetic_ppp.obs',
  'synthetic_ppp.sp3': 'demo/fixtures/synthetic_ppp.sp3',
  'synthetic_ppp.clk': 'demo/fixtures/synthetic_ppp.clk',
};

/**
 * factoryFor(tool) -> Emscripten module factory. Exported so the pipeline can
 * also be driven by another backend (e.g. a native shim during development).
 */
export async function runSmoke(factoryFor, log = console.log) {
  const files = new Map(Object.entries(FIXTURES).map(([name, path]) => [name, readFileSync(join(ROOT, path))]));
  const text = (name) => new TextDecoder().decode(files.get(name));

  async function run(cmd) {
    const stdout = [];
    const stderr = [];
    const result = await runTool(await factoryFor(cmd.tool), {
      args: cmd.args,
      inputs: cmd.inputs.map((name) => ({ name, data: files.get(name) })),
      onStdout: (line) => stdout.push(line),
      onStderr: (line) => stderr.push(line),
    });
    // Emscripten's Node quit handler records the tool's exit status here.
    process.exitCode = 0;
    log(`$ ${cmd.commandLine}  -> exit ${result.exitCode}, ${result.elapsedMs.toFixed(0)} ms`);
    assert.equal(result.exitCode, 0, `${cmd.commandLine}\n${[...stdout, ...stderr].join('\n')}`);
    for (const out of result.outputs) files.set(out.name, out.data);
    for (const name of cmd.outputs) assert.ok(files.has(name), `${cmd.tool} did not write ${name}`);
    return { ...result, stdout: stdout.join('\n') };
  }

  const rover = await run(buildConvertCommand({
    input: 'rover.rtcm', format: 'rtcm', gpsWeek: 2324, obsOut: 'rover.obs', quiet: true,
  }));
  assert.match(rover.stdout, /exported_obs_epochs=201\b/);
  await run(buildConvertCommand({
    input: 'base.rtcm', format: 'rtcm', gpsWeek: 2324, obsOut: 'base.obs', navOut: 'base.nav', quiet: true,
  }));
  // The 1005 ARP becomes the base header position (no --base-ecef needed).
  assert.match(text('base.obs'), /-3961905\.0275\s+3348993\.7526\s+3698211\.7831\s+APPROX POSITION XYZ/);

  const info = await run(buildInfoCommand({ files: ['rover.obs', 'base.nav'], countRecords: true }));
  assert.match(info.stdout, /epoch count: 201/);
  assert.match(info.stdout, /ephemeris count: \d+/);

  // Observation viewer input: the rover header has no position, so the tool seeds it from SPP.
  await run(buildVisibilityCommand({ obs: 'rover.obs', nav: 'base.nav' }));
  const vis = parseVisibilityCsv('rover_vis.csv', text('rover_vis.csv'));
  const visSummary = JSON.parse(text('rover_vis_summary.json'));
  log(`Visibility: ${vis.n} rows, ${vis.sats.length} satellites, position ${visSummary.receiver_position_source}`);
  assert.match(visSummary.receiver_position_source, /^spp-epoch-\d+$/);
  assert.equal(visSummary.epochs_processed, 201);
  assert.equal(vis.n, visSummary.rows_written);
  assert.ok(vis.sats.length >= 8);
  assert.ok(vis.lli.some((v) => v & 1), 'expected LLI slip flags in the rover data');

  const reference = parseSolution('reference.csv', text('reference.csv'));
  const origin = [reference.lat[0], reference.lon[0], reference.h[0]];

  await run(buildPostCommand({
    mode: 'kinematic', rover: 'rover.obs', base: 'base.obs', nav: 'base.nav', out: 'rover_rtk.pos', kml: true, verbose: true,
  }));
  const rtk = parseSolution('rover_rtk.pos', text('rover_rtk.pos'));
  const fixCount = rtk.q.filter((q) => q === Q.FIX).length;
  const fixErr = errorSummary(referenceErrors(rtk, reference, origin), (i) => isFixQuality(rtk.q[i]));
  log(`RTK: ${rtk.n} epochs, ${fixCount} FIX, FIX horizontal RMS ${fixErr.horizontalRms.toFixed(3)} m`);
  assert.equal(rtk.n, 201);
  assert.ok(fixCount >= 180, `expected >= 180 FIX epochs, got ${fixCount}`);
  assert.ok(fixErr.horizontalRms < 0.15, `FIX horizontal RMS ${fixErr.horizontalRms} m`);
  assert.match(text('rover_rtk.kml'), /<kml/);

  await run(buildPostCommand({ mode: 'single', rover: 'rover.obs', nav: 'base.nav', out: 'rover_spp.pos' }));
  const spp = parseSolution('rover_spp.pos', text('rover_spp.pos'));
  const sppErr = errorSummary(referenceErrors(spp, reference, origin));
  log(`SPP: ${spp.n} epochs, horizontal RMS ${sppErr.horizontalRms.toFixed(3)} m`);
  assert.equal(spp.n, 201);
  assert.ok(sppErr.horizontalRms < 5, `SPP horizontal RMS ${sppErr.horizontalRms} m`);
  assert.ok(JSON.parse(text('rover_spp_summary.json')));

  await run(buildPostCommand({
    mode: 'ppp-static', rover: 'synthetic_ppp.obs', sp3: 'synthetic_ppp.sp3', clk: 'synthetic_ppp.clk', out: 'ppp.pos',
  }));
  const ppp = parseSolution('ppp.pos', text('ppp.pos'));
  log(`PPP: ${ppp.n} epochs, first ${ppp.lat[0].toFixed(6)}, ${ppp.lon[0].toFixed(6)}`);
  assert.ok(ppp.n >= 1);
  // The synthetic receiver sits at 35N 139E.
  assert.ok(Math.abs(ppp.lat[ppp.n - 1] - 35) < 1e-4 && Math.abs(ppp.lon[ppp.n - 1] - 139) < 1e-4);

  // Argument errors come back as a non-zero exit code, not a crash.
  const bad = await runTool(await factoryFor('gnss_solve'), { args: ['--no-such-option'] });
  process.exitCode = 0;
  assert.notEqual(bad.exitCode, 0);
  log('wasm smoke test passed');
}

if (process.argv[1] && resolve(process.argv[1]) === fileURLToPath(import.meta.url)) {
  const dist = resolve(process.argv[2] || 'build-wasm/wasm/dist');
  await runSmoke(async (tool) => (await import(pathToFileURL(join(dist, `${tool}.js`)).href)).default);
}
