#!/usr/bin/env node
// Runs the browser app's demo pipeline against the WebAssembly build in Node,
// through the same engine and command builders the web GUI uses:
//
//   node wasm/test/smoke.mjs build-wasm/wasm/dist [--qzss <samples dir>]
//
// RTCM3 -> RINEX (gnss_convert), RINEX info, visibility (gnss_visibility),
// kinematic RTK (gnss_solve), SPP and static PPP on the repository fixtures,
// with accuracy checks against the fixture reference trajectory. With
// --qzss, also MADOCA-PPP and CLAS PPP from QZSS L6 logs on the public
// MADOCALIB / CLASLIB samples (QZSS_SAMPLES below), checked against the
// native gnss ppp results of the same commands.

import assert from 'node:assert/strict';
import { readFileSync } from 'node:fs';
import { basename, join, resolve } from 'node:path';
import { fileURLToPath, pathToFileURL } from 'node:url';

import { runTool } from '../../web/js/engine.js';
import {
  buildConvertCommand, buildInfoCommand, buildPostCommand, buildVisibilityCommand,
} from '../../web/js/commands.js';
import { parseSolution, parseVisibilityCsv, Q } from '../../web/js/formats.js';
import { errorSummary, isFixQuality, referenceErrors } from '../../web/js/analysis.js';
import { exportOptionsToml } from '../../web/js/options.js';

const ROOT = fileURLToPath(new URL('../../', import.meta.url));

const FIXTURES = {
  'rover.rtcm': 'tests/fixtures/realtime_stress/rover.rtcm',
  'base.rtcm': 'tests/fixtures/realtime_stress/base.rtcm',
  'reference.csv': 'tests/fixtures/realtime_stress/reference.csv',
  'imu.csv': 'tests/fixtures/realtime_stress/imu.csv',
  'synthetic_ppp.obs': 'demo/fixtures/synthetic_ppp.obs',
  'synthetic_ppp.sp3': 'demo/fixtures/synthetic_ppp.sp3',
  'synthetic_ppp.clk': 'demo/fixtures/synthetic_ppp.clk',
};

// Public QZSS L6 samples, relative to the --qzss directory: MADOCALIB
// 0089f7dc (the CI MADOCA matrix pin; IGS station MIZU, 2025-04-01 00:00-01:00)
// and CLASLIB 4320e844 (2019-08-27). The Docs workflow checks out exactly
// these files.
export const QZSS_SAMPLES = {
  madocalib: [
    'sample_data/data/rinex/MIZU00JPN_R_20250910000_01D_30S_MO.rnx',
    'sample_data/data/rinex/BRDM00DLR_S_20250910000_01D_MN.rnx',
    'sample_data/data/igs20.atx',
    'sample_data/data/l6_is-qzss-mdc-004/2025/091/2025091A.204.l6',
    'sample_data/data/l6_is-qzss-mdc-004/2025/091/2025091A.206.l6',
    'sample_data/data/l6/2025/091/2025091A.200.l6',
    'sample_data/data/l6/2025/091/2025091A.201.l6',
  ],
  claslib: ['data/0627239Q.obs', 'data/sept_2019239.nav', 'data/2019239Q.l6'],
};

/**
 * factoryFor(tool) -> Emscripten module factory. Exported so the pipeline can
 * also be driven by another backend (e.g. a native shim during development).
 * options.qzss: directory holding the QZSS_SAMPLES trees (madocalib/, claslib/).
 */
export async function runSmoke(factoryFor, log = console.log, options = {}) {
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

  // "Save options" TOML through --config gives the same solution as the flags.
  const optionSet = {
    mode: 'static', rover: 'rover.obs', base: 'base.obs', nav: 'base.nav', preset: 'survey', ratio: '2.5',
    elevationMask: '12', kml: false, baseEcef: ['-3961905.0275', '3348993.7526', '3698211.7831'],
    extra: '--min-ar-sats 6',
  };
  await run(buildPostCommand({ ...optionSet, out: 'flags.pos' }));
  files.set('options.toml', new TextEncoder().encode(exportOptionsToml(optionSet)));
  await run(buildPostCommand({
    mode: 'static', rover: 'rover.obs', base: 'base.obs', nav: 'base.nav', kml: false, out: 'config.pos',
    extra: '--config options.toml', workspaceNames: [...files.keys()],
  }));
  assert.equal(text('config.pos'), text('flags.pos'), '--config TOML and flags differ');
  log('Options TOML: --config run identical to the flags run');

  // RTK + IMU (gnss_fuse, tight coupling, PPC Tokyo lever arm).
  await run(buildPostCommand({
    mode: 'rtk-imu', rover: 'rover.obs', base: 'base.obs', nav: 'base.nav', imu: 'imu.csv',
    leverArm: ['0.31', '0', '0.55'], tightCoupling: true, out: 'rover_fused.pos', kml: true,
  }));
  const fused = parseSolution('rover_fused.pos', text('rover_fused.pos'));
  const fusedRtk = parseSolution('rover_fused_rtk.pos', text('rover_fused_rtk.pos'));
  const fusedErr = errorSummary(referenceErrors(fused, reference, origin));
  const fusedRtkFix = fusedRtk.q.filter((q) => q === Q.FIX).length;
  log(`RTK + IMU: ${fused.n} fused epochs, horizontal RMS ${fusedErr.horizontalRms.toFixed(3)} m; RTK ${fusedRtk.n} epochs, ${fusedRtkFix} FIX`);
  assert.ok(fused.n >= 180, `fused epochs: ${fused.n}`);
  assert.ok(fusedErr.horizontalRms < 0.3, `fused horizontal RMS ${fusedErr.horizontalRms} m`);
  assert.equal(fusedRtk.n, 201);
  assert.ok(fusedRtkFix >= 180, `RTK FIX under fusion: ${fusedRtkFix}`);
  // Fused epochs carry no satellite count; the KML must still draw all of them.
  const fusedKml = text('rover_fused.kml').match(/<coordinates>([\s\S]*?)<\/coordinates>/g) || [];
  const fusedKmlPoints = fusedKml.join('\n').split('\n').filter((line) => /^-?\d/.test(line)).length;
  assert.equal(fusedKmlPoints, fused.n, `fused KML points: ${fusedKmlPoints}`);

  // Batch FGO (gnss_fgo, Eigen backend, real-data-float preset).
  await run(buildPostCommand({
    mode: 'fgo', rover: 'rover.obs', base: 'base.obs', nav: 'base.nav', fgoPreset: 'real-data-float', out: 'rover_fgo.pos',
  }));
  const fgo = parseSolution('rover_fgo.pos', text('rover_fgo.pos'));
  const fgoErr = errorSummary(referenceErrors(fgo, reference, origin));
  const fgoSummary = JSON.parse(text('rover_fgo_summary.json'));
  log(`FGO: ${fgo.n} epochs, horizontal RMS ${fgoErr.horizontalRms.toFixed(3)} m`);
  assert.equal(fgo.n, 201);
  assert.ok(fgoErr.horizontalRms < 0.3, `FGO horizontal RMS ${fgoErr.horizontalRms} m`);
  assert.ok(fgoSummary && typeof fgoSummary === 'object', 'FGO summary JSON');

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

  if (options.qzss) {
    for (const [tree, paths] of Object.entries(QZSS_SAMPLES)) {
      for (const path of paths) files.set(basename(path), readFileSync(join(options.qzss, tree, path)));
    }
    const ecefDistance = (s, xyz) => Math.hypot(s.x[s.n - 1] - xyz[0], s.y[s.n - 1] - xyz[1], s.z[s.n - 1] - xyz[2]);

    // MADOCA PPP-AR with L6D ionosphere, 1 h kinematic (the matrix's pppar-ion
    // case). Native gnss ppp: 91 PPP FIX / 27 float, final position below.
    const madocaOpts = {
      mode: 'ppp-kinematic', rover: 'MIZU00JPN_R_20250910000_01D_30S_MO.rnx', nav: 'BRDM00DLR_S_20250910000_01D_MN.rnx',
      corrections: 'madoca', l6e: ['2025091A.204.l6', '2025091A.206.l6'], l6d: ['2025091A.200.l6', '2025091A.201.l6'],
      antex: 'igs20.atx', enableAr: true, maxEpochs: '120', out: 'mizu_madoca.pos',
    };
    await run(buildPostCommand(madocaOpts));
    const madoca = parseSolution('mizu_madoca.pos', text('mizu_madoca.pos'));
    const madocaFix = madoca.q.filter((q) => q === Q.PPP_FIX).length;
    const madocaDelta = ecefDistance(madoca, [-3857171.260557, 3108692.773214, 4004040.105784]);
    log(`MADOCA-PPP: ${madoca.n} epochs, ${madocaFix} PPP FIX (native 91), final ${(madocaDelta * 1000).toFixed(2)} mm from native`);
    assert.equal(madoca.n, 118);
    assert.ok(madocaFix >= 85, `MADOCA PPP FIX epochs: ${madocaFix}`);
    assert.ok(madocaDelta < 0.05, `MADOCA final position ${madocaDelta} m from native`);
    files.set('madoca.toml', new TextEncoder().encode(exportOptionsToml(madocaOpts)));
    // Options from the TOML (AR, ratio, convergence), inputs on the command line.
    await run(buildPostCommand({
      ...madocaOpts, enableAr: false, out: 'mizu_madoca_config.pos',
      extra: '--config madoca.toml', workspaceNames: [...files.keys()],
    }));
    assert.equal(text('mizu_madoca_config.pos'), text('mizu_madoca.pos'), 'MADOCA --config TOML and flags differ');

    // CLAS from the raw L6 log (native Compact SSR decoder), 300 epochs static.
    await run(buildPostCommand({
      mode: 'ppp-static', rover: '0627239Q.obs', nav: 'sept_2019239.nav', corrections: 'clas', clasL6: '2019239Q.l6',
      maxEpochs: '300', out: 'clas.pos',
    }));
    const clas = parseSolution('clas.pos', text('clas.pos'));
    const clasDelta = ecefDistance(clas, [-3957234.628684, 3310366.314382, 3737530.126659]);
    log(`CLAS: ${clas.n} epochs, final ${(clasDelta * 1000).toFixed(2)} mm from native`);
    assert.equal(clas.n, 300);
    assert.ok(clasDelta < 0.05, `CLAS final position ${clasDelta} m from native`);
  }

  // Argument errors come back as a non-zero exit code, not a crash.
  const bad = await runTool(await factoryFor('gnss_solve'), { args: ['--no-such-option'] });
  process.exitCode = 0;
  assert.notEqual(bad.exitCode, 0);
  log('wasm smoke test passed');
}

if (process.argv[1] && resolve(process.argv[1]) === fileURLToPath(import.meta.url)) {
  const args = process.argv.slice(2);
  const qzssAt = args.indexOf('--qzss');
  const qzss = qzssAt >= 0 ? resolve(args.splice(qzssAt, 2)[1]) : null;
  const dist = resolve(args[0] || 'build-wasm/wasm/dist');
  await runSmoke(async (tool) => (await import(pathToFileURL(join(dist, `${tool}.js`)).href)).default, console.log, { qzss });
}
