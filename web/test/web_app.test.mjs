// Unit tests for the browser app's pure modules (no wasm needed):
//   node --test web/test/web_app.test.mjs
import assert from 'node:assert/strict';
import { readFileSync } from 'node:fs';
import { test } from 'node:test';

import {
  buildConvertCommand, buildInfoCommand, buildPostCommand, buildVisibilityCommand, defaultPosName, splitArgs,
} from '../js/commands.js';
import { parseSolution, parseVisibilityCsv, Q, rinexObsTimeSpan, sniffKind } from '../js/formats.js';
import { exportOptionsToml, extraArgsToEntries, importOptionsToml, parseFlatToml } from '../js/options.js';
import {
  ERROR_BINS_M, errorClasses, errorClassLabels, errorSummary, nearestIndex, referenceErrors, solutionSummary, toEnu,
} from '../js/analysis.js';
import { addWorkerfsReadAhead } from '../js/engine.js';
import { describeStored } from '../js/store.js';
import { Workspace } from '../js/workspace.js';
import { calendarToGpsSeconds, ecefToLlh, formatGpst, gpsSeconds, llhToEcef } from '../js/geo.js';
import { chooseZoom, enuPlane, TILE_SOURCES, tileCorner, tilesForView, tileUrl, tileXY } from '../js/tiles.js';

const root = new URL('../../', import.meta.url);
const fixture = (path) => readFileSync(new URL(path, root));

// Rows from `gnss solve` / `gnss spp` runs on tests/fixtures/realtime_stress.
const SOLVE_POS = `% LibGNSS++ Position Solution
% GPS_Week GPS_TOW X(m) Y(m) Z(m) Lat(deg) Lon(deg) Height(m) Status NumSat PDOP Ratio Baseline(m) RTKIter RTKObs
2324 187470.000 -3961767.8640 3349009.2099 3698310.1615 35.667552644 139.791093447 40.1859 3 7 2.00 0.0 169.5024 4 24
2324 187470.200 -3961767.9041 3349009.2082 3698310.1877 35.667552681 139.791093748 40.2253 4 7 2.00 12.5 169.4850 2 24
`;
const SPP_POS = `% LibGNSS++ Position Solution
% Format: pos
% Columns: GPS_Week GPS_TOW X(m) Y(m) Z(m) Lat(deg) Lon(deg) Height(m) Status Satellites PDOP Ratio FixedAmbiguities Iterations
2324 187470.000000 -3961773.251602 3349013.914165 3698315.752960 35.667556 139.791092 49.256217 1 7 2.780225 0.000000 0 3
`;
const SOLVE_LLH = `% GPS_Week GPS_TOW Lat(deg) Lon(deg) Height(m) Status
2324 187470.000 35.667552644 139.791093447 40.1859 3
`;

test('geodesy round trip and GPST calendar', () => {
  const xyz = llhToEcef(35.667552644, 139.791093447, 40.1859);
  const llh = ecefToLlh(...xyz);
  assert.ok(Math.abs(llh[0] - 35.667552644) < 1e-9);
  assert.ok(Math.abs(llh[1] - 139.791093447) < 1e-9);
  assert.ok(Math.abs(llh[2] - 40.1859) < 1e-4);
  // 2024-07-23 04:04:30 GPST is week 2324, tow 187470 (fixture header / first epoch).
  assert.equal(calendarToGpsSeconds(2024, 7, 23, 4, 4, 30), gpsSeconds(2324, 187470));
  assert.equal(formatGpst(gpsSeconds(2324, 187470.2), 1), '2024/07/23 04:04:30.2');
});

test('sniffKind recognises RINEX, SP3, receiver logs and solutions', () => {
  const enc = (s) => new TextEncoder().encode(s);
  const obs = '     3.04           OBSERVATION DATA    M: Mixed            RINEX VERSION / TYPE\n';
  const nav = '     3.04           N: GNSS NAV DATA    M: Mixed            RINEX VERSION / TYPE\n';
  const clk = '     3.00           C                   G                   RINEX VERSION / TYPE\n';
  assert.equal(sniffKind('a.rnx', enc(obs)), 'obs');
  assert.equal(sniffKind('a.rnx', enc(nav)), 'nav');
  assert.equal(sniffKind('a.clk', enc(clk)), 'clk');
  assert.equal(sniffKind('x.sp3', enc('#dP2024  7 23  0  0  0.00000000     96 ORBIT IGS20 HLM  IGS\n')), 'sp3');
  assert.equal(sniffKind('rover.rtcm', fixture('tests/fixtures/realtime_stress/rover.rtcm').subarray(0, 64)), 'rtcm');
  assert.equal(sniffKind('log.bin', new Uint8Array([0xb5, 0x62, 1, 7])), 'ubx');
  assert.equal(sniffKind('rtk.pos', enc(SOLVE_POS)), 'pos');
  assert.equal(sniffKind('reference.csv', enc('GPS TOW (s),GPS Week\n')), 'csv');
  assert.equal(sniffKind('demo.obs', fixture('demo/fixtures/synthetic_ppp.obs').subarray(0, 4096)), 'obs');
  assert.equal(sniffKind('demo.sp3', fixture('demo/fixtures/synthetic_ppp.sp3').subarray(0, 4096)), 'sp3');
  assert.equal(sniffKind('demo.clk', fixture('demo/fixtures/synthetic_ppp.clk').subarray(0, 4096)), 'clk');
});

test('rinexObsTimeSpan reads TIME OF FIRST / LAST OBS', () => {
  const header = [
    '  2024     7    23     4     4   30.0000000     GPS         TIME OF FIRST OBS   ',
    '  2024     7    23     4     5   10.0000000     GPS         TIME OF LAST OBS    ',
  ].join('\n');
  const span = rinexObsTimeSpan(header);
  assert.equal(span.first, gpsSeconds(2324, 187470));
  assert.equal(span.last - span.first, 40);
});

test('parseSolution reads libgnss++ pos variants by column name', () => {
  const rtk = parseSolution('rtk.pos', SOLVE_POS);
  assert.equal(rtk.n, 2);
  assert.deepEqual([...rtk.q], [Q.FLOAT, Q.FIX]);
  assert.equal(rtk.ns[0], 7);
  assert.equal(rtk.ratio[1], 12.5);
  assert.equal(rtk.pdop[0], 2.0);
  // Continuous GPST seconds (~1.4e9) keep sub-microsecond resolution.
  assert.ok(Math.abs(rtk.t[1] - rtk.t[0] - 0.2) < 1e-6);

  const spp = parseSolution('spp.pos', SPP_POS);
  assert.equal(spp.q[0], Q.SINGLE);
  assert.ok(Math.abs(spp.x[0] - -3961773.251602) < 1e-6);

  const llh = parseSolution('rtk_llh.pos', SOLVE_LLH);
  assert.equal(llh.q[0], Q.FLOAT);
  const xyz = llhToEcef(35.667552644, 139.791093447, 40.1859);
  assert.ok(Math.abs(llh.x[0] - xyz[0]) < 1e-6);
});

test('parseSolution reads RTKLIB pos and reference CSV fixtures', () => {
  const rtklib = parseSolution('rtklib.pos', fixture('tests/fixtures/rtk/rtklib_rtk_result.pos').toString());
  assert.ok(rtklib.n > 100);
  assert.equal(rtklib.t[0], gpsSeconds(1316, 518400));
  assert.equal(rtklib.q[0], Q.FIX);
  assert.equal(rtklib.ns[0], 7);
  assert.equal(rtklib.ratio[0], 25.0);
  assert.ok(Math.abs(rtklib.lat[0] - 35.132064255) < 1e-12);

  const utc = parseSolution('utc.pos', '%  UTC                   latitude(deg) longitude(deg)  height(m)   Q  ns\n'
    + '2024/07/23 04:04:12.000   35.0 139.0 40.0   5   8\n');
  assert.equal(utc.t[0], gpsSeconds(2324, 187470));

  const ref = parseSolution('reference.csv', fixture('tests/fixtures/realtime_stress/reference.csv').toString());
  assert.ok(ref.n >= 200);
  assert.equal(ref.t[0], gpsSeconds(2324, 187470));
  assert.ok(Math.abs(ref.lat[0] - 35.66755248) < 1e-9);

  assert.throws(() => parseSolution('x.txt', 'hello world\n'), /not a libgnss\+\+/);
});

test('reference errors and statistics', () => {
  const ref = parseSolution('reference.csv', fixture('tests/fixtures/realtime_stress/reference.csv').toString());
  const origin = [ref.lat[0], ref.lon[0], ref.h[0]];
  // The reference against itself has zero error at every epoch.
  const self = referenceErrors(ref, ref, origin);
  const summary = errorSummary(self);
  assert.equal(summary.matched, ref.n);
  assert.ok(summary.horizontalRms < 1e-6 && summary.u.rms < 1e-6);
  // A solution 1 m east of the reference.
  const rtk = parseSolution('rtk.pos', SOLVE_POS);
  const err = referenceErrors(rtk, ref, origin);
  const enuSol = toEnu(rtk, origin);
  assert.ok(Number.isFinite(err.e[0]) && Number.isFinite(enuSol.e[0]));
  const stats = solutionSummary(rtk, enuSol);
  assert.equal(stats.counts.FIX, 1);
  assert.equal(stats.counts.FLOAT, 1);
  assert.equal(stats.fixRate, 0.5);
  assert.equal(nearestIndex(Float64Array.from([1, 2, 3]), 2.4), 1);
});

test('post-processing commands mirror the CLI', () => {
  const rtk = buildPostCommand({
    mode: 'kinematic', rover: 'rover.obs', base: 'base.obs', nav: 'base.nav',
    elevationMask: '15', preset: 'low-cost', iono: 'auto', kml: true, verbose: true,
    extra: '--arfilter --min-ar-sats 6',
  });
  assert.equal(rtk.tool, 'gnss_solve');
  assert.deepEqual(rtk.inputs, ['rover.obs', 'base.obs', 'base.nav']);
  assert.deepEqual(rtk.outputs, ['rover_rtk.pos', 'rover_rtk.kml']);
  assert.deepEqual(rtk.args, [
    '--rover', '/in/rover.obs', '--base', '/in/base.obs', '--nav', '/in/base.nav',
    '--out', '/out/rover_rtk.pos', '--mode', 'kinematic', '--preset', 'low-cost',
    '--elevation-mask-deg', '15', '--kml', '/out/rover_rtk.kml', '--verbose',
    '--arfilter', '--min-ar-sats', '6',
  ]);
  assert.equal(rtk.commandLine,
    'gnss solve --rover rover.obs --base base.obs --nav base.nav --out rover_rtk.pos --mode kinematic '
    + '--preset low-cost --elevation-mask-deg 15 --kml rover_rtk.kml --verbose --arfilter --min-ar-sats 6');

  const ecef = buildPostCommand({
    mode: 'static', rover: 'r.obs', base: 'b.obs', nav: 'n.nav', baseEcef: ['-3961905.0275', '3348993.7526', '3698211.7831'],
  });
  assert.ok(ecef.args.join(' ').includes('--mode static --base-ecef -3961905.0275 3348993.7526 3698211.7831 --no-kml'));

  const spp = buildPostCommand({ mode: 'single', rover: 'r.obs', nav: 'n.nav', maxEpochs: '10', ionoFree: true });
  assert.equal(spp.tool, 'gnss_spp');
  assert.deepEqual(spp.args, [
    '--obs', '/in/r.obs', '--nav', '/in/n.nav', '--out', '/out/r_spp.pos', '--max-epochs', '10',
    '--ionosphere-free', '--summary-json', '/out/r_spp_summary.json',
  ]);

  const ppp = buildPostCommand({
    mode: 'ppp-static', rover: 'synthetic_ppp.obs', sp3: 'synthetic_ppp.sp3', clk: 'synthetic_ppp.clk',
    elevationMask: 10, extra: '--antex igs20.atx', workspaceNames: ['igs20.atx'],
  });
  assert.equal(ppp.tool, 'gnss_ppp');
  assert.deepEqual(ppp.inputs, ['synthetic_ppp.obs', 'synthetic_ppp.sp3', 'synthetic_ppp.clk', 'igs20.atx']);
  assert.ok(ppp.args.includes('--static'));
  assert.deepEqual(ppp.args.slice(-2), ['--antex', '/in/igs20.atx']);
  assert.deepEqual(ppp.args.slice(ppp.args.indexOf('--elevation-mask'), ppp.args.indexOf('--elevation-mask') + 2), ['--elevation-mask', '10']);

  assert.throws(() => buildPostCommand({ mode: 'kinematic', rover: 'r.obs', nav: 'n.nav' }), /needs a base file/);
  assert.throws(() => buildPostCommand({ mode: 'ppp-static', rover: 'r.obs', sp3: 's.sp3' }), /navigation file or both SP3 and CLK/);
  assert.equal(defaultPosName('run/rover.24o', 'single'), 'run/rover_spp.pos');
});

test('convert and rinex-info commands', () => {
  const rtcm = buildConvertCommand({
    input: 'base.rtcm', format: 'rtcm', gpsWeek: 2324, obsOut: 'base.obs', navOut: 'base.nav', quiet: true,
  });
  assert.equal(rtcm.commandLine,
    'gnss convert --input base.rtcm --format rtcm --gps-week 2324 --obs-out base.obs --nav-out base.nav --quiet');
  const ubx = buildConvertCommand({ input: 'log.ubx', format: 'ubx', gpsWeek: 2324, obsOut: 'log.obs', navOut: 'log.nav' });
  assert.deepEqual(ubx.args, ['--input', '/in/log.ubx', '--format', 'ubx', '--obs-out', '/out/log.obs', '--nav-rinex-out', '/out/log.nav']);
  assert.throws(() => buildConvertCommand({ input: 'x.rtcm', format: 'rtcm' }), /at least one RINEX output/);

  const info = buildInfoCommand({ files: ['rover.obs', 'base.nav'], countRecords: true });
  assert.deepEqual(info.args, ['--count-records', '/in/rover.obs', '/in/base.nav']);
  assert.equal(info.commandLine, 'gnss rinex-info --count-records rover.obs base.nav');
});

test('splitArgs handles quotes and escapes', () => {
  assert.deepEqual(splitArgs(` --a 1  --b "two words" 'x y' c\\ d ""`), ['--a', '1', '--b', 'two words', 'x y', 'c d', '']);
  assert.deepEqual(splitArgs(''), []);
  assert.throws(() => splitArgs('"open'), /unterminated/);
});

test('options TOML export matches the CLI --config schema and round-trips', () => {
  const opts = {
    mode: 'static', rover: 'rover.obs', base: 'base.obs', nav: 'base.nav', preset: 'survey', iono: 'iflc',
    ratio: '2.5', elevationMask: '15', maxEpochs: '', kml: false, verbose: true,
    baseEcef: ['-3961905.0275', '3348993.7526', '3698211.7831'],
    extra: '--arfilter --min-ar-sats 6 --no-glonass --rtk-snr-reference-dbhz 42',
  };
  const toml = exportOptionsToml(opts, 'gnss solve ...');
  const { sections } = parseFlatToml(toml);
  assert.deepEqual(sections.gnss_solve, [
    ['mode', 'static'], ['preset', 'survey'], ['iono', 'iflc'], ['ratio', '2.5'], ['elevation_mask_deg', '15'],
    ['base_ecef', ['-3961905.0275', '3348993.7526', '3698211.7831']], ['kml', false], ['verbose', true],
    ['arfilter', true], ['min_ar_sats', '6'], ['no_glonass', true], ['rtk_snr_reference_dbhz', '42'],
  ]);
  assert.match(toml, /^base_ecef = \[-3961905\.0275, 3348993\.7526, 3698211\.7831\]$/m);
  assert.match(toml, /^preset = "survey"$/m);
  assert.doesNotMatch(toml, /rover|base\.obs/); // input/output files are not written

  const back = importOptionsToml(toml);
  assert.equal(back.mode, 'static');
  assert.deepEqual(back.fields, {
    kml: false, verbose: true, enableAr: false, basePos: 'ecef', preset: 'survey', iono: 'iflc', ratio: '2.5',
    elevationMask: '15', baseX: '-3961905.0275', baseY: '3348993.7526', baseZ: '3698211.7831',
  });
  assert.equal(back.extra, '--arfilter --min-ar-sats 6 --no-glonass --rtk-snr-reference-dbhz 42');

  const ppp = exportOptionsToml({ mode: 'ppp-kinematic', elevationMask: 10, enableAr: true, extra: '' });
  assert.deepEqual(parseFlatToml(ppp).sections.gnss_ppp, [['kinematic', true], ['elevation_mask', '10'], ['enable_ar', true]]);
  assert.equal(importOptionsToml(ppp).mode, 'ppp-kinematic');
  assert.throws(() => exportOptionsToml({ mode: 'single' }), /gnss spp has no --config/);
  assert.throws(() => extraArgsToEntries('stray --x'), /does not follow an --option/);
});

test('options import reads the repository example configs', () => {
  const solve = importOptionsToml(fixture('configs/examples/solve.example.toml').toString());
  assert.equal(solve.mode, 'kinematic');
  assert.equal(solve.fields.preset, 'low-cost');
  assert.equal(solve.fields.ratio, '2.4');
  assert.equal(solve.fields.maxEpochs, '0');
  assert.deepEqual(solve.files, { data_dir: 'run1', nav: 'base.nav', out: 'rtk_solution.pos' });
  assert.equal(solve.extra, '--max-subset-ar-drop-steps 18 --rtk-snr-weighting --no-arfilter');

  const ppp = importOptionsToml(fixture('configs/examples/ppp_static.toml').toString());
  assert.equal(ppp.mode, 'ppp-static');
  assert.equal(ppp.fields.enableAr, true);
  assert.equal(ppp.extra, '--estimate-troposphere --ar-ratio-threshold 3.0');

  const odd = importOptionsToml('[gnss_solve]\nmode = "auto"\nratio = "sat-count"\npreset = "unknown"\n');
  assert.equal(odd.mode, 'kinematic');
  assert.deepEqual(odd.notes, ['mode = "auto" has no GUI equivalent; using kinematic']);
  assert.equal(odd.extra, '--ratio sat-count --preset unknown');
});

test('WORKERFS read-ahead returns the same bytes with far fewer blob reads', () => {
  // Stand-ins for Emscripten's WORKERFS: Blob-backed nodes read with a
  // synchronous FileReader (FileReaderSync exists only in browser workers).
  class BytesBlob extends Blob {
    constructor(bytes) { super([bytes]); this.bytes = bytes; }
    slice(start, end) { return new BytesBlob(this.bytes.subarray(start, end)); }
  }
  const data = new Uint8Array(1_000_003).map((_, i) => (i * 7919) % 251);
  const blob = new BytesBlob(data);
  let reads = 0;
  const workerfs = {
    reader: { readAsArrayBuffer(b) { reads += 1; return b.bytes.slice().buffer; } },
    stream_ops: {
      read(stream, buffer, offset, length, position) {
        if (position >= stream.node.size) return 0;
        const chunk = stream.node.contents.slice(position, position + length);
        buffer.set(new Uint8Array(workerfs.reader.readAsArrayBuffer(chunk)), offset);
        return chunk.size;
      },
    },
  };
  const stream = { node: { contents: blob, size: data.length } };
  assert.ok(addWorkerfsReadAhead(workerfs, 65536));
  assert.equal(addWorkerfsReadAhead(workerfs, 65536), false); // patched once
  // Sequential 4 KiB reads, a seek back, an unaligned tail and a large read.
  const out = new Uint8Array(data.length + 16);
  let pos = 0;
  while (pos < data.length) {
    const n = workerfs.stream_ops.read(stream, out, pos, 4096, pos);
    assert.ok(n > 0);
    pos += n;
  }
  assert.deepEqual(out.subarray(0, data.length), data);
  assert.equal(workerfs.stream_ops.read(stream, out, 0, 10, data.length), 0);
  const seek = new Uint8Array(300);
  assert.equal(workerfs.stream_ops.read(stream, seek, 0, 300, 12345), 300);
  assert.deepEqual(seek, data.subarray(12345, 12645));
  assert.equal(workerfs.stream_ops.read(stream, seek, 0, 300, data.length - 7), 7);
  assert.deepEqual(seek.subarray(0, 7), data.subarray(data.length - 7));
  const big = new Uint8Array(200_000);
  assert.equal(workerfs.stream_ops.read(stream, big, 0, big.length, 5), big.length);
  assert.deepEqual(big, data.subarray(5, 200_005));
  // ~1 MB in 4 KiB reads: 16 chunk reads instead of ~245.
  assert.ok(reads < 25, `blob reads: ${reads}`);
});

// Rows from `gnss visibility` on the converted realtime_stress rover file.
const VIS_CSV = `epoch_index,week,tow,satellite,system,signal,azimuth_deg,elevation_deg,snr_dbhz,has_pseudorange,has_carrier_phase,has_doppler,lli
1,2324,187470.000,G05,GPS,GPS_L1CA,293.612610,39.445566,41.500000,1,1,1,0
1,2324,187470.000,E11,Galileo,GAL_E1,125.958785,44.705874,38.375000,1,1,1,0
1,2324,187470.000,G13,GPS,GPS_L1CA,80.1,20.2,,1,0,1,0
2,2324,187470.200,G05,GPS,GPS_L1CA,293.613000,39.445000,41.250000,1,1,1,1
`;

test('visibility CSV: sniffing, parsing and satellite order', () => {
  assert.equal(sniffKind('rover_vis.csv', new TextEncoder().encode(VIS_CSV)), 'vis');
  const vis = parseVisibilityCsv('rover_vis.csv', VIS_CSV);
  assert.equal(vis.n, 4);
  assert.deepEqual(vis.sats, ['G05', 'G13', 'E11']); // GPS before Galileo, PRN order
  assert.deepEqual([...vis.sat], [0, 2, 1, 0]);
  assert.deepEqual([...vis.sys], [0, 2, 0, 0]);
  assert.deepEqual([...vis.lli], [0, 0, 0, 1]);
  assert.ok(Number.isNaN(vis.snr[2]));
  assert.equal(vis.epochs.length, 2);
  assert.equal(vis.t[0], gpsSeconds(2324, 187470));
  assert.ok(vis.hasLli);
  // Older CSVs without the lli column still parse.
  const legacy = VIS_CSV.replace(/,lli\n/, '\n').replace(/,(0|1)\n/g, '\n');
  assert.equal(parseVisibilityCsv('old.csv', legacy).hasLli, false);
  assert.throws(() => parseVisibilityCsv('x.csv', 'a,b\n1,2\n'), /not a gnss visibility CSV/);
});

test('visibility command', () => {
  const vis = buildVisibilityCommand({ obs: 'rover.obs', nav: 'base.nav', minElevation: '10' });
  assert.equal(vis.commandLine,
    'gnss visibility --obs rover.obs --nav base.nav --csv rover_vis.csv --summary-json rover_vis_summary.json '
    + '--min-elevation-deg 10 --quiet');
  assert.deepEqual(vis.outputs, ['rover_vis.csv', 'rover_vis_summary.json']);
  const ecef = buildVisibilityCommand({ obs: 'r.obs', nav: 'n.nav', receiverEcef: ['-3961767.63', '3349008.712', '3698309.78'] });
  assert.ok(ecef.args.join(' ').includes('--receiver-ecef -3961767.63 3349008.712 3698309.78'));
  assert.throws(() => buildVisibilityCommand({ obs: 'r.obs' }), /navigation file/);
});

test('RTK + IMU command, IMU sniffing and options round trip', () => {
  const imuHead = 'GPS TOW (s), GPS Week, Acc X (m/s^2), Acc Y (m/s^2), Acc Z (m/s^2), Ang Rate X (deg/s),  Ang Rate Y (deg/s),  Ang Rate Z (deg/s)\n';
  assert.equal(sniffKind('imu.csv', new TextEncoder().encode(imuHead)), 'imu');
  assert.equal(sniffKind('imu.csv', fixture('tests/fixtures/realtime_stress/imu.csv').subarray(0, 4096)), 'imu');
  // The reference trajectory (attitude and velocity, no accelerations) stays a reference CSV.
  assert.equal(sniffKind('reference.csv', fixture('tests/fixtures/realtime_stress/reference.csv').subarray(0, 4096)), 'csv');

  const opts = {
    mode: 'rtk-imu', rover: 'rover.obs', base: 'base.obs', nav: 'base.nav', imu: 'imu.csv',
    leverArm: ['0.31', '0', '0.55'], tightCoupling: true, imuFormat: 'ppc', preset: 'low-cost', ratio: '2.4', kml: false,
  };
  const cmd = buildPostCommand(opts);
  assert.equal(cmd.tool, 'gnss_fuse');
  assert.deepEqual(cmd.inputs, ['rover.obs', 'base.obs', 'base.nav', 'imu.csv']);
  assert.deepEqual(cmd.outputs, ['rover_fused.pos', 'rover_fused_rtk.pos']);
  assert.equal(cmd.commandLine,
    'gnss fuse --rover rover.obs --base base.obs --nav base.nav --imu imu.csv --out rover_fused.pos '
    + '--rtk-pos-out rover_fused_rtk.pos --lever-arm 0.31,0,0.55 --navi776-tc --preset low-cost --ratio 2.4');
  assert.throws(() => buildPostCommand({ ...opts, imu: '' }), /needs an IMU \(CSV\) file/);
  const loose = buildPostCommand({ ...opts, leverArm: ['', '', ''], tightCoupling: false, imuFormat: 'rtklibexplorer-sf', kml: true });
  assert.ok(loose.args.includes('--imu-format') && !loose.args.includes('--lever-arm') && !loose.args.includes('--navi776-tc'));
  assert.deepEqual(loose.outputs, ['rover_fused.pos', 'rover_fused_rtk.pos', 'rover_fused.kml']);
  assert.ok(loose.commandLine.endsWith('--kml rover_fused.kml'));

  const toml = exportOptionsToml(opts);
  assert.match(toml, /^\[gnss_fuse\]$/m);
  assert.deepEqual(parseFlatToml(toml).sections.gnss_fuse, [
    ['lever_arm', ['0.31', '0', '0.55']], ['navi776_tc', true], ['preset', 'low-cost'], ['ratio', '2.4'],
  ]);
  assert.match(toml, /# Use: gnss fuse --config/);
  const back = importOptionsToml(toml);
  assert.equal(back.mode, 'rtk-imu');
  assert.equal(back.tool, 'gnss_fuse');
  assert.deepEqual([back.fields.leverX, back.fields.leverY, back.fields.leverZ], ['0.31', '0', '0.55']);
  assert.equal(back.fields.tightCoupling, true);
  assert.equal(back.fields.preset, 'low-cost');
  assert.equal(back.extra, '');
  const fromCli = importOptionsToml('[gnss_fuse]\nimu = "data/run1/imu.csv"\nlever_arm = "0.5,0,1"\nimu_format = "rtklibexplorer-sf"\nzupt = true\n');
  assert.deepEqual(fromCli.files, { imu: 'imu.csv' });
  assert.deepEqual([fromCli.fields.leverX, fromCli.fields.leverY, fromCli.fields.leverZ], ['0.5', '0', '1']);
  assert.equal(fromCli.fields.imuFormat, 'rtklibexplorer-sf');
  assert.equal(fromCli.fields.tightCoupling, false);
  assert.equal(fromCli.extra, '--zupt');
});

test('map tiles: ENU placement, tile choice and coverage', () => {
  const origin = [35.667552644, 139.791093447, 40.19];
  const plane = enuPlane(origin);
  const near = (a, b, tol) => assert.ok(Math.abs(a - b) < tol, `${a} vs ${b}`);
  const [e0, n0] = plane.toEnu(origin[0], origin[1]);
  near(e0, 0, 1e-6); near(n0, 0, 1e-6);
  // toLatLon() takes the tangent-plane point, which rises above the origin
  // height by d^2 / 2R (57 m at 27 km); that is all the round trip loses.
  for (const [e, n, tol] of [[1234.5, -678.9, 0.01], [-15000, 22000, 0.5]]) {
    const [lat, lon] = plane.toLatLon(e, n);
    const [e2, n2] = plane.toEnu(lat, lon);
    near(e2, e, tol); near(n2, n, tol);
  }
  // Tile corners land where toEnu() of the solution points does.
  const sol = parseSolution('t.pos', `% GPS_Week GPS_TOW Lat(deg) Lon(deg) Height(m) Status
2324 0.0 ${origin[0] + 0.009} ${origin[1] + 0.011} ${origin[2]} 1
`);
  const enu = toEnu(sol, origin);
  const [pe, pn] = plane.toEnu(sol.lat[0], sol.lon[0]);
  near(pe, enu.e[0], 1e-6); near(pn, enu.n[0], 1e-6);

  // The tile under a point matches the standard slippy-map formula.
  const z = 16;
  const n = 2 ** z;
  const rad = (origin[0] * Math.PI) / 180;
  const expectX = Math.floor(((origin[1] + 180) / 360) * n);
  const expectY = Math.floor(((1 - Math.log(Math.tan(rad) + 1 / Math.cos(rad)) / Math.PI) / 2) * n);
  const [one] = tilesForView({ x0: -1, x1: 1, y0: -1, y1: 1 }, plane, z);
  assert.deepEqual([one.z, one.x, one.y], [z, expectX, expectY]);
  assert.ok(one.x0 <= 0 && one.x1 >= 0 && one.y0 <= 0 && one.y1 >= 0);
  const [cLat, cLon] = tileCorner(expectX, expectY, z);
  const [fx, fy] = tileXY(cLat, cLon, z);
  near(fx, expectX, 1e-6); near(fy, expectY, 1e-6);
  assert.equal(tileUrl(TILE_SOURCES.osm, one.z, one.x, one.y), `https://tile.openstreetmap.org/16/${expectX}/${expectY}.png`);
  assert.equal(tileUrl(TILE_SOURCES['gsi-photo'], 16, 1, 2), 'https://cyberjapandata.gsi.go.jp/xyz/seamlessphoto/16/1/2.jpg');

  // A view is covered. Neighbouring tiles meet to within the curvature of
  // meridians / parallels across one tile in the ENU plane: under 0.1 m here,
  // where a 600 px plot shows ~2.7 m per pixel.
  const view = { x0: -900, x1: 700, y0: -400, y1: 650 };
  const tiles = tilesForView(view, plane, 15);
  assert.ok(tiles.length > 1);
  assert.ok(Math.min(...tiles.map((t) => t.x0)) <= view.x0 && Math.max(...tiles.map((t) => t.x1)) >= view.x1);
  assert.ok(Math.min(...tiles.map((t) => t.y0)) <= view.y0 && Math.max(...tiles.map((t) => t.y1)) >= view.y1);
  for (const a of tiles) {
    const right = tiles.find((b) => b.y === a.y && b.x === a.x + 1);
    if (right) near(right.x0, a.x1, 0.1);
    const below = tiles.find((b) => b.x === a.x && b.y === a.y + 1);
    if (below) near(below.y1, a.y0, 0.1);
  }
  assert.deepEqual(tilesForView({ x0: -1e5, x1: 1e5, y0: -1e5, y1: 1e5 }, plane, 18), [], 'too many tiles');

  // One tile pixel per screen pixel: ~1.94 m/px is zoom 16 at Tokyo; clamped to the source range.
  const res16 = (156543.03392 * Math.cos(rad)) / 2 ** 16;
  assert.equal(chooseZoom(res16, origin[0], 19), 16);
  assert.equal(chooseZoom(res16 / 2, origin[0], 19), 17);
  assert.equal(chooseZoom(res16 / 2, origin[0], 19, 2), 18);
  assert.equal(chooseZoom(0.001, origin[0], TILE_SOURCES['gsi-std'].maxZoom), 18);
  assert.equal(chooseZoom(1e7, origin[0], 19), 0);
});

test('workspace keeps a restored file\'s time stamp; stored-files summary', async () => {
  const ws = new Workspace();
  const events = [];
  ws.addEventListener('change', (e) => events.push(e.detail));
  const restored = await ws.add('rover.obs', new Blob(['     3.04           OBSERVATION DATA    M                   RINEX VERSION / TYPE\n']), 'sample', { added: 1234 });
  assert.equal(restored.added, 1234);
  assert.equal(restored.kind, 'obs');
  const fresh = await ws.add('later.pos', '% x\n', 'output');
  assert.ok(fresh.added > 1234);
  assert.deepEqual(ws.list().map((f) => f.name), ['later.pos', 'rover.obs']);
  ws.remove('rover.obs');
  ws.clear();
  assert.deepEqual(events, [{ added: 'rover.obs' }, { added: 'later.pos' }, { removed: 'rover.obs' }, {}]);

  const blob = (n) => ({ blob: { size: n } });
  assert.equal(describeStored([]), '0 files, 0 kB');
  assert.equal(describeStored([blob(1500)]), '1 file, 2 kB');
  assert.equal(describeStored([blob(90e6), blob(36e6)]), '2 files, 126.0 MB');
  assert.equal(describeStored([blob(2.5e9)]), '1 file, 2.50 GB');
});

test('horizontal error classes for colouring a track', () => {
  const err = {
    e: Float64Array.from([0.03, 0.2, 0.6, -2, 30, NaN, 0.1]),
    n: Float64Array.from([0.04, 0, 0, 0, 0, NaN, 0]),
  };
  // 0.05 m, 0.2, 0.6, 2, 30, unmatched, exactly on the 0.1 m edge (goes up).
  assert.deepEqual([...errorClasses(err)], [0, 1, 2, 3, 4, 5, 1]);
  assert.deepEqual(errorClassLabels(), ['< 0.1 m', '0.1–0.3 m', '0.3–1 m', '1–3 m', '≥ 3 m', 'no reference']);
  assert.equal(errorClassLabels().length, ERROR_BINS_M.length + 2);
  assert.deepEqual([...errorClasses({ e: [0.5], n: [0] }, [1])], [0]);
});

test('FGO command and options', () => {
  const opts = {
    mode: 'fgo', rover: 'rover.obs', base: 'base.obs', nav: 'base.nav', fgoPreset: 'real-data-fixed',
    ratio: '2.5', elevationMask: '15', maxEpochs: '400', kml: true, verbose: true, baseEcef: null,
  };
  const cmd = buildPostCommand(opts);
  assert.equal(cmd.tool, 'gnss_fgo');
  assert.deepEqual(cmd.inputs, ['rover.obs', 'base.obs', 'base.nav']);
  assert.deepEqual(cmd.outputs, ['rover_fgo.pos', 'rover_fgo_summary.json']);
  assert.equal(cmd.commandLine,
    'gnss fgo --obs rover.obs --base base.obs --nav base.nav --out rover_fgo.pos --preset real-data-fixed '
    + '--epoch-lambda-fixed-output --report-batch-fix-as-float '
    + '--lambda-ratio-threshold 2.5 --min-elevation 15 --max-epochs 400 --summary-json rover_fgo_summary.json');
  assert.equal(defaultPosName('rover.obs', 'fgo'), 'rover_fgo.pos');
  const plain = buildPostCommand({ mode: 'fgo', rover: 'r.obs', base: 'b.obs', nav: 'n.nav' });
  assert.ok(plain.args.join(' ').includes('--preset real-data-float'), 'float preset by default');
  assert.ok(!plain.args.includes('--epoch-lambda-fixed-output'), 'no AR flags for the float preset');
  assert.throws(() => buildPostCommand({ ...opts, base: '' }), /needs a base file/);
  assert.throws(() => exportOptionsToml(opts), /gnss fgo has no --config/);
});
