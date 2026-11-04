// Unit tests for the browser app's pure modules (no wasm needed):
//   node --test web/test/web_app.test.mjs
import assert from 'node:assert/strict';
import { readFileSync } from 'node:fs';
import { test } from 'node:test';

import {
  buildConvertCommand, buildInfoCommand, buildPostCommand, defaultPosName, splitArgs,
} from '../js/commands.js';
import { parseSolution, Q, rinexObsTimeSpan, sniffKind } from '../js/formats.js';
import { errorSummary, nearestIndex, referenceErrors, solutionSummary, toEnu } from '../js/analysis.js';
import { addWorkerfsReadAhead } from '../js/engine.js';
import { calendarToGpsSeconds, ecefToLlh, formatGpst, gpsSeconds, llhToEcef } from '../js/geo.js';

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
