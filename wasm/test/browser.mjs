// Browser end-to-end test of the GUI with the real WebAssembly tools.
//
//   node wasm/test/browser.mjs <site dir> [--qzss <samples dir>] [--screenshots <dir>]
//
// <site dir> is the output of wasm/assemble_site.py's parent (the GUI under
// <site dir>/app/). The site is served on localhost and driven in headless
// Chromium with Playwright: the demo pipeline, RTK + IMU, FGO, SPP and PPP
// runs, the Observations and Plot views (map tiles answered locally, error
// colouring, PNG export) and "Keep files after reload"; with --qzss, also
// MADOCA-PPP and CLAS on the public QZSS L6 samples (smoke.mjs QZSS_SAMPLES)
// added through the file input. It fails on any page error, console error or
// request to another site than the map tile hosts.
// Playwright is resolved from $PLAYWRIGHT_PREFIX/node_modules when set (CI
// installs it there), else from the usual module paths.
import { strict as assert } from 'node:assert';
import { createReadStream, existsSync, mkdirSync, statSync } from 'node:fs';
import { createServer } from 'node:http';
import { createRequire } from 'node:module';
import { extname, join, normalize, resolve } from 'node:path';
import { deflateSync } from 'node:zlib';

import { QZSS_SAMPLES } from './smoke.mjs';

const args = process.argv.slice(2);
const siteDir = resolve(args[0] || 'site');
const shotsAt = args.indexOf('--screenshots');
const shotsDir = shotsAt >= 0 ? resolve(args[shotsAt + 1]) : null;
const qzssAt = args.indexOf('--qzss');
const qzssDir = qzssAt >= 0 ? resolve(args[qzssAt + 1]) : null;
if (!existsSync(join(siteDir, 'app', 'index.html'))) throw new Error(`no app/index.html under ${siteDir}`);
if (shotsDir) mkdirSync(shotsDir, { recursive: true });

const require = createRequire(process.env.PLAYWRIGHT_PREFIX
  ? join(process.env.PLAYWRIGHT_PREFIX, 'package.json') : import.meta.url);
const { chromium } = require('playwright');

const TYPES = {
  '.html': 'text/html', '.js': 'text/javascript', '.mjs': 'text/javascript', '.css': 'text/css',
  '.json': 'application/json', '.wasm': 'application/wasm', '.png': 'image/png', '.svg': 'image/svg+xml',
};
const server = createServer((req, res) => {
  const url = new URL(req.url, 'http://localhost');
  let path = normalize(join(siteDir, decodeURIComponent(url.pathname)));
  if (!path.startsWith(siteDir) || !existsSync(path)) { res.writeHead(404); res.end(); return; }
  if (statSync(path).isDirectory()) path = join(path, 'index.html');
  res.writeHead(200, { 'content-type': TYPES[extname(path)] || 'application/octet-stream' });
  createReadStream(path).pipe(res);
});
await new Promise((ok) => server.listen(0, '127.0.0.1', ok));
const base = `http://127.0.0.1:${server.address().port}/app/`;

// A plain 256x256 tile so the map path runs without the network.
function tilePng() {
  const crcTable = Array.from({ length: 256 }, (_, n) => {
    let c = n;
    for (let k = 0; k < 8; k += 1) c = c & 1 ? 0xedb88320 ^ (c >>> 1) : c >>> 1;
    return c >>> 0;
  });
  const crc = (buf) => { let c = 0xffffffff; for (const b of buf) c = crcTable[(c ^ b) & 0xff] ^ (c >>> 8); return (c ^ 0xffffffff) >>> 0; };
  const chunk = (type, data) => {
    const body = Buffer.concat([Buffer.from(type), data]);
    const out = Buffer.alloc(12 + data.length);
    out.writeUInt32BE(data.length, 0); body.copy(out, 4); out.writeUInt32BE(crc(body), 8 + data.length);
    return out;
  };
  const ihdr = Buffer.alloc(13);
  ihdr.writeUInt32BE(256, 0); ihdr.writeUInt32BE(256, 4); ihdr[8] = 8; ihdr[9] = 2;
  const raw = Buffer.alloc(256 * (1 + 256 * 3));
  for (let y = 0; y < 256; y += 1) for (let x = 0; x < 256; x += 1) raw.set([226, 232, 222], y * 769 + 1 + x * 3);
  return Buffer.concat([Buffer.from([137, 80, 78, 71, 13, 10, 26, 10]), chunk('IHDR', ihdr),
    chunk('IDAT', deflateSync(raw)), chunk('IEND', Buffer.alloc(0))]);
}
const TILE = tilePng();
const TILE_HOSTS = ['tile.openstreetmap.org', 'cyberjapandata.gsi.go.jp'];

const browser = await chromium.launch();
const context = await browser.newContext({ viewport: { width: 1440, height: 1000 }, deviceScaleFactor: shotsDir ? 2 : 1, acceptDownloads: true });
const page = await context.newPage();
const problems = [];
const external = [];
let tiles = 0;
page.on('pageerror', (e) => problems.push(`page error: ${e.message}`));
page.on('console', (m) => { if (m.type() === 'error') problems.push(`console error: ${m.text()}`); });
page.on('dialog', (d) => d.accept());
await context.route(/^https?:\/\//, (route) => {
  const url = new URL(route.request().url());
  if (url.hostname === '127.0.0.1') return route.continue();
  if (TILE_HOSTS.includes(url.hostname)) {
    tiles += 1;
    return route.fulfill({ status: 200, contentType: 'image/png', headers: { 'access-control-allow-origin': '*' }, body: TILE });
  }
  external.push(url.href);
  return route.abort();
});

const log = (...a) => console.log(...a);
const text = (sel) => page.textContent(sel).then((t) => (t || '').replace(/\s+/g, ' ').trim());
const files = () => page.$$eval('#file-list li:not(.empty) .name', (els) => els.map((e) => e.firstChild.textContent));
const shot = async (name, locator) => { if (shotsDir) await (locator || page).screenshot({ path: join(shotsDir, name) }); };

try {
  await page.goto(base, { waitUntil: 'networkidle' });
  await page.waitForFunction(() => !document.querySelector('#run-sample').disabled, null, { timeout: 60000 });
  assert.match(await text('#engine'), /WebAssembly/);
  log('engine:', await text('#engine'));

  // Demo pipeline: RTCM3 -> RINEX -> observations -> RTK -> Plot vs reference.
  let t0 = Date.now();
  await page.click('#run-sample');
  await page.waitForFunction(() => /Demo pipeline (finished|stopped)/.test(document.querySelector('#status').textContent), null, { timeout: 300000 });
  assert.match(await text('#status'), /Demo pipeline finished/);
  const stats = await text('#plot-stats');
  const fix = Number((stats.match(/FIX\s*(\d+)/) || [])[1]);
  assert.ok(fix >= 180, `demo RTK FIX epochs: ${fix}`);
  log(`demo pipeline: ${((Date.now() - t0) / 1000).toFixed(1)} s, ${fix} FIX`);
  for (const name of ['rover.obs', 'base.obs', 'base.nav', 'rover_vis.csv', 'rover_rtk.pos', 'rover_rtk.kml']) {
    assert.ok((await files()).includes(name), `missing ${name}`);
  }
  assert.ok(await page.locator('#cv-sky').count(), 'observation skyplot');

  const post = async (label, setup, expectOutputs) => {
    await page.click('button[data-tool="post"]');
    await setup();
    await page.waitForTimeout(150);
    const cmd = await text('#tool-post [data-cmd]');
    assert.ok(!cmd.startsWith('#'), `${label}: ${cmd}`);
    t0 = Date.now();
    await page.click('#post-form button[type="submit"]');
    await page.waitForFunction(() => /Done|Exit code|Failed|Error/.test(document.querySelector('#tool-post [data-meta]').textContent), null, { timeout: 600000 });
    const meta = await text('#tool-post [data-meta]');
    assert.match(meta, /^Done/, `${label}: ${meta}`);
    const outputs = await text('#tool-post [data-outputs]');
    for (const name of expectOutputs) assert.ok(outputs.includes(name), `${label}: missing output ${name} in ${outputs}`);
    log(`${label}: ${((Date.now() - t0) / 1000).toFixed(1)} s · $ ${cmd}`);
  };
  const mode = (m) => page.selectOption('#post-form select[name="mode"]', m);

  await post('RTK + IMU', async () => {
    await mode('rtk-imu');
    await page.selectOption('#post-form select[name="imu"]', 'imu.csv');
    await page.dispatchEvent('#post-form select[name="imu"]', 'change');
  }, ['rover_fused.pos', 'rover_fused_rtk.pos', 'rover_fused.kml']);
  assert.equal(await page.inputValue('#post-form input[name="leverX"]'), '0.31', 'sample IMU lever arm');
  await shot('web_app_post.png');
  await post('FGO real-data-float', async () => {
    await mode('fgo');
    await page.selectOption('#post-form select[name="fgoPreset"]', 'real-data-float');
  }, ['rover_fgo.pos', 'rover_fgo_summary.json']);
  await post('FGO real-data-fixed', async () => {
    await page.selectOption('#post-form select[name="fgoPreset"]', 'real-data-fixed');
    await page.fill('#post-form input[name="out"]', 'rover_fgo_fixed.pos');
  }, ['rover_fgo_fixed.pos']);
  await post('SPP', async () => { await page.fill('#post-form input[name="out"]', ''); await mode('single'); }, ['rover_spp.pos']);
  await post('PPP static', async () => {
    await mode('ppp-static');
    await page.selectOption('#post-form select[name="rover"]', 'synthetic_ppp.obs');
    await page.waitForFunction(() => document.querySelector('#post-form select[name="nav"]').value === '', null, { timeout: 10000 });
    assert.match(await text('#status'), /navigation set to \(none\)/);
    await page.selectOption('#post-form select[name="sp3"]', 'synthetic_ppp.sp3');
    await page.selectOption('#post-form select[name="clk"]', 'synthetic_ppp.clk');
  }, ['synthetic_ppp_ppp.pos']);

  // Plot: no tile request before a map is chosen; then map, error colours, PNG.
  assert.equal(tiles, 0, 'tiles requested before choosing a map');
  await page.click('button[data-tool="plot"]');
  await page.selectOption('#plot-sol1', 'rover_rtk.pos');
  await page.selectOption('#plot-sol2', 'rover_fused.pos');
  await page.selectOption('#plot-ref', 'reference.csv');
  await page.selectOption('#plot-color', 'error');
  await page.selectOption('#plot-map', 'osm');
  await page.waitForFunction(() => /Map zoom \d+$/.test(document.querySelector('#plot-map-note span')?.textContent || ''), null, { timeout: 30000 });
  assert.ok(tiles > 0, 'map tiles requested');
  assert.match(await text('#plot-legend'), /horizontal error:/);
  assert.match(await text('#plot-stats'), /Horizontal RMS/);
  await shot('web_app_plot.png', page.locator('.card.gt'));
  const [png] = await Promise.all([page.waitForEvent('download'), page.click('[data-png="gt"]')]);
  const pngPath = await png.path();
  assert.ok(statSync(pngPath).size > 10000, 'ground-track PNG');
  log(`plot: map ${await text('#plot-map-note')}, PNG ${statSync(pngPath).size} bytes, ${tiles} tile requests`);

  // Keep files across a reload, then clean up.
  const before = await files();
  await page.check('#keep-files');
  await page.waitForFunction(() => /kept: \d+ files/.test(document.querySelector('#keep-files-note').textContent), null, { timeout: 60000 });
  await page.reload({ waitUntil: 'networkidle' });
  await page.waitForFunction((n) => document.querySelectorAll('#file-list li:not(.empty)').length === n, before.length, { timeout: 60000 });
  assert.deepEqual(await files(), before);
  await page.uncheck('#keep-files');
  await page.waitForTimeout(500);
  log(`keep files: ${before.length} files restored after reload`);

  if (qzssDir) {
    // QZSS L6 PPP: the samples go in through the file input like a user's files.
    const paths = Object.entries(QZSS_SAMPLES).flatMap(([tree, list]) => list.map((path) => join(qzssDir, tree, path)));
    const names = paths.map((path) => path.split('/').pop());
    await page.setInputFiles('#file-input', paths);
    await page.waitForFunction((want) => {
      const have = [...document.querySelectorAll('#file-list li:not(.empty) .name')].map((e) => e.firstChild.textContent);
      return want.every((name) => have.includes(name));
    }, names, { timeout: 60000 });
    const fixedSolutions = async () => Number((/PPP fixed solutions: (\d+)/.exec(await text('#tool-post [data-log]')) || [])[1]);

    await post('MADOCA-PPP (L6E + L6D, AR)', async () => {
      await mode('ppp-kinematic');
      await page.selectOption('#post-form select[name="pppCorr"]', 'madoca');
      await page.selectOption('#post-form select[name="rover"]', 'MIZU00JPN_R_20250910000_01D_30S_MO.rnx');
      await page.selectOption('#post-form select[name="nav"]', 'BRDM00DLR_S_20250910000_01D_MN.rnx');
      await page.selectOption('#post-form select[name="antex"]', 'igs20.atx');
      await page.check('#post-form input[name="enableAr"]');
      await page.fill('#post-form input[name="maxEpochs"]', '120');
      const ticked = (list) => page.$$eval(`${list} input:checked`, (els) => els.map((e) => e.value));
      assert.deepEqual(await ticked('#post-l6e'), ['2025091A.204.l6', '2025091A.206.l6'], 'L6E logs ticked by kind');
      assert.deepEqual(await ticked('#post-l6d'), ['2025091A.200.l6', '2025091A.201.l6'], 'L6D logs ticked by kind');
      assert.ok(await page.isHidden('#post-form select[name="sp3"]'), 'SP3 hidden for MADOCA');
    }, ['MIZU00JPN_R_20250910000_01D_30S_MO_madoca.pos', 'MIZU00JPN_R_20250910000_01D_30S_MO_madoca.kml']);
    const madocaFix = await fixedSolutions();
    assert.ok(madocaFix >= 85, `MADOCA PPP fixed solutions: ${madocaFix}`);
    log(`MADOCA-PPP: ${madocaFix} PPP fixed solutions`);
    await shot('web_app_madoca.png');

    await post('CLAS (L6, experimental)', async () => {
      await mode('ppp-static');
      await page.selectOption('#post-form select[name="pppCorr"]', 'clas');
      await page.selectOption('#post-form select[name="rover"]', '0627239Q.obs');
      await page.selectOption('#post-form select[name="nav"]', 'sept_2019239.nav');
      await page.selectOption('#post-form select[name="clasL6"]', '2019239Q.l6');
      await page.selectOption('#post-form select[name="antex"]', '');
      await page.fill('#post-form input[name="maxEpochs"]', '60');
      assert.ok(await page.isHidden('#post-form input[name="enableAr"]'), 'AR hidden for CLAS');
    }, ['0627239Q_clas.pos']);
    assert.match(await text('#tool-post [data-cmd]'), /--ssr 2019239Q\.l6 --clas-osr/);
  }

  assert.deepEqual(external, [], `requests to other sites: ${external.join(', ')}`);
  assert.deepEqual(problems, []);
  log('browser test passed');
} catch (error) {
  if (shotsDir || process.env.CI) await page.screenshot({ path: join(shotsDir || '.', 'browser-test-failure.png'), fullPage: true }).catch(() => {});
  console.error(problems.join('\n'));
  throw error;
} finally {
  await browser.close();
  server.close();
}
