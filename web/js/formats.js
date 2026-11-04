// File-kind sniffing and solution parsers (libgnss++ .pos, RTKLIB .pos,
// reference CSV). Pure functions: used by the browser app and the Node tests.

import { calendarToGpsSeconds, ecefToLlh, gpsSeconds, GPST_UTC_LEAP_SECONDS, llhToEcef } from './geo.js';

// Solution quality, numbered like RTKLIB's Q column where one exists.
export const Q = Object.freeze({
  NONE: 0, FIX: 1, FLOAT: 2, SBAS: 3, DGPS: 4, SINGLE: 5, PPP: 6, PPP_FIX: 7, DR: 8,
});
export const Q_LABELS = Object.freeze({
  0: 'NONE', 1: 'FIX', 2: 'FLOAT', 3: 'SBAS', 4: 'DGPS', 5: 'SINGLE', 6: 'PPP', 7: 'PPP-FIX', 8: 'DR',
});

// libgnss::SolutionStatus (include/libgnss++/core/types.hpp) -> Q.
const LIBGNSS_STATUS_TO_Q = [Q.NONE, Q.SINGLE, Q.DGPS, Q.FLOAT, Q.FIX, Q.PPP, Q.PPP_FIX, Q.DR];

/**
 * Guess what a file is from its name and first bytes.
 * Returns one of: obs, nav, sp3, clk, rtcm, ubx, pos, vis, csv, kml, json, toml, other.
 */
export function sniffKind(name, headBytes) {
  const lower = name.toLowerCase();
  const bytes = headBytes instanceof Uint8Array ? headBytes : new Uint8Array(headBytes || []);
  if (bytes.length >= 2 && bytes[0] === 0xb5 && bytes[1] === 0x62) return 'ubx';
  if (bytes.length >= 1 && bytes[0] === 0xd3) return 'rtcm';
  const text = new TextDecoder('latin1').decode(bytes.subarray(0, 4096));
  const firstLine = text.split(/\r?\n/, 1)[0] || '';
  if (firstLine.includes('RINEX VERSION / TYPE')) {
    const type = firstLine.charAt(20).toUpperCase();
    if (type === 'O') return 'obs';
    if (type === 'C') return 'clk';
    if ('NGHLBJIEFQ'.includes(type) || /NAV/.test(firstLine)) return 'nav';
  }
  if (/^#[a-d][PV]/.test(firstLine)) return 'sp3';
  if (firstLine.startsWith('epoch_index,week,tow,satellite,')) return 'vis';
  if (/^%/.test(firstLine) || /\.pos$/.test(lower)) return 'pos';
  if (/\.(rtcm3?|rtc)$/.test(lower)) return 'rtcm';
  if (/\.ubx$/.test(lower)) return 'ubx';
  if (/\.(obs|\d\do)$/.test(lower) || /_mo\.rnx$/.test(lower)) return 'obs';
  if (/\.(nav|\d\d[nglpfhq])$/.test(lower) || /_[a-z]n\.rnx$/.test(lower)) return 'nav';
  if (/\.(sp3|eph)$/.test(lower)) return 'sp3';
  if (/\.clk(_\d+s)?$/.test(lower)) return 'clk';
  if (/\.csv$/.test(lower)) return 'csv';
  if (/\.kml$/.test(lower)) return 'kml';
  if (/\.json$/.test(lower)) return 'json';
  if (/\.toml$/.test(lower)) return 'toml';
  return 'other';
}

/** Rover RINEX TIME OF FIRST / LAST OBS as continuous GPST seconds, if present. */
export function rinexObsTimeSpan(headerText) {
  const read = (label) => {
    const line = headerText.split(/\r?\n/).find((l) => l.slice(60).startsWith(label));
    if (!line) return null;
    const f = line.slice(0, 43).trim().split(/\s+/).map(Number);
    if (f.length < 6 || f.some((v) => !Number.isFinite(v))) return null;
    return calendarToGpsSeconds(f[0], f[1], f[2], f[3], f[4], f[5]);
  };
  return { first: read('TIME OF FIRST OBS'), last: read('TIME OF LAST OBS') };
}

function makeColumns(n) {
  return {
    t: new Float64Array(n),
    lat: new Float64Array(n),
    lon: new Float64Array(n),
    h: new Float64Array(n),
    x: new Float64Array(n),
    y: new Float64Array(n),
    z: new Float64Array(n),
    q: new Uint8Array(n),
    ns: new Float64Array(n).fill(NaN),
    ratio: new Float64Array(n).fill(NaN),
    pdop: new Float64Array(n).fill(NaN),
  };
}

function finish(name, format, rows) {
  rows.sort((a, b) => a.t - b.t);
  const n = rows.length;
  const c = makeColumns(n);
  for (let i = 0; i < n; i += 1) {
    const r = rows[i];
    c.t[i] = r.t;
    let { lat, lon, h, x, y, z } = r;
    if (!Number.isFinite(x)) [x, y, z] = llhToEcef(lat, lon, h);
    if (!Number.isFinite(lat)) [lat, lon, h] = ecefToLlh(x, y, z);
    c.lat[i] = lat; c.lon[i] = lon; c.h[i] = h;
    c.x[i] = x; c.y[i] = y; c.z[i] = z;
    c.q[i] = r.q;
    if (r.ns !== undefined) c.ns[i] = r.ns;
    if (r.ratio !== undefined) c.ratio[i] = r.ratio;
    if (r.pdop !== undefined) c.pdop[i] = r.pdop;
  }
  return { name, format, n, ...c };
}

// The libgnss++ writers name their columns in a "% [Columns: ]GPS_Week
// GPS_TOW ..." header line; the set differs between tools and --format.
const LIBGNSS_DEFAULT_COLUMNS = [
  'gps_week', 'gps_tow', 'x(m)', 'y(m)', 'z(m)', 'lat(deg)', 'lon(deg)', 'height(m)',
  'status', 'satellites', 'pdop', 'ratio',
];

function parseLibgnssPos(name, lines) {
  const header = lines.find((l) => /^%\s*(Columns:\s*)?GPS_Week\s+GPS_TOW/i.test(l));
  const columns = header
    ? header.replace(/^%\s*(Columns:\s*)?/i, '').trim().split(/\s+/).map((c) => c.toLowerCase())
    : LIBGNSS_DEFAULT_COLUMNS;
  const col = (...names) => names.map((n) => columns.indexOf(n)).find((i) => i >= 0) ?? -1;
  const iX = col('x(m)');
  const iLat = col('lat(deg)');
  const iH = col('height(m)');
  const iStatus = col('status');
  const iNs = col('satellites', 'numsat');
  const iPdop = col('pdop');
  const iRatio = col('ratio');
  if (iX < 0 && (iLat < 0 || iH < 0)) throw new Error(`${name}: no position columns in header`);
  const rows = [];
  for (const line of lines) {
    if (!line || line.startsWith('%')) continue;
    const v = line.trim().split(/\s+/).map(Number);
    if (v.length < 5) continue;
    const t = gpsSeconds(v[0], v[1]);
    if (!Number.isFinite(t)) continue;
    const row = { t, q: iStatus >= 0 ? (LIBGNSS_STATUS_TO_Q[v[iStatus]] ?? Q.NONE) : Q.NONE };
    if (iX >= 0) { row.x = v[iX]; row.y = v[iX + 1]; row.z = v[iX + 2]; }
    if (iLat >= 0) { row.lat = v[iLat]; row.lon = v[iLat + 1]; row.h = v[iH]; }
    if (!Number.isFinite(row.x ?? row.lat)) continue;
    if (iNs >= 0) row.ns = v[iNs];
    if (iPdop >= 0) row.pdop = v[iPdop];
    if (iRatio >= 0) row.ratio = v[iRatio];
    rows.push(row);
  }
  return finish(name, 'libgnss++ pos', rows);
}

function parseRtklibPos(name, lines) {
  const columnHeader = lines.find((l) => /^%\s*(GPST|UTC|JST)\b/.test(l)) || '';
  const timeSystem = (columnHeader.match(/^%\s*(GPST|UTC|JST)/) || [null, 'GPST'])[1];
  const xyz = /x-ecef/i.test(columnHeader);
  let offset = 0;
  if (timeSystem === 'UTC') offset = GPST_UTC_LEAP_SECONDS;
  if (timeSystem === 'JST') offset = GPST_UTC_LEAP_SECONDS - 9 * 3600;
  const rows = [];
  for (const line of lines) {
    if (!line || line.startsWith('%')) continue;
    const f = line.trim().split(/\s+/);
    if (f.length < 7) continue;
    let t;
    if (f[0].includes('/')) {
      const d = f[0].split('/').map(Number);
      const c = f[1].split(':').map(Number);
      t = calendarToGpsSeconds(d[0], d[1], d[2], c[0], c[1], c[2]);
    } else {
      t = gpsSeconds(Number(f[0]), Number(f[1]));
    }
    if (!Number.isFinite(t)) continue;
    const v = f.slice(2).map(Number);
    const row = { t: t + offset, q: Number.isFinite(v[3]) ? v[3] : Q.NONE, ns: v[4] };
    if (xyz) { row.x = v[0]; row.y = v[1]; row.z = v[2]; } else { row.lat = v[0]; row.lon = v[1]; row.h = v[2]; }
    // ... sdn sde sdu sdne sdeu sdun age ratio: ratio is column 13 when present.
    if (v.length >= 13) row.ratio = v[12];
    if (!Number.isFinite(row.x ?? row.lat)) continue;
    rows.push(row);
  }
  return finish(name, `RTKLIB pos (${timeSystem}${xyz ? ', xyz' : ''})`, rows);
}

function parseReferenceCsv(name, lines) {
  const header = lines[0].split(',').map((s) => s.trim().toLowerCase());
  const find = (re) => header.findIndex((h) => re.test(h));
  const iTow = find(/tow/);
  const iWeek = find(/week/);
  const iLat = find(/^lat/);
  const iLon = find(/^lon/);
  const iH = find(/height|^h(\b|\s)|alt/);
  const iX = find(/ecef x|^x\b/);
  if (iTow < 0 || iWeek < 0 || ((iLat < 0 || iLon < 0 || iH < 0) && iX < 0)) {
    throw new Error('CSV needs GPS week, TOW and latitude/longitude/height (or ECEF) columns');
  }
  const rows = [];
  for (const line of lines.slice(1)) {
    const f = line.split(',').map(Number);
    if (f.length < header.length) continue;
    const t = gpsSeconds(f[iWeek], f[iTow]);
    if (!Number.isFinite(t)) continue;
    const row = { t, q: Q.NONE };
    if (iLat >= 0 && iLon >= 0 && iH >= 0) {
      row.lat = f[iLat]; row.lon = f[iLon]; row.h = f[iH];
    } else {
      row.x = f[iX]; row.y = f[iX + 1]; row.z = f[iX + 2];
    }
    rows.push(row);
  }
  return finish(name, 'reference CSV', rows);
}

/**
 * Parse a solution or reference trajectory. Throws on unknown content.
 * Result: { name, format, n, t, lat, lon, h, x, y, z, q, ns, ratio, pdop }
 * with t in continuous GPST seconds.
 */
export function parseSolution(name, text) {
  const lines = text.split(/\r?\n/);
  const head = lines.slice(0, 40).join('\n');
  let sol;
  if (/LibGNSS\+\+|^%\s*(Columns:\s*)?GPS_Week\s+GPS_TOW/im.test(head)) {
    sol = parseLibgnssPos(name, lines);
  } else if (/^%\s*(GPST|UTC|JST)\b/m.test(head) || /^%\s*program\s*:/m.test(head)) {
    sol = parseRtklibPos(name, lines);
  } else if (lines[0] && lines[0].includes(',') && /[a-z]/i.test(lines[0])) {
    sol = parseReferenceCsv(name, lines);
  } else {
    throw new Error(`${name}: not a libgnss++ / RTKLIB solution or reference CSV`);
  }
  if (sol.n === 0) throw new Error(`${name}: no solution epochs found`);
  return sol;
}

// GNSS system order and one-letter prefixes used by the observation viewer.
export const SYSTEMS = ['GPS', 'GLONASS', 'Galileo', 'BeiDou', 'QZSS', 'SBAS', 'NavIC'];
const SYSTEM_PREFIX = { G: 0, R: 1, E: 2, C: 3, J: 4, S: 5, I: 6 };

/**
 * Parse a `gnss visibility` CSV (epoch_index,week,tow,satellite,system,
 * signal,azimuth_deg,elevation_deg,snr_dbhz,has_pseudorange,
 * has_carrier_phase,has_doppler[,lli]).
 * Result: { name, n, t, az, el, snr, lli, sat (Uint16 index into sats),
 *           sys (Uint8 index into SYSTEMS), signal: string[], sats: string[],
 *           epochs: Float64Array of distinct times, hasLli }
 * Satellites are sorted by system then PRN, as RTKPLOT lists them.
 */
export function parseVisibilityCsv(name, text) {
  const lines = text.split(/\r?\n/);
  const header = (lines[0] || '').split(',');
  const col = (key) => header.indexOf(key);
  const iWeek = col('week'); const iTow = col('tow'); const iSat = col('satellite');
  const iSig = col('signal'); const iAz = col('azimuth_deg'); const iEl = col('elevation_deg');
  const iSnr = col('snr_dbhz'); const iLli = col('lli');
  if ([iWeek, iTow, iSat, iAz, iEl].some((i) => i < 0)) {
    throw new Error(`${name}: not a gnss visibility CSV`);
  }
  const rows = [];
  for (let k = 1; k < lines.length; k += 1) {
    if (!lines[k]) continue;
    const f = lines[k].split(',');
    if (f.length <= iEl) continue;
    const t = gpsSeconds(Number(f[iWeek]), Number(f[iTow]));
    if (!Number.isFinite(t)) continue;
    rows.push({ t, f });
  }
  const names = [...new Set(rows.map((r) => r.f[iSat]))];
  const order = (s) => [SYSTEM_PREFIX[s[0]] ?? 9, Number(s.slice(1)) || 0];
  names.sort((a, b) => {
    const [sa, pa] = order(a); const [sb, pb] = order(b);
    return sa - sb || pa - pb || a.localeCompare(b);
  });
  const index = new Map(names.map((s, i) => [s, i]));
  const n = rows.length;
  const out = {
    name, n,
    t: new Float64Array(n), az: new Float64Array(n), el: new Float64Array(n),
    snr: new Float64Array(n), lli: new Uint8Array(n), sat: new Uint16Array(n), sys: new Uint8Array(n),
    signal: new Array(n), sats: names, hasLli: iLli >= 0,
  };
  for (let i = 0; i < n; i += 1) {
    const { t, f } = rows[i];
    out.t[i] = t;
    out.az[i] = Number(f[iAz]);
    out.el[i] = Number(f[iEl]);
    out.snr[i] = iSnr >= 0 && f[iSnr] !== '' ? Number(f[iSnr]) : NaN;
    out.lli[i] = iLli >= 0 ? Number(f[iLli]) || 0 : 0;
    out.sat[i] = index.get(f[iSat]);
    out.sys[i] = SYSTEM_PREFIX[f[iSat][0]] ?? 6;
    out.signal[i] = iSig >= 0 ? f[iSig] : '';
  }
  out.epochs = Float64Array.from(new Set(out.t)).sort();
  if (n === 0) throw new Error(`${name}: no visibility rows`);
  return out;
}
