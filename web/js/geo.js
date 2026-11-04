// WGS84 coordinate helpers and GPS time conversion (no dependencies).

const WGS84_A = 6378137.0;
const WGS84_F = 1.0 / 298.257223563;
const WGS84_E2 = WGS84_F * (2.0 - WGS84_F);
const DEG = Math.PI / 180.0;

export const SECONDS_PER_WEEK = 604800;
// 1980-01-06T00:00:00Z, the GPS time origin (GPST ignores leap seconds).
const GPS_EPOCH_MS = Date.UTC(1980, 0, 6);
// GPST - UTC since 2017-01-01. Only used to read UTC/JST-stamped RTKLIB files.
export const GPST_UTC_LEAP_SECONDS = 18;

/** Geodetic (deg, deg, m) -> ECEF [x, y, z] (m). */
export function llhToEcef(latDeg, lonDeg, h) {
  const lat = latDeg * DEG;
  const lon = lonDeg * DEG;
  const sinLat = Math.sin(lat);
  const n = WGS84_A / Math.sqrt(1.0 - WGS84_E2 * sinLat * sinLat);
  return [
    (n + h) * Math.cos(lat) * Math.cos(lon),
    (n + h) * Math.cos(lat) * Math.sin(lon),
    (n * (1.0 - WGS84_E2) + h) * sinLat,
  ];
}

/** ECEF (m) -> geodetic [latDeg, lonDeg, h]. */
export function ecefToLlh(x, y, z) {
  const p = Math.hypot(x, y);
  let lat = Math.atan2(z, p * (1.0 - WGS84_E2));
  let h = 0.0;
  for (let i = 0; i < 8; i += 1) {
    const sinLat = Math.sin(lat);
    const n = WGS84_A / Math.sqrt(1.0 - WGS84_E2 * sinLat * sinLat);
    h = p / Math.cos(lat) - n;
    lat = Math.atan2(z, p * (1.0 - WGS84_E2 * n / (n + h)));
  }
  return [lat / DEG, Math.atan2(y, x) / DEG, h];
}

/**
 * Returns a function mapping ECEF [x, y, z] to local [e, n, u] (m) around
 * the given geodetic origin.
 */
export function enuFrame(latDeg, lonDeg, h) {
  const origin = llhToEcef(latDeg, lonDeg, h);
  const lat = latDeg * DEG;
  const lon = lonDeg * DEG;
  const sl = Math.sin(lat);
  const cl = Math.cos(lat);
  const so = Math.sin(lon);
  const co = Math.cos(lon);
  return (x, y, z) => {
    const dx = x - origin[0];
    const dy = y - origin[1];
    const dz = z - origin[2];
    return [
      -so * dx + co * dy,
      -sl * co * dx - sl * so * dy + cl * dz,
      cl * co * dx + cl * so * dy + sl * dz,
    ];
  };
}

/** GPS week + time of week -> continuous GPST seconds since the GPS epoch. */
export function gpsSeconds(week, tow) {
  return week * SECONDS_PER_WEEK + tow;
}

/** Calendar GPST (Y, M, D, h, m, s) -> continuous GPST seconds. */
export function calendarToGpsSeconds(year, month, day, hour, minute, second) {
  const ms = Date.UTC(year, month - 1, day, hour, minute, 0) - GPS_EPOCH_MS;
  return ms / 1000 + second;
}

/** Continuous GPST seconds -> { week, tow }. */
export function splitGpsSeconds(t) {
  const week = Math.floor(t / SECONDS_PER_WEEK);
  return { week, tow: t - week * SECONDS_PER_WEEK };
}

/** "YYYY/MM/DD hh:mm:ss.s" for a continuous GPST value (no leap seconds). */
export function formatGpst(t, decimals = 1) {
  const whole = Math.floor(t);
  const date = new Date(GPS_EPOCH_MS + whole * 1000);
  const pad = (v, n = 2) => String(v).padStart(n, '0');
  const frac = (t - whole).toFixed(decimals).slice(1);
  return `${date.getUTCFullYear()}/${pad(date.getUTCMonth() + 1)}/${pad(date.getUTCDate())} `
    + `${pad(date.getUTCHours())}:${pad(date.getUTCMinutes())}:${pad(date.getUTCSeconds())}`
    + (decimals > 0 ? frac : '');
}

/** "hh:mm:ss" of a continuous GPST value, for time-axis ticks. */
export function formatGpstClock(t) {
  const s = ((Math.round(t) % 86400) + 86400) % 86400;
  const pad = (v) => String(v).padStart(2, '0');
  return `${pad(Math.floor(s / 3600))}:${pad(Math.floor((s % 3600) / 60))}:${pad(s % 60)}`;
}

/** Current GPS week from the host clock (the CLI's default for RTCM input). */
export function currentGpsWeek(nowMs = Date.now()) {
  const gpst = (nowMs - GPS_EPOCH_MS) / 1000 + GPST_UTC_LEAP_SECONDS;
  return Math.floor(gpst / SECONDS_PER_WEEK);
}
