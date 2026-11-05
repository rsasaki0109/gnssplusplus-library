// Solution analysis for the plot view: local ENU, reference matching and
// RTKPLOT-style statistics. Pure functions over parseSolution() results.

import { enuFrame, ecefToLlh } from './geo.js';
import { Q, Q_LABELS } from './formats.js';

/** Mean geodetic position of a solution (through ECEF, so it is well defined). */
export function meanLlh(sol) {
  let x = 0; let y = 0; let z = 0; let n = 0;
  for (let i = 0; i < sol.n; i += 1) {
    if (!Number.isFinite(sol.x[i])) continue;
    x += sol.x[i]; y += sol.y[i]; z += sol.z[i]; n += 1;
  }
  if (n === 0) throw new Error('solution has no finite positions');
  return ecefToLlh(x / n, y / n, z / n);
}

/** Origin for the local frame. mode: 'mean' | 'first' | 'reference' | [lat, lon, h]. */
export function chooseOrigin(mode, sol, reference) {
  if (Array.isArray(mode)) return mode;
  if (mode === 'reference' && reference) return meanLlh(reference);
  if (mode === 'first') return [sol.lat[0], sol.lon[0], sol.h[0]];
  return meanLlh(sol);
}

/** Local E/N/U arrays of a solution around origin [lat, lon, h]. */
export function toEnu(sol, origin) {
  const frame = enuFrame(origin[0], origin[1], origin[2]);
  const e = new Float64Array(sol.n);
  const n = new Float64Array(sol.n);
  const u = new Float64Array(sol.n);
  for (let i = 0; i < sol.n; i += 1) {
    const v = frame(sol.x[i], sol.y[i], sol.z[i]);
    e[i] = v[0]; n[i] = v[1]; u[i] = v[2];
  }
  return { e, n, u };
}

/** Index of the element of sorted t nearest to value (or -1 for empty t). */
export function nearestIndex(t, value) {
  if (t.length === 0) return -1;
  let lo = 0;
  let hi = t.length - 1;
  while (hi - lo > 1) {
    const mid = (lo + hi) >> 1;
    if (t[mid] < value) lo = mid; else hi = mid;
  }
  return Math.abs(t[lo] - value) <= Math.abs(t[hi] - value) ? lo : hi;
}

/**
 * Per-epoch E/N/U error of sol against a reference trajectory, both in the
 * frame of `origin`. Epochs without a reference sample within tolerance get NaN.
 */
export function referenceErrors(sol, reference, origin, toleranceS = 0.005) {
  const s = toEnu(sol, origin);
  const r = toEnu(reference, origin);
  const de = new Float64Array(sol.n).fill(NaN);
  const dn = new Float64Array(sol.n).fill(NaN);
  const du = new Float64Array(sol.n).fill(NaN);
  for (let i = 0; i < sol.n; i += 1) {
    const j = nearestIndex(reference.t, sol.t[i]);
    if (j < 0 || Math.abs(reference.t[j] - sol.t[i]) > toleranceS) continue;
    de[i] = s.e[i] - r.e[j];
    dn[i] = s.n[i] - r.n[j];
    du[i] = s.u[i] - r.u[j];
  }
  return { e: de, n: dn, u: du };
}

function percentile(sorted, p) {
  if (sorted.length === 0) return NaN;
  const k = Math.min(sorted.length - 1, Math.max(0, Math.ceil(p * sorted.length) - 1));
  return sorted[k];
}

function moments(values) {
  let n = 0; let sum = 0; let sq = 0;
  for (const v of values) {
    if (!Number.isFinite(v)) continue;
    n += 1; sum += v; sq += v * v;
  }
  if (n === 0) return { n, mean: NaN, std: NaN, rms: NaN };
  const mean = sum / n;
  return { n, mean, std: Math.sqrt(Math.max(0, sq / n - mean * mean)), rms: Math.sqrt(sq / n) };
}

/** Error summary over the epochs selected by mask(i). */
export function errorSummary(err, mask = () => true) {
  const e = []; const n = []; const u = []; const h = [];
  for (let i = 0; i < err.e.length; i += 1) {
    if (!mask(i) || !Number.isFinite(err.e[i])) continue;
    e.push(err.e[i]); n.push(err.n[i]); u.push(err.u[i]);
    h.push(Math.hypot(err.e[i], err.n[i]));
  }
  const hs = Float64Array.from(h).sort();
  const us = Float64Array.from(u.map(Math.abs)).sort();
  const hm = moments(h);
  return {
    matched: h.length,
    e: moments(e),
    n: moments(n),
    u: moments(u),
    horizontalRms: Math.sqrt(h.reduce((a, v) => a + v * v, 0) / (h.length || NaN)),
    horizontal50: percentile(hs, 0.5),
    horizontal95: percentile(hs, 0.95),
    horizontalMax: hs.length ? hs[hs.length - 1] : NaN,
    vertical95: percentile(us, 0.95),
    horizontalMean: hm.mean,
  };
}

/** Counts per quality class plus span and E/N/U spread about the origin. */
export function solutionSummary(sol, enu) {
  const counts = {};
  for (let i = 0; i < sol.n; i += 1) {
    const label = Q_LABELS[sol.q[i]] || 'NONE';
    counts[label] = (counts[label] || 0) + 1;
  }
  const fixCount = (counts.FIX || 0) + (counts['PPP-FIX'] || 0);
  return {
    epochs: sol.n,
    start: sol.t[0],
    end: sol.t[sol.n - 1],
    counts,
    fixRate: sol.n ? fixCount / sol.n : NaN,
    e: moments(enu.e),
    n: moments(enu.n),
    u: moments(enu.u),
  };
}

export function isFixQuality(q) {
  return q === Q.FIX || q === Q.PPP_FIX;
}

/** Horizontal-error bin edges (m) for colouring a track by its error. */
export const ERROR_BINS_M = [0.1, 0.3, 1, 3];

/**
 * Per-epoch class of the horizontal error: 0 .. ERROR_BINS_M.length for the
 * bins below each edge and above the last, ERROR_BINS_M.length + 1 where the
 * epoch has no reference match.
 */
export function errorClasses(err, bins = ERROR_BINS_M) {
  const cls = new Uint8Array(err.e.length);
  for (let i = 0; i < cls.length; i += 1) {
    const h = Math.hypot(err.e[i], err.n[i]);
    if (!Number.isFinite(h)) { cls[i] = bins.length + 1; continue; }
    let k = 0;
    while (k < bins.length && h >= bins[k]) k += 1;
    cls[i] = k;
  }
  return cls;
}

/** Legend labels for errorClasses(): "< 0.1 m", "0.1-0.3 m", ..., "≥ 3 m", "no reference". */
export function errorClassLabels(bins = ERROR_BINS_M) {
  const labels = [`< ${bins[0]} m`];
  for (let k = 1; k < bins.length; k += 1) labels.push(`${bins[k - 1]}–${bins[k]} m`);
  labels.push(`≥ ${bins[bins.length - 1]} m`, 'no reference');
  return labels;
}
