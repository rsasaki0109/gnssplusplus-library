// Web Mercator ("slippy map") tile backgrounds for the ground-track plot.
// No map library: tiles are drawn straight into the plot canvas.
//
// The ground track stays in ENU metres around the plot origin. Each tile is
// placed by converting its corner latitude / longitude to ENU and drawn as the
// rectangle between its north-west and south-east corners. Within a tile the
// projection difference is a rotation by the meridian convergence
// (dlon * sin(lat)), well under a pixel at the spans a trajectory covers;
// the map is not drawn for views wider than MAX_SPAN_M.

import { ecefToLlh, enuFrame, llhToEcef } from './geo.js';

const R = 6378137.0; // Web Mercator sphere radius (m)
const DEG = Math.PI / 180;
const TILE_PX = 256;
export const MAX_SPAN_M = 300e3;
const RETRY_MS = 30e3; // a failed tile is requested again after this long

/**
 * Tile sources. `url` uses {z}/{x}/{y}; `attribution` is plain text and
 * `link` the page the attribution points to (both shown under the plot).
 */
export const TILE_SOURCES = {
  osm: {
    label: 'OpenStreetMap',
    url: 'https://tile.openstreetmap.org/{z}/{x}/{y}.png',
    maxZoom: 19,
    attribution: '© OpenStreetMap contributors',
    link: 'https://www.openstreetmap.org/copyright',
  },
  'gsi-std': {
    label: 'GSI standard map (Japan)',
    url: 'https://cyberjapandata.gsi.go.jp/xyz/std/{z}/{x}/{y}.png',
    maxZoom: 18,
    attribution: '地理院タイル (GSI Tiles)',
    link: 'https://maps.gsi.go.jp/development/ichiran.html',
  },
  'gsi-photo': {
    label: 'GSI aerial photo (Japan)',
    url: 'https://cyberjapandata.gsi.go.jp/xyz/seamlessphoto/{z}/{x}/{y}.jpg',
    maxZoom: 18,
    attribution: '地理院タイル 全国最新写真 (GSI Tiles, seamless photo)',
    link: 'https://maps.gsi.go.jp/development/ichiran.html',
  },
};

export function tileUrl(source, z, x, y) {
  return source.url.replace('{z}', z).replace('{x}', x).replace('{y}', y);
}

/** Tile column / row (fractional) of a point at zoom z. */
export function tileXY(latDeg, lonDeg, z) {
  const n = 2 ** z;
  const lat = Math.max(-85.05112878, Math.min(85.05112878, latDeg)) * DEG;
  return [((lonDeg + 180) / 360) * n, ((1 - Math.log(Math.tan(lat) + 1 / Math.cos(lat)) / Math.PI) / 2) * n];
}

/** Latitude / longitude (deg) of tile corner (x, y) at zoom z. */
export function tileCorner(x, y, z) {
  const n = 2 ** z;
  return [Math.atan(Math.sinh(Math.PI * (1 - (2 * y) / n))) / DEG, (x / n) * 360 - 180];
}

/**
 * ENU frame around origin [latDeg, lonDeg, h] with both directions:
 *   toEnu(latDeg, lonDeg) -> [e, n] at the origin height
 *   toLatLon(e, n) -> [latDeg, lonDeg] of the tangent-plane point
 */
export function enuPlane(origin) {
  const [lat0, lon0, h0] = origin;
  const frame = enuFrame(lat0, lon0, h0);
  const o = llhToEcef(lat0, lon0, h0);
  const sl = Math.sin(lat0 * DEG); const cl = Math.cos(lat0 * DEG);
  const so = Math.sin(lon0 * DEG); const co = Math.cos(lon0 * DEG);
  return {
    origin,
    toEnu: (latDeg, lonDeg) => frame(...llhToEcef(latDeg, lonDeg, h0)).slice(0, 2),
    toLatLon: (e, n) => {
      const [lat, lon] = ecefToLlh(o[0] - so * e - sl * co * n, o[1] + co * e - sl * so * n, o[2] + cl * n);
      return [lat, lon];
    },
  };
}

/**
 * Zoom level whose tiles show at about one tile pixel per screen pixel for a
 * view of `metresPerPixel` ground metres per CSS pixel at latitude latDeg.
 */
export function chooseZoom(metresPerPixel, latDeg, maxZoom, dpr = 1) {
  const res0 = ((2 * Math.PI * R) / TILE_PX) * Math.cos(latDeg * DEG); // m per tile pixel at zoom 0
  const z = Math.round(Math.log2((res0 * dpr) / metresPerPixel));
  return Math.max(0, Math.min(maxZoom, Number.isFinite(z) ? z : 0));
}

/**
 * Tiles covering ENU view {x0, x1, y0, y1} at zoom z, each with its ENU
 * rectangle {x0, x1, y0, y1}. Returns [] when more than maxTiles would be
 * needed.
 */
export function tilesForView(view, plane, z, maxTiles = 96) {
  const n = 2 ** z;
  let tx0 = Infinity; let tx1 = -Infinity; let ty0 = Infinity; let ty1 = -Infinity;
  for (const fx of [0, 0.5, 1]) {
    for (const fy of [0, 0.5, 1]) {
      const [lat, lon] = plane.toLatLon(view.x0 + fx * (view.x1 - view.x0), view.y0 + fy * (view.y1 - view.y0));
      const [tx, ty] = tileXY(lat, lon, z);
      tx0 = Math.min(tx0, tx); tx1 = Math.max(tx1, tx);
      ty0 = Math.min(ty0, ty); ty1 = Math.max(ty1, ty);
    }
  }
  const clamp = (v) => Math.max(0, Math.min(n - 1, Math.floor(v)));
  [tx0, tx1, ty0, ty1] = [clamp(tx0), clamp(tx1), clamp(ty0), clamp(ty1)];
  if ((tx1 - tx0 + 1) * (ty1 - ty0 + 1) > maxTiles) return [];
  const tiles = [];
  for (let ty = ty0; ty <= ty1; ty += 1) {
    for (let tx = tx0; tx <= tx1; tx += 1) {
      const [w, nTop] = plane.toEnu(...tileCorner(tx, ty, z));
      const [east, south] = plane.toEnu(...tileCorner(tx + 1, ty + 1, z));
      tiles.push({ z, x: tx, y: ty, x0: w, x1: east, y0: south, y1: nTop });
    }
  }
  return tiles;
}

/**
 * Browser tile cache + painter. paint() draws what is loaded (falling back to
 * a cached parent tile while a tile loads) and requests the rest; onChange()
 * runs when a requested tile arrives so the plot can repaint.
 */
export class TileLayer {
  constructor({ onChange, maxCached = 600 } = {}) {
    this.onChange = onChange || (() => {});
    this.maxCached = maxCached;
    this.cache = new Map(); // url -> { img, state: 'loading' | 'ok' | 'error' }
    this.version = 0;
    this.loaded = 0;
    this.failed = 0;
    this.pending = false;
  }

  entry(url) {
    let e = this.cache.get(url);
    if (e && !(e.state === 'error' && Date.now() - e.failedAt > RETRY_MS)) {
      this.cache.delete(url); // refresh LRU position
      this.cache.set(url, e);
      return e;
    }
    if (e) this.cache.delete(url);
    const img = new Image();
    img.crossOrigin = 'anonymous'; // keep the canvas untainted (the sources send CORS headers)
    img.decoding = 'async';
    e = { img, state: 'loading' };
    img.onload = () => { e.state = 'ok'; this.loaded += 1; this.changed(); };
    img.onerror = () => { e.state = 'error'; e.failedAt = Date.now(); this.failed += 1; this.changed(); };
    img.src = url;
    this.cache.set(url, e);
    while (this.cache.size > this.maxCached) this.cache.delete(this.cache.keys().next().value);
    return e;
  }

  changed() {
    this.version += 1;
    if (this.pending) return;
    this.pending = true;
    requestAnimationFrame(() => { this.pending = false; this.onChange(); });
  }

  /**
   * Paint tiles of `source` under ENU view `v` into ctx, given rect r (CSS px)
   * and toPixel helpers t.px / t.py. Returns { zoom, requested, missing }
   * (missing: still loading) or { tooWide: true }. Tiles are picked for one tile pixel per CSS pixel (not
   * per device pixel), as map clients do by default, to keep the load on the
   * tile servers low.
   */
  paint(ctx, source, plane, v, r, t) {
    if (Math.max(v.x1 - v.x0, v.y1 - v.y0) > MAX_SPAN_M) return { tooWide: true };
    let z = chooseZoom((v.x1 - v.x0) / r.width, plane.origin[0], source.maxZoom);
    let tiles = tilesForView(v, plane, z);
    while (!tiles.length && z > 0) { z -= 1; tiles = tilesForView(v, plane, z); }
    let missing = 0;
    for (const tile of tiles) {
      const e = this.entry(tileUrl(source, tile.z, tile.x, tile.y));
      if (e.state === 'ok') {
        this.drawTile(ctx, e.img, tile, t, 0, 0, TILE_PX);
        continue;
      }
      if (e.state === 'loading') missing += 1;
      // Parent tile (up to 4 levels up) already in the cache, scaled up.
      for (let up = 1; up <= 4 && tile.z - up >= 0; up += 1) {
        const k = 2 ** up;
        const parent = this.cache.get(tileUrl(source, tile.z - up, Math.floor(tile.x / k), Math.floor(tile.y / k)));
        if (parent && parent.state === 'ok') {
          const sub = TILE_PX / k;
          this.drawTile(ctx, parent.img, tile, t, (tile.x % k) * sub, (tile.y % k) * sub, sub);
          break;
        }
      }
    }
    return { zoom: z, requested: tiles.length, missing };
  }

  drawTile(ctx, img, tile, t, sx, sy, sSize) {
    const left = t.px(tile.x0);
    const right = t.px(tile.x1);
    const top = t.py(tile.y1);
    const bottom = t.py(tile.y0);
    // Whole CSS pixels, overlapping by a hair to avoid seams.
    const x = Math.floor(left);
    const y = Math.floor(top);
    ctx.drawImage(img, sx, sy, sSize, sSize, x, y, Math.ceil(right) - x + 0.5, Math.ceil(bottom) - y + 0.5);
  }
}
