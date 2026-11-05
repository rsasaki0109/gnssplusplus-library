// Minimal canvas plotting for the RTKPLOT-style view: points/lines with
// per-point classes, nice ticks, GPST time axes, wheel zoom, drag pan,
// double-click fit and a shared hover cursor.

import { formatGpstClock } from './geo.js';

const TIME_STEPS = [0.1, 0.2, 0.5, 1, 2, 5, 10, 15, 30, 60, 120, 300, 600, 900, 1800, 3600,
  7200, 10800, 21600, 43200, 86400];

function niceStep(span, target) {
  const raw = span / Math.max(1, target);
  const mag = 10 ** Math.floor(Math.log10(raw));
  const norm = raw / mag;
  return (norm < 1.5 ? 1 : norm < 3.5 ? 2 : norm < 7.5 ? 5 : 10) * mag;
}

function timeStep(span, target) {
  const raw = span / Math.max(1, target);
  return TIME_STEPS.find((s) => s >= raw) || TIME_STEPS[TIME_STEPS.length - 1];
}

function formatNumber(v, step) {
  const decimals = Math.max(0, Math.min(6, -Math.floor(Math.log10(step) + 1e-9)));
  return v.toFixed(decimals);
}

// Grid, axes and data are painted once into an offscreen canvas and reused
// while only the hover cursor changes: with a few hundred thousand points a
// full repaint per mouse move takes hundreds of milliseconds.
function prepareCanvas(canvas) {
  const dpr = window.devicePixelRatio || 1;
  const w = canvas.clientWidth;
  const h = canvas.clientHeight;
  if (w === 0 || h === 0) return null;
  if (canvas.width !== Math.round(w * dpr) || canvas.height !== Math.round(h * dpr)) {
    canvas.width = Math.round(w * dpr);
    canvas.height = Math.round(h * dpr);
  }
  return { dpr, w, h };
}

// Offscreen layer `slot` of owner (same size as owner.canvas), repainted only
// when its key changes.
function cachedLayer(owner, slot, key, paint) {
  const canvas = owner.canvas;
  owner.layerCache = owner.layerCache || {};
  let entry = owner.layerCache[slot];
  if (!entry) {
    entry = { canvas: document.createElement('canvas'), key: null };
    owner.layerCache[slot] = entry;
  }
  if (entry.key !== key || entry.canvas.width !== canvas.width || entry.canvas.height !== canvas.height) {
    entry.canvas.width = canvas.width;
    entry.canvas.height = canvas.height;
    paint(entry.canvas.getContext('2d'));
    entry.key = key;
  }
  return entry.canvas;
}

function cachedBase(owner, key, paint, under = null) {
  const canvas = owner.canvas;
  const base = cachedLayer(owner, 'base', key, paint);
  const ctx = canvas.getContext('2d');
  ctx.setTransform(1, 0, 0, 1, 0, 0);
  ctx.clearRect(0, 0, canvas.width, canvas.height);
  if (under) ctx.drawImage(under, 0, 0);
  ctx.drawImage(base, 0, 0);
  return ctx;
}

export function cssVar(name, fallback) {
  const value = getComputedStyle(document.documentElement).getPropertyValue(name).trim();
  return value || fallback;
}

/**
 * A plot bound to one canvas.
 *   opts.timeAxis     x values are GPST seconds
 *   opts.equalAspect  same metres per pixel on both axes (ground track)
 *   opts.yLabel / xLabel
 *   opts.integerY     y ticks on whole numbers (counts)
 *   opts.yTickFormat(v) -> label  custom y tick labels (e.g. satellite IDs)
 *   opts.yStep        fixed y tick step (overrides the automatic one)
 *   opts.onHover(x, y) / onLeave()
 *   opts.onViewChange(view)  after user zoom/pan (used to link time axes)
 *   opts.background  { active() -> bool, key() -> string,
 *                      paint(ctx, view, rect, toPixel, dpr) }  a layer under the
 *                    grid and data (map tiles), cached separately so it can
 *                    repaint without repainting the data
 */
export class PlotPanel {
  constructor(canvas, opts = {}) {
    this.canvas = canvas;
    this.opts = opts;
    this.layers = [];
    this.view = null;
    this.cursor = null;
    this.margin = { left: 58, right: 10, top: 8, bottom: opts.timeAxis || opts.xLabel ? 26 : 18 };
    this.observer = new ResizeObserver(() => this.draw());
    this.observer.observe(canvas);
    this.bindEvents();
  }

  /**
   * layers: [{ type: 'points' | 'line', x, y, cls?: Uint8Array, colors: {cls: color} | color,
   *            size?, width?, order? }]
   */
  setLayers(layers, { keepView = false } = {}) {
    this.layers = layers;
    this.version = (this.version || 0) + 1;
    if (!keepView || !this.view) this.fit();
    this.draw();
  }

  dataBounds() {
    let x0 = Infinity; let x1 = -Infinity; let y0 = Infinity; let y1 = -Infinity;
    for (const layer of this.layers) {
      for (let i = 0; i < layer.x.length; i += 1) {
        const x = layer.x[i]; const y = layer.y[i];
        if (!Number.isFinite(x) || !Number.isFinite(y)) continue;
        if (x < x0) x0 = x; if (x > x1) x1 = x;
        if (y < y0) y0 = y; if (y > y1) y1 = y;
      }
    }
    if (!Number.isFinite(x0)) return { x0: 0, x1: 1, y0: 0, y1: 1 };
    return { x0, x1, y0, y1 };
  }

  fit() {
    const b = this.dataBounds();
    const padX = (b.x1 - b.x0) * 0.04 || (this.opts.timeAxis ? 30 : 1);
    const padY = (b.y1 - b.y0) * 0.08 || 1;
    this.view = { x0: b.x0 - padX, x1: b.x1 + padX, y0: b.y0 - padY, y1: b.y1 + padY };
  }

  setXRange(x0, x1) {
    if (!this.view) this.fit();
    this.view.x0 = x0;
    this.view.x1 = x1;
    // Fit y to the data inside the new x window.
    let y0 = Infinity; let y1 = -Infinity;
    for (const layer of this.layers) {
      for (let i = 0; i < layer.x.length; i += 1) {
        const x = layer.x[i]; const y = layer.y[i];
        if (x < x0 || x > x1 || !Number.isFinite(y)) continue;
        if (y < y0) y0 = y; if (y > y1) y1 = y;
      }
    }
    if (Number.isFinite(y0)) {
      const pad = (y1 - y0) * 0.08 || 1;
      this.view.y0 = y0 - pad;
      this.view.y1 = y1 + pad;
    }
    this.draw();
  }

  setCursor(cursor) {
    this.cursor = cursor;
    this.draw();
  }

  plotRect() {
    const w = this.canvas.clientWidth;
    const h = this.canvas.clientHeight;
    const m = this.margin;
    return { left: m.left, top: m.top, width: Math.max(10, w - m.left - m.right), height: Math.max(10, h - m.top - m.bottom) };
  }

  // Effective view (with equal-aspect expansion) for the current canvas size.
  effectiveView() {
    const v = { ...this.view };
    if (this.opts.equalAspect) {
      const r = this.plotRect();
      const sx = (v.x1 - v.x0) / r.width;
      const sy = (v.y1 - v.y0) / r.height;
      const s = Math.max(sx, sy);
      const cx = (v.x0 + v.x1) / 2;
      const cy = (v.y0 + v.y1) / 2;
      v.x0 = cx - (s * r.width) / 2; v.x1 = cx + (s * r.width) / 2;
      v.y0 = cy - (s * r.height) / 2; v.y1 = cy + (s * r.height) / 2;
    }
    return v;
  }

  toPixel(v, r) {
    const kx = r.width / (v.x1 - v.x0);
    const ky = r.height / (v.y1 - v.y0);
    return {
      px: (x) => r.left + (x - v.x0) * kx,
      py: (y) => r.top + r.height - (y - v.y0) * ky,
      dx: (px) => v.x0 + (px - r.left) / kx,
      dy: (py) => v.y0 + (r.top + r.height - py) / ky,
    };
  }

  draw() {
    const size = prepareCanvas(this.canvas);
    if (!size) return;
    if (!this.view) this.fit();
    const v = this.effectiveView();
    const bg = this.opts.background && this.opts.background.active() ? this.opts.background : null;
    const viewKey = [size.w, size.h, size.dpr, v.x0, v.x1, v.y0, v.y1].join('|');
    const key = [viewKey, this.version, this.yStepKey(), bg ? 'bg' : ''].join('|');
    const under = bg ? cachedLayer(this, 'background', `${viewKey}|${bg.key()}`, (layer) => this.paintBackground(layer, size, bg)) : null;
    const ctx = cachedBase(this, key, (base) => this.paintBase(base, size, Boolean(bg)), under);
    ctx.setTransform(size.dpr, 0, 0, size.dpr, 0, 0);
    this.paintCursor(ctx);
  }

  paintBackground(ctx, { dpr, w, h }, bg) {
    ctx.setTransform(dpr, 0, 0, dpr, 0, 0);
    ctx.fillStyle = cssVar('--plot-bg', '#fff');
    ctx.fillRect(0, 0, w, h);
    const r = this.plotRect();
    const v = this.effectiveView();
    ctx.save();
    ctx.beginPath();
    ctx.rect(r.left, r.top, r.width, r.height);
    ctx.clip();
    bg.paint(ctx, v, r, this.toPixel(v, r), dpr);
    ctx.restore();
  }

  yStepKey() {
    return `${this.opts.yStep || ''}:${this.opts.integerY ? 1 : 0}:${this.opts.yLabel || ''}`;
  }

  paintCursor(ctx) {
    if (!this.cursor || !this.view) return;
    const r = this.plotRect();
    const t = this.toPixel(this.effectiveView(), r);
    ctx.save();
    ctx.beginPath();
    ctx.rect(r.left, r.top, r.width, r.height);
    ctx.clip();
    ctx.strokeStyle = cssVar('--accent', '#0f766e');
    ctx.lineWidth = 1;
    if (this.cursor.x !== undefined && this.cursor.y === undefined) {
      const px = Math.round(t.px(this.cursor.x)) + 0.5;
      ctx.beginPath(); ctx.moveTo(px, r.top); ctx.lineTo(px, r.top + r.height); ctx.stroke();
    } else if (this.cursor.y !== undefined) {
      const px = t.px(this.cursor.x); const py = t.py(this.cursor.y);
      ctx.lineWidth = 2;
      ctx.beginPath(); ctx.arc(px, py, 7, 0, Math.PI * 2); ctx.stroke();
    }
    ctx.restore();
  }

  // With a background layer the plot area is left transparent so the layer
  // painted under this one shows through.
  paintBase(ctx, { dpr, w, h }, transparentPlot = false) {
    ctx.setTransform(dpr, 0, 0, dpr, 0, 0);
    const colors = {
      bg: cssVar('--plot-bg', '#fff'),
      grid: transparentPlot ? cssVar('--map-grid', 'rgba(0, 0, 0, 0.12)') : cssVar('--plot-grid', '#e5e7eb'),
      axis: cssVar('--plot-axis', '#6b7280'),
      text: cssVar('--muted', '#6b7280'),
    };
    ctx.fillStyle = colors.bg;
    ctx.fillRect(0, 0, w, h);
    const r = this.plotRect();
    const v = this.effectiveView();
    const t = this.toPixel(v, r);
    if (transparentPlot) ctx.clearRect(r.left, r.top, r.width, r.height);

    // Grid and tick labels.
    ctx.font = '11px system-ui, sans-serif';
    ctx.fillStyle = colors.text;
    ctx.strokeStyle = colors.grid;
    ctx.lineWidth = 1;
    const xStep = this.opts.timeAxis ? timeStep(v.x1 - v.x0, r.width / 90) : niceStep(v.x1 - v.x0, r.width / 80);
    let yStep = niceStep(v.y1 - v.y0, r.height / 36);
    if (this.opts.integerY) yStep = Math.max(1, Math.round(yStep));
    if (this.opts.yStep) yStep = this.opts.yStep;
    ctx.textAlign = 'center';
    ctx.textBaseline = 'top';
    for (let x = Math.ceil(v.x0 / xStep) * xStep; x <= v.x1; x += xStep) {
      const px = Math.round(t.px(x)) + 0.5;
      ctx.beginPath(); ctx.moveTo(px, r.top); ctx.lineTo(px, r.top + r.height); ctx.stroke();
      const label = this.opts.timeAxis ? formatGpstClock(x) : formatNumber(x, xStep);
      ctx.fillText(label, px, r.top + r.height + 4);
    }
    ctx.textAlign = 'right';
    ctx.textBaseline = 'middle';
    for (let y = Math.ceil(v.y0 / yStep) * yStep; y <= v.y1; y += yStep) {
      const py = Math.round(t.py(y)) + 0.5;
      ctx.beginPath(); ctx.moveTo(r.left, py); ctx.lineTo(r.left + r.width, py); ctx.stroke();
      ctx.fillText(this.opts.yTickFormat ? this.opts.yTickFormat(y) : formatNumber(y, yStep), r.left - 6, py);
    }
    ctx.strokeStyle = colors.axis;
    ctx.strokeRect(r.left + 0.5, r.top + 0.5, r.width, r.height);
    if (this.opts.yLabel) {
      ctx.save();
      ctx.translate(12, r.top + r.height / 2);
      ctx.rotate(-Math.PI / 2);
      ctx.textAlign = 'center';
      ctx.fillText(this.opts.yLabel, 0, 0);
      ctx.restore();
    }
    if (this.opts.xLabel) {
      ctx.textAlign = 'right';
      ctx.textBaseline = 'bottom';
      if (transparentPlot) { // keep the label legible over map tiles
        ctx.save();
        ctx.lineWidth = 3;
        ctx.lineJoin = 'round';
        ctx.strokeStyle = colors.bg;
        ctx.strokeText(this.opts.xLabel, r.left + r.width - 4, r.top + r.height - 3);
        ctx.restore();
      }
      ctx.fillText(this.opts.xLabel, r.left + r.width - 4, r.top + r.height - 3);
    }

    // Data, clipped to the plot area.
    ctx.save();
    ctx.beginPath();
    ctx.rect(r.left, r.top, r.width, r.height);
    ctx.clip();
    const layers = [...this.layers].sort((a, b) => (a.order || 0) - (b.order || 0));
    for (const layer of layers) {
      if (layer.type === 'line') this.drawLine(ctx, layer, t);
      else if (layer.type === 'vticks') this.drawTicks(ctx, layer, t, v);
      else this.drawPoints(ctx, layer, t, v);
    }
    ctx.restore();
  }

  drawLine(ctx, layer, t) {
    ctx.strokeStyle = layer.colors;
    ctx.lineWidth = layer.width || 1;
    ctx.beginPath();
    let pen = false;
    for (let i = 0; i < layer.x.length; i += 1) {
      const x = layer.x[i]; const y = layer.y[i];
      if (!Number.isFinite(x) || !Number.isFinite(y)) { pen = false; continue; }
      if (pen) ctx.lineTo(t.px(x), t.py(y)); else ctx.moveTo(t.px(x), t.py(y));
      pen = true;
    }
    ctx.stroke();
  }

  // Short vertical marks centred on each point (cycle slips on a timeline).
  drawTicks(ctx, layer, t, v) {
    const half = (layer.size || 10) / 2;
    ctx.strokeStyle = layer.colors;
    ctx.lineWidth = layer.width || 2;
    ctx.beginPath();
    for (let i = 0; i < layer.x.length; i += 1) {
      const x = layer.x[i]; const y = layer.y[i];
      if (!(x >= v.x0 && x <= v.x1 && y >= v.y0 && y <= v.y1)) continue;
      const px = Math.round(t.px(x)) + 0.5;
      const py = t.py(y);
      ctx.moveTo(px, py - half);
      ctx.lineTo(px, py + half);
    }
    ctx.stroke();
  }

  drawPoints(ctx, layer, t, v) {
    const size = layer.size || 3;
    const half = size / 2;
    const draw = (color, pick) => {
      ctx.fillStyle = color;
      for (let i = 0; i < layer.x.length; i += 1) {
        if (pick && !pick(i)) continue;
        const x = layer.x[i]; const y = layer.y[i];
        if (!(x >= v.x0 && x <= v.x1 && y >= v.y0 && y <= v.y1)) continue;
        ctx.fillRect(t.px(x) - half, t.py(y) - half, size, size);
      }
    };
    if (typeof layer.colors === 'string' || !layer.cls) {
      draw(typeof layer.colors === 'string' ? layer.colors : '#888');
      return;
    }
    // Draw low-quality classes last so they stay visible (as RTKPLOT does).
    const classes = Object.keys(layer.colors).map(Number).sort((a, b) => a - b);
    for (const cls of classes) draw(layer.colors[cls], (i) => layer.cls[i] === cls);
  }

  bindEvents() {
    const canvas = this.canvas;
    let drag = null;
    const local = (event) => {
      const rect = canvas.getBoundingClientRect();
      return { x: event.clientX - rect.left, y: event.clientY - rect.top };
    };
    const transform = () => this.toPixel(this.effectiveView(), this.plotRect());
    const commit = () => {
      this.draw();
      if (this.opts.onViewChange) this.opts.onViewChange(this.view);
    };

    canvas.addEventListener('wheel', (event) => {
      if (!this.view) return;
      event.preventDefault();
      const p = local(event);
      const tr = transform();
      const cx = tr.dx(p.x); const cy = tr.dy(p.y);
      const k = event.deltaY > 0 ? 1.25 : 0.8;
      const v = this.effectiveView();
      const zoomY = this.opts.equalAspect || event.shiftKey;
      const zoomX = this.opts.equalAspect || !event.shiftKey;
      this.view = {
        x0: zoomX ? cx - (cx - v.x0) * k : v.x0,
        x1: zoomX ? cx + (v.x1 - cx) * k : v.x1,
        y0: zoomY ? cy - (cy - v.y0) * k : v.y0,
        y1: zoomY ? cy + (v.y1 - cy) * k : v.y1,
      };
      commit();
    }, { passive: false });

    canvas.addEventListener('pointerdown', (event) => {
      if (!this.view) return;
      canvas.setPointerCapture(event.pointerId);
      drag = { start: local(event), view: this.effectiveView() };
    });
    canvas.addEventListener('pointermove', (event) => {
      const p = local(event);
      if (drag) {
        const r = this.plotRect();
        const v = drag.view;
        const ddx = ((p.x - drag.start.x) / r.width) * (v.x1 - v.x0);
        const ddy = ((p.y - drag.start.y) / r.height) * (v.y1 - v.y0);
        this.view = { x0: v.x0 - ddx, x1: v.x1 - ddx, y0: v.y0 + ddy, y1: v.y1 + ddy };
        commit();
        return;
      }
      if (this.opts.onHover && this.view) {
        const tr = transform();
        this.opts.onHover(tr.dx(p.x), tr.dy(p.y));
      }
    });
    const end = () => { drag = null; };
    canvas.addEventListener('pointerup', end);
    canvas.addEventListener('pointercancel', end);
    canvas.addEventListener('pointerleave', () => { if (this.opts.onLeave) this.opts.onLeave(); });
    canvas.addEventListener('dblclick', () => {
      this.fit();
      commit();
    });
  }

  /** Data distance (in pixels) helper for nearest-point hover. */
  pixelScale() {
    const r = this.plotRect();
    const v = this.effectiveView();
    return { kx: r.width / (v.x1 - v.x0), ky: r.height / (v.y1 - v.y0) };
  }
}

/**
 * Polar azimuth / elevation plot (RTKPLOT skyplot). North up, east right,
 * zenith at the centre, 0 deg elevation on the outer ring.
 *   setData({ az, el, cls, colors, labels: [{ az, el, text, color }] })
 *   setCursor([{ az, el, text, color }])  highlighted satellites
 *   opts.onHover(index | -1)  nearest sample within 12 px
 */
export class SkyPlot {
  constructor(canvas, opts = {}) {
    this.canvas = canvas;
    this.opts = opts;
    this.data = null;
    this.cursor = [];
    new ResizeObserver(() => this.draw()).observe(canvas);
    canvas.addEventListener('pointermove', (event) => this.hover(event));
    canvas.addEventListener('pointerleave', () => this.opts.onHover && this.opts.onHover(-1));
  }

  setData(data) { this.data = data; this.version = (this.version || 0) + 1; this.draw(); }

  setCursor(marks) { this.cursor = marks || []; this.draw(); }

  geometry() {
    const w = this.canvas.clientWidth;
    const h = this.canvas.clientHeight;
    return { cx: w / 2, cy: h / 2, radius: Math.max(10, Math.min(w, h) / 2 - 22) };
  }

  project(az, el, g) {
    const r = g.radius * (1 - Math.max(0, Math.min(90, el)) / 90);
    const a = (az * Math.PI) / 180;
    return [g.cx + r * Math.sin(a), g.cy - r * Math.cos(a)];
  }

  draw() {
    const size = prepareCanvas(this.canvas);
    if (!size) return;
    const ctx = cachedBase(this, [size.w, size.h, size.dpr, this.version].join('|'), (base) => this.paintBase(base, size));
    ctx.setTransform(size.dpr, 0, 0, size.dpr, 0, 0);
    const g = this.geometry();
    ctx.textBaseline = 'middle';
    ctx.textAlign = 'center';
    ctx.font = 'bold 10px system-ui, sans-serif';
    for (const mark of this.cursor) {
      const [x, y] = this.project(mark.az, mark.el, g);
      ctx.fillStyle = mark.color;
      ctx.beginPath(); ctx.arc(x, y, 9, 0, Math.PI * 2); ctx.fill();
      ctx.fillStyle = '#fff';
      ctx.fillText(mark.text, x, y + 0.5);
    }
  }

  paintBase(ctx, { dpr, w, h }) {
    ctx.setTransform(dpr, 0, 0, dpr, 0, 0);
    ctx.fillStyle = cssVar('--plot-bg', '#fff');
    ctx.fillRect(0, 0, w, h);
    const g = this.geometry();
    const grid = cssVar('--plot-grid', '#e5e7eb');
    const text = cssVar('--muted', '#6b7280');
    ctx.strokeStyle = grid;
    ctx.lineWidth = 1;
    for (const el of [0, 15, 30, 45, 60, 75]) {
      ctx.beginPath();
      ctx.arc(g.cx, g.cy, g.radius * (1 - el / 90), 0, Math.PI * 2);
      ctx.stroke();
    }
    for (let az = 0; az < 360; az += 30) {
      const [x, y] = this.project(az, 0, g);
      ctx.beginPath(); ctx.moveTo(g.cx, g.cy); ctx.lineTo(x, y); ctx.stroke();
    }
    ctx.strokeStyle = cssVar('--plot-axis', '#9ca3af');
    ctx.beginPath(); ctx.arc(g.cx, g.cy, g.radius, 0, Math.PI * 2); ctx.stroke();
    ctx.fillStyle = text;
    ctx.font = '11px system-ui, sans-serif';
    ctx.textAlign = 'center';
    ctx.textBaseline = 'middle';
    for (const [label, az] of [['N', 0], ['E', 90], ['S', 180], ['W', 270]]) {
      const [x, y] = this.project(az, -14, g);
      ctx.fillText(label, x, y);
    }
    ctx.textAlign = 'left';
    for (const el of [30, 60]) ctx.fillText(`${el}°`, g.cx + 3, g.cy - g.radius * (1 - el / 90) + 8);

    const d = this.data;
    if (d) {
      const classes = Object.keys(d.colors).map(Number);
      for (const cls of classes) {
        ctx.fillStyle = d.colors[cls];
        for (let i = 0; i < d.az.length; i += 1) {
          if (d.cls[i] !== cls || !Number.isFinite(d.el[i])) continue;
          const [x, y] = this.project(d.az[i], d.el[i], g);
          ctx.fillRect(x - 1, y - 1, 2.2, 2.2);
        }
      }
      ctx.font = '10px system-ui, sans-serif';
      ctx.textAlign = 'center';
      ctx.textBaseline = 'bottom';
      for (const label of d.labels || []) {
        const [x, y] = this.project(label.az, label.el, g);
        ctx.fillStyle = label.color || text;
        ctx.fillText(label.text, x, y - 3);
      }
    }
  }

  hover(event) {
    if (!this.opts.onHover || !this.data) return;
    const rect = this.canvas.getBoundingClientRect();
    const px = event.clientX - rect.left;
    const py = event.clientY - rect.top;
    const g = this.geometry();
    let best = -1;
    let bestD = 144;
    for (let i = 0; i < this.data.az.length; i += 1) {
      if (!(this.data.cls[i] in this.data.colors)) continue;
      const [x, y] = this.project(this.data.az[i], this.data.el[i], g);
      const dd = (x - px) ** 2 + (y - py) ** 2;
      if (dd < bestD) { bestD = dd; best = i; }
    }
    this.opts.onHover(best);
  }
}
