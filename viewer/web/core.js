// core.js: geometry, sampling, tier alignment and palettes. Renderer-free; no verdicts are computed here
// (scores, flags and judge claims come from the data). Alignment modes:
//   event     piecewise-linear through anchors both runs share (task_started, gate_cleared n, and task_finished
//             only when both finished the same way); past the last anchor the offset carries, never a stretch
//   progress  fraction of path length; flagged "route diverged" when lengths differ by more than 1.5x
//   relative  both clocks start at task_started, no warp

export function bisect(a, t) {
  let lo = 0, hi = a.length - 1;
  if (t <= a[0]) return 0;
  if (t >= a[hi]) return hi;
  while (hi - lo > 1) { const m = (lo + hi) >> 1; a[m] <= t ? (lo = m) : (hi = m); }
  return lo;
}
export const wrap = (d) => Math.atan2(Math.sin(d), Math.cos(d));

/** The pose at t: linear between samples, yaw the short way round; `clamped` when t is outside the run. */
export function sample(tr, t) {
  const T = tr.t, n = T.length;
  if (!n) return null;
  const i = bisect(T, t), j = Math.min(i + 1, n - 1);
  const f = j === i ? 0 : Math.max(0, Math.min(1, (t - T[i]) / (T[j] - T[i])));
  return { i, x: tr.x[i] + (tr.x[j] - tr.x[i]) * f, y: tr.y[i] + (tr.y[j] - tr.y[i]) * f,
    yaw: tr.yaw[i] + wrap(tr.yaw[j] - tr.yaw[i]) * f, v: tr.v[i] + (tr.v[j] - tr.v[i]) * f,
    clamped: t < T[0] || t > T[n - 1] };
}
export function arc(tr) {
  if (tr._arc) return tr._arc;
  const s = new Float64Array(tr.t.length);
  for (let i = 1; i < s.length; i++) s[i] = s[i - 1] + Math.hypot(tr.x[i] - tr.x[i - 1], tr.y[i] - tr.y[i - 1]);
  return (tr._arc = s);
}
export function pathTo(tr, t) { const s = arc(tr); return s[bisect(tr.t, t)] || 0; }
const interp = (X, Y, x) => { const i = bisect(X, x), j = Math.min(i + 1, X.length - 1); const f = j === i ? 0 : Math.max(0, Math.min(1, (x - X[i]) / (X[j] - X[i]))); return Y[i] + (Y[j] - Y[i]) * f; };

export function anchors(run) {
  const a = { task_started: 0 };
  let out = null;
  for (const e of run.events) {
    if (e.kind === 'gate_cleared' && !(`gate ${e.gate}` in a)) a[`gate ${e.gate}`] = e.t;
    if (e.kind === 'task_finished') { a.task_finished = e.t; out = e.outcome ?? e.text; }
  }
  return { a, out };
}

export function makeAlign(ref, other, mode) {
  if (!ref || !other) return null;
  if (mode === 'relative') return { mode, anchors: [['task_started', 0, 0]], map: (t) => t, inv: (t) => t, confidence: 'n/a', note: 'both clocks start at task_started; no warp' };
  if (mode === 'progress') {
    const sa = arc(ref.track), sb = arc(other.track), La = sa[sa.length - 1] || 1, Lb = sb[sb.length - 1] || 1;
    const map = (t) => interp(sb, other.track.t, (interp(ref.track.t, sa, t) / La) * Lb);
    const inv = (t) => interp(sa, ref.track.t, (interp(other.track.t, sb, t) / Lb) * La);
    const ratio = Math.max(La, Lb) / Math.min(La, Lb);
    return { mode, anchors: [['path 0%', 0, 0], ['path 100%', ref.track.t.at(-1), other.track.t.at(-1)]], map, inv, pathRatio: ratio,
      confidence: ratio > 1.5 ? 'low' : 'ok',
      note: ratio > 1.5 ? `route diverged: ${La.toFixed(1)} m vs ${Lb.toFixed(1)} m (×${ratio.toFixed(1)}), residuals hidden` : `by fraction of path (${La.toFixed(1)} m vs ${Lb.toFixed(1)} m)` };
  }
  const A = anchors(ref), B = anchors(other);
  if (A.out !== B.out) { delete A.a.task_finished; delete B.a.task_finished; } // a give-up is not a finish line
  const pts = Object.keys(A.a).filter((k) => k in B.a).map((k) => [k, A.a[k], B.a[k]]).sort((p, q) => p[1] - q[1]);
  const xs = pts.map((p) => p[1]), ys = pts.map((p) => p[2]);
  const lin = (X, Y, t) => {
    if (X.length === 1 || t <= X[0]) return Y[0] + (t - X[0]);
    if (t >= X[X.length - 1]) return Y[Y.length - 1] + (t - X[X.length - 1]);
    let i = 0; while (i < X.length - 2 && t > X[i + 1]) i++;
    return Y[i] + ((Y[i + 1] - Y[i]) * (t - X[i])) / (X[i + 1] - X[i] || 1e-9);
  };
  return { mode: 'event', anchors: pts, map: (t) => lin(xs, ys, t), inv: (t) => lin(ys, xs, t), confidence: pts.length >= 2 ? 'ok' : 'low',
    note: pts.length >= 2 ? `warped through ${pts.length} shared anchors` : 'only task_started is shared: after it the clocks run free' };
}

export function residuals(ref, other, al, dt = 0.5) {
  if (!al || (al.mode === 'progress' && al.confidence === 'low')) return [];
  const out = [], t1 = ref.track.t.at(-1);
  for (let t = Math.max(0, ref.track.t[0]); t <= t1; t += dt) {
    const a = sample(ref.track, t), tb = al.map(t), b = sample(other.track, tb);
    if (!a || !b) continue;
    out.push({ t, tb, ax: a.x, ay: a.y, bx: b.x, by: b.y, d: Math.hypot(a.x - b.x, a.y - b.y), dpsi: wrap(b.yaw - a.yaw), dv: b.v - a.v, clamped: b.clamped });
  }
  return out;
}
/** First time |Δxy| stays above thr for `hold` s: a geometric marker, not a verdict. */
export function firstDivergence(res, thr = 1.0, hold = 2.0) {
  let s = null;
  for (const r of res) {
    if (r.d > thr && !r.clamped) { if (s == null) s = r.t; if (r.t - s >= hold) return { t: s, thr, hold }; } else s = null;
  }
  return null;
}

// ---------- palettes ----------
export const SPEED = ['#4F7BFF', '#3FC8E0', '#5FD38A', '#F2D45C'];
export const RESID = ['#16304d', '#c9531f', '#f28f63', '#ffd9c2'];
export const SCORE = ['#2a1f3d', '#5b3a8c', '#3f7fbf', '#3fc8b0', '#d7f27a'];
export const TIER = { fast: '#3987e5', unity: '#C792EA', real: '#f0efec' };
export const DECISION = { execute: '#7DD3DC', search: '#F2D45C', retry: '#f28f63', give_up: '#ff5a52' };
export const VARIANT = ['#3FC8E0', '#F2A65A', '#C792EA', '#5FD38A', '#FF6B8B', '#F2D45C', '#7B9CFF', '#E0E0E0', '#4DD0A8', '#D98FFF'];
const hx = (c) => { const n = parseInt(c.slice(1), 16); return [(n >> 16) & 255, (n >> 8) & 255, n & 255]; };
export function ramp(stops, f) {
  f = Math.max(0, Math.min(1, Number.isFinite(f) ? f : 0));
  const k = f * (stops.length - 1), i = Math.min(Math.floor(k), stops.length - 2), a = hx(stops[i]), b = hx(stops[i + 1]), g = k - i;
  return `rgb(${a.map((v, j) => Math.round(v + (b[j] - v) * g)).join(',')})`;
}
export function tierColor(tier, i = 0) { return TIER[tier] || VARIANT[i % VARIANT.length]; }
export function elementColor(label) {
  const l = String(label || '').toLowerCase();
  if (l.startsWith('red')) return '#ff5a52'; if (l.startsWith('green')) return '#36D399'; if (l.startsWith('yellow')) return '#F2D45C';
  if (l.startsWith('black')) return '#8a93a6'; if (l.startsWith('blue')) return '#4F7BFF'; if (l.startsWith('white')) return '#f0efec';
  if (l.startsWith('orange') || l.includes('boat')) return '#f28f63';
  return '#9aa3b2';
}
export const isStructure = (e) => String(e.role || '').startsWith('pier') || (!e.label && e.r > 0.8);

// ---------- world <-> screen (y up) ----------
export class View {
  constructor() { this.cx = 0; this.cy = 0; this.s = 20; this.w = 1; this.h = 1; }
  fit(b, w, h, pad = 40) {
    this.w = w; this.h = h;
    const bw = Math.max(1, b[2] - b[0]), bh = Math.max(1, b[3] - b[1]);
    this.s = Math.max(0.05, Math.min((w - 2 * pad) / bw, (h - 2 * pad) / bh)); this.cx = (b[0] + b[2]) / 2; this.cy = (b[1] + b[3]) / 2;
  }
  sx(x) { return this.w / 2 + (x - this.cx) * this.s; }
  sy(y) { return this.h / 2 - (y - this.cy) * this.s; }
  wx(px) { return this.cx + (px - this.w / 2) / this.s; }
  wy(py) { return this.cy - (py - this.h / 2) / this.s; }
  zoomAt(px, py, k) { const x = this.wx(px), y = this.wy(py); this.s *= k; this.cx = x - (px - this.w / 2) / this.s; this.cy = y + (py - this.h / 2) / this.s; }
}
export function bounds(tracks, els, pad = 2) {
  const b = [Infinity, Infinity, -Infinity, -Infinity];
  const add = (x, y) => { if (!Number.isFinite(x) || !Number.isFinite(y)) return; b[0] = Math.min(b[0], x); b[1] = Math.min(b[1], y); b[2] = Math.max(b[2], x); b[3] = Math.max(b[3], y); };
  for (const tr of tracks) if (tr) for (let i = 0; i < tr.t.length; i++) add(tr.x[i], tr.y[i]);
  for (const e of els || []) add(e.x, e.y);
  if (!Number.isFinite(b[0])) return [-10, -10, 10, 10];
  return [b[0] - pad, b[1] - pad, b[2] + pad, b[3] + pad];
}
export function nearElements(course, tracks, margin = 12) {
  if (!course) return [];
  const b = bounds(tracks, [], margin);
  return course.elements.filter((e) => e.x >= b[0] && e.x <= b[2] && e.y >= b[1] && e.y <= b[3]);
}

/** A catamaran hull at (x, y, yaw), L x B metres (drawn at least `minPx` long). */
export function hull(ctx, v, x, y, yaw, L, B, o = {}) {
  const k = Math.max(1, (o.minPx || 0) / (L * v.s));
  const l = (L * k * v.s) / 2, b = (B * k * v.s) / 2;
  ctx.save(); ctx.translate(v.sx(x), v.sy(y)); ctx.rotate(-yaw);
  if (o.alpha != null) ctx.globalAlpha = o.alpha;
  ctx.beginPath(); ctx.moveTo(l * 1.05, 0); ctx.quadraticCurveTo(l * 0.75, b, l * 0.15, b); ctx.lineTo(-l, b * 0.92);
  ctx.lineTo(-l, -b * 0.92); ctx.lineTo(l * 0.15, -b); ctx.quadraticCurveTo(l * 0.75, -b, l * 1.05, 0); ctx.closePath();
  if (o.glow) { ctx.shadowColor = o.glow; ctx.shadowBlur = o.blur || 14; }
  ctx.fillStyle = o.fill || '#0F1217'; ctx.fill(); ctx.shadowBlur = 0;
  ctx.lineWidth = o.lw || 1.5; ctx.strokeStyle = o.stroke || '#7DD3DC'; ctx.stroke();
  if (o.detail !== false && l > 8) {
    ctx.globalAlpha *= 0.5; ctx.beginPath(); ctx.moveTo(-l * 0.9, b * 0.5); ctx.lineTo(l * 0.7, b * 0.5); ctx.moveTo(-l * 0.9, -b * 0.5); ctx.lineTo(l * 0.7, -b * 0.5); ctx.stroke(); ctx.globalAlpha /= 0.5;
  }
  ctx.beginPath(); ctx.arc(l * 0.85, 0, Math.max(1.5, l * 0.12), 0, 7); ctx.fillStyle = o.bow || o.stroke || '#7DD3DC'; ctx.fill();
  ctx.restore();
}

export function sizeCanvas(c) {
  const r = c.getBoundingClientRect(), d = window.devicePixelRatio || 1;
  const w = Math.max(1, Math.round(r.width * d)), h = Math.max(1, Math.round(r.height * d));
  if (c.width !== w || c.height !== h) { c.width = w; c.height = h; }
  const ctx = c.getContext('2d'); ctx.setTransform(1, 0, 0, 1, 0, 0); ctx.clearRect(0, 0, c.width, c.height); ctx.setTransform(d, 0, 0, d, 0, 0);
  return [ctx, r];
}

export const fmt = {
  t: (v) => (v == null || !Number.isFinite(v) ? '–' : `${v < 0 ? '−' : ''}${Math.abs(v).toFixed(1)} s`),
  m: (v) => (v == null || !Number.isFinite(v) ? '–' : `${v.toFixed(2)} m`),
  ms: (v) => (v == null || !Number.isFinite(v) ? '–' : `${v.toFixed(2)} m/s`),
  deg: (v) => (v == null || !Number.isFinite(v) ? '–' : `${Math.round((v * 180) / Math.PI)}°`),
  pts: (v) => (v == null ? '–' : `${v > 0 ? '+' : ''}${v}`),
  n: (v, d = 2) => (v == null || !Number.isFinite(v) ? '–' : (+v).toFixed(d)),
};
export const esc = (s) => String(s ?? '').replace(/[&<>"']/g, (c) => ({ '&': '&amp;', '<': '&lt;', '>': '&gt;', '"': '&quot;', "'": '&#39;' }[c]));
export const lastBefore = (list, t) => { let o = null; for (const e of list) { if (e.t <= t) o = e; else break; } return o; };
