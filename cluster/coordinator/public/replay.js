// replay.js: the 2-D ghost theater on a canvas, domain-agnostic. A focus run (speed-ramp trail, hull with a lit
// nose, event marks), ghost runs on the same clock (faint path, hull, label) and a fleet mode (many runs, a
// comet tail each, coloured by the caller). Tracks are {t, x, y, yaw, v} arrays; the course is {elements:
// [{x, y, r, label, role}]}. The only domain table is COLORS (element colour by label prefix).
// mount(canvas, opts) -> {play, pause, toggle, seek, setSpeed, destroy, duration, t}
const RAMP = [[0, '#4F7BFF'], [0.35, '#3FC8E0'], [0.68, '#5FD38A'], [1, '#F2D45C']];
const COLORS = { red: '#FF5F57', green: '#36D399', yellow: '#FFD23F', black: '#A6B0C2', blue: '#6C9EF0', white: '#E4E8EF', banner: '#C792EA', '': '#5F6775' };
const hex = (c) => { const n = parseInt(c.slice(1), 16); return [n >> 16, (n >> 8) & 255, n & 255]; };

/** A colour on the TUI ramp (blue, cyan, green, gold) at f in [0, 1]. */
export function ramp(f) {
  f = Math.max(0, Math.min(1, f || 0));
  for (let i = 1; i < RAMP.length; i++) if (f <= RAMP[i][0]) {
    const [a, ca] = RAMP[i - 1], [b, cb] = RAMP[i], k = (f - a) / (b - a), A = hex(ca), B = hex(cb);
    return `rgb(${A.map((v, j) => Math.round(v + (B[j] - v) * k)).join(',')})`;
  }
  return RAMP[RAMP.length - 1][1];
}
const elementColor = (label) => COLORS[Object.keys(COLORS).find((k) => k && String(label).startsWith(k)) ?? ''];

/** The pose at time t (linear between samples, yaw the short way round). */
export function sample(tr, t) {
  const T = tr.t, n = T.length - 1;
  if (t <= T[0]) return { x: tr.x[0], y: tr.y[0], yaw: tr.yaw[0], v: tr.v[0], i: 0 };
  if (t >= T[n]) return { x: tr.x[n], y: tr.y[n], yaw: tr.yaw[n], v: tr.v[n], i: n, end: true };
  let lo = 0, hi = n;
  while (hi - lo > 1) { const m = (lo + hi) >> 1; T[m] <= t ? (lo = m) : (hi = m); }
  const k = (t - T[lo]) / (T[hi] - T[lo]);
  let dy = tr.yaw[hi] - tr.yaw[lo]; dy = Math.atan2(Math.sin(dy), Math.cos(dy));
  return { x: tr.x[lo] + (tr.x[hi] - tr.x[lo]) * k, y: tr.y[lo] + (tr.y[hi] - tr.y[lo]) * k, yaw: tr.yaw[lo] + dy * k, v: tr.v[lo] + (tr.v[hi] - tr.v[lo]) * k, i: lo };
}

export function mount(canvas, o) {
  const ctx = canvas.getContext('2d');
  const th = { bg: '#0B0E12', grid: '#161B22', ghost: '#C792EA', hull: '#E9F3F6', accent: '#7DD3DC', text: '#7C8595', font: '11px Geist, system-ui, sans-serif', vmax: 2, ...o.theme };
  const focus = o.track || null, ghosts = o.ghosts || [], fleet = !!o.fleet;
  const all = [focus, ...ghosts.map((g) => g.track)].filter(Boolean);
  let x0 = Infinity, y0 = Infinity, x1 = -Infinity, y1 = -Infinity;
  for (const tr of all) for (let i = 0; i < tr.x.length; i++) {
    x0 = Math.min(x0, tr.x[i]); x1 = Math.max(x1, tr.x[i]); y0 = Math.min(y0, tr.y[i]); y1 = Math.max(y1, tr.y[i]);
  }
  if (!all.length) { x0 = y0 = -10; x1 = y1 = 10; }
  const pad = 6; x0 -= pad; y0 -= pad; x1 += pad; y1 += pad;
  const elements = (o.course?.elements || []).filter((e) => e.x > x0 && e.x < x1 && e.y > y0 && e.y < y1);
  const duration = Math.max(0.001, ...all.map((tr) => tr.t[tr.t.length - 1]));
  const reduce = matchMedia('(prefers-reduced-motion: reduce)').matches;
  let t = o.t0 || 0, speed = o.speed || 8, playing = false, raf = 0, last = 0, W = 0, H = 0, S = 1, ox = 0, oy = 0, dpr = 1;
  const base = document.createElement('canvas'), bctx = base.getContext('2d');

  function fit() {
    const r = canvas.getBoundingClientRect();
    dpr = window.devicePixelRatio || 1; W = r.width; H = r.height;
    for (const c of [canvas, base]) { c.width = Math.max(1, W * dpr); c.height = Math.max(1, H * dpr); }
    ctx.setTransform(dpr, 0, 0, dpr, 0, 0); bctx.setTransform(dpr, 0, 0, dpr, 0, 0);
    S = Math.min((W - 24) / (x1 - x0), (H - 24) / (y1 - y0));
    ox = (W - (x1 - x0) * S) / 2; oy = (H - (y1 - y0) * S) / 2;
    drawBase();
  }
  const P = (x, y) => [ox + (x - x0) * S, H - (oy + (y - y0) * S)];
  const path = (c, tr, from = 0, to = tr.x.length - 1) => {
    c.beginPath();
    for (let i = from; i <= to; i++) { const p = P(tr.x[i], tr.y[i]); i > from ? c.lineTo(p[0], p[1]) : c.moveTo(p[0], p[1]); }
  };

  // The static layer: grid, course, every ghost's full path. Redrawn only on resize.
  function drawBase() {
    const c = bctx;
    c.fillStyle = th.bg; c.fillRect(0, 0, W, H);
    const step = S * 5 < 14 ? 25 : 5;
    c.strokeStyle = th.grid; c.lineWidth = 1; c.beginPath();
    for (let gx = Math.ceil(x0 / step) * step; gx < x1; gx += step) { const a = P(gx, y0), b = P(gx, y1); c.moveTo(a[0] + .5, a[1]); c.lineTo(b[0] + .5, b[1]); }
    for (let gy = Math.ceil(y0 / step) * step; gy < y1; gy += step) { const a = P(x0, gy), b = P(x1, gy); c.moveTo(a[0], a[1] + .5); c.lineTo(b[0], b[1] + .5); }
    c.stroke();
    for (const e of elements) {
      const p = P(e.x, e.y);
      c.fillStyle = elementColor(e.label); c.globalAlpha = e.role === 'obstacle' ? 0.55 : 0.9;
      c.beginPath(); c.arc(p[0], p[1], Math.max(2.2, e.r * S), 0, 7); c.fill();
    }
    c.globalAlpha = fleet ? Math.min(0.3, Math.max(0.06, 2.4 / Math.sqrt(ghosts.length || 1))) : 0.28; c.lineWidth = 1;
    for (const g of ghosts) { c.strokeStyle = g.color || th.ghost; path(c, g.track); c.stroke(); }
    c.globalAlpha = 1;
    if (focus && !fleet) {
      c.setLineDash([2, 4]); c.strokeStyle = th.text; c.globalAlpha = 0.35; path(c, focus); c.stroke(); c.setLineDash([]); c.globalAlpha = 1;
    }
    const m10 = 10 * S;
    c.fillStyle = th.text; c.font = th.font; c.fillRect(14, H - 16, m10, 1.5); c.fillText('10 m', 14, H - 22);
  }

  function hull(p, yaw, color, alpha, nose, L = Math.max(1.06 * S, 11)) {
    const B = L * 0.42;
    ctx.save(); ctx.translate(p[0], p[1]); ctx.rotate(-yaw); ctx.globalAlpha = alpha;
    ctx.beginPath(); ctx.moveTo(L * 0.55, 0); ctx.quadraticCurveTo(L * 0.1, B * 0.62, -L * 0.45, B * 0.5);
    ctx.lineTo(-L * 0.45, -B * 0.5); ctx.quadraticCurveTo(L * 0.1, -B * 0.62, L * 0.55, 0); ctx.closePath();
    ctx.fillStyle = color; ctx.fill();
    if (nose) {
      ctx.shadowColor = nose; ctx.shadowBlur = 10; ctx.fillStyle = nose;
      ctx.beginPath(); ctx.moveTo(L * 0.55, 0); ctx.lineTo(L * 0.22, B * 0.33); ctx.lineTo(L * 0.22, -B * 0.33); ctx.closePath(); ctx.fill();
    }
    ctx.restore();
  }

  function draw() {
    ctx.setTransform(1, 0, 0, 1, 0, 0); ctx.drawImage(base, 0, 0); ctx.setTransform(dpr, 0, 0, dpr, 0, 0);
    const tail = o.tail ?? 8;
    for (const g of ghosts) {
      const tr = g.track, s = sample(tr, t), col = g.color || th.ghost;
      if (fleet && !s.end) {
        let from = s.i; while (from > 0 && tr.t[from] > t - tail) from--;
        ctx.strokeStyle = col; ctx.globalAlpha = 0.9; ctx.lineWidth = 2.2; ctx.lineCap = 'round';
        path(ctx, tr, from, s.i); const p = P(s.x, s.y); ctx.lineTo(p[0], p[1]); ctx.stroke();
      }
      hull(P(s.x, s.y), s.yaw, col, s.end ? 0.22 : fleet ? 0.85 : 0.55, null, fleet ? Math.max(0.9 * S, 9) : undefined);
      if (g.label && !fleet) { const p = P(s.x, s.y); ctx.fillStyle = col; ctx.globalAlpha = 0.85; ctx.font = th.font; ctx.fillText(g.label, p[0] + 9, p[1] - 8); }
      ctx.globalAlpha = 1;
    }
    let s = null;
    if (focus) {
      const tr = focus; s = sample(tr, t);
      ctx.lineWidth = 2.2; ctx.lineCap = 'round';
      for (let i = 1; i <= s.i; i++) {
        const a = P(tr.x[i - 1], tr.y[i - 1]), b = P(tr.x[i], tr.y[i]);
        ctx.strokeStyle = ramp(tr.v[i] / th.vmax); ctx.beginPath(); ctx.moveTo(a[0], a[1]); ctx.lineTo(b[0], b[1]); ctx.stroke();
      }
      const p = P(s.x, s.y), a = P(tr.x[s.i], tr.y[s.i]);
      ctx.strokeStyle = ramp(s.v / th.vmax); ctx.beginPath(); ctx.moveTo(a[0], a[1]); ctx.lineTo(p[0], p[1]); ctx.stroke();
      for (const e of o.marks || []) if (e.t <= t) {
        const q = sample(tr, e.t), m = P(q.x, q.y);
        ctx.strokeStyle = e.bad ? '#F0898C' : th.accent; ctx.globalAlpha = 0.8; ctx.lineWidth = 1.2;
        ctx.beginPath(); ctx.arc(m[0], m[1], 5, 0, 7); ctx.stroke(); ctx.globalAlpha = 1;
      }
      ctx.strokeStyle = th.accent; ctx.globalAlpha = 0.18; ctx.lineWidth = 6;
      const q = P(tr.x[Math.max(0, s.i - 3)], tr.y[Math.max(0, s.i - 3)]);
      ctx.beginPath(); ctx.moveTo(p[0], p[1]); ctx.lineTo(q[0], q[1]); ctx.stroke(); ctx.globalAlpha = 1;
      hull(p, s.yaw, th.hull, 1, th.accent);
    }
    o.onTime && o.onTime(t, s);
  }
  function loop(ts) {
    if (!playing) return;
    const dt = last ? (ts - last) / 1000 : 0; last = ts; t += dt * speed;
    if (t >= duration) { t = duration; playing = false; o.onState && o.onState(false); }
    draw(); if (playing) raf = requestAnimationFrame(loop);
  }
  const api = {
    duration,
    get t() { return t; },
    play() { if (t >= duration) t = 0; playing = true; last = 0; o.onState && o.onState(true); raf = requestAnimationFrame(loop); },
    pause() { playing = false; cancelAnimationFrame(raf); o.onState && o.onState(false); },
    toggle() { playing ? api.pause() : api.play(); },
    seek(nt) { t = Math.max(0, Math.min(duration, nt)); draw(); },
    setSpeed(k) { speed = k; },
    destroy() { playing = false; cancelAnimationFrame(raf); ro.disconnect(); },
  };
  const ro = new ResizeObserver(() => { fit(); draw(); });
  ro.observe(canvas);
  fit(); draw();
  if (o.autoplay && !reduce) api.play();
  return api;
}
