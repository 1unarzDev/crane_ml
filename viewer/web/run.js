// run.js: the chart room. One run in depth: the course, the speed-ramped track, the hull and its goal, a
// second tier as a dashed ghost with residual whiskers between matched points, the judge's pins and spans,
// and an eight-lane timeline. Keys: space play, j/l ±5 s, ,/. ±0.1 s, [ ] event stepping, f fit, 1-9 T R P layers.
import { sample, makeAlign, residuals, firstDivergence, ramp, SPEED, RESID, TIER, DECISION, elementColor, isStructure,
  View, bounds, nearElements, hull, sizeCanvas, fmt, esc, lastBefore } from './core.js';
import { judgeFor, judgePair } from './data.js';

const LAYERS = [
  ['Water', [['water', 'Water field', null, '1'], ['grid', '5 m survey grid', null, '2']]],
  ['Course', [['course', 'Buoys and gates', '#ff5a52', '3'], ['footprints', 'Task footprints', '#22324a', '4'], ['labels', 'Element labels', null, '5']]],
  ['Vehicle', [['track', 'Track, speed ramp', `linear-gradient(90deg,${SPEED.join(',')})`, '6'], ['hull', 'Hull', '#7DD3DC', '7'], ['goal', 'Current goal', '#F2D45C', '8'], ['future', 'Path ahead', '#9ec5f4', '9']]],
  ['Tiers', [['ghost', 'Other tier (ghost)', '#C792EA', 'T'], ['whiskers', 'Residual whiskers |Δxy|', '#f28f63', 'R']]],
  ['Judge', [['pins', 'Contacts, near misses, divergence', '#ff5a52', 'P'], ['spans', 'Judge insight spans', '#EDCB78', 'J']]],
];
const TL_LANES = [['task', 'Task', 18], ['decision', 'Decisions', 18], ['behaviour', 'Behaviour', 18], ['judge', 'Judge', 22], ['speed', 'Speed m/s', 38], ['resid', '|Δxy| m', 34], ['channels', 'Perception', 12], ['real', 'Real tier', 12]];
const LEFT = 96;

export async function mount(host, ctx) {
  const { src, route } = ctx;
  const id = route.args[0];
  const A = await src.run(id);
  let vsId = route.q.vs ?? (await src.twin(A));
  if (vsId === id || vsId === 'none') vsId = null;
  const [B, course, reports] = await Promise.all([vsId ? src.run(vsId).catch(() => null) : null, src.course(A.course), src.judge([id, vsId])]);
  ctx.study = A.study;
  ctx.pair = B ? [A.id, B.id] : null;
  const tierA = A.tier || A.arm || 'run', tierB = B ? (B.tier && B.tier !== A.tier ? B.tier : B.arm || 'other') : null;
  const colA = TIER[A.tier] || '#7DD3DC', colB = B ? (TIER[B.tier] && B.tier !== A.tier ? TIER[B.tier] : '#C792EA') : null;
  const jA = judgeFor(A, reports, B?.id), jB = B ? judgeFor(B, reports, A.id) : [];
  const jp = B ? judgePair(A.id, B.id, reports) : null;
  const L = course?.vessel?.length || 1.06, BM = course?.vessel?.beam || 0.9;
  const vmax = Math.max(0.5, quant(A.track.v, 0.98) * 1.05);
  const S = { t: route.q.t != null ? +route.q.t : Infinity, playing: false, speed: 4, align: route.q.align || 'event', sel: null, view: new View(), show: { a: true, b: true },
    layers: { water: true, grid: true, course: true, footprints: true, labels: true, track: true, hull: true, goal: true, future: true, ghost: true, whiskers: true, pins: true, spans: true } };
  let al, res, div, els, tmin, tmax, ripple = 0, raf = 0, alive = true;

  host.innerHTML = `<div class="room">
    <aside class="layers" id="layers"></aside>
    <section class="mapwrap" id="mapwrap"><canvas id="map" aria-label="Map of the run"></canvas><div class="banner" id="banner" hidden></div>
      <div class="hud tl" id="hudtl"></div><div class="hud br" id="hudbr"></div><div class="scale" id="scale"></div><div class="tip" id="tip"></div></section>
    <aside class="insp" id="insp"></aside>
    <footer class="tl"><div class="ctrl"><button class="play" id="play" aria-label="Play">▶</button><div class="seg" id="speed"></div>
      <div class="clock" id="clock"></div><div class="seg" id="align" role="group" aria-label="Alignment"></div><div class="note al" id="alnote"></div></div>
      <canvas id="tl" aria-label="Timeline"></canvas></footer></div>`;
  const $ = (s) => host.querySelector(`#${s}`);
  const mc = $('map'), tc = $('tl');

  function load() {
    al = B ? makeAlign(A, B, S.align) : null;
    res = B ? residuals(A, B, al) : [];
    div = firstDivergence(res);
    els = nearElements(course, [A.track, B && B.track]);
    tmin = Math.min(0, A.track.t[0] ?? 0); tmax = A.track.t.at(-1) ?? 1;
    S.t = Math.max(tmin, Math.min(S.t, tmax));
    chips(); renderLayers();
  }
  function fit() { const r = mc.getBoundingClientRect(); S.view.fit(bounds([A.track, B && B.track], els, 3), r.width, r.height, 40); }

  function chips() {
    const sc = (r, c, n) => `<span class="chip ${r.score > 0 ? 'o' : r.score < 0 || r.outcome === 'failed' ? 'f' : ''}" style="border-color:${c}66">${esc(n)} ${r.score ?? '–'} pts · ${esc(r.outcome)}</span>`;
    const tiers = `<button class="chip t" data-k="a" aria-pressed="${S.show.a}"><i style="background:${colA}"></i>${esc(tierA)}</button>` +
      (B ? `<button class="chip t" data-k="b" aria-pressed="${S.show.b}"><i style="background:${colB}"></i>${esc(tierB)}</button>` : '') +
      `<span class="chip absent" title="No real-boat source adapter yet: the real tier is absent"><i></i>real · absent</span>`;
    ctx.chips(tiers + sc(A, colA, tierA) + (B ? sc(B, colB, tierB) : '') + `<span class="chip">${A.seed != null ? `seed ${A.seed} · ` : ''}${esc((A.task || A.study || '').replace('_', ' '))}</span>` +
      (A.flags?.length ? `<span class="chip w">${esc(A.flags.join(' '))}</span>` : ''));
    document.querySelectorAll('#chips .chip.t').forEach((b) => (b.onclick = () => { S.show[b.dataset.k] = !S.show[b.dataset.k]; chips(); draw(); }));
    const msgs = [];
    if (B?.channels?.perception === 'empty' && S.show.b) msgs.push(`<b>${esc(tierB)} perception empty</b>: the object map was empty, so its decisions ran blind`);
    if (B?.channels?.validity === 'absent' && S.show.b) msgs.push(`<b>${esc(tierB)} validity not reported</b> by this player`);
    if (al && al.confidence === 'low') msgs.push(`<b>Alignment low</b>: ${esc(al.note)}`);
    if (A.lod) msgs.push('<b>Overview LOD</b> (2 Hz): fine detail is interpolated');
    const ban = $('banner'); ban.hidden = !msgs.length; ban.innerHTML = msgs.join('<br>');
  }

  // ---------- layers and channels ----------
  const CH_NOTE = { pose: 'planar pose', decisions: 'task decisions', goals: 'goals in the world frame', referee: 'referee report', perception: 'object map', validity: 'run validity', plan: 'nav plan', nlc_tube: 'NLC tube' };
  function chanHTML(run, name, onlyGaps) {
    const rows = Object.entries(run.channels).filter(([, s]) => !onlyGaps || s !== 'present');
    if (!rows.length) return '';
    return `<h3 class="lbl">Channels · ${esc(name)}</h3>` + rows.map(([k, s]) => `<div class="chan"><span class="st ${esc(s)}" title="${esc(s)}"></span><span class="n">${esc(k)} <em>${esc(s)}</em></span><span></span><span class="note">${esc(CH_NOTE[k] || '')}</span></div>`).join('');
  }
  function renderLayers() {
    let h = '';
    for (const [g, ls] of LAYERS) {
      h += `<h3 class="lbl">${g}</h3>`;
      for (const [k, n, c, kb] of ls) h += `<label class="lyr"><input type="checkbox" data-l="${k}" ${S.layers[k] ? 'checked' : ''}>${c ? `<span class="sw" style="background:${c}"></span>` : ''}${n}<kbd>${kb}</kbd></label>`;
    }
    h += chanHTML(A, tierA, false) + (B ? chanHTML(B, tierB, true) : '');
    h += `<h3 class="lbl">Channels · real</h3><div class="chan"><span class="st absent"></span><span class="n">all <em>absent</em></span><span></span><span class="note">no real-boat source adapter yet</span></div>`;
    $('layers').innerHTML = h;
  }

  // ---------- map ----------
  function drawWater(c, v) {
    const g = c.createRadialGradient(v.w * 0.45, v.h * 0.4, 20, v.w * 0.5, v.h * 0.5, Math.max(v.w, v.h) * 0.75);
    g.addColorStop(0, '#10243c'); g.addColorStop(0.55, '#0a1729'); g.addColorStop(1, '#04070d');
    c.fillStyle = g; c.fillRect(0, 0, v.w, v.h);
    if (!S.layers.water) return;
    c.save(); c.globalAlpha = 0.06; c.strokeStyle = '#7fc4ff'; c.lineWidth = 1;
    for (let k = 0; k < 26; k++) {
      const y0 = (k * 37.3 + ripple * 6) % v.h; c.beginPath();
      for (let x = 0; x <= v.w; x += 24) { const y = y0 + Math.sin(x * 0.012 + k + ripple * 0.4) * 6; x ? c.lineTo(x, y) : c.moveTo(x, y); }
      c.stroke();
    }
    c.restore();
  }
  function drawGrid(c, v) {
    if (!S.layers.grid) return;
    const step = v.s * 5 < 28 ? 25 : 5;
    c.strokeStyle = '#22324a'; c.lineWidth = 0.6; c.globalAlpha = 0.55;
    const x0 = Math.floor(v.wx(0) / step) * step, x1 = v.wx(v.w), y0 = Math.floor(v.wy(v.h) / step) * step, y1 = v.wy(0);
    c.beginPath();
    for (let x = x0; x <= x1; x += step) { c.moveTo(v.sx(x), 0); c.lineTo(v.sx(x), v.h); }
    for (let y = y0; y <= y1; y += step) { c.moveTo(0, v.sy(y)); c.lineTo(v.w, v.sy(y)); }
    c.stroke(); c.globalAlpha = 1; c.fillStyle = '#3a4e6b'; c.font = '9px Chivo Mono, monospace';
    for (let x = x0; x <= x1; x += step * 2) c.fillText(`${x} m`, v.sx(x) + 3, v.h - 4);
  }
  function drawCourse(c, v) {
    if (!course) return;
    if (S.layers.footprints) for (const t of course.tasks || []) {
      if (!t.footprint?.length) continue;
      c.beginPath(); t.footprint.forEach((p, i) => (i ? c.lineTo(v.sx(p[0]), v.sy(p[1])) : c.moveTo(v.sx(p[0]), v.sy(p[1])))); c.closePath();
      c.fillStyle = 'rgba(63,200,224,0.035)'; c.fill(); c.setLineDash([3, 4]); c.strokeStyle = '#1f3a55'; c.stroke(); c.setLineDash([]);
    }
    if (!S.layers.course) return;
    const byId = Object.fromEntries(course.elements.filter((e) => e.id).map((e) => [e.id, e]));
    for (const g of course.gates || []) {
      const p = byId[g.port], s = byId[g.starboard]; if (!p || !s) continue;
      c.beginPath(); c.moveTo(v.sx(p.x), v.sy(p.y)); c.lineTo(v.sx(s.x), v.sy(s.y)); c.strokeStyle = 'rgba(240,239,236,0.18)'; c.setLineDash([2, 3]); c.stroke(); c.setLineDash([]);
      if (S.layers.labels && v.s > 6) { c.fillStyle = '#6f87a6'; c.font = '10px Chivo, sans-serif'; c.fillText(`${String(g.id).split('.').pop()} ${String(g.task || '').replace('_', ' ')}`, v.sx(g.x) + 8, v.sy(g.y) - 4); }
    }
    for (const e of els.filter(isStructure)) { c.beginPath(); c.arc(v.sx(e.x), v.sy(e.y), e.r * v.s, 0, 7); c.fillStyle = 'rgba(58,70,88,0.28)'; c.fill(); c.strokeStyle = 'rgba(120,135,160,0.35)'; c.setLineDash([2, 3]); c.stroke(); c.setLineDash([]); }
    for (const e of els.filter((e) => !isStructure(e))) {
      const col = elementColor(e.label), r = Math.max(3, e.r * v.s);
      c.save(); c.shadowColor = col; c.shadowBlur = 10; c.beginPath(); c.arc(v.sx(e.x), v.sy(e.y), r, 0, 7); c.fillStyle = col; c.globalAlpha = 0.9; c.fill(); c.restore();
      c.beginPath(); c.arc(v.sx(e.x), v.sy(e.y), r + 2.5, 0, 7); c.strokeStyle = 'rgba(255,255,255,0.15)'; c.lineWidth = 1; c.stroke();
      if (S.sel?.kind === 'element' && S.sel.el === e) { c.beginPath(); c.arc(v.sx(e.x), v.sy(e.y), r + 7, 0, 7); c.strokeStyle = '#7DD3DC'; c.lineWidth = 1.5; c.stroke(); c.lineWidth = 1; }
    }
    const st = course.start; if (st && v.s > 2) { c.save(); c.translate(v.sx(st[0]), v.sy(st[1])); c.fillStyle = '#F2D45C'; c.fillRect(0, -10, 1.5, 10); c.beginPath(); c.moveTo(1.5, -10); c.lineTo(9, -7); c.lineTo(1.5, -4); c.fill(); c.restore(); }
  }
  function polyline(c, v, tr, from, to) { c.beginPath(); let s = false; for (let i = from; i <= to; i++) { const x = v.sx(tr.x[i]), y = v.sy(tr.y[i]); s ? c.lineTo(x, y) : c.moveTo(x, y); s = true; } }
  function idxAt(tr, t) { let lo = 0, hi = tr.t.length - 1; if (t < tr.t[0]) return -1; while (hi - lo > 1) { const m = (lo + hi) >> 1; tr.t[m] <= t ? (lo = m) : (hi = m); } return tr.t[hi] <= t ? hi : lo; }
  function speedTrack(c, v, tr, upto) {
    const k = idxAt(tr, upto), n = tr.t.length - 1;
    c.lineCap = 'round';
    if (k > 0) {
      c.save(); c.globalAlpha = 0.35; c.lineWidth = 6; c.strokeStyle = '#3FC8E0'; c.filter = 'blur(3px)'; polyline(c, v, tr, 0, k); c.stroke(); c.restore();
      c.lineWidth = 2.2;
      for (let i = 1; i <= k; i++) { c.beginPath(); c.moveTo(v.sx(tr.x[i - 1]), v.sy(tr.y[i - 1])); c.lineTo(v.sx(tr.x[i]), v.sy(tr.y[i])); c.strokeStyle = ramp(SPEED, tr.v[i] / vmax); c.stroke(); }
    }
    if (S.layers.future && k < n) { c.globalAlpha = 0.18; c.lineWidth = 1.2; c.strokeStyle = '#9ec5f4'; polyline(c, v, tr, Math.max(0, k), n); c.stroke(); c.globalAlpha = 1; }
  }
  function ghostTrack(c, v, tr, upto, col) {
    const k = idxAt(tr, upto), n = tr.t.length - 1;
    c.save(); c.setLineDash([5, 4]); c.lineWidth = 1.6; c.strokeStyle = col; c.globalAlpha = 0.85;
    if (k > 0) { polyline(c, v, tr, 0, k); c.stroke(); }
    if (S.layers.future && k < n) { c.globalAlpha = 0.15; polyline(c, v, tr, Math.max(0, k), n); c.stroke(); }
    c.restore();
  }
  const toA = (t) => (al ? al.inv(t) : t);
  function pins() {
    const out = [];
    for (const [run, which, col] of [[A, 'a', colA], [B, 'b', colB]]) {
      if (!run || !S.show[which]) continue;
      for (const e of run.events) {
        if (e.kind !== 'contact' && e.kind !== 'near_miss' && e.kind !== 'strike' && e.kind !== 'failure') continue;
        const p = Number.isFinite(e.x) && Number.isFinite(e.y) ? e : sample(run.track, e.t);
        if (!p) continue;
        out.push({ kind: e.kind, which, col, t: which === 'a' ? e.t : toA(e.t), x: p.x, y: p.y, title: `${e.kind === 'near_miss' ? 'Near miss' : e.kind === 'contact' ? 'Contact' : e.kind} · ${String(e.id || '').split('.').slice(-2).join('.')}`,
          body: `${which === 'a' ? tierA : tierB} · clearance ${fmt.m(e.clearance_m)}${e.speed_mps != null ? ` at ${fmt.ms(e.speed_mps)}` : ''}${e.geometric ? ' · geometric (fast tier never stops)' : ''}`, ev: e });
      }
    }
    if (div && B) {
      const r = res.find((q) => q.t >= div.t);
      if (r) out.push({ kind: 'div', which: 'pair', t: div.t, x: (r.ax + r.bx) / 2, y: (r.ay + r.by) / 2, title: 'First divergence',
        body: `|Δxy| > ${div.thr} m held ${div.hold} s from ${fmt.t(div.t)} (${al.mode} alignment). Geometry, not a verdict.`, ev: { t: div.t, d: r.d, align: al.mode } });
    }
    return out;
  }
  function spans() {
    const out = jA.map((j) => ({ ...j, which: 'a' }));
    for (const j of jB) out.push({ ...j, which: 'b', t0: toA(j.t0), t1: toA(j.t1) });
    return out.sort((a, b) => a.t0 - b.t0);
  }

  function draw() {
    if (!alive) return;
    const [c, r] = sizeCanvas(mc), v = S.view; v.w = r.width; v.h = r.height;
    drawWater(c, v); drawGrid(c, v); drawCourse(c, v);
    const tA = S.t, tB = al ? al.map(S.t) : S.t;
    if (S.layers.whiskers && B && S.show.b && res.length) {
      c.lineWidth = 1;
      for (const q of res) { if (q.t > tA) break; c.strokeStyle = ramp(RESID, q.d / 3); c.globalAlpha = q.clamped ? 0.15 : 0.75; c.beginPath(); c.moveTo(v.sx(q.ax), v.sy(q.ay)); c.lineTo(v.sx(q.bx), v.sy(q.by)); c.stroke(); }
      c.globalAlpha = 1;
    }
    // judge spans on the map: the track segment inside each insight window, highlighted
    if (S.layers.spans) for (const j of spans()) {
      const run = j.which === 'a' ? A : B, t0 = j.which === 'a' ? j.t0 : al ? al.map(j.t0) : j.t0, t1 = j.which === 'a' ? j.t1 : al ? al.map(j.t1) : j.t1;
      if (!run || !S.show[j.which]) continue;
      const i0 = Math.max(0, idxAt(run.track, t0)), i1 = idxAt(run.track, t1); if (i1 <= i0) continue;
      c.save(); c.strokeStyle = j.kind.startsWith('divergence') ? '#F2D45C' : '#EDCB78'; c.globalAlpha = 0.22; c.lineWidth = 10; c.lineCap = 'round'; polyline(c, v, run.track, i0, i1); c.stroke(); c.restore();
    }
    if (S.layers.ghost && B && S.show.b) ghostTrack(c, v, B.track, tB, colB);
    if (S.layers.track && S.show.a) speedTrack(c, v, A.track, tA);
    const pa = sample(A.track, tA), pb = B ? sample(B.track, tB) : null;
    if (S.layers.goal) for (const [run, which, tt, p] of [[A, 'a', tA, pa], [B, 'b', tB, pb]]) {
      if (!run || !S.show[which] || !p) continue;
      const g = lastBefore(run.events.filter((e) => e.lane === 'goal' && Number.isFinite(e.x)), tt); if (!g) continue;
      c.save(); c.setLineDash([1, 4]); c.strokeStyle = which === 'a' ? '#F2D45C' : colB; c.globalAlpha = 0.85; c.beginPath(); c.moveTo(v.sx(p.x), v.sy(p.y)); c.lineTo(v.sx(g.x), v.sy(g.y)); c.stroke(); c.setLineDash([]);
      c.translate(v.sx(g.x), v.sy(g.y)); c.rotate(Math.PI / 4); c.strokeRect(-5, -5, 10, 10); c.restore();
    }
    if (S.layers.pins) for (const p of pins()) {
      const x = v.sx(p.x), y = v.sy(p.y), past = p.t <= tA;
      c.save(); c.globalAlpha = past ? 1 : 0.35; const col = p.kind === 'div' ? '#F2D45C' : p.kind === 'near_miss' ? '#EDCB78' : '#ff5a52';
      c.strokeStyle = col; c.fillStyle = col; c.lineWidth = 1.5; c.beginPath(); c.moveTo(x, y); c.lineTo(x, y - 18); c.stroke();
      c.beginPath(); if (p.kind === 'div') { c.moveTo(x, y - 18); c.lineTo(x + 11, y - 14); c.lineTo(x, y - 10); } else c.arc(x, y - 21, 4, 0, 7); c.fill();
      if (p.which === 'b') { c.strokeStyle = colB; c.beginPath(); c.arc(x, y - 21, 6.5, 0, 7); c.stroke(); }
      c.restore();
    }
    if (S.layers.hull) {
      if (pb && S.show.b) hull(c, v, pb.x, pb.y, pb.yaw, L, BM, { stroke: colB, fill: 'rgba(30,16,44,0.7)', glow: colB, blur: 8, minPx: 12, alpha: pb.clamped ? 0.4 : 1 });
      if (pa && S.show.a) hull(c, v, pa.x, pa.y, pa.yaw, L, BM, { stroke: '#7DD3DC', fill: '#0F1217', glow: '#3FC8E0', blur: 16, minPx: 14 });
    }
    const d = pa && pb ? Math.hypot(pa.x - pb.x, pa.y - pb.y) : null;
    $('hudtl').innerHTML = pa ? `${esc(tierA)} x ${pa.x.toFixed(2)} y ${pa.y.toFixed(2)} ψ ${fmt.deg(pa.yaw)} · ${fmt.ms(pa.v)}` + (pb ? `<br><span style="color:${colB}">${esc(tierB)} at ${fmt.t(tB)}${pb.clamped ? ' (ended)' : ''}</span> · |Δxy| ${fmt.m(d)}` : '') : '';
    $('hudbr').textContent = `${v.s.toFixed(1)} px/m · drag to pan, wheel to zoom, F to fit`;
    const m = v.s > 15 ? 2 : v.s > 5 ? 5 : v.s > 2 ? 10 : 25; $('scale').innerHTML = `<i style="width:${m * v.s}px"></i>${m} m`;
    drawTimeline(); renderInsp(pa, pb, tB);
  }

  // ---------- inspector ----------
  function rowsOf(run) { return run ? run.events.filter((e) => e.kind === 'row') : []; }
  function renderInsp(pa, pb, tB) {
    const dA = lastBefore(A.events.filter((e) => e.lane === 'decision'), S.t), dB = B ? lastBefore(B.events.filter((e) => e.lane === 'decision'), tB) : null;
    const bA = lastBefore(A.events.filter((e) => e.lane === 'behaviour'), S.t), bB = B ? lastBefore(B.events.filter((e) => e.lane === 'behaviour'), tB) : null;
    let h = `<div class="sec"><h4><span>Playhead</span><span>${esc(tierA)} clock</span></h4><div class="big">${fmt.t(S.t)}</div>
      <div class="kv" style="margin-top:6px"><span></span><span class="h" style="color:${colA}">${esc(tierA)}</span><span class="h" style="color:${colB || ''}">${B ? esc(tierB) : ''}</span>
      <span class="k">t</span><span>${fmt.t(S.t)}</span><span>${B ? fmt.t(tB) : ''}</span>
      <span class="k">x, y</span><span>${pa ? `${pa.x.toFixed(2)}, ${pa.y.toFixed(2)}` : '–'}</span><span>${pb ? `${pb.x.toFixed(2)}, ${pb.y.toFixed(2)}` : ''}</span>
      <span class="k">heading</span><span>${fmt.deg(pa?.yaw)}</span><span>${pb ? fmt.deg(pb.yaw) : ''}</span>
      <span class="k">speed</span><span>${fmt.ms(pa?.v)}</span><span>${pb ? fmt.ms(pb.v) : ''}</span>
      <span class="k">behaviour</span><span style="font-family:var(--sans)">${esc(bA?.text || '–')}</span><span style="font-family:var(--sans)">${B ? esc(bB?.text || '–') : ''}</span></div></div>`;
    h += `<div class="sec"><h4><span>Decision in force</span></h4>` +
      (dA ? `<div class="quote" style="border-color:${DECISION[dA.decision] || '#7DD3DC'}">${esc(dA.text)}<small>${esc(tierA)} · ${esc(dA.decision || 'decision')} at ${fmt.t(dA.t)}</small></div>` : '<div class="quote">none yet</div>') +
      (dB ? `<div class="quote" style="border-color:${DECISION[dB.decision] || colB}">${esc(dB.text)}<small>${esc(tierB)} · ${esc(dB.decision || 'decision')} at ${fmt.t(dB.t)} (own clock)</small></div>` : '') + '</div>';
    if (S.sel) h += `<div class="sec"><h4><span>Selected</span><button class="copy" id="copy">copy as JSON</button></h4><div class="ev"><div class="t">${S.sel.t != null ? fmt.t(S.sel.t) : ''}</div><b>${esc(S.sel.title)}</b><div style="color:var(--tx2);margin-top:3px">${esc(S.sel.body)}</div><pre>${esc(JSON.stringify(S.sel.ev, null, 1))}</pre></div></div>`;
    const js = spans();
    if (js.length || jp) {
      h += `<div class="sec"><h4><span>Judge</span><span>${jp ? esc(jp.report.id) : ''}</span></h4>`;
      if (jp) {
        const p = jp.pair, dv = p.divergence || {};
        h += `<div class="kv" style="grid-template-columns:auto 1fr"><span class="k">outcome</span><span>${esc((p.outcome || []).join(' → '))}</span>
          <span class="k">path rms</span><span>${fmt.m(dv.path_rms)} (max ${fmt.m(dv.path_max)})</span>
          ${p.code_mismatch?.length ? `<span class="k">code</span><span style="color:var(--warn)">tiers ran different code</span>` : ''}</div>`;
      }
      const seen = new Set();
      for (const j of js) {
        const key = `${j.id}:${j.which}`; if (seen.has(key)) continue; seen.add(key);
        h += `<div class="ins ${esc(j.severity || '')}" data-t="${j.t0}"><b>${esc(j.label)}</b> ${j.claim ? esc(j.claim) : ''}<small>${j.which === 'a' ? esc(tierA) : esc(tierB)} · ${fmt.t(j.t0)} – ${fmt.t(j.t1)} · ${esc((j.channels || []).join(', '))}</small></div>`;
      }
      h += '</div>';
    }
    const rows = rowsOf(A), rowsB = Object.fromEntries(rowsOf(B).map((r) => [r.id, r]));
    if (rows.length || A.referee) {
      h += `<div class="sec"><h4><span>Referee rows</span><span>${esc(tierA)}${B ? ` / ${esc(tierB)}` : ''}</span></h4><table class="rows">` +
        rows.slice(0, 14).map((r) => { const o = rowsB[r.id]; return `<tr data-t="${r.t}"><td style="font-family:var(--mono);color:var(--tx3)">${esc(r.id)}</td><td>${esc(r.text)}</td><td class="p s-${esc(r.status)}">${r.points}/${r.possible}</td><td class="p s-${esc(o?.status || '')}">${o ? o.points : ''}</td></tr>`; }).join('') +
        `</table>${A.referee?.summary ? `<div class="note" style="margin-top:6px">${esc(A.referee.summary)}</div>` : ''}</div>`;
    }
    const flagsOf = (run) => run?.referee ? (run.referee.clean ? '<span class="chip o">clean</span>' : (run.referee.failed || []).map((f) => `<span class="chip f">${esc(f)}</span>`).join(' ')) : '';
    if (A.referee || B?.referee) h += `<div class="sec"><h4><span>Reliability flags</span></h4>${[[A, tierA, colA], [B, tierB, colB]].filter((x) => x[0]?.referee).map(([r, n, c]) => `<div style="margin:3px 0"><span style="color:${c}">${esc(n)}</span> ${flagsOf(r)}<div class="note" style="font-family:var(--sans)">${esc((r.referee.causes || []).join('; '))}</div></div>`).join('')}</div>`;
    if (A.fidelity) h += `<div class="sec"><h4><span>Fidelity</span></h4><div class="note" style="font-family:var(--sans);font-size:11.5px">${esc(A.fidelity)}</div></div>`;
    h += `<div class="sec"><h4><span>Run</span></h4><div class="note">${esc(A.id)}${B ? `<br>vs ${esc(B.id)}` : ''}</div>${B ? `<p style="margin:8px 0 0"><a href="${ctx.href('tiers', [A.id, B.id], { t: S.t.toFixed(1), align: S.align })}">Open the tier bench →</a></p>` : ''}${A.study ? `<p style="margin:4px 0 0"><a href="${ctx.href('batch', [A.study], { seed: A.seed != null ? `${A.seed}${A.task ? `|${A.task}` : ''}` : '', focus: A.arm || '' })}">This study as a batch →</a></p>` : ''}</div>`;
    $('insp').innerHTML = h;
    const cp = $('copy'); if (cp) cp.onclick = () => navigator.clipboard?.writeText(JSON.stringify(S.sel.ev));
  }

  // ---------- timeline ----------
  let hatchPat = null;
  function hatch(c) { if (hatchPat) return hatchPat; const h = document.createElement('canvas'); h.width = h.height = 6; const x = h.getContext('2d'); x.strokeStyle = '#3a4658'; x.beginPath(); x.moveTo(0, 6); x.lineTo(6, 0); x.stroke(); return (hatchPat = c.createPattern(h, 'repeat')); }
  function drawTimeline() {
    const [c, r] = sizeCanvas(tc), w = r.width, X = (t) => LEFT + ((t - tmin) / (tmax - tmin || 1)) * (w - LEFT - 14);
    c.clearRect(0, 0, w, r.height);
    c.fillStyle = '#16233a'; c.font = '9.5px Chivo Mono, monospace';
    const step = tmax > 300 ? 60 : tmax > 150 ? 20 : tmax > 60 ? 10 : 5;
    for (let t = Math.ceil(tmin / step) * step; t <= tmax; t += step) { c.fillStyle = '#132036'; c.fillRect(X(t), 0, 1, r.height); c.fillStyle = '#3a4e6b'; c.fillText(`${t}s`, X(t) + 2, r.height - 3); }
    let y = 6;
    for (const [k, name, hgt] of TL_LANES) {
      c.fillStyle = '#8d8c85'; c.font = '10.5px Chivo, sans-serif'; c.fillText(name, 10, y + hgt / 2 + 4);
      c.fillStyle = 'rgba(255,255,255,0.025)'; c.fillRect(LEFT, y, w - LEFT - 14, hgt);
      if (k === 'task' || k === 'decision') for (const [run, which] of [[A, 'a'], [B, 'b']]) {
        if (!run || !S.show[which]) continue;
        for (const e of run.events) {
          if (e.lane !== k) continue;
          const tt = which === 'a' ? e.t : toA(e.t); if (tt < tmin || tt > tmax) continue;
          const x = X(tt), cy = y + hgt / 2 + (which === 'a' ? -3 : 3);
          c.fillStyle = k === 'decision' ? DECISION[e.decision] || '#7DD3DC' : e.kind === 'task_finished' ? (/FAIL|ABORT|GIVE/i.test(e.text) ? '#ff5a52' : '#36D399') : e.kind === 'gate_cleared' ? '#36D399' : '#c3c2b7';
          c.beginPath(); if (which === 'a') { c.moveTo(x, cy - 5); c.lineTo(x + 4, cy); c.lineTo(x, cy + 5); c.lineTo(x - 4, cy); } else c.arc(x, cy, 3, 0, 7); c.fill();
          if (which === 'b') { c.strokeStyle = colB; c.lineWidth = 1; c.stroke(); }
        }
      }
      if (k === 'behaviour') {
        const pal = ['#13324a', '#1b4561', '#21576f', '#2a6a7d', '#244e6b'];
        for (const [run, which, off] of [[A, 'a', 0], [B, 'b', hgt / 2]]) {
          if (!run || !S.show[which]) continue;
          const bs = run.events.filter((e) => e.lane === 'behaviour');
          bs.forEach((b, i) => {
            const t0 = which === 'a' ? b.t : toA(b.t), n = bs[i + 1], t1 = n ? (which === 'a' ? n.t : toA(n.t)) : (which === 'a' ? tmax : toA(run.track.t.at(-1)));
            const x0 = X(Math.max(tmin, t0)), x1 = X(Math.min(tmax, t1)); if (x1 <= x0) return;
            c.fillStyle = which === 'a' ? pal[i % pal.length] : `rgba(199,146,234,${0.15 + 0.1 * (i % 3)})`; c.fillRect(x0, y + off, Math.max(1, x1 - x0 - 1), hgt / 2 - 1);
            if (x1 - x0 > 70 && which === 'a') { c.fillStyle = '#c3c2b7'; c.font = '9.5px Chivo, sans-serif'; c.fillText(String(b.text || '').slice(0, Math.floor((x1 - x0) / 6)), x0 + 3, y + off + 7.5); }
          });
        }
      }
      if (k === 'judge') {
        for (const j of S.layers.spans ? spans() : []) { const x0 = X(Math.max(tmin, j.t0)), x1 = X(Math.min(tmax, j.t1)); c.fillStyle = j.which === 'a' ? 'rgba(237,203,120,0.25)' : 'rgba(199,146,234,0.25)'; c.fillRect(x0, y + 1, Math.max(2, x1 - x0), hgt - 2); }
        for (const p of pins()) {
          const x = X(p.t); c.fillStyle = p.kind === 'div' ? '#F2D45C' : p.kind === 'near_miss' ? '#EDCB78' : '#ff5a52';
          if (p.kind === 'div') { c.fillRect(x, y + 2, Math.max(2, X(tmax) - x), 3); c.font = '9.5px Chivo, sans-serif'; c.fillText('▶ first divergence', x + 3, y + 15); } else { c.beginPath(); c.arc(x, y + hgt / 2, 3.5, 0, 7); c.fill(); }
        }
        for (const rw of rowsOf(A)) { const x = X(rw.t); c.fillStyle = rw.points > 0 ? '#36D399' : '#5b6575'; c.fillRect(x - 0.5, y + hgt - 6, 1.5, 6); }
      }
      if (k === 'speed') for (const [run, which, col] of [[A, 'a', colA], [B, 'b', colB]]) {
        if (!run || !S.show[which]) continue;
        c.beginPath(); c.strokeStyle = col; c.lineWidth = 1.2; if (which === 'b') c.setLineDash([3, 2]);
        let s = false; for (let i = 0; i < run.track.t.length; i++) { const tt = which === 'a' ? run.track.t[i] : toA(run.track.t[i]); if (tt < tmin || tt > tmax) continue; const yy = y + hgt - Math.min(1, run.track.v[i] / vmax) * hgt; s ? c.lineTo(X(tt), yy) : c.moveTo(X(tt), yy); s = true; }
        c.stroke(); c.setLineDash([]);
      }
      if (k === 'resid') {
        if (res.length) {
          const m = Math.max(1, ...res.map((q) => q.d)); c.beginPath(); c.moveTo(X(res[0].t), y + hgt);
          for (const q of res) c.lineTo(X(q.t), y + hgt - (q.d / m) * hgt); c.lineTo(X(res.at(-1).t), y + hgt); c.closePath();
          const g = c.createLinearGradient(0, y, 0, y + hgt); g.addColorStop(0, 'rgba(242,143,99,0.85)'); g.addColorStop(1, 'rgba(201,83,31,0.1)'); c.fillStyle = g; c.fill();
          c.fillStyle = '#8d8c85'; c.font = '9px Chivo Mono, monospace'; c.fillText(`max ${m.toFixed(1)} m`, w - 74, y + 9);
        } else { c.fillStyle = '#8d8c85'; c.font = '10px Chivo, sans-serif'; c.fillText(B ? 'hidden: alignment confidence low' : 'no second tier', LEFT + 6, y + hgt / 2 + 4); }
      }
      if (k === 'channels') {
        const empty = [[A, tierA], [B, tierB]].filter(([run]) => run && run.channels.perception === 'empty');
        if (empty.length) { c.fillStyle = hatch(c); c.fillRect(LEFT, y, w - LEFT - 14, hgt); }
        c.fillStyle = '#c3c2b7'; c.font = '10px Chivo, sans-serif';
        c.fillText([[A, tierA], [B, tierB]].filter(([run]) => run).map(([run, n]) => `${n}: ${run.channels.perception || 'not declared'}`).join(' · '), LEFT + 6, y + hgt - 2);
      }
      if (k === 'real') { c.setLineDash([3, 3]); c.strokeStyle = '#4a5568'; c.strokeRect(LEFT + 0.5, y + 0.5, w - LEFT - 15, hgt - 1); c.setLineDash([]); c.fillStyle = '#6b7280'; c.font = '10px Chivo, sans-serif'; c.fillText('absent: no real-boat source adapter yet', LEFT + 6, y + hgt - 2); }
      y += hgt + 4;
    }
    if (al?.mode === 'event') for (const [, ta] of al.anchors) { c.fillStyle = '#7DD3DC'; c.fillRect(X(ta) - 0.5, 0, 1, 6); }
    const px = X(S.t); c.fillStyle = 'rgba(125,211,220,0.12)'; c.fillRect(px - 4, 0, 8, r.height); c.fillStyle = '#7DD3DC'; c.fillRect(px - 0.75, 0, 1.5, r.height);
    $('clock').textContent = `${fmt.t(S.t)} / ${fmt.t(tmax)}`;
    $('alnote').innerHTML = al ? `align <b>${al.mode}</b> · ${esc(al.note)}` : 'single tier';
  }

  // ---------- interaction ----------
  const segs = () => {
    $('align').innerHTML = ['event', 'progress', 'relative'].map((k) => `<button data-k="${k}" aria-pressed="${S.align === k}" ${B ? '' : 'disabled'}>${k}</button>`).join('');
    $('speed').innerHTML = [1, 4, 16].map((k) => `<button data-k="${k}" aria-pressed="${S.speed === k}">${k}×</button>`).join('');
  };
  segs();
  $('align').onclick = (e) => { const b = e.target.closest('button'); if (!b) return; S.align = b.dataset.k; ctx.remember({ align: S.align }); segs(); load(); draw(); };
  $('speed').onclick = (e) => { const b = e.target.closest('button'); if (!b) return; S.speed = +b.dataset.k; segs(); };
  $('layers').onchange = (e) => { const k = e.target.dataset.l; if (k) { S.layers[k] = e.target.checked; draw(); } };
  const toggle = () => { S.playing = !S.playing; if (S.playing && S.t >= tmax) S.t = tmin; $('play').textContent = S.playing ? '❚❚' : '▶'; if (!S.playing) ctx.remember({ t: S.t.toFixed(1) }); };
  $('play').onclick = toggle;
  let last = performance.now(), frames = 0;
  const reduce = matchMedia('(prefers-reduced-motion: reduce)').matches;
  function tick(now) {
    if (!alive) return;
    const dt = Math.min(0.1, (now - last) / 1000); last = now;
    if (!reduce) ripple += dt;
    if (S.playing) { S.t += dt * S.speed; if (S.t >= tmax) { S.t = tmax; toggle(); } draw(); } else if (S.layers.water && !reduce && ++frames % 6 === 0) draw();
    raf = requestAnimationFrame(tick);
  }
  let scrub = false;
  const scrubTo = (e) => { const r = tc.getBoundingClientRect(); const f = (e.clientX - r.left - LEFT) / (r.width - LEFT - 14); S.t = Math.max(tmin, Math.min(tmax, tmin + f * (tmax - tmin))); draw(); };
  tc.onpointerdown = (e) => { scrub = true; tc.setPointerCapture(e.pointerId); scrubTo(e); };
  tc.onpointermove = (e) => scrub && scrubTo(e);
  tc.onpointerup = () => { scrub = false; ctx.remember({ t: S.t.toFixed(1) }); };
  let drag = null;
  mc.onpointerdown = (e) => { drag = { x: e.clientX, y: e.clientY, moved: false }; mc.setPointerCapture(e.pointerId); };
  mc.onpointermove = (e) => {
    if (drag) { const dx = e.clientX - drag.x, dy = e.clientY - drag.y; if (Math.abs(dx) + Math.abs(dy) > 2) drag.moved = true; S.view.cx -= dx / S.view.s; S.view.cy += dy / S.view.s; drag.x = e.clientX; drag.y = e.clientY; draw(); return; }
    const h = pick(e), tip = $('tip');
    if (h) { const r = mc.getBoundingClientRect(); tip.style.display = 'block'; tip.style.left = `${e.clientX - r.left + 14}px`; tip.style.top = `${e.clientY - r.top + 10}px`; tip.innerHTML = `<b>${esc(h.title)}</b><br><span style="color:var(--tx3)">${esc(h.body)}</span>`; } else tip.style.display = 'none';
  };
  mc.onpointerleave = () => ($('tip').style.display = 'none');
  mc.onpointerup = (e) => { if (drag && !drag.moved) { const h = pick(e); if (h) { S.sel = h; if (h.t != null) S.t = Math.max(tmin, Math.min(tmax, h.t)); draw(); } } drag = null; };
  mc.onwheel = (e) => { e.preventDefault(); const r = mc.getBoundingClientRect(); S.view.zoomAt(e.clientX - r.left, e.clientY - r.top, e.deltaY < 0 ? 1.15 : 1 / 1.15); draw(); };
  function pick(e) {
    const r = mc.getBoundingClientRect(), px = e.clientX - r.left, py = e.clientY - r.top, v = S.view;
    if (S.layers.pins) for (const p of pins()) if (Math.hypot(v.sx(p.x) - px, v.sy(p.y) - 21 - py) < 9) return p;
    for (const el of els) if (Math.hypot(v.sx(el.x) - px, v.sy(el.y) - py) < Math.max(7, el.r * v.s + 3)) return { kind: 'element', el, title: el.id || el.label, body: `${el.label} · r ${el.r} m · ${el.role}${el.task ? ` · ${el.task}` : ''}`, ev: el, t: null };
    let best = null, bd = 10; const tr = A.track;
    for (let i = 0; i < tr.t.length; i++) { const d = Math.hypot(v.sx(tr.x[i]) - px, v.sy(tr.y[i]) - py); if (d < bd) { bd = d; best = i; } }
    if (best != null) return { kind: 'track', t: tr.t[best], title: `${tierA} track at ${fmt.t(tr.t[best])}`, body: `speed ${fmt.ms(tr.v[best])} · heading ${fmt.deg(tr.yaw[best])} · click to pin time`, ev: { t: tr.t[best], x: tr.x[best], y: tr.y[best], yaw: tr.yaw[best], v: tr.v[best] } };
    return null;
  }
  $('insp').onclick = (e) => { const el = e.target.closest('[data-t]'); if (el && el.dataset.t !== '') { S.t = Math.max(tmin, Math.min(tmax, +el.dataset.t)); ctx.remember({ t: S.t.toFixed(1) }); draw(); } };
  const onKey = (e) => {
    if (/SELECT|INPUT|TEXTAREA/.test(e.target.tagName)) return;
    const k = e.key;
    if (k === ' ') { e.preventDefault(); toggle(); }
    else if (k === 'l' || k === 'L') S.t = Math.min(tmax, S.t + 5);
    else if (k === 'j' || k === 'J') S.t = Math.max(tmin, S.t - 5);
    else if (k === '.') S.t = Math.min(tmax, S.t + 0.1);
    else if (k === ',') S.t = Math.max(tmin, S.t - 0.1);
    else if (k === ']' || k === '[') { const ts = A.events.filter((x) => x.lane !== 'goal' && x.lane !== 'behaviour').map((x) => x.t); S.t = k === ']' ? ts.find((t) => t > S.t + 0.05) ?? S.t : [...ts].reverse().find((t) => t < S.t - 0.05) ?? S.t; }
    else if (k === 'f' || k === 'F') fit();
    else {
      const l = LAYERS.flatMap((g) => g[1]).find((x) => x[3].toLowerCase() === k.toLowerCase());
      if (!l) return;
      S.layers[l[0]] = !S.layers[l[0]]; renderLayers();
    }
    draw();
  };
  addEventListener('keydown', onKey);
  const ro = new ResizeObserver(() => { fit(); draw(); }); ro.observe(host.querySelector('.mapwrap'));
  load(); fit(); draw(); raf = requestAnimationFrame(tick);
  return { destroy() { alive = false; cancelAnimationFrame(raf); ro.disconnect(); removeEventListener('keydown', onKey); } };
}

function quant(arr, q) { const a = Array.from(arr).filter(Number.isFinite).sort((x, y) => x - y); return a.length ? a[Math.min(a.length - 1, Math.floor(q * a.length))] : 1; }
