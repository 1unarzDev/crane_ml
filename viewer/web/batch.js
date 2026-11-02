// batch.js: the batch board. A study's variants (arms, else iterations) against its seeds: the points matrix,
// each variant's score ECDF, the per-seed delta against a baseline, and the ghost race (every variant on one
// course at once: comet trails, contact pulses, a live leaderboard), a lap chart and the focused variant's
// small multiples. Click a cell for its seed and variant; a leaderboard row sets focus, shift-click (or a
// second click) sets the baseline; arrows step seeds and variants.
import { sample, ramp, SCORE, VARIANT, TIER, DECISION, elementColor, isStructure, View, bounds, hull, sizeCanvas, fmt, esc, pathTo } from './core.js';

const progressKinds = new Set(['gate_cleared', 'stage_completed', 'docked', 'returned_home', 'task_finished']);
const isContact = (e) => e.kind === 'contact' || e.kind === 'strike';

export async function mount(host, ctx) {
  const { src, route } = ctx;
  const study = route.args[0];
  const fleet = await src.fleet(study);
  const runs = fleet.models;
  if (!runs.length) throw new Error(`no stored trajectories in ${study}`);
  ctx.study = study;
  const key = (r) => r.arm || (r.tier ? r.tier : `iter ${r.iteration}`);
  const vnames = [...new Set(runs.map(key))].sort((a, b) => (a === 'unity') - (b === 'unity') || a.localeCompare(b, undefined, { numeric: true }));
  // a column is a scenario: the seed, and the task too when the study mixes tasks (one seed, different courses)
  const multiTask = new Set(runs.map((r) => r.task || '')).size > 1;
  const colOf = (r) => (multiTask ? `${r.seed}|${r.task || ''}` : `${r.seed}`);
  const ABBR = { nav_channel: 'nav', follow_path: 'path', docking: 'dock', return_home: 'home', speed_challenge: 'speed' };
  const seeds = [...new Set(runs.filter((r) => r.seed != null).map(colOf))].sort((a, b) => parseFloat(a) - parseFloat(b) || a.localeCompare(b));
  const colLabel = (k) => { const [sd, tk] = String(k).split('|'); return `s${sd}${tk ? ` ${ABBR[tk] || tk.slice(0, 5)}` : ''}`; };
  const VC = vnames.map((n, i) => (n === 'unity' ? TIER.unity : VARIANT[i % VARIANT.length]));
  const cell = (vi, s) => runs.filter((r) => key(r) === vnames[vi] && colOf(r) === s);
  const best = (vi, s) => cell(vi, s).sort((a, b) => (b.score ?? -1e9) - (a.score ?? -1e9))[0] || null;
  const scored = runs.filter((r) => r.score != null);
  const pmax = Math.max(1, ...scored.map((r) => r.score)), pmin = Math.min(0, ...scored.map((r) => r.score));
  const mean = (a) => (a.length ? a.reduce((x, y) => x + y, 0) / a.length : null);
  const courses = new Map();
  const qSeed = route.q.seed != null ? String(route.q.seed) : null;
  const S = { seed: seeds.includes(qSeed) ? qSeed : seeds.find((k) => qSeed != null && (k.split('|')[0] === qSeed || k === qSeed.split('|')[0])) ?? (seeds.find((s) => vnames.filter((_, vi) => cell(vi, s).length).length > 1) || seeds[0]), focus: Math.max(0, vnames.indexOf(route.q.focus)), base: 0,
    hide: new Set(), t: 0, playing: false, speed: 10, trails: true, contacts: true, view: new View() };
  if (!route.q.focus || vnames.indexOf(route.q.focus) < 0) { const f = vnames.findIndex((_, vi) => vi !== vnames.length - 1 && cell(vi, S.seed).length); if (f >= 0) S.focus = f; }
  S.base = vnames.indexOf('unity') >= 0 ? vnames.indexOf('unity') : vnames.length - 1;
  if (S.focus === S.base && vnames.length > 1) S.base = S.focus === 0 ? 1 : 0;
  let race = [], course = null, tEnd = 1, raf = 0, alive = true, pulse = 0;

  host.innerHTML = `<div class="board">
    <section class="pane"><h4><span>Points · variant × seed</span><span>${esc(study)} · ${runs.length} runs</span></h4><div class="matrix" id="matrix"></div>
      <div class="legend"><span class="rampbar" style="background:linear-gradient(90deg,${SCORE.join(',')})"></span><span>${pmin} – ${pmax} pts</span><span>· corner: runs in the cell · ✕ contacts</span></div><div id="stats"></div></section>
    <section class="pane"><div class="duo"><div><h4><span>Score ECDF per variant</span><span>dot: this seed</span></h4><canvas id="ecdf"></canvas></div>
      <div><h4><span>Δ points vs <em id="basename" style="font-style:normal;color:#efeaf2"></em></span><span>per seed</span></h4><canvas id="delta"></canvas></div></div>
      <div class="notes" id="notes" style="margin-top:10px"></div></section>
    <section class="theater"><div class="thead"><b id="seedt"></b><span id="seedm"></span><span style="margin-left:auto"></span><button class="play" id="play" aria-label="Play">▶</button><div class="seg" id="speed"></div><div class="seg" id="opts"></div><span class="clock" id="clock"></span></div>
      <div class="stage"><canvas id="th" aria-label="Ghost race"></canvas><div class="toast" id="toast"></div></div></section>
    <section class="side"><div class="pane lb" id="board"></div><div class="pane lap"><h4><span>Lap chart · decisions, gates, contacts</span><span>drag to scrub</span></h4><canvas id="lap"></canvas></div></section>
    <section class="pane" style="grid-column:1/3"><h4><span id="thumbh"></span><span>click a seed</span></h4><div class="thumbs" id="thumbs"></div></section>
  </div>`;
  const $ = (s) => host.querySelector(`#${s}`);

  // ---------- matrix ----------
  function renderMatrix() {
    const m = $('matrix');
    m.style.gridTemplateColumns = `minmax(110px,180px) repeat(${seeds.length}, minmax(34px,1fr)) 92px`;
    let h = '<div></div>' + seeds.map((s) => `<div class="mh ${s === S.seed ? 'sel' : ''}" data-seed="${esc(s)}">${esc(colLabel(s))}</div>`).join('') + '<div class="mh">mean</div>';
    vnames.forEach((n, vi) => {
      h += `<div class="rl ${S.hide.has(vi) ? 'off' : ''}" data-v="${vi}" title="${esc(n)}: click to hide"><i style="background:${VC[vi]}"></i><span>${esc(n)}</span></div>`;
      for (const s of seeds) {
        const rs = cell(vi, s), sc = rs.filter((r) => r.score != null), sel = s === S.seed && vi === S.focus;
        if (!rs.length) { h += `<div class="cell nil ${s === S.seed ? 'colsel' : ''}">·</div>`; continue; }
        if (!sc.length) { h += `<div class="cell nil ${sel ? 'sel' : ''}" data-v="${vi}" data-seed="${esc(s)}" title="${esc(colLabel(s))}: not scored">–</div>`; continue; }
        const v = mean(sc.map((r) => r.score)), f = (v - pmin) / (pmax - pmin || 1), nc = rs.reduce((a, r) => a + r.events.filter(isContact).length, 0);
        h += `<div class="cell ${sel ? 'sel' : ''} ${s === S.seed ? 'colsel' : ''}" data-v="${vi}" data-seed="${esc(s)}" style="background:${ramp(SCORE, f)};color:${f > 0.6 ? '#141019' : '#efeaf2'}" title="${esc(n)} · ${esc(colLabel(s))}: ${sc.map((r) => r.score).join(', ')} pts over ${rs.length} run(s)">${Math.round(v)}${rs.length > 1 ? `<span class="n">${rs.length}</span>` : ''}${nc ? `<span class="x">✕${nc}</span>` : ''}</div>`;
      }
      const all = runs.filter((r) => key(r) === n && r.score != null).map((r) => r.score), fin = runs.filter((r) => key(r) === n && r.outcome === 'finished').length, tot = runs.filter((r) => key(r) === n).length;
      h += `<div class="rs">${all.length ? mean(all).toFixed(0) : '–'}<small>${fin}/${tot} finished</small></div>`;
    });
    m.innerHTML = h;
  }

  // ---------- ECDF and delta ----------
  function drawECDF() {
    const [c, r] = sizeCanvas($('ecdf')), w = r.width, h = r.height, P = 28;
    const X = (v) => P + ((v - pmin) / (pmax - pmin || 1)) * (w - P - 10), Y = (f) => h - P - f * (h - P - 10);
    c.strokeStyle = '#2a2236'; c.beginPath(); for (const f of [0, 0.5, 1]) { c.moveTo(P, Y(f)); c.lineTo(w - 10, Y(f)); } c.stroke();
    c.fillStyle = '#857c90'; c.font = '9.5px Chivo Mono, monospace'; c.fillText(`${pmin}`, X(pmin), h - P + 13); c.fillText(`${pmax}`, X(pmax) - 16, h - P + 13); c.fillText('1.0', 2, Y(1) + 3); c.fillText('0.5', 2, Y(0.5) + 3);
    vnames.forEach((_, vi) => {
      if (S.hide.has(vi)) return;
      const pts = runs.filter((r) => key(r) === vnames[vi] && r.score != null).map((r) => r.score).sort((a, b) => a - b); if (!pts.length) return;
      c.strokeStyle = VC[vi]; c.lineWidth = vi === S.focus ? 2.4 : 1.3; c.beginPath(); c.moveTo(X(pmin), Y(0));
      pts.forEach((p, i) => { c.lineTo(X(p), Y(i / pts.length)); c.lineTo(X(p), Y((i + 1) / pts.length)); }); c.lineTo(X(pmax), Y(1)); c.stroke();
      const cur = best(vi, S.seed); if (cur?.score != null) { const k = pts.indexOf(cur.score); c.fillStyle = VC[vi]; c.beginPath(); c.arc(X(cur.score), Y((k + 0.5) / pts.length), 3.2, 0, 7); c.fill(); }
    });
    c.lineWidth = 1;
  }
  function drawDelta() {
    const [c, r] = sizeCanvas($('delta')), w = r.width, h = r.height, P = 28, b = S.base; $('basename').textContent = vnames[b];
    const ds = [];
    vnames.forEach((_, vi) => { if (vi === b || S.hide.has(vi)) return; for (const s of seeds) { const a = cell(vi, s).filter((x) => x.score != null), o = cell(b, s).filter((x) => x.score != null); if (a.length && o.length) ds.push({ vi, s, d: mean(a.map((x) => x.score)) - mean(o.map((x) => x.score)) }); } });
    const m = Math.max(50, ...ds.map((d) => Math.abs(d.d))), X = (s) => P + ((seeds.indexOf(s) + 0.5) / seeds.length) * (w - P - 10), Y = (d) => (h - P) / 2 + 5 - (d / m) * ((h - P) / 2 - 6);
    c.strokeStyle = '#4a3f58'; c.beginPath(); c.moveTo(P, Y(0)); c.lineTo(w - 10, Y(0)); c.stroke();
    c.fillStyle = '#857c90'; c.font = '9.5px Chivo Mono, monospace'; c.fillText(`+${m.toFixed(0)}`, 2, Y(m) + 8); c.fillText(`−${m.toFixed(0)}`, 2, Y(-m));
    const every = Math.max(1, Math.ceil(seeds.length / Math.max(1, Math.floor((w - P) / 64))));
    seeds.forEach((s, i) => { if (i % every === 0 || s === S.seed) c.fillText(colLabel(s), X(s) - 14, h - P + 13); });
    c.fillStyle = '#ffffff10'; c.fillRect(X(S.seed) - 10, 6, 20, h - P - 6);
    for (const d of ds) { c.fillStyle = VC[d.vi]; c.beginPath(); c.arc(X(d.s) + (d.vi - vnames.length / 2) * 2.4, Y(d.d), d.vi === S.focus ? 4 : 2.8, 0, 7); c.fill(); }
    if (!ds.length) { c.fillStyle = '#857c90'; c.fillText('no seed shared with the baseline', P + 6, 16); }
  }

  // ---------- the ghost race ----------
  async function setup() {
    race = vnames.map((_, vi) => ({ vi, r: best(vi, S.seed) })).filter((x) => x.r);
    tEnd = Math.max(1, ...race.map((x) => x.r.track.t.at(-1)));
    const sha = race[0]?.r.course;
    if (sha && !courses.has(sha)) courses.set(sha, await src.course(sha));
    course = courses.get(sha) || null;
    S.t = Math.min(S.t, tEnd);
  }
  function fit() { const r = $('th').getBoundingClientRect(); S.view.fit(bounds(race.map((x) => x.r.track), [], 5), r.width, r.height, 30); }
  function drawTheater() {
    if (!alive) return;
    const [c, r] = sizeCanvas($('th')), v = S.view; v.w = r.width; v.h = r.height;
    const g = c.createLinearGradient(0, 0, 0, v.h); g.addColorStop(0, '#17121f'); g.addColorStop(1, '#0b0910'); c.fillStyle = g; c.fillRect(0, 0, v.w, v.h);
    const st = course?.start || [race[0]?.r.track.x[0] || 0, race[0]?.r.track.y[0] || 0];
    c.strokeStyle = '#241c30'; c.lineWidth = 1; for (let k = 10; k <= 150; k += 10) { c.beginPath(); c.arc(v.sx(st[0]), v.sy(st[1]), k * v.s, 0, 7); c.stroke(); }
    c.fillStyle = '#4b3b6b'; c.font = '9.5px Chivo Mono, monospace'; for (let k = 20; k <= 140; k += 20) c.fillText(`${k} m`, v.sx(st[0]) + k * v.s + 3, v.sy(st[1]) - 3);
    if (course) {
      for (const t of course.tasks || []) { if (!t.footprint?.length) continue; c.beginPath(); t.footprint.forEach((p, i) => (i ? c.lineTo(v.sx(p[0]), v.sy(p[1])) : c.moveTo(v.sx(p[0]), v.sy(p[1])))); c.closePath(); c.fillStyle = '#ffffff05'; c.fill(); c.strokeStyle = '#3a2f48'; c.setLineDash([3, 4]); c.stroke(); c.setLineDash([]); }
      for (const e of course.elements) { if (isStructure(e)) { c.beginPath(); c.arc(v.sx(e.x), v.sy(e.y), e.r * v.s, 0, 7); c.strokeStyle = '#4b3b6b'; c.stroke(); continue; } c.save(); c.shadowColor = elementColor(e.label); c.shadowBlur = 6; c.beginPath(); c.arc(v.sx(e.x), v.sy(e.y), Math.max(2, e.r * v.s), 0, 7); c.fillStyle = elementColor(e.label); c.fill(); c.restore(); }
    }
    const order = race.filter((x) => !S.hide.has(x.vi)).sort((a, b) => (a.vi === S.focus) - (b.vi === S.focus));
    for (const { vi, r: run } of order) {
      const tr = run.track, foc = vi === S.focus, col = VC[vi];
      c.lineCap = 'round';
      if (S.trails) {
        c.strokeStyle = col; c.globalAlpha = foc ? 0.22 : 0.08; c.lineWidth = 1; c.beginPath(); let s = false;
        for (let i = 0; i < tr.t.length && tr.t[i] <= S.t; i++) { s ? c.lineTo(v.sx(tr.x[i]), v.sy(tr.y[i])) : c.moveTo(v.sx(tr.x[i]), v.sy(tr.y[i])); s = true; }
        c.stroke();
        for (let i = 1; i < tr.t.length && tr.t[i] <= S.t; i++) { const age = S.t - tr.t[i]; if (age > 40) continue; c.globalAlpha = (1 - age / 40) * (foc ? 1 : 0.65); c.lineWidth = foc ? 3.2 : 2; c.beginPath(); c.moveTo(v.sx(tr.x[i - 1]), v.sy(tr.y[i - 1])); c.lineTo(v.sx(tr.x[i]), v.sy(tr.y[i])); c.stroke(); }
        c.globalAlpha = 1;
      }
      if (S.contacts) for (const e of run.events) {
        if (!isContact(e) || e.t > S.t) continue;
        const p = Number.isFinite(e.x) ? e : sample(tr, e.t); const age = S.t - e.t, R = 6 + Math.max(0, 22 - age * 0.6);
        c.strokeStyle = col; c.globalAlpha = Math.max(0.3, 1 - age / 40); c.beginPath(); c.arc(v.sx(p.x), v.sy(p.y), age < 40 ? R + Math.sin(pulse * 6) * 1.5 : 5, 0, 7); c.stroke(); c.globalAlpha = 1;
        c.fillStyle = '#ff6b5e'; c.beginPath(); c.arc(v.sx(p.x), v.sy(p.y), 2.2, 0, 7); c.fill();
      }
      const p = sample(tr, S.t), done = S.t > tr.t.at(-1);
      hull(c, v, p.x, p.y, p.yaw, 1.06, 0.9, { stroke: col, fill: foc ? col : '#141019', glow: col, blur: foc ? 18 : 8, detail: false, bow: '#fff', minPx: foc ? 15 : 11, alpha: done ? 0.45 : 1 });
      if (foc) { c.fillStyle = col; c.font = `${foc ? 600 : 400} 11px Chivo, sans-serif`; c.fillText(vnames[vi], v.sx(p.x) + 11, v.sy(p.y) - 9); }
    }
    renderBoard(); drawLap();
    $('clock').textContent = `${fmt.t(S.t)} / ${fmt.t(tEnd)}`;
  }
  function renderBoard() {
    const rows = race.filter((x) => !S.hide.has(x.vi)).map(({ vi, r }) => ({ vi, r, st: r.events.filter((e) => progressKinds.has(e.kind) && e.t <= S.t).length, c: r.events.filter((e) => isContact(e) && e.t <= S.t).length, fin: S.t >= r.track.t.at(-1), pl: pathTo(r.track, S.t) }))
      .sort((a, b) => b.st - a.st || a.c - b.c || b.pl - a.pl);
    $('board').innerHTML = `<h3>At ${fmt.t(S.t)} · progress · path · contacts · final</h3>` + rows.map((x, i) => `<div class="brow ${x.fin ? 'fin' : ''}" data-v="${x.vi}" title="${esc(x.r.id)}"><i style="background:${VC[x.vi]}${x.vi === S.focus ? ';box-shadow:0 0 0 2px #fff' : ''}"></i><span class="nm">${i + 1}. ${esc(vnames[x.vi])}${x.vi === S.base ? ' <em style="color:#857c90">(base)</em>' : ''}</span><span>${x.st} ✓ · ${x.pl.toFixed(0)} m</span><span class="c">${x.c ? `${x.c}✕` : ''}</span><span>${x.r.score ?? '–'}</span></div>`).join('');
    const f = race.find((x) => x.vi === S.focus)?.r;
    let dec = null; if (f) for (const e of f.events) { if (e.lane === 'decision' && e.t <= S.t) dec = e; }
    $('toast').innerHTML = dec ? `<span style="color:${VC[S.focus]}">${esc(vnames[S.focus])}</span> · ${esc(dec.decision || 'decision')} at ${fmt.t(dec.t)}: “${esc(dec.text)}”` : `${esc(vnames[S.focus] || '')}: no decision yet`;
  }
  const lapGeo = { L: 150, R: 50 };
  function drawLap() {
    const cv = $('lap'), [c, r] = sizeCanvas(cv), w = r.width, vis = race.filter((x) => !S.hide.has(x.vi)), H = (r.height - 18) / Math.max(1, vis.length);
    const X = (t) => lapGeo.L + (t / tEnd) * (w - lapGeo.L - lapGeo.R); lapGeo.w = w;
    vis.forEach(({ vi, r: run }, k) => {
      const y = 4 + k * H; c.fillStyle = VC[vi]; c.font = `${vi === S.focus ? 600 : 400} 11px Chivo, sans-serif`; c.fillText(vnames[vi].slice(0, 22), 10, y + H / 2 + 4);
      c.fillStyle = '#ffffff08'; c.fillRect(lapGeo.L, y + 2, X(run.track.t.at(-1)) - lapGeo.L, H - 4);
      const decs = run.events.filter((e) => e.lane === 'decision');
      decs.forEach((d, i) => { const t1 = decs[i + 1]?.t ?? run.track.t.at(-1); c.fillStyle = DECISION[d.decision] || '#7DD3DC'; c.globalAlpha = 0.28; c.fillRect(X(d.t), y + 3, Math.max(1, X(t1) - X(d.t)), H - 6); c.globalAlpha = 1; });
      for (const e of run.events) {
        if (e.kind === 'gate_cleared' || e.kind === 'stage_completed') { c.fillStyle = '#36D399'; c.fillRect(X(e.t) - 1, y + 2, 2, H - 4); }
        if (isContact(e)) { c.fillStyle = '#ff6b5e'; c.beginPath(); c.arc(X(e.t), y + H / 2, 2.8, 0, 7); c.fill(); }
        if (e.kind === 'task_finished') { c.fillStyle = /FAIL|ABORT/i.test(e.text) ? '#ff6b5e' : '#36D399'; c.font = '10px Chivo, sans-serif'; c.fillText('⚑', X(e.t) - 3, y + H / 2 + 4); }
      }
      c.fillStyle = '#857c90'; c.font = '9.5px Chivo Mono, monospace'; c.fillText(fmt.t(run.track.t.at(-1)), X(run.track.t.at(-1)) + 3, y + H / 2 + 3);
    });
    c.fillStyle = '#857c90'; c.font = '9.5px Chivo Mono, monospace'; const st = tEnd > 300 ? 60 : tEnd > 120 ? 30 : 10; for (let t = 0; t <= tEnd; t += st) c.fillText(`${t}s`, X(t) + 2, r.height - 3);
    c.fillStyle = '#efeaf2'; c.fillRect(X(S.t) - 0.75, 0, 1.5, r.height);
  }

  function statsTable() {
    const med = (a) => { const b = [...a].sort((x, y) => x - y); return b.length ? (b.length % 2 ? b[(b.length - 1) / 2] : (b[b.length / 2 - 1] + b[b.length / 2]) / 2) : null; };
    const rows = vnames.map((n, vi) => {
      const rs = runs.filter((r) => key(r) === n), sc = rs.filter((r) => r.score != null).map((r) => r.score);
      return `<tr class="${S.hide.has(vi) ? 'off' : ''}"><td><i style="display:inline-block;width:8px;height:8px;border-radius:2px;background:${VC[vi]};margin-right:6px"></i>${esc(n)}</td><td>${rs.length}</td><td>${Math.round((100 * rs.filter((r) => r.outcome === 'finished').length) / (rs.length || 1))}%</td><td>${sc.length ? mean(sc).toFixed(0) : '–'}</td><td>${sc.length ? med(sc) : '–'}</td><td>${rs.reduce((a, r) => a + r.events.filter(isContact).length, 0) || ''}</td><td>${fmt.t(mean(rs.map((r) => r.track.t.at(-1))))}</td><td>${rs.reduce((a, r) => a + r.events.filter((e) => e.decision === 'retry' || e.decision === 'give_up').length, 0) || ''}</td></tr>`;
    }).join('');
    return `<table class="g4" style="margin-top:10px"><tr><th>variant</th><th>runs</th><th>finished</th><th>mean</th><th>median</th><th>contacts</th><th>mean time</th><th>retries</th></tr>${rows}</table>`;
  }

  // ---------- small multiples, notes ----------
  function renderThumbs() {
    const n = vnames[S.focus], rs = runs.filter((r) => key(r) === n).sort((a, b) => seeds.indexOf(colOf(a)) - seeds.indexOf(colOf(b)) || (b.score ?? 0) - (a.score ?? 0));
    $('thumbh').textContent = `${n} · ${rs.length} runs`;
    $('thumbs').innerHTML = rs.map((r, i) => `<div class="thumb ${colOf(r) === S.seed ? 'sel' : ''}" data-seed="${esc(colOf(r))}" data-i="${i}" title="${esc(r.id)}"><canvas></canvas><span>${esc(colLabel(colOf(r)))} · ${r.score ?? '–'} · ${esc(r.outcome)}</span></div>`).join('');
    [...$('thumbs').children].forEach((el, i) => {
      const r = rs[i], [c, rc] = sizeCanvas(el.querySelector('canvas')), vw = new View(); vw.fit(bounds([r.track], [], 3), rc.width, rc.height, 6);
      const bad = r.outcome !== 'finished' || r.events.some(isContact);
      c.strokeStyle = bad ? '#ff6b5e' : VC[S.focus]; c.globalAlpha = 0.9; c.lineWidth = 1.1; c.beginPath();
      for (let j = 0; j < r.track.t.length; j++) j ? c.lineTo(vw.sx(r.track.x[j]), vw.sy(r.track.y[j])) : c.moveTo(vw.sx(r.track.x[j]), vw.sy(r.track.y[j]));
      c.stroke(); c.globalAlpha = 1;
      for (const e of r.events) if (isContact(e)) { const p = Number.isFinite(e.x) ? e : sample(r.track, e.t); c.fillStyle = '#ff6b5e'; c.beginPath(); c.arc(vw.sx(p.x), vw.sy(p.y), 2, 0, 7); c.fill(); }
    });
  }
  function renderNotes() {
    const m = vnames.map((n, vi) => ({ vi, n, s: runs.filter((r) => key(r) === n && r.score != null).map((r) => r.score) })).filter((x) => x.s.length);
    const top = [...m].sort((a, b) => mean(b.s) - mean(a.s))[0];
    const unscored = runs.filter((r) => r.score == null).length;
    $('notes').innerHTML = `<b>Reading this batch.</b> ${top ? `Highest mean: <span style="color:${VC[top.vi]}">${esc(top.n)}</span> (${mean(top.s).toFixed(0)} pts over ${top.s.length} runs). ` : ''}` +
      `Cells average every run of a variant on a seed (the corner counts them); the race and the lap chart show each variant's best run on ${esc(colLabel(S.seed))}. ` +
      `${unscored ? `${unscored} runs carry no score and are left out of the ECDF and Δ. ` : ''}Fast-tier contacts are geometric and never stop the boat: a flag to re-run in Unity, not a strike verdict. ` +
      `Means and ECDFs describe; they are not a comparison verdict (the judge's paired test is).`;
    $('stats').innerHTML = statsTable();
    $('seedt').textContent = colLabel(S.seed).replace(/^s/, 'Seed ');
    $('seedm').textContent = `${course?.name || ''} · ${race.length} variants racing`;
    $('speed').innerHTML = [5, 10, 30].map((k) => `<button data-k="${k}" aria-pressed="${S.speed === k}">${k}×</button>`).join('');
    $('opts').innerHTML = `<button data-o="trails" aria-pressed="${S.trails}">trails</button><button data-o="contacts" aria-pressed="${S.contacts}">contacts</button><button data-o="fit">fit</button>`;
  }
  async function renderAll() {
    await setup(); if (!alive) return;
    ctx.remember({ seed: S.seed, focus: vnames[S.focus] });
    ctx.chips(`<span class="chip">${esc(study)}</span><span class="chip">${vnames.length} variants × ${seeds.length} seeds</span><span class="chip" style="border-color:${VC[S.focus]}">focus ${esc(vnames[S.focus])}</span>${race.find((x) => x.vi === S.focus) ? `<a class="chip" href="${ctx.href('run', [race.find((x) => x.vi === S.focus).r.id])}">open run →</a>` : ''}`);
    renderMatrix(); drawECDF(); drawDelta(); renderThumbs(); renderNotes(); drawTheater();
  }

  // ---------- interaction ----------
  $('matrix').onclick = (e) => {
    const c = e.target.closest('[data-seed]'), rl = e.target.closest('.rl');
    if (rl) { const vi = +rl.dataset.v; S.hide.has(vi) ? S.hide.delete(vi) : S.hide.add(vi); renderAll(); return; }
    if (c) { S.seed = c.dataset.seed; if (c.dataset.v != null) S.focus = +c.dataset.v; S.t = 0; setup().then(() => { fit(); renderAll(); }); }
  };
  $('board').onclick = (e) => { const b = e.target.closest('.brow'); if (!b) return; const vi = +b.dataset.v; if (e.shiftKey || S.focus === vi) S.base = vi; S.focus = vi; renderAll(); };
  $('thumbs').onclick = (e) => { const el = e.target.closest('.thumb'); if (el) { S.seed = el.dataset.seed; S.t = 0; setup().then(() => { fit(); renderAll(); }); } };
  $('speed').onclick = (e) => { const b = e.target.closest('button'); if (b) { S.speed = +b.dataset.k; renderNotes(); } };
  $('opts').onclick = (e) => { const b = e.target.closest('button'); if (!b) return; if (b.dataset.o === 'fit') fit(); else S[b.dataset.o] = !S[b.dataset.o]; renderNotes(); drawTheater(); };
  const toggle = () => { S.playing = !S.playing; if (S.playing && S.t >= tEnd) S.t = 0; $('play').textContent = S.playing ? '❚❚' : '▶'; };
  $('play').onclick = toggle;
  let on = false; const lap = $('lap');
  const lapTo = (e) => { const r = lap.getBoundingClientRect(); S.t = Math.max(0, Math.min(tEnd, ((e.clientX - r.left - lapGeo.L) / (lapGeo.w - lapGeo.L - lapGeo.R)) * tEnd)); drawTheater(); };
  lap.onpointerdown = (e) => { on = true; lap.setPointerCapture(e.pointerId); lapTo(e); }; lap.onpointermove = (e) => on && lapTo(e); lap.onpointerup = () => (on = false);
  const th = $('th'); let drag = null;
  th.onpointerdown = (e) => { drag = { x: e.clientX, y: e.clientY }; th.setPointerCapture(e.pointerId); };
  th.onpointermove = (e) => { if (!drag) return; S.view.cx -= (e.clientX - drag.x) / S.view.s; S.view.cy += (e.clientY - drag.y) / S.view.s; drag = { x: e.clientX, y: e.clientY }; drawTheater(); };
  th.onpointerup = () => (drag = null);
  th.onwheel = (e) => { e.preventDefault(); const r = th.getBoundingClientRect(); S.view.zoomAt(e.clientX - r.left, e.clientY - r.top, e.deltaY < 0 ? 1.15 : 1 / 1.15); drawTheater(); };
  const onKey = (e) => {
    if (/SELECT|INPUT/.test(e.target.tagName)) return;
    if (e.key === ' ') { e.preventDefault(); toggle(); return; }
    let changed = false;
    if (e.key === 'ArrowRight' || e.key === 'ArrowLeft') { const i = seeds.indexOf(S.seed) + (e.key === 'ArrowRight' ? 1 : -1); if (i >= 0 && i < seeds.length) { S.seed = seeds[i]; S.t = 0; changed = true; } }
    if (e.key === 'ArrowDown' || e.key === 'ArrowUp') { const i = S.focus + (e.key === 'ArrowDown' ? 1 : -1); if (i >= 0 && i < vnames.length) { S.focus = i; changed = true; } }
    if (changed) { e.preventDefault(); setup().then(() => { fit(); renderAll(); }); }
  };
  addEventListener('keydown', onKey);
  let last = performance.now();
  function tick(now) { if (!alive) return; const dt = Math.min(0.1, (now - last) / 1000); last = now; pulse += dt; if (S.playing) { S.t += dt * S.speed; if (S.t >= tEnd) { S.t = tEnd; toggle(); } drawTheater(); } raf = requestAnimationFrame(tick); }
  const ro = new ResizeObserver(() => { fit(); drawTheater(); drawECDF(); drawDelta(); }); ro.observe(host.querySelector('.board'));
  await setup(); fit();
  S.t = +route.q.t || Math.min(tEnd, Math.max(...race.map((x) => x.r.track.t.at(-1) * 0.6)));
  await renderAll(); raf = requestAnimationFrame(tick);
  return { destroy() { alive = false; cancelAnimationFrame(raf); ro.disconnect(); removeEventListener('keydown', onKey); } };
}
