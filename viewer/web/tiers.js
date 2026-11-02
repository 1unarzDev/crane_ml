// tiers.js: the tier bench. Two runs on one clock: synchronised maps (linked brushing: hover either track to
// see the matched point on the other), the clock-warp diagram (shared anchors as connectors, 5 s stretch
// ticks, scrub from either axis), residual strips, and the judge's record of the pair: outcome, gap origin
// per segment, the first differing decision, the G4 table and how the scores rank across every pair.
import { sample, makeAlign, residuals, firstDivergence, ramp, RESID, TIER, DECISION, elementColor, isStructure,
  View, bounds, nearElements, hull, sizeCanvas, fmt, esc, bisect } from './core.js';
import { judgeFor, judgePair } from './data.js';

export async function mount(host, ctx) {
  const { src, route } = ctx;
  const [aId, bId] = route.args;
  if (!bId) throw new Error('the tier bench needs two runs: #/tiers/<a>/<b>');
  const [A, B] = await Promise.all([src.run(aId), src.run(bId)]);
  const [course, reports] = await Promise.all([src.course(A.course || B.course), src.judge([aId, bId])]);
  ctx.study = A.study; ctx.pair = [A.id, B.id];
  const nA = A.tier || A.arm || 'A', nB = B.tier && B.tier !== A.tier ? B.tier : B.arm || 'B';
  const cA = TIER[A.tier] || '#3FC8E0', cB = TIER[B.tier] && B.tier !== A.tier ? TIER[B.tier] : '#C792EA';
  const jp = judgePair(A.id, B.id, reports);
  const g4 = reports.find((r) => r.report.g4)?.report;
  const spansA = judgeFor(A, reports, B.id), spansB = judgeFor(B, reports, A.id);
  const L = course?.vessel?.length || 1.06, BM = course?.vessel?.beam || 0.9;
  const S = { t: route.q.t != null ? +route.q.t : Infinity, align: route.q.align || 'event', playing: false, speed: 4, hover: null, shared: true };
  let al, res, div, tmax, raf = 0, alive = true;
  const els = nearElements(course, [A.track, B.track]);
  const views = { a: new View(), b: new View() };

  host.innerHTML = `<div class="bench">
    <section class="tcol"><header><b style="color:${cA}">${esc(nA)}</b><span class="note">${esc(A.arm || '')}</span><span id="ha"></span></header><div class="mbox"><canvas id="ma"></canvas><div class="hud tl" id="ta"></div></div></section>
    <section class="tcol"><header><b style="color:${cB}">${esc(nB)}</b><span class="note">${esc(B.arm || '')}</span><span id="hb"></span></header><div class="mbox"><canvas id="mb"></canvas><div class="hud tl" id="tb"></div></div></section>
    <section class="realcol"><b>REAL · ABSENT</b><span>No real-boat source adapter yet, so there is no third column to compare against. The plant card's band and the 11-01 bags will land here.</span><span class="note">channels: all absent</span></section>
    <section class="panel span3 warp"><div class="ctrl" style="padding:0 0 6px;border:0"><button class="play" id="play" aria-label="Play">▶</button><div class="seg" id="speed"></div><div class="clock" id="clock"></div><div class="seg" id="align"></div><div class="seg" id="cam"></div><div class="note al" id="alnote"></div></div><canvas id="warp" aria-label="Clock warp"></canvas></section>
    <section class="panel span3 resid"><h4><span>Residuals on the ${esc(nA)} clock</span><span id="divn"></span></h4><canvas id="res" aria-label="Residual strips"></canvas></section>
    <section class="span3 trio"><div class="panel" id="judge"></div><div class="panel" id="g4"></div><div class="panel rank"><h4><span>Rank agreement · every pair</span><span id="rankh"></span></h4><canvas id="rank"></canvas><div class="note" id="rankn"></div></div></section>
  </div>`;
  const $ = (s) => host.querySelector(`#${s}`);

  function load() {
    al = makeAlign(A, B, S.align); res = residuals(A, B, al); div = firstDivergence(res);
    tmax = A.track.t.at(-1) || 1; S.t = Math.max(0, Math.min(S.t, tmax));
    $('alnote').innerHTML = `align <b>${al.mode}</b> · ${esc(al.note)}`;
    $('divn').textContent = div ? `first divergence ${fmt.t(div.t)} (|Δxy| > ${div.thr} m for ${div.hold} s; geometry, not a verdict)` : res.length ? 'no divergence above 1 m' : 'hidden: alignment confidence low';
    const sc = (r, c, n) => `<span class="chip ${r.score > 0 ? 'o' : r.score < 0 ? 'f' : ''}" style="border-color:${c}66">${esc(n)} ${r.score ?? '–'} pts · ${esc(r.outcome)}</span>`;
    ctx.chips(sc(A, cA, nA) + sc(B, cB, nB) + `<span class="chip">${A.seed != null ? `seed ${A.seed} · ` : ''}${esc((A.task || '').replace('_', ' '))}</span>` + (jp?.pair?.code_mismatch?.length ? '<span class="chip w" title="the two tiers ran different nav or testing commits: the gap mixes tier and code">code differs</span>' : ''));
    $('ha').textContent = `${fmt.t(A.track.t.at(-1))} · ${A.events.filter((e) => e.lane === 'decision').length} decisions`;
    $('hb').textContent = `${fmt.t(B.track.t.at(-1))} · ${B.events.filter((e) => e.lane === 'decision').length} decisions`;
  }
  function fit() {
    const b = bounds([A.track, B.track], els, 3);
    for (const k of ['a', 'b']) {
      const r = $(`m${k}`).getBoundingClientRect();
      views[k].fit(S.shared ? b : bounds([k === 'a' ? A.track : B.track], [], 3), r.width, r.height, 26);
    }
  }

  function drawMap(k) {
    const run = k === 'a' ? A : B, other = k === 'a' ? B : A, col = k === 'a' ? cA : cB, ocol = k === 'a' ? cB : cA, v = views[k];
    const [c, r] = sizeCanvas($(`m${k}`)); v.w = r.width; v.h = r.height;
    const g = c.createRadialGradient(v.w * 0.5, v.h * 0.45, 10, v.w * 0.5, v.h * 0.5, Math.max(v.w, v.h) * 0.7);
    g.addColorStop(0, '#0f2238'); g.addColorStop(1, '#04070d'); c.fillStyle = g; c.fillRect(0, 0, v.w, v.h);
    const step = v.s * 5 < 28 ? 25 : 5; c.strokeStyle = '#1a2840'; c.lineWidth = 0.6; c.beginPath();
    for (let x = Math.floor(v.wx(0) / step) * step; x <= v.wx(v.w); x += step) { c.moveTo(v.sx(x), 0); c.lineTo(v.sx(x), v.h); }
    for (let y = Math.floor(v.wy(v.h) / step) * step; y <= v.wy(0); y += step) { c.moveTo(0, v.sy(y)); c.lineTo(v.w, v.sy(y)); }
    c.stroke();
    for (const e of els) { if (isStructure(e)) continue; c.beginPath(); c.arc(v.sx(e.x), v.sy(e.y), Math.max(2.5, e.r * v.s), 0, 7); c.fillStyle = elementColor(e.label); c.globalAlpha = 0.85; c.fill(); c.globalAlpha = 1; }
    const t = k === 'a' ? S.t : al.map(S.t);
    // the other tier, faint, for reference
    c.strokeStyle = ocol; c.globalAlpha = 0.18; c.lineWidth = 1; c.setLineDash([4, 4]); line(c, v, other.track, 0, other.track.t.length - 1); c.stroke(); c.setLineDash([]); c.globalAlpha = 1;
    // own track: decisions colour the segments
    const decs = run.events.filter((e) => e.lane === 'decision');
    const i1 = bisect(run.track.t, t);
    c.lineWidth = 2.2; c.lineCap = 'round';
    for (let i = 1; i <= i1; i++) {
      let d = null; for (const e of decs) { if (e.t <= run.track.t[i]) d = e; else break; }
      c.strokeStyle = d ? DECISION[d.decision] || col : col; c.beginPath(); c.moveTo(v.sx(run.track.x[i - 1]), v.sy(run.track.y[i - 1])); c.lineTo(v.sx(run.track.x[i]), v.sy(run.track.y[i])); c.stroke();
    }
    c.globalAlpha = 0.25; c.lineWidth = 1; c.strokeStyle = col; line(c, v, run.track, i1, run.track.t.length - 1); c.stroke(); c.globalAlpha = 1;
    // judge spans
    for (const j of k === 'a' ? spansA : spansB) { const a0 = bisect(run.track.t, j.t0), a1 = bisect(run.track.t, j.t1); if (a1 > a0) { c.save(); c.strokeStyle = '#EDCB78'; c.globalAlpha = 0.2; c.lineWidth = 9; line(c, v, run.track, a0, a1); c.stroke(); c.restore(); } }
    for (const e of run.events) if ((e.kind === 'contact' || e.kind === 'near_miss') && Number.isFinite(e.x)) { c.strokeStyle = e.kind === 'contact' ? '#ff5a52' : '#EDCB78'; c.lineWidth = 1.5; c.beginPath(); c.arc(v.sx(e.x), v.sy(e.y), 7, 0, 7); c.stroke(); }
    const p = sample(run.track, t);
    if (p) hull(c, v, p.x, p.y, p.yaw, L, BM, { stroke: col, fill: '#0b111a', glow: col, blur: 12, minPx: 13, alpha: p.clamped ? 0.4 : 1 });
    // linked brushing: the hovered instant on both maps
    if (S.hover) {
      const th = k === S.hover.k ? S.hover.t : S.hover.k === 'a' ? al.map(S.hover.t) : al.inv(S.hover.t);
      const q = sample(run.track, th);
      if (q) { c.strokeStyle = '#fff'; c.lineWidth = 1.2; c.beginPath(); c.arc(v.sx(q.x), v.sy(q.y), 9, 0, 7); c.stroke(); c.beginPath(); c.moveTo(v.sx(q.x) - 14, v.sy(q.y)); c.lineTo(v.sx(q.x) + 14, v.sy(q.y)); c.moveTo(v.sx(q.x), v.sy(q.y) - 14); c.lineTo(v.sx(q.x), v.sy(q.y) + 14); c.stroke();
        c.fillStyle = '#fff'; c.font = '10.5px Chivo Mono, monospace'; c.fillText(`${fmt.t(th)}${q.clamped ? ' (ended)' : ''}`, v.sx(q.x) + 12, v.sy(q.y) - 10); }
    }
    $(`t${k}`).innerHTML = p ? `${fmt.t(t)} · ${fmt.ms(p.v)} · ψ ${fmt.deg(p.yaw)}${p.clamped ? ' · ended' : ''}` : '';
  }
  function line(c, v, tr, a, b) { c.beginPath(); for (let i = a; i <= b; i++) { const x = v.sx(tr.x[i]), y = v.sy(tr.y[i]); i > a ? c.lineTo(x, y) : c.moveTo(x, y); } }

  // ---------- warp ----------
  const W = { L: 96, R: 20, yA: 34, yB: 112 };
  function drawWarp() {
    const cv = $('warp'), [c, r] = sizeCanvas(cv), w = r.width;
    const bMin = Math.min(0, B.track.t[0]), bMax = B.track.t.at(-1), span = Math.max(tmax, bMax - bMin, 1);
    const XA = (t) => W.L + (t / span) * (w - W.L - W.R), XB = (t) => W.L + ((t - bMin) / span) * (w - W.L - W.R);
    Object.assign(W, { XA, XB, span, bMin, w });
    c.font = '12px Chivo, sans-serif'; c.fillStyle = cA; c.fillText(`${nA} clock`, 8, W.yA + 4); c.fillStyle = cB; c.fillText(`${nB} clock`, 8, W.yB + 4);
    const curve = (ta, tb) => { c.beginPath(); c.moveTo(XA(ta), W.yA + 5); c.bezierCurveTo(XA(ta), W.yA + 40, XB(tb), W.yB - 40, XB(tb), W.yB - 5); };
    c.strokeStyle = '#1d2c44'; c.lineWidth = 1;
    for (let t = 0; t <= tmax; t += 5) { curve(t, al.map(t)); c.stroke(); }
    for (const [n, ta, tb] of al.anchors) { c.strokeStyle = '#7DD3DC'; c.lineWidth = 2; curve(ta, tb); c.stroke(); c.fillStyle = '#7DD3DC'; c.font = '10.5px Chivo Mono, monospace'; c.fillText(n, (XA(ta) + XB(tb)) / 2 + 5, (W.yA + W.yB) / 2); }
    for (const [run, X, y, col, top] of [[A, XA, W.yA, cA, true], [B, XB, W.yB, cB, false]]) {
      c.strokeStyle = col; c.lineWidth = 3; c.beginPath(); c.moveTo(X(run.track.t[0]), y); c.lineTo(X(run.track.t.at(-1)), y); c.stroke();
      c.fillStyle = '#5d6b82'; c.font = '9.5px Chivo Mono, monospace'; const st = span > 200 ? 20 : 10;
      for (let t = Math.ceil(run.track.t[0] / st) * st; t <= run.track.t.at(-1); t += st) { c.fillRect(X(t), y + (top ? -8 : 4), 1, 4); c.fillText(`${t}`, X(t) + 2, y + (top ? -10 : 16)); }
      for (const e of run.events) {
        if (e.lane !== 'decision' && e.lane !== 'task') continue;
        c.fillStyle = e.lane === 'decision' ? DECISION[e.decision] || col : e.kind === 'gate_cleared' ? '#36D399' : e.kind === 'task_finished' ? (/FAIL|ABORT/i.test(e.text) ? '#ff5a52' : '#36D399') : '#c3c2b7';
        c.beginPath(); c.arc(X(e.t), y, e.lane === 'decision' ? 4.5 : 3, 0, 7); c.fill(); c.strokeStyle = '#04070d'; c.lineWidth = 1; c.stroke();
      }
    }
    const tb = al.map(S.t); c.strokeStyle = '#ff5a52'; c.lineWidth = 1.5; c.beginPath(); c.moveTo(XA(S.t), W.yA - 14); c.lineTo(XA(S.t), W.yA + 5);
    c.bezierCurveTo(XA(S.t), W.yA + 40, XB(tb), W.yB - 40, XB(tb), W.yB - 5); c.lineTo(XB(tb), W.yB + 14); c.stroke();
    $('clock').textContent = `${nA} ${fmt.t(S.t)} ⇄ ${nB} ${fmt.t(tb)}`;
  }

  // ---------- residual strips ----------
  const R = { L: 96, R: 20 };
  function drawRes() {
    const cv = $('res'), [c, r] = sizeCanvas(cv), w = r.width, X = (t) => R.L + (t / tmax) * (w - R.L - R.R); R.X = X; R.w = w;
    const rows = [['|Δxy| m', (q) => q.d, false], ['Δheading °', (q) => (q.dpsi * 180) / Math.PI, true], ['Δspeed m/s', (q) => q.dv, true]];
    const H = (r.height - 12) / rows.length;
    rows.forEach(([name, f, signed], k) => {
      const y0 = k * H + 4, h = H - 8;
      c.fillStyle = '#8d8c85'; c.font = '11px Chivo, sans-serif'; c.fillText(name, 8, y0 + h / 2 + 4);
      c.strokeStyle = '#16243a'; c.strokeRect(R.L, y0, w - R.L - R.R, h);
      if (!res.length) { c.fillStyle = '#8d8c85'; c.fillText('hidden: alignment confidence low', R.L + 8, y0 + h / 2 + 4); return; }
      const vals = res.map(f), m = Math.max(1e-6, ...vals.map(Math.abs)), Y = (v) => (signed ? y0 + h / 2 - (v / m) * (h / 2) : y0 + h - (v / m) * h);
      if (!signed) { const thr = 1; if (thr < m) { c.fillStyle = 'rgba(255,90,82,0.06)'; c.fillRect(R.L, y0, w - R.L - R.R, Y(thr) - y0); c.strokeStyle = 'rgba(255,90,82,0.4)'; c.setLineDash([3, 3]); c.beginPath(); c.moveTo(R.L, Y(thr)); c.lineTo(w - R.R, Y(thr)); c.stroke(); c.setLineDash([]); } }
      else { c.strokeStyle = '#1d2c44'; c.beginPath(); c.moveTo(R.L, y0 + h / 2); c.lineTo(w - R.R, y0 + h / 2); c.stroke(); }
      if (!signed) { // filled, coloured by magnitude
        for (let i = 1; i < res.length; i++) { const q = res[i]; c.fillStyle = ramp(RESID, q.d / 3); c.globalAlpha = q.clamped ? 0.25 : 0.9; c.fillRect(X(res[i - 1].t), Y(q.d), Math.max(1, X(q.t) - X(res[i - 1].t)), y0 + h - Y(q.d)); }
        c.globalAlpha = 1;
      } else { c.beginPath(); res.forEach((q, i) => (i ? c.lineTo(X(q.t), Y(f(q))) : c.moveTo(X(q.t), Y(f(q))))); c.strokeStyle = k === 1 ? '#9ec5f4' : '#C792EA'; c.lineWidth = 1.3; c.stroke(); c.lineWidth = 1; }
      const cl = res.filter((q) => q.clamped);
      if (cl.length) { c.fillStyle = 'rgba(141,140,133,0.12)'; c.fillRect(X(cl[0].t), y0, X(cl.at(-1).t) - X(cl[0].t), h); c.fillStyle = '#8d8c85'; c.font = '10px Chivo Mono, monospace'; c.fillText(`${nB} ended`, X(cl[0].t) + 4, y0 + 11); }
      c.fillStyle = '#8d8c85'; c.font = '10px Chivo Mono, monospace'; c.fillText(`max ${m.toFixed(m < 10 ? 2 : 0)}`, R.L + 4, y0 + 11);
    });
    for (const j of spansA) { c.fillStyle = 'rgba(237,203,120,0.12)'; c.fillRect(X(Math.max(0, j.t0)), 0, Math.max(2, X(Math.min(tmax, j.t1)) - X(Math.max(0, j.t0))), 4); }
    if (div) { c.strokeStyle = '#F2D45C'; c.setLineDash([3, 3]); c.beginPath(); c.moveTo(X(div.t), 0); c.lineTo(X(div.t), r.height); c.stroke(); c.setLineDash([]); c.fillStyle = '#F2D45C'; c.font = '11px Chivo, sans-serif'; c.fillText('first divergence', X(div.t) + 4, r.height - 3); }
    c.fillStyle = '#7DD3DC'; c.fillRect(X(S.t) - 0.75, 0, 1.5, r.height);
  }

  // ---------- judge panels ----------
  const MOTION = { straight: '#3f7fbf', turn: '#c792ea', hold: '#5b6575' };
  function renderJudge() {
    let h = `<h4><span>The judge on this pair</span><span>${jp ? esc(jp.report.id) : ''}</span></h4>`;
    if (!jp) h += `<div class="note">${src.mode === 'local' ? 'No judge report pairs these two runs.' : 'Judge reports are read locally (python -m viewer serve).'}</div>`;
    else {
      const p = jp.pair, d = p.divergence || {}, ev = p.events || {};
      h += `<div class="kv" style="grid-template-columns:auto 1fr;margin-bottom:6px"><span class="k">outcomes</span><span>${esc((p.outcome || []).join(' → '))}</span>
        <span class="k">valid</span><span>${esc((p.valid || []).join(' / '))}</span>
        <span class="k">path residual</span><span>rms ${fmt.m(d.path_rms)} · max ${fmt.m(d.path_max)}</span>
        <span class="k">first divergence</span><span>${fmt.t(d.first_divergence_s)}</span>
        <span class="k">durations</span><span>${fmt.t(d.duration_low)} / ${fmt.t(d.duration_high)}</span></div>`;
      const segs = d.segments || [];
      if (segs.length) {
        const t0 = segs[0].t0, t1 = segs.at(-1).t1, tot = t1 - t0 || 1;
        h += `<div class="note">Gap origin by segment (${esc(nB)} clock; width = time, colour = motion; tall = local drift)</div><div class="seglist">${segs.map((s) => `<i title="${esc(s.label)} · ${esc(s.motion)} · local drift ${fmt.n(s.local_sum)} m · reset ${fmt.n(s.reset)} m" style="width:${((s.t1 - s.t0) / tot) * 100}%;background:${MOTION[s.motion] || '#3a4658'};opacity:${0.35 + Math.min(0.65, (s.local_sum || 0) / 10)}"></i>`).join('')}</div>`;
      }
      if (ev.kind) h += `<div class="ins ${ev.kind === 'decision' ? '' : 'info'}"><b>First differing ${esc(ev.kind)}</b> “${esc(ev.low)}” vs “${esc(ev.high)}”<small>at ${fmt.t(ev.first_t)} · residual ${fmt.m(ev.residual_m)} · ${ev.matched}/${ev.n_low}–${ev.n_high} events matched (agreement ${Math.round((ev.agreement || 0) * 100)}%)</small></div>`;
      if (p.code_mismatch?.length) h += `<div class="ins"><b>Different code</b> on the two tiers: ${esc(p.code_mismatch.map((m) => Object.entries(m).map(([k, v]) => `${k} ${v.map((x) => String(x).slice(0, 7)).join('≠')}`).join(', ')).join('; '))}<small>the gap mixes tier and code</small></div>`;
    }
    for (const j of [...spansA, ...spansB].slice(0, 6)) h += `<div class="ins" data-t="${j.t0}"><b>${esc(j.label)}</b> ${esc(j.claim || '')}<small>${fmt.t(j.t0)} – ${fmt.t(j.t1)}</small></div>`;
    $('judge').innerHTML = h;
  }
  function renderG4() {
    if (!g4?.g4?.metrics) { $('g4').innerHTML = `<h4><span>G4 · tier gap per objective</span></h4><div class="note">No G4 table for these runs.</div>`; return; }
    const ms = Object.entries(g4.g4.metrics);
    // each row on its own scale (objectives have different units): the band, the CI and zero are what matter
    let span = 1;
    const P = (v) => 50 + (v / span) * 46;
    let h = `<h4><span>G4 · tier gap per objective</span><span>${esc(g4.n_pairs)} pairs</span></h4><table class="g4"><tr><th>objective</th><th>bias [95% CI] vs band</th><th>bias</th><th>in band</th><th>verdict</th></tr>`;
    for (const [k, m] of ms) {
      const lo = m.bias_ci95?.[0], hi = m.bias_ci95?.[1];
      span = Math.max(Math.abs(lo ?? 0), Math.abs(hi ?? 0), Math.abs(m.bias ?? 0), m.band || 0) || 1;
      h += `<tr><td>${esc(k)}</td><td><span class="ci" title="band ±${m.band}"><span class="band" style="left:${P(-m.band)}%;right:${100 - P(m.band)}%"></span><span class="z" style="left:50%"></span>${lo != null ? `<span class="cib" style="left:${P(lo)}%;width:${P(hi) - P(lo)}%"></span>` : ''}<span class="pt" style="left:${P(m.bias)}%"></span></span></td><td>${fmt.n(m.bias)}</td><td>${Math.round((m.band_coverage || 0) * 100)}%</td><td class="s-${esc(m.verdict)}">${esc(m.verdict)}</td></tr>`;
    }
    h += '</table>';
    const fm = g4.g4.failure_modes;
    if (fm) h += `<div class="note" style="margin-top:8px">failure-mode agreement ${Math.round(fm.agreement * 100)}% over ${fm.n}: ${esc(Object.entries(fm.table).sort((a, b) => b[1] - a[1]).slice(0, 4).map(([k, v]) => `${k} ×${v}`).join(' · '))}</div>`;
    for (const i of (g4.insights || []).filter((x) => /^(tiers|hint)\./.test(x.kind)).slice(0, 5)) h += `<div class="ins ${esc(i.severity || '')}"><b>${esc(i.kind.split('.').pop().replace(/_/g, ' '))}</b> ${esc(i.claim)}<small>${esc(i.basis || '')}</small></div>`;
    $('g4').innerHTML = h;
  }
  function drawRank() {
    const ps = (src.idx?.pairs || []).filter((p) => p.low_score != null && p.high_score != null);
    const cv = $('rank'), [c, r] = sizeCanvas(cv), w = r.width, h = r.height, P = 34;
    if (!ps.length) { c.fillStyle = '#8d8c85'; c.font = '11px Chivo, sans-serif'; c.fillText('rank agreement needs the local pair index', 10, 20); return; }
    const rk = src.idx.rank; $('rankh').textContent = rk ? `ρ ${rk.spearman} · MMRV ${rk.mmrv} pts · ${rk.reversed_pairs} reversed` : '';
    $('rankn').textContent = 'off-diagonal: the tiers disagree on the score; ring: this pair. ρ and MMRV are computed by the server.';
    const xs = ps.map((p) => p.low_score), ys = ps.map((p) => p.high_score), lo = Math.min(...xs, ...ys) - 20, hi = Math.max(...xs, ...ys) + 20;
    const X = (v) => P + ((v - lo) / (hi - lo)) * (w - P - 12), Y = (v) => h - P - ((v - lo) / (hi - lo)) * (h - P - 12);
    c.strokeStyle = '#1d2c44'; c.strokeRect(P, 12, w - P - 12, h - P - 12); c.setLineDash([3, 3]); c.beginPath(); c.moveTo(X(lo), Y(lo)); c.lineTo(X(hi), Y(hi)); c.stroke(); c.setLineDash([]);
    c.fillStyle = '#8d8c85'; c.font = '10px Chivo Mono, monospace'; c.fillText(`${nA} pts →`, w - 90, h - 8); c.fillText(`↑ ${nB} pts`, P + 4, 24);
    const groups = {}; ps.forEach((p) => (groups[`${p.low_score},${p.high_score}`] ||= []).push(p));
    for (const [k, g] of Object.entries(groups)) {
      const [fx, uy] = k.split(',').map(Number), cur = g.some((p) => p.low === A.id && p.high === B.id), rad = 4 + Math.sqrt(g.length) * 3, ok = fx === uy;
      c.beginPath(); c.arc(X(fx), Y(uy), rad, 0, 7); c.fillStyle = ok ? '#36D399' : '#ff5a52'; c.globalAlpha = 0.25; c.fill(); c.globalAlpha = 1; c.strokeStyle = ok ? '#36D399' : '#ff5a52'; c.stroke();
      c.fillStyle = '#c3c2b7'; c.fillText(`${g.length}`, X(fx) + rad + 3, Y(uy) + 4);
      if (cur) { c.strokeStyle = '#fff'; c.lineWidth = 2; c.beginPath(); c.arc(X(fx), Y(uy), rad + 4, 0, 7); c.stroke(); c.lineWidth = 1; }
    }
  }

  function drawAll() { if (!alive) return; drawMap('a'); drawMap('b'); drawWarp(); drawRes(); }

  // ---------- interaction ----------
  const segs = () => {
    $('align').innerHTML = ['event', 'progress', 'relative'].map((k) => `<button data-k="${k}" aria-pressed="${S.align === k}">${k}</button>`).join('');
    $('speed').innerHTML = [1, 4, 16].map((k) => `<button data-k="${k}" aria-pressed="${S.speed === k}">${k}×</button>`).join('');
    $('cam').innerHTML = `<button data-k="shared" aria-pressed="${S.shared}">shared camera</button><button data-k="own" aria-pressed="${!S.shared}">own</button>`;
  };
  segs();
  $('align').onclick = (e) => { const b = e.target.closest('button'); if (!b) return; S.align = b.dataset.k; ctx.remember({ align: S.align }); segs(); load(); drawAll(); };
  $('speed').onclick = (e) => { const b = e.target.closest('button'); if (!b) return; S.speed = +b.dataset.k; segs(); };
  $('cam').onclick = (e) => { const b = e.target.closest('button'); if (!b) return; S.shared = b.dataset.k === 'shared'; segs(); fit(); drawAll(); };
  const toggle = () => { S.playing = !S.playing; if (S.playing && S.t >= tmax) S.t = 0; $('play').textContent = S.playing ? '❚❚' : '▶'; if (!S.playing) ctx.remember({ t: S.t.toFixed(1) }); };
  $('play').onclick = toggle;
  function scrub(cv, fn) { let on = false; cv.onpointerdown = (e) => { on = true; cv.setPointerCapture(e.pointerId); fn(e); }; cv.onpointermove = (e) => on && fn(e); cv.onpointerup = () => { on = false; ctx.remember({ t: S.t.toFixed(1) }); }; }
  scrub($('warp'), (e) => {
    const r = $('warp').getBoundingClientRect(), x = e.clientX - r.left, y = e.clientY - r.top, f = (x - W.L) / (W.w - W.L - W.R);
    S.t = y < (W.yA + W.yB) / 2 ? f * W.span : al.inv(W.bMin + f * W.span); S.t = Math.max(0, Math.min(tmax, S.t)); drawAll();
  });
  scrub($('res'), (e) => { const r = $('res').getBoundingClientRect(); S.t = Math.max(0, Math.min(tmax, ((e.clientX - r.left - R.L) / (R.w - R.L - R.R)) * tmax)); drawAll(); });
  for (const k of ['a', 'b']) {
    const cv = $(`m${k}`), run = k === 'a' ? A : B;
    cv.onpointermove = (e) => {
      const r = cv.getBoundingClientRect(), px = e.clientX - r.left, py = e.clientY - r.top, v = views[k];
      let best = null, bd = 14; for (let i = 0; i < run.track.t.length; i++) { const d = Math.hypot(v.sx(run.track.x[i]) - px, v.sy(run.track.y[i]) - py); if (d < bd) { bd = d; best = i; } }
      S.hover = best == null ? null : { k, t: run.track.t[best] }; drawMap('a'); drawMap('b');
    };
    cv.onpointerleave = () => { S.hover = null; drawMap('a'); drawMap('b'); };
    cv.onclick = () => { if (S.hover) { S.t = Math.max(0, Math.min(tmax, S.hover.k === 'a' ? S.hover.t : al.inv(S.hover.t))); ctx.remember({ t: S.t.toFixed(1) }); drawAll(); } };
    cv.onwheel = (e) => { e.preventDefault(); const r = cv.getBoundingClientRect(); for (const kk of S.shared ? ['a', 'b'] : [k]) views[kk].zoomAt(e.clientX - r.left, e.clientY - r.top, e.deltaY < 0 ? 1.15 : 1 / 1.15); drawAll(); };
  }
  host.querySelector('.trio').onclick = (e) => { const el = e.target.closest('[data-t]'); if (el) { S.t = Math.max(0, Math.min(tmax, +el.dataset.t)); drawAll(); } };
  const onKey = (e) => {
    if (/SELECT|INPUT/.test(e.target.tagName)) return;
    if (e.key === ' ') { e.preventDefault(); toggle(); }
    else if (e.key === ']' || e.key === '[') {
      const ts = [...A.events.filter((x) => x.lane === 'decision' || x.lane === 'task').map((x) => x.t), ...B.events.filter((x) => x.lane === 'decision' || x.lane === 'task').map((x) => al.inv(x.t))].sort((a, b) => a - b);
      S.t = e.key === ']' ? ts.find((t) => t > S.t + 0.05) ?? S.t : [...ts].reverse().find((t) => t < S.t - 0.05) ?? S.t; drawAll();
    } else if (e.key === 'f' || e.key === 'F') { fit(); drawAll(); }
  };
  addEventListener('keydown', onKey);
  let last = performance.now();
  function tick(now) { if (!alive) return; const dt = Math.min(0.1, (now - last) / 1000); last = now; if (S.playing) { S.t += dt * S.speed; if (S.t >= tmax) { S.t = tmax; toggle(); } drawAll(); } raf = requestAnimationFrame(tick); }
  const ro = new ResizeObserver(() => { fit(); drawAll(); drawRank(); }); ro.observe(host.querySelector('.bench'));
  load(); renderJudge(); renderG4(); fit(); drawAll(); drawRank(); raf = requestAnimationFrame(tick);
  return { destroy() { alive = false; cancelAnimationFrame(raf); ro.disconnect(); removeEventListener('keydown', onKey); } };
}
