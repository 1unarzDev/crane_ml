// app.js: the viewer shell. Hash routes (ids are URI-encoded):
//   #/run/<id>[?vs=<id>&t=&align=]   one run in depth, a second tier as a ghost (the chart room)
//   #/tiers/<a>/<b>[?t=&align=]      two runs on one clock: warp, residuals, linked maps, the judge's pair record
//   #/batch/<study>[?seed=&focus=]   variants x seeds, ECDF, per-seed delta, ghost race, lap chart
// Local (`python -m viewer serve`) adds the run index and judge reports; on the dashboard (/viewer/) the same
// page reads the public feed only.
import { Source } from './data.js';
import { esc } from './core.js';

const $ = (id) => document.getElementById(id);
const src = await new Source().init();
let current = null, token = 0;

export function parseRoute() {
  const h = location.hash.replace(/^#\/?/, '');
  const [path, qs] = h.split('?');
  const parts = path.split('/').filter(Boolean).map(decodeURIComponent);
  return { mode: parts[0] || '', args: parts.slice(1), q: Object.fromEntries(new URLSearchParams(qs || '')) };
}
export function href(mode, args = [], q = {}) {
  const qs = new URLSearchParams(Object.entries(q).filter(([, v]) => v !== undefined && v !== null && v !== '')).toString();
  return `#/${[mode, ...args.map(encodeURIComponent)].join('/')}${qs ? `?${qs}` : ''}`;
}
/** Replace the query without a re-route (playhead, alignment, focus): deep links stay current. */
export function remember(q) {
  const r = parseRoute();
  const next = href(r.mode, r.args, { ...r.q, ...q });
  if (next !== location.hash) history.replaceState(null, '', next);
}

const label = (r) => [r.arm || r.tier || r.study, r.seed != null ? `s${r.seed}` : null, r.task?.replace('_', ' '), r.score != null ? `${r.score} pts` : null, r.outcome].filter(Boolean).join(' · ');

function defaultRoute() {
  if (src.mode !== 'local' || !src.idx.runs.length) return null;
  const ps = src.idx.pairs || [];
  const p = ps.find((x) => x.task === 'nav_channel' && x.low_score > 0 && x.high_score < 0) || ps.find((x) => x.low_score != null && x.high_score != null && x.low_score !== x.high_score) || ps[0];
  if (p) return href('run', [p.low]);
  return href('run', [src.idx.runs[0].run_id]);
}

function header(r, ctx) {
  const pick = $('pick');
  const runId = r.mode === 'run' ? r.args[0] : r.mode === 'tiers' ? r.args[0] : null;
  const study = r.mode === 'batch' ? r.args[0] : ctx.study;
  const tiersArgs = r.mode === 'tiers' ? r.args : ctx.pair;
  $('modes').innerHTML = [
    ['run', 'Run', runId ? href('run', [runId], ctx.pair ? { vs: ctx.pair[1] } : {}) : null],
    ['tiers', 'Tiers', tiersArgs && tiersArgs[1] ? href('tiers', tiersArgs) : null],
    ['batch', 'Batch', study ? href('batch', [study]) : null],
  ].map(([m, n, h]) => `<a href="${h || '#'}" class="${h ? '' : 'off'}" ${r.mode === m ? 'aria-current="page"' : ''}>${n}</a>`).join('');
  $('src').textContent = src.mode === 'local' ? `local · ${src.idx.runs.length} runs` : 'cluster feed';
  $('src').title = src.mode === 'local' ? 'python -m viewer serve: local crane-traj files and judge reports' : 'crane.lunarz.dev public feed (no judge reports)';
  if (src.mode !== 'local') { pick.innerHTML = ''; return; }
  if (r.mode === 'batch') {
    pick.innerHTML = `<select id="pk" aria-label="Study">${src.idx.studies.map((s) => `<option value="${esc(s.study)}" ${s.study === study ? 'selected' : ''}>${esc(s.study)} · ${s.runs} runs · ${s.arms.length} variants</option>`).join('')}</select>`;
    $('pk').onchange = (e) => (location.hash = href('batch', [e.target.value]));
  } else if (r.mode === 'tiers') {
    const ps = src.idx.pairs || [];
    pick.innerHTML = `<select id="pk" aria-label="Tier pair">${ps.map((p) => `<option value="${esc(p.low)}|${esc(p.high)}" ${p.low === r.args[0] && p.high === r.args[1] ? 'selected' : ''}>pair ${esc(p.id.slice(0, 8))} · s${p.seed} ${esc((p.task || '').replace('_', ' '))} · fast ${p.low_score ?? '–'} / unity ${p.high_score ?? '–'}</option>`).join('')}</select>`;
    $('pk').onchange = (e) => { const [a, b] = e.target.value.split('|'); location.hash = href('tiers', [a, b]); };
  } else {
    const by = {};
    for (const x of src.idx.runs) (by[x.study || ''] ||= []).push(x);
    pick.innerHTML = `<select id="pk" aria-label="Run">${Object.entries(by).map(([s, rs]) => `<optgroup label="${esc(s)}">${rs.map((x) => `<option value="${esc(x.run_id)}" ${x.run_id === runId ? 'selected' : ''}>${esc(label(x))} · ${esc(x.run_id.slice(0, 18))}</option>`).join('')}</optgroup>`).join('')}</select>`;
    $('pk').onchange = (e) => (location.hash = href('run', [e.target.value]));
  }
}

async function route() {
  const my = ++token;
  let r = parseRoute();
  if (!r.mode) {
    const d = defaultRoute();
    if (d) { history.replaceState(null, '', d); r = parseRoute(); }
  }
  if (current?.destroy) current.destroy();
  current = null;
  const host = $('view');
  if (!['run', 'tiers', 'batch'].includes(r.mode) || !r.args[0]) {
    header(r, {});
    host.innerHTML = `<div class="empty"><div><p>Nothing to show yet.</p><p class="note">${src.mode === 'local' ? 'No runs under the served roots. <code>python -m viewer ingest SRC --out DIR</code> writes crane-traj files.' : 'Open a run from the dashboard: its run page links here as “Deep view”.'}</p></div></div>`;
    return;
  }
  host.innerHTML = '<div class="boot">Loading…</div>';
  const ctx = { src, route: r, href, remember, chips: (html) => ($('chips').innerHTML = html), study: null, pair: null };
  $('chips').innerHTML = '';
  try {
    const mod = await import(`./${r.mode}.js`);
    if (my !== token) return;
    current = await mod.mount(host, ctx);
    if (my !== token) { current?.destroy?.(); return; }
    header(r, ctx);
  } catch (e) {
    if (my !== token) return;
    console.error(e);
    header(r, ctx);
    host.innerHTML = `<div class="empty"><div><p>${e.status === 404 ? 'This run has no stored trajectory.' : e.status === 410 ? 'This trajectory is no longer kept.' : e.status === 503 ? 'On the archive node, which is offline right now.' : 'This view could not be built.'}</p><p class="note">${esc(e.message)}</p></div></div>`;
  }
}

addEventListener('hashchange', () => { const r = parseRoute(); if (current?.update && current.update(r)) return; route(); });
route();
