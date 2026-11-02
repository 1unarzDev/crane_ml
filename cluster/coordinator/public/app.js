// crane run dashboard ("Ledger"): the public, read-only view of the cluster's queue. Reads /v1/feed and its SSE
// stream, the job pages read /v1/feed/jobs/:id, and replays read the /v1/feed replay routes. Hash routes:
// #/now #/runs #/campaigns #/workers #/job/:id #/run/:run_id #/fleet/:study. No build step.
import { decode, track } from './traj.js';
import { mount, ramp } from './replay.js';

const $ = (s, el = document) => el.querySelector(s);
const esc = (s) => String(s ?? '').replace(/[&<>"']/g, (c) => ({ '&': '&amp;', '<': '&lt;', '>': '&gt;', '"': '&quot;', "'": '&#39;' }[c]));
const STAGES = ['fast', 'unity', 'real'];
const WORDS = ['No', 'One', 'Two', 'Three', 'Four', 'Five', 'Six', 'Seven', 'Eight', 'Nine'];
const num = (n) => (n < 10 ? WORDS[n] : String(n));
const plural = (n, w) => `${num(n)} ${w}${n === 1 ? '' : 's'}`;
const OP_KEY = 'crane.operator';

let FEED = null, connState = 'connecting', lastView = 'now', theater = null, jobCache = null, routeKey = '';

// ---------------------------------------------------------------- formatting
function dur(s) {
  if (s == null || isNaN(s)) return '–';
  s = Math.max(0, s);
  if (s < 1) return `${Math.round(s * 1000)} ms`;
  if (s < 59.5) return `${s.toFixed(s < 10 ? 1 : 0)} s`;
  s = Math.round(s);
  const m = Math.floor(s / 60), r = s % 60;
  if (m < 60) return r ? `${m} m ${r} s` : `${m} m`;
  const h = Math.floor(m / 60), mm = m % 60;
  return mm ? `${h} h ${mm} m` : `${h} h`;
}
function ago(iso) {
  if (!iso) return '–';
  const s = (Date.now() - Date.parse(iso)) / 1000;
  if (s < 45) return 'just now';
  if (s < 3600) return `${Math.round(s / 60)} min ago`;
  if (s < 86400) return `${Math.round(s / 3600)} h ago`;
  return `${Math.round(s / 86400)} d ago`;
}
const clock = (iso) => (iso ? new Date(iso).toLocaleTimeString([], { hour: '2-digit', minute: '2-digit', hour12: false }) : '–');
const left = (iso) => (iso ? Math.max(0, (Date.parse(iso) - Date.now()) / 1000) : null);
const isLegacy = (j) => j.purpose === '(legacy)';
const perRun = (j) => j.estimate.model_run_s ?? j.estimate.agent_run_s;
const failed = (r) => r.status !== 'ok' || ['failed', 'timeout', 'invalid', 'FAILURE'].includes(r.outcome);
const byId = () => Object.fromEntries(FEED.jobs.map((j) => [j.id, j]));
const runsOf = (id) => FEED.runs.filter((r) => r.job_id === id).sort((a, b) => a.seed - b.seed);
const jobOfRun = (rid) => /^(.*)\.a\d+\.s-?\d+$/.exec(rid)?.[1] ?? null;
const stage = (j) => (j.hypothesis?.stage ? `<span class="stage ${esc(j.hypothesis.stage)}">${esc(j.hypothesis.stage)}</span>` : '');
const who = (j) => [j.submitter && `<b>${esc(j.submitter)}</b>`, j.campaign && esc(j.campaign), j.hypothesis && `${esc(j.hypothesis.name)} ${esc(j.hypothesis.arm)} ${stage(j)}`]
  .filter(Boolean).map((x) => `<span>${x}</span>`).join('');
const live = (iso) => `<span data-left="${esc(iso)}">${dur(left(iso))}</span>`;

// ---------------------------------------------------------------- feed
async function getJSON(url) {
  const r = await fetch(url, { headers: { accept: 'application/json' } });
  if (!r.ok) throw Object.assign(new Error(`${url}: ${r.status}`), { status: r.status });
  return r.json();
}
function setFeed(f) {
  const same = FEED && JSON.stringify({ ...FEED, generated_at: 0 }) === JSON.stringify({ ...f, generated_at: 0 });
  FEED = f; connState = 'live'; status();
  if (!same) render(false);
}
function connect() {
  getJSON('/v1/feed?recent=200').then(setFeed, () => { connState = 'error'; status(); });
  if (!('EventSource' in window)) return setInterval(() => getJSON('/v1/feed?recent=200').then(setFeed, () => {}), 10000);
  const es = new EventSource('/v1/feed/stream?recent=200');
  es.addEventListener('feed', (e) => { try { setFeed(JSON.parse(e.data)); } catch {} });
  es.onerror = () => { connState = 'error'; status(); };
  es.onopen = () => { if (connState === 'error') { connState = 'live'; status(); } };
}
function status() {
  const el = $('#clock'); if (!el) return;
  const age = FEED ? (Date.now() - Date.parse(FEED.generated_at)) / 1000 : 0;
  const st = connState === 'error' ? 'err' : age > 60 ? 'stale' : FEED ? 'ok' : 'wait';
  const text = st === 'err' ? 'can’t reach the feed, retrying' : st === 'stale' ? `feed ${dur(age).replace(/ \d+ s$/, '')} old`
    : FEED ? `${clock(FEED.generated_at)}, ${plural(FEED.summary.workers_live, 'worker').toLowerCase()} live` : 'connecting';
  el.className = `clock ${st}`; el.innerHTML = `<i></i>${esc(text)}`;
}
setInterval(() => {
  status();
  document.querySelectorAll('[data-left]').forEach((el) => { el.textContent = dur(left(el.dataset.left)); });
}, 1000);

// ---------------------------------------------------------------- now
const LOG1 = Math.log(7200);
const lx = (s) => Math.max(0, Math.min(100, (Math.log(Math.max(1, s)) / LOG1) * 100));
function durGlyph(j) {
  const model = j.estimate.job_s, agent = j.estimate.agent_run_s != null ? j.estimate.agent_run_s * j.runs_total : null;
  const off = agent && model && Math.abs(model / agent - 1) > 0.15;
  return `<div class="dur" title="model ${dur(model)}${agent ? `, agent ${dur(agent)}` : ''}"><div class="g"><i style="width:${lx(model)}%"></i>${agent ? `<u style="left:${lx(agent)}%"></u>` : ''}</div>
    <div class="l"><span>${j.runs_total} × ${dur(perRun(j))}</span><b>${dur(model)}</b></div>${off ? `<div class="l warn">agent said ${dur(agent)}</div>` : ''}</div>`;
}
function seedStrip(j) {
  const rs = runsOf(j.id); if (!rs.length) return '<div></div>';
  const mx = Math.max(...rs.map((r) => r.score || 0), 1e-9);
  return `<div class="strip" title="${rs.length} runs; height is the score">${rs.slice(0, 60).map((r) => `<i class="${failed(r) ? 'f' : ''}" style="height:${Math.max(3, ((r.score || 0) / mx) * 22)}px"></i>`).join('')}</div>`;
}
function progress(j) {
  const ok = j.progress.runs_ok, f = j.progress.runs_failed;
  return j.runs_total <= 40
    ? `<div class="prog">${Array.from({ length: j.runs_total }, (_, i) => `<i class="${i < ok ? 'ok' : i < ok + f ? 'f' : ''}"></i>`).join('')}</div>`
    : `<div class="prog cont"><i style="width:${(j.progress.fraction || 0) * 100}%"></i></div>`;
}
function viewNow() {
  const s = FEED.summary, J = byId();
  const liveW = FEED.workers.filter((w) => w.live), busy = liveW.filter((w) => w.job).length;
  const queued = FEED.jobs.filter((j) => j.state === 'queued').sort((a, b) => a.queue_position - b.queue_position);
  const clear = new Date(Date.parse(FEED.generated_at) + s.drain_s * 1000).toISOString();
  const idleSince = FEED.jobs.filter((j) => j.ended_at).map((j) => j.ended_at).sort().pop();
  const lede = !s.workers_live ? `No workers live; ${plural(s.queued, 'job').toLowerCase()} will wait.`
    : !s.queued && !busy ? `Nothing waiting. ${plural(liveW.length, 'worker')} idle${idleSince ? ` since <em>${clock(idleSince)}</em>` : ''}.`
    : `${plural(busy, 'worker')} busy. ${plural(s.queued, 'job')} waiting, <span class="d">${s.queued_runs} runs;</span> the queue clears at <em>${clock(clear)}</em>.`;
  const berths = liveW.map((w) => {
    const j = w.job && J[w.job];
    if (!j) return `<div class="berth idle"><div class="w"><span>${esc(w.alias)}</span><span>idle</span></div><div class="p">Waiting for a job it can run.</div><div class="by">${esc(w.backends.join(', '))}</div></div>`;
    const done = j.progress.runs_ok + j.progress.runs_failed;
    return `<a class="berth" href="#/job/${esc(j.id)}"><div class="w"><b>${esc(w.alias)}</b><span>${esc(j.backend)}</span></div>
      <div class="p">${esc(j.purpose)}</div><div class="by">${esc(j.submitter)}${j.hypothesis ? `, ${esc(j.hypothesis.arm)} ${stage(j)}` : ''}</div>${progress(j)}
      <div class="rem"><span class="mute">${done} of ${j.runs_total}${j.progress.runs_failed ? `, <span class="bad">${j.progress.runs_failed} failed</span>` : ''}</span><span><b>${live(j.eta?.done_at)}</b> <span class="mute">left</span></span></div></a>`;
  }).join('');
  const row = (j) => `<a class="item" href="#/job/${esc(j.id)}"><div class="t">${j.eta?.start_at ? clock(j.eta.start_at) : '–'}</div><div class="n">${j.queue_position ?? ''}</div>
      <div><div class="p">${esc(j.purpose)}</div><div class="m">${who(j)}${j.eta?.note ? `<span class="warn">${esc(j.eta.note)}</span>` : ''}</div></div>${durGlyph(j)}</a>`;
  const rest = queued.slice(8), restS = rest.reduce((a, j) => a + (j.estimate.job_s || 0), 0);
  const next = queued.slice(0, 8).map(row).join('') + (rest.length ? `<details class="more"><summary>and ${rest.length} more, ${dur(restS)} of work</summary>${rest.map(row).join('')}</details>` : '');
  const past = FEED.jobs.filter((j) => j.ended_at && !isLegacy(j)).sort((a, b) => Date.parse(b.ended_at) - Date.parse(a.ended_at)).slice(0, 5).map(pastItem).join('');
  return [
    ['lede', `<p class="lede">${lede}</p>`],
    ['now', `<section><h2>Now</h2><div class="now">${berths || '<div class="berth idle"><div class="p">No worker is live.</div></div>'}</div></section>`],
    ['next', `<section><h2>Next <span>start time, then place in the queue</span></h2><div class="scale"><span>1 s</span><span>1 m</span><span>1 h</span></div><div class="list">${next || '<div class="item empty"><div></div><div></div><div class="p mute">Nothing is waiting.</div></div>'}</div></section>`],
    ['earlier', past ? `<section><h2>Earlier</h2><div class="list">${past}</div></section>` : ''],
  ];
}
function pastItem(j) {
  const line = j.judge || j.error;
  return `<a class="item past" href="#/job/${esc(j.id)}"><div class="t">${ago(j.ended_at).replace(' ago', '')}</div><div class="n"></div>
    <div><div class="p">${esc(j.purpose)}</div>${line ? `<div class="verdict ${j.error ? 'bad' : j.state === 'done' ? 'ok' : ''}">${esc(line)}</div>` : ''}
    <div class="m"><span class="st ${esc(j.state)}">${esc(j.state)}</span>${who(j)}${j.replay.available ? `<span class="rp">replay ${j.replay.runs} runs</span>` : ''}</div></div>${seedStrip(j)}</a>`;
}

// ---------------------------------------------------------------- runs
function viewRuns() {
  const J = byId(), groups = new Map();
  for (const r of FEED.runs) (groups.get(r.job_id) ?? groups.set(r.job_id, []).get(r.job_id)).push(r);
  const ent = [...groups].filter(([id]) => J[id]);
  const main = ent.filter(([id]) => !isLegacy(J[id])), legacy = ent.filter(([id]) => isLegacy(J[id]));
  const grp = ([id, rs]) => {
    const j = J[id], mx = Math.max(...rs.map((r) => r.wall_s || 0), 0.01);
    rs.sort((a, b) => a.seed - b.seed);
    return `<div class="grp"><a class="p clamp" href="#/job/${esc(id)}" title="${esc(j.purpose)}">${esc(j.purpose)}</a>
      <div class="ticks">${rs.map((r) => `<a href="#/run/${esc(r.run_id)}" title="seed ${r.seed}, ${dur(r.wall_s)}, ${esc(r.outcome || r.status)}${r.score != null ? `, score ${r.score}` : ''}${r.replay_state ? `, replay ${r.replay_state}` : ''}" class="${failed(r) ? 'f' : 'win'}${r.replay ? '' : ' nr'}" style="height:${Math.max(4, ((r.wall_s || 0) / mx) * 34)}px"></a>`).join('')}</div>
      <div class="m"><span class="st ${esc(j.state)}">${esc(j.state)}</span><span>${rs.length} runs</span><span>${[...new Set(rs.map((r) => r.worker).filter(Boolean))].map(esc).join(', ')}</span><span>last ${ago(rs.reduce((a, r) => (a > r.at ? a : r.at), ''))}</span>${rs.some((r) => r.replay) ? '<span class="rp">replayable</span>' : ''}</div></div>`;
  };
  return [
    ['lede', `<p class="lede sub">${FEED.runs.length} recent runs from ${plural(groups.size, 'job').toLowerCase()}. <span class="d">Each bar is a run; its height is the wall time.</span></p>
      <p class="legend"><i class="win"></i>finished <i class="f"></i>failed <i class="win nr"></i>no replay kept</p>`],
    ['groups', main.map(grp).join('') || '<p class="mute">No runs in the feed yet.</p>'],
    ['legacy', legacy.length ? `<details class="more legacy"><summary>${legacy.length} jobs from before purposes were required</summary>${legacy.map(grp).join('')}</details>` : ''],
  ];
}

// ---------------------------------------------------------------- campaigns
function campaigns() {
  const out = new Map();
  for (const j of FEED.jobs) {
    if (isLegacy(j)) continue;
    const key = j.campaign || '(no campaign)';
    const c = out.get(key) ?? out.set(key, { name: key, jobs: [], hyps: new Map(), subs: new Set(), studies: new Set(), last: 0 }).get(key);
    c.jobs.push(j); if (j.submitter) c.subs.add(j.submitter);
    if (j.replay.available && j.replay.study) c.studies.add(j.replay.study);
    c.last = Math.max(c.last, Date.parse(j.ended_at || j.started_at || j.created_at));
    if (j.hypothesis) {
      const h = c.hyps.get(j.hypothesis.name) ?? c.hyps.set(j.hypothesis.name, { name: j.hypothesis.name, arms: new Map() }).get(j.hypothesis.name);
      const cell = h.arms.get(j.hypothesis.arm) ?? h.arms.set(j.hypothesis.arm, {}).get(j.hypothesis.arm);
      (cell[j.hypothesis.stage] ??= []).push(j);
    }
  }
  return [...out.values()].sort((a, b) => (b.hyps.size > 0) - (a.hyps.size > 0) || b.last - a.last);
}
const stageOf = (h) => STAGES[Math.max(0, ...[...h.arms.values()].flatMap((cell) => Object.entries(cell).filter(([, js]) => js.some((j) => ['done', 'running', 'queued'].includes(j.state))).map(([st]) => STAGES.indexOf(st))))] || 'fast';
function metricLine(j) {
  const m = j.metrics || {}, k = Object.keys(m)[0];
  return k ? `${k} ${m[k].mean}` : '';
}
function viewCampaigns() {
  const cs = campaigns();
  const PR = { running: 0, queued: 1, done: 2, failed: 3, dead: 3, cancelled: 4 };
  const fleetLinks = (c) => [...c.studies].map((s) => `<a class="rp" href="#/fleet/${encodeURIComponent(s)}">ghost fleet: ${esc(s)}</a>`).join('');
  return [['lede', `<p class="lede sub">${plural(cs.length, 'campaign')}. <span class="d">A hypothesis rides the line from the fast tier to Unity to the real boat.</span></p>`],
    ...cs.map((c) => {
      const hd = `<div class="hd"><h3>${esc(c.name)}</h3><span>${[...c.subs].map(esc).join(', ')}, ${plural(c.jobs.length, 'job').toLowerCase()}, last ${ago(new Date(c.last).toISOString())}</span>${fleetLinks(c)}</div>`;
      if (!c.hyps.size) return [`c-${c.name}`, `<div class="camp">${hd}<div class="list flatlist">${c.jobs.map((j) => `<a class="item" href="#/job/${esc(j.id)}"><div class="st ${esc(j.state)}">●</div><div class="p">${esc(j.purpose)}</div><div class="st ${esc(j.state)} r">${esc(j.state)}</div></a>`).join('')}</div></div>`];
      const hy = [...c.hyps.values()].map((h) => {
        const rows = [...h.arms].map(([arm, cell]) => {
          let reach = -1; STAGES.forEach((s, i) => { if ((cell[s] || []).length) reach = i; });
          return `<div class="arm">${esc(arm)}</div>` + STAGES.map((s, i) => {
            const j = [...(cell[s] || [])].sort((a, b) => PR[a.state] - PR[b.state])[0];
            const cls = ['seg', i === 0 && 'first', i === 2 && 'last', i < reach && 'reached', i === reach && 'reached edge'].filter(Boolean).join(' ');
            const cap = j ? (j.state === 'done' ? metricLine(j) || 'done' : j.state === 'running' ? `${Math.round((j.progress.fraction || 0) * 100)} %` : j.state) : '';
            return `<div class="${cls}">${j ? `<a class="stn ${esc(j.state)}" href="#/job/${esc(j.id)}" title="${esc(j.purpose)}" aria-label="${esc(arm)} ${s}: ${esc(j.state)}"></a><span class="cap">${esc(cap)}</span>` : ''}</div>`;
          }).join('');
        }).join('');
        return `<div class="hyp"><div class="hn">${esc(h.name)} <span class="stage ${stageOf(h)}">at ${stageOf(h)}</span></div>
          <div class="metro"><div></div>${STAGES.map((s) => `<div class="sh">${s}</div>`).join('')}${rows}</div></div>`;
      }).join('');
      return [`c-${c.name}`, `<div class="camp">${hd}${hy}</div>`];
    })];
}

// ---------------------------------------------------------------- workers
function viewWorkers() {
  const J = byId();
  const ws = FEED.workers.map((w) => {
    const j = w.job && J[w.job], n = FEED.runs.filter((r) => r.worker === w.alias).length;
    return `<div class="wk"><div><h3><i class="${w.live ? '' : 'off'}"></i>${esc(w.alias)}</h3><div class="bk">${w.backends.map(esc).join('<br>')}</div></div>
      <div>${j ? `<a href="#/job/${esc(j.id)}" class="p">${esc(j.purpose)}</a><div class="prog cont"><i style="width:${(j.progress.fraction || 0) * 100}%"></i></div>` : `<span class="mute">${w.live ? 'Idle.' : 'Offline.'}</span>`}</div>
      <div class="ws">${w.live ? 'live' : 'offline'}, seen ${ago(w.last_seen)}<br>${j ? `${live(j.eta?.done_at)} left on this job<br>` : ''}${n} runs in the feed</div></div>`;
  }).join('');
  const bk = Object.entries(FEED.backends).sort((a, b) => b[1].runs_last_hour - a[1].runs_last_hour).map(([n, b]) =>
    `<tr><td>${esc(n)}</td><td class="r">${dur(b.median_run_s)}</td><td class="r">${b.ratio && b.n >= 5 ? `${b.ratio.toFixed(2)}×` : '<span class="mute">learning</span>'}</td><td class="r">${b.n}</td><td class="r">${b.runs_last_hour}</td></tr>`).join('');
  return [['lede', `<p class="lede sub">${plural(FEED.summary.workers_live, 'worker')} live. <span class="d">Estimates learn each backend’s real pace.</span></p>`],
    ['ws', `<section>${ws}</section>`],
    ['model', `<section><h2>Duration model</h2><div class="tw"><table class="tbl"><thead><tr><th>Backend</th><th class="r">Median run</th><th class="r">Actual ÷ agent</th><th class="r">Samples</th><th class="r">Runs last hour</th></tr></thead><tbody>${bk}</tbody></table></div></section>`]];
}

// ---------------------------------------------------------------- job page
function jobSide(j, runs) {
  const est = j.estimate, ok = runs.filter((r) => r.wall_s != null);
  const act = ok.length ? ok.reduce((a, r) => a + r.wall_s, 0) / ok.length : null;
  const mx = Math.max(est.agent_run_s || 0, est.model_run_s || 0, act || 0, 0.001);
  const bar = (lbl, v, c) => (v == null ? '' : `<div class="row"><span>${lbl}</span><div><i style="width:${(v / mx) * 100}%;background:${c}"></i></div><span class="r">${dur(v)}</span></div>`);
  const si = j.hypothesis ? STAGES.indexOf(j.hypothesis.stage) : -1;
  const ladder = j.hypothesis ? `<div class="stage-big">${STAGES.map((s, i) => `<span class="${i === si ? 'on' : i < si ? 'past' : ''}">${s}</span>`).join('')}</div>` : '';
  const when = j.state === 'running' ? `${live(j.eta?.done_at)} left, done ${clock(j.eta?.done_at)}` : j.state === 'queued' ? (j.eta?.start_at ? `starts ${clock(j.eta.start_at)}` : esc(j.eta?.note || 'waiting')) : `${esc(j.state)} ${ago(j.ended_at)}`;
  const facts = [
    ['State', `<span class="st ${esc(j.state)}">${when}</span>`], ['Submitted by', esc(j.submitter || '–')], ['Campaign', esc(j.campaign || '–')],
    ['Backend', esc(j.backend)], ['Worker', esc(j.worker || runs[0]?.worker || '–')],
    ['Runs', `${j.progress.runs_ok}${j.progress.runs_failed ? ` + <span class="bad">${j.progress.runs_failed} failed</span>` : ''} of ${j.runs_total}`],
    ['Job estimate', dur(est.job_s)], ['Attempt', esc(j.attempt)],
    ...Object.entries(j.metrics || {}).map(([k, v]) => [esc(k), `${esc(v.mean)} <span class="mute">n=${esc(v.n)}</span>`]),
  ];
  return `${ladder}<div class="ebar"><h2>Time per run</h2>${bar('agent', est.agent_run_s, 'var(--warn)')}${bar(`model n=${est.model_n}`, est.model_run_s, 'var(--fg2)')}${bar('actual', act, 'var(--accent)')}</div>
    <div class="facts">${facts.map(([k, v]) => `<div><span>${k}</span><b>${v}</b></div>`).join('')}</div><div id="renders"></div>`;
}
function seedRow(runs, sel) {
  if (!runs.some((r) => r.replay)) return '';
  return `<div class="seedrow">${runs.map((r) => `<a href="#/run/${esc(r.run_id)}" class="${failed(r) ? 'f' : ''}${r.run_id === sel ? ' sel' : ''}${r.replay ? '' : ' nr'}" title="seed ${r.seed}: ${esc(r.outcome || r.status)}${r.score != null ? `, score ${r.score}` : ''}${r.replay ? '' : ', no replay'}">${esc(r.seed)}</a>`).join('')}</div>`;
}
const pickRun = (runs, rid) => rid || (runs.find((r) => r.replay && failed(r)) || runs.find((r) => r.replay) || {}).run_id;
function viewJob(data, rid) {
  const { job: j, runs } = data, sel = pickRun(runs, rid);
  return [['job', `<a class="back" href="#/${lastView}">Back to ${lastView}</a>
  <div class="jp"><div class="main">
    <p class="purpose">${esc(j.purpose)}</p><div class="m">${who(j)}</div>
    ${j.judge ? `<div class="verdict ok">${esc(j.judge)}</div>` : ''}${j.error ? `<div class="verdict bad">${esc(j.error)}</div>` : ''}
    <div id="th"></div><div id="seeds">${seedRow(runs, sel)}</div><div id="evs"></div>
  </div><aside id="jside">${jobSide(j, runs)}</aside></div>`]];
}

// trajectories: decoded once per URL
const trajCache = new Map();
function loadTrack(run, lod) {
  const h = lod ? '' : run.traj_sha256 || run.lod_sha256 || '';
  const url = `/v1/feed/runs/${encodeURIComponent(run.run_id)}/traj?lod=${lod ? 1 : 0}${h ? `&h=${h}` : ''}`;
  if (!trajCache.has(url)) trajCache.set(url, fetch(url).then(async (r) => {
    if (!r.ok) throw Object.assign(new Error(String(r.status)), { status: r.status });
    return track(await decode(await r.arrayBuffer()));
  }));
  return trajCache.get(url);
}
const courseCache = new Map();
function loadCourse(sha) {
  if (!sha) return Promise.resolve(null);
  if (!courseCache.has(sha)) courseCache.set(sha, getJSON(`/v1/feed/courses/${sha}`).catch(() => null));
  return courseCache.get(sha);
}
const evText = (e) => [e.kind, e.task, e.index != null ? `#${e.index}` : '', e.id, e.cause, e.reason, e.text].filter((x) => x !== undefined && x !== null && x !== '').map(String).join(' ');
const BAD = /fail|contact|hit|collision|timeout/i;

function theaterShell(caption) {
  return `<div class="theater"><canvas id="cv"></canvas><div class="tcap">${caption}</div>
    <div class="tctl"><button class="pp" id="pp" aria-label="Play">▶</button><input id="sc" type="range" min="0" max="1000" value="0" aria-label="Scrub"><span id="tt" class="tt"></span>
    <span class="k">${[1, 4, 8, 16].map((k) => `<button data-k="${k}" aria-pressed="${k === 8}">${k}×</button>`).join('')}</span></div></div>`;
}
function wireTheater(host, opts) {
  theater = mount($('#cv', host), {
    speed: 8, ...opts,
    onTime: (t) => { if (!theater) return; $('#sc', host).value = Math.round((t / theater.duration) * 1000); $('#tt', host).textContent = `${t.toFixed(0)} / ${theater.duration.toFixed(0)} s`; },
    onState: (p) => { $('#pp', host).textContent = p ? '❚❚' : '▶'; $('#pp', host).setAttribute('aria-label', p ? 'Pause' : 'Play'); },
  });
  $('#pp', host).onclick = () => theater.toggle();
  $('#sc', host).oninput = (e) => theater.seek((e.target.value / 1000) * theater.duration);
  host.querySelectorAll('.k button').forEach((b) => (b.onclick = () => { theater.setSpeed(+b.dataset.k); host.querySelectorAll('.k button').forEach((x) => x.setAttribute('aria-pressed', x === b)); }));
}

async function mountTheater(data, rid) {
  const host = $('#th'); if (!host) return;
  const { job: j, runs } = data, sel = pickRun(runs, rid), run = runs.find((r) => r.run_id === sel);
  const nore = (msg) => { host.innerHTML = `<div class="nore">${msg}</div>`; };
  if (!run || !run.replay) {
    if (runs.some((r) => r.replay_state === 'gone')) return nore('This job’s trajectories are no longer kept.');
    return nore(j.state === 'queued' || j.state === 'running' ? 'Replays appear here once runs finish and their trajectories are archived.' : 'This job kept no trajectories, so there is nothing to replay.');
  }
  host.innerHTML = '<div class="nore">Loading the replay…</div>';
  const key = routeKey;
  try {
    const tr = await loadTrack(run, false);
    const [course, gh] = await Promise.all([loadCourse(tr.header.course?.truth_sha256 || run.course), getJSON(`/v1/feed/ghosts?run=${encodeURIComponent(run.run_id)}&limit=6`).catch(() => ({ runs: [] }))]);
    const ghosts = (await Promise.all(gh.runs.map((g) => loadTrack(g, true).then((t) => ({ track: t, label: g.arm || `iter ${g.iteration}`, color: '#C792EA' }), () => null)))).filter(Boolean);
    if (key !== routeKey) return;
    const out = run.outcome && run.outcome !== 'unknown' ? run.outcome : run.status;
    host.innerHTML = theaterShell(`<b>Seed ${esc(run.seed)}</b>, <span class="${failed(run) ? 'bad' : 'ok'}">${esc(out)}</span>${run.score != null ? `, score ${esc(run.score)}` : ''}<br>${ghosts.length ? `ghosts: ${ghosts.map((g) => esc(g.label)).join(', ')} on the same seed` : ''}${course ? '' : '<br>course not stored'}`);
    const marks = tr.events.filter((e) => BAD.test(e.kind)).map((e) => ({ t: e.t, bad: true }));
    wireTheater(host, { track: tr, course, ghosts, marks, theme: { vmax: Math.max(2, ...Array.from(tr.v).slice(0, 5000)) } });
    $('#evs').innerHTML = tr.events.length ? `<div class="evs">${tr.events.slice(0, 200).map((e) => `<button data-t="${e.t}" class="${BAD.test(e.kind) ? 'bad' : ''}"><span>${Number(e.t).toFixed(1)} s</span>${esc(evText(e))}</button>`).join('')}</div>` : '';
    $('#evs').querySelectorAll('button').forEach((d) => (d.onclick = () => theater.seek(+d.dataset.t)));
    const first = tr.events.find((e) => BAD.test(e.kind));
    theater.seek(first ? Math.max(0, first.t - 3) : theater.duration * 0.4);
  } catch (e) {
    if (key === routeKey) nore(e.status === 503 ? 'On the archive node, which is offline right now; try again later.' : e.status === 410 ? 'This trajectory is no longer kept.' : 'This replay could not be read.');
  }
}

// renders: requests and videos; the button needs an operator token (kept for this tab only)
const opToken = () => { try { return sessionStorage.getItem(OP_KEY); } catch { return null; } };
let opName = null;
async function checkOperator() {
  const t = opToken(); opName = null;
  if (t) try { opName = (await (await fetch('/v1/operator/whoami', { headers: { authorization: `Bearer ${t}` } })).json()).name || null; } catch {}
  $('#op').textContent = opName ? `operator: ${opName} (sign out)` : 'operator sign-in';
}
async function mountRenders(j) {
  const host = $('#renders'); if (!host) return;
  const study = j.replay.study;
  const [a, b] = await Promise.all([getJSON(`/v1/feed/renders?job=${encodeURIComponent(j.id)}`), study ? getJSON(`/v1/feed/renders?study=${encodeURIComponent(study)}`) : { renders: [] }].map((p) => Promise.resolve(p).catch(() => ({ renders: [] }))));
  const all = [...new Map([...a.renders, ...b.renders].map((r) => [r.id, r])).values()];
  const vids = all.filter((r) => r.video), pend = all.filter((r) => !r.video && ['requested', 'rendering'].includes(r.status));
  const can = !!opName && j.replay.available;
  host.innerHTML = `<h2>Theater</h2>${vids.map((r) => `<figure class="vid"><video controls preload="none" src="${esc(r.video.url)}"></video><figcaption>${esc(r.purpose)} <a href="${esc(r.video.url)}">open</a></figcaption></figure>`).join('')}
    ${pend.map((r) => `<p class="pend"><span class="st running">${esc(r.status)}</span> ${esc(r.purpose)} <span class="mute">${ago(r.created_at)}, ${esc(r.requested_by)}</span></p>`).join('')}
    <button class="btn" id="rq" ${can ? '' : 'disabled'} title="${opName ? (j.replay.available ? '' : 'This job has no trajectories to render') : 'Needs an operator token; the page is read-only'}">Render in Unity theater</button>
    ${opName ? '' : '<p class="hint">Read-only. An operator token enables rendering.</p>'}<form id="rqf" hidden><label for="rqp">Why render it (shown publicly)</label><textarea id="rqp" rows="3" minlength="20" maxlength="600" required></textarea><button class="btn">Request render</button><span id="rqm" class="mute"></span></form>`;
  $('#rq').onclick = () => { $('#rqf').hidden = false; $('#rqp').focus(); };
  $('#rqf').onsubmit = async (e) => {
    e.preventDefault();
    const r = await fetch('/v1/operator/renders', { method: 'POST', headers: { authorization: `Bearer ${opToken()}`, 'content-type': 'application/json' }, body: JSON.stringify({ job_id: j.id, study, purpose: $('#rqp').value }) });
    const body = await r.json().catch(() => ({}));
    $('#rqm').textContent = r.ok ? `Requested (${body.status}).` : body.error || `failed (${r.status})`;
    if (r.ok) setTimeout(() => mountRenders(j), 600);
  };
}

// ---------------------------------------------------------------- fleet
function viewFleet(study) {
  return [['fleet', `<a class="back" href="#/campaigns">Back to campaigns</a><p class="lede sub" id="flede">Ghost fleet of <em>${esc(study)}</em>. <span class="d">Loading…</span></p><div id="th"></div><div id="fleg"></div>`]];
}
async function mountFleet(study) {
  const host = $('#th'), key = routeKey, q = `study=${encodeURIComponent(study)}&k=5&limit=150`;
  try {
    const idx = await getJSON(`/v1/feed/fleet?${q}`);
    if (!idx.runs.length) { $('#flede').innerHTML = `Ghost fleet of <em>${esc(study)}</em>. <span class="d">No stored trajectories in this study.</span>`; return; }
    const buf = new Uint8Array(await (await fetch(`/v1/feed/fleet.pack?${q}&h=${idx.pack_sha256}`)).arrayBuffer());
    const v = new DataView(buf.buffer), files = [];
    for (let p = 0; p + 4 <= buf.length;) { const n = v.getUint32(p, true); files.push(buf.subarray(p + 4, p + 4 + n)); p += 4 + n; }
    const its = [...new Set(idx.runs.map((r) => r.iteration))].sort((a, b) => a - b);
    const color = (it) => ramp(its.length > 1 ? its.indexOf(it) / (its.length - 1) : 1);
    const ghosts = (await Promise.all(files.map((f, i) => (f.length ? decode(f).then((d) => ({ track: track(d), color: color(idx.runs[i].iteration) }), () => null) : null)))).filter(Boolean);
    if (key !== routeKey || !ghosts.length) return;
    // One course under the fleet only when every run shares it (a study on many courses draws paths alone).
    const courses = new Set(idx.runs.map((r) => r.course));
    const course = courses.size === 1 ? await loadCourse([...courses][0]) : null;
    $('#flede').innerHTML = `Ghost fleet of <em>${esc(study)}</em>. <span class="d">${ghosts.length} runs from ${plural(its.length, 'iteration').toLowerCase()}: per iteration the best five and every failure${course ? '' : ', on their own courses'}.</span>`;
    host.innerHTML = theaterShell(`<b>${ghosts.length} ghosts</b><br>coloured by iteration`);
    wireTheater(host, { fleet: true, ghosts, course, tail: 8 });
    theater.seek(theater.duration * 0.3);
    $('#fleg').innerHTML = `<div class="fleg"><span>iteration ${its[0]}</span><i style="background:linear-gradient(90deg,${[0, 0.35, 0.68, 1].map(ramp).join(',')})"></i><span>${its[its.length - 1]}</span></div>`;
  } catch {
    if (key === routeKey) host.innerHTML = '<div class="nore">This fleet could not be loaded.</div>';
  }
}

// ---------------------------------------------------------------- render + router
const VIEWS = { now: viewNow, runs: viewRuns, campaigns: viewCampaigns, workers: viewWorkers };
const HTML = new WeakMap();
function paint(sections) {
  const view = $('#view');
  const old = new Map([...view.children].map((el) => [el.dataset.k, el]));
  const next = sections.map(([k, html]) => {
    const prev = old.get(k);
    if (prev && HTML.get(prev) === html) return prev;
    const el = document.createElement('div'); el.dataset.k = k; HTML.set(el, html); el.innerHTML = html; return el;
  });
  if (next.length !== view.children.length || next.some((el, i) => view.children[i] !== el)) view.replaceChildren(...next);
}
function parseRoute() {
  const [, v, ...rest] = location.hash.split('/');
  return { v: v || 'now', id: rest.length ? decodeURIComponent(rest.join('/')) : null };
}
async function render(navigated) {
  const { v, id } = parseRoute();
  const key = `${v}/${id}`;
  if (navigated || key !== routeKey) { theater && theater.destroy(); theater = null; routeKey = key; jobCache = null; }
  const navOn = VIEWS[v] ? v : v === 'fleet' ? 'campaigns' : lastView;
  document.querySelectorAll('#nav a').forEach((a) => (a.getAttribute('href') === `#/${navOn}` ? a.setAttribute('aria-current', 'page') : a.removeAttribute('aria-current')));
  if (v === 'job' || v === 'run') {
    const jid = v === 'run' ? jobOfRun(id) : id;
    try {
      const data = await getJSON(`/v1/feed/jobs/${encodeURIComponent(jid)}`);
      if (key !== routeKey) return;
      const sig = JSON.stringify(data.job) + JSON.stringify(data.runs);
      if (jobCache && jobCache.sig === sig) return;
      const first = !jobCache; jobCache = { sig, data };
      if (first) {
        paint(viewJob(data, v === 'run' ? id : null));
        document.title = `${data.job.purpose.slice(0, 60)} · crane`;
        mountTheater(data, v === 'run' ? id : null); mountRenders(data.job);
      } else {
        $('#jside').innerHTML = jobSide(data.job, data.runs); mountRenders(data.job);
        $('#seeds').innerHTML = seedRow(data.runs, pickRun(data.runs, v === 'run' ? id : null));
      }
    } catch (e) {
      if (key === routeKey) paint([['job', `<a class="back" href="#/${lastView}">Back to ${lastView}</a><p class="lede sub">${e.status === 404 ? 'That job is not in the feed.' : 'The job could not be loaded.'}</p>`]]);
    }
    return;
  }
  if (v === 'fleet') {
    if (navigated || !$('#flede')) { paint(viewFleet(id)); document.title = 'Ghost fleet · crane'; mountFleet(id); }
    return;
  }
  if (!FEED) return paint([['lede', '<p class="lede sub mute">Connecting to the feed…</p>']]);
  const cur = VIEWS[v] ? v : 'now';
  lastView = cur;
  document.title = 'crane runs';
  paint(VIEWS[cur]());
  if (navigated) window.scrollTo(0, 0);
}
document.addEventListener('keydown', (e) => {
  if (e.target.closest('input,textarea,select')) return;
  if (e.key === ' ' && theater) { e.preventDefault(); theater.toggle(); }
  if (e.key === 'Escape' && /^#\/(job|run|fleet)/.test(location.hash)) location.hash = `#/${lastView}`;
});
$('#op').onclick = async (e) => {
  e.preventDefault();
  if (opName) { try { sessionStorage.removeItem(OP_KEY); } catch {} }
  else {
    const t = prompt('Operator token (kept in this tab only):');
    if (t) try { sessionStorage.setItem(OP_KEY, t.trim()); } catch {}
  }
  await checkOperator();
  if (jobCache) mountRenders(jobCache.data.job);
};
addEventListener('hashchange', () => render(true));
checkOperator();
connect();
render(true);
