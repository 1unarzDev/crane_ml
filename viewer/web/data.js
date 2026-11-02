// data.js: where runs come from. One page, two sources with the same feed routes (replay.ts):
//   local  `python -m viewer serve` adds /v1/viewer/index (runs, studies, tier pairs) and judge reports
//   feed   the dashboard at crane.lunarz.dev: runs, courses, ghosts and fleets only
// A run becomes {id, header, tier, arm, seed, study, track{t,x,y,yaw,v}, events, t0, zero, channels}, with
// every time on the run's clock from its task_started (or its first pose).
import { decode, fleetUnpack, agents } from './ctraj.js';

const enc = encodeURIComponent;
async function get(url, kind = 'json') {
  const r = await fetch(url);
  if (!r.ok) { const e = new Error(`${url}: HTTP ${r.status}`); e.status = r.status; throw e; }
  return kind === 'json' ? r.json() : new Uint8Array(await r.arrayBuffer());
}

const LANES = { decision: 'decision', goal: 'goal', behaviour: 'behaviour', contact: 'judge', near_miss: 'judge', strike: 'judge', row: 'judge', failure: 'judge' };
const TASKISH = /^(task_|gate_|stage_|docked|undocked|returned_home|debris|species)/;

function smoothSpeed(t, x, y) {
  const v = new Float64Array(t.length);
  for (let i = 0; i < t.length; i++) {
    const a = Math.max(0, i - 2), b = Math.min(t.length - 1, i + 2), dt = t[b] - t[a] || 1e-9;
    v[i] = Math.hypot(x[b] - x[a], y[b] - y[a]) / dt;
  }
  return v;
}

/** A decoded ctraj -> the viewer's run model. `brief` (feed row) fills what a public header leaves out. */
export function model(id, d, brief = {}) {
  const h = d.header || {};
  const ag = agents(d)[0] || 'robot';
  const X = d.columns[`${ag}.x`] || new Float64Array(0), Y = d.columns[`${ag}.y`] || new Float64Array(0);
  const PSI = d.columns[`${ag}.yaw`] || new Float64Array(X.length);
  const raw = (d.events || []).filter((e) => Number.isFinite(e.t));
  const zero = raw.find((e) => e.kind === 'task_started')?.t ?? 0;
  const T = new Float64Array(X.length);
  for (let i = 0; i < T.length; i++) T[i] = (d.firstTick + i) / d.hz - zero;
  const events = raw.map((e) => ({ ...e, t: e.t - zero, lane: e.lane || LANES[e.kind] || (TASKISH.test(e.kind) ? 'task' : 'other'),
    text: e.text ?? e.reason ?? e.why ?? e.cause ?? e.id ?? e.kind, decision: e.decision ?? (e.kind === 'decision' ? e.type : undefined) }))
    .sort((a, b) => a.t - b.t);
  const ch = { ...(h.channels || {}) };
  ch.pose ??= X.length ? 'present' : 'absent';
  ch.decisions ??= events.some((e) => e.lane === 'decision') ? 'present' : 'empty';
  ch.goals ??= events.some((e) => e.lane === 'goal') ? 'present' : 'empty';
  return {
    id, header: h, brief, tier: h.tier || brief.tier || null, arm: h.arm ?? brief.arm ?? null, study: h.study ?? brief.study ?? null,
    seed: h.seed ?? brief.seed ?? null, task: h.task ?? brief.task ?? null, iteration: h.iteration ?? brief.iteration ?? 0,
    score: h.score ?? brief.score ?? null, outcome: h.outcome ?? brief.outcome ?? 'unknown', course: h.course?.truth_sha256 ?? brief.course ?? null,
    pair: h.pair ?? brief.pair ?? null, referee: h.referee || null, fidelity: h.fidelity || null, flags: h.flags || brief.flags || [],
    lod: !!h.lod, t0: Number(h.t0_s ?? d.t0 ?? 0), zero, hz: d.hz,
    track: { t: T, x: X, y: Y, yaw: PSI, v: smoothSpeed(T, X, Y) }, events, channels: ch,
  };
}

export class Source {
  constructor() { this.mode = 'feed'; this.idx = null; this.cache = new Map(); this.reports = null; }
  async init() {
    try {
      const r = await fetch('/v1/viewer/index');
      if (r.ok && (r.headers.get('content-type') || '').includes('json')) { this.idx = await r.json(); this.mode = 'local'; }
    } catch { /* the dashboard: feed only */ }
    return this;
  }
  briefOf(id) { return this.idx?.runs.find((r) => r.run_id === id) || null; }
  memo(key, fn) { if (!this.cache.has(key)) this.cache.set(key, fn().catch((e) => { this.cache.delete(key); throw e; })); return this.cache.get(key); }
  run(id, { lod = false, brief = null } = {}) {
    return this.memo(`run:${id}:${lod}`, async () => model(id, await decode(await get(`/v1/feed/runs/${enc(id)}/traj?lod=${lod ? 1 : 0}`, 'bin')), brief || this.briefOf(id) || {}));
  }
  course(sha) { return sha ? this.memo(`course:${sha}`, () => get(`/v1/feed/courses/${enc(sha)}`)).catch(() => null) : Promise.resolve(null); }
  ghosts(id) { return this.memo(`ghosts:${id}`, () => get(`/v1/feed/ghosts?run=${enc(id)}&limit=16`)).catch(() => ({ runs: [] })); }
  fleet(study) {
    return this.memo(`fleet:${study}`, async () => {
      const idx = await get(`/v1/feed/fleet?study=${enc(study)}&k=100&limit=300`);
      const files = fleetUnpack(await get(`/v1/feed/fleet.pack?study=${enc(study)}&k=100&limit=300&h=${idx.pack_sha256}`, 'bin'));
      const runs = (await Promise.all(files.map((f, i) => (f.length ? decode(f).then((d) => model(idx.runs[i].run_id, d, idx.runs[i]), () => null) : null)))).filter(Boolean);
      return { ...idx, models: runs };
    });
  }
  /** The judge's evidence about these runs (local): [{id, verdict, report: {insights, pairs, g4, split}, feed}]. */
  judge(ids) {
    ids = ids.filter(Boolean);
    if (this.mode !== 'local' || !this.idx?.judge?.length || !ids.length) return Promise.resolve([]);
    return this.memo(`judge:${ids.join(',')}`, () => get(`/v1/viewer/judge?runs=${ids.map(enc).join(',')}`)).catch(() => []);
  }
  /** The run's twin on another tier: its pair (local), else a ghost of another arm or tier. */
  async twin(run) {
    if (run.pair?.high && run.pair.high !== run.id) return run.pair.high;
    const p = this.idx?.pairs?.find((x) => x.high === run.id);
    if (p) return p.low;
    const g = await this.ghosts(run.id);
    const other = g.runs.find((r) => r.replay !== 'gone' && ((r.tier && r.tier !== run.tier) || (r.arm || '') !== (run.arm || '')));
    return other?.run_id || g.runs[0]?.run_id || null;
  }
}

/** Judge anchors and insights touching a run: [{t0, t1, label, kind, claim, severity, report}] on the run's clock. */
export function judgeFor(run, reports, other = null) {
  const out = [];
  if (!run) return out;
  for (const r of reports || []) {
    const ins = Object.fromEntries((r.report.insights || []).map((i) => [i.id, i]));
    for (const [iid, anchors] of Object.entries(r.feed || {})) {
      for (const a of anchors) {
        if (a[0] !== run.id) continue;
        const base = run.t0 + run.zero, i = ins[iid], sp = i?.scope?.pair;
        if (other && Array.isArray(sp) && !sp.includes(other)) continue; // another pair's evidence about a shared run
        out.push({ id: iid, report: r.id, t0: a[1] - base, t1: a[2] - base, channels: a[3], label: a[4] || iid.split(':')[0],
          kind: iid.split(':')[0], claim: i?.claim || null, severity: i?.severity || null });
      }
    }
  }
  return out.sort((a, b) => a.t0 - b.t0);
}

/** The judge's per-pair record for (low, high) and the report it came from. */
export function judgePair(low, high, reports) {
  for (const r of reports || []) {
    const p = (r.report.pairs || []).find((x) => (x.low === low && x.high === high) || (x.low === high && x.high === low));
    if (p) return { report: r, pair: p };
  }
  return null;
}
