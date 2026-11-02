// The public, read-only run feed: GET /v1/feed (JSON), GET /v1/feed/jobs/:id (one job and its runs) and
// GET /v1/feed/stream (SSE). No token: everything here is sanitized (no tokens, no specs, no repo URLs or
// paths, worker aliases instead of ids, free text through the secret scrubber). Estimates: the submitter's
// est_seconds per run, a per-backend duration model (medians over recent runs) that corrects it, per-worker
// speed from history, and ETAs from a list-scheduling pass over the queue in lease order.
import { Hono } from "hono";
import type { AppEnv } from "./auth";
import { Caps, Needs, Policy, matches } from "./match";
import { alias, scrubText } from "./purpose";
import { hubStub } from "./hubstub";
import { Env, HttpError, intVar, nowIso, parseJson } from "./util";

export const feedRoutes = new Hono<AppEnv>();

export const FEED_SCHEMA = "crane-feed/1";
const LIVE_MS = 10 * 60_000; // a worker seen within this long counts as live
const MODEL_SAMPLE = 2000; // recent ok runs the duration model reads
const MODEL_MIN_N = 5; // samples before the model overrides the submitter
const RECENT_DEFAULT = 30;
const ACTIVE_LIMIT = 500;
const FALLBACK_RUN_S = 60; // a legacy job with no estimate and no model
const SPEED_CLAMP: [number, number] = [0.25, 4];
const METRICS_PER_JOB = 6;

type Row = Record<string, unknown>;

const median = (xs: number[]): number | null => {
  if (!xs.length) return null;
  const s = [...xs].sort((a, b) => a - b);
  const m = s.length >> 1;
  return s.length % 2 ? s[m] : (s[m - 1] + s[m]) / 2;
};
/** Rounds to d decimals; without d, to 3 below 1 (sub-second fast-sim runs), 2 below 10, else 1. */
const round = (x: number | null, d?: number) => {
  if (x === null || !Number.isFinite(x)) return null;
  const k = d ?? (Math.abs(x) < 1 ? 3 : Math.abs(x) < 10 ? 2 : 1);
  return Math.round(x * 10 ** k) / 10 ** k;
};
const clamp = (x: number, [lo, hi]: [number, number]) => Math.min(hi, Math.max(lo, x));

export interface BackendModel {
  median_run_s: number | null; // median wall time per run, recent ok runs
  ratio: number | null; // median(actual / submitter's estimate), runs with an estimate
  n: number; // runs behind median_run_s
  n_ratio: number;
  runs_last_hour: number;
}

/** Per-backend duration model and per-(worker, backend) speed factors from recent ok runs. */
export function durationModel(rows: { backend: string; worker_id: string; wall_s: number; est_seconds: number | null; created: string }[], nowMs: number) {
  const by = new Map<string, { walls: number[]; ratios: number[]; hour: number }>();
  for (const r of rows) {
    const b = by.get(r.backend) ?? { walls: [], ratios: [], hour: 0 };
    b.walls.push(r.wall_s);
    if (r.est_seconds && r.est_seconds > 0) b.ratios.push(r.wall_s / r.est_seconds);
    if (nowMs - Date.parse(r.created) <= 3600_000) b.hour++;
    by.set(r.backend, b);
  }
  const backends: Record<string, BackendModel> = {};
  for (const [k, b] of by)
    backends[k] = { median_run_s: median(b.walls), ratio: median(b.ratios), n: b.walls.length, n_ratio: b.ratios.length, runs_last_hour: b.hour };
  const perWorker = new Map<string, number[]>();
  for (const r of rows) {
    const m = backends[r.backend]?.median_run_s;
    if (!m) continue;
    const key = `${r.worker_id}\u0000${r.backend}`;
    (perWorker.get(key) ?? perWorker.set(key, []).get(key)!).push(r.wall_s / m);
  }
  const speed = new Map<string, number>();
  for (const [k, xs] of perWorker) if (xs.length >= 3) speed.set(k, clamp(median(xs)!, SPEED_CLAMP));
  return { backends, speed };
}

/** A run's wall time for the model: a batched backend (all seeds in prepare, mhseals_testing's
 * fastsim_roboboat) reports ~0 s per seed, so a finished attempt's runs take at least their share of
 * the job's leased time (lease to end over the attempt's runs in the sample). */
export function runWalls<R extends { job_id: string; run_attempt: number; attempt: number; leased_at_ms: number | null; ended_at: string | null; wall_s: number }>(rows: R[]): R[] {
  const n = new Map<string, number>();
  for (const r of rows) if (r.run_attempt === r.attempt) n.set(r.job_id, (n.get(r.job_id) ?? 0) + 1);
  return rows.map((r) => {
    const end = r.ended_at ? Date.parse(r.ended_at) : NaN;
    if (r.run_attempt !== r.attempt || !r.leased_at_ms || !Number.isFinite(end) || end <= r.leased_at_ms) return r;
    return { ...r, wall_s: Math.max(r.wall_s, (end - r.leased_at_ms) / 1000 / n.get(r.job_id)!) };
  });
}

type ModelRow = Parameters<typeof durationModel>[0][number] & Parameters<typeof runWalls>[0][number];

/** The model's seconds per run for a job: the estimate times the backend's ratio, else the backend median. */
export function modelRunSeconds(est: number | null, m: BackendModel | undefined): number | null {
  if (m && m.n_ratio >= MODEL_MIN_N && m.ratio !== null && est) return est * m.ratio;
  if (m && m.n >= MODEL_MIN_N && m.median_run_s !== null) return m.median_run_s;
  return null;
}

interface SchedWorker {
  id: string;
  caps: Caps;
  policy: Policy;
  free_at: number;
}

export interface FeedJob extends Row {
  job_id: string;
  purpose: string;
  submitter: string | null;
  campaign: string | null;
  arm: string | null;
  backend: string;
  status: string;
  needs_json: string;
  seeds_json: string;
  spec_json: string;
  est_seconds: number | null;
  leased_at_ms: number | null;
  lease_worker: string | null;
  attempt: number;
}

const STATE: Record<string, string> = { leased: "running", done: "done", dead: "dead", failed: "failed", cancelled: "cancelled", queued: "queued" };

/**
 * Where the feed's inputs come from. Without one, everything is read from D1 (the colo cache memoizes the
 * slow parts). The Hub passes its memory: the active jobs, the workers and the live jobs' run aggregates,
 * plus a memo that refreshes the D1-backed history (ended jobs, replay, insight, recent runs, the duration
 * model) only after something changed and no more often than its interval.
 */
export interface FeedSource {
  active: FeedJob[];
  workers: Row[];
  live: { counts: Row[]; metrics: Row[] };
  memo: <T>(key: string, kind: "history" | "model" | "ended", fn: () => Promise<T>) => Promise<T>;
}

const JOB_COLS = `j.job_id, j.hypothesis_id, j.stage_index, j.arm, j.backend, j.seeds_json, j.needs_json, j.spec_json,
      j.priority, j.status, j.attempt, j.lease_worker, j.leased_at_ms, j.last_error, j.created, j.ended_at, j.purpose,
      j.est_seconds, j.submitter, j.campaign, j.judge, h.name AS hyp_name`;

/** Feed order of active jobs: lease order (priority, then age). */
export const byLeaseOrder = (a: Row, b: Row) =>
  Number(a.priority) - Number(b.priority) || String(a.created).localeCompare(String(b.created));

/** Run counts and metric means of these jobs' current attempts. */
async function aggregate(env: Env, jobIds: string) {
  const [c, m] = await env.DB.batch([
    env.DB.prepare(
      `SELECT r.job_id, r.status, COUNT(*) AS n FROM runs r JOIN jobs j ON j.job_id = r.job_id AND r.attempt = j.attempt
        WHERE r.job_id IN (SELECT value FROM json_each(?1)) GROUP BY r.job_id, r.status`,
    ).bind(jobIds),
    env.DB.prepare(
      `SELECT r.job_id, m.name, AVG(m.value) AS mean, COUNT(*) AS n FROM runs r
         JOIN jobs j ON j.job_id = r.job_id AND r.attempt = j.attempt JOIN metrics m ON m.run_id = r.run_id
        WHERE r.job_id IN (SELECT value FROM json_each(?1)) GROUP BY r.job_id, m.name`,
    ).bind(jobIds),
  ]);
  return { counts: c.results as Row[], metrics: m.results as Row[] };
}

const RUN_COLS = `r.run_id, r.job_id, r.worker_id, r.attempt, r.seed, r.status, r.wall_s, r.error, r.created,
                  m.score, m.outcome, m.study, m.iteration, (a.key IS NOT NULL AND a.deleted_at IS NULL) AS replay,
                  CASE WHEN a.key IS NULL THEN NULL WHEN a.deleted_at IS NOT NULL THEN 'gone' WHEN a.r2_deleted_at IS NOT NULL THEN 'archived' ELSE 'ready' END AS replay_state,
                  a.sha256 AS traj_sha256, m.course_sha256`;

/** The D1-backed history of a snapshot: ended jobs, replay and insight rows for the listed jobs, recent runs. */
async function loadHistory(env: Env, recent: number, jobId: string | undefined, activeIds: string[]) {
  const [ended] = await env.DB.batch([
    jobId
      ? env.DB.prepare(`SELECT ${JOB_COLS} FROM jobs j LEFT JOIN hypotheses h ON h.id = j.hypothesis_id WHERE j.job_id = ?1`).bind(jobId)
      : env.DB.prepare(
          `SELECT ${JOB_COLS} FROM jobs j LEFT JOIN hypotheses h ON h.id = j.hypothesis_id
            WHERE j.ended_at IS NOT NULL ORDER BY j.ended_at DESC LIMIT ?1`,
        ).bind(recent),
  ]);
  const endedRows = ended.results as FeedJob[];
  const ids = JSON.stringify([...new Set([...activeIds, ...endedRows.map((j) => j.job_id)])]);
  const [replay, insight, runs] = await env.DB.batch([
    env.DB.prepare(
      `SELECT m.job_id, SUM(a.deleted_at IS NULL) AS n, SUM(a.deleted_at IS NULL AND a.r2_deleted_at IS NOT NULL) AS archived, MIN(m.study) AS study
         FROM run_meta m JOIN artifacts a ON a.key = m.traj_key
        WHERE m.job_id IN (SELECT value FROM json_each(?1)) GROUP BY m.job_id`,
    ).bind(ids),
    env.DB.prepare(
      `SELECT job_id, insight FROM run_meta WHERE insight IS NOT NULL AND job_id IN (SELECT value FROM json_each(?1))
        ORDER BY created DESC`,
    ).bind(ids),
    jobId
      ? env.DB.prepare(
          `SELECT ${RUN_COLS} FROM runs r LEFT JOIN run_meta m ON m.run_id = r.run_id LEFT JOIN artifacts a ON a.key = m.traj_key
            WHERE r.job_id = ?1 ORDER BY r.attempt, r.seed`,
        ).bind(jobId)
      : env.DB.prepare(
          `SELECT ${RUN_COLS} FROM runs r LEFT JOIN run_meta m ON m.run_id = r.run_id LEFT JOIN artifacts a ON a.key = m.traj_key
            ORDER BY r.created DESC LIMIT ?1`,
        ).bind(recent),
  ]);
  return { ended: endedRows, replay: replay.results as Row[], insight: insight.results as Row[], runs: runs.results as Row[] };
}

const MODEL_SQL = `SELECT j.backend, r.worker_id, r.wall_s, j.est_seconds, r.created, r.job_id, r.attempt AS run_attempt, j.attempt,
          j.leased_at_ms, j.ended_at FROM runs r JOIN jobs j ON j.job_id = r.job_id
        WHERE r.status = 'ok' AND r.wall_s IS NOT NULL ORDER BY r.created DESC LIMIT ${MODEL_SAMPLE}`;

/** Builds the sanitized feed snapshot (from D1, or from the Hub's memory with `src`). */
export async function buildFeed(env: Env, opts: { recent?: number; jobId?: string; origin?: string } = {}, nowMs = Date.now(), src?: FeedSource) {
  const recent = opts.recent ?? RECENT_DEFAULT;
  let activeJobs: FeedJob[];
  let workerRows: Row[];
  if (src) {
    activeJobs = [...src.active].sort(byLeaseOrder).slice(0, ACTIVE_LIMIT);
    workerRows = src.workers;
  } else {
    const [active, workers] = await env.DB.batch([
      env.DB.prepare(
        `SELECT ${JOB_COLS} FROM jobs j LEFT JOIN hypotheses h ON h.id = j.hypothesis_id
          WHERE j.status IN ('queued','leased') ORDER BY j.priority, j.created LIMIT ${ACTIVE_LIMIT}`,
      ),
      env.DB.prepare("SELECT id, name, state, last_seen, disabled, caps_json, policy_json FROM workers"),
    ]);
    activeJobs = active.results as FeedJob[];
    workerRows = workers.results as Row[];
  }
  const activeIds = activeJobs.map((j) => j.job_id);
  const hist = src
    ? await src.memo(`hist:${recent}:${opts.jobId ?? ""}`, "history", () => loadHistory(env, recent, opts.jobId, activeIds))
    : await loadHistory(env, recent, opts.jobId, activeIds);
  // The duration model reads MODEL_SAMPLE runs (~2 rows each): refreshed every MODEL_CACHE_S, not every snapshot.
  const loadModel = async () => (await env.DB.prepare(MODEL_SQL).all()).results as ModelRow[];
  const modelRows = src ? await src.memo("model", "model", loadModel) : await memo(env, opts.origin, "model", MODEL_CACHE_S, loadModel);
  const endedJobs = hist.ended.filter((j) => !activeJobs.some((a) => a.job_id === j.job_id));
  const jobs = opts.jobId ? [...activeJobs.filter((j) => j.job_id === opts.jobId), ...endedJobs] : [...activeJobs, ...endedJobs];
  // Run counts and metric means scan every run and metric of every listed job (the feed's largest read, which
  // exhausted D1's daily row budget on 11-02). An ended job's runs no longer change, so its aggregates are
  // cached per (job, attempt, ended_at) set; live jobs' come from the Hub's memory (or D1 without one).
  const isLive = (j: FeedJob) => j.status === "queued" || j.status === "leased";
  const endedJobs_ = jobs.filter((j) => !isLive(j));
  const endedKey = "ended:" + (await sha256(endedJobs_.map((j) => `${j.job_id}:${j.attempt}:${j.ended_at}`).join(",")));
  const endedIds = JSON.stringify(endedJobs_.map((j) => j.job_id));
  const [liveAgg, endedAgg] = await Promise.all([
    src ? src.live : aggregate(env, JSON.stringify(jobs.filter(isLive).map((j) => j.job_id))),
    !endedJobs_.length
      ? { counts: [], metrics: [] }
      : src
        ? src.memo(endedKey, "ended", () => aggregate(env, endedIds))
        : memo(env, opts.origin, endedKey, ENDED_CACHE_S, () => aggregate(env, endedIds)),
  ]);
  const listed = new Set(jobs.map((j) => j.job_id));
  const counts = { results: [...liveAgg.counts, ...endedAgg.counts].filter((r) => listed.has(r.job_id as string)) };
  const metrics = { results: [...liveAgg.metrics, ...endedAgg.metrics].filter((r) => listed.has(r.job_id as string)) };
  const replay = { results: hist.replay };
  const insight = { results: hist.insight };
  const recentRuns = { results: hist.runs };
  // ---- model, workers
  const { backends, speed } = durationModel(runWalls(modelRows), nowMs);
  const aliasOf = new Map(workerRows.map((w) => [w.id as string, alias(w.name as string)]));
  const live = (w: Row) => !w.disabled && !!w.last_seen && nowMs - Date.parse(w.last_seen as string) <= LIVE_MS;

  const runCounts = new Map<string, { ok: number; failed: number }>();
  for (const r of counts.results as Row[]) {
    const c = runCounts.get(r.job_id as string) ?? { ok: 0, failed: 0 };
    c[r.status === "failed" ? "failed" : "ok"] += r.n as number;
    runCounts.set(r.job_id as string, c);
  }
  const metricRows = new Map<string, Row[]>();
  for (const m of metrics.results as Row[]) (metricRows.get(m.job_id as string) ?? metricRows.set(m.job_id as string, []).get(m.job_id as string)!).push(m);
  const replayOf = new Map((replay.results as Row[]).map((r) => [r.job_id as string, r]));
  const insightOf = new Map<string, string>();
  for (const r of insight.results as Row[]) if (!insightOf.has(r.job_id as string)) insightOf.set(r.job_id as string, r.insight as string);

  // ---- per-job estimates
  const runS = (j: FeedJob) => {
    const m = modelRunSeconds(j.est_seconds, backends[j.backend]);
    return { agent: j.est_seconds, model: m, used: m ?? j.est_seconds ?? backends[j.backend]?.median_run_s ?? null };
  };
  const nSeeds = (j: FeedJob) => parseJson<number[]>(j.seeds_json, []).length;
  const speedOf = (wid: string, backend: string) => speed.get(`${wid}\u0000${backend}`) ?? 1;

  // ---- ETAs: running jobs free their worker at their expected end; queued jobs go, in lease order, to the
  // eligible live worker that frees up first.
  const sched: SchedWorker[] = workerRows
    .filter((w) => live(w) && w.state !== "paused")
    .map((w) => ({ id: w.id as string, caps: parseJson<Caps>(w.caps_json as string, {}), policy: parseJson<Policy>(w.policy_json as string, {}), free_at: nowMs }));
  const eta = new Map<string, { start: number | null; end: number | null; position: number | null; note: string | null }>();
  for (const j of activeJobs.filter((x) => x.status === "leased")) {
    const done = runCounts.get(j.job_id);
    const left = Math.max(0, nSeeds(j) - (done ? done.ok + done.failed : 0));
    const per = (runS(j).used ?? FALLBACK_RUN_S) * (j.lease_worker ? speedOf(j.lease_worker, j.backend) : 1) * 1000; // ms per run
    const start = j.leased_at_ms ?? nowMs;
    // Remaining: the unrun seeds, less the time already spent on the one in progress.
    const sinceRun = Math.min(per, Math.max(0, nowMs - start - (nSeeds(j) - left) * per));
    const end = nowMs + Math.max(0, left * per - sinceRun);
    eta.set(j.job_id, { start, end, position: 0, note: null });
    const w = sched.find((s) => s.id === j.lease_worker);
    if (w) w.free_at = Math.max(w.free_at, end);
  }
  let position = 0;
  for (const j of activeJobs.filter((x) => x.status === "queued")) {
    position++;
    const needs = parseJson<Needs>(j.needs_json, {});
    const ok = sched.filter((w) => matches(j.backend, needs, w.caps, w.policy));
    if (!ok.length) {
      eta.set(j.job_id, { start: null, end: null, position, note: `no live worker can run ${j.backend} with these needs` });
      continue;
    }
    const w = ok.reduce((a, b) => (b.free_at < a.free_at ? b : a));
    const dur = nSeeds(j) * (runS(j).used ?? FALLBACK_RUN_S) * speedOf(w.id, j.backend) * 1000;
    eta.set(j.job_id, { start: w.free_at, end: w.free_at + dur, position, note: null });
    w.free_at += dur;
  }

  const view = (j: FeedJob) => {
    const n = nSeeds(j);
    const c = runCounts.get(j.job_id) ?? { ok: 0, failed: 0 };
    const r = runS(j);
    const e = eta.get(j.job_id);
    const spec = parseJson<Record<string, unknown>>(j.spec_json, {});
    const rows = metricRows.get(j.job_id) ?? [];
    const primary = typeof spec.metric === "string" ? spec.metric : null;
    rows.sort((a, b) => Number(b.name === primary) - Number(a.name === primary) || String(a.name).localeCompare(String(b.name)));
    const rep = replayOf.get(j.job_id);
    const state = STATE[j.status] ?? j.status;
    return {
      id: j.job_id,
      state,
      purpose: scrubText(j.purpose),
      submitter: scrubText(j.submitter, 64),
      campaign: scrubText(j.campaign, 100),
      hypothesis: j.hypothesis_id
        ? { id: j.hypothesis_id, name: scrubText(j.hyp_name as string, 120), stage: scrubText(spec.stage as string, 40), arm: scrubText(j.arm, 64) }
        : null,
      backend: j.backend,
      runs_total: n,
      estimate: {
        agent_run_s: round(r.agent),
        model_run_s: round(r.model),
        model_n: backends[j.backend]?.n ?? 0,
        job_s: r.used === null ? null : round(r.used * n, 0),
      },
      queue_position: state === "queued" ? (e?.position ?? null) : null,
      eta: e && (state === "queued" || state === "running")
        ? { start_at: e.start === null ? null : nowIso(e.start), done_at: e.end === null ? null : nowIso(e.end), remaining_s: e.end === null ? null : round((e.end - nowMs) / 1000, 0), note: e.note }
        : null,
      progress: { runs_ok: c.ok, runs_failed: c.failed, fraction: n ? round((c.ok + c.failed) / n, 3) : null },
      worker: state === "running" ? aliasOf.get(j.lease_worker as string) ?? null : null,
      attempt: j.attempt,
      created_at: j.created,
      started_at: j.leased_at_ms ? nowIso(j.leased_at_ms) : null,
      ended_at: j.ended_at ?? null,
      metrics: Object.fromEntries(
        rows.slice(0, METRICS_PER_JOB).map((m) => [m.name, { mean: round(m.mean as number, 4), n: m.n }]),
      ),
      judge: scrubText((j.judge as string | null) ?? insightOf.get(j.job_id) ?? null, 200),
      error: state === "dead" || state === "failed" ? scrubText(j.last_error as string | null, 160) : null,
      replay: {
        available: !!rep && (rep.n as number) > 0,
        runs: (rep?.n as number) ?? 0,
        archived: (rep?.archived as number) ?? 0,
        study: rep ? scrubText(rep.study as string, 100) : null,
      },
    };
  };

  const runView = (r: Row) => ({
    run_id: r.run_id,
    job_id: r.job_id,
    seed: r.seed,
    attempt: r.attempt,
    status: r.status,
    wall_s: round(r.wall_s as number | null),
    worker: aliasOf.get(r.worker_id as string) ?? null,
    at: r.created,
    score: r.score ?? null,
    outcome: r.outcome ?? null,
    study: scrubText(r.study as string | null, 100),
    iteration: r.iteration ?? null,
    replay: !!r.replay,
    replay_state: (r.replay_state as string | null) ?? null,
    traj_sha256: (r.traj_sha256 as string | null) ?? null,
    course: (r.course_sha256 as string | null) ?? null,
    error: scrubText(r.error as string | null, 160),
  });

  const jobViews = jobs.map(view);
  const queued = jobViews.filter((j) => j.state === "queued");
  const lastEnd = Math.max(nowMs, ...[...eta.values()].map((e) => e.end ?? 0));
  const out = {
    schema: FEED_SCHEMA,
    generated_at: nowIso(nowMs),
    summary: {
      queued: queued.length,
      running: jobViews.filter((j) => j.state === "running").length,
      queued_runs: queued.reduce((s, j) => s + j.runs_total, 0),
      drain_s: activeJobs.length ? round((lastEnd - nowMs) / 1000, 0) : 0,
      workers_live: workerRows.filter(live).length,
    },
    workers: workerRows
      .filter((w) => !w.disabled)
      .map((w) => ({
        alias: aliasOf.get(w.id as string),
        live: live(w),
        state: live(w) ? ((w.state as string) ?? "idle") : "offline",
        last_seen: w.last_seen ?? null,
        backends: parseJson<Caps>(w.caps_json as string, {}).backends ?? [],
        job: activeJobs.find((j) => j.status === "leased" && j.lease_worker === w.id)?.job_id ?? null,
      }))
      .sort((a, b) => Number(b.live) - Number(a.live) || String(a.alias).localeCompare(String(b.alias))),
    backends: Object.fromEntries(
      Object.entries(backends).map(([k, m]) => [k, { median_run_s: round(m.median_run_s), ratio: round(m.ratio, 3), n: m.n, runs_last_hour: m.runs_last_hour }]),
    ),
    jobs: jobViews,
    runs: (recentRuns.results as Row[]).map(runView),
  };
  return out;
}

// ------------------------------------------------------------------ routes

const PUBLIC_HEADERS = {
  "access-control-allow-origin": "*",
  "x-content-type-options": "nosniff",
};

const cacheSeconds = (env: Env) => intVar(env.FEED_CACHE_S, 15);
const MODEL_CACHE_S = 300;
const ENDED_CACHE_S = 86_400;

async function sha256(text: string): Promise<string> {
  const d = await crypto.subtle.digest("SHA-256", new TextEncoder().encode(text));
  return [...new Uint8Array(d)].map((b) => b.toString(16).padStart(2, "0")).join("");
}

/** A JSON value through the colo cache under this Worker's origin for ttl s (off without an origin or when FEED_CACHE_S is 0, as in tests). */
async function memo<T>(env: Env, origin: string | undefined, key: string, ttl: number, fn: () => Promise<T>): Promise<T> {
  const cache = origin && cacheSeconds(env) > 0 && typeof caches !== "undefined" ? (caches as unknown as { default: Cache }).default : null;
  const req = new Request(`${origin}/v1/feed/_memo/${encodeURIComponent(key)}`);
  if (cache) {
    const hit = await cache.match(req);
    if (hit) return (await hit.json()) as T;
  }
  const v = await fn();
  if (cache) await cache.put(req, new Response(JSON.stringify(v), { headers: { "cache-control": `public, max-age=${ttl}` } }));
  return v;
}

/** The snapshot through the colo cache (FEED_CACHE_S, default 15 s), so many viewers cost one Hub call; the
 * Hub builds it from memory (src/hub.ts), so a snapshot costs no D1 read unless the history changed. */
async function cachedFeed(env: Env, url: URL, recent: number): Promise<string> {
  const ttl = cacheSeconds(env);
  const key = new Request(`${url.origin}/v1/feed?recent=${recent}&_c=1`);
  const cache = ttl > 0 && typeof caches !== "undefined" ? (caches as unknown as { default: Cache }).default : null;
  if (cache) {
    const hit = await cache.match(key);
    if (hit) return hit.text();
  }
  const body = await hubStub(env).feed(recent);
  if (cache) await cache.put(key, new Response(body, { headers: { "cache-control": `public, max-age=${ttl}` } }));
  return body;
}

function recentParam(v: string | undefined): number {
  if (v === undefined) return RECENT_DEFAULT;
  const n = Number(v);
  if (!Number.isInteger(n) || n < 0 || n > 200) throw new HttpError(400, "recent must be an integer in [0, 200]");
  return n;
}

feedRoutes.get("/v1/feed", async (c) => {
  const body = await cachedFeed(c.env, new URL(c.req.url), recentParam(c.req.query("recent")));
  return new Response(body, {
    headers: { "content-type": "application/json; charset=utf-8", "cache-control": `public, max-age=${cacheSeconds(c.env)}`, ...PUBLIC_HEADERS },
  });
});

feedRoutes.get("/v1/feed/jobs/:id", async (c) => {
  const id = c.req.param("id");
  if (!/^[A-Za-z0-9._-]{1,64}$/.test(id)) throw new HttpError(400, "bad job id");
  const body = await hubStub(c.env).feedJob(id);
  if (body === null) throw new HttpError(404, "unknown job");
  return new Response(body, {
    headers: { "content-type": "application/json; charset=utf-8", "cache-control": "public, max-age=5", ...PUBLIC_HEADERS },
  });
});

/**
 * SSE: an `event: feed` with the whole snapshot at connect and whenever it changes (polled every
 * `interval` s, 2..60, default 5), a comment ping every 15 s otherwise, closed after `max_s` (default 300,
 * at most 900; EventSource reconnects by itself after `retry`). `once=1` sends one event and closes.
 * The stream lives in the Hub, which polls its in-memory snapshot (no D1 read per poll).
 */
feedRoutes.get("/v1/feed/stream", async (c) => {
  const q = (k: string, d: number, lo: number, hi: number) => {
    const v = c.req.query(k);
    const n = v === undefined ? d : Number(v);
    if (!Number.isFinite(n) || n < lo || n > hi) throw new HttpError(400, `${k} must be in [${lo}, ${hi}]`);
    return n;
  };
  const u = new URL(c.req.url);
  const fwd = new URL("https://hub/v1/feed/stream");
  fwd.searchParams.set("interval", String(q("interval", 5, 2, 60)));
  fwd.searchParams.set("max_s", String(q("max_s", 300, 1, 900)));
  fwd.searchParams.set("recent", String(recentParam(c.req.query("recent"))));
  if (u.searchParams.get("once") === "1") fwd.searchParams.set("once", "1");
  return hubStub(c.env).fetch(new Request(fwd, { signal: c.req.raw.signal }));
});
