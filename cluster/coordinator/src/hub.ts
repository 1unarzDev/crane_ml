// The Hub: one Durable Object holding the coordinator's hot state in memory: the queued and leased jobs, the
// workers (tokens, liveness, caps), config and repo versions, the live jobs' run aggregates and the feed
// snapshot. Leasing, heartbeats, auth and the feed are served from memory (no D1 read per poll); D1 stays
// the record: every job transition (lease, finish, release, reclaim) is written through with a conditional
// UPDATE, run reports are inserted, and heartbeat-only state (worker last_seen/state/stats, lease expiry) is
// written in batches by the cron sweep. Routes that change jobs, workers or config in D1 directly (admin,
// hypotheses, join, webhook) tell the Hub to re-read just those rows (sync*). State is loaded from D1 when
// the object starts.
import { DurableObject } from "cloudflare:workers";
import { Book, addTo, bookReads, countingDb, dayKey, hourKey, mergeBook } from "./budget";
import { CFG, JobRow, finishJob, jobView, leaseTtlMs, maxAttempts } from "./db";
import { FEED_SCHEMA, FeedJob, FeedSource, buildFeed, byLeaseOrder } from "./feed";
import { Caps, Needs, Policy, matches, sanitizeCaps, sanitizePolicy } from "./match";
import { runMetaStatement } from "./runmeta";
import { Env, HttpError, compareVersions, intVar, isObject, isStringArray, nowIso, parseJson } from "./util";

export interface HubWorker {
  id: string;
  token_sha256: string;
  name: string;
  caps_json: string;
  policy_json: string;
  client_version: string | null;
  state: string | null;
  stats_json: string | null;
  last_seen: string | null;
  disabled: number;
}
type MemJob = JobRow & Record<string, unknown> & { hyp_name?: string | null };
interface MemRun {
  job_id: string;
  attempt: number;
  status: string;
  metrics: Record<string, number>;
}
/** A route's answer as JSON text, so HttpErrors survive the RPC boundary. */
export interface HubRes {
  status: number;
  body: string;
}
const res = (body: unknown, status = 200): HubRes => ({ status, body: JSON.stringify(body) });
interface Memo {
  val: unknown;
  at: number;
  hv: number;
}

const STATES = new Set(["idle", "running", "paused", "updating", "stopping"]);
const METRIC_NAME = /^[A-Za-z0-9._:/-]{1,128}$/;
const LONG_POLL_MAX_S = 25; // a lease call's wait_s cap (clients time out at 60 s)
const LIVE_MS = 10 * 60_000;
const SNAP_FRESH_MS = 30_000; // an unchanged snapshot is rebuilt this often (clocks, ETAs, liveness)
const HISTORY_MAX_S = 600; // the D1-backed history is re-read at least this often while anything changed
const MODEL_MIN_S = 900; // the duration model (2000 runs) is re-read at most this often
const MODEL_MAX_S = 6 * 3600;
const BOOK_HOURS = 49;
const DAY_MS = 86_400_000;



export class Hub extends DurableObject<Env> {
  jobs = new Map<string, MemJob>(); // queued and leased
  claiming = new Set<string>(); // picked by a lease call, D1 claim in flight
  reclaiming = new Set<string>();
  workers = new Map<string, HubWorker>();
  byToken = new Map<string, string>();
  config: Record<string, string> = {};
  repos: Record<string, string | null> = {};
  runs = new Map<string, Map<string, MemRun>>(); // job -> run id -> run (live jobs only)
  version = 0; // bumped on any change the feed shows
  histVersion = 0; // bumped when D1-backed history changed (runs, ended jobs, verdicts)
  dirtyWorkers = new Set<string>();
  dirtyLeases = new Set<string>();
  waiters = new Set<() => void>();
  snaps = new Map<string, { v: number; at: number; body: string }>();
  memos = new Map<string, Memo>();
  inflight = new Map<string, Promise<unknown>>();
  book: Book = {};
  alerted: string | null = null; // the UTC day the guard was last announced
  loaded = false;
  loading: Promise<void> | null = null;
  bookLoaded = false;

  // ------------------------------------------------------------ plumbing

  /** D1 through the read counter, labelled hub:<op>. */
  private db(op: string): D1Database {
    return countingDb(this.env.DB, () => `hub:${op}`, (label, meta) => addTo(this.book, hourKey(), label, meta?.rows_read ?? 0, meta?.rows_written ?? 0));
  }
  private envFor(op: string): Env {
    return { ...this.env, DB: this.db(op) };
  }

  private async ready() {
    if (!this.bookLoaded) {
      this.book = { ...((await this.ctx.storage.get<Book>("book")) ?? {}), ...this.book };
      this.alerted = (await this.ctx.storage.get<string>("alerted")) ?? null;
      this.bookLoaded = true;
    }
    if (this.loaded) return;
    this.loading ??= this.load(true).finally(() => (this.loading = null));
    await this.loading;
  }

  private async guard(fn: () => Promise<HubRes>): Promise<HubRes> {
    try {
      return await fn();
    } catch (e) {
      if (e instanceof HttpError) return res({ error: e.message, ...e.extra }, e.status);
      throw e;
    }
  }

  private bump(history = false) {
    this.version++;
    if (history) this.histVersion++;
  }

  private wake() {
    const ws = [...this.waiters];
    this.waiters.clear();
    for (const w of ws) w();
  }

  private authed(tokSha: string): HubWorker {
    const id = this.byToken.get(tokSha);
    const w = id ? this.workers.get(id) : undefined;
    if (!w || w.disabled) throw new HttpError(401, "unauthorized");
    return w;
  }

  private setWorker(w: HubWorker) {
    const old = this.workers.get(w.id);
    if (old && old.token_sha256 !== w.token_sha256) this.byToken.delete(old.token_sha256);
    this.workers.set(w.id, w);
    this.byToken.set(w.token_sha256, w.id);
  }

  private setJob(row: MemJob, cold = false) {
    const mem = this.jobs.get(row.job_id);
    if (row.status === "leased") {
      // D1's lease_expires lags (heartbeats are written in batches): keep the later of the two, and after a
      // restart give every lease one TTL so live holders can heartbeat before anything is reclaimed.
      let exp = row.lease_expires ?? 0;
      if (mem && mem.status === "leased" && mem.lease_worker === row.lease_worker && mem.attempt === row.attempt)
        exp = Math.max(exp, mem.lease_expires ?? 0);
      if (cold) exp = Math.max(exp, Date.now() + leaseTtlMs(this.env));
      row.lease_expires = exp;
    }
    this.jobs.set(row.job_id, row);
  }

  private dropJob(id: string) {
    this.jobs.delete(id);
    this.runs.delete(id);
    this.dirtyLeases.delete(id);
  }

  // ------------------------------------------------------------ loading and sync

  /** (Re)loads the hot state from D1. `cold`: a fresh start, where leases get a grace TTL. */
  async load(cold = false) {
    const db = this.db("load");
    const [jobs, workers, cfg, repos] = await db.batch([
      db.prepare(
        `SELECT j.*, h.name AS hyp_name FROM jobs j LEFT JOIN hypotheses h ON h.id = j.hypothesis_id
          WHERE j.status IN ('queued','leased')`,
      ),
      db.prepare("SELECT id, token_sha256, name, caps_json, policy_json, client_version, state, stats_json, last_seen, disabled FROM workers"),
      db.prepare("SELECT key, value FROM config"),
      db.prepare("SELECT name, main_version FROM repos"),
    ]);
    this.jobs.clear();
    this.runs.clear();
    for (const r of jobs.results as MemJob[]) this.setJob(r, cold);
    this.workers.clear();
    this.byToken.clear();
    for (const w of workers.results as unknown as HubWorker[]) this.setWorker(w);
    this.config = Object.fromEntries((cfg.results as { key: string; value: string }[]).map((r) => [r.key, r.value]));
    this.repos = Object.fromEntries((repos.results as { name: string; main_version: string | null }[]).map((r) => [r.name, r.main_version]));
    await this.loadRuns([...this.jobs.keys()]);
    this.dirtyLeases.clear();
    this.dirtyWorkers.clear();
    this.snaps.clear();
    this.memos.clear();
    this.loaded = true;
    this.bump(true);
    this.wake();
  }

  /** The current attempt's runs and metrics of these (live) jobs, into memory. */
  private async loadRuns(ids: string[]) {
    if (!ids.length) return;
    const db = this.db("load");
    const [runs, metrics] = await db.batch([
      db.prepare("SELECT run_id, job_id, attempt, status FROM runs WHERE job_id IN (SELECT value FROM json_each(?1))").bind(JSON.stringify(ids)),
      db.prepare(
        `SELECT m.run_id, r.job_id, m.name, m.value FROM metrics m JOIN runs r ON r.run_id = m.run_id
          WHERE r.job_id IN (SELECT value FROM json_each(?1))`,
      ).bind(JSON.stringify(ids)),
    ]);
    for (const id of ids) this.runs.delete(id);
    for (const r of runs.results as { run_id: string; job_id: string; attempt: number; status: string }[]) {
      const m = this.runs.get(r.job_id) ?? this.runs.set(r.job_id, new Map()).get(r.job_id)!;
      m.set(r.run_id, { job_id: r.job_id, attempt: r.attempt, status: r.status, metrics: {} });
    }
    for (const r of metrics.results as { run_id: string; job_id: string; name: string; value: number }[]) {
      const run = this.runs.get(r.job_id)?.get(r.run_id);
      if (run) run.metrics[r.name] = r.value;
    }
  }

  /** Re-reads these jobs from D1 (after an admin submit, cancel or requeue). */
  async syncJobs(ids: string[]) {
    await this.ready();
    if (!ids.length) return;
    const db = this.db("sync");
    const { results } = await db
      .prepare(
        `SELECT j.*, h.name AS hyp_name FROM jobs j LEFT JOIN hypotheses h ON h.id = j.hypothesis_id
          WHERE j.job_id IN (SELECT value FROM json_each(?1))`,
      )
      .bind(JSON.stringify(ids))
      .all<MemJob>();
    this.applyRows(ids, results);
  }

  /** Re-reads a hypothesis's active jobs (after it was created, promoted, extended or rejected). */
  async syncHypothesis(hid: string) {
    await this.ready();
    const db = this.db("sync");
    const { results } = await db
      .prepare(
        `SELECT j.*, h.name AS hyp_name FROM jobs j LEFT JOIN hypotheses h ON h.id = j.hypothesis_id
          WHERE j.hypothesis_id = ?1 AND j.status IN ('queued','leased')`,
      )
      .bind(hid)
      .all<MemJob>();
    const mine = [...this.jobs.values()].filter((j) => j.hypothesis_id === hid).map((j) => j.job_id);
    this.applyRows([...new Set([...mine, ...results.map((r) => r.job_id)])], results);
  }

  private applyRows(ids: string[], rows: MemJob[]) {
    const got = new Map(rows.map((r) => [r.job_id, r]));
    const added: string[] = [];
    for (const id of ids) {
      if (this.claiming.has(id) || this.reclaiming.has(id)) continue; // that call writes the outcome itself
      const r = got.get(id);
      if (r && (r.status === "queued" || r.status === "leased")) {
        if (!this.jobs.has(id)) added.push(id);
        this.setJob(r);
      } else this.dropJob(id);
    }
    this.bump(true);
    this.wake();
    return added;
  }

  async syncWorkers(ids: string[]) {
    await this.ready();
    const db = this.db("sync");
    const { results } = await db
      .prepare(
        `SELECT id, token_sha256, name, caps_json, policy_json, client_version, state, stats_json, last_seen, disabled FROM workers
          WHERE id IN (SELECT value FROM json_each(?1))`,
      )
      .bind(JSON.stringify(ids))
      .all<HubWorker>();
    for (const w of results) {
      // Keep the liveness this object saw since the last flush.
      const mem = this.workers.get(w.id);
      if (mem && (mem.last_seen ?? "") > (w.last_seen ?? "")) Object.assign(w, { last_seen: mem.last_seen, state: mem.state, stats_json: mem.stats_json });
      this.setWorker(w);
    }
    this.bump();
  }

  /** Re-reads config and repo versions (client release gate, retention, webhook pushes). */
  async syncConfig() {
    await this.ready();
    const db = this.db("sync");
    const [cfg, repos] = await db.batch([db.prepare("SELECT key, value FROM config"), db.prepare("SELECT name, main_version FROM repos")]);
    this.config = Object.fromEntries((cfg.results as { key: string; value: string }[]).map((r) => [r.key, r.value]));
    this.repos = Object.fromEntries((repos.results as { name: string; main_version: string | null }[]).map((r) => [r.name, r.main_version]));
  }

  /** Something the feed's history shows changed in D1 (a verdict, an artifact). */
  async touchHistory() {
    this.bump(true);
  }

  /** Writes pending state, then reloads everything from D1 (POST /v1/admin/hub/reload). */
  async reload() {
    await this.ready();
    await this.flush();
    await this.load(false);
    return this.state();
  }

  state() {
    let queued = 0;
    let leased = 0;
    for (const j of this.jobs.values()) j.status === "leased" ? leased++ : queued++;
    return { queued, leased, workers: this.workers.size, dirty_workers: this.dirtyWorkers.size, dirty_leases: this.dirtyLeases.size, waiters: this.waiters.size, version: this.version };
  }

  // ------------------------------------------------------------ worker API

  async auth(tokSha: string): Promise<{ id: string; name: string; caps_json: string; policy_json: string; client_version: string | null } | null> {
    await this.ready();
    const id = this.byToken.get(tokSha);
    const w = id ? this.workers.get(id) : undefined;
    if (!w || w.disabled) return null;
    return { id: w.id, name: w.name, caps_json: w.caps_json, policy_json: w.policy_json, client_version: w.client_version };
  }

  async reposFor(tokSha: string): Promise<HubRes> {
    return this.guard(async () => {
      await this.ready();
      this.authed(tokSha);
      return res({ repos: this.repos });
    });
  }

  /** POST /v1/lease. `wait_s` (0..25, optional) holds an empty answer until a job arrives or the wait ends. */
  async lease(tokSha: string, b: Record<string, unknown>, headerVersion: string | null): Promise<HubRes> {
    return this.guard(async () => {
      await this.ready();
      const w = this.authed(tokSha);
      const reported = headerVersion ?? (typeof b.client_version === "string" ? b.client_version : null);
      const version = reported ?? w.client_version ?? "0.0.0";
      const minV = this.config[CFG.minClientVersion] ?? this.env.MIN_CLIENT_VERSION ?? "0.0.0";
      if (compareVersions(version, minV) < 0)
        throw new HttpError(426, "client too old", { min_client_version: minV, latest_client_version: this.config[CFG.latestClientVersion] ?? null });
      const maxJobs = b.max_jobs === undefined ? 1 : Number(b.max_jobs);
      if (!Number.isInteger(maxJobs) || maxJobs < 0) throw new HttpError(400, "max_jobs must be a non-negative integer");
      const waitS = typeof b.wait_s === "number" && Number.isFinite(b.wait_s) ? Math.min(LONG_POLL_MAX_S, Math.max(0, b.wait_s)) : 0;
      const caps = b.caps !== undefined ? sanitizeCaps(b.caps) : sanitizeCaps(parseJson(w.caps_json, {}));
      const policy = b.policy !== undefined ? sanitizePolicy(b.policy) : sanitizePolicy(parseJson(w.policy_json, {}));
      const state = typeof b.state === "string" && STATES.has(b.state) ? b.state : "idle";
      const capsJson = JSON.stringify(caps);
      const policyJson = JSON.stringify(policy);
      if (capsJson !== w.caps_json || policyJson !== w.policy_json || (reported && reported !== w.client_version)) this.bump();
      Object.assign(w, { caps_json: capsJson, policy_json: policyJson, client_version: reported ?? w.client_version });
      const deadline = Date.now() + waitS * 1000;
      let jobs: JobRow[] = [];
      for (;;) {
        await this.reclaim(Date.now());
        jobs = maxJobs > 0 ? await this.claim(w.id, caps, policy, Math.min(maxJobs, intVar(this.env.MAX_LEASE_JOBS, 4))) : [];
        if (jobs.length || maxJobs === 0 || Date.now() >= deadline) break;
        this.touchWorker(w, state);
        await this.waitForWork(deadline - Date.now());
      }
      this.touchWorker(w, jobs.length ? "running" : state);
      return res({ jobs: jobs.map(jobView), repos: this.repos, lease_ttl_s: leaseTtlMs(this.env) / 1000 });
    });
  }

  private touchWorker(w: HubWorker, state: string | null, stats: string | null = null) {
    const wasLive = !!w.last_seen && Date.now() - Date.parse(w.last_seen) <= LIVE_MS;
    if (!wasLive || (state && state !== w.state)) this.bump();
    w.last_seen = nowIso();
    if (state) w.state = state;
    if (stats) w.stats_json = stats;
    this.dirtyWorkers.add(w.id);
  }

  private waitForWork(ms: number): Promise<void> {
    return new Promise((resolve) => {
      const done = () => {
        clearTimeout(t);
        this.waiters.delete(done);
        resolve();
      };
      const t = setTimeout(done, Math.max(0, ms));
      this.waiters.add(done);
    });
  }

  /** Claims up to max matching queued jobs: picked in memory, confirmed by a conditional UPDATE in D1. */
  private async claim(workerId: string, caps: Caps, policy: Policy, max: number): Promise<JobRow[]> {
    const backends = new Set(caps.backends ?? []);
    const picks: MemJob[] = [];
    for (const j of [...this.jobs.values()].filter((x) => x.status === "queued" && !this.claiming.has(x.job_id) && backends.has(x.backend)).sort(byLeaseOrder)) {
      if (picks.length >= max) break;
      if (matches(j.backend, parseJson<Needs>(j.needs_json, {}), caps, policy)) picks.push(j);
    }
    if (!picks.length) return [];
    for (const p of picks) this.claiming.add(p.job_id);
    const now = Date.now();
    const db = this.db("lease");
    let res: D1Result[];
    try {
      res = await db.batch(
        picks.map((p) =>
          db
            .prepare(
              `UPDATE jobs SET status = 'leased', lease_worker = ?2, lease_expires = ?3, leased_at_ms = ?4, attempt = attempt + 1
                WHERE job_id = ?1 AND status = 'queued' RETURNING *`,
            )
            .bind(p.job_id, workerId, now + leaseTtlMs(this.env), now),
        ),
      );
    } finally {
      for (const p of picks) this.claiming.delete(p.job_id);
    }
    const out: JobRow[] = [];
    const stale: string[] = [];
    picks.forEach((p, i) => {
      const row = (res[i].results as MemJob[])[0];
      if (row) {
        this.jobs.set(p.job_id, { ...row, hyp_name: p.hyp_name });
        this.runs.get(p.job_id)?.clear(); // a new attempt
        out.push(row);
      } else stale.push(p.job_id); // changed in D1 behind our back (cancelled): re-read it
    });
    if (stale.length) await this.syncJobs(stale);
    if (out.length) this.bump();
    return out;
  }

  async heartbeat(tokSha: string, b: Record<string, unknown>): Promise<HubRes> {
    return this.guard(async () => {
      await this.ready();
      const w = this.authed(tokSha);
      const ids = b.job_ids ?? [];
      if (!isStringArray(ids)) throw new HttpError(400, "job_ids must be an array of strings");
      const now = Date.now();
      const state = typeof b.state === "string" && STATES.has(b.state) ? b.state : null;
      const stats = isObject(b.stats) ? JSON.stringify(b.stats).slice(0, 4000) : null;
      const extended: string[] = [];
      for (const id of ids) {
        const j = this.jobs.get(id);
        if (!j || j.status !== "leased" || j.lease_worker !== w.id || this.reclaiming.has(id)) continue;
        // A heartbeat guarantees at least one TTL from now, without unbounded growth.
        j.lease_expires = Math.max(j.lease_expires ?? 0, now + leaseTtlMs(this.env));
        this.dirtyLeases.add(id);
        extended.push(id);
      }
      this.touchWorker(w, state, stats);
      // Revoked: the lease expired and was reclaimed, or an admin cancelled the job. The worker must stop it.
      return res({ extended, revoked: ids.filter((id) => !extended.includes(id)) });
    });
  }

  async release(tokSha: string, b: Record<string, unknown>): Promise<HubRes> {
    return this.guard(async () => {
      await this.ready();
      const w = this.authed(tokSha);
      const ids = b.job_ids ?? [];
      if (!isStringArray(ids)) throw new HttpError(400, "job_ids must be an array of strings");
      const db = this.db("release");
      // A voluntary release (shutdown, user became active) does not count as a failed attempt.
      const [, rel] = await db.batch([
        db.prepare(
          `UPDATE leases SET outcome = 'released' WHERE worker_id = ?1 AND outcome IS NULL
              AND job_id IN (SELECT value FROM json_each(?2))`,
        ).bind(w.id, JSON.stringify(ids)),
        db.prepare(
          `UPDATE jobs SET status = 'queued', lease_worker = NULL, lease_expires = NULL, attempt = MAX(attempt - 1, 0)
            WHERE status = 'leased' AND lease_worker = ?1 AND job_id IN (SELECT value FROM json_each(?2))
            RETURNING job_id`,
        ).bind(w.id, JSON.stringify(ids)),
      ]);
      const released = (rel.results as { job_id: string }[]).map((r) => r.job_id);
      for (const id of released) {
        const j = this.jobs.get(id);
        if (j) Object.assign(j, { status: "queued", lease_worker: null, lease_expires: null, attempt: Math.max(j.attempt - 1, 0) });
        this.dirtyLeases.delete(id);
      }
      if (released.length) {
        this.bump();
        this.wake();
      }
      return res({ released });
    });
  }

  private holder(tokSha: string, jobId: string): { w: HubWorker; job: MemJob } {
    const w = this.authed(tokSha);
    const job = this.jobs.get(jobId);
    if (!job || job.status !== "leased" || job.lease_worker !== w.id || this.reclaiming.has(jobId)) throw new HttpError(409, "not the lease holder");
    return { w, job };
  }

  /** The job if this token's worker holds its lease (artifact uploads). */
  async held(tokSha: string, jobId: string): Promise<HubRes> {
    return this.guard(async () => {
      await this.ready();
      const { w, job } = this.holder(tokSha, jobId);
      return res({ worker_id: w.id, job });
    });
  }

  async reportRun(tokSha: string, jobId: string, b: Record<string, unknown>): Promise<HubRes> {
    return this.guard(async () => {
      await this.ready();
      const { w, job } = this.holder(tokSha, jobId);
      const seed = b.seed;
      if (!Number.isSafeInteger(seed) || !parseJson<number[]>(job.seeds_json, []).includes(seed as number))
        throw new HttpError(400, "seed must be one of the job's seeds");
      const status = b.status === "failed" ? "failed" : "ok";
      const metrics = b.metrics ?? {};
      if (!isObject(metrics)) throw new HttpError(400, "metrics must be an object of numbers");
      const pairs = Object.entries(metrics);
      if (pairs.length > 200 || !pairs.every(([k, v]) => METRIC_NAME.test(k) && typeof v === "number" && Number.isFinite(v)))
        throw new HttpError(400, "metrics must map names to finite numbers (at most 200)");
      const runId = `${job.job_id}.a${job.attempt}.s${seed}`;
      const wall = typeof b.wall_s === "number" && Number.isFinite(b.wall_s) ? b.wall_s : null;
      const error = typeof b.error === "string" ? b.error.slice(0, 2000) : null;
      const env = this.envFor("runs");
      const meta = runMetaStatement(env, job, runId, seed as number, b.meta);
      await env.DB.batch([
        env.DB.prepare("DELETE FROM metrics WHERE run_id = ?").bind(runId),
        env.DB.prepare(
          `INSERT INTO runs(run_id, job_id, worker_id, attempt, seed, status, wall_s, error, created)
           VALUES (?1, ?2, ?3, ?4, ?5, ?6, ?7, ?8, ?9)
           ON CONFLICT(run_id) DO UPDATE SET status = excluded.status, wall_s = excluded.wall_s, error = excluded.error`,
        ).bind(runId, job.job_id, w.id, job.attempt, seed, status, wall, error, nowIso()),
        env.DB.prepare("INSERT INTO metrics(run_id, name, value) SELECT ?1, key, value FROM json_each(?2)").bind(runId, JSON.stringify(metrics)),
        ...(meta ? [meta] : []),
      ]);
      const m = this.runs.get(job.job_id) ?? this.runs.set(job.job_id, new Map()).get(job.job_id)!;
      m.set(runId, { job_id: job.job_id, attempt: job.attempt, status, metrics: metrics as Record<string, number> });
      this.bump(true);
      return res({ run_id: runId });
    });
  }

  async complete(tokSha: string, jobId: string, b: Record<string, unknown>): Promise<HubRes> {
    return this.guard(async () => {
      await this.ready();
      const { w, job } = this.holder(tokSha, jobId);
      const ok = b.status === "ok";
      if (!ok && b.status !== "failed") throw new HttpError(400, "status must be ok or failed");
      const error = typeof b.error === "string" ? b.error : null;
      const status = await finishJob(this.envFor("complete"), job, w.id, ok, error);
      if (status === "queued") {
        Object.assign(job, { status: "queued", lease_worker: null, lease_expires: null, last_error: error ? error.slice(0, 2000) : null });
        this.dirtyLeases.delete(jobId);
        this.wake();
      } else this.dropJob(jobId);
      this.bump(true);
      return res({ job_id: jobId, status });
    });
  }

  /** Expired leases go back to the queue, or to 'dead' once the job used up its attempts. */
  async reclaim(nowMs = Date.now()): Promise<{ requeued: string[]; dead: string[] }> {
    const out = { requeued: [] as string[], dead: [] as string[] };
    const expired = [...this.jobs.values()].filter(
      (j) => j.status === "leased" && (j.lease_expires ?? 0) < nowMs && !this.reclaiming.has(j.job_id) && !this.claiming.has(j.job_id),
    );
    if (!expired.length) return out;
    for (const j of expired) this.reclaiming.add(j.job_id);
    const db = this.db("reclaim");
    const stmts: D1PreparedStatement[] = [];
    for (const j of expired)
      stmts.push(
        db.prepare("UPDATE leases SET outcome = 'expired' WHERE job_id = ?1 AND attempt = ?2 AND outcome IS NULL").bind(j.job_id, j.attempt),
        db.prepare(
          `UPDATE jobs
              SET status = CASE WHEN attempt >= COALESCE(max_attempts, ?2) THEN 'dead' ELSE 'queued' END,
                  last_error = 'lease expired (worker ' || lease_worker || ')',
                  ended_at = CASE WHEN attempt >= COALESCE(max_attempts, ?2) THEN ?3 ELSE ended_at END,
                  lease_worker = NULL, lease_expires = NULL
            WHERE job_id = ?1 AND status = 'leased'
            RETURNING job_id, status, last_error`,
        ).bind(j.job_id, maxAttempts(this.env), nowIso(nowMs)),
      );
    let res: D1Result[];
    try {
      res = await db.batch(stmts);
    } finally {
      for (const j of expired) this.reclaiming.delete(j.job_id);
    }
    expired.forEach((j, i) => {
      const row = (res[2 * i + 1].results as { job_id: string; status: string; last_error: string }[])[0];
      this.dirtyLeases.delete(j.job_id);
      if (row?.status === "queued") {
        Object.assign(j, { status: "queued", lease_worker: null, lease_expires: null, last_error: row.last_error });
        out.requeued.push(j.job_id);
      } else {
        this.dropJob(j.job_id);
        if (row) out.dead.push(j.job_id);
      }
    });
    this.bump(true);
    this.wake();
    return out;
  }

  /** Writes the batched heartbeat state (worker liveness, lease expiry) to D1, and the read book to storage. */
  async flush() {
    const db = this.db("flush");
    const stmts: D1PreparedStatement[] = [];
    const workers = [...this.dirtyWorkers];
    const leases = [...this.dirtyLeases];
    for (const id of workers) {
      const w = this.workers.get(id);
      if (!w) continue;
      stmts.push(
        db.prepare(
          `UPDATE workers SET last_seen = ?2, state = ?3, stats_json = COALESCE(?4, stats_json), caps_json = ?5, policy_json = ?6,
                  client_version = ?7 WHERE id = ?1`,
        ).bind(w.id, w.last_seen, w.state, w.stats_json, w.caps_json, w.policy_json, w.client_version),
      );
    }
    for (const id of leases) {
      const j = this.jobs.get(id);
      if (!j || j.status !== "leased") continue;
      stmts.push(
        db.prepare("UPDATE jobs SET lease_expires = ?2 WHERE job_id = ?1 AND status = 'leased' AND lease_worker = ?3").bind(j.job_id, j.lease_expires, j.lease_worker),
      );
    }
    for (let i = 0; i < stmts.length; i += 50) await db.batch(stmts.slice(i, i + 50));
    for (const id of workers) this.dirtyWorkers.delete(id);
    for (const id of leases) this.dirtyLeases.delete(id);
    await this.saveBook();
  }

  /** The cron's 5-minute sweep: reclaim expired leases and flush. */
  async sweep(nowMs = Date.now()) {
    await this.ready();
    const r = await this.reclaim(nowMs);
    await this.flush();
    return { ...r, degraded: this.degraded() };
  }

  // ------------------------------------------------------------ feed

  private feedMinMs() {
    return intVar(this.env.FEED_CACHE_S, 15) > 0 ? 2000 : 0;
  }

  private async memo<T>(key: string, kind: "history" | "model" | "ended", fn: () => Promise<T>): Promise<T> {
    const histS = intVar(this.env.FEED_HISTORY_S, 60);
    if (intVar(this.env.FEED_CACHE_S, 15) <= 0) return fn(); // tests: always fresh
    const [minS, maxS] = kind === "history" ? [histS, HISTORY_MAX_S] : kind === "model" ? [MODEL_MIN_S, MODEL_MAX_S] : [Infinity, Infinity];
    const e = this.memos.get(key);
    const now = Date.now();
    if (e) {
      const age = (now - e.at) / 1000;
      const changed = e.hv !== this.histVersion;
      // Over the read guard, the history stays as it is until the day resets.
      if (this.degraded() || !(age >= maxS || (changed && age >= minS))) return e.val as T;
    }
    const running = this.inflight.get(key);
    if (running) return running as Promise<T>;
    const hv = this.histVersion;
    const p = fn()
      .then((val) => {
        this.memos.set(key, { val, at: Date.now(), hv });
        if (kind === "ended") this.pruneMemos("ended:", 16);
        if (kind === "history") this.pruneMemos("hist:", 16);
        return val;
      })
      .catch((err) => {
        if (e) return e.val as T; // D1 down or out of quota: the last history it had
        throw err;
      })
      .finally(() => this.inflight.delete(key));
    this.inflight.set(key, p);
    return p;
  }

  private pruneMemos(prefix: string, keep: number) {
    const ks = [...this.memos.entries()].filter(([k]) => k.startsWith(prefix)).sort((a, b) => b[1].at - a[1].at);
    for (const [k] of ks.slice(keep)) this.memos.delete(k);
  }

  private source(): FeedSource {
    const active = [...this.jobs.values()] as unknown as FeedJob[];
    const workers = [...this.workers.values()].map((w) => ({ id: w.id, name: w.name, state: w.state, last_seen: w.last_seen, disabled: w.disabled, caps_json: w.caps_json, policy_json: w.policy_json }));
    const counts: Record<string, unknown>[] = [];
    const metrics: Record<string, unknown>[] = [];
    for (const [jobId, runs] of this.runs) {
      const j = this.jobs.get(jobId);
      if (!j) continue;
      const c = new Map<string, number>();
      const sums = new Map<string, { s: number; n: number }>();
      for (const r of runs.values()) {
        if (r.attempt !== j.attempt) continue;
        c.set(r.status, (c.get(r.status) ?? 0) + 1);
        for (const [k, v] of Object.entries(r.metrics)) {
          const a = sums.get(k) ?? { s: 0, n: 0 };
          a.s += v;
          a.n++;
          sums.set(k, a);
        }
      }
      for (const [status, n] of c) counts.push({ job_id: jobId, status, n });
      for (const [name, a] of sums) metrics.push({ job_id: jobId, name, mean: a.s / a.n, n: a.n });
    }
    return { active, workers, live: { counts, metrics }, memo: (k, kind, fn) => this.memo(k, kind, fn) };
  }

  budget() {
    const limit = intVar(this.env.D1_READ_BUDGET, 5_000_000);
    const reads = this.readsToday();
    const frac = Number(this.env.D1_GUARD_FRAC ?? "0.6");
    return { degraded: reads >= frac * limit, frac: Math.round((reads / limit) * 100) / 100, guard_frac: frac };
  }

  private readsToday(nowMs = Date.now()) {
    const day = dayKey(nowMs);
    let n = 0;
    for (const [h, labels] of Object.entries(this.book)) if (h.startsWith(day)) for (const c of Object.values(labels)) n += c.r;
    return n;
  }

  degraded() {
    const b = this.budget();
    if (b.degraded && this.alerted !== dayKey()) {
      this.alerted = dayKey();
      console.warn(`d1 read guard: ${this.readsToday()} rows read today (${Math.round(b.frac * 100)}% of the budget); feed history frozen until 00:00 UTC`);
      void this.ctx.storage.put("alerted", this.alerted);
      this.bump();
    }
    return b.degraded;
  }

  /** The snapshot body for GET /v1/feed?recent=N: rebuilt from memory on change (at most every 2 s) or every 30 s. */
  async feed(recent: number): Promise<string> {
    await this.ready();
    const key = `r${recent}`;
    const c = this.snaps.get(key);
    const now = Date.now();
    if (c && ((c.v === this.version && now - c.at < SNAP_FRESH_MS) || now - c.at < this.feedMinMs())) return c.body;
    const v = this.version;
    const f = await buildFeed(this.envFor("feed"), { recent }, now, this.source());
    const body = JSON.stringify({ ...f, budget: this.budget() });
    this.snaps.set(key, { v, at: now, body });
    return body;
  }

  /** GET /v1/feed/jobs/:id from memory and that job's history; null when the job is unknown. */
  async feedJob(id: string): Promise<string | null> {
    await this.ready();
    const f = await buildFeed(this.envFor("feed_job"), { jobId: id }, Date.now(), this.source());
    const job = f.jobs.find((j) => j.id === id);
    if (!job) return null;
    return JSON.stringify({ schema: FEED_SCHEMA, generated_at: f.generated_at, job, runs: f.runs });
  }

  /** SSE (GET /v1/feed/stream), from the in-memory snapshot. */
  async fetch(req: Request): Promise<Response> {
    const url = new URL(req.url);
    if (url.pathname !== "/v1/feed/stream") return new Response("not found", { status: 404 });
    const interval = Number(url.searchParams.get("interval") ?? 5) * 1000;
    const maxMs = Number(url.searchParams.get("max_s") ?? 300) * 1000;
    const once = url.searchParams.get("once") === "1";
    const recent = Number(url.searchParams.get("recent") ?? 30);
    const enc = new TextEncoder();
    const { readable, writable } = new TransformStream<Uint8Array, Uint8Array>();
    const writer = writable.getWriter();
    const loop = async () => {
      const t0 = Date.now();
      let last = "";
      let lastSent = 0;
      let lastFeed = 0;
      try {
        await writer.write(enc.encode("retry: 5000\n\n"));
        while (true) {
          const body = await this.feed(recent);
          // Changes that matter: ETAs, clocks, heartbeats and the read fraction move every poll, so they refresh every 30 s instead.
          const key = body.replace(/"generated_at":"[^"]*"|"drain_s":[^,}]*|"eta":\{[^}]*\}|"last_seen":"[^"]*"|"frac":[^,}]*/g, "");
          if (key !== last || Date.now() - lastFeed >= 30_000) {
            await writer.write(enc.encode(`event: feed\ndata: ${body}\n\n`));
            last = key;
            lastSent = lastFeed = Date.now();
          } else if (Date.now() - lastSent >= 15_000) {
            await writer.write(enc.encode(": ping\n\n"));
            lastSent = Date.now();
          }
          if (once || Date.now() - t0 + interval > maxMs) break;
          await new Promise((r) => setTimeout(r, interval));
        }
        await writer.close();
      } catch {
        try {
          await writer.abort();
        } catch {}
      }
    };
    void loop();
    return new Response(readable, {
      headers: {
        "content-type": "text/event-stream; charset=utf-8",
        "cache-control": "no-cache, no-transform",
        "x-accel-buffering": "no",
        "access-control-allow-origin": "*",
        "x-content-type-options": "nosniff",
      },
    });
  }

  // ------------------------------------------------------------ read budget

  /** Merges an isolate's read tally (pushed at most once a minute per isolate). */
  async addReads(b: Book) {
    if (!this.bookLoaded) await this.ready();
    mergeBook(this.book, b);
    this.degraded();
  }

  private async saveBook() {
    const cutoff = hourKey(Date.now() - BOOK_HOURS * 3_600_000);
    for (const h of Object.keys(this.book)) if (h < cutoff) delete this.book[h];
    await this.ctx.storage.put("book", this.book);
  }

  /** GET /v1/admin/reads: today's reads per route, the last 48 hours, and the guard. */
  async readStats() {
    await this.ready();
    const day = dayKey();
    const routes: Record<string, { r: number; w: number; q: number }> = {};
    for (const [h, labels] of Object.entries(this.book))
      if (h.startsWith(day)) for (const [l, c] of Object.entries(labels)) addTo({ x: routes } as Book, "x", l, c.r, c.w, c.q);
    const hours = Object.entries(this.book)
      .map(([hour, labels]) => ({ hour, reads: Object.values(labels).reduce((s, c) => s + c.r, 0), writes: Object.values(labels).reduce((s, c) => s + c.w, 0) }))
      .sort((a, b) => a.hour.localeCompare(b.hour));
    const b = this.budget();
    return {
      day,
      reads_today: this.readsToday(),
      budget: intVar(this.env.D1_READ_BUDGET, 5_000_000),
      ...b,
      resets_in_s: Math.ceil((DAY_MS - (Date.now() % DAY_MS)) / 1000),
      routes: Object.entries(routes)
        .map(([label, c]) => ({ label, reads: c.r, writes: c.w, queries: c.q }))
        .sort((a, b) => b.reads - a.reads),
      hours,
      hub: this.state(),
      note: "this Worker's own reads only; the account's daily limit is shared with other projects",
      total_48h: bookReads(this.book),
    };
  }
}
