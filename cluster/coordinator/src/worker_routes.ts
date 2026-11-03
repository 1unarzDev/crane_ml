// Routes used by worker daemons: join, lease, heartbeat, release, runs, completion.
import { Hono } from "hono";
import { AppEnv, requireWorker } from "./auth";
import { CFG, finishJob, getConfig, heldJob, jobView, leaseJobs, leaseTtlMs, minClientVersion, reclaimExpired } from "./db";
import { leaseCaps, sanitizeCaps, sanitizePolicy } from "./match";
import { runMetaStatement } from "./runmeta";
import {
  HttpError,
  compareVersions,
  intVar,
  isObject,
  isStringArray,
  json,
  nowIso,
  parseJson,
  randomId,
  randomToken,
  readJson,
  sha256Hex,
} from "./util";

export const workerRoutes = new Hono<AppEnv>();

const STATES = new Set(["idle", "running", "paused", "updating", "stopping"]);

export async function repoVersions(env: AppEnv["Bindings"]): Promise<Record<string, string | null>> {
  const { results } = await env.DB.prepare("SELECT name, main_version FROM repos").all<{ name: string; main_version: string | null }>();
  return Object.fromEntries(results.map((r) => [r.name, r.main_version]));
}

workerRoutes.get("/v1/ping", async (c) => {
  const cfg = await getConfig(c.env);
  return json({
    ok: true,
    min_client_version: cfg[CFG.minClientVersion] ?? c.env.MIN_CLIENT_VERSION ?? "0.0.0",
    latest_client_version: cfg[CFG.latestClientVersion] ?? null,
  });
});

workerRoutes.post("/v1/join", async (c) => {
  const b = await readJson(c.req.raw);
  const { invite, name } = b;
  if (typeof invite !== "string" || !invite) throw new HttpError(400, "invite required");
  if (typeof name !== "string" || !name.trim() || name.length > 100) throw new HttpError(400, "name required (<=100 chars)");
  const workerId = randomId("w_");
  const token = randomToken();
  const now = nowIso();
  const inviteSha = await sha256Hex(invite);
  // One transaction: the worker row exists only if the invite was unused, and
  // the invite is consumed only by that worker.
  const [ins] = await c.env.DB.batch([
    c.env.DB.prepare(
      `INSERT INTO workers(id, token_sha256, name, caps_json, policy_json, client_version, created, last_seen)
       SELECT ?1, ?2, ?3, ?4, ?5, ?6, ?7, ?7
        WHERE EXISTS (SELECT 1 FROM invites WHERE code_sha256 = ?8 AND used_by IS NULL)`,
    ).bind(
      workerId,
      await sha256Hex(token),
      name.trim(),
      JSON.stringify(sanitizeCaps(b.caps)),
      JSON.stringify(sanitizePolicy(b.policy)),
      typeof b.client_version === "string" ? b.client_version.slice(0, 40) : null,
      now,
      inviteSha,
    ),
    c.env.DB.prepare(
      `UPDATE invites SET used_by = ?1, used_at = ?2
        WHERE code_sha256 = ?3 AND used_by IS NULL AND EXISTS (SELECT 1 FROM workers WHERE id = ?1)`,
    ).bind(workerId, now, inviteSha),
  ]);
  if (!ins.meta.changes) throw new HttpError(403, "invalid or used invite");
  return json({ worker_id: workerId, token });
});

workerRoutes.get("/v1/repos", requireWorker, async (c) => json({ repos: await repoVersions(c.env) }));

workerRoutes.post("/v1/lease", requireWorker, async (c) => {
  const w = c.get("worker");
  const b = await readJson(c.req.raw);
  const reported = c.req.header("x-client-version") ?? (typeof b.client_version === "string" ? b.client_version : null);
  const version = reported ?? w.client_version ?? "0.0.0";
  const minV = await minClientVersion(c.env);
  if (compareVersions(version, minV) < 0) {
    const cfg = await getConfig(c.env);
    throw new HttpError(426, "client too old", { min_client_version: minV, latest_client_version: cfg[CFG.latestClientVersion] ?? null });
  }
  const maxJobs = b.max_jobs === undefined ? 1 : Number(b.max_jobs);
  if (!Number.isInteger(maxJobs) || maxJobs < 0) throw new HttpError(400, "max_jobs must be a non-negative integer");
  // Fresh capabilities/policy replace the stored ones (a machine can gain a GPU or change its policy).
  const caps = b.caps !== undefined ? sanitizeCaps(b.caps) : sanitizeCaps(parseJson(w.caps_json, {}));
  const policy = b.policy !== undefined ? sanitizePolicy(b.policy) : sanitizePolicy(parseJson(w.policy_json, {}));
  const state = typeof b.state === "string" && STATES.has(b.state) ? b.state : "idle";

  const now = Date.now();
  await reclaimExpired(c.env, now);
  const jobs = maxJobs > 0 ? await leaseJobs(c.env, w.id, leaseCaps(caps, b.backends), policy, Math.min(maxJobs, intVar(c.env.MAX_LEASE_JOBS, 4)), now) : [];
  await c.env.DB.prepare(
    `UPDATE workers SET last_seen = ?2, client_version = COALESCE(?3, client_version), caps_json = ?4, policy_json = ?5,
            state = ?6 WHERE id = ?1`,
  )
    .bind(w.id, nowIso(now), reported, JSON.stringify(caps), JSON.stringify(policy), jobs.length ? "running" : state)
    .run();
  return json({
    jobs: jobs.map(jobView),
    repos: await repoVersions(c.env),
    lease_ttl_s: leaseTtlMs(c.env) / 1000,
  });
});

workerRoutes.post("/v1/heartbeat", requireWorker, async (c) => {
  const w = c.get("worker");
  const b = await readJson(c.req.raw);
  const ids = b.job_ids ?? [];
  if (!isStringArray(ids)) throw new HttpError(400, "job_ids must be an array of strings");
  const now = Date.now();
  const state = typeof b.state === "string" && STATES.has(b.state) ? b.state : null;
  const stats = isObject(b.stats) ? JSON.stringify(b.stats).slice(0, 4000) : null;
  // A heartbeat guarantees at least one TTL from now, without unbounded growth.
  const [ext] = await c.env.DB.batch([
    c.env.DB.prepare(
      `UPDATE jobs SET lease_expires = MAX(COALESCE(lease_expires, 0), ?3)
        WHERE status = 'leased' AND lease_worker = ?1 AND job_id IN (SELECT value FROM json_each(?2))
        RETURNING job_id`,
    ).bind(w.id, JSON.stringify(ids), now + leaseTtlMs(c.env)),
    c.env.DB.prepare(
      "UPDATE workers SET last_seen = ?2, state = COALESCE(?3, state), stats_json = COALESCE(?4, stats_json) WHERE id = ?1",
    ).bind(w.id, nowIso(now), state, stats),
  ]);
  const extended = new Set((ext.results as { job_id: string }[]).map((r) => r.job_id));
  // Revoked: the lease expired and was reclaimed, or an admin cancelled the job. The worker must stop it.
  return json({ extended: ids.filter((id) => extended.has(id)), revoked: ids.filter((id) => !extended.has(id)) });
});

workerRoutes.post("/v1/release", requireWorker, async (c) => {
  const w = c.get("worker");
  const b = await readJson(c.req.raw);
  const ids = b.job_ids ?? [];
  if (!isStringArray(ids)) throw new HttpError(400, "job_ids must be an array of strings");
  // A voluntary release (shutdown, user became active) does not count as a failed attempt.
  const [, rel] = await c.env.DB.batch([
    c.env.DB.prepare(
      `UPDATE leases SET outcome = 'released' WHERE worker_id = ?1 AND outcome IS NULL
          AND job_id IN (SELECT value FROM json_each(?2))`,
    ).bind(w.id, JSON.stringify(ids)),
    c.env.DB.prepare(
      `UPDATE jobs SET status = 'queued', lease_worker = NULL, lease_expires = NULL, attempt = MAX(attempt - 1, 0)
        WHERE status = 'leased' AND lease_worker = ?1 AND job_id IN (SELECT value FROM json_each(?2))
        RETURNING job_id`,
    ).bind(w.id, JSON.stringify(ids)),
  ]);
  return json({ released: (rel.results as { job_id: string }[]).map((r) => r.job_id) });
});

const METRIC_NAME = /^[A-Za-z0-9._:/-]{1,128}$/;

workerRoutes.post("/v1/jobs/:id/runs", requireWorker, async (c) => {
  const w = c.get("worker");
  const job = await heldJob(c.env, c.req.param("id"), w.id);
  if (!job) throw new HttpError(409, "not the lease holder");
  const b = await readJson(c.req.raw);
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
  const meta = runMetaStatement(c.env, job, runId, seed as number, b.meta);
  await c.env.DB.batch([
    c.env.DB.prepare("DELETE FROM metrics WHERE run_id = ?").bind(runId),
    c.env.DB.prepare(
      `INSERT INTO runs(run_id, job_id, worker_id, attempt, seed, status, wall_s, error, created)
       VALUES (?1, ?2, ?3, ?4, ?5, ?6, ?7, ?8, ?9)
       ON CONFLICT(run_id) DO UPDATE SET status = excluded.status, wall_s = excluded.wall_s, error = excluded.error`,
    ).bind(runId, job.job_id, w.id, job.attempt, seed, status, wall, error, nowIso()),
    c.env.DB.prepare("INSERT INTO metrics(run_id, name, value) SELECT ?1, key, value FROM json_each(?2)").bind(
      runId,
      JSON.stringify(metrics),
    ),
    ...(meta ? [meta] : []),
  ]);
  return json({ run_id: runId });
});

workerRoutes.post("/v1/jobs/:id/complete", requireWorker, async (c) => {
  const w = c.get("worker");
  const job = await heldJob(c.env, c.req.param("id"), w.id);
  if (!job) throw new HttpError(409, "not the lease holder");
  const b = await readJson(c.req.raw);
  const ok = b.status === "ok";
  if (!ok && b.status !== "failed") throw new HttpError(400, "status must be ok or failed");
  const status = await finishJob(c.env, job, w.id, ok, typeof b.error === "string" ? b.error : null);
  return json({ job_id: job.job_id, status });
});
