// Admin API (ADMIN_TOKEN) and read-only status views (viewer token or admin).
import { Hono } from "hono";
import { hotDays, hotDaysStmt } from "./archive";
import { AppEnv, requireAdmin, requireViewer } from "./auth";
import { CFG, getConfig, jobView, JobRow, rawRetentionDays, setConfigStmt } from "./db";
import { hubStub } from "./hubstub";
import { jobInsert, JobInput } from "./jobs";
import { scrub } from "./purpose";
import { HttpError, compareVersions, isObject, isStringArray, json, nowIso, parseJson, randomToken, readJson, sha256Hex } from "./util";

export const adminRoutes = new Hono<AppEnv>();
adminRoutes.use("/v1/admin/*", requireAdmin);

const VERSION_RE = /^\d+(\.\d+){0,3}([A-Za-z0-9.+_-]*)$/;

adminRoutes.post("/v1/admin/invite", async (c) => {
  const b = await readJson<{ n?: unknown }>(c.req.raw);
  const n = b.n === undefined ? 1 : Number(b.n);
  if (!Number.isInteger(n) || n < 1 || n > 100) throw new HttpError(400, "n must be an integer in 1..100");
  const invites = Array.from({ length: n }, () => randomToken(16));
  const shas = await Promise.all(invites.map((code) => sha256Hex(code)));
  await c.env.DB.prepare("INSERT INTO invites(code_sha256, created) SELECT value, ?2 FROM json_each(?1) WHERE true")
    .bind(JSON.stringify(shas), nowIso())
    .run();
  return json({ invites });
});

adminRoutes.post("/v1/admin/viewer_token", async (c) => {
  const b = await readJson<{ name?: unknown }>(c.req.raw);
  const name = typeof b.name === "string" && b.name.trim() ? b.name.trim().slice(0, 64) : "viewer";
  const token = randomToken();
  await c.env.DB.prepare("INSERT INTO viewer_tokens(token_sha256, name, created) VALUES (?, ?, ?)").bind(await sha256Hex(token), name, nowIso()).run();
  return json({ token, name });
});

adminRoutes.post("/v1/admin/workers/:id/disable", async (c) => {
  const b = await readJson<{ disabled?: unknown }>(c.req.raw);
  const r = await c.env.DB.prepare("UPDATE workers SET disabled = ? WHERE id = ?").bind(b.disabled === false ? 0 : 1, c.req.param("id")).run();
  if (!r.meta.changes) throw new HttpError(404, "unknown worker");
  await hubStub(c.env).syncWorkers([c.req.param("id")]);
  return json({ ok: true });
});

adminRoutes.post("/v1/admin/jobs", async (c) => {
  const b = await readJson<{ jobs?: unknown }>(c.req.raw);
  if (!Array.isArray(b.jobs) || b.jobs.length === 0 || b.jobs.length > 1000) throw new HttpError(400, "jobs must be a list of 1..1000 jobs");
  const made = [];
  for (const j of b.jobs) {
    if (!isObject(j)) throw new HttpError(400, "each job must be an object");
    made.push(await jobInsert(c.env, j as unknown as JobInput));
  }
  try {
    await c.env.DB.batch(made.map((m) => m.stmt));
  } catch (e) {
    if (String(e).includes("UNIQUE")) throw new HttpError(409, "a job_id already exists");
    throw e;
  }
  await hubStub(c.env).syncJobs(made.map((m) => m.id));
  return json({ job_ids: made.map((m) => m.id) });
});

/** Cancels queued and leased jobs; holders learn it from their next heartbeat (revoked). */
adminRoutes.post("/v1/admin/jobs/cancel", async (c) => {
  const b = await readJson(c.req.raw);
  if (!isStringArray(b.job_ids)) throw new HttpError(400, "job_ids must be a list of strings");
  const { results } = await c.env.DB.prepare(
    `UPDATE jobs SET status = 'cancelled', lease_worker = NULL, lease_expires = NULL, ended_at = ?2
      WHERE status IN ('queued','leased') AND job_id IN (SELECT value FROM json_each(?1)) RETURNING job_id`,
  )
    .bind(JSON.stringify(b.job_ids), nowIso())
    .all<{ job_id: string }>();
  await hubStub(c.env).syncJobs(results.map((r) => r.job_id));
  return json({ cancelled: results.map((r) => r.job_id) });
});

/** A one-line verdict on a job (a judge's or the submitter's), shown in the public feed. */
adminRoutes.post("/v1/admin/jobs/judge", async (c) => {
  const b = await readJson(c.req.raw);
  if (typeof b.job_id !== "string") throw new HttpError(400, "job_id required");
  const line = typeof b.line === "string" ? b.line.replace(/\s+/g, " ").trim() : "";
  if (!line || line.length > 200) throw new HttpError(400, "line must be 1..200 chars");
  if (scrub(line) !== line) throw new HttpError(400, "line looks like it holds a secret, an address or a local path; the feed is public");
  const r = await c.env.DB.prepare("UPDATE jobs SET judge = ?2 WHERE job_id = ?1").bind(b.job_id, line).run();
  if (!r.meta.changes) throw new HttpError(404, "unknown job");
  await hubStub(c.env).touchHistory();
  return json({ ok: true });
});

/** Requeues dead or failed jobs with a fresh attempt budget. */
adminRoutes.post("/v1/admin/jobs/requeue", async (c) => {
  const b = await readJson(c.req.raw);
  if (!isStringArray(b.job_ids)) throw new HttpError(400, "job_ids must be a list of strings");
  const { results } = await c.env.DB.prepare(
    `UPDATE jobs SET status = 'queued', attempt = 0, ended_at = NULL WHERE status IN ('dead','failed')
        AND job_id IN (SELECT value FROM json_each(?1)) RETURNING job_id`,
  )
    .bind(JSON.stringify(b.job_ids))
    .all<{ job_id: string }>();
  await hubStub(c.env).syncJobs(results.map((r) => r.job_id));
  return json({ requeued: results.map((r) => r.job_id) });
});

/** D1 reads per route today and per hour (48 h), and the read guard (src/budget.ts, src/hub.ts). */
adminRoutes.get("/v1/admin/reads", async (c) => json(await hubStub(c.env).readStats()));

/** Writes the Hub's batched state and reloads it from D1 (after editing jobs or workers in D1 by hand). */
adminRoutes.post("/v1/admin/hub/reload", async (c) => json(await hubStub(c.env).reload()));

/** Registers a repo (url, main branch) and optionally sets main_version by hand. */
adminRoutes.put("/v1/admin/repos", async (c) => {
  const b = await readJson(c.req.raw);
  if (typeof b.name !== "string" || !b.name) throw new HttpError(400, "name required");
  const str = (v: unknown) => (typeof v === "string" && v ? v : null);
  await c.env.DB.prepare(
    `INSERT INTO repos(name, url, main_branch, main_version, updated_at) VALUES (?1, ?2, COALESCE(?3, 'main'), ?4, ?5)
     ON CONFLICT(name) DO UPDATE SET url = COALESCE(?2, repos.url), main_branch = COALESCE(?3, repos.main_branch),
       main_version = COALESCE(?4, repos.main_version), updated_at = ?5`,
  )
    .bind(b.name, str(b.url), str(b.main_branch), str(b.main_version), nowIso())
    .run();
  await hubStub(c.env).syncConfig();
  return json(await c.env.DB.prepare("SELECT * FROM repos WHERE name = ?").bind(b.name).first());
});

/** Client-version gate: workers older than min_version get 426 on /v1/lease. */
adminRoutes.put("/v1/admin/client_release", async (c) => {
  const b = await readJson(c.req.raw);
  if (typeof b.version !== "string" || !VERSION_RE.test(b.version)) throw new HttpError(400, "version must look like 0.2.0");
  const stmts = [setConfigStmt(c.env, CFG.latestClientVersion, b.version)];
  if (b.min_version !== undefined) {
    if (typeof b.min_version !== "string" || !VERSION_RE.test(b.min_version)) throw new HttpError(400, "min_version must look like 0.1.0");
    if (compareVersions(b.min_version, b.version) > 0) throw new HttpError(400, "min_version cannot exceed version");
    stmts.push(setConfigStmt(c.env, CFG.minClientVersion, b.min_version));
  }
  await c.env.DB.batch(stmts);
  await hubStub(c.env).syncConfig();
  return json(await getConfig(c.env));
});

/** Retention: raw_days for new raw artifacts (existing ones keep their expiry); hot_days {kind: days}, how
 * long each kind stays in R2 once an archive node holds it (archive.ts; unset kinds keep their defaults). */
adminRoutes.put("/v1/admin/retention", async (c) => {
  const b = await readJson(c.req.raw);
  if (b.raw_days === undefined && b.hot_days === undefined) throw new HttpError(400, "raw_days or hot_days required");
  const stmts = [];
  if (b.raw_days !== undefined) {
    if (!Number.isInteger(b.raw_days) || (b.raw_days as number) < 1) throw new HttpError(400, "raw_days must be a positive integer");
    stmts.push(setConfigStmt(c.env, CFG.rawRetentionDays, String(b.raw_days)));
  }
  if (b.hot_days !== undefined) stmts.push(hotDaysStmt(c.env, b.hot_days));
  await c.env.DB.batch(stmts);
  await hubStub(c.env).syncConfig();
  return json({ raw_days: await rawRetentionDays(c.env), hot_days: await hotDays(c.env) });
});

// ------------------------------------------------------------------ read-only views

adminRoutes.get("/v1/status", requireViewer, async (c) => {
  const counts = await c.env.DB.prepare("SELECT backend, status, COUNT(*) AS n FROM jobs GROUP BY backend, status").all();
  const workers = await c.env.DB.prepare(
    `SELECT w.id, w.name, w.caps_json, w.policy_json, w.client_version, w.state, w.last_seen, w.disabled,
            (SELECT COUNT(*) FROM jobs j WHERE j.lease_worker = w.id AND j.status = 'leased') AS leased
       FROM workers w ORDER BY w.last_seen DESC LIMIT 500`,
  ).all<Record<string, unknown>>();
  return json({
    jobs: counts.results,
    workers: workers.results.map(({ caps_json, policy_json, ...w }) => ({
      ...w,
      caps: parseJson(caps_json as string, {}),
      policy: parseJson(policy_json as string, {}),
    })),
    raw_retention_days: await rawRetentionDays(c.env),
  });
});

adminRoutes.get("/v1/jobs", requireViewer, async (c) => {
  const status = c.req.query("status");
  const hyp = c.req.query("hypothesis");
  const { results } = await c.env.DB.prepare(
    `SELECT * FROM jobs WHERE (?1 IS NULL OR status = ?1) AND (?2 IS NULL OR hypothesis_id = ?2)
      ORDER BY created DESC LIMIT 500`,
  )
    .bind(status ?? null, hyp ?? null)
    .all<JobRow>();
  return json({ jobs: results.map((j) => ({ ...jobView(j), lease_worker: j.lease_worker, created: j.created })) });
});

adminRoutes.get("/v1/jobs/:id/runs", requireViewer, async (c) => {
  const { results } = await c.env.DB.prepare(
    `SELECT r.run_id, r.worker_id, r.attempt, r.seed, r.status, r.wall_s, r.error, json_group_object(m.name, m.value) FILTER (WHERE m.name IS NOT NULL) AS metrics
       FROM runs r LEFT JOIN metrics m ON m.run_id = r.run_id WHERE r.job_id = ? GROUP BY r.run_id ORDER BY r.attempt, r.seed`,
  )
    .bind(c.req.param("id"))
    .all<Record<string, unknown>>();
  return json({ runs: results.map((r) => ({ ...r, metrics: parseJson(r.metrics as string, {}) })) });
});
