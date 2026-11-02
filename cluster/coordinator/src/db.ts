import { Caps, Needs, Policy, matches } from "./match";
import { Env, intVar, nowIso, parseJson } from "./util";

// ---------------------------------------------------------------- config

export const CFG = {
  latestClientVersion: "latest_client_version",
  minClientVersion: "min_client_version",
  rawRetentionDays: "raw_retention_days",
} as const;

export async function getConfig(env: Env): Promise<Record<string, string>> {
  const { results } = await env.DB.prepare("SELECT key, value FROM config").all<{ key: string; value: string }>();
  const out: Record<string, string> = {};
  for (const r of results) out[r.key] = r.value;
  return out;
}

export function setConfigStmt(env: Env, key: string, value: string): D1PreparedStatement {
  return env.DB.prepare(
    "INSERT INTO config(key, value, updated_at) VALUES (?1, ?2, ?3) " +
      "ON CONFLICT(key) DO UPDATE SET value = excluded.value, updated_at = excluded.updated_at",
  ).bind(key, value, nowIso());
}

export async function minClientVersion(env: Env): Promise<string> {
  const cfg = await getConfig(env);
  return cfg[CFG.minClientVersion] ?? env.MIN_CLIENT_VERSION ?? "0.0.0";
}

export async function rawRetentionDays(env: Env): Promise<number> {
  const cfg = await getConfig(env);
  return intVar(cfg[CFG.rawRetentionDays], intVar(env.RAW_RETENTION_DAYS, 14));
}

// ---------------------------------------------------------------- leases

export const leaseTtlMs = (env: Env): number => intVar(env.LEASE_TTL_SECONDS, 600) * 1000;
export const maxAttempts = (env: Env): number => intVar(env.MAX_ATTEMPTS, 3);

export interface ReclaimResult {
  requeued: string[];
  dead: string[];
}

/** Expired leases go back to the queue, or to 'dead' once the job used up its attempts. */
export async function reclaimExpired(env: Env, nowMs = Date.now()): Promise<ReclaimResult> {
  const [, upd] = await env.DB.batch([
    env.DB.prepare(
      `UPDATE leases SET outcome = 'expired'
        WHERE outcome IS NULL AND (job_id, attempt) IN (
          SELECT job_id, attempt FROM jobs WHERE status = 'leased' AND lease_expires < ?1)`,
    ).bind(nowMs),
    env.DB.prepare(
      `UPDATE jobs
          SET status = CASE WHEN attempt >= COALESCE(max_attempts, ?2) THEN 'dead' ELSE 'queued' END,
              last_error = 'lease expired (worker ' || lease_worker || ')',
              ended_at = CASE WHEN attempt >= COALESCE(max_attempts, ?2) THEN ?3 ELSE ended_at END,
              lease_worker = NULL, lease_expires = NULL
        WHERE status = 'leased' AND lease_expires < ?1
        RETURNING job_id, status`,
    ).bind(nowMs, maxAttempts(env), nowIso(nowMs)),
  ]);
  const out: ReclaimResult = { requeued: [], dead: [] };
  for (const r of upd.results as { job_id: string; status: string }[]) (r.status === "dead" ? out.dead : out.requeued).push(r.job_id);
  return out;
}

export interface JobRow {
  job_id: string;
  hypothesis_id: string | null;
  stage_index: number | null;
  arm: string | null;
  backend: string;
  repos_json: string;
  spec_json: string;
  seeds_json: string;
  needs_json: string;
  priority: number;
  status: string;
  attempt: number;
  max_attempts: number | null;
  lease_worker: string | null;
  lease_expires: number | null;
  last_error: string | null;
  created: string;
  purpose: string;
  est_seconds: number | null;
  submitter: string | null;
  campaign: string | null;
}

const SCAN_LIMIT = 200;

/**
 * Leases up to maxJobs queued jobs this worker can run. Candidates are read in
 * priority order, filtered by capability matching, then claimed one by one with
 * a conditional UPDATE, so two workers racing for the same job can't both win.
 */
export async function leaseJobs(env: Env, workerId: string, caps: Caps, policy: Policy, maxJobs: number, nowMs = Date.now()): Promise<JobRow[]> {
  const { results } = await env.DB.prepare(
    `SELECT job_id, backend, needs_json FROM jobs
      WHERE status = 'queued' AND backend IN (SELECT value FROM json_each(?1))
      ORDER BY priority, created LIMIT ?2`,
  )
    .bind(JSON.stringify(caps.backends ?? []), SCAN_LIMIT)
    .all<{ job_id: string; backend: string; needs_json: string }>();
  const out: JobRow[] = [];
  const expires = nowMs + leaseTtlMs(env);
  for (const r of results) {
    if (out.length >= maxJobs) break;
    if (!matches(r.backend, parseJson<Needs>(r.needs_json, {}), caps, policy)) continue;
    const got = await env.DB.prepare(
      `UPDATE jobs SET status = 'leased', lease_worker = ?2, lease_expires = ?3, leased_at_ms = ?4, attempt = attempt + 1
        WHERE job_id = ?1 AND status = 'queued' RETURNING *`,
    )
      .bind(r.job_id, workerId, expires, nowMs)
      .first<JobRow>();
    if (got) out.push(got);
  }
  return out;
}

/** The job if this worker currently holds its lease, else null. */
export async function heldJob(env: Env, jobId: string, workerId: string): Promise<JobRow | null> {
  return await env.DB.prepare("SELECT * FROM jobs WHERE job_id = ? AND status = 'leased' AND lease_worker = ?")
    .bind(jobId, workerId)
    .first<JobRow>();
}

/** Finishes the holder's lease: ok -> done; failed -> retried, or dead after max attempts. */
export async function finishJob(env: Env, job: JobRow, workerId: string, ok: boolean, error: string | null): Promise<string> {
  const now = nowIso();
  const limit = job.max_attempts ?? maxAttempts(env);
  const status = ok ? "done" : job.attempt >= limit ? "dead" : "queued";
  await env.DB.batch([
    env.DB.prepare("UPDATE leases SET outcome = ?3 WHERE job_id = ?1 AND attempt = ?2").bind(job.job_id, job.attempt, ok ? "done" : "failed"),
    env.DB.prepare(
      `UPDATE jobs SET status = ?3, lease_worker = NULL, lease_expires = NULL, last_error = ?4,
              done_at = CASE WHEN ?3 = 'done' THEN ?5 ELSE done_at END,
              done_by = CASE WHEN ?3 = 'done' THEN ?2 ELSE done_by END,
              ended_at = CASE WHEN ?3 IN ('done','dead') THEN ?5 ELSE ended_at END
        WHERE job_id = ?1 AND status = 'leased' AND lease_worker = ?2`,
    ).bind(job.job_id, workerId, status, error ? error.slice(0, 2000) : null, now),
  ]);
  return status;
}

/** Public job shape handed to workers and status readers. */
export function jobView(j: JobRow) {
  return {
    job_id: j.job_id,
    hypothesis_id: j.hypothesis_id,
    stage_index: j.stage_index,
    arm: j.arm,
    backend: j.backend,
    repos: parseJson(j.repos_json, []),
    spec: parseJson(j.spec_json, {}),
    seeds: parseJson(j.seeds_json, []),
    needs: parseJson(j.needs_json, {}),
    priority: j.priority,
    status: j.status,
    attempt: j.attempt,
    lease_expires: j.lease_expires ? nowIso(j.lease_expires) : null,
    last_error: j.last_error,
    purpose: j.purpose,
    est_seconds: j.est_seconds,
    submitter: j.submitter,
    campaign: j.campaign,
  };
}
