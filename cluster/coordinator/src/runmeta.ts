// run_meta: what the ghost theater selects runs by (study, iteration, score, outcome), written with a
// run's report (POST /v1/jobs/:id/runs {meta}) and read through GET /v1/runs/query (viewer token).
import { Hono } from "hono";
import { AppEnv, requireViewer } from "./auth";
import type { JobRow } from "./db";
import { HttpError, isObject, json, nowIso, parseJson } from "./util";

export const runMetaRoutes = new Hono<AppEnv>();

const OUTCOMES = new Set(["finished", "failed", "timeout", "invalid", "unknown"]);
const TRAJ_NAME = /^[A-Za-z0-9._-][A-Za-z0-9._/-]{0,199}$/;
const MAX_PARAMS_BYTES = 16_384;

function str(v: unknown, max: number, field: string): string | null {
  if (v === undefined || v === null) return null;
  if (typeof v !== "string" || v.length > max) throw new HttpError(400, `meta.${field} must be a string of at most ${max} chars`);
  return v;
}

function num(v: unknown, field: string): number | null {
  if (v === undefined || v === null) return null;
  if (typeof v !== "number" || !Number.isFinite(v)) throw new HttpError(400, `meta.${field} must be a finite number`);
  return v;
}

function int(v: unknown, field: string): number | null {
  const n = num(v, field);
  if (n !== null && !Number.isSafeInteger(n)) throw new HttpError(400, `meta.${field} must be an integer`);
  return n;
}

/** The run_meta INSERT for a run report's ``meta`` (validated), or null without one. */
export function runMetaStatement(env: AppEnv["Bindings"], job: JobRow, runId: string, seed: number, meta: unknown) {
  if (meta === undefined || meta === null) return null;
  if (!isObject(meta)) throw new HttpError(400, "meta must be an object");
  const params = meta.params ?? {};
  if (!isObject(params)) throw new HttpError(400, "meta.params must be an object");
  const paramsJson = JSON.stringify(params);
  if (paramsJson.length > MAX_PARAMS_BYTES) throw new HttpError(400, `meta.params is larger than ${MAX_PARAMS_BYTES} bytes`);
  const outcome = meta.outcome ?? "unknown";
  if (typeof outcome !== "string" || !OUTCOMES.has(outcome)) throw new HttpError(400, `meta.outcome must be one of ${[...OUTCOMES]}`);
  const valid = meta.valid;
  if (valid !== undefined && valid !== null && typeof valid !== "boolean") throw new HttpError(400, "meta.valid must be a boolean");
  const trajName = str(meta.traj_name, 200, "traj_name");
  if (trajName !== null && (!TRAJ_NAME.test(trajName) || trajName.split("/").some((p) => p === ".." || p === "")))
    throw new HttpError(400, "meta.traj_name is not an artifact name");
  const study = str(meta.study, 200, "study") ?? job.hypothesis_id ?? "adhoc";
  const iteration = int(meta.iteration, "iteration") ?? job.stage_index ?? 0;
  return env.DB.prepare(
    `INSERT INTO run_meta(run_id, job_id, study, hypothesis_id, arm, iteration, generation, seed, params_json, params_sha256,
        score, outcome, failure_reason, progress, valid, insight, course_sha256, course_family, build_hash, recorder,
        water_mode, duration_s, traj_key, traj_sha256, created)
     VALUES (?1, ?2, ?3, ?4, ?5, ?6, ?7, ?8, ?9, ?10, ?11, ?12, ?13, ?14, ?15, ?16, ?17, ?18, ?19, ?20, ?21, ?22, ?23, ?24, ?25)
     ON CONFLICT(run_id) DO UPDATE SET study = excluded.study, iteration = excluded.iteration, generation = excluded.generation,
        params_json = excluded.params_json, params_sha256 = excluded.params_sha256, score = excluded.score,
        outcome = excluded.outcome, failure_reason = excluded.failure_reason, progress = excluded.progress,
        valid = excluded.valid, insight = excluded.insight, course_sha256 = excluded.course_sha256,
        course_family = excluded.course_family, build_hash = excluded.build_hash, recorder = excluded.recorder,
        water_mode = excluded.water_mode, duration_s = excluded.duration_s, traj_key = excluded.traj_key,
        traj_sha256 = excluded.traj_sha256`,
  ).bind(
    runId,
    job.job_id,
    study,
    job.hypothesis_id,
    job.arm,
    iteration,
    int(meta.generation, "generation") ?? 0,
    seed,
    paramsJson,
    str(meta.params_sha256, 64, "params_sha256"),
    num(meta.score, "score"),
    outcome,
    str(meta.failure_reason, 200, "failure_reason"),
    num(meta.progress, "progress"),
    valid === undefined || valid === null ? null : valid ? 1 : 0,
    str(meta.insight, 300, "insight"),
    str(meta.course_sha256, 64, "course_sha256"),
    str(meta.course_family, 100, "course_family"),
    str(meta.build_hash, 64, "build_hash"),
    str(meta.recorder, 40, "recorder"),
    str(meta.water_mode, 20, "water_mode"),
    num(meta.duration_s, "duration_s"),
    trajName === null ? null : `artifacts/${job.job_id}/${runId}/${trajName}`,
    str(meta.traj_sha256, 64, "traj_sha256"),
    nowIso(),
  );
}

interface MetaRow extends Record<string, unknown> {
  params_json: string;
  valid: number | null;
}

/**
 * Runs by study or hypothesis (each job's final attempt only). Query: study | hypothesis (one required),
 * iter_from, iter_to, outcome (comma list), valid (1|0), recorder, course_sha256, order (desc: highest
 * score first, default; asc), top_k (best k per iteration by that order), limit (default 1000, at most
 * 5000), offset. Each run carries its traj artifact's key, sha256 and size (null until uploaded).
 */
runMetaRoutes.get("/v1/runs/query", requireViewer, async (c) => {
  const q = (k: string) => c.req.query(k) ?? null;
  const study = q("study");
  const hyp = q("hypothesis");
  if (!study && !hyp) throw new HttpError(400, "study or hypothesis required");
  const intq = (k: string, min: number, max: number) => {
    const v = q(k);
    if (v === null) return null;
    const n = Number(v);
    if (!Number.isSafeInteger(n) || n < min || n > max) throw new HttpError(400, `${k} must be an integer in [${min}, ${max}]`);
    return n;
  };
  const order = q("order") ?? "desc";
  if (order !== "desc" && order !== "asc") throw new HttpError(400, "order must be desc or asc");
  const outcomes = q("outcome")?.split(",").filter(Boolean) ?? null;
  if (outcomes && !outcomes.every((o) => OUTCOMES.has(o))) throw new HttpError(400, `outcome must list ${[...OUTCOMES]}`);
  const valid = q("valid");
  if (valid !== null && valid !== "0" && valid !== "1") throw new HttpError(400, "valid must be 0 or 1");
  const limit = intq("limit", 1, 5000) ?? 1000;
  const offset = intq("offset", 0, 1_000_000_000) ?? 0;
  const dir = order === "desc" ? "DESC" : "ASC";
  const { results } = await c.env.DB.prepare(
    `SELECT * FROM (
       SELECT m.*, r.attempt, a.sha256 AS artifact_sha256, a.size AS traj_size, a.deleted_at AS traj_deleted_at,
              ROW_NUMBER() OVER (PARTITION BY m.study, m.iteration ORDER BY (m.score IS NULL), m.score ${dir}, m.run_id)
                AS rank_in_iteration
         FROM run_meta m
         JOIN runs r ON r.run_id = m.run_id
         JOIN jobs j ON j.job_id = r.job_id AND j.attempt = r.attempt
         LEFT JOIN artifacts a ON a.key = m.traj_key
        WHERE (?1 IS NULL OR m.study = ?1) AND (?2 IS NULL OR m.hypothesis_id = ?2)
          AND (?3 IS NULL OR m.iteration >= ?3) AND (?4 IS NULL OR m.iteration <= ?4)
          AND (?5 IS NULL OR m.outcome IN (SELECT value FROM json_each(?5)))
          AND (?6 IS NULL OR m.valid = ?6) AND (?7 IS NULL OR m.recorder = ?7) AND (?8 IS NULL OR m.course_sha256 = ?8))
     WHERE ?9 IS NULL OR rank_in_iteration <= ?9
     ORDER BY study, iteration, rank_in_iteration
     LIMIT ?10 OFFSET ?11`,
  )
    .bind(
      study,
      hyp,
      intq("iter_from", -1_000_000, 1_000_000_000),
      intq("iter_to", -1_000_000, 1_000_000_000),
      outcomes ? JSON.stringify(outcomes) : null,
      valid === null ? null : Number(valid),
      q("recorder"),
      q("course_sha256"),
      intq("top_k", 1, 100_000),
      limit + 1,
      offset,
    )
    .all<MetaRow>();
  const more = results.length > limit;
  const runs = results.slice(0, limit).map(({ params_json, valid, artifact_sha256, traj_sha256, ...m }) => ({
    ...m,
    params: parseJson(params_json, {}),
    valid: valid === null ? null : valid === 1,
    traj_sha256: (artifact_sha256 as string | null) ?? (traj_sha256 as string | null) ?? null,
  }));
  return json({ runs, next_offset: more ? offset + limit : null });
});
