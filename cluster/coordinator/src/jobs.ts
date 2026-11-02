// Job creation shared by the admin API and hypothesis stages.
import { parseNeeds } from "./match";
import { Env, HttpError, isObject, nowIso, randomId } from "./util";

export const SAFE_NAME = /^[A-Za-z0-9._-]{1,64}$/;
const REPO_NAME = /^[A-Za-z0-9._/-]{1,200}$/;
const REF = /^[A-Za-z0-9._/@^~-]{1,200}$/;
export const MAX_SEEDS = 10_000;

export interface RepoRef {
  name: string;
  url?: string;
  ref: string;
}

export interface JobInput {
  job_id?: string;
  hypothesis_id?: string | null;
  stage_index?: number | null;
  arm?: string | null;
  backend: string;
  repos?: RepoRef[];
  spec?: Record<string, unknown>;
  seeds?: number[];
  needs?: unknown;
  priority?: number;
  max_attempts?: number | null;
}

export function parseSeeds(v: unknown, field = "seeds"): number[] {
  if (v === undefined) return [0];
  if (!Array.isArray(v) || v.length === 0 || v.length > MAX_SEEDS || !v.every((s) => Number.isSafeInteger(s)))
    throw new HttpError(400, `${field} must be a non-empty list of integers`);
  return v as number[];
}

/**
 * Resolves "@main" refs to the repo's current main_version (set by the GitHub
 * webhook), so every job records the exact commit it ran. Other refs pass through.
 */
export async function resolveRepos(env: Env, v: unknown): Promise<RepoRef[]> {
  if (v === undefined) return [];
  if (!Array.isArray(v) || v.length > 16) throw new HttpError(400, "repos must be a list (at most 16)");
  const out: RepoRef[] = [];
  for (const r of v) {
    if (!isObject(r) || typeof r.name !== "string" || !REPO_NAME.test(r.name)) throw new HttpError(400, "repos[].name must be a repo name");
    if (typeof r.ref !== "string" || !REF.test(r.ref)) throw new HttpError(400, `repos[${r.name}].ref must be a git ref`);
    const known = await env.DB.prepare("SELECT url, main_version FROM repos WHERE name = ?")
      .bind(r.name)
      .first<{ url: string | null; main_version: string | null }>();
    let ref = r.ref;
    if (ref === "@main") {
      if (!known?.main_version) throw new HttpError(400, `repo ${r.name} has no main_version yet (webhook or PUT /v1/admin/repos)`);
      ref = known.main_version;
    }
    const url = typeof r.url === "string" ? r.url : (known?.url ?? undefined);
    if (!url) throw new HttpError(400, `repo ${r.name} needs a url (in the job or registered via PUT /v1/admin/repos)`);
    out.push({ name: r.name, url, ref });
  }
  return out;
}

/** Validates one job and returns its INSERT statement and id. */
export async function jobInsert(env: Env, j: JobInput): Promise<{ id: string; stmt: D1PreparedStatement }> {
  if (typeof j.backend !== "string" || !SAFE_NAME.test(j.backend)) throw new HttpError(400, "backend must be a name");
  const id = j.job_id ?? randomId("j_");
  if (!SAFE_NAME.test(id)) throw new HttpError(400, "job_id must match [A-Za-z0-9._-]{1,64}");
  if (j.spec !== undefined && !isObject(j.spec)) throw new HttpError(400, "spec must be an object");
  let needs;
  try {
    needs = parseNeeds(j.needs);
  } catch (e) {
    throw new HttpError(400, String(e));
  }
  const priority = j.priority ?? 100;
  if (!Number.isInteger(priority)) throw new HttpError(400, "priority must be an integer");
  if (j.max_attempts != null && (!Number.isInteger(j.max_attempts) || j.max_attempts < 1)) throw new HttpError(400, "max_attempts must be >= 1");
  const repos = await resolveRepos(env, j.repos);
  const stmt = env.DB.prepare(
    `INSERT INTO jobs(job_id, hypothesis_id, stage_index, arm, backend, repos_json, spec_json, seeds_json, needs_json,
                      priority, max_attempts, created)
     VALUES (?1, ?2, ?3, ?4, ?5, ?6, ?7, ?8, ?9, ?10, ?11, ?12)`,
  ).bind(
    id,
    j.hypothesis_id ?? null,
    j.stage_index ?? null,
    j.arm ?? null,
    j.backend,
    JSON.stringify(repos),
    JSON.stringify(j.spec ?? {}),
    JSON.stringify(parseSeeds(j.seeds)),
    JSON.stringify(needs),
    priority,
    j.max_attempts ?? null,
    nowIso(),
  );
  return { id, stmt };
}
