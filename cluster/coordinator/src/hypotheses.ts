// Hypotheses: a claim about one metric, tested by staged promotion. Each stage
// runs the baseline and every active variant on the same seeds with one backend;
// the analysis (paired test, successive halving) runs client-side in Python and
// posts its decision to /advance, which enqueues the next stage or finishes.
import { Hono } from "hono";
import { AppEnv, requireAdmin, requireViewer } from "./auth";
import { JobInput, SAFE_NAME, jobInsert, parseSeeds, resolveRepos } from "./jobs";
import { checkCampaign, checkEst, checkPurpose, checkSubmitter } from "./purpose";
import { HttpError, isObject, json, nowIso, parseJson, randomId, readJson } from "./util";

export const hypRoutes = new Hono<AppEnv>();

interface Variant {
  name: string;
  ref: string;
  params?: Record<string, unknown>;
  active?: boolean;
}
interface Stage {
  name: string;
  backend: string;
  params?: Record<string, unknown>;
  needs?: unknown;
  priority?: number;
  seeds_per_job?: number;
  est_seconds?: number; // per run; overrides the hypothesis's
}
interface HypRow {
  id: string;
  name: string;
  claim: string;
  metric: string;
  direction: string;
  repo: string | null;
  baseline_ref: string;
  variants_json: string;
  scenario: string | null;
  seeds_json: string;
  budget_json: string;
  stages_json: string;
  promotion_json: string;
  stage_index: number;
  status: string;
  decisions_json: string;
  submitted_by: string | null;
  purpose: string;
  est_seconds: number | null;
  campaign: string | null;
  created: string;
  updated: string;
}

const MAX_JOBS_PER_ENQUEUE = 2000;

function parseStages(v: unknown): Stage[] {
  if (!Array.isArray(v) || v.length === 0 || v.length > 8) throw new HttpError(400, "stages must be a list of 1..8 stages");
  return v.map((s, i) => {
    if (!isObject(s) || typeof s.backend !== "string" || !SAFE_NAME.test(s.backend)) throw new HttpError(400, `stages[${i}].backend required`);
    if (s.params !== undefined && !isObject(s.params)) throw new HttpError(400, `stages[${i}].params must be an object`);
    const spj = s.seeds_per_job ?? 1;
    if (!Number.isInteger(spj) || (spj as number) < 1) throw new HttpError(400, `stages[${i}].seeds_per_job must be >= 1`);
    const est = s.est_seconds === undefined ? undefined : checkEst(s.est_seconds, `stages[${i}].est_seconds`);
    return { ...(s as unknown as Stage), name: typeof s.name === "string" ? s.name : `stage${i + 1}`, seeds_per_job: spj as number, est_seconds: est };
  });
}

function parseVariants(v: unknown): Variant[] {
  if (!Array.isArray(v) || v.length === 0 || v.length > 64) throw new HttpError(400, "variants must be a list of 1..64 variants");
  const names = new Set<string>();
  return v.map((x, i) => {
    if (!isObject(x) || typeof x.name !== "string" || !SAFE_NAME.test(x.name) || x.name === "baseline")
      throw new HttpError(400, `variants[${i}].name must be a name other than 'baseline'`);
    if (names.has(x.name)) throw new HttpError(400, `duplicate variant ${x.name}`);
    names.add(x.name);
    if (typeof x.ref !== "string" || !x.ref) throw new HttpError(400, `variants[${i}].ref required`);
    if (x.params !== undefined && !isObject(x.params)) throw new HttpError(400, `variants[${i}].params must be an object`);
    return { name: x.name, ref: x.ref, params: x.params as Record<string, unknown> | undefined, active: true };
  });
}

/** Jobs for one stage: every arm (baseline + active variants) on the same seeds. */
async function stageJobs(env: AppEnv["Bindings"], h: HypRow, stageIndex: number, seeds: number[], arms: string[]): Promise<D1PreparedStatement[]> {
  const stage = parseJson<Stage[]>(h.stages_json, [])[stageIndex];
  const variants = parseJson<Variant[]>(h.variants_json, []);
  const budget = parseJson<{ max_runs?: number }>(h.budget_json, {});
  if (budget.max_runs !== undefined) {
    const used = await env.DB.prepare(
      "SELECT COALESCE(SUM(json_array_length(seeds_json)), 0) AS n FROM jobs WHERE hypothesis_id = ? AND status <> 'cancelled'",
    )
      .bind(h.id)
      .first<{ n: number }>();
    if ((used?.n ?? 0) + seeds.length * arms.length > budget.max_runs)
      throw new HttpError(409, `budget: ${used?.n ?? 0} + ${seeds.length * arms.length} runs exceeds max_runs ${budget.max_runs}`);
  }
  const chunk = stage.seeds_per_job ?? 1;
  if (Math.ceil(seeds.length / chunk) * arms.length > MAX_JOBS_PER_ENQUEUE) throw new HttpError(400, "too many jobs; raise seeds_per_job");
  const out: D1PreparedStatement[] = [];
  for (const arm of arms) {
    const v = variants.find((x) => x.name === arm);
    const ref = arm === "baseline" ? h.baseline_ref : v!.ref;
    for (let i = 0; i < seeds.length; i += chunk) {
      const job: JobInput = {
        hypothesis_id: h.id,
        stage_index: stageIndex,
        arm,
        backend: stage.backend,
        repos: h.repo ? [{ name: h.repo, ref }] : [],
        spec: { ...(stage.params ?? {}), ...(v?.params ?? {}), scenario: h.scenario, metric: h.metric, arm, stage: stage.name, ref },
        seeds: seeds.slice(i, i + chunk),
        needs: stage.needs,
        priority: stage.priority,
        purpose: h.purpose,
        est_seconds: stage.est_seconds ?? h.est_seconds,
        submitter: h.submitted_by,
        campaign: h.campaign,
      };
      out.push((await jobInsert(env, job, true)).stmt);
    }
  }
  return out;
}

const activeArms = (h: HypRow) => ["baseline", ...parseJson<Variant[]>(h.variants_json, []).filter((v) => v.active !== false).map((v) => v.name)];

hypRoutes.post("/v1/hyp", requireAdmin, async (c) => {
  const b = await readJson(c.req.raw);
  for (const f of ["name", "claim", "metric"] as const)
    if (typeof b[f] !== "string" || !(b[f] as string).trim()) throw new HttpError(400, `${f} required`);
  if (b.direction !== "increase" && b.direction !== "decrease") throw new HttpError(400, "direction must be increase or decrease");
  if (typeof b.baseline_ref !== "string" || !b.baseline_ref) throw new HttpError(400, "baseline_ref required");
  const stages = parseStages(b.stages);
  const purpose = checkPurpose(b.purpose);
  const est = b.est_seconds === undefined && stages.every((s) => s.est_seconds !== undefined) ? null : checkEst(b.est_seconds);
  const submitter = checkSubmitter(b.submitter ?? b.submitted_by);
  const campaign = checkCampaign(b.campaign);
  const variants = parseVariants(b.variants);
  const seeds = parseSeeds(b.seeds);
  const repo = typeof b.repo === "string" ? b.repo : null;
  let baseline = b.baseline_ref as string;
  if (repo) {
    // Validate every ref once and pin "@main" to the commit it means now, so all stages share one baseline.
    [{ ref: baseline }] = await resolveRepos(c.env, [{ name: repo, url: b.repo_url, ref: baseline }]);
    await resolveRepos(c.env, variants.map((v) => ({ name: repo, url: b.repo_url, ref: v.ref })));
    if (typeof b.repo_url === "string")
      await c.env.DB.prepare("INSERT INTO repos(name, url) VALUES (?, ?) ON CONFLICT(name) DO NOTHING").bind(repo, b.repo_url).run();
  }
  const now = nowIso();
  const h: HypRow = {
    id: randomId("h_"),
    name: (b.name as string).slice(0, 200),
    claim: (b.claim as string).slice(0, 4000),
    metric: b.metric as string,
    direction: b.direction,
    repo,
    baseline_ref: baseline,
    variants_json: JSON.stringify(variants),
    scenario: typeof b.scenario === "string" ? b.scenario : null,
    seeds_json: JSON.stringify(seeds),
    budget_json: JSON.stringify(isObject(b.budget) ? b.budget : {}),
    stages_json: JSON.stringify(stages),
    promotion_json: JSON.stringify(isObject(b.promotion) ? b.promotion : {}),
    stage_index: 0,
    status: "running",
    decisions_json: "[]",
    submitted_by: submitter,
    purpose,
    est_seconds: est,
    campaign,
    created: now,
    updated: now,
  };
  const insert = c.env.DB.prepare(
    `INSERT INTO hypotheses(id, name, claim, metric, direction, repo, baseline_ref, variants_json, scenario, seeds_json,
                            budget_json, stages_json, promotion_json, submitted_by, created, updated, purpose, est_seconds, campaign)
     VALUES (?1, ?2, ?3, ?4, ?5, ?6, ?7, ?8, ?9, ?10, ?11, ?12, ?13, ?14, ?15, ?15, ?16, ?17, ?18)`,
  ).bind(h.id, h.name, h.claim, h.metric, h.direction, repo, baseline, h.variants_json, h.scenario, h.seeds_json, h.budget_json,
    h.stages_json, h.promotion_json, submitter, now, purpose, est, campaign);
  const jobs = await stageJobs(c.env, h, 0, seeds, activeArms(h));
  await c.env.DB.batch([insert, ...jobs]);
  return json({ id: h.id, stage: stages[0].name, jobs: jobs.length, baseline_ref: baseline });
});

function hypView(h: HypRow) {
  const stages = parseJson<Stage[]>(h.stages_json, []);
  return {
    id: h.id, name: h.name, claim: h.claim, metric: h.metric, direction: h.direction, repo: h.repo,
    baseline_ref: h.baseline_ref, variants: parseJson(h.variants_json, []), scenario: h.scenario,
    seeds: parseJson(h.seeds_json, []), budget: parseJson(h.budget_json, {}), stages,
    promotion: parseJson(h.promotion_json, {}), stage_index: h.stage_index,
    stage: h.status === "running" ? stages[h.stage_index]?.name : h.status,
    status: h.status, decisions: parseJson(h.decisions_json, []), created: h.created, updated: h.updated,
    purpose: h.purpose, est_seconds: h.est_seconds, submitter: h.submitted_by, campaign: h.campaign,
  };
}

async function loadHyp(env: AppEnv["Bindings"], id: string): Promise<HypRow> {
  const h = await env.DB.prepare("SELECT * FROM hypotheses WHERE id = ?").bind(id).first<HypRow>();
  if (!h) throw new HttpError(404, "unknown hypothesis");
  return h;
}

hypRoutes.get("/v1/hyp", requireViewer, async (c) => {
  const { results } = await c.env.DB.prepare("SELECT * FROM hypotheses ORDER BY created DESC LIMIT 100").all<HypRow>();
  return json({ hypotheses: results.map(hypView) });
});

hypRoutes.get("/v1/hyp/:id", requireViewer, async (c) => {
  const h = await loadHyp(c.env, c.req.param("id"));
  const { results } = await c.env.DB.prepare(
    "SELECT stage_index, arm, status, COUNT(*) AS n FROM jobs WHERE hypothesis_id = ? GROUP BY stage_index, arm, status",
  )
    .bind(h.id)
    .all();
  const used = await c.env.DB.prepare(
    "SELECT COALESCE(SUM(json_array_length(seeds_json)), 0) AS n FROM jobs WHERE hypothesis_id = ? AND status <> 'cancelled'",
  )
    .bind(h.id)
    .first<{ n: number }>();
  return json({ ...hypView(h), jobs: results, runs_used: used?.n ?? 0 });
});

/** Per-run metric values of finished jobs: only the runs of each job's final (completed) attempt. */
hypRoutes.get("/v1/hyp/:id/results", requireViewer, async (c) => {
  const h = await loadHyp(c.env, c.req.param("id"));
  const stage = c.req.query("stage") === undefined ? h.stage_index : Number(c.req.query("stage"));
  const { results } = await c.env.DB.prepare(
    `SELECT j.arm, r.seed, r.status, m.name, m.value
       FROM jobs j JOIN runs r ON r.job_id = j.job_id AND r.attempt = j.attempt
       LEFT JOIN metrics m ON m.run_id = r.run_id
      WHERE j.hypothesis_id = ?1 AND j.stage_index = ?2 AND j.status = 'done'`,
  )
    .bind(h.id, stage)
    .all<{ arm: string; seed: number; status: string; name: string | null; value: number | null }>();
  const pending = await c.env.DB.prepare(
    "SELECT COUNT(*) AS n FROM jobs WHERE hypothesis_id = ? AND stage_index = ? AND status IN ('queued','leased')",
  )
    .bind(h.id, stage)
    .first<{ n: number }>();
  const dead = await c.env.DB.prepare("SELECT COUNT(*) AS n FROM jobs WHERE hypothesis_id = ? AND stage_index = ? AND status = 'dead'")
    .bind(h.id, stage)
    .first<{ n: number }>();
  return json({ id: h.id, stage_index: stage, metric: h.metric, direction: h.direction, pending: pending?.n ?? 0, dead: dead?.n ?? 0, rows: results });
});

hypRoutes.post("/v1/hyp/:id/advance", requireAdmin, async (c) => {
  const h = await loadHyp(c.env, c.req.param("id"));
  if (h.status !== "running") throw new HttpError(409, `hypothesis is ${h.status}`);
  const b = await readJson(c.req.raw);
  const action = b.action;
  const variants = parseJson<Variant[]>(h.variants_json, []);
  if (b.variants !== undefined) {
    if (!Array.isArray(b.variants) || !b.variants.every((n) => variants.some((v) => v.name === n && v.active !== false)))
      throw new HttpError(400, "variants must name active variants");
    for (const v of variants) v.active = v.active !== false && (b.variants as string[]).includes(v.name);
    h.variants_json = JSON.stringify(variants);
  }
  const decisions = parseJson<unknown[]>(h.decisions_json, []);
  decisions.push({ at: nowIso(), stage_index: h.stage_index, action, variants: b.variants ?? null, evidence: isObject(b.evidence) ? b.evidence : null });
  const stages = parseJson<Stage[]>(h.stages_json, []);
  let stmts: D1PreparedStatement[] = [];
  let status = "running";
  let stageIndex = h.stage_index;
  if (action === "promote") {
    if (!variants.some((v) => v.active !== false)) throw new HttpError(400, "no active variant to promote");
    stageIndex += 1;
    if (stageIndex >= stages.length) status = "candidate";
    else stmts = await stageJobs(c.env, h, stageIndex, parseJson<number[]>(h.seeds_json, []), activeArms(h));
  } else if (action === "extend") {
    const seeds = parseSeeds(b.seeds, "seeds");
    stmts = await stageJobs(c.env, h, stageIndex, seeds, activeArms(h));
  } else if (action === "reject") {
    status = "rejected";
    stmts = [
      c.env.DB.prepare("UPDATE jobs SET status = 'cancelled', ended_at = ?2 WHERE hypothesis_id = ?1 AND status = 'queued'").bind(h.id, nowIso()),
    ];
  } else throw new HttpError(400, "action must be promote, extend or reject");
  await c.env.DB.batch([
    ...stmts,
    c.env.DB.prepare(
      "UPDATE hypotheses SET stage_index = ?2, status = ?3, variants_json = ?4, decisions_json = ?5, updated = ?6 WHERE id = ?1",
    ).bind(h.id, stageIndex, status, h.variants_json, JSON.stringify(decisions), nowIso()),
  ]);
  return json({ id: h.id, status, stage_index: stageIndex, stage: status === "running" ? stages[stageIndex].name : status, jobs: action === "reject" ? 0 : stmts.length });
});
