// Public replay reads for the dashboard (no token, like /v1/feed): a run's crane-traj (full or the overview LOD,
// header cut to public keys), a course's elements and boundary by truth hash, the same seed's runs from the same
// study (ghosts), and a study's fleet (index JSON plus one pack of LODs, `u32 len, bytes` repeated, the
// theater.traj.fleet_pack format). Everything is content addressed: a URL with ?h=<sha256> is cached immutably.
// Reads go through archive.ts's fetchArtifact: R2, else the archive node (503 while it is offline).
import { Hono } from "hono";
import { fetchArtifact } from "./archive";
import type { AppEnv } from "./auth";
import { scrubText } from "./purpose";
import { Env, HttpError, json, sha256Hex } from "./util";
// @ts-ignore: a plain ES module shared with the page (public/traj.js)
import { TrajError, parse, withHeader } from "../public/traj.js";

export const replayRoutes = new Hono<AppEnv>();

const PUBLIC = { "access-control-allow-origin": "*", "x-content-type-options": "nosniff" };
const IMMUTABLE = "public, max-age=31536000, immutable";
const RUN_ID = /^[A-Za-z0-9._-]{1,128}$/;
const SHA = /^[0-9a-f]{16,64}$/;
const HEADER_KEYS = ["schema", "hz", "t0_s", "ticks", "duration_s", "columns", "box", "pose_dims", "robot", "frame", "outcome", "progress", "valid", "score", "seed", "iteration", "lod", "parent_sha256"];

export interface ReplayRow {
  run_id: string;
  job_id: string;
  study: string;
  arm: string | null;
  seed: number | null;
  iteration: number;
  score: number | null;
  outcome: string;
  course_sha256: string | null;
  traj_key: string | null;
  sha256: string | null;
  deleted_at: string | null;
  r2_deleted_at: string | null;
  lod_key: string | null;
  lod_sha256: string | null;
}

const ROW_SQL = `SELECT m.run_id, m.job_id, m.study, m.arm, m.seed, m.iteration, m.score, m.outcome, m.course_sha256, m.traj_key,
    a.sha256, a.deleted_at, a.r2_deleted_at, l.key AS lod_key, l.sha256 AS lod_sha256
  FROM run_meta m
  JOIN runs r ON r.run_id = m.run_id
  JOIN jobs j ON j.job_id = r.job_id AND j.attempt = r.attempt
  LEFT JOIN artifacts a ON a.key = m.traj_key
  LEFT JOIN artifacts l ON l.key = replace(m.traj_key, '.ctraj', '.lod.ctraj') AND l.deleted_at IS NULL AND l.r2_deleted_at IS NULL`;

/** Replay state of a run_meta row: ready (in R2), archived (only the archive node has it), gone, or null. */
export const replayState = (r: { traj_key: string | null; sha256: string | null; deleted_at: string | null; r2_deleted_at: string | null }) =>
  !r.traj_key || !r.sha256 ? null : r.deleted_at ? "gone" : r.r2_deleted_at ? "archived" : "ready";

/** The run's public summary as the page's ghost and fleet lists carry it. */
const brief = (r: ReplayRow) => ({
  run_id: r.run_id,
  job_id: r.job_id,
  arm: scrubText(r.arm, 64),
  seed: r.seed,
  iteration: r.iteration,
  score: r.score,
  outcome: r.outcome,
  course: r.course_sha256,
  replay: replayState(r),
  traj_sha256: r.sha256,
  lod_sha256: r.lod_sha256,
});

/** A header that keeps only public keys: no params, paths, ids or build details; the course by hash and name. */
export function publicHeader(h: Record<string, unknown>): Record<string, unknown> {
  const out: Record<string, unknown> = {};
  for (const k of HEADER_KEYS) if (h[k] !== undefined) out[k] = h[k];
  const c = h.course as Record<string, unknown> | undefined;
  if (c && typeof c === "object") {
    const course: Record<string, unknown> = {};
    if (typeof c.truth_sha256 === "string") course.truth_sha256 = c.truth_sha256;
    for (const k of ["name", "family"]) if (typeof c[k] === "string") course[k] = scrubText(c[k] as string, 100);
    out.course = course;
  }
  if (typeof h.study === "string") out.study = scrubText(h.study, 100);
  return out;
}

/** The public bytes of a stored ctraj (its header cut by publicHeader), or null when it is gone everywhere. */
async function publicTraj(env: Env, key: string, url: string): Promise<Uint8Array | null> {
  let res: Response;
  try {
    res = await fetchArtifact(env, key, new Request(url));
  } catch (e) {
    if (e instanceof HttpError && e.status === 404) return null;
    throw e; // 503: archived, the archive node is offline
  }
  const bytes = new Uint8Array(await res.arrayBuffer());
  try {
    const { header } = parse(bytes, false);
    return withHeader(bytes, publicHeader(header));
  } catch (e) {
    if (e instanceof TrajError) throw new HttpError(422, `stored trajectory is unreadable: ${(e as Error).message}`);
    throw e;
  }
}

/** Serves through the colo cache: `make` runs only on a miss. */
async function edgeCached(url: string, make: () => Promise<Response>): Promise<Response> {
  const cache = typeof caches !== "undefined" ? (caches as unknown as { default: Cache }).default : null;
  const key = new Request(url);
  const hit = cache && (await cache.match(key));
  if (hit) return hit;
  const res = await make();
  if (cache && res.ok && /max-age=[1-9]/.test(res.headers.get("cache-control") ?? "")) await cache.put(key, res.clone());
  return res;
}

async function runRow(env: Env, runId: string): Promise<ReplayRow> {
  if (!RUN_ID.test(runId)) throw new HttpError(400, "bad run id");
  const r = await env.DB.prepare(`${ROW_SQL} WHERE m.run_id = ?1`).bind(runId).first<ReplayRow>();
  if (!r || !r.traj_key || !r.sha256) throw new HttpError(404, "no trajectory for this run");
  return r;
}

const bin = (body: Uint8Array, cache: string, extra: Record<string, string> = {}) =>
  new Response(body, { headers: { "content-type": "application/octet-stream", "content-length": String(body.length), "cache-control": cache, ...PUBLIC, ...extra } });

/** GET /v1/feed/runs/:id/traj[?lod=1][&h=sha256]: the run's ctraj; lod=1 serves run.lod.ctraj when the worker wrote one. */
replayRoutes.get("/v1/feed/runs/:id/traj", async (c) => {
  const url = new URL(c.req.url);
  return edgeCached(url.toString(), async () => {
    const r = await runRow(c.env, c.req.param("id"));
    if (r.deleted_at) return json({ error: "gone", hint: "No copy of this trajectory is kept any more." }, 410, PUBLIC);
    const wantLod = url.searchParams.get("lod") === "1" && !!r.lod_key;
    const key = wantLod ? r.lod_key! : r.traj_key!;
    const sha = wantLod ? r.lod_sha256 : r.sha256;
    const body = await publicTraj(c.env, key, c.req.url);
    if (!body) return json({ error: "gone" }, 410, PUBLIC);
    const h = url.searchParams.get("h");
    return bin(body, h && h === sha ? IMMUTABLE : "public, max-age=60", { etag: `"${sha}"`, "x-crane-lod": wantLod ? "1" : "0" });
  });
});

/** Course truth reduced to what a 2-D replay draws. */
export function reduceCourse(t: Record<string, any>) {
  const n = (v: unknown) => (typeof v === "number" && Number.isFinite(v) ? Math.round(v * 1000) / 1000 : 0);
  return {
    name: scrubText(t.name, 100),
    family: scrubText(t.family, 100),
    start: [n(t.startX), n(t.startY), n(t.startYaw)],
    boundary: Array.isArray(t.boundary) ? t.boundary.filter(Array.isArray).map((p: unknown[]) => [n(p[0]), n(p[1])]) : [],
    elements: (Array.isArray(t.elements) ? t.elements : []).map((e: Record<string, unknown>) => ({
      x: n(e.x),
      y: n(e.y),
      r: n(e.radius ?? e.r ?? 0.25) || 0.25,
      label: scrubText(String(e.label ?? ""), 60) ?? "",
      role: scrubText(String(e.role ?? ""), 30) ?? "",
      task: scrubText(String(e.task ?? ""), 60) ?? "",
    })),
  };
}

/**
 * GET /v1/feed/courses/:truth: {name, family, start, boundary, elements[{x,y,r,label,role,task}]}. Read from
 * R2 truths/<sha> (the content-addressed truth a worker uploads once), else from a run's course_truth.json
 * artifact; the reduced form is kept at courses/<sha>.json, so it outlives raw retention.
 */
replayRoutes.get("/v1/feed/courses/:truth", async (c) => {
  const truth = c.req.param("truth");
  if (!SHA.test(truth)) throw new HttpError(400, "bad truth hash");
  return edgeCached(new URL(c.req.url).toString(), async () => {
    const env = c.env;
    const reduced = await env.BUCKET.get(`courses/${truth}.json`);
    if (reduced) return new Response(reduced.body, { headers: { "content-type": "application/json; charset=utf-8", "cache-control": IMMUTABLE, ...PUBLIC } });
    let src = await env.BUCKET.get(`truths/${truth}`);
    if (!src) {
      const a = await env.DB.prepare(
        `SELECT a.key FROM run_meta m JOIN artifacts a ON a.run_id = m.run_id AND a.name = 'course_truth.json' AND a.deleted_at IS NULL
          WHERE m.course_sha256 = ?1 LIMIT 1`,
      )
        .bind(truth)
        .first<{ key: string }>();
      src = a ? await env.BUCKET.get(a.key) : null;
    }
    if (!src) return json({ error: "course not stored" }, 404, { ...PUBLIC, "cache-control": "public, max-age=300" });
    let body: string;
    try {
      body = JSON.stringify(reduceCourse(JSON.parse(await src.text())));
    } catch {
      throw new HttpError(422, "stored course truth is not JSON");
    }
    await env.BUCKET.put(`courses/${truth}.json`, body, { httpMetadata: { contentType: "application/json" } });
    return new Response(body, { headers: { "content-type": "application/json; charset=utf-8", "cache-control": IMMUTABLE, ...PUBLIC } });
  });
});

const intq = (v: string | undefined, d: number, lo: number, hi: number, name: string) => {
  if (v === undefined || v === "") return d;
  const n = Number(v);
  if (!Number.isSafeInteger(n) || n < lo || n > hi) throw new HttpError(400, `${name} must be an integer in [${lo}, ${hi}]`);
  return n;
};

/** GET /v1/feed/ghosts?run=ID | study=S&seed=N [&limit=8]: other runs of the same seed in the same study. */
replayRoutes.get("/v1/feed/ghosts", async (c) => {
  const limit = intq(c.req.query("limit"), 8, 1, 32, "limit");
  let study = c.req.query("study") ?? null;
  let seed: number | null = c.req.query("seed") === undefined ? null : intq(c.req.query("seed"), 0, -(2 ** 31), 2 ** 31, "seed");
  const runId = c.req.query("run") ?? null;
  if (runId) {
    const r = await runRow(c.env, runId);
    study = r.study;
    seed = r.seed;
  }
  if (!study || seed === null) throw new HttpError(400, "run, or study and seed, required");
  const { results } = await c.env.DB.prepare(
    `${ROW_SQL} WHERE m.study = ?1 AND m.seed = ?2 AND (?3 IS NULL OR m.run_id != ?3) AND a.sha256 IS NOT NULL AND a.deleted_at IS NULL AND a.r2_deleted_at IS NULL
      ORDER BY m.iteration DESC, (m.score IS NULL), m.score DESC, m.run_id LIMIT ?4`,
  )
    .bind(study, seed, runId, limit)
    .all<ReplayRow>();
  return json({ study: scrubText(study, 100), seed, runs: results.map(brief) }, 200, { ...PUBLIC, "cache-control": "public, max-age=30" });
});

/** The fleet selection: per iteration the best `k` by score plus every failure, newest iterations first, capped. */
async function fleetRows(env: Env, q: (k: string) => string | undefined): Promise<{ study: string; rows: ReplayRow[] }> {
  const study = q("study");
  if (!study || study.length > 200) throw new HttpError(400, "study required");
  const k = intq(q("k"), 5, 1, 100, "k");
  const limit = intq(q("limit"), 120, 1, 300, "limit");
  const { results } = await env.DB.prepare(
    `SELECT * FROM (
       SELECT x.*, ROW_NUMBER() OVER (PARTITION BY x.iteration ORDER BY (x.score IS NULL), x.score DESC, x.run_id) AS rk
         FROM (${ROW_SQL} WHERE m.study = ?1 AND a.sha256 IS NOT NULL AND a.deleted_at IS NULL AND a.r2_deleted_at IS NULL) x)
      WHERE rk <= ?2 OR outcome IN ('failed', 'timeout')
      ORDER BY iteration DESC, rk LIMIT ?3`,
  )
    .bind(study, k, limit)
    .all<ReplayRow>();
  return { study, rows: results.sort((a, b) => a.iteration - b.iteration || (b.score ?? -Infinity) - (a.score ?? -Infinity)) };
}

/** GET /v1/feed/fleet?study=S[&k=5][&limit=120]: the fleet's runs (iteration, score, outcome, hashes). */
replayRoutes.get("/v1/feed/fleet", async (c) => {
  const { study, rows } = await fleetRows(c.env, (k) => c.req.query(k));
  const its = [...new Set(rows.map((r) => r.iteration))];
  return json(
    { study: scrubText(study, 100), iterations: its, runs: rows.map(brief), pack_sha256: await sha256Hex(rows.map((r) => r.lod_sha256 ?? r.sha256).join(",")) },
    200,
    { ...PUBLIC, "cache-control": "public, max-age=30" },
  );
});

/** GET /v1/feed/fleet.pack?(same query)[&h=pack_sha256]: the fleet's LODs (full files where no LOD exists) in index order. */
replayRoutes.get("/v1/feed/fleet.pack", async (c) => {
  const url = new URL(c.req.url);
  return edgeCached(url.toString(), async () => {
    const { rows } = await fleetRows(c.env, (k) => c.req.query(k));
    const files: Uint8Array[] = [];
    for (let i = 0; i < rows.length; i += 6) {
      const got = await Promise.all(rows.slice(i, i + 6).map((r) => publicTraj(c.env, r.lod_key ?? r.traj_key!, c.req.url).catch(() => null)));
      for (const g of got) files.push(g ?? new Uint8Array(0)); // an empty entry keeps the index order
    }
    const total = files.reduce((s, f) => s + 4 + f.length, 0);
    const out = new Uint8Array(total);
    const v = new DataView(out.buffer);
    let p = 0;
    for (const f of files) {
      v.setUint32(p, f.length, true);
      out.set(f, p + 4);
      p += 4 + f.length;
    }
    const h = url.searchParams.get("h");
    const sha = await sha256Hex(rows.map((r) => r.lod_sha256 ?? r.sha256).join(","));
    return bin(out, h && h === sha ? IMMUTABLE : "public, max-age=60");
  });
});
