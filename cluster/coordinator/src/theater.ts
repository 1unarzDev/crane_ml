// Theater renders from the dashboard. The public page shows each job's and study's render requests and plays
// finished videos (GET /v1/feed/renders, GET /v1/feed/renders/:id/video with Range). Its "Render in Unity
// theater" button needs an operator token: POST /v1/operator/renders records a request with a purpose. The
// render itself stays `python3 -m theater.render` (a cluster job with its own purpose): whoever runs it lists
// GET /v1/admin/renders?status=requested, then reports progress and uploads the video through the admin routes.
import { Hono } from "hono";
import { AppEnv, UNAUTHORIZED, operatorName, requireAdmin } from "./auth";
import { checkPurpose, scrubText } from "./purpose";
import { HttpError, intVar, json, nowIso, randomId, randomToken, readJson, sha256Hex } from "./util";

export const theaterRoutes = new Hono<AppEnv>();

const PUBLIC = { "access-control-allow-origin": "*", "x-content-type-options": "nosniff" };
const ID = /^[A-Za-z0-9._-]{1,128}$/;
const STATUSES = new Set(["requested", "rendering", "done", "failed", "cancelled"]);
const VIDEO_TYPES: Record<string, string> = { mp4: "video/mp4", webm: "video/webm", gif: "image/gif" };

interface RenderRow {
  id: string;
  job_id: string | null;
  study: string | null;
  run_id: string | null;
  purpose: string;
  requested_by: string;
  status: string;
  note: string | null;
  video_key: string | null;
  video_type: string | null;
  video_size: number | null;
  created: string;
  updated: string;
}

const view = (r: RenderRow) => ({
  id: r.id,
  job_id: r.job_id,
  study: scrubText(r.study, 100),
  run_id: r.run_id,
  purpose: scrubText(r.purpose),
  requested_by: scrubText(r.requested_by, 64),
  status: r.status,
  note: scrubText(r.note, 200),
  video: r.video_key ? { url: `/v1/feed/renders/${r.id}/video`, type: r.video_type, size: r.video_size } : null,
  created_at: r.created,
  updated_at: r.updated,
});

/** GET /v1/feed/renders?job=ID | study=S: render requests and videos, newest first (public). */
theaterRoutes.get("/v1/feed/renders", async (c) => {
  const job = c.req.query("job") ?? null;
  const study = c.req.query("study") ?? null;
  if (job !== null && !ID.test(job)) throw new HttpError(400, "bad job id");
  if (study !== null && study.length > 200) throw new HttpError(400, "bad study");
  const { results } = await c.env.DB.prepare(
    `SELECT * FROM theater_renders WHERE (?1 IS NULL OR job_id = ?1) AND (?2 IS NULL OR study = ?2) AND status != 'cancelled'
      ORDER BY created DESC LIMIT 20`,
  )
    .bind(job, study)
    .all<RenderRow>();
  return json({ renders: results.map(view) }, 200, { ...PUBLIC, "cache-control": "public, max-age=10" });
});

/** GET /v1/feed/renders/:id/video: the render's video from R2, with a single Range for seeking (public). */
theaterRoutes.get("/v1/feed/renders/:id/video", async (c) => {
  const id = c.req.param("id");
  if (!ID.test(id)) throw new HttpError(400, "bad render id");
  const r = await c.env.DB.prepare("SELECT video_key, video_type, video_size FROM theater_renders WHERE id = ?1").bind(id).first<RenderRow>();
  if (!r?.video_key) throw new HttpError(404, "no video for this render");
  const m = /^bytes=(\d*)-(\d*)$/.exec(c.req.header("range") ?? "");
  const size = r.video_size ?? 0;
  let range: { offset: number; length: number } | undefined;
  if (m && size) {
    const start = m[1] === "" ? Math.max(0, size - Number(m[2])) : Number(m[1]);
    const end = m[1] === "" || m[2] === "" ? size - 1 : Math.min(size - 1, Number(m[2]));
    if (start > end || start >= size) return new Response(null, { status: 416, headers: { "content-range": `bytes */${size}`, ...PUBLIC } });
    range = { offset: start, length: end - start + 1 };
  }
  const obj = await c.env.BUCKET.get(r.video_key, range ? { range } : undefined);
  if (!obj) return json({ error: "archived", tier: "archive" }, 410, PUBLIC);
  const headers: Record<string, string> = {
    "content-type": r.video_type ?? "application/octet-stream",
    "accept-ranges": "bytes",
    "cache-control": "public, max-age=86400",
    ...PUBLIC,
  };
  if (range) {
    headers["content-range"] = `bytes ${range.offset}-${range.offset + range.length - 1}/${size}`;
    headers["content-length"] = String(range.length);
    return new Response(obj.body, { status: 206, headers });
  }
  headers["content-length"] = String(obj.size);
  return new Response(obj.body, { headers });
});

/** GET /v1/operator/whoami: 200 {name} for an operator or admin token (the page enables its render button). */
theaterRoutes.get("/v1/operator/whoami", async (c) => {
  const name = await operatorName(c.env, c.req.raw);
  if (!name) throw UNAUTHORIZED();
  return json({ name });
});

/** POST /v1/operator/renders {job_id | study, run_id?, purpose}: a theater render request (operator token). */
theaterRoutes.post("/v1/operator/renders", async (c) => {
  const name = await operatorName(c.env, c.req.raw);
  if (!name) throw UNAUTHORIZED();
  const b = await readJson(c.req.raw);
  const purpose = checkPurpose(b.purpose);
  const job = typeof b.job_id === "string" ? b.job_id : null;
  const study = typeof b.study === "string" ? b.study.slice(0, 200) : null;
  const run = typeof b.run_id === "string" ? b.run_id : null;
  if (!job && !study) throw new HttpError(400, "job_id or study required");
  for (const v of [job, run]) if (v !== null && !ID.test(v)) throw new HttpError(400, "bad id");
  if (job && !(await c.env.DB.prepare("SELECT 1 FROM jobs WHERE job_id = ?1").bind(job).first())) throw new HttpError(404, "unknown job");
  const open = await c.env.DB.prepare(
    "SELECT id FROM theater_renders WHERE status IN ('requested','rendering') AND job_id IS ?1 AND study IS ?2 AND run_id IS ?3",
  )
    .bind(job, study, run)
    .first<{ id: string }>();
  if (open) return json({ id: open.id, status: "already requested" });
  const id = randomId("r_");
  const now = nowIso();
  await c.env.DB.prepare(
    `INSERT INTO theater_renders(id, job_id, study, run_id, purpose, requested_by, created, updated) VALUES (?1, ?2, ?3, ?4, ?5, ?6, ?7, ?7)`,
  )
    .bind(id, job, study, run, purpose, name, now)
    .run();
  return json({ id, status: "requested" });
});

// ---- admin: mint operator tokens, work the queue, attach videos

theaterRoutes.post("/v1/admin/operator_token", requireAdmin, async (c) => {
  const b = await readJson<{ name?: unknown }>(c.req.raw);
  const name = typeof b.name === "string" && b.name.trim() ? b.name.trim().slice(0, 64) : "operator";
  const token = randomToken();
  await c.env.DB.prepare("INSERT INTO operator_tokens(token_sha256, name, created) VALUES (?, ?, ?)").bind(await sha256Hex(token), name, nowIso()).run();
  return json({ token, name });
});

theaterRoutes.get("/v1/admin/renders", requireAdmin, async (c) => {
  const status = c.req.query("status") ?? null;
  if (status !== null && !STATUSES.has(status)) throw new HttpError(400, `status must be one of ${[...STATUSES]}`);
  const { results } = await c.env.DB.prepare("SELECT * FROM theater_renders WHERE ?1 IS NULL OR status = ?1 ORDER BY created LIMIT 200")
    .bind(status)
    .all<RenderRow>();
  return json({ renders: results.map((r) => ({ ...view(r), render_job_id: (r as unknown as Record<string, unknown>).render_job_id ?? null })) });
});

/** POST /v1/admin/renders/:id {status?, note?, render_job_id?} */
theaterRoutes.post("/v1/admin/renders/:id", requireAdmin, async (c) => {
  const b = await readJson(c.req.raw);
  if (b.status !== undefined && (typeof b.status !== "string" || !STATUSES.has(b.status))) throw new HttpError(400, `status must be one of ${[...STATUSES]}`);
  const r = await c.env.DB.prepare(
    `UPDATE theater_renders SET status = COALESCE(?2, status), note = COALESCE(?3, note), render_job_id = COALESCE(?4, render_job_id), updated = ?5
      WHERE id = ?1`,
  )
    .bind(c.req.param("id"), b.status ?? null, typeof b.note === "string" ? b.note.slice(0, 200) : null, typeof b.render_job_id === "string" ? b.render_job_id : null, nowIso())
    .run();
  if (!r.meta.changes) throw new HttpError(404, "unknown render");
  return json({ ok: true });
});

/** PUT /v1/admin/renders/:id/video/<name>.(mp4|webm|gif): stores the video in R2 and marks the render done. */
theaterRoutes.put("/v1/admin/renders/:id/video/:name", requireAdmin, async (c) => {
  const id = c.req.param("id");
  const name = c.req.param("name");
  const ext = /^[A-Za-z0-9._-]{1,80}\.(mp4|webm|gif)$/.exec(name)?.[1];
  if (!ext) throw new HttpError(400, "name must end in .mp4, .webm or .gif");
  if (!(await c.env.DB.prepare("SELECT 1 FROM theater_renders WHERE id = ?1").bind(id).first())) throw new HttpError(404, "unknown render");
  const max = intVar(c.env.MAX_ARTIFACT_MB, 95) * 1024 * 1024;
  const body = await c.req.arrayBuffer();
  if (body.byteLength > max) throw new HttpError(413, `video larger than ${max} bytes`);
  const sha = await sha256Hex(body);
  const key = `renders/${id}/${name}`;
  await c.env.BUCKET.put(key, body, { httpMetadata: { contentType: VIDEO_TYPES[ext] }, customMetadata: { sha256: sha } });
  await c.env.DB.prepare(
    `UPDATE theater_renders SET video_key = ?2, video_type = ?3, video_size = ?4, video_sha256 = ?5, status = 'done', updated = ?6 WHERE id = ?1`,
  )
    .bind(id, key, VIDEO_TYPES[ext], body.byteLength, sha, nowIso())
    .run();
  return json({ key, sha256: sha, size: body.byteLength, url: `/v1/feed/renders/${id}/video` });
});
