// R2 artifacts, proxied through the Worker (no S3 credentials needed).
// Retention classes: 'summary' is kept forever; 'raw' expires after N days
// (config raw_retention_days, else RAW_RETENTION_DAYS) unless pinned. With an archive node, both leave R2
// after their kind's hot window once archived, and reads fall back to the archive (archive.ts).
import { Hono } from "hono";
import { fetchArtifact, kindOf } from "./archive";
import { AppEnv, requireAdmin, requireReader, requireViewer } from "./auth";
import { rawRetentionDays } from "./db";
import { hubStub } from "./hubstub";
import { HttpError, bearer, intVar, json, nowIso, readJson, sha256Hex } from "./util";

export { sweepArtifacts, type SweepResult } from "./archive";

export const artifactRoutes = new Hono<AppEnv>();

const ART_NAME = /^[A-Za-z0-9._-][A-Za-z0-9._/-]{0,199}$/;
const DAY_MS = 86_400_000;

artifactRoutes.put("/v1/jobs/:id/artifacts/*", async (c) => {
  const jobId = c.req.param("id");
  const tok = bearer(c.req.raw);
  if (!tok || tok.length > 256) throw new HttpError(401, "unauthorized");
  const held = await hubStub(c.env).held(await sha256Hex(tok), jobId); // the Hub's memory, no D1 read
  if (held.status !== 200) return new Response(held.body, { status: held.status, headers: { "content-type": "application/json; charset=utf-8" } });
  const prefix = `/v1/jobs/${jobId}/artifacts/`;
  const name = decodeURIComponent(new URL(c.req.url).pathname.slice(prefix.length));
  if (!ART_NAME.test(name) || name.split("/").some((p) => p === ".." || p === "")) throw new HttpError(400, "bad artifact name");
  const retention = c.req.query("retention") ?? "raw";
  if (retention !== "raw" && retention !== "summary") throw new HttpError(400, "retention must be raw or summary");
  const run = c.req.query("run") ?? null;
  if (run !== null && !run.startsWith(`${jobId}.`)) throw new HttpError(400, "run must belong to this job");
  const maxBytes = intVar(c.env.MAX_ARTIFACT_MB, 95) * 1024 * 1024;
  const declared = Number(c.req.header("content-length") ?? "0");
  if (declared > maxBytes) throw new HttpError(413, `artifact larger than ${maxBytes} bytes`);
  const body = await c.req.arrayBuffer();
  if (body.byteLength > maxBytes) throw new HttpError(413, `artifact larger than ${maxBytes} bytes`);
  const sha = await sha256Hex(body);
  const want = c.req.header("x-content-sha256");
  if (want && want.toLowerCase() !== sha) throw new HttpError(400, "body does not match X-Content-Sha256");
  const key = `artifacts/${jobId}/${run ?? "job"}/${name}`;
  const expires = retention === "raw" ? nowIso(Date.now() + (await rawRetentionDays(c.env)) * DAY_MS) : null;
  await c.env.BUCKET.put(key, body, { customMetadata: { sha256: sha, job_id: jobId, retention } });
  await c.env.DB.prepare(
    `INSERT INTO artifacts(key, job_id, run_id, name, retention, size, sha256, expires_at, created, kind)
     VALUES (?1, ?2, ?3, ?4, ?5, ?6, ?7, ?8, ?9, ?10)
     ON CONFLICT(key) DO UPDATE SET
       archived_at = CASE WHEN artifacts.sha256 = excluded.sha256 THEN artifacts.archived_at END,
       archive_bytes = CASE WHEN artifacts.sha256 = excluded.sha256 THEN artifacts.archive_bytes END,
       archive_codec = CASE WHEN artifacts.sha256 = excluded.sha256 THEN artifacts.archive_codec END,
       size = excluded.size, sha256 = excluded.sha256, retention = excluded.retention, kind = excluded.kind,
       expires_at = CASE WHEN artifacts.pinned = 1 THEN NULL ELSE excluded.expires_at END, deleted_at = NULL,
       r2_deleted_at = NULL, hot_at = excluded.created`,
  )
    .bind(key, jobId, run, name, retention, body.byteLength, sha, expires, nowIso(), kindOf(name, retention))
    .run();
  return json({ key, sha256: sha, size: body.byteLength, retention, expires_at: expires });
});

artifactRoutes.get("/v1/artifacts", requireViewer, async (c) => {
  const job = c.req.query("job");
  if (!job) throw new HttpError(400, "job query parameter required");
  const { results } = await c.env.DB.prepare(
    `SELECT key, run_id, name, retention, kind, size, sha256, pinned, expires_at, created,
            CASE WHEN r2_deleted_at IS NULL THEN 'hot' ELSE 'archived' END AS tier
       FROM artifacts WHERE job_id = ? AND deleted_at IS NULL ORDER BY key`,
  )
    .bind(job)
    .all();
  return json({ artifacts: results });
});

/** One artifact (Range supported): from R2, else from the archive node (503 with Retry-After when offline). */
artifactRoutes.get("/v1/artifact/*", requireReader, async (c) => {
  const key = decodeURIComponent(new URL(c.req.url).pathname.slice("/v1/artifact/".length));
  if (!key.startsWith("artifacts/")) throw new HttpError(404, "not found");
  try {
    return await fetchArtifact(c.env, key, c.req.raw);
  } catch (e) {
    if (e instanceof HttpError && e.status === 503) {
      return json({ error: e.message, ...e.extra }, 503, { "retry-after": "300" });
    }
    throw e;
  }
});

/** Pin or unpin raw artifacts by key or by job. Pinned artifacts never expire; unpinning restarts the clock. */
artifactRoutes.post("/v1/admin/artifacts/pin", requireAdmin, async (c) => {
  const b = await readJson(c.req.raw);
  const pinned = b.pinned !== false;
  if (typeof b.key !== "string" && typeof b.job_id !== "string") throw new HttpError(400, "key or job_id required");
  const expires = nowIso(Date.now() + (await rawRetentionDays(c.env)) * DAY_MS);
  const where = typeof b.key === "string" ? "key = ?3" : "job_id = ?3";
  const r = await c.env.DB.prepare(
    `UPDATE artifacts SET pinned = ?1, expires_at = CASE WHEN retention = 'summary' OR ?1 = 1 THEN NULL ELSE ?2 END
      WHERE ${where} AND deleted_at IS NULL`,
  )
    .bind(pinned ? 1 : 0, expires, b.key ?? b.job_id)
    .run();
  return json({ updated: r.meta.changes, pinned });
});
