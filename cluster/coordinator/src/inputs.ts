// Job inputs: content-addressed blobs in R2 (inputs/<sha256>) that an admin uploads before creating a job and
// any worker (or viewer) reads by hash, e.g. a theater-render job's manifest, track pack and course truth.
// Inputs are kept (no expiry): they are what a job ran on.
import { Hono, MiddlewareHandler } from "hono";
import { AppEnv, UNAUTHORIZED, requireAdmin, requireViewer, requireWorker } from "./auth";
import { HttpError, bearer, intVar, json, sha256Hex } from "./util";

export const inputRoutes = new Hono<AppEnv>();

const SHA = /^[0-9a-f]{64}$/;

inputRoutes.put("/v1/admin/inputs/:sha", requireAdmin, async (c) => {
  const want = c.req.param("sha");
  if (!SHA.test(want)) throw new HttpError(400, "input key must be a lowercase sha256");
  const maxBytes = intVar(c.env.MAX_ARTIFACT_MB, 95) * 1024 * 1024;
  if (Number(c.req.header("content-length") ?? "0") > maxBytes) throw new HttpError(413, `input larger than ${maxBytes} bytes`);
  const body = await c.req.arrayBuffer();
  if (body.byteLength > maxBytes) throw new HttpError(413, `input larger than ${maxBytes} bytes`);
  const sha = await sha256Hex(body);
  if (sha !== want) throw new HttpError(400, "body does not match its sha256");
  const name = (c.req.query("name") ?? "").slice(0, 200);
  await c.env.BUCKET.put(`inputs/${sha}`, body, { customMetadata: { sha256: sha, name } });
  return json({ key: `inputs/${sha}`, sha256: sha, size: body.byteLength });
});

/** A worker token, or a viewer/admin token. */
const requireWorkerOrViewer: MiddlewareHandler<AppEnv> = async (c, next) => {
  if (!bearer(c.req.raw)) throw UNAUTHORIZED();
  try {
    await requireWorker(c, async () => {});
    return next();
  } catch {
    return requireViewer(c, next);
  }
};

inputRoutes.get("/v1/inputs/:sha", requireWorkerOrViewer, async (c) => {
  const sha = c.req.param("sha");
  if (!SHA.test(sha)) throw new HttpError(400, "input key must be a lowercase sha256");
  const obj = await c.env.BUCKET.get(`inputs/${sha}`);
  if (!obj) throw new HttpError(404, "not found");
  return new Response(obj.body, {
    headers: { "content-type": "application/octet-stream", "content-length": String(obj.size), "x-content-sha256": sha },
  });
});
