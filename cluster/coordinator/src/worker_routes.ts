// Routes used by worker daemons: join, lease, heartbeat, release, runs, completion.
import { Hono } from "hono";
import type { AppEnv } from "./auth";
import { CFG, getConfig } from "./db";
import type { HubRes } from "./hub";
import { hubStub } from "./hubstub";
import { sanitizeCaps, sanitizePolicy } from "./match";
import { HttpError, bearer, json, nowIso, randomId, randomToken, readJson, sha256Hex } from "./util";

export const workerRoutes = new Hono<AppEnv>();

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
  await hubStub(c.env).syncWorkers([workerId]);
  return json({ worker_id: workerId, token });
});

/** The bearer token's hash (the Hub keys workers by it), or a 401. */
async function tokSha(req: Request): Promise<string> {
  const tok = bearer(req);
  if (!tok || tok.length > 256) throw new HttpError(401, "unauthorized");
  return sha256Hex(tok);
}
const send = (r: HubRes) => new Response(r.body, { status: r.status, headers: { "content-type": "application/json; charset=utf-8" } });

// Served from the Hub's memory: no D1 read per poll (src/hub.ts).
workerRoutes.get("/v1/repos", async (c) => send(await hubStub(c.env).reposFor(await tokSha(c.req.raw))));

workerRoutes.post("/v1/lease", async (c) =>
  send(await hubStub(c.env).lease(await tokSha(c.req.raw), await readJson(c.req.raw), c.req.header("x-client-version") ?? null)),
);

workerRoutes.post("/v1/heartbeat", async (c) => send(await hubStub(c.env).heartbeat(await tokSha(c.req.raw), await readJson(c.req.raw))));

workerRoutes.post("/v1/release", async (c) => send(await hubStub(c.env).release(await tokSha(c.req.raw), await readJson(c.req.raw))));

workerRoutes.post("/v1/jobs/:id/runs", async (c) =>
  send(await hubStub(c.env).reportRun(await tokSha(c.req.raw), c.req.param("id"), await readJson(c.req.raw))),
);

workerRoutes.post("/v1/jobs/:id/complete", async (c) =>
  send(await hubStub(c.env).complete(await tokSha(c.req.raw), c.req.param("id"), await readJson(c.req.raw))),
);
