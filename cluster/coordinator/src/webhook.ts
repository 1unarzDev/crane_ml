// GitHub push webhook: a push to a repo's main branch sets repos.main_version.
// Workers see the new version in every lease response and update their caches between jobs.
import { Hono } from "hono";
import type { AppEnv } from "./auth";
import { HttpError, constantTimeEqual, hmacSha256Hex, isObject, json, nowIso } from "./util";

export const webhookRoutes = new Hono<AppEnv>();

const SHA = /^[0-9a-f]{40}$/;

webhookRoutes.post("/v1/webhook/github", async (c) => {
  const secret = c.env.GITHUB_WEBHOOK_SECRET;
  if (!secret) throw new HttpError(503, "webhook secret not configured");
  const body = await c.req.arrayBuffer();
  const sig = c.req.header("x-hub-signature-256") ?? "";
  if (!constantTimeEqual(sig, `sha256=${await hmacSha256Hex(secret, body)}`)) throw new HttpError(401, "unauthorized");
  const event = c.req.header("x-github-event") ?? "";
  if (event === "ping") return json({ ok: true, event });
  if (event !== "push") return json({ ok: true, ignored: event });
  let p: unknown;
  try {
    p = JSON.parse(new TextDecoder().decode(body));
  } catch {
    throw new HttpError(400, "invalid JSON body");
  }
  if (!isObject(p) || !isObject(p.repository)) throw new HttpError(400, "not a push payload");
  const repo = p.repository;
  const name = typeof repo.full_name === "string" ? repo.full_name : null;
  const after = typeof p.after === "string" ? p.after : "";
  if (!name) throw new HttpError(400, "repository.full_name missing");
  const known = await c.env.DB.prepare("SELECT main_branch FROM repos WHERE name = ?").bind(name).first<{ main_branch: string }>();
  const branch = known?.main_branch ?? (typeof repo.default_branch === "string" ? repo.default_branch : "main");
  if (p.ref !== `refs/heads/${branch}` || !SHA.test(after) || /^0+$/.test(after)) return json({ ok: true, ignored: p.ref });
  const url = typeof repo.clone_url === "string" ? repo.clone_url : null;
  await c.env.DB.prepare(
    `INSERT INTO repos(name, url, main_branch, main_version, updated_at) VALUES (?1, ?2, ?3, ?4, ?5)
     ON CONFLICT(name) DO UPDATE SET main_version = excluded.main_version, updated_at = excluded.updated_at,
       url = COALESCE(repos.url, excluded.url)`,
  )
    .bind(name, url, branch, after, nowIso())
    .run();
  return json({ ok: true, repo: name, main_version: after });
});
