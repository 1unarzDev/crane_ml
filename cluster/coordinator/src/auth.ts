import type { MiddlewareHandler } from "hono";
import { Env, HttpError, bearer, sha256Hex } from "./util";

export interface WorkerRow {
  id: string;
  name: string;
  caps_json: string;
  policy_json: string;
  client_version: string | null;
}

export type AppEnv = {
  Bindings: Env;
  Variables: { worker: WorkerRow };
};

// Every auth failure produces this exact response, so callers can't learn
// which token kinds exist or whether a token is valid elsewhere.
export const UNAUTHORIZED = () => new HttpError(401, "unauthorized");

async function isAdminToken(env: Env, tok: string): Promise<boolean> {
  return !!env.ADMIN_TOKEN && (await sha256Hex(tok)) === (await sha256Hex(env.ADMIN_TOKEN));
}

export const requireWorker: MiddlewareHandler<AppEnv> = async (c, next) => {
  const tok = bearer(c.req.raw);
  if (!tok) throw UNAUTHORIZED();
  const row = await c.env.DB.prepare(
    "SELECT id, name, caps_json, policy_json, client_version FROM workers WHERE token_sha256 = ? AND disabled = 0",
  )
    .bind(await sha256Hex(tok))
    .first<WorkerRow>();
  if (!row) throw UNAUTHORIZED();
  c.set("worker", row);
  await next();
};

export const requireAdmin: MiddlewareHandler<AppEnv> = async (c, next) => {
  const tok = bearer(c.req.raw);
  if (!tok || !(await isAdminToken(c.env, tok))) throw UNAUTHORIZED();
  await next();
};

/** Read-only views: a viewer token (POST /v1/admin/viewer_token) or the admin token. */
export const requireViewer: MiddlewareHandler<AppEnv> = async (c, next) => {
  const tok = bearer(c.req.raw);
  if (!tok || tok.length > 256) throw UNAUTHORIZED();
  if (await isAdminToken(c.env, tok)) return next();
  const row = await c.env.DB.prepare("SELECT name FROM viewer_tokens WHERE token_sha256 = ?").bind(await sha256Hex(tok)).first();
  if (!row) throw UNAUTHORIZED();
  await next();
};

async function isArchiveToken(env: Env, tok: string): Promise<boolean> {
  const row = await env.DB.prepare("SELECT name FROM archive_tokens WHERE token_sha256 = ?").bind(await sha256Hex(tok)).first();
  return !!row;
}

/** An archive node (crane-archive): /v1/archive/* and artifact reads, nothing else. The admin token passes too. */
export const requireArchive: MiddlewareHandler<AppEnv> = async (c, next) => {
  const tok = bearer(c.req.raw);
  if (!tok || tok.length > 256) throw UNAUTHORIZED();
  if ((await isAdminToken(c.env, tok)) || (await isArchiveToken(c.env, tok))) return next();
  throw UNAUTHORIZED();
};

/** Artifact reads: a viewer, the admin or an archive node. */
export const requireReader: MiddlewareHandler<AppEnv> = async (c, next) => {
  const tok = bearer(c.req.raw);
  if (!tok || tok.length > 256) throw UNAUTHORIZED();
  if ((await isAdminToken(c.env, tok)) || (await isArchiveToken(c.env, tok))) return next();
  const row = await c.env.DB.prepare("SELECT name FROM viewer_tokens WHERE token_sha256 = ?").bind(await sha256Hex(tok)).first();
  if (!row) throw UNAUTHORIZED();
  await next();
};
