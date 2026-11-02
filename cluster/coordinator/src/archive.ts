// Storage tiers (design-storage.md section 2): R2 is hot, an archive node (crane-archive, content-addressed by
// sha256, any host behind a Cloudflare Tunnel) is cold. Without an archive node the old rule holds: raw
// artifacts are deleted after expiry and summaries are kept. With one, an artifact leaves R2 only when it is
// archived (sha256 confirmed), past HOT_DAYS[kind] and not protected; a lagging archive holds items in R2 until
// R2 passes R2_SOFT_CAP_GB, and only then are expired raw items deleted (counted, never silent).
import { Hono } from "hono";
import { AppEnv, requireAdmin, requireArchive } from "./auth";
import { getConfig, setConfigStmt } from "./db";
import { Env, HttpError, intVar, isObject, json, nowIso, parseJson, randomToken, readJson, sha256Hex } from "./util";

export const archiveRoutes = new Hono<AppEnv>();

export const KINDS = ["ctraj", "lod", "summary", "raw", "bundle", "crane", "render_master", "render_final"] as const;
export type Kind = (typeof KINDS)[number];

/** Days an artifact stays in R2 after upload (or after it was copied back from the archive). */
export const DEFAULT_HOT_DAYS: Record<Kind, number> = {
  ctraj: 60,
  lod: 365,
  summary: 365,
  raw: 14,
  bundle: 14,
  crane: 7,
  render_master: 14,
  render_final: 90,
};
const HOT_DAYS_CFG = "hot_days_json";
const LAG_MS = 72 * 3_600_000;
const SHA_RE = /^[0-9a-f]{64}$/;
const NODE_RE = /^[A-Za-z0-9._-]{1,64}$/;
const CODECS = new Set(["none", "zstd19", "xz", "pack", "pack+zstd19", "pack+xz"]);

export function kindOf(name: string, retention: string): Kind {
  const n = name.toLowerCase();
  if (n.endsWith(".lod.ctraj")) return "lod";
  if (n.endsWith(".ctraj")) return "ctraj";
  if (n.endsWith(".crane") || n.endsWith(".crane.zst")) return "crane";
  if (/\.tar\.(zst|gz|xz)$/.test(n)) return "bundle";
  if (n.endsWith(".gif")) return "render_final";
  if (/\.(mp4|mkv|webm)$/.test(n)) return /chunk|master|h264/.test(n) ? "render_master" : "render_final";
  return retention === "summary" ? "summary" : "raw";
}

export async function hotDays(env: Env): Promise<Record<Kind, number>> {
  const cfg = await getConfig(env);
  const over = parseJson<Record<string, number>>(cfg[HOT_DAYS_CFG], {});
  const raw = intVar(cfg.raw_retention_days, intVar(env.RAW_RETENTION_DAYS, DEFAULT_HOT_DAYS.raw));
  return { ...DEFAULT_HOT_DAYS, raw, ...over };
}

/** PUT /v1/admin/retention's hot_days: {kind: days}; unknown kinds and non-positive days are refused. */
export function hotDaysStmt(env: Env, hot: unknown): D1PreparedStatement {
  if (!isObject(hot)) throw new HttpError(400, "hot_days must be an object of kind -> days");
  for (const [k, v] of Object.entries(hot)) {
    if (!(KINDS as readonly string[]).includes(k)) throw new HttpError(400, `unknown kind ${k}`, { kinds: KINDS });
    if (!Number.isInteger(v) || (v as number) < 1) throw new HttpError(400, `hot_days.${k} must be a positive integer`);
  }
  return setConfigStmt(env, HOT_DAYS_CFG, JSON.stringify(hot));
}

// SQL: the artifact's hot window has ended (?hot = the HOT_DAYS JSON, ?now = ISO time).
const PAST_HOT = (hot: string, now: string) =>
  `julianday(COALESCE(a.hot_at, a.created)) + COALESCE(json_extract(${hot}, '$.' || COALESCE(a.kind, 'raw')), 14) < julianday(${now})`;

interface NodeRow {
  name: string;
  root_kind: string | null;
  total_bytes: number | null;
  free_bytes: number | null;
  used_bytes: number | null;
  objects: number | null;
  state: string | null;
  last_seen: string;
}

export interface ArchiveState {
  mode: "none" | "archive";
  healthy: boolean;
  nodes: NodeRow[];
  r2_bytes: number;
  lag_bytes: number;
}

export async function archiveState(env: Env, nowMs = Date.now()): Promise<ArchiveState> {
  const hot = JSON.stringify(await hotDays(env));
  const [nodes, r2, lag] = await env.DB.batch([
    env.DB.prepare("SELECT name, root_kind, total_bytes, free_bytes, used_bytes, objects, state, last_seen FROM archive_nodes ORDER BY name"),
    env.DB.prepare("SELECT COALESCE(SUM(size), 0) AS b FROM artifacts WHERE deleted_at IS NULL AND r2_deleted_at IS NULL"),
    env.DB.prepare(
      `SELECT COALESCE(SUM(a.size), 0) AS b FROM artifacts a
        WHERE a.deleted_at IS NULL AND a.r2_deleted_at IS NULL AND a.archived_at IS NULL AND ${PAST_HOT("?1", "?2")}`,
    ).bind(hot, nowIso(nowMs)),
  ]);
  const rows = nodes.results as unknown as NodeRow[];
  const lagBytes = Number((lag.results[0] as { b: number }).b);
  const fresh = rows.some((n) => nowMs - Date.parse(n.last_seen) < LAG_MS && n.state !== "full");
  const maxLag = intVar(env.ARCHIVE_MAX_LAG_GB, 50) * 1e9;
  return {
    mode: rows.length ? "archive" : "none",
    healthy: fresh && lagBytes <= maxLag,
    nodes: rows,
    r2_bytes: Number((r2.results[0] as { b: number }).b),
    lag_bytes: lagBytes,
  };
}

export interface SweepResult {
  deleted: string[];      // gone everywhere (deleted_at)
  r2_deleted: string[];   // archived, left R2 (r2_deleted_at)
  held: number;           // past their window, waiting for the archive
}

/** The cron sweep: tiers artifacts out of R2 (see the file comment). */
export async function sweepArtifacts(env: Env, nowMs = Date.now(), limit = 500): Promise<SweepResult> {
  const now = nowIso(nowMs);
  const st = await archiveState(env, nowMs);
  const out: SweepResult = { deleted: [], r2_deleted: [], held: 0 };
  const legacyDelete = async () => {
    const { results } = await env.DB.prepare(
      `SELECT key FROM artifacts WHERE retention = 'raw' AND pinned = 0 AND deleted_at IS NULL
          AND expires_at IS NOT NULL AND expires_at < ?1 LIMIT ?2`,
    )
      .bind(now, limit)
      .all<{ key: string }>();
    const keys = results.map((r) => r.key);
    if (keys.length) {
      await env.BUCKET.delete(keys);
      await env.DB.prepare(
        "UPDATE artifacts SET deleted_at = ?2, r2_deleted_at = COALESCE(r2_deleted_at, ?2) WHERE key IN (SELECT value FROM json_each(?1))",
      )
        .bind(JSON.stringify(keys), now)
        .run();
    }
    out.deleted = keys;
  };
  if (st.mode === "none") {
    await legacyDelete();
    return out;
  }
  const hot = JSON.stringify(await hotDays(env));
  const { results } = await env.DB.prepare(
    `SELECT a.key FROM artifacts a
      WHERE a.deleted_at IS NULL AND a.r2_deleted_at IS NULL AND a.archived_at IS NOT NULL AND a.pinned = 0
        AND ${PAST_HOT("?1", "?2")} AND a.key NOT IN (SELECT key FROM protected_artifacts) LIMIT ?3`,
  )
    .bind(hot, now, limit)
    .all<{ key: string }>();
  const keys = results.map((r) => r.key);
  if (keys.length) {
    await env.BUCKET.delete(keys);
    await env.DB.prepare("UPDATE artifacts SET r2_deleted_at = ?2 WHERE key IN (SELECT value FROM json_each(?1))")
      .bind(JSON.stringify(keys), now)
      .run();
  }
  out.r2_deleted = keys;
  const softCap = intVar(env.R2_SOFT_CAP_GB, 200) * 1e9;
  if (!st.healthy && st.r2_bytes > softCap) await legacyDelete();
  const held = await env.DB.prepare(
    `SELECT COUNT(*) AS n FROM artifacts a WHERE a.deleted_at IS NULL AND a.r2_deleted_at IS NULL AND a.archived_at IS NULL
        AND a.pinned = 0 AND ${PAST_HOT("?1", "?2")}`,
  )
    .bind(hot, now)
    .first<{ n: number }>();
  out.held = held?.n ?? 0;
  return out;
}

// ------------------------------------------------------------------ reads with archive fallback

interface ArtRow {
  key: string;
  sha256: string;
  kind: string | null;
  size: number;
  archived_at: string | null;
  deleted_at: string | null;
  r2_deleted_at: string | null;
}

const REHOT = new Set(["ctraj", "lod", "summary"]);

const offline = (detail: string) =>
  new HttpError(503, "archived (offline)", { tier: "archive", state: "offline", detail, retry_after_s: 300 });

/**
 * An artifact's bytes from R2, else from the archive node (streamed through the Worker, Range passed on).
 * Trajectories and summaries fetched whole from the archive are copied back into R2 with a fresh hot clock.
 * Throws 404 when gone and 503 {tier: archive, state: offline} when only an unreachable archive has it.
 */
export async function fetchArtifact(env: Env, key: string, req: Request): Promise<Response> {
  const range = req.headers.get("range");
  const obj = await env.BUCKET.get(key, range ? { range: req.headers } : undefined);
  if (obj) return r2Response(obj, range);
  const row = await env.DB.prepare(
    "SELECT key, sha256, kind, size, archived_at, deleted_at, r2_deleted_at FROM artifacts WHERE key = ?",
  )
    .bind(key)
    .first<ArtRow>();
  if (!row || row.deleted_at || !row.archived_at) throw new HttpError(404, "not found");
  const res = await archiveGet(env, row.sha256, range);
  const headers = new Headers({
    "content-type": "application/octet-stream",
    "x-content-sha256": row.sha256,
    "x-storage-tier": "archive",
  });
  for (const h of ["content-length", "content-range", "accept-ranges"]) {
    const v = res.headers.get(h);
    if (v) headers.set(h, v);
  }
  if (!range && res.status === 200 && REHOT.has(row.kind ?? "") && row.size <= 32 * 1024 * 1024) {
    const body = await res.arrayBuffer();
    if ((await sha256Hex(body)) !== row.sha256) throw offline("archive returned other bytes");
    await env.BUCKET.put(key, body, { customMetadata: { sha256: row.sha256, retention: "rehot" } });
    await env.DB.prepare("UPDATE artifacts SET r2_deleted_at = NULL, hot_at = ?2 WHERE key = ?1").bind(key, nowIso()).run();
    headers.set("content-length", String(body.byteLength));
    return new Response(body, { status: 200, headers });
  }
  return new Response(res.body, { status: res.status, headers });
}

function r2Response(obj: R2ObjectBody, range: string | null): Response {
  const headers = new Headers({
    "content-type": "application/octet-stream",
    "x-content-sha256": obj.customMetadata?.sha256 ?? "",
    "accept-ranges": "bytes",
  });
  const r = obj.range as { offset?: number; length?: number } | undefined;
  if (range && r && (r.offset !== undefined || r.length !== undefined)) {
    const offset = r.offset ?? 0;
    const length = r.length ?? obj.size - offset;
    headers.set("content-range", `bytes ${offset}-${offset + length - 1}/${obj.size}`);
    headers.set("content-length", String(length));
    return new Response(obj.body, { status: 206, headers });
  }
  headers.set("content-length", String(obj.size));
  return new Response(obj.body, { headers });
}

async function archiveGet(env: Env, sha: string, range: string | null): Promise<Response> {
  if (!env.ARCHIVE_URL) throw offline("no archive configured");
  const headers: Record<string, string> = { "user-agent": "crane-coordinator" };
  if (env.ARCHIVE_ACCESS_ID && env.ARCHIVE_ACCESS_SECRET) {
    headers["cf-access-client-id"] = env.ARCHIVE_ACCESS_ID;
    headers["cf-access-client-secret"] = env.ARCHIVE_ACCESS_SECRET;
  }
  if (range) headers.range = range;
  let res: Response;
  try {
    res = await fetch(`${env.ARCHIVE_URL.replace(/\/$/, "")}/o/${sha}`, { headers, redirect: "manual" });
  } catch (e) {
    throw offline(String(e).slice(0, 120));
  }
  if (res.status === 404) throw new HttpError(404, "not found", { tier: "archive" });
  if (res.status !== 200 && res.status !== 206) throw offline(`archive answered ${res.status}`);
  const got = res.headers.get("x-content-sha256");
  if (got && got !== sha) throw offline("archive returned another object");
  return res;
}

// ------------------------------------------------------------------ archive node routes (archive token)

/** What to pull next: in R2, not archived, older than min_age_h; the soonest to leave R2 first. */
archiveRoutes.get("/v1/archive/pending", requireArchive, async (c) => {
  const limit = Math.min(Math.max(intVar(c.req.query("limit"), 200), 1), 1000);
  const minAgeH = Math.max(intVar(c.req.query("min_age_h"), 24), 0);
  const hot = JSON.stringify(await hotDays(c.env));
  const { results } = await c.env.DB.prepare(
    `SELECT a.key, a.sha256, a.size, COALESCE(a.kind, 'raw') AS kind, a.created,
            (a.key IN (SELECT key FROM protected_artifacts)) AS protected
       FROM artifacts a
      WHERE a.archived_at IS NULL AND a.deleted_at IS NULL AND a.r2_deleted_at IS NULL AND a.created < ?2
      ORDER BY julianday(COALESCE(a.hot_at, a.created)) + COALESCE(json_extract(?1, '$.' || COALESCE(a.kind, 'raw')), 14), a.key
      LIMIT ?3`,
  )
    .bind(hot, nowIso(Date.now() - minAgeH * 3_600_000), limit)
    .all();
  return json({ items: results.map((r) => ({ ...r, protected: !!r.protected })) });
});

/** The daemon stored these (fsynced): {node, items: [{key, sha256, archive_bytes, codec}]}. */
archiveRoutes.post("/v1/archive/confirm", requireArchive, async (c) => {
  const b = await readJson(c.req.raw);
  const node = nodeName(b.node);
  const items = itemsOf(b.items, 1000);
  const now = nowIso();
  const stmts = items.map((it) => {
    if (typeof it.key !== "string" || typeof it.sha256 !== "string" || !SHA_RE.test(it.sha256)) throw new HttpError(400, "each item needs key and sha256");
    const codec = typeof it.codec === "string" && CODECS.has(it.codec) ? it.codec : "none";
    const bytes = Number.isInteger(it.archive_bytes) ? (it.archive_bytes as number) : null;
    return c.env.DB.prepare(
      `UPDATE artifacts SET archived_at = ?3, archive_bytes = ?4, archive_codec = ?5, archive_node = ?6
        WHERE key = ?1 AND sha256 = ?2 AND deleted_at IS NULL`,
    ).bind(it.key, it.sha256, now, bytes, codec, node);
  });
  const res = stmts.length ? await c.env.DB.batch(stmts) : [];
  const refused = items.filter((_, i) => !res[i].meta.changes).map((it) => it.key);
  return json({ confirmed: items.length - refused.length, refused });
});

/** {node, root_kind, total, free, used, objects, state, ...health}; answers with the coordinator's view. */
archiveRoutes.post("/v1/archive/heartbeat", requireArchive, async (c) => {
  const b = await readJson(c.req.raw);
  const node = nodeName(b.node);
  const n = (v: unknown) => (typeof v === "number" && Number.isFinite(v) ? Math.round(v) : null);
  const state = typeof b.state === "string" && ["ok", "degraded", "full"].includes(b.state) ? b.state : "ok";
  const health = JSON.stringify(b).slice(0, 16_000);
  await c.env.DB.prepare(
    `INSERT INTO archive_nodes(name, root_kind, total_bytes, free_bytes, used_bytes, objects, state, health_json, last_seen)
     VALUES (?1, ?2, ?3, ?4, ?5, ?6, ?7, ?8, ?9)
     ON CONFLICT(name) DO UPDATE SET root_kind = excluded.root_kind, total_bytes = excluded.total_bytes,
       free_bytes = excluded.free_bytes, used_bytes = excluded.used_bytes, objects = excluded.objects,
       state = excluded.state, health_json = excluded.health_json, last_seen = excluded.last_seen`,
  )
    .bind(node, typeof b.root_kind === "string" ? b.root_kind.slice(0, 32) : null, n(b.total), n(b.free), n(b.used), n(b.objects), state, health, nowIso())
    .run();
  const st = await archiveState(c.env);
  return json({ healthy: st.healthy, r2_bytes: st.r2_bytes, lag_bytes: st.lag_bytes, hot_days: await hotDays(c.env) });
});

/**
 * Objects the archive lost: {node, items: [{sha256, reason: corrupt|evicted|lost}]}. While R2 still has a
 * copy the artifact goes back to pending (re-pulled); otherwise it is gone everywhere (deleted_at). An
 * eviction also drops any R2 copy past its hot window, so evicted classes do not loop back.
 */
archiveRoutes.post("/v1/archive/missing", requireArchive, async (c) => {
  const b = await readJson(c.req.raw);
  nodeName(b.node);
  const items = itemsOf(b.items, 1000);
  const now = nowIso();
  let repull = 0;
  let gone = 0;
  for (const it of items) {
    if (typeof it.sha256 !== "string" || !SHA_RE.test(it.sha256)) throw new HttpError(400, "each item needs sha256");
    const evicted = it.reason === "evicted";
    const r1 = await c.env.DB.prepare(
      `UPDATE artifacts SET archived_at = NULL, archive_codec = NULL, archive_bytes = NULL, archive_node = NULL
        WHERE sha256 = ?1 AND deleted_at IS NULL AND r2_deleted_at IS NULL AND ?2 = 0`,
    )
      .bind(it.sha256, evicted ? 1 : 0)
      .run();
    repull += r1.meta.changes;
    const { results } = await c.env.DB.prepare(
      `SELECT key FROM artifacts WHERE sha256 = ?1 AND deleted_at IS NULL AND (r2_deleted_at IS NOT NULL OR ?2 = 1)
          AND key NOT IN (SELECT key FROM protected_artifacts)`,
    )
      .bind(it.sha256, evicted ? 1 : 0)
      .all<{ key: string }>();
    const keys = results.map((r) => r.key);
    if (keys.length) {
      await c.env.BUCKET.delete(keys);
      await c.env.DB.prepare(
        `UPDATE artifacts SET deleted_at = ?2, r2_deleted_at = COALESCE(r2_deleted_at, ?2) WHERE key IN (SELECT value FROM json_each(?1))`,
      )
        .bind(JSON.stringify(keys), now)
        .run();
      gone += keys.length;
    }
  }
  return json({ repull, gone });
});

/** The sha256s of the protected set: the daemon never evicts them. */
archiveRoutes.get("/v1/archive/protected", requireArchive, async (c) => {
  const { results } = await c.env.DB.prepare("SELECT DISTINCT sha256 FROM protected_artifacts LIMIT 200000").all<{ sha256: string }>();
  return json({ sha256: results.map((r) => r.sha256) });
});

// ------------------------------------------------------------------ admin and public health

archiveRoutes.post("/v1/admin/archive_token", requireAdmin, async (c) => {
  const b = await readJson<{ name?: unknown }>(c.req.raw);
  const name = typeof b.name === "string" && NODE_RE.test(b.name.trim()) ? b.name.trim() : "archive";
  const token = randomToken();
  await c.env.DB.prepare("INSERT INTO archive_tokens(token_sha256, name, created) VALUES (?, ?, ?)").bind(await sha256Hex(token), name, nowIso()).run();
  return json({ token, name });
});

archiveRoutes.get("/v1/admin/archive", requireAdmin, async (c) => {
  const st = await archiveState(c.env);
  const tiers = await c.env.DB.prepare(
    `SELECT COALESCE(kind, 'raw') AS kind,
            SUM(deleted_at IS NULL AND r2_deleted_at IS NULL) AS hot, SUM(deleted_at IS NULL AND archived_at IS NOT NULL) AS archived,
            SUM(deleted_at IS NOT NULL) AS gone, SUM(CASE WHEN deleted_at IS NULL AND r2_deleted_at IS NULL THEN size ELSE 0 END) AS hot_bytes,
            SUM(CASE WHEN deleted_at IS NULL AND archived_at IS NOT NULL THEN COALESCE(archive_bytes, size) ELSE 0 END) AS archive_bytes
       FROM artifacts GROUP BY 1 ORDER BY 1`,
  ).all();
  return json({ ...st, hot_days: await hotDays(c.env), kinds: tiers.results });
});

/** Public (D-20261102-19): state and percent used only; no names, paths or sizes of anything. */
archiveRoutes.get("/v1/feed/archive", async (c) => {
  const st = await archiveState(c.env);
  const total = st.nodes.reduce((s, n) => s + (n.total_bytes ?? 0), 0);
  const used = st.nodes.reduce((s, n) => s + (n.used_bytes ?? 0), 0);
  const fresh = st.nodes.some((n) => Date.now() - Date.parse(n.last_seen) < LAG_MS);
  const degraded = st.nodes.some((n) => n.state === "degraded");
  const full = st.nodes.some((n) => n.state === "full");
  const state = st.mode === "none" ? "none" : !fresh ? "offline" : full ? "full" : degraded ? "degraded" : st.healthy ? "ok" : "lagging";
  return json({ schema: "crane-archive-health/1", state, used_pct: total ? Math.round((1000 * used) / total) / 10 : null }, 200, {
    "cache-control": "public, max-age=60",
  });
});

function nodeName(v: unknown): string {
  if (typeof v !== "string" || !NODE_RE.test(v)) throw new HttpError(400, "node must be a short name");
  return v;
}

function itemsOf(v: unknown, max: number): Record<string, unknown>[] {
  if (!Array.isArray(v) || !v.every(isObject)) throw new HttpError(400, "items must be an array of objects");
  if (v.length > max) throw new HttpError(400, `at most ${max} items`);
  return v as Record<string, unknown>[];
}
