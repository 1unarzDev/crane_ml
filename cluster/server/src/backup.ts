// Nightly SQLite backups: VACUUM INTO a snapshot, quick_check it, gzip it into <dir> (the newest `keep` stay), and
// register it as a raw artifact (artifacts/_coordinator/backups/<stamp>/coordinator.sqlite.gz). The archive node
// pulls it like any artifact once it is a day old and mirrors it to Drive within 24 h; the hot copy expires with
// the raw window. Only the admin token and archive nodes may read that prefix (server.ts).
import { createHash } from "node:crypto";
import { createReadStream, createWriteStream } from "node:fs";
import { mkdir, readFile, readdir, rm, stat } from "node:fs/promises";
import path from "node:path";
import { DatabaseSync } from "node:sqlite";
import { pipeline } from "node:stream/promises";
import { createGzip } from "node:zlib";
import type { Runtime } from "./runtime";

export const BACKUP_PREFIX = "artifacts/_coordinator/";

export interface BackupResult {
  file: string;
  bytes: number;
  sha256: string;
  key: string | null;
}

export async function backup(rt: Runtime, dir: string, keep = 7, register = true, nowMs = Date.now()): Promise<BackupResult> {
  await mkdir(dir, { recursive: true });
  const stamp = new Date(nowMs).toISOString().replace(/[-:]/g, "").slice(0, 13); // 20261102T0930
  const snap = path.join(dir, `.coordinator-${stamp}.sqlite`);
  const file = path.join(dir, `coordinator-${stamp}.sqlite.gz`);
  await rm(snap, { force: true });
  await rt.hub.flush(); // batched heartbeat state goes in too
  rt.d1.db.exec(`VACUUM INTO '${snap.replace(/'/g, "''")}'`);
  try {
    const check = new DatabaseSync(snap, { readOnly: true });
    const r = check.prepare("PRAGMA quick_check").get() as Record<string, string>;
    check.close();
    if (Object.values(r)[0] !== "ok") throw new Error(`backup snapshot failed quick_check: ${JSON.stringify(r)}`);
    await pipeline(createReadStream(snap), createGzip({ level: 6 }), createWriteStream(file));
  } finally {
    await rm(snap, { force: true });
  }
  const bytes = await readFile(file);
  const sha256 = createHash("sha256").update(bytes).digest("hex");
  const olds = (await readdir(dir)).filter((f) => /^coordinator-\d{8}T\d{4}\.sqlite\.gz$/.test(f)).sort();
  for (const f of olds.slice(0, Math.max(0, olds.length - keep))) await rm(path.join(dir, f), { force: true });

  let key: string | null = null;
  if (register) {
    key = `${BACKUP_PREFIX}backups/${stamp}/coordinator.sqlite.gz`;
    const days = Number((await rt.d1.prepare("SELECT value FROM config WHERE key = 'raw_retention_days'").first<string>("value").catch(() => null)) ?? rt.env.RAW_RETENTION_DAYS ?? 14);
    const now = new Date(nowMs).toISOString();
    const expires = new Date(nowMs + (Number.isFinite(days) ? days : 14) * 86_400_000).toISOString();
    await rt.bucket.put(key, bytes, { customMetadata: { sha256, retention: "raw" }, httpMetadata: { contentType: "application/gzip" } });
    await rt.d1
      .prepare(
        `INSERT INTO artifacts(key, job_id, run_id, name, retention, size, sha256, expires_at, created, kind)
         VALUES (?1, NULL, NULL, 'coordinator.sqlite.gz', 'raw', ?2, ?3, ?4, ?5, 'raw')
         ON CONFLICT(key) DO UPDATE SET size = excluded.size, sha256 = excluded.sha256, expires_at = excluded.expires_at,
           created = excluded.created, archived_at = NULL, deleted_at = NULL, r2_deleted_at = NULL, hot_at = NULL`,
      )
      .bind(key, bytes.byteLength, sha256, expires, now)
      .run();
  }
  return { file, bytes: (await stat(file)).size, sha256, key };
}
