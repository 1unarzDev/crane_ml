// crane-server migration tool (node dist/migrate.mjs <command>):
//   import DUMP.sql [--db PATH] [--force]   a `wrangler d1 export` dump -> a fresh SQLite file, then the server's
//                                           pending migrations; checks integrity and foreign keys, prints row counts
//   pull-r2 [--data DIR] [--dry-run]        copies every R2 object (Cloudflare API: CLOUDFLARE_API_TOKEN,
//                                           CLOUDFLARE_ACCOUNT_ID, R2_BUCKET) into the disk store; skips copies
//                                           already there with the same size and md5; resumable
//   counts [--db PATH]                      row counts per table (compare with the D1 side before the cutover)
// Export on a machine with wrangler (innovation): cluster/server/deploy/d1-export.sh.
import { existsSync, readFileSync, renameSync, rmSync } from "node:fs";
import path from "node:path";
import { CfApiBucket, DiskBucket } from "./bucket";
import { SqliteD1, applyMigrations } from "./d1";
import { readMigrations } from "./runtime";

const args = process.argv.slice(2);
const flag = (name: string) => args.includes(`--${name}`);
const opt = (name: string, d?: string) => {
  const i = args.indexOf(`--${name}`);
  return i >= 0 ? args[i + 1] : d;
};
const here = path.dirname(new URL(import.meta.url).pathname);
const dataDir = process.env.CRANE_DATA_DIR ?? "/data";
const dbPath = () => opt("db", process.env.CRANE_DB ?? path.join(dataDir, "coordinator.sqlite"))!;

export function counts(d1: SqliteD1): Record<string, number> {
  const tables = (d1.db.prepare("SELECT name FROM sqlite_master WHERE type = 'table' AND name NOT LIKE 'sqlite_%' ORDER BY name").all() as { name: string }[]).map((r) => r.name);
  const out: Record<string, number> = {};
  for (const t of tables) out[t] = Number((d1.db.prepare(`SELECT COUNT(*) AS n FROM "${t.replace(/"/g, '""')}"`).get() as { n: number }).n);
  return out;
}

/** Loads a D1 export into a new SQLite file at `db` (written next to it, then renamed into place). */
export function importDump(dumpPath: string, db: string, migrationsDir: string, force = false) {
  if (existsSync(db) && !force) throw new Error(`${db} exists; pass --force to replace it (the old file is kept as ${db}.pre-import)`);
  const tmp = `${db}.importing`;
  for (const f of [tmp, `${tmp}-wal`, `${tmp}-shm`]) rmSync(f, { force: true });
  const d1 = new SqliteD1(tmp);
  try {
    d1.db.exec("PRAGMA foreign_keys = OFF");
    d1.db.exec("BEGIN");
    d1.db.exec(readFileSync(dumpPath, "utf8"));
    d1.db.exec("COMMIT");
    d1.db.exec("PRAGMA foreign_keys = ON");
    const applied = applyMigrations(d1, readMigrations(migrationsDir));
    const fk = d1.db.prepare("PRAGMA foreign_key_check").all();
    if (fk.length) throw new Error(`foreign key violations after import: ${JSON.stringify(fk.slice(0, 5))}`);
    const ic = Object.values(d1.db.prepare("PRAGMA integrity_check").get() as Record<string, string>)[0];
    if (ic !== "ok") throw new Error(`integrity_check: ${ic}`);
    const c = counts(d1);
    d1.db.exec("PRAGMA wal_checkpoint(TRUNCATE)");
    d1.close();
    if (existsSync(db)) {
      renameSync(db, `${db}.pre-import`);
      for (const s of ["-wal", "-shm"]) rmSync(db + s, { force: true });
    }
    renameSync(tmp, db);
    for (const s of ["-wal", "-shm"]) rmSync(tmp + s, { force: true });
    return { applied, counts: c };
  } catch (e) {
    try {
      d1.close();
    } catch {}
    throw e;
  }
}

/** Copies R2 into the disk store through the Cloudflare API (about 3 objects/s, under the API's rate limit). */
export async function pullR2(src: CfApiBucket, dst: DiskBucket, dryRun: boolean, log = console.log) {
  let cursor: string | undefined;
  const t = { listed: 0, bytes: 0, copied: 0, skipped: 0, failed: 0 };
  do {
    const page = await src.list(cursor, 1000);
    cursor = page.cursor;
    for (const o of page.objects) {
      t.listed++;
      t.bytes += o.size;
      const have = await dst.head(o.key);
      if (have && have.size === o.size && (!o.etag || have.etag === o.etag)) {
        t.skipped++;
        continue;
      }
      if (dryRun) continue;
      try {
        const got = await src.get(o.key);
        if (!got) throw new Error("gone");
        await dst.put(o.key, await got.arrayBuffer(), { customMetadata: o.custom_metadata ?? {}, httpMetadata: o.http_metadata ?? {} });
        t.copied++;
      } catch (e) {
        t.failed++;
        log(`failed ${o.key}: ${String(e).slice(0, 120)}`);
      }
      await new Promise((r) => setTimeout(r, 330));
      if ((t.copied + t.failed) % 200 === 0) log(`pull-r2: ${JSON.stringify(t)}`);
    }
  } while (cursor);
  return t;
}

async function cli() {
  const cmd = args[0];
  if (cmd === "import") {
    const dump = args[1];
    if (!dump) throw new Error("usage: import DUMP.sql [--db PATH] [--force]");
    const r = importDump(dump, dbPath(), opt("migrations", path.join(here, "migrations"))!, flag("force"));
    console.log(JSON.stringify({ db: dbPath(), applied: r.applied, counts: r.counts }, null, 2));
  } else if (cmd === "pull-r2") {
    const E = process.env;
    if (!E.CLOUDFLARE_API_TOKEN || !E.CLOUDFLARE_ACCOUNT_ID) throw new Error("CLOUDFLARE_API_TOKEN and CLOUDFLARE_ACCOUNT_ID are required");
    const src = new CfApiBucket({ accountId: E.CLOUDFLARE_ACCOUNT_ID, bucket: E.R2_BUCKET ?? "crane-cluster-artifacts", token: E.CLOUDFLARE_API_TOKEN });
    const dst = new DiskBucket(opt("dir", E.STORAGE_DIR ?? path.join(opt("data", dataDir)!, "bucket"))!);
    console.log(JSON.stringify(await pullR2(src, dst, flag("dry-run"))));
  } else if (cmd === "counts") {
    const d1 = new SqliteD1(dbPath());
    console.log(JSON.stringify(counts(d1), null, 2));
    d1.close();
  } else {
    console.error("commands: import DUMP.sql [--db PATH] [--force] | pull-r2 [--data DIR] [--dry-run] | counts [--db PATH]");
    process.exit(2);
  }
}

if (process.argv[1] && /migrate\.(mjs|ts|js)$/.test(process.argv[1])) {
  cli().catch((e) => {
    console.error(String(e));
    process.exit(1);
  });
}
