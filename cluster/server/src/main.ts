// crane-server: the coordinator self-hosted. Configuration comes from the environment (see ../README.md); the
// Worker's [vars] (wrangler.toml) are the defaults, so both deployments behave alike.
import { mkdirSync } from "node:fs";
import { createServer } from "node:http";
import path from "node:path";
import { getRequestListener } from "@hono/node-server";
import defaults from "../.build/vars.json";
import { Assets } from "./assets";
import { type Bucket, CfApiBucket, DiskBucket, ReadThroughBucket, S3Bucket } from "./bucket";
import { createRuntime } from "./runtime";
import { createHandler, newStatus, startScheduler } from "./server";

const E = process.env;
const num = (k: string, d: number) => (E[k] !== undefined && E[k] !== "" && Number.isFinite(Number(E[k])) ? Number(E[k]) : d);

/** The Worker's environment: its [vars], overridden by any variable of the same name, plus secrets. */
const VAR_KEYS = [
  ...Object.keys(defaults),
  "ADMIN_TOKEN", "GITHUB_WEBHOOK_SECRET", "ARCHIVE_URL", "ARCHIVE_ACCESS_ID", "ARCHIVE_ACCESS_SECRET",
  "FEED_HISTORY_S", "D1_READ_BUDGET", "D1_GUARD_FRAC", "MIN_CLIENT_VERSION",
];

export function workerVars(): Record<string, string> {
  const v: Record<string, string> = { ...(defaults as Record<string, string>) };
  // No D1 quota here: the read counter stays (GET /v1/admin/reads) but the guard never trips by default.
  v.D1_READ_BUDGET = "1000000000000";
  for (const k of VAR_KEYS) if (E[k] !== undefined) v[k] = E[k] as string;
  return v;
}

export function makeBucket(dataDir: string): Bucket {
  const kind = E.STORAGE ?? "disk";
  let primary: Bucket;
  if (kind === "disk") primary = new DiskBucket(E.STORAGE_DIR ?? path.join(dataDir, "bucket"));
  else if (kind === "s3" || kind === "r2")
    primary = new S3Bucket({
      endpoint: E.S3_ENDPOINT ?? `https://${E.CLOUDFLARE_ACCOUNT_ID}.r2.cloudflarestorage.com`,
      bucket: E.S3_BUCKET ?? "crane-cluster-artifacts",
      accessKeyId: E.S3_ACCESS_KEY_ID ?? "",
      secretAccessKey: E.S3_SECRET_ACCESS_KEY ?? "",
      region: E.S3_REGION ?? "auto",
    });
  else throw new Error(`STORAGE must be disk or s3, not ${kind}`);
  // During the cutover: objects still only in R2 are read through the Cloudflare API and copied in.
  if (E.R2_FALLBACK === "api" && E.CLOUDFLARE_API_TOKEN && E.CLOUDFLARE_ACCOUNT_ID)
    return new ReadThroughBucket(primary, new CfApiBucket({ accountId: E.CLOUDFLARE_ACCOUNT_ID, bucket: E.R2_BUCKET ?? "crane-cluster-artifacts", token: E.CLOUDFLARE_API_TOKEN }));
  return primary;
}

async function main() {
  const dataDir = E.CRANE_DATA_DIR ?? "/data";
  mkdirSync(dataDir, { recursive: true });
  const here = path.dirname(new URL(import.meta.url).pathname);
  const vars = workerVars();
  if (!vars.ADMIN_TOKEN) throw new Error("ADMIN_TOKEN is not set");
  const rt = createRuntime({
    dbPath: E.CRANE_DB ?? path.join(dataDir, "coordinator.sqlite"),
    migrationsDir: E.CRANE_MIGRATIONS ?? path.join(here, "migrations"),
    bucket: makeBucket(dataDir),
    vars,
  });
  if (rt.applied.length) console.log(`migrations applied: ${rt.applied.join(", ")}`);
  await rt.sweep(Date.now(), false); // loads the Hub from the database (leases get their grace TTL) before serving
  const assets = new Assets(E.CRANE_PUBLIC ?? path.join(here, "public"));
  const status = newStatus();
  const pending = new Set<Promise<unknown>>();
  const handler = createHandler(rt, assets, status, pending);
  const server = createServer(getRequestListener(handler, { overrideGlobalObjects: false }));
  server.keepAliveTimeout = 65_000;
  server.requestTimeout = 0; // long polls (25 s) and SSE (up to 900 s) hold requests open
  const sched = startScheduler(rt, status, {
    sweepS: num("SWEEP_S", 60),
    flushS: num("FLUSH_S", 15),
    tierEveryH: num("TIER_EVERY_H", 6),
    backupHourUtc: num("BACKUP_HOUR_UTC", 9),
    backupDir: E.BACKUP_DIR ?? path.join(dataDir, "backups"),
    backupKeep: num("BACKUP_KEEP", 7),
  });
  const port = num("PORT", 8787);
  const host = E.HOST ?? "127.0.0.1";
  server.listen(port, host, () => console.log(`crane-server on http://${host}:${port} (db ${rt.d1.path}, storage ${rt.bucket.kind})`));
  let stopping = false;
  const stop = async (sig: string) => {
    if (stopping) return;
    stopping = true;
    console.log(`${sig}: flushing the Hub and closing`);
    sched.stop();
    server.close();
    server.closeAllConnections?.();
    await rt.close();
    process.exit(0);
  };
  process.on("SIGTERM", () => void stop("SIGTERM"));
  process.on("SIGINT", () => void stop("SIGINT"));
}

main().catch((e) => {
  console.error(e);
  process.exit(1);
});
