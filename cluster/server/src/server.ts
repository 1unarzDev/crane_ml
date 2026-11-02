// The HTTP front (health, backup guard, dashboard assets, then the Worker's app) and the in-process scheduler
// that replaces the Worker's cron: reclaim + flush every SWEEP_S, the batched Hub writes every FLUSH_S, artifact
// tiering every TIER_EVERY_H hours, and the nightly backup.
import type { Assets } from "./assets";
import { BACKUP_PREFIX, backup } from "./backup";
import { type Runtime, executionContext } from "./runtime";
import { sha256Hex } from "../.build/core/src/util";

export interface Status {
  started: number;
  lastSweep: { at: string; result: unknown } | null;
  lastBackup: { at: string; file: string; bytes: number; key: string | null } | null;
  lastError: { at: string; what: string; error: string } | null;
}

export function newStatus(): Status {
  return { started: Date.now(), lastSweep: null, lastBackup: null, lastError: null };
}

const json = (d: unknown, status = 200) =>
  new Response(JSON.stringify(d), { status, headers: { "content-type": "application/json; charset=utf-8", "cache-control": "no-store" } });

/** May this bearer read the coordinator's own backups? The admin token or an archive node's token. */
async function backupReader(rt: Runtime, req: Request): Promise<boolean> {
  const m = /^Bearer\s+(\S+)\s*$/i.exec(req.headers.get("authorization") ?? "");
  if (!m || m[1].length > 256) return false;
  const sha = await sha256Hex(m[1]);
  const admin = String(rt.env.ADMIN_TOKEN ?? "");
  if (admin && sha === (await sha256Hex(admin))) return true;
  return !!(await rt.d1.prepare("SELECT 1 AS ok FROM archive_tokens WHERE token_sha256 = ?").bind(sha).first());
}

export function createHandler(rt: Runtime, assets: Assets | null, status: Status, pending: Set<Promise<unknown>>) {
  return async (req: Request): Promise<Response> => {
    const url = new URL(req.url);
    if (url.pathname === "/healthz" && (req.method === "GET" || req.method === "HEAD")) {
      let db = "ok";
      try {
        rt.d1.db.prepare("SELECT 1").get();
      } catch (e) {
        db = String(e).slice(0, 200);
      }
      const body = {
        ok: db === "ok",
        uptime_s: Math.round((Date.now() - status.started) / 1000),
        db,
        storage: rt.bucket.kind,
        hub: rt.hub.state(),
        last_sweep: status.lastSweep,
        last_backup: status.lastBackup,
        last_error: status.lastError,
      };
      return json(body, body.ok ? 200 : 503);
    }
    // wrangler dev --test-scheduled's hook, for the end-to-end test (CRANE_TEST_SCHEDULED=1 only).
    if (url.pathname === "/__scheduled" && process.env.CRANE_TEST_SCHEDULED === "1") return json(await rt.sweep(Date.now(), true));
    if (url.pathname.startsWith(`/v1/artifact/${BACKUP_PREFIX}`) && !(await backupReader(rt, req))) return json({ error: "not found" }, 404);
    const a = assets?.serve(req);
    if (a) return a;
    return rt.worker.fetch(req as never, rt.env as never, executionContext(pending) as never);
  };
}

export interface SchedOpts {
  sweepS: number;
  flushS: number;
  tierEveryH: number;
  backupHourUtc: number; // -1: no nightly backup
  backupDir: string;
  backupKeep: number;
  log?: (msg: string) => void;
}

export function startScheduler(rt: Runtime, status: Status, o: SchedOpts) {
  const log = o.log ?? ((m: string) => console.log(m));
  const fail = (what: string, e: unknown) => {
    status.lastError = { at: new Date().toISOString(), what, error: String(e).slice(0, 500) };
    console.error(what, e);
  };
  let lastTier = Math.floor(Date.now() / (o.tierEveryH * 3_600_000));
  let lastBackupDay = "";
  let busy = false;
  const tick = async () => {
    if (busy) return;
    busy = true;
    try {
      const now = Date.now();
      const slot = Math.floor(now / (o.tierEveryH * 3_600_000));
      const tier = slot !== lastTier;
      const r = await rt.sweep(now, tier);
      if (tier) lastTier = slot;
      status.lastSweep = { at: new Date(now).toISOString(), result: r };
      if (r.requeued.length || r.dead.length || r.artifacts_deleted || r.artifacts_r2_deleted) log(`sweep ${JSON.stringify(r)}`);
      const d = new Date(now);
      const day = d.toISOString().slice(0, 10);
      if (o.backupHourUtc >= 0 && d.getUTCHours() === o.backupHourUtc && day !== lastBackupDay) {
        lastBackupDay = day;
        const b = await backup(rt, o.backupDir, o.backupKeep);
        status.lastBackup = { at: d.toISOString(), file: b.file, bytes: b.bytes, key: b.key };
        log(`backup ${b.file} (${b.bytes} bytes)`);
      }
    } catch (e) {
      fail("sweep", e);
    } finally {
      busy = false;
    }
  };
  const sweepT = setInterval(tick, o.sweepS * 1000);
  const flushT = setInterval(() => rt.hub.flush().catch((e) => fail("flush", e)), o.flushS * 1000);
  return {
    tick,
    stop() {
      clearInterval(sweepT);
      clearInterval(flushT);
    },
  };
}
