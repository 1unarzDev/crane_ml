// The coordinator in one Node process: the Worker's code (.build/core, the Hub patched in) over SQLite, a bucket
// and one in-process Hub. Shared by the HTTP server (main.ts), the tests and the migration tool.
import { readFileSync, readdirSync } from "node:fs";
import path from "node:path";
import worker, { Hub, sweep } from "../.build/core/src/index";
import type { Bucket } from "./bucket";
import { type Migration, SqliteD1, applyMigrations } from "./d1";
import { KvStorage } from "./shims/workers";

export type Vars = Record<string, string | undefined>;

export interface Runtime {
  env: Record<string, unknown>;
  d1: SqliteD1;
  hub: Hub;
  bucket: Bucket;
  worker: typeof worker;
  /** The cron's sweep: reclaim, flush, and (tier=true) artifact tiering. */
  sweep: (nowMs?: number, tier?: boolean) => ReturnType<typeof sweep>;
  close: () => Promise<void>;
  applied: string[];
}

export function readMigrations(dir: string): Migration[] {
  return readdirSync(dir)
    .filter((f) => f.endsWith(".sql"))
    .sort()
    .map((f) => ({ name: f, sql: readFileSync(path.join(dir, f), "utf8") }));
}

/** An ExecutionContext whose waitUntil promises are kept (errors logged) so shutdown can wait for them. */
export function executionContext(pending: Set<Promise<unknown>>) {
  return {
    waitUntil(p: Promise<unknown>) {
      const q = p.catch((e) => console.error("waitUntil", e)).finally(() => pending.delete(q));
      pending.add(q);
    },
    passThroughOnException() {},
    props: {},
  };
}

export function createRuntime(opts: { dbPath: string; migrationsDir: string; bucket: Bucket; vars: Vars }): Runtime {
  const d1 = new SqliteD1(opts.dbPath);
  const applied = applyMigrations(d1, readMigrations(opts.migrationsDir));
  const env: Record<string, unknown> = { ...opts.vars, DB: d1, BUCKET: opts.bucket };
  const storage = new KvStorage(d1);
  const pending = new Set<Promise<unknown>>();
  const ctx = {
    storage,
    id: { toString: () => "hub", name: "hub" },
    waitUntil: (p: Promise<unknown>) => executionContext(pending).waitUntil(p),
    blockConcurrencyWhile: <T>(fn: () => Promise<T>) => fn(),
  };
  // The Hub's env is the raw one (its own D1 reads are counted inside it), like a Durable Object's.
  const hub = new Hub(ctx as never, env as never);
  env.HUB = {
    idFromName: (name: string) => ({ toString: () => name, name }),
    get: () => hub,
    getByName: () => hub,
  };
  return {
    env,
    d1,
    hub,
    bucket: opts.bucket,
    worker,
    applied,
    sweep: (nowMs = Date.now(), tier = true) => sweep(env as never, nowMs, tier),
    async close() {
      await Promise.allSettled([...pending]);
      await hub.flush();
      d1.close();
    },
  };
}
