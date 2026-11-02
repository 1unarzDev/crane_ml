// "cloudflare:test" on Node: the coordinator's own vitest suites (.build/core/test) run unchanged against the
// self-hosted runtime (SQLite file in WAL mode, disk bucket, one in-process Hub). Each test file gets a fresh one.
import { mkdtempSync, readFileSync } from "node:fs";
import os from "node:os";
import path from "node:path";
import vars from "../.build/vars.json";
import { DiskBucket } from "../src/bucket";
import { type Runtime, createRuntime } from "../src/runtime";

const server = path.resolve(path.dirname(new URL(import.meta.url).pathname), "..");
const golden = path.resolve(server, "../../theater/tests/golden");
const dir = mkdtempSync(path.join(os.tmpdir(), "crane-server-test-"));

export const runtime: Runtime = createRuntime({
  dbPath: path.join(dir, "coordinator.sqlite"),
  migrationsDir: path.join(server, ".build/core/migrations"),
  bucket: new DiskBucket(path.join(dir, "bucket")),
  vars: {
    ...(vars as Record<string, string>),
    ADMIN_TOKEN: "test-admin-token",
    GITHUB_WEBHOOK_SECRET: "test-hook-secret",
    FEED_CACHE_S: "0",
    ARCHIVE_URL: "",
  },
});
Object.assign(runtime.env, {
  TEST_MIGRATIONS: [],
  GOLDEN_CTRAJ: readFileSync(path.join(golden, "small.ctraj")).toString("base64"),
  GOLDEN_JSON: readFileSync(path.join(golden, "small.json"), "utf8"),
});

// eslint-disable-next-line @typescript-eslint/no-explicit-any
export const env: any = runtime.env;
export const tmpDir = dir;

type Ctx = { waitUntil(p: Promise<unknown>): void; passThroughOnException(): void; _p: Promise<unknown>[] };
export function createExecutionContext(): Ctx {
  const ps: Promise<unknown>[] = [];
  return { waitUntil: (p) => void ps.push(p), passThroughOnException() {}, _p: ps };
}
export async function waitOnExecutionContext(ctx: Ctx) {
  await Promise.allSettled(ctx._p);
}
export function createScheduledController(o: { scheduledTime?: number; cron?: string } = {}) {
  return { scheduledTime: o.scheduledTime ?? Date.now(), cron: o.cron ?? "", noRetry() {} };
}
export async function runInDurableObject<O, R>(stub: O, fn: (instance: O, state: unknown) => R | Promise<R>): Promise<R> {
  return fn(stub, (stub as { ctx?: unknown }).ctx);
}
export async function applyD1Migrations() {
  // createRuntime applied them already.
}
export type D1Migration = { name: string; queries: string[] };
