import { createExecutionContext, createScheduledController, env, waitOnExecutionContext } from "cloudflare:test";
import worker from "../src/index";

export const ADMIN = "test-admin-token";
export const BASE = "https://coord.test";

export interface CallOpts {
  token?: string;
  json?: unknown;
  body?: BodyInit | Uint8Array;
  headers?: Record<string, string>;
}

export async function call(method: string, path: string, o: CallOpts = {}): Promise<Response> {
  const headers: Record<string, string> = { ...(o.headers ?? {}) };
  if (o.token) headers.authorization = `Bearer ${o.token}`;
  let body: BodyInit | undefined;
  if (o.json !== undefined) {
    headers["content-type"] = "application/json";
    body = JSON.stringify(o.json);
  } else if (o.body !== undefined) body = o.body as BodyInit;
  const ctx = createExecutionContext();
  const res = await worker.fetch(new Request(BASE + path, { method, headers, body }), env, ctx);
  await waitOnExecutionContext(ctx);
  return res;
}

export async function callJson<T = any>(method: string, path: string, o: CallOpts = {}): Promise<{ status: number; body: T }> {
  const r = await call(method, path, o);
  const text = await r.text();
  let body: any = null;
  try {
    body = text ? JSON.parse(text) : null;
  } catch {
    body = text;
  }
  return { status: r.status, body };
}

export const admin = (method: string, path: string, json?: unknown) => callJson(method, path, { token: ADMIN, json });

export async function runCron(): Promise<void> {
  const ctx = createExecutionContext();
  await worker.scheduled(createScheduledController({ scheduledTime: Date.now(), cron: "*/5 * * * *" }), env, ctx);
  await waitOnExecutionContext(ctx);
}

export const CPU_CAPS = { os: "linux", arch: "x86_64", cpu: 8, ram_gb: 16, gpu: null, vulkan: false, tags: [], backends: ["dummy", "command"] };
export const GPU_CAPS = { ...CPU_CAPS, cpu: 16, ram_gb: 64, gpu: "rtx", gpu_count: 1, vram_gb: 12, vulkan: true };

export async function makeWorker(name = "w", caps: Record<string, unknown> = CPU_CAPS, extra: Record<string, unknown> = {}) {
  const inv = await admin("POST", "/v1/admin/invite", { n: 1 });
  const r = await callJson("POST", "/v1/join", { json: { invite: inv.body.invites[0], name, caps, client_version: "0.1.0", ...extra } });
  if (r.status !== 200) throw new Error(`join: ${r.status} ${JSON.stringify(r.body)}`);
  return { ...(r.body as { worker_id: string; token: string }), caps };
}

export async function addJobs(jobs: Record<string, unknown>[]): Promise<string[]> {
  const r = await admin("POST", "/v1/admin/jobs", { jobs: jobs.map((j) => ({ backend: "dummy", ...j })) });
  if (r.status !== 200) throw new Error(`jobs: ${r.status} ${JSON.stringify(r.body)}`);
  return r.body.job_ids;
}

export function lease(w: { token: string; caps: Record<string, unknown> }, max_jobs = 1, extra: Record<string, unknown> = {}) {
  return callJson("POST", "/v1/lease", { token: w.token, json: { caps: w.caps, max_jobs, ...extra } });
}

/** Moves every lease into the past so the next reclaim treats it as expired. */
export async function expireLeases(): Promise<void> {
  await env.DB.prepare("UPDATE jobs SET lease_expires = 1 WHERE status = 'leased'").run();
}

export async function resetTables(): Promise<void> {
  await env.DB.batch(
    ["metrics", "runs", "artifacts", "leases", "jobs", "hypotheses", "repos", "config"].map((t) => env.DB.prepare(`DELETE FROM ${t}`)),
  );
}
