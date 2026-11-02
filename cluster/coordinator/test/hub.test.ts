// The Hub (src/hub.ts): hot state in memory, D1 as the record, and the read budget (src/budget.ts).
import { env, runInDurableObject } from "cloudflare:test";
import { beforeEach, describe, expect, it } from "vitest";
import { Book, bookReads, countingDb } from "../src/budget";
import type { Hub } from "../src/hub";
import { pushReads } from "../src/index";
import { addJobs, admin, call, callJson, hub, lease, makeWorker, resetTables } from "./helpers";

beforeEach(resetTables);

const hubReads = () => runInDurableObject(hub(), (h: Hub) => bookReads(h.book));
const withEnv = (vars: Record<string, string>) =>
  runInDurableObject(hub(), (h: Hub) => {
    (h as unknown as { env: Record<string, unknown> }).env = { ...(h as unknown as { env: Record<string, unknown> }).env, ...vars };
  });

describe("hub", () => {
  it("idle lease polls and heartbeats read nothing from D1", async () => {
    const [id] = await addJobs([{}]);
    const w = await makeWorker("w");
    expect((await lease(w)).body.jobs[0].job_id).toBe(id);
    const idle = await makeWorker("idle");
    const before = await hubReads();
    for (let i = 0; i < 20; i++) {
      expect((await lease(idle)).body.jobs).toEqual([]);
      const hb = await callJson("POST", "/v1/heartbeat", { token: w.token, json: { job_ids: [id], state: "running" } });
      expect(hb.body.extended).toEqual([id]);
    }
    expect(await hubReads()).toBe(before);
  });

  it("a long poll answers as soon as a job arrives", async () => {
    const w = await makeWorker("w");
    const t0 = Date.now();
    const p = lease(w, 1, { wait_s: 10 });
    await new Promise((r) => setTimeout(r, 300));
    const [id] = await addJobs([{}]);
    const r = await p;
    expect(r.body.jobs.map((j: { job_id: string }) => j.job_id)).toEqual([id]);
    expect(Date.now() - t0).toBeLessThan(5000);
    // with nothing to lease it waits out wait_s (capped at 25 s), then answers empty
    const t1 = Date.now();
    expect((await lease(w, 1, { wait_s: 1 })).body.jobs).toEqual([]);
    expect(Date.now() - t1).toBeGreaterThanOrEqual(900);
  });

  it("a job cancelled in D1 behind the Hub's back is not leased", async () => {
    const [a, b] = await addJobs([{ priority: 1 }, { priority: 2 }]);
    await env.DB.prepare("UPDATE jobs SET status = 'cancelled' WHERE job_id = ?").bind(a).run();
    const w = await makeWorker("w");
    const r = await lease(w, 1);
    // the stale pick fails its conditional UPDATE and drops out; the next call gets the other job
    expect(r.body.jobs.map((j: { job_id: string }) => j.job_id)).not.toContain(a);
    const got = r.body.jobs.length ? r.body.jobs : (await lease(w, 1)).body.jobs;
    expect(got.map((j: { job_id: string }) => j.job_id)).toEqual([b]);
    expect(await runInDurableObject(hub(), (h: Hub) => h.jobs.has(a))).toBe(false);
  });

  it("restarts from D1 with a grace TTL on live leases; heartbeats are written in batches", async () => {
    const [id] = await addJobs([{}]);
    const w = await makeWorker("w");
    await lease(w);
    await env.DB.prepare("UPDATE jobs SET lease_expires = 1 WHERE job_id = ?").bind(id).run(); // stale in D1
    const exp = await runInDurableObject(hub(), async (h: Hub) => {
      await h.load(true);
      return h.jobs.get(id)!.lease_expires!;
    });
    expect(exp).toBeGreaterThan(Date.now() + 200_000);
    const hb = await callJson("POST", "/v1/heartbeat", { token: w.token, json: { job_ids: [id] } });
    expect(hb.body.extended).toEqual([id]);
    expect(await env.DB.prepare("SELECT lease_expires FROM jobs WHERE job_id = ?").bind(id).first("lease_expires")).toBe(1);
    await runInDurableObject(hub(), (h: Hub) => h.flush());
    expect(await env.DB.prepare("SELECT lease_expires FROM jobs WHERE job_id = ?").bind(id).first("lease_expires")).toBeGreaterThan(Date.now());
  });

  it("serves the feed from memory, re-reading history only after a change", async () => {
    await withEnv({ FEED_CACHE_S: "15", FEED_HISTORY_S: "0" });
    try {
      const [id] = await addJobs([{ seeds: [1, 2] }]);
      const w = await makeWorker("w");
      await lease(w);
      await callJson("GET", "/v1/feed"); // first build reads the history
      await new Promise((r) => setTimeout(r, 2100));
      const before = await hubReads();
      for (let i = 0; i < 5; i++) await runInDurableObject(hub(), (h: Hub) => h.feed(30));
      expect(await hubReads()).toBe(before); // unchanged: no D1
      await callJson("POST", `/v1/jobs/${id}/runs`, { token: w.token, json: { seed: 1, wall_s: 3, metrics: { score: 2 } } });
      await new Promise((r) => setTimeout(r, 2100));
      const f = JSON.parse(await runInDurableObject(hub(), (h: Hub) => h.feed(30)));
      const j = f.jobs.find((x: { id: string }) => x.id === id);
      expect(j.progress).toMatchObject({ runs_ok: 1 }); // live aggregates from memory
      expect(j.metrics.score).toEqual({ mean: 2, n: 1 });
      expect(f.runs.map((r: { seed: number }) => r.seed)).toEqual([1]); // history re-read after the run
      expect(f.budget).toMatchObject({ degraded: false });
    } finally {
      await withEnv({ FEED_CACHE_S: "0", FEED_HISTORY_S: "60" });
    }
  });

  it("the read guard freezes the history past 60% of the budget and the stats name the routes", async () => {
    const h0 = new Date().toISOString().slice(0, 13);
    await runInDurableObject(hub(), (h: Hub) => h.addReads({ [h0]: { "GET /v1/feed": { r: 3_100_000, w: 0, q: 10 } } }));
    const r = await admin("GET", "/v1/admin/reads");
    expect(r.status).toBe(200);
    expect(r.body).toMatchObject({ degraded: true, budget: 5_000_000, guard_frac: 0.6 });
    expect(r.body.reads_today).toBeGreaterThanOrEqual(3_100_000);
    expect(r.body.routes[0]).toMatchObject({ label: "GET /v1/feed", reads: 3_100_000 });
    expect(r.body.hours.at(-1).hour).toBe(h0);
    const feed = (await callJson("GET", "/v1/feed")).body;
    expect(feed.budget.degraded).toBe(true);
    expect((await callJson("GET", "/v1/admin/reads")).status).toBe(401);
    const swept = await runInDurableObject(hub(), (h: Hub) => h.sweep());
    expect(swept.degraded).toBe(true);
    await runInDurableObject(hub(), (h: Hub) => {
      h.book = {};
    });
  });

  it("the Worker tallies its own D1 reads per route and pushes them to the Hub", async () => {
    await admin("GET", "/v1/status"); // reads D1 in the Worker
    await pushReads(env, undefined, true);
    const r = await admin("GET", "/v1/admin/reads");
    const labels = r.body.routes.map((x: { label: string }) => x.label);
    expect(labels).toContain("GET /v1/status");
    expect(labels.some((l: string) => l.startsWith("hub:"))).toBe(true);
  });

  it("the SSE stream comes from the Hub", async () => {
    const r = await call("GET", "/v1/feed/stream?once=1");
    expect(r.status).toBe(200);
    const text = await r.text();
    expect(text).toMatch(/^retry: 5000/);
    expect(JSON.parse(text.split("data: ")[1]).schema).toBe("crane-feed/1");
  });
});

describe("countingDb", () => {
  it("counts rows read from each statement's meta, for all, first, run and batch", async () => {
    await env.DB.prepare("CREATE TABLE IF NOT EXISTS t_count (x INTEGER)").run();
    await env.DB.prepare("DELETE FROM t_count").run();
    await env.DB.prepare("INSERT INTO t_count SELECT value FROM json_each('[1,2,3,4,5]')").run();
    const book: Book = {};
    const db = countingDb(env.DB, () => "test", (l, m) => {
      const h = (book.h ??= {});
      const c = (h[l] ??= { r: 0, w: 0, q: 0 });
      c.r += m?.rows_read ?? 0;
      c.w += m?.rows_written ?? 0;
      c.q++;
    });
    expect((await db.prepare("SELECT x FROM t_count").all()).results).toHaveLength(5);
    expect(await db.prepare("SELECT x FROM t_count ORDER BY x").first("x")).toBe(1);
    await db.batch([db.prepare("SELECT COUNT(*) FROM t_count"), db.prepare("UPDATE t_count SET x = x + 1 WHERE x = ?").bind(5)]);
    expect(book.h.test.q).toBe(4);
    expect(book.h.test.r).toBeGreaterThanOrEqual(15);
    expect(book.h.test.w).toBeGreaterThanOrEqual(1);
  });
});
