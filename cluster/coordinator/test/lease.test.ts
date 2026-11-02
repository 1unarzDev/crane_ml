import { env } from "cloudflare:test";
import { beforeEach, describe, expect, it } from "vitest";
import { CPU_CAPS, GPU_CAPS, addJobs, admin, callJson, expireLeases, flushHub, lease, makeWorker, resetTables, runCron } from "./helpers";

beforeEach(resetTables);

describe("leases", () => {
  it("concurrent leases never hand a job out twice", async () => {
    const ids = await addJobs(Array.from({ length: 12 }, () => ({})));
    const ws = await Promise.all(Array.from({ length: 10 }, (_, i) => makeWorker(`racer${i}`)));
    const res = await Promise.all(ws.map((w) => lease(w, 2)));
    const got = res.flatMap((r) => r.body.jobs.map((j: { job_id: string }) => j.job_id));
    expect(new Set(got).size).toBe(got.length);
    expect(new Set(got)).toEqual(new Set(ids));
  });

  it("matches needs to capabilities and honours priority", async () => {
    const [gpuJob] = await addJobs([{ needs: { gpu: true, vulkan: true }, priority: 1 }]);
    const [cpuJob] = await addJobs([{ priority: 50 }]);
    const cpu = await makeWorker("cpu");
    const gpu = await makeWorker("gpu", GPU_CAPS);
    const a = await lease(cpu, 4);
    expect(a.body.jobs.map((j: { job_id: string }) => j.job_id)).toEqual([cpuJob]);
    const b = await lease(gpu, 4);
    expect(b.body.jobs.map((j: { job_id: string }) => j.job_id)).toEqual([gpuJob]);
    expect(b.body.jobs[0]).toMatchObject({ backend: "dummy", seeds: [0], attempt: 1, needs: { gpu: true, vulkan: true } });
  });

  it("only the backends a worker registered are leased", async () => {
    await addJobs([{ backend: "render" }]);
    const w = await makeWorker("w");
    expect((await lease(w)).body.jobs).toHaveLength(0);
    expect((await lease({ ...w, caps: { ...CPU_CAPS, backends: ["render"] } })).body.jobs).toHaveLength(1);
  });

  it("an expired lease is retried, then dead-lettered after max attempts", async () => {
    const [id] = await addJobs([{ max_attempts: 2 }]);
    const a = await makeWorker("a");
    const b = await makeWorker("b");
    expect((await lease(a)).body.jobs[0].job_id).toBe(id);
    await expireLeases();
    await runCron();
    let row = await env.DB.prepare("SELECT status, attempt, last_error FROM jobs WHERE job_id = ?").bind(id).first<any>();
    expect(row).toMatchObject({ status: "queued", attempt: 1 });
    expect(row.last_error).toContain("lease expired");
    // the dead worker's heartbeat now learns the job was revoked
    const hb = await callJson("POST", "/v1/heartbeat", { token: a.token, json: { job_ids: [id] } });
    expect(hb.body).toEqual({ extended: [], revoked: [id] });
    expect((await lease(b)).body.jobs[0]).toMatchObject({ job_id: id, attempt: 2 });
    await expireLeases();
    await runCron();
    row = await env.DB.prepare("SELECT status FROM jobs WHERE job_id = ?").bind(id).first<any>();
    expect(row.status).toBe("dead");
    const outcomes = await env.DB.prepare("SELECT outcome FROM leases WHERE job_id = ? ORDER BY attempt").bind(id).all<any>();
    expect(outcomes.results.map((r) => r.outcome)).toEqual(["expired", "expired"]);
  });

  it("heartbeat extends, release requeues without spending an attempt", async () => {
    const [id] = await addJobs([{}]);
    const w = await makeWorker("w");
    await lease(w);
    await expireLeases(Date.now() + 1000);
    const hb = await callJson("POST", "/v1/heartbeat", { token: w.token, json: { job_ids: [id], state: "running" } });
    expect(hb.body.extended).toEqual([id]);
    await flushHub(); // the Hub writes lease expiry to D1 in batches
    const exp = await env.DB.prepare("SELECT lease_expires FROM jobs WHERE job_id = ?").bind(id).first<{ lease_expires: number }>();
    expect(exp!.lease_expires).toBeGreaterThan(Date.now() + 200_000);
    const rel = await callJson("POST", "/v1/release", { token: w.token, json: { job_ids: [id] } });
    expect(rel.body.released).toEqual([id]);
    expect(await env.DB.prepare("SELECT status, attempt FROM jobs WHERE job_id = ?").bind(id).first()).toEqual({ status: "queued", attempt: 0 });
  });

  it("runs, metrics and completion; a failed run is retried", async () => {
    const [id] = await addJobs([{ seeds: [1, 2] }]);
    const w = await makeWorker("w");
    await lease(w);
    const other = await makeWorker("other");
    expect((await callJson("POST", `/v1/jobs/${id}/runs`, { token: other.token, json: { seed: 1 } })).status).toBe(409);
    expect((await callJson("POST", `/v1/jobs/${id}/runs`, { token: w.token, json: { seed: 9 } })).status).toBe(400);
    const r1 = await callJson("POST", `/v1/jobs/${id}/runs`, { token: w.token, json: { seed: 1, metrics: { score: 0.5, t: 3 }, wall_s: 2 } });
    expect(r1.body.run_id).toBe(`${id}.a1.s1`);
    const fail = await callJson("POST", `/v1/jobs/${id}/complete`, { token: w.token, json: { status: "failed", error: "boom" } });
    expect(fail.body.status).toBe("queued");
    await lease(w);
    await callJson("POST", `/v1/jobs/${id}/runs`, { token: w.token, json: { seed: 1, metrics: { score: 0.7 } } });
    await callJson("POST", `/v1/jobs/${id}/runs`, { token: w.token, json: { seed: 2, metrics: { score: 0.9 } } });
    const ok = await callJson("POST", `/v1/jobs/${id}/complete`, { token: w.token, json: { status: "ok" } });
    expect(ok.body.status).toBe("done");
    const runs = await admin("GET", `/v1/jobs/${id}/runs`);
    expect(runs.body.runs.map((r: any) => [r.attempt, r.seed, r.metrics.score])).toEqual([[1, 1, 0.5], [2, 1, 0.7], [2, 2, 0.9]]);
  });

  it("client-version gate returns 426 below the minimum", async () => {
    await addJobs([{}]);
    const w = await makeWorker("old");
    await admin("PUT", "/v1/admin/client_release", { version: "0.3.0", min_version: "0.2.0" });
    const r = await lease(w, 1, { client_version: "0.1.9" });
    expect(r.status).toBe(426);
    expect(r.body.min_client_version).toBe("0.2.0");
    expect((await lease(w, 1, { client_version: "0.2.0" })).status).toBe(200);
  });

  it("invites are single-use; tokens are kind-specific", async () => {
    const inv = await admin("POST", "/v1/admin/invite", { n: 1 });
    const code = inv.body.invites[0];
    expect((await callJson("POST", "/v1/join", { json: { invite: code, name: "x", caps: CPU_CAPS } })).status).toBe(200);
    expect((await callJson("POST", "/v1/join", { json: { invite: code, name: "y", caps: CPU_CAPS } })).status).toBe(403);
    const w = await makeWorker("w");
    expect((await callJson("POST", "/v1/admin/invite", { token: w.token, json: {} })).status).toBe(401);
    const v = await admin("POST", "/v1/admin/viewer_token", { name: "me" });
    expect((await callJson("GET", "/v1/status", { token: v.body.token })).status).toBe(200);
    expect((await callJson("POST", "/v1/lease", { token: v.body.token, json: {} })).status).toBe(401);
  });
});
