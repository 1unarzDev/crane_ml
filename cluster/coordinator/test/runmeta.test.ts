import { beforeEach, describe, expect, it } from "vitest";
import { ADMIN, addJobs, admin, callJson, lease, makeWorker, resetTables } from "./helpers";

beforeEach(resetTables);

const post = (token: string, job: string, body: unknown) => callJson("POST", `/v1/jobs/${job}/runs`, { token, json: body });
const put = (token: string, job: string, run: string, name: string, body: string) =>
  callJson("PUT", `/v1/jobs/${job}/artifacts/${name}?retention=summary&run=${run}`, { token, body: new TextEncoder().encode(body) });
const query = (q: string) => callJson("GET", `/v1/runs/query?${q}`, { token: ADMIN });

describe("run_meta", () => {
  it("indexes a run's meta and selects by study, iteration, outcome and per-iteration top k", async () => {
    const ids = await addJobs([{ seeds: [1, 2, 3], spec: {} }, { seeds: [4], spec: {} }]);
    const w = await makeWorker("w");
    await lease(w, 2);
    const scores = [0.2, 0.9, 0.5];
    for (const [i, seed] of [1, 2, 3].entries()) {
      const r = await post(w.token, ids[0], {
        seed,
        metrics: { m: scores[i] },
        meta: { study: "s1", iteration: 0, score: scores[i], outcome: i === 0 ? "failed" : "finished", valid: true,
                params: { "a.b": i }, traj_name: "run.ctraj", recorder: "fastsim", progress: 0.5 },
      });
      expect(r.status).toBe(200);
      if (seed === 2) expect((await put(w.token, ids[0], r.body.run_id, "run.ctraj", "traj-bytes")).status).toBe(200);
    }
    expect((await post(w.token, ids[1], { seed: 4, metrics: {}, meta: { study: "s1", iteration: 1, score: 0.1 } })).status).toBe(200);

    const all = await query("study=s1");
    expect(all.status).toBe(200);
    expect(all.body.runs.map((r: { seed: number }) => r.seed)).toEqual([2, 3, 1, 4]);
    const best = all.body.runs[0];
    expect(best).toMatchObject({ study: "s1", iteration: 0, rank_in_iteration: 1, valid: true, params: { "a.b": 1 },
                                 traj_key: `artifacts/${ids[0]}/${ids[0]}.a1.s2/run.ctraj`, traj_size: 10, outcome: "finished" });
    expect(best.traj_sha256).toHaveLength(64);
    expect(all.body.runs[1].traj_size).toBeNull();
    expect(all.body.runs[3].outcome).toBe("unknown");

    expect((await query("study=s1&top_k=1")).body.runs.map((r: { seed: number }) => r.seed)).toEqual([2, 4]);
    expect((await query("study=s1&outcome=failed")).body.runs.map((r: { seed: number }) => r.seed)).toEqual([1]);
    expect((await query("study=s1&iter_from=1")).body.runs.map((r: { seed: number }) => r.seed)).toEqual([4]);
    expect((await query("study=s1&order=asc&top_k=1")).body.runs.map((r: { seed: number }) => r.seed)).toEqual([1, 4]);
    const page = await query("study=s1&limit=3");
    expect(page.body.runs).toHaveLength(3);
    expect(page.body.next_offset).toBe(3);
    expect((await query("study=s1&limit=3&offset=3")).body.runs).toHaveLength(1);
  });

  it("validates meta and the query; viewers only", async () => {
    const [id] = await addJobs([{ seeds: [1] }]);
    const w = await makeWorker("w");
    await lease(w);
    expect((await post(w.token, id, { seed: 1, metrics: {}, meta: { outcome: "exploded" } })).status).toBe(400);
    expect((await post(w.token, id, { seed: 1, metrics: {}, meta: { traj_name: "../x" } })).status).toBe(400);
    expect((await post(w.token, id, { seed: 1, metrics: {}, meta: { score: "high" } })).status).toBe(400);
    expect((await post(w.token, id, { seed: 1, metrics: {}, meta: { params: [1] } })).status).toBe(400);
    expect((await post(w.token, id, { seed: 1, metrics: {} })).status).toBe(200);
    expect((await query("")).status).toBe(400);
    expect((await query("study=x&order=sideways")).status).toBe(400);
    expect((await callJson("GET", "/v1/runs/query?study=x", { token: w.token })).status).toBe(401);
    expect((await query("study=adhoc")).body.runs).toEqual([]);
  });

  it("keeps only each job's final attempt and takes study and iteration from a hypothesis job", async () => {
    const [id] = await addJobs([{ seeds: [1], max_attempts: 3 }]);
    const w = await makeWorker("w");
    await lease(w);
    await post(w.token, id, { seed: 1, metrics: {}, meta: { study: "s", score: 1 } });
    await callJson("POST", `/v1/jobs/${id}/complete`, { token: w.token, json: { status: "failed", error: "x" } });
    await lease(w);
    await post(w.token, id, { seed: 1, metrics: {}, meta: { study: "s", score: 2 } });
    const r = await query("study=s");
    expect(r.body.runs.map((x: { score: number; attempt: number }) => [x.score, x.attempt])).toEqual([[2, 2]]);
    await admin("GET", "/v1/status");
  });
});
