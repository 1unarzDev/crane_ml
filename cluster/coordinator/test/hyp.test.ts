import { beforeEach, describe, expect, it } from "vitest";
import { GPU_CAPS, admin, callJson, lease, makeWorker, resetTables } from "./helpers";

beforeEach(resetTables);

const HYP = {
  name: "faster-turns",
  claim: "a larger turn gain raises the score",
  metric: "score",
  direction: "increase",
  purpose: "Tests whether a larger turn gain raises the course score on the fast tier.",
  est_seconds: 20,
  submitter: "vitest",
  baseline_ref: "base",
  variants: [
    { name: "a", ref: "va", params: { effect: 0.2 } },
    { name: "b", ref: "vb" },
  ],
  scenario: "family-1",
  seeds: [1, 2, 3],
  budget: { max_runs: 40 },
  stages: [
    { name: "fast", backend: "dummy" },
    { name: "gpu", backend: "dummy", needs: { gpu: true } },
  ],
};

async function runAll(w: { token: string; caps: Record<string, unknown> }, score: (j: any, seed: number) => number) {
  for (;;) {
    const r = await lease(w, 4);
    if (r.body.jobs.length === 0) return;
    for (const j of r.body.jobs) {
      for (const s of j.seeds) await callJson("POST", `/v1/jobs/${j.job_id}/runs`, { token: w.token, json: { seed: s, metrics: { score: score(j, s) } } });
      await callJson("POST", `/v1/jobs/${j.job_id}/complete`, { token: w.token, json: { status: "ok" } });
    }
  }
}

describe("hypotheses", () => {
  it("stages: same seeds per arm, results by arm, promote/extend/candidate, budget", async () => {
    const sub = await admin("POST", "/v1/hyp", HYP);
    expect(sub.status).toBe(200);
    expect(sub.body.jobs).toBe(9); // 3 arms x 3 seeds
    const id = sub.body.id;
    const cpu = await makeWorker("cpu");
    await runAll(cpu, (j, s) => s + (j.spec.effect ?? 0));
    const res = await admin("GET", `/v1/hyp/${id}/results`);
    expect(res.body.pending).toBe(0);
    expect(res.body.rows).toHaveLength(9);
    expect(res.body.rows.filter((r: any) => r.arm === "a").map((r: any) => r.value).sort()).toEqual([1.2, 2.2, 3.2]);

    // extend the current stage for variant a only (b is dropped by halving)
    const ext = await admin("POST", `/v1/hyp/${id}/advance`, { action: "extend", variants: ["a"], seeds: [4, 5] });
    expect(ext.body).toMatchObject({ status: "running", stage: "fast", jobs: 4 });
    await runAll(cpu, (j, s) => s + (j.spec.effect ?? 0));

    const pro = await admin("POST", `/v1/hyp/${id}/advance`, { action: "promote", evidence: { p: 0.01 } });
    expect(pro.body).toMatchObject({ stage: "gpu", jobs: 6 });
    // stage 2 needs a GPU: the CPU worker gets nothing
    expect((await lease(cpu, 4)).body.jobs).toHaveLength(0);
    await runAll(await makeWorker("gpu", GPU_CAPS), () => 1);
    const st = await admin("GET", `/v1/hyp/${id}`);
    expect(st.body.variants.map((v: any) => v.active)).toEqual([true, false]);
    // budget: 9 + 4 + 6 = 19 runs used; extending by 11 seeds x 2 arms would exceed 40
    const over = await admin("POST", `/v1/hyp/${id}/advance`, { action: "extend", seeds: Array.from({ length: 11 }, (_, i) => 10 + i) });
    expect(over.status).toBe(409);
    const done = await admin("POST", `/v1/hyp/${id}/advance`, { action: "promote" });
    expect(done.body).toMatchObject({ status: "candidate" });
    const fin = await admin("GET", `/v1/hyp/${id}`);
    expect(fin.body.decisions.map((d: any) => d.action)).toEqual(["extend", "promote", "promote"]);
    expect((await admin("POST", `/v1/hyp/${id}/advance`, { action: "promote" })).status).toBe(409);
  });

  it("reject cancels queued jobs; bad specs are refused", async () => {
    const sub = await admin("POST", "/v1/hyp", HYP);
    const rej = await admin("POST", `/v1/hyp/${sub.body.id}/advance`, { action: "reject" });
    expect(rej.body.status).toBe("rejected");
    const jobs = await admin("GET", `/v1/jobs?hypothesis=${sub.body.id}`);
    expect(jobs.body.jobs.every((j: any) => j.status === "cancelled")).toBe(true);
    expect((await admin("POST", "/v1/hyp", { ...HYP, direction: "up" })).status).toBe(400);
    expect((await admin("POST", "/v1/hyp", { ...HYP, variants: [{ name: "baseline", ref: "x" }] })).status).toBe(400);
    expect((await admin("POST", "/v1/hyp", { ...HYP, stages: [{ backend: "dummy", needs: { gpus: 2 } }] })).status).toBe(400);
  });
});
