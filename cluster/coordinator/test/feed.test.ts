import { env } from "cloudflare:test";
import { beforeEach, describe, expect, it } from "vitest";
import { durationModel, modelRunSeconds, runWalls } from "../src/feed";
import { alias, checkPurpose, scrub } from "../src/purpose";
import { JOB_META, addJobs, admin, call, callJson, lease, makeWorker, resetTables } from "./helpers";

beforeEach(resetTables);

const report = (w: { token: string }, id: string, seed: number, wall_s: number, metrics: Record<string, number> = {}) =>
  callJson("POST", `/v1/jobs/${id}/runs`, { token: w.token, json: { seed, wall_s, metrics } });
const complete = (w: { token: string }, id: string) => callJson("POST", `/v1/jobs/${id}/complete`, { token: w.token, json: { status: "ok" } });

describe("job submission requires purpose, estimate and submitter", () => {
  const post = (j: Record<string, unknown>) => admin("POST", "/v1/admin/jobs", { jobs: [{ backend: "dummy", ...JOB_META, ...j }] });

  it("refuses missing, short, generic or secret-bearing purposes", async () => {
    for (const purpose of [undefined, "", "test", "quick sanity check", "test test test test test test", "smoke test"]) {
      const r = await post({ purpose });
      expect(r.status, String(purpose)).toBe(400);
      expect(r.body.error).toMatch(/purpose/);
    }
    const leak = await post({ purpose: "Retries the upload with token=abcd1234efgh5678 to see if auth was the cause." });
    expect(leak.status).toBe(400);
    expect(leak.body.error).toMatch(/secret/);
    const path = await post({ purpose: "Checks whether the plant card in /home/someone/x.yaml moves the score at all." });
    expect(path.status).toBe(400);
  });

  it("refuses a missing or silly estimate and a missing submitter", async () => {
    for (const est_seconds of [undefined, 0, -3, "60", 1e9]) expect((await post({ est_seconds })).status).toBe(400);
    expect((await post({ submitter: undefined })).status).toBe(400);
    expect((await post({ submitter: "" })).status).toBe(400);
    expect((await post({ campaign: "boat-test-2026-11-08" })).status).toBe(200);
  });

  it("hypotheses require them too and stage jobs inherit them", async () => {
    const hyp = {
      name: "h", claim: "c", metric: "score", direction: "increase", baseline_ref: "b", variants: [{ name: "v", ref: "v" }],
      seeds: [1, 2], stages: [{ name: "fast", backend: "dummy" }],
    };
    expect((await admin("POST", "/v1/hyp", hyp)).status).toBe(400);
    const ok = await admin("POST", "/v1/hyp", { ...hyp, purpose: "Tests whether variant v raises the score over the baseline.", est_seconds: 12, submitted_by: "agent-7", campaign: "c1" });
    expect(ok.status).toBe(200);
    const feed = (await callJson("GET", "/v1/feed")).body;
    expect(feed.jobs).toHaveLength(4);
    for (const j of feed.jobs) {
      expect(j.purpose).toBe("Tests whether variant v raises the score over the baseline.");
      expect(j.submitter).toBe("agent-7");
      expect(j.campaign).toBe("c1");
      expect(j.estimate.agent_run_s).toBe(12);
      expect(j.hypothesis).toMatchObject({ id: ok.body.id, name: "h", stage: "fast" });
    }
  });
});

describe("public feed", () => {
  it("needs no token, shows queue positions, ETAs, progress and worker aliases, and leaks nothing", async () => {
    const w = await makeWorker("lunarz@innovation.lan");
    const [a, b, c] = await addJobs([
      { seeds: [1, 2, 3, 4], priority: 1, spec: { workdir: "/home/lunarz/secret-dir", metric: "score" }, repos: [{ name: "org/r", url: "https://x:hunter2@example.com/r.git", ref: "abc" }] },
      { seeds: [1, 2], est_seconds: 100 },
      { seeds: [1] },
    ]);
    const leased = await lease(w, 1);
    expect(leased.body.jobs[0].job_id).toBe(a);
    await report(w, a, 1, 25, { score: 0.5, other: 2 });

    const r = await call("GET", "/v1/feed");
    expect(r.status).toBe(200);
    expect(r.headers.get("access-control-allow-origin")).toBe("*");
    const text = await r.text();
    const feed = JSON.parse(text);
    expect(feed.schema).toBe("crane-feed/1");
    for (const bad of [w.token, w.worker_id, "secret-dir", "hunter2", "example.com", "workdir", "lunarz", "\"needs\"", "\"spec\""])
      expect(text, bad).not.toContain(bad);

    const byId = Object.fromEntries(feed.jobs.map((j: any) => [j.id, j]));
    expect(byId[a]).toMatchObject({ state: "running", worker: "innovation", runs_total: 4, queue_position: null, progress: { runs_ok: 1, runs_failed: 0, fraction: 0.25 } });
    expect(byId[a].metrics.score).toEqual({ mean: 0.5, n: 1 });
    expect(Object.keys(byId[a].metrics)[0]).toBe("score");
    expect(byId[b]).toMatchObject({ state: "queued", queue_position: 1, estimate: { agent_run_s: 100, model_run_s: null, job_s: 200 } });
    expect(byId[c]).toMatchObject({ state: "queued", queue_position: 2 });
    // One worker: b starts when a ends and c when b ends.
    expect(Date.parse(byId[b].eta.start_at)).toBe(Date.parse(byId[a].eta.done_at));
    expect(Date.parse(byId[c].eta.start_at)).toBe(Date.parse(byId[b].eta.done_at));
    expect(byId[a].eta.remaining_s).toBe(90);
    expect(byId[b].eta.remaining_s).toBe(290);
    expect(byId[c].eta.remaining_s).toBe(320);
    expect(feed.summary).toMatchObject({ queued: 2, running: 1, queued_runs: 3, workers_live: 1 });
    expect(feed.workers).toEqual([expect.objectContaining({ alias: "innovation", live: true, job: a })]);
    expect(feed.runs[0]).toMatchObject({ job_id: a, seed: 1, worker: "innovation", replay: false });
  });

  it("a queued job no live worker can run says so", async () => {
    await makeWorker("cpu");
    const [id] = await addJobs([{ needs: { gpu: true } }]);
    const j = (await callJson("GET", "/v1/feed")).body.jobs.find((x: any) => x.id === id);
    expect(j.eta).toMatchObject({ start_at: null, done_at: null });
    expect(j.eta.note).toMatch(/no live worker/);
  });

  it("the duration model corrects the submitter's estimate after enough runs", async () => {
    const w = await makeWorker("simwin");
    // Submitters said 10 s; the runs took 30 s.
    const [done] = await addJobs([{ seeds: [1, 2, 3, 4, 5, 6], est_seconds: 10 }]);
    await lease(w, 1);
    for (const s of [1, 2, 3, 4, 5, 6]) await report(w, done, s, 30);
    await complete(w, done);
    const [next] = await addJobs([{ seeds: [1, 2], est_seconds: 20 }]);
    const feed = (await callJson("GET", "/v1/feed")).body;
    expect(feed.backends.dummy).toMatchObject({ median_run_s: 30, ratio: 3, n: 6 });
    const j = feed.jobs.find((x: any) => x.id === next);
    expect(j.estimate).toMatchObject({ agent_run_s: 20, model_run_s: 60, model_n: 6, job_s: 120 });
    const fin = feed.jobs.find((x: any) => x.id === done);
    expect(fin).toMatchObject({ state: "done", eta: null, progress: { runs_ok: 6, fraction: 1 } });
    expect(fin.ended_at).toBeTruthy();
  });

  it("legacy rows read (legacy); judge lines and run_meta insights show; secrets in old text are scrubbed", async () => {
    const [id] = await addJobs([{}]);
    await env.DB.prepare("UPDATE jobs SET purpose = '(legacy)', est_seconds = NULL, submitter = NULL, last_error = 'boom at /home/u/x key=sk-abcdefghijklmnopqrstu', status = 'dead', ended_at = ?2 WHERE job_id = ?1")
      .bind(id, new Date().toISOString())
      .run();
    expect((await admin("POST", "/v1/admin/jobs/judge", { job_id: id, line: "worse than baseline on 5 of 6 seeds" })).status).toBe(200);
    expect((await admin("POST", "/v1/admin/jobs/judge", { job_id: id, line: "see 192.168.1.4" })).status).toBe(400);
    expect((await callJson("POST", "/v1/admin/jobs/judge", { json: { job_id: id, line: "x" } })).status).toBe(401);
    const j = (await callJson("GET", `/v1/feed/jobs/${id}`)).body.job;
    expect(j).toMatchObject({ purpose: "(legacy)", state: "dead", judge: "worse than baseline on 5 of 6 seeds", estimate: { agent_run_s: null } });
    expect(j.error).not.toMatch(/home|sk-/);
  });

  it("SSE sends the snapshot as an event", async () => {
    await addJobs([{}]);
    const r = await call("GET", "/v1/feed/stream?once=1");
    expect(r.headers.get("content-type")).toMatch(/text\/event-stream/);
    const text = await r.text();
    expect(text).toMatch(/^retry: 5000\n\nevent: feed\ndata: \{"schema":"crane-feed\/1"/);
    expect((await call("GET", "/v1/feed/stream?interval=0")).status).toBe(400);
  });
});

describe("scrubber and aliases", () => {
  it("redacts secrets, credentials, addresses and paths but keeps ordinary text", () => {
    const keep = "Tests whether desired_linear_vel 1.2 beats 0.9 on roboboat_2026_t12 at commit 1a2b3c4d5e6f7a8b9c0d1e2f3a4b5c6d7e8f9a0b.";
    expect(scrub(keep)).toBe(keep);
    for (const s of [
      "Bearer abcdefghijklmnop", "ghp_abcdefghijklmnopqrstuvwxyz0123", "AKIAABCDEFGHIJKLMNOP", "sk-ant-api03-abcdefghijklmnopqrst",
      "password: hunter2", "https://u:p@host/x", "eyJhbGciOiJIUzI1.eyJzdWIiOiIxMjM0.SflKxwRJSMeKKF2QT4fwpM", "/home/lunarz/.config/x",
      "~/.ssh/id_ed25519", "C:\\Users\\me\\x", "10.0.0.5", "100.101.102.103", "me@example.com",
      "3q2-7wEXAMPLEk9Zt0kRandomToken0123456789abc",
    ])
      expect(scrub(`x ${s} y`), s).not.toContain(s);
  });

  it("checkPurpose accepts a real sentence", () => {
    expect(checkPurpose("  Tests whether the new plant epoch moves duration_s on the seed bank.  ")).toBe(
      "Tests whether the new plant epoch moves duration_s on the seed bank.",
    );
  });

  it("aliases are short host-like names", () => {
    expect(alias("lunarz@innovation.lan")).toBe("innovation");
    expect(alias("SimWin")).toBe("simwin");
    expect(alias("")).toBe(null);
  });

  it("model math", () => {
    const now = Date.now();
    const rows = Array.from({ length: 6 }, (_, i) => ({ backend: "b", worker_id: i < 3 ? "w1" : "w2", wall_s: i < 3 ? 10 : 40, est_seconds: 20, created: new Date(now).toISOString() }));
    const { backends, speed } = durationModel(rows, now);
    expect(backends.b).toMatchObject({ median_run_s: 25, ratio: 1.25, n: 6, runs_last_hour: 6 });
    expect(speed.get("w1\u0000b")).toBe(0.4);
    expect(speed.get("w2\u0000b")).toBe(1.6);
    expect(modelRunSeconds(8, backends.b)).toBe(10);
    expect(modelRunSeconds(null, backends.b)).toBe(25);
    expect(modelRunSeconds(8, undefined)).toBe(null);
  });

  it("a batched job's runs take their share of the lease", () => {
    const base = { job_id: "j", attempt: 1, run_attempt: 1, leased_at_ms: 1_000_000, ended_at: new Date(1_030_000).toISOString(), wall_s: 0.001 };
    const out = runWalls([base, base, { ...base, wall_s: 25 }, { ...base, run_attempt: 0 }, { ...base, job_id: "k", ended_at: null }]);
    expect(out.map((r) => r.wall_s)).toEqual([10, 10, 25, 0.001, 0.001]);
  });
});
