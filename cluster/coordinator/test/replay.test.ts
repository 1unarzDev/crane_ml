import { env } from "cloudflare:test";
import { beforeEach, describe, expect, it } from "vitest";
// @ts-ignore: the page's plain ES module
import { crc32, decode, parse, track, withHeader } from "../public/traj.js";
import { ADMIN, addJobs, admin, call, callJson, lease, makeWorker, resetTables } from "./helpers";

beforeEach(resetTables);

const golden = () => Uint8Array.from(atob(env.GOLDEN_CTRAJ), (ch) => ch.charCodeAt(0));
const want = () => JSON.parse(env.GOLDEN_JSON);
const hex = async (b: Uint8Array) => [...new Uint8Array(await crypto.subtle.digest("SHA-256", b))].map((x) => x.toString(16).padStart(2, "0")).join("");

describe("traj.js reads crane-traj/1 as the Python writer writes it", () => {
  it("decodes the golden file to the golden values", async () => {
    const bytes = golden();
    const w = want();
    expect(await hex(bytes)).toBe(w.sha256);
    const d = await decode(bytes);
    expect(d.ticks).toBe(w.ticks);
    expect(d.hz).toBe(w.hz);
    expect(Object.keys(d.columns)).toEqual(w.columns);
    expect(parse(bytes).index).toHaveLength(w.blocks);
    expect(d.events).toEqual(w.events);
    for (const [i, sample] of Object.entries(w.samples as Record<string, Record<string, number>>))
      for (const [k, v] of Object.entries(sample)) expect(Math.abs(d.columns[k][Number(i)] - v), `${k}[${i}]`).toBeLessThan(0.0051);
    const tr = track(d);
    expect(tr.x.length).toBe(w.ticks);
    expect(tr.t[20]).toBeCloseTo(1, 9);
  });

  it("rewrites only the header and keeps the crc valid; refuses corrupt files", async () => {
    const bytes = golden();
    const out = withHeader(bytes, { seed: 9, hz: 20 });
    expect(parse(out).header).toEqual({ hz: 20, seed: 9 });
    const a = await decode(bytes);
    const b = await decode(out);
    expect(b.columns["boat.x"]).toEqual(a.columns["boat.x"]);
    expect(b.events).toEqual(a.events);
    const bad = bytes.slice();
    bad[100] ^= 1;
    expect(() => parse(bad)).toThrow(/crc/);
    expect(crc32(new TextEncoder().encode("123456789"))).toBe(0xcbf43926);
  });
});

/** A worker that ran `seeds` of one job in study `study`, uploading run.ctraj (+ LOD and course truth). */
async function recorded(study: string, seeds: number[], o: { lod?: boolean; truth?: string; arm?: string; iteration?: number; scores?: number[] } = {}) {
  const [id] = await addJobs([{ seeds, spec: {} }]);
  const w = await makeWorker(`w-${study}-${o.iteration ?? 0}`);
  await lease(w);
  const g = golden();
  const header = { ...parse(g).header, params: { "FollowPath.v": 1.1, secret: "/home/x/y" }, build: { crane_commit: "abc" }, study, course: { truth_sha256: o.truth ?? null, name: "c1", pair_key: "pk" } };
  const full = withHeader(g, header);
  const runs: string[] = [];
  for (const [i, seed] of seeds.entries()) {
    const r = await callJson("POST", `/v1/jobs/${id}/runs`, {
      token: w.token,
      json: { seed, wall_s: 2, metrics: {}, meta: { study, iteration: o.iteration ?? 0, score: o.scores?.[i] ?? seed, outcome: "finished", traj_name: "run.ctraj", course_sha256: o.truth } },
    });
    const run = r.body.run_id;
    runs.push(run);
    const put = (name: string, body: Uint8Array, retention = "summary") =>
      call("PUT", `/v1/jobs/${id}/artifacts/${name}?retention=${retention}&run=${run}`, { token: w.token, body });
    expect((await put("run.ctraj", full)).status).toBe(200);
    if (o.lod) await put("run.lod.ctraj", withHeader(g, { lod: { hz: 2, of_hz: 20 }, hz: 20 }));
    if (o.truth)
      await put("course_truth.json", new TextEncoder().encode(JSON.stringify({ name: "c1", startX: 1, boundary: [[0, 0], [10, 0]], elements: [{ x: 3, y: 1, radius: 0.2, label: "red_pole_buoy", role: "gate", task: "entry_gates", id: "secret-id" }] })), "raw");
  }
  return { id, runs, w };
}

describe("public replay endpoints", () => {
  it("serve a run's trajectory with a public header, the LOD on request, and the course", async () => {
    const truth = "ab".repeat(32);
    const { id, runs } = await recorded("s1", [1, 2], { lod: true, truth });
    const r = await call("GET", `/v1/feed/runs/${runs[0]}/traj`);
    expect(r.status).toBe(200);
    expect(r.headers.get("access-control-allow-origin")).toBe("*");
    const body = new Uint8Array(await r.arrayBuffer());
    const h = parse(body).header;
    expect(h).toMatchObject({ seed: 7, study: "s1", course: { truth_sha256: truth, name: "c1" } });
    for (const k of ["params", "build"]) expect(h).not.toHaveProperty(k);
    expect(h.course).not.toHaveProperty("pair_key");
    expect((await decode(body)).ticks).toBe(700);

    const lod = await call("GET", `/v1/feed/runs/${runs[0]}/traj?lod=1`);
    expect(lod.headers.get("x-crane-lod")).toBe("1");
    expect(parse(new Uint8Array(await lod.arrayBuffer())).header.lod).toEqual({ hz: 2, of_hz: 20 });

    const feed = (await callJson("GET", `/v1/feed/jobs/${id}`)).body;
    expect(feed.job.replay).toMatchObject({ available: true, runs: 2, archived: 0, study: "s1" });
    expect(feed.runs[0]).toMatchObject({ replay: true, replay_state: "ready", course: truth });
    expect(feed.runs[0].traj_sha256).toHaveLength(64);
    const imm = await call("GET", `/v1/feed/runs/${runs[0]}/traj?h=${feed.runs[0].traj_sha256}`);
    expect(imm.headers.get("cache-control")).toContain("immutable");

    const course = await callJson("GET", `/v1/feed/courses/${truth}`);
    expect(course.status).toBe(200);
    expect(course.body).toEqual({ name: "c1", family: null, start: [1, 0, 0], boundary: [[0, 0], [10, 0]], elements: [{ x: 3, y: 1, r: 0.2, label: "red_pole_buoy", role: "gate", task: "entry_gates" }] });
    expect(await env.BUCKET.get(`courses/${truth}.json`)).not.toBeNull();
    expect((await call("GET", `/v1/feed/courses/${"cd".repeat(32)}`)).status).toBe(404);
    expect((await call("GET", "/v1/feed/courses/nothex")).status).toBe(400);
    expect((await call("GET", "/v1/feed/runs/nope/traj")).status).toBe(404);
  });

  it("say archived when only the archive has a trajectory (503 while it is offline) and gone when nothing has it", async () => {
    const { id, runs } = await recorded("s2", [1]);
    const key = `artifacts/${id}/${runs[0]}/run.ctraj`;
    await env.BUCKET.delete(key);
    await env.DB.prepare("UPDATE artifacts SET archived_at = '2026-01-01T00:00:00Z', r2_deleted_at = '2026-01-02T00:00:00Z' WHERE key = ?1").bind(key).run();
    const off = await callJson("GET", `/v1/feed/runs/${runs[0]}/traj`);
    expect(off.status).toBe(503);
    expect(off.body).toMatchObject({ tier: "archive", state: "offline" });
    let feed = (await callJson("GET", `/v1/feed/jobs/${id}`)).body;
    expect(feed.runs[0].replay_state).toBe("archived");
    expect(feed.job.replay).toMatchObject({ available: true, archived: 1 });
    await env.DB.prepare("UPDATE artifacts SET deleted_at = '2026-01-03T00:00:00Z' WHERE key = ?1").bind(key).run();
    const gone = await callJson("GET", `/v1/feed/runs/${runs[0]}/traj`);
    expect(gone.status).toBe(410);
    expect(gone.body.error).toBe("gone");
    feed = (await callJson("GET", `/v1/feed/jobs/${id}`)).body;
    expect(feed.runs[0]).toMatchObject({ replay: false, replay_state: "gone" });
  });

  it("find the same seed's ghosts in the study and pack a fleet by iteration", async () => {
    const a = await recorded("s3", [1, 2, 3], { iteration: 0, scores: [5, 9, 1] });
    const b = await recorded("s3", [1, 2], { iteration: 1, scores: [7, 8] });
    await recorded("other", [1]);
    const gh = await callJson("GET", `/v1/feed/ghosts?run=${a.runs[0]}`);
    expect(gh.status).toBe(200);
    expect(gh.body.runs.map((r: { run_id: string }) => r.run_id)).toEqual([b.runs[0]]);
    expect((await callJson("GET", "/v1/feed/ghosts?study=s3&seed=2")).body.runs).toHaveLength(2);
    expect((await callJson("GET", "/v1/feed/ghosts")).status).toBe(400);

    const f = await callJson("GET", "/v1/feed/fleet?study=s3&k=2");
    expect(f.status).toBe(200);
    expect(f.body.iterations).toEqual([0, 1]);
    expect(f.body.runs.map((r: { iteration: number; score: number }) => [r.iteration, r.score])).toEqual([[0, 9], [0, 5], [1, 8], [1, 7]]);
    const pack = new Uint8Array(await (await call("GET", `/v1/feed/fleet.pack?study=s3&k=2&h=${f.body.pack_sha256}`)).arrayBuffer());
    const v = new DataView(pack.buffer);
    let p = 0;
    const files = [];
    while (p < pack.length) {
      const n = v.getUint32(p, true);
      files.push(pack.subarray(p + 4, p + 4 + n));
      p += 4 + n;
    }
    expect(files).toHaveLength(4);
    expect(parse(files[0]).header.study).toBe("s3");
    expect((await callJson("GET", "/v1/feed/fleet")).status).toBe(400);
  });
});

describe("theater renders", () => {
  it("needs an operator token to request, shows requests publicly and serves videos with Range", async () => {
    const [job] = await addJobs([{ seeds: [1] }]);
    const purpose = "Shows the tuned lookahead against the baseline on the task-1 course for the review.";
    expect((await callJson("POST", "/v1/operator/renders", { json: { job_id: job, purpose } })).status).toBe(401);
    expect((await callJson("GET", "/v1/operator/whoami", { token: "nope" })).status).toBe(401);
    expect((await callJson("POST", "/v1/admin/operator_token", { json: { name: "x" } })).status).toBe(401);
    const tok = (await admin("POST", "/v1/admin/operator_token", { name: "owner laptop" })).body.token;
    expect((await callJson("GET", "/v1/operator/whoami", { token: tok })).body).toEqual({ name: "owner laptop" });
    // An operator token is not an admin or viewer token.
    expect((await callJson("GET", "/v1/admin/renders", { token: tok })).status).toBe(401);
    expect((await callJson("POST", "/v1/operator/renders", { token: tok, json: { job_id: job, purpose: "render" } })).status).toBe(400);
    expect((await callJson("POST", "/v1/operator/renders", { token: tok, json: { job_id: "j_nope", purpose } })).status).toBe(404);
    const r = await callJson("POST", "/v1/operator/renders", { token: tok, json: { job_id: job, purpose } });
    expect(r.body.status).toBe("requested");
    expect((await callJson("POST", "/v1/operator/renders", { token: tok, json: { job_id: job, purpose } })).body).toEqual({ id: r.body.id, status: "already requested" });

    const pub = await callJson("GET", `/v1/feed/renders?job=${job}`);
    expect(pub.body.renders).toHaveLength(1);
    expect(pub.body.renders[0]).toMatchObject({ status: "requested", requested_by: "owner laptop", purpose, video: null });
    expect((await admin("GET", "/v1/admin/renders?status=requested")).body.renders).toHaveLength(1);
    expect((await admin("POST", `/v1/admin/renders/${r.body.id}`, { status: "rendering", render_job_id: "j_render" })).status).toBe(200);

    const video = new Uint8Array(1000).map((_, i) => i % 251);
    const up = await call("PUT", `/v1/admin/renders/${r.body.id}/video/final.mp4`, { token: ADMIN, body: video });
    expect(up.status).toBe(200);
    const done = (await callJson("GET", `/v1/feed/renders?job=${job}`)).body.renders[0];
    expect(done).toMatchObject({ status: "done", video: { url: `/v1/feed/renders/${r.body.id}/video`, type: "video/mp4", size: 1000 } });
    const whole = await call("GET", done.video.url);
    expect(whole.status).toBe(200);
    expect(new Uint8Array(await whole.arrayBuffer())).toEqual(video);
    const part = await call("GET", done.video.url, { headers: { range: "bytes=10-19" } });
    expect(part.status).toBe(206);
    expect(part.headers.get("content-range")).toBe("bytes 10-19/1000");
    expect(new Uint8Array(await part.arrayBuffer())).toEqual(video.slice(10, 20));
    expect((await call("GET", done.video.url, { headers: { range: "bytes=2000-" } })).status).toBe(416);
    expect((await call("PUT", `/v1/admin/renders/${r.body.id}/video/x.exe`, { token: ADMIN, body: video })).status).toBe(400);
  });
});
