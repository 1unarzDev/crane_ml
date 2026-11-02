import { env } from "cloudflare:test";
import { afterEach, beforeEach, describe, expect, it, vi } from "vitest";
import { sweepArtifacts } from "../src/archive";
import type { Env } from "../src/util";
import { ADMIN, addJobs, admin, call, callJson, lease, makeWorker, resetTables, runCron } from "./helpers";

beforeEach(resetTables);
afterEach(() => vi.restoreAllMocks());

const put = (token: string, job: string, name: string, q: string, body: string) =>
  callJson("PUT", `/v1/jobs/${job}/artifacts/${name}${q}`, { token, body: new TextEncoder().encode(body) });
const age = (key: string, days: number) =>
  env.DB.prepare("UPDATE artifacts SET created = ?2, expires_at = CASE WHEN expires_at IS NULL THEN NULL ELSE ?2 END WHERE key = ?1")
    .bind(key, new Date(Date.now() - days * 86_400_000).toISOString())
    .run();
const row = (key: string) => env.DB.prepare("SELECT * FROM artifacts WHERE key = ?").bind(key).first<Record<string, any>>();

async function archiveToken() {
  const r = await admin("POST", "/v1/admin/archive_token", { name: "innovation" });
  expect(r.status).toBe(200);
  return r.body.token as string;
}

async function setup() {
  const [id] = await addJobs([{ seeds: [1, 2] }]);
  const w = await makeWorker("w");
  await lease(w);
  const runs: string[] = [];
  for (const [seed, score] of [[1, 0.2], [2, 0.9]]) {
    const r = await callJson("POST", `/v1/jobs/${id}/runs`, {
      token: w.token,
      json: { seed, metrics: { m: score }, meta: { study: "s", iteration: 0, score, valid: true, traj_name: "run.ctraj" } },
    });
    runs.push(r.body.run_id);
  }
  const up = async (run: string, name: string, retention: string, body: string) =>
    (await put(w.token, id, name, `?retention=${retention}&run=${run}`, body)).body.key as string;
  return {
    id,
    w,
    loser: { traj: await up(runs[0], "run.ctraj", "summary", "loser-traj"), log: await up(runs[0], "player.log", "raw", "loser-log"),
             lod: await up(runs[0], "run.lod.ctraj", "summary", "loser-lod") },
    best: { traj: await up(runs[1], "run.ctraj", "summary", "best-traj"), log: await up(runs[1], "player.log", "raw", "best-log") },
  };
}

describe("storage tiers", () => {
  it("names kinds at upload and keeps the old sweep while no archive node exists", async () => {
    const s = await setup();
    expect((await row(s.loser.traj))!.kind).toBe("ctraj");
    expect((await row(s.loser.lod))!.kind).toBe("lod");
    expect((await row(s.loser.log))!.kind).toBe("raw");
    await age(s.loser.log, 30);
    await runCron();
    expect((await row(s.loser.log))!.deleted_at).not.toBeNull();
    expect(await env.BUCKET.get(s.loser.log)).toBeNull();
  });

  it("holds unarchived items, tiers archived ones out after their window, and never the protected set", async () => {
    const s = await setup();
    const tok = await archiveToken();
    expect((await callJson("POST", "/v1/archive/heartbeat", { token: tok, json: { node: "innovation", total: 1e12, used: 1e9, root_kind: "dir" } })).status).toBe(200);
    for (const k of [s.loser.traj, s.loser.log, s.loser.lod, s.best.traj, s.best.log]) await age(k, 100);
    await runCron();
    for (const k of [s.loser.traj, s.loser.log]) expect((await row(k))!.deleted_at).toBeNull();
    expect(await env.BUCKET.get(s.loser.log)).not.toBeNull();

    const pending = await callJson("GET", "/v1/archive/pending?min_age_h=1", { token: tok });
    expect(pending.status).toBe(200);
    expect(pending.body.items).toHaveLength(5);
    const prot = pending.body.items.filter((i: { protected: boolean }) => i.protected).map((i: { key: string }) => i.key).sort();
    expect(prot).toEqual([s.best.log, s.best.traj].sort());
    const items = pending.body.items.map((i: { key: string; sha256: string; size: number }) => ({ key: i.key, sha256: i.sha256, archive_bytes: i.size, codec: "none" }));
    items.push({ key: s.loser.traj, sha256: "0".repeat(64) });
    const conf = await callJson("POST", "/v1/archive/confirm", { token: tok, json: { node: "innovation", items } });
    expect(conf.body).toMatchObject({ confirmed: 5, refused: [s.loser.traj] });

    await runCron();
    // past 100 days: ctraj (60) and raw (14) leave R2; the LOD (365) stays; the best run stays whole
    expect((await row(s.loser.traj))!.r2_deleted_at).not.toBeNull();
    expect((await row(s.loser.log))!.r2_deleted_at).not.toBeNull();
    expect((await row(s.loser.log))!.deleted_at).toBeNull();
    expect(await env.BUCKET.get(s.loser.traj)).toBeNull();
    expect(await env.BUCKET.get(s.loser.lod)).not.toBeNull();
    expect(await env.BUCKET.get(s.best.traj)).not.toBeNull();
    expect(await env.BUCKET.get(s.best.log)).not.toBeNull();
    const listed = await admin("GET", `/v1/artifacts?job=${s.id}`);
    expect(listed.body.artifacts.find((a: { key: string }) => a.key === s.loser.traj).tier).toBe("archived");
    expect((await admin("GET", "/v1/admin/archive")).body.mode).toBe("archive");
  });

  it("a lagging archive holds until R2 passes its soft cap, then expired raw items go", async () => {
    const s = await setup();
    const tok = await archiveToken();
    await callJson("POST", "/v1/archive/heartbeat", { token: tok, json: { node: "innovation" } });
    await env.DB.prepare("UPDATE archive_nodes SET last_seen = '2000-01-01T00:00:00.000Z'").run();
    await age(s.loser.log, 30);
    let r = await sweepArtifacts(env as unknown as Env);
    expect(r.deleted).toEqual([]);
    expect(r.held).toBeGreaterThan(0);
    r = await sweepArtifacts({ ...(env as unknown as Env), R2_SOFT_CAP_GB: "0" });
    expect(r.deleted).toEqual([s.loser.log]);
    expect((await callJson("GET", "/v1/feed/archive")).body).toMatchObject({ state: "offline" });
  });

  it("reads fall back to the archive: 503 when offline, bytes (and a re-hot copy) when online", async () => {
    const s = await setup();
    const tok = await archiveToken();
    await callJson("POST", "/v1/archive/heartbeat", { token: tok, json: { node: "innovation", total: 100, used: 25 } });
    const r = (await row(s.loser.traj))!;
    await callJson("POST", "/v1/archive/confirm", { token: tok, json: { node: "innovation", items: [{ key: s.loser.traj, sha256: r.sha256 }] } });
    await age(s.loser.traj, 100);
    await runCron();
    expect(await env.BUCKET.get(s.loser.traj)).toBeNull();

    const off = await call("GET", `/v1/artifact/${s.loser.traj}`, { token: tok });
    expect(off.status).toBe(503);
    expect(off.headers.get("retry-after")).toBe("300");
    expect(await off.json()).toMatchObject({ tier: "archive", state: "offline" });

    const seen: Request[] = [];
    vi.spyOn(globalThis, "fetch").mockImplementation(async (input: RequestInfo | URL, init?: RequestInit) => {
      const req = new Request(input, init);
      seen.push(req);
      return new Response("loser-traj", { headers: { "x-content-sha256": r.sha256 } });
    });
    const archEnv = { ...(env as unknown as Env), ARCHIVE_URL: "https://archive.test", ARCHIVE_ACCESS_ID: "id", ARCHIVE_ACCESS_SECRET: "sec" };
    const { fetchArtifact } = await import("../src/archive");
    const res = await fetchArtifact(archEnv, s.loser.traj, new Request("https://x/"));
    expect(await res.text()).toBe("loser-traj");
    expect(res.headers.get("x-storage-tier")).toBe("archive");
    expect(seen[0].url).toBe(`https://archive.test/o/${r.sha256}`);
    expect(seen[0].headers.get("cf-access-client-id")).toBe("id");
    expect(await (await env.BUCKET.get(s.loser.traj))!.text()).toBe("loser-traj");
    expect((await row(s.loser.traj))!.r2_deleted_at).toBeNull();
    expect((await callJson("GET", "/v1/feed/archive")).body).toEqual({ schema: "crane-archive-health/1", state: "ok", used_pct: 25 });
  });

  it("serves Range reads from R2", async () => {
    const s = await setup();
    const res = await call("GET", `/v1/artifact/${s.best.traj}`, { token: ADMIN, headers: { range: "bytes=2-5" } });
    expect(res.status).toBe(206);
    expect(await res.text()).toBe("st-t");
    expect(res.headers.get("content-range")).toBe("bytes 2-5/9");
  });

  it("archive tokens reach only the archive routes and artifact reads; missing objects re-pull or go", async () => {
    const s = await setup();
    const tok = await archiveToken();
    const viewer = (await admin("POST", "/v1/admin/viewer_token", { name: "v" })).body.token;
    expect((await callJson("GET", "/v1/archive/pending", { token: viewer })).status).toBe(401);
    expect((await callJson("GET", "/v1/admin/archive", { token: tok })).status).toBe(401);
    expect((await callJson("POST", "/v1/admin/jobs", { token: tok, json: { jobs: [] } })).status).toBe(401);
    expect((await call("GET", `/v1/artifact/${s.loser.log}`, { token: tok })).status).toBe(200);

    const a = (await row(s.loser.log))!;
    const b = (await row(s.loser.traj))!;
    await callJson("POST", "/v1/archive/heartbeat", { token: tok, json: { node: "innovation" } });
    await callJson("POST", "/v1/archive/confirm", { token: tok, json: { node: "innovation", items: [a, b].map((x) => ({ key: x.key, sha256: x.sha256 })) } });
    await env.DB.prepare("UPDATE artifacts SET r2_deleted_at = ?2 WHERE key = ?1").bind(b.key, new Date().toISOString()).run();
    const m = await callJson("POST", "/v1/archive/missing", {
      token: tok,
      json: { node: "innovation", items: [{ sha256: a.sha256, reason: "corrupt" }, { sha256: b.sha256, reason: "corrupt" }] },
    });
    expect(m.body).toEqual({ repull: 1, gone: 1 });
    expect((await row(a.key))!.archived_at).toBeNull();
    expect((await row(b.key))!.deleted_at).not.toBeNull();
    const p = await callJson("GET", "/v1/archive/protected", { token: tok });
    expect(p.body.sha256).toContain((await row(s.best.traj))!.sha256);
  });

  it("hot days are configurable by kind", async () => {
    const r = await admin("PUT", "/v1/admin/retention", { hot_days: { ctraj: 30 } });
    expect(r.status).toBe(200);
    expect(r.body.hot_days).toMatchObject({ ctraj: 30, lod: 365, raw: r.body.raw_days });
    expect((await admin("PUT", "/v1/admin/retention", { hot_days: { nope: 3 } })).status).toBe(400);
    expect((await admin("PUT", "/v1/admin/retention", {})).status).toBe(400);
  });
});
