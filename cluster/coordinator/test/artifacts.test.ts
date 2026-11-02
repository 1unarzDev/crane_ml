import { env } from "cloudflare:test";
import { beforeEach, describe, expect, it } from "vitest";
import { ADMIN, addJobs, admin, call, callJson, lease, makeWorker, resetTables, runCron } from "./helpers";

beforeEach(resetTables);

const put = (token: string, job: string, name: string, q: string, body: string) =>
  callJson("PUT", `/v1/jobs/${job}/artifacts/${name}${q}`, { token, body: new TextEncoder().encode(body) });

describe("artifacts", () => {
  it("only the holder uploads; summary never expires; raw expires unless pinned", async () => {
    const [id] = await addJobs([{ seeds: [3] }]);
    const w = await makeWorker("w");
    const other = await makeWorker("o");
    await lease(w);
    expect((await put(other.token, id, "x.txt", "", "no")).status).toBe(409);
    expect((await put(w.token, id, "a%2F..%2Fx", "", "no")).status).toBe(400);
    const s = await put(w.token, id, "summary.json", `?retention=summary&run=${id}.a1.s3`, '{"score":1}');
    expect(s.status).toBe(200);
    expect(s.body).toMatchObject({ key: `artifacts/${id}/${id}.a1.s3/summary.json`, retention: "summary", expires_at: null });
    await admin("PUT", "/v1/admin/retention", { raw_days: 3 });
    const r = await put(w.token, id, "logs/raw.bin", "", "raw-bytes");
    expect(r.status).toBe(200);
    const days = (new Date(r.body.expires_at).getTime() - Date.now()) / 86_400_000;
    expect(days).toBeGreaterThan(2.9);
    expect(days).toBeLessThan(3.1);
    const r2 = await put(w.token, id, "pinned.bin", "", "keep");

    const dl = await call("GET", `/v1/artifact/${r.body.key}`, { token: ADMIN });
    expect(await dl.text()).toBe("raw-bytes");
    const list = await admin("GET", `/v1/artifacts?job=${id}`);
    expect(list.body.artifacts).toHaveLength(3);

    expect((await admin("POST", "/v1/admin/artifacts/pin", { key: r2.body.key })).body.updated).toBe(1);
    await env.DB.prepare("UPDATE artifacts SET expires_at = '2000-01-01T00:00:00.000Z' WHERE retention = 'raw' AND pinned = 0").run();
    await runCron();
    expect(await env.BUCKET.get(r.body.key)).toBeNull();
    expect(await env.BUCKET.get(r2.body.key)).not.toBeNull();
    expect(await env.BUCKET.get(s.body.key)).not.toBeNull();
    const after = await admin("GET", `/v1/artifacts?job=${id}`);
    expect(after.body.artifacts.map((a: { name: string }) => a.name).sort()).toEqual(["pinned.bin", "summary.json"]);
  });
});
