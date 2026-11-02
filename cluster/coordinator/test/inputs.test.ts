import { beforeEach, describe, expect, it } from "vitest";
import { ADMIN, admin, call, callJson, makeWorker, resetTables } from "./helpers";

beforeEach(resetTables);

async function sha(text: string): Promise<string> {
  const d = await crypto.subtle.digest("SHA-256", new TextEncoder().encode(text));
  return [...new Uint8Array(d)].map((b) => b.toString(16).padStart(2, "0")).join("");
}

describe("inputs", () => {
  it("admin uploads by sha256; workers and viewers read; the hash is checked", async () => {
    const body = '{"schema":"crane-theater/1"}';
    const h = await sha(body);
    const w = await makeWorker("w");
    expect((await callJson("PUT", `/v1/admin/inputs/${h}`, { token: w.token, body: new TextEncoder().encode(body) })).status).toBe(401);
    expect((await callJson("PUT", `/v1/admin/inputs/${"0".repeat(64)}`, { token: ADMIN, body: new TextEncoder().encode(body) })).status).toBe(400);
    const up = await callJson("PUT", `/v1/admin/inputs/${h}?name=manifest.json`, { token: ADMIN, body: new TextEncoder().encode(body) });
    expect(up.status).toBe(200);
    expect(up.body).toMatchObject({ key: `inputs/${h}`, sha256: h, size: body.length });
    const got = await call("GET", `/v1/inputs/${h}`, { token: w.token });
    expect(got.status).toBe(200);
    expect(got.headers.get("x-content-sha256")).toBe(h);
    expect(await got.text()).toBe(body);
    const viewer = (await admin("POST", "/v1/admin/viewer_token", {})).body.token;
    expect((await call("GET", `/v1/inputs/${h}`, { token: viewer })).status).toBe(200);
    expect((await call("GET", `/v1/inputs/${h}`, { token: "nope" })).status).toBe(401);
    expect((await call("GET", `/v1/inputs/${h}`)).status).toBe(401);
    expect((await call("GET", `/v1/inputs/${"a".repeat(64)}`, { token: w.token })).status).toBe(404);
  });
});
