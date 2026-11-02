import { beforeEach, describe, expect, it } from "vitest";
import { hmacSha256Hex } from "../src/util";
import { JOB_META, addJobs, admin, callJson, resetTables } from "./helpers";

beforeEach(resetTables);

const SHA = "a".repeat(40);

async function hook(payload: unknown, event = "push", secret = "test-hook-secret") {
  const body = new TextEncoder().encode(JSON.stringify(payload));
  const sig = `sha256=${await hmacSha256Hex(secret, body.buffer as ArrayBuffer)}`;
  return callJson("POST", "/v1/webhook/github", { body, headers: { "x-github-event": event, "x-hub-signature-256": sig } });
}

const push = (ref: string, after = SHA) => ({
  ref,
  after,
  repository: { full_name: "org/sim", clone_url: "https://example.invalid/org/sim.git", default_branch: "main" },
});

describe("github webhook", () => {
  it("rejects a bad signature", async () => {
    expect((await hook(push("refs/heads/main"), "push", "wrong")).status).toBe(401);
  });

  it("a push to main sets main_version; other branches are ignored", async () => {
    expect((await hook({}, "ping")).body.event).toBe("ping");
    expect((await hook(push("refs/heads/feature", "b".repeat(40)))).body.ignored).toBe("refs/heads/feature");
    const r = await hook(push("refs/heads/main"));
    expect(r.body).toEqual({ ok: true, repo: "org/sim", main_version: SHA });
    // "@main" in a job now resolves to that exact commit
    const [id] = await addJobs([{ repos: [{ name: "org/sim", ref: "@main" }] }]);
    const jobs = await admin("GET", "/v1/jobs");
    const j = jobs.body.jobs.find((x: { job_id: string }) => x.job_id === id);
    expect(j.repos).toEqual([{ name: "org/sim", url: "https://example.invalid/org/sim.git", ref: SHA }]);
  });

  it("an unknown repo's @main is refused", async () => {
    const r = await admin("POST", "/v1/admin/jobs", { jobs: [{ backend: "dummy", ...JOB_META, repos: [{ name: "org/x", url: "u", ref: "@main" }] }] });
    expect(r.status).toBe(400);
  });
});
