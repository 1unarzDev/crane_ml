// The self-hosted pieces around the Worker's code: the D1 shim, the disk and S3 stores, the HTTP front (assets,
// health, backup guard), backups, the D1 import and a restart that keeps the Hub's batched state.
import { createHash } from "node:crypto";
import { mkdtempSync, readFileSync } from "node:fs";
import { createServer } from "node:http";
import type { AddressInfo } from "node:net";
import os from "node:os";
import path from "node:path";
import { DatabaseSync } from "node:sqlite";
import { gunzipSync } from "node:zlib";
import { describe, expect, it } from "vitest";
import { addJobs, admin, lease, makeWorker, resetTables } from "../.build/core/test/helpers";
import { Assets } from "../src/assets";
import { backup } from "../src/backup";
import { DiskBucket, S3Bucket } from "../src/bucket";
import { SqliteD1 } from "../src/d1";
import { counts, importDump } from "../src/migrate";
import { createRuntime } from "../src/runtime";
import { createHandler, newStatus, startScheduler } from "../src/server";
import { runtime, tmpDir } from "./cf-test";

const here = path.dirname(new URL(import.meta.url).pathname);
const server = path.resolve(here, "..");
const migrations = path.join(server, ".build/core/migrations");
const tmp = () => mkdtempSync(path.join(os.tmpdir(), "crane-srv-"));
const handler = () => createHandler(runtime, new Assets(path.join(server, ".build/core/public")), newStatus(), new Set());
const get = (p: string, headers: Record<string, string> = {}) => handler()(new Request(`http://x${p}`, { headers }));

describe("D1 on SQLite", () => {
  const d1 = new SqliteD1(":memory:");
  d1.db.exec("CREATE TABLE t (id INTEGER PRIMARY KEY, v TEXT NOT NULL, b INTEGER); CREATE TABLE log (n INTEGER);");
  d1.db.exec("CREATE TRIGGER t_log AFTER UPDATE ON t BEGIN INSERT INTO log VALUES (NEW.id); END;");

  it("binds numbered params and booleans, returns RETURNING rows and counts changes without trigger rows", async () => {
    await d1.batch([d1.prepare("INSERT INTO t(id, v, b) VALUES (?1, ?2, ?3)").bind(1, "a", true), d1.prepare("INSERT INTO t(id, v, b) VALUES (?1, ?2, ?3)").bind(2, "b", false)]);
    const r = await d1.prepare("UPDATE t SET v = v || ?2 WHERE id <= ?1 RETURNING id").bind(2, "!").run();
    expect(r.results.map((x) => x.id)).toEqual([1, 2]);
    expect(r.meta.changes).toBe(2);
    expect((await d1.prepare("SELECT v FROM t WHERE id = ?").bind(1).first("v"))).toBe("a!");
    expect(await d1.prepare("SELECT b FROM t ORDER BY id").raw({ columnNames: true })).toEqual([["b"], [1], [0]]);
    expect((await d1.prepare("SELECT * FROM t WHERE id = 9").first())).toBeNull();
    expect((await d1.prepare("SELECT COUNT(*) AS n FROM t").run()).meta.changes).toBe(0);
  });

  it("rolls a failed batch back whole", async () => {
    await expect(d1.batch([d1.prepare("INSERT INTO t(id, v) VALUES (3, 'c')"), d1.prepare("INSERT INTO t(id, v) VALUES (4, NULL)")])).rejects.toThrow(/NOT NULL/);
    expect(await d1.prepare("SELECT COUNT(*) AS n FROM t WHERE id = 3").first("n")).toBe(0);
  });
});

describe("stores", () => {
  it("disk: metadata, Range by header, offset/length and suffix, delete", async () => {
    const b = new DiskBucket(tmp());
    await b.put("a/b/c.bin", "0123456789", { customMetadata: { sha256: "x" }, httpMetadata: { contentType: "text/plain" } });
    const whole = await b.get("a/b/c.bin");
    expect(await whole!.text()).toBe("0123456789");
    expect(whole!.customMetadata.sha256).toBe("x");
    expect(whole!.httpMetadata.contentType).toBe("text/plain");
    const r = await b.get("a/b/c.bin", { range: new Headers({ range: "bytes=2-4" }) });
    expect([await r!.text(), r!.range]).toEqual(["234", { offset: 2, length: 3 }]);
    expect(await (await b.get("a/b/c.bin", { range: { offset: 8 } }))!.text()).toBe("89");
    expect(await (await b.get("a/b/c.bin", { range: { suffix: 3 } }))!.text()).toBe("789");
    expect(await (await b.get("a/b/c.bin", { range: new Headers({ range: "bytes=-2" }) }))!.text()).toBe("89");
    await b.delete(["a/b/c.bin"]);
    expect(await b.get("a/b/c.bin")).toBeNull();
  });

  it("s3: signed requests, metadata headers and ranges", async () => {
    const objs = new Map<string, { body: Buffer; headers: Record<string, string> }>();
    const seen: string[] = [];
    const srv = createServer((req, res) => {
      seen.push(`${req.method} ${req.url} ${String(req.headers.authorization ?? "").slice(0, 16)}`);
      const chunks: Buffer[] = [];
      req.on("data", (c) => chunks.push(c));
      req.on("end", () => {
        const o = objs.get(req.url!);
        if (req.method === "PUT") {
          const h: Record<string, string> = {};
          for (const [k, v] of Object.entries(req.headers)) if (k.startsWith("x-amz-meta-") || k === "content-type") h[k] = String(v);
          objs.set(req.url!, { body: Buffer.concat(chunks), headers: h });
          res.writeHead(200, { etag: '"e1"' }).end();
        } else if (req.method === "DELETE") {
          objs.delete(req.url!);
          res.writeHead(204).end();
        } else if (!o) res.writeHead(404).end();
        else {
          const m = /bytes=(\d+)-(\d+)/.exec(String(req.headers.range ?? ""));
          const hdr = { ...o.headers, etag: '"e1"', "content-length": String(o.body.length) };
          if (req.method === "HEAD") res.writeHead(200, hdr).end();
          else if (m) {
            const [s, e] = [Number(m[1]), Number(m[2])];
            res.writeHead(206, { ...hdr, "content-length": String(e - s + 1), "content-range": `bytes ${s}-${e}/${o.body.length}` }).end(o.body.subarray(s, e + 1));
          } else res.writeHead(200, hdr).end(o.body);
        }
      });
    });
    await new Promise<void>((r) => srv.listen(0, "127.0.0.1", r));
    const b = new S3Bucket({ endpoint: `http://127.0.0.1:${(srv.address() as AddressInfo).port}`, bucket: "bk", accessKeyId: "AK", secretAccessKey: "SK" });
    await b.put("artifacts/j/run 1.json", "hello world", { customMetadata: { sha256: "abc" } });
    const o = await b.get("artifacts/j/run 1.json");
    expect([await o!.text(), o!.customMetadata.sha256, o!.size]).toEqual(["hello world", "abc", 11]);
    const r = await b.get("artifacts/j/run 1.json", { range: new Headers({ range: "bytes=6-" }) });
    expect([await r!.text(), r!.range, r!.size]).toEqual(["world", { offset: 6, length: 5 }, 11]);
    await b.delete("artifacts/j/run 1.json");
    expect(await b.get("artifacts/j/run 1.json")).toBeNull();
    expect(seen[0]).toMatch(/^PUT \/bk\/artifacts\/j\/run%201\.json AWS4-HMAC-SHA256/);
    srv.close();
  });
});

describe("HTTP front", () => {
  it("serves the dashboard with its _headers, revalidates by etag, and leaves /v1 to the API", async () => {
    const r = await get("/");
    expect(r.status).toBe(200);
    expect(r.headers.get("content-type")).toMatch(/text\/html/);
    expect(r.headers.get("content-security-policy")).toMatch(/default-src 'self'/);
    const js = await get("/app.js");
    expect(js.headers.get("cache-control")).toBe("public, max-age=0, must-revalidate");
    expect((await get("/app.js", { "if-none-match": js.headers.get("etag")! })).status).toBe(304);
    expect((await get("/v1/ping")).status).toBe(200);
    const miss = await get("/nope.txt");
    expect([miss.status, await miss.json()]).toEqual([404, { error: "not found" }]);
  });

  it("answers /healthz with the db, the store and the Hub", async () => {
    const r = await get("/healthz");
    const b = await r.json();
    expect(r.status).toBe(200);
    expect(b).toMatchObject({ ok: true, db: "ok", storage: "disk", hub: { queued: expect.any(Number) } });
  });
});

describe("backups", () => {
  it("snapshot, gzip, keep N, register for the archive; only admin and archive tokens read them", async () => {
    await resetTables();
    await addJobs([{ seeds: [1] }]);
    const dir = path.join(tmpDir, "backups");
    const b1 = await backup(runtime, dir, 1, true, Date.UTC(2026, 10, 1, 9));
    const b2 = await backup(runtime, dir, 1, true, Date.UTC(2026, 10, 2, 9));
    const files = (await import("node:fs")).readdirSync(dir);
    expect(files).toEqual([path.basename(b2.file)]);
    const raw = gunzipSync(readFileSync(b2.file));
    const restored = path.join(tmpDir, "restored.sqlite");
    (await import("node:fs")).writeFileSync(restored, raw);
    const db = new DatabaseSync(restored, { readOnly: true });
    expect((db.prepare("SELECT COUNT(*) AS n FROM jobs").get() as { n: number }).n).toBe(1);
    db.close();
    expect(b2.sha256).toBe(createHash("sha256").update(readFileSync(b2.file)).digest("hex"));
    const row = await runtime.d1.prepare("SELECT retention, kind, sha256 FROM artifacts WHERE key = ?").bind(b2.key).first();
    expect(row).toEqual({ retention: "raw", kind: "raw", sha256: b2.sha256 });
    expect(b1.key).not.toBe(b2.key);

    const viewer = (await admin("POST", "/v1/admin/viewer_token", { name: "v" })).body.token;
    const arch = (await admin("POST", "/v1/admin/archive_token", { name: "a" })).body.token;
    const path2 = `/v1/artifact/${b2.key}`;
    expect((await get(path2, { authorization: `Bearer ${viewer}` })).status).toBe(404);
    expect((await get(path2)).status).toBe(404);
    const viaArchive = await get(path2, { authorization: `Bearer ${arch}` });
    expect(viaArchive.status).toBe(200);
    expect(Buffer.from(await viaArchive.arrayBuffer()).equals(readFileSync(b2.file))).toBe(true);
    expect((await get(path2, { authorization: "Bearer test-admin-token" })).status).toBe(200);
  });

  it("the scheduler sweeps and takes the nightly backup at its hour", async () => {
    const status = newStatus();
    const hour = new Date().getUTCHours();
    const s = startScheduler(runtime, status, { sweepS: 3600, flushS: 3600, tierEveryH: 6, backupHourUtc: hour, backupDir: path.join(tmpDir, "sched"), backupKeep: 2, log: () => {} });
    await s.tick();
    s.stop();
    expect(status.lastSweep?.result).toMatchObject({ requeued: [], dead: [] });
    expect(status.lastBackup?.bytes).toBeGreaterThan(0);
    expect(status.lastError).toBeNull();
  });
});

describe("migration from D1", () => {
  it("imports a wrangler d1 export, applies the server's migrations, and the queue works", async () => {
    const dir = tmp();
    const db = path.join(dir, "coordinator.sqlite");
    const r = importDump(path.join(here, "fixtures/d1-export.sql"), db, migrations);
    expect(r.applied).toEqual(["0006_run_meta_job.sql"]);
    expect(r.counts).toMatchObject({ jobs: 2, runs: 1, metrics: 1, workers: 1, artifacts: 1, config: 1 });
    expect(() => importDump(path.join(here, "fixtures/d1-export.sql"), db, migrations)).toThrow(/exists/);
    const rt = createRuntime({ dbPath: db, migrationsDir: migrations, bucket: new DiskBucket(path.join(dir, "b")), vars: { ADMIN_TOKEN: "t", LEASE_TTL_SECONDS: "300" } });
    expect(rt.applied).toEqual([]);
    const job = await rt.d1.prepare("SELECT purpose FROM jobs WHERE job_id = 'j_d'").first("purpose");
    expect(job).toMatch(/it's quoted/);
    expect(rt.hub.state()).toMatchObject({ queued: 0 });
    await rt.hub.reload();
    expect(rt.hub.state()).toMatchObject({ queued: 1, workers: 1 });
    expect(counts(rt.d1).d1_migrations).toBe(6);
    await rt.close();
  });
});

describe("restart", () => {
  it("close() writes the Hub's batched heartbeat state; a new process gives live leases a grace TTL", async () => {
    await resetTables();
    const [id] = await addJobs([{}]);
    const w = await makeWorker("w");
    expect((await lease(w)).body.jobs[0].job_id).toBe(id);
    const dir = tmp();
    const db = path.join(dir, "c.sqlite");
    const rt = createRuntime({ dbPath: db, migrationsDir: migrations, bucket: new DiskBucket(path.join(dir, "b")), vars: { ADMIN_TOKEN: "a", LEASE_TTL_SECONDS: "300" } });
    const call = (r: typeof rt, method: string, p: string, json?: unknown, token = "a") =>
      r.worker.fetch(new Request(`http://x${p}`, { method, headers: { authorization: `Bearer ${token}`, "content-type": "application/json" }, body: json ? JSON.stringify(json) : undefined }) as never, r.env as never, { waitUntil() {}, passThroughOnException() {} } as never);
    const jobs = { jobs: [{ backend: "dummy", purpose: "Checks that a restart keeps leases and the batched heartbeat state.", est_seconds: 30, submitter: "vitest" }] };
    const jid = (await (await call(rt, "POST", "/v1/admin/jobs", jobs)).json()).job_ids[0];
    const inv = (await (await call(rt, "POST", "/v1/admin/invite", { n: 1 })).json()).invites[0];
    const tok = (await (await call(rt, "POST", "/v1/join", { invite: inv, name: "w2", caps: { backends: ["dummy"] }, client_version: "0.1.0" })).json()).token;
    expect((await (await call(rt, "POST", "/v1/lease", { caps: { backends: ["dummy"] } }, tok)).json()).jobs[0].job_id).toBe(jid);
    await call(rt, "POST", "/v1/heartbeat", { job_ids: [jid], state: "running" }, tok);
    await rt.close();
    const seen = new DatabaseSync(db, { readOnly: true }).prepare("SELECT state, last_seen FROM workers WHERE name = 'w2'").get() as { state: string; last_seen: string };
    expect(seen.state).toBe("running");
    const rt2 = createRuntime({ dbPath: db, migrationsDir: migrations, bucket: new DiskBucket(path.join(dir, "b")), vars: { ADMIN_TOKEN: "a", LEASE_TTL_SECONDS: "300" } });
    await rt2.sweep(Date.now(), false);
    const hb = await (await call(rt2, "POST", "/v1/heartbeat", { job_ids: [jid] }, tok)).json();
    expect(hb.extended).toEqual([jid]);
    await rt2.close();
  });
});
