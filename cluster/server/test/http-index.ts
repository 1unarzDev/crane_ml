// CRANE_TEST_HTTP=1: the suites' worker.fetch goes over a real socket to the server's HTTP front (node:http +
// @hono/node-server, assets, health, backup guard), in this process so the tests still reach env.DB and the Hub.
// Requests use node:http, not fetch, because some tests stub globalThis.fetch.
import { request as httpRequest, createServer } from "node:http";
import type { AddressInfo } from "node:net";
import path from "node:path";
import { getRequestListener } from "@hono/node-server";
import real from "../.build/core/src/index";
import { Assets } from "../src/assets";
import { createHandler, newStatus } from "../src/server";
import { runtime } from "./cf-test";

export * from "../.build/core/src/index";

const server = path.resolve(path.dirname(new URL(import.meta.url).pathname), "..");
let base: Promise<string> | null = null;

function start(): Promise<string> {
  const handler = createHandler(runtime, new Assets(path.join(server, ".build/core/public")), newStatus(), new Set());
  const srv = createServer(getRequestListener(handler, { overrideGlobalObjects: false }));
  return new Promise((resolve) => srv.listen(0, "127.0.0.1", () => resolve(`http://127.0.0.1:${(srv.address() as AddressInfo).port}`)));
}

async function httpFetch(req: Request): Promise<Response> {
  (globalThis as { __craneHttpCalls?: number }).__craneHttpCalls = ((globalThis as { __craneHttpCalls?: number }).__craneHttpCalls ?? 0) + 1;
  base ??= start();
  const u = new URL(req.url);
  const target = new URL(u.pathname + u.search, await base);
  const body = req.body ? Buffer.from(await req.arrayBuffer()) : undefined;
  const headers: Record<string, string> = {};
  req.headers.forEach((v, k) => (headers[k] = v));
  if (body) headers["content-length"] = String(body.byteLength);
  return new Promise((resolve, reject) => {
    const r = httpRequest(target, { method: req.method, headers }, (res) => {
      const chunks: Buffer[] = [];
      res.on("data", (c) => chunks.push(c));
      res.on("end", () => {
        const h = new Headers();
        for (const [k, v] of Object.entries(res.headers)) if (v !== undefined) h.set(k, Array.isArray(v) ? v.join(", ") : v);
        const s = res.statusCode ?? 500;
        resolve(new Response([101, 204, 205, 304].includes(s) ? null : Buffer.concat(chunks), { status: s, headers: h }));
      });
      res.on("error", reject);
    });
    r.on("error", reject);
    r.end(body);
  });
}

export default { ...real, fetch: (req: Request) => httpFetch(req) };
