// Proves CRANE_TEST_HTTP=1 really sends the suites' requests over the socket (see test/http-index.ts).
import { expect, it } from "vitest";
import { callJson } from "../.build/core/test/helpers";

it.runIf(process.env.CRANE_TEST_HTTP === "1")("suite requests go over HTTP in http mode", async () => {
  const before = (globalThis as { __craneHttpCalls?: number }).__craneHttpCalls ?? 0;
  expect((await callJson("GET", "/v1/ping")).status).toBe(200);
  expect((globalThis as { __craneHttpCalls?: number }).__craneHttpCalls).toBe(before + 1);
});
