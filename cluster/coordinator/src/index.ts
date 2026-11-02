import { Hono } from "hono";
import { adminRoutes } from "./admin_routes";
import { archiveRoutes } from "./archive";
import { artifactRoutes, sweepArtifacts } from "./artifacts";
import type { AppEnv } from "./auth";
import { Book, bookReads, bookSink, countingDb } from "./budget";
import { hubStub } from "./hubstub";
import { runMetaRoutes } from "./runmeta";
import { feedRoutes } from "./feed";
import { replayRoutes } from "./replay";
import { theaterRoutes } from "./theater";
import { hypRoutes } from "./hypotheses";
import { inputRoutes } from "./inputs";
import { Env, HttpError, errorResponse, json } from "./util";
import { webhookRoutes } from "./webhook";
import { workerRoutes } from "./worker_routes";

const app = new Hono<AppEnv>();

// ---- D1 read budget: every query in this isolate is tallied per route and hour (src/budget.ts); the tally
// goes to the Hub at most once a minute (or past 20k rows), which keeps the day's book and the guard.
let tally: Book = {};
let lastPush = 0;
const PUSH_MS = 60_000;

export function pushReads(env: Env, ctx: { waitUntil(p: Promise<unknown>): void } | undefined, force = false) {
  const n = bookReads(tally);
  if (!Object.keys(tally).length || (!force && Date.now() - lastPush < PUSH_MS && n < 20_000)) return;
  const b = tally;
  tally = {};
  lastPush = Date.now();
  const p = hubStub(env)
    .addReads(b)
    .catch(() => {});
  if (ctx) ctx.waitUntil(p);
  return p;
}

export const countedEnv = (env: Env, label: () => string): Env => ({ ...env, DB: countingDb(env.DB, label, (l, m) => bookSink(tally)(l, m)) });

app.use("*", async (c, next) => {
  const env = c.env;
  c.env = countedEnv(env, () => `${c.req.method} ${c.req.routePath}`);
  await next();
  let ctx;
  try {
    ctx = c.executionCtx;
  } catch {}
  pushReads(env, ctx);
});

app.onError((err, c) => {
  if (err instanceof HttpError) return errorResponse(err);
  console.error("unhandled", err);
  // D1's daily row-read quota (free plan, shared by the account): say so, so clients and logs see the cause.
  if (/exceeded D1's .*limit/.test(String(err))) {
    const resetS = Math.ceil((86_400_000 - (Date.now() % 86_400_000)) / 1000);
    return new Response(JSON.stringify({ error: "D1 daily quota exhausted; resets at 00:00 UTC", retry_after_s: resetS }), {
      status: 503,
      headers: { "content-type": "application/json; charset=utf-8", "retry-after": String(resetS) },
    });
  }
  return json({ error: "internal error" }, 500);
});
app.notFound(() => json({ error: "not found" }, 404));

app.route("/", archiveRoutes);
app.route("/", feedRoutes);
app.route("/", replayRoutes);
app.route("/", theaterRoutes);
app.route("/", workerRoutes);
app.route("/", artifactRoutes);
app.route("/", inputRoutes);
app.route("/", adminRoutes);
app.route("/", hypRoutes);
app.route("/", runMetaRoutes);
app.route("/", webhookRoutes);

/** Hours between artifact tiering sweeps: each scans the artifacts table several times (D1 rows read). */
const TIER_EVERY_H = 6;

/**
 * Cron sweep: the Hub reclaims expired leases (retry or dead-letter) and writes its batched heartbeat state
 * every run; artifacts are tiered out of R2 (archive.ts) on the first run of every TIER_EVERY_H hours (or
 * always, when `tier` is true), unless the day's D1 reads passed the guard.
 */
export async function sweep(env: Env, nowMs = Date.now(), tier = true) {
  const leases = await hubStub(env).sweep(nowMs);
  const doTier = tier && !leases.degraded;
  const cenv = countedEnv(env, () => "cron");
  const artifacts = doTier ? await sweepArtifacts(cenv, nowMs) : { deleted: [], r2_deleted: [], held: 0 };
  pushReads(env, undefined, true);
  return {
    requeued: leases.requeued,
    dead: leases.dead,
    degraded: leases.degraded,
    artifacts_deleted: artifacts.deleted.length,
    artifacts_r2_deleted: artifacts.r2_deleted.length,
    artifacts_held: artifacts.held,
  };
}

export default {
  fetch: app.fetch,
  async scheduled(controller: ScheduledController, env: Env, ctx: ExecutionContext) {
    const t = controller.scheduledTime;
    const tier = t % (TIER_EVERY_H * 3_600_000) < 5 * 60_000; // the cron fires every 5 min
    ctx.waitUntil(
      sweep(env, Date.now(), tier).then((r) => {
        if (r.requeued.length || r.dead.length || r.artifacts_deleted || r.artifacts_r2_deleted) console.log("sweep", JSON.stringify(r));
      }),
    );
  },
} satisfies ExportedHandler<Env>;

export { app };
export { Hub } from "./hub";
