import { Hono } from "hono";
import { adminRoutes } from "./admin_routes";
import { archiveRoutes } from "./archive";
import { artifactRoutes, sweepArtifacts } from "./artifacts";
import type { AppEnv } from "./auth";
import { reclaimExpired } from "./db";
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
 * Cron sweep: reclaim expired leases (retry or dead-letter) every run, and tier artifacts out of R2
 * (archive.ts) on the first run of every TIER_EVERY_H hours (or always, when `tier` is true).
 */
export async function sweep(env: Env, nowMs = Date.now(), tier = true) {
  const leases = await reclaimExpired(env, nowMs);
  const artifacts = tier ? await sweepArtifacts(env, nowMs) : { deleted: [], r2_deleted: [], held: 0 };
  return {
    ...leases,
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
