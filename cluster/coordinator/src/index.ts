import { Hono } from "hono";
import { adminRoutes } from "./admin_routes";
import { archiveRoutes } from "./archive";
import { artifactRoutes, sweepArtifacts } from "./artifacts";
import type { AppEnv } from "./auth";
import { reclaimExpired } from "./db";
import { runMetaRoutes } from "./runmeta";
import { feedRoutes } from "./feed";
import { hypRoutes } from "./hypotheses";
import { Env, HttpError, errorResponse, json } from "./util";
import { webhookRoutes } from "./webhook";
import { workerRoutes } from "./worker_routes";

const app = new Hono<AppEnv>();

app.onError((err, c) => {
  if (err instanceof HttpError) return errorResponse(err);
  console.error("unhandled", err);
  return json({ error: "internal error" }, 500);
});
app.notFound(() => json({ error: "not found" }, 404));

app.route("/", archiveRoutes);
app.route("/", feedRoutes);
app.route("/", workerRoutes);
app.route("/", artifactRoutes);
app.route("/", adminRoutes);
app.route("/", hypRoutes);
app.route("/", runMetaRoutes);
app.route("/", webhookRoutes);

/** Cron sweep: reclaim expired leases (retry or dead-letter) and tier artifacts out of R2 (archive.ts). */
export async function sweep(env: Env, nowMs = Date.now()) {
  const leases = await reclaimExpired(env, nowMs);
  const artifacts = await sweepArtifacts(env, nowMs);
  return {
    ...leases,
    artifacts_deleted: artifacts.deleted.length,
    artifacts_r2_deleted: artifacts.r2_deleted.length,
    artifacts_held: artifacts.held,
  };
}

export default {
  fetch: app.fetch,
  async scheduled(_controller: ScheduledController, env: Env, ctx: ExecutionContext) {
    ctx.waitUntil(
      sweep(env).then((r) => {
        if (r.requeued.length || r.dead.length || r.artifacts_deleted || r.artifacts_r2_deleted) console.log("sweep", JSON.stringify(r));
      }),
    );
  },
} satisfies ExportedHandler<Env>;

export { app };
