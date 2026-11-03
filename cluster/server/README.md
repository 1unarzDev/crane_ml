# crane-server: the coordinator, self-hosted

The same HTTP API, auth, dashboard and behaviour as the Worker in `../coordinator`, as one Node process on a host
we run (creation), without D1's daily read quota. It is the Worker's own code: `scripts/prepare.mjs` copies
`../coordinator/{src,test,migrations,public}` and applies `hub.patch` (the in-memory Hub, reverted from the Worker
on 2026-11-02), and esbuild bundles that with small shims:

| Cloudflare | here |
|---|---|
| D1 | `src/d1.ts`: SQLite file (node:sqlite, WAL, foreign keys on), D1's prepare/bind/first/all/run/raw/batch/exec; a batch is one transaction |
| Durable Object (the Hub) | one in-process `Hub` (`src/runtime.ts`): queue, leases, heartbeats, workers, feed snapshot in memory; job transitions written through, heartbeat state flushed every `FLUSH_S` (15 s) and on SIGTERM |
| R2 | `src/bucket.ts`: `STORAGE=disk` (default, `/data/bucket`) or `STORAGE=s3` (any S3 API, R2 included); optional `R2_FALLBACK=api` reads objects not yet copied from R2 through the Cloudflare API |
| cron | `src/server.ts`: reclaim + flush every `SWEEP_S` (60 s), artifact tiering every `TIER_EVERY_H` (6 h), nightly backup at `BACKUP_HOUR_UTC` (9) |
| static assets | `src/assets.ts`: `public/` with its `_headers`; `/v1/*` goes to the API |
| SSE | served directly by the Hub's stream (`/v1/feed/stream`) |

Cold storage is unchanged: the archive node (`../archive`) pulls from this server like it pulls from the Worker,
and mirrors to Drive. `ARCHIVE_URL` is its tunnel (`https://crane-archive.lunarz.dev`, Access service token); a mesh
URL works once the archive listens on the mesh and accepts requests without an Access JWT. `R2_SOFT_CAP_GB` caps the
hot disk store (15 on creation).

Backups: the server `VACUUM INTO`s a snapshot nightly, checks it, gzips it to `/data/backups` (newest 7 kept) and
registers it as a raw artifact under `artifacts/_coordinator/`, which the archive node pulls and sends to Drive.
Only the admin token and archive tokens may read that prefix. `GET /healthz` reports the database, the store, the
Hub and the last sweep, backup and error.

## Build and test

```sh
npm ci
npm test               # the Worker's 11 vitest suites (hub.test included) + server tests, on this runtime
npm run test:http      # the same suites with every request over a real socket
npm run build          # dist/: main.mjs, migrate.mjs, migrations/, public/
python3 ../tests/e2e_local.py --server   # two real worker daemons, SIGKILL retry, hypothesis to candidate
docker build -f cluster/server/Dockerfile -t crane-server:local .   # from the repo root; runs both test modes
```

One D1-only test is skipped: it sums D1's `meta.rows_read`, which SQLite does not report.

## Run

- Compose: `compose.yaml` (port on 127.0.0.1:8787, volume `crane-data`, 768 MB cap). Settings and secrets in `.env`:
  `ADMIN_TOKEN`, `GITHUB_WEBHOOK_SECRET`, `ARCHIVE_URL`, `ARCHIVE_ACCESS_ID`, `ARCHIVE_ACCESS_SECRET`, and any
  Worker var (`LEASE_TTL_SECONDS`, `MAX_ARTIFACT_MB`, ...); defaults are `wrangler.toml`'s `[vars]`.
- systemd instead: `deploy/install-systemd.sh` (user unit `deploy/crane-server.service`, node >= 24).
- Deploy to creation from innovation: `deploy/creation.sh` (build, ship the image, write `.env` from
  `~/.config/crane-cluster/secrets.env`, start); `deploy/creation.sh --rollback` restarts the previous image.

## Migrate from Cloudflare

```sh
deploy/migrate-d1.sh            # wrangler d1 export --remote -> import on the host (migrate.mjs import, --force keeps .pre-import)
deploy/migrate-d1.sh --pull-r2  # every R2 object into the disk store (21k objects / 1 GB on 11-02: ~2 h; resumable)
```

`migrate.mjs counts` prints row counts per table to compare with D1.

## Cutover

1. Parallel run: deploy, import and pull R2 while the Worker still serves `crane.lunarz.dev`. Check the copy
   read-only: `/healthz`, `/v1/feed`, `/v1/status` against the Worker's; nothing leases from it yet (no DNS points at it).
2. Switch at a quiet moment (feed shows nothing leased, so no run report lands on the old side): run
   `deploy/migrate-d1.sh` again (final export), then `--pull-r2` again (only new objects). Remove the Worker's custom
   domain (`routes` in `../coordinator/wrangler.toml`, then `wrangler deploy`, or the dashboard), and add the public
   hostname `crane.lunarz.dev -> http://127.0.0.1:8787` on creation's tunnel (not `localhost`: it resolves to ::1 there,
   where nothing listens, and the edge answers 522). Workers ride out the minute in between:
   the client waits out an unreachable coordinator (5 s doubling to 5 min).
3. Rollback: remove the tunnel hostname and restore the Worker's custom domain; D1 and R2 still hold the state of the
   switch (anything done on the server since is not copied back). Keep D1 and R2 for at least three weeks.

Done 2026-11-03: Worker domain removed 00:07:48 UTC, final export/import 00:07:49-00:08:05, tunnel hostname and a
proxied CNAME to creation's tunnel at 00:08:07, serving from 00:09:52 (the gap was 2 min; workers rode it out).
`wrangler.toml` keeps the route commented out so a `wrangler deploy` can't take the hostname back. Until the first
R2 pull completes the server reads missing objects through the Cloudflare API (`R2_FALLBACK=api` in `.env.extra` on
creation, with a token limited to R2 read, expiring 2026-11-24); remove those three lines from `.env.extra` and
`.env` and `docker compose up -d` once `--pull-r2` reports nothing left to copy. The Worker's cron still runs on
the frozen D1 (reclaims, tiering only of objects the archive confirmed).

Rollback, in order: delete the CNAME `crane.lunarz.dev -> <tunnel>.cfargotunnel.com`, remove the `crane.lunarz.dev`
ingress rule from creation's tunnel (remotely managed: API `cfd_tunnel/<id>/configurations` or the dashboard), then
attach the custom domain again (`PUT /accounts/<id>/workers/domains {hostname, service: crane-cluster, environment:
production, zone_id}` or the dashboard). Workers back off and come back on their own. `deploy/creation.sh --rollback`
is a different thing: it restarts the server on its previous image.

## LiteLLM

`litellm/`: the proxy (one OpenAI-compatible endpoint on 127.0.0.1:4000 and on the host's WARP mesh address,
`http://100.96.0.23:4000` on creation; no public hostname: it spends provider keys, so it stays mesh-only) with Postgres (keys, per-key budgets,
usage log) and Redis (24 h response cache). Routes: `local/*` to Ollama on innovation and simwin over the mesh
(spread over whichever is up; off while the ALD Ollama workers are paused), and `openrouter/*`, `groq/*`, `gemini/*`
passed through with `OPEN_ROUTER_API_KEY`, `GROQ_API_KEY`, `GOOGLE_AI_STUDIO_KEY`. New keys default to $5 per 30
days, at most $50. Deploy: `deploy/creation.sh --litellm` (keys from the environment file, never committed); mint a
key with `curl -H "Authorization: Bearer $LITELLM_MASTER_KEY" localhost:4000/key/generate -d '{"max_budget": 2}'`.
