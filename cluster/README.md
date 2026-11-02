# crane cluster

A small, domain-agnostic job cluster for simulation experiments. One coordinator
(a Cloudflare Worker with D1 and R2) hands out jobs. Worker daemons run on whatever
machines are available: Linux GPU and CPU boxes, Windows and macOS desktops, shared
lab machines. Hypotheses move through staged promotion, decided by paired tests
against a baseline on identical seeds.

Nothing here knows about any particular robot, simulator or domain. Domain code
plugs in as a **backend** (`type: "package.module:Class"` in the worker YAML) or as a
container image driven by the `command` backend.

## Architecture

```
                         GitHub (push to main)
                                 │ HMAC-signed webhook
                                 ▼
 crane-hyp CLI ──admin──▶ ┌──────────────────────────────────────────┐ ◀──viewer── status readers
 (submit/status/report)   │ coordinator (Cloudflare Worker, Hono)    │
   │                      │  /v1/hyp      hypotheses, stages         │
   │ promotion.py         │  /v1/admin/*  invites, jobs, repos, gate │
   │ (paired Wilcoxon,    │  /v1/lease    capability matching        │
   │  Holm, halving)      │  /v1/heartbeat, /release, /jobs/:id/*    │
   │                      │  cron */5: reclaim leases, sweep R2      │
   └── decision ─────────▶│                                          │
                          │  D1: workers invites repos hypotheses    │
                          │      jobs leases runs metrics artifacts  │
                          │  R2: artifacts/<job>/<run>/<name>        │
                          └───────────────▲──────────────────────────┘
                                          │ HTTPS, per-worker bearer token
               ┌──────────────────────────┼───────────────────────────┐
               ▼                          ▼                           ▼
     crane-worker (Linux GPU)   crane-worker (Windows)      crane-worker (lab box)
     systemd --user, nice 10    scheduled task, pythonw     quiet hours, pauses
      │                                                     on user activity
      ├─ between jobs: git mirror refresh (main moved), image pull
      └─ per job: git checkout of exact refs ─▶ Backend.prepare(job)
                  for seed: Backend.run(seed) ─▶ run_dir ─▶ Backend.metrics(run_dir)
                            POST runs+metrics, PUT artifacts, heartbeat throughout
```

## Pieces

| Path | What |
|---|---|
| `coordinator/` | The Worker (TypeScript). `migrations/0001_init.sql` is the schema; `src/match.ts` the scheduler matching; `src/hypotheses.ts` staged promotion; `src/artifacts.ts` R2 and retention; `src/webhook.ts` GitHub. |
| `crane_cluster/worker.py` | `crane-worker join|run|caps`: the daemon. |
| `crane_cluster/backends/` | `Backend` interface and registry; `dummy` and `command` backends. |
| `crane_cluster/promotion.py` | Paired Wilcoxon signed-rank test (exact for n ≤ 30), Holm correction, successive halving, and the promote/extend/reject decision. |
| `crane_cluster/hyp.py` | `crane-hyp submit|status|report`. |
| `crane_cluster/idle.py` | Nice/ionice, quiet hours, user-idle detection (X11, logind, Windows, macOS). |
| `crane_cluster/gitcache.py` | One bare mirror per repo; detached checkouts at exact commits. |
| `install/` | `install.sh` (systemd user unit) and `install.ps1` (scheduled task). |
| `examples/` | `worker.yaml`, `hypothesis.yaml`. |
| `tests/` | Python unit tests and `e2e_local.py`. |

The Python side is stdlib-only (Python ≥ 3.9). It uses PyYAML when that is installed;
otherwise it parses the YAML subset the config files use.

## Concepts

**Job**: one backend invocation over a list of seeds at exact repo refs:
`{backend, repos: [{name, url, ref}], spec, seeds, needs, priority, max_attempts}`.
A ref of `@main` resolves at creation to the repo's `main_version`, which the
webhook sets. Every job therefore records the commit it ran.

**Needs and capabilities**: a job's `needs` (`os`, `arch`, `gpu`, `vulkan`,
`min_cpu`, `min_ram_gb`, `min_vram_gb`, `tags`, `est_minutes`) are matched against
what each worker reports (detected, then overridden by its YAML `caps`) and the
backends it offers. The worker's availability `policy` adds two coordinator-side
rules: `max_est_minutes` refuses long jobs, and `require_tags` reserves a machine
for tagged jobs. Quiet hours and pause-on-activity are enforced by the worker.

**Leases**: a lease lasts `LEASE_TTL_SECONDS`, and each heartbeat extends it. When a
worker vanishes, its lease expires and the job is requeued. A job that has used
`max_attempts` (expired or failed) goes to `dead`, the dead-letter state, until an
admin calls `/v1/admin/jobs/requeue`. A voluntary release (user became active,
shutdown) does not count as an attempt. Cancelled or reclaimed jobs come back as
`revoked` on the holder's next heartbeat, and the worker stops them.

**Runs and metrics**: one run per seed per attempt (`<job>.a<attempt>.s<seed>`),
with a flat map of numeric metrics. Analysis reads only each job's final attempt.

**Artifacts**: workers upload files through the Worker into R2. A `summary` artifact
(metrics, summary, provenance) is kept forever. A `raw` artifact (logs, recordings,
bags) expires after `raw_retention_days` (`PUT /v1/admin/retention`, default
`RAW_RETENTION_DAYS` = 14) unless it is pinned (`POST /v1/admin/artifacts/pin`). The
cron sweep deletes expired raw objects and keeps their rows with `deleted_at`.
Uploads are proxied, so a single file must fit the Workers request-body limit
(`MAX_ARTIFACT_MB`, default 95).

**Hypothesis**: a claim, a metric, a direction, a baseline ref, variants (refs plus
optional params), a scenario family, seeds, a run budget, and a list of stages,
each with a backend, params and needs. Example: `examples/hypothesis.yaml`. Each
stage runs the baseline and every active variant on the same seeds.
`crane-hyp report ID --apply` then:

1. waits until the stage has no queued or leased jobs;
2. with several variants, applies successive halving: it keeps the best
   `ceil(k/eta)` by mean paired difference and adds seeds for them (`extend`);
3. tests each remaining variant with a one-sided paired Wilcoxon test against the
   baseline, Holm-corrected. A variant with at least `min_pairs` pairs,
   adjusted p < `alpha` and median gain ≥ `min_effect` is promoted to the next
   stage. Promotion past the last stage makes the hypothesis a `candidate`;
4. rejects when every variant is significantly worse, or when the result is still
   inconclusive at `max_seeds` or the budget. Otherwise it extends with more seeds.

Every decision is stored with its evidence (`decisions` on `/v1/hyp/:id`).

## Worker

```sh
install/install.sh --coordinator https://COORDINATOR --invite CODE          # Linux/macOS
powershell -ExecutionPolicy Bypass -File install\install.ps1 -Coordinator https://COORDINATOR -Invite CODE
```

The installer copies `crane_cluster` (no pip), writes `~/.config/crane-cluster/worker.yaml`
if missing, joins with the invite (the token goes to `~/.config/crane-cluster/token`,
mode 600) and starts the service. Configure backends, caps and policy in the YAML
(see `examples/worker.yaml`), then restart the service. Re-run the installer
without `--invite` to update the code. A host that must never run jobs opts out
with an empty `/etc/crane-cluster/deny`. On macOS without systemd, the installer
prints the run command.

Isolation: each job gets `CRANE_PORT_BASE` and `CRANE_ISOLATION_ID` (template fields
`{port_base}`, `{isolation_id}`), so a backend can derive non-overlapping ports or
middleware domain ids.

### Writing a backend

```python
from crane_cluster.backends import Backend

class MyBackend(Backend):
    def prepare(self, ctx):          # ctx.job, ctx.spec, ctx.repos[name] -> checkout, ctx.work_dir
        super().prepare(ctx)
    def run(self, seed):             # -> run dir containing metrics.json (and artifacts/ for raw files)
        d = self.run_dir(seed); ...; return d
    def metrics(self, run_dir):      # default reads run_dir/metrics.json
        ...
    def cancel(self): ...            # stop promptly when the lease is revoked
```

Register it in the worker YAML as `mine: {type: "my_pkg.module:MyBackend", ...options}`.

## Testing

```sh
cd coordinator && npm ci && npx vitest run && npx tsc --noEmit   # Worker tests, local D1/R2 (miniflare)
cd .. && python3 -m unittest discover -s tests -t .              # promotion, YAML, backends, git cache
python3 tests/e2e_local.py                                       # wrangler dev + two workers (about 1 min)
```

The end-to-end test runs `wrangler dev` with local state in a temp dir (no login,
nothing deployed). It installs two workers with `install.sh --no-service` and kills
one mid-job with SIGKILL. It then checks that the expired lease is retried and
finished by the other worker, and walks a hypothesis on a local git repo through
halving, stage 1 and stage 2 to `candidate`.

## Deploying

Nothing is deployed by default, and `wrangler.toml` carries a placeholder database id.
From `coordinator/`:

```sh
npm ci
npx wrangler login
npx wrangler d1 create crane-cluster            # copy database_id into wrangler.toml
npx wrangler r2 bucket create crane-cluster
npx wrangler d1 migrations apply crane-cluster --remote
openssl rand -base64 32 | npx wrangler secret put ADMIN_TOKEN
openssl rand -hex 32    | tee /dev/stderr | npx wrangler secret put GITHUB_WEBHOOK_SECRET
# custom domain (zone must be on the same Cloudflare account): add to wrangler.toml
#   routes = [{ pattern = "cluster.example.org", custom_domain = true }]
npx wrangler deploy
curl https://cluster.example.org/v1/ping
```

GitHub, for each repo whose `main` workers should track: Settings → Webhooks →
payload URL `https://cluster.example.org/v1/webhook/github`, content type
`application/json`, the webhook secret, push events only.

Admin calls use `Authorization: Bearer $ADMIN_TOKEN`: `POST /v1/admin/invite {"n": 3}`,
`POST /v1/admin/viewer_token`, `PUT /v1/admin/repos {"name","url"}`,
`PUT /v1/admin/client_release {"version","min_version"}` (workers below
`min_version` get HTTP 426), `PUT /v1/admin/retention {"raw_days"}`.
