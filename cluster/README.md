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
for tagged jobs. Quiet hours and pause-on-activity are enforced by the worker, as is
a host's own rule: `pause_when: <shell command>` in the worker YAML runs between jobs
and on every heartbeat, and while it exits 0 the worker releases its lease (no attempt
penalty) and leases nothing; it resumes once the command exits nonzero.

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

**Run meta and trajectories**: a backend may also report a run's `run_meta(run_dir, metrics)`, sent
with the run (`meta`) and stored in `run_meta` (migration `0002`): study, iteration, params, score,
outcome, progress, validity, course and build hashes and the name of its trajectory artifact. The
`unity` and `fastsim` backends record a `run.ctraj` (crane-traj/1, `theater/traj.py`, a few hundred KB
at most), stamp it with the job's `theater` spec (study, iteration, insight, score_metric) or its
hypothesis and stage, and upload it as a `summary` artifact, so it never expires. `GET /v1/runs/query`
(viewer) selects runs by study or hypothesis, iterations, outcome and per-iteration top k; the ghost
theater's `python3 -m theater select` turns that into a render manifest. A coordinator that refuses a
meta never costs the run: the worker posts it again without.

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
prints the run command. `--memory-max 6G` caps the service and its jobs (systemd
`MemoryMax`, no swap).

Turning a worker on and off (`crane-worker` is in `~/.local/share/crane-cluster/bin`):

```sh
crane-worker off      # no jobs until `on`: the current job is released now (no attempt penalty),
                      # survives reboots (flag ~/.config/crane-cluster/off; the service is stopped and disabled)
crane-worker on       # removes the flag, enables and starts the service
crane-worker status   # off | paused (reason) | on, running job J | on, idle
```

Pause rules a host can set in its YAML: `policy.quiet_hours`, `policy.pause_on_activity`
with `policy.idle_seconds` (Linux reads keyboard/mouse activity from `/dev/input`
when the user may, e.g. the `input` group, so it works under Wayland; else
`xprintidle` or logind), and `pause_when: <shell command>` (exit 0 = pause).

Windows: `install.ps1` takes Python from `-Python`, else makes a uv venv
(`%LOCALAPPDATA%\crane-cluster\venv`, with `-Packages numpy,pyyaml` for fastsim), else uses
`python`/`py` (the Microsoft Store stubs don't count); git is needed only for jobs with `repos`.
`-Background` registers `crane-worker` as an S4U task: it runs whether or not anyone is logged in,
in session 0, so no window it or a player opens ever reaches a desktop, and a repeating trigger
restarts it after a reboot. Session 0 can't see the user's input, so `-Background` also registers
`crane-idleprobe` (at logon, hidden, in the desktop session); it writes the desktop's idle time to
`%LOCALAPPDATA%\crane-cluster\input-idle.json` and `pause_on_activity` reads it (no fresh report:
nobody is logged in, never paused).

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

### crane backends (`backends/`, outside the domain-agnostic core)

- `fastsim` (`backends.fastsim:ClusterBackend`): the fast-sim core per seed. With a spec `repo` the
  workdir is inside that checkout; on a host without git, the option `root` (the host's crane copy)
  resolves a relative `workdir`, so one job runs anywhere. A copy names its commit in a `COMMIT`
  file at its root (`<sha>[ dirty]`), which goes into every run identity.
- `unity` (`backends.unity:ClusterBackend`): a crane player per seed, without ROS or SITL: the
  course comes from `--crane-course-family` and the seed, the run is the player's benchmark, and
  the metrics are the numeric leaves of `result.json` plus `course_ready` and `player_s`
  (`course_truth.json`, `player.log` are artifacts). `batchmode: true` (default) renders on the
  GPU without a window; `false` is refused where a window would be visible. Full-stack jobs (Nav2,
  MAVROS, the ROS TCP endpoint) need the ROS workspace and stay Linux-only: give them `needs.os: linux`.
  A player is a few hundred MB, over the artifact limit, so it is copied to each host (scp) and
  named by `player:`; multipart or presigned R2 upload would let workers fetch it themselves.

### Hosts

- **innovation** (Linux): `crane-worker.service`, `~/.config/crane-cluster/worker.yaml`,
  tags `innovation, nlc`; fastsim from a crane checkout (`path`/`root`).
- **simwin** (`ssh simwin`, Windows 11 "COLLIN", user Mrmag, i7-12700 20 threads, RTX 3060 12 GB,
  16 GB): `install.ps1 -Background -Packages numpy,pyyaml`, tags `simwin, windows, nlc`, backends
  `fastsim` and `unity`. The crane copy is `%LOCALAPPDATA%\crane-cluster\crane` (`cluster/` and
  `fastsim/` plus `COMMIT`), the player `%LOCALAPPDATA%\crane-cluster\players\CRANE-Worker-Windows\CRANE.exe`
  (`CranePerformanceBuild.BuildLinuxWorker --crane-build-target windows64`, built on Linux),
  config `%APPDATA%\crane-cluster\worker.yaml`, log `%LOCALAPPDATA%\crane-cluster\worker.log`.
  Update: copy a new crane tree there, replace `lib\crane_cluster` with its `cluster\crane_cluster`,
  then `Stop-ScheduledTask crane-worker; Start-ScheduledTask crane-worker`.
  Its ALD (Ollama) LLM tasks are paused for the cluster (owner, 2026-11-02). To resume them:
  `Enable-ScheduledTask ald-worker; Enable-ScheduledTask ald-wake; Start-ScheduledTask ald-worker`
  (`ald-install-once` was left as is). To pause the cluster there instead: `Disable-ScheduledTask crane-worker`.

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

The coordinator runs at `https://crane.lunarz.dev`; `wrangler.toml` holds its database id,
bucket and custom domain, and the admin token and webhook secret live in
`~/.config/crane-cluster/secrets.env` on the deploying machine. A fresh deployment, from
`coordinator/`:

```sh
npm ci
npx wrangler login
npx wrangler d1 create crane-cluster            # copy database_id into wrangler.toml
npx wrangler r2 bucket create crane-cluster-artifacts
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
