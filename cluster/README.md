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
| `crane_cluster/gitcache.py` | One bare mirror per repo; detached (optionally sparse) checkouts at exact commits. |
| `crane_cluster/provision.py` | What a joining host gets (D-20261102-17): tools, repos, backends, `tools.json`. |
| `install/` | `install.sh` (systemd user unit), `install.ps1` (scheduled task), `provision.yaml` (the plan). |
| `archive/` | `crane-archive`, the cold storage node: a content-addressed, deduplicated store with scrubs and a quota, the pull loop, and the tunnel origin; storage tiers and hardware in `archive/README.md`. |
| `examples/` | `worker.yaml`, `hypothesis.yaml`. |
| `tests/` | Python unit tests and `e2e_local.py`. |

The Python side is stdlib-only (Python ≥ 3.9). It uses PyYAML when that is installed;
otherwise it parses the YAML subset the config files use.

## Concepts

**Job**: one backend invocation over a list of seeds at exact repo refs:
`{backend, repos: [{name, url, ref}], spec, seeds, needs, priority, max_attempts,
purpose, est_seconds, submitter, campaign}`.
A ref of `@main` resolves at creation to the repo's `main_version`, which the
webhook sets. Every job therefore records the commit it ran.

**Purpose, estimate, submitter**: every submission (`POST /v1/admin/jobs`, `POST /v1/hyp`)
must carry `purpose`, 1-2 sentences written by the submitting agent on what hypothesis the
job tests or what it is for (20-600 chars; placeholders like "test" or "sweep" are refused,
and so is text that looks like a secret, an address or a local path, because the feed is
public), `est_seconds`, the estimated seconds per run (per seed), and `submitter`, the
agent or session label. `campaign` (a boat test, a study) is optional. A hypothesis's
stage jobs inherit them; a stage's own `est_seconds` overrides the hypothesis's. Rows
from before migration `0003` read `(legacy)`. `POST /v1/admin/jobs/judge {job_id, line}`
attaches a one-line verdict.

**Run feed** (public, no token): `GET /v1/feed[?recent=N]` (schema `crane-feed/1`),
`GET /v1/feed/jobs/:id` (one job and its runs) and `GET /v1/feed/stream` (SSE: an
`event: feed` with the snapshot at connect and on change, `?interval=`, `?max_s=`,
`?once=1`). Each job shows its state, purpose, submitter, campaign, hypothesis and
stage, the submitter's estimate next to the duration model's (per backend: the median
of `actual / estimate` over the last 2000 ok runs once it has 5, else the median run
time), its queue position and ETA (a list-scheduling pass over the queue in lease
order onto the live workers that can run it, each scaled by its own speed history),
progress, worker alias, metric means, the judge line (or the latest run_meta insight)
and replay availability (runs with a kept `.ctraj`). Nothing private leaves: no
tokens, worker ids, specs, needs, repo URLs or paths; free text goes through a
secret scrubber (`src/purpose.ts`). Snapshots are cached per colo for `FEED_CACHE_S`.

**Dashboard** (`coordinator/public/`, Workers static assets at `/`, no build step): the
public, read-only view of the feed. Now (one sentence, a berth per live worker, the
queue with start times and a log-scale estimate glyph, recent results), Runs,
Campaigns (each hypothesis arm as a line from fast to unity to real), Workers (with the
duration model) and a job page per job (`#/job/:id`, `#/run/:run_id`) with a 2-D ghost
replay: the run's `.ctraj` decoded in the browser (`public/traj.js`, tested on
`theater/tests/golden`), the course, and the same seed from the study's other runs as
ghosts; `#/fleet/:study` replays a study's per-iteration best and failures coloured by
iteration. Its token-free reads: `GET /v1/feed/runs/:run_id/traj[?lod=1]` (header cut to
public keys; `run.lod.ctraj` when the worker wrote one; read through the archive fallback,
`503` while only an offline archive has it, `410` once gone), `GET /v1/feed/courses/:truth_sha256` (from `truths/<sha>` or a run's
`course_truth.json`, kept reduced at `courses/<sha>.json`), `GET /v1/feed/ghosts?run=`,
`GET /v1/feed/fleet?study=` and `/v1/feed/fleet.pack` (LODs as `u32 len, bytes`), and
`GET /v1/feed/renders?job=|study=` with `.../renders/:id/video` (Range). The "Render in
Unity theater" button needs an operator token (`POST /v1/admin/operator_token`, pasted
into the page, kept for the tab): it records a request (`POST /v1/operator/renders`,
with a purpose); whoever runs `theater.render` lists `GET /v1/admin/renders?status=requested`,
reports `POST /v1/admin/renders/:id {status, render_job_id}` and attaches the video with
`PUT /v1/admin/renders/:id/video/final.mp4`. The storage design's `/v1/replay/*` (handles,
tiers) can take over the replay reads; the page only needs these paths.

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

**Job inputs**: files a job runs on that are not in a repo (a render's manifest and track pack): an admin
uploads each as `PUT /v1/admin/inputs/<sha256>` (the body must hash to the key; kept, never swept) and names
them in the job spec; any worker or viewer reads `GET /v1/inputs/<sha256>`, and the backend checks the hash.

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
if missing, provisions the host, joins with the invite (the token goes to `~/.config/crane-cluster/token`,
mode 600) and starts the service.

Provisioning (`install/provision.yaml`, run by `python -m crane_cluster.provision`; `--minimal` /
`-Minimal` skips it) is user-level and idempotent, so a re-run upgrades: uv and a venv with the
packages (numpy, pyyaml, py_trees, mcap, mcap-ros2-support); git (Windows: portable MinGit from the
latest git-for-windows release in `%USERPROFILE%\Tools\MinGit`; Linux: the system git, else an owner
step); a static ffmpeg (BtbN) unless one is on PATH; the .NET SDK (`dotnet-install`, for crane's course
generator) unless the system has it; a Vulkan check; a Unity player (`--player ZIP|DIR|URL`; none is
published on the coordinator, a player is over its artifact limit) and `UnityEngine.CoreModule.dll`
(`--unity-managed DIR|DLL|URL`, else a Unity editor or a Mono player on the host); and the plan's repos,
mirrored in the worker's git cache and checked out at main under `<data dir>/repos/<name>`. Each plan
backend whose requirements are met and that the YAML lacks is added to it (`fastsim`, `fastsim_roboboat`,
`unity`). Tool folders and environment go to `tools.json` beside the config, which the worker puts on
its PATH at start (no system PATH change). Detected tags join the YAML's: the os, `gpu`, `vulkan` and the
tools found (`git`, `ffmpeg`, `dotnet`, `docker`). A re-run restarts the worker when it holds no job. Configure backends, caps and policy in the YAML
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

On Windows it is `%LOCALAPPDATA%\crane-cluster\bin\crane-worker.cmd on|off|status`: `off` writes the flag
and disables the scheduled task (the running worker releases its job and idles), `on` enables and starts it.

Pause rules a host can set in its YAML: `policy.quiet_hours`, `policy.pause_on_activity`
with `policy.idle_seconds` (Linux reads keyboard/mouse activity from `/dev/input`
when the user may, e.g. the `input` group, so it works under Wayland; else
`xprintidle` or logind), and `pause_when: <shell command>` (exit 0 = pause).

Windows: `install.ps1` takes Python from `-Python`, else makes a uv venv
(`%LOCALAPPDATA%\crane-cluster\venv`; uv is downloaded to `~\.local\bin` when missing), else uses
`python`/`py` (the Microsoft Store stubs don't count).
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
- `theater-render` (`backends.theater:ClusterBackend`): a ghost-theater video (`theater/README.md`), one seed
  per chunk of frames (`spec.chunk_frames`), rendered by the worker player (`--crane-theater`) batchmode on
  the GPU and encoded by ffmpeg (`ffmpeg:` path, or `ffmpeg_url` + `ffmpeg_sha256` of a pinned build). Its
  inputs (manifest, track pack, world files) are job inputs (see Concepts), fetched by hash. Artifacts per chunk:
  `video.mp4`, `frames.jsonl`, `render_report.json`, `contact.png` (summary), `player.log` (raw). Jobs need
  `{gpu: true, tags: [theater-render]}`; a worker gets the tag once a render spike passed there.
  `python3 -m theater.render submit|assemble` creates the job and joins its chunks.

### Hosts

- **innovation** (Linux): `crane-worker.service`, `~/.config/crane-cluster/worker.yaml`,
  tags `innovation, nlc`; fastsim from a crane checkout (`path`/`root`).
- **simwin** (`ssh simwin`, Windows 11 "COLLIN", user Mrmag, i7-12700 20 threads, RTX 3060 12 GB,
  16 GB): `install.ps1 -Background -UnityManaged <UnityEngine.CoreModule.dll of 6000.5.10f1>`, tags
  `simwin, windows, nlc`, backends `fastsim`, `fastsim_nlc`, `fastsim_roboboat` and `unity`. Tools in
  `%USERPROFILE%\Tools` (MinGit, ffmpeg, dotnet, unity-managed), repos in `%LOCALAPPDATA%\crane-cluster\repos`.
  The crane copy for `fastsim`/`unity` is `%LOCALAPPDATA%\crane-cluster\crane` (`cluster/` and `fastsim/`
  plus `COMMIT`). The `unity` player `%LOCALAPPDATA%\crane-cluster\players\CRANE-Worker-Windows\CRANE.exe`
  (`CranePerformanceBuild.BuildLinuxWorker --crane-build-target windows64`, built on Linux) is not on the
  host as of 2026-11-02: give it with `-Player`. Config `%APPDATA%\crane-cluster\worker.yaml`, log
  `%LOCALAPPDATA%\crane-cluster\worker.log`. Update: copy `cluster/` there and re-run the installer.
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

Later deploys go through `./deploy.sh` (`npm run deploy`; `CRANE_URL` for another domain): it runs
`wrangler deploy`, then requires `/v1/ping` and `/v1/feed` to answer 200 within a minute and runs
`wrangler rollback` otherwise, so a broken deploy cannot leave workers and the study's `nlc.cluster`
without a coordinator (they fall back to local runs). It does not roll back a failure that was already
there before the deploy. It also copies crane_ml's `viewer/web` and the ctraj codec into `public/viewer/`
(ignored by git), so the dashboard's run pages link to the deep view at `/viewer/#/run/<run_id>`.

D1 on the free plan allows 5M rows read per day for the whole account: the feed caches ended jobs'
aggregates and the duration model, and the artifact tiering sweep runs every 6 h, to stay under it.
Past it every D1 route answers 503 `D1 daily quota exhausted` until 00:00 UTC.

GitHub, for each repo whose `main` workers should track: Settings → Webhooks →
payload URL `https://cluster.example.org/v1/webhook/github`, content type
`application/json`, the webhook secret, push events only.

Admin calls use `Authorization: Bearer $ADMIN_TOKEN`: `POST /v1/admin/invite {"n": 3}`,
`POST /v1/admin/viewer_token`, `PUT /v1/admin/repos {"name","url"}`,
`PUT /v1/admin/client_release {"version","min_version"}` (workers below
`min_version` get HTTP 426), `PUT /v1/admin/retention {"raw_days"}`.
