# Studies on the research cluster

```sh
crane cluster submit study.yaml --dry-run           # the plan: jobs per stage/arm/block, est per job, compute hours
crane cluster submit study.yaml                      # OUT/<study>/cluster/jobs.json, then research submit
crane cluster wait boat01 --timeout 3600             # one bounded wait, then the status table (exit 75 on timeout)
crane monitor boat01 --follow                        # local progress.json + the cluster, per stage/arm/block
crane cluster fetch boat01                           # rows into OUT/boat01/<stage>/<arm>/<cell>/, sha256-checked
crane study report study.yaml                        # reads local and fetched cells the same way
```

OUT is `--out`, else `CRANE_STUDIES`, else `~/.cache/crane/studies`. `submit`, `wait`, `status` and `fetch` take a
study, its OUT dir or a plan file; bare `crane cluster status` is `research status`. Other verbs (`crane cluster
digest`, `crane cluster why JOB`, `crane cluster cancel ...`) go to the research CLI unchanged; raw job ids and
research flags (`-p`) go to `research` itself.

An adaptive stage goes after the stage it reads:

```sh
crane cluster submit study.yaml --stage R && crane cluster wait nlc08 && crane cluster fetch nlc08
crane study promote study.yaml --stage C              # writes the promoted spec (top-k of stage R)
crane cluster submit study.promoted.yaml --stage C
```

## What a study compiles to

`crane.cluster.compile.compile(study, stage=None, block=None, ...)` gives one rc-job/1 body per (cell, seed chunk).

| Field | Value |
|---|---|
| `kind` | `crane_batch` (the `crane-batch` backend: one slot, every core); `gpu_command` for a GPU cell; `--kind` overrides |
| `seeds` | `[0]`: a batch job reports one run. The chunk is `spec.seeds` |
| `est_seconds` | per-unit estimate x chunk + 30 s, for the whole job |
| `needs` | `tags` (`--tag`, `--cloud` adds `cloud-ok` + `cpu`/`torch`, the spec's `pin.tags`, and `pin.hosts` as a worker tag), `gpu`, `vram_gb`, `min_cpu` |
| `repos` | `crane_ml` and the spec's own repo, each a local path at its exact HEAD commit (research submit bundles them) |
| `spec` | `crane.cluster.job/1`: study, stage, arm, block, role, cell, run_cell, cell_dir, seeds, spec_file, commit, dedup, touched_held_out, targets, `run`, `argv` |
| `job_id` | `<study>.<stage>.<arm>.c<i>.<dedup>`; dedup digests the chunk's unit cache keys, the commit and the spec |
| `campaign`, `purpose`, `expect` | from the spec (`--campaign`, `--purpose` override) |

A unit is one (cell, seed), its key `UnitCache.key(cell.unit(seed, commit))`, the key `crane study` caches it
under. Units already in the study's unit cache (OUT/<study>/cache) are left out, so a resubmitted plan only carries
what is missing, and an identical plan is `exists`. A changed plan with the same units and commit (other tags,
purpose, estimate) is a 409; submit names the fix.

**Chunks.** A crane.study spec packs its own jobs (`pack: {unit_s, target_s}`), and a job runs
`crane study run-cell SPEC --cell <its job id> --out DIR`. For any other study the compiler chunks and adds
`--seeds a,b,c`.

**Sizing.** Units per job = 15 min / per-unit estimate, at most 30 min, chunks as even as possible and the same across
arms (paired seeds progress together; chunk i gets priority +i). The estimate comes from `--est-unit-s`, the cell or
study (`est_unit_s`, `estimate(cell)`), `--est-from bench.json` (bench.fast, one worker) or `--measure` (one seed run
locally, never for T2). A unit over 30 min runs alone (warned), over 240 min is refused; a cell under 5 min is warned.

**Refused (exit 2):** uncommitted changes in the pinned dirs (`--allow-dirty` runs the commit), an uncommitted spec, a
spec outside `crane/runtime.sparse` (rented boxes would not have it), a cell or spec naming field data
(`field_corpus_*`, a bag dir such as `bags/`, `rosbag/`, `lake_bags/`, an `.mcap`/`.db3`/`.bag` file) or a pinned
commit tracking any, no time estimate, no purpose. Derived tables committed under other names pass.

**Held-out.** A held-out cell (its block's or its condition split's role) needs `--confirm`, and then the study's guard:
the `targets:` file committed, clean and older than now (`crane.study.guard`). With no guard to ask, the cell is
refused. Held-out jobs carry `--confirm` to run-cell, `touched_held_out` and the targets commit.

## On the worker

`spec.run` (py_command, gpu_command) and `spec.argv` (crane-command) are `python3 -m crane_backends.batch --job JSON`;
the `crane-batch` backend runs the same thing in process. It runs
`crane study run-cell SPEC --cell JOB --out DIR` with every core, then leaves in `$CRANE_RUN_DIR`:

| File | |
|---|---|
| `progress.json` | `{done, total, failed, rate_per_s, eta_s, phase, updated}`, atomic; the runner heartbeats it every 30 s while a unit runs |
| `cell.json` | study, stage, arm, block, role, cell, seeds, host, job_id, stall_s, commit, touched_held_out |
| `metrics.json` | rows, failed, units, wall_s, units_per_s, touched_held_out, and `<column>.mean` on dev blocks only; at most 1000 finite numbers |
| `failure.json` | reason (`crashed`, `timed out without progress`, `cancelled`), exit, a traceback sample |
| `artifacts/rows/` | the rows and `manifest.json` (`crane.data.manifest/1`, sha256 per file); each file at most 95 MB |

With an output root (`out_root` option, `CRANE_OUT_ROOT`, or `/workspace/out` on a rented box) the rows go to
`<root>/<study>/<job_id>/` and only `manifest.json` stays under `artifacts/rows/`. stdout gets one progress line and
one `RC-PROGRESS {"done": n, "total": m}` line a minute. A child whose progress does not move for `stall_s`
(max(600, 3 x unit + 120) s) is stopped.

**Signals.** SIGTERM or SIGINT stop the child's whole process group (exit 143). SIGUSR1 lets the current unit finish
(at most 60 s), then stops the group and exits 75, which the worker counts as a release. If the runner itself dies
(kill -9, a lost lease), a watchdog in its own session stops the group, so no all-cores child outlives its job.

worker.yaml: `crane_batch: {type: crane-batch, slots: 1}` (entry point `crane-batch` in pyproject.toml).

## Fetch and monitor

`crane cluster fetch` takes box pulls (`~/.cache/vast-out/<box>/<study>/<job_id>/`) first, else the coordinator's
artifacts (`RC_VIEWER_TOKEN` or `platform-viewer.token`). Each job lands in `<cell>/parts/<job_id>/`; the cell's
`manifest.json` lists every part's files, so `read_rows(cell_dir)` returns them all.

`crane monitor` reads every `progress.json` under OUT with its `cell.json`, `failure.json` and `metrics.json`, plus the
plan's jobs from the project job list and the public feed. Interim means appear for dev blocks only; held-out and
other blocks show progress only, and a held-out block that has started is flagged `touched_held_out`. A
`progress.json` older than 180 s (or the job's `stall_s`) means its runner is gone, and the job shows as stalled.
