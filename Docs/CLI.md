# crane CLI

```sh
crane doctor                                            what is installed, what works, what to run next
crane run --domain toy --seeds 0:3                      T0 abstract, scenarios made on demand
crane run --domain boat --tier T1 --seeds 0:2           T1 dynamic, the RoboBoat reference mission
crane sim up --headless                                 the T2 world, used like the robot
crane view --live --domain boat --tier T1 --seed 3      the native sandbox: a live sim with sliders
crane help                                              every noun and verb; crane NOUN VERB --help for one
```

Generated from `crane/commands.py` by `crane help --write`; `crane help --check` fails when it drifts. `crane NOUN VERB --help` prints the same entry.

## Conventions

`crane NOUN [VERB] [selectors] [flags]`. Output is plain aligned text; a failure is one line, `crane NOUN VERB: cause; fix: CMD`; output ends with `next: CMD` where there is a next step.

| Selector | Meaning |
|---|---|
| `--domain` | A domain: config/domains/DOMAIN.yaml. |
| `--tier` | The tier: T0 abstract, T1 dynamic, T2 unity. |
| `--env KIND:NAME` | The world: course:NAME, scene:NAME or world:NAME. |
| `--robot` | The domain's robot. |
| `--card REF` | A plant card ref or a profile JSON. |
| `--seed` | One seed. |
| `--seeds SPEC` | Seeds: N, A:B (B exclusive), a,b,c, @FILE or block:NAME. |
| `--set KEY=VALUE` | Override a config key; dotted keys nest, the value is YAML (repeatable). |
| `--out` | Output dir (default ~/.cache/crane/runs/DOMAIN/TIER/STAMP). |

| Global flag | Meaning |
|---|---|
| `--json` | One JSON document on stdout (JSONL for streams). |
| `--quiet` | Only results and failures. |
| `--dry-run` | Print the resolved plan; run nothing. |

Exit codes: 0 ok; 1 a check or run failed (judge FAIL included); 2 usage or config error; 75 a bounded wait (heavy, cluster) timed out.

## sim

T2: a domain's Unity world plus its ROS side, used like the robot.

### crane sim up

```sh
crane sim up --headless --seed 3 --current 0
crane sim up --card characterized --headless
crane sim up --dry-run
```

Start a domain's T2 world (the Unity player through heavy unity, plus its ROS side) and return once the course is ready; prints the status block.

What you'll see: one line as the ROS side starts and one as the player is queued on `heavy unity`, then the status block (mode, ROS_DOMAIN_ID, endpoint, player pgid, ROS container, course, current, logs, next:). --dry-run: the resolved player argv and docker run, nothing starts (crane.sim.plan/1 with --json).

Waits up to 60 s for the ROS endpoint and 300 s (SIM_UP_READY_S) for the course. Without --display, a window only on a terminal with a display, else headless under xvfb-run. Refuses (exit 2) while a sim runs. Exit 0 ready, 1 the world failed, 2 usage or config. Everything specific comes from config/domains/DOMAIN.yaml tiers.T2. Env: ROS_DOMAIN_ID, ASTRO_STATE / MHSEALS_HARDWARE_CONFIG (passed into the ROS side), the image env, SIM_UP_*, CRANE_PLAYER, HEAVY. Same as Tools/RoboBoat/sim_up.

| Param | Type | Description |
|---|---|---|
| `--seed` | int | The world seed (default 1). |
| `--env` | env | KIND:NAME, the world (default course:roboboat_2026); --course FAMILY is the same as --env course:FAMILY. |
| `--card` | path | A plant profile JSON or a card name (--profile-json is the same). |
| `--profile` | str | The player's runtime profile (default interactive-low). |
| `--current` | str | SPEED,TOWARD_DEG (m/s, deg), or 0 for still water (default the scene current). |
| `--display` | str | Window or xvfb (--window, --headless are the same). One of `window`, `headless`. |
| `--player` | path | The player (else CRANE_PLAYER, then the build). |
| `--robot` | str | The domain's robot. |
| `--set` | str | Pin one world parameter (repeatable). |
| `--domain` | str | config/domains/DOMAIN.yaml (default boat). |
| `--dry-run` | bool | Print the resolved player argv and docker run; start nothing. |
| `--json` | bool | Progress on stderr, the status as one JSON document. |

raw: `Tools/RoboBoat/sim_up up`; --json: `crane.sim.status/1`

### crane sim down

```sh
crane sim down
```

Stop what crane sim up started: its player's process group (TERM, KILL after 10 s) and its ROS container, nothing else.

What you'll see: the down line.

| Param | Type | Description |
|---|---|---|
| `--json` | bool | One JSON document instead of the down line. |

raw: `Tools/RoboBoat/sim_up down`; --json: `crane.sim.down/1`

### crane sim restart

```sh
crane sim restart nav
crane sim restart player --json
```

Restart one part of the running world and keep the rest: nav (only the Nav2 nodes, so new nav2_limits in the state take effect), ros (the ROS container; the player stays) or player.

What you'll see: one line per step, then `crane sim: PART restarted, ready: ...` and next:. --json: {schema, restarted, ready, at, status}.

nav stops the nav nodes named by the domain recipe (tiers.T2.ros.restart.nav) inside the ROS container, starts them again from the same launch (only:=navigation), and waits up to the recipe's ready_s (90 s) for bt_navigator to be active; a domain without the recipe refuses and names `restart ros`. ros re-runs the saved docker run, waits up to 60 s for the endpoint and 120 s for the ready nodes. player re-runs the saved player argv, waits up to 300 s (SIM_UP_READY_S) for the course. Nav goals in flight are lost. Exit 0 ok, 1 failed, 2 usage, config or nothing running, 75 timed out waiting.

| Param | Type | Description |
|---|---|---|
| `what` | str | The part to restart. One of `nav`, `ros`, `player`. |
| `--json` | bool | One JSON document. |
| `--domain` | str | config/domains/DOMAIN.yaml (default boat). |

raw: `docker exec CONTAINER pkill ...; docker exec -d CONTAINER ros2 launch ...`; --json: `crane.sim.restart/1`

### crane sim status

```sh
crane sim status
```

The status block (mode, ROS_DOMAIN_ID, endpoint, player, ROS container, course, current, logs, next commands). Read-only.

What you'll see: the status block, ending with next:. --json: {schema, domain, mode, player, ros, course, current, logs, next}.

| Param | Type | Description |
|---|---|---|
| `--json` | bool | One JSON document. |

raw: `Tools/RoboBoat/sim_up status`; --json: `crane.sim.status/1`

### crane sim check

```sh
crane sim check --wait 120
```

Is the running sim ready (player, course, plant, ROS side, nav stack)? --wait SECONDS polls until ready; CHECK INPUTS runs an offline Tools/SimCheck check.

What you'll see: one row per check (ok/FAIL, name, detail), then `sim check: ready` or the failed names. Exit 1 when not ready, 75 when --wait SECONDS pass first. --json: {schema, ready, exit, failed, notices, checks[{name, ok, detail}]}.

| Param | Type | Description |
|---|---|---|
| `--wait` | int | Poll every 2 s until ready or this many seconds pass. |
| `--json` | bool | One JSON document. |
| `--domain` | str | config/domains/DOMAIN.yaml (default boat). |

raw: `Tools/RoboBoat/sim_up check`; --json: `crane.sim.check/1`

### crane sim logs

```sh
crane sim logs ros -n 100
```

The last lines of the player, player stdout and ROS logs, each under its path.

| Param | Type | Description |
|---|---|---|
| `which` | list | Which logs (default all three). One of `player`, `stdout`, `ros`. |
| `-n` | int | Lines per log. (default `40`) |

raw: `Tools/RoboBoat/sim_up logs`

## run

Episodes at a tier, one run dir per seed.

### crane run

```sh
crane run --domain toy --seeds 0:3
crane run --domain boat --tier T1 --seeds 0:2 --out /tmp/runs
crane run --domain boat --tier T1 --astro-test speed-surge --card /tmp/fit/profile.json
```

Run episodes at a tier, one run dir per seed (OUT/VARIANT/s<seed>). T0 abstract and T1 dynamic; T2 is not yet available (use crane sim up). Missing scenarios are generated on demand.

What you'll see: one line per seed (run dir, outcome, sim and wall time), then runs/s and next:. --dry-run prints the resolved plan and runs nothing; --json prints one summary document.

| Param | Type | Description |
|---|---|---|
| `--domain` | str | A domain: config/domains/DOMAIN.yaml (boat, toy). |
| `--config` | path | A fastsim domain YAML, used as is. |
| `--tier` | tier | The tier (default: the domain's first tier, T0 with --config). One of `T0`, `T1`, `T2`, `abstract`, `dynamic`, `unity`. |
| `--seeds` | str | N, A:B (B exclusive), a,b,c, @FILE or block:NAME (required, except with --astro-test: 0). |
| `--out` | path | Output root; runs go to OUT/VARIANT/s<seed> (default ~/.cache/crane/runs/DOMAIN/TIER/STAMP). |
| `--workers` | int | Worker processes (default the config's workers). |
| `--set` | str | Override a config key; dotted keys nest, the value is YAML (repeatable). |
| `--variant` | str | The name of this design or parameter set. (default `base`) |
| `--generate` | bool | Regenerate the seeds' scenarios (missing ones are always made). |
| `--allow-dirty` | bool | Run an adapter tree with uncommitted changes. |
| `--force` | bool | Move a run dir holding another run (or an incomplete one) to <dir>.stale-N and rerun it; without it such a dir is refused and matching ones are reused. |
| `--engine` | str | fastsim: one scalar episode per seed; batched: every seed at once on the batched engine (Docs/Engine.md). One of `fastsim`, `batched`. (default `fastsim`) |
| `--backend` | str | With --engine batched: numpy (float64 reference), numba (every core) or jax (GPU). One of `numpy`, `numba`, `jax`. (default `numba`) |
| `--dtype` | str | With --engine batched: default float64; jax on a consumer GPU wants float32 (bench/RESULTS.md). One of `float64`, `float32`. |
| `--record` | int | With --engine batched: also write traj.ctraj with every K-th track row (default: run.json, events.jsonl, metrics.json only). |
| `--astro-test` | str | An astro test or group (still, couple-*, speed-*, steps-*, nav2, all) at T0/T1, with metrics named as astro's result fields. |
| `--card` | str | With --astro-test: a profile JSON, a plant card or a factorial manifest (repeatable). |
| `--tests` | path | With --astro-test: astro's tests file or a bundle holding one. |
| `--space` | str | With --astro-test: the tests file's space (default large). |
| `--amp` | float | With --astro-test: the effort amplitude (default the space's). |
| `--json` | bool | One summary document. |
| `--dry-run` | bool | Print the resolved plan; run nothing. |
| `--quiet` | bool | Only results and failures. |

raw: `python3 -m fastsim.batch --config CONFIG --seeds 0:3 --out OUT`; --json: `crane.run.summary/1`

## study

A study spec: arms x axes x seed blocks, run locally or on the cluster.

### crane study init

```sh
crane study init --out study.yaml
crane study init --domain boat > boat_study.yaml
```

Write a commented crane.study/1 spec to start from (stdout, or --out FILE).

What you'll see: the spec on stdout, or `wrote FILE` and next:.

| Param | Type | Description |
|---|---|---|
| `--domain` | str | The domain its sim arms run (default toy). |
| `--out` | path | Write the spec here instead of stdout. |
| `--force` | bool | Overwrite FILE. |
| `--json` | bool | One JSON document on stdout (JSONL for streams). |

--json: `crane.study.init/1`

### crane study plan

```sh
crane study plan examples/studies/toy_paired.yaml
crane study plan examples/studies/boat_cards.yaml --json
```

Count what a study spec compiles to: cells, units, jobs and estimated time per stage, with what is already cached. Nothing runs.

What you'll see: one row per stage (tiers, cells, units, done, held-out units, jobs, est, per job), the seed blocks and splits, pending adaptive stages, then next:. --json adds jobs[] (job_id, cell, seeds, est_seconds, kind, campaign, purpose, expect, pin, argv): the cluster compiler's input.

A spec error is one line naming the key and the fix (exit 2). Estimates use pack.unit_s; jobs pack a cell's seeds to about pack.target_s (default 900 s) each.

| Param | Type | Description |
|---|---|---|
| `spec` | path | The study spec (YAML). |
| `--stage` | str | Only this stage. |
| `--block` | str | Only this seed block or split. |
| `--out` | path | The study output to count done units in (default ~/.cache/crane/studies/NAME). |
| `--json` | bool | One JSON document with jobs[]. |

--json: `crane.study.plan/1`

### crane study run

```sh
crane study run examples/studies/toy_paired.yaml --max-workers 4
crane study run examples/studies/toy_paired.yaml --block test --confirm
crane study run examples/studies/adaptive.yaml --stage V
```

Run a study locally: every stage in order (or --stage), resuming from the unit cache; held-out blocks wait for --block B --confirm and a committed targets file.

What you'll see: a progress line per minute, then one row per stage (cells, units, ran, reused, failed), the held-out blocks skipped, where the rows and manifest are, and next:.

OUT/progress.json is rewritten at least every 60 s (CRANE_RUN_DIR overrides where). Units are cached in OUT/cache/records.jsonl on (study, stage, arm, cell hash, seed, card hash, code commit) as each finishes; rerunning the same command runs only the missing ones. A held-out block refuses (exit 2) without --confirm, or when the spec's targets file is not committed, has local changes or is not older than now. Exit 0 ok, 1 units failed, 2 spec, usage or refusal.

| Param | Type | Description |
|---|---|---|
| `spec` | path | The study spec (YAML). |
| `--stage` | str | Only this stage (its select stage must be done). |
| `--block` | str | Only this seed block or split (a held-out one needs --confirm). |
| `--where` | str | local runs here; cluster: use crane study compile, then research submit. One of `local`, `cluster`. (default `local`) |
| `--confirm` | bool | Run held-out blocks (needs the committed targets file). |
| `--max-workers` | int | Worker processes. (default `1`) |
| `--out` | path | The study output (default ~/.cache/crane/studies/NAME). |
| `--json` | bool | One summary document; progress lines go to stderr. |
| `--dry-run` | bool | Print the plan (as study plan); run nothing. |
| `--quiet` | bool | Only results and failures. |

--json: `crane.study.run/1`

### crane study status

```sh
crane study status examples/studies/toy_paired.yaml
crane study status ~/.cache/crane/studies/toy_paired
```

A study's progress per stage and block, failures with a sample traceback, and interim per-arm means on dev blocks only (held-out blocks: progress only).

What you'll see: one row per stage and block (role, units, done, %), the progress.json line, the latest failure and its traceback tail, interim dev means per arm, then next:. Exit 1 when units failed.

| Param | Type | Description |
|---|---|---|
| `target` | path | A spec, or a study output dir. |
| `--out` | path | With a spec: its output (default ~/.cache/crane/studies/NAME). |
| `--json` | bool | One JSON document. |

--json: `crane.study.status/1`

### crane study report

```sh
crane study report ~/.cache/crane/studies/toy_paired
crane study report /tmp/study --metric succeeded --json
```

Paired analysis of a study output: each arm minus the baseline per cell, paired on the seed (bootstrap CI, sign-flip permutation, Holm across cells), via judge.stats.

What you'll see: the pooled table (stage, block, arm-baseline, metric, seeds, cells, diff, 95% CI, p, cells with Holm p < 0.05), held-out blocks still running, and where report.md / report.json went.

Reads OUT/manifest.json and its rows (or every */manifest.json under OUT, as a fetched cluster output). A held-out block enters only when all its units are done.

| Param | Type | Description |
|---|---|---|
| `out` | path | A study output dir. |
| `--baseline` | str | The arm the others are compared with (default the spec's). |
| `--metric` | str | Only these metrics (repeatable). |
| `--bootstrap` | int | Bootstrap resamples. (default `2000`) |
| `--json` | bool | One JSON document (also written to OUT/report.json). |

--json: `crane.study.report/1`

### crane study promote

```sh
crane study promote examples/studies/adaptive.yaml --stage V
```

Freeze an adaptive stage's selection (top-k of the stage it reads) into a spec whose stage names the chosen arms, to run here or send to the cluster.

What you'll see: the ranking (rank, group, mean, n, chosen) and the written spec, then next:.

| Param | Type | Description |
|---|---|---|
| `spec` | path | The study spec (YAML). |
| `--stage` | str | The stage with select:. |
| `--out` | path | The study output holding the source stage (default ~/.cache/crane/studies/NAME). |
| `--write` | path | Where the promoted spec goes (default SPEC.STAGE.yaml beside the spec). |
| `--json` | bool | One JSON document. |

--json: `crane.study.promote/1`

### crane study run-cell

```sh
crane study run-cell study.yaml --cell JOB_ID --out /workspace/out/STUDY/JOB_ID
```

Run one cell or one packed job of a study into DIR: what a cluster job runs (rows, manifest.json with sha256, metrics.json, progress.json).

What you'll see: one line: ran, reused, failed, rows and DIR. Exit 1 when units failed, 2 a refusal.

JOB_ID is a jobs[].job_id of crane study plan --json (CELL_ID, or CELL_ID.jK for one seed chunk). Held-out cells need --confirm and the committed targets file, as study run, or on a box with no git checkout --targets-sha256 from the compiled job. metrics.json holds at most 1000 finite numbers; for held-out cells only unit counts.

| Param | Type | Description |
|---|---|---|
| `spec` | path | The study spec (YAML). |
| `--cell` | str | The cell or job id. |
| `--out` | path | The job output dir. |
| `--seeds` | str | Only these of its seeds. |
| `--confirm` | bool | Run a held-out cell. |
| `--targets-sha256` | str | The targets file hash the submit side checked in git (crane study compile pins it); stands in for git on a box. |
| `--json` | bool | One JSON document. |

--json: `crane.study.run_cell/1`

### crane study compile

```sh
crane study compile examples/studies/toy_paired.yaml --write /tmp/jobs.json
crane study compile examples/studies/toy_paired.yaml --block test --confirm --json
```

Compile a study's jobs to rc-job/1 bodies for the research cluster: one crane_batch or gpu_command job per cell seed chunk (5-30 min), each running crane study run-cell through crane-batch. The compiler crane cluster submit uses.

What you'll see: {jobs: [rc-job/1 ...]} on stdout, or `N jobs in FILE` and next: research submit.

Needs purpose (20-600 chars, no local path) and pack.unit_s. Jobs pin crane_ml (and the spec's repo) at the exact commit, carry dedup ids from the unit keys and the commit, needs from pin:, campaign and expect; units in the study's unit cache are left out; field data is refused. Held-out jobs are left out without --confirm; with it the targets file must be committed, clean and older than now (checked here in git), and each held-out job pins its sha256, which crane-batch passes to run-cell as --targets-sha256 for the box to check. Exit 2 on a refusal.

| Param | Type | Description |
|---|---|---|
| `spec` | path | The study spec (YAML). |
| `--stage` | str | Only this stage. |
| `--block` | str | Only this seed block or split. |
| `--confirm` | bool | Include held-out jobs (needs the committed targets file). |
| `--allow-dirty` | bool | Compile with uncommitted changes (jobs run the commit). |
| `--write` | path | Write {jobs: [...]} here instead of stdout. |
| `--json` | bool | One JSON document with jobs[]. |

--json: `crane.study.compile/1`

## runs

The local run index: list, show, query, diff.

### crane runs ls

```sh
crane runs ls --domain boat --tier T1
crane runs ls --study nlc08 --metric time --limit 20
```

List indexed runs, newest first: crane.run/1 dirs, fastsim run dirs and study outputs (manifest.json).

What you'll see: an aligned table (run, domain, tier, seed, outcome, arm, created, any --metric columns, path), then shown/matched/indexed counts and next:. A new, empty index first indexes ~/.cache/crane/runs.

The index is ~/.cache/crane/index.sqlite (CRANE_INDEX moves it); crane runs index ROOT adds roots. --cluster adds the coordinator's runs of --study (GET /v1/runs/query, read-only; CRANE_COORDINATOR, and CRANE_VIEWER_TOKEN or CRANE_TOKEN); offline it says so on stderr and lists the local runs. A held-out run that is not done is sealed: outcome and metrics read `sealed`, none are stored, and stderr says how many.

| Param | Type | Description |
|---|---|---|
| `--domain` | str | Only this domain. |
| `--tier` | tier | Only this tier. One of `T0`, `T1`, `T2`, `abstract`, `dynamic`, `unity`. |
| `--study` | str | Only this study. |
| `--where` | str | A condition, as crane runs query --where. |
| `--metric` | str | Show this metric as a column (repeatable). |
| `--limit` | int | At most this many rows (0: all). (default `50`) |
| `--cluster` | bool | Also the coordinator's runs of --study. |
| `--json` | bool | One JSON document: {schema, index, n, matched, indexed, runs}. |

raw: `sqlite3 ~/.cache/crane/index.sqlite "SELECT * FROM runs"`; --json: `crane.runs.list/1`

### crane runs query

```sh
crane runs query --where "outcome=TIMEOUT and tier=T1 and metric.time>120"
crane runs query --where "arm in (a, b) and not dirty=true" --metric min_clearance_m
```

Runs matching --where: FIELD OP VALUE joined by and / or / not and parentheses; FIELD is a column or metric.NAME.

What you'll see: the crane runs ls table for the matching runs. A bad expression is one line naming the column and an example (exit 2).

OP is = != < <= > >= or ~ (glob; a value without * or ? matches anywhere) or in (A, B). Columns: run_id, path, source, domain, tier, kind, seed, card, backend, dtype, commit, dirty, outcome, created, study, stage, arm, block, run_sha256, sealed. A sealed (held-out, not done) run has no outcome or metrics, so metric terms never match it. `null` tests for a missing value; commit= and run_id= take a hex prefix. Parsed to SQL with bound parameters, never evaluated.

| Param | Type | Description |
|---|---|---|
| `--where` | str | The condition. |
| `--metric` | str | Show this metric as a column (repeatable). |
| `--limit` | int | At most this many rows (0: all). (default `50`) |
| `--study` | str | With --cluster: the study the coordinator selects. |
| `--cluster` | bool | Also the coordinator's runs of --study. |
| `--json` | bool | One JSON document, as crane runs ls. |

--json: `crane.runs.list/1`

### crane runs show

```sh
crane runs show 3fa2c1d0
crane runs show ~/.cache/crane/runs/toy/T0/base-1a2b3c4d/base/s3
```

One run: its meta, metrics and files.

What you'll see: meta (run, path, source, domain, tier, seed, card, commit, outcome, study fields, identity hashes), the metrics, the files with sizes, then next: crane view RUN.

| Param | Type | Description |
|---|---|---|
| `run` | str | A run dir, or a run_id (a prefix will do) from the index. |
| `--json` | bool | One JSON document: {schema, run, meta, files, next}. |

--json: `crane.runs.show/1`

### crane runs diff

```sh
crane runs diff 3fa2c1d0 77b0e9aa
```

Two runs' meta and metric deltas, and why their identities differ (config, scenario, adapter, seed, core).

What you'll see: `identity: the same run` or which identity parts differ with the changed keys (config and adapter keys old -> new, the scenario names, the seeds, the core commits), the meta fields that differ, then each differing metric with A, B and B-A.

| Param | Type | Description |
|---|---|---|
| `a` | str | A run dir or run_id. |
| `b` | str | A run dir or run_id. |
| `--json` | bool | One JSON document: {schema, same_run, identity, meta, metrics}. |

--json: `crane.runs.diff/1`

### crane runs index

```sh
crane runs index ~/.cache/crane/runs /tmp/study-out
crane runs index --rebuild
```

Index the runs under ROOTs (default: the roots indexed before, else ~/.cache/crane/runs); incremental by mtime, then sha256.

What you'll see: each root, then `N records under R root(s): added, changed, unchanged, removed; total indexed`.

Reads crane.run/1 dirs, fastsim run dirs (their crane.run/1 meta, nothing written) and study outputs (manifest.json crane.data.manifest/1 with metrics.json). Records gone from a root are dropped, and so is a remembered root that is gone. Hidden and *.stale-N dirs are skipped. Held-out records stay sealed until done (the record says complete, or every progress.json up to the root is phase done) and are reread each time.

| Param | Type | Description |
|---|---|---|
| `roots` | list | Directories to index. |
| `--rebuild` | bool | Drop every row first and read everything again. |
| `--json` | bool | One JSON document with the counts. |

--json: `crane.runs.index/1`

## repro

Rerun one run and check it reproduces.

### crane repro

```sh
crane repro 3fa2c1d0 --check
crane repro ~/.cache/crane/runs/toy/T0/base-1a2b3c4d/base/s3
```

Rerun one run at its recorded commit (a temporary git worktree) and compare run_sha256, the files and the metrics.

What you'll see: run_sha256, tree_sha (or the first differing file) and metrics, each same or differs, then `repro: reproduced` or the rerun kept for crane runs diff. With --check, exit 1 when it does not reproduce.

Reruns fastsim runs (T0/T1): that commit's fastsim on the scenario file in the run dir, with the recorded config and provenance. Refuses with one line (exit 2) a run made from a dirty tree (its diff sha when recorded), a commit no repository here has, or a non-fastsim record. The worktree is always removed; your checkout is never touched.

| Param | Type | Description |
|---|---|---|
| `run` | str | A run dir, or a run_id from the index. |
| `--check` | bool | Exit 1 when the rerun differs. |
| `--timeout` | int | Seconds the rerun may take. (default `1800`) |
| `--json` | bool | One JSON document (crane.repro/1). |

raw: `git worktree add --detach TMP COMMIT, then fastsim.run.simulate on the recorded inputs`; --json: `crane.repro/1`

## data

Datasets from runs: export, schema, splits.

### crane data export

```sh
crane data export ~/.cache/crane/runs/toy/T0 --format npz --out /tmp/ds
crane data export /tmp/runs --select seed,outcome,obs --where 'seed>=10' --out /tmp/ds2
crane data export /tmp/ds --split /tmp/ds/splits.json:train --out /tmp/train
```

Run dirs, batch outputs and datasets to one columnar dataset (npz, parquet, csv or jsonl) plus manifest.json with a sha256 per file; --select, --where and --split pick the rows.

What you'll see: one line (rows, columns, files, MB, format), one row per file (path, rows, sha256), the manifest path, a note for confirmed held-out rows or non-finite floats, next:. --json: {schema, out, manifest, rows, columns, files, held_count, nonfinite}.

A run dir gives one metrics-first row (seed, outcome, condition.*, metrics, commit, run_sha256) and its recorded arrays as obs / traj columns (record: in the domain YAML, Docs/Data.md). Array columns are stacked to rows x T x channels (shorter rows NaN-padded, COL@len their lengths); with csv or parquet they go to PART.arrays.npz beside each file. Files roll over at 90 MB. Held-out rows (role, block_role or tag held-out, held_out true) are refused without --confirm. Exit 0 written, 2 usage (no rows, an unknown column, a non-empty --out, held-out rows).

| Param | Type | Description |
|---|---|---|
| `sources` | list | Run dirs, batch outputs, datasets (dirs with manifest.json) or row files (.parquet .csv .npz .jsonl). |
| `--format` | str | The file format (parquet needs pyarrow: the [data] extra). One of `npz`, `parquet`, `csv`, `jsonl`. (default `npz`) |
| `--select` | str | Columns to keep, comma-separated (default all). |
| `--where` | str | Keep rows where COLUMN OP VALUE holds; OP is = != > >= < <=; = and != take a,b (repeatable, all must hold). |
| `--split` | str | Keep the rows of one split of a crane data splits file. |
| `--confirm` | bool | Export held-out rows too (refused without it). |
| `--out` | path | The dataset dir to write (new or empty). |
| `--json` | bool | One JSON document. |

raw: `python3 -c "from crane.data.export import export; export([SRC], OUT)"`; --json: `crane.data.export/1`

### crane data schema

```sh
crane data schema /tmp/ds
crane data schema ~/.cache/crane/runs/toy/T0 --json
```

The columns of a dataset or of run dirs: name, type and, for arrays, rate and channels; plus the row count.

What you'll see: one line (rows, columns, files), one row per column (name, type, Hz and channels for arrays), next:. --json: {schema, sources, columns, rows, files, arrays}.

| Param | Type | Description |
|---|---|---|
| `sources` | list | A dataset dir, row files or run dirs. |
| `--json` | bool | One JSON document. |

raw: `python3 -c "import json; print(json.load(open('DS/manifest.json'))['columns'])"`; --json: `crane.data.schema/1`

### crane data splits

```sh
crane data splits /tmp/ds --by seed
crane data splits /tmp/ds --by timeline --held-out source=real --ratios train=0.8,calib=0.2
crane data splits /tmp/ds --by condition --domain boat --out /tmp/splits.json
```

A dataset's units (seeds, conditions or timelines) to named splits by a salted hash; held-out units go to held-out and never to train or calib.

What you'll see: one line (units, by, rows, out), one row per split (units, rows, fraction), the held-out rule, next:. --json: the split file itself plus out and written.

A row is held-out when its role / block_role / tag column says held-out, held_out is true, its block or seed is a held-out block of --domain's seeds, or it matches --held-out. A unit with any held-out row is held-out. A unit keeps its split as data grows (the hash, not the count, decides). Writing over an earlier split file refuses to move a unit into train or calib from elsewhere. Exit 0 written, 2 usage.

| Param | Type | Description |
|---|---|---|
| `sources` | list | A dataset dir, row files or run dirs. |
| `--by` | str | The unit: seed, condition (the condition.* columns), timeline, or COLUMN[,COLUMN]. (default `seed`) |
| `--ratios` | str | Split names and fractions (adding up to 1). (default `train=0.7,calib=0.15,test=0.15`) |
| `--held-out` | str | Rows matching COLUMN OP VALUE are held-out too (repeatable; e.g. source=real). |
| `--domain` | str | Its seeds: blocks with role held-out mark their seeds held-out. |
| `--salt` | str | The hash salt (another salt, another assignment). (default `crane`) |
| `--out` | path | The split file (default SRC/splits.json for one dataset dir). |
| `--dry-run` | bool | Print the splits; write nothing. |
| `--json` | bool | One JSON document. |

--json: `crane.data.splits/1`

## monitor

Progress of local and cluster work in one view.

### crane monitor

```sh
crane monitor boat01
crane monitor boat01 --follow --interval 60
crane monitor ~/.cache/crane/studies/boat01 --local --json
```

Progress of local and cluster work in one view: every progress.json under a study (or a dir) merged with its cluster jobs, per stage, arm and block.

What you'll see: one row per (stage, arm, block): done/total, rate, ETA, failed, jobs running/queued/done/failed, hosts and interim metric means (dev blocks only; held-out and other blocks show progress only), then failures with their reason (crashed, timed out without progress, cancelled), host and a traceback line, and next:. --json: {schema, at, where, study, sources, notes, finished, groups[{stage, arm, block, role, done, total, failed, rate_per_s, eta_s, hosts, jobs, failures, metrics}]}; with --follow one document per refresh (JSONL).

Local: progress.json with cell.json, failure.json and metrics.json beside it. Cluster: when OUT/STUDY/cluster/jobs.json exists, the project job list (research token) and the public feed (/v1/feed); unreachable means a note, not a failure. A progress.json older than 180 s counts as stalled. --follow refreshes every --interval s (5..300) and stops when every job has ended or after --timeout s (exit 75).

| Param | Type | Description |
|---|---|---|
| `target` | str | A study name under OUT, or a dir (default every study under OUT). |
| `--out` | path | The studies root (default CRANE_STUDIES, else ~/.cache/crane/studies). |
| `--follow` | bool | Refresh until every job has ended or --timeout passes. |
| `--interval` | float | Seconds between refreshes (5..300). (default `30.0`) |
| `--timeout` | float | With --follow: stop after this many seconds. (default `3600.0`) |
| `--local` | bool | Local progress files only; do not ask the cluster. |
| `--json` | bool | One JSON document (one per refresh with --follow). |

raw: `cat OUT/STUDY/**/progress.json; research status -p crane --campaign STUDY`; --json: `crane.monitor/1`

## env (alias: course)

Worlds: courses, scenes, worlds (kind from --env).

### crane env gen

```sh
crane env gen --seed 3 --out /tmp/c3.json
crane env gen --seeds 0:50 --out /tmp/courses
```

Generate course truth with the Unity course generator (headless, through heavy unity).

What you'll see: CRANE_COURSE_* lines. Options: [--family F] --seed N --out T.json | --seeds A:B --out DIR | --validate DIR --out V.json; --mode, --set. The family defaults to the generator's. Arguments are checked before the heavy unity slot is taken; --dry-run prints the command only.

| Param | Type | Description |
|---|---|---|
| `--dry-run` | bool | Print the resolved plan; run nothing. |

raw: `bin/heavy unity Tools/RoboBoat/generate.sh`

### crane env validate

```sh
crane env validate /tmp/c3.json
```

The independent constraint check of a course truth; exit 0 when every row passes.

What you'll see: one row per constraint. Options: TRUTH.json [--spec S] [--json].

raw: `python3 Tools/RoboBoat/validate.py`

### crane env render

```sh
crane env render /tmp/c3.json > c3.svg
```

A course truth as an SVG on stdout.

What you'll see: SVG. Options: TRUTH.json [--track T.jsonl].

raw: `python3 Tools/RoboBoat/render.py`

### crane env explain

```sh
crane env explain /tmp/c3.json
```

A course truth's tasks, objects and verdicts.

Not yet available; today use `python3 Tools/RoboBoat/course_truth.py summary TRUTH.json`.

### crane env diff

```sh
crane env diff a.json b.json
```

Two course truths side by side.

Not yet available; today use `python3 Tools/RoboBoat/course_truth.py diff A B`.

### crane env coverage

```sh
crane env coverage /tmp/courses
```

Task parameters and corner flags binned across course truths.

Not yet available; today use `python3 Tools/RoboBoat/coverage.py TRUTH.json...`.

## card

Plant cards: show, export, fit, validate.

### crane card show

```sh
crane card show
crane card show FAMILY/ROBOT@HEAD --json
```

A plant card: one row per block (parameters with their sd and status) and the validation rows.

What you'll see: a text table; --json: the card document (robot, sha, status, blocks, validation).

| Param | Type | Description |
|---|---|---|
| `ref` | str | FAMILY/ROBOT@HEAD, a sha prefix or a card dir (default: plantcard's default card). |
| `--json` | bool | The card as one JSON document. |

raw: `python3 -m plantcard show`; --json: `crane.card/1`

### crane card export

```sh
crane card export fastsim --out /tmp/card.json
```

One of a card's tier views (unity | nlc | control | fastsim) as JSON.

What you'll see: JSON on stdout, or --out FILE.

| Param | Type | Description |
|---|---|---|
| `view` | str | The tier view. One of `unity`, `nlc`, `control`, `fastsim`. |
| `ref` | str | The card (default: plantcard's default card). |
| `--out` | path | Write the JSON here. |

raw: `python3 -m plantcard export`

### crane card fit

```sh
crane card fit bundle.tgz --out /tmp/fit
```

Fit a card from field data.

Not yet available; today use `crane fit BUNDLE.tgz --bags DIR`.

### crane card factorial

```sh
crane card factorial spec.yaml
```

Card variants from a factorial spec, plus a manifest.

Not yet available; today use `crane fit BUNDLE.tgz --factorial SPEC.yaml`.

### crane card validate

```sh
crane card validate FAMILY/ROBOT@HEAD
```

A card's held-out prediction coverage.

Not yet available; today use `crane fit with two or more bundles`.

### crane card parity

```sh
crane card parity FAMILY/ROBOT@HEAD
```

A card's tier views agree (T1 vs T2).

Not yet available; today use `crane compare --tiers LOW HIGH`.

## fit

Field data to a profile and a plant card.

### crane fit

```sh
crane fit bundle-173031.tgz --bags ~/astro_dock/bags/sim/2026-11-24/bags --out /tmp/fit
```

An astro bundle to a profile (through the gate) and a plant card (through its bags as evidence); prints each parameter's status and why.

What you'll see: the gate verdict per parameter (identified / provisional / prior, and why), the evidence, the card and the held-out validation when there are two or more sessions.

Writes OUT/profile.json (sim_up --profile-json), OUT/profile.gate.json, OUT/cards and OUT/fit.json. A bundle made without --bags gives the gate only; --bags DIR reads bags beside it. --factorial SPEC.yaml writes card variants and a manifest.

| Param | Type | Description |
|---|---|---|
| `bundles` | list | Astro bundles (.tgz); each is one session. |
| `--out` | path | Output dir (default BUNDLE.fit/ beside the first bundle, else ~/.cache/crane/fit/BUNDLE/ when that is not writable). |
| `--bags` | path | A bag dir to read besides the bundle (read only; repeatable). |
| `--base` | str | The base profile the gate imports into (default Ideal). |
| `--robot` | str | The card's robot name (default astro/<state file stem>). |
| `--n` | int | Rollouts per held-out segment. (default `64`) |
| `--no-card` | bool | Stop after the gate (no evidence, no card). |
| `--factorial` | path | Card variants from a spec (measured vs prior levels per property). |

raw: `python3 -m Tools.Characterization.from_bundle`

## field

Field data as crane.run/1 records.

### crane field import

```sh
crane field import ~/astro_dock/bags/sim/2026-11-24/bags/171208-smoke --out /tmp/field
```

A bundle (made with --bags), a bag dir or an .mcap file to crane.run/1 records: odometry trajectory, /events and log.txt lines, frame and clock.

What you'll see: one line per bag: run dir, duration, odometry samples and rate, path length, events, frame, clock.

| Param | Type | Description |
|---|---|---|
| `source` | path | A bundle, a rosbag2 dir or an .mcap file. |
| `--out` | path | The run dir (one bag) or the root (several). |
| `--odom` | str | The odometry topic. (default `/odom/local`) |
| `--events` | str | The astro events topic. (default `/events`) |
| `--frame` | str | ID=enu\|ned for an odometry frame id that is not a REP 103/105 name. |
| `--hz` | float | The trajectory rate (Hz). (default `20.0`) |
| `--domain` | str | The run's domain. (default `boat`) |

## compare

Compare two run sets.

### crane compare

```sh
crane compare --field bundle-173031.tgz runs/sim
crane compare --field bundle-173031.tgz runs/sim --bounds file --fit /tmp/fit
```

Two run sets as a Markdown table and JSON: variant A/B on one frame and clock, tier gap (--tiers) or field vs sim per astro test (--field BUNDLE RUNS).

What you'll see: a Markdown table on stdout (--json: {kind, result} instead) and, with --out, compare.json and compare.md. With --bounds: pass/FAIL per row, the gate and a verdict line; exit 0 pass, 1 FAIL, 2 usage.

| Param | Type | Description |
|---|---|---|
| `runs` | list | Two run sets, A then B; with --field, the sim runs. |
| `--tiers` | bool | A is the lower tier, B the higher. |
| `--field` | path | An astro bundle to compare the runs against. |
| `--metric` | str | Only these metrics (repeatable). |
| `--direction` | str | --tiers: better is higher or lower. One of `higher`, `lower`. (default `higher`) |
| `--band` | float | --tiers: the allowed \|low - high\| gap (metric units). |
| `--fidelity` | path | --tiers: also replay B's goal windows on CONFIG's vehicle. |
| `--bounds` | str | --field: pass/FAIL per row against config/fidelity/DOMAIN.yaml ('file') or FILE, and the fit's gate; a verdict line, exit 1 on FAIL ('card': not yet available). |
| `--fit` | path | --bounds: the `crane fit --out` dir whose gate is required (default fit-<bundle>). |
| `--domain` | str | --bounds: config/fidelity/DOMAIN.yaml. (default `boat`) |
| `--out` | path | Write compare.json and compare.md here. |
| `--json` | bool | One JSON document: {schema, kind, result}; with --bounds, result holds rows, gate and verdict. |

--json: `crane.compare/1`

## judge

Score runs against a judge pack.

### crane judge fastsim

```sh
crane judge fastsim /tmp/runs --out /tmp/runs-judge
```

Judge every fastsim run under ROOT with the generic motion pack.

What you'll see: the one-line verdict; with --out, report.json and Markdown. Exit 1 on FAIL.

| Param | Type | Description |
|---|---|---|
| `root` | path | The runs root. |
| `--out` | path | The report dir. |
| `--json` | bool | The report as one JSON document. |

raw: `python3 -m judge fastsim`; --json: `crane.judge.report/1`

### crane judge oneline

```sh
crane judge oneline /tmp/runs-judge/report.json
```

A report's one-line verdict; exit 1 on FAIL.

| Param | Type | Description |
|---|---|---|
| `report` | path | report.json |
| `--json` | bool | One JSON document on stdout (JSONL for streams). |

raw: `python3 -m judge oneline`

### crane judge markdown

```sh
crane judge markdown /tmp/runs-judge/report.json
```

A report as Markdown; exit 1 on FAIL.

| Param | Type | Description |
|---|---|---|
| `report` | path | report.json |

raw: `python3 -m judge markdown`

### crane judge demo

```sh
crane judge demo
```

The toy quadrotor on two tiers: a plant fault, a decision fault and a clean control.

| Param | Type | Description |
|---|---|---|
| `--out` | path | Write the reports here. |

raw: `python3 -m judge demo`

### crane judge run

```sh
crane judge run --pack generic_motion /tmp/runs
```

Judge runs against a named pack.

Not yet available; today use `crane judge fastsim ROOT`.

### crane judge report

```sh
crane judge report /tmp/runs-judge/report.json
```

A report as text, Markdown or JSON.

Not yet available; today use `crane judge markdown REPORT.json`.

## check

Acceptance checks on recorded results.

### crane check axis

```sh
crane check axis results.json
```

Axis and sign conventions of recorded results.

Not yet available; today use `crane sim check axis INPUTS`.

### crane check sitl

```sh
crane check sitl results.json
```

The SITL acceptance check.

Not yet available; today use `crane sim check sitl INPUTS`.

### crane check docking

```sh
crane check docking results.json
```

The docking acceptance check.

Not yet available; today use `crane sim check docking INPUTS`.

### crane check smoke

```sh
crane check smoke results.json
```

The smoke check.

Not yet available; today use `crane sim check smoke INPUTS`.

### crane check truth-subscribers

```sh
crane check truth-subscribers
```

No node outside the allowlist subscribes to /crane/truth/*.

Not yet available; today use `python3 Tools/RoboBoat/check_truth_subscribers.py`.

## view

The native sandbox window.

### crane view

```sh
crane view --live --domain boat --tier T1 --seed 3
crane view /tmp/runs/base/s3 --screenshot /tmp/view.png --at 60
```

The native sandbox window. Replay runs overlaid in one frame and clock, or run a T0/T1 sim live with sliders, rerun from t, and Promote to a crane run spec. --screenshot renders offscreen (no window).

What you'll see: a window: the top-down map, the event lanes, play/step/speed/scrub; live mode adds the parameter sliders, Rerun from t or 0 and Promote. With --screenshot, a PNG and no window.

| Param | Type | Description |
|---|---|---|
| `runs` | list | crane.run/1 dirs, fastsim run dirs or .ctraj files. |
| `--live` | bool | Run the sim in-process (with --domain, --tier T0\|T1, --seed). |
| `--domain` | str | live: the domain. (default `boat`) |
| `--tier` | tier | live: the tier. One of `T0`, `T1`, `abstract`, `dynamic`. (default `T1`) |
| `--seed` | int | live: the scenario seed. |
| `--offset` | str | Run 2, 3, ...: the first run's time of its t = 0. |
| `--transform` | str | Run 2, 3, ...: {from, to, xyz, yaw} mapping its origin onto the first run's. |
| `--3d` | bool | Open in the 3-D view (6-DOF records). |
| `--screenshot` | path | Render offscreen to this PNG and exit. |
| `--at` | float | Screenshot: the playhead time (s; default the end). |
| `--promote-dir` | path | Where Promote writes specs. (default `runs/promoted`) |

## board

The web replay board.

### crane board

```sh
crane board /tmp/runs
```

The web replay board over local runs (crane.run/1 records and .ctraj), courses and judge reports; prints its URL.

What you'll see: a local URL (default http://127.0.0.1:8765) listing every record under ROOTS, with their event lanes and constraint layers.

| Param | Type | Description |
|---|---|---|
| `roots` | list | Dirs to index (default .). |
| `--port` | int | Local port. (default `8765`) |
| `--host` | str | The address to bind. (default `127.0.0.1`) |

raw: `python3 -m viewer serve`

## media

Renders, shots and assembled videos.

### crane media render

```sh
crane media render RUNS --out /tmp/media
```

Render runs to video.

Not yet available; today use `python3 -m theater select ... then theater/pack`.

### crane media shot

```sh
crane media shot RUN --out /tmp/shot.png
```

A screenshot of the board or a run.

Not yet available; today use `crane view RUN --screenshot OUT.png, or python3 -m viewer shot`.

### crane media assemble

```sh
crane media assemble /tmp/media
```

Assemble rendered clips.

Not yet available; today use `ffmpeg on the rendered clips`.

## bench

Throughput benchmarks.

### crane bench player

```sh
crane bench player --sweep 1 2 4
```

The Unity player real-time factor: one worker, or a sweep over worker counts, through heavy unity.

What you'll see: per-worker results under PerformanceResults/ and, for a sweep, a summary.json per worker count. Player settings come from env (CRANE_PLAYER, CRANE_DURATION, CRANE_SCENE, ...).

| Param | Type | Description |
|---|---|---|
| `--worker` | int | Run one worker with this id. |
| `--sweep` | list | Worker counts to sweep (default config/bench.yaml). |

raw: `bin/heavy unity Tools/Bench/sweep_workers.sh 1 2 4`

### crane bench fast

```sh
crane bench fast --engine batched --domain boat --tier T1 --backend numba,jax --E 1024,4096 --out /tmp/bench
crane bench fast --equivalence --tier T1 --seeds 0:500
crane bench fast --domain boat --tier T1 --seeds 0:50 --out /tmp/b
```

Fast-tier throughput: the scalar table B, the batched engine (--engine batched) or the gate-3 equivalence report (--equivalence).

What you'll see: bench.fast's tables: bench.json/bench.md (scalar) or batched.json/batched.md (episodes/s per backend and E, x table B, the time split); arguments go to bench.fast unchanged.

raw: `python3 -m bench.fast`

### crane bench cluster

```sh
crane bench cluster --domain boat
```

Throughput and cost on the cluster.

Not yet available; today use `python3 -m bench.fast with its --fresh-jobs, on a cluster host`.

## cluster

Research-cluster jobs: submit, wait, status, fetch (other verbs go to the research CLI).

### crane cluster submit

```sh
crane cluster submit study.yaml --dry-run
crane cluster submit study.yaml --est-unit-s 12
crane cluster submit study.yaml --stage A --block dev --cloud
crane cluster submit /tmp/jobs.json
```

Compile a study spec to rc-job/1 jobs (one per cell x seed chunk, 5-30 min each, pinned to the exact commit) and submit them with the research CLI; a jobs.json goes as is.

What you'll see: the plan table (stage, arm, block, role, kind, jobs, units, est per job, compute hours) and any warnings, then the created/exists/duplicate counts and next:. --dry-run sends nothing (with --json: the plan and every job body).

Jobs: kind crane_batch (the crane-batch backend: one slot, every core) or gpu_command for a GPU cell; seeds [0] with the chunk in spec.seeds; spec.run and spec.argv run python3 -m crane_backends.batch, which runs crane study run-cell SPEC --cell JOB --out .. (the study's own job ids and seed chunks; --confirm --targets-sha256 on held-out jobs); job ids come from the unit cache keys and the commit, so resubmitting the same plan is idempotent. Refused (exit 2): uncommitted changes (--allow-dirty runs the commit), a spec outside crane/runtime.sparse, a cell naming field data (~/field_corpus_*, a bag dir, .mcap/.db3/.bag), a held-out block whose targets file is not committed before now, a cell with no time estimate. Without --confirm held-out cells are left out (only held-out selected: exit 2). The same compiler as crane study compile. The plan is saved to OUT/STUDY/cluster/jobs.json before research submit. An adaptive stage goes after its source: submit --stage A, wait, fetch, crane study promote --stage B, submit the promoted spec.

| Param | Type | Description |
|---|---|---|
| `source` | str | A crane.study/1 spec, or an rc-job/1 list. |
| `--stage` | str | Only this stage. |
| `--block` | str | Only this seed block. |
| `--est-unit-s` | float | Seconds per (cell, seed) on one job (default the study's estimate). |
| `--est-from` | path | A bench.fast bench.json: seconds per episode at one worker. |
| `--measure` | bool | Time one seed of the first cell locally (never for T2). |
| `--project` | str | The research-cluster project. (default `crane`) |
| `--campaign` | str | The campaign label (default the study name). |
| `--purpose` | str | 1-2 sentences, public on the feed (default the spec purpose:). |
| `--tag` | str | needs.tags (repeatable; all required). |
| `--cloud` | bool | Allow rented boxes: needs.tags += cloud-ok and cpu (torch for GPU cells); only with an approved REQUESTS.md row. |
| `--kind` | str | Override the backend kind (default the spec's kind:, gpu_command for a GPU cell). One of `crane_batch`, `gpu_command`. |
| `--confirm` | bool | Include held-out blocks (crane.study.guard checks the targets file in git; each job pins its sha256 for the box). |
| `--allow-dirty` | bool | Submit with uncommitted changes (jobs run the commit). |
| `--no-skip-done` | bool | Keep seeds already in the study's unit cache. |
| `--out` | path | The studies root (default CRANE_STUDIES, else ~/.cache/crane/studies). |
| `--dry-run` | bool | Print the plan; send nothing. |
| `--json` | bool | One JSON document. |

raw: `research submit -p crane OUT/STUDY/cluster/jobs.json`; --json: `crane.cluster.submit/1`

### crane cluster wait

```sh
crane cluster wait boat01 --timeout 3600
```

Block until a study's jobs end, up to --timeout seconds (one research wait), then print the status table.

What you'll see: one line on stderr naming the jobs and the timeout, then the status table (as cluster status). Exit 0 every job done, 1 some failed or were cancelled, 75 the timeout passed first (the jobs are untouched), 2 no plan or no research CLI.

| Param | Type | Description |
|---|---|---|
| `study` | str | A study name, its OUT dir, or a plan. |
| `--timeout` | float | Seconds to wait at most. (default `3600.0`) |
| `--out` | path | The studies root (default CRANE_STUDIES, else ~/.cache/crane/studies). |
| `--json` | bool | The final status as one JSON document. |

raw: `research wait -p crane JOB... --timeout 3600`; --json: `crane.cluster.status/1`

### crane cluster status

```sh
crane cluster status boat01
```

A study's cluster jobs per stage, arm and block, counted by state, with the failures (digest style; read-only).

What you'll see: a header with the state counts, one row per (stage, arm, block) with jobs, units and a column per state, then failures (state, job, last error line) and next:. --json: {schema, study, project, campaign, states, groups, failures, finished}. Without STUDY: research status (every job).

| Param | Type | Description |
|---|---|---|
| `study` | str | A study name, its OUT dir, or a plan (default: research status, every job). |
| `--out` | path | The studies root (default CRANE_STUDIES, else ~/.cache/crane/studies). |
| `--json` | bool | One JSON document. |

raw: `research status -p crane --campaign STUDY`; --json: `crane.cluster.status/1`

### crane cluster fetch

```sh
crane cluster fetch boat01
```

Bring a study's finished cluster outputs into OUT/STUDY/<stage>/<arm>/<cell>/, the layout a local run writes, each file checked by sha256.

What you'll see: one row per job not yet in place (fetched, missing, failed; rows; from a box pull or the cluster), the counts, and next:. Exit 1 when a file fails its sha256 check.

Box pulls (~/.cache/vast-out/BOX/STUDY/JOB/, CRANE_VAST_OUT) first, else the coordinator's artifacts (a viewer token: RC_VIEWER_TOKEN or ~/.config/research-cluster/platform-viewer.token). Each job lands in <cell>/parts/JOB/; the cell's manifest.json lists every part's files. Safe to rerun.

| Param | Type | Description |
|---|---|---|
| `study` | str | A study name, its OUT dir, or a plan. |
| `--out` | path | The studies root (default CRANE_STUDIES, else ~/.cache/crane/studies). |
| `--boxes-only` | bool | Only box pulls; do not ask the coordinator. |
| `--json` | bool | One JSON document. |

raw: `research wait -p crane JOB... --out DIR; rsync from the box`; --json: `crane.cluster.fetch/1`

## build

Build the player, images and the wheel.

### crane build player

```sh
crane build player --tarball
```

Build the headless Linux player through heavy unity and stamp its manifest with the source commit.

What you'll see: the Unity command, then the stamped Builds/CRANE-Worker/crane-build-manifest.json and, with --tarball, Builds/crane-player-<commit12>-linux-x86_64.tar.zst and its .sha256.

Needs the project's Unity editor. --skip-unity only stamps and packs an existing Builds/CRANE-Worker.

| Param | Type | Description |
|---|---|---|
| `--tarball` | bool | Also write the player tarball and its .sha256. |
| `--skip-unity` | bool | Stamp and pack an existing build only. |
| `--editor` | path | The Unity editor (default UNITY_EDITOR, else the ProjectVersion.txt editor). |
| `--builds` | path | The Builds dir (default <repo>/Builds). |
| `--dry-run` | bool | Print the Unity command and stop. |

### crane build images

```sh
crane build images core gpu
crane build images sim --player Builds/crane-player-X.tar.zst
```

Build the crane docker images (core, gpu, sim) tagged with the commit (crane:KIND-<commit12>); sim packs a player tarball into crane:core.

What you'll see: the docker build command per image, then one row per image (tag, size unpacked) and next:. --dry-run prints the commands only.

core < 300 MB compressed; gpu = python:3.12-slim + jax[cuda12] + numba, < 4 GB; sim = core + the player + xvfb/Mesa Vulkan, about 1.2 GB. sim takes the tarball from --player (default the newest Builds/crane-player-*.tar.* with its .sha256) and is built on the core image of the same run. Never pushes: .github/workflows/images.yml pushes lunarzdev/crane:KIND-<sha> (never :latest).

| Param | Type | Description |
|---|---|---|
| `kinds` | list | core, gpu, sim (default core). One of `core`, `gpu`, `sim`. |
| `--player` | path | The player tarball for sim (its .sha256 beside it). |
| `--repo` | str | The image name (lunarzdev/crane in CI). (default `crane`) |
| `--dry-run` | bool | Print the docker build commands; build nothing. |

raw: `docker build -f Dockerfile.KIND -t crane:KIND-COMMIT .`

### crane build wheel

```sh
crane build wheel
crane build wheel --out dist
```

Build the crane wheel and sdist (uv build, else python3 -m build) and their sha256.

What you'll see: the build command, then one row per file (name, size, sha256[:12]) and next:.

| Param | Type | Description |
|---|---|---|
| `--out` | path | Where the files go (default ~/.cache/crane/dist). |
| `--dry-run` | bool | Print the build command only. |
| `--json` | bool | One JSON document. |

raw: `uv build --out-dir DIR`

## deploy

Sync crane and the T2 player to a host.

### crane deploy

```sh
crane deploy HOST --dry-run
crane deploy HOST --player
crane deploy HOST --ref w5-builds
```

Bring a host up to this checkout over ssh: check its disk, sync the code (git fetch from this machine + ff-only, or rsync when it has no checkout), send the T2 player with --player (sha256 checked there, current + prev kept), then run crane doctor there.

What you'll see: the probe line (free disk, checkout, player), one row per step as it runs, then the host's crane doctor table and next:. --dry-run: the probe and the plan; nothing changes on the host.

Touches only ~/crane_ml and the player store (~/.local/share/crane/player) on HOST. Refuses (exit 2) when HOST has less free space than the player (tarball and unpacked) plus 1 GB, or when its checkout cannot fast-forward. The player tarball is --tarball, else the newest Builds/crane-player-*.tar.* with its .sha256, else the release player; rsync --partial resumes a cut transfer. HOST fetches from --from (default ssh://THISHOST/REPO, CRANE_DEPLOY_FROM). Without a .venv on HOST, bin/crane runs on its python3; next: names install.sh. local:DIR deploys into DIR as the home (no ssh; for tests). Exit 0 ok, 1 a step or the remote doctor failed, 2 usage or a refusal.

| Param | Type | Description |
|---|---|---|
| `host` | str | HOST or USER@HOST for ssh, or local:DIR. |
| `--player` | bool | Also send and install the T2 player. |
| `--tarball` | path | The player tarball (implies --player). |
| `--ref` | str | The branch or commit HOST fast-forwards to. (default `main`) |
| `--from` | str | The git URL HOST fetches from (default ssh://THISHOST/REPO). |
| `--dry-run` | bool | Probe HOST and print the plan; change nothing. |
| `--json` | bool | One JSON document. |

raw: `ssh HOST 'cd ~/crane_ml && git fetch ssh://THISHOST/REPO main && git merge --ff-only FETCH_HEAD'`; --json: `crane.deploy/1`

## update

Update crane and the T2 player from a release.

### crane update

```sh
crane update
crane update --dry-run
crane update --version 0.3.0 --player-only
```

Update this checkout (git fetch + ff-only) or this install (the release wheel), then the T2 player from the release into the player store (sha256 checked, current + prev kept).

What you'll see: the release line, one row per step (code, player), then next:. --dry-run prints the steps only.

The release is release.json of the latest GitHub Release, or of v<VERSION> (CRANE_RELEASES overrides the base URL or names a local dir laid out the same way). A checkout with no release yet still fast-forwards. Refuses (exit 2) when the store's disk has less free space than the player (tarball and unpacked) plus 1 GB. Exit 1 when a file fails its sha256 check. --tarball installs a local player tarball (crane deploy runs it on the host).

| Param | Type | Description |
|---|---|---|
| `--version` | str | That release (default the latest); a checkout fast-forwards to its tag vX.Y.Z. |
| `--player-only` | bool | Only the player. |
| `--no-player` | bool | Only the code. |
| `--tarball` | path | Install this local player tarball instead of the release's. |
| `--sha256` | str | --tarball's digest (default its .sha256 file). |
| `--store` | path | The player store (default ~/.local/share/crane/player). |
| `--dry-run` | bool | Print the steps; change nothing. |
| `--json` | bool | One JSON document. |

raw: `git fetch && git merge --ff-only @{u}; curl -LO RELEASE/ASSET; sha256sum -c`; --json: `crane.update/1`

## heavy

The machine-wide concurrency guard.

### crane heavy

```sh
crane heavy contain test python3 -m pytest fastsim/tests -q
```

The machine-wide concurrency guard for Unity, sims and test suites.

What you'll see: heavy's own output: it waits for a slot (bounded, exit 75 on timeout), runs CMD and releases it. `crane heavy status` lists the slots, the queue and ETAs.

raw: `bin/heavy`

## doctor

Check this machine.

### crane doctor

```sh
crane doctor
```

Check this machine (python, extras, the T2 player and its flags, heavy, astro_dock, GPU) and print the next commands.

What you'll see: a table of checks (ok / off / missing / warn / FAIL with details), then next:. --json: {schema, ok, checks[{name, state, detail}], next[]}.

Read-only; safe any time. Exit 1 only when the core (python >= 3.10, numpy, pyyaml) is broken.

| Param | Type | Description |
|---|---|---|
| `--json` | bool | One JSON document. |
| `--quiet` | bool | Only results and failures. |

--json: `crane.doctor/1`

## setup

The setup wizard.

### crane setup

```sh
crane setup
crane setup --yes --components core,fast,player --domains boat
crane setup --yes --dry-run
```

The setup wizard: detect this machine, choose components and domains, show sizes and times, install them (install.sh, the release player), then run crane doctor.

What you'll see: the detected machine (OS, cores, GPU and driver, docker, astro_dock, Unity editor, display), the component and domain choices with sizes and time estimates, one line per install step, then crane doctor. --dry-run stops after the plan.

Interactive with rich and questionary (the [setup] extra), else plain prompts; --yes takes the defaults or --components/--domains and asks nothing. Components: core, fast, gpu, view, field, data (pip extras, through install.sh), player (the release player, crane update --player-only), astro (a read-only check of the astro integration). The choices are saved to ~/.config/crane/setup.json. Never builds astro images or workspaces (astro owns those).

| Param | Type | Description |
|---|---|---|
| `--yes` | bool | Ask nothing: the defaults, or the lists given. |
| `--components` | str | Comma list (default core, plus fast on a multicore box, gpu with an NVIDIA driver, player with a display or astro_dock). |
| `--domains` | str | Comma list of config/domains names (default all). |
| `--dry-run` | bool | Detect and plan; install nothing. |
| `--json` | bool | One JSON document (implies --yes). |

raw: `./install.sh --yes --components core,fast; crane update --player-only`; --json: `crane.setup/1`

## help

The commands, and the help.yaml / Docs/CLI.md check.

### crane help

```sh
crane help sim
crane help --check
```

The nouns and verbs; help NOUN for one noun; --check fails when help.yaml or Docs/CLI.md drift from the registry, --write regenerates them.

What you'll see: one line per command; with NOUN, its verbs with examples.

| Param | Type | Description |
|---|---|---|
| `topic` | list | A noun (and verb). |
| `--check` | bool | Regenerate into a temp dir; exit 1 when the committed files differ. |
| `--write` | bool | Regenerate help.yaml and Docs/CLI.md in place. |
| `--json` | bool | The registry as one JSON document. |
