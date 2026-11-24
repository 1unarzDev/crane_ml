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
crane study init --domain boat > study.yaml
```

Write a study spec (crane.study/1) to start from.

Not yet available; today use `a hypothesis.yaml beside your runs`.

### crane study plan

```sh
crane study plan study.yaml
```

Print the jobs a study spec compiles to (local batch or rc-job/1).

Not yet available; today use `crane run --domain D --seeds A:B --dry-run`.

### crane study run

```sh
crane study run study.yaml
```

Run a study: resume, progress, held-out guard.

Not yet available; today use `crane run --domain D --tier T --seeds A:B per arm`.

### crane study status

```sh
crane study status study.yaml
```

A study's progress per cell.

Not yet available; today use `crane heavy status, or crane cluster status`.

### crane study report

```sh
crane study report study.yaml
```

Paired analysis per cell (bootstrap, sign-flip, Holm) through judge.stats.

Not yet available; today use `crane compare A B`.

### crane study promote

```sh
crane study promote study.yaml --top 5
```

Rerun a stage's top-k at a higher tier.

Not yet available; today use `crane run --tier T1 on the chosen seeds`.

## runs

The local run index: list, show, query, diff.

### crane runs ls

```sh
crane runs ls --domain boat
```

List local runs, newest first.

Not yet available; today use `crane board ~/.cache/crane/runs`.

### crane runs show

```sh
crane runs show RUN_ID
```

One run's header, metrics and files.

Not yet available; today use `cat RUN/run.json`.

### crane runs query

```sh
crane runs query --where tier=T1
```

Runs matching --where.

Not yet available; today use `crane board ~/.cache/crane/runs`.

### crane runs diff

```sh
crane runs diff RUN_A RUN_B
```

Two runs' configs and metrics side by side.

Not yet available; today use `crane compare A B`.

### crane runs index

```sh
crane runs index
```

Rebuild the local run index.

Not yet available; today use `crane board ~/.cache/crane/runs`.

## repro

Rerun one run and check it reproduces.

### crane repro

```sh
crane repro RUN_ID --check
```

Rerun one run from its record and check it reproduces.

Not yet available; today use `crane run with the run.json seed, config and overrides, then crane compare`.

## data

Datasets from runs: export, schema, splits.

### crane data export

```sh
crane data export ~/.cache/crane/runs/toy/T0 --format npz --out /tmp/ds
crane data export /tmp/runs --select seed,outcome,obs --where 'seed>=10' --out /tmp/ds2
crane data export /tmp/ds --split /tmp/ds/splits.json:train --out /tmp/train
```

Run dirs, batch outputs and datasets to one columnar dataset (npz, parquet, csv or jsonl) plus manifest.json with a sha256 per file; --select, --where and --split pick the rows.

What you'll see: one line (rows, columns, files, MB, format), one row per file (path, rows, sha256), the manifest path, a note for held-out rows or non-finite floats, next:. --json: {schema, out, manifest, rows, columns, files, held_count, nonfinite}.

A run dir gives one metrics-first row (seed, outcome, condition.*, metrics, commit, run_sha256) and its recorded arrays as obs / traj columns (record: in the domain YAML, Docs/Data.md). Array columns are stacked to rows x T x channels (shorter rows NaN-padded, COL@len their lengths); with csv or parquet they go to PART.arrays.npz beside each file. Files roll over at 90 MB. Exit 0 written, 2 usage (no rows, an unknown column, a non-empty --out).

| Param | Type | Description |
|---|---|---|
| `sources` | list | Run dirs, batch outputs, datasets (dirs with manifest.json) or row files (.parquet .csv .npz .jsonl). |
| `--format` | str | The file format (parquet needs pyarrow: the [data] extra). One of `npz`, `parquet`, `csv`, `jsonl`. (default `npz`) |
| `--select` | str | Columns to keep, comma-separated (default all). |
| `--where` | str | Keep rows where COLUMN OP VALUE holds; OP is = != > >= < <=; = and != take a,b (repeatable, all must hold). |
| `--split` | str | Keep the rows of one split of a crane data splits file. |
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
crane monitor --follow
```

Progress of local and cluster work in one view (progress.json and the cluster feed).

Not yet available; today use `crane heavy status, and crane cluster status`.

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
crane bench fast --domain boat --tier T1 --seeds 0:50
```

The T0/T1 throughput table.

Not yet available; today use `python3 -m bench.fast --domain boat --tier T1 --seeds 0:50 --out DIR`.

### crane bench cluster

```sh
crane bench cluster --domain boat
```

Throughput and cost on the cluster.

Not yet available; today use `python3 -m bench.fast with its --fresh-jobs, on a cluster host`.

## cluster

Research-cluster jobs.

### crane cluster

```sh
crane cluster status
```

Research-cluster jobs: submit, wait, status, fetch (the research CLI).

What you'll see: the research CLI's output; arguments go to it unchanged. Found at $RESEARCH_CLUSTER/bin/research or ~/research-cluster/bin/research.

raw: `research`

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
crane build images core
```

The crane docker images (core, gpu, sim), pinned by commit.

Not yet available; today use `docker build on the Dockerfiles`.

### crane build wheel

```sh
crane build wheel
```

The crane wheel and sdist.

Not yet available; today use `python3 -m build`.

## deploy

Install crane and the player on a host.

### crane deploy

```sh
crane deploy HOST
```

Install crane (a venv) and the player on a host, then run crane doctor there.

Not yet available; today use `rsync the checkout, then ./install.sh --yes on the host`.

## update

Update this checkout from a release.

### crane update

```sh
crane update
```

Update this checkout from a release.

Not yet available; today use `git pull, then ./install.sh --yes`.

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
```

The setup wizard.

Not yet available; today use `./install.sh [--yes] [--components core,fast,gpu,view,field,setup]`.

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
