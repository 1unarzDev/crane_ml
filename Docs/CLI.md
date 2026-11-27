# crane CLI

```sh
crane doctor                                            check this computer; says what to run next
crane sim up                                            start the 3-D simulator and wait until it is ready
crane run --domain toy --seeds 0:3                      simulate seeds 0, 1 and 2 in the light simulator
crane run --domain boat --tier T1 --seeds 0:2           the boat's course with physics, seeds 0 and 1
crane view --live --domain boat --tier T1 --seed 3      try settings live with sliders in a window
crane help                                              every command; crane NOUN VERB --help for one
```

Generated from `crane/commands.py` by `crane help --write`; `crane help --check` fails when it drifts. `crane NOUN VERB --help` prints the same entry.

## Conventions

`crane NOUN [VERB] [selectors] [flags]`. Output is plain aligned text; a failure is one line, `crane NOUN VERB: cause; fix: CMD`; output ends with `next: CMD` where there is a next step.

| Selector | Meaning |
|---|---|
| `--domain` | Which kind of robot world, e.g. boat; one file each in config/domains. |
| `--tier` | How detailed: T0 abstract (no physics), T1 dynamic (physics) or T2 unity (the 3-D simulator). |
| `--env KIND:NAME` | Which world, written KIND:NAME, e.g. course:roboboat_2026. |
| `--robot` | Which robot, e.g. blastoise (default the domain's). |
| `--card REF` | The robot's physics numbers: a card name or a profile file, e.g. characterized. |
| `--seed` | One seed, e.g. 3. |
| `--seeds SPEC` | Which seeds, e.g. 0:10 (0 to 9), 5, 1,4,9, @FILE (a list in a file) or block:NAME (a named group). |
| `--set KEY=VALUE` | Change one setting, KEY=VALUE, e.g. time_limit_s=60; dots reach inside, e.g. a.b=1 (repeatable). |
| `--out` | Where to save the output (default ~/.cache/crane/runs/DOMAIN/TIER/...). |

| Global flag | Meaning |
|---|---|
| `--json` | Print the result as one JSON document instead of text, for scripts. |
| `--quiet` | Print only the results and any failures, not the progress lines. |
| `--dry-run` | Show what would happen, step by step, without doing any of it. |

Exit codes: 0 ok; 1 a check or run failed (judge FAIL included); 2 usage or config error; 75 a bounded wait (heavy, cluster) timed out.

## sim

The live simulator the robot software drives: the full 3-D world (T2) or its light twin (T1).

### crane sim up

```sh
crane sim up
crane sim up roboboat-high
crane sim up roboboat-twin
crane sim up --fidelity high --window
crane sim up --stack bridge
crane sim up --current 0
crane sim up --headless --seed 3
crane sim up --card characterized --headless
crane sim up --tier T1
crane sim up --tier T1 --odom truth --rate 2
crane sim up --tier T1 --world pool
crane sim up --tier T1 --world pool --nav preset:=pool
crane sim up --tier T1 --instance 2
crane sim up --dry-run
```

Start the simulator and wait until the course is ready to drive.

What you'll see: a line as each part starts, then a status block (what is running, where the logs are) ending with next:.

Name the sim to start, e.g. crane sim up roboboat-high (crane sim list shows them); with none it starts this machine's default (crane sim use SIM). If the same sim is already up it is reused at once; a different one is refused (--fresh replaces it).

| Param | Type | Description |
|---|---|---|
| `sim` | str | Which sim to start, e.g. roboboat-high (default auto: this machine's, from crane sim use; else the domain defaults). (default `auto`) |
| `--fidelity` | str | How detailed the sim is: twin (light, no 3-D world), low, high or eval (default auto: the sim's, else low). (default `auto`) |
| `--stack` | str | The software beside the world: astro (the robot's own), bridge (crane's ROS link, bring your own nodes) or none (default auto: astro when astro_dock is here, else bridge). (default `auto`) |
| `--fresh` | bool | Restart the sim even if the same one is already up (default off). |
| `--seed` | int | Which course layout to build; the same number always gives the same buoys, e.g. 3 (default 1). (default `1`) |
| `--env` | env | Which world to load, written KIND:NAME, e.g. course:roboboat_2026 (the default); --course NAME is a shortcut. (default `course:roboboat_2026`) |
| `--card` | path | The boat's physics settings: a fitted profile file or a card name, e.g. characterized (default none: the built-in boat). (default `none`) |
| `--profile` | str | Expert: the player's quality preset, e.g. evaluation-high (default auto: the fidelity's). (default `auto`) |
| `--current` | str | The water current: default (the course's own, 0.15 metres per second toward 270 degrees), 0 for still water, or SPEED,TOWARD_DEG, e.g. 0.3,90. (default `default`) |
| `--display` | str | auto opens a window when you have a screen and runs without one otherwise; --window and --headless are shortcuts. One of `auto`, `window`, `headless`. (default `auto`) |
| `--player` | path | The simulator program to run, e.g. Builds/CRANE-Worker/CRANE.x86_64 (default auto: the newest one found). |
| `--tier` | str | Expert: T2 is the full 3-D simulator, T1 the light twin, T0 runs only through crane run (default auto: the fidelity's). One of `T2`, `T1`, `T0`. (default `auto`) |
| `--odom` | str | T1 only: sensors feeds the robot's own position estimate like on the water (default); truth hands it perfect positions instead. One of `sensors`, `truth`. (default `sensors`) |
| `--rate` | float | T1 only: how many times faster than real time to run, e.g. 2 (default 1). (default `1`) |
| `--world` | str | T1 only: course (the default) or a rectangular pool whose walls the lidar sees, pool for 10 by 4.5 metres or pool:LxW for another size, e.g. pool:9x5; the boat starts 1.5 metres from one end, facing along it. --env pool:LxW does the same. (default `course`) |
| `--robot` | str | Which robot to put in the world, e.g. blastoise (the default). (default `blastoise`) |
| `--set` | str | Pin one course setting as TASK_ID.PARAM=VALUE, e.g. a gate spacing in metres; give it again for more (default none). (default `none`) |
| `--domain` | str | Which kind of robot world to use, e.g. boat (the default) or sub. (default `boat`) |
| `--nav` | str | Hand the navigation part a launch argument, e.g. preset:=pool; give it again for more. A restart of nav reuses them (default none). (default `none`) |
| `--nav-args` | str | Several --nav at once in one quoted text, e.g. "preset:=pool params_file:=/path/variant.yaml"; a path must be under the workspace, ~/.config/astro or ~/.config/mhseals so the robot container sees it (default none). (default `none`) |
| `--instance` | int | Which parallel world to use, 0 for the default world, 1 and up, or auto for the first free one, e.g. 2: its own ROS_DOMAIN_ID (the usual one plus the number), port, container and state folder, so several T1 worlds can run side by side (default none: the one world; the environment variable CRANE_SIM_INSTANCE sets it too). (default `none`) |
| `--dry-run` | bool | Show exactly what would start, without starting anything (default off). |
| `--json` | bool | Print the final status as one JSON document for scripts; progress goes to stderr. |

raw: `Tools/RoboBoat/sim_up up`; --json: `crane.sim.status/1`

Details: Starts the Unity player through heavy unity plus its ROS side (the sim_up_ros container), then prints the status block (mode, ROS_DOMAIN_ID, endpoint, player pgid, ROS container, course, current, logs, next:). Waits up to 60 s for the ROS endpoint and 300 s (SIM_UP_READY_S) for the course. --tier T1 (tiers.T1.sim, Docs/SimT1.md): no Unity; a ROS node steps the plant-card twin and publishes what the player does (/crane/odom, /imu/raw, /gps/fix, /clock; /odom/mavros with --odom sensors; /odom/local, /odom/global and TF with --odom truth, which turns localization off) and drives on /thrusters/pwm. --tier T0 refuses naming crane run. --dry-run prints the resolved player argv and docker run (crane.sim.plan/1 with --json). Exit 0 ready, 1 the world failed, 2 usage, config or a sim already running. Everything specific comes from config/domains/DOMAIN.yaml tiers.T2 and the stack (stacks.NAME, config/stacks). A sim (config/sims/NAME.yaml, ~/.config/crane/sims) names domain, course and fidelity; this machine's ~/.config/crane/crane.yaml sim block names its default sim, display, player and stack; flags win (crane sim show explains each value). The ROS_DOMAIN_ID is the sim's own (101 plus the instance), never a robot vessel's. A script never takes over a sim a person started: its T1 world goes to the next instance, its T2 world is refused. Env: CRANE_SIM, ROS_DOMAIN_ID (only with ASTRO_VESSEL=sim), ASTRO_STATE / MHSEALS_HARDWARE_CONFIG (passed into the ROS side), CRANE_SIM_OWNER, the image env, SIM_UP_*, CRANE_PLAYER, HEAVY. Same as Tools/RoboBoat/sim_up up.

### crane sim down

```sh
crane sim down
crane sim down --instance 2
```

Stop the simulator that crane sim up started.

What you'll see: one line saying what was stopped.

Only stops what crane sim up started; other programs are left alone. Safe to run when nothing is running.

| Param | Type | Description |
|---|---|---|
| `--instance` | int | Which parallel world to use, 0 for the default world, 1 and up, or auto for the first free one, e.g. 2: its own ROS_DOMAIN_ID (the usual one plus the number), port, container and state folder, so several T1 worlds can run side by side (default none: the one world; the environment variable CRANE_SIM_INSTANCE sets it too). (default `none`) |
| `--force` | bool | Stop it even when a person started it and this is a script (default off). |
| `--json` | bool | Print what was stopped as one JSON document, for scripts. |

raw: `Tools/RoboBoat/sim_up down`; --json: `crane.sim.down/1`

Details: Stops the player's process group (TERM, then KILL after 10 s) and the sim_up_ros container. Same as Tools/RoboBoat/sim_up down.

### crane sim restart

```sh
crane sim restart nav
crane sim restart nav preset:=pool
crane sim restart player --json
```

Restart one part of the running simulator (navigation, ROS or the 3-D world) and keep the rest, e.g. after changing navigation settings.

What you'll see: one line per step, then `crane sim: PART restarted, ready: ...` and next:.

Use nav after editing nav settings: only the navigation restarts, so the course and boat stay where they are. Any goal the boat was driving to is lost.

| Param | Type | Description |
|---|---|---|
| `what` | str | Which part to restart: nav (navigation only), ros (all the robot software) or player (the 3-D world). One of `nav`, `ros`, `player`. |
| `words` | list | Only with nav: launch arguments for the navigation part, e.g. preset:=pool. They replace the ones from the last start; with none, those are used again (default none). (default `none`) |
| `--nav-args` | str | The same words in one quoted text, e.g. "preset:=pool params_file:=/path/variant.yaml"; a path must be under the workspace, ~/.config/astro or ~/.config/mhseals (default none). (default `none`) |
| `--instance` | int | Which parallel world to use, 0 for the default world, 1 and up, or auto for the first free one, e.g. 2: its own ROS_DOMAIN_ID (the usual one plus the number), port, container and state folder, so several T1 worlds can run side by side (default none: the one world; the environment variable CRANE_SIM_INSTANCE sets it too). (default `none`) |
| `--json` | bool | Print the result as one JSON document instead of text, for scripts. |
| `--domain` | str | Which kind of robot world is running, e.g. boat (the default). |

raw: `docker exec CONTAINER pkill ...; docker exec -d CONTAINER ros2 launch ...`; --json: `crane.sim.restart/1`

Details: T1: ros restarts the twin node with the container (from the course start); player refuses. nav stops the nav nodes named by the domain recipe (tiers.T2.ros.restart.nav) inside the ROS container, starts them again from the same launch (only:=navigation), and waits up to the recipe's ready_s (90 s) for bt_navigator to be active; a domain without the recipe refuses and names `restart ros`. ros re-runs the saved docker run, waits up to 60 s for the endpoint and 120 s for the ready nodes. player re-runs the saved player argv and waits up to 300 s (SIM_UP_READY_S) for the course. --json: {schema, restarted, ready, at, status}. Exit 0 ok, 1 failed, 2 usage, config or nothing running, 75 timed out waiting.

### crane sim status

```sh
crane sim status
crane sim status --instance 2
```

Show whether the simulator is running and where its parts and logs are; changes nothing.

What you'll see: the status block (what is running, the course and current, the log files), ending with next:.

| Param | Type | Description |
|---|---|---|
| `--instance` | int | Which parallel world to use, 0 for the default world, 1 and up, or auto for the first free one, e.g. 2: its own ROS_DOMAIN_ID (the usual one plus the number), port, container and state folder, so several T1 worlds can run side by side (default none: the one world; the environment variable CRANE_SIM_INSTANCE sets it too). (default `none`) |
| `--json` | bool | Print the status as one JSON document instead of text, for scripts. |

raw: `Tools/RoboBoat/sim_up status`; --json: `crane.sim.status/1`

Details: The block: mode, ROS_DOMAIN_ID, endpoint, player, ROS container, course, current, logs, next. --json: {schema, domain, mode, player, ros, course, current, logs, next}. Same as Tools/RoboBoat/sim_up status.

### crane sim check

```sh
crane sim check --wait 120
```

Check the running simulator is ready to drive (world, course, boat, robot software, navigation); can wait until it is.

What you'll see: one row per check (ok or FAIL, with a reason), then `sim check: ready` or what failed.

Ends with `sim check: ready` when everything is up. Right after crane sim up, give it time with --wait 120 rather than checking once.

| Param | Type | Description |
|---|---|---|
| `--wait` | int | Keep checking every 2 seconds until ready, for at most this many seconds, e.g. 120 (default: check once). |
| `--instance` | int | Which parallel world to use, 0 for the default world, 1 and up, or auto for the first free one, e.g. 2: its own ROS_DOMAIN_ID (the usual one plus the number), port, container and state folder, so several T1 worlds can run side by side (default none: the one world; the environment variable CRANE_SIM_INSTANCE sets it too). (default `none`) |
| `--json` | bool | Print the checks as one JSON document instead of text, for scripts. |
| `--domain` | str | Which kind of robot world is running, e.g. boat (the default). |

raw: `Tools/RoboBoat/sim_up check`; --json: `crane.sim.check/1`

Details: Exit 1 when not ready, 75 when --wait SECONDS pass first. --json: {schema, ready, exit, failed, notices, checks[{name, ok, detail}]}. `crane sim check CHECK INPUTS` runs an offline Tools/SimCheck check on recorded results instead. Same as Tools/RoboBoat/sim_up check.

### crane sim logs

```sh
crane sim logs ros -n 100
```

Show the last lines of the simulator's logs, to see why something went wrong.

What you'll see: the last lines of each log, under its file path.

| Param | Type | Description |
|---|---|---|
| `which` | list | Which logs: player, stdout, ros or t1 (default the running tier's). One of `player`, `stdout`, `ros`, `t1`. |
| `-n` | int | How many lines of each log to show, e.g. 100 (default 40). (default `40`) |
| `--instance` | int | Which parallel world to use, 0 for the default world, 1 and up, or auto for the first free one, e.g. 2: its own ROS_DOMAIN_ID (the usual one plus the number), port, container and state folder, so several T1 worlds can run side by side (default none: the one world; the environment variable CRANE_SIM_INSTANCE sets it too). (default `none`) |

raw: `Tools/RoboBoat/sim_up logs`

Details: The logs: player (the Unity player log), stdout (its console output) and ros (the ROS container). Same as Tools/RoboBoat/sim_up logs.

### crane sim list

```sh
crane sim list
```

List the sims you can start by name, with this machine's default marked.

What you'll see: one row per sim (* is the default): name, domain, course or world, fidelity, what it is for.

| Param | Type | Description |
|---|---|---|
| `--json` | bool | Print the list as one JSON document, for scripts. |

raw: `ls config/sims ~/.config/crane/sims`; --json: `crane.sim.list/1`

Details: The repo's config/sims/*.yaml and your own ~/.config/crane/sims/*.yaml (yours win by name).

### crane sim show

```sh
crane sim show
crane sim show roboboat-high --seed 3
```

Show what crane sim up would start and where each setting comes from; starts nothing.

What you'll see: one row per setting with its source (flag, this machine, the sim, the domain default).

| Param | Type | Description |
|---|---|---|
| `sim` | str | Which sim, e.g. roboboat-high (default auto: this machine's). (default `auto`) |
| `--json` | bool | Print it as one JSON document, for scripts. |

raw: `crane sim up --dry-run`; --json: `crane.sim.recipe/1`

Details: Takes the same words and options as up. --json: {schema, sim, about, identity, values, sources}; the identity is what up compares to reuse a running sim.

### crane sim use

```sh
crane sim use roboboat-high
crane sim use roboboat-high --display window
```

Choose the sim a bare crane sim up starts on this machine, and how it shows.

What you'll see: one line naming the sim and the file it was saved in.

e.g. crane sim use roboboat-high --display window on a laptop without xvfb; crane sim use none clears it.

| Param | Type | Description |
|---|---|---|
| `sim` | str | The sim a bare up starts, e.g. roboboat-high; none clears it. |
| `--display` | str | How this machine shows the 3-D world (default auto). One of `auto`, `window`, `headless`. (default `auto`) |
| `--stack` | str | This machine's stack: auto, astro, bridge or none. (default `auto`) |
| `--player` | path | This machine's simulator program. |
| `--json` | bool | Print the saved settings as JSON, for scripts. |

raw: `$EDITOR ~/.config/crane/crane.yaml`; --json: `crane.sim.use/1`

Details: Writes the sim block of ~/.config/crane/crane.yaml (CRANE_CONFIG moves it): default, display, player, stack. Other blocks are kept.

### crane sim env

```sh
crane sim env
crane sim env --instance 2
```

Print the lines that point your own ROS tools at the running sim (no astro needed).

What you'll see: export lines: ROS_DOMAIN_ID, ROS_AUTOMATIC_DISCOVERY_RANGE and the endpoint.

Run eval "$(crane sim env)" in a shell, then ros2 topic list sees the sim. With astro, astro on sim does the same.

| Param | Type | Description |
|---|---|---|
| `--instance` | int | Which parallel world to use, 0 for the default world, 1 and up, or auto for the first free one, e.g. 2: its own ROS_DOMAIN_ID (the usual one plus the number), port, container and state folder, so several T1 worlds can run side by side (default none: the one world; the environment variable CRANE_SIM_INSTANCE sets it too). (default `none`) |

raw: `export ROS_DOMAIN_ID=101`

## run

Simulate many seeds quickly in the light simulators (T0, T1) and save each one.

### crane run

```sh
crane run --domain toy --seeds 0:3
crane run --domain boat --tier T1 --seeds 0:2 --out /tmp/runs
crane run --domain sub --tier T1 --engine batched --seeds 0:64
crane run --domain boat --tier T1 --astro-test speed-surge --card /tmp/fit/profile.json
crane run --domain modular --replay timelines.npz --seeds 0:24 --card example#plated
```

Simulate many seeds quickly in a light simulator (T0 or T1) and save each run in its own folder; for the full 3-D simulator use crane sim up.

What you'll see: one line per seed (where it was saved, how it ended, sim and wall time), then runs per second and next:.

You need --domain (or --config) and --seeds, e.g. crane run --domain toy --seeds 0:3. Rerunning the same command reuses finished runs instead of running them again.

| Param | Type | Description |
|---|---|---|
| `--domain` | str | Which kind of robot world, e.g. boat (also sub, drones, modular, toy); one file each in config/domains. |
| `--config` | path | Use this simulator config file as it is, instead of a domain, e.g. my_domain.yaml. |
| `--tier` | tier | How detailed: T0 abstract (no physics) or T1 dynamic (physics) (default the domain's first tier, T0 with --config). One of `T0`, `T1`, `T2`, `abstract`, `dynamic`, `unity`. |
| `--seeds` | str | Which seeds to run, e.g. 0:10 (0 to 9), 5, or 1,4,9 (required, except with --astro-test: 0). |
| `--out` | path | Where to save the runs (default ~/.cache/crane/runs/DOMAIN/TIER/...). |
| `--workers` | int | How many processes to run at once, e.g. 8 (default the config's). |
| `--set` | str | Change one setting, KEY=VALUE, e.g. time_limit_s=60; dots reach inside, e.g. a.b=1 (repeatable). |
| `--variant` | str | A name for this set of settings, used in the folder name, e.g. heavier-hull (default base). (default `base`) |
| `--generate` | bool | Make the scenarios again even if they exist (missing ones are always made). |
| `--allow-dirty` | bool | Run even though the code has uncommitted changes. |
| `--force` | bool | Rerun a seed whose folder holds another or an unfinished run (the old one is kept as <dir>.stale-N). |
| `--engine` | str | fastsim runs one seed at a time; batched runs every seed at once, much faster for many seeds. One of `fastsim`, `batched`. (default `fastsim`) |
| `--backend` | str | With --engine batched: numpy (slow, exact reference), numba (every CPU core) or jax (GPU). One of `numpy`, `numba`, `jax`. (default `numba`) |
| `--dtype` | str | With --engine batched: number precision (default float64; use float32 for jax on a gaming GPU). One of `float64`, `float32`. |
| `--record` | int | With --engine batched: also save the track, keeping every K-th step (default 0: results only). |
| `--replay` | path | Replay recorded command timelines from an npz file (modular); every seed replays every timeline. |
| `--robot` | str | Which robot, e.g. mecanum-pair for modular (default the domain's). |
| `--env` | env | Which world, e.g. route:random for modular. |
| `--astro-test` | str | Run one of astro's characterization tests in the sim, e.g. speed-surge or all, with results named as astro names them. |
| `--card` | str | The robot's physics numbers: a profile file or a card name from the domain's card folder, e.g. /tmp/fit/profile.json (repeatable). |
| `--tests` | path | With --astro-test: astro's tests file, or a bundle holding one. |
| `--space` | str | With --astro-test: which size of test space, e.g. large (the default). |
| `--amp` | float | With --astro-test: how hard to push, as an effort fraction, e.g. 0.5 (default the test's). |
| `--json` | bool | Print the result as one JSON document instead of text, for scripts (default off). |
| `--dry-run` | bool | Show what would run; run nothing. |
| `--quiet` | bool | Print only the results and any failures, not the progress lines. |

raw: `python3 -m fastsim.batch --config CONFIG --seeds 0:3 --out OUT`; --json: `crane.run.summary/1`

Details: Runs go to OUT/VARIANT/s<seed> (default ~/.cache/crane/runs/DOMAIN/TIER/KEY). Missing scenarios are generated on demand. T2 is not available here (crane sim up). A run dir holding another run, or an incomplete one, is refused unless --force. --dry-run prints the resolved plan; --json prints one crane.run.summary/1 document. --engine batched runs every seed at once (Docs/Engine.md); jax on a consumer GPU wants --dtype float32 (bench/RESULTS.md). modular: the kernel is a port of the capplan truth stepper (FP0 module), not corpus-identified; corpus validation pending.

## study

Plan, run and analyse an experiment: several settings over many seeds, compared fairly.

### crane study init

```sh
crane study init --out study.yaml
crane study init --domain boat > boat_study.yaml
```

Write a starter study file, with comments, for you to edit.

What you'll see: the study file on screen, or `wrote FILE` and next:.

| Param | Type | Description |
|---|---|---|
| `--domain` | str | Which kind of robot world its runs use, e.g. boat (default toy). |
| `--out` | path | Save it to this file instead of printing it. |
| `--force` | bool | Overwrite FILE if it exists. |
| `--json` | bool | Print the result as one JSON document instead of text, for scripts. |

--json: `crane.study.init/1`

Details: The file is a crane.study/1 spec: arms (the settings to compare), axes, seed blocks, stages.

### crane study plan

```sh
crane study plan examples/studies/toy_paired.yaml
crane study plan examples/studies/teamgame_like.yaml
crane study plan examples/studies/boat_cards.yaml --json
```

Show how big a study is and roughly how long it will take, before running anything.

What you'll see: one row per stage (how many runs, how many already done, how many jobs, estimated time), the seed groups, then next:.

Nothing runs, so it is safe to try. A mistake in the study file is one line naming the setting and the fix.

| Param | Type | Description |
|---|---|---|
| `spec` | path | The study file (YAML). |
| `--stage` | str | Only this stage, e.g. A. |
| `--block` | str | Only this group of seeds, e.g. dev. |
| `--out` | path | Where the study's results are, to count what is done (default ~/.cache/crane/studies/NAME). |
| `--json` | bool | Print the plan, with every job, as one JSON document. |

--json: `crane.study.plan/1`

Details: Rows: tiers, cells, units, done, held-out units, jobs, est, per job; plus design points, seed blocks, splits and pending adaptive stages. --json adds jobs[] (job_id, cell, seeds, est_seconds, kind, campaign, purpose, expect, pin, argv): the cluster compiler's input. Estimates use pack.unit_s; jobs pack a cell's seeds to about pack.target_s (default 900 s) each. A spec error exits 2.

### crane study run

```sh
crane study run examples/studies/toy_paired.yaml --max-workers 4
crane study run examples/studies/toy_paired.yaml --block test --confirm
crane study run examples/studies/adaptive.yaml --stage V
```

Run a study on this computer, picking up where it stopped.

What you'll see: a progress line each minute, then one row per stage (runs done, reused, failed), and where the results are, then next:.

Rerunning the same command only runs what is missing. Held-out seeds (kept back for the final test) only run with --block NAME --confirm, once the targets file is committed.

| Param | Type | Description |
|---|---|---|
| `spec` | path | The study file (YAML). |
| `--stage` | str | Only this stage, e.g. V (the stage it reads from must be done). |
| `--block` | str | Only this group of seeds, e.g. dev (a held-out one needs --confirm). |
| `--where` | str | local runs here; for the cluster use crane cluster submit. One of `local`, `cluster`. (default `local`) |
| `--confirm` | bool | Allow the held-out seeds to run (needs the committed targets file). |
| `--max-workers` | int | How many processes to run at once, e.g. 4. (default `1`) |
| `--out` | path | Where to save results (default ~/.cache/crane/studies/NAME). |
| `--json` | bool | Print the summary as one JSON document; progress lines go to stderr. |
| `--dry-run` | bool | Show the plan (like study plan); run nothing. |
| `--quiet` | bool | Print only the results and any failures, not the progress lines. |

--json: `crane.study.run/1`

Details: Writes OUT/progress.json at least every 60 s (CRANE_RUN_DIR overrides where). Units are cached in OUT/cache/records.jsonl on (study, stage, arm, cell hash, seed, card hash, code commit) within 1 s of finishing. A held-out block refuses (exit 2) without --confirm, or when the spec's targets file is not committed, has local changes or is not older than now. --where cluster: use crane study compile, then research submit (or crane cluster submit). Exit 0 ok, 1 units failed, 2 spec, usage or refusal.

### crane study status

```sh
crane study status examples/studies/toy_paired.yaml
crane study status ~/.cache/crane/studies/toy_paired
```

Show how far a study has got, any failures, and early results on the practice seeds.

What you'll see: one row per stage and seed group (runs done, percent), the latest failure with its error, early averages per setting, then next:.

Held-out seeds show progress only, never results, until they finish. Exit 1 means some runs failed; the error is shown.

| Param | Type | Description |
|---|---|---|
| `target` | path | The study file, or its results folder. |
| `--out` | path | With a study file: its results folder (default ~/.cache/crane/studies/NAME). |
| `--json` | bool | Print the result as one JSON document instead of text, for scripts (default off). |

--json: `crane.study.status/1`

Details: Rows: stage, block, role, units, done, %; the progress.json line; the latest failure and its traceback tail; interim dev means per arm.

### crane study report

```sh
crane study report ~/.cache/crane/studies/toy_paired
crane study report /tmp/study --metric succeeded --json
```

Analyse a finished study: how much each setting beats or trails the baseline, and whether the difference is real or chance.

What you'll see: a table (setting minus baseline, the difference, its 95% range, the p-value) and where report.md and report.json were saved.

Each setting is compared with the baseline on the same seeds, so luck cancels out. Held-out seeds count only once all of them have finished.

| Param | Type | Description |
|---|---|---|
| `out` | path | The study's results folder. |
| `--baseline` | str | The setting the others are compared with, e.g. base (default the study file's). |
| `--metric` | str | Only these results, e.g. succeeded (repeatable). |
| `--bootstrap` | int | How many resamples for the ranges (more is slower and steadier). (default `2000`) |
| `--json` | bool | Print the report as one JSON document (also saved to OUT/report.json). |

--json: `crane.study.report/1`

Details: Paired on the seed per cell: bootstrap CI, sign-flip permutation, Holm across cells, via judge.stats. Reads OUT/manifest.json and its rows (or every */manifest.json under OUT, as a fetched cluster output). Held-out targets get met, not met or inconclusive: a direction, an interval or threshold (relative too), a gap closed, a share of cells, or a pinned targets_fn.

### crane study promote

```sh
crane study promote examples/studies/adaptive.yaml --stage V
```

Take the best settings from one stage of a study and write them into the next stage, ready to run.

What you'll see: the ranking (which settings were chosen) and the new study file, then next:.

| Param | Type | Description |
|---|---|---|
| `spec` | path | The study file (YAML). |
| `--stage` | str | The stage that chooses from an earlier one, e.g. V. |
| `--out` | path | The results folder holding the earlier stage (default ~/.cache/crane/studies/NAME). |
| `--write` | path | Where to save the new study file (default SPEC.STAGE.yaml beside it). |
| `--json` | bool | Print the result as one JSON document instead of text, for scripts (default off). |

--json: `crane.study.promote/1`

Details: Freezes an adaptive stage's selection (top-k of the stage it reads, its select: key) into a spec whose stage names the chosen arms.

### crane study run-cell

```sh
crane study run-cell study.yaml --cell JOB_ID --out /workspace/out/STUDY/JOB_ID
```

Run one piece of a study into a folder; this is what each cluster job runs, you rarely call it yourself.

What you'll see: one line: ran, reused, failed, rows and the folder. Exit 1 when runs failed, 2 a refusal.

| Param | Type | Description |
|---|---|---|
| `spec` | path | The study file (YAML). |
| `--cell` | str | Which piece, by its job id from study plan --json. |
| `--out` | path | The folder to write into. |
| `--seeds` | str | Only these of its seeds, e.g. 0,1,2. |
| `--confirm` | bool | Allow a held-out piece to run. |
| `--targets-sha256` | str | The checked targets file fingerprint, for machines with no git (crane study compile fills it in). |
| `--json` | bool | Print the result as one JSON document instead of text, for scripts (default off). |

--json: `crane.study.run_cell/1`

Details: Writes rows, manifest.json with sha256, metrics.json and progress.json. JOB_ID is a jobs[].job_id of crane study plan --json (CELL_ID, or CELL_ID.jK for one seed chunk). Held-out cells need --confirm and the committed targets file, as study run, or on a box with no git checkout --targets-sha256 from the compiled job. metrics.json holds at most 1000 finite numbers; for held-out cells only unit counts.

### crane study compile

```sh
crane study compile examples/studies/toy_paired.yaml --write /tmp/jobs.json
crane study compile examples/studies/toy_paired.yaml --block test --confirm --json
```

Turn a study into the list of cluster jobs, without sending them; crane cluster submit does this for you.

What you'll see: the job list on screen, or `N jobs in FILE` and next: research submit.

The study needs a purpose: line and a time estimate per run. Jobs run the exact committed code, so commit first.

| Param | Type | Description |
|---|---|---|
| `spec` | path | The study file (YAML). |
| `--stage` | str | Only this stage, e.g. A. |
| `--block` | str | Only this group of seeds, e.g. dev. |
| `--confirm` | bool | Include the held-out seeds (needs the committed targets file). |
| `--allow-dirty` | bool | Go ahead with uncommitted changes (the jobs still run the last commit). |
| `--write` | path | Save the job list to this file instead of printing it. |
| `--json` | bool | Print the job list as one JSON document. |

--json: `crane.study.compile/1`

Details: rc-job/1 bodies for the research cluster: one crane_batch or gpu_command job per cell seed chunk (5-30 min), each running crane study run-cell through crane-batch. Needs purpose (20-600 chars, no local path) and pack.unit_s. Jobs pin crane_ml (and the spec's repo) at the exact commit, carry dedup ids from the unit keys and the commit, needs from pin:, campaign and expect; units in the study's unit cache are left out; field data is refused. Held-out jobs are left out without --confirm; with it the targets file must be committed, clean and older than now, and each held-out job pins its sha256, which crane-batch passes to run-cell as --targets-sha256. Exit 2 on a refusal.

## runs

Find and inspect runs you already have: list, search, show one, compare two.

### crane runs ls

```sh
crane runs ls --domain boat --tier T1
crane runs ls --study nlc08 --metric time --limit 20
```

List the runs you have, newest first, with how each one ended.

What you'll see: a table (run id, domain, tier, seed, outcome, setting, date, any results you asked for, folder), then counts and next:.

The first time, it finds runs under ~/.cache/crane/runs by itself; add other folders with crane runs index. Held-out runs that are not finished show `sealed` instead of results.

| Param | Type | Description |
|---|---|---|
| `--domain` | str | Only runs of this kind of world, e.g. boat. |
| `--tier` | tier | Only runs at this level of detail, e.g. T1. One of `T0`, `T1`, `T2`, `abstract`, `dynamic`, `unity`. |
| `--study` | str | Only runs of this study, e.g. boat01. |
| `--where` | str | Only runs matching a condition, e.g. 'seed>10' (as crane runs query). |
| `--metric` | str | Add a column for this result, e.g. time (repeatable). |
| `--limit` | int | Show at most this many rows (0 for all). (default `50`) |
| `--cluster` | bool | Also list the cluster's runs of --study. |
| `--json` | bool | Print the result as one JSON document instead of text, for scripts (default off). |

raw: `sqlite3 ~/.cache/crane/index.sqlite "SELECT * FROM runs"`; --json: `crane.runs.list/1`

Details: The index is ~/.cache/crane/index.sqlite (CRANE_INDEX moves it). --cluster adds the coordinator's runs of --study (GET /v1/runs/query, read-only; CRANE_COORDINATOR, and CRANE_VIEWER_TOKEN or CRANE_TOKEN); offline it says so on stderr and lists the local runs. A sealed run stores no outcome or metrics, and stderr says how many. --json: {schema, index, n, matched, indexed, runs}.

### crane runs query

```sh
crane runs query --where "outcome=TIMEOUT and tier=T1 and metric.time>120"
crane runs query --where "arm in (a, b) and not dirty=true" --metric min_clearance_m
```

Find runs that match a condition, e.g. every T1 run that timed out.

What you'll see: the same table as crane runs ls, for the matching runs.

Write conditions like "outcome=TIMEOUT and metric.time>120". A typo is one line naming the column and showing an example.

| Param | Type | Description |
|---|---|---|
| `--where` | str | The condition, e.g. "tier=T1 and outcome=TIMEOUT". |
| `--metric` | str | Add a column for this result, e.g. min_clearance_m (repeatable). |
| `--limit` | int | Show at most this many rows (0 for all). (default `50`) |
| `--study` | str | With --cluster: which study to ask the cluster about. |
| `--cluster` | bool | Also search the cluster's runs of --study. |
| `--json` | bool | Print the result as one JSON document instead of text, for scripts (default off). |

--json: `crane.runs.list/1`

Details: FIELD OP VALUE joined by and / or / not and parentheses; FIELD is a column or metric.NAME. OP is = != < <= > >= or ~ (glob; a value without * or ? matches anywhere) or in (A, B). Columns: run_id, path, source, domain, tier, kind, seed, card, backend, dtype, commit, dirty, outcome, created, study, stage, arm, block, run_sha256, sealed. A sealed (held-out, not done) run has no outcome or metrics, so metric terms never match it. `null` tests for a missing value; commit= and run_id= take a hex prefix. Parsed to SQL with bound parameters, never evaluated. A bad expression exits 2.

### crane runs show

```sh
crane runs show 3fa2c1d0
crane runs show ~/.cache/crane/runs/toy/T0/base-1a2b3c4d/base/s3
```

Show everything about one run: its settings, results and files.

What you'll see: the settings (domain, tier, seed, card, code version, outcome), the results, the files with sizes, then next: crane view RUN.

| Param | Type | Description |
|---|---|---|
| `run` | str | A run folder, or a run id from crane runs ls (the first few characters will do). |
| `--json` | bool | Print the result as one JSON document instead of text, for scripts (default off). |

--json: `crane.runs.show/1`

Details: Meta also shows the study fields and identity hashes. --json: {schema, run, meta, files, next}.

### crane runs diff

```sh
crane runs diff 3fa2c1d0 77b0e9aa
```

Compare two runs: which settings differ and how the results changed.

What you'll see: `identity: the same run`, or what differs (settings old -> new, scenario, seed, code version), then each result that changed with A, B and B-A.

| Param | Type | Description |
|---|---|---|
| `a` | str | The first run: a folder or a run id. |
| `b` | str | The second run: a folder or a run id. |
| `--json` | bool | Print the result as one JSON document instead of text, for scripts (default off). |

--json: `crane.runs.diff/1`

Details: Identity parts: config, scenario, adapter, seed, core commit. --json: {schema, same_run, identity, meta, metrics}.

### crane runs index

```sh
crane runs index ~/.cache/crane/runs /tmp/study-out
crane runs index --rebuild
```

Teach crane about runs in other folders, so crane runs ls and query find them.

What you'll see: each folder, then how many runs were added, changed, unchanged or removed.

Only new or changed files are read again, so it is quick to rerun. Runs whose files are gone drop out of the list.

| Param | Type | Description |
|---|---|---|
| `roots` | list | Folders to look in, e.g. /tmp/study-out (default the ones used before). |
| `--rebuild` | bool | Forget everything and read all folders again. |
| `--json` | bool | Print the counts as one JSON document. |

--json: `crane.runs.index/1`

Details: Incremental by mtime, then sha256. Default roots: the ones indexed before, else ~/.cache/crane/runs. Reads crane.run/1 dirs, fastsim run dirs (their crane.run/1 meta, nothing written) and study outputs (manifest.json crane.data.manifest/1 with metrics.json). A remembered root that is gone is dropped. Hidden and *.stale-N dirs are skipped. Held-out records stay sealed until done (the record says complete, or every progress.json up to the root is phase done) and are reread each time.

## repro

Rerun an old run and check it comes out the same.

### crane repro

```sh
crane repro 3fa2c1d0 --check
crane repro ~/.cache/crane/runs/toy/T0/base-1a2b3c4d/base/s3
```

Rerun an old run with the exact code it was made with, and check the result is the same.

What you'll see: for the fingerprint, the files and the results: same or differs; then `repro: reproduced`, or where the rerun was kept so you can crane runs diff it.

Your own checkout is never touched. It refuses runs made from uncommitted code, since that code cannot be recovered.

| Param | Type | Description |
|---|---|---|
| `run` | str | A run folder, or a run id from crane runs ls. |
| `--check` | bool | Exit with 1 when the rerun differs (for scripts). |
| `--timeout` | int | Give up after this many seconds. (default `1800`) |
| `--json` | bool | Print the result as one JSON document instead of text, for scripts (default off). |

raw: `git worktree add --detach TMP COMMIT, then fastsim.run.simulate on the recorded inputs`; --json: `crane.repro/1`

Details: Reruns fastsim runs (T0/T1) in a temporary git worktree at the recorded commit: that commit's fastsim on the scenario file in the run dir, with the recorded config and provenance; compares run_sha256, tree_sha (or the first differing file) and metrics. Refuses with one line (exit 2) a dirty-tree run (its diff sha when recorded), a commit no repository here has, or a non-fastsim record. The worktree is always removed. With --check, exit 1 when it does not reproduce.

## data

Turn runs into a dataset file for analysis or training, and split it fairly.

### crane data export

```sh
crane data export ~/.cache/crane/runs/toy/T0 --format npz --out /tmp/ds
crane data export /tmp/runs --select seed,outcome,obs --where 'seed>=10' --out /tmp/ds2
crane data export /tmp/ds --split /tmp/ds/splits.json:train --out /tmp/train
```

Turn runs into one dataset file (a table, one row per run) for analysis or training.

What you'll see: one line (rows, columns, files, size), one row per file written, and next:.

Pick rows with --where and columns with --select; the output folder must be new or empty. Held-out runs are refused unless you add --confirm.

| Param | Type | Description |
|---|---|---|
| `sources` | list | Run folders, study results, datasets or row files (.parquet .csv .npz .jsonl). |
| `--format` | str | The file type (parquet needs the [data] extra installed). One of `npz`, `parquet`, `csv`, `jsonl`. (default `npz`) |
| `--select` | str | Only these columns, comma-separated, e.g. seed,outcome (default all). |
| `--where` | str | Only rows where a condition holds, e.g. 'seed>=10'; OP is = != > >= < <= (repeatable, all must hold). |
| `--split` | str | Only the rows of one split from crane data splits, e.g. splits.json:train. |
| `--confirm` | bool | Include held-out rows (refused without it). |
| `--out` | path | The dataset folder to write (new or empty). |
| `--json` | bool | Print the result as one JSON document instead of text, for scripts (default off). |

raw: `python3 -c "from crane.data.export import export; export([SRC], OUT)"`; --json: `crane.data.export/1`

Details: Writes npz, parquet, csv or jsonl plus manifest.json with a sha256 per file. A run dir gives one metrics-first row (seed, outcome, condition.*, metrics, commit, run_sha256) and its recorded arrays as obs / traj columns (record: in the domain YAML, Docs/Data.md). Array columns are stacked to rows x T x channels (shorter rows NaN-padded, COL@len their lengths); with csv or parquet they go to PART.arrays.npz beside each file. Files roll over at 90 MB. Held-out rows: role, block_role or tag held-out, or held_out true. --json: {schema, out, manifest, rows, columns, files, held_count, nonfinite}. Exit 0 written, 2 usage (no rows, an unknown column, a non-empty --out, held-out rows).

### crane data schema

```sh
crane data schema /tmp/ds
crane data schema ~/.cache/crane/runs/toy/T0 --json
```

List the columns of a dataset (or of some runs) and how many rows there are.

What you'll see: one line (rows, columns, files), one row per column (name, type, and for recorded signals their rate and channels), then next:.

| Param | Type | Description |
|---|---|---|
| `sources` | list | A dataset folder, row files or run folders. |
| `--json` | bool | Print the result as one JSON document instead of text, for scripts (default off). |

raw: `python3 -c "import json; print(json.load(open('DS/manifest.json'))['columns'])"`; --json: `crane.data.schema/1`

Details: --json: {schema, sources, columns, rows, files, arrays}.

### crane data splits

```sh
crane data splits /tmp/ds --by seed
crane data splits /tmp/ds --by timeline --held-out source=real --ratios train=0.8,calib=0.2
crane data splits /tmp/ds --by condition --domain boat --out /tmp/splits.json
```

Divide a dataset into train, calibration and test parts, keeping held-out data out of training.

What you'll see: one line (units, how they were grouped, rows, file), one row per part (units, rows, fraction), the held-out rule, then next:.

Everything from one seed (or condition, or timeline) lands in the same part, so nothing leaks between parts. A unit keeps its part as the data grows.

| Param | Type | Description |
|---|---|---|
| `sources` | list | A dataset folder, row files or run folders. |
| `--by` | str | What stays together: seed, condition, timeline, or a column name, e.g. session. (default `seed`) |
| `--ratios` | str | Part names and their shares, adding up to 1, e.g. train=0.8,calib=0.2. (default `train=0.7,calib=0.15,test=0.15`) |
| `--held-out` | str | Rows matching this are held-out too, e.g. source=real (repeatable). |
| `--domain` | str | Also hold out the seeds this domain's config marks as held-out, e.g. boat. |
| `--salt` | str | Change this to get a different random division. (default `crane`) |
| `--out` | path | The split file to write (default SRC/splits.json for one dataset folder). |
| `--dry-run` | bool | Show the parts; write nothing. |
| `--json` | bool | Print the result as one JSON document instead of text, for scripts (default off). |

--json: `crane.data.splits/1`

Details: Units go to named splits by a salted hash. A row is held-out when its role / block_role / tag column says held-out, held_out is true, its block or seed is a held-out block of --domain's seeds, or it matches --held-out; a unit with any held-out row is held-out and never goes to train or calib. Writing over an earlier split file refuses to move a unit into train or calib from elsewhere. --json: the split file itself plus out and written. Exit 0 written, 2 usage.

## monitor

See how far running work has got, here and on the cluster.

### crane monitor

```sh
crane monitor boat01
crane monitor boat01 --follow --interval 60
crane monitor ~/.cache/crane/studies/boat01 --local --json
```

See how far running studies have got, on this computer and the cluster, in one table.

What you'll see: one row per stage, setting and seed group: done/total, speed, time left, failures, jobs and hosts; then any failures with their reason, and next:.

Add --follow to keep it updating until the work ends. Early averages appear for practice seeds only; held-out seeds show progress only.

| Param | Type | Description |
|---|---|---|
| `target` | str | A study name, e.g. boat01, or a folder (default every study). |
| `--out` | path | Where studies are kept (default CRANE_STUDIES, else ~/.cache/crane/studies). |
| `--follow` | bool | Keep updating until every job has ended or --timeout passes. |
| `--interval` | float | Seconds between updates (5 to 300). (default `30.0`) |
| `--timeout` | float | With --follow: stop after this many seconds. (default `3600.0`) |
| `--local` | bool | Only this computer; do not ask the cluster. |
| `--json` | bool | Print one JSON document (one per update with --follow). |

raw: `cat OUT/STUDY/**/progress.json; research status -p crane --campaign STUDY`; --json: `crane.monitor/1`

Details: Local: every progress.json under a study (or a dir) with cell.json, failure.json and metrics.json beside it. Cluster: when OUT/STUDY/cluster/jobs.json exists, the project job list (research token) and the public feed (/v1/feed); unreachable means a note, not a failure. A progress.json older than 180 s counts as stalled. Failure reasons: crashed, timed out without progress, cancelled. --follow refreshes every --interval s (5..300) and stops when every job has ended or after --timeout s (exit 75). --json: {schema, at, where, study, sources, notes, finished, groups[{stage, arm, block, role, done, total, failed, rate_per_s, eta_s, hosts, jobs, failures, metrics}]}; with --follow one document per refresh (JSONL).

## env (alias: course)

Make, check and draw the courses the robot drives.

### crane env gen

```sh
crane env gen --seed 3 --out /tmp/c3.json
crane env gen --seeds 0:50 --out /tmp/courses
```

Make new random courses (buoy layouts) from seeds and save them as files.

What you'll see: a few CRANE_COURSE_* lines saying what was written.

Give --seed N --out FILE.json for one course, or --seeds A:B --out DIR for many. It waits its turn for Unity, so it can take a while on a busy computer.

| Param | Type | Description |
|---|---|---|
| `--dry-run` | bool | Show what would happen, step by step, without doing any of it. |

raw: `bin/heavy unity Tools/RoboBoat/generate.sh`

Details: Runs the Unity course generator headless through heavy unity. Options: [--family F] --seed N --out T.json | --seeds A:B --out DIR | --validate DIR --out V.json; --mode, --set task.param=v. The family defaults to the generator's. Arguments are checked before the heavy unity slot is taken; --dry-run prints the command only.

### crane env pack

```sh
crane env pack --domain boat --seeds 0:64 --out boat-courses.tar.zst
```

Bundle the courses for a range of seeds into one file, so a computer that cannot make courses (a rented box) can still run them.

What you'll see: the pack path, file count and size. The pack holds manifest.json (domain, generator identity and version, seeds, sha256 per file) and the files. Without --out it goes to ~/.cache/crane-packs/DOMAIN-KIND-SHA.tar.zst. A rented box without dotnet and the Unity editor reads it through crane env unpack and CRANE_SCENARIOS, or a study spec `inputs: [PACK]`.

| Param | Type | Description |
|---|---|---|
| `--domain` | str | The domain whose scenarios to pack. |
| `--seeds` | str | Seeds: N, A:B (B exclusive), a,b,c, @FILE or block:NAME. |
| `--out` | path | The pack file (default under ~/.cache/crane-packs). |
| `--json` | bool | Print the result as one JSON document instead of text, for scripts. |
| `--quiet` | bool | Print only the results and any failures, not the progress lines. |

--json: `crane.env.pack/1`

### crane env unpack

```sh
crane env unpack boat-courses.tar.zst /tmp/courses
```

Open a course bundle into a folder and check every file is intact.

| Param | Type | Description |
|---|---|---|
| `pack` | path | A pack from crane env pack. |
| `dir` | path | Where the scenario files go. |
| `--json` | bool | Print the result as one JSON document instead of text, for scripts. |
| `--quiet` | bool | Print only the results and any failures, not the progress lines. |

--json: `crane.env.unpack/1`

### crane env validate

```sh
crane env validate /tmp/c3.json
```

Check a course file follows the course rules; exit 0 when it does.

What you'll see: one row per rule, pass or fail.

Give it a course file, e.g. one crane env gen made.

raw: `python3 Tools/RoboBoat/validate.py`

Details: The independent constraint check of a course truth. Options: TRUTH.json [--spec S] [--json].

### crane env render

```sh
crane env render /tmp/c3.json > c3.svg
```

Draw a course file as a picture (SVG).

What you'll see: the SVG on stdout; redirect it to a file.

Give it a course file; open the SVG in a web browser to look at it.

raw: `python3 Tools/RoboBoat/render.py`

Details: Options: TRUTH.json [--track T.jsonl] (a track to draw over it).

### crane env explain

```sh
crane env explain /tmp/c3.json
```

Describe a course file in words: its tasks, objects and checks.

Not yet available; today use `python3 Tools/RoboBoat/course_truth.py summary TRUTH.json`.

### crane env diff

```sh
crane env diff a.json b.json
```

Show two course files side by side.

Not yet available; today use `python3 Tools/RoboBoat/course_truth.py diff A B`.

### crane env coverage

```sh
crane env coverage /tmp/courses
```

Show how well a set of courses covers the range of task settings and corner cases.

Not yet available; today use `python3 Tools/RoboBoat/coverage.py TRUTH.json...`.

## card

Look at and export a robot's plant card (its measured physics numbers).

### crane card show

```sh
crane card show
crane card show FAMILY/ROBOT@HEAD --json
```

Show a robot's plant card: its physics numbers, how sure each one is, and how well it was checked.

What you'll see: a table, one row per group of numbers (value, spread, status), then the check results.

| Param | Type | Description |
|---|---|---|
| `ref` | str | Which card: FAMILY/ROBOT@HEAD, the first characters of its id, or a card folder (default the standard card). |
| `--json` | bool | Print the card as one JSON document. |

raw: `python3 -m plantcard show`; --json: `crane.card/1`

Details: Rows per block: parameters with their sd and status; then the validation rows. --json: the card document (robot, sha, status, blocks, validation).

### crane card export

```sh
crane card export fastsim --out /tmp/card.json
```

Write a robot's plant card in the format one simulator or tool reads.

What you'll see: JSON on screen, or saved with --out FILE.

| Param | Type | Description |
|---|---|---|
| `view` | str | Which format: unity (the 3-D simulator), fastsim (the light simulators), or control and nlc (for those tools). One of `unity`, `nlc`, `control`, `fastsim`. |
| `ref` | str | Which card (default the standard card). |
| `--out` | path | Save it to this file. |

raw: `python3 -m plantcard export`

Details: Tier views: unity (T2), fastsim (T0/T1), nlc and control.

### crane card fit

```sh
crane card fit bundle.tgz --out /tmp/fit
```

Make a plant card from field data.

Not yet available; today use `crane fit BUNDLE.tgz --bags DIR`.

### crane card factorial

```sh
crane card factorial spec.yaml
```

Make a set of card variants (e.g. measured vs default for each number) to test which ones matter.

Not yet available; today use `crane fit BUNDLE.tgz --factorial SPEC.yaml`.

### crane card validate

```sh
crane card validate FAMILY/ROBOT@HEAD
```

Check how well a card predicts sessions it was not fitted on.

Not yet available; today use `crane fit with two or more bundles`.

### crane card parity

```sh
crane card parity FAMILY/ROBOT@HEAD
```

Check a card behaves the same in the light and the full simulator (T1 vs T2).

Not yet available; today use `crane compare --tiers LOW HIGH`.

## fit

Turn a day's test bundle into updated simulator settings.

### crane fit

```sh
crane fit bundle-173031.tgz --bags ~/astro_dock/bags/sim/2026-11-24/bags --out /tmp/fit
```

Turn a day's bundle into updated simulator settings, with why for each.

What you'll see: each setting with its verdict (measured, provisional, or left at the default) and why, the evidence, the plant card, and how well it predicts held-out sessions when you gave two or more.

Give it the bundle astro wrote after a day of testing; add --bags DIR for the recordings too, which gives a full plant card. Use the result with crane sim up --card OUT/profile.json.

| Param | Type | Description |
|---|---|---|
| `bundles` | list | Astro's test bundles (.tgz), one per session, e.g. bundle-173031.tgz. |
| `--out` | path | Where to save the results, e.g. /tmp/fit (default BUNDLE.fit/ beside the first bundle, else ~/.cache/crane/fit/BUNDLE/). |
| `--bags` | path | A folder of recordings to read as well, e.g. ~/astro_dock/bags/sim/2026-11-24/bags (only read; repeatable). |
| `--base` | str | The settings to start from and update, e.g. Ideal (the default). |
| `--robot` | str | The name to give the robot's card, e.g. astro/blastoise (default astro/ plus the state file name). |
| `--n` | int | How many simulated tries per held-out check, e.g. 128 (more is slower and steadier). (default `64`) |
| `--no-card` | bool | Stop after the settings; make no plant card (default off). |
| `--factorial` | path | Also write card variants from a plan file (measured vs default per number), e.g. spec.yaml. |

raw: `python3 -m Tools.Characterization.from_bundle`

Details: The bundle goes through the gate to a profile; the bags give evidence for a plant card. Verdicts: identified / provisional / prior. Writes OUT/profile.json (sim_up --profile-json), OUT/profile.gate.json, OUT/cards and OUT/fit.json. A bundle made without --bags gives the gate only. --factorial SPEC.yaml writes card variants and a manifest. Held-out validation runs when there are two or more sessions.

## field

Turn real-robot recordings into runs crane can replay and compare.

### crane field import

```sh
crane field import ~/astro_dock/bags/sim/2026-11-24/bags/171208-smoke --out /tmp/field
```

Turn a real-robot recording into a run that crane can replay, view and compare with the sim.

What you'll see: one line per recording: where it was saved, how long it is, how many position samples, distance travelled, events.

| Param | Type | Description |
|---|---|---|
| `source` | path | A bundle, a recording folder or an .mcap file. |
| `--out` | path | Where to save: a run folder (one recording) or a parent folder (several). |
| `--odom` | str | The topic holding the robot position. (default `/odom/local`) |
| `--events` | str | The topic holding astro's event messages. (default `/events`) |
| `--frame` | str | Say which way a position frame faces, e.g. odom=ned, when its name does not say. |
| `--hz` | float | How many position samples per second to keep. (default `20.0`) |
| `--domain` | str | Which kind of robot it is, e.g. boat. (default `boat`) |

Details: Reads a bundle (made with --bags), a rosbag2 dir or an .mcap file into crane.run/1 records: the odometry trajectory, /events and log.txt lines, frame and clock. --frame ID=enu|ned names an odometry frame id that is not a REP 103/105 name.

## compare

Put two sets of runs side by side, e.g. the boat vs the sim.

### crane compare

```sh
crane compare --field bundle-173031.tgz runs/sim
crane compare --field bundle-173031.tgz runs/sim --bounds file --fit /tmp/fit
```

Compare two sets of runs side by side (e.g. the boat vs the sim).

What you'll see: a table on screen and, with --out, compare.json and compare.md. With --bounds, pass or FAIL per row and a verdict line.

For boat vs sim, give the bundle with --field and the sim runs after it. With --bounds file, exit 1 means some result is further off than allowed.

| Param | Type | Description |
|---|---|---|
| `runs` | list | Two sets of runs, A then B, e.g. runs/a runs/b; with --field, just the sim runs. |
| `--tiers` | bool | A is the lighter simulator and B the fuller one; show the gap (default off). |
| `--field` | path | Compare the runs with a real day's bundle, e.g. bundle-173031.tgz. |
| `--metric` | str | Only compare these results, e.g. time (repeatable; default all). |
| `--direction` | str | With --tiers: whether a higher or a lower number is better. One of `higher`, `lower`. (default `higher`) |
| `--band` | float | With --tiers: the biggest gap you'll accept, in the result's own units, e.g. 0.5. |
| `--fidelity` | path | With --tiers: also replay B's goals on another vehicle's config, e.g. config.yaml. |
| `--bounds` | str | With --field: pass or FAIL each row against allowed limits, e.g. file (the standard limits) or your own FILE. |
| `--fit` | path | With --bounds: the crane fit output folder to check too, e.g. /tmp/fit (default fit-<bundle>). |
| `--domain` | str | With --bounds: which limits file to use, e.g. boat (the default). (default `boat`) |
| `--out` | path | Also save compare.json and compare.md here, e.g. /tmp/cmp. |
| `--json` | bool | Print the comparison as one JSON document instead of the table, for scripts. |

--json: `crane.compare/1`

Details: Kinds: variant A/B on one frame and clock; tier gap (--tiers, A the lower tier); field vs sim per astro test (--field BUNDLE RUNS). --bounds file reads config/fidelity/DOMAIN.yaml, --bounds FILE a file of your own, plus the fit's gate (--fit DIR, default fit-<bundle>); 'card' is not yet available. --fidelity CONFIG replays B's goal windows on CONFIG's vehicle. --json: {schema, kind, result}; with --bounds, result holds rows, gate and verdict. Exit 0 pass, 1 FAIL, 2 usage.

## judge

Score runs pass or fail against a set of rules.

### crane judge fastsim

```sh
crane judge fastsim /tmp/runs --out /tmp/runs-judge
```

Score every run in a folder pass or fail with the standard motion rules.

What you'll see: one verdict line; with --out, a full report. Exit 1 on FAIL.

| Param | Type | Description |
|---|---|---|
| `root` | path | The folder of runs. |
| `--out` | path | Where to save the report. |
| `--json` | bool | Print the report as one JSON document. |

raw: `python3 -m judge fastsim`; --json: `crane.judge.report/1`

Details: Judges every fastsim run under ROOT with the generic motion pack; --out writes report.json and Markdown.

### crane judge oneline

```sh
crane judge oneline /tmp/runs-judge/report.json
```

Print a saved report's verdict in one line; exit 1 on FAIL.

| Param | Type | Description |
|---|---|---|
| `report` | path | The report.json file. |
| `--json` | bool | Print the result as one JSON document instead of text, for scripts. |

raw: `python3 -m judge oneline`

### crane judge markdown

```sh
crane judge markdown /tmp/runs-judge/report.json
```

Print a saved report as readable Markdown; exit 1 on FAIL.

| Param | Type | Description |
|---|---|---|
| `report` | path | The report.json file. |

raw: `python3 -m judge markdown`

### crane judge demo

```sh
crane judge demo
```

See what judging does on a toy drone: one physics fault, one decision fault and a clean run.

| Param | Type | Description |
|---|---|---|
| `--out` | path | Where to save the reports. |

raw: `python3 -m judge demo`

### crane judge run

```sh
crane judge run --pack generic_motion /tmp/runs
```

Score runs against a named set of rules.

Not yet available; today use `crane judge fastsim ROOT`.

### crane judge report

```sh
crane judge report /tmp/runs-judge/report.json
```

Print a saved report as text, Markdown or JSON.

Not yet available; today use `crane judge markdown REPORT.json`.

## check

Acceptance checks on recorded results.

### crane check axis

```sh
crane check axis results.json
```

Check recorded results use the right directions and signs.

Not yet available; today use `crane sim check axis INPUTS`.

### crane check sitl

```sh
crane check sitl results.json
```

The software-in-the-loop acceptance check on recorded results.

Not yet available; today use `crane sim check sitl INPUTS`.

### crane check docking

```sh
crane check docking results.json
```

The docking acceptance check on recorded results.

Not yet available; today use `crane sim check docking INPUTS`.

### crane check smoke

```sh
crane check smoke results.json
```

A quick does-it-basically-work check on recorded results.

Not yet available; today use `crane sim check smoke INPUTS`.

### crane check truth-subscribers

```sh
crane check truth-subscribers
```

Check no robot software secretly reads the simulator's ground truth.

Not yet available; today use `python3 Tools/RoboBoat/check_truth_subscribers.py`.

## view

Watch runs replay, or try settings live, in a window.

### crane view

```sh
crane view --live --domain boat --tier T1 --seed 3
crane view /tmp/runs/base/s3 --screenshot /tmp/view.png --at 60
```

Watch runs replay, or try settings live.

What you'll see: a window with a top-down map, a timeline of events and play/step/speed controls; live mode adds sliders for the settings, Rerun, and Promote.

Give run folders to replay them together, or --live to drive a light sim with sliders. Promote saves the slider settings as a file crane run can use.

| Param | Type | Description |
|---|---|---|
| `runs` | list | Run folders or track files to replay together, e.g. /tmp/runs/base/s3. |
| `--live` | bool | Run a light sim right here with sliders, instead of replaying (default off). |
| `--domain` | str | With --live: which kind of robot world, e.g. boat (the default). (default `boat`) |
| `--tier` | tier | With --live: T0 (no physics) or T1 (physics, the default). One of `T0`, `T1`, `abstract`, `dynamic`. (default `T1`) |
| `--seed` | int | With --live: which scenario to load, e.g. 3 (default 0). |
| `--offset` | str | For the second and later runs: when their start happened on the first run's clock, in seconds, e.g. 12.5. |
| `--transform` | str | For the second and later runs: how to move them onto the first run's map, e.g. '{"xyz": [1, 0, 0], "yaw": 90}'. |
| `--3d` | bool | Open in the 3-D view, for robots that move in all directions. |
| `--screenshot` | path | Save a picture to this file instead of opening a window, e.g. /tmp/view.png. |
| `--at` | float | With --screenshot: the moment to capture, in seconds, e.g. 60 (default the end). |
| `--promote-dir` | path | Where Promote saves settings files, e.g. runs/promoted (the default). (default `runs/promoted`) |

Details: Replays crane.run/1 dirs, fastsim run dirs or .ctraj files overlaid in one frame and clock (--offset and --transform line up runs 2, 3, ...). --live runs a T0/T1 sim in-process; Rerun from t or 0; Promote writes a crane run spec to --promote-dir. --screenshot renders offscreen to a PNG and opens no window.

## board

Browse and replay runs in your web browser.

### crane board

```sh
crane board /tmp/runs
```

Browse and replay your runs, courses and reports in a web browser.

What you'll see: a local web address (default http://127.0.0.1:8765) to open; it lists every run under the folders, with their events and course layers.

| Param | Type | Description |
|---|---|---|
| `roots` | list | Folders to look in (default the current one). |
| `--port` | int | The local port to serve on. (default `8765`) |
| `--host` | str | The address to listen on (the default is only this computer). (default `127.0.0.1`) |

raw: `python3 -m viewer serve`

Details: Indexes crane.run/1 records and .ctraj files, course truths and judge reports under ROOTS.

## media

Make videos and screenshots of runs.

### crane media render

```sh
crane media render RUNS --out /tmp/media
```

Make a video of runs.

Not yet available; today use `python3 -m theater select ... then theater/pack`.

### crane media shot

```sh
crane media shot RUN --out /tmp/shot.png
```

Take a screenshot of the board or a run.

Not yet available; today use `crane view RUN --screenshot OUT.png, or python3 -m viewer shot`.

### crane media assemble

```sh
crane media assemble /tmp/media
```

Join rendered clips into one video.

Not yet available; today use `ffmpeg on the rendered clips`.

## bench

Measure how fast the simulators run on this computer.

### crane bench player

```sh
crane bench player --sweep 1 2 4
```

Measure how much faster than real time the 3-D simulator runs here, with one or several copies at once.

What you'll see: results per copy under PerformanceResults/ and, when trying several counts, a summary for each.

| Param | Type | Description |
|---|---|---|
| `--worker` | int | Run a single copy, numbered with this id. |
| `--sweep` | list | Try these numbers of copies at once, e.g. 1 2 4 (default from config/bench.yaml). |

raw: `bin/heavy unity Tools/Bench/sweep_workers.sh 1 2 4`

Details: Runs the Unity player through heavy unity: one worker, or a sweep over worker counts (summary.json per count). Player settings come from env (CRANE_PLAYER, CRANE_DURATION, CRANE_SCENE, ...).

### crane bench fast

```sh
crane bench fast --engine batched --domain boat --tier T1 --backend numba,jax --E 1024,4096 --out /tmp/bench
crane bench fast --equivalence --tier T1 --seeds 0:500
crane bench fast --domain boat --tier T1 --seeds 0:50 --out /tmp/b
```

Measure how many runs per second the light simulators (T0, T1) manage here.

What you'll see: tables of runs per second, saved as bench.json/bench.md (or batched.json/batched.md).

| Param | Type | Description |
|---|---|---|
| `args` | list | bench.fast's own options, e.g. --domain boat --tier T1 --seeds 0:50 (python3 -m bench.fast --help lists them). |

raw: `python3 -m bench.fast`

Details: The scalar table B, the batched engine (--engine batched: episodes/s per backend and E, x table B, the time split) or the gate-3 equivalence report (--equivalence). Arguments go to bench.fast unchanged (python3 -m bench.fast --help).

### crane bench cluster

```sh
crane bench cluster --domain boat
```

Measure speed and cost on the cluster.

Not yet available; today use `python3 -m bench.fast with its --fresh-jobs, on a cluster host`.

## cluster

Send a study to the shared compute cluster and bring the results back.

### crane cluster submit

```sh
crane cluster submit study.yaml --dry-run
crane cluster submit study.yaml --est-unit-s 12
crane cluster submit study.yaml --stage A --block dev --cloud
crane cluster submit /tmp/jobs.json
```

Send a study to the shared compute cluster as a batch of jobs.

What you'll see: the plan (jobs per stage and setting, time per job, total compute hours) and any warnings, then how many jobs were created or already there, and next:.

Try --dry-run first to see the plan and cost. Commit your changes first: jobs run the exact committed code, and sending the same plan twice does not duplicate jobs.

| Param | Type | Description |
|---|---|---|
| `source` | str | A study file, or a ready job list. |
| `--stage` | str | Only this stage, e.g. A. |
| `--block` | str | Only this group of seeds, e.g. dev. |
| `--est-unit-s` | float | Seconds one run takes, to size the jobs, e.g. 12 (default the study's estimate). |
| `--est-from` | path | Take the time per run from a crane bench fast result (bench.json). |
| `--measure` | bool | Time one run here first to size the jobs (never for T2). |
| `--project` | str | The cluster project to submit under. (default `crane`) |
| `--campaign` | str | A label grouping the jobs (default the study name). |
| `--purpose` | str | One or two sentences on why, shown publicly on the cluster feed (default the study file's purpose:). |
| `--tag` | str | Only run on machines with this tag, e.g. gpu (repeatable; all required). |
| `--cloud` | bool | Allow rented cloud machines (only with an approved request). |
| `--kind` | str | Force CPU (crane_batch) or GPU (gpu_command) jobs (default the study file's choice). One of `crane_batch`, `gpu_command`. |
| `--confirm` | bool | Include the held-out seeds (needs the committed targets file). |
| `--allow-dirty` | bool | Go ahead with uncommitted changes (the jobs still run the last commit). |
| `--no-skip-done` | bool | Run seeds again even if their results are already saved. |
| `--out` | path | Where studies are kept (default CRANE_STUDIES, else ~/.cache/crane/studies). |
| `--dry-run` | bool | Show the plan; send nothing. |
| `--json` | bool | Print the result as one JSON document instead of text, for scripts (default off). |

raw: `research submit -p crane OUT/STUDY/cluster/jobs.json`; --json: `crane.cluster.submit/1`

Details: Compiles a crane.study/1 spec to rc-job/1 jobs (one per cell x seed chunk, 5-30 min each, pinned to the exact commit) and submits them with the research CLI; a jobs.json goes as is. Jobs: kind crane_batch (one slot, every core) or gpu_command for a GPU cell; seeds [0] with the chunk in spec.seeds; spec.run and spec.argv run python3 -m crane_backends.batch, which runs crane study run-cell SPEC --cell JOB --out .. (--confirm --targets-sha256 on held-out jobs); job ids come from the unit cache keys and the commit, so resubmitting is idempotent. Refused (exit 2): uncommitted changes (--allow-dirty runs the commit), a spec outside crane/runtime.sparse, a cell naming field data (~/field_corpus_*, a bag dir, .mcap/.db3/.bag), a held-out block whose targets file is not committed before now, a cell with no time estimate. Without --confirm held-out cells are left out (only held-out selected: exit 2). The plan is saved to OUT/STUDY/cluster/jobs.json before research submit. An adaptive stage goes after its source: submit --stage A, wait, fetch, crane study promote --stage B, submit the promoted spec. --cloud adds needs.tags cloud-ok and cpu (torch for GPU cells) and needs an approved REQUESTS.md row.

### crane cluster wait

```sh
crane cluster wait boat01 --timeout 3600
```

Wait until a study's cluster jobs finish (up to a time limit), then show how they went.

What you'll see: one line saying what it waits for and for how long, then the status table (as cluster status).

Exit 0 means every job finished. If the time limit passes first the jobs keep running; run it again or check with crane cluster status.

| Param | Type | Description |
|---|---|---|
| `study` | str | A study name, e.g. boat01, its results folder, or a job list. |
| `--timeout` | float | Stop waiting after this many seconds. (default `3600.0`) |
| `--out` | path | Where studies are kept (default CRANE_STUDIES, else ~/.cache/crane/studies). |
| `--json` | bool | Print the final status as one JSON document. |

raw: `research wait -p crane JOB... --timeout 3600`; --json: `crane.cluster.status/1`

Details: One research wait. Exit 0 every job done, 1 some failed or were cancelled, 75 the timeout passed first (the jobs are untouched), 2 no plan or no research CLI.

### crane cluster status

```sh
crane cluster status boat01
```

Show a study's cluster jobs: how many are queued, running, done or failed, and why any failed.

What you'll see: a header with the counts, one row per stage, setting and seed group, then failures with their last error line, and next:.

| Param | Type | Description |
|---|---|---|
| `study` | str | A study name, its results folder, or a job list (default every job). |
| `--out` | path | Where studies are kept (default CRANE_STUDIES, else ~/.cache/crane/studies). |
| `--json` | bool | Print the result as one JSON document instead of text, for scripts (default off). |

raw: `research status -p crane --campaign STUDY`; --json: `crane.cluster.status/1`

Details: Read-only, digest style. --json: {schema, study, project, campaign, states, groups, failures, finished}. Without STUDY: research status (every job).

### crane cluster fetch

```sh
crane cluster fetch boat01
```

Bring a study's finished cluster results back to this computer, laid out as if it ran here.

What you'll see: one row per job brought back (fetched, missing or failed, rows, from where), the counts, and next:.

Safe to run again; it only fetches what is missing. Every file is checked, and exit 1 means one arrived damaged.

| Param | Type | Description |
|---|---|---|
| `study` | str | A study name, e.g. boat01, its results folder, or a job list. |
| `--out` | path | Where studies are kept (default CRANE_STUDIES, else ~/.cache/crane/studies). |
| `--boxes-only` | bool | Only take results already copied off rented machines; do not ask the cluster. |
| `--json` | bool | Print the result as one JSON document instead of text, for scripts (default off). |

raw: `research wait -p crane JOB... --out DIR; rsync from the box`; --json: `crane.cluster.fetch/1`

Details: Lands in OUT/STUDY/<stage>/<arm>/<cell>/parts/JOB/; the cell's manifest.json lists every part's files, each checked by sha256. Box pulls (~/.cache/vast-out/BOX/STUDY/JOB/, CRANE_VAST_OUT) first, else the coordinator's artifacts (a viewer token: RC_VIEWER_TOKEN or ~/.config/research-cluster/platform-viewer.token).

## build

Build the simulator program, the docker images or the Python package.

### crane build player

```sh
crane build player --tarball
```

Build the 3-D simulator program (no window) from this checkout, labelled with the code version.

What you'll see: the Unity command it runs, then the build label file and, with --tarball, the packed program.

Needs this project's Unity editor installed, and waits its turn for Unity. --skip-unity only re-labels and packs a build you already have.

| Param | Type | Description |
|---|---|---|
| `--tarball` | bool | Also pack it into one file to copy to other computers. |
| `--skip-unity` | bool | Label and pack an existing build only. |
| `--editor` | path | The Unity editor to use (default UNITY_EDITOR, else the project's version). |
| `--builds` | path | The builds folder (default Builds in this checkout). |
| `--dry-run` | bool | Show the Unity command; build nothing. |

Details: Builds the headless Linux player through heavy unity and stamps Builds/CRANE-Worker/crane-build-manifest.json with the source commit; --tarball writes Builds/crane-player-<commit12>-linux-x86_64.tar.zst and its .sha256.

### crane build images

```sh
crane build images core gpu
crane build images sim --player Builds/crane-player-X.tar.zst
```

Build the docker images used on cluster and rented machines (core, gpu, sim), labelled with the code version.

What you'll see: the docker command per image, then one row per image (name, size) and next:.

It never uploads anything; publishing happens in CI. sim needs a packed simulator from crane build player --tarball.

| Param | Type | Description |
|---|---|---|
| `kinds` | list | Which images: core, gpu, sim (default core). One of `core`, `gpu`, `sim`. |
| `--player` | path | For sim: the packed simulator file (its .sha256 beside it). |
| `--repo` | str | The image name. (default `crane`) |
| `--dry-run` | bool | Show the docker commands; build nothing. |

raw: `docker build -f Dockerfile.KIND -t crane:KIND-COMMIT .`

Details: Tags crane:KIND-<commit12>. core < 300 MB compressed; gpu = python:3.12-slim + jax[cuda12] + numba, < 4 GB; sim = core + the player + xvfb/Mesa Vulkan, about 1.2 GB. sim takes the tarball from --player (default the newest Builds/crane-player-*.tar.* with its .sha256) and is built on the core image of the same run. .github/workflows/images.yml pushes lunarzdev/crane:KIND-<sha> (never :latest).

### crane build wheel

```sh
crane build wheel
crane build wheel --out dist
```

Build the crane Python package files, with their checksums.

What you'll see: the build command, then one row per file (name, size, checksum) and next:.

| Param | Type | Description |
|---|---|---|
| `--out` | path | Where the files go (default ~/.cache/crane/dist). |
| `--dry-run` | bool | Show the build command only. |
| `--json` | bool | Print the result as one JSON document instead of text, for scripts (default off). |

raw: `uv build --out-dir DIR`

Details: The wheel and sdist, by uv build (else python3 -m build), and their sha256.

## deploy

Copy crane (and the simulator) onto another computer you work from.

### crane deploy

```sh
crane deploy HOST --dry-run
crane deploy HOST --player
crane deploy HOST --components core,fast --player
```

Copy this version of crane (and, if you ask, the 3-D simulator) onto another computer you work from, then check it there.

What you'll see: a line on what the other computer has, one row per step as it runs, then the crane doctor check from there and next:.

Try --dry-run first; it changes nothing there. Robot computers are refused: robot code goes through `astro hot`.

| Param | Type | Description |
|---|---|---|
| `host` | str | The computer to set up: its ssh name, e.g. lab-pc, or USER@HOST. |
| `--player` | bool | Also copy and install the 3-D simulator. |
| `--tarball` | path | The packed simulator file to send (implies --player). |
| `--ref` | str | Which code version to send: a branch or commit (default HEAD, what you have checked out). (default `HEAD`) |
| `--components` | str | Which parts to install there, e.g. core,fast (default core; only when it has none yet or you give this). |
| `--no-venv` | bool | Skip installing the Python environment there. |
| `--from` | str | Where the other computer fetches the code from (default this computer over ssh). |
| `--dry-run` | bool | Look at the other computer and show the plan; change nothing. |
| `--json` | bool | Print the result as one JSON document instead of text, for scripts (default off). |

raw: `ssh HOST 'cd ~/crane_ml && git fetch ssh://THISHOST/REPO main && git merge --ff-only FETCH_HEAD'`; --json: `crane.deploy/1`

Details: Over ssh: checks the host's disk, syncs the code (git fetch from this machine + ff-only, or rsync when it has no checkout), makes its .venv (install.sh), sends the T2 player with --player (sha256 checked there, current + prev kept), then runs crane doctor there. Touches only ~/crane_ml, ~/.local/bin/crane and the player store (~/.local/share/crane/player) on HOST. Refuses (exit 2) a robot computer (a host whose roles in astro's registry, ASTRO_PROFILES/*/devices.yaml, include a robot role and no operator role; config/deploy.yaml), a host with less free space than the player (tarball and unpacked) plus 1 GB, a checkout that cannot fast-forward, or a ~/crane_ml that is neither a crane checkout nor an earlier deploy. The .venv step runs install.sh --yes --no-clone --components LIST there (uv, Python 3.12) when HOST has no .venv or --components is given. The player tarball is --tarball, else the newest Builds/crane-player-*.tar.* with its .sha256 (a warn row when its commit is not the code's), else the release player; rsync --partial resumes a cut transfer. HOST fetches the full commit from --from (default ssh://THISHOST/REPO, CRANE_DEPLOY_FROM). local:DIR deploys into DIR as the home (no ssh; for tests). Exit 0 ok, 1 a step or the remote doctor failed, 2 usage or a refusal.

## update

Get the latest crane and simulator on this computer.

### crane update

```sh
crane update
crane update --dry-run
crane update --version 0.3.0 --player-only
```

Get the latest crane and 3-D simulator on this computer.

What you'll see: which release it found, one row per step (code, simulator), then next:.

The previous simulator is kept, so a bad update can be rolled back. It refuses when the disk is too full, before downloading anything.

| Param | Type | Description |
|---|---|---|
| `--version` | str | Get this release, e.g. 0.3.0 (default the latest). |
| `--player-only` | bool | Only update the simulator. |
| `--no-player` | bool | Only update the code. |
| `--tarball` | path | Install this packed simulator file instead of downloading one. |
| `--sha256` | str | --tarball's checksum (default read from its .sha256 file). |
| `--store` | path | Where simulators are kept (default ~/.local/share/crane/player). |
| `--dry-run` | bool | Show the steps; change nothing. |
| `--json` | bool | Print the result as one JSON document instead of text, for scripts (default off). |

raw: `git fetch && git merge --ff-only @{u}; curl -LO RELEASE/ASSET; sha256sum -c`; --json: `crane.update/1`

Details: Updates this checkout (git fetch + ff-only) or this install (the release wheel), then the T2 player from the release into the player store (sha256 checked, current + prev kept). The release is release.json of the latest GitHub Release, or of v<VERSION> (CRANE_RELEASES overrides the base URL or names a local dir laid out the same way). A checkout with no release yet still fast-forwards. Refuses (exit 2) when the store's disk has less free space than the player (tarball and unpacked) plus 1 GB. Exit 1 when a file fails its sha256 check. --tarball installs a local player tarball (crane deploy runs it on the host).

## heavy

Take turns on this computer for big jobs (Unity, sims, test suites).

### crane heavy

```sh
crane heavy contain test python3 -m pytest fastsim/tests -q
```

Run a big job (Unity, a sim, a test suite) only when this computer has room, so jobs take turns instead of freezing it.

What you'll see: the job's own output, once it gets its turn.

Put the kind of job and the command after it, e.g. crane heavy unity CMD, or crane heavy contain test CMD for a test suite; crane heavy status shows what is running and waiting. A job that waits too long gives up with exit 75 rather than hanging.

raw: `bin/heavy`

Details: heavy's own CLI (bin/heavy): it waits for a slot (bounded, exit 75 on timeout), runs CMD and releases it; slots come from config/heavy.yaml. `crane heavy status` lists the slots, the queue and ETAs.

## doctor

Check this computer can run the simulator; says what to run next.

### crane doctor

```sh
crane doctor
```

Check this computer can run the simulator; prints what to run next.

What you'll see: a table of checks (ok, off, missing, warn or FAIL, each with a reason), then next:.

Safe to run any time; it changes nothing. Things marked missing are optional extras; only FAIL needs fixing.

| Param | Type | Description |
|---|---|---|
| `--json` | bool | Print the checks as one JSON document instead of a table, for scripts. |
| `--quiet` | bool | Print only the results and any failures, not the progress lines. |

--json: `crane.doctor/1`

Details: Checks python, the extras, the T2 player and its flags, heavy, astro_dock and the GPU. Exit 1 only when the core (python >= 3.10, numpy, pyyaml) is broken. --json: {schema, ok, checks[{name, state, detail}], next[]}.

## setup

A guided install of crane on this computer.

### crane setup

```sh
crane setup
crane setup --yes --components core,fast,player --domains boat
crane setup --yes --dry-run
```

Install crane on this computer step by step: it looks at the machine, suggests what to install, then does it.

What you'll see: what it found (system, cores, GPU, docker, astro, Unity, screen), the choices with sizes and times, one line per install step, then crane doctor.

Answer the questions, or add --yes to take the suggestions. It never touches astro's own images or workspaces.

| Param | Type | Description |
|---|---|---|
| `--yes` | bool | Ask nothing: take the suggestions, or the lists given. |
| `--components` | str | Which parts to install, e.g. core,fast,player (default suggested from this computer). |
| `--domains` | str | Which robot worlds to set up, e.g. boat,sub (default all). |
| `--dry-run` | bool | Look and plan; install nothing. |
| `--json` | bool | Print the result as one JSON document (implies --yes). |

raw: `./install.sh --yes --components core,fast; crane update --player-only`; --json: `crane.setup/1`

Details: Interactive with rich and questionary (the [setup] extra), else plain prompts; --yes takes the defaults or --components/--domains. Components: core, fast, gpu, view, field, data (pip extras, through install.sh), player (the release player, crane update --player-only), astro (a read-only check of the astro integration). The choices are saved to ~/.config/crane/setup.json. --dry-run stops after the plan.

## help

List every command, and keep the help files up to date.

### crane help

```sh
crane help sim
crane help --check
```

List every crane command in one line each, or the commands of one noun.

What you'll see: one line per command; with a noun, its commands and an example each.

For everything about one command, run crane NOUN VERB --help.

| Param | Type | Description |
|---|---|---|
| `topic` | list | A noun (and verb), e.g. sim. |
| `--check` | bool | Exit 1 when the help files are out of date. |
| `--write` | bool | Rewrite the help files (help.yaml and Docs/CLI.md). |
| `--json` | bool | Print every command as one JSON document. |

Details: --check regenerates help.yaml and Docs/CLI.md into a temp dir and exits 1 when they differ from crane/commands.py; --write regenerates them in place.
