# Tools/Rig: headless sim runs, a job queue and an in-run watchdog

The player runs on the host (batchmode, Vulkan on an X display); the ROS stack runs in the
rig's own container (private IPC, `--init`, host network). `run_sim.sh` is one run; `rigq`
queues runs, caps them, and stops the ones that hang, stall or never move.

## Setup (once per host)

    cp Tools/Rig/rig.local.example.yaml Tools/Rig/rig.local.yaml   # this host's paths (gitignored)
    Tools/Rig/container.sh up                                       # creates the crane_rig container

Every host-specific value (player, courses, profiles, image, mounts, display) comes from
`rig.local.yaml`, `$CRANE_RIG_LOCAL` or the `${VAR}` defaults in `rig.yaml`.

## Use

    Tools/Rig/rigq submit --name smoke --drive manual_surge -- -d 20     # queue; starts a worker
    Tools/Rig/rigq list                                                 # or --json
    Tools/Rig/rigq results -n 10
    Tools/Rig/rigq cancel ID
    Tools/Rig/rigq worker --slots 1                                     # foreground worker instead

Everything after `--` is `run_sim.sh` options (`run_sim.sh -h`); the queue owns `--domain`,
`--port` and the session dir (`sessions_dir/rigq/<id>`). `--kind cmd|build -- ARGV...` queues any
command (builds are capped at `queue.builders` per host). Every command takes `--json`.

## Queue and watchdog

- State is plain files under `queue.dir`: `jobs/<id>.json`, `logs/<id>.log`, `results.jsonl`.
- Caps: `queue.slots` jobs from this queue, the host's `max_instances` rig slots (lock files in
  `lock_dir`, shared with any other rig copy), `queue.builders` builds.
- Domains: one per running job from `queue.domains` (at most 232, never 201-230), skipping any
  another rig run holds. Before reuse the domain is cleared: stray container processes on it and
  command_mux's saved holds (`/tmp/mhseals/command_mux_<domain>.json`).
- Watchdog (`watchdog.py`, limits in `rig.yaml watchdog:` or per job): `hard_timeout`,
  `no_start`, `stalled` (no new sim time or topic data, from `probe.py` in the container),
  `no_motion` (motion commanded for `motion_grace_s` but under `min_travel_m` travelled: a
  drive held by the mux), `post_timeout`. On a trip: TERM to the runner's and the player's
  process groups (exact PIDs from the session's `run.env`), KILL after `grace_s`, then every host
  process tagged `RIGQ_JOB=<id>` and every container process with the run's `SIMRIG_DIR`. The
  reason goes to the job and `results.jsonl`, and the next job starts.
- `rigq submit --inject-hang S` freezes the player S seconds after ready: a watchdog test.

## Tests

    python3 -I -m pytest Tools/Rig/tests -q     # no ROS, Docker or player needed
