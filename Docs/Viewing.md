# Viewing runs

```bash
crane view --live --domain boat --tier T1 --seed 3        # the sandbox: the sim in-process, with sliders
crane view runs/boat/base/s3 unity-s3.ctraj               # replay, overlaid in one frame and on one clock
crane board runs/                                          # the web board over every run under runs/
```

`crane view` is the native window for development: look at a system, change a parameter, see what it does.
`crane board` is the web replay board; the cluster dashboard deploys the same page. Both read
[`crane.run/1`](RunRecord.md) records and bare `.ctraj` files. `crane view` needs the `[view]` extra:
`uv venv .venv -p 3.12 && uv pip install --python .venv -e '.[view]'`.

## `crane view RUN…`: replay
- **What.** The top-down map shows each run's trail up to the playhead (the rest faint), the vehicle outline at
  its heading, the world layers and the constraint volumes. Under it, the timeline has one row per event lane.
- **When.** Look at one run, or put up to 6 side by side: tier against tier, variant against variant, sim
  against field.
- **Command.**
  ```bash
  crane view runs/boat-t1/base/s3                               # a fastsim run dir (its crane.run.json)
  crane view runs/boat-t1/base/s3 runs/boat-t0/base/s3          # T1 against T0
  crane view runs/a unity.ctraj                                 # a bare .ctraj opens as a T2 unity run
  crane view runs/a runs/b --transform '{"from": "dock", "to": "pool", "xyz": [10, 0, 0], "yaw": 0}'
  ```
- **What you'll see.**
  - The runs are drawn in the first run's frame and on its clock. ENU and NED are converted; NED is drawn
    with north up, like ENU.
  - Different origins are refused unless `--transform` maps them. A sim clock beside a wall clock is refused
    unless `--offset` places the second run (the first run's time of its t = 0).
  - Samples inside a keep-out volume, or outside every keep-in volume, are red.
  - The side panel shows each run's tier, kind, outcome and pose at the playhead, and the events so far.
  - Controls: Play/Pause, Step, the speed slider (0.25x to 64x), the scrub bar, and dragging the playhead on
    the timeline. The 3-D button draws the trails with z (6-DOF records).
  - A recorded run holds poses, not the sim state, so it reruns from 0 only. **Rerun from 0** re-simulates the
    first run that names its fastsim config (fastsim run dirs do), live, beside the recording.

## `crane view --live`: the sandbox
- **What.** The domain's sim runs in-process (T0 or T1, fastsim's scalar loop). It has sliders for the
  parameters the domain declares, and you can rerun from any point.
- **When.** Try an idea: a current, a plant delay, pose noise, a heavier card, another gain. See where the run
  departs, then keep the settings as a batch.
- **Command.**
  ```bash
  crane view --live --domain boat --tier T1 --seed 3
  crane view --live --domain boat --tier T0 --seed 3 runs/boat-t1/base/s3     # live T0 beside a recorded T1
  ```
- **What you'll see.**
  - **Play** steps the sim at the chosen speed. A slider acts from the current time.
  - **Rerun from t** restores the in-memory snapshot at or before the playhead (one is kept every sim second)
    and goes on with the current sliders. **Rerun from 0** starts over with them.
  - With the same sliders, a rerun gives the same states sample for sample.
  - **Promote** writes `runs/promoted/<name>/spec.yaml`, plus `card.json` when a card slider moved. It prints
    the `crane run` command and copies it to the clipboard. The command runs the same config with the
    sliders as `--set` overrides. Disturbances `crane run` cannot apply (plant delay, pose and heading noise)
    are listed under `view_only`.
  - A seed without a scenario file gets one from the domain's generator, and the window says so.

The sliders come from the domain's `view:` section (`config/domains/boat.yaml`):

```yaml
view:
  frame: {world: enu, origin: scenario, body: flu}
  outline: [[0.62, 0.0], [0.4, 0.45], [-0.53, 0.45], [-0.53, -0.45], [0.4, -0.45]]
  lanes: {phase: mission, goal: goal}          # event kind -> timeline lane
  card: [vehicle.profile, adapter.options.profile]
  current: {T1: vehicle.current}               # T1's plant takes the current; T0 drifts kinematically
  params:
    - {name: current_speed, unit: m/s, min: 0, max: 1, step: 0.05, bind: 'disturbance:current_speed'}
    - {name: kp_surge, min: 0, max: 6, step: 0.1, bind: 'config:vehicle.kp.0', tiers: [T1]}
    - {name: mass, unit: kg, min: 10, max: 80, step: 1, bind: 'card:mass', tiers: [T1]}
```

A `bind` is one of:
- `config:KEY`: the tier's fastsim config, where an int part indexes a list;
- `card:KEY`: a number in the plant card;
- `disturbance:NAME`: `current_speed`, `current_toward` (deg, ENU, the way the water flows), `delay_s`,
  `pose_noise_m` or `yaw_noise_deg`.

A plant or limits change rebuilds the plant at the current pose and body velocity. Live mode goes through
`crane.view.live.LiveSim` (`reset`, `step(n)`, `snapshot`/`restore`, `set_param`); `FastsimLive` implements it.

## `crane view --screenshot`: no window
- **What.** It renders the same window offscreen (`QT_QPA_PLATFORM=offscreen`) to a PNG and exits.
- **When.** Agents, tests and CI: anything that must not open a window.
- **Command.**
  ```bash
  crane view runs/boat-t1/base/s3 --screenshot /tmp/view.png --at 60
  crane view --live --domain boat --tier T1 --seed 3 --at 90 --screenshot /tmp/live.png   # sims to t = 90
  ```
- **What you'll see.** `crane view: /tmp/view.png at t = 60.0 s`. Without `--at`, the image is at the end of
  the run (live: at the outcome). The 3-D view needs an OpenGL display, so a screenshot is always the 2-D map.

## `crane board ROOTS…`: the web board
- **What.** A local web page over every run under ROOTS: one run in depth, two tiers on one clock, a batch at a
  glance.
- **When.** Browse many runs, share a link, or look at what the cluster dashboard shows.
- **Command.**
  ```bash
  crane board runs/ ~/judge-reports --port 8765
  python3 -m viewer shot 'http://127.0.0.1:8765/#/run/<run_id>?t=60' /tmp/board.png   # headless, no window
  ```
- **What you'll see.** `crane viewer: N runs, … -> http://127.0.0.1:8765/`. The board serves:
  - every crane.run/1 record (`crane.run.json`, or `run.json` with that schema), with its trajectory, meta,
    constraint layers and `events.jsonl`;
  - every bare `.ctraj` that is not part of a record;
  - course truths and judge reports.

  A record is served as one `.ctraj`. Its events carry their lane, so log lines get their own **Log** lane.
  Its constraint volumes draw on the map (toggle with `C`). A fastsim batch `OUT/<variant>/s<seed>` is study
  OUT, arm `<variant>`.

`python3 -m viewer ingest SRC… --out DATA [--study S] [--arm A]` copies runs into a self-contained board root,
as served, with each run's overview LOD.

## Layers and events a run can carry
Constraint volumes go in the meta as `layers`, or beside it as `layers.json`
(`{"schema": "crane.layers/1", "layers": [...]}`). Both are in the run's frame:

```json
{"type": "constraint", "mode": "keep_out", "shape": "polygon", "id": "no-go", "points": [[4, -1], [6, -1], [6, 3], [4, 3]]}
{"type": "constraint", "mode": "keep_in", "shape": "corridor", "id": "lane", "points": [[0, 0], [40, 0]], "width": 4}
{"type": "constraint", "mode": "keep_out", "shape": "circle", "id": "swimmer", "points": [[20, -3]], "radius": 1.5}
```

Events are `events.jsonl` lines `{t, kind, ...}`:
- a log line is `{"t": 45.0, "kind": "log", "text": "/events: lidar dropout", "level": "warn"}`;
- `t_end` makes a span;
- `lane` overrides the domain's lane for the kind.

World layers come from the scenario file a run keeps (`course_truth.json` for the boat).
