# Procedural courses and ground truth (stream C)

Courses are generated at runtime from a family of task assets; no scene edits.

## Run
- Player: `--crane-course-family roboboat_2026 --crane-course-seed N [--crane-course-mode iid|corner=lo|hi|mixed]
  [--crane-course-set task.param=value]... --crane-course-truth DIR/course_truth.json [--crane-detection-labels oracle|camera]
  [--crane-ros-course-truth]`. A `--crane-roboboat-course FILE.json` whose JSON names `family` (and `seed`, `mode`, `set`) does
  the same, so the rig needs no change; a 2025 element list still goes to the legacy builder. `--crane-course-replay truth.json`
  rebuilds a recorded course. Ready line: `CRANE_COURSE_READY name= seed= sha256=` (plus the legacy `CRANE_ROBOBOAT_COURSE_READY`).
- Headless, no sim slot: `Tools/RoboBoat/generate.sh --family roboboat_2026 --seed N --out t.json` (or `--seeds A:B --out DIR`).
- Same family sha + seed + overrides (+ crane commit) gives the same `generator.truth_sha256`, in the editor, the player and CI.

## Files
- `course_truth.json`: `crane.course_truth/2` (schema string still `crane-roboboat-course-truth-v1`, `schema_version: 2`),
  world frame (= /crane/odom). Keeps the v1 element and bay keys; adds ids, roles, kinds, gates, bays with pier/slip/sign/indicator,
  tasks with resolved params, anchors, footprints and semantics, states with programs, and the generator's own row verdicts.
- `perception_truth.json`: x, y, z, label (in the run's label mode) and an opaque id only; for mhseals_nav sim_backend.
- `states.jsonl`: one `{t_sim, id, value}` per state change (lights, signs, horn blasts).
- ROS (opt-in): `/crane/truth/course` every 2 s, `/crane/truth/states` on change + 1 Hz, both std_msgs/String JSON;
  `/crane/sim/horn` `{on, freq_hz}` boat-side. Only nodes on `Tools/RoboBoat/truth_allowlist.yaml` may subscribe to
  `/crane/truth/*`; the rig runs `check_truth_subscribers.py`.

## Rules
`Tools/RoboBoat/constraints/roboboat_2026.json` (a link to `Assets/Config/Course/Resources/roboboat_2026.constraints.json`) is the
single spec: G (course), T1-T6 (tasks), P1-P5 (paths), tagged H (handbook page), L (live amendment) or D (our choice).
The generator checks every placed task's rows after each task and re-forks the one that breaks a row; `validate.py` re-judges a
truth independently. Paths treat the boat as a disc of beam/2 + 0.15 m, so a gate with clear >= beam + 0.3 passes.

Relative orientation (all D): entry gate 2 turns <= 15 deg off gate 1 (T1.gate2_yaw); the debris channel starts past gate 2 and
heads within `debris_clearance.heading_deg` +-15 deg of gate 2's normal (T2.heading); the marina turns rigidly by
`marina.yaw_deg` +-90 deg off the shore normal with its shore-most spine corner `shore_gap` off the shore (T5.yaw, T5.shore);
the speed leg turns by `speed_challenge.yaw_deg` +-180 deg about its centre (G4 and the path rows keep it clear). Yaw params made
with `ParamRange.Angle` stay iid in corner modes. `supply_drop.min_docked` (1-2) hulls sit in free red slips (never green, so
never the target; never beside a moored boat; `marina.free_red_min` keeps one free): T4.docked, T4.dock_green_free (H p.54),
T4.dock_moored_free, T5.free_red, judged geometrically with the `in_bay` predicate.

## Agents
`course_truth.py summary|query|render|diff|validate|explain`. `explain SESSION` joins truth, track.jsonl and events into a
per-task verdict (gates and direction, clearances, direction-source colour at the gate vs the circling sweep, slip tier, horn
zone). `coverage.py` bins resolved params across sessions; failing tuples go in `regressions.json`.

## Adding a task
One `XTask : CourseTaskDefinition<XParams>` (Place, Generate), its asset in the family, catalog kinds (and stream A prefabs with a
`CourseObjectAnchor`; until then a placeholder primitive stands in), and its rows in the spec. Nothing else changes.
