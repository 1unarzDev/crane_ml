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
- `perception_truth.json`: x, y, z, label (in the run's label mode) and an opaque id only; for mhseals_nav sim_backend. In camera mode the id (also the Detection3D id) hashes a colour-free key (task, role, ordinal) with a per-process cryptographic salt, so it changes every run and cannot be recomputed from the seed; in oracle mode it is seed-stable.
- `states.jsonl`: one `{t_sim, id, value}` per state change (lights, signs, horn blasts).
- ROS (opt-in): `/crane/truth/course` every 2 s, `/crane/truth/states` on change + 1 Hz, both std_msgs/String JSON;
  `/crane/sim/horn` `{on, freq_hz}` boat-side. Only nodes on `Tools/RoboBoat/truth_allowlist.yaml` may subscribe to
  `/crane/truth/*`; the rig runs `check_truth_subscribers.py`.

## Rules
`Tools/RoboBoat/constraints/roboboat_2026.json` (a link to `Assets/Config/Course/Resources/roboboat_2026.constraints.json`) is the
single spec: G (course), T1-T6 (tasks), P1-P5 (paths), tagged H (handbook page), L (live amendment) or D (our choice).
The generator checks every placed task's rows after each task and re-forks the one that breaks a row; `validate.py` re-judges a
truth independently. Paths treat the boat as a disc of beam/2 + 0.15 m, so a gate with clear >= beam + 0.3 passes.

Rows judge the built objects, never the generator's params or semantics: gate poses come from their posts (T2.bend is the
curvature through consecutive channel gates), colours from the `states` (T3.direction_marker/_sign map the source's state at t=0
to the circling direction), the marina yaw from the spine against the shore normal, the rooting from the shore line alone.

Relative orientation and placement (all D; rationale: the handbook p.40 course map):
- entry gate 2 turns <= 15 deg off gate 1 (T1.gate2_yaw); the debris channel starts past gate 2 and heads within
  `debris_clearance.heading_deg` +-15 deg of gate 2's normal (T2.heading).
- the marina turns rigidly by `marina.yaw_deg` +-90 deg off the shore normal with its shore-most spine corner `shore_gap` off
  the shore line (T5.yaw, T5.shore); it slides toward side S until its mouth (`anchor marina.outer_end`) is `mouth_clear` 11 m
  from the start and the entry lane (T5.mouth_clear), so zone 2 always fits.
- speed: any position and heading on side S, but the gate lies beyond entry gate 2 along its normal (T3.beyond_gate2, 3 m) and
  its opening faces the boat's approach within 60 deg, from the channel exit and from gate 2 (T3.approach_exit,
  T3.approach_gate2); `speed_challenge.yaw_deg` +-60 turns it off the mean approach bearing, scaled to stay inside both.
- supply: `min_docked` (1-2) hulls sit in free red slips, never green (so never the target, H p.54), never in or beside a
  moored boat's slip on the same pier (`marina.free_red_min` keeps one such slip): T4.docked, T4.dock_green_free,
  T4.dock_moored_free, T4.dock_beside_moored, T5.free_red. The rest scatter one by one among the tasks: each within 25 m of
  another task's object (T4.near_task), >= 8 m apart (T4.spacing), by at least two different tasks (T4.scatter); free hulls
  have no common footprint.
- harbor zone 2: the buoy 7-12 m off the marina mouth (T6.zone2) and >= 15 m from the start and the entry lane
  (T6.zone2_start, T6.zone2_lane).
- Yaw params made with `ParamRange.Angle` stay iid in corner modes.

Slips: `bays[].slip` and `semantics.target.slip` are the slip's position on its pier, 1 at the shore root; the banner numeral is
`bays[].sign.number` and `semantics.target.number`.

## Agents
`course_truth.py summary|query|render|diff|validate|explain`. `explain SESSION` joins truth, track.jsonl and events into a
per-task verdict (gates and direction, clearances, direction-source colour at the gate vs the circling sweep, slip tier, horn
zone). `coverage.py` bins resolved params across sessions; failing tuples go in `regressions.json`.

## Adding a task
One `XTask : CourseTaskDefinition<XParams>` (Place, Generate), its asset in the family, catalog kinds (and stream A prefabs with a
`CourseObjectAnchor`; until then a placeholder primitive stands in), and its rows in the spec. Nothing else changes.
