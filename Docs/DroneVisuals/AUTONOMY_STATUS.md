# CRANE drone autonomy: status, architecture and validation

_Owner: autonomy lead (research/drone-sim worktree). Started 2026-11-09. Evidence labels follow
Docs/ResearchOrchestration/CONTRACTS.md. Each milestone below carries its own label. Nothing here is
PHYSICALLY_CALIBRATED._

Code: `Assets/Scripts/Drones/Autonomy/**`. Tests: `Assets/Tests/Editor/DroneAutonomy*`.

## M0 Architecture decision (DECIDED 2026-11-09)

### Where autonomy runs for now

The autonomy stack runs **in process, as pure C#, in lockstep with the plant**.
- Every module is stepped on the plant's 5 ms log tick by one deterministic scheduler
  (`AutonomousMission.Step`).
- Simulated compute time for each module comes from a seeded latency model (lognormal body plus a
  Pareto stall tail, the NavPilot `LATENCY_TAILS` port). Wall-clock compute time is never used.
- A module's output therefore becomes visible at `t_start + c_sampled`, so a run is bit-reproducible
  from (forest seed, mission, run seed, config).

Why:
1. **Determinism and replay.** Failures can be replayed exactly. ROS-TCP has no lockstep barrier
   (AUTONOMY_DESIGN §0), and PX4 SITL lockstep does not exist yet (PX4_LOCKSTEP_PLAN.md is a plan).
2. **Speed.** There is no serialization or process hop. The same code runs headless in Unity batch
   and outside Unity: `Crane.Drones.Autonomy` is a `noEngineReferences` assembly, so a .NET harness
   (`Assets/Scripts/Drones/Autonomy/Harness~`, ignored by Unity) can compile and run missions while
   the shared Unity slot is busy.
3. **One contract set, many backends.** Each stage talks to the next through a plain C# interface,
   with typed, timestamped (int64 ns sim-clock) messages. The interfaces mirror the ROS 2 / PX4
   topics in AUTONOMY_DESIGN §2. A later ROS 2 or PX4 SITL deployment wraps the same modules behind
   nodes and changes no module contract.

### Pipeline and interface boundary

```
IRayWorld (ManifestRayWorld | Unity collider world)
   |  ray queries (sensors only), ground-truth queries (metrics only, never visible to the stack)
   v
IDepthSensor  -> DepthFrame   (acq_ns, avail_ns, intrinsics, float[] depth, invalid mask)
ILidarSensor  -> LidarScan    (start_ns, end_ns, avail_ns, per-point offset ns, sensor-frame xyz, skewed)
IStateEstimator -> StateEstimate (t_ns, p/v/yaw in the local odom frame, sigma, valid, input age)
              -> MapAlignment  (local odom -> global/prior frame, believed offset and sigma)
IOccupancyMap  <- Insert(DepthFrame|LidarScan, pose history)     (log-odds voxels, conservative miss)
IPriorMap      (forest manifest, optionally stale, incomplete or misaligned)
IGlobalRouter  -> route / cost-to-go (corridor waypoints or grid A*/wavefront on prior + map)
ILocalPlanner  -> PlanDecision (chosen primitive, v_cmd, d_free, stop-rule terms, feasible set size)
SetpointArbiter (RC > supervisor brake/hold > planner; stall watchdog; "no safe maneuver" stop)
   -> TrajectorySetpoint (pos/vel/acc/yaw/yaw-rate, valid_until_ns, source tag)
IFlightControllerLink  <- Submit(TrajectorySetpoint); -> FcStatus
   in-process: InProcessControllerLink -> CraneMultirotorController -> CraneAerialPlantCore
   later: PX4 offboard (uXRCE-DDS TrajectorySetpoint) / PX4 SITL lockstep bridge
```

Cross-cutting contracts:
- `IAutonomyRecorder`: every module reports ground truth, sensor frame metadata, estimates, map
  stats, decisions, arbiter actions and comms events through this one sink. The Unity adapter
  writes them into crane-drone-log/1 (`events.jsonl` kinds `autonomy_*`). The .NET harness writes
  JSONL. The recorder is the experiment harness's tap point.
- `ICommsLink`: a timestamped message bus with send/receive, drop and delay. For one drone it is
  a no-op. For two drones each stack gets an endpoint. Every message is also given to the
  recorder.
- `AutonomyConfig`: one serializable record holding every rate, latency, noise and gain. It is
  hashed into the run manifest.
- Units: velocities are m/s and accelerations m/s². The planner emits only kinematic setpoints. The
  force/torque conversion (mass, inertia) stays inside the flight controller. **Velocity gains
  and force gains are never mixed.**

### M2b flight-controller swap seam

`IFlightControllerLink` is the only path from autonomy to actuation.
- `InProcessControllerLink` drives `CraneMultirotorController` in velocity mode, with altitude and
  yaw closed on the setpoint. It runs at 500 Hz inside the plant and uses truth state.
- A PX4 implementation has the same signature. It must:
  - map a `TrajectorySetpoint` to `/fmu/in/trajectory_setpoint` (ENU to NED);
  - publish `offboard_control_mode` at 50–100 Hz;
  - report `FcStatus` from `vehicle_status`/`failsafe_flags`.
- For SITL lockstep, the plant-side `IAerialRotorCommandSource` swaps to
  `CranePx4LockstepBridge` (PX4_LOCKSTEP_PLAN.md). The autonomy side does not change.

### What transfers to the Jetson + PX4 split

These transfer as-is:
- module contracts and message fields;
- the stop rule (p99 age plus the simulated brake table);
- the arbiter and watchdog semantics;
- the latency budget accounting;
- the log schema.

These do not transfer:
- the in-process controller (PX4 replaces it);
- the ground-truth-plus-drift estimator (FAST-LIO2, DLIO or similar replaces it behind
  `IStateEstimator`);
- the raycast sensors (real drivers replace them behind `IDepthSensor`/`ILidarSensor`).

Timing semantics carry over: acquisition stamps, availability after latency, and receipt age at
the consumer.

### Interfaces the infrastructure/harness agent should build against

`IRayWorld`, `IGroundTruthWorld`, `IDepthSensor`, `ILidarSensor`, `IStateEstimator`,
`IOccupancyMap`, `IPriorMap`, `IGlobalRouter`, `ILocalPlanner`, `IFlightControllerLink`,
`IAutonomyRecorder`, `ICommsLink`, `AutonomyConfig`, `MissionSpec`, `MissionResult`,
`AutonomousMission` (one pure-C# closed-loop mission: plant, stack and metrics), all in
namespace `Crane.Drones.Autonomy`.

## Roadmap

| # | Milestone | Status |
|---|---|---|
| M0 | Architecture decision | DECIDED (this section) |
| M1 | Sensors: depth raycast image with stereo noise and thin-structure dropout; L2-like LiDAR with per-point time and motion skew; latency and stamps | IMPLEMENTED, FIXTURE_VALIDATED |
| M2 | Estimator: truth plus VIO-like drift (local random walk, separate global-alignment drift) behind `IStateEstimator` | IMPLEMENTED, FIXTURE_VALIDATED |
| M2b | FC integration path: in-process now, PX4 offboard or SITL later via `IFlightControllerLink` | seam defined; PX4 NOT_RUN |
| M3 | Map: sparse log-odds voxels, raycast clearing with conservative miss, window ESDF; prior from the manifest (stale/incomplete options) | IMPLEMENTED, FIXTURE_VALIDATED |
| M4 | Local planning: MAP_RHP 3D primitives, NavPilot safety layer (p99 age plus simulated brake table), REACTIVE fallback, corridor route plus wavefront cost-to-go | IMPLEMENTED; closed loop validated in M5 |
| M4b | Safety recovery: stall watchdog, brake-to-hover, "no safe maneuver" stop, recovery and replan after a near miss | IMPLEMENTED, FIXTURE_VALIDATED (arbiter test); exercised in every M5 run |
| M5 | End-to-end validation, ≥ 20 DEV flights at 2–3 m/s, then a speed sweep | DONE in .NET (24 flights at 2.5 m/s: 22/24 success, 0 collisions; 84-flight sweep). Unity collider world LIVE_SIM_VALIDATED partial (7/7 at report time, batch still running) |
| M6 | One rendered autonomous flight (onboard plus third-person stills) | PENDING (last step of the running Unity batch) |
| next | Reliable low speed → high-speed operation → two-drone missions (independent stacks, `ICommsLink` events) | planned |

## Implementation notes per milestone

All defaults live in `AutonomyConfig`. Every value is a [U] prior: NavPilot's numbers, vendor sheets, or first principles.

### M1 Sensors: IMPLEMENTED, FIXTURE_VALIDATED (tests below)

**`DepthRaycastSensor`**: OAK-D S2 stereo stand-in.
- Ray grid: 80x50 rays over 80x55 deg at 15 Hz, available 45 ms after acquisition.
- Each ray carries its decimated-pixel footprint, so thin structures still register. This is NavPilot's angular z-buffer idea.
  - Analytic world: the capsule radius grows by `beam * t`.
  - Unity: a sphere cast of the footprint at 4 m.
- Thin-structure dropout, per struck structure per frame: `P(valid) = logistic((w_eff - 3 px) / 0.8)`.
  - Native width `w = f D / r` at 640 px.
  - Motion blur `b = f (v_lat / r + |yaw rate|) t_exp`.
  - A dropped structure is seen **through**: the background behind it is returned, so free space is carved through the branch. This is the dangerous failure mode, and it is intentional.
- Disparity noise: `sigma_z = z^2 * 0.15 px / (f B)`.
- 3 % invalid pixels carry no information.
- A ray with no return within 15 m is free evidence out to 8 m.
- The pose is truth at acquisition. Rolling shutter is NOT modelled.

**`LidarL2Sensor`**: Unitree L2 stand-in.
- 64 kpts/s, 10 Hz sweeps.
- Non-repetitive pattern: golden-angle azimuth and equal-area elevation over [-48, +48] deg, rotated every sweep. The mount is an assumption.
- Each point is cast from the truth pose of the 5 ms tick it falls in.
- Each point is stored in the sensor frame of its own instant, with a per-point offset in ns. The scan therefore carries real motion skew.
- Range noise sigma 2 cm, 2 % point dropout, range 20 m.
- No-return rays are kept as miss directions, which give free evidence.
- Available 65 ms after the sweep ends.

**Worlds** (`IRayWorld`):
- `ForestManifestWorld`:
  - Holds every flight-band capsule from the manifest (trunk segments, branches, mid-story stems), plus logs and rocks.
  - Uses a 0.5 m 3D DDA grid.
  - Terrain is inverse-distance interpolated from the manifest's ground-anchored points. Unity's terrain is not in the manifest.
  - In Unity runs, terrain comes from `Terrain.SampleHeight`.
- `UnityColliderRayWorld`: PhysX colliders on layer Environment(9). The diameter of a struck structure is looked up from the nearest manifest capsule.

### M2 Estimation: IMPLEMENTED, FIXTURE_VALIDATED

`DriftingTruthEstimator : IStateEstimator`.
- Local drift: random walk of 0.03 m/sqrt(m) in x and y, stepped every 0.25 m of travel (NavPilot e2). Also 0.01 m/sqrt(m) in z and 0.0005 rad/sqrt(m) in yaw.
- Global-alignment offset: a separate, independent process (constant plus ramp), exposed as `MapAlignment`.
- Velocity: rotated by the yaw error, plus 0.02 m/s noise.
- Output: 100 Hz with 10 ms latency.
- Keeps a 2 s pose history for frame registration and LiDAR deskew.
- The map is built at the *estimated* poses.

### M2b FC seam: IMPLEMENTED (in-process); PX4 NOT_RUN

- `IFlightControllerLink.Submit(TrajectorySetpoint)` takes kinematic fields only.
- `InProcessControllerLink` puts the controller in velocity mode with a yaw target. The controller uses truth state, which stands in for PX4 EKF2 on good odometry.

### M3 Mapping: IMPLEMENTED, FIXTURE_VALIDATED

`SparseVoxelMap`:
- 0.2 m log-odds voxels in 16^3 chunks.
- Exact voxel traversal from the sensor to (hit - 0.15 m), capped at 12 m.
- NavPilot's conservative miss update, at most once per voxel per frame: `l_hit +0.85`, `l_miss -0.1`, clamped to [-1, 3.5], occupied above 0.4.
- LiDAR points are deskewed with the estimated pose at each point's stamp.
- At most the 3 newest frames per sensor per cycle.

`PlanningWindow`:
- A dense 22x22x5 m box around the estimate.
- Truncated exact distance field: voxel-centre metric, up to 0.6 m (an ESDF wherever the planner thresholds it).

`ManifestPriorMap`:
- Stems and limbs of at least 6 cm in the 0.5–4.5 m AGL band.
- Optionally stale (`PriorDropFraction` of trees removed) or misaligned (`PriorShiftX/Y`).
- The default `PriorUse = route` feeds the prior to the global cost-to-go only; local feasibility stays sensor-only.
- `belief` also injects it into the window.

### M4 Planning: IMPLEMENTED

**`PrimitivePlanner` (MAP_RHP, 3D)**, ported from NavPilot stacks/core.py `evaluate()`:
- Primitives: 48 headings x {-12, 0, +12} deg climb x {1, .75, .5, .3, .15} v_cap. That is 720 velocity commands, each with altitude hold toward the route altitude.
- Each is forward-simulated through a first-order velocity loop (tau 0.33 s, |a| ≤ 0.8 of the tilt/accel limit) for `clip(H / v, 2, 8)` s.
- `d_free` is the arc length to the first blocked sample, extended straight to H = 14 m. A sample is blocked if:
  - its ESDF distance is below `r_body + 0.15 + res/2`, or
  - it is in unknown space, or
  - it is outside the window, or
  - it is outside the altitude band.
- NavPilot's inflation-blob exception applies near the start.
- Feasible iff `StopRule.Dist(max(v_cmd, |v|)) <= d_free`.
- Score (NavPilot): `v g + 0.15 v_cap g - 0.35 |dheading| |v| + 0.6 min(d, H)/H - 0.4 |dz_end| / 2`. Here `g` is the cost-to-go progress per metre at 4 m.

**`StopRule`**:
- `tau` is the rolling p99 of measured perception-to-command age over 200 cycles. The prior is 0.12 s until 20 samples arrive.
- The brake table D(v) is **simulated on the real plant and controller**: cruise, then a zero-velocity command. For example, 2.5 m/s gives 0.94 m and 6 m/s gives 3.05 m.

**REACTIVE**: same planner, but the map is rebuilt every cycle from only the newest frames, with an 8 m horizon, goal-bearing score and no climb set (`Planner = reactive`).

**`CorridorRouter`**:
- Mission route (the corridor waypoints, or a straight cross-forest line) mapped to the local frame with the believed alignment.
- Plus a 0.5 m 8-connected Dijkstra cost-to-go to the goal every 0.5 s.
- Blocked cells come from window voxels in the ±0.8 m band around the route altitude, plus prior columns.

Compute time is sampled per cycle from NavPilot's "nominal" tail (median 25 ms, Pareto stalls). The command takes effect at `t + c`.

### M4b Safety recovery: IMPLEMENTED

`SetpointArbiter`:
- **Stall watchdog:** no fresh command for 150 ms while moving → brake to hover.
- **Estimate stale:** input age over 100 ms → brake.
- **No feasible primitive** ("neither source supports a safe maneuver") → brake to hover. Yaw keeps turning toward the most open direction, for coverage.
- **Belief near miss:** ESDF at the estimate below `r_body + 0.15` → recovery, at 0.4 x speed for 1 s with an immediate cost-to-go recompute.
- Each action is recorded as an `autonomy_arbiter` event.

## M5 End-to-end validation (2026-11-10)

### Setup

- **Forest:** DEV seed **1337**, the only generated Drone Forest. See "Limits" for why no other DEV seed was built.
- **Missions** (`MissionCatalog`, identical in both harnesses), 12 in total:
  - `corridor_fwd` and `corridor_rev`: the 119 m CorridorWaypoint polyline. It is cleared of trunks within 1.3 m; branches and mid-story are *not* cleared.
  - `cross_0..9`: seeded straight routes of at least 35 m between interior points, cutting through uncleared forest at 2.2 m AGL. These are the hard cases.
- **Stack:** MAP_RHP, `PriorUse = route` (the prior is used only for the global cost-to-go), depth and LiDAR, nominal compute tail, calm wind.
- **Success:** within 1.5 m (xy) of the goal.
- **Collision:** ground-truth manifest clearance below the body radius (0.268 m, max rotor hub plus prop) against any capsule with r ≥ 1 cm. Thinner twigs count as twig contacts and are not terminal (as NavPilot).
- **Stuck:** less than 1 m of along-route progress in 12 s.

There are two harnesses with the same stack and missions:
- **(A) .NET, analytic manifest world:** sensors raycast the manifest capsules plus the interpolated terrain.
- **(B) Unity batch:** sensors raycast the scene's PhysX colliders, with Unity terrain. Ground truth is still the manifest. Every run is logged as crane-drone-log/1 (state 200 Hz, IMU 500 Hz, `autonomy_*` events).

### Validation table, v_lim = 2.5 m/s, run seeds 1–2 (24 flights per harness)

**(A) .NET manifest world, 24 flights:**

| v_lim | n | success | 95% CI (Wilson) | collisions | stuck | progress mean | min hard clearance, min / p10 [m] | twig contacts / run | mean speed [m/s] | perception→command age p50 / p99 / max [ms] | no-safe-maneuver stops / run | watchdog trips / run | true stopping-margin violations [% of moving ticks] |
|---|---|---|---|---|---|---|---|---|---|---|---|---|---|
| 2.5 | 24 | **22/24** | 0.74–0.98 | **0** | 2 | 0.96 | 0.20 / 0.24 | 0.2 | 2.24 | 96 / 139 / 234 | 3.1 | 0.4 | 0.02 |

Both failures are `stuck` (local minimum):
- `cross_5` seed 1 stopped at 84 %.
- `cross_1` seed 2 stopped at 21 %. It oscillated for 12 s in a pocket at about (5, 39) with 145–240 of 720 primitives feasible and d_free of 1.5–3 m.

Cause: the 0.5 m cost-to-go grid treats raw occupancy as blocked **without body inflation**. It routes through gaps narrower than the 0.84 m the vehicle needs, and the local planner then cannot follow. Fix (not applied, so results stay consistent): inflate or penalize cost-to-go cells by `r_body + 0.15`, and add a route-level escape (backtrack along the travelled path).

**(B) Unity collider world (in progress at report time; partial):**

At report time **7 of 24 flights were complete: 7/7 success, 0 collisions** (seed 1 corridor_fwd, corridor_rev, cross_0..4).

Run status:
- The batch (one `--render` invocation) is still running.
- It continues with the remaining 17 flights at 2.5 m/s, then the sweep at 4 and 6 m/s (12 missions each), then M6.
- Results append to `/tmp/autonomy_unity/m5/summary.jsonl`. Each run's log is in `/tmp/autonomy_unity/m5/auto_s1337_<mission>_map_rhp_v<v>_r<seed>/`.
- Re-summarize with `python3 Assets/Scripts/Drones/Autonomy/Harness~/summarize.py /tmp/autonomy_unity/m5/summary.jsonl --runs`.

The Unity world runs about 2.8x slower than real time (113–178 s of wall time per 40–57 s flight).

`cross_k` in Unity is not necessarily the same route as `cross_k` in .NET. The start and goal acceptance test uses the terrain model: Unity terrain versus the interpolated one.

| VLimit | n | success | 95% CI | collisions | stuck/timeout | progress mean | min hard clearance (min / p10) [m] | twig contacts / run | mean speed [m/s] | age p50 / p99 / max [ms] | no-safe stops / run | watchdog / run | margin viol. [%] |
|---|---|---|---|---|---|---|---|---|---|---|---|---|---|
| 2.5 | 8 | 8/8 | 0.68-1.00 | 0 | 0 | 1.00 | 0.21 / 0.24 | 0.1 | 2.18 | 99 / 138 / 212 | 4.2 | 0.8 | 0.01 |

| mission | v_lim | seed | outcome | time [s] | progress | min hard clr [m] | mean v | age p99 [ms] | no-safe | wd | recov | note |
|---|---|---|---|---|---|---|---|---|---|---|---|---|
| corridor_fwd | 2.5 | 1 | success | 43.9 | 1.00 | 0.45 | 2.40 | 137 | 2 | 1 | 0 |  |
| corridor_rev | 2.5 | 1 | success | 50.0 | 1.00 | 0.46 | 2.27 | 137 | 3 | 1 | 1 |  |
| cross_0 | 2.5 | 1 | success | 57.0 | 1.00 | 0.26 | 1.88 | 137 | 4 | 1 | 2 |  |
| cross_1 | 2.5 | 1 | success | 46.0 | 1.00 | 0.31 | 2.01 | 137 | 5 | 1 | 1 |  |
| cross_2 | 2.5 | 1 | success | 52.4 | 1.00 | 0.36 | 2.18 | 137 | 7 | 1 | 1 |  |
| cross_3 | 2.5 | 1 | success | 20.6 | 1.00 | 0.47 | 2.35 | 138 | 0 | 0 | 0 |  |
| cross_4 | 2.5 | 1 | success | 41.9 | 1.00 | 0.21 | 1.96 | 138 | 11 | 1 | 3 |  |
| cross_5 | 2.5 | 1 | success | 21.9 | 1.00 | 0.55 | 2.38 | 137 | 2 | 0 | 0 |  |

Label: **LIVE_SIM_VALIDATED (partial, n = 7)** for low-speed flight in the Unity collider world.


### Speed sweep (A, .NET manifest world, 12 missions x run seed 1 per speed)

| VLimit | n | success | 95% CI | collisions | stuck/timeout | progress mean | min hard clearance (min / p10) [m] | twig contacts / run | mean speed [m/s] | age p50 / p99 / max [ms] | no-safe stops / run | watchdog / run | margin viol. [%] |
|---|---|---|---|---|---|---|---|---|---|---|---|---|---|
| 2 | 12 | 12/12 | 0.76-1.00 | 0 | 0 | 1.00 | 0.22 / 0.30 | 0.0 | 1.81 | 98 / 141 / 212 | 2.2 | 1.0 | 0.00 |
| 2.5 | 12 | 11/12 | 0.65-0.99 | 0 | 1 | 0.99 | 0.24 / 0.26 | 0.0 | 2.24 | 99 / 139 / 212 | 3.2 | 0.8 | 0.02 |
| 3 | 12 | 10/12 | 0.55-0.95 | 0 | 2 | 0.90 | 0.20 / 0.24 | 0.4 | 2.48 | 99 / 140 / 212 | 3.0 | 0.7 | 0.03 |
| 4 | 12 | 12/12 | 0.76-1.00 | 0 | 0 | 1.00 | 0.11 / 0.19 | 1.1 | 3.38 | 99 / 141 / 212 | 4.8 | 0.6 | 0.13 |
| 5 | 12 | 9/12 | 0.47-0.91 | 2 | 1 | 0.86 | -0.01 / 0.02 | 2.6 | 3.80 | 100 / 140 / 212 | 5.5 | 0.2 | 0.51 |
| 6 | 12 | 6/12 | 0.25-0.75 | 5 | 1 | 0.80 | -0.04 / -0.02 | 3.6 | 4.44 | 100 / 141 / 212 | 5.7 | 0.2 | 1.38 |
| 8 | 12 | 7/12 | 0.32-0.81 | 4 | 1 | 0.77 | -0.04 / -0.03 | 3.6 | 5.05 | 102 / 138 / 212 | 5.0 | 0.1 | 1.39 |

The 2.5 m/s row is seed 1 only.

**Where reliability drops:**
- Up to 4 m/s there are **no collisions** (0 in 48 flights; stuck failures only).
- Minimum hard clearance shrinks with speed: 0.30 m at 2 m/s, 0.24 at 2.5, 0.11 at 4.
- True stopping-margin violations rise from ≤ 0.03 % to 0.13 % of moving ticks.
- **Collisions start at v_lim = 5 m/s** (2/12). They are frequent at 6 (5/12) and 8 (4/12).
- At those limits the mean achieved speed is only 3.8–5.1 m/s: the stop rule throttles.

**What was struck** (11 collisions in total, all grazing, min clearance −0.01 to −0.04 m):
- Limbs and trunk segments of r 1.1–10 cm (e.g. tree_426 seg r = 10.4 cm, tree_748 r = 7.8 cm, tree_590 r = 8.1 cm).
- Thin limbs of r 1.1–1.7 cm.
- One young-conifer stem.
- Impact speeds were 0.7–5.5 m/s.
- The margin-violation rate at 5–8 m/s (0.5–1.4 %) shows the belief and the truth disagreeing near obstacles.
- Leading hypotheses, NOT yet attributed: inflation of only 0.15 m plus half a voxel at 0.2 m resolution; thin-limb see-through under motion blur; ESDF truncation at 0.6 m.

**Conservative operating point for the shared infrastructure, on this forest: v_lim ≤ 4 m/s** (0/48 collisions, 95 % upper bound about 7.4 %).


## M6 Rendered flight

NOT YET PRODUCED at report time. The render pass is the **last** step of the running Unity batch.

What it renders:
- the hero AOS7 quad posed from the plant, with its colliders disabled so it never blocks sensor rays;
- the onboard OAK-D RGB through `RenderProfiles` onboard only (leak check enforced);
- a third-person chase camera through the cinematic volume only;
- still pairs at t = 6, 18 and 32 s of `corridor_fwd` at 2.5 m/s.

Output path: `/tmp/autonomy_unity/m5/render_corridor_fwd/{onboard_rgb,third_person}_t*.png`. The run is also logged as `auto_s1337_corridor_fwd_map_rhp_v2.5_r1_render/`.

If the batch is stopped first, run the render only (one invocation):

```
unity-drones --render LOG -executeMethod Crane.Drones.Autonomy.AutonomyBatchCli.Run --drone-seed 1337 \
  --autonomy-out DIR --autonomy-render-only --autonomy-render-mission corridor_fwd --autonomy-render-speed 2.5
```


## Tests

`Assets/Tests/Editor/DroneAutonomy/DroneAutonomyTests.cs`, assembly `Crane.Drones.Autonomy.Tests` (10 tests). The tests cover:
- depth range and latency;
- thin-twig dropout against a trunk;
- LiDAR per-point stamps, plus skew present (≥ 0.15 m at 3 m/s) and removed by deskew (< 5 cm);
- estimator drift ~ sigma sqrt(d), with the alignment kept separate;
- map hit, free and shadow;
- p99 tau and a monotone simulated brake table;
- the arbiter's watchdog, no-safe-maneuver and stale-estimate brakes;
- no feasible primitive at 6 m/s, 1.3 m from a wall;
- a closed-loop fixture mission through 14 trunks;
- config hash stability.

Status:
- **All 10 pass** under the .NET shim (`Harness~/TestShim`, a minimal NUnit shim): FIXTURE_VALIDATED.
- The Unity build compiles them with 0 errors (Unity log of the M5 batch).
- A Unity EditMode run (`-testFilter Crane.Drones.Autonomy.Tests`) was NOT_RUN, to save the shared Unity slot.

## Limits, stubs and honest caveats

- **One forest.** `BuildDroneForest.Build` for a new seed would also delete and recreate the shared lighting profiles (`ForestLighting.Build` → `Profiles/*.asset`), which carry uncommitted visual-agent work. So no other DEV forest was built.
  - Generalization across forests is **UNKNOWN**.
  - Needed: a manifest-only (or profile-preserving) build entry point from the environment owner. `MissionCatalog` and both harnesses take any manifest.
- **Perception-to-command age p99 ≈ 138 ms exceeds the AUTONOMY_DESIGN 120 ms gate.** This is structural, not stalls: 15 Hz stereo, 45 ms latency, the 20 Hz cycle and a 25 ms median compute. The stop rule uses the measured p99, so it is safe, but the gate fails as configured.
- **Estimator:** truth plus drift only (no VIO/LIO). The controller is in-process on truth state (no PX4). No wind (calm). No rolling shutter. Sensor images and LiDAR point files are not written to the logs (events carry per-frame metadata).
- **Terrain:** in the .NET world it is interpolated from manifest anchors. Branches above 5.5 m AGL and foliage are not in the manifest, so they are neither obstacles nor ground truth.
- **Not exercised in M5:** `PriorUse = belief`, a stale or shifted prior, REACTIVE, and multi-drone (`ICommsLink` is only `LocalCommsBus`).
- **Speed:** the Unity collider world runs about 2.6–2.9x slower than real time, and .NET about 1.5x.

## Next steps (ordered)

1. Cost-to-go inflation and a backtrack escape, to remove the low-speed `stuck` failures. Re-run M5 in both harnesses.
2. Environment owner: a manifest-only forest build for DEV seeds. Then M5 on 10 or more forests.
3. Speed: map the collision onset (see the sweep). Candidate causes to test in order:
   - (a) inflation (0.15 m + half voxel) against grazing contacts on 1–10 cm limbs;
   - (b) thin-limb see-through dropout under blur at speed;
   - (c) the age term at 138 ms.
   Add NavPilot's collision-cause labels (mapped / detected / in-view at t_need) to `MissionResult`.
4. Bring the age p99 under 120 ms: 30 Hz depth or a lower-latency stereo profile, and an event-driven planner on frame arrival.
5. Fault-injection suite for M4b (stall bursts, estimator dropouts, a stale prior), with false-abort and missed-abort counts.
6. PX4 SITL behind `IFlightControllerLink` (PX4_LOCKSTEP_PLAN), then two-drone missions with `ICommsLink` events.

## How to run

**.NET harness (no Unity):**

```
cd Assets/Scripts/Drones/Autonomy/Harness~ && dotnet build -c Release -o /tmp/autonomy_build
nice -n 19 dotnet /tmp/autonomy_build/AutonomyHarness.dll \
  --manifest Assets/Drones/Environment/Manifests/forest_seed1337_manifest.json \
  --profile Tools/Aerial/profiles/aos7_v5_f90_6s.json --out /tmp/ah --cross 10 --seeds 1,2 --speeds 2.5 [--events] [--set Field=Value]
```

The tests outside Unity live in `Harness~/TestShim`: `dotnet build -c Release -o /tmp/autonomy_tests && dotnet /tmp/autonomy_tests/TestShim.dll`.

On 2026-11-10 the .NET runs used `nice -n 19` directly, with at most 4 processes. All three `fsrun` slots were held by other studies. These runs are C#, not Python.

**Unity batch** (one invocation):

```
unity-drones --render LOG -executeMethod Crane.Drones.Autonomy.AutonomyBatchCli.Run --drone-seed 1337 \
  --autonomy-out DIR --autonomy-cross 10 --autonomy-speeds 2.5 --autonomy-seeds 1,2 [--autonomy-sweep-speeds 4,6] \
  [--autonomy-render-mission corridor_fwd]
```

Raw results from this report:
- `/tmp/ah_m5a`, `/tmp/ah_m5b`: .NET M5.
- `/tmp/ah_swA`, `/tmp/ah_swB`: .NET sweep.
- `/tmp/autonomy_unity/m5`: Unity.
