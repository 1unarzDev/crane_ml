# CRANE experiment harness: specs, DR, failure labels, replay, fidelity

_Owner: infrastructure agent (research/drone-sim worktree), 2026-11-10. Evidence labels follow
Docs/ResearchOrchestration/CONTRACTS.md. Nothing here is PHYSICALLY_CALIBRATED._

The harness drives the autonomy lead's stack (`Crane.Drones.Autonomy`, `AutonomousMission`) through its **public
surface only**. Core was not edited. Every fault, wind, comms and probe hook goes through an existing seam:
- the `IRayWorld` sensor world;
- the `DepthAcquired` / `LidarCompleted` callbacks;
- the run-private `AutonomyConfig` (the estimator and sensors re-read it every tick);
- `StepPlant`;
- `ICommsLink`;
- `IAutonomyRecorder`.

| Part | Where | Label |
|---|---|---|
| Spec, guard, DR, faults, wind, comms, runner, labeller, replay bundle (pure C#, `noEngineReferences`) | `Assets/Scripts/Drones/Harness/` (asmdef `Crane.Drones.Harness`) | IMPLEMENTED, FIXTURE_VALIDATED (.NET) |
| NUnit tests (Unity asmdef `Crane.Drones.Harness.Tests`, run outside Unity through a shim) | `Assets/Scripts/Drones/Harness/Tests/` | 7/7 PASS (.NET shim); Unity compile NOT_RUN (slot/memory gate, see below) |
| .NET CLI (`run`, `guard`, `replay`, `test`, `test-lead`, `export-navpilot`) | `Assets/Scripts/Drones/Autonomy/Harness~/Experiment/` | IMPLEMENTED |
| Aggregation (paired cluster bootstrap), replay plot, NavPilot 2.5D twin, fidelity report, DR registry | `Tools/CraneHarness/` | IMPLEMENTED; Python tests 6/6 |
| Unity backend for harness specs | not built: the lead's `AutonomyBatchCli` is the Unity path today | NOT_RUN |

## 1. Experiment spec (`crane-experiment/1`)

```json
{
  "schema": "crane-experiment/1", "name": "dev_dr_demo", "split": "dev",
  "forests": [1337],                       // or [{"seed": S, "manifest": PATH}], "manifest_pattern": ".../forest_seed{seed}_manifest.json"
  "missions": {"cross": 10, "corridor": true, "only": ["cross_3"]},   // MissionCatalog (identical in Unity and .NET)
  "speeds": [4.0], "run_seeds": [1, 2], "planner": "map_rhp",
  "config": {"InflateExtra": 0.25},        // any AutonomyConfig field (kinematic gains only; force gains live in the FC)
  "dr": {"family": "dev_stress", "samples": 2, "seed": 11},
  "wind": "calm", "lighting": "GoldenHour",
  "faults": [{"kind": "sensor_delay", "sensor": "all", "t0": 6.0, "t1": 6.6, "extra_s": 0.35}],
  "comms": {"drop_p": 0.05, "delay_s": 0.03, "jitter_s": 0.02, "heartbeat_hz": 2, "failsafe_timeout_s": 0, "outages": [[4, 7]]},
  "n_drones": 2, "logs": "events", "label_failures": true, "bundle_failures": true, "allow_heldout": false
}
```

**Run expansion.**
- One run is generated per (forest × speed × run seed × mission × DR sample).
- Each run is written as a self-contained `run_spec.json` (`crane-run-spec/1`).
- Run id: `{exp}-f{forest}-{mission}-v{v}-r{seed}-d{dr}`.
- `pair_key` is (forest, mission, run seed, DR sample). It is speed- and condition-independent, so conditions pair.
- `cluster` is (forest, mission).

**Logs** (`logs`):
- `events`: `manifest.json` and `events.jsonl` (crane-drone-log/1 manifest fields; `autonomy_*`, `comms_*` and `collision` events, each with a `drone` tag).
- `full`: adds a 200 Hz `state.csv`, which makes the run a valid crane-drone-log/1 directory for `Tools/DroneData/drone_log.py`.
- Peers in multi-drone runs log to `peer{k}/`.

**N drones.**
- Independent stacks step in lockstep on one clock.
- Drone k flies the catalog mission k positions after the primary.
- Drones do not sense each other: the static world is the only obstacle set. The minimum inter-drone separation is reported.
- A lossy link (`LossyCommsLink`) carries GCS heartbeats and pose broadcasts.
- `failsafe_timeout_s > 0` aborts a drone after that long without a GCS heartbeat. This is PX4 datalink-loss semantics; the outcome is `comms_abort` with label `comms`.

**Faults** (`FaultInjector`):

| Fault | Effect |
|---|---|
| `sensor_blackout` | Frames never become available |
| `sensor_delay` | Availability is pushed back (perception stall) |
| `drift_burst` | Estimator drift sigma is scaled up |
| `align_jump` | Global-alignment step |
| `depth_invalid` | Invalid-pixel probability is raised |
| `thin_blind` | Thin-structure w50 is raised |

Planner compute stalls are randomized through the latency-tail config fields (`PlanPStall`, `PlanStallMinS`, ...). A scheduled stall would need a Core hook.

**Wind.**
- `WindField`: mean wind plus `CraneDrydenTurbulence` evaluated at **true AGL** from the manifest terrain.
- It is applied through `StepPlant` every 5 ms.
- It is not the plant's built-in turbulence. The .NET plant runs with `GroundHeight = -1000`, so the built-in model would use roughly 1000 m scales.

## 2. Split guard and domain randomization

**Guard** (`SplitGuard`, the C# twin of `Tools/DroneData/splits.py`). It fails closed and runs before anything executes or is written. One experiment uses one split. It refuses:
- forest seeds outside every range, or mixed across splits;
- a DR family registered to another split;
- wind or lighting values that are not values of the experiment's split;
- held-out seeds, winds, lightings and families, unless both the spec (`allow_heldout`) **and** the CLI (`--allow-heldout`) opt in;
- any resolved number inside a held-out **reserved interval** (`heldout_reserved` in `dr_families.json`). This covers every DR draw and every explicit config override, so nobody can hand-tune into the held-out tails.

Replay re-checks the guard.

**DR registry.** `Tools/CraneHarness/dr_families.json` (`crane-dr-families/1`) is consistent with splits.json. Every dimension is logged in the run spec, the results row and the log manifest. Draws are keyed on (dr seed, forest, mission, run seed, sample), never on speed, so paired conditions see the same draw.

| family | split | dimensions |
|---|---|---|
| dev_nominal | dev | disparity noise, invalid px, thin-branch w50, LiDAR dropout, stall probability, local drift, wind (calm / light_breeze) |
| dev_stress | dev | thin w50 3–4 px, exposure (motion blur), depth latency, stall probability and size, drift, branch thinning, wind |
| train_wide | train | all sensor/latency/drift dimensions plus stem and branch thinning, wind (calm / light_breeze / gusty) and direction, lighting (GoldenHour / Overcast) |
| val_shift | val | shifted noise, thin w50 and stall tail; moderate_gusty, Dusk |
| test_shift | test | drift and latency shift; strong_steady, Noon |
| **heldout_tails** | heldout | thin w50 4.2–5.5 px, disparity noise 0.4–0.8 px, stalls (p 0.03–0.08, 0.15–0.4 s), drift 0.1–0.25 m/√m, branch keep ≤ 0.3; storm_gust / turbulent_shear; LowSunGlare / HeavyFog |

Wind presets map the splits.json wind names to an under-canopy mean and a Dryden W20 [U].

Lighting is logged and guarded only. It is for later vision runs and has no effect in the raycast sensors.

**Forest density.**
- Stem and branch density can only be *thinned* on one manifest (`ForestThinning`: removed capsules get radius −1000 m on a run-private world). This works on the .NET world only.
- Denser forests need new manifests. `BuildManifestOnly` does not exist yet: no match in any worktree on 2026-11-10. Once the environment owner adds it, list the new seeds in `forests` and nothing else changes.

## 3. Failure labels and replay

**Labelling** happens after every failed run:
- The harness re-executes the run deterministically from its `RunSpec`, with a pass-through `ProbeRayWorld` watching the struck capsules.
- It checks that the same outcome happens at the **same tick** against the **same object** (`replay_reproduced`).
- It then labels the failure (first matching rule) and writes `replay_bundle.json`.

Rules: `t_need = t_contact − StopRule.Dist(v_recent) / v_recent`.

| label | rule |
|---|---|
| `comms` | datalink-loss failsafe abort |
| `latency_stall` | mapped by t_need, and a deadline miss, watchdog trip or injected perception stall in the last 1 s (or detected but not yet mapped while stalled) |
| `thin_branch_miss` / `map_lag` | mapped by t_need, then carved free before contact (thin: D < 4 cm); or detected but not mapped (`map_lag`) |
| `brake_infeasible` | mapped and kept, and a no-safe-maneuver brake began within 1.5 s of contact at v > 1 m/s with true clearance below the brake distance D(v) |
| `estimator_drift` | mapped and kept, and the local error changed by more than the inflation budget (InflateExtra + res/2 = 0.25 m) between first mapping and contact; or stuck with local error > 1 m |
| `geometry_graze` | mapped and kept, and the belief ESDF at the estimate was still ≥ body radius at contact (quantization or inflation under-represents the surface) |
| `tracking_lag` | mapped and kept, the belief knew it was inside the body radius, the vehicle did not stop |
| `thin_branch_miss` / `sensor_miss` | in view by t_need but never detected (every stereo return dropped, no LiDAR return) |
| `occlusion` | not in view by t_need |
| `local_minimum` | stuck (< 1 m progress in 12 s) or timeout |

**Replay bundle** (`crane-replay-bundle/1`):
- run spec, evidence and the 20 Hz truth and estimated trajectories;
- belief voxels (flight band ±0.7 m, 14 m box) mapped to world;
- truth capsules in the same box;
- the last planner decision and arbiter action;
- the last 5 s of per-cycle probe rows (mapped, belief distance, mode, τ, speed, true clearance);
- the last 3 s of events.

`Tools/CraneHarness/plot_replay.py` draws the top-down plot (`replay_topdown.png`).

**Run the replay for a run id:**

```
dotnet /tmp/crane_exp/CraneExperiment.dll replay --out /tmp/crane_harness/attrib_v6 --run-id attrib_v6-f1337-cross_3-v6-r1-d0
```

**Determinism evidence:**
- **FIXTURE_VALIDATED:** two runs of a fixture are bit-identical (path length and event-stream digest).
- **Collision fixture:** a thin-curtain collision replays to the same tick and object, labelled `thin_branch_miss`.
- **Stuck fixture:** an enclosed goal replays as `local_minimum`.
- **Comms fixture:** a datalink outage replays as `comms`.
- **Real manifest:** all **11/11** of the lead's colliding sweep runs re-executed through the harness to the **same collision time and object** as in `/tmp/ah_sw{A,B}` (e.g. tree_344/seg267910 at 2.880 s). Every replay reproduced (`replay_reproduced = true`). So the harness neither perturbs nor reorders the stack.

## 4. Collision attribution for the lead's speed sweep (v_lim ≥ 5 m/s)

**Data.** 11 collisions: v5 2, v6 5, v8 4, run seed 1, DEV forest 1337, calm, default config. Run dirs: `/tmp/crane_harness/attrib_v{5,6,8}/runs/*`, with bundles and `replay_topdown.png`.

**Labels.**

| labeller pass | breakdown |
|---|---|
| First pass, as run (no `brake_infeasible` rule yet; drift threshold 0.15 m) | estimator_drift 5, tracking_lag 4, latency_stall 1, geometry_graze 1 |
| Refined rule, re-derived offline from the bundles' per-cycle rows (the C# labeller now implements it; a fresh re-run is stage 5 of `run_pending.sh`) | **brake_infeasible 9**, latency_stall 1, tracking_lag 1 |

Why the first pass was misleading: its drift labels sat just above an arbitrary 0.15 m cut (Δe 0.16–0.21 m), below the 0.25 m inflation budget.

**What the replays show:**

1. **No thin-branch see-through, no map lag.** Every struck object was **in the map before t_need and still in the map at contact**:
   - first mapped 0.7–10 s before contact;
   - the stereo model did drop up to half of the thin-limb returns (e.g. 2630 of 5520 for a 2.6 cm conifer stem), but LiDAR returns kept the limb in the map.

   The lead's hypothesis (b) is rejected as the primary cause.
2. **Latency is secondary.** One run (cross_1 at 6 m/s) had a deadline miss or watchdog trip in the last second. Even there the same brake pattern appears. The perception-to-command age p99 is about 141 ms at every speed.
3. **The dominant mechanism is an inevitable-collision state.** In 10/11 runs:
   - the planner went from "feasible primitive exists" to "no feasible primitive" in one cycle;
   - this happened 0.25–0.56 s before contact, at 4.4–7.9 m/s;
   - true clearance at that moment was 0.55–1.58 m, against a brake distance D(v) of 1.95–4.26 m;
   - the belief roughly agreed with the truth (belief ESDF saturated at 0.8 m in 8/10, true clearance about 1 m).

   The arbiter then commands **brake to hover along the current velocity**, which decelerates on a straight line. The drone hits at 0.7–5.5 m/s (median about 2.3 m/s). That explains the "grazes": the contacts are shallow (−1 to −4 cm) because the vehicle is decelerating, and often turning, at contact.

   Cause: feasibility was certified along **curved** primitives (`d_free ≥ Dist(v)` along the rollout). Neither the fallback nor the next cycle is held to that certificate:
   - (i) the brake does not follow the last certified primitive;
   - (ii) the next cycle's reachable set at 5–8 m/s through 1–2 m gaps can be empty;
   - (iii) the rollout model (first-order τ 0.33 s, 0.8·a_max) is more agile than the real plant's braking in a turn.
4. **Inflation (hypothesis a) is not the trigger, but it sets how often the trap is entered.** The stack only commits to gaps where the margin r_body + 0.15 + res/2 = 0.50 m holds in belief.

These are **mechanism findings** from deterministic replays (LIVE_SIM-free, .NET manifest world). The counterfactual ablations are written but **NOT_RUN** because of the CPU cap (`specs/cf_{nodrift,nodrop,infl25,fastperc}_v{5,6,8}.json`, stage 1 of `run_pending.sh`). They will say how many of the 11 disappear when one factor is removed.

**Suggested fixes for the lead** (not applied; Core is the lead's):
- the no-safe-maneuver fallback should track the last certified primitive's braking profile, not a zero-velocity step;
- require feasibility of the *next-cycle* set: certify a primitive only if its endpoint state still has a feasible stop (a two-step / backup-trajectory check, as in OPT_BACKUP or FASTER);
- use a brake table that includes turning.

## 5. Fidelity validation: NavPilot 2.5D vs CRANE 3D

**Same scenario family.**
- `export-navpilot` writes each straight `cross_k` mission of the forest manifest as a `crane-navpilot-stand/1` stand in the route frame:
  - vertical capsules crossing the slab become stems (the thickest per owner);
  - every other capsule in the 1–3 m AGL band becomes a branch, with heights converted to AGL;
  - typically 160–450 stems and 11k–30k branch capsules per route.
- `navpilot_bridge.py` runs a trimmed copy of NavPilot's E2 episode loop (E2 file not edited) with the **MAP_RHP** stack. CRANE conventions were adopted so that only 3D vs 2.5D differs:
  - CRANE body radius;
  - CRANE success, stuck and time-limit rules;
  - prior = stems ≥ 6 cm, no phantoms or drift ramps;
  - nominal latency tail, calm wind;
  - the true stopping-margin metric and every age sample are logged.
- `fidelity.py` compares the two (the CRANE side being the lead's sweep rows, bit-identical through the harness), per speed:
  - success and collision rates (cluster bootstrap, cluster = mission);
  - the **paired** CRANE − NavPilot difference over the same missions;
  - age p50/p99 and margin-violation fraction;
  - min hard clearance and mean speed.

**Status: NOT_RUN** beyond a smoke test. The orchestrator capped CPU before the 70-run NavPilot sweep could start; it is stage 2/3 of `run_pending.sh`.

Smoke (cross_4, 4 m/s, seed 1):

| | outcome | time | perception→command age p99 |
|---|---|---|---|
| NavPilot | success | 18.8 s | 133 ms |
| CRANE (lead row) | success | — | 141 ms |

**Expected divergences**, stated before the data (not results):
- NavPilot sees every branch only where it crosses a ±0.35 m slab and cannot climb. It should have **more** stuck failures in branchy routes and **fewer** grazes on oblique limbs that CRANE meets in 3D.
- Its point-mass brake is shorter than the RK4/cascade brake (NavPilot brake table vs CRANE `SimulateBrakeTable`), so the CRANE collision onset should be at a **lower** v_lim.
- Age distributions should agree within about 10 ms: same rates, latencies and compute tail.
- Margin violations: the CRANE metric is per 20 ms tick, the NavPilot metric per planner cycle. Both are fractions of moving samples.

**Hardware validation plan** (real AOS7; no live hardware was commanded):

| log from the real vehicle | calibrates |
|---|---|
| Brake tests: straight-line stops from 2/4/6/8 m/s (PX4 ulog `vehicle_local_position`, `trajectory_setpoint`), plus braking out of a 30–60° turn | `StopRule` brake table (currently simulated); the straight-vs-turning brake gap behind `brake_infeasible`; NavPilot `tau_track` / `jerk_max` |
| Latency profiling: camera exposure timestamp → depth message → planner output → `trajectory_setpoint` (OAK-D device clock, ROS 2 stamps, `uorb` `trajectory_setpoint.timestamp`) | `DepthLatencyS`, `LidarLatencyS`, `PlanMedianS` / `PlanSigma`, and the stall tail `PlanPStall` / `PlanStallMinS` / `PlanStallAlpha`; the p99 τ in the stop rule |
| Static thin-branch targets (dowels of 6–40 mm at 2–10 m, at several lateral speeds, OAK-D S2 depth) | `ThinW50Px`, `ThinWScalePx`, `ExposureS` (blur), `DispSigmaPx`, `InvalidP` |
| L2 point clouds against the same targets | `LidarDropP`, `LidarRangeSigma`, effective points per second at range |
| VIO/LIO drift on loops with RTK or total-station truth under canopy | `LocalDriftSigma`, `YawDriftSigma`, `VelNoiseSigma`, alignment ramp |
| Hover and gust response with an anemometer under canopy | wind presets (mean, W20), Dryden scale lengths at AGL |
| Datalink RSSI and loss logs in forest | comms `drop_p`, `outages`, failsafe timeout |

## 6. Commands

```
# build + tests (no Unity)
cd Assets/Scripts/Drones/Autonomy/Harness~/Experiment && dotnet build -c Release -o /tmp/crane_exp
dotnet /tmp/crane_exp/CraneExperiment.dll test
fsrun-quick /home/lunarz/.cache/research/fs-venv/bin/python -I -m unittest discover -s Tools/CraneHarness -p 'test_*.py'
# experiment (long lane, ONE process)
fsrun dotnet /tmp/crane_exp/CraneExperiment.dll run --spec Tools/CraneHarness/specs/dev_dr_demo.json --out /tmp/crane_harness/dev_dr_demo
python Tools/CraneHarness/aggregate.py /tmp/crane_harness/dev_dr_demo --by speed [--pair speed 4 6]
python Tools/CraneHarness/plot_replay.py RUN_DIR/replay_bundle.json
# everything still pending (counterfactuals, fidelity sweep, DR smoke, relabel)
fsrun Tools/CraneHarness/run_pending.sh
```

**Unity.**
- The harness assembly is `noEngineReferences` and compiles with C# 9 under .NET 10.
- A Unity compile (`unity-drones LOG -quit`) was **NOT_RUN**: the lead's Unity batch held the slot and MemAvailable stayed below 8 GiB.
- Run it once before the orchestrator commits: `unity-drones /tmp/harness_compile.log -quit`, then check for `Crane.Drones.Harness` errors.
- Running harness specs inside Unity (collider world) is future work. It would wrap `AutonomyBatchCli`'s world setup around `ExperimentRunner` with a `UnityColliderRayWorld`.
