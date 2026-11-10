# Fleet phase 1: STATUS

Branch `research/fleet` (worktree `/home/lunarz/worktrees/crane-fleet`), local commits only.
Evidence labels: IMPLEMENTED (code exists, not exercised in Unity), FIXTURE_VALIDATED (EditMode/unit tests
pass), LIVE_SIM_VALIDATED (exercised in a real Unity run whose outputs were inspected), NOT_RUN.
Everything is provisional geometry and provisional parameters (the shared profile has no measured values).

## What exists

| Item | Where | Evidence |
|---|---|---|
| Shared spec copies + sync script | `Tools/Fleet/spec/`, `Assets/StreamingAssets/Fleet/`, `Tools/Fleet/sync_spec.sh` | LIVE_SIM_VALIDATED (runs load them; sha256 in every manifest) |
| CRANE overlay: values the shared profile does not define (frame origin, motor lag, sensor mounts, colours, scene/still settings, qualification routes) | `Tools/Fleet/spec/crane_fleet_sim_v0.json` | LIVE_SIM_VALIDATED |
| `Crane.Fleet.Core` (noEngineReferences, namespace `Crane.Fleet`): JSON, spec model, `RoverDescription`, mecanum IK/FK, plant, encoders, gyro, odometry, qualification script, OBJ/mesh-manifest | `Assets/Scripts/Fleet/Core/` | FIXTURE_VALIDATED (also compiles and runs under plain `dotnet`) |
| `Crane.Fleet.Sim`: warehouse builder, `RoverRig` adapter, `FleetRun` (logs), `CadSwapCheck`, `FleetBoot` (player flags), `FleetStills` | `Assets/Scripts/Fleet/Sim/` | LIVE_SIM_VALIDATED |
| `Crane.Fleet.Editor`: batch CLI (`Run`, `Check`, `Batch`, `BuildPlayer`) | `Assets/Scripts/Fleet/Editor/FleetCli.cs` | LIVE_SIM_VALIDATED |
| `Crane.Fleet.Tests` (EditMode, 16 tests) | `Assets/Tests/Fleet/` | FIXTURE_VALIDATED: 16/16 pass |
| Fleet player (only scene: empty `Assets/Scenes/Fleet/FleetEmpty.unity`; world built at runtime) | `Builds/CRANE-Fleet/CRANE.x86_64` | LIVE_SIM_VALIDATED |
| Mesh-manifest hook (`meshes/<role>/manifest.json` next to the profile replaces primitives) | `Core/MeshManifest.cs`, `Sim/RoverRig.cs` | FIXTURE_VALIDATED (test fixture OBJ); no real CAD yet |
| Trajectory in crane-traj/1 (4 agents, truth, ENU, 20 Hz) | `trajectory.ctraj` in each run dir | LIVE_SIM_VALIDATED (read back by `theater/traj.py`, no problems) |
| Fleet Theater world | none | NOT_RUN |

## Commands

```sh
cd /home/lunarz/worktrees/crane-fleet
Tools/Fleet/sync_spec.sh                    # refresh spec copies from /home/lunarz/crane_ml/Tools/Fleet/spec
Tools/Fleet/fleet.sh test                   # EditMode tests
Tools/Fleet/fleet.sh batch "run=1:$HOME/.cache/crane-fleet/runs/fleet_warehouse_a_v0_qual_1"   # seeded run (editor)
Tools/Fleet/fleet.sh check                  # CAD-swap check -> ~/.cache/crane-fleet/check/cad_swap_check.json
Tools/Fleet/fleet.sh build                  # fleet player (about 3 min)
Tools/Fleet/fleet.sh player-run 1 DIR       # seeded run in the player, -batchmode -nographics
Tools/Fleet/fleet.sh player-check REPORT    # CAD-swap check in the player
Tools/Fleet/fleet.sh still 1 DIR            # stills: player -batchmode (GPU) under xvfb-run
python3 Tools/Fleet/summarize_run.py RUN_DIR
python3 Tools/Fleet/compare_runs.py RUN_A RUN_B
```

All Unity processes go through `heavy unity` and `flock ~/.cache/crane-fleet/unity.lock` (inside `fleet.sh`).
Players read the worktree spec (`--crane-fleet-spec`), so data edits need no rebuild; code edits do.

## Model summary

- Frames: world = scenario ENU, Unity = (x, z, y); chassis FLU with origin on the floor under the wheel-rectangle
  centre (assumption; the profile does not say). One converter: `Sim/FleetFrames.cs`.
- Plant (Core, `MecanumPlant`): command -> body limits -> IK -> common wheel-speed desaturation -> first-order
  motor lag -> per-wheel Gauss-Markov slip (floor sigma) -> FK x lateral efficiency -> body acceleration toward
  that twist (tracking time constant) -> per-wheel traction limit mu*N_i (CoM + load transfer) and torque/radius,
  one common scale. Unity body: one Rigidbody, chassis box collider, four frictionless wheel spheres; the plant
  force and yaw torque are applied at the CoM. Physics stepped manually (`simulationMode = Script`, dt 5 ms).
- Sensors: quantized encoders (1440 cpr, 100 Hz), gyro z (white noise + bias random walk + initial bias, 200 Hz).
  Estimate = encoder translation with ideal kinematics + gyro heading, initialised from truth at t = 0.
- Qualification script: scout to the loading area, others to assembly slots (farthest first, staggered),
  then strafe left/right 0.3 m and a 2*pi rotation, open loop. The controller reads evaluator truth.

## Results (2026-11-09)

- EditMode: 16/16 pass (kinematics identity, zero-slip achieved = commanded to 1e-9, lateral efficiency, wheel
  limit direction, normal loads, traction cap, encoders/odometry, RNG streams, spec resolution, mating pose,
  JSON, mesh manifest, warehouse geometry, CAD-swap check, in-process determinism).
- Seeded run (seed 1, 4 rovers): `script_complete` at t = 52.81 s, 10562 steps, 5.9 s wall (editor), no contacts.
- Determinism, seed 1: byte-identical `states.jsonl` and `events.jsonl` across two separate editor processes, the
  headless player and the player with graphics. `trajectory.ctraj` columns and events identical (header differs
  only in the recorded commit). Seed 2 differs from row 0 (start jitter), as intended.
- Caveat: a run that shares a process with other physics content first (e.g. the CAD check, then the run) is
  not bitwise identical: positions agree within 1.1e-6 m, contact enter/exit events flicker at grazing contacts.
  The default PhysX scene keeps history. Each run in its own process (or its own physics scene) avoids it.
- CAD-swap check: pass in the editor and the player; all wheels in contact (|gap| < 1e-6 m), frames exact. With
  primitives built from the same numbers this is a pipeline guard; it becomes informative with CAD collision
  meshes. It caught the world-scaling bug on its first run.
- Qualification metrics (seed 1): strafe 0.3 m commanded -> 0.236 to 0.242 m truth (lateral efficiency 0.85 plus
  motor lag), rotate 6.283 rad -> 6.22 to 6.24 rad. Odometry believes 0.289 m (it does not know the lateral
  efficiency), so the wheel-odometry estimate drifts 0.2 to 0.9 m and 0.06 to 0.2 rad over a 53 s run.

## Known defects / open items

- Still render: see the handoff note and the still section below (first attempt rendered sky only).
- `contact` events report impulse 0 at collision enter (enter events carry no impulse yet).
- Lifter lift plate and face markers are render-only; the lift plate overlaps the front docking face (visual).
- Inertia = profile box inertia scaled by total/base mass (derived, not from CAD); scout arm proxy not fitted.
- No rover-rover avoidance; routes are staggered in time by hand.
- Unity rewrites `ProjectSettings/ProjectSettings.asset` (drops a define) and `.vscode/settings.json` on import;
  these are reverted, not committed.
