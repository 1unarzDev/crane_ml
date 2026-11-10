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

- Frames: world = scenario ENU, Unity = (x, z, y); chassis FLU with origin = `base_link` at the body-underside
  centre, as in `Tools/Fleet/assets/ASSET_MANIFEST_SPEC.md` (overlay `frame_convention.origin`; the profile itself
  does not say). Face, CoM and sensor-mount z are base-frame; deck and mast heights are above ground. One
  converter: `Sim/FleetFrames.cs`. (Evidence collected under the earlier floor-origin convention is archived in
  `~/.cache/crane-fleet/archive/floor_origin/`.)
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

- EditMode: 16/16 pass, re-run after the frame-convention change (kinematics identity, zero-slip achieved = commanded to 1e-9, lateral efficiency, wheel
  limit direction, normal loads, traction cap, encoders/odometry, RNG streams, spec resolution, mating pose,
  JSON, mesh manifest, warehouse geometry, CAD-swap check, in-process determinism).
- Seeded run (seed 1, 4 rovers): `script_complete` at t = 52.81 s, 10562 steps, about 4 s wall (editor), no
  contacts; base_link rides at z = 0.0300 m (= ground clearance), tilt 0. (With the first route, three rovers grazed
  the shelf corner; the lane waypoint moved to (3.5, 3.3); archived in `archive/route_v1/`.)
- Determinism, seed 1: byte-identical `states.jsonl`, `events.jsonl` and `trajectory.ctraj` across two separate
  editor processes, the headless player and the player with graphics (the still run), all at commit d95a788.
  Seed 2 differs from row 0 (start jitter), as intended. Reports: `~/.cache/crane-fleet/determinism_*.json`.
- Caveat: a run that shares a process with other physics content first (e.g. the CAD check, then the run) is
  not bitwise identical: positions agree within 1.1e-6 m, contact enter/exit events flicker at grazing contacts.
  The default PhysX scene keeps history. Each run in its own process (or its own physics scene) avoids it.
- CAD-swap check: pass in the editor and the player; all wheels in contact (|gap| < 1e-6 m), frames exact. With
  primitives built from the same numbers this is a pipeline guard; it becomes informative with CAD collision
  meshes. It caught the world-scaling bug on its first run.
- Qualification metrics (seed 1): strafe 0.3 m commanded -> 0.236 to 0.242 m truth (lateral efficiency 0.85 plus
  motor lag), rotate 6.283 rad -> 6.217 to 6.235 rad. Odometry believes 0.289 m (it does not know the lateral
  efficiency), so the wheel-odometry estimate drifts 0.2 to 0.9 m and 0.06 to 0.2 rad over a 53 s run.

- CAD-swap check is data-driven and informative (player, no rebuild): a modified profile (wheel radius 0.06,
  clearance 0.045, lx 0.15, front/rear face z 0.075) passes with no code change
  (`check/cad_swap_check_modified_profile.json`); a cargo mesh manifest whose collision hull reaches 1 cm below the
  wheel plane fails with 36 findings (rover rides 10 mm high on the hull; `check/cad_swap_check_bad_cad.json`).
  Variants in `~/.cache/crane-fleet/spec_variants/`.

## Still (LIVE_SIM_VALIDATED, first picture only, not polished)

`~/.cache/crane-fleet/stills/seed1/{wide_start,close_start,wide_assembly}.png` (1600x900, Vulkan on the RTX
4070 Ti SUPER, player `-batchmode` under `xvfb-run`, HDRP Balanced). The first attempt
(`archive/floor_origin/stills/seed1/`) rendered only the sky: the fleet player has no scene materials, so HDRP/Lit variants were stripped
and every draw was dropped silently; fixed by shipping `Assets/Resources/Fleet/FleetLit.mat` as the colour template.
What is wrong with the current stills:
- No ambient/indirect light: shadowed areas are near-black. The runtime scene has no baked or updated ambient
  probe; only the sun lights anything.
- The scene is lit like an outdoor courtyard (one sun at 55 deg, no roof, no ceiling lights). The south wall's
  shadow covers half the start zone, so lifter_1 and cargo_1 are almost invisible at t = 0.
- Shadow edges are very soft and banded (default cascades over a 12 m view).
- Outside the arena the HDRI sky's lower hemisphere shows as a flat brown backdrop; floor margin visible.
- Rovers are about 25 px long in the wide shot; roles are readable by colour (orange scout with mast, blue
  lifter, two greens for cargo) only in the lit area. Wheels look grounded; no z-fighting seen on zone patches.
- The doorway is hidden from above by its lintel; face markers (magenta) are tiny; sensors are not drawn.

## Known defects / open items

- `contact` events report impulse 0 at collision enter (enter events carry no impulse yet).
- Lifter lift plate and face markers are render-only; the lift plate overlaps the front docking face (visual).
- Inertia = profile box inertia scaled by total/base mass (derived, not from CAD); scout arm proxy not fitted.
- No rover-rover avoidance; routes are staggered in time by hand.
- Unity rewrites `ProjectSettings/ProjectSettings.asset` (drops a define) and `.vscode/settings.json` on import;
  these are reverted, not committed.

---

# Phase 2 (2026-11-10)

Coordinator accepted the chassis-origin decision: `base_link` = centre of the body underside, kept in the CRANE-only overlay
(`frame_convention.origin = body_underside`), matching the asset pipeline.

Unity launch rule (operator, 2026-11-10): `Tools/Fleet/fleet.sh` waits until no `unity.*` heavy slot is held by anyone and at least
12 GB is available before every launch (then `flock` + `heavy unity`). Much of phase 2's wall time was spent waiting on this.

## What exists (phase 2)

| Item | Where | Evidence |
|---|---|---|
| Docking core imported into `Crane.Fleet.Core` (namespaces `Crane.Fleet.Docking/Graph/Profile`); its `ProfileLoader` (System.Text.Json) and `DeterministicRng` removed; single loader = `FleetSpec`, single RNG = `FleetRng`; `DockingProfileBuilder` fills the core's typed profile from the spec and runs its `Validate()`; one `Pose2` (the core's) | `Assets/Scripts/Fleet/Core/Docking/` (`SOURCE_SHA256.txt` = hashes of the imported sources) | FIXTURE_VALIDATED (EditMode: latch on an ideal approach; profile from spec) and LIVE_SIM_VALIDATED (demo run) |
| Asset manifests (crane.fleet_asset_manifest/v0) and OBJ meshes, provisional, copied with provenance and sha256 | `Tools/Fleet/assets_provisional_v0/` (`PROVENANCE.md`, `SHA256SUMS`), mirrored to `Assets/StreamingAssets/Fleet/assets_provisional_v0/` | LIVE_SIM_VALIDATED |
| Asset visuals: one transform per part, origin under parent, joints driven from state (wheels from plant wheel angles, hooks from the connector machine, lift carriage and fingers from the lifter state, state LED per face), materials from the manifest slots, role colour override, tag textures (+10 ids for cargo_2) | `Sim/FleetAssets.cs`, `Sim/RoverRig.cs` | LIVE_SIM_VALIDATED (built in runs; render see stills) |
| With a manifest present: sensor frames come from the manifest (overlay placeholders dropped), chassis/deck collision boxes from the manifest; wheel contact spheres, mass and docking faces from the profile | `Core/FleetSpec.cs` | LIVE_SIM_VALIDATED |
| CAD-swap check extended to the imported assets: render-wheel bottom on the floor, manifest wheel origins = profile axles, manifest face origins = profile faces | `Sim/CadSwapCheck.cs` | see results |
| Per-run physics scene (player: `CreateScene` with `LocalPhysicsMode.Physics3D`; editor: preview scene) | `Sim/FleetPhysicsScene.cs` | FIXTURE_VALIDATED (EditMode `RunIsIndependentOfWhatRanBeforeInTheProcess`) |
| Demo mission (scripted, steers from truth; logged in the manifest as `controller_input`) | `Sim/DemoMission.cs`, overlay `demo` | LIVE_SIM_VALIDATED |
| Indoor look (LOOK.md): tiled floor texture, matte walls, yellow door jambs and door tape, shelf/pallet dressing, tinted zones with tape, ceiling + south wall on cut-away layer 28, rect-light grid + fill + door practicals, cinematic (layer 30) and neutral sensor (layer 31) volumes, sensor camera with look frame-settings off and no AA | `Sim/FleetLook.cs`, overlay `look` | see stills |
| Event-triggered shots (time or logged event + offset; absolute, rover/payload-relative, onboard sensor cameras), `stills.jsonl` per still with the logged state, optional clip (PNG frames + ffmpeg) | `Sim/FleetBoot.cs` (`FleetShots`), overlay `demo_shots` | see stills |

## Demo mission (overlay `demo`)

Scout drives behind the shelf to a viewpoint and "finds" the long payload (camera `cam_left`: within hfov/2, within 4 m, ray reaches
the payload first). Lifter waits, then grips it off a 0.10 m stand (staging: the 0.15 m lift travel cannot otherwise clear the 0.152 m
deck surface), lifts 0.07 m more and carries it **kinematically** (rigid grip; lifter load and tipping not simulated; a static check is
logged: 6 kg > 5 kg profile lift capacity, combined CoM 0.28 m ahead of the 0.14 m front axle, i.e. the lifter would tip). cargo_1 parks in
the assembly zone; cargo_2 drives to a pre-dock pose and creeps in at 0.03 m/s while the docking core's `ConnectorMachine` is stepped
every physics step with the live face-to-face pose (gap, lateral, yaw, height, closing speed): APPROACH -> MAGNET_CAPTURED ->
EngageHooks -> hooks sensed -> Verify -> LATCHED_VERIFIED, at which a `FixedJoint` (follower -> leader, collision between them off) is
created at the current relative pose. FixedJoint rather than an ArticulationBody parent: both rovers are Rigidbodies driven by the same
force-level plant; re-parenting into an articulation would replace the plant's body and its validated behaviour mid-run, while a fixed
joint between two 6.8 kg bodies at 200 Hz is well-conditioned (equal masses, no long lever). The lifter lowers the payload onto the
coupled decks by opening the gripper (payload becomes dynamic, drops about 1.8 cm, rests by physics), backs off and clears. The pair
drives through the doorway (follower command = leader twist plus w x r) to delivery; the scout watches from the east hall.
