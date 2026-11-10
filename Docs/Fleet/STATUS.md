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

## Phase 2, reprioritised (user, 2026-11-10): closed loop and generalization over polish

| Item | Where | Evidence |
|---|---|---|
| Perception core imported as its own assemblies: `Crane.Fleet.Perception` (rover side, noEngineReferences) and `Crane.Fleet.Perception.Evaluator` (truth side, referenced only by `Crane.Fleet.Sim`): the truth firewall holds at assembly level. Its own profile loader and RNG remain inside it (a second loader; dedupe later) | `Assets/Scripts/Fleet/Perception/`, `PerceptionEvaluator/` (`SOURCE_SHA256.txt`) | IMPLEMENTED (compiles; exercised offline in dotnet) |
| `FleetPerceptionHost`: FleetSimulator whose truth trajectories are the live rigid bodies (per-step history; frames at past acquisition times get the last state at or before). Uses the perception core's own camera mounts and tag geometry, which differ from the asset manifest's (integration gap) | `Sim/FleetAutonomyHost.cs` | IMPLEMENTED |
| Autonomy API (engine-free, truth-free): `IRoverIO` (perception provider, CRANE odometry, design description, twist/lift/gripper/connector commands, controller report) and `IFleetAutonomy` (Initialize/Step/Done) for the autonomy library to implement | `Assets/Scripts/Fleet/AutonomyApi/RoverIO.cs` (asmdef `Crane.Fleet.AutonomyApi`) | IMPLEMENTED (the autonomy library has not landed; no `IFleetAutonomy` implementation is loaded yet) |
| Fallback closed-loop docking (`TagDockingController`): own-camera tag observations composed with odometry at acquisition, windowed mean (and its covariance) of the target in the odometry frame; stand-off at 0.25 m inside decode range, >= 15 unflagged observations, then odometry-only final approach (the last 10 cm are below the camera minimum range), connector Approach / EngageHooks / Verify from the connector status; failed attempt -> back to stand-off | `AutonomyApi/TagDockingController.cs` | FIXTURE_VALIDATED offline (below); LIVE pending |
| Generic `CouplingManager`: Approach pairs the face with the nearest facing face of another rover; one ConnectorMachine per coupling stepped every physics step with the live face pose; LATCHED_VERIFIED -> FixedJoint; RELEASED -> joint removed (separation) | `Sim/FleetAutonomyHost.cs` | LIVE_SIM_VALIDATED via the earlier scripted path; closed-loop path pending |
| Demo mission: cargo_2 now closed loop (odometry navigation to the PLANNED rendezvous, then tag docking); scout, lifter, leader parking and coupled transport remain `scripted_truth` fallback, labelled per rover per step (`controller` events, `mission.controllers` in states, `controllers` in stills.jsonl) | `Sim/DemoMission.cs` | pending live run |
| Failure injection at runtime: `--crane-fleet-failures "rover:target:mode@t;..."`, wheels dead_free (dropped from the weighted FK and the traction share), dead_locked (speed 0, drags), weak (torque and speed x0.3); camera lost (no tags); encoders lost (odometry frozen) | `Core/Mecanum.cs` (WheelHealth, ForwardWeighted), `Sim/FleetRun.cs` | FIXTURE_VALIDATED (dotnet: dead_free keeps full control on 3 wheels, dead_locked makes the body yaw/drift, weak curves); LIVE pending |
| Paired-run harness: jobs file -> per-job run dirs, `summary.jsonl` (success, outcome, reason, first divergence vs `pair_of`), failure replays (`replay.json`; frames with a GPU) | `Sim/FleetHarness.cs`, `fleet.sh jobs|jobs-player`, `Tools/Fleet/jobs/demo_failures_v0.json` | IMPLEMENTED |
| Harness requests: `fleet.sh request-run --scenario --config --mission --seed --variant [--failures] [--out] [--labels]`; manifest `labels` (scenario id/hash, configuration and hash, variant, seed, floor, failure state, mission, controller input, interface pair types, coupling faces, shape class, payload class); estimator covariance in `mission.tag_docking.estimate_covariance` | `fleet.sh`, `Sim/FleetRun.cs`, `Sim/DemoMission.cs` | IMPLEMENTED; module-graph configurations are refused explicitly (`unsupported_configuration`) until the shared generator's assembly API exists |

Offline closed-loop docking check (`Tools/Fleet/offline/closed_loop_docking_mc/`, dotnet; real perception core, idealized kinematic
rovers, a constant odometry offset of up to 8 cm at hand-over, leader parked up to 5 cm / 0.05 rad off plan, capture modelled as gap
<= 8 mm inside +-1 cm and +-0.087 rad): 30/30 latched on the first attempt; about 15 to 20 observations used and 80 to 140 rejected as
ambiguous (near-frontal PnP flips) per docking; yaw error at capture up to 0.031 rad, close to the 0.035 rad hook yaw tolerance, so in
the full ConnectorMachine (per-hook seating) one-hook outcomes are expected: tag yaw is the weak axis, as the perception handoff says.

## Phase 2: closed-loop autonomy integrated (Crane.Fleet.Autonomy)

| Item | Where | Evidence |
|---|---|---|
| Autonomy library + its Capability dependency imported (own asmdefs, noEngineReferences); compiles unchanged against CRANE's `Crane.Fleet.Core` (docking profile from `DockingProfileBuilder`, no System.Text.Json) | `Assets/Scripts/Fleet/Autonomy/`, `Capability/` (`SOURCE_SHA256.txt`) | LIVE_SIM_VALIDATED |
| `AutonomyMission`: per-rover `IObservationSource` (own quantised encoders and raw gyro from the plant at their rates, perception-core tag observations incl. payload tags generated with the core's TagObserver, connector reports, lift/gripper driver state) and `IActuatorSink` (wheel-speed set-points straight into `MecanumPlant.WheelCommandOverride`, connector pulses, lift/gripper); `FleetAutonomy.Step(dt)` every physics step, synchronously (deterministic; planning stalls a tick instead of running on another thread); estimates and their 3x3 covariance logged next to truth in `states.jsonl` (`mission.autonomy.estimates`) | `Sim/AutonomyMission.cs` | LIVE_SIM_VALIDATED |
| Mission in the library's format with CRANE setup keys (`crane_payload`, `crane_max_duration_s`) | `Tools/Fleet/missions/autonomy_demo_v0.json`, `_v1.json` (8 cm place clearance) | LIVE_SIM_VALIDATED |
| Integration bug found and fixed: the library pulses connector Approach only to a PRESENT machine; CRANE created machines only on Approach (deadlock: dock looped servo/backoff for 290 s, `stills/autonomy_attempt1_dock_deadlock/`). Now a machine (FREE) exists once a facing partner face is within 0.6 m; idle separated ones are dropped | `Sim/FleetAutonomyHost.cs` | LIVE_SIM_VALIDATED |
| Editor launch guard: `timeout 3000` inside heavy (an editor hung once at startup for 85 min holding the only slot; killed after diagnosis: idle main thread in futex, log ended before the method ran) | `Tools/Fleet/fleet.sh` | IMPLEMENTED |

### Closed-loop results (harness `Tools/Fleet/jobs/autonomy_demo_v0.json`, mission v0, editor headless, 2026-11-10 03:48-04:33)

Outputs: `~/.cache/crane-fleet/jobs/autonomy_demo_v0/` (`summary.jsonl`, `runs/<id>/`, `replays/<id>/replay.json`).

| Job | Library verdict | Physical outcome (evaluator) |
|---|---|---|
| auto_s1 | all 9 tasks succeeded, t = 124 s (287 s wall) | scout found the payload by its tag at 23.0 s; lift ok; **tag-only dock latched in 11 s, 1 attempt** (capture 41.4 s, LATCHED_VERIFIED + LOAD_VALIDATED 42.2 s, residual 0.7 mm / 0.45 mrad); **handoff failed physically**: the lifter's estimate (5-8 cm, then 20 cm while pushing) drove it into cargo_1 with 2 cm planned clearance, the shove yawed the coupled pair -0.45 rad (its own estimate stayed at +0.015: skidding wheels read no motion), the released payload hit a deck-lip edge and fell to the floor at (4.43, 3.77); the pair then transported with a 0.33 rad heading error and clipped the divider. The library reported success throughout (it does not sense payload support after handoff). |
| auto_s1_repeat | identical | **byte-identical** states/events/trajectory to auto_s1 although it ran after it in the same process (per-run physics scene + deterministic library) |
| auto_s2 | all succeeded, t = 122 s | same failure mode as s1 (systematic: place-pose clearance vs estimate error), dock latched first attempt in 12 s |
| auto_s1_cargo2_rl_weak (weak wheel @5 s) | **failed, class Support** at the transport precondition | first divergence from auto_s1 at t = 17.4 s (cargo_2); dock: 5 back-offs, attempt 1 failed (Docking), attempt 2 latched at 278.8 s; payload fell off at handoff; the library's support check refused transport (payload COM 0.42 m outside the support hull): a correct refusal |
| scripted_demo_s1 (fallback) | n/a | **REGRESSION, not delivered**: lifter and cargo_1 both route through (2.2, 2.6) after an untested lifter-route edit (step timeouts at 70 and 130 s); the fallback tag docker then saw no tag. The scripted path is not validated in this build. |

Defects to fix next: (1) handoff place pose must account for the estimator covariance (v1 mission uses 8 cm clearance; better: the library should hold off when the pre-place estimate uncertainty exceeds the clearance); (2) the library should detect an external shove of a stationary assembly (wheel odometry cannot; tag observations of a peer or overhead could); (3) payload support is not sensed after release (no deck load cell / payload tag check), so "handoff succeeded" is belief, not fact; (4) transport through the doorway on a 0.33 rad heading error grazed the divider: tracking used the estimate, which was wrong; (5) scripted fallback regression above; (6) weak-wheel docking needed 5 back-offs (re-tune gains on the dynamic plant, as the library README warns).

Note (process): a stills run at 04:35 used the player built at 03:06, before the connector-pairing fix, and deadlocked at docking
again (`stills/autonomy_attempt2_stale_player/`). The player must be rebuilt after every code change; `fleet.sh` does not check
staleness yet (next: compare the build manifest's commit with HEAD before a player launch).

### The major finding: "mission succeeded" was physically false in every closed-loop run

In auto_s1, auto_s1_repeat, auto_s2 and the stills run (mission v1), the transport task ended "Succeeded" while the coupled pair
was jammed against the divider BELOW the doorway (truth cargo_1 (5.71, 2.88-2.96); the opening spans y 3.1-3.9), with the
library's estimate at the goal (8.94, 3.42-3.45): a 3.2 m estimate error. The wheels kept turning against the wall and wheel
odometry integrated motion that did not happen; the reported covariance stayed small in x (sigma about 7 cm). Combined with the
handoff failure (payload on the floor next to the pair), the demo mission delivered nothing, while the executive reported success.
This is a predicted-vs-executed disagreement of class "perception/estimation" in promotion_v0 terms, and exactly why CRANE must not
take the autonomy's own success flag: the harness classifies outcomes from evaluator truth (summary.jsonl reports
`mission_succeeded`, not `..._payload_delivered`; to fix next: make the harness outcome require the evaluator's delivery check).
Library-side fixes to propose: exteroceptive correction during transport (peer/overhead/fixed-station tags), contact/stall detection
from commanded-vs-achieved wheel torque or a bumper, and a delivery check from the payload's tag.

### Stills, closed-loop run (mission v1, seed 1): `~/.cache/crane-fleet/stills/demo_seed1/` (LIVE_SIM_VALIDATED; labelled closed loop, Crane.Fleet.Autonomy)

`stills.jsonl` records per image the trigger, time, camera and the task/phase of every rover.
- `00_establish.png` (t 1 s): indoor and legible: tiled floor, matte walls, yellow door jambs, tinted zones with tape, real asset
  rovers with tags and mecanum rollers. Defects: shelf renders as a dark monolith; gantry beam near-black; south wall cut away so the
  camera looks in from outside.
- `01_scout_finds_payload_onboard.png` (t 23.0 s, sensor profile, scout cam_left frame of the asset manifest): payload visible
  low-left; its tag is not drawn (exists only in the perception model); the onboard frame is the manifest camera, not the perception
  core's mount that produced the detection (integration gap).
- `01b_scout_finds_payload_wide.png`: scout facing the payload's east end; payload on its 3 cm pallet (pallet orientation fixed).
- `02_lift.png` (t 68.2 s): lifter holding the long box clear of the pallet; reads correctly; kinematic carry (no load on the lifter).
- `03a_wide_formation_capture.png` (t 41.6 s): coupled cargo pair in the violet zone, rovers about 120-130 px long; scout already
  through the doorway; the pallet stack occludes the lower right of the zone (camera placement defect).
- `03b_dock_hooks_closing.png` / `03c_dock_latched.png` (t 42.25 / 42.36 s): faces flush, wheels on the floor, tags crisp; the two
  images are nearly identical because hooks close in 0.8 s and the trigger was capture + 0.9 s; the hooks sit inside the pocket and
  are NOT visible from this angle; the state LED renders black (emissive not applied). Hook engagement is not shown visually.
- `04a_wide_handoff.png` (t 91.3 s, release): the lifter approached from the north side at an angle and holds the box BESIDE the
  pair, not over the decks: the v1 clearance did not fix the root cause (lifter estimate and approach geometry).
- `04b_payload_on_decks.png` (t 92.8 s): the payload leans against cargo_1's side, one end on the floor: support failed; matches
  the logs.
- `05_through_doorway.png` (t 106.9 s): the coupled pair, without the payload, pressed against the divider beside the opening.
- `06_scout_watches_onboard.png` (t 109.9 s, sensor profile): through the doorway the scout sees the pair still in the west hall.
- `07_done.png` (t 123.1 s, at the library's "mission succeeded"): the delivery zone is empty.
Earlier attempts kept for the record: `stills/autonomy_attempt1_dock_deadlock/` (connector-pairing deadlock), `stills/autonomy_attempt2_stale_player/` (stale player build).

Launch rule fix (IMPLEMENTED, not yet exercised): `fleet.sh` now takes `unity.lock` first and checks memory and all heavy slots inside
it, and closes the lock fd for children. Before, the check ran before a long flock wait (an orphaned Xvfb from a killed run held the
lock), so at 04:45 my build may have overlapped another session's short player by a few seconds.
