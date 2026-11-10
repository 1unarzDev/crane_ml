# Crane.Fleet.Autonomy

Engine-independent closed-loop autonomy for the modular mecanum fleet: estimation, capability-conditioned planning, tracking,
docking, mission execution. Lets CRANE run genuine autonomous behaviour from each rover's own simulated observations instead of
scripted, truth-steered motion (promotion_v0.md §2). ASSUMED-PARAMETER code: every physical number comes from
`Tools/Fleet/spec/fleet_profile_v0.json` (all `provisional`) or from `AutonomyAssumptions` (registry in code). Not committed.

- Library `Crane.Fleet.Autonomy/`: `netstandard2.1`, C# 9, namespace `Crane.Fleet`, assembly `Crane.Fleet.Autonomy`, no engine
  references, no package references. Project references: `Crane.Fleet.Docking` (assembly `Crane.Fleet.Core`), `Crane.Fleet.Perception`,
  `Crane.Fleet.Capability`.
- Tests `Crane.Fleet.Autonomy.Tests/` (xUnit, net10.0) including the headless harness `Harness/KinematicWorld.cs`.
- Solution `Tools/Fleet/core_cs/Crane.Fleet.Autonomy.sln`. Run:
  `TMPDIR=~/.cache/crane-fleet/tmp nice dotnet test Tools/Fleet/core_cs/Crane.Fleet.Autonomy.sln`
  (`CRANE_AUTONOMY_REPORT=<dir>` appends the harness tallies to `<dir>/harness_report.txt`;
  `CRANE_DEBUG_SCENARIO=nav|dock|line|crab|loaded|fail|sep|sepp CRANE_DEBUG_SEED=n dotnet test --filter PrintScenario` prints one run's event log).

## Layout

| Path | Contents |
|---|---|
| `Common/Interfaces.cs` | `IClock`, `IObservationSource`, `IActuatorSink`, `EncoderSample`, `GyroSample`, `ConnectorReport`, `LiftReport` |
| `Common/Se2.cs` | `Twist2`, SE(2) exp/log/compose/Jacobians (on Docking's `Pose2`), allocation-free `Lin` helpers |
| `Common/RoverSpec.cs` | `RoverSpec.FromProfile` (wheel geometry, limits, faces, floor lateral efficiency and slip, deck/lift), `AutonomyAssumptions` + `Registry` |
| `Capability/` | `AssemblyModel` (module graph: modules, poses, wheel health, payload footprint; `Merge`, `Without`, `MatedPose`), `ICapability`, `AssemblyCapability`, `CapabilityLibraryBuilder` (default, wraps `MotionAnalyzer`), `AnalyticCapabilityBuilder` (fallback) |
| `Estimation/BodyEkf.cs` | per-assembly EKF on SE(2) + body twist + per-module gyro bias, delayed-measurement buffer and re-application |
| `Estimation/RelativePoseEstimator.cs` | face-to-face relative pose in the docking frame (3x3 covariance), own camera + peer/overhead pairs, capture-envelope test |
| `Planning/LatticePlanner.cs` | `PlanMap` (oriented boxes), hybrid-A*/state-lattice planner whose primitives come from `ICapability` |
| `Control/Controllers.cs` | `WheelAllocator` (degraded allocation), `TrajectoryTracker`, `WheelHealthMonitor` (online dead_locked / weak identification) |
| `Mission/` | `MissionSpec` (JSON task list), `mission_schema_v0.json`, `MissionExecutive` (dependency DAG, retries, failure classes), behaviours, `FleetAutonomy` (top level) |

## API in one screen

```csharp
var profile = FleetProfile.Parse(profileJson);                         // Crane.Fleet.Perception loader (no System.Text.Json)
var dock    = Crane.Fleet.Profile.ProfileLoader.LoadOrThrow(profileJson); // Docking profile (release invariant, capture budget)
var cfg = new FleetConfig { Map = map, DockingProfile = dock };          // map: PlanMap with obstacle boxes (from the scenario)
foreach (var r in roster) { cfg.Rovers.Add(RoverSpec.FromProfile(profile, r.Id, r.Role, floor)); cfg.InitialPoses[r.Id] = r.InitialEstimate; }
var fleet = new FleetAutonomy(cfg, clock, sources, sinks);              // one IObservationSource + IActuatorSink per rover
fleet.LoadMission(MissionSpec.Parse(missionJson));
// every control tick, after the simulator advanced and stamped its sensors:
fleet.Step(dt);
// fleet.Executive.Done / Succeeded / FirstFailureClass / Records ; fleet.Events (typed, timestamped) ; fleet.Agents (estimates)
```

## Integration interfaces (what CRANE must provide)

Per tick, in this order: advance physics; publish sensor samples with acquisition stamps; set `clock.Now`; call `fleet.Step(dt)`;
apply the sink commands for the next physics interval. The library never reads truth, wall clock or randomness; identical inputs
give identical outputs (tested).

`IObservationSource` (one per rover; each `Read*` returns what was DELIVERED since the previous call, into a caller-owned list
that it clears first):
- `ReadEncoders`: `EncoderSample {Stamp, Fl, Fr, Rl, Rr}` cumulative quantised counts at the profile rate (100 Hz). Stamp = sample
  time; counts must correspond to that time (interpolate if the physics step is longer than the encoder period).
- `ReadGyro`: `GyroSample {Stamp, RateZ}` raw (bias and noise included), 200 Hz.
- `ReadTagObservations`: Perception `RelPoseObservation` (pose `T_parent_child`, 6x6 covariance in parent axes for `[dp; dtheta]`,
  `StampAcquired`, `StampDelivered <= now`, `AmbiguousPose`). Own camera: parent `"<self>/base_link"`, child `"<other>/base_link"` or
  `"<object>/base_link"`. Peers through their estimate, fixed stations and overhead: parent `"world"`. World-frame observations of a
  rover must be delivered to that rover's source (the docker's source must also deliver world-frame observations of its partner).
  Perception's `FleetSimulator` already produces exactly this.
- `ReadConnector(face)`: `ConnectorReport` from the coupling's `ConnectorMachine` (state, hook sensor reads, `FailedAttempts`,
  `RetriesExhausted`, last `RequestedAction`, partner). `Present = false` if that face has no machine. The machine runs on the
  simulator side, fed with the true face pose (that is the connector's physics, not the robot's knowledge).
- `ReadLift()`: lift height, gripper closed, holding object.

`IActuatorSink` (one per rover): `SetWheelSpeeds(fl, fr, rl, rr)` rad/s already within limits and compensated for the profile
lateral efficiency; `CommandConnector(face, cmd)` is called every tick for every face with `DockingCommand.None` except for a
one-tick pulse (feed it to `ConnectorInputs.Command`; the machine is edge-triggered); `SetLift(height, gripperClosed)`.
Optional `IEstimatePublisher.PublishEstimate(stamp, pose, cov3)`: forward to Perception `ReportSelfEstimate` (peer observations
need it) and to ROS.

`IClock.Now`: the simulation time used for every stamp. `FleetAutonomy.Step(dt)`: single entry point.

Optional channels (implement on the same source object; detected with `is`):
- `IInertialSource.ReadAccel`: `AccelSample {Stamp, Ax, Ay}` body-frame specific force, gravity removed, 200 Hz (rigid-body
  acceleration of the chassis plus noise). Enables impact / stall / shove detection; without it only encoder-vs-gyro yaw slip is
  detected, and a body stuck against a wall while driving straight is NOT detectable.
- `IPoseFixSource.ReadPoseFixes`: `PoseFix {Stamp, Pose (world pose of this rover's base), Cov3, SourceId}` from LiDAR scan matching
  against the scenario map, a fixed tracker, or any other absolute source.
- `LiftReport.DeckLoadValid / DeckLoadKg`: deck load cell or contact switches on cargo decks (used to confirm a handoff).
- `FleetConfig.Landmarks[name] = Pose2`: static tags (door jambs, walls) whose frame `"<name>/base_link"` the own cameras may see;
  CRANE must deliver their detections like any other own-camera `RelPoseObservation` (child `"<name>/base_link"`).

Mission outcome: the library never reports success from its own belief. When every task succeeded it sets
`Executive.BelievedComplete` (event `mission_believed_complete`); the host must evaluate the world (the evaluator's delivery check)
and call `Executive.ConfirmByHost(ok, reason)`. Only then is `Executive.Succeeded` true; a rejection gives class `HostRejected`.

Per-frame cost: the estimator, allocator and tracker work on preallocated buffers; allocation happens on structural events
(plan, merge, split, capability change), task transitions and in the docking prior. A full-rank plan across the 10 x 7 m
warehouse takes tens of node expansions (milliseconds); reduced-mobility searches cost about 1 ms per expansion and are bounded by
`PlanMaxExpansionsReduced` (25 000), so an infeasible reduced-mobility plan can stall a tick for tens of seconds: run planning
off the physics thread in CRANE or lower the bound.

### Integration checklist for the Unity owner

1. Assemblies: add `Crane.Fleet.Autonomy.asmdef` (`noEngineReferences: true`, references `Crane.Fleet.Core`, `Crane.Fleet.Perception`,
   `Crane.Fleet.Capability`); copy the sources without `bin/obj/csproj` and without the test project. Type names avoid every
   public name in CRANE's `Crane.Fleet.Core` (`Twist2`, not `Twist`; a test enforces the list).
2. Profile: parse the shared profile with Perception's `FleetProfile.Parse(TextAsset.text)`; the Docking profile needs
   `System.Text.Json` (or fill `DockingProfile` by its setters, see the Docking README). Pass the scenario floor to `RoverSpec.FromProfile`.
3. Map: build `PlanMap(0, 0, arenaX, arenaY)` from the scenario boxes (divider with doorway: `AddWallWithDoor`; elevated beams
   that do not block driving must be left out).
4. Sources: wrap `RoverRig` sensors (encoders, gyro) and the Perception `RoverObservationProvider` per rover; deliver connector
   reports from each coupling's `ConnectorMachine` (one machine per face pair, keyed by the docker face); lift/gripper from the rig.
5. Sinks: wheel speeds into `MecanumPlant` (replaces the truth-steered `GoToController`), connector pulses into the machine inputs,
   lift targets into the lift joint. At `MAGNET_CAPTURED` the simulator creates the physical joint; at `RELEASED` removes it.
6. Initial estimates: `cfg.InitialPoses` = the start pose as the robot believes it (e.g. surveyed start slot), never the
   per-seed truth; `InitialPosSigma/YawSigma` its uncertainty.
7. Logging: record `fleet.Events`, `Executive.Records` (state, attempts, class, reason) and per-agent estimates next to truth in
   `states.jsonl`; the disagreement log (promotion_v0 §3) takes the first failed task's class.
8. Keep the controller input label `estimate` in the run manifest; the harness-only `KinematicWorld` must not be copied.

## Mission format (`crane.fleet_mission/v0`, JSON Schema `Mission/mission_schema_v0.json`)

A mission is a task list; tasks run as a dependency DAG. A task starts when all `after` tasks succeeded (if `after` is absent:
the latest earlier task that names one of its rovers) and none of its rovers is held by another running task. Rovers are
resolved to their CURRENT assembly at every tick (after a dock, either member id addresses the assembly). A failed attempt is
retried up to `retries` times with a fresh behaviour (replanned); `Support` and `Configuration` failures are not retried. When a
task fails for good, dependants are `Skipped` and the mission reports the first failure's class.

| type | parameters | preconditions | success |
|---|---|---|---|
| `goto` | `agent`, `goal` [x,y,yaw] and/or `route` [[x,y],...], `tol` [pos,yaw], `cruise` | a feasible plan for the agent's current capability | agent's member `agent` within tolerance of the goal (estimate) |
| `explore` | `agent`, `region` [x0,y0,x1,y1], `target`, `viewpoints` (opt), `dwell` | | the target's tag seen by the agent's own camera (pose put on the blackboard) |
| `lift` | `agent`, `object`, `object_pose` (if not on the blackboard), `grasp_offset`, `height` | lift present; object located | gripper closed holding the object at height |
| `rendezvous` | `target`, `docker`, `target_face`, `docker_face`, `meeting_pose` | | target at the meeting pose, docker at the pre-dock pose (0.5 m face gap) |
| `dock` | `docker`, `docker_face`, `target`, `target_face` | faces compatible; target holds still | connector `LATCHED_VERIFIED`/`LOAD_VALIDATED`; agents merged |
| `assemble` | `sequence` [{docker, docker_face, target, target_face}, ...] | | every step docked in order |
| `transport` | `agent`, `goal`, `payload` (opt), `tol`, `cruise` | belief graph support verified; all payload supporters in the agent | goal reached with every coupling still verified |
| `separate` | `docker`, `docker_face`, `target`, `target_face`, `clear_distance` | `ModuleGraph.CanRelease` (release invariant) | connector `RELEASED`, agents split, docker cleared |
| `handoff` | `from`, `supporters`, `place_on`, `place_offset`, `payload`, `payload_type`, `hold_offset`, `clear_distance` (`place_pose` legacy) | `from` holds an object; receiver tags visible | aligned on the receiver's tags, released, support confirmed (load cell or payload tag on the deck); payload added to the belief graph |
| `wait` | `agents`, `duration` | | time elapsed |

Top level: `schema`, `mission_id`, `initial_couplings` [{docker, docker_face, target, target_face}], `initial_payloads` [{id, type,
supporters, com}], `tasks`. Failure classes: `HostRejected` (host disagreed with a believed-complete mission), `Geometry` (no route for this footprint/capability), `Traction` (no feasible primitive
or persistent deviation), `Planner`, `Docking`, `Perception`, `Support`, `Timeout`, `Configuration`.

Differences from what `Tools/Fleet/harness/promote.build_mission` emits today (accepted aliases are mapped by `TaskSpec.NormalizeAliases`):
- `kind` is accepted for `type`; `undock` maps to `separate`; `actors[0]`/`reference_member` to `agent`; `follower`/`leader`,
  `face_follower`/`face_leader` and `coupling` (`"leader.face-follower.face"`) to `docker`/`target` and faces; `route_m` to `route`.
- Not supported (rejected with a message): `load`, `deliver`, `rotate`. Use `handoff` (lifter places onto decks) for `load`,
  `transport` + `separate` for `deliver`, and a `goto` with a yaw for `rotate`.
- `undock` needs the faces (`coupling` or explicit fields); the harness's undock tasks have none.
- Timing fields (`t_start_s`, `t_end_s`, `t_game`) are ignored: execution is closed-loop and event-driven; order comes from `after`
  (or the implicit per-rover order). `route_cells`, `passes_chokepoints`, `assignments` are kept only as provenance.
- `goto` with `assembly: true` needs no special handling: the agent is the rover's current assembly.

## Design notes and assumptions

- Estimator: wheel rates are measured per wheel (differenced over >= 40 ms) with noise `2 x slip_frac x |rate| + quantisation`;
  the profile floor's lateral efficiency is part of the measurement model (the CRANE plant has it). Gyro biases are learned at
  standstill (zero-velocity updates and at-rest gyro samples; the mission starts after 1.5 s still) and are Schmidt "consider"
  states for wheel updates, because time-correlated slip otherwise walks the bias. A locked wheel's encoder is not used.
- Relative estimate: own-camera PnP covariance mapped through the face levers keeps the anisotropy (good cross-range, poor frontal
  yaw); ambiguous own-camera PnP solutions are discarded; innovation gate chi2(3) = 16.3. The world estimates only give a coarse prior.
  World-frame (overhead/peer) ambiguous solutions are used with covariance x4 (`WorldAmbiguousInflation`): a near-frontal
  overhead view of a horizontal top tag is almost always flagged, and the two branches differ mainly in tilt, not in (x, y, yaw).
- Docking speed is tied to the covariance: inside 0.18 m the estimate must predict capture at 2.5 sigma against the guide
  envelope (guide half width + magnet pull-in from the docking profile, acceptance angle); otherwise hold for up to 2 s, then
  back off to 0.30 m and re-observe.
- Capability: for a mecanum assembly a dead_free wheel removes a row but leaves rank 3; dead_locked adds a hard row (rank 2);
  weak scales the limit. Online identification detects dead_locked and weak from commanded vs measured rates. dead_free is
  kinematically INVISIBLE on a four-mecanum module when commands are consistent (the other wheels produce the commanded twist and
  the free wheel's encoder reads the matching rate), so it needs a motor-driver fault flag: call `ReportWheelHealth`.
- Reduced mobility: extra 6 cm planning margin and a 6 cm replan deviation, since errors outside the commandable subspace
  cannot be corrected by feedback.

## Harness results (headless, seeds; details in the handoff `Docs/ResearchOrchestration/AGENT_HANDOFFS/fleet/autonomy_lib.md`)

Single-rover doorway navigation 10/10 (truth error median 3 cm); tag-only docking from 0.5 m with unknown +-3 cm / +-5 deg start
error 30/30 (also 12/12 at 6 cm / 10 deg, 6/6 at 6 px corner noise); line assembly + doorway transport 8/8; side-by-side pair
crabbing through a 0.65 m doorway 8/8; separation 6/6, and 6/6 correctly refused (Support) with a payload across the coupling;
weak wheel mid-mission 6/6; locked wheel mid-mission (single rover) 4/4; locked wheel on a loaded line before the doorway 0/2,
both reported infeasible (Geometry). No unsafe release.

## Contact safeguards (CRANE phase-2 failures, D69-fleet)

CRANE showed two overconfidence failures the kinematic harness could not: a coupled pair jammed against the divider while odometry
integrated 3.2 m with ~7 cm reported sigma, and a lifter 5-20 cm off released the payload beside the decks. Now:
- Stall/shove/slip detection per module (`CheckStall`): accelerometer-integrated body velocity (leaky, 1.5 s, toward the encoder
  velocity) vs encoder velocity (residual > 0.07 m/s for 0.15 s), and encoder vs gyro yaw rate (> 0.12 rad/s for 0.3 s). On
  detection the pose covariance is inflated (0.25 m, 0.08 rad), the agent is flagged `Stalled`, and while flagged every metre of
  encoder-claimed motion adds a metre of sigma. The navigator backs off 0.15 m opposite to the last command, holds 2 s for fixes, then
  replans; more than 3 stalls in a task fails it with class `Traction`. Docking contact suppresses detection.
- Absolute fixes during any task: world-frame tags (peer, overhead), known landmarks, `IPoseFixSource`, and cross-rover fixes (a
  rover whose camera sees another rover's tags fixes that rover when its own position sigma is under half the target's). A streak of
  4 gate-rejected fixes with mutually consistent innovations re-localises the agent on the fix (the estimate, not the fixes, was wrong).
- Handoff is defined in the receiver's frame (`place_on`, `place_offset`): the lifter stops 0.25 m short, aligns on the receiver's
  tags with a relative estimate that must pass a covariance gate (8 mm, 0.02 rad), makes the final blind move on that estimate,
  releases, backs off 0.55 m, and reports done only after support is confirmed (deck load cells >= 70 % of the payload mass, or the
  payload's tag seen inside the deck area); otherwise class `Support`.
- Each safeguard has a `FleetConfig` switch (`StallDetection`, `Relocalisation`, `CrossRoverFixes`, `HandoffTagAlignment`,
  `PostReleaseSupportCheck`); the reproduction tests run with them off (failure reproduced: believed complete, host rejects) and on.

Harness reproductions (`ContactFailureReproductionTests`): wall jam after an unseen 0.35 m shove: safeguards off -> believed
complete, host rejects (2.7 m from goal); on + door-jamb tags -> 3/3 delivered (shove detected, fixes or relocalisation); on without
any absolute fix -> stalls repeatedly and fails `Traction`, never claims success. Handoff with +0.15/+0.17 m lifter odometry bias:
off -> payload falls, believed complete, host rejects; on -> 3/3 placed and confirmed by the payload tag, and by load cells; tag
alignment off but support check on -> `Support` failure instead of success. Scan-match fixes re-localise a confident 0.5 m error.

## Lift geometry (CRANE v2 seed 2, "goal_in_collision" twice)

Cause: CRANE puts the payload (box_long, 0.80 x 0.25 m) into the planning map. The lift task planned a fixed 0.6 m pre-grasp pose
and the grasp pose (`grasp_offset` 0.237 m) with that box as an obstacle. The pre-grasp lifter body reached 1 cm into the
0.40 m half-length, inside the 3 cm margin; the grasp pose overlaps the object by design (the lifter's gripper envelope reaches it).
It was not a neighbour or footprint inflation. Fix: the pre-grasp standoff is now object half-length + lifter half-length + margin +
0.12 m (at least 0.6 m). Navigation to it uses `PlanRequest.GoalSearch`: if a goal is in collision, nearby goals are tried before
failing (back-offs up to 0.5 m along -x, +-10 cm lateral, +-0.3 rad), and a `goal_adjusted` event is logged. The final grasp move
plans with the grasped object excluded (`PlanRequest.IgnoreObstacleIds`). Harness tests `LiftGeometryTests` reproduce the CRANE
geometry: the old goals collide, and the lift now succeeds.

Held objects (CRANE seed 4, carry plan `start_in_collision`): once a lift is confirmed, `FleetAutonomy.ObjectPicked` removes the
object's box from the planning map and adds it to the carrier's footprint (`held:<id>`, at the grasp pose) for every plan until
release. `ObjectReleased` drops it from the footprint at release. A payload confirmed on the decks then rides in the receiving
assembly's footprint; a payload observed elsewhere is re-inserted into the map at its observed pose (`ReinsertObstacle`). If the
map has no box for the object, the lift task's optional `payload_type` gives its size. No interface change: CRANE keeps the
payload in the map.

## Known gaps

No dynamic obstacle avoidance between moving agents (other agents are static obstacles at plan time; tasks own their rovers
exclusively). No relative estimate from a moving target (target holds still). Lift/handoff are thin (no contact model of the
gripper, payload placement by the harness's simple rule). Peer-observer views are supported by the estimator but the harness
exercises only own camera and overhead. The harness plant is kinematic (no traction limit, no inertia, locked wheel = hard
constraint), so "a two-module group cannot drag a locked wheel" (q3) is not modelled; tracking quality in CRANE's dynamic
plant needs re-tuning of `AutonomyAssumptions`.
