# Reference environments

CRANE reference environments keep three layers separate: canonical geometry/collision,
simulation semantics/physics, and optional visual presentation. Visual objects never own the
authoritative colliders. Every canonical object has a stable `CraneSemanticIdentity`; this is a
provenance handle, not proof that a robot sensed or consumed information about the object.
`CraneSemanticEvidenceHighlighter` resolves those IDs during playback and applies a render-only
material-property overlay to matching presentation renderers. The validation fixture checks that
highlighting resolves at least one renderer and leaves the scene's collider count unchanged.
The pinned upstream architecture and asset-license review is recorded in
[ReferenceEnvironmentSourceAudit.md](ReferenceEnvironmentSourceAudit.md); AWSIM, Flightmare, and
Robotics Warehouse assets remain references rather than bundled dependencies.

## TurtleBot3 Waffle warehouse

`TurtleBot3 Warehouse Validation` adapts the integration and layout-generation patterns of
Unity's Nav2/SLAM example using CRANE-owned primitive geometry. The separate Robotics Warehouse
repository has no license file at the pinned revision, so none of its meshes or textures were
copied. TurtleBot3 dimensions come from the Apache-2.0 example.

The active ecological layout is the manifest-driven 20 × 26 m
`crane-industrial-warehouse-v2`, retained in
`Assets/Resources/ReferenceEnvironments/unity_turtlebot3_industrial_warehouse_v2.json`. It has
20 canonical boxes, eight semantic regions, and five route contracts covering alternative lower
aisles, a center route divider, cross-aisles, a narrow gate, clutter, a work zone, and a dead end.
Manifest version 2.2.0 also defines five scenario contracts: static full-width blockage, delayed
blockage, temporary blockage, a four-wall temporary recovery enclosure, and an occupied goal
region. Scenario colliders remain canonical;
their collider and presentation objects change state together at deterministic fixed-simulation
times, while scheduled and actual boundaries are retained only in evaluator truth.
The scene serializes the manifest asset so the exact route/layout contract is a player-build
dependency. The older simple-warehouse manifest and its successful short smoke remain historical
infrastructure; they are not evidence that the v2 ecological routes pass.

The 2026-09-19 headless Nav2 smoke succeeded for a 2 m goal in 6.08 s, displaced 1.469 m (within
the configured 0.55 m goal tolerance), captured 136 LiDAR scans and 22 populated costmap
observations, and ran at RTF 1.00007. This validates the integration path, not benchmark difficulty
or explanation fidelity.

On 2026-09-21, the first v2 cross-aisle calibration retained the canonical warehouse rather than
substituting the controlled corridor. The 13 m goal timed out after 60 s: the robot displaced
2.770 m, received 597 returned controller commands and 199 costmap observations, turned near the
center divider, then oscillated while creeping toward the side aisle. A second predeclared run
used a warehouse-only 0.22 m costmap radius and 0.55 m inflation radius, conservatively matched to
the manifest's 0.188 m base circumscribed radius. It also timed out at essentially the same pose
(2.775 m displacement), although maximum occupied costmap cells fell from 8,750 to 4,996. This
rules out oversized costmap inflation as the sole limiting cause; it is not a navigation pass.
The observed controller request of 0.8 m/s was also clamped by the physical base to 0.26 m/s while
its angular request was not proportionally scaled. These negative calibrations remain retained.

The next diagnostic found the decisive integration defect: ROS FLU positive yaw had been applied
as Unity positive-Y torque even though Unity `(x,z)` maps to ROS `(-y,x)`, reversing every turn
relative to the published pose. After correcting that coordinate boundary, matching the ecological
controller to the 0.26 m/s plant limit, and treating the manifest endpoint as a position-only goal
region, the unchanged 13 m goal succeeded in 54.63 s. The sampled trajectory was 13.61 m long,
excursed 2.00 m into the west aisle, passed the center divider, and returned toward the goal. The
valid run retained 527 controller commands, 204 costmap observations, zero clock rewinds, and zero
stale/rejected commands. A controlled 1 m corridor non-regression also succeeded with actual
motion. This is one nominal ecological route pass, not a recovery/blockage suite.

The headless warehouse validator independently reports 20 canonical colliders, 20 collider-free
renderers, 28 unique semantic identities, collision/drop support, a semantic LiDAR hit on
`rack-center-blocker`, unchanged-collider evidence highlighting, 0.150 m differential drive, and
19.1 degrees of turn response. Its aggregate nominal-route record reports `STRUCTURAL_PASS`,
`PHYSICS_PASS`, `SENSOR_PASS`, `NAVIGATION_PASS`, `HEADLESS_PASS`, and `EXPLANATION_READY` while
leaving failure/recovery and interactive gates `NOT_RUN` and the overall verdict `PARTIAL`.

The first scenario calibration is intentionally negative. Under the installed stock continuously
replanning Nav2 tree, a full-width barrier caused substantial route reversal and path changes but
the action remained active until the 75 s client deadline. An occupied 3 m goal likewise remained
active until its 45 s client deadline while Nav2 repeatedly passed replacement paths to the
controller. The final occupied-goal replication displaced 1.742 m, returned 439 controller
commands, retained 165 costmap observations, and ended with client status `timeout`. Neither run is
a Nav2 abort or recovery pass. Their launchers expect `timeout` so regression checks preserve the
observed behavior rather than laundering cancellation into mission failure. A separate, explicit,
bounded ecological BT/configuration is required before claiming terminal failure or recovery.

That bounded ecological policy now exists separately as
`nav2_warehouse_replanning_deadline.xml`. It retains the stock 1 Hz replanning and recovery
subtrees but wraps them in a 70 s BehaviorTree.CPP steady-clock `Timeout`; the fixture's client
deadline is later at 80 s. The exact policy pair produced nominal success in 53.38 s with 12.60 m
displacement, then produced an action abort in 71.10 s under the full-width barrier after 4.57 m
of exploratory displacement. Thus the result is a task-policy deadline, not a client timeout, and
the obstacle run cannot by itself establish physical causation. An earlier attempted root guard
using Nav2 Jazzy `TimeExpired` did not terminate: that condition returns failure while waiting and
reinitializes when ticked again from inactive state. The failed run is retained as negative
calibration rather than hidden. An instrumented replication retained 629 delivered BT transitions,
the task-policy `Timeout` entering `RUNNING`, and zero recoveries. The terminal transition may be
absent because the installed Nav2 Jazzy topic logger can omit terminal-tick transitions, so the
action result and exact BT hash remain necessary provenance. This makes the bounded deadline
mechanism explanation-ready for the narrow task-policy claim, not for obstacle causation.

The separate development tree `nav2_warehouse_replanning_recovery.xml` gives the same navigation
and recovery policy a 90 s task deadline. In the manifest's temporary-enclosure scenario, four
canonical walls activated around the nominal trajectory at simulation time 18.040 s and were
removed at 34.040 s. The first 70 s calibration exercised 15 recovery leaf invocations and resumed
motion after removal, but aborted about 1.4 m short of the goal; that negative result is retained.
One bounded 90 s follow-up succeeded in 82.93 s with maximum recovery feedback 20, 4,499 delivered
BT transitions, planner and controller failures, contextual costmap clears, system-level clearing,
spin, wait, and backup transitions. The exact policy SHA-256 is
`9fb490be79d16c482d9c1b1a8f2f946d142e0d21ed3d4f5901aa3065304329d1`. A matching unobstructed
control succeeded in 53.62 s with zero recoveries. These are development calibrations, not
independent explanation-study episodes; delivered transitions and evaluator-only wall timing do
not establish which physical observation Nav2 consumed or prove obstacle causation.

The prospectively fixed `warehouse_repetition_contract_v1.json` then required three distinct
runtime/navigation artifacts with the same exact scenario identity, success after at least one
recorded recovery, route acceptance, and every structural, physics, sensor, navigation, recovery,
headless, and explanation-readiness gate. The temporary-enclosure scenario passed 3/3 runs. They
displaced 12.590--12.597 m, sampled 14.295--15.918 m paths, completed in 70.522--82.931 s, and
reported maximum recovery feedback from 16 to 20. The configuration SHA-256 is
`e18a16cbf9524e1dec4df9fae352ea9255b9ec2d6343dfe2cb6cc8a4150fe242`; the contract SHA-256 is
`9cb872500cd19a638eb8f39446f125a93e3c63d2028d0dc22212c123ab0b295f`; and the aggregate summary
SHA-256 is `818fd1c4118c1579d76e50f9222f06ddf68868b17f11e8a8b89bfa977a062033`.
Recovery-count variability is retained rather than treated as a scenario invariant. These
exact-condition repetitions establish operational reproducibility, not three independent study
episodes, controller consumption, or physical causation.

The warehouse now attaches a presentation-only reference inspection controller to its spectator
camera. It provides top-down overview, oblique, and robot-follow views; a route/environment HUD;
the robot trajectory; semantic evidence highlighting; and visible wireframes for the canonical
box colliders. These overlays do not add or mutate colliders, rigid bodies, sensors, or navigation
state. Keyboard controls are `1`/`2`/`3` for views and `H`/`C`/`T` for semantic, collider, and
trajectory overlays. Equivalent launch flags make screenshots reproducible without synthesized
keyboard input:

```bash
CRANE.x86_64 --crane-profile interactive-high \
  --crane-scene "TurtleBot3 Warehouse Validation" --crane-disable-ros \
  --crane-inspection-view oblique \
  --crane-inspection-semantic-overlay --crane-inspection-collider-overlay
```

An isolated 1280 × 720 X11 player inspection on 2026-09-22 directly exercised keyboard input.
Keys `1` and `2` selected readable Overview and Oblique views; `H`, `C`, and `T` changed the HUD
and the corresponding semantic, collider, and trajectory presentation; and `3` selected Follow.
The first Follow audit exposed an empty view because the original chase position lay outside the
south warehouse boundary. The presentation-only controller now tests the chase sightline, falls
back to an unobstructed lateral view near a boundary, and retains an overhead last resort. The
rebuild showed the robot at readable scale with nearby canonical geometry while leaving that
geometry untouched. Final screenshot SHA-256 values are
`2c3faf7ff564b418529e4e5d527f320a4761f15949e4584440f38f7658bd4efe` (Overview),
`0048802b8a66088ba72377a8da2707547d7281c86affbf03ab5c5ddebf6822ef` (Oblique),
`3ff56d9237fa891236ec7aa7bed0c0929055025a41746ae1969eb6d6db073420` (overlays), and
`a242bc03496842466c3010061e9cd0acc04b59510ce68823d30cace2a521b7f7` (Follow).
The interactive gate therefore passes for the warehouse inspection contract. The same final build
reproduced the prior headless validator SHA-256
`cd3dde90795bfb1c76a60eba9d2de8f1246bb8b39ab2ab6cbdc4fa65a199eda6`, including 20 canonical
colliders, 20 collider-free renderers, and unchanged structural/physics/sensor/explanation gates.

## Configurable land navigation proving ground

`crane-land-proving-ground-v1` is a manifest-driven replacement for the warehouse geometry in the
existing TurtleBot3 validation scene. The original catalog at
`Assets/Resources/ReferenceEnvironments/crane_land_proving_ground_v1.json` has SHA-256
`bea854292d7298e04f37ed9f6cc50f49d59d51fdec0e1b78a9d5f9247beee720` and defines eight seeded
layouts: staggered obstacles, an S-turn slalom, offset gates, a narrow doorway, a U-trap, alternate
corridors, complete blockage, and a dynamic gate. The generator, not the launcher, owns manifest
validation, separate canonical and visual layers, stable semantic IDs, fixed-simulation-time
schedules, configuration hashes, and evaluator-only truth. The launcher selects only a layout and
seed. Existing corridor truth remains a separate schema and is unchanged.

The failed v1 slalom remains immutable. The additive
`crane_land_proving_ground_v2.json` catalog (SHA-256
`3f249258362fb47ceb135b7423e77ba5716bd1ebf4b3db7cf3650a8f8f0640f2`) contains only the corrected
`slalom-s-turn-v2` revision, with four alternating partial-width barrier banks. Launchers select
the catalog explicitly through `CRANE_PROVING_GROUND_CATALOG`; the default remains `v1`, so old
commands and artifact identities do not silently change.

The dedicated Nav2 fixture uses a 44 m longitudinal global costmap so the 18 m goal remains inside
the rolling window. The first alternate-corridors calibration used the earlier 30 m window and
aborted immediately; the planner explicitly reported the goal outside its bounds. This negative
calibration is retained rather than reclassified as a geometry or navigation failure.

All eight v1 layouts have runtime navigation evidence; seven satisfy their declared behavioral
gate, and the failed slalom has an additive qualified v2 revision:

- `alternate-corridors-v1` succeeded in 71.21 s with 17.573 m endpoint displacement, 17.930 m of
  sampled trajectory, a 1.10 m lateral excursion, 628 delivered BT transitions, 711 returned
  controller commands, 268 costmap observations, and zero recovery feedback;
- `dynamic-gate-v1` activated its canonical gate at simulation time 20.040 s and removed it at
  45.040 s, then succeeded in 91.38 s with maximum recovery feedback 1, a delivered `FollowPath`
  failure and contextual local-costmap clear, 22.328 m sampled trajectory, and 1.723 m lateral
  span; this sequence does not establish which scan Nav2 consumed or that the gate physically
  caused the recovery;
- `complete-blockage-v1` aborted under the separate 70 s bounded task policy at 70.33 s, before
  the 80 s client deadline, after 17.973 m of sampled exploratory motion and 4.54 m lateral span.
  This supports task-policy deadline provenance, not obstacle causation.
- `narrow-doorway-v1` passed the centered 1.0 m opening in 69.06 s with 17.467 m endpoint
  displacement, 17.542 m sampled path, 0.236 m lateral span, 610 delivered BT transitions, and
  zero recovery feedback. This is a narrow-passage success; it is not labeled replanning.
- `u-trap-v1` succeeded in 98.16 s after advancing into the trap region, reversing across 14
  sampled intervals, reaching 3.003 m lateral excursion on an exterior route, and returning to
  the goal. Its sampled path was 25.207 m versus 17.604 m endpoint displacement, with 871
  delivered BT transitions and zero recovery feedback. The trajectory establishes a substantial
  route change; delivered costmaps do not establish which observation caused it.
- `staggered-obstacles-v1` succeeded in 73.06 s with 17.546 m endpoint displacement and an
  18.331 m sampled path. Its trajectory reached +0.506 m and -0.347 m laterally and changed
  lateral direction three times, satisfying the separately versioned weave gate. The first launch
  on ROS domain 233 was infrastructure-invalid because Fast DDS could not derive valid ports; the
  valid rerun used domain 220.
- `offset-gates-v1` succeeded in 76.86 s with a 19.571 m sampled path. It traversed both sides of
  the centerline, reaching +1.597 m and -1.455 m with two sampled lateral direction changes, so it
  satisfies the offset-route gate.
- **NEGATIVE CALIBRATION RETAINED:** `slalom-s-turn-v1` succeeded in 69.46 s but followed the exact
  centerline for 17.478 m with zero angular command and zero lateral direction changes. The current
  bollards leave a straight route and therefore fail the declared S-turn navigation gate. This run
  is retained and is not counted as a qualified slalom.
- `slalom-s-turn-v2` succeeded in 82.96 s with 17.577 m endpoint displacement, a 21.122 m sampled
  path, lateral extrema of +1.349 m and -1.248 m, and four lateral direction changes. It delivered
  727 BT transitions, 818 returned controller commands, and 314 costmap observations with zero
  recoveries. These metrics satisfy the predeclared v2 S-turn gate; they do not establish which
  delivered observation Nav2 consumed.

The final Linux build passed the v2 headless validator with seven canonical colliders, seven
collider-free renderers, nine unique semantic IDs, a semantic sensor hit on
`slalom-bank-west-near`, evidence highlighting, collision/drop support, and differential
drive/turn response. It reports
`STRUCTURAL_PASS`, `PHYSICS_PASS`, `SENSOR_PASS`, `HEADLESS_PASS`, and `EXPLANATION_READY` while
leaving its aggregate navigation and interactive fields `NOT_RUN`; per-route evidence is recorded
separately above. Isolated 1280 × 720 inspection confirmed the v2 environment/layout HUD, semantic
highlight, collider wireframes, and trajectory state. A focused `2` keypress changed the HUD and
camera from Overview to Oblique, directly validating keyboard view control. The first keyboard
attempt omitted the named scene and opened the default aquatic scene; it is invalid calibration
and contributes no environment evidence.

`summarize_environment_qa.py` now resolves either a warehouse route or proving-ground layout
behind the same command interface. It verifies exact manifest and per-run configuration hashes,
scenario-contract parity, expected terminal status, navigation displacement, and independent
structural/physics/sensor/headless/explanation records. It emits artifact paths together with
their SHA-256 hashes and retains `interactive=NOT_RUN` unless that gate is supplied separately.
The separately versioned v1/v2 navigation-gate catalogs add declared trajectory/recovery
acceptance criteria without changing either canonical scene manifest or invalidating earlier run
identity. All eight scenario motifs now have a qualified revision producing `NAVIGATION_PASS`,
`HEADLESS_PASS`, and `EXPLANATION_READY`; dynamic-gate recovery-success and expected
complete-blockage abort also produce `FAILURE_RECOVERY_PASS`. The historical v1 slalom summary
remains `BLOCKED` while the v2 slalom summary passes. Aggregate records still do not silently infer
interactive evidence from headless runs.

`summarize_environment_repetitions.py` applies the prospectively written
`land_proving_ground_repetition_contract_v1.json` to distinct per-run QA artifacts. It requires
three runs with identical configuration identity, distinct runtime/navigation hashes, the declared
terminal status, route acceptance, and every required gate. Exact-condition repetitions establish
operational reproducibility only; the output explicitly rejects treating them as independent
scenario instances or statistical sample size.

Three explanation-relevant scenarios pass this repetition gate:

- `slalom-s-turn-v2`: 3/3 success and S-turn acceptance; 21.121–22.207 m sampled paths,
  82.961–89.875 s action time, exactly four lateral direction changes, and zero recoveries;
- `dynamic-gate-v1`: 3/3 recovery-followed-by-success; 22.382–23.330 m sampled paths,
  91.383–98.323 s action time, and maximum recovery feedback varying from 1 to 8;
- `complete-blockage-v1`: 3/3 abort under the 70-second BT task-policy deadline before the
  80-second client horizon; 17.519–18.151 m exploratory paths and zero recovery feedback.

The repetition contract SHA-256 is
`bfdb6d521c5e5476304169ab5b5bc55a2f00e21c3745f5e93b7df219821d1d17`. Aggregate result SHA-256
values are respectively `e5a10178e8104a3009e792af35b3e0f2bbb876617dd45c6b15266117e62bbc65`,
`21010265d272b2c4b6d738f2312ba31f40c8d824c0307c337a0259325033c112`, and
`f7c6a43286c06a5fe3c11873e355d0b70dc084515c56cf09d8175c8ce6b081fe`. The original first-run
summaries remain untouched; where they predated current artifact-hash fields, new derived summaries
were generated from the unchanged retained inputs.

```bash
CRANE_PROVING_GROUND_LAYOUT=alternate-corridors-v1 \
  Tools/Performance/run_land_proving_ground_nav2_fixture.sh

CRANE_PLAYER=/path/to/CRANE.x86_64 \
CRANE_PROVING_GROUND_LAYOUT=alternate-corridors-v1 \
  Tools/ReferenceEnvironments/run_reference_validation.sh land-proving-ground \
  /tmp/land-proving-ground.json

CRANE_PROVING_GROUND_CATALOG=v2 \
CRANE_PROVING_GROUND_LAYOUT=slalom-s-turn-v2 \
  Tools/Performance/run_land_proving_ground_nav2_fixture.sh
```

## Ecological explanation question contract

`Tools/ReferenceEnvironments/ecological_explanation_contract_v1.json` prospectively maps five
qualified land mechanisms to ten evidence-rich question contracts: warehouse recovery-success,
dynamic-gate recovery-success, bounded blockage termination, corrected S-turn trajectory shape,
and U-trap route change. Each question declares the robot-visible evidence fields it needs, the
claim classes those fields may support, and claims that must be withheld. Five questions require a
partial answer by construction because retained runtime evidence cannot establish physical cause,
controller consumption, no-path, optimality, or a counterfactual.

The contract is `DEVELOPMENT_ONLY_NOT_FROZEN`. It is additive ecological infrastructure and cannot
change the frozen F/G/H split, questions, prompts, or inclusion rules. Environment qualification
hashes select useful mechanisms but are not independent study episodes or model results. The
validator binds every entry to the exact manifest, environment, runtime seed, scenario/layout, and
configuration hash; rejects evaluator-only fields presented as robot-visible; requires distinct
question identities and explicit withholding boundaries; and requires a declared repetition pass
to identify at least three runs.

```bash
python3 Tools/ReferenceEnvironments/validate_ecological_explanation_contract.py
```

The validated contract SHA-256 is
`bfd344cb22ad307a7dd9be2fb885a555466aae3a5ca53e0e87f1fa2f62f7bd3c`.
This is a reproducible handoff to the explanation pipeline; explanation generation,
information-parity audit, blinded annotation, and statistical evaluation remain **NOT_RUN**.

The passive Nav2 fixture now retains an ordered, bounded `BehaviorTreeLog` stream using
`Tools/Performance/bt_transition_capture.py`. It assigns stable transition IDs and derives a unique
recovery invocation ID only from an observed `IDLE -> RUNNING` edge on an explicitly configured
recovery leaf. Nav2 feedback recovery counts are retained separately. Capture output also reports
duplicates, truncation, pre-goal records, open/incomplete invocations, configured terminal-node
observation, and limitations. Since Jazzy's topic does not provide a publisher sequence number,
the current capture conservatively reports whole-history completeness as `not_proven` and never
makes an exact recovery-count claim from the observed invocation list alone.

## F1TENTH occupancy maps

`Tools/ReferenceEnvironments/f1tenth_map_generator.py` reads standard PNG/YAML maps, applies the
ROS bottom-left origin and YAML yaw, traces occupied-cell contours, simplifies them at an explicit
metric tolerance, and emits deterministic Unity-ready collider descriptors with semantic IDs and
source hashes. An optional centerline CSV supplies a reproducible start pose; it does not replace
the occupancy map as the collision authority. `CraneF1TenthMapImport` constructs separate primitive
collision and presentation layers plus a planar F1TENTH-class Ackermann body and 270-degree LiDAR.

```bash
python3 Tools/ReferenceEnvironments/f1tenth_map_generator.py \
  /path/to/map.yaml --centerline /path/to/centerline.csv \
  --environment-id f1tenth-example-v1 --upstream-project owner/repository \
  --upstream-version COMMIT --output /tmp/map.canonical.json

unity run . --editor-version 6000.5.10f1 -- \
  -nographics -executeMethod CraneF1TenthMapImport.CreateScene \
  --crane-f1tenth-manifest /tmp/map.canonical.json \
  --crane-f1tenth-output-scene \
    "Assets/Generated/ReferenceEnvironments/example/F1TENTH Example Validation.unity"
```

Third-party maps remain external inputs. The official racetrack collection is GPL-3.0 and has
additional layout-provenance concerns, so no track data is committed by default. The 2026-09-19
proof used `f1tenth/f1tenth_racetracks` Spielberg at commit
`b95c4eff766f6367d66b310ea20cd2c9563712c0`. A 0.10 m tolerance reduced 29,200 raster boundary
edges to 290 canonical wall boxes across four closed contours. Two null-graphics runs were
byte-identical. The validator found 291 canonical colliders, 291 non-colliding visual renderers,
one LiDAR, a semantic horizontal ray hit on `track-wall-c0003-s0012`, four grounded wheels,
0.295 m planar drive displacement, and 5.388 degrees of steering response. The track body freezes
roll and pitch because the source occupancy benchmark is planar; this is not vehicle-dynamics
equivalence with F1TENTH Gym or hardware. Full-lap control, ROS transport, centerline adherence,
and comparison of simplified wall positions against the source simulator are **NOT_RUN**.

The 2026-09-19 semantic-playback regression highlighted the raycast evidence ID `track-floor` in
one presentation renderer while retaining all 291 canonical colliders. The result JSON SHA-256 was
`ff63701f54e625f420b72ad96485e0c5cf7c8efb2c185ea06f04562c01089662`.

## PX4 x500-class walls

`PX4 Walls Validation` reconstructs the four primitive boxes in PX4-gazebo-models
`worlds/walls.sdf` at pinned commit `bb0b9cf974acf4f1bcb5f5fcf80b88841562dea9`.
Gazebo ENU `(x,y,z)` is mapped explicitly to Unity `(x,z,y)`. Each source box remains a separate
canonical `BoxCollider` with a semantic ID; presentation cubes have no colliders. The source's
infinite ground plane is represented by a documented 100 m square test envelope matching its
visual extent.

The vehicle is an x500-class semantic/reference platform using CRANE's already validated
`MultirotorDynamics`. It is not a claim of PX4 SITL, actuator, or Gazebo dynamics equivalence.
Exact source hash, conversion, and collision contract are retained in
`Assets/Resources/ReferenceEnvironments/px4_gazebo_walls.json`.

The headless validation checks exact collider transforms, ray-query resolution to the expected
semantic wall ID, rigid-body contact against that wall, and the normal aerial dynamics suite.

```bash
Tools/ReferenceEnvironments/run_reference_validation.sh px4-walls \
  /tmp/crane-px4-walls-validation.json
```

The launcher is root-relative, always selects the requested scene with the `train-cpu` profile and
`-batchmode -nographics`, disables unrelated ROS transport, and verifies the scene against the
player's build manifest. This avoids briefly initializing the default aquatic scene, opening the
interactive submarine window, or starting an irrelevant ROS reconnect loop. It also supports `px4-aruco`,
`px4-windy`, `clearpath-pipeline`, and a generated `f1tenth-spielberg` scene when that scene was
included in the player build with `--crane-extra-scene`. TurtleBot3's closed-loop acceptance uses
`Tools/Performance/run_turtlebot3_nav2_fixture.sh` because it intentionally requires ROS/Nav2.

## PX4 ArUco landmark

`PX4 ArUco Validation` reconstructs the pinned 0.5 m square marker as a deterministic nearest-
neighbor 6x6 black/white texture. The semantic landmark and its visual quad have no collider,
matching the source SDF. A downward physics query through the tag must hit `ground-plane`, which
guards against a visual upgrade silently changing task feasibility. This validates render/collision
separation; camera-based marker detection remains **NOT_RUN**.

## PX4 windy

`PX4 Windy Validation` maps the pinned SDF wind `(5,2,0)` m/s from Gazebo ENU into Unity
`(5,0,2)` m/s and applies it through CRANE's air-relative multirotor drag model. Validation requires
measured displacement in both horizontal components. This makes the CRANE scenario behavior
checkable without claiming that the source SDF proves equivalent aerodynamic behavior in Gazebo.
Two headless repetitions were byte-identical and measured `(3.267, 4.732)` m displacement in
Unity x/z over the fixture interval. The non-proportional response is retained as an explicit model
calibration limitation; only deterministic signed response is claimed.

## Clearpath pipeline offline import

`Tools/ReferenceEnvironments/sdf_reference_converter.py` resolves local `model://` mesh
resources, hashes the world and every dependency, retains SDF model poses/scales and semantic
names, and emits a Unity import manifest. Source assets and the generated scene live under
`Assets/Generated/ReferenceEnvironments/`, which is intentionally Git-ignored. The manifest keeps
collision and visual roles separate even when the source names the same DAE for both. Collada
hierarchy is preserved; unsupported STL is converted deterministically to OBJ. Blender/FBX
conversion is **NOT_RUN** because Blender is not installed locally.

For Clearpath simulator 2.9.4 at commit
`ee098ad6f67b4e35d77841ed6f004b8f86cd77e4`, the pipeline proof used:

```bash
python3 Tools/ReferenceEnvironments/sdf_reference_converter.py \
  /path/to/clearpath_simulator/clearpath_gz/worlds/pipeline.sdf \
  --resource-root /path/to/clearpath_simulator/clearpath_gz/meshes \
  --output-root Assets/Generated/ReferenceEnvironments/clearpath_pipeline \
  --unity-asset-root Assets/Generated/ReferenceEnvironments/clearpath_pipeline \
  --environment-id clearpath-pipeline-2.9.4-v1 \
  --upstream-project clearpathrobotics/clearpath_simulator --upstream-version 2.9.4

unity run . --editor-version 6000.5.10f1 -- \
  -nographics \
  -executeMethod CraneReferenceEnvironmentImport.CreateImportedReferenceScene \
  --crane-reference-manifest \
    Assets/Generated/ReferenceEnvironments/clearpath_pipeline/manifest.json \
  --crane-reference-output-scene \
    "Assets/Generated/ReferenceEnvironments/clearpath_pipeline/Clearpath Pipeline Validation.unity"
```

This is an offline import, not a navigation run. The null-graphics flags prevent an unnecessary
Unity window; robot motion is validated separately by an explicit runtime fixture and goal.

The headless proof loaded 11 separate collision meshes and 13 renderers over bounds approximately
199.25 × 11.33 × 128.87 m, with no renderers in canonical collision and no colliders in visual
presentation. A physics query hit semantic object `clearpath-pipeline`, and a rigid-body drop
reported actual contact. The generated scene includes a Jackal-dimension/class differential body
and one 2-D LiDAR configuration.

`Tools/Performance/run_clearpath_pipeline_nav2_fixture.sh` reuses the stock land Nav2 fixture but
selects the generated scene, differential command adapter, and `base_scan`. The land bootstrap
recognizes this scene without disabling or replacing its imported canonical environment; synthetic
corridor walls and blockers are not permitted in this mode. A 2026-09-21 graphics-free smoke used
a temporary worker built with the generated scene and a 1 m forward goal. The action succeeded in
2.978 s after 10 returned controller commands and 0.497 m odometry displacement (within the 0.55 m
goal tolerance). It retained 451 LiDAR scans, four costmap observations with up to 11,668 occupied
cells, zero stale/rejected commands, valid transport, and RTF 1.00002. Topic/service delivery does
not prove controller consumption. The evaluator record identifies
`clearpath-pipeline-2.9.4-v1`, `clearpath-jackal-class-differential`, and
`referenceEnvironmentPreserved=true`.

This validates a local ROS/Nav2 motion and sensor-transport smoke on the imported geometry. It does
not validate a representative pipeline route, calibrated spawn/goal catalog, native Gazebo
equivalence, high-fidelity materials, or Jackal hardware dynamics. Those remain **NOT_RUN**.

The 2026-09-19 semantic-playback regression resolved `clearpath-pipeline` through the imported
model root to 10 presentation renderers without changing the 11 source-derived colliders. The
result JSON SHA-256 was
`00fa2d8d6d9febb0005cbbbe1a87fdc761e31d88dfbf429788ce488538e90686`.

## Status

- TurtleBot3 warehouse: v2.2 canonical layout and ecological route contracts **IMPLEMENTED**;
  structural/physics/sensor/headless/explanation gates and one representative west-aisle detour
  **PASSED**. Direct keyboard Overview/Oblique/Follow and semantic/collider/trajectory inspection
  **PASSED**; temporary-enclosure recovery-success passes a three-run exact-condition repetition
  gate. Its evidence-rich question/withholding contracts are **VALIDATED**, while explanation
  generation and annotation remain **NOT_RUN**.
- Configurable land proving ground: all eight deterministic layouts **IMPLEMENTED**; structural,
  physics, sensor, headless, explanation, and deterministic inspection controls **TESTED**;
  every scenario motif has one behaviorally qualified development run across the retained v1
  catalog and additive corrected v2 slalom. Direct keyboard view switching **PASSED** for v2.
  Corrected S-turn, dynamic recovery-success, and bounded blockage abort each pass a three-run
  exact-condition repetition gate. Contracts for those mechanisms plus U-trap route evidence are
  **VALIDATED**; explanation generation and annotation remain **NOT_RUN**.
- F1TENTH conversion and Unity import: **IMPLEMENTED / TESTED** on synthetic fixtures and the
  pinned external Spielberg map; full-lap controller/ROS validation **NOT_RUN**.
- PX4 walls: **IMPLEMENTED / TESTED** headlessly; ArUco: **IMPLEMENTED / TESTED** for layer
  separation, camera detection **NOT_RUN**; windy: **IMPLEMENTED / TESTED** for deterministic
  directional response, physical calibration **NOT_RUN**.
- Clearpath pipeline offline import: **IMPLEMENTED / TESTED** for source resolution, layer
  separation, bounds, semantic ray query, mesh contact, and a 1 m ROS/Nav2 sensor/motion smoke;
  representative route and native-Gazebo comparison **NOT_RUN**.
- AWSIM/Flightmare: **AUDITED** design references. Shinjuku and Flightmare environment art are not
  imported because the inspected terms do not provide a clean permissive redistribution path.
