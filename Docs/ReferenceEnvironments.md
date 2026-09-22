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
Manifest version 2.1.0 also defines four scenario contracts: static full-width blockage, delayed
blockage, temporary blockage, and an occupied goal region. Scenario colliders remain canonical;
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

An isolated 1280 × 720 player inspection on 2026-09-22 confirmed readable overview and oblique
views, route/environment identity, semantic overlay state, visible canonical-collider wireframes,
and the trajectory layer. Manual keyboard polling was not directly exercised, so the interactive
gate remains conservatively `PARTIAL` rather than a full pass. The same build reran the headless
warehouse validator successfully; presentation tooling did not change its 20 canonical colliders,
20 collider-free renderers, or existing structural/physics/sensor/explanation verdicts.

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

- TurtleBot3 warehouse: v2 canonical layout and ecological route contracts **IMPLEMENTED**;
  structural/physics/sensor/headless/explanation gates and one representative west-aisle detour
  **PASSED**. Isolated overview/oblique/HUD/semantic/collider inspection is **PARTIAL** pending a
  direct manual-keyboard check; failure/recovery scenario qualification is **NOT_RUN**.
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
