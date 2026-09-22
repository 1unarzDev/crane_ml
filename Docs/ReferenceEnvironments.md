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

Provenance and layer contracts are retained in
`Assets/Resources/ReferenceEnvironments/unity_turtlebot3_simple_warehouse.json`.

The 2026-09-19 headless Nav2 smoke succeeded for a 2 m goal in 6.08 s, displaced 1.469 m (within
the configured 0.55 m goal tolerance), captured 136 LiDAR scans and 22 populated costmap
observations, and ran at RTF 1.00007. This validates the integration path, not benchmark difficulty
or explanation fidelity.

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

- TurtleBot3 warehouse: **IMPLEMENTED / TESTED** headless and with Nav2.
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
