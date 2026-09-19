# Reference environments

CRANE reference environments keep three layers separate: canonical geometry/collision,
simulation semantics/physics, and optional visual presentation. Visual objects never own the
authoritative colliders. Every canonical object has a stable `CraneSemanticIdentity`; this is a
provenance handle, not proof that a robot sensed or consumed information about the object.

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
./Builds/CRANE-Worker/CRANE.x86_64 -batchmode -nographics \
  --crane-aerial-validation --crane-aerial-scene "PX4 Walls Validation" \
  --crane-output /tmp/crane-px4-walls-validation.json
```

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
and one 2-D LiDAR configuration. ROS sensor transport, Nav2 traversal, spawn/goal calibration,
corridor-width checks, native Gazebo comparison, and high-fidelity material matching are
**NOT_RUN**; the run must not be presented as those validations.

## Status

- TurtleBot3 warehouse: **IMPLEMENTED / TESTED** headless and with Nav2.
- F1TENTH conversion and Unity import: **IMPLEMENTED / TESTED** on synthetic fixtures and the
  pinned external Spielberg map; full-lap controller/ROS validation **NOT_RUN**.
- PX4 walls: **IMPLEMENTED / TESTED** headlessly; ArUco: **IMPLEMENTED / TESTED** for layer
  separation, camera detection **NOT_RUN**; windy: **IMPLEMENTED / TESTED** for deterministic
  directional response, physical calibration **NOT_RUN**.
- Clearpath pipeline offline import: **IMPLEMENTED / TESTED** for source resolution, layer
  separation, bounds, semantic ray query, and mesh contact; navigation/sensor transport **NOT_RUN**.
- AWSIM/Flightmare: design references; asset reuse **DEFERRED** pending per-asset terms.
