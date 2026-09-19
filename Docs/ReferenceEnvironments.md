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
ROS bottom-left origin and YAML yaw, extracts occupied-cell boundaries, merges collinear edges,
and emits deterministic Unity-ready collider descriptors with semantic IDs and source hashes.

```bash
python3 Tools/ReferenceEnvironments/f1tenth_map_generator.py \
  /path/to/map.yaml --output /tmp/map.canonical.json
```

Third-party maps remain external inputs. The official racetrack collection is GPL-3.0 and has
additional layout-provenance concerns, so no track data is committed by default.

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

## Status

- TurtleBot3 warehouse: **IMPLEMENTED / TESTED** headless and with Nav2.
- F1TENTH conversion: **IMPLEMENTED / TESTED** on a synthetic fixture; external tracks **NOT_RUN**.
- PX4 walls: **IMPLEMENTED / TESTED** headlessly; ArUco: **IMPLEMENTED / TESTED** for layer
  separation, camera detection **NOT_RUN**; windy: **IMPLEMENTED / TESTED** for deterministic
  directional response, physical calibration **NOT_RUN**.
- Clearpath conversion: **DEFERRED** until native formats are healthy.
- AWSIM/Flightmare: design references; asset reuse **DEFERRED** pending per-asset terms.
