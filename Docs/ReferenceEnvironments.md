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

## Status

- TurtleBot3 warehouse: **IMPLEMENTED / TESTED** headless and with Nav2.
- F1TENTH conversion: **IMPLEMENTED / TESTED** on a synthetic fixture; external tracks **NOT_RUN**.
- PX4 walls/ArUco/wind: **NOT_RUN**.
- Clearpath conversion: **DEFERRED** until native formats are healthy.
- AWSIM/Flightmare: design references; asset reuse **DEFERRED** pending per-asset terms.
