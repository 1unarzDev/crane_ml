# Replacing provisional parts with CAD (procedure + designer checklist)

Everything in `~/.cache/crane-fleet/assets_v0/` is generated placeholder geometry (`provenance: provisional`). Sections can be swapped one at a time; the rest stays provisional.

## 1. Export formats and units
* **Meshes:** one **OBJ per movable part**, triangulated, with normals and UVs, material groups via `usemtl`. Export in **metres**. If the CAD tool exports millimetres, scale by 0.001 on export (a 420 m long rover is the typical symptom; `scale.*` checks catch it).
* **Axes:** x forward, y left, z up, right-handed (not Unity's frame; the consumer converts, see `ASSET_MANIFEST_SPEC.md`). Many CAD tools are Y-up: rotate on export, do not "fix" it in Unity.
* **Part frame:** each part's OBJ is in its **own frame**: origin at the joint pivot (hooks, wheels axle centre, lift carriage origin, finger root), orientation as in the table below. Static parts (`chassis_body`, `deck`, `mast`) use the `base_link` frame: **origin at the centre of the body underside**, ground plane at z = -ground_clearance.
* **Collision:** boxes/cylinders/spheres listed in the manifest, or convex-hull OBJs (`type: convex_mesh`, `file`). Keep cables, bumpers and clearance envelopes here, not in the render mesh.
* **Mass properties:** from the CAD mass-properties report **about the centre of mass, axes parallel to base_link (not principal axes)**, kg and kg·m². Fill the full 3x3 tensor. Careful: many tools print *products* of inertia `Pxy = ∫xy dm`; the tensor entry is `Ixy = -Pxy`. Total mass includes wheels, cables, battery; list the arm-proxy mass separately under `optional_masses`.
* Provenance for anything measured on hardware (mass on a scale, COM by pendulum) is `measured`, not `cad`.

## 2. Naming in CAD (maps 1:1 to the manifest)
| CAD body / coordinate system | manifest |
|---|---|
| `chassis_body` | render part, role `body` |
| `wheel_FL`, `wheel_FR`, `wheel_RL`, `wheel_RR` (rollers as sub-groups `roller_00..NN`, roller 00 = the one touching the ground at the authored pose) | wheel parts |
| `hook_front_a`, `hook_front_b`, `hook_rear_a`, `hook_rear_b` (A: face y>0, B: face y<0), frames `FRAME_pivot_<face>_<a\|b>` | hooks + `hook_pivots` |
| `FRAME_face_front/rear/left/right` (x out of the face, z up) | `docking_faces[].origin` |
| `FRAME_catch_<face>_<n>` | `catch_pins[].position` (face frame) |
| `FRAME_engage_<face>_<a\|b>` (point in the hook crook, hook closed) | `hooks[].engage_point` |
| `tag_front`, `tag_rear`, `tag_top` (+ `FRAME_tag_*`, z into the tag) | tag parts and frames |
| `mast`, `stereo_bar`, `lidar_base`, `lidar_head`, `rear_camera`, `arm_pedestal`, `arm_proxy_envelope` | scout parts |
| `lift_column`, `fork_carriage`, `finger_left`, `finger_right`, `front_camera`, `front_tof` | lifter parts |
| `deck`, `front_camera`, `front_tof` | cargo parts |
| `FRAME_cam_*` (x = boresight), `FRAME_lidar`, `FRAME_tof_*`, `FRAME_imu`, `FRAME_com`, `FRAME_wheel_<FL..RR>` (axle centre, y = spin axis, + = forward) | `sensors.frames`, `mass_properties.com_xyz`, `wheels.items` |
Material slots: use the existing names (`shell_<role>`, `guide`, `hook`, `pin`, `roller`, `wheel_hub`, ...) or add new ones to `render.materials`.

## 3. Procedure
1. `cp -r ~/.cache/crane-fleet/assets_v0 ~/.cache/crane-fleet/assets_cad_v1` (or a repo path). Keep `generator_defaults.json` untouched.
2. Replace OBJ files for the parts you have; keep names. Update `render.parts[].origin/joint/materials` if frames moved.
3. Replace the matching **section** values (collision / mass_properties / wheels / sensors / attachments / actuated). In every section you replaced set `"provenance": "cad"` (or `"measured"`) and `"source_file"` to the CAD export or measurement sheet (path relative to the manifest or repo root). Sections you did not replace stay `provisional`; never mix provenances inside one section.
4. **Update `Tools/Fleet/spec/fleet_profile_v0.json`** (single source of truth): put the CAD value in, set its `provenance` to `cad`/`measured`, bump `profile_id` (e.g. `mecanum_fleet_cad_v1`) and the file name if you keep versions, and set the same `profile.profile_id` and `sha256` in every manifest. The validator fails if manifest and profile disagree; fix the profile, not the tolerance.
5. Run `python Tools/Fleet/assets/validate_manifest.py <assets>/*/*.manifest.json --report report.json`. Must pass (no FAIL): `schema`, `files.*`, `scale.*`, `profile.*`, `wheels_ground`, `symmetry`, `roller_handedness`, `mass.*`, `winding`, `actuated`, `mating.*` (gap within [-5, +10] mm, no body/hook overlap, hook engage points within `hook_position_tolerance` of the catch pins, guide axes collinear). Review each WARN (`bbox.render_vs_collision`, `watertight`, `mating.*.guide_width`, `mating.*.other_parts`); a WARN you keep needs a sentence in the section `note`.
6. Run `render_preview.py --assets <assets>` and look at the PNGs (roller lean in the top view, hooks, tag orientation).
7. Hand the directory + `report.json` to the Unity side. Commit generator changes only if you changed generator code.

## 4. Mechanical designer checklist (one page)
Everything below is currently `provisional` (84 profile fields + 97 generator defaults; the complete list with current values: `python Tools/Fleet/assets/designer_checklist.py --markdown`). **Geometry** = CAD export, **M** = number or measurement.

**A. Chassis and wheels**
- Geometry: body shell (outer size 0.42 x 0.30 x 0.10 now), chamfers, wheel wells (cut-back now 0.03 m), top plate, underside; mecanum wheel model (radius 0.0485, width 0.045, **roller count, roller angle 45 deg, roller profile, hub, hand per corner**).
- M: wheel centres (lx 0.14, ly 0.175), ground clearance 0.03, overall width with wheels 0.40, encoder counts/rev, max wheel speed and torque, max speed/yaw rate.

**B. Mass properties (per role: scout, lifter, cargo; scout also with arm proxy)**
- M: total mass, COM xyz in base_link, full inertia tensor about the COM (CAD report; weigh the real rover and compare).

**C. Docking face (front/rear active, left/right available)**
- Geometry: pocket (depth 0.04, half-height 0.025, width now derived from hook sweep), **two hooks per face** (arm length/width/thickness, prong, pivot position, up/down stagger +-0.011 so identical faces mate, or the real coupler), catch pins (position, diameter), guide funnel (chamfer width >= 0.010, flare / acceptance angle 0.087 rad), tag position, magnet seat.
- M: face poses and heights `[x,y,z,yaw]` for all four faces (**left/right faces sit at y = +-0.20, outside the 0.15 body and the wheel envelope: mounting concept?**), hook spacing 0.12, hook open/closed angles and travel, actuation time 0.8 s, rated load 60 N, position tolerance 2 mm, yaw tolerance 0.035 rad, magnet type, pull-in range, hold force 25 N, gap, connector lateral compliance.

**D. Sensors and tags**
- Geometry + M: mounting pose of each camera (stereo pair baseline 0.075, rear/front cams), LiDAR, ToF, IMU; body sizes; lens choice (hfov 1.40 now), rates, latency, ToF fov/range, LiDAR range/points; AprilTag positions and sizes (face 0.05, top 0.12), id allocation, printed black-border size.

**E. Role hardware**
- Scout: mast (position, diameter, height 0.35), stereo bar, LiDAR puck size, rear camera, **arm base position and reach envelope (0.45)**, arm mass 2.5 / payload 0.5.
- Lifter: column and carriage layout, **how the front docking pocket stays usable (carriage plate currently blocks it unless raised)**, lift travel 0.15, speed 0.05, capacity 5 kg, fork plate size, gripper finger size, opening range (0.30 max, min opening?), close time 1.0 s.
- Cargo: deck size 0.40 x 0.30, **deck height 0.14 (taken as top above ground)**, lip, capacity 8 kg, tag position when payload covers the top.

**F. Collision**: simplified envelopes including cables and bumpers; which parts may touch the payload.

**G. Payloads (box_small 0.30x0.20x0.15 / 2 kg, box_long 0.80x0.25x0.15 / 6 kg, tray_wide 0.60x0.60x0.08 / 5 kg)**: real sizes, mass, COM, contact patches (feet), grasp faces.

**H. Measured on hardware (not CAD)**: floor friction / lateral efficiency / slip for tile, carpet, rubber; actuator latencies; IMU noise; camera latency.
