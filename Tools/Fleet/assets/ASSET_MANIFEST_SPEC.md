# crane.fleet_asset_manifest/v0 — specification

Schema file: `fleet_asset_manifest_v0.schema.json` (JSON Schema 2020-12; `minischema.py` validates it without third-party packages).
One manifest per rover role (`scout`, `lifter`, `cargo`) and one for `payloads`. Written by `gen_provisional.py`
(provisional) or by hand/CAD export script (cad), checked by `validate_manifest.py`.

## 1. Coordinate conventions (read this first)

| | Manifest / OBJ / profile | Unity |
|---|---|---|
| handedness | right | **left** |
| x | forward | right |
| y | left | up |
| z | up | forward |
| units | metres, kilograms, radians | metres |

Pose = `{xyz:[m], rpy:[rad]}`, child frame expressed in its parent; `R = Rz(yaw) Ry(pitch) Rx(roll)` (URDF).
`base_link` origin is the centre of the body underside; the ground plane is `z = -ground_clearance`.
OBJ vertices are in the **part frame** (not the base frame); the part's `origin` places it in its parent.
OBJ winding is counter-clockwise seen from outside; `vt` origin is bottom-left.

### Conversion to Unity
`(x_u, y_u, z_u) = (-y, z, x)`, matrix `C = [[0,-1,0],[0,0,1],[1,0,0]]`, `det C = -1`. Because the determinant is -1:
1. **Reverse triangle winding** `(a,b,c) -> (a,c,b)` (Unity front faces are clockwise in its left-handed frame). Forgetting this gives inside-out meshes.
2. Rotations conjugate: `R_u = C R C^T`. A yaw of `psi` (left turn) is Unity euler **y = -psi** (degrees `-psi*180/pi`). Quaternion `(x,y,z,w) -> (y, -z, -x, w)`.
3. Normals map like positions; UVs are unchanged; joint axes map like positions, and the rotation sense flips together with (2).

Worked example: front-face hook A pivot `(0.21 - 0.03, 0.06, 0.06 + 0.011)` — or simply the front face point `(0.21, 0.06, 0.06)` — becomes `(-0.06, 0.06, 0.21)` in Unity
(0.21 m ahead = +z, 0.06 m to the left = -x, 0.06 m up = +y). Rear face yaw `pi` becomes euler y = -180. `frames.py` implements and tests all of this (`frames.worked_example()`).

If Unity's built-in OBJ importer is used instead of a runtime loader: it negates x and flips winding itself, so feed it OBJ coordinates `(y, z, x)` (a pure rotation of our frame, right-handed y-up) and the result equals the conversion above.

## 2. Sections (each has its own `provenance` ∈ provisional|cad|measured and `source_file`)

| section | content |
|---|---|
| `render` | `materials` (slot -> colour/alpha/texture), `parts[]`: `name`, `file` (OBJ, relative to the manifest), `parent` (base or another part), `origin`, `materials` (slots used), `kind` (solid\|quad), `default_visible`, `hideable_group`, `joint{type,axis,limits,authored_value}`, `role_in_assembly` (body, hook, wheel, tag, sensor, lift, gripper, deck, envelope, marker). Movable parts (wheels, hooks, lift carriage, fingers, lidar head) are separate OBJs. |
| `collision` | `shapes[]`: box (`size`), cylinder (`radius`,`height`, axis = shape z), sphere, or convex_mesh (`file`); each with `parent` + `origin`. May differ from render geometry. |
| `mass_properties` | `mass`, `com_xyz` (base frame), full 3x3 `inertia_tensor` about the COM in base-parallel axes, `optional_masses` (e.g. arm proxy), `derivation`. |
| `wheels` | `items[FL,FR,RL,RR]`: `position` (axle), `axis` (+y), `radius`, `width`, `roller_count`, `roller_angle` (45 deg), `roller_angle_sign`, `ground_roller_axis_xy`, `render_part`, `collision_shape`. |
| `sensors` | `frames[]`: camera (link frame + `optical_origin`), lidar2d, tof, imu; `params` from the profile. |
| `attachments` | `docking_faces[]` (front, rear modelled; left, right frame only), `hook_pivots[]`, `tags[]` (family, id, size, frame). |
| `actuated` | `hooks[]` (open/closed/authored angles), `lift` (axis, range, docking_value), `gripper` (fingers, opening range), `lidar_spin`, `arm_proxy`. |
| `payloads` | (payload manifest only) size, mass, COM, inertia, collision, `contact_patches`, `grasp_faces`. |

Replacing one section from CAD (say `mass_properties`) changes only that section's provenance/source_file.

## 3. Wheels: standard X convention
Wheel frame: origin at the axle, axes parallel to base; **spin axis +y for all wheels; positive spin rolls the rover forward**.
`roller_angle_sign` s is the sign of the y-component of the ground-contact roller axis `a = (sin α, s cos α)` (α = 45 deg to the axle), oriented forward.
Rule `s = -sign(x·y)`: FL -1, FR +1, RL +1, RR -1 (FL‖RR, FR‖RL). Inverse kinematics `w_i = (vx + k_i vy + (lx+ly) kz_i wz)/r`,
`k=(-1,+1,+1,-1)`, `kz=(-1,+1,-1,+1)`; i.e. `vy = r/4(-wFL+wFR+wRL-wRR)` (strafe left when FL,RR reverse and FR,RL forward).
Naming caveat: sources disagree on whether "X" refers to the ground-contact rollers or the top-visible ones; the kinematic table above is authoritative.
Seen from above, the **top-visible** rollers form an "X" (FL `\`, FR `/`, RL `/`, RR `\` with forward up and left on the left); the ground-contact rollers form the diamond.
**Verify:** (1) all forward → +x; (2) FL,RR reverse + FR,RL forward → +y; (3) `validate_manifest.py` (`roller_handedness`) recomputes the lean of the lowest roller group of every wheel OBJ (principal axis) and compares with the rule; (4) look at the top view preview.

## 4. Sensors
Camera `origin` is the link frame (x = boresight, y left, z up). `optical_origin` is the optical frame **x right, y down, z forward (optical axis)**, i.e. `R_opt = R_link · [[0,0,1],[-1,0,0],[0,-1,0]]`. In Unity a camera looks along +z with y up: use the link frame converted with `C`; image rows run opposite to optical y.
LiDAR frame: scan in its xy-plane, angle counter-clockwise from +x, z up. ToF boresight +x. IMU frame = base axes.

## 5. Attachments
**Docking face frame:** origin on the mating plane at the face centre, `+x` = outward normal (away from the rover), `+z` up, `y = z × x`.
**Mating:** `T_world_B · F_B = T_world_A · F_A · Rz(π)`: origins coincide, outward normals oppose, z stays up, y flips.
Faces are hermaphroditic. Each has two hooks (hook A at face y>0 on the *upper* level +dz, hook B at y<0 on the *lower* level -dz, so identical faces never collide) and two catch pins (`pin_hi` at y=-yc upper level, `pin_lo` at y=+yc lower). A hook's `engage_point` (hook frame, closed) must sit on the opposing face's catch pin when mated; the validator measures this. Hooks are authored closed; `open_angle` swings them out of the way (mirror-signed for A/B). The guide is a chamfered funnel at the pocket mouth (material slot `guide`), `chamfer_width >= profile.guide_half_width`.
**Hook pivot frames:** origin at the pivot, orientation = face frame, rotation about +z (right-hand rule, in the face frame).
**Tag frame (AprilTag3 convention):** x right, y down as seen by a viewer, z *into* the tag; `outward_normal` is given explicitly. `size` = edge of the black border square (detector convention); the textured quad is `size·10/8` with the one-cell white quiet ring. Textures: `tag36h11_<id:05d>.png`, generated by `apriltag36h11.py` from the published family codes, verified against reference PNGs of AprilRobotics/apriltag-imgs.

## 6. Joint semantics for the consumer
Part world pose = `parent_world · origin · J(q - authored_value)`. Revolute/continuous: rotation about `joint.axis` (part frame) by `q-authored` rad; prismatic: translation along `axis` by `q-authored` m. Hooks: `q` = angle, closed = authored = 0. Lift (`fork_carriage`): `q` in `[0, 0.15]` m; the front docking pocket is blocked unless `q = lift.docking_value`. Fingers: travel `q` m along `axis` (toward the centre) from the fully open authored pose. Wheels: spin about +y. Parts with `default_visible=false` (arm envelope, side-dock markers) are hidden unless enabled.

## 7. Validator coverage
`schema`, `files.*` (all referenced files, OBJ contents, material slots), `scale.*` (mm/m), `profile.*` (wheel positions, face poses, deck height, tags, sensors, lift/gripper/mast), `wheels_ground`, `symmetry`, `roller_handedness`, `mass.*` (positive definite, triangle inequality, box-consistency factor 3, COM inside collision), `bbox.render_vs_collision`, `watertight`, `winding`, `actuated`, `mating.*` (gap, overlaps by column integration, hook engagement, guide alignment).
