# Unity import and flight check for the AOS 7 V5 CAD model (READY TO RUN, NOT RUN)

Gate: the orchestrator releases Unity (modular pilot owns the machine). All commands go through
`~/.cache/research/fs-scripts/unity-drones` (AGENT_RULES 1-10): one invocation at a time, no retry loops, no full EditMode run,
`-nographics` for import/compile, `--render` only for captures. Seed 1337 everywhere.
Nothing below modifies `Tools/Aerial/profiles/*.json`, the RoboBoat defaults or shared asmdefs.

## 0. What is already in place (no Unity needed)
* `Assets/Robot Descriptions/aos7v5/aos7v5.urdf` plus `meshes/visual/*.stl` (per link and material) and `meshes/collision/aos7v5_hull.stl`.
* `Assets/Robot Descriptions/aos7v5/Aos7v5.urdfprofile.asset`: rootYawDeg 0 (REP-103), no rules. Without it `UrdfSync.FindProfile`
  walks up to `Assets/Robot Descriptions/Surface.urdfprofile.asset` and would add boat Thruster/Buoyancy components to a drone.
  Unity will create the `.meta` files (new GUIDs) on the first import; commit them afterwards like the boats'.
* `aos7v5_multirotor.yaml` (physics sidecar) and `Tools/DroneCAD/aos7v5/proposed_profile_aos7_cad.json` (CRANE-schema profile).

## 1. Import the URDF (mesh import + prefab sync + frame check)
```
~/.cache/research/fs-scripts/unity-drones /tmp/aos7v5_sync.log \
  -executeMethod Crane.Urdf.UrdfSync.Batch -urdf "Assets/Robot Descriptions/aos7v5/aos7v5.urdf"
```
Expect exit 0, a `URDF_SYNC ok prefab=Assets/Robot Descriptions/aos7v5/aos7v5.prefab ... errors=0` line and
`worst_frame_mm <= 1.000 worst_frame_deg <= 0.100` (`UrdfSync.MaxPosErrorM / MaxAngErrorDeg`). Warnings about links without
`<inertial>` (the 9 sensor frames) are expected: the boats' `laser_frame` is the same. Then verify the committed state:
```
~/.cache/research/fs-scripts/unity-drones /tmp/aos7v5_check.log \
  -executeMethod Crane.Urdf.UrdfSync.Batch -urdf "Assets/Robot Descriptions/aos7v5/aos7v5.urdf" -check
```
Pass criteria: exit 0; 30 links (21 physical + 9 frames); four `rotor_i` continuous joints about local up;
`oak_d_rgb_camera_optical_frame` has +Z along the camera axis (forward).

## 2. Prefab adapter and the rotor spinner (written, compiles against the Unity DLLs outside Unity; not yet run in the editor)
New asmdefs (own folders, no shared asmdef touched): `Assets/Drones/Models/Rotors/` = `Crane.Drones.Models.Rotors` (runtime `RotorSpinner`),
`.../Editor` = `Crane.Drones.Models.Rotors.Editor` (`AOS7V5PrefabAdapter`), `.../Tests/Editor` = `Crane.Drones.Models.Rotors.Tests`.
* `RotorSpinner` turns each `rotor_i` link transform about its own shaft (local up) by `-spinSign * omega * dt` (visual only; no physics),
  composes with the link's rest rotation, keeps the hub fixed, and optionally fades the blades into a blur disc above 90..280 rad/s.
  Feed it `SetRotorSpeeds(plant.RotorSpeeds)` (index = motor-1: FR CCW, RL CCW, FL CW, RR CW).
* `AOS7V5PrefabAdapter.Run` instantiates `Assets/Robot Descriptions/aos7v5/aos7v5.prefab`, unpacks it, **removes every ArticulationBody**
  (the URDF sync puts one on each link; the plant is kinematic), checks each rotor shaft is body-up, adds the spinner and saves
  `Assets/Drones/Models/AOS7V5Quad.prefab`. It does not add `DroneVisualRig` (the procedural rig); the demo/flight CLIs that read a
  `DroneVisualRig` need a one-line shim in your own asmdef, or the spinner can be driven by the plant adapter directly.
```
~/.cache/research/fs-scripts/unity-drones /tmp/aos7v5_adapter.log -executeMethod Crane.Drones.Models.AOS7V5PrefabAdapter.Run
~/.cache/research/fs-scripts/unity-drones /tmp/aos7v5_rotor_tests.log -runTests -testPlatform EditMode -assemblyNames Crane.Drones.Models.Rotors.Tests -testResults /tmp/aos7v5_rotor_tests.xml
```
Tests (`RotorSpinnerTests`): URDF rotor joints are continuous about +z with parent `motor_i`; world positions equal the quad-X shaft positions
(+-0.127, +-0.092, 0.040); the spinner turns 4 rigs with the right CCW/CW sign and magnitude (0.1 rad per 10 rad/s x 0.01 s), keeps hubs fixed and tips on their circle;
angles stay bounded; and (once the prefab is synced) `rotor_i` are separate transforms under `motor_i` with body-up shaft axes and their own meshes.
Import path: `UrdfSync.FindProfile` looks in the URDF's own folder first, so `Aos7v5.urdfprofile.asset` (rootYawDeg 0, no rules) wins over
`Surface.urdfprofile.asset` (boat thruster/buoyancy rules never apply); the sync builds one transform per URDF link, so the rotors stay separate.

## 3. Physics profile for the test
Copy (do not overwrite) `Tools/DroneCAD/aos7v5/proposed_profile_aos7_cad.json` to a scratch directory and point the plant at it by absolute
path (`CraneAerialVehicle.profile` accepts an absolute path; `CraneAerialProfile.Resolve` does too). The CRANE profile
`aos7_v5_f90_6s` stays the default and the baseline.
For the two CLIs that hard-code the name/prefab (`DroneForestFlightCli`, lines 27 and 84), make the check on a scratch copy of the
CLI in an editor asmdef of your own (change only `PrefabPath` to `AOS7V5Quad.prefab` and the `"aos7_v5_f90_6s"` literal to the absolute
path), never in place. Both profiles run through the same code path, so any difference is the model.

## 4. Flights (seed 1337, logged, split guard as usual)
Hover (6 s, repro twice, bit-identical expected; compare against the baseline run of the same CLI with the current profile):
```
~/.cache/research/fs-scripts/unity-drones /tmp/aos7v5_hover.log -executeMethod Crane.Drones.Logging.DroneReproCli.Run \
  --drone-seed 1337 --drone-out /tmp/aos7v5_hover --drone-duration 6 --drone-repeat 2
```
Corridor (Drone Forest, onboard OAK-D cameras at the CAD optical frame, render mode):
```
~/.cache/research/fs-scripts/unity-drones --render /tmp/aos7v5_corridor.log -executeMethod Crane.Drones.Integration.DroneForestFlightCli.Run \
  --drone-seed 1337 --drone-out /tmp/aos7v5_corridor --drone-duration 40 --drone-speed 3.5 --drone-wind calm
```
(add `--drone-splits <file>` / `--allow-heldout` exactly as the existing runs do; copy the invocation from `Docs/DroneVisuals/flights/seed1337_run1/manifest.json`.)

## 5. Pass / fail
* Hover: z drift < 2 cm over 6 s after the 1 s settle; position error is dominated by the trim spread (2.4% rotor-speed spread,
  CoM 3 mm aft of the rotor centroid), which the cascaded controller must absorb: report the pitch integrator state.
* Corridor: no collision, cross-track <= the baseline run's; log hover thrust fraction 0.209 and rotor rpm 8965 (CAD) versus
  0.157 and 7767 (baseline); report the difference in `manifest.json`, not a pass/fail on it.
* Visuals: OAK-D optical frame renders forward; rotor hubs spin the right way (rotor_0/1 CCW, rotor_2/3 CW from above); L2 frame
  upright; no mesh missing (STL imports with the per-material names).
* If the stock importer rejects the STL scale: the meshes are in metres, `scale` is not set (same as the Blastoise T200 note).
