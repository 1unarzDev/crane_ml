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
Pass criteria: exit 0; 29 links (20 physical + 9 frames); four `rotor_i` continuous joints about local up;
`oak_d_rgb_camera_optical_frame` has +Z along the camera axis (forward).

## 2. Swap into the aerial demo (adapter, editor-only, own asmdef)
The demo and flight CLIs load `Assets/Drones/Models/AOS7Quad.prefab` (hard-coded in `DemoSpec.prefab`, `AutonomyBatchCli.PrefabPath`,
`DroneForestFlightCli.PrefabPath`) and read a `DroneVisualRig` from it (hub local +Y = spin axis, body = Unity RUF, forward = +Z).
Do not edit those constants. Create `Assets/Drones/Editor/AOS7V5Adapter.cs` under the existing `Crane.Drones.Models` editor asmdef:

```csharp
// builds Assets/Drones/Models/AOS7V5Quad.prefab = aos7v5.prefab + DroneVisualRig (rotor map, markers). Batch: -executeMethod Crane.Drones.Models.AOS7V5Adapter.Run
public static void Run() {
  var src = AssetDatabase.LoadAssetAtPath<GameObject>("Assets/Robot Descriptions/aos7v5/aos7v5.prefab");
  var root = (GameObject)PrefabUtility.InstantiatePrefab(src);
  var rig = root.AddComponent<DroneVisualRig>();
  string[] names = { "rotor_0", "rotor_1", "rotor_2", "rotor_3" };           // PX4 motor 1..4 = FR, RL, FL, RR = CRANE M1..M4
  RotorSpin[] spin = { RotorSpin.CCW, RotorSpin.CCW, RotorSpin.CW, RotorSpin.CW };
  rig.rotors = new DroneVisualRig.Rotor[4];
  for (int i = 0; i < 4; i++) {
    var hub = FindDeep(root.transform, names[i]);
    float tilt = Vector3.Angle(hub.up, root.transform.up);                    // must be 0: hub local +Y = up
    if (tilt > 0.1f) throw new System.Exception($"{names[i]} hub up is {tilt} deg off body up; add a child pivot");
    rig.rotors[i] = new DroneVisualRig.Rotor { name = names[i], motorIndex = i + 1, hub = hub, spin = spin[i] };
  }
  rig.cameraMount = FindDeep(root.transform, "oak_d_rgb_camera_optical_frame");
  rig.lidarMount  = FindDeep(root.transform, "l2_lidar_frame");
  rig.imuMount    = FindDeep(root.transform, "cube_imu_link");
  var com = new GameObject("CoM").transform; com.SetParent(root.transform, false);
  com.localPosition = DroneVisualRig.FluToUnity(new Vector3(-0.002986f, 0f, 0.017271f)); rig.comMarker = com;
  PrefabUtility.SaveAsPrefabAsset(root, "Assets/Drones/Models/AOS7V5Quad.prefab");
}
```
(`FindDeep` = recursive Transform search by name.) The rotor links are rotated by the sync into Unity's frame; if the tilt assertion
fires, wrap each rotor in a pivot child instead of loosening it. Run:
```
~/.cache/research/fs-scripts/unity-drones /tmp/aos7v5_adapter.log -executeMethod Crane.Drones.Models.AOS7V5Adapter.Run
```
Pass: exit 0, no tilt exception, `DroneVisualRig.BodyFlu(rotor hub)` equals the sidecar `position_base_link_m` to 1 mm (log it from the adapter).

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
