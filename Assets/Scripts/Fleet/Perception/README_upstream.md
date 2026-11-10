# Crane.Fleet.Perception

Engine-independent perception model for the planned four-rover mecanum fleet. Targets `netstandard2.1`, C# 9, no engine references and no package dependencies (a 100-line JSON reader is included). Namespace root `Crane.Fleet`; the assembly name is `Crane.Fleet.Perception`.

Principle: each rover gets only the observations it could have on hardware. Ground truth is held in `Crane.Fleet.Perception.Evaluator`, which no rover-side type may reference; a reflection test over the assembly enforces this.

Build and test (niced, one process):

```
export TMPDIR=~/.cache/crane-fleet/tmp
nice dotnet test Tools/Fleet/core_cs/Crane.Fleet.Perception.sln
python3 -I Tools/Fleet/core_cs/Crane.Fleet.Perception.Tests/parity/check_parity.py
```

The tests locate `Tools/Fleet/spec/*.json` by walking up from the test binary. The ROS 2 wire contract is `Tools/Fleet/spec/ros2_test_interface_v0.md`.

## Layout

| Path | Contents |
|---|---|
| `Core/MathTypes.cs` | `Vec3`, `Mat3`, `Pose` (`x_parent = R x_child + P`), small dense `Matrix`, deterministic `Rng` with named substreams |
| `Core/FleetProfile.cs` | loader/validator for `fleet_profile_v0.json` and `_v1.json` (see "Profile versions"); every `{value, provenance}` leaf is kept in `Leaves` and per-sensor `Provenance` dictionaries; throws `ProfileException` with all errors |
| `Core/Scenario.cs` | `warehouse_a_v0.json` to `Obb` boxes (obstacles, divider wall split at the doorway with lintel, outer walls), overhead camera poses and modes |
| `Geometry/CameraAndTag.cs` | `CameraIntrinsics` from resolution and hfov, `CameraMount`, `TagSpec` (planar square, four corners), `RoverGeometry` (tags on active faces and top, derived from the profile) |
| `Geometry/Visibility.cs` | `TagVisibility.Evaluate`: view angle, behind camera, all-four-corners-in-image, minimum range, minimum projected size, line of sight on all four corners |
| `Observation/PlanarPnP.cs` | DLT homography, rigid decomposition, flip initialisation, Levenberg-Marquardt on reprojection error; both minima kept; Gauss-Newton covariance |
| `Observation/TagObserver.cs` | corner noise (base, obliquity, motion blur), dropout, latency jitter, `FrameSchedule` rate limiting |
| `Observation/RelPoseObservation.cs` | `RelPoseObservation`, `ObservationAging.Age`, `PoseUncertainty` (covariance composition) |
| `Sensors/Sensors.cs` | `SensorSuite.ForRover`, mecanum kinematics, `EncoderModel`, `ImuModel`, `TofModel`, `LidarModel`, `DepthModel` (D435 depth) |
| `Rover/IObservationProvider.cs` | rover-facing API: `IObservationProvider`, `IObservationBackend`, `RoverObservationProvider` |
| `Evaluator/*.cs` | truth side: `EvaluatorTruth`, `Trajectories`, `ObservationScorer`, `FleetSimulator` (implements the backend) |

## API in one screen

```csharp
var profile  = FleetProfile.Load(".../fleet_profile_v0.json");
var scenario = Scenario.Load(".../warehouse_a_v0.json");

// Evaluator side
var truth = new EvaluatorTruth();
truth.SetTrajectory("lifter_1", Trajectories.Static(Pose.FromYaw(1, 1, 0, 0)));
truth.SetTrajectory("cargo_1",  Trajectories.ConstantTwist(Pose.FromYaw(2, 1, 0, Math.PI), 0.1, 0, 0));
truth.SetOverheadMode("oh_west", OverheadMode.Occluded);
var sim = new FleetSimulator(profile, scenario, truth, seed: 42);

// Rover side (cannot reach `truth` or `sim`)
IObservationProvider lifter = sim.ProviderFor("lifter_1");
lifter.ReportSelfEstimate(stamp, myPoseEstimate, myCovariance);      // 6x6, [dp; dtheta] in world axes
foreach (RelPoseObservation o in lifter.PollTagObservations(now)) {  // delivered <= now, never repeated
    // o.StampAcquired, o.StampDelivered, o.SourceId, o.SourceKind, o.FrameParent/Child, o.Pose, o.Covariance, o.Age
    var aged = ObservationAging.Age(o, now, new DeadReckoningDrift(1e-5, 1e-6), motionSinceAcquisition);
}
var enc = lifter.PollEncoders(now); var imu = lifter.PollImu(now); var tof = lifter.PollTof("tof_front", now);
```

Observation sources (`ObservationSourceKind`): `OwnCamera` (parent `<id>/base_link`), `PeerObserver` (another rover's camera, composed with that rover's reported pose ESTIMATE so its self-localisation error enters the shared-frame covariance; parent `world`), `FixedStation` and `Overhead` (surveyed pose with a constant per-seed survey bias; parent `world`; overhead modes working, occluded with per-frame block probability 0.6, disabled).

Conventions: a pose is `T_parent_child`. Covariance is 6x6 in parent axes for the twist `[dp; dtheta]` with `p' = p + dp`, `R' = Exp(dtheta) R` (`Pose.Perturbed`/`Pose.ErrorFrom`). Camera optical frame: x right, y down, z forward. Tag frame: x right, y up, z out of the face; corners (-h,-h), (h,-h), (h,h), (-h,h). Robot frame: x forward, y left, z up.

Determinism: every random draw comes from `Rng.ForStream(seed, name)` with `name` containing source, frame index and tag id (FNV-1a hash, not `string.GetHashCode`), so results do not depend on polling cadence, consumer count or platform.

## How pose error arises

Corners get Gaussian pixel noise (`CornerSigmaPx`, extended by the obliquity term and by motion blur = image speed x exposure). `PlanarPnP` solves the pose from the four noisy corners and returns the Gauss-Newton covariance `sigma^2 (J^T J)^-1`. Consequences visible in the tests: depth error grows with range squared, cross-range with range; rotation error is large near frontal views because of the planar two-solution ambiguity (flagged when the alternate minimum is within `AmbiguityRatio x 2 sigma^2` of the best cost; 98%+ of wrong-branch solutions are flagged); the base-frame pose of the target inherits the tag-to-base lever arm (0.21 m for a face tag), so a 15 degree tag rotation error is a 5 cm base error.

Calibration: among unflagged solutions the normalised error squared has mean 5.9 (6 degrees of freedom expected) and 95th percentile 12.5 against the chi-square 12.59, with and without blur and obliquity gain. The first-order covariance does not describe the flagged frontal cases; consumers should inflate or discard `AmbiguousPose` observations.

## Assumptions and the measurement that replaces each

| # | Assumption | Where | Replace with |
|---|---|---|---|
| 1 | Camera: pinhole, no distortion, principal point at the image centre, fx = fy, from profile resolution and hfov (provisional) | `CameraIntrinsics` | checkerboard calibration of the chosen lens: K, distortion, hfov |
| 2 | Camera/ToF/LiDAR mount poses (front camera 1 cm inside the front face; scout cameras on the 0.35 m mast; ToF at face height 0.06 m) | `SensorSuite` | CAD, then a measured extrinsic calibration (target board or tag board) |
| 3 | Tag sizes 5 cm (faces) and 12 cm (top) and mount poses (2 mm proud of the face; top tag at body-top centre, y along forward) | `RoverGeometry` | calipers on the printed tags; CAD or a total-station survey of tag-to-base transforms |
| 4 | Minimum projected tag size 32 px (4 px per cell x 8 cells of the black border) | `VisibilityParams.MinPixels` | detection rate versus distance and angle with the real camera, lens, exposure and detector settings (decimation, blur). Cited basis is a community test, not a paper; `observer_value` uses 20 px |
| 5 | Maximum viewing angle 65 degrees, minimum range 0.10 m | `VisibilityParams` | the same detection-rate sweep; lens minimum focus distance |
| 6 | Corner noise 1.0 px (1 sigma per axis) | `TagNoiseParams.CornerSigmaPx` | reprojection residual of detected corners on a static, well-lit tag at several ranges (RMS after fitting the pose) |
| 7 | Obliquity gain 0.5 (sigma x1.7 at 65 degrees) | `ObliquityNoiseGain` | corner reprojection RMS versus view angle at fixed pixel size |
| 8 | Exposure 4 ms, blur coefficient 0.29 of the blur length | `ExposureS`, `BlurCornerCoeff` | camera exposure setting from the driver; detection error versus a motorised slider speed |
| 9 | Dropout 10% plus a ramp up to 40% extra approaching the pixel limit | `BaseDropout`, `RampMax` | fraction of frames with no detection on a static visible tag |
| 10 | Latency 0.03 s (profile) with 2 ms jitter; 0 transport delay; peer link 20 ms | `LatencyJitterS`, `PeerLinkDelayS` | timestamp comparison with a hardware trigger or LED flash; ping and publish-to-receive histograms |
| 11 | Overhead resolution 1920x1080 (the profile gives only hfov, height, rate and latency) | `SimulatorConfig` | the purchased overhead camera's datasheet and calibration |
| 12 | Survey error of stations and overhead cameras: 4 mm and 0.2 degrees per axis, constant per run | `StationSpec` | repeat the camera-to-world calibration and use the spread |
| 13 | Occluded overhead mode blocks a whole frame with probability 0.6 | `SimulatorConfig` | logged fraction of frames with the view blocked in the real hall |
| 14 | Encoder slip noise: profile `slip_sigma_frac` (3% on tile) applied as a per-sample multiplicative error on each wheel increment | `EncoderModel` | odometry versus motion capture or a marked-floor course on each surface |
| 15 | Gyro: white noise = density / sqrt(dt); bias random walk density = profile `gyro_bias_instability` (read as rad/s/sqrt(s)) | `ImuModel` | Allan deviation from a 2 hour stationary log |
| 16 | ToF noise 5 mm + 1% of range, 1 mm quantum, 1% dropout, nine-ray cone fan | `TofModel` | bench range sweep against a target at known distances and angles |
| 17 | LiDAR noise 1 cm + 0.2% of range, 1% dropout; no incidence-angle or reflectivity model | `LidarModel` | range sweep on matt and glossy targets; incidence-angle sweep |
| 18 | Occlusion by axis-aligned or yawed boxes only; rover bodies are boxes of the profile body size; the last 6 mm of a corner ray is ignored | `Obb`, `VisibilityParams.OcclusionEps` | CAD collision envelopes (the profile lists them as an open question) |
| 19 | Rovers' own estimates are supplied by the rover through `ReportSelfEstimate`; no estimator is modelled here | `FleetSimulator` | the real localiser's output and covariance |
| 20 | Wheel-slip dead-reckoning drift for ageing is a parameter (`DeadReckoningDrift`), not derived | `ObservationAging` | odometry error growth versus time from the encoder/IMU calibration runs |

## Dropping into a Unity 6 assembly

1. Copy the `Crane.Fleet.Perception` folder (without `bin/`, `obj/` and `*.csproj`) to `Assets/Scripts/Fleet/Perception/` in a Unity project; or build the DLL (`dotnet build -c Release`) and place `Crane.Fleet.Perception.dll` under `Assets/Plugins/` (Unity accepts `netstandard2.1` assemblies).
2. For the source option, add `Crane.Fleet.Perception.asmdef` next to the sources:

```json
{
  "name": "Crane.Fleet.Perception",
  "rootNamespace": "Crane.Fleet.Perception",
  "references": [],
  "includePlatforms": [],
  "excludePlatforms": [],
  "allowUnsafeCode": false,
  "overrideReferences": false,
  "autoReferenced": false,
  "noEngineReferences": true
}
```

`noEngineReferences: true` makes the compiler reject any `UnityEngine` use, which keeps the library honest. Rover-controller assemblies reference `Crane.Fleet.Perception` but must exclude the `Evaluator` folder from their view: place the `Evaluator` sources in a second asmdef `Crane.Fleet.Perception.Evaluator` that references the first, and have only evaluator/referee assemblies reference that one. The truth firewall then holds at assembly level as well as by the reflection test (move the `Evaluator` namespace folder; `Rover` and the core stay unchanged because the core never references `Evaluator`).
3. The code uses only `System`, `System.Collections.Generic`, `System.Globalization`, `System.IO` (loaders only), `System.Linq`. Unity 6 supports C# 9, which is what `LangVersion` pins. Loaders take strings (`FleetProfile.Parse`, `Scenario.Parse`), so a build can pass `TextAsset.text` instead of using file paths.

## Known gaps

* No optical lens distortion, rolling shutter (global shutter is the profile's), vignetting, or lighting model; corner noise is isotropic except through blur magnitude.
* The tag-to-base lever arm uses the nominal mount; mounting error is not sampled.
* A tag partly out of frame, behind a thin occluder narrower than a corner, or with a single blocked corner is rejected by rule; real detectors sometimes still decode.
* No payload or other dynamic occluders besides rover bodies; the elevated beam in `warehouse_a_v0` only hides ground-level points from the overhead cameras at ranges beyond about 6.5 m (far outside decode range at 12 cm and 1920 px), so with the scenario as written the beam does not change any overhead outcome.
* The IMU has gyro-z only (no accelerometer model); the encoder does not model backlash or per-wheel scale error.
* Peer observation delivery is broadcast to every other rover; there is no radio loss model.

## Profile versions (v0 and v1)

`FleetProfile.Parse` reads both `fleet_profile_v0.json` (values unchanged; locked by the golden test `parity/profile_v0_golden.txt`, regenerate only deliberately with `CRANE_WRITE_GOLDEN=1`) and `fleet_profile_v1.json` (CAD v1 rovers).

**Numeric versus descriptive leaves.** A `{value, provenance}` leaf is numeric (a number or an array of numbers) unless its path is listed in `FleetProfile.DescriptivePathPatterns` (`chassis.wheel.motor`, `docking.magnet`, `docking.partner_acquisition`, `roles.*.compute`, `roles.*.mcu`, `roles.*.lifter_option`, `roles.*.arm_proxy.servo`). Descriptive leaves are kept verbatim, with provenance and source, in `FleetProfile.Metadata` (`Meta(path)`, `Text(path)`); they never enter `Leaves`, and `Num`/`Vec` on them throw. A non-numeric value on any other leaf fails with `ProfileException` naming the path (`path: value is not numeric ...`, or `path: value[i] ...` for arrays). An unlisted new descriptive field therefore fails loudly until the schema list is extended. Numeric leaves now also carry `Provenanced.Source`.

**Sensors.** Role tokens accepted: `wheel_encoder`, `imu`, `lidar_2d` (an optional parenthesised model name is informational), `stereo`, `mono_camera_ov9281(front|rear|mast)`, `range_tof(front|rear)`, `realsense_d435(depth + infra1/infra2 + color)` (streams checked). `roles.*.required_sensor_frames` and `stereo_pair` are parsed into `RoleSpec`; `SensorSuite.SensorIds()` returns exactly those ids per role (v1 test).

| Sensor | Model | Datasheet values (profile, with source in `Sources`) |
|---|---|---|
| RealSense D435 depth | `DepthMount` + `DepthModel`: ray cast on a strided grid, z along the axis, sigma_z = z^2 * 0.08 px / (fx * baseline) (ASSUMPTION: subpixel RMS from the D400 whitepaper), 1 mm quantisation, invalid outside 0.3-3.0 m | 1280x720, 87 deg H, 30 Hz (up to 90), 0.3-3 m, 16UC1. https://www.intel.com/content/www/us/en/products/sku/128255/intel-realsense-depth-camera-d435/specifications.html ; https://www.mouser.com/datasheet/2/612/Intel_3_28_2023_D435if_Launch_Deck-3135151.pdf |
| D435 infra1/infra2 | two MONO `CameraMount3` (`stereo_left`, `stereo_right`, like the v0 pair), 50 mm apart, global shutter, 1280x720, 87 deg | same sources; baseline `sensors.realsense_d435_body.ir_baseline` must equal `sensors.stereo_baseline` |
| D435 colour | declared only (`SensorSuite.Unsimulated`, `camera_color`): rolling shutter, provisional values | no model |
| RPLIDAR C1 | existing `LidarMount`/`LidarModel`: 12 m (white; 6 m on black), 10 Hz (8-12), 500 points (5 kHz, 0.72 deg), model name in `LidarSpec.Model` | https://www.slamtec.com/en/C1/Spec (also: min range 0.05 m, accuracy +-30 mm, TTL UART) |
| Mast camera | Arducam OV9281 mono global shutter `CameraMount3` named `cam_front`, 1280x800, 70 deg H, 60 Hz, pitched down by `roles.lifter.mast_camera_pitch` | https://cdn.robotshop.com/media/A/Adu/RB-Adu-256/pdf/arducam-1mp-ov9281-usb-camera-120fps-global-shutter-uvc-low-distortion-m12-lens-datasheet.pdf |

**Mount poses.** The profile has no mounting poses, so for a profile with a D435 (v1) the mounts come from `SensorSuite.CadV1`, transcribed from `Tools/Fleet/assets/cad_v1/urdf/*.urdf`: scout D435 `camera_link` (0.1508, 0.0175, 0.1025) (infra1 = depth origin, infra2 50 mm to -y), scout laser (-0.122, -0.072, 0.1517), lifter mast camera (0.0986, -0.04, 0.1262), cargo front camera (0.1698, 0, 0.104). v0 keeps its provisional mounts. Replace with the manifest import when the manifests are available here.
