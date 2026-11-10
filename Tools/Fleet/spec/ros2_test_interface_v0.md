# Fleet ROS 2 test interface, v0 (specification only, no ROS code)

Status: draft for the four-rover mecanum fleet (`scout_1`, `lifter_1`, `cargo_1`, `cargo_2`, from `fleet_profile_v0.json`).
Purpose: one wire contract that the CRANE simulator, a hardware bring-up rig and the offline replay all expose, so a rover-side stack is tested against the same topics in every setting.
Counterpart library: `Tools/Fleet/core_cs/Crane.Fleet.Perception` (its `IObservationProvider` and `RelPoseObservation` map one-to-one onto the topics below).
Every rate, latency and size below is read from the profile, so a profile edit changes this contract; values are `provisional` until the profile says otherwise.

## 1. Rules

1. Rover namespace is `/fleet/<rover_id>/`. A rover node lives in its own namespace and may touch only topics under it plus the shared topics in section 6.
2. Header stamp is ACQUISITION time: the simulation or wall time at which the world was sampled (shutter centre for the camera, scan start for LiDAR, sample instant for IMU and encoders). Delivery time is never written into a header; it is the receive time on the consumer (bag time), and latency is `receive - header.stamp`. Where a delivery stamp must travel in-band (shared observations) it has its own field (`stamp_delivered`).
3. Time source: `use_sim_time:=true` everywhere in simulation, driven by `/clock`. `/clock` is episode-relative and may jump backwards at reset, which the existing CRANE rig already handles (`Docs/ArchitectureAndDesign.md`, episode reset rows). On hardware `/clock` is absent and stamps are system time from a disciplined clock (chrony/PTP); the max tolerated cross-rover skew is 5 ms (assumption; the camera latency is 30 ms).
4. Truth never appears under a rover namespace and rover nodes never subscribe to `/fleet/truth/**` (section 8).
5. Message types come from standard packages wherever one exists; custom messages live in one package, `fleet_msgs`, and are limited to the five in section 5.
6. Quaternions are x, y, z, w, unit norm. Covariances are row-major 6x6 `[x y z roll pitch yaw]` in the axes of the parent frame of the pose (`Pose.Perturbed` convention of the library: `p' = p + dp`, `R' = Exp(dtheta) R`).

## 2. Frames (tf prefixes)

| Frame | Meaning |
|---|---|
| `world` | shared fixed frame; the scenario frame (x east, y north, origin south-west inside corner, z up) |
| `<rover_id>/odom` | rover-local dead-reckoning frame, continuous |
| `<rover_id>/base_link` | rover base: x forward, y left, z up, origin on the floor under the chassis centre |
| `<rover_id>/<sensor>_optical` | camera optical frame: x right, y down, z forward (REP 103 optical convention) |
| `<rover_id>/lidar`, `<rover_id>/tof_front`, `<rover_id>/imu` | sensor frames |
| `<rover_id>/face_<name>_tag`, `<rover_id>/top_tag` | tag frames: x right, y up, z out of the visible face (tag edge = black border) |
| `overhead/<camera_id>_optical` | fixed cameras, e.g. `overhead/oh_west_optical` (poses from the scenario file) |

`tf` is namespaced by remapping `/tf` and `/tf_static` to `/fleet/<rover_id>/tf` and `/fleet/<rover_id>/tf_static`; the evaluator and RViz subscribe to all four. The `odom -> base_link` transform is published by the rover's own estimator; `world -> odom` is published only by that estimator too (it is the rover's belief, not truth). Static mounts (`base_link -> *_optical`, tag mounts) are CAD-derived values from the profile (`provisional` today) published on `tf_static` with `TRANSIENT_LOCAL` QoS.

## 3. Rover topics

QoS profiles used below (names are shorthand, defined in section 7): SENSOR, RELIABLE, LATCHED, CMD.

### 3.1 Common to all four rovers

| Topic (under `/fleet/<id>/`) | Type | Dir | Rate | QoS | Notes |
|---|---|---|---|---|---|
| `cmd_vel` | `geometry_msgs/TwistStamped` | in | 50 Hz | CMD | body twist: linear.x forward, linear.y left, angular.z yaw. Header stamp = command creation time; the rover rejects commands older than 0.25 s (watchdog zeroes the wheels). Limits from profile: 0.5 m/s, 1.5 rad/s |
| `wheel/joint_states` | `sensor_msgs/JointState` | out | 100 Hz | SENSOR | names `fl, fr, rl, rr`; `position` = quantised wheel angle in rad (counts x 2pi/1440), `velocity` = count-difference rate in rad/s, `effort` empty. Header = sample instant |
| `wheel/encoder_counts` | `fleet_msgs/WheelEncoderCounts` | out | 100 Hz | SENSOR | exact integer counts and counts-per-rev (JointState cannot carry integers; this is the lossless record) |
| `imu` | `sensor_msgs/Imu` | out | 200 Hz | SENSOR | frame `<id>/imu`. Only the yaw rate is modelled: `angular_velocity.z` valid, x and y set to 0 with covariance `-1` marking not-modelled; `orientation_covariance[0] = -1`; `linear_acceleration_covariance[0] = -1`. `angular_velocity_covariance[8]` = (noise_density^2 / dt) |
| `odom` | `nav_msgs/Odometry` | out | 50 Hz | RELIABLE | `header.frame_id = <id>/odom`, `child_frame_id = <id>/base_link`; covariance fields filled by the rover estimator |
| `pose_estimate` | `geometry_msgs/PoseWithCovarianceStamped` | out | 20 Hz | RELIABLE | rover's own belief of `base_link` in `world`. Peers use the value in force at the shared observation's acquisition time (library: `ReportSelfEstimate`) |
| `observations/own` | `fleet_msgs/RelPoseObservation` | out | per detection | RELIABLE | tag sightings from this rover's cameras, parent = `<id>/base_link`, child = target `<id>/base_link` |
| `connector/<face>/state` | `fleet_msgs/ConnectorState` | out | 10 Hz + on change | LATCHED | `<face>` in `front, rear` (active faces from the profile; `left, right` reserved) |
| `connector/<face>/cmd` | `fleet_msgs/ConnectorCommand` | in | on demand | RELIABLE | engage, release, verify |

### 3.2 Role-specific sensors

| Topic (under `/fleet/<id>/`) | Type | Rovers | Rate | QoS | Notes |
|---|---|---|---|---|---|
| `cam_front/image_raw` | `sensor_msgs/Image` | lifter, cargo | 60 Hz | SENSOR | `encoding = mono8`, 1280x800, global shutter, no row skew; frame `<id>/cam_front_optical` |
| `cam_front/camera_info` | `sensor_msgs/CameraInfo` | lifter, cargo | with image | SENSOR | `distortion_model = plumb_bob`, `d = [0,0,0,0,0]` (lens not chosen), `k = [f,0,640, 0,f,400, 0,0,1]`, `f = 640 / tan(hfov/2)` = 759.8 px at hfov 1.40 rad; `p`, `r` standard; same stamp as the image |
| `cam_front/tag_detections` | `apriltag_msgs/AprilTagDetectionArray` | lifter, cargo | with image | SENSOR | raw detector output: family `tag36h11`, `id`, `hamming`, `decision_margin`, `centre`, `corners` (4 points, AprilTag order: counter-clockwise from bottom-left as seen from the front), `homography`. Verified to exist: `christianrauch/apriltag_msgs`. The package has no pose field, hence `tag_poses` below |
| `cam_front/tag_poses` | `fleet_msgs/TagPoseArray` | lifter, cargo | with image | SENSOR | pose of each detected tag in the camera optical frame with 6x6 covariance, `ambiguous` flag (second PnP minimum within 8 x 2 sigma^2 cost), noisy corner pixels. Same header stamp as the image it came from, index-aligned with `tag_detections` |
| `tof_front/range` | `sensor_msgs/Range` | lifter, cargo | 30 Hz | SENSOR | `radiation_type = INFRARED`, `field_of_view = 0.44` rad (full cone), `max_range = 2.0`; an invalid return is `range = +inf` |
| `stereo_left/*`, `stereo_right/*` | as `cam_front/*` | scout | 60 Hz | SENSOR | baseline 0.075 m; hardware-synchronised pair (equal stamps) |
| `cam_rear/*` | as `cam_front/*` | scout | 60 Hz | SENSOR | |
| `lidar/scan` | `sensor_msgs/LaserScan` | scout | 10 Hz | SENSOR | 720 points over 2pi, `range_max = 12.0`; frame `<id>/lidar`; no return = `+inf`; the stamp is scan start, `time_increment` set |

## 4. Rates and latencies (all from the profile)

| Stream | Rate | Latency camera-to-delivery |
|---|---|---|
| wheel encoders | 100 Hz | negligible |
| IMU | 200 Hz | negligible |
| camera | 60 Hz | 0.03 s |
| ToF | 30 Hz | negligible |
| LiDAR | 10 Hz | negligible (scan assembly time is inside the scan) |
| overhead camera | 30 Hz | 0.15 s |

Latency is realised in simulation by delaying publication, never by back-dating the stamp; the stamp always stays at acquisition.

## 5. Custom messages (`fleet_msgs`)

```
# RelPoseObservation.msg  (shared form of the library type of the same name)
std_msgs/Header header            # stamp = ACQUISITION time; frame_id = frame_parent
builtin_interfaces/Time stamp_delivered
string source_id                  # e.g. "lifter_1/cam_front", "oh_west"
uint8 source_kind                 # 0 OWN_CAMERA, 1 PEER_OBSERVER, 2 FIXED_STATION, 3 OVERHEAD
string frame_child                # frame_parent is header.frame_id
geometry_msgs/PoseWithCovariance pose   # child in parent; covariance in parent axes
float64 age                       # seconds since acquisition at the moment of publication
int32 tag_id                      # -1 when not tag-derived
bool ambiguous                    # planar pose ambiguity flagged

# TagPose.msg
int32 id
string family                     # "tag36h11"
float64 tag_size                  # metres, black border edge
geometry_msgs/PoseWithCovariance pose   # tag in header.frame_id (camera optical)
float64[8] corners_px             # u0 v0 ... u3 v3 as measured
bool ambiguous
# TagPoseArray.msg : std_msgs/Header header ; TagPose[] tags

# WheelEncoderCounts.msg
std_msgs/Header header
int64[4] counts                   # fl fr rl rr
float64[4] rate_rad_s
uint32 counts_per_rev             # 1440 (provisional)

# ConnectorState.msg
std_msgs/Header header
string face
string state                      # FREE APPROACH MAGNET_CAPTURED HOOK_A_ENGAGED HOOK_B_ENGAGED LATCHED_VERIFIED LOAD_VALIDATED RELEASE_REQUESTED RELEASED
string fault                      # "" | PARTIAL FALSE_ENGAGEMENT OBSTRUCTED VERIFICATION_LOST STUCK_HOOK
uint8 hook_a                      # 0 RETRACTED 1 MOVING 2 ENGAGED 3 FAULT
uint8 hook_b
bool magnet_captured
bool support_verified             # release invariant input: support verified on the remaining module graph
uint32 command_seq_ack

# ConnectorCommand.msg
std_msgs/Header header
uint32 seq
uint8 command                     # 0 ENGAGE, 1 RELEASE, 2 VERIFY
```

State and fault names are copied verbatim from `docking.states` and `docking.fault_states` in the profile. A `RELEASE` command is refused (state unchanged, `fault` unchanged, ack still incremented) unless `support_verified` is true, which is the profile's release invariant.

Unity side: custom messages need ROS-TCP-Connector message generation from the `.msg` files; standard and `apriltag_msgs` types need generation too (the existing project generates `Std` and `Sensor`).

## 6. Shared observation topics

| Topic | Type | QoS | Publishers | Notes |
|---|---|---|---|---|
| `/fleet/shared_observations` | `fleet_msgs/RelPoseObservation` | RELIABLE (depth 50) | each rover (as PEER_OBSERVER), fixed stations, overhead cameras | parent frame is always `world`. A peer observation is the observer's `observations/own` sighting composed with the observer's `pose_estimate` that was in force at acquisition, so its self-localisation error is inside the covariance. The observer never receives its own peer observations back as peer observations |
| `/fleet/overhead/<camera_id>/status` | `std_msgs/String` | LATCHED | overhead node | `working`, `occluded`, `disabled`. This is a health report a deployed system could produce; the true cause of an `occluded` condition is a truth topic (section 8) |
| `/fleet/clock_status` | `std_msgs/String` | LATCHED | rig | `sim` or `wall`; informational |

## 7. QoS

| Name | Reliability | Durability | History | Deadline / lifespan |
|---|---|---|---|---|
| SENSOR | BEST_EFFORT | VOLATILE | KEEP_LAST 5 | lifespan 0.5 s |
| RELIABLE | RELIABLE | VOLATILE | KEEP_LAST 20 | none |
| LATCHED | RELIABLE | TRANSIENT_LOCAL | KEEP_LAST 1 | none |
| CMD | RELIABLE | VOLATILE | KEEP_LAST 1 | deadline 0.1 s (a missed deadline raises the watchdog) |

Image topics sit behind `ros_tcp_endpoint` in the Unity configuration, which does not preserve QoS faithfully; the bridge subscribes with the profile above and publishes to ROS with the same names, and the evaluator measures end-to-end age from header stamps, so a QoS mismatch shows up as measured age instead of hiding.

## 8. Evaluator-only truth topics and the allow-list rule

| Topic | Type | Rate | Content |
|---|---|---|---|
| `/fleet/truth/<rover_id>/pose` | `geometry_msgs/PoseStamped` | 100 Hz | true `base_link` in `world`, stamped with simulation time |
| `/fleet/truth/<rover_id>/twist` | `geometry_msgs/TwistStamped` | 100 Hz | true body twist |
| `/fleet/truth/<rover_id>/wheel_angles` | `sensor_msgs/JointState` | 100 Hz | unquantised, slip-free wheel angles |
| `/fleet/truth/overhead/<camera_id>/mode` | `std_msgs/String` | on change, LATCHED | injected mode, including the cause of occlusion |
| `/fleet/truth/scene/boxes` | `visualization_msgs/MarkerArray` | LATCHED | occluders as built |
| `/fleet/truth/contact` | `std_msgs/String` (JSON) | on event | connector contact and payload events for scoring |

Rule: only nodes on the allow-list may subscribe to any `/fleet/truth/**` topic. Rover nodes live in `/fleet/<rover_id>` namespaces and are never on it. The list (names are fully-qualified node names without the leading slash; a trailing `*` matches a prefix):

```yaml
# truth_allowlist_fleet.yaml  (same format as Tools/RoboBoat/truth_allowlist.yaml)
allow:
  - fleet_referee*
  - rosbag2_recorder*
  - rviz*
  - ros_tcp_endpoint*
topics:
  - /fleet/truth/scout_1/pose
  - /fleet/truth/scout_1/twist
  - /fleet/truth/scout_1/wheel_angles
  - /fleet/truth/lifter_1/pose
  - /fleet/truth/lifter_1/twist
  - /fleet/truth/lifter_1/wheel_angles
  - /fleet/truth/cargo_1/pose
  - /fleet/truth/cargo_1/twist
  - /fleet/truth/cargo_1/wheel_angles
  - /fleet/truth/cargo_2/pose
  - /fleet/truth/cargo_2/twist
  - /fleet/truth/cargo_2/wheel_angles
  - /fleet/truth/overhead/oh_west/mode
  - /fleet/truth/overhead/oh_east/mode
  - /fleet/truth/scene/boxes
  - /fleet/truth/contact
```

Enforcement: the rig runs the existing `check_truth_subscribers.py --allowlist truth_allowlist_fleet.yaml` at readiness and every 30 s; exit 1 fails the run. `rosbag2_recorder*` records truth only into the evaluator's bag, never the rover-visible bag. In the C# library the same boundary is the `Crane.Fleet.Perception.Evaluator` namespace, enforced by a reflection test.

## 9. What can be reused from the existing simulator (read-only references in `/home/lunarz/worktrees/crane-fleet`)

| Existing | Reuse |
|---|---|
| `Assets/Scripts/Utils/ROS/ROSPublisher.cs`: `Initialize<T>(topic, frameId, createMessage, Hz, manualPublish, acquisitionProvider)` and `CreateHeader(acquisitionTime)` which writes `sec`/`nanosec` from the acquisition time | Reuse as is for every standard message: pass an acquisition provider so the header carries acquisition time and the delivery tick is journaled separately. Needs per-rover topic prefix and frame ids (the class already takes both). A `TwistStamped` subscriber needs a small new `ROSSubscriber` use, not a new class |
| `CraneObservationAcquisition(Tick, Time)` and `CraneObservationJournal` metadata (`AcquisitionTime`, `CompletionTime`, sequence, episode) | Reuse directly: the library's `StampAcquired`/`StampDelivered` are the same pair. The journal gives the evaluator an independent age measurement |
| `Docs/ArchitectureAndDesign.md` acquisition passages (lines near 797: "an acquisition timestamp identifies when the world was observed; a completion timestamp identifies processing delay") | This document adopts the rule verbatim (section 1, rule 2) |
| `Tools/RoboBoat/truth_allowlist.yaml` and `check_truth_subscribers.py` | The script reads any allow-list through `--allowlist`, matches `namespace/name` with `fnmatch`, and exits 0/1/2; reuse unchanged with the fleet list above. Not reusable: the fixed `/crane/truth/*` topic names and the two-topic list; the fleet list is per rover |
| `ROSClock.cs` and the `/clock` reset handling | Reuse for `/clock`; the fleet needs one clock shared by all four rover scenes (single Unity process assumed) |
| `ros_tcp_endpoint` (`/home/lunarz/astro_dock/src/ros_tcp_endpoint`, read-only reference) | The bridge for all topics above; custom `fleet_msgs` and `apriltag_msgs` types must be generated on the Unity side before the endpoint can carry them |

Not available in the reference: per-rover namespacing helpers, a `TwistStamped` command path, `apriltag_msgs`, `fleet_msgs`, and a multi-rover truth publisher; these are the new work.

## 10. Conformance checks for a test rig

1. Every topic in sections 3 and 6 exists with the stated type and QoS; extra topics under a rover namespace fail the check.
2. For each stream, measured rate within 5% of the profile rate over 10 s.
3. `receive_time - header.stamp` for camera topics is within 0.03 s +- 3 sigma of the configured jitter in simulation; for overhead observations 0.15 s.
4. No `header.stamp` is greater than the current clock; stamps within one topic are monotone non-decreasing except at an episode reset.
5. `check_truth_subscribers.py` returns 0 with a fleet rig running.
6. Replay the C# parity case file (`Crane.Fleet.Perception.Tests/parity/perception_parity_v0.json`) through a scene built from each case; the `tag_poses` topic is present exactly when the case's expected `visible` is true (dropout set to zero).
