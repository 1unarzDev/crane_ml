# Public sensor datasets for a CRANE-Fidelity pilot

Primary-source audit: 2026-10-11. Public datasets support an external-data pilot;
they do not establish physical calibration of the CRANE platform. A fitted error
model must be named for its source sensor, firmware/processing, mounting,
conditions, and truth quality. ZED, ZED 2, and ZED 2i measurements are distinct.

## Candidate inventory

| Publisher dataset | Sensor/data match | Download and terms | Timing and truth limits | Appropriate role |
|---|---|---|---|---|
| [DTU Roadrunners](https://data.dtu.dk/articles/dataset/DTU_Roadrunners_Autonomous_Ecocar_Race_Track_Dataset/14229995) | VLP-16 packets, ZED 2 SVO/IMU, Piksi Multi RTK CSV, custom odometry | [Publisher API](https://api.figshare.com/v2/articles/14229995); smallest archive `05-2020 DTU.zip`, 79,032,173 bytes, [direct download](https://ndownloader.figshare.com/files/26875652). CC BY-NC-SA 4.0 | ROS/SVO/GPS clock relationship and exact sample overlap require inspection. GPS quality flags distinguish fixed RTK from degraded fixes. No actuator command/feedback path is listed. | VLP-16 and ZED 2 ingestion pilot; independent RTK comparison where clocks and frames can be validated |
| [FLOBOT](https://lcas.github.io/FLOBOT/) | VLP-16, RGB-D, listed floor-facing original ZED; human/object annotations | Linked ROS bags and labels for supermarket, warehouse, airport. CC BY-NC-SA 4.0 | Software ROS timestamp synchronization; no hardware synchronization bound established by the page. World-frame robot pose is provided but independent truth provenance requires further inspection. Download table primarily lists VLP-16/Xtion, so listed ZED hardware does not prove released ZED frames. | VLP-16 human detection/miss behavior pilot; no ZED 2i depth or plant-equivalence claim |
| [TIERS multi-lidar](https://github.com/TIERS/tiers-lidars-dataset) | VLP-16 `/velodyne_points`, other spinning/solid-state lidars, L515 RGB/depth, MOCAP on selected sequences | Publisher links University of Turku/SharePoint bags; small reference CSVs in repo. MIT license described as academic use | Selected MOCAP sequences differ from long sequences with SLAM-derived references; do not treat the latter as independent truth. Original smallest listed main bag is 7.3 GB without MOCAP. [Enhanced release](https://github.com/TIERS/tiers-lidars-dataset-enhanced) explicitly adds PTP and ICP/RTK references; applies to that release only. | Reference-aware VLP-16 evaluation and lidar-agnostic model comparison; no ZED match |
| [UT CODa](https://amrl.cs.utexas.edu/coda/documentation.html) | Explicit ZED 2i `cam3`, campus scenes, RGB-D, object annotations; primary lidar is 128-channel | [Download page](https://amrl.cs.utexas.edu/coda/download.html) links separate RGB-D sequences, TACC, and preview data. [Official devkit](https://github.com/ut-amrl/coda-devkit) identifies a noncommercial dataset license distinct from software licensing | Conversion groups topics into synchronized frames; grouped frames do not independently establish original exposure timing or transport latency. Object labels support detection evaluation; metric depth truth needs separate evidence. | Most directly matched ZED 2i observation/detection candidate; second ingestion target after first pilot |
| [MIT Sea Grant marine perception](https://seagrant.mit.edu/auvlab-datasets-marine-perception-1/) | Surface-vessel video/IR, VLP-16 lidar, radar, GPS/IMU on selected recordings | [Access page](https://seagrant.mit.edu/auvlab-datasets-marine-perception-2-3/) lists archives and CC BY-NC-SA 4.0. Downloads request name, email, intended use; no form submitted | Some sailboat recordings provide independent target GPS. Corrected/original lidar topics differ in rotation. Not a RoboBoat actuator calibration dataset. Direct fetch returned 502/disconnected in this environment; publisher's indexed access description remained readable. | Marine perception extension when access is obtained; avoid blocking land pilot |

The [MHSeals organization](https://github.com/MHSeals) exposes robot software and
simulation repositories. This audit did not identify a publisher-verified,
synchronized MHSeals physical bag download with command, actuator feedback,
independent pose, and sensor streams. Image/object detection datasets alone cannot
substitute for paired physical navigation measurements.

## Operational pilot and provenance

The May 2020 DTU archive was selected because it is a small direct-download
publisher artifact with a published checksum. The local, ignored evidence
directory is:

`PerformanceResults/crane-fidelity-20261011/public/dtu-roadrunners/`

It preserves publisher API metadata, archive inventory, original downloaded
bytes, verified MD5/SHA-256, and inspection output. Inspect the exact archive before
assuming that every sensor named on the overall dataset page occurs in this
subset. There is no basis to fit ZED 2i error from a subset without camera frames,
or to fit vehicle dynamics without recorded input and independently measured
output. The exact ingestion outcome is appended below and reflected in the main
evidence manifest.

The archive's `2020-05-26.bag` was metadata-ingested without rewriting its raw
bytes. It contains `/gyro_angle` (`std_msgs/Float32`, 7,961 records),
`/car_pose_estimate` (`std_msgs/Float32MultiArray`, 6,403 records), and
`/velodyne_packets` (`velodyne_msgs/VelodyneScan`, 1,326 records) over 133.944 s.
The Velodyne payload remains undecoded because its custom message definition is
not in the repository; no point/intensity distribution was fitted. The 2020
subset contains no listed ZED SVO, command, or actuator feedback stream. The
machine-readable report is
`PerformanceResults/crane-fidelity-20261011/public/dtu-roadrunners/raw/ingestion/report.json`.

The independently ingested LILocBench public bag is documented by the other
pilot's download/ingestion manifest. It can exercise command/odometry/reference
alignment. Headerless `Twist` commands retain unknown acquisition time; bag receive
time is preserved as recorder time. Neither public source identifies the exact
CRANE industrial robot's actuator response.

## Boundaries for reported metrics

1. Use public sensor samples to report measured cadence, header/recorder clock
   behavior, fields/units, valid/missing returns, and empirical statistics. Do not
   subtract timestamps from unrelated clocks as a measured sensor latency.
2. Treat trajectory errors against SLAM/odometry as estimator disagreement unless
   an independent reference and frame/clock transform are documented. Report RTK
   fix quality and reference uncertainty; exclude invalid fixes transparently.
3. Reserve by whole run/day/route/condition before fitting. A random row split can
   leak temporally correlated sensor residuals and repeated site geometry.
4. Detection calibration needs a fixed real detector/version, outputs including
   confidence, annotations/unknown-label handling, and held-out evaluation. Object
   annotations plus perfect simulation boxes are insufficient.
5. Public replay supports methods and calibration pilots. Improved sim-to-real
   closed-loop transfer remains blocked until repeated paired physical tasks on
   an identified platform are available.
