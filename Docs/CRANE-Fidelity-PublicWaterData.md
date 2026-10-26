# Public surface-water data shortlist

Reviewed 2026-10-11 for a CRANE-Fidelity pilot. These sources can exercise ingestion, sensor
distribution, annotation, timing, or environmental-context tooling. None substitutes for
calibration of CRANE's robot, VLP-16/ZED 2i pair, actuator, or ROS closed loop. Access and license
terms should be rechecked at download time and the exact archive hash recorded.

Downloaded the openly licensed AutoNaut CSV for an external environmental-context pilot. It has
20,384 rows and 21 columns (including timestamp, latitude/longitude, turbidity, salinity,
temperature, dissolved oxygen and chlorophyll), 4,383,155 bytes, and SHA-256
`a629cb760cfee6c59c185e2d323089555b940d2bbd6e134a370521e3864be25e`. The publisher MD5 matches.
Raw data, publisher metadata, attribution, and an explicit evidence-boundary summary are retained
outside Git under `PerformanceResults/crane-fidelity-20261011/public-water-autonaut/`. No actuator
or independent robot trajectory record was found in the column schema; it is a context ingestion
pilot and does not close a calibration hypothesis.

| Source | First-party access and license | Useful evidence | Gaps for CRANE-Fidelity |
|---|---|---|---|
| [USVInland dataset](https://github.com/ORCA-TECH/USVInland-Dataset) and [publisher dataset page](https://www.orca-tech.cn/datasets.html) | The project README identifies a real inland-waterway USV with lidar, stereo camera, radar, and INS, and links the publisher download. The repository does not state a dataset license; obtain written terms before redistribution. | Multi-sensor inland scenes for checking timestamp/frame/file ingestion, stereo/laser/radar modality handling, SLAM/odometry and water segmentation. | README describes perception/localization tasks, not actuator commands or application timing. Sensor models, calibration targets, ROS schema, and license are unresolved. It is not VLP-16/ZED 2i evidence. |
| [WaterScenes](https://github.com/WaterScenes/WaterScenes), [project site](https://waterscenes.github.io), [paper](https://doi.org/10.1109/TITS.2024.3418192) | First-party README (repository) reports 54,120 synchronized sets of RGB, 4-D radar, GPS and IMU, with intrinsic/extrinsic parameters and precise timestamps; downloads link to Google Drive/Baidu. Repository has no explicit license file; verify dataset terms before redistribution. | Radar-camera detection/segmentation, adverse weather/time/waterway cohorts, synchronization and extrinsic parsing, detector confidence/latency pilot work. | No actuator/command or application-time stream; no VLP-16, ZED 2i, depth images, or plant trajectories. Radar point semantics cannot be treated as LiDAR noise. |
| [MODS evaluation repository](https://github.com/bborja/mods_evaluation) linked from the [ViCoS resource index](https://www.vicos.si/resources/) | Maintainer README links the full MODS download and evaluation code. Dataset redistribution terms are not clearly stated in the README, so keep downloaded data external until confirmed. | USV maritime obstacle/water-edge benchmark; 94 sequences in the maintainer example, obstacle TP/FP/FN and water-edge metrics. Useful for detector failure and held-out condition reporting. | RGB/segmentation benchmark, not raw ROS or actuator data; no depth/LiDAR timing and no plant identification. Verify both code and data licenses before reuse. |
| [MODD/MODD2 and MaSTr1325 maintainer resources](https://www.vicos.si/resources/modd/) and [source/evaluation repository](https://github.com/bborja/modd) | Maintainer page links video, IMU, calibration sequences and annotations for MODD2, and image/mask archives for MaSTr1325. The source repository declares GPL-3.0; dataset-specific terms must be confirmed before redistribution. | MODD2 real-USV video plus IMU, weather/glare/reflection cases, calibration sequences and obstacle/water-edge annotations; MaSTr1325 has 1,325 USV images with pixel labels and synchronized inertial measurements. | Primarily vision/IMU annotation. No command/actuator/application timestamps, VLP-16/ZED 2i streams, or dynamics ground truth. IMU synchronization is not enough for H1/H4. |
| [USV AutoNaut Frohavet 2022](https://doi.org/10.5281/zenodo.8283108) | Zenodo record 8283108 is open, CC BY 4.0, and provides `AutoNaut_Frohavet_2022.csv` with a checksum. The record identifies NTNU/SINTEF authors and April 2022 Frohavet measurements. | Environmental context: timestamped position and ocean measurements including turbidity, temperature, salinity, dissolved oxygen and chlorophyll. Can condition a site/sea-state split or test environmental metadata ingestion. | One-minute environmental samples, not synchronized robot pose/velocity/commands, actuator telemetry, or raw sensor streams. Not suitable for plant or sensor calibration. |
| [iSCSS26 vessel manoeuvring dataset](https://doi.org/10.5281/zenodo.20627303) | Zenodo record 20627303 is open, CC BY 4.0 and includes six CSV turning-circle/zig-zag files plus metadata and a license. The record explicitly says it was generated in MARUS, a simulator. | Deterministic manoeuvre schema, actuation-traceable virtual sea-trial format, turning-circle/zig-zag metric extraction and importer tests. | Synthetic/simulated, with propulsion variables identified as simulation-consistent proxies. It must never be counted as physical evidence or used to claim sim-to-real validation. |
| [LILocBench localization benchmark](https://www.ipb.uni-bonn.de/html/projects/localization_benchmark/) | University of Bonn source page; the pinned Dingo `dynamics_0.bag`/ground truth is CC-BY-NC-SA-4.0 (see `Tools/Campus/public_pilot_sources.json`). | Existing external physical ingestion pilot: wheel odometry, unstamped `Twist`, SICK laser scans, joint states and reference trajectory. Useful for testing raw-bag parsing, timing diagnostics, scan validity and held-out trajectory residual code. | Clearpath Dingo and SICK/RealSense sensors differ from CRANE's target platform; commands are unstamped and actuator application time is unknown. This is physical public data, not CRANE validation. |

The first four sources are strong perception pilots; AutoNaut is environmental context; iSCSS26 is
explicitly simulation; and LILocBench is the only currently ingested physical motion pilot in this
checkout. Public sources do not close H1-H4. The passive recorder in
[`Tools/Fidelity/physical_ros_capture.py`](../Tools/Fidelity/physical_ros_capture.py) remains the
collection path for synchronized CRANE-platform data when hardware becomes available.

The [maintainer's vessel-system-identification repository](https://github.com/martinlarsalbert/System-identification-of-Vessel-Manoeuvring-Models)
also points to [wPCC model-test data](https://data.mendeley.com/datasets/j5zdrhr9bf) and retains
filtered trajectories, identified models, and simulated results. The first-party
[code archive](https://doi.org/10.5281/zenodo.7185580) is MIT licensed. This is a useful physical
ship-model identification lead, but the Mendeley data page returned HTTP 403 from this environment;
raw control columns, independent state measurements, data license, and relevance to small USVs
were not verified. Do not ingest it as a measured RoboBoat profile without those checks.

