# Passive physical ROS 2 capture

`Tools/Fidelity/physical_ros_capture.py` is a passive subscriber. It dynamically resolves each
requested ROS message class using `rosidl_runtime_py.utilities.get_message`. It creates no
application publishers, command services, clients, command programs, or actuator calls. Standard
ROS node parameter and logging facilities remain available. It accepts arbitrary installed
message types, including pose/odometry, commands, LiDAR, IMU, clock, and RGB/depth images.

The recorder executed successfully in a disposable ROS 2 Humble Docker container on 2026-10-11.
The retained acceptance fixture contains **synthetic messages**, not physical-robot data. It
received 553 messages across odometry, command, LaserScan, PointCloud2, IMU, image, and clock topics; verified raw CDR
deserialization and file hashes; reported an intentionally missing topic; finalized a manifest on
SIGTERM; and rejected
`use_sim_time=true`. The host has no `rclpy`; its dependency failure is retained separately.
No actual hardware capture has occurred.

## Collection

Create a metadata JSON file with nonempty `platformId`, `sessionId`, `siteId`, and
`measurementSource`. Include sensor serials, firmware, survey/frame transforms, clock synchronization
method and uncertainty, conditions, route, and command-program identity when available. Metadata is
retained verbatim. Confirm clocks on the physical ROS graph use the intended system/device clock;
the recorder rejects its own `use_sim_time=true`, but cannot infer every publisher's clock domain.

```bash
source /opt/ros/jazzy/setup.bash
python3 Tools/Fidelity/physical_ros_capture.py \
  --output /data/crane/session-001 --metadata /data/crane/session-001-metadata.json \
  --seconds 180 \
  --topic /odom=nav_msgs/msg/Odometry \
  --topic /cmd_vel=geometry_msgs/msg/Twist \
  --topic /scan=sensor_msgs/msg/LaserScan \
  --topic /imu=sensor_msgs/msg/Imu \
  --topic /camera/depth/image_rect_raw=sensor_msgs/msg/Image
```

Use the physical device's actual topic types and names. Add `/points=sensor_msgs/msg/PointCloud2`
for point clouds and the camera's actual Image topic for RGB. `/clock=rosgraph_msgs/msg/Clock` can be
recorded for diagnostics without controlling the recorder clock. `Twist` has no acquisition header:
its acquisition timestamp is recorded as null. Unavailable sensor topics are explicitly listed in
the manifest; omit sensors absent from the platform instead of fabricating observations.

The recorder preserves payloads and uses sensor-data QoS (best effort, volatile) for subscriptions.
This receives both reliable and best-effort sensor publishers but does not certify lossless delivery.
Reception/serialization/disk overhead and requested topics can affect throughput, especially for
camera images. Topic counts are recorder-side counts; publisher sequence/loss evidence and device
synchronization remain separate requirements. Source metadata alone is not physical ground truth.

## Envelope and termination artifacts

The output directory must be empty of prior evidence. It contains:

- `records.jsonl.gz`: one `crane-physical-ros-record-v1` JSON record per received message.
- `manifest.json`: source marker, metadata, requested topic/types, topic counts, missing topics,
  start/end time, recorder and compressed-record hashes, errors, status, and termination reason.

Each record stores `topic`, `messageType`, per-topic `sequence`, raw `cdrBase64` and `cdrSha256`,
the converted `message`, original `acquisitionStamp` if the message has a header, and independent
`receptionRosNanoseconds`, `receptionMonotonicNanoseconds`, and `receptionUnixNanoseconds`.
NaN/Infinity become null in the JSON message view; raw CDR bytes retain them exactly. Reception is
sampled before conversion and disk I/O. Device publication time and actuator application time are
unavailable unless the selected message carries them; no timestamp is invented.

Duration completion and SIGINT/SIGTERM write the manifest. Missing topics produce
`partial_missing_topics` and exit 2. Missing Python ROS dependencies produce `dependency_failure`
with a traceback and manifest. Other runtime failures produce `capture_failure`. A hard kill or
power failure may prevent finalization and must be treated as incomplete evidence.

## Reproduce the transport fixture

```bash
docker run --rm --name crane_fidelity_capture_fixture --network none \
  -e ROS_DOMAIN_ID=187 -v "$PWD:/workspace" ros:humble-ros-core \
  bash -lc 'source /opt/ros/humble/setup.bash; python3 /workspace/Tools/Fidelity/synthetic_ros_capture_fixture.py /workspace/PerformanceResults/capture-fixture-new'
python3 -m pytest -q Tools/Fidelity/test_physical_ros_capture.py
```

The fixture creates only synthetic publishers and an isolated recorder, writes
`dataSource=synthetic-fixture` / `physicalRobotEvidence=false`, and never communicates with hardware.
Recorded acceptance evidence is under
`PerformanceResults/crane-fidelity-20261011/passive-recorder/passive-capture-loopback-result.json`.
