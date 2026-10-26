# CRANE-Fidelity timing and sensor audit

Audit date: 2026-10-11. Frozen starting commit:
`976ff1ec258c9d3cc8b873265c1ab4536966250c` (matches the previous audited baseline).
This document records runtime implementation findings, not physical sensor validation.

## Runtime chain and available evidence

`CraneSimulationClock` runs before ordinary fixed-update components and advances
`CraneRuntimeMetrics.SimulationTick` once per Unity fixed step. Episode changes reset
the simulated ROS clock; physics components enqueue or apply forces in their fixed
updates, and Unity performs PhysX integration after those callbacks. Ordinary sensor
publishers acquire in `FixedUpdate`, so their samples represent the state available
at that callback, generally the preceding completed physics step. The replay
recorder captures at `WaitForFixedUpdate` after integration. The same integer tick
alone does not prove that a sensor sampled the post-integration state written in
that replay frame. Execution phase must accompany any precise timing claim.

| Stage | Existing evidence | Limit |
|---|---|---|
| Acquisition | Sensor tick/time in `CraneObservationMetadata`; captured request identity for async cameras | GPU readback request time is not a measured camera exposure timestamp; render-texture production/exposure phase is not independently logged |
| Construction completion | `CompletionTick`/`CompletionTime` in observation metadata | Journal is called before `ROSConnection.Publish`; completion is not transport publication |
| ROS submission | Profiler marker around the connector call | No per-message enqueue, serialization, TCP write, DDS publication, or receiver timestamp is recorded in Unity's observation journal |
| ROS delivery | ROS fixture/bridge receives headers and stamps commands using the latest delivered odometry | Does not prove which observation Nav2 internally consumed |
| Command receive | `CraneActionReceipt.ReceiveTick`, sequence, source, episode | Sub-tick wall timing is unavailable; ROS command adapters derive source ticks from a stamp divided by fixed delta |
| Application | `CraneActionGate` accepts on the actuator's `FixedUpdate`; accepted payload recorded after mutation | Command acceptance time is not measured physical wheel/thruster response; shared metrics describe most recent accepted action |
| Replay | v5 acquisition/completion metadata and v2 receive/application ticks; v4 build provenance | Metadata-only sensor payload policy does not reproduce GPU noise, detector inference, exposure history, or internal controller state |

`CraneActionGate` rejects cross-episode receipts, duplicate/out-of-order sequences,
and stale commands under the configured bounded policy. Unstamped thruster and SITL
messages retain unknown source-observation provenance. The one-slot mailbox
intentionally replaces pending commands. Replacement count is not recorded, so
accepted-action counts alone cannot measure delivered-command losses.

## Targeted corrections made in this work

1. `ROSPublisher.CreateHeader(double)` permits the ROS header to identify captured
   acquisition time. The zero-argument API remains compatible for synchronous
   consumers. RGB/depth async image constructors now use their request's captured
   simulation time instead of callback completion time.
2. Camera callbacks capture the exact request metadata object. Completion removes
   that object by identity. Reset clears pending identities, preventing an old
   callback from consuming a new episode's metadata and tolerating completion order
   changes. Tests cover the stale callback path without reading GPU buffers.
3. Delayed `Lidar2D` samples carry captured acquisition tick/time beside their
   immutable message payload. `ROSPublisher` reads the returned sample's metadata
   after construction. This removes the mismatch where an old header/payload was
   journaled as a new acquisition. LiDAR reset also clears delayed scans and restores
   the configured random seed.

The existing delay queue warmup repeats its oldest sample; this behavior was
preserved to avoid an unreviewed change to delivery behavior. Repeated publication
is not an independent acquisition and should be accounted for when comparing
sensor distributions. `GeometricDepthCamera` captures synchronously in message
construction; it does not require an async request identity. Its collider-derived
depth remains a lower-fidelity baseline, not a calibrated ZED depth model.

## Verification

`Assets/Tests/Editor/SensorAcquisitionTimingTests.cs` exercises explicit headers,
RGB/depth captured timestamps, stale callbacks after reset, and actual delayed
LiDAR publication through the observation journal. Run on the pinned Unity editor:

```bash
Unity -batchmode -nographics -projectPath /home/lunarz/crane_ml \
  -runTests -testPlatform EditMode \
  -testFilter Sim.Tests.Editor.SensorAcquisitionTimingTests \
  -testResults PerformanceResults/crane-fidelity-20261011/sensor-timing-tests.xml \
  -logFile PerformanceResults/crane-fidelity-20261011/sensor-timing-tests.log \
  --crane-disable-ros
```

The suite covers metadata correctness and reset isolation. It cannot establish
physical camera exposure, sensor driver transport delay, hardware clock offset,
physical actuator application, or prediction of a robot's closed-loop behavior.
The run's actual verdict is recorded in the research evidence manifest.

## Sensor calibration readiness

`Lidar3D` is an ideal collider raycast/XYZ cloud model with uniform configured
angles, not a full VLP-16 observation model. Measured beam angles, firing offsets,
per-point relative time, ring/intensity/return fields, angular and distance errors,
motion distortion, correlated dropout, and material-dependent residuals remain.
`Lidar2D`'s bounded Gaussian/dropout/material/scan-delay parameters are engineering
priors; they are not fitted from physical measurements. Material names do not
identify calibrated optical reflectance or actual material response curves.

Rendered RGB/depth use shaders and bounded GPU readback, while geometric depth
uses pinhole rays against colliders. Neither establishes ZED 2i-compatible
stereo-depth failures, confidence, temporal correlation, image/depth alignment,
exposure/gain/rolling-shutter behavior, or condition-dependent residuals.
`BoundingBox3D` samples a perfect object registry and heuristic geometry/confidence;
it is not an evaluated physical detector with calibrated false-positive and
missed-detection processes. Existing GPS/IMU/odometry covariance and error values
must remain labeled engineering priors until validated against independent truth.

## Required physical timing contract

For every recorded source, preserve raw acquisition header time, bag/recorder
receive time, publisher time when independently instrumented, clock identity,
clock synchronization method/offset/uncertainty, timestamp origin, sequence/frame,
and known dropped/repeated records. For actions, add command identity, linked
observation identity with an explicit provenance kind, command publication,
receive/application timing, and physical actuator feedback. Unknown timestamps
must be null with an explanation; never substitute bag time for acquisition time.

Latency subtraction is valid only within a known shared clock domain or after a
measured clock conversion. Acquisition-to-recorder difference includes transport,
driver stamping, buffering, and clock offset; it is not sensor latency by itself.
H4 remains blocked until synchronized physical timing and closed-loop observations
exist. Public bags can exercise ingestion and characterize available clocks, but
cannot fill missing publication or physical actuation timestamps by inference.
