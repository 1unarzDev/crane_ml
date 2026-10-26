# CRANE-Fidelity pilot evidence, 2026-10-11

The user confirmed that physical CRANE robot logs and hardware are not available yet. They will
provide actual robot bags later and authorized public data for the pilot. This work therefore
separates public physical ingestion, CRANE simulation repeatability, and robot calibration.

The baseline remains `976ff1ec258c9d3cc8b873265c1ab4536966250c`. A separate Unity 6000.5.10f1
development worker was built in `Builds/CRANE-Fidelity` with the existing build method; retained
workers were preserved. Raw outputs are ignored under `PerformanceResults/crane-fidelity-20261011`.

## Public physical ingestion

Downloaded and processed LILocBench `dynamics_0` from the
[University of Bonn publisher](https://www.ipb.uni-bonn.de/html/projects/localization_benchmark/).
The external Dingo platform provides command/odometry/scan/joint streams and an AprilTag-based
reference trajectory. Its CC-BY-NC-SA-4.0 data are kept out of the Git repository. Source URLs,
artifact SHA-256 hashes, attribution, and limits are in
[`public_pilot_sources.json`](../Tools/Campus/public_pilot_sources.json).

Measured directly from the downloaded bag:

| Stream | Records | Header-derived rate |
| --- | ---: | ---: |
| Front laser scan | 2,400 | 15.0001 Hz |
| Rear laser scan | 2,400 | 15.0001 Hz |
| Wheel odometry | 2,328 | 14.5501 Hz |
| Joint state | 2,329 | 14.5564 Hz |
| Velocity command | 6,866 | Acquisition stamp absent |
| Published independent reference | 3,194 | approximately 20 Hz |

The bag spans 159.978 s. The reference spans 159.951 s and its raw sampled 3-D path length is
54.052 m. Reference sampling contains a maximum 150 ms interval; missing intervals are retained.
Scans and odometry have nonzero increasing top-level header stamps. Every command is an unstamped
Twist; its acquisition, publication, and actuator application times remain unknown. Record-minus-
header differences are reported separately and are **not** transport-latency measurements.

Each laser produced 1,946,400 ray records. The front stream contains 18,507 non-finite or
out-of-range returns; the rear contains 19,940. These are observed validity counts, not calibrated
dropout probabilities: an unavailable return can also reflect range limits or scene geometry.
Joint efforts retain their raw ROS field semantics; electrical-current units are not inferred.

The importer exports poses with a fixed FLU-to-RUF axis/yaw conversion, commands, joint states,
scan statistics, reference poses, timing distributions, and hashes. It does not register trajectories,
derive a sensor-error profile from raw ranges, or substitute recording time for acquisition.

Reproduce the ingestion after downloading and verifying the pinned source artifacts:

```bash
python3 Tools/Campus/download_public_pilot.py \
  --output PerformanceResults/public/lilocbench
python3 -m venv /tmp/crane-fidelity-venv
/tmp/crane-fidelity-venv/bin/pip install rosbags==0.11.0
/tmp/crane-fidelity-venv/bin/python Tools/Campus/ingest_public_bag.py \
  path/to/dynamics_0.bag --output path/to/new-output-directory \
  --source-url https://www.ipb.uni-bonn.de/html/projects/localization_benchmark/ \
  --license CC-BY-NC-SA-4.0 --ground-truth-tum path/to/dynamics_0_gt.txt
```

This proves a physical-data ingestion path. The platform and sensors differ from the CRANE
flagship robot, and command timing is incomplete, so it does not validate H1-H4 or physical transfer.

## CRANE simulation repeatability

Ran three separate CAL-02 processes with contact wheels, a 10 ms fixed step, nominal engineering
parameters, the same seed, no graphics, and ROS transport suppressed. Each run executed the fixed
5 s, 0.2 m/s velocity step followed by stopping.

- All three workers reported valid runs near 1.000 real-time factor and zero collisions.
- Sampled path length was 1.004595 m; observed 90% rise time was 0.400 s.
- Sampled stopping distance was 0.044169 m.
- The velocity gate passed: steady body speed 0.199994 m/s, startup displacement 0.000047 m.
- Existing parity comparisons for runs 1/2 and 1/3 passed all gates. At the common resampled times,
  pose, yaw, speed, and angular-rate residuals were exactly zero in the serialized evidence.

This is simulator repeatability for one command profile. It is not real-to-real repeatability or
a physical plant fit. The contact gate does not support CAL-02; an attempted invocation rejected
that trial, and the existing CAL-02 velocity gate was used instead.

Commands:

```bash
CRANE_PLAYER="$PWD/Builds/CRANE-Fidelity/CRANE.x86_64" \
CRANE_RESULT_ROOT="$PWD/PerformanceResults/new-cal02-run" \
  bash Tools/Campus/run_calibration.sh CAL-02 0.01 headless
python3 Tools/Campus/check_velocity_step.py PerformanceResults/new-cal02-run
python3 Tools/Campus/compare_physics_runs.py path/to/run1 path/to/run2 \
  --output path/to/comparison.json
```

## Remaining hypothesis gates

| Hypothesis | Current evidence | Blocking input |
| --- | --- | --- |
| H1 plant | Synthetic response and repeatability; public ingestion | CRANE commands, actuator state, independent physical trajectories and held-out trials |
| H2 sensor | Engineering-prior injection; public scan stream audit | Surveyed targets, measured errors, conditions, repeated sequences |
| H3 detection | Geometry/visibility heuristic | Real detector outputs plus annotations and inference timing |
| H4 temporal | Runtime provenance and captured-acquisition timestamp corrections; Unity timing tests | Hardware clock synchronization and publication/receive/application events |
| H5 procedural | Existing deterministic topology/dynamic fixtures | Held-out physical routes and equal-budget task comparisons |
| H6 tradeoff | Existing simulator throughput benchmarks | Paired physical predictivity across fidelity conditions |

No hypothesis is reported confirmed. A neutral or negative residual result must be retained when
actual bags arrive. Sensor/plant profile fitting, live hardware collection, real-to-real repeatability,
and physical closed-loop comparisons remain explicitly blocked by the user's unavailable robot data.
