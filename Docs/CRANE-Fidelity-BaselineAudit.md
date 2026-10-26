# CRANE-Fidelity baseline audit

Audit date: 2026-10-11  
Repository: `crane_ml`  
Current `HEAD`: `976ff1ec258c9d3cc8b873265c1ab4536966250c` (`origin/main`, same as the 2026-10-10 audited baseline)

The checkout has no tracked modifications. The only pre-existing untracked file at audit time is
`Docs/LeanCraneDevelopmentPlan.md`.

## Validated baseline

- Unity 6000.5.10f1 / HDRP 17.5.0 is the documented resolved stack.
- Aquatic Roboboat and Robosub scenes have validated water/vehicle/sensor benchmarks.
- Land has repeatable flat-ground Ackermann and TurtleBot3 campus fixtures; aerial has analytic
  multirotor and direct-PWM UDP protocol-to-plant fixtures.
- ROS-TCP transport, fixed-step action gating, acquisition/receive/application provenance, scene
  reload, authoritative replay metadata, and live Jazzy Nav2 authoritative-odometry navigation
  fixtures are implemented (see `Docs/ArchitectureAndDesign.md:42-60`).
- Geometric depth is validated for non-aquatic null-graphics use; GPU depth is validated at 2× on
  the reference machine. Throughput is reported separately from sensor and task validity.
- Reproducible Python contract/evidence tests pass: `python3 -m pytest -q Tools/Campus` (62 passed,
  2026-10-11).

The checked-in runtime fixture entrypoints include `Tools/Campus/run_calibration.sh`,
`Tools/Performance/run_turtlebot3_nav2_fixture.sh`, `run_land_proving_ground_nav2_fixture.sh`,
`run_nav2_controller_fixture.sh`, `run_roboboat_evidence_capture.sh`,
`run_roboboat_command_response.sh`, `run_sitl_udp_fixture.sh`, and
`run_aerial_sitl_fixture.sh`. Their retained outputs under `PerformanceResults/` are simulator,
transport, or protocol evidence; none is a physical-robot run.

## Evidence boundaries

The repository currently contains no physical ROS bags, MCAP/DB3 logs, synchronized hardware
telemetry, or detector output captures. Existing calibration runs are Unity-only command-response
experiments. `Tools/Campus/calibration_schema.json` requires simulated time, pose, yaw, and velocity;
`Tools/Campus/analyze_calibration.py` defaults to `physicalFitAvailable=False` and its optional
real-file path performs nearest timestamp matching. `Tools/Campus/sensor_error_evidence.py` labels
its results engineering-prior injection evidence, not measured sensor failure rates.

Consequently, the following remain unvalidated: real actuator/plant identification and held-out
trajectory prediction; measured LiDAR (VLP-16-compatible) and ZED 2i noise/dropout/correlation;
detector confidence/missed detections/false positives/3-D error/latency; acquisition-publication-
reception-application timing against hardware clocks; physical site geometry distributions; and any
physical closed-loop sim-to-real claim. These limits are also stated in
`Docs/ArchitectureAndDesign.md:29-36,47-55,549-603` and
`Docs/SimulationPhysicsAndRuntimeModes.md:154-167,226-229,241-258`.

## Highest-value missing seam

Add one immutable TurtleBot3 physical acquisition contract and importer before expanding domains.
Each record should carry command/actuator state, pose/velocity, raw sensor or detector summary,
device and simulation clocks, acquisition/publication/reception/application timestamps, route/site
metadata, uncertainty, provenance hashes, and an explicit calibration/held-out split. Fit profiles
from the calibration split, load them into the existing campus plant/sensor components, and report
held-out residuals without retuning. This is the minimum infrastructure needed to test H1-H5 and to
make a physical closed-loop claim; unavailable hardware/logs are the current blocker to that claim.

