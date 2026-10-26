# CRANE-Fidelity current state

State date: 2026-10-11. Baseline `HEAD` is `976ff1ec258c9d3cc8b873265c1ab4536966250c`,
matching `origin/main` and the prior audit. The worktree contains the measurement-ready
infrastructure changes described below; the pre-existing untracked
`Docs/LeanCraneDevelopmentPlan.md` is preserved.

## Implemented and verified

- Acquisition-aligned calibration manifests and held-out residual evaluation are implemented
  in `Tools/Campus/heldout_calibration.py` and `validate_heldout_calibration.py`. The evaluator
  checks hashes, clock/frame/profile provenance, explicit time origins, factor-disjoint splits,
  overlap, interpolation gaps, duplicate paths, and fair baseline grids.
- The fit-to-runtime path is implemented in `Tools/Campus/fit_calibration_profile.py` and
  `Assets/Scripts/Physics/Land/CraneCampusCalibrationProfile.cs`. Synthetic or future physical
  fit artifacts are canonical, hash-checked, applied to plant/LiDAR parameters, and recorded in
  campus events; tampered or mismatched artifacts fail closed.
- Passive ROS 2 capture is implemented in `Tools/Fidelity/physical_ros_capture.py`. It preserves
  raw CDR, message views, acquisition headers, independent reception clocks, topic gaps, hashes,
  and explicit physical/synthetic source markers. The retained Docker loopback fixture has 553
  synthetic records across seven topics and rejects `use_sim_time=true`.
- Async RGB/depth and delayed LiDAR provenance fixes are implemented. The Unity EditMode timing,
  profile-loader, and replay-provenance report `PerformanceResults/crane-fidelity-20261011/profile-loader-test/results-replay-v5.xml`
  records 11/11 passing tests.
- Authoritative replay runtime metadata now records the explicit seed and calibration-profile path
  and SHA-256 when supplied, so replay evidence can be tied to the loaded profile artifact.
- The machine-readable synthetic integration evidence is retained at
  `PerformanceResults/crane-fidelity-20261011/profile-loader-test/evidence.json`.
- The Unity worker was built and three CAL-02 simulation runs were compared. They reproduce the
  same nominal command response at approximately 1x real time with zero collisions; this is
  simulation repeatability, not physical calibration.
- Public pilot ingestion is operational for LILocBench Dingo data. DTU Roadrunners and AutoNaut
  sources are also retained as bounded sensor/environment metadata pilots. Source rights and
  limits are recorded in the public-data audit documents.
- Combined Python validation currently passes with 91 tests:
  `python3 -m pytest -q Tools/Campus Tools/Fidelity`.

## Current evidence boundary

No CRANE robot hardware, synchronized CRANE ROS bag/MCAP, actuator feedback, VLP-16-equivalent
calibration sequence, ZED 2i capture, detector output log, or surveyed CRANE route is available.
The public datasets use other platforms or lack the timing/actuation evidence needed for CRANE
claims. No physical plant, sensor, detector, timing, held-out residual, or sim-to-real result is
reported as complete.

## Next gate

When the first CRANE bag arrives, validate it with the passive recorder manifest and physical
calibration contract, measure real-to-real repeatability before fitting, then fit one land plant
and sensor profile using whole-trial splits. The first physical closed-loop route or docking run
must remain held out until the profile residual gate passes.
