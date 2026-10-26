# CRANE-Fidelity status and evidence boundary

This document records the state of the CRANE-Fidelity research path at the current
repository baseline. It is deliberately evidence-first: simulator capability, synthetic
fixtures, and throughput measurements are reported separately from physical validation.

## Frozen baseline

The working tree was inspected on 2026-10-11. The tracked checkout is
`976ff1ec258c9d3cc8b873265c1ab4536966250c`, the same commit as `origin/main` and the
previously audited baseline. The only pre-existing working-tree change is the untracked
`Docs/LeanCraneDevelopmentPlan.md`; it is preserved.

The existing Unity project is Unity `6000.5.10f1` with HDRP `17.5.0`. The accepted runtime
chain is:

```text
CraneSimulationClock / FixedUpdate
  -> environment and vehicle physics
  -> sensor acquisition
  -> ROS serialization and transport
  -> external controller
  -> action mailbox
  -> CraneActionGate
  -> fixed-step actuator application
```

`CraneObservationJournal` records acquisition and observation-construction completion metadata
for observations. `CraneActionGate` records source-observation, receive, and application ticks
for accepted commands. These are useful provenance seams, but they do not make a run physical
without synchronized physical measurements.

## What is validated

- The land path has a repeatable flat-ground Ackermann/differential-drive fixture and synthetic
  CAL-01 through CAL-08 command programs.
- LiDAR, geometric depth, ROS transport, action freshness, reset, replay, Nav2 fixtures, and
  throughput have automated checks described in the architecture and performance documents.
- Existing campus calibration outputs are explicitly simulation evidence. Their current analyzer
  reports `physicalFitAvailable: false` unless an external real log is supplied.
- The current LiDAR error injector uses configurable engineering-prior sigma, dropout, material
  response, and scan delay. The detector uses geometry, visibility, and a confidence heuristic.
  Neither has been fitted to a physical sensor or detector output in this checkout.

The original Campus baseline suite passed with 62 tests. After adding the strict held-out
manifest/importer, public-data, and profile-integration fixtures, the combined Campus/Fidelity
suite passes with 91 tests. These tests establish software contracts and synthetic repeatability; they do not
establish robot, sensor, or site validity.

The pinned Unity EditMode acquisition, profile-loader, and replay-provenance suite passes 11/11
tests under Unity `6000.5.10f1`; its report is retained at
`PerformanceResults/crane-fidelity-20261011/profile-loader-test/results-replay-v5.xml`.

## Evidence that is not present

At baseline inspection the repository contained no physical TurtleBot3/industrial-campus run,
ROS bag, MCAP, synchronized
clock record, actuator current/RPM trace, wheel encoder capture, VLP-16 packet stream, ZED 2i
capture, detector output log, or surveyed site measurement. Consequently the following claims
are currently blocked:

- calibrated plant or actuator parameters;
- measured sensor noise, dropout, temporal correlation, or latency;
- detector confidence, false-positive, missed-detection, 3-D box, or inference-latency models;
- held-out physical trajectory, event-timing, sensor-distribution, or closed-loop navigation
  validation;
- a physical sim-to-real improvement claim for any platform or domain.

A subsequently downloaded public Dingo bag exercises physical ingestion. Its measured contents
and limitations are recorded in [Pilot evidence](CRANE-Fidelity-PilotEvidence.md); it does not
replace unavailable CRANE robot data or close the calibration gates above.

The surface-water and aerial/underwater paths remain conditional extensions. They must not block
the minimum land study, and their engineering models must not be described as physically validated.

## Minimum publishable land study

Use one instrumented TurtleBot3 (or the named CRANE industrial/campus platform) and a surveyed,
repeatable calibration route. Every trial must contain:

1. a command program with a trial UUID and device-clock timestamps;
2. actuator command and, when available, wheel encoder/current measurements;
3. ground-truth or reference pose with its own acquisition timestamps;
4. sensor records (at least the target LiDAR and one timing reference);
5. ROS acquisition, publication, receive, and application timestamps;
6. calibration metadata: robot/sensor serials, firmware/software revisions, frame transforms,
   clock synchronization method and uncertainty, site/material identifiers, environmental
   conditions, and file hashes;
7. an explicit split manifest assigning whole trials/routes/days to calibration, development,
   and held-out evaluation. Samples from a held-out trial must never be used for fitting or
   threshold selection.

The first experiment should isolate acceleration, braking, turning, obstacle clearance, and one
docking or recovery task. Repeat each command profile across at least two sessions; reserve a
session or route for evaluation. Report residuals and event timing with uncertainty, then compare
the calibrated profile against the nominal CRANE model, independent Gaussian noise, and the ideal
sensor condition under equal command sequences and simulation budgets.

## Acceptance gates before a sim-to-real claim

The physical path is ready for a paper claim only when all gates below have artifacts in the
repository or an immutable results store:

- schema validation and manifest/hash verification pass;
- clock offset and drift are estimated from recorded synchronization evidence;
- real-to-real repeatability is reported before simulator fitting;
- a calibration profile is generated using only the calibration split;
- held-out trajectory, event timing, and sensor residuals are reported without post-hoc retuning;
- a closed-loop route or docking task is repeated in simulation and on the robot;
- failures, invalid layouts, dropped messages, and stale actions are counted rather than removed;
- the report states the tested land/surface-water conditions and does not generalize beyond them.

Until those gates pass, CRANE-Fidelity should be described as a measurement-ready calibration and
benchmark infrastructure effort, not as a demonstrated sim-to-real improvement.

