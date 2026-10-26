# CRANE-Fidelity blockers and next actions

## `PHYSICAL_DATA_BLOCKED`: CRANE platform measurements unavailable

The workspace currently has no physical CRANE robot endpoint or synchronized CRANE bags/MCAP.
This blocks the minimum publishable gates for measured plant/actuator behavior, sensor and
detector calibration, clock latency, held-out physical residuals, and paired closed-loop transfer.

Available public data cannot remove this blocker: LILocBench is a Clearpath Dingo with unstamped
commands; the DTU subset lacks ZED frames and commands; AutoNaut is environmental context only;
other candidates lack the required combination of actuation, independent truth, and timing.

## Prepared collection path

1. Run `Tools/Fidelity/physical_ros_capture.py` passively on the physical ROS graph with metadata
   containing platform/session/site IDs, serials, firmware, frame transforms, clock method and
   uncertainty, conditions, route, and command-program identity.
2. Preserve raw CDR and all acquisition/reception/application timestamps; reject or explicitly
   mark missing clocks rather than substituting recorder time.
3. Export whole trials into the manifest defined by `Tools/Campus/heldout_calibration_schema.json`.
4. Measure repeated real runs before fitting any simulator profile. Reserve a route/day/condition
   split for held-out evaluation and report failures, drops, stale actions, and invalid layouts.
5. Fit one land plant and sensor profile with `Tools/Campus/fit_calibration_profile.py`, load it
   through `--crane-campus-calibration-profile`, and execute the held-out route or docking
   comparison against ideal, engineering-prior, i.i.d. Gaussian, and simplified baselines.

No additional hardware-dependent work should be represented as complete until those artifacts exist.
