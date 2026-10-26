# Physical calibration data contract

The checked-in campus calibration runs are simulation evidence and retain
`physicalFitAvailable: false`. They must not be presented as robot calibration.
Physical measurements become eligible for a held-out residual report only after
they are exported into the acquisition-aligned contract described here.

`analyze_calibration.py --real` is now deliberately disabled: an arbitrary JSONL
stream cannot set `physicalFitAvailable` to true. Simulation-only invocations
retain their prior behavior and result format. The explicit fit-to-runtime path is
documented in [CalibrationProfileIntegration](CalibrationProfileIntegration.md).

`Tools/Campus/heldout_calibration_schema.json` defines a manifest with:

- `timeBasis: "acquisition"` and `coordinateFrame: "unity-right-up-forward"`;
- fit and held-out trials with scalar command, route, material, day, and
  condition factors;
- separate simulation and physical JSONL paths (and an optional nominal
  baseline simulation path).
- expected SHA-256 hashes, source kind (`physical` or `synthetic-fixture`),
  synchronization/clock-uncertainty metadata, a documented frame transform,
  calibration-profile provenance, and comparison coverage/interpolation limits.

Each physical JSONL row uses `acquisitionTime` in seconds, `position` (`x`,
`y`, `z` in metres), `yaw` in degrees, and optionally `velocity`. The exporter
must record the source clock and the one-time FLU-to-Unity conversion in the
manifest or acquisition notes. Receive or file-write timestamps cannot replace
acquisition time. Simulation telemetry normally uses `simulationTime`; custom
timestamp fields can be declared per manifest or trial.
Every physical sample must carry `clockUncertaintySeconds`, bounded by the
manifest's declared maximum. Every trial explicitly declares
`simTimeOffsetSeconds`, `physicalTimeOffsetSeconds`, and `timeOriginEvidence`.
The evaluator subtracts these recorded origins once; raw Unity telemetry is not
assumed to start at command-program zero.

Validate a dataset with:

```bash
python3 Tools/Campus/validate_heldout_calibration.py path/to/manifest.json
```

The validator rejects duplicate trials and canonically resolved sample paths, non-monotonic streams,
missing factors, fit/held-out factor leakage, missing acquisition timestamps,
and comparisons with no overlapping time range. It pairs streams by linear
interpolation inside the measured range, never extrapolates or shifts a trial
to improve its fit, and writes SHA-256 provenance for every input. It reports
position, yaw, and optional velocity residuals separately for fit and held-out
trials. If a `baseline` stream is supplied, the report records the measured
comparison only when candidate and baseline use the same comparison time grid.
Selected `heldOutFactors` must be disjoint between fit and held-out trials;
the study can declare one factor or several, without claiming untested factors.

Without preregistered position and yaw residual thresholds, the output state is
`RESIDUALS_COMPUTED`. Explicit thresholds produce `HELDOUT_RESIDUAL_PASS` or
`HELDOUT_RESIDUAL_FAIL`. Synthetic input always produces
`SYNTHETIC_FIXTURE_RESIDUALS`. These states do not promote a nominal model into a
measured calibration profile.

## Calibration profile artifact

`Tools/Campus/calibration_profile_schema.json` and
`Tools/Campus/calibration_profile.py` define the strict
`crane-calibration-profile-v1` artifact. The fitter consumes only explicit
`profileObservation` objects in trials assigned to the `fit` split. It fails closed
when a parameter is absent; it never fills a missing value from an engineering
prior or from a held-out sample. The current fitter aggregates the fit observations
with deterministic medians and records the fit trial IDs and a hash of the manifest
identity before the profile reference is attached. The dataset manifest retains the
SHA-256 hashes of each fit stream.

For a synthetic fixture or a physical identification export, fit and attach a
profile with:

```bash
python3 Tools/Campus/fit_calibration_profile.py dataset/manifest.json \
  --output dataset/land-profile.json --id campus-land --version 1 \
  --source-kind physical --update-manifest dataset/manifest-with-profile.json
python3 Tools/Campus/validate_heldout_calibration.py dataset/manifest-with-profile.json
```

When `artifactPath` is present, the held-out evaluator verifies canonical JSON,
the SHA-256 artifact hash, source kind, exact fit trial IDs, and the pre-profile
manifest identity before reading residual streams. A generated profile is an
identification artifact, not evidence that the corresponding CRANE plant or sensor
is physically valid; that boundary remains until synchronized physical logs and
held-out real runs are available.

The report boundary is deliberate: passing this tool establishes acquisition-
aligned residual evidence only. It does not establish physical closed-loop
navigation, detector validity, or sim-to-real transfer. Those claims require
the synchronized sensor/action logs and repeated real runs described by the
research protocol.

