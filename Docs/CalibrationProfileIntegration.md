# Calibration profile integration

`Tools/Campus/fit_calibration_profile.py` creates a canonical
`crane-calibration-profile-v1` artifact from explicit `profileObservation` records in fit trials
only. It uses medians as a deterministic baseline estimator; it never infers parameters from
simulation priors, recorder time, or held-out samples. The artifact contains fit trial IDs,
the pre-profile manifest hash, source kind, and a canonical SHA-256 sidecar. The dataset
manifest retains the input stream hashes.

The Unity campus accepts the artifact with:

```text
--crane-campus-calibration-profile path/to/profile.json
```

`CraneCampusCalibrationProfile` validates the schema, source kind, bounds, and provenance,
computes the artifact hash, applies plant and LiDAR values, and records the profile identity and
hash in campus events. Existing command-line scalar options remain the nominal/engineering-prior
baseline when no profile is supplied. Synthetic fixtures are labeled `synthetic-fixture` and do
not establish physical calibration.

The held-out evaluator verifies the artifact hash, source kind, fit trial IDs, and fit-manifest
identity before computing residuals. A tampered or mismatched profile fails closed.

Authoritative replay headers also retain the explicit `--crane-seed`, profile path, and profile
SHA-256 in their runtime metadata, allowing replay results to be matched to the exact loaded
artifact.
