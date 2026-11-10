# CRANE local patches to the imported autonomy library

The imported copy (`Assets/Scripts/Fleet/Autonomy`) differs from upstream (`crane_ml/Tools/Fleet/core_cs/Crane.Fleet.Autonomy`) by
`crane_local_patches.diff`. That diff is for the library owner to apply upstream; the next import replaces the copy.

## The patches
- **Handoff carry keeps the receivers as obstacles** when tag alignment is on. The carry goes to the pre-place pose, using goal search.
- **New failure class `Tipping`, not retried.**
  - It comes from `LiftReport.TiltRad`, a new field holding the IMU tilt.
  - The threshold is `AutonomyAssumptions.TipTiltLimit` (0.30 rad).

## Tests (dotnet, no Unity)
`CranePatchTests.cs` runs against the upstream harness, which is compiled read-only from crane_ml. The project lives in the cache;
`autpatch.csproj.template` is a copy of it.

    mkdir -p ~/.cache/crane-fleet/autpatch && cp autpatch.csproj.template ~/.cache/crane-fleet/autpatch/autpatch.csproj
    ln -sfn /home/lunarz/worktrees/crane-fleet/Tools/Fleet/spec ~/.cache/crane-fleet/autpatch/spec
    cd ~/.cache/crane-fleet/autpatch && nice -n 10 dotnet test

## Results (2026-11-10)
- 100 of 103 tests pass.
- The 3 failures are environment-only: those upstream tests scan source paths relative to the upstream tree (schema file, engine
  references, type names).
- The handoff test fails 5/5 without the patch ("lifter_1 hit cargo_1") and passes 5/5 with it.
- Tipping has no harness test: the upstream harness does not report `TiltRad`. It is verified in CRANE.
