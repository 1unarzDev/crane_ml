# Multi-rate metamorphic test for the .NET RK4 plant: spec for the ops agent (NOT RUN by D3)

Status: DRAFT spec, 2026-11-10. D3 ran only the NavPilot half (`results/D3_dev.md` section 4). Purpose: decide whether the .NET
harness may integrate the plant at 500 or 250 Hz instead of 1000 Hz (expected 2-4x harness speed-up; record the verdict in HARNESS.md).
Acceptance rule (same as NavPilot): collision outcome agrees on >= 99 % of paired runs AND |min hard-clearance difference| <= 2 cm
(report mean, median, p95).

## Finding: the plant rate is NOT a config knob today
- `AutonomousMission.cs:87`: `new CraneAerialPlantCore(vehicle, 1000.0, 500.0)` (substep Hz, control Hz hard-coded).
- `CraneAerialPlantCore` ctor throws if `substepHz < 1000` ("plant must run at >= 1 kHz"); `ControlEvery = round(substepHz/controlHz)`;
  `AttachImu(new CraneAerialImu(500.0, ...))` requires the IMU rate to divide the substep rate; `Advance(dt)` uses `n = round(dt/SubstepDt)`
  sub-steps and silently changes h when `dt` is not a multiple of `SubstepDt` (5 ms tick / 4 ms substep -> n = 1, h = 5 ms).
- Config specs (`"config": {...}`) set `AutonomyConfig` public fields by reflection (`ExperimentSpec.cs:184`); there is no plant-rate field.

## Required code change (ops agent; behind default-preserving fields)
Add to `AutonomyConfig`: `public double PlantSubstepHz = 1000.0, PlantControlHz = 500.0, ImuRateHz = 500.0;` and use them at
AutonomousMission.cs:87-88. Relax the ctor guard to `>= 250` ONLY when `PlantSubstepHz` is explicitly set (keep 1000 default; add a
unit test that the default path is bit-identical to the current one: `Tools/CraneHarness/test_crane_harness.py` / `test-lead`).
Constraints to satisfy in every arm: `TickNs` must be a whole number of substeps and `substepHz % controlHz == 0`, `substepHz % ImuRateHz == 0`.

## Arms (hold TickNs = 4 000 000 ns, i.e. 250 Hz event tick, in ALL arms so sensors/planner/command latching are identical)
| arm | PlantSubstepHz | PlantControlHz | ImuRateHz | substeps per tick | note |
|---|---|---|---|---|---|
| ref | 1000 | 500 | 500 | 4 | the current plant, but with TickNs 4 ms (a re-baseline: it differs from the 5 ms-tick runs) |
| A | 500 | 500 | 500 | 2 | plant integration only (control/IMU unchanged) |
| B | 250 | 250 | 250 | 1 | plant + control + IMU rates all halved/quartered (a documented confound: controller discretisation changes) |
Report A (clean integration-step test) and B separately. If TickNs must stay 5 ms for continuity, use rates 1000/500/200 Hz (5/10/25 ms
multiples are not available at 2.5 ms substeps: 200 Hz => 1 substep per tick) instead of 250.

## Runs
Reuse the recfeas/attrib setup: `Tools/CraneHarness/specs/recfeas_sweep.json` forest 1337 (manifest in that spec), missions `cross` 10 +
`corridor`, planner `map_rhp`, wind calm, `run_seeds` [1,2]. Speeds 5, 6, 8 m/s only (the collision-relevant regime: 0 collisions at
<= 4 m/s). Paired by `pair_key`. Size: 11 missions x 3 speeds x 2 seeds = 66 runs per arm x 3 arms = 198 runs (~1.5 core-h at ~25 s each if
the ~0.4 real-time factor of the other batches holds; arms A/B should be 1.3-2x faster). One process at a time under `fsrun`.
Cheaper first pass: speeds {6, 8}, seed 1 only: 44 runs (reference expected collision rate 41-75 % at these speeds gives informative pairs).
Statistic: outcome agreement with Wilson 95 % CI; min-clearance difference from the run summary (`min_hard_clearance`); for disagreeing pairs
dump the replay bundles (`bundle_failures`) and compare the first tick of divergence. Because collisions at 5-8 m/s are near-chaotic
(NavPilot analogue below), also run `ref` twice with RunSeed+0 to confirm bit-determinism, and `ref` with a 1e-9 m initial-position
perturbation as the chaos floor: a lower rate "passes" only if its disagreement is within that floor.

## Expected NavPilot-side reference
See `results/D3_dev.md` section 4 for the NavPilot 200/100/50 Hz numbers (event tick fixed at 5 ms in all arms).
