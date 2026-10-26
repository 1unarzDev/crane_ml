# I1 council: hydrodynamics profile decision (systems & architecture critic)

Date: 2026-10-20. This was a read-only review. The only file written is this one. No git writes, no Unity, no ROS, no hardware.
Inputs: CONTRACTS.md, PREREGISTRATION.md, I1-truth-inventory.md, I1-adversarial-design.md. I also read the code and retained evidence cited below.

## 0. The finding that changes the question

The brief assumes one "live RoboBoat water physics". There are two, and the failing characterization cases run on the one without FossenDynamics.

| Plant | Where | Hull model | Thrust | Who runs it |
|---|---|---|---|---|
| **Legacy** | `crane_ml` main @ `976ff1e`. Scene `Roboboat Course.unity:3685-3688` enables Fossen. | FossenDynamics (V1/V2 defects) + GeneralDynamics (V3) + PhysX damping 4/15 + Current | 3500 N/thruster (D17) | Nav2 baselines on main (`nav2-navigate-loop-v1`, feed-forward 0.21/0.24 in `ROSOmniXCommand`) |
| **RoboBoatPhysics** | Unmerged branches `roboboat-thruster-pwm` @ `bf82744` and `test/characterization-faults` @ `dccb3e6`. Code: `Assets/Scripts/Controllers/RoboBoatPhysics.cs`. | `RoboBoatHull`: diagonal linear+quadratic drag in the REP-103 body frame, relative to the water. **Fossen, GeneralDynamics and Current are disabled** (`RoboBoatPhysics.cs:100-104`). Residual PhysX damping 0.05/0.2. | T200 51.5/40.2 N, spin-up 0.1 s | Default whenever `--crane-roboboat-contract` is set, unless `--crane-roboboat-legacy-physics` is also set (`:41-56`). This is every mhseals characterization case. |

Evidence: **34/34** `player.log` files under `PerformanceResults/calibration-*` and `calibrated-*` contain `CRANE_ROBOBOAT_PHYSICS mass=30.0000019 … thrust=51.5/40.2N`. That line is logged only by `Apply()`, which disables Fossen. The `reference` run (`calibration-isolated-reference-drag-20261019/reference/worker-0/player.log:557`) shows it, along with `CRANE_ROBOBOAT_CONTRACT_REPLACED sensor=imu_link type=Imu`.

**So the Fossen defects V1, V2 and V3 cannot explain the `reference` failure.** The two problems are separate. Coupling them in one decision would send a hydro fix after a criterion problem.

## 1. Why `reference` fails (diagnosis from retained data)

`calibration-isolated-reference-drag-20261019/reference/result.json`:
- The automated acceptance checks pass (translation ≤ 0.05 and yaw ≤ 0.03).
- The manual check `translation_cross_axis_rms_at_most_0_05_m_s` fails. The only failing row is the after-decoupling reverse-surge row: command −0.4 gives sway mean 0.052 m/s, RMS **0.0534**, against a 0.05 limit. In the same row yaw is 0.023 rad/s. Before decoupling the same row was 0.0116 m/s sway and −0.0018 rad/s yaw.

Worst translation cross-axis RMS before and after decoupling, by run:

| Run | before | after |
|---|---|---|
| calibration-faults/reference | 0.0538 | 0.2093 |
| faults-instrumented/reference.previous | 0.0539 | 0.2087 |
| faults-instrumented/reference | 0.0389 | 0.0504 |
| isolated-reference-drag/reference | 0.0437 | 0.0534 |

Reading:
1. The uncorrected plant sits at 0.039–0.054, right on the 0.05 limit. Each configuration has n = 1, so the pass/fail bit is mostly run-to-run variance.
2. The fitted decoupling never improves the worst row. In two runs it made it 4× worse. That points to the mhseals characterization/decoupling procedure, or to a mismatch between the fit envelope (±0.2 windows) and the verification envelope (±0.4 with thrust ∝ cmd·|cmd| and a 0.78 reverse/forward ratio; see OOF-01 and OOF-03 in the adversarial design).
3. The physically expected cross-axis mechanism is the rigid-body kinematic term, sway ≈ m·u·r / Y_v. For 30 kg, u = 0.77 and r = 0.023: about 0.53 N over a sway drag slope of about 17 N/(m/s), so ~0.03 m/s. This is real physics, not a bug. Real water adds a Munk/added-mass term with the same structure, so a *corrected* Fossen would add cross-coupling, not remove it.
4. Another candidate is truth-point lever arm. The yaw rows show 0.033 m/s surge at r = −0.12. That fits a body-velocity reference point offset from the COM (`CRANE_ROBOBOAT_INERTIA root_local=(0.05,1.21,1.12)`). This is a HYPOTHESIS.

### Falsifiable checks

- **F0 (Fossen hypothesis: already falsified; make it permanent).** Add a component census line to the run log listing every enabled hydro component. Make the runner reject a trial if FossenDynamics or GeneralDynamics is enabled under the contract. If anyone still believes Fossen matters for `reference`: under a fixed seed, run `reference` with FossenDynamics deleted from the scene and with it present. The truth traces should match bitwise, which shows Fossen is irrelevant here.
- **F1 (criterion and variance vs. fitter).** Run `reference` 5 times each with (a) the fitted decoupling and (b) decoupling forced to identity, under the same verification envelope. If both fail at a similar rate, the 0.05 m/s limit is below this plant's noise floor plus kinematic coupling. The criterion then needs a derived tolerance, e.g. a 95th-percentile of the nominal plant over N seeds plus a predicted m·u·r/Y_v allowance. If only (a) fails, the decoupling fit or its extrapolation is the defect, and it lives in mhseals_testing/hardware, not in CRANE hydro.
- **F2 (mechanism).** Regress sway on u·r per window from `truth.json`. Slope ≈ m/Y_v (about 1.7 s at this operating point) with R² > 0.8 means kinematic coupling explains the leakage. Also re-express the truth velocity at the COM: if the yaw-row surge leak disappears, it was a lever-arm artifact.

OPT-00 (nominal must pass before any MUST_FAIL counts) stays blocked until F1 is resolved. The fix belongs in the criterion or the fitter, not in hydrodynamics.

## 2. Options, with concrete failure modes

**(A) Leave production as is and document the defects.**
- The legacy main plant keeps 1.69× surge and 2.0× sway over-responsiveness (V1). With small M − M_A it can diverge (D1). It also gives a −450 N impulse on in-place reset (D2), so every main-branch Nav2 A→B→A reset comparison (RT-02) is contaminated.
- The heading-dependent damping (D5) and the −2ψ normal error (D6) make the plant depend on yaw. That breaks rotation equivariance (INV-02), so any heading-swept water experiment on main is uninterpretable. It undermines P6, the water/land sign comparison.
- Docs still claim "Fossen disabled" (`RoboBoatNavigationBaseline.md:159`). Someone will cite main-branch water runs as fidelity evidence.
- Acceptable only as an interim for main, and only once the docs are corrected and main's water runs are labelled "legacy plant, not fidelity evidence".

**(B) Flag-gated `corrected-v1` Fossen alongside legacy, A/B with the characterization cases.**
- The A/B is a null test as specified. The characterization cases run with Fossen disabled (§0), so both arms would be bitwise identical and the comparison would show "no effect".
- Running the cases with `--crane-roboboat-legacy-physics` to make Fossen active brings back 3500 N thrust, PhysX 4/15 damping and GeneralDynamics, so three drag models stay stacked (H1). A corrected Fossen inside that stack cannot be isolated, because any change can be absorbed by the other two dampers.
- It repairs a component that the newer plant has already replaced. That doubles the maintenance surface.

**(C) Make corrected the default and retune or record the controller.**
- This violates CONTRACTS ("validated controller defaults … not silently"). There is also no physical boat data to retune against, so retuning fits the controller to a second unvalidated plant.
- It moves every retained main Nav2 baseline onto a different plant without a migration record.
- It does not fix `reference`, which is a criterion/fitter problem (§1).

**(D) Recommended: freeze legacy, adopt the existing RoboBoatHull plant as the candidate, and add the missing physics to it as an opt-in term.**
1. **Freeze legacy Fossen and GeneralDynamics.** Legacy stays the default on main, unchanged. Mark them `DEFECTIVE_LEGACY` in docs and in run metadata. Add a guard: any run whose metadata claims fidelity must not have them enabled.
2. **Make RoboBoatHull the hydro candidate.** It is already flag-structured (contract default, legacy opt-out) and body-frame correct. Its angular damping projects world ω onto world body axes, which is valid. It is current-relative and is the plant the mhseals controllers are actually characterized on.
3. **What RoboBoatHull lacks is added mass and Munk/C_A coupling, not sign fixes.** Add an opt-in `--crane-hydro-profile=hull-am-v1`. It folds M_A into the effective mass and inertia (no ν̇ feedback, so no explicit-difference instability and nothing to reset), and adds the skew-symmetric C_A(ν)ν with the Munk yaw moment in body frame. A/B against `hull-v0`, the current default.
4. Fix `reference` separately via F1 (criterion and fitter), so the A/B has a passing nominal.

D keeps B's discipline (flag-gated, default unchanged, A/B) but applies it to the plant that is actually used, with one hydro model on the hull.

## 3. How controller defaults depend on the defective dynamics

- **Main** (`ROSOmniXCommand`): the linear/yaw feed-forward 0.21/0.24, proportional gains, and the 1.2 m/s top speed / 0.655 m coast in `RoboBoatNavigationBaseline.md` were all measured on the legacy stack. That stack has the V1 over-responsiveness, the D4 surge/sway coefficient swap (surge damped with Yv = 40, Yvv = 35), GeneralDynamics' constant "quadratic" term (static drag at low speed), and 3500 N thrust. Feed-forward ≈ drag/thrust at cruise, so it inherits all of these. Fixing Fossen alone changes the steady surge drag (Xu = 30 vs Yv = 40) and the transient (1.69×). Feed-forward would be off by roughly the drag ratio, and the gains tuned for a low-inertia plant would overshoot less but respond more slowly. H1 (mutual tuning hides the defects) is the expected outcome, not a risk.
- **Branch** (mhseals VelocityController, characterization `velocity_gain` ≈ 1.86/1.26/1.43, motion limits): fitted on RoboBoatHull. It does not depend on Fossen. It would shift under `hull-am-v1`, mostly through acceleration limits (+M_A). A controller record per hydro profile is required.

## 4. Promotion criteria (what "correct" can mean without boat data)

Without physical data, "correct" can only mean *analytically consistent*, never *calibrated*. Label: FIXTURE_VALIDATED at best, never PHYSICALLY_CALIBRATED.

Gate 1. Analytic invariants (EditMode pure math plus a short in-scene check). All must pass:
- 1-DOF step: a = F/(M + M_A) to 1e-4. No dependence on the previous step's ν̇.
- C_A skew-symmetric. νᵀC_A(ν)ν = 0, so Coriolis does no work.
- INV-01 energy: water-relative KE is non-increasing at zero command.
- INV-02 rotation equivariance at φ ∈ {30°, 90°, 217°}. This kills D5 and D6-class bugs.
- Munk moment sign: N = (Y_v̇ − X_u̇)·u·v. A body yawing toward broadside is destabilizing.
- INV-08 timestep invariance at dt 0.02 vs 0.01, within 1 %.
- Reset A→B→A hash-identical (RT-02) on the Blastoise articulation, not the toy.
- RT-06: flag absent means physics hash equals the baseline worker. MUT-12 must be killed.

Gate 2. Behavioural plausibility against published priors, with priors declared before the run:
- T200 bollard thrust matches the Blue Robotics datasheet.
- The M_A/M ratio sits in a catamaran literature band. Sway M_A is much larger than surge M_A.
- Turning-circle sideslip is bounded by the drag ratio (INV-06).
- Top speed of 2.5 m/s is kept, since steady drag is unchanged by added mass.

Gate 3. The nominal characterization passes with the candidate: `reference` green over ≥ 3 seeds, after F1 has fixed the criterion. All MUST_FAIL cases (`weak_rr`, `angled_fr`, `combined_current`) still fail. The fault detection signal must survive the profile change.

Gate 4. Promotion to the **default** needs physical evidence: ≥ 1 held-out real session (a different site or session, per PREREGISTRATION) of surge/sway/yaw steps and coasts. On it, the candidate must beat `hull-v0` on prediction error with a CI that excludes 0. Until then the default does not change. Gates 1-3 qualify the profile only as an "analytically consistent alternative".

## 5. Land/campus fixes: do they need gating?

| Fix | Consumers found | Gate? |
|---|---|---|
| IMU gyro sign (`CraneCampusSensorSuite.cs:27-29`, `Imu.cs:35`) | `/campus/imu`: **no consumer in `/home/lunarz/astro_dock/src`** (mhseals_nav/testing/hardware and ros_tcp_endpoint grepped; the EKFs fuse `/imu/raw`). In crane_ml only `Tools/Campus/capture_ros.py` (records it) and `validate_run.py:25` (cadence only). Mirrors in `crane_explain/packages/crane_ml` and the `crane_explain/worktrees/*` copies are clones of the same code, not consumers. Boat `/imu/raw` on the contract path already uses the correct `CraneROSNavigationState.ToRosAngularVector` (`CraneRoboBoatSensorContract.cs:160`). `Imu.cs` is disabled in Roboboat. | **No behavioural gate needed.** It is a convention bug with no compensating consumer. Fix it unconditionally, with a sign test and a `validate_run.py` IMU/odom sign check. Record `imuAngularConvention=axial-v2` in run metadata so old captures are not compared against new ones. Retained `/campus/imu` captures should be marked sign-inverted. |
| Profile fail-closed without `--crane-campus-model` (D9) | Replay headers that hash the profile | **No gate.** This is a correctness and provenance fix that turns silent mislabelling into an error. Any existing run that would now throw was already mislabelled. |
| Wheel-scale branch sign (D10) | The default force branch with `leftWheelScale ≠ 1`. Fitted profiles are medians of supplied observations, so none is physical. | **Gate by profile schema version**, not by a CLI flag. Bump the profile schema and refuse old-schema profiles with scale ≠ 1, since they encode the wrong sign for the force branch. Defaults (1.0/1.0) are unaffected. |
| Double lag (D11) | `motorLagSeconds` from profiles, `--crane-campus-motor-lag`; CAL-02 parity runs | **Gate it.** It changes the default campus plant at τ = 0.15 (63 % rise at 0.37 s → 0.15 s), so CAL-02 and campus Nav2 baselines shift. Use a `--crane-campus-actuator-model=single-lag-v1` flag. Keep legacy as default until a CAL re-baseline is recorded. Then flip it with an explicit DECISIONS.md entry. |

## 6. Uncommitted user work to avoid (git status @ `976ff1e`)

Modified (do not edit without coordinating):
- `CraneAuthoritativeReplay.cs`
- `Physics/Land/CraneCampusRobotModel.cs`: the user is actively wiring the profile here (D9/D10/D11 all live in this file)
- `Physics/Land/CraneIndustrialCampus.cs`
- `Sensors/Lidar/Lidar2D.cs`
- `Sensors/Vision/RosCameraAsync.cs`, `RosDepthCameraAsync.cs`
- `Utils/Performance/CraneObservationJournal.cs`
- `Utils/ROS/ROSPublisher.cs`
- `Tests/Editor/RoboBoatEditorTests.asmdef`
- `README.md`, `THIRD_PARTY_NOTICES.md`
- `Tools/Campus/analyze_calibration.py`

Untracked user work:
- `CraneCampusCalibrationProfile.cs`
- `SensorAcquisitionTimingTests.cs`
- `Tools/Campus/{calibration_profile,fit_calibration_profile,heldout_calibration,…}.py`
- `Tools/Fidelity/`
- the `Docs/CRANE*`, `Docs/CalibrationProfileIntegration.md` and `Docs/PhysicalCalibrationDataContract.md` docs
- `LICENSE`

Clean (safe targets):
- `Physics/Water/Dynamics/FossenDynamics.cs`, `GeneralDynamics.cs`
- `Sensors/Nav/CraneCampusSensorSuite.cs`, `Sensors/Nav/Imu.cs`
- `Tools/Campus/validate_run.py`

The RoboBoatHull work lives on unmerged branches (`roboboat-thruster-pwm` worktree at `/home/lunarz/crane_ml-roboboat-pwm`, characterization worktree at `/home/lunarz/worktrees/crane-characterization`). Hydro work should go there or in a new branch off them, not on main.

New EditMode tests need an asmdef reference. `RoboBoatEditorTests.asmdef` is user-modified, so put new tests in a **new** test asmdef (or wait for the user to commit).

## 7. Ordered steps (lowest risk first)

1. **Docs and labels only:**
   - Correct `RoboBoatNavigationBaseline.md:159` ("Fossen disabled" is false on main).
   - Record in DECISIONS.md that main water runs use the legacy plant and the characterization runs use RoboBoatHull.
   - Add the runner-side component census (F0).
   - Touches no user files.
2. **Run F1 and F2 offline/live on `reference`.** Decide criterion versus fitter, and unblock OPT-00. The change lands in mhseals_testing (CRANE-owned worktree) or in the criterion definition, not in hydro.
3. **IMU gyro sign** in `CraneCampusSensorSuite.cs` and `Imu.cs` (both clean files):
   - add a sign test in a new test asmdef
   - add the IMU/odom sign check in `validate_run.py`
   - add `imuAngularConvention` to the run metadata
4. **Pure-math hydro fixture (PY plus new EditMode asmdef):**
   - the Gate-1 invariants for `hull-v0`, which should pass already
   - a reference implementation of `hull-am-v1`
   - the legacy Fossen as a mutant (must go red on 1-DOF, skew and INV-02)

   No scene changes.
5. **`--crane-hydro-profile=hull-am-v1` on the RoboBoatHull branch,** default `hull-v0`:
   - write the profile ID and parameters into run metadata and the replay header
   - RT-06 hash parity with the flag absent
6. **A/B characterization:** `hull-v0` vs `hull-am-v1`, ≥ 3 seeds each, with the fixed criterion (Gate 3). Record controller fits per profile.
7. **Campus D9/D10/D11, after the user commits** `CraneCampusRobotModel.cs` and `CraneIndustrialCampus.cs`. Then: fail-closed, schema-versioned wheel scale, and the flag-gated single lag, with CAL re-baseline before flipping the lag default.
8. **Defer:**
   - Do not repair legacy Fossen or GeneralDynamics in place.
   - Retire them when RoboBoatPhysics merges to main, through an explicit DECISIONS.md entry carrying the main Nav2 re-baseline.
   - Gate-4 promotion waits for physical boat data.

## 8. Risks

- **Mis-targeted effort.** Building corrected Fossen (option B) yields a null A/B and false confidence that the "hydro is fine". Mitigation: F0 census.
- **Criterion tuning as p-hacking.** Loosening 0.05 m/s until `reference` passes is not acceptable. The tolerance must be derived from N-seed nominal statistics plus the analytic kinematic allowance, and frozen in PREREGISTRATION before any comparative run.
- **Branch divergence.** Main (legacy) and the branches (RoboBoatHull) answer different questions. Water evidence must carry the plant ID or results get mixed. P6 (water vs land sign) must use one declared plant.
- **Added mass changes the fault signal.** `hull-am-v1` may shrink or grow the `weak_rr`/`angled_fr` cross-axis signatures. Gate 3 requires MUST_FAIL cases to stay red.
- **"Analytically correct" is not "real".** Without boat data, `hull-am-v1` M_A values are literature priors, which is the same epistemic status as today's drag coefficients. Never label it PHYSICALLY_CALIBRATED.
- **User-work collision.** D9-D11 sit in files the user is editing now. Sequence those changes after the user commits.
- **IMU sign flip breaks historical comparisons.** Old `/campus/imu` captures become sign-inverted relative to new ones. Mitigation: the metadata convention tag.
