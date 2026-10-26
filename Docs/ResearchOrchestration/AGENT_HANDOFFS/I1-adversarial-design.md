# I1 — Adversarial intervention matrix (design only)

Author: I1 experiment & adversarial-testing designer. Date: 2026-10-20.
Scope: read-only survey and design. Nothing was created except this file. No git writes, no ROS, no hardware.
Evidence labels follow CONTRACTS.md.

## 1. Existing precedents

| # | Repo @ commit | Path | Injects | Checks | Status (this session) |
|---|---|---|---|---|---|
| P1 | crane-characterization `test/characterization-faults` @ `dccb3e6` (one commit on top of `bf82744`; `main..HEAD` = 9 commits / 31 files because main is behind the PWM line) | `Assets/Scripts/Controllers/RoboBoatCalibrationFaults.cs`, `Assets/Scripts/Actuators/Motors/Thruster.cs` (`SetThrustCurve(..., yawDegrees)`) | You opt in with `--crane-calibration-faults <json>`. Per-thruster `strengths` [0.01,4], per-thruster mount yaw `angles` [-45,45] deg, `drag` multipliers [1,20] on the quadratic RoboBoatHull surge/sway/yaw drag (velocity-dependent, relative to the current), and `dragOffset` (bow/port lever arm, [-1,1] m) that creates drag-induced yaw. It logs `CRANE_CALIBRATION_FAULTS`. | Range validation only. There is no EditMode test of the fault component. | IMPLEMENTED. No unit test. |
| P2 | mhseals_testing-characterization `test/characterization-faults` @ `a645657` (`0a0c070..a645657`, 7 commits, 10 files) | `test/run_crane_characterization.py`, `test/crane_characterization_probe.py` | CASES: `reference`, `weak_rr` (RR 0.6), `angled_fr` (FR 15 deg), `hull_drag` (3/3/2, offset 0.1/0.25), `combined_current` (strengths, angles, drag, offset, plus current 0.2 m/s at 30 deg). These run through the real EKF, VelocityController and PWM in Unity, on isolated domains (42/142 excluded). | `status==MEASURED`, all `acceptance` checks (velocity RMS, e.g. `yaw_error_at_most_0_03_rad_s`) and all `manual_acceptance` checks (`translation_cross_axis_rms_at_most_0_05_m_s`, `yaw_cross_axis_rms_at_most_0_03_rad_s`). The pulse procedure requires `PROVISIONAL_VALIDATED`. | Live. I did not rerun it. Existing results are in `crane_ml/PerformanceResults/calibration-*-20261019/summary.json`: **every case `passed=False`.** `reference` fails the manual translation cross-axis check. `angled_fr`/`weak_rr` fail both manual cross-axis checks. `combined_current` fails `yaw_error` and the manual checks. Pulse `weak_rr` is `REFUSED` ("predicted directional residual remains too large"). |
| P3 | same @ `a645657` | `test/test_characterization.py` | Synthetic GAIN/BIAS plant. World current (0.1, 0.15) with 64 headings. A single heading. Squared thrust through `THRUSTER_MIXER`. Mixed-only bag. Under-excited bag. | Current/bias separation abs 1e-6. With one heading it returns `separated: False` and folds the current into the body bias. Squared thrust gives decoupling = I (1e-8). Mixed-only bag raises `single-axis`. | 72 passed, 1 skipped (together with P4 and the balance tests). `python -I -m pytest`, pythonpath set to the testing and hardware worktrees. |
| P4 | same @ `a645657` | `test/test_thrust_correction.py` | Saturation recorded as linear. Sign-even yaw bias. Singular response. Noisy main axis. Repeats with high uncertainty. | Refuses the fit or raises. | Passes (counted in P3). |
| P5 | mhseals_hardware-characterization, detached @ `26c59f7` (on main, no extra commits) | `test/test_velocity_law.py`, `test/test_identification.py`, `test/test_mixer.py` | World current through a 180 deg turn. A 15 % weaker plant. Saturation windup. Stale odometry. Swapped wiring. A backwards thruster. | The drift stays world-frame (abs 0.02). Model error is not taken for current (< 0.01 m/s). Integral stays < 0.1 at saturation. Odometry stale after 1.0 s gives zero effort. Wiring and reversal are identified. | 32 passed. |
| P6 | mhseals_nav-characterization, detached @ `04a9f4a` | — | No characterization-specific diff. | — | N/A |
| P7 | crane_ml main @ `976ff1e` + **uncommitted user work** | `Assets/Tests/Editor/SensorAcquisitionTimingTests.cs` (untracked) | An old camera callback arriving after a new episode. A delayed lidar journal across reset. | The header uses acquisition time. Old-episode metadata is rejected. The journal clears on reset. | EditMode, NOT_RUN. |
| P8 | crane_ml main (tracked + untracked) | `Tools/Campus/test_{heldout_calibration,dynamic_evidence,sensor_error_evidence,calibration_profile,drive_calibration_gates,ingest_public_bag}.py`, `Tools/Fidelity/test_physical_ros_capture.py` | Factor leakage, hash tamper, non-monotonic stamps, missing clock uncertainty, stale-sample interpolation, unstamped command, wrong noise scale. | Fails closed. | 92 passed offline (Tools/Campus + Tools/Fidelity + one Performance test). |
| P9 | crane-characterization @ `dccb3e6` | `Tools/Performance/test_roboboat_command_response_fixture.py`; `Assets/Tests/Editor/RoboBoatThrusterPwmTests.cs` | Full-surge apply/release. Mixer through PWM. Wiring. | — | Python: 1 passed. EditMode: NOT_RUN. |

**Gaps:**
- There is no actuator-lag or timestamp fault injection. Only `T200SpinUp` is fixed at 0.1 s.
- There are no out-of-family tests.
- There are no dynamics-invariant or mutation tests.
- No test checks that physics is unchanged when the fault flag is absent.
- Most important: **the nominal control (`reference`) fails the live acceptance**. That makes every live MUST_FAIL result uninterpretable. A failure cannot be told apart from a broken criterion.

No secrets were seen or copied. The runner passes only the ROS image name and paths.

## 2. Conventions

- **Reference fixture plant (PY tier):** 3-DOF surge/sway/yaw, explicit fixed dt = 0.02 s. Constants are cited from `RoboBoatPhysics.cs` @ `dccb3e6`, not altered:
  - mass 30 kg
  - T200 thrust 51.5 N forward, 40.2 N reverse; thrust ∝ cmd·|cmd|; spin-up τ = 0.1 s
  - hull drag (linear, quadratic): Surge (10, 19.4), Sway (15, 41.7), Yaw (10, 19.3)
  - `THRUSTER_MIXER` from mhseals_hardware @ `26c59f7`
  - Izz and the thruster lever arms are fixture-only values, labelled `FIXTURE_ONLY`. They are not boat claims.
- **Outcome semantics:** a test is green when the *expected* outcome occurs.
  - MUST_FAIL: the validator returns FAIL.
  - MUST_ABSTAIN: the estimator returns `UNIDENTIFIABLE` or `separated: False`, and emits no point estimate. A confident wrong estimate is red.
- **Noise:** unless stated, velocity noise σ = 0.02 m/s and yaw-rate noise σ = 0.005 rad/s. Seeds are fixed: 20 seeds per case. Criteria apply to the worst seed unless marked "median".
- **Tiers:**
  - PY: pure-python numerical fixture. Runnable now with numpy and pytest, no ROS.
  - EM: Unity EditMode.
  - LIVE: Unity worker plus the isolated ROS stack (P2 harness).

## 3. Matrix

### 3.1 Identifiability (ID)

| ID | Intervention | Data / excitation | Estimator | Pass criterion | Expected | Tier |
|---|---|---|---|---|---|---|
| ID-01 | FR thruster sign reversed | Each output pulsed alone at ±0.3 and ±0.6, 3 s holds | Per-output LS of body accel direction | All 4 signs correct. Column cosine to truth ≥ 0.99 | PASS | PY (EM analog: `test_identification`) |
| ID-02 | Mount yaw error FR +15 deg (= `angled_fr`), RR strength 0.6 (= `weak_rr`) | Per-thruster independent excitation as in ID-01 | Per-thruster force direction and magnitude from initial accel, known mass | \|angle err\| ≤ 1.5 deg, strength err ≤ 3 % | PASS | PY then LIVE |
| ID-03 | Same faults as ID-02 | **Surge-only** mixed commands (collinear) | Joint 8-parameter fit (4 strengths, 4 angles) | Fisher information rank < 8, or condition number > 1e6 → `UNIDENTIFIABLE` | MUST_ABSTAIN | PY |
| ID-04 | Body bias (0.02, -0.01, 0) plus world current (0.10, 0.15) m/s | Holds at 64 headings uniform over 360 deg, with noise | `characterize_samples` drift split (CRANE reimplementation, or read-only import) | \|current err\| ≤ 0.01 m/s, \|bias err\| ≤ 0.01 (median), ≤ 0.02 (worst) | PASS | PY |
| ID-05 | Same as ID-04 | Heading span ≤ 20 deg (and a 1-heading case) | Same | `separated: False`. Any `separated: True` with err > 0.02 is red | MUST_ABSTAIN | PY |
| ID-06 | Current ramps 0 → 0.2 m/s during the heading sweep (as with `Disturbance` ramp-in) | 64 headings over 120 s | Same plus residual whiteness (Ljung–Box p < 0.01) | Flags misspecification, or abstains. It must not report a constant current with err > 0.03 | MUST_ABSTAIN | PY |
| ID-07 | **Actuator dead time 0.15 s vs observation stamp lag 0.15 s** (two plants) | Step and chirp commands; command and odom stamps only | Latency fit plus split attribution | Total latency within ±10 ms for both plants. The split is reported as `UNIDENTIFIABLE`. The two plants' logs are equal to 1e-9 after stamp alignment | MUST_ABSTAIN (split) | PY |
| ID-08 | Same as ID-07 | Plus an independent truth clock (Unity authoritative pose stamps) | Same | Split recovered within ±10 ms each | PASS | PY then LIVE |
| ID-09 | **Weak yaw excitation (\|r\| ≤ 0.02 rad/s) vs lateral drag offset** (`dragOffset` port 0.25 m) and yaw drag ×2 | Sway holds, tiny yaw | Joint fit of yaw-drag scale and lever arm | Relative std > 50 %, or correlation > 0.95 → abstain | MUST_ABSTAIN | PY |
| ID-10 | Same as ID-09 | Yaw chirp ±0.4 rad/s plus sway holds at 3 speeds | Same | Lever arm ±0.03 m, yaw scale ±10 % | PASS | PY |
| ID-11 | Linear vs quadratic drag | Single surge speed | 2-coefficient drag fit | Abstains (1 equation, 2 unknowns) | MUST_ABSTAIN | PY |
| ID-12 | Global thrust scale 0.8 (voltage sag) vs drag ×1.25 | Steady holds only | Thrust/drag fit | Steady-only data → `UNIDENTIFIABLE` (only the ratio). Adding a transient with known mass gives a PASS: each within ±5 % | MUST_ABSTAIN / PASS | PY |

### 3.2 Out-of-family (OOF)

| ID | Intervention | Data | Estimator | Pass criterion | Expected | Tier |
|---|---|---|---|---|---|---|
| OOF-01 | True quadratic hull drag, thrust ∝ cmd² | Fit on surge 0.2–0.6 m/s | Constant velocity gain K (current mhseals model form) | In-range held-out err ≤ 5 % (PASS). The validator on held-out 1.5–2.3 m/s must report err > 15 % and FAIL | MUST_FAIL (held-out) | PY |
| OOF-02 | Current 0.2 m/s | Fit at heading 0 only, test at heading 180 | Body-bias model (ID-05 fallback) | Held-out velocity err > 0.3 m/s is detected → FAIL | MUST_FAIL | PY |
| OOF-03 | Reverse factor 40.2/51.5 | Forward-only data | Symmetric gain | Predicted reverse err > 20 % → FAIL. No reverse limit is granted (P3 analog) | MUST_FAIL | PY |
| OOF-04 | Any | Query outside the declared fitted envelope (speed, heading, sign) | Model `valid_envelope` | Returns ABSTAIN, not a value | MUST_ABSTAIN | PY |
| OOF-05 | `combined_current` gains tuned at 0.2 m/s / 30 deg | Held-out 0.3 m/s / 120 deg | VelocityController | yaw_error ≤ 0.03 rad/s or a reported FAIL. Expect FAIL until shown otherwise | MUST_FAIL (provisional) | LIVE |

### 3.3 Dynamics invariants (INV) and mutation kills (MUT)

| ID | Property | Excitation | Pass criterion | Expected | Tier |
|---|---|---|---|---|---|
| INV-01 | Energy decay at zero command, no current or wind | Start at 2 m/s, 0.5 rad/s | KE non-increasing every step (+1e-9 J slack). KE(10 s) ≤ 1 % of KE0. With current: water-relative KE decays | PASS | PY then EM (RoboBoatHull.Drag) |
| INV-02 | Rotation equivariance | Rotate the initial state and the current by φ ∈ {30, 90, 217} deg | Rotated trajectory within 1e-6 m (PY) / 1e-3 m (EM, float) over 20 s | PASS | PY, EM |
| INV-03 | Saturation | Command 2× max; mixed combined command | Outputs ∈ [-1, 1]. Trajectory at 2× equals 1× (1e-9). Desaturated direction cosine ≥ 0.999. Thrust ≤ 51.5 / 40.2 N | PASS | PY, EM (`test_combined_command_is_desaturated` analog) |
| INV-04 | Reverse asymmetry | Full forward vs full reverse | Thrust ratio 0.781 ± 1 %. Reverse steady speed < forward | PASS | PY, EM |
| INV-05 | Port/starboard mirror | Mirrored command | Nominal mirrored within 1e-6. With `angled_fr`, mirror residual ≥ 0.05 m/s (must be detected) | PASS / MUST_FAIL | PY |
| INV-06 | Nonholonomic slip bound | Boat: constant-radius turn, slip angle ≤ bound from the sway/surge drag ratio. Campus diff-drive: body lateral velocity ≤ 0.02 m/s without slip; with friction lowered, lateral > 0.05 m/s is detected | As stated | PASS / MUST_FAIL (injected) | PY (unicycle fixture), EM/LIVE for CraneCampusRobotModel |
| INV-07 | Delay → stopping-distance bound | v0 = 1.5 m/s, stop command, dead time d ∈ {0, 50, 100, 200} ms | s(d) − s(0) ∈ v0·d·[0.95, 1.05] and monotonic in d | PASS | PY, LIVE |
| INV-08 | Timestep invariance | dt 0.02 vs 0.01 | Position err ≤ 1 % of path length over 10 s | PASS | PY, EM |
| INV-09 | Determinism | Same seed twice | Bitwise trajectory hash equal | PASS | PY, EM |
| INV-10 | Thrust non-superposition | Mixed vs summed axis holds | Gives no spurious decoupling (|C − I| ≤ 1e-8; P3 analog) | PASS | PY |

**Mutants:** the suite must go red on each one. Mutation score = 100 % is required.

| Mutant | Description | Killed by |
|---|---|---|
| MUT-01 | Drag sign flipped (energy injection) | INV-01 |
| MUT-02 | Anisotropic drag applied in the world frame instead of the body frame | INV-02 |
| MUT-03 | Output clamp removed | INV-03 |
| MUT-04 | `backK` = 1 | INV-04 |
| MUT-05 | Mixer rows FL↔FR swapped | INV-05, ID-01 |
| MUT-06 | Delay buffer skipped, or applied twice | INV-07 |
| MUT-07 | Force multiplied by dt twice | INV-08 |
| MUT-08 | Unseeded noise | INV-09 |
| MUT-09 | Estimator always returns `separated: True` | ID-05 |
| MUT-10 | Estimator fits saturated data linearly | P4 analog |
| MUT-11 | Latency estimator always splits 50/50 | ID-07 |
| MUT-12 | Fault component active without the flag | RT-06 |

### 3.4 Perception and timing (PT)

| ID | Intervention | Check | Pass criterion | Expected | Tier |
|---|---|---|---|---|---|
| PT-01 | Odom stamp +50 ms, constant | Latency estimate against the truth clock | ±5 ms | PASS | PY then LIVE |
| PT-02 | Non-monotonic or duplicate stamps | Ingest reader | Rejected (P8 analog) | MUST_FAIL | PY |
| PT-03 | Sensor frozen (held last value, fresh stamps) | Staleness detector | Flagged within 2 periods | MUST_FAIL (stream) | PY |
| PT-04 | 20 % bursty drop, gaps > 0.25 s | Truth windows | No interpolation across gaps. Window abstains (P2 rule: ≥ 30 samples) | MUST_ABSTAIN | PY |
| PT-05 | Gyro bias 0.01 rad/s | Bias vs yaw-drag split | Without a zero-command rest window → abstain. With rest → ±0.002 rad/s | MUST_ABSTAIN / PASS | PY |
| PT-06 | Clock rate skew 100 ppm over 600 s | Drift detector | 60 ms ± 6 ms detected | PASS | PY |
| PT-07 | LiDAR range ×1.02; detection frame rotated 2 deg | Against course-truth buoys | Range err and bearing err detected above 1 % and 1 deg | MUST_FAIL | EM/LIVE |
| PT-08 | Header stamped with receipt time (mutant) | `ExplicitHeaderUsesAcquisitionTime` | Red under the mutant | MUST_FAIL | EM (P7) |
| PT-09 | Old-episode camera callback | `OldCameraCallbackCannotConsumeNewEpisodeMetadata` | Rejected | PASS | EM (P7) |

### 3.5 Runtime (RT)

| ID | Intervention | Pass criterion | Expected | Tier |
|---|---|---|---|---|
| RT-01 | Record, then replay the episode | Observation and command stream hash identical (tolerance 0) | PASS | PY (journal), EM (CraneAuthoritativeReplay) |
| RT-02 | A→B→A: run config A, then B (faults on), then A | A2 hash == A1. No leak of spool, integrator, drift observer or fault component | PASS | PY (VelocityLaw/plant reset), LIVE |
| RT-03 | Command older than the freshness bound (1.0 s per P5), and a stamp more than 50 ms in the future | Zero effort within 1 tick, counted | MUST_FAIL (command rejected) | PY, EM |
| RT-04 | Packet from episode N−1 after reset to N | Dropped, counter +1, never applied | MUST_FAIL (packet rejected) | PY, EM |
| RT-05 | Burst of 10× queue capacity | Queue stays bounded. Oldest dropped. Added latency ≤ 1 period | PASS | PY |
| RT-06 | Fault flag absent | Physics hash == baseline worker. Out-of-range fault JSON throws | PASS / MUST_FAIL | EM |
| RT-07 | Run without the `CRANE_CALIBRATION_FAULTS` log | Trial marked invalid (P2 already does this) | MUST_FAIL | PY (log parser) |

### 3.6 Optimizer validity (OPT)

| ID | Intervention | Pass criterion | Expected | Tier |
|---|---|---|---|---|
| OPT-00 | **Nominal control:** the `reference` case must PASS before any MUST_FAIL is counted | Live acceptance is green on `reference` over 3 seeds | PASS (currently red, see P2) | LIVE |
| OPT-01 | Loophole policies: idle, spin in place, oscillate at a gate to farm crossings, exit-and-re-enter, exploit reset | Each scores < the minimum honest-baseline score | MUST_FAIL (loophole) | PY (scorer on synthetic traces) |
| OPT-02 | Invalid trials: stale observations, truncated log, fault not confirmed, missing seed, source-hash mismatch | Excluded from the objective, never scored as low cost | MUST_FAIL (trial) | PY |
| OPT-03 | Tuning on held-out cases | Rejected (P8 factor-leakage analog) | MUST_FAIL | PY |
| OPT-04 | Best-of-N selection | Rerun on 5 fresh seeds; the 95 % CI contains the reported score | PASS | PY then LIVE |

## 4. Proposed layout (not created)

```
crane_ml/Tools/Adversarial/
  plant3dof.py          # reference plant; constants cite RoboBoatPhysics.cs@dccb3e6; FIXTURE_ONLY Izz/arms
  faults.py             # dataclasses superset of RoboBoatCalibrationFaults JSON (+lag, stamp_offset, gyro_bias, current_ramp)
  excitation.py         # per-thruster pulses, heading sweeps, chirps, steps
  estimators.py         # CRANE-owned; optional read-only adapter to mhseals_testing.analysis at a pinned rev
  invariants.py
  mutants.py            # context managers that patch plant/estimator
  matrix.json           # registry: id, intervention, excitation, estimator, criterion, tol, expected, tier
  run_matrix.py         # emits evidence JSON with CONTRACTS labels, seeds, source hashes
  test_identifiability.py  test_out_of_family.py  test_invariants.py
  test_mutation_kill.py    test_timing_faults.py  test_runtime_contracts.py  test_optimizer_validity.py
Assets/Tests/Editor/AdversarialPhysicsInvariantTests.cs   # INV-01..04,08,09, RT-06 (EM)
```

The EM tests need `RoboBoatEditorTests.asmdef`. That file currently has **uncommitted user edits**, so coordinate before touching it.

## 5. Priority

1. PY now: ID-01..07, 09..12, OOF-01..04, INV-01..05/07..10, all MUT, PT-01..06, RT-01..05/07, OPT-01..03.
2. EM: INV parity on RoboBoatHull, RT-06, PT-07..09.
3. LIVE: OPT-00 first, then ID-02/08, OOF-05, RT-02, INV-07.
