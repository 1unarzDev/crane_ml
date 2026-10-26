# I1 independent verification: Tools/Adversarial and the C# PhysicsInvariants tests

Verifier: independent agent; did not write any of the code under review. 2026-10-21.
Repo writes: this file only. All experiments ran on copies under `/tmp/crane-verify-adv/`.
Scripts: `/tmp/crane-verify-adv/scripts/` (`abstain_counter.py`, `tolerances.py`, `chi2gate.py`,
`indep_noise_and_delay.py`, `mutants.py`, `r17_repro.py`, `r17_ablation*.py`). R17 output:
`/tmp/crane-verify-adv/r17_results.txt`.

Baseline reproduced: `python -I -m pytest Tools/Adversarial -q` gives 69 passed, 1 xfailed, 32 s.

## Verdicts

| Area | Verdict |
|---|---|
| Tools/Adversarial, abstention (ID/OOF/PT) | ACCEPT-WITH-FIXES |
| Tools/Adversarial, mutation suite | ACCEPT-WITH-FIXES (4 of my 7 new mutants survive) |
| Tools/Adversarial, flagged tolerances | ACCEPT (none was tuned to pass; two are far too loose) |
| Tools/Adversarial, RT-06 flag-off | ACCEPT-WITH-FIXES (the flag-off half is tautological) |
| C# PhysicsInvariants @ 81b3137 (static) | ACCEPT-WITH-FIXES (one KnownDefect test cannot fail on the defect) |

## 1. MUST_ABSTAIN tests: right reason? Identifiable counter-cases

None of the MUST_ABSTAIN tests asserts the abstain `reason`. They only check `status == ABSTAIN`.
Here are the observed reasons, and what each gate does on identifiable data.

| ID | Observed reason | Identifiable counter-case | Result |
|---|---|---|---|
| ID-03 surge-only | `rank_deficient_fisher`, rank 3 (structural) | 4 experiments, one per output, at +-0.6 (8 pulses) | OK, angles within 0.13 deg. Not a count gate. With +0.6 only it abstains `sign_pattern_ambiguous` with best = second chi2: a real alias, because sign -1 equals angle+180 deg with strength x0.781. It is reachable only because angles are bounded to +-pi, not to the physical +-45 deg. |
| ID-05 0 deg / 1 heading | `rank_deficient_heading_diversity` (structural) | see below | correct |
| ID-05 20 deg / 64 headings | `insufficient_heading_diversity`, predicted std 0.0245 > 0.01 | same 20 deg span with 640 samples, or sigma 0.001 | **Does not abstain** (std 0.0078 / 0.0012; errors <= 0.015). The gate is noise and count based, not diversity based, so the label is wrong. 3 headings at 0/120/240 (well spread) **do abstain** (std 0.0115). |
| ID-05 (bug) | n/a | **4 or 5 headings evenly spread over 360 deg** | **ZeroDivisionError crash** in `ljung_box_q` (lags = 5 >= n). An identifiable input crashes the estimator. |
| ID-06 ramp | `residual_not_white` (Q 172-251 against crit 29.8) | **same ramp, headings visited in random order** | **FALSE IDENTIFY: 20/20 seeds return OK with current error 0.088-0.092 (tolerance 0.03).** The Ljung-Box gate only sees the ramp because sequential heading order turns it into autocorrelation. Residual chi2/dof is 4.6-5.6, so a goodness-of-fit gate would catch it, but none exists. Undetected ramps up to ~0.05 m/s also bias the estimate. |
| ID-07 split | hardcoded: `truth_log is None` -> `UNIDENTIFIABLE` | n/a | The abstention is unconditional (true by construction). The real evidence is check (1), bit-equal logs, which holds only from rest with zero initial command. A coasting start would make the split identifiable and it would still say UNIDENTIFIABLE. That is safe-side, but not a test of the estimator. |
| ID-09 | `yaw_drag_scale_rel_std` (0.80), not the lever | 6 open-loop rows (same count) | OK, levers 0.099/0.25, k_yaw 2.00. The abstention is all-or-nothing: yaw-controlled holds with both Fx and Fy identify both levers but still abstain because k_yaw (r ~ 0) does not. |
| ID-11 single speed | `unidentifiable_drag_terms`, smin/noise 0.23 | 2 speeds x 3 (n = 6), including 55/62 N | OK, coefficients within 20 % / 7 %. Genuine structural abstention. |
| ID-12 steady | `only_ratio_identifiable`, smin/noise 0.64 | transient rows | OK (ratio >= 30). Genuine. |
| PT-05 no_rest | `no_rest_window` (`sum(mask) < 30`) | genuine 29-sample rest window | abstains (count threshold). The rest mask is also taken on trust: a 40-sample "rest" mask placed during a turn returns **OK, bias 0.365 vs truth 0.01 (FALSE IDENTIFY)**. |

Mixer gate side finding: the chi2 at the TRUE parameters is up to 827 on 48 residuals. Cause:
`make_observations(log, cfg, seed)` uses the same seed for all 16 pulses of one fit, so their
noise is identical and common-mode. With independent per-pulse seeds, ID-02 still passes
(worst 0.70 deg, 0.5 %), but chi2 stays at 115-493 for dof 40. `measure_pulse_wrench` sigma
leaves out the noisy drag/Coriolis integral, so the "chi2" is not chi2-scaled. `estimate_mixer`
has no goodness-of-fit gate, so a confidently wrong model can return OK.

## 2. Mutation kill: 7 new mutants (copies under /tmp/crane-verify-adv/mutants/)

| Mutant | Killed by run_suite? | Killed by any pytest test? |
|---|---|---|
| N1 lever-arm sign in `column()` (plant and allocator consistent) | yes, ID-01 only (estimator has its own formula). INV-10 is blind because allocator and plant share `column()`. | ID-01, ID-02 |
| **N2 spool alpha uses fixed `DT` (dt-dependent lag)** | **SURVIVES** (INV-08 err 0.071 m against tol 0.119 m; baseline err is 0.018 m) | **SURVIVES** |
| **N3 thruster lag as a pure 5-tick delay** | **SURVIVES** | only by the unit test `test_thrust_report_...` (one-tick applied force), which is not in the suite |
| **N4 dead-time ticks computed with fixed `DT`** | **SURVIVES** (INV-07 runs at one dt only) | **SURVIVES** |
| N5 Coriolis sign flip (still skew, so workless) | ID-01 only, min_cos 0.984 against 0.99 (thin margin, a side effect). INV-13 is blind by construction. | ID-01, ID-02 |
| **N6 `build_vessel` ignores `enabled`** | **SURVIVES**: `build_vessel` is never called by any test (Harness.make duplicates it) | **SURVIVES** |
| N7 quadratic drag 8 % low | INV-12 (5 % tol); a 4 % error would survive | INV-12 |

The 17 registered mutant kills are genuine: the baseline passes and each mutant fails a named
killer. But the suite has no dt-convergence check strong enough for O(dt) lag errors, no lag-shape
check, no multi-dt fault-layer check, and no signed Coriolis check.

## 3. Flagged tolerances

| Tolerance | Evidence | Judgement |
|---|---|---|
| Whiteness p < 0.001 | On the 20 fixed seeds max Q = 21.0 < crit(0.01) = 23.2, so 0.01 would also have passed: **not tuned to pass**. The family-wise argument (1-0.99^20 = 18 %) is arithmetically right, but Monte Carlo (20 000 draws) puts the real false-alarm rate at 0.018 (nominal 0.01) and 0.003 (nominal 0.001). Ljung-Box on regression residuals plus Wilson-Hilferty is anti-conservative by ~2-3x. Power margin on ID-06 is ~6x. | Principled, but state the measured alpha and correct dof for the 4 fitted parameters. |
| k = 3 in `min_sv_vs_noise` | Abstain cases have smin/noise 0.23 and 0.64; identify cases have >= 30 and >= 66. Any k in [0.7, 29] gives identical outcomes. | Not tuned; insensitive. |
| `second < 2*best + 9` | second/best = 800 to 58 000 in ID-01/02. The "+9" assumes calibrated chi2, which this is not (section 1). | Not tuned; meaningless as a statistic. Replace with a chi2/dof GOF gate plus a likelihood-ratio gap. |
| Drag 10 % quad / 25 % lin | Observed max error 1.1 % / 3.2 %. Predicted 1-sigma 0.45 % / 1.45 %. Noise-free fit is exact. | Not tuned, but 22 / 17 sigma: far too loose. Use ~5 sigma (2.5 % / 7 %). |
| 8-tick (0.16 s) delay | 0.15/0.02 = 7.5 rounds to 8 ticks, and **0.17 also rounds to 8** (banker's rounding). At 0.15 or 0.17, total error is 11 ms (tol 10 ms). | Principled for the bit-exact comparison, but it hides that `FaultConfig` silently quantises `dead_time_s` by up to +-10 ms. INV-07 compares against the nominal d, not `dead_time_realised_s`. Reject non-multiples or test against the realised value. |

## 4. RT-06 flag-off bit identity

- The flag-off half (`h.make(nonneutral, enabled=False)`) instantiates `VesselModel`, so it compares
  base with base: tautological. The fault code is never constructed.
- The neutral half does run `FaultedVessel._external/_current_world/_delay_command` (40/40/10 calls
  in 10 steps) and is raw-byte identical, including signed zeros (checked without the -0
  normalisation in `trajectory_hash`). The delay-buffer branch is not exercised (ticks = 0).
- The production-like `adv_faults.build_vessel` has no test (mutant N6 survives).
- Fix: call `F.build_vessel`. Assert that a non-neutral config with `enabled=False` gives a byte-equal
  trajectory and that `enabled=True` changes it. Add a neutral run with `dead_time_s = 0` but the
  buffer path forced. Python only: the C# component remains EM / NOT_RUN.

## 5. R17: reproduced numerically, mechanism refined

Setup (throwaway, `/tmp/crane-verify-adv/scripts/r17_repro.py`):
- Plant: adv_plant, with an effort-space X mixer derived from `column()`.
- Fitter: a re-implementation of mhseals_testing@a645657 `fit()`. It uses single-axis and neutral samples, `v = b + G e (+ drift)`, and the same `1e-8` ridge.
- Decoupling: `C = G^-1 diag(G)` (mhseals_hardware `decoupling_for`, with `decoupling_negative = None`).
- Schedule: fit at +-0.1/+-0.2 plus a turn, verify at +-0.4.
- Holds start fully settled. This matters: without a reset, 4 s settles leave carry-over that creates false asymmetry.

| Plant variant (noise free) | Effect of the sign-averaged fitted C |
|---|---|
| Linear thrust, linear drag, RR 0.95 | Works: cross 0.0375 -> 0.0000 (control). |
| Quadratic thrust, backK 0.781, linear drag, RR 0.95 | **Reverse rows worse, forward rows better**: surge- 0.0023->0.0028, sway- 0.078->0.089, yaw- 0.080->0.096. This is the R17 signature. |
| Linear symmetric thrust, quadratic drag, RR 0.95 | All rows worse, both signs, even when verified inside the fit envelope (+-0.2). |
| Nominal fixture (quadratic thrust and drag, backK 0.781) | Yaw rows 0.089 -> 0.153 / 0.219; with 0.01 m/s odometry noise, surge rows 0.010 -> 0.016-0.023. |
| Symmetric reverse (backK = 1), no asymmetry | C = I exactly; no change. |

Conclusion: R17 is a fitter/decoupling-method defect, not noise. Two mechanisms:
1. Sign-averaged G on a backK-asymmetric plant. This gives the reverse-row signature, as hypothesised.
2. Velocity-space decoupling under quadratic drag. Cross leaks are small-signal and see the linear-drag slope. The diagonal gains are large-signal, so C over-corrects in both directions.

A naive per-sign refit (C+/C-) did **not** fix it in the fixture: its one-sided intercept/slope fit is biased. So fitting with `decoupling_negative` alone is not shown to be sufficient.

Caveat: the fixture layout is not CRANE's. Pre-fit cross values in the fixture are 0.09-0.17 m/s, against 0.012-0.019 m/s in CRANE. The reproduction is qualitative.

## 6. C# PhysicsInvariants @ 81b3137 (static review)

**6.1 `Coriolis_PowerIsZero_Planar` cannot fail on the legacy defect, although it is tagged `KnownDefectV2`.**
- Legacy `CalculateCoriolisForceTorque` puts rows 0-1 into `Tc` (never `Fc`) and never reads rows 3-5. So for planar motion, F = 0 and torque lands on Unity x/z, while w has only a y component.
- Power is therefore exactly 0 (re-implemented: P = 0.000, |wrench| = 3.10 > 0.5 guard). It passes on legacy.
- It would also pass on a sign-flipped skew C (my N5 analogue). The magnitude test `Coriolis_PlanarMotion_StaysInSurgeSwayYaw` uses `Mathf.Abs`, so it is sign-blind too.

Proposed replacement, `Coriolis_Planar_SignedWrenchMatchesFossen`, `[TestCase]` over
(u,v,r) = (1,0.2,0.5), (-0.7,0.3,-0.4), (0.5,-0.6,0.8):
- Assert the signed body-FLU wrench equals -C_A(nu)nu = (X = +a22 v r, Y = -a11 u r, N = (a11-a22) u v). In Unity terms: RelForce.z = X, RelForce.x = -Y, Torque.y = -N, all other components 0, tol 1e-4.
- Add a seeded loop over 50 random 6-DOF states asserting nu.tau = 0 and tau(2 nu) = 4 tau(nu).
- Set XdotU/YdotV explicitly via reflection, with a11 != a22 (for example 6/8 and 9/4), so the a11/a22 routing cannot coincide.
- The 6-DOF power test is discriminating on legacy (P = -0.036). Keep it.

**6.2 `ZeroCommand_EnergyDoesNotIncrease_FullFossenStack` is likely non-discriminating.**
- With damping on, dissipation (~0.6 J/step) swamps the legacy misrouted Coriolis power (~0.003-0.04 W).
- Explicit Euler with a lagged added-mass force is not energy-consistent anyway.
- Run Coriolis with Damping = false, and use instantaneous power at many states rather than integrated energy.

**6.3 The dt inference from the added-mass force can hide defects.**
- `EffectiveDt()` returns MaSurge/|F|, so dt_inf = dt_true x XdotU / Ma_eff.
- In the step-response test, the hydro force per step is Ma_eff dv/dt_true = MaSurge dv/dt_inf. The test is therefore exactly invariant to any wrong added-mass magnitude: 2x, or YdotV used in surge.
- It is also invariant to a dt-unit defect in `GetStateDot` (deltaTime vs fixedDeltaTime, dividing by dt twice).
- Only the sign is tested. The test also relies on the serialized default XdotU = 6 without setting it.
- Fix: read `Time.deltaTime` (the same static the component reads), assert the component-implied dt equals it, set XdotU explicitly, and repeat with XdotU = 12 and with sway (YdotV) to catch DOF mix-ups.

**6.4 Reference value: correct.**
- Fossen 2011 gives (M_RB + M_A) nu_dot = tau with M_A = -diag(X_udot, ...) = diag(a11, ...), a11 > 0. So a = 10/(23.468 + 6) = 0.33935 m/s^2, which matches the test.
- It assumes `XdotU = 6` is the positive added mass a11 (= -X_udot). Under the SNAME reading (X_udot = +6), the "defect" is the parameter sign, not the formula. The test's expected value is right either way; the docstring should say which.
- The Munk/Coriolis magnitudes (0.4, 3.0, 0.8) also match -C_A(nu)nu.
- The explicit lagged scheme converges, since Ma/M = 0.256 < 1. The legacy fixed point is 10/17.468 = 0.572, so the test discriminates.
- Note: the C# tier uses M = 23.468 kg while the PY tier uses 30 kg. Each is self-consistent, but they do not cross-check each other.

## Required fixes, ranked

1. Add a goodness-of-fit gate (residual chi2/dof against sigma) to `separate_current_bias` and `estimate_mixer`. Add an ID-06 variant with shuffled heading order (currently 20/20 false identifications).
2. Replace or fix C# `Coriolis_PowerIsZero_Planar`; it passes on the known defect. Add signed-component asserts and make the Coriolis tests sign-sensitive.
3. Remove the circular dt inference: compare against `Time.deltaTime`, set coefficients explicitly, and vary XdotU.
4. Fix the `ljung_box_q` crash for n <= lags; abstain or reduce the lag count.
5. Kill N2/N3/N4:
   - Tighten INV-08 to ~2x the baseline error, or check convergence order.
   - Add a first-order step-shape check (63 % at tau) to the suite.
   - Run INV-07 at dt = 0.01 and assert against `dead_time_realised_s`.
   - Reject non-tick-multiple dead times.
6. Make RT-06 call `build_vessel` (kill N6) and test non-neutral flag-off.
7. Assert the abstain `reason` in every MUST_ABSTAIN test. Rename ID-05's statistical gate, or add a structural heading-spread gate as upstream does (`heading_spread < 0.9`).
8. Use independent noise seeds per pulse. Include the drag-integral noise in `measure_pulse_wrench` sigma.
9. Tighten the drag tolerances to ~5 sigma. Validate the PT-05 rest mask against commands and velocity.
10. R17: keep the crane_testing port deferred. The defect is reproduced in the fixture. A sign-split decoupling alone is not shown to fix it; the decoupling needs a force-space or operating-point-aware formulation.
