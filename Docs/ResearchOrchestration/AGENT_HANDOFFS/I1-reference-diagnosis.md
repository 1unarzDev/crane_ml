# I1 reference diagnosis: why `reference` fails the cross-axis row, and what tolerance is defensible

Date: 2026-10-21. Read-only analysis of retained runs. No Unity, no ROS, no existing file modified. The only files written are this document and `Tools/Adversarial/diagnostics/` (run with `python3 -I`; numpy only). Follows `I1-council-hydro.md` section 1 (F1, F2).

## 0. Summary

1. The 0.0534 m/s on a 0.05 m/s limit is **not statistically distinguishable** from 0.05. Within-run 95% CI is [0.047, 0.059]. Run-to-run, a single row's metric moves by SD 0.011. The nominal plant with the identity decoupling already gives a worst-row value of 0.0475 +/- 0.0075 (n=4), and two of those four exceed 0.05.
2. The criterion does not model or exclude turn-induced sway. It never reads yaw rate for the translation gate. A fixed 0.05 limit leaves no room: the noise floor (about 0.0475) plus the kinematic leak allowed by its own yaw gate (about 0.020) is already about 0.067.
3. F2 (sway on u*r) gives slope -0.85 s (CI -0.87 to -0.83), R2 0.976. That fit is carried entirely by the mixed surge+yaw windows. In the failing-row regime (|u*r| < 0.05) the slope is unidentified (CI -1.15 to +1.04). Removing the modelled turn term takes the failing row from 0.0534 to 0.0385 (CI 0.031 to 0.040), but that relies on extrapolation.
4. Proposed tolerance: 0.087 m/s (provisional, n=4 noise sample). `reference` passes (0.0534 and 0.0504). Every MUST_FAIL case is still flagged, with margin at least 1.9x at the final stage.
5. The lever-arm hypothesis is falsified. The yaw-row translation leak is mostly even in r, not odd.
6. F1 is only partly necessary. The identity-decoupling arm is already recorded as the `before` stage of every run, under the same envelope. See section 6.

## 1. Data located and the signal map

Used 15 runs that have `truth.json` + `held-windows.json` + a MEASURED `result.json`, under `PerformanceResults/calibration-*-20261019*`. Other runs have only `result.json` with the per-row rms (including the 4th nominal run `calibration-faults-20261019/reference`); I used `result.json` for those in the section 5 table. Nothing relevant exists in `/home/lunarz/worktrees/crane-characterization/PerformanceResults` (those are Nav2/perf runs, no calibration cases). `mhseals_testing-characterization` holds source, not results.

- `truth.json`: `[[t, {vx, vy, wz, x, y, yaw, ...}], ...]` at about 50 Hz (about 35k samples per run). `vx, vy, wz` are body-frame velocity (REP-103) and yaw rate. Frame check: body-frame hypothesis rms error 0.039 m/s versus world-frame 0.595 m/s against differentiated position (`d3_f2.py`).
- `held-windows.json`: 85 windows with `command` [surge, sway, yaw] effort, `start`, `end`, `label`. Commands are normalised effort. The velocity commands are in `closed_loop` only.
- `result.json`: `before`, `after_balance`, `after_decoupling` / `after_characterization`, `closed_loop`, with per-row `mean` and `rms`.
- `d0_reproduce.py` rebuilds every effort row from truth + windows (samples in (start+5 s, end]) and reproduces the reported rms to at most 6.6e-4 m/s in all 15 runs. So the window mapping is validated.

The reference plant is `RoboBoatHull` (m = 30 kg, sway drag 15 N s/m linear + 41.7 N s^2/m^2 quadratic; `RoboBoatPhysics.cs`). Fossen is irrelevant here, as the council found.

## 2. Part (c): what the criterion actually computes (mhseals_testing a645657)

`git show a645657:test/crane_characterization_probe.py`, `verify("effort")` and `effort_acceptance` (lines about 143-231):

```
for each (axis, sign): hold command 8 s (after an 8 s neutral hold); measure truth in (start+5, end]  -> about 150 samples (3 s)
rms = sqrt(mean((vel - [0,0,0])**2, axis=0))          # effort mode: about ZERO, not demeaned
translation_cross_axis_rms_at_most_0_05_m_s = all( norm([rms[i] for i in range(2) if i != axis]) <= 0.05 )
yaw_cross_axis_rms_at_most_0_03_rad_s       = all( rms[2] <= 0.03 for rows with axis != 2 )
```

Properties:
- The run used mhseals_testing 4170d9e plus patch; the retained `probe.py` in the run directory has the same `verify` and `effort_acceptance` text as a645657 (diff shows only unrelated harness changes).
- The metric is **raw RMS about zero**. It is mean-dominated: for the failing row the sway mean is 0.0519 and the SD is 0.0124, so rms = sqrt(0.0519^2 + 0.0124^2) = 0.0534. This is a steady sideslip (about 3.9 degrees at 0.77 m/s), not noise.
- For surge and sway rows the translation cross is one component. For yaw rows it is the norm of both translation components.
- **It is not built to exclude turn-induced sway.** Yaw rate never enters the translation gate. There is no u*r term, no demeaning, no normalisation by speed, and no tolerance for the kinematic coupling that a rigid body must show (sway ~ -(m/Y_v) u r). The only link is the separate yaw gate `rms[2] <= 0.03`, which bounds r and so bounds turn-induced sway at roughly |c| u 0.03 (about 0.020 at c = -0.85 s, u = 0.78). That consumes 40% of a 0.05 budget before any noise.
- The limit is the same absolute number at every speed and for every axis.

## 3. Part (a): noise of the cross-axis metric

Method: for each row of each nominal run, moving-block bootstrap of the 3 s window (block = 3x the integrated autocorrelation time of the squared cross components, 5 to 50 samples, 4000 resamples; only 3 to 6 blocks per window, so the intervals are approximate). Run-to-run noise from replicates of identical configuration: the identity-decoupling `before` stage of the nominal runs. Script: `d2_noise.py`, output `noise_results.json`.

### Within-run (3 s window, 95% block-bootstrap CI)

Typical SD of the metric 0.001 to 0.005; CI half-width 0.002 to 0.008 (yaw rows widest, up to 0.010).

Failing row (isolated-reference-drag, after decoupling, surge -0.4): metric **0.0534, CI [0.0472, 0.0593]**, SD 0.0032. The CI contains 0.05.

### Per-row, nominal `reference`, current fitter (all 6 rows, 95% CI)

| row | before (identity), 3 runs | after: isolated-reference-drag | after: faults-instrumented |
|---|---|---|---|
| surge + | 0.0147 / 0.0098 / 0.0212 (mean 0.0153, SD 0.0057) | 0.0233 [0.0207, 0.0269] | 0.0269 [0.0187, 0.0325] |
| surge - | 0.0116 / 0.0169 / 0.0242 (0.0175, 0.0063) | **0.0534 [0.0472, 0.0593]** | 0.0356 [0.0339, 0.0431] |
| sway + | 0.0173 / 0.0206 / 0.0208 (0.0195, 0.0020) | 0.0165 [0.0139, 0.0178] | 0.0121 [0.0099, 0.0144] |
| sway - | 0.0186 / 0.0152 / 0.0212 (0.0183, 0.0030) | 0.0191 [0.0160, 0.0215] | **0.0507 [0.0385, 0.0564]** |
| yaw + | 0.0167 / 0.0385 / 0.0539 (0.0364, 0.0187) | 0.0292 [0.0234, 0.0396] | 0.0305 [0.0208, 0.0347] |
| yaw - | 0.0437 / 0.0388 / 0.0491 (0.0438, 0.0052) | 0.0272 [0.0206, 0.0328] | 0.0319 [0.0268, 0.0373] |

(Row order in the probe is axis 0, 1, 2 with sign +1 then -1. `before` columns list the three runs with truth windows in the order isolated-reference-drag, faults-instrumented/reference, faults-instrumented/reference.previous.)

### Run-to-run

- Worst-row statistic T (the number the gate sees) for identity decoupling, 4 nominal runs: 0.0538, 0.0539, 0.0389, 0.0437. Mean 0.0475, SD 0.0075. Two of four exceed 0.05.
- Between-run SD of a single row's metric after the fit (2 runs x 6 rows, pooled): 0.0106.
- Within-run SD (about 0.003) is small next to run-to-run (about 0.009 pooled over rows before fitting, 0.011 after). The run-to-run component dominates.

### Is 0.0534 distinguishable from 0.05?

No.
- Within-run CI [0.0472, 0.0593] includes 0.05.
- Against run-to-run noise: z = (0.0534 - 0.05)/0.0106 = 0.32, p = 0.37.
- If T after fit has the same distribution as T before fit: P(T > 0.05) = 0.37, P(T >= 0.0534) = 0.22.
- The second current-fitter run also exceeds the limit, at 0.0507 on a different row (sway -). So the binary pass/fail on this gate is close to a coin flip for this plant. The exceedances are +7% and +1.4% of the limit.

Two further runs (`calibration-faults/reference`, `faults-instrumented/reference.previous`) show T = 0.2093 and 0.2087 after fitting. These ran the superseded fitter (no `fit_sample_count`; fitted on mixed commands; decoupling off-diagonals up to 0.15 and 0.27 versus 0.01 for the current fitter). They are not noise; they are a defective fit that the criterion rightly flags. Exclude them from the noise estimate.

## 4. Part (b): F2 regression, sway on u*r

Script `d3_f2.py`, output `f2_results.json`. Data: the three nominal runs with windows (57 windows, 9037 samples). Windows with a commanded sway and the velocity-controller windows are excluded. Quasi-steady part of each window only. Regression: `vy = a + b*u + c*(u*r)`, CIs from a window-cluster bootstrap (3000 resamples). Body-frame truth.

| Subset | n windows | const a | u coeff b | **u*r coeff c (s)** | R2 |
|---|---|---|---|---|---|
| All windows | 57 | 0.0030 [-0.0040, 0.0105] | 0.0141 [-0.0033, 0.0364] | **-0.850 [-0.873, -0.834]** | **0.976** |
| Pure-axis only (no mixed surge+yaw) | 51 | 0.0030 | 0.0157 | -1.33 [-2.66, +0.94] | 0.246 |
| Failing-row regime, max abs(u*r) < 0.05 | 50 | 0.0013 | 0.0053 | +0.25 [-1.15, +1.04] | 0.017 |

Adding an `r` term (lever arm) changes nothing: the all-windows u*r coefficient is -0.850 [-0.876, -0.830] and the r coefficient is 0.0002 [-0.0097, 0.0152]. Leave-one-run-out c: -0.849, -0.851, -0.850.

Physical prior (council): c = -m/Y_v; with m = 30 and Y_v = 15 + 1.5 = 16.5 N s/m, c = -1.82 s. The fit magnitude is about half that. That is consistent with the quadratic sway term, which raises the effective drag slope above the linear value once v is nonzero. Sign is as expected.

Reading:
- The regression has R2 0.976 only because six mixed windows carry |u*r| up to 0.8. Without them R2 drops to 0.25, and in the failing row's own regime the coefficient is unidentified. So F2 confirms the mechanism exists. It does not prove the size of the effect at u*r = -0.018.
- Failing row (after decoupling, surge -0.4): mean u*r = -0.0179 (u = -0.772, r = +0.0232 rad/s). Turn-induced sway predicted = c*u*r = +0.0152 m/s, about 29% of the 0.0519 mean.
- **Residual cross-axis after removing the u*r term: 0.0385 m/s, 95% CI [0.0312, 0.0402]** (coefficient and block resampling jointly). Removing the full constant + u*r part gives 0.0461.
- Where the yaw came from: before the decoupling the same row had r = -0.0018 rad/s, so the decoupling itself injected the yaw leak (0.023 rad/s, below the 0.03 yaw gate). The turn-induced sway is a second-order consequence of the fit.
- The remaining sway is not explained by u*r. About 0.037 m/s stays unexplained (even in the surge sign: sway mean is +0.0116 for surge + and +0.0519 for surge -, so it is not an odd, proportional-to-command coupling that a linear decoupling matrix could cancel).

### Lever-arm hypothesis (council item 4): not supported

Pure-yaw identity-decoupling windows (21): `vx = a + k*r`, with a rigid COM offset predicting a = 0.
- vx: even part a = -0.0129 [-0.0201, -0.0075]; odd slope k = 0.0225 m/rad [0.0097, 0.0409] (implied offset about 2 cm, worth 0.002 m/s at r = 0.09).
- vy: a = 0.0046 [0.0001, 0.0087]; k = 0.0016 [-0.0047, 0.0100].

The translation leak in the yaw rows is mostly even in the yaw sign, not odd, so a body-point offset explains at most about 10% of it. The even part points to a thruster-asymmetry or nonlinearity effect (thrust ~ cmd*abs(cmd), reverse/forward ratio 0.78), not to the truth reference point.

## 5. Part (d): principled tolerance

Statistic: T = max over the 6 rows of the cross-axis rms, evaluated at the **final** stage (after decoupling), as in `manual_acceptance`.

Definition: **tol = Q95 + K_kin**

- **Q95 = 0.0673.** One-sided 95% prediction bound for T under the nominal plant with identity decoupling (n = 4 runs: mean 0.0475, SD 0.0075, t = 2.353, factor sqrt(1 + 1/4)). Parametric-bootstrap CI of the bound: 0.051 to 0.084. n = 4 is thin; see Uncertainty.
- **K_kin = |c| u_max r_gate = 0.85 x 0.78 x 0.03 = 0.0199** (CI upper 0.0204). This is the sway that a rigid body must show at the largest yaw rate the existing yaw gate permits. With the physics prior c = 1.82 the term is 0.0425, but that conservative tolerance (0.110) fails the detection requirement below.
- **tol = 0.0872 m/s.**

Check against every recorded run (final-stage T; "FLAG" = criterion fails):

| case | runs | T_final | current 0.05 | tol 0.087 | min margin (min T / tol) |
|---|---|---|---|---|---|
| reference, current fitter | 2 | 0.0534, 0.0504 | FLAG, FLAG | **pass, pass** | 0.58 (passes) |
| reference, old fitter (defective fit) | 2 | 0.2093, 0.2087 | FLAG | FLAG | 2.39 |
| weak_rr (MUST_FAIL) | 3 | 0.1644, 0.1684, 0.3488 | FLAG | **FLAG** | 1.89 |
| angled_fr (MUST_FAIL) | 1 | 0.2022 | FLAG | **FLAG** | 2.32 |
| combined_current (MUST_FAIL) | 8 | 0.1931 to 0.2548 | FLAG | **FLAG** | 2.21 |
| hull_drag (not MUST_FAIL) | 1 | 0.1029 | FLAG | FLAG | 1.18 |

Pre-fit (`before`) detectability, if someone gates before decoupling: weak_rr 0.188 to 0.207 (margin 2.2), combined_current 0.195 to 0.197 (2.2), **angled_fr 0.1005 (margin 1.15)**, hull_drag 0.0338 (not detected under any tolerance including 0.05). So the pre-fit angled_fr margin is thin, though still above tol; at a 0.087 limit and noise SD about 0.009, P(detect) is about 0.93 for it.

Upper limit on any tolerance: it must stay below the smallest required-fault T. That is 0.1005 pre-fit (angled_fr) or 0.1644 final stage (weak_rr). The conservative tolerance 0.110 would lose angled_fr pre-fit and hull_drag post-fit. **A tolerance in the band 0.087 to 0.10 is the only range that passes `reference` with margin and keeps every MUST_FAIL case detected on both stages.**

The tolerance is not a blind loosening: it is zero-free (no tuned constant), it came out of the measured nominal T and a physical term, and it is bounded above by the faults.

## 6. F1 feasibility (not run)

- The identity-decoupling arm is already recorded. Every run's `before` stage is the same plant, same verification envelope (+/-0.4 effort), identity decoupling. Pairs for the current fitter: T 0.0437 to 0.0534 and 0.0389 to 0.0504 (paired change +0.0097, +0.0115). Identity fails 2 of 4 nominal runs (0.0538, 0.0539); the fitted decoupling fails 2 of 2 current-fitter runs. These do not discriminate: both arms straddle 0.05.
- The reverse rows are the ones that rise after the fit (surge -: 0.0116 to 0.0534 and 0.0169 to 0.0356; sway -: 0.0186 to 0.0191 and 0.0152 to 0.0507). The saved profile has `decoupling_negative: null`, so one sign-averaged matrix is applied to both signs while the plant is sign-asymmetric. This is a hypothesis consistent with the data, not tested.
- If live runs are wanted: the probe already has `--save-only` (save the model without applying the proposed C), which is exactly the identity arm; no code change. A run is about 700 s of sim per case (`truth.json` t_max about 709 s). Past `save-only` attempts on `combined_current` ended FAILED on stale odometry (3 runs), so expect some losses.
- Power, exact Fisher one-sided test, binary fail/pass: fitted 100% vs identity 50% fail needs about 10 runs per arm for 83% (6 per arm: 34%; 8: 64%). Fitted 90% vs 50%: about 12 per arm for 60%. A **paired continuous** comparison of T (each run gives both stages) is cheaper and answers the question better; I could not size it because n = 2 current-fitter pairs gives no SD of the paired difference.
- Recommendation: do not run F1 as a binary fail-rate test. Run N >= 10 nominal repeats once, record both stages, and use the paired T differences plus the identity T distribution to freeze the tolerance.

## 7. Uncertainty

- Noise-floor sample: n = 4 identity-decoupling T values, n = 2 current-fitter fits. The Q95 bound has a CI 0.051 to 0.084; the tolerance 0.087 inherits that (0.071 to 0.104). The lower end sits inside the 0.087 to 0.10 working band; the upper end (0.104) is above the angled_fr pre-fit signal.
- Both nominal-fit runs are from one fixture (same fault-free plant, same seed settings?). Seed control was not verified; "run-to-run" here may be partly deterministic differences between different decoupling fits, not physical noise.
- The turn term c is identified only at |u*r| about 0.8 from mixed windows. Extrapolation to u*r about 0.018 is an assumption.
- Block-bootstrap intervals use 3 to 6 blocks per 3 s window and are approximate.
- Two of the 15 runs used a different `mhseals_testing` revision (1483897, 97d62f4) from the others (4170d9e); they are fault cases only and do not affect the nominal estimates.
- The reproduction from `truth.json` differs from the reported rms by at most 6.6e-4; this is below every conclusion here.

## 8. Proposed decision on the criterion

1. Do not touch hydrodynamics for this failure. It is the criterion plus the fitter (matches the council).
2. Replace the fixed 0.05 with `tol = Q95(identity T over >= 10 nominal repeats) + |c| u_max r_gate`, with the formula and the data frozen in PREREGISTRATION before the comparative runs. Provisional value 0.087 m/s from the current n = 4.
3. Keep the yaw gate (0.03 rad/s) unchanged; it fixes `r_gate` in the kinematic term.
4. Optionally report, not gate: sideslip ratio |mean(v)|/|u| and the u*r-adjusted residual as diagnostics.
5. Accept only if all MUST_FAIL cases remain flagged at the final stage with margin >= 1.5, and `reference` passes on >= 90% of repeats (check, not tuning).
6. Treat the sign-averaged decoupling on a sign-asymmetric plant as the separate fitter defect (reverse rows rise after the fit). That belongs in mhseals_testing and is worth its own test.

## 9. Falsifiers

- If, with N >= 10 nominal repeats, Q95 of identity T exceeds 0.095, the derived tolerance reaches the angled_fr pre-fit signal (0.1005) and the tolerance cannot be used at the pre-fit stage; move the gate to the final stage only, or reduce the noise.
- If the u*r coefficient estimated in the |u*r| < 0.05 regime (needs deliberately injected small yaw leaks) disagrees with -0.85 beyond its CI, K_kin is wrong.
- If the current-fitter `reference` T after fit systematically exceeds the identity T by more than the kinematic term allows (paired difference > 0.02 at N >= 6), the fitter, not the criterion, is the defect.
- If `reference` repeats with a fixed seed give identical T, "noise" here is deterministic fit sensitivity, and the tolerance should then be stated as a bound on fit quality, not as a statistical quantile.
- If any MUST_FAIL case drops below the tolerance in a repeat, the tolerance is rejected.

## 10. Risks

- Calling a tolerance "principled" on n = 4. Present 0.087 as provisional; do not fold it into PREREGISTRATION until the repeat set exists.
- Choosing K_kin from the prior c = 1.82 (tolerance 0.110) would silently lose angled_fr and hull_drag detection.
- The hull_drag case is invisible pre-fit (0.034) under any tolerance. Do not count it as a MUST_FAIL signal for this gate.
- Accepting the tolerance would also accept a decoupling that increases T from 0.044 to 0.053; that is a quality loss that this gate can no longer see. Pair it with a "no worse than identity by more than K" check.

## 11. Next action

Run one batch of N >= 10 `reference` repeats (each recording `before` and `after`) with the existing harness, seeds logged, in a new results directory. Re-run `d4_tolerance.py` (it reads `result.json`; add the new runs) and freeze the tolerance.

## Scripts (all in `/home/lunarz/crane_ml/Tools/Adversarial/diagnostics/`, run with `python3 -I <script>`)

- `diag_common.py`: loader, window mapping, metric.
- `d0_reproduce.py`: reproduces `result.json` rms from `truth.json` + `held-windows.json`.
- `d1_table.py`: per-row cross-axis for every run and stage.
- `d2_noise.py`: block-bootstrap CIs and run-to-run table; writes `noise_results.json`.
- `d3_f2.py`: F2 regression, row-level residual, frame check; writes `f2_results.json`.
- `d4_tolerance.py`: tolerance, fault detection, lever-arm parity, F1 power; writes `tolerance_results.json` (reads `f2_results.json`; run d3 first).
