# D2 DEV results: belief-space search, verify and report under correlated detections

Evidence label: IMPLEMENTED (synthetic, mission level; every number is DEV only: MissionSim dev layouts 0-1999, tuning 0-23, sweeps 100-219; no held-out seed was touched, none of 91000-91019 or 92000-92031). Nothing here is LIVE_SIM_VALIDATED or PHYSICALLY_CALIBRATED. The held-out draft is `PREREG_BELIEF.md` (DRAFT, not frozen). Code: `Tools/BeliefSVR/` (`bs_exact.py`, `bs_sim.py`, `bs_policies.py`, `bs_sweep.py`, `bs_report.py`). Unit tests: ............ ---------------------------------------------------------------------- Ran 12 tests in 4.535s  OK.

## 0. Verdict (read this first)

- **Exact (the main result).** Minimal examples exist for all three effects (section 2). At rho = 0.6 the exact optimum beats a policy planned with the independence belief by 1.7 to 1.9 in J on two 2-cell instances (E[false reports] x1.33 and x2.04). An independent own view is worth 1.6 time units at s = 1 and is never used at rho = 0. A partner look beats the independent own view for every delay up to the horizon at s >= 1 (and up to d = 3 at s = 0, pP = 0.6).
- **At scale the exact structure mostly does not survive threshold tuning.** Comparing the best grid point per family at lambda = 300 s per false report, at the measured rho (0.6, the Vision ICC 0.61) and measured f: in the verification-heavy regime CORR is within 1 % of IND_BAYES and of FIXK (CIs include 0) and 8 % [2, 13] better than the ESS-deflated SPRT (the SPRT-type rules use a coarse grid, see 5). In the coverage-bound regime CORR ties the deflated SPRT and CI rules (0 % [-2, 2]) and is 12 % [9, 15] better than IND_BAYES even with IND's threshold tuned (to 0.999).
- **H1 (>= 2x false-report inflation at matched time):** NOT supported at the measured rho: 1.06 [0.85, 1.37] verification-heavy, 1.67 [1.24, 2.25] coverage-bound, 1.21 [1.04, 1.48] at rho 0.8. The 2x is reached only in the exact instance B and at high f with long frontiers (noisy CIs).
- **H2 (>= 15 % gain vs fixed-k):** NOT supported at measured f (-0 % [-5, 4] at rho 0.6, verification-heavy). Supported only at low f (verification-heavy: 21 % [17, 27] at rho 0.3, 10 % [6, 15] at 0.6) and at high f in the coverage regime (37 %), where fixed-k is simply a poor rule.
- **H3 (gain lives in the verification-bound regime):** weakly. The gains over the best other rule are 3-6 % (CIs touch 0) at LOW f in the verification regime and about 0 % in the coverage regime; at measured f nothing is detectable in either.
- **Kill check (section 6).** K1 (FR inflation < 1.3 at measured rho): FIRES in the verification-heavy regime (1.06), not in the coverage-bound regime (1.67). K2 (a cheap fix within 5 % of the best): FIRES in the coverage-bound regime (the deflated SPRT / CI rules are within 1 % of the best) and, in the verification-heavy regime, the deflated SPRT / CI rules are 10 % above the best but an ordinary Bayes threshold tuned for the false-report cost (IND_BAYES, theta up to 0.999) is within 1 %, which is the same kill in practice. Recommendation: write this up as "correlation changes the calibration of the stated posterior, not the tuned decision", fold it into RoleSpec as a calibration note, and keep the exact decision-structure result (sections 2, 8) as the contribution.
- **Not a model of the real world.** The cross-view independence, the decoy fire rate per view and the background alert rate are PRIOR values (section 10 lists what the Vision rerun must tighten).

## 1. Difficulty tuning (DEV only)

Known problem: on the MissionSim val split almost every policy succeeded <= 25 % (oracle 0.75), so nothing was discriminated. The search area (K cells by public prior, target drawn inside it), the deadline D, and the review / view-change durations were tuned on layouts 0-23 at rho 0.6, f measured, with a 7-policy subset (success / false reports per mission). Selection criterion, fixed before looking at the policy comparison: ORACLE success >= 0.85, COVER success in 0.35-0.55 (so room above and below), and COVER failures dominated by `route` (coverage-bound) or `visibility` (verification-heavy) rather than by deadline starvation.

| candidate regime (K cells, D s) | ORACLE (succ / FR) | COVER (succ / FR) | FIXK2 (succ / FR) | SPRT@3.0 (succ / FR) | SPRT_DEFL@2.0 (succ / FR) | IND_BAYES@0.5 (succ / FR) | CORR@0.5 (succ / FR) | CORR_PARTNER@0.5 (succ / FR) | COVER miss causes | verif. time share (CORR@0.5) |
|---|---|---|---|---|---|---|---|---|---|---|
| cand_K100_D600 | 0.94 / 0.0 | 0.33 / 1.7 | 0.19 / 0.6 | 0.25 / 0.9 | 0.33 / 1.5 | 0.23 / 0.8 | 0.10 / 0.2 | 0.12 / 0.3 | route 26, visibility 6 | n/a |
| cand_K100_D900 | 0.94 / 0.0 | 0.50 / 2.1 | 0.35 / 0.7 | 0.46 / 1.2 | 0.50 / 1.9 | 0.44 / 1.0 | 0.19 / 0.3 | 0.23 / 0.4 | route 10, visibility 14 | n/a |
| cand_K100_D1500 | 0.94 / 0.0 | 0.65 / 2.9 | 0.44 / 0.9 | 0.62 / 1.4 | 0.62 / 2.6 | 0.52 / 1.1 | 0.25 / 0.4 | 0.31 / 0.5 | visibility 17 | n/a |
| cand_K100_D2400 | 0.94 / 0.0 | 0.65 / 3.5 | 0.44 / 1.0 | 0.62 / 1.8 | 0.65 / 3.1 | 0.67 / 1.2 | 0.31 / 0.5 | 0.33 / 0.5 | visibility 17 | n/a |
| cand_K30_D600 | 0.85 / 0.0 | 0.46 / 1.0 | 0.29 / 0.4 | 0.42 / 0.5 | 0.46 / 0.9 | 0.40 / 0.5 | 0.35 / 0.5 | 0.38 / 0.5 | route 2, visibility 24 | n/a |
| cand_K30_D900 | 0.85 / 0.0 | 0.46 / 1.1 | 0.31 / 0.4 | 0.44 / 0.6 | 0.46 / 0.9 | 0.46 / 0.6 | 0.40 / 0.5 | 0.44 / 0.6 | route 2, visibility 24 | n/a |
| cand_K30_D1500 | 0.85 / 0.0 | 0.46 / 1.1 | 0.31 / 0.4 | 0.44 / 0.6 | 0.46 / 0.9 | 0.52 / 0.6 | 0.54 / 0.7 | 0.58 / 0.7 | route 2, visibility 24 | n/a |
| cand_K15_D600 | 0.90 / 0.0 | 0.46 / 0.6 | 0.42 / 0.2 | 0.44 / 0.3 | 0.46 / 0.5 | 0.60 / 0.4 | 0.62 / 0.3 | 0.62 / 0.3 | route 2, visibility 24 | n/a |
| cand_K15_D900 | 0.90 / 0.0 | 0.46 / 0.6 | 0.42 / 0.2 | 0.44 / 0.3 | 0.46 / 0.5 | 0.67 / 0.6 | 0.75 / 0.4 | 0.75 / 0.4 | route 2, visibility 24 | n/a |

| second pass (K, D, review s, view-change s) | ORACLE (succ / FR) | COVER (succ / FR) | FIXK2 (succ / FR) | SPRT@3.0 (succ / FR) | SPRT_DEFL@2.0 (succ / FR) | IND_BAYES@0.5 (succ / FR) | CORR@0.5 (succ / FR) | CORR_PARTNER@0.5 (succ / FR) | COVER miss causes | verif. time share (CORR@0.5) |
|---|---|---|---|---|---|---|---|---|---|---|
| cand2_K15_D900_rev20_sw25 | 0.90 / 0.0 | 0.46 / 0.6 | 0.42 / 0.2 | 0.44 / 0.3 | 0.46 / 0.5 | 0.67 / 0.6 | 0.67 / 0.4 | 0.69 / 0.4 | route 2, visibility 24 | 0.14 |
| cand2_K15_D1500_rev20_sw25 | 0.90 / 0.0 | 0.46 / 0.6 | 0.42 / 0.2 | 0.44 / 0.3 | 0.46 / 0.5 | 0.71 / 0.9 | 0.79 / 0.4 | 0.79 / 0.4 | route 2, visibility 24 | 0.14 |
| cand2_K15_D1500_rev30_sw25 | 0.90 / 0.0 | 0.46 / 0.6 | 0.42 / 0.2 | 0.44 / 0.3 | 0.46 / 0.5 | 0.71 / 0.9 | 0.79 / 0.4 | 0.79 / 0.4 | route 2, visibility 24 | 0.19 |
| cand2_K15_D1500_rev30_sw40 | 0.90 / 0.0 | 0.46 / 0.6 | 0.42 / 0.2 | 0.44 / 0.3 | 0.46 / 0.5 | 0.71 / 0.9 | 0.77 / 0.4 | 0.75 / 0.4 | route 2, visibility 24 | 0.19 |
| cand2_K8_D1500_rev30_sw25 | 0.90 / 0.0 | 0.46 / 0.4 | 0.40 / 0.1 | 0.46 / 0.2 | 0.46 / 0.3 | 0.85 / 1.2 | 0.92 / 0.6 | 0.92 / 0.6 | route 2, visibility 24 | 0.01 |
| cand2_K30_D1500_rev30_sw25 | 0.85 / 0.0 | 0.46 / 1.1 | 0.31 / 0.4 | 0.44 / 0.6 | 0.46 / 0.9 | 0.50 / 0.6 | 0.48 / 0.6 | 0.52 / 0.7 | route 2, visibility 24 | 0.22 |

Chosen: **coverage-bound** = K 100, D 900 s (COVER 0.50, 24 of 48 missions failed: route 10, visibility 14; ORACLE 0.94); **verification-heavy** = K 30, D 1500 s, review look 8 s, view change 12 s, partner delay 25 s (ORACLE 0.85, COVER 0.46 with failures dominated by an unseen target view; the verification time share of CORR is in the last column). The second pass tried slower reviews (20-30 s) and costlier view changes (25-40 s); it did not raise the verification share enough to justify a different regime, so the first choice stays. The first-pass miss causes show the honest limit of this regime: 24 of 26 COVER failures are `visibility` (the target's scan view is bad), a failure no verification policy can fix; only more coverage (a second pass over a cell from another class) can.

## 2. Exact minimal examples (`bs_exact.py`, `bs_exact_experiments.py`, full tables in `EXACT.md`)

Model: <= 3 view classes (own class 0 and 1, partner class 2), n = 2 cells, horizon H = 6, per (cell, class) latent shared by repeat looks (Gaussian copula calibrated so the binary ICC equals rho), decoys persistent with the same ICC, J = E[time of the correct report] + (1 - P(succ))(H + c_miss) + c_FR E[#false reports], c_FR = c_miss = 40. Instance A: pT = pP = 0.6, fd = 0.2, pid = 0.3, alpha = 0.02, s = 1 (own view change), d = 2. Instance B: pT = 0.8, pid = 0.6. State space <= 1.2 M (partner), <= 3 k (solo); the exact solve takes 1-30 s.

**(a) Independence over-reports.** Posterior of the target after k repeat alerts (A, rho = 0.6): independent 0.89 / 0.967 / 0.989 for k = 1 / 2 / 3, correct 0.89 / 0.925 / 0.933, i.e. odds inflation 1.0 / 2.4 / 6.5 (the brief's 2.2x at k = 3 is the equicorrelation LLR identity for the Gaussian case; the binary-latent case is larger because the latent saturates). Policy-level (planned with rb = 0, run in the rho world): E[FR] ratio 1.00 / 1.22 / 1.33 / 1.47 at rho 0 / 0.3 / 0.6 / 0.8 (A) and 1.00 / 1.48 / 2.04 / 2.27 (B); J penalty 1.7 (A, rho 0.6) and 1.9 (B). Fixed rule, instance B, rho 0.6 and threshold 0.95: independence belief reports with P(succ) 0.82 and E[FR] 0.062, the correlation-aware belief with 0.36 and 0.013, i.e. the same nominal threshold means a 4.8x lower false-report rate but 2.3x fewer reports.

**(b) Independent-view verification beats a repeat look.** Instance A, J(repeat only) - J(full): 0 at rho = 0 for every s; at rho 0.3: 1.16 / 0.40 / 0.00 for s = 0 / 1 / 2; at rho 0.6: 2.32 / 1.61 / 0.94; at rho 0.8: 2.88 / 2.19 / 1.50. Share of non-first looks that are independent-view: 0 at rho 0, 0.64 / 0.45 / 0 at rho 0.3 (s = 0 / 1 / 2), 0.96-1.0 at rho >= 0.6 (prediction P3 holds). The switch is therefore at a rho between 0 and 0.3 that falls as s falls (s = 2 needs rho > 0.3). Subtlety found by the solver: after a first ALERT in A the optimal action is to report (posterior 0.89); the independent view is used after a MISS (a repeat miss from a bad view is nearly uninformative at high rho). The instance C table in `EXACT.md` (fd 0.35, pid 0.6) shows the same gains (0.78 at rho 0.6, s = 1).

**(c) Partner vs solo as a function of delay.** Instance A, rho 0.6, pP = 0.6. J: repeat only 9.76; solo (repeat + own independent view) 8.14 at s = 1, 7.44 at s = 0; partner-only (no own independent view) 5.52 / 6.71 / 7.30 / 7.74 / 7.90 for d = 1 ... 5. Partner-only beats solo for d <= 5 at s >= 1 (the horizon-limited bound, d* >= 5) and for d <= 3 at s = 0; at pP = 0.8 d* >= 5 at every s. The partner's gain over repeat-only grows with rho (d = 2: 1.57, 2.28, 3.05, 3.32 at rho 0 / 0.3 / 0.6 / 0.8) and it helps even at rho = 0 because it adds an independent look at no drone time; the solo view helps only when rho > 0.

Caveats: instances are tiny and hand-chosen inside the Vision range (alerts of 0.6 and decoy fire 0.2 per look); n_cells is 2, so travel and search structure are absent; the exact policies ignore comms loss.

## 3. Policies and cost

Reused from MissionSim: layouts, prior, blockages, travel matrix, scan times, the Vision ObsModel marginals. Not reused: MissionSim's observation and comms loop (no correlation knob, no verify/report action API). `bs_sim.py` is a one-drone mission engine with scan / review / partner / report actions at 1 s resolution plus a partner drone that answers after d0 s with an independent view-class-2 look; a wrong report costs c_FR in J and rules the cell out; the target latent and decoy latents are copula draws, all common random numbers across policies.

| policy | what it is |
|---|---|
| COVER | coverage sweep by prior mass per second, report the first alert |
| FIXK k=1..4 | sweep, re-review an alerted cell k times (same class), report only if all alert |
| SPRT | Wald SPRT per cell on the iid per-look LLR, repeat looks, A in {1,2,3,4.5,6}, B = -1 |
| SPRT_DEFL / SPRT_DEFL_ALT | the same with each class's LLR deflated by 1 + (k-1) rho (ESS deflation); ALT alternates view classes |
| CI_THR | covariance-intersection style: each class contributes its mean per-look LLR, classes add |
| IND_BAYES | exact belief assuming independent looks (rb = 0), report at posterior >= theta (0.2-0.9, plus 0.95-0.999) |
| CORR | exact belief with the copula latent integrated on 24 Gauss-Hermite nodes (rb = rho); view class chosen by KL information per second; the analytic correlation-aware threshold policy |
| CORR_SAMEVIEW / CORR_PARTNER | CORR restricted to the own class / with the partner option |
| POMCP-lite | CORR threshold policy decides explore vs verify, then a 12-particle rollout (CRN) picks among report / review same / review other / partner; one-step improvement of a default rollout policy, NOT a UCT tree |
| ORACLE | knows the target (labelled) |
| Learned | designed only, section 9 |

Cost (measured, `Tools/Frontier/costs.jsonl`, sim `belief_svr`): COVER ~2 ms per mission, FIXK / SPRT 2-3 ms, IND_BAYES 4-5 ms, CORR 5-8 ms, POMCP-lite 20-40 ms, exact solves 1-30 s each. The DEV sweep (120 layouts x 2 reps x 24 conditions x 46 policies = 264,960 missions) took about 20 CPU-min; the whole D2 DEV study (including exact, tuning, three sweep extensions, reports, calibration) about 0.7 CPU-h on one slot (budget 3 CPU-h). Peak RSS under 50 MB.

## 4. Sweep (120 DEV layouts 100-219, 2 reps; rho {0, .3, .6, .8} x f {low, measured, high} x 2 regimes)

f levels (per-look background alert alpha, persistent decoy marginal fire fd, decoy density factor): low (0.01, 0.10, 1x), measured (0.05, 0.28, 1x), high (0.15, 0.50, 2x). measured uses the Vision persistent-fire 0.8 times the MissionSim per-view decoy visibility 0.35; alpha is a PRIOR downscale of the Vision upper bound. rho in the sweep is the within-view-class ICC; the measured Vision value 0.61 sits at the 0.6 column.

### 4.1 Verification-heavy regime, measured f, rho 0.6 (240 missions per policy; 95 % cluster bootstrap over 120 layouts)

| policy | success | T_cens s (misses = D) | false reports / mission | J = T + 300 FR | verifications per true report | miss causes (n missions) |
|---|---|---|---|---|---|---|
| COVER | 0.54 [0.47, 0.59] | 839 [775, 927] | 1.52 [1.32, 1.75] | 1297 | 0.0 | route 6, visibility 104 |
| FIXK1 | 0.49 [0.43, 0.56] | 907 [818, 989] | 0.45 [0.36, 0.55] | 1042 | 4.3 | association 4, inference 8, route 6, visibility 104 |
| FIXK2 | 0.45 [0.39, 0.50] | 964 [893, 1042] | 0.35 [0.27, 0.43] | 1068 | 7.0 | association 8, inference 15, route 6, visibility 104 |
| SPRT@3.0 | 0.52 [0.45, 0.57] | 886 [814, 965] | 0.63 [0.52, 0.73] | 1076 | 11.1 | association 3, inference 3, route 6, visibility 104 |
| SPRT_DEFL@2.0 | 0.53 [0.47, 0.60] | 853 [770, 941] | 1.37 [1.20, 1.58] | 1264 | 4.5 | association 1, inference 1, route 6, visibility 104 |
| SPRT_DEFL_ALT@2.0 | 0.52 [0.45, 0.58] | 872 [788, 965] | 1.32 [1.13, 1.50] | 1268 | 4.4 | association 3, inference 3, route 6, visibility 104 |
| CI_THR@2.0 | 0.52 [0.44, 0.58] | 874 [806, 970] | 1.32 [1.12, 1.48] | 1269 | 5.5 | association 3, inference 3, route 6, visibility 104 |
| IND_BAYES@0.5 | 0.62 [0.55, 0.68] | 842 [764, 918] | 0.70 [0.58, 0.83] | 1053 | 10.7 | association 5, inference 6, route 6, visibility 74 |
| IND_BAYES@0.99 | 0.56 [0.50, 0.63] | 936 [860, 1008] | 0.53 [0.42, 0.68] | 1096 | 26.2 | association 14, inference 10, route 6, visibility 75 |
| CORR@0.35 | 0.71 [0.65, 0.77] | 808 [732, 884] | 0.79 [0.63, 0.95] | 1045 | 6.5 | association 2, inference 1, route 8, visibility 58 |
| CORR@0.5 | 0.62 [0.56, 0.69] | 899 [807, 982] | 0.60 [0.47, 0.72] | 1078 | 10.8 | association 4, inference 6, route 14, visibility 67 |
| CORR_SAMEVIEW@0.35 | 0.61 [0.55, 0.67] | 839 [770, 909] | 0.73 [0.60, 0.87] | 1058 | 7.5 | association 6, inference 4, route 6, visibility 78 |
| CORR_PARTNER@0.35 | 0.72 [0.67, 0.78] | 801 [735, 887] | 0.80 [0.65, 0.96] | 1040 | 6.0 | association 2, inference 1, route 7, visibility 56 |
| ORACLE | 0.93 [0.90, 0.97] | 147 [100, 199] | 0.00 [0.00, 0.00] | 147 | 9.0 | route 6, visibility 10 |

### 4.2 Coverage-bound regime, measured f, rho 0.6

| policy | success | T_cens s (misses = D) | false reports / mission | J = T + 300 FR | verifications per true report | miss causes (n missions) |
|---|---|---|---|---|---|---|
| COVER | 0.36 [0.29, 0.43] | 730 [689, 768] | 2.86 [2.61, 3.12] | 1589 | 0.0 | route 92, visibility 61 |
| FIXK1 | 0.31 [0.25, 0.38] | 756 [724, 793] | 0.84 [0.72, 0.96] | 1008 | 10.2 | association 3, inference 5, route 96, visibility 61 |
| FIXK2 | 0.27 [0.21, 0.33] | 779 [741, 807] | 0.65 [0.57, 0.76] | 974 | 16.5 | association 7, inference 10, route 98, visibility 61 |
| SPRT@3.0 | 0.33 [0.26, 0.40] | 750 [709, 793] | 1.07 [0.91, 1.21] | 1070 | 27.3 | association 1, route 102, visibility 58 |
| SPRT_DEFL@2.0 | 0.33 [0.27, 0.39] | 746 [704, 783] | 2.50 [2.28, 2.78] | 1494 | 12.4 | association 3, route 99, visibility 59 |
| CI_THR@2.0 | 0.33 [0.26, 0.39] | 753 [720, 790] | 2.38 [2.14, 2.65] | 1468 | 10.1 | association 2, inference 1, route 100, visibility 58 |
| IND_BAYES@0.5 | 0.28 [0.21, 0.34] | 779 [743, 811] | 0.85 [0.72, 0.99] | 1033 | 36.2 | association 4, inference 5, route 111, visibility 54 |
| IND_BAYES@0.99 | 0.23 [0.17, 0.29] | 812 [782, 837] | 0.66 [0.55, 0.78] | 1009 | 66.4 | association 5, inference 11, route 118, visibility 51 |
| CORR@0.35 | 0.22 [0.16, 0.28] | 800 [764, 829] | 0.47 [0.39, 0.56] | 940 | 40.0 | association 5, inference 9, route 123, visibility 50 |
| CORR@0.5 | 0.16 [0.11, 0.22] | 821 [787, 850] | 0.23 [0.16, 0.29] | 888 | 60.0 | association 4, inference 19, route 133, visibility 45 |
| CORR_PARTNER@0.35 | 0.23 [0.17, 0.30] | 793 [754, 820] | 0.47 [0.39, 0.56] | 934 | 30.0 | association 3, inference 9, route 123, visibility 49 |
| ORACLE | 0.95 [0.93, 0.98] | 93 [70, 118] | 0.00 [0.00, 0.00] | 93 | 6.2 | visibility 11 |

## 5. Hypothesis tables (all conditions; best grid point per family at lambda = 300 s per false report; gains are relative J reductions; 95 % cluster bootstrap CIs, best point re-selected per resample)

"cheap-fix J above best" = J(best of deflated SPRT, deflated alternating SPRT, CI threshold) / J(best of everything else) - 1. Grids for the SPRT-type rules are coarse (A in {1, 2, 3, 4.5, 6}, jumps in FR of about 4x between A = 2 and A = 3), so their 8-16 % deficit is partly grid resolution; a finer A grid would shrink it. IND_BAYES carries an extended theta grid (to 0.999).

### Verification-heavy regime

| regime | f | rho | H1 FR inflation at matched T | J gain vs FIXK | J gain vs IND_BAYES | J gain vs deflated SPRT / CI | cheap-fix J above best | view-class choice | partner |
|---|---|---|---|---|---|---|---|---|---|
| verification | low | 0.0 | 1.00 [1.00, 1.00] | 31% [26, 37] | 0% [-1, 0] | 16% [12, 20] | 19% [14, 25] | 0% [0, 0] | 0% [0, 0] |
| verification | low | 0.3 | 1.50 [1.09, 2.12] | 21% [17, 27] | 3% [-0, 6] | 13% [9, 17] | 15% [10, 21] | 6% [3, 9] | -0% [-1, 1] |
| verification | low | 0.6 | 1.43 [0.79, 2.08] | 10% [6, 15] | 5% [-2, 10] | 12% [8, 15] | 15% [11, 20] | 5% [-0, 9] | 2% [0, 4] |
| verification | low | 0.8 | 1.01 [0.74, 1.55] | 0% [-4, 5] | 1% [-4, 5] | 6% [2, 10] | 7% [5, 12] | -1% [-5, 4] | 0% [0, 4] |
| verification | measured | 0.0 | 1.00 [1.00, 1.00] | 20% [13, 26] | 0% [0, 0] | 5% [1, 10] | 6% [1, 11] | 0% [0, 0] | 0% [0, 0] |
| verification | measured | 0.3 | 1.20 [0.97, 1.48] | 7% [1, 13] | -1% [-4, 3] | 4% [-1, 9] | 5% [1, 10] | -0% [-3, 3] | -0% [-2, 1] |
| verification | measured | 0.6 | 1.06 [0.85, 1.37] | -0% [-5, 4] | -1% [-5, 4] | 8% [2, 13] | 10% [5, 17] | -0% [-4, 4] | 1% [-0, 2] |
| verification | measured | 0.8 | 1.21 [1.04, 1.48] | -1% [-5, 3] | 2% [-2, 6] | 13% [6, 19] | 16% [10, 25] | 3% [-1, 6] | -0% [-0, 3] |
| verification | high | 0.0 | 1.00 [1.00, 1.00] | -0% [-6, 6] | 0% [0, 0] | -8% [-13, -3] | 0% [-1, 0] | 0% [0, 0] | 0% [0, 0] |
| verification | high | 0.3 | 1.45 [1.15, 1.91] | -3% [-10, 1] | 8% [1, 10] | 1% [-3, 3] | 4% [0, 8] | -0% [-2, 2] | -0% [-2, 1] |
| verification | high | 0.6 | 1.23 [1.05, 1.60] | -0% [-8, 8] | 7% [-0, 11] | 0% [-2, 3] | 1% [-1, 9] | 0% [-1, 2] | 1% [-1, 2] |
| verification | high | 0.8 | 1.39 [1.19, 1.67] | -0% [-7, 8] | 8% [2, 12] | 3% [-0, 7] | 5% [1, 13] | 1% [-2, 4] | 1% [-1, 3] |

### Coverage-bound regime

| regime | f | rho | H1 FR inflation at matched T | J gain vs FIXK | J gain vs IND_BAYES | J gain vs deflated SPRT / CI | cheap-fix J above best | view-class choice | partner |
|---|---|---|---|---|---|---|---|---|---|
| coverage | low | 0.0 | 1.00 [1.00, 1.00] | 5% [2, 9] | 0% [-1, 0] | 1% [-2, 3] | 1% [0, 4] | 0% [0, 0] | 0% [0, 0] |
| coverage | low | 0.3 | 0.53 [0.27, 1.22] | 2% [-1, 5] | 1% [-2, 4] | 0% [-2, 2] | 0% [-1, 3] | 4% [1, 5] | -1% [-1, 1] |
| coverage | low | 0.6 | 1.28 [0.82, 2.01] | 3% [-1, 5] | 4% [1, 6] | -2% [-4, 0] | -1% [-3, 1] | 4% [1, 7] | 0% [-1, 2] |
| coverage | low | 0.8 | 1.07 [0.71, 1.93] | 5% [1, 8] | 6% [3, 9] | -2% [-4, 0] | -2% [-4, 1] | 7% [4, 10] | -0% [-1, 1] |
| coverage | measured | 0.0 | 1.00 [1.00, 1.00] | 2% [-2, 6] | 0% [-0, 0] | 3% [0, 6] | 3% [0, 7] | 0% [0, 0] | 0% [0, 0] |
| coverage | measured | 0.3 | 1.35 [0.92, 2.32] | 1% [-1, 3] | 8% [5, 12] | 1% [-1, 2] | 1% [-0, 3] | 3% [1, 5] | 1% [-1, 2] |
| coverage | measured | 0.6 | 1.67 [1.24, 2.25] | 5% [2, 9] | 12% [9, 15] | -0% [-2, 2] | -0% [-1, 3] | 2% [0, 6] | 0% [-2, 2] |
| coverage | measured | 0.8 | 1.69 [1.35, 2.45] | 11% [8, 15] | 14% [11, 18] | 1% [-2, 2] | 1% [-2, 3] | 4% [1, 9] | -0% [-2, 2] |
| coverage | high | 0.0 | 1.00 [1.00, 1.00] | -7% [-11, -3] | -6% [-8, -4] | -12% [-15, -8] | -4% [-6, -1] | 0% [0, 0] | 0% [0, 0] |
| coverage | high | 0.3 | 3.95 [1.51, 17.08] | 24% [20, 28] | 9% [7, 11] | -0% [-1, 1] | -0% [-1, 1] | 0% [0, 2] | 0% [-1, 0] |
| coverage | high | 0.6 | 1.17 [1.07, 14.60] | 37% [33, 40] | 12% [9, 14] | -0% [-0, 0] | -0% [-0, 0] | 0% [0, 0] | -0% [-0, 0] |
| coverage | high | 0.8 | 2.61 [1.68, 4.58] | 43% [39, 46] | 13% [10, 15] | -1% [-2, 0] | -1% [-1, 0] | -0% [-1, 1] | 0% [-0, 0] |

### 5.1 Partner delay (verification-heavy, rho 0.6, measured f, 60 layouts)

DELAY (verification regime, rho 0.6, f measured): best-grid J = T_cens + 300 FR; gain of partner over solo CORR
  verification_d0=10: J CORR 1024, J PARTNER 1021  paired gain 0.0% [-0.7, 1.1]  (n layouts 60, same grid point pairs CORR@0.35 vs CORR_PARTNER@0.35)
  verification_d0=25: J CORR 1024, J PARTNER 1025  paired gain -0.1% [-0.4, 0.1]  (n layouts 60, same grid point pairs CORR@0.35 vs CORR_PARTNER@0.35)
  verification_d0=60: J CORR 1024, J PARTNER 1022  paired gain 0.1% [0.0, 0.3]  (n layouts 60, same grid point pairs CORR@0.35 vs CORR_PARTNER@0.35)
  verification_d0=120: J CORR 1024, J PARTNER 1023  paired gain 0.0% [0.0, 0.1]  (n layouts 60, same grid point pairs CORR@0.35 vs CORR_PARTNER@0.35)
POMCP-lite vs analytic (measured f, both regimes; first 60 layouts)
  coverage rho=0.0: POMCP 940[869,999], CORR 840[788,887], CORR_PARTNER 840[790,883], IND 840[784,885], FIXK 865[824,888], cheap 867[802,924], COVER 1580[1424,1722]
  coverage rho=0.3: POMCP 999[924,1053], CORR 892[860,902], CORR_PARTNER 879[848,904], IND 944[893,993], FIXK 875[834,907], cheap 887[851,910], COVER 1616[1480,1772]
  coverage rho=0.6: POMCP 992[905,1078], CORR 900[850,910], CORR_PARTNER 888[841,905], IND 1002[929,1066], FIXK 936[866,1004], cheap 892[856,900], COVER 1598[1441,1762]
  coverage rho=0.8: POMCP 976[893,1046], CORR 875[817,918], CORR_PARTNER 882[841,921], IND 1001[927,1068], FIXK 962[883,1028], cheap 878[825,900], COVER 1541[1401,1714]
  verification rho=0.0: POMCP 1025[910,1173], CORR 924[816,1046], CORR_PARTNER 924[814,1032], IND 924[830,1047], FIXK 1180[1063,1289], cheap 977[848,1096], COVER 1345[1171,1521]
  verification rho=0.3: POMCP 1081[905,1242], CORR 1026[867,1157], CORR_PARTNER 1020[880,1142], IND 999[869,1146], FIXK 1073[945,1184], cheap 1058[932,1181], COVER 1319[1137,1487]
  verification rho=0.6: POMCP 1031[875,1186], CORR 1024[865,1177], CORR_PARTNER 1025[884,1165], IND 998[844,1135], FIXK 1027[889,1143], cheap 1109[976,1210], COVER 1267[1087,1455]
  verification rho=0.8: POMCP 988[820,1128], CORR 980[832,1134], CORR_PARTNER 977[823,1096], IND 972[829,1121], FIXK 972[840,1110], cheap 1140[1006,1256], COVER 1243[1071,1434]

Partner gain is within +-1 % at every delay (10 to 120 s) because the partner look is rarely the cheapest information per second once the own-view option and the report threshold are tuned; the exact result in (c) used a partner look that is free for the drone and whose cost is only the delay. POMCP-lite is worse than the analytic policy it refines (J 940-1000 vs 840-900 in the coverage regime; CIs overlap with IND_BAYES at rho >= 0.3): the one-step rollout improvement is dominated by the mismatch between the default rollout policy and the real threshold rule. It is reported, not tuned further.

## 6. Kill check

K1 (FR inflation < 1.3 at measured rho 0.6): verification-heavy 1.06 [0.85, 1.37] (fires), coverage-bound 1.67 [1.24, 2.25] (does not). K2 (a cheap fix within 5 % of the best): coverage-bound, the deflated SPRT / CI rules are within 1 % (fires); verification-heavy, they are 10 % above the best (does not fire for them) but a tuned IND_BAYES threshold is within 1 % of CORR (fires in practice). Therefore: **at measured rho and measured f the decision-level case for correlation-aware selection is not established at mission scale; the exact decision structure is.** The only pocket with a detectable correlation-aware advantage is low f in the verification-heavy regime (3-6 % over the best other rule, CIs reaching 0), which the Vision decoy-rate measurement decides (section 10).

## 7. Calibration of the stated report posterior (H0 sanity check; DEV layouts 100-139, rho 0.6, f measured, 80 missions per cell)

| regime | belief | n reports | mean stated posterior | realised precision | by bin [stated -> realised (n)] |
|---|---|---|---|---|---|
| coverage | IND_BAYES (rb=0) | 83 | 0.61 | 0.19 | 0.55 -> 0.07 (42); 0.67 -> 0.32 (41) |
| coverage | CORR (rb=rho) | 30 | 0.55 | 0.33 | 0.53 -> 0.40 (25); 0.64 -> 0.00 (5) |
| coverage | IND_BAYES theta 0.9 | 75 | 0.93 | 0.20 | 0.93 -> 0.20 (75) |
| coverage | CORR theta 0.9 | 2 | 0.92 | 0.00 | 0.92 -> 0.00 (2) |
| verification | IND_BAYES (rb=0) | 94 | 0.65 | 0.52 | 0.57 -> 0.32 (22); 0.67 -> 0.58 (72) |
| verification | CORR (rb=rho) | 91 | 0.58 | 0.49 | 0.54 -> 0.40 (60); 0.65 -> 0.68 (31) |
| verification | IND_BAYES theta 0.9 | 85 | 0.93 | 0.54 | 0.93 -> 0.54 (85) |
| verification | CORR theta 0.9 | 22 | 0.91 | 0.55 | 0.91 -> 0.55 (22) |

Both beliefs over-claim at scale, the independence belief by more at the 0.5 threshold, and at theta = 0.9 equally (0.93 stated, about 0.2 realised in the coverage regime, 0.54 in the verification regime). The reason is not only correlation: the belief uses a map-based mean target TPR and a known decoy fire rate, while the truth draws a per-person occlusion (heterogeneous target visibility) and the report goes to the argmax cell over 30-100 cells (winner's curse). So the correlation fix explains part of the over-claim (the exact examples isolate it); at scale model misspecification and selection dominate. This is also why the deflated and tuned rules do as well as the exact copula belief.

## 8. Failure analysis (miss-cause labels, all 240 missions per cell)

Labels: route (target cell never looked at), visibility (looked at, never alerted: bad view), inference (alerted but never reported by D), association (a false report happened and the true report did not follow), communication (partner answer pending at the deadline).

- Verification-heavy, measured f, rho 0.6: COVER misses 110 of 240 (visibility 104, route 6); FIXK2 133 (adds inference 15, association 8); CORR@0.35 misses 69 (visibility 58, route 8, association 2, inference 1) and ORACLE 17 (visibility 10, route 6). Most misses are the target's bad view: not fixable by verification, only by re-viewing a cell not yet alerted, which no policy here does proactively (limitation).
- Coverage-bound: COVER route 92 + visibility 61 (of 240); CORR trades coverage for verification and its misses shift to route (133 at theta 0.5) with fewer false reports (0.23 vs 2.86 per mission); this is the coverage-bound regime's null result in the RoleSpec sense.
- At high theta (CORR@0.9) inference misses dominate (93 of 240): the correlation-aware belief rarely reaches 0.9 within the deadline because repeat alerts saturate at about 0.93 in the exact calibration.
- False reports are mostly decoys: of the 1649 false reports of all IND_BAYES grid points in the verification regime at measured f, rho 0.6, 1121 (68 %) are decoys and 32 % background alerts; CORR 585 of 943 (62 %).

## 9. Learned policies: design only (nothing trained, nothing submitted)

Jobs are emitted but never submitted by `bs_learned_jobs.py` into `jobs/d2_learned_jobs.json` (18 rc-job/1 entries; the training scripts `bs_learn_bc.py` and `bs_learn_ppo.py` named in the commands do not exist yet).
1. `d2_exact_dataset` (CPU, fastsim, 288 instances in 16 shards of 18, 30 s per instance): random tiny instances (n <= 6, 2 classes, H <= 6; rho, pT, fd, pid, s, d drawn from the DEV grids), expectimax state-action pairs; features = per-cell posterior under rb = rho, per-(cell, class) alert / miss counts, elapsed time, ruled-out mask, partner pending flag; label = optimal action.
2. `d2-bc-mlp` (GPU, vram 2 GB, 900 s, preemptible, checkpoint each epoch, `--resume`): behaviour cloning with a 128-unit MLP over the belief features; cross-entropy on actions, loss weighted by the Q-gap between best and second best action.
3. `d2-ppo-belief` (GPU, vram 4 GB, 4 h, depends on 2): PPO fine-tune in `bs_sim.py` DEV layouts 100-219 with reward -time - c_FR per false report, the BC policy as init and as a KL anchor; evaluated by the same section-5 tables. Held-out evaluation only after PREREG_BELIEF is frozen.
Success criterion set now: beats the best analytic family by >= 5 % J with a CI excluding 0 at measured rho and f in the verification-heavy regime; otherwise do not run held-out.

## 10. What the Vision reduced rerun (16 Unity jobs) must tighten

Sensitivity from this DEV study (what moves the verdict):
1. **Decoy persistent-fire probability per view and decoy density (fd, pid).** The only pocket where correlation-aware selection wins (3-6 %) vanishes between f = low (fd 0.10) and f = measured (fd 0.28). Needed: +-0.10 absolute on fd, which is about 80 independent decoy-view units (a decoy seen from a view class counts once however many frames; frames within a view are correlated). 16 jobs cannot give 80 decoy-views unless each job renders >= 5 decoys from >= 1 class; plan the rerun scenes with decoys (NOMAD labels only the actor, so the measured false-alert rate is an upper bound).
2. **Cross-view-class correlation (assumed 0 here).** Every independent-view result (b) assumes decorrelation across classes. Needed: +-0.15 on the correlation between two view classes of the same target. Requires >= 2 distinct view classes per sequence (top_30m and top_50m in `RERUN_PLAN.md` are the cheapest pair).
3. **Within-class ICC rho** (0.61 from NOMAD, tau 0.5). The verdict is insensitive to rho in 0.3-0.8 (H1 1.06-1.21 in the verification regime), so a wide CI is acceptable. Simulation of the copula estimator (`icc_ci.py`, ANOVA ICC, 400 replicates): half-width at 16 sequences x 8 looks per class is 0.15-0.17 (0.18-0.22 at 4 looks); 32 sequences 0.11-0.13; 64 sequences 0.08-0.09; 128 sequences about 0.05-0.06. The 16-job rerun therefore pins rho only to about +-0.16, enough for this decision (a +-0.10 target needs about 64 sequences with 8 looks per class, i.e. 4 sequences per job on the same 16 jobs). Estimate it on detector OUTPUT (alert per frame), not on latent visibility, as NOVELTY_D2 warns.
4. **Per-person TPR heterogeneity** (logit scale spread of the target alert probability; section 7): the dominant source of over-claiming at scale. Needed: +-0.3 on the logit SD, which comes free from the same per-sequence alert rates.
5. **Background alert rate alpha per look** (PRIOR 0.05): lower priority (high-f cells differ by 2 % in J).

## 11. Limitations and open items

- Binary alerts only: the detector confidence score is unused (the Vision model offers a likelihood ratio on the score; using it is the obvious next step and likely helps the belief policies).
- One target; reports are instantaneous; the partner is a free independent look with delay d0, not a drone with its own search.
- The Vélez-style correlated path planner, Kress index, Jin Confirm assignment, Sandino POMDP-inspect and a UCT-tree POMCP baselines requested by NOVELTY_D2 are not implemented.
- The ESS-deflated SPRT / CI grids are coarse (section 5); K2 on a finer grid could move either way.
- Grid points were selected on the same DEV layouts they are compared on (optimism applies equally to all families and is bootstrapped, but absolute J are optimistic); the held-out run fixes the grids first.
- MissionExact handoff (orchestrator): `Tools/MissionExact/results/` still holds only `q3.json`; its q1/q2 runs (OOM) and the figures were not produced here and were not rerun (fsrun-quick lane contention); `bs_exact.py` is an independent, memory-light (<= 1.2 M states) formulation of the exact problem for this study, not an edit of MissionExact.
