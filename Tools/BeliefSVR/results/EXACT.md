# Exact minimal examples (MissionExact extension, `Tools/BeliefSVR/exact.py`)

Evidence label: IMPLEMENTED (synthetic; deterministic exact expectimax, no seeds). TPR/FPR levels are *inputs* chosen inside the range of `Tools/Vision/obs_model.py` (pass-level TPR ~0.6, decoy marginal fire 0.2, = the MissionSim/Vision 0.8 fire x 0.35 per-view visibility rounded down); they are not fitted here.

Model: one target among n cells; each non-target cell holds a persistent decoy w.p. pid; per (cell, view class) a Gaussian-copula latent shared by repeat looks, ICC rho (the Vision ICC); independent across view classes; decoys alert w.p. fd with the same ICC; empty cells alert iid w.p. alpha. Actions: look same view / look from the other own view class (+s steps), move, partner look (class 3, free for the drone, result after d steps), wait, report. J = E[time of the correct report] + (1-P(succ))(H+cmiss) + cFR E[#false reports]; a false report rules the cell out and the mission continues. Base instance: n=2, prior uniform, pT=pP=0.6, fd=0.2, pid=0.3, alpha=0.02, H=6, cFR=cmiss=40, s=1, d=2 (instance A).

## (a) An independence-assuming belief over-reports

Belief calibration: posterior that cell 0 is the target after k repeat alerts from one view class (base instance). Over-confidence = ratio of odds.

| rho | k alerts | P independent | P correlated (true) | odds inflation |
|---|---|---|---|---|
| 0.3 | 1 | 0.890 | 0.890 | 1.00 |
| 0.3 | 2 | 0.967 | 0.942 | 1.81 |
| 0.3 | 3 | 0.989 | 0.957 | 4.06 |
| 0.6 | 1 | 0.890 | 0.890 | 1.00 |
| 0.6 | 2 | 0.967 | 0.925 | 2.39 |
| 0.6 | 3 | 0.989 | 0.933 | 6.48 |
| 0.8 | 1 | 0.890 | 0.887 | 1.04 |
| 0.8 | 2 | 0.967 | 0.913 | 2.81 |
| 0.8 | 3 | 0.989 | 0.921 | 7.70 |

Exact optimal policy planned with the true rho vs planned assuming independence (rho_b=0) but run in the true-rho world:

| rho | J opt | J indep-planner | E[FR] opt | E[FR] indep | FR ratio | succ opt | succ indep | T|succ opt | T|succ indep |
|---|---|---|---|---|---|---|---|---|---|
| 0.0 | 6.54 | 6.54 | 0.095 | 0.095 | 1.00 | 1.000 | 1.000 | 2.73 | 2.73 |
| 0.3 | 7.77 | 8.18 | 0.114 | 0.139 | 1.22 | 1.000 | 1.000 | 3.21 | 2.63 |
| 0.6 | 8.14 | 9.83 | 0.137 | 0.183 | 1.33 | 1.000 | 1.000 | 2.66 | 2.52 |
| 0.8 | 8.30 | 10.75 | 0.141 | 0.207 | 1.47 | 1.000 | 1.000 | 2.66 | 2.47 |

Second instance B (n=2, pT=0.8, fd=0.2, pid=0.6, cFR=cmiss=40, H=6):

| rho | J opt | J indep-planner | E[FR] opt | E[FR] indep | FR ratio | succ opt | succ indep | T|succ opt | T|succ indep |
|---|---|---|---|---|---|---|---|---|---|
| 0.0 | 4.86 | 4.86 | 0.055 | 0.055 | 1.00 | 1.00 | 1.00 | 2.65 | 2.65 |
| 0.3 | 6.24 | 7.23 | 0.080 | 0.119 | 1.48 | 1.00 | 1.00 | 3.03 | 2.47 |
| 0.6 | 6.40 | 8.25 | 0.073 | 0.149 | 2.04 | 1.00 | 1.00 | 3.47 | 2.28 |
| 0.8 | 6.61 | 9.28 | 0.078 | 0.178 | 2.27 | 1.00 | 1.00 | 3.48 | 2.17 |

Fixed rule (look at the best cell from the same view, report when belief >= theta), instance B, true rho=0.6. T_cens = E[min(T_report, H)] counts unreported missions as H.

| theta | belief | E[FR] | P(succ) | T_cens |
|---|---|---|---|---|
| 0.9 | independent | 0.063 | 0.822 | 3.23 |
| 0.9 | correlated | 0.056 | 0.800 | 3.28 |
| 0.95 | independent | 0.062 | 0.820 | 3.25 |
| 0.95 | correlated | 0.013 | 0.362 | 4.92 |
| 0.97 | independent | 0.049 | 0.760 | 4.17 |
| 0.97 | correlated | 0.008 | 0.329 | 5.34 |
| 0.99 | independent | 0.048 | 0.748 | 4.17 |
| 0.99 | correlated | 0.000 | 0.000 | 6.00 |

## (b) Independent-view verification beats a repeat look

Gain = J(repeat/own-view-only) - J(full action set). n_indep / n_repeat are expected counts under the optimal policy. 'after alert' is the optimal action at the first-look-alert state.

| rho | s | J full | J repeat-only | gain | n_indep | n_repeat | E[FR] full | E[FR] repeat-only | after alert | after miss |
|---|---|---|---|---|---|---|---|---|---|---|
| 0.0 | 0 | 6.54 | 6.54 | 0.00 | 0.00 | 1.44 | 0.095 | 0.095 | ('R', 0) | ('L', 0, 1) |
| 0.0 | 1 | 6.54 | 6.54 | 0.00 | 0.00 | 1.44 | 0.095 | 0.095 | ('R', 0) | ('L', 0, 1) |
| 0.0 | 2 | 6.54 | 6.54 | 0.00 | 0.00 | 1.44 | 0.095 | 0.095 | ('R', 0) | ('L', 0, 1) |
| 0.3 | 0 | 7.01 | 8.17 | 1.16 | 0.89 | 0.51 | 0.110 | 0.139 | ('R', 0) | ('L', 1, 1) |
| 0.3 | 1 | 7.77 | 8.17 | 0.40 | 0.56 | 0.69 | 0.114 | 0.139 | ('R', 0) | ('L', 0, 1) |
| 0.3 | 2 | 8.17 | 8.17 | 0.00 | 0.00 | 1.29 | 0.139 | 0.139 | ('R', 0) | ('L', 0, 1) |
| 0.6 | 0 | 7.44 | 9.76 | 2.32 | 0.67 | 0.03 | 0.136 | 0.199 | ('R', 0) | ('L', 1, 1) |
| 0.6 | 1 | 8.14 | 9.76 | 1.61 | 0.66 | 0.03 | 0.137 | 0.199 | ('R', 0) | ('L', 1, 2) |
| 0.6 | 2 | 8.82 | 9.76 | 0.94 | 0.66 | 0.00 | 0.138 | 0.199 | ('R', 0) | ('L', 1, 3) |
| 0.8 | 0 | 7.61 | 10.49 | 2.88 | 0.67 | 0.03 | 0.140 | 0.218 | ('R', 0) | ('L', 1, 1) |
| 0.8 | 1 | 8.30 | 10.49 | 2.19 | 0.66 | 0.03 | 0.141 | 0.218 | ('R', 0) | ('L', 1, 2) |
| 0.8 | 2 | 8.98 | 10.49 | 1.50 | 0.66 | 0.00 | 0.142 | 0.218 | ('R', 0) | ('L', 1, 3) |

In instance A the optimal move after one alert is to report (P=0.89); independent views are used after a MISS (a repeat miss from a bad view is nearly uninformative at high rho). Instance C (pT=0.6, fd=0.35, pid=0.6) is a case where an alert alone is not enough to report, so the verify-view choice is visible directly:

| rho | s | J full | J repeat-only | gain | n_indep | n_repeat | after alert |
|---|---|---|---|---|---|---|---|
| 0.0 | 0 | 11.21 | 11.21 | 0.00 | 0.00 | 1.95 | ('L', 0, 1) |
| 0.0 | 1 | 11.21 | 11.21 | 0.00 | 0.00 | 1.95 | ('L', 0, 1) |
| 0.0 | 2 | 11.21 | 11.21 | 0.00 | 0.00 | 1.95 | ('L', 0, 1) |
| 0.3 | 0 | 11.63 | 12.34 | 0.71 | 0.66 | 0.47 | ('R', 0) |
| 0.3 | 1 | 12.22 | 12.34 | 0.12 | 0.23 | 0.31 | ('R', 0) |
| 0.3 | 2 | 12.34 | 12.34 | 0.00 | 0.00 | 1.09 | ('R', 0) |
| 0.6 | 0 | 11.72 | 13.11 | 1.38 | 0.61 | 0.07 | ('R', 0) |
| 0.6 | 1 | 12.33 | 13.11 | 0.78 | 0.59 | 0.00 | ('R', 0) |
| 0.6 | 2 | 12.88 | 13.11 | 0.22 | 0.31 | 0.00 | ('R', 0) |
| 0.8 | 0 | 11.78 | 13.42 | 1.64 | 0.59 | 0.00 | ('R', 0) |
| 0.8 | 1 | 12.38 | 13.42 | 1.04 | 0.59 | 0.00 | ('R', 0) |
| 0.8 | 2 | 12.95 | 13.42 | 0.47 | 0.32 | 0.00 | ('R', 0) |

Prediction P3 (fraction of non-first same-cell looks that are independent-view, over rho and s):

| rho | s=0 | s=1 | s=2 |
|---|---|---|---|
| 0.0 | 0.00 | 0.00 | 0.00 |
| 0.3 | 0.64 | 0.45 | 0.00 |
| 0.6 | 0.96 | 0.96 | 1.00 |
| 0.8 | 0.96 | 0.96 | 1.00 |

## (c) Partner verification vs solo verification as a function of delay

Substitution comparison. J_repeat: own repeat looks only. J_solo(s): own repeat + own independent view (cost s). J_partner(d): own repeat + partner look (class 3, free for the drone, result after d steps, TPR pP), NO own independent view. d* = largest delay at which partner-only is at least as good as solo (J_partner <= J_solo). 'both' = everything allowed (gain over J_solo).

| pP | s | J_repeat | J_solo | J_partner d=1 | d=2 | d=3 | d=4 | d=5 | d* | both(d=2) gain over solo |
|---|---|---|---|---|---|---|---|---|---|---|
| 0.6 | 0 | 9.76 | 7.44 | 5.52 | 6.71 | 7.30 | 7.74 | 7.90 | 3 | 1.94 |
| 0.6 | 1 | 9.76 | 8.14 | 5.52 | 6.71 | 7.30 | 7.74 | 7.90 | 5 | 2.04 |
| 0.6 | 2 | 9.76 | 8.82 | 5.52 | 6.71 | 7.30 | 7.74 | 7.90 | 5 | 2.22 |
| 0.8 | 0 | 9.76 | 7.44 | 3.69 | 4.94 | 6.07 | 6.59 | 6.90 | 5 | 3.13 |
| 0.8 | 1 | 9.76 | 8.14 | 3.69 | 4.94 | 6.07 | 6.59 | 6.90 | 5 | 3.46 |
| 0.8 | 2 | 9.76 | 8.82 | 3.69 | 4.94 | 6.07 | 6.59 | 6.90 | 5 | 3.88 |

Partner-only gain over repeat-only vs rho (pP=0.6, s=1; compare with the solo gain from (b)):

| rho | solo gain (s=1) | partner gain d=1 | d=2 | d=3 | d=4 |
|---|---|---|---|---|---|
| 0.0 | 0.00 | 2.73 | 1.57 | 0.91 | 0.37 |
| 0.3 | 0.40 | 3.49 | 2.28 | 1.62 | 0.97 |
| 0.6 | 1.61 | 4.23 | 3.05 | 2.46 | 2.01 |
| 0.8 | 2.19 | 4.33 | 3.32 | 2.74 | 2.36 |

Wall time 83 s (CPU, single process).
