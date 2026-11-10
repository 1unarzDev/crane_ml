# Agent-loss result: multiplicity correction (fresh seeds 800-811)

Script: `../scripts/loss_multiplicity.py` (new; reads `beacon_final_A_raw.npy` only, edits nothing). Run time about 1 min, nice 19.
Data: 12 fresh instances (seeds 800-811) x 200 episodes, episodes paired by index across strategies. Contrasts: HEALTH - IMPLICIT and
HEALTH - INDEP in the four permanent-loss regimes R7a-d, ideal link (d0_loss0). Secondary: HEALTH - IMPLICIT on the bursty link d1_burst30.

Methods
- Instance-level paired t-test: the 12 per-instance mean paired differences, df = 11, two-sided.
- Cluster bootstrap: instances, then episodes (the falsify.boot scheme). The B=2000, rng-seed-1 CI reproduces the published tables exactly.
  Two-sided percentile p-value p = 2 min(P*(mean<=0), P*(mean>=0)) from a separate B = 20000 run. When no replicate crossed 0 the table
  prints 5e-05; read that as p < 1e-4.
- Holm and Benjamini-Hochberg (BH) adjustments within each 4-regime family ("Holm4", "BH4"), plus Holm over the combined 8 tests
  ("Holm8", i.e. both contrasts x 4 regimes). inst>0 = the number of instances with a positive mean difference.

## Results

### HEALTH - IMPLICIT, ideal link

| regime | d | boot 95% CI (B=2000) | t_11 95% CI | inst>0 | t | p_t | Holm4 p_t | BH4 p_t | p_boot | Holm4 p_boot | BH4 p_boot | Holm8 p_t | Holm8 p_boot |
|---|---|---|---|---|---|---|---|---|---|---|---|---|---|
| R7a | +0.049 | [+0.028,+0.068] | [+0.037,+0.061] | 12/12 | 8.85 | 2.5e-06 | 9.9e-06 | 9.9e-06 | 5e-05 | 0.0002 | 6.7e-05 | 2e-05 | 0.0004 |
| R7b | +0.061 | [+0.035,+0.089] | [+0.036,+0.087] | 12/12 | 5.30 | 0.00025 | 0.00051 | 0.00034 | 5e-05 | 0.0002 | 6.7e-05 | 0.0015 | 0.0004 |
| R7c | +0.085 | [+0.053,+0.115] | [+0.056,+0.115] | 11/12 | 6.42 | 4.9e-05 | 0.00015 | 9.9e-05 | 5e-05 | 0.0002 | 6.7e-05 | 0.00035 | 0.0004 |
| R7d | +0.028 | [+0.006,+0.050] | [+0.008,+0.048] | 11/12 | 3.11 | 0.01 | 0.01 | 0.01 | 0.017 | 0.017 | 0.017 | 0.02 | 0.033 |

### HEALTH - INDEP, ideal link

| regime | d | boot 95% CI (B=2000) | t_11 95% CI | inst>0 | t | p_t | Holm4 p_t | BH4 p_t | p_boot | Holm4 p_boot | BH4 p_boot | Holm8 p_t | Holm8 p_boot |
|---|---|---|---|---|---|---|---|---|---|---|---|---|---|
| R7a | +0.033 | [+0.010,+0.058] | [+0.014,+0.052] | 9/12 | 3.87 | 0.0026 | 0.0079 | 0.0035 | 0.0075 | 0.015 | 0.01 | 0.0099 | 0.022 |
| R7b | +0.037 | [+0.013,+0.061] | [+0.017,+0.058] | 10/12 | 3.99 | 0.0021 | 0.0079 | 0.0035 | 0.0017 | 0.0051 | 0.0034 | 0.0099 | 0.0068 |
| R7c | +0.028 | [+0.003,+0.053] | [+0.006,+0.049] | 9/12 | 2.83 | 0.016 | 0.016 | 0.016 | 0.038 | 0.038 | 0.038 | 0.02 | 0.038 |
| R7d | +0.030 | [+0.011,+0.050] | [+0.014,+0.047] | 11/12 | 4.03 | 0.002 | 0.0079 | 0.0035 | 0.0008 | 0.0032 | 0.0032 | 0.0099 | 0.004 |

### HEALTH - IMPLICIT, bursty link d1_burst30 (secondary)

| regime | d | boot 95% CI (B=2000) | t_11 95% CI | inst>0 | t | p_t | Holm4 p_t | BH4 p_t | p_boot | Holm4 p_boot | BH4 p_boot |
|---|---|---|---|---|---|---|---|---|---|---|---|
| R7a | +0.035 | [+0.016,+0.052] | [+0.023,+0.046] | 11/12 | 6.79 | 3e-05 | 0.00012 | 0.00012 | 0.0001 | 0.0002 | 0.00013 |
| R7b | +0.046 | [+0.024,+0.068] | [+0.025,+0.068] | 11/12 | 4.71 | 0.00064 | 0.0013 | 0.00085 | 5e-05 | 0.0002 | 0.0001 |
| R7c | +0.055 | [+0.030,+0.077] | [+0.035,+0.075] | 11/12 | 6.11 | 7.7e-05 | 0.00023 | 0.00015 | 5e-05 | 0.0002 | 0.0001 |
| R7d | +0.021 | [+0.003,+0.041] | [+0.004,+0.039] | 10/12 | 2.64 | 0.023 | 0.023 | 0.023 | 0.029 | 0.029 | 0.029 |

## Verdict

- All 8 primary tests survive Holm and BH at 0.05, under both the t-test and the bootstrap, within each 4-regime family and over the
  combined 8. HEALTH - IMPLICIT is robust: Holm-adjusted p <= 0.017 in every regime, and the effect is positive on 11-12 of 12 instances.
- HEALTH - INDEP is the weakest claim. Its largest adjusted p is in R7c (early loss): t Holm 0.016, bootstrap Holm 0.038, and only
  9/12 instances are positive (also 9/12 in R7a). It passes, but with little margin. If any further contrasts reported in the same section
  (the hedge comparisons, the bursty-link rows) were added to the family, R7c could cross 0.05 under the bootstrap.
- Secondary (bursty link): HEALTH - IMPLICIT survives Holm in all four regimes (largest adjusted p 0.029, R7d).
- Caveats: this is a post hoc correction of an exploratory analysis, not a preregistered test. With 12 clusters the t and bootstrap
  p-values agree to within a factor of about 2-3, which is reassuring, but tail p-values below 1e-3 should not be read literally.
