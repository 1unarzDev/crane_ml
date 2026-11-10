# Preregistered hypotheses: held-out verdicts (seeds 200–207)

Paired cluster bootstrap with 2000 resamples (`analyze_curve.paired`). Rules are in `Tools/ForestSearch/PREREG.md`.

## 5x5, travel 0

| test | λ=0 | λ=1 | λ=2 | λ=3 | verdict |
|---|---|---|---|---|---|
| H1 ORACLE−IMPLICIT, d0_loss0 | +0.000 [+0.000,+0.000] | +0.008 [-0.009,+0.026] | +0.009 [-0.010,+0.030] | +0.010 [-0.008,+0.026] | PASS |
| H2 HYBRID−IMPLICIT, d1_loss0 | – | -0.043 [-0.062,-0.023] | -0.043 [-0.061,-0.023] | -0.049 [-0.069,-0.029] | PASS |
| H2 HYBRID−IMPLICIT, d1_burst30 | – | -0.035 [-0.049,-0.021] | -0.033 [-0.051,-0.017] | -0.036 [-0.054,-0.020] | PASS |
| H3 HYBRID_P−HYBRID, d1_loss0 | – | +0.038 [+0.018,+0.059] | +0.033 [+0.013,+0.055] | +0.043 [+0.028,+0.057] | PASS |
| (descr.) HYBRID_P−IMPLICIT, d1_loss0 | – | -0.005 [-0.019,+0.010] | -0.009 [-0.028,+0.010] | -0.006 [-0.021,+0.009] | – |
| H4 FULL−IMPLICIT, d1_loss0 | -0.076 [-0.099,-0.053] | – | – | – | PASS |
| H5 INDEP−IMPLICIT, d0_loss0 | -0.083 [-0.107,-0.057] | -0.077 [-0.105,-0.049] | -0.090 [-0.113,-0.067] | -0.129 [-0.155,-0.105] | PASS |

## 5x5, travel 1

| test | λ=0 | λ=1 | λ=2 | λ=3 | verdict |
|---|---|---|---|---|---|
| H1 ORACLE−IMPLICIT, d0_loss0 | +0.000 [+0.000,+0.000] | +0.000 [-0.010,+0.011] | -0.001 [-0.013,+0.011] | -0.005 [-0.018,+0.010] | PASS |
| H4 FULL−IMPLICIT, d1_loss0 | -0.025 [-0.057,+0.005] | – | – | – | FAIL |
| H5 INDEP−IMPLICIT, d0_loss0 | -0.070 [-0.092,-0.049] | -0.056 [-0.074,-0.038] | -0.065 [-0.085,-0.045] | -0.070 [-0.088,-0.050] | PASS |

## 6x6, travel 0

| test | λ=0 | λ=1 | λ=2 | λ=3 | verdict |
|---|---|---|---|---|---|
| H1 ORACLE−IMPLICIT, d0_loss0 | +0.000 [+0.000,+0.000] | +0.000 [-0.022,+0.020] | +0.014 [-0.003,+0.031] | -0.005 [-0.021,+0.013] | FAIL |
| H2 HYBRID−IMPLICIT, d1_loss0 | – | -0.048 [-0.074,-0.022] | -0.039 [-0.063,-0.016] | -0.063 [-0.086,-0.041] | PASS |
| H2 HYBRID−IMPLICIT, d1_burst30 | – | -0.030 [-0.055,-0.008] | -0.028 [-0.047,-0.011] | -0.051 [-0.070,-0.033] | PASS |
| H3 HYBRID_P−HYBRID, d1_loss0 | – | +0.045 [+0.026,+0.064] | +0.040 [+0.017,+0.062] | +0.054 [+0.032,+0.077] | PASS |
| (descr.) HYBRID_P−IMPLICIT, d1_loss0 | – | -0.003 [-0.023,+0.016] | +0.001 [-0.015,+0.018] | -0.009 [-0.025,+0.009] | – |
| H4 FULL−IMPLICIT, d1_loss0 | -0.085 [-0.121,-0.052] | – | – | – | PASS |
| H5 INDEP−IMPLICIT, d0_loss0 | -0.087 [-0.121,-0.057] | -0.087 [-0.118,-0.054] | -0.104 [-0.133,-0.077] | -0.153 [-0.178,-0.128] | PASS |

## 6x6, travel 1

| test | λ=0 | λ=1 | λ=2 | λ=3 | verdict |
|---|---|---|---|---|---|
| H1 ORACLE−IMPLICIT, d0_loss0 | +0.000 [+0.000,+0.000] | -0.006 [-0.018,+0.005] | +0.002 [-0.013,+0.015] | -0.003 [-0.015,+0.010] | PASS |
| H4 FULL−IMPLICIT, d1_loss0 | -0.019 [-0.038,-0.004] | – | – | – | PASS |
| H5 INDEP−IMPLICIT, d0_loss0 | -0.061 [-0.089,-0.035] | -0.062 [-0.085,-0.040] | -0.066 [-0.083,-0.050] | -0.079 [-0.105,-0.055] | PASS |

H1 decision rule: it fired (6x6 travel 0, λ=2). That cell is non-equivalent but shows no significant positive value. The rule conflated the two; this is disclosed in the paper.
