# Falsification sweep of the lead claim (greedy + shared default plan: comms add nothing over IMPLICIT)

Code: ../falsify.py (new; subclass FEpisode of HEpisode; no existing file edited). Link d0_loss0, lam=1 base event rates unless stated.
Paired diffs vs IMPLICIT, 95% cluster bootstrap (instances, then episodes). Hit rule: >= +0.03 and CI > 0 for ORACLE or HYBRID_P.
Screen = seeds 500-503 x 200 episodes. Confirm = seeds 504-511 x 200 episodes (only the screen hits).

Result: claim survives in regimes 1-6 and 8; it FAILS under permanent agent loss (R7a-d), confirmed on 8 held-out seeds.

Caveats on the loss model: the lost agent's sensor dies (every later look aborts) but its radio stays up, so HYBRID_P learns of it
from ABORT records. ORACLE is also told the partner is dead (drops the partner's in-flight look as virtual). A full radio-and-sensor
loss was not tested. Under persistent degradation (R6) the same ORACLE information gives no detectable gain.

Raw output (screen first, then confirm):


seeds [500, 501, 502, 503] episodes 200

| id | regime | IMPLICIT P | ORACLE-IMPL | HYBRID_P-IMPL | INDEP-IMPL |
|---|---|---|---|---|---|
| R0 | baseline 5x5 t0 H16 | 0.701 | -0.021 [-0.044,+0.004] | -0.021 [-0.044,+0.004] | -0.071 [-0.107,-0.036] |
| R1a4 | short horizon t0 H4 | 0.355 | +0.005 [-0.013,+0.024] | +0.005 [-0.013,+0.024] | -0.129 [-0.171,-0.086] |
| R1a8 | short horizon t0 H8 | 0.545 | -0.005 [-0.029,+0.016] | -0.005 [-0.029,+0.016] | -0.107 [-0.154,-0.058] |
| R1b8 | short horizon travel1 H8 | 0.269 | +0.000 [-0.004,+0.004] | +0.000 [-0.004,+0.004] | -0.026 [-0.052,-0.004] |
| R1b14 | short horizon travel1 H14 | 0.424 | +0.003 [-0.015,+0.021] | +0.003 [-0.015,+0.021] | -0.051 [-0.084,-0.020] |
| R2a | peaked prior (^3) t0 H12 | 0.719 | +0.016 [-0.004,+0.035] | +0.016 [-0.004,+0.035] | -0.066 [-0.121,-0.019] |
| R2b | sparse prior (3 cells) t1 H20 | 0.835 | +0.001 [+0.000,+0.005] | +0.001 [+0.000,+0.005] | -0.005 [-0.013,+0.000] |
| R3a | high hint+false hint t0 H16 | 0.749 | +0.005 [-0.019,+0.031] | +0.005 [-0.019,+0.031] | -0.114 [-0.154,-0.077] |
| R3b | high hint+false hint t1 H30 | 0.728 | +0.001 [-0.018,+0.019] | +0.001 [-0.018,+0.018] | -0.096 [-0.151,-0.042] |
| R4a | 8x8 travel1 H40 | 0.512 | -0.013 [-0.034,+0.005] | -0.013 [-0.034,+0.005] | -0.089 [-0.122,-0.055] |
| R4b | 10x10 travel1 H50 | 0.471 | +0.004 [-0.014,+0.022] | +0.004 [-0.014,+0.022] | -0.049 [-0.085,-0.019] |
| R5 | asym starts 8x8 t1 H40 (both at far corner) | 0.477 | +0.005 [-0.020,+0.030] | +0.003 [-0.022,+0.026] | -0.100 [-0.133,-0.069] |
| R5b | asym starts 8x8 t2 H60 (0 at 0, 1 at 63) | 0.520 | -0.004 [-0.016,+0.009] | -0.004 [-0.016,+0.009] | -0.084 [-0.142,-0.026] |
| R6a | persistent degraded sensor t0 H16 | 0.561 | +0.011 [-0.021,+0.045] | +0.022 [-0.006,+0.051] | -0.046 [-0.098,+0.005] |
| R6b | persistent degraded sensor t1 H30 | 0.499 | -0.013 [-0.041,+0.015] | -0.005 [-0.033,+0.021] | -0.033 [-0.072,+0.009] |
| R7a | agent loss t0 H16 | 0.534 | +0.055 [+0.014,+0.095] | +0.044 [+0.006,+0.085] | +0.029 [-0.020,+0.077] | **HIT**
| R7b | agent loss t1 H30 | 0.484 | +0.039 [-0.001,+0.080] | +0.028 [-0.015,+0.070] | +0.025 [-0.030,+0.079] |
| R7c | agent 0 loss early (t in 1..4) t0 H16 | 0.456 | +0.124 [+0.077,+0.175] | +0.095 [+0.050,+0.139] | +0.085 [+0.044,+0.124] | **HIT**
| R7d | agent loss 8x8 t1 H40 | 0.372 | +0.029 [-0.010,+0.070] | +0.025 [-0.003,+0.052] | -0.026 [-0.064,+0.007] |
| R8a | low q=0.3 t0 H24 | 0.532 | +0.009 [-0.033,+0.050] | +0.009 [-0.033,+0.050] | -0.004 [-0.045,+0.036] |
| R8b | low q=0.15 t1 H40 | 0.477 | -0.006 [-0.040,+0.033] | -0.009 [-0.040,+0.029] | -0.037 [-0.084,+0.014] |

seeds [504, 505, 506, 507, 508, 509, 510, 511] episodes 200

| id | regime | IMPLICIT P | ORACLE-IMPL | HYBRID_P-IMPL | INDEP-IMPL |
|---|---|---|---|---|---|
| R7a | agent loss t0 H16 | 0.501 | +0.050 [+0.016,+0.083] | +0.042 [+0.006,+0.077] | +0.020 [-0.014,+0.054] | **HIT**
| R7b | agent loss t1 H30 | 0.477 | +0.039 [+0.013,+0.066] | +0.028 [+0.001,+0.054] | -0.001 [-0.033,+0.033] | **HIT**
| R7c | agent 0 loss early (t in 1..4) t0 H16 | 0.453 | +0.074 [+0.032,+0.111] | +0.054 [+0.019,+0.089] | +0.051 [+0.015,+0.088] | **HIT**
| R7d | agent loss 8x8 t1 H40 | 0.306 | +0.031 [+0.003,+0.061] | +0.029 [+0.001,+0.059] | +0.023 [-0.004,+0.055] | **HIT**
| R0 | baseline 5x5 t0 H16 | 0.679 | +0.000 [-0.020,+0.019] | +0.000 [-0.020,+0.019] | -0.096 [-0.126,-0.066] |
