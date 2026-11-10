
## Part C: seeds 800-811, 100 episodes/cell


### trans_m

| link | strategy | P(detect) | d vs IMPLICIT | d vs HEALTH | bytes/ep |
|---|---|---|---|---|---|
| d0_loss0 | IMPLICIT | 0.652 | - | +0.028 [+0.003,+0.051]* | 0.0 |
| d0_loss0 | HEALTH | 0.624 | -0.028 [-0.051,-0.003]* | - | 8.9 |
| d0_loss0 | HB_TO | 0.632 | -0.019 [-0.042,+0.002] | +0.008 [-0.003,+0.020] | 80.7 |
| d0_loss0 | DEV | 0.642 | -0.009 [-0.043,+0.021] | +0.018 [-0.008,+0.045] | 54.7 |
| d0_loss0 | DEV_SD | 0.641 | -0.011 [-0.043,+0.017] | +0.017 [-0.008,+0.043] | 40.8 |
| d0_loss0 | DEV_SD_B12 | 0.628 | -0.023 [-0.054,+0.004] | +0.004 [-0.021,+0.030] | 16.6 |

Lost-agent frames (episodes that reached the failure): 
| link | strategy | frames before death | frames after death | P(any post-death frame) | median lag of 1st (steps) | partner frames |
|---|---|---|---|---|---|---|
| d0_loss0 | DEV | 0.35 | 3.97 | 0.87 | 1 | 1.69 |
| d0_loss0 | DEV_SD | 0.35 | 2.40 | 0.91 | 0 | 1.69 |
| d0_loss0 | DEV_SD_B12 | 0.31 | 0.63 | 0.63 | 0 | 0.80 |

### intermit

| link | strategy | P(detect) | d vs IMPLICIT | d vs HEALTH | bytes/ep |
|---|---|---|---|---|---|
| d0_loss0 | IMPLICIT | 0.656 | - | +0.045 [+0.018,+0.073]* | 0.0 |
| d0_loss0 | HEALTH | 0.611 | -0.045 [-0.073,-0.018]* | - | 9.7 |
| d0_loss0 | HB_TO | 0.630 | -0.026 [-0.052,+0.001] | +0.019 [+0.003,+0.036]* | 93.9 |
| d0_loss0 | DEV | 0.642 | -0.014 [-0.048,+0.017] | +0.031 [+0.000,+0.063] | 56.4 |
| d0_loss0 | DEV_SD | 0.649 | -0.007 [-0.045,+0.028] | +0.038 [+0.008,+0.069]* | 58.1 |
| d0_loss0 | DEV_SD_B12 | 0.623 | -0.033 [-0.065,-0.004]* | +0.012 [-0.015,+0.036] | 17.0 |

Lost-agent frames (episodes that reached the failure): 
| link | strategy | frames before death | frames after death | P(any post-death frame) | median lag of 1st (steps) | partner frames |
|---|---|---|---|---|---|---|
| d0_loss0 | DEV | 0.24 | 3.91 | 0.86 | 1 | 1.55 |
| d0_loss0 | DEV_SD | 0.24 | 4.09 | 0.90 | 0 | 1.55 |
| d0_loss0 | DEV_SD_B12 | 0.24 | 0.68 | 0.68 | 0 | 0.76 |

### total

| link | strategy | P(detect) | d vs IMPLICIT | d vs HEALTH | bytes/ep |
|---|---|---|---|---|---|
| d0_loss0 | IMPLICIT | 0.525 | - | +0.000 [+0.000,+0.000] | 0.0 |
| d0_loss0 | HEALTH | 0.525 | +0.000 [+0.000,+0.000] | - | 0.0 |
| d0_loss0 | HB_TO | 0.565 | +0.040 [+0.016,+0.066]* | +0.040 [+0.016,+0.066]* | 43.4 |
| d0_loss0 | DEV | 0.512 | -0.013 [-0.048,+0.019] | -0.013 [-0.048,+0.019] | 20.7 |
| d0_loss0 | DEV_SD | 0.512 | -0.013 [-0.048,+0.019] | -0.013 [-0.048,+0.019] | 20.7 |
| d0_loss0 | DEV_SD_B12 | 0.527 | +0.003 [-0.032,+0.033] | +0.003 [-0.032,+0.033] | 11.1 |

Lost-agent frames (episodes that reached the failure): 
| link | strategy | frames before death | frames after death | P(any post-death frame) | median lag of 1st (steps) | partner frames |
|---|---|---|---|---|---|---|
| d0_loss0 | DEV | 0.35 | 0.00 | 0.00 | nan | 1.82 |
| d0_loss0 | DEV_SD | 0.35 | 0.00 | 0.00 | nan | 1.82 |
| d0_loss0 | DEV_SD_B12 | 0.31 | 0.00 | 0.00 | nan | 0.81 |

### beacon_lost

| link | strategy | P(detect) | d vs IMPLICIT | d vs HEALTH | bytes/ep |
|---|---|---|---|---|---|
| d0_loss0 | IMPLICIT | 0.525 | - | +0.000 [+0.000,+0.000] | 0.0 |
| d0_loss0 | HEALTH | 0.525 | +0.000 [+0.000,+0.000] | - | 8.9 |
| d0_loss0 | HB_TO | 0.562 | +0.037 [+0.008,+0.068]* | +0.037 [+0.008,+0.068]* | 79.1 |
| d0_loss0 | DEV | 0.550 | +0.025 [-0.007,+0.058] | +0.025 [-0.007,+0.058] | 97.9 |
| d0_loss0 | DEV_SD | 0.518 | -0.007 [-0.039,+0.023] | -0.007 [-0.039,+0.023] | 29.8 |
| d0_loss0 | DEV_SD_B12 | 0.527 | +0.003 [-0.032,+0.033] | +0.003 [-0.032,+0.033] | 16.8 |

Lost-agent frames (episodes that reached the failure): 
| link | strategy | frames before death | frames after death | P(any post-death frame) | median lag of 1st (steps) | partner frames |
|---|---|---|---|---|---|---|
| d0_loss0 | DEV | 0.35 | 8.75 | 0.87 | 1 | 1.76 |
| d0_loss0 | DEV_SD | 0.35 | 1.04 | 0.91 | 0 | 1.81 |
| d0_loss0 | DEV_SD_B12 | 0.31 | 0.63 | 0.63 | 0 | 0.81 |

### perm_late

| link | strategy | P(detect) | d vs IMPLICIT | d vs HEALTH | bytes/ep |
|---|---|---|---|---|---|
| d0_loss0 | IMPLICIT | 0.633 | - | +0.001 [-0.016,+0.018] | 0.0 |
| d0_loss0 | HEALTH | 0.632 | -0.001 [-0.018,+0.016] | - | 5.5 |
| d0_loss0 | HB_TO | 0.632 | -0.001 [-0.018,+0.016] | +0.000 [+0.000,+0.000] | 67.6 |
| d0_loss0 | DEV | 0.622 | -0.012 [-0.042,+0.018] | -0.011 [-0.036,+0.013] | 50.5 |
| d0_loss0 | DEV_SD | 0.632 | -0.001 [-0.030,+0.027] | +0.000 [-0.024,+0.024] | 29.9 |
| d0_loss0 | DEV_SD_B12 | 0.626 | -0.007 [-0.036,+0.018] | -0.007 [-0.033,+0.018] | 15.1 |

Lost-agent frames (episodes that reached the failure): 
| link | strategy | frames before death | frames after death | P(any post-death frame) | median lag of 1st (steps) | partner frames |
|---|---|---|---|---|---|---|
| d0_loss0 | DEV | 1.15 | 4.94 | 0.92 | 1 | 2.09 |
| d0_loss0 | DEV_SD | 1.15 | 1.05 | 0.94 | 0 | 2.19 |
| d0_loss0 | DEV_SD_B12 | 0.73 | 0.25 | 0.25 | 0 | 0.93 |
