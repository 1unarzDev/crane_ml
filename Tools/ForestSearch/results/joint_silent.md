# Silent repairs for the joint planner (joint_silent.py)

5x5, travel 1, events 'both', ideal link (silent => no messages), joint K=6, B=14, 300 episodes per instance, paired cluster bootstrap (instances, then episodes; 4000 resamples). Compute via fsrun.

Variants (all message-free; trust computed from the common-knowledge event rate r = abort+alpha or from the agent's own private-event count; trust = 1 => unmodified joint planner, so Proposition 1 holds, tested in test_joint_silent.py):
- A (i) disc: partner's future rollout looks weighted by trust. A1 time (trust=(1-r)^k, k = partner nominal looks done), A2 same with r/2, A3 own (trust = 0.6^own_events).
- B (ii) mix: trust*V(nominal forecast) + (1-trust)*V(alt: partner idle at last completed nominal cell, in-flight look not counted, greedy from my belief). B1 time, B2 time r/2, B3 own.
- C (iii) own-path: trust < 0.5 => single-agent rollout (partner only via in-flight slot, as greedy models it), else joint. C1 time, C2 own (0.6^own_events).

## Dev (seeds 100-102), used to select
| strategy | P(detect) | -jORACLE | -jIMPLICIT | -gIMPLICIT |
|---|---|---|---|---|
| jORACLE | 0.777 | - | +0.032 [+0.008,+0.059] | +0.010 [-0.027,+0.047] |
| jIMPLICIT | 0.744 | -0.032 [-0.059,-0.008] | - | -0.022 [-0.069,+0.019] |
| gIMPLICIT | 0.767 | -0.010 [-0.047,+0.027] | +0.022 [-0.019,+0.069] | - |
| A1_disc_time | 0.749 | -0.028 [-0.057,-0.001] | +0.004 [-0.027,+0.034] | -0.018 [-0.062,+0.031] |
| A2_disc_time_half | 0.732 | -0.044 [-0.076,-0.016] | -0.012 [-0.039,+0.013] | -0.034 [-0.088,+0.017] |
| A3_disc_own | 0.756 | -0.021 [-0.049,+0.007] | +0.011 [-0.017,+0.040] | -0.011 [-0.044,+0.020] |
| B1_mix_time | 0.741 | -0.036 [-0.061,-0.007] | -0.003 [-0.038,+0.034] | -0.026 [-0.057,+0.006] |
| B2_mix_time_half | 0.749 | -0.028 [-0.053,-0.001] | +0.004 [-0.027,+0.039] | -0.018 [-0.048,+0.011] |
| B3_mix_own | 0.762 | -0.014 [-0.041,+0.013] | +0.018 [-0.013,+0.054] | -0.004 [-0.031,+0.022] |
| C1_own_time | 0.762 | -0.014 [-0.048,+0.018] | +0.018 [-0.019,+0.052] | -0.004 [-0.034,+0.026] |
| C2_own_own | 0.763 | -0.013 [-0.034,+0.009] | +0.019 [-0.006,+0.044] | -0.003 [-0.038,+0.030] |

Selected: C2_own_own (smallest gap to joint ORACLE, -0.013, tightest CI). Selection among 8 variants on 3 instances is noisy; no variant is distinguishable from jIMPLICIT at 95%.

## Held-out (seeds 200-207), single evaluation of the selected variant
| strategy | P(detect) | -jORACLE | -jIMPLICIT | -gIMPLICIT |
|---|---|---|---|---|
| jORACLE | 0.770 | - | +0.039 [+0.017,+0.061] | +0.006 [-0.013,+0.024] |
| jIMPLICIT | 0.730 | -0.039 [-0.061,-0.017] | - | -0.033 [-0.056,-0.011] |
| gIMPLICIT | 0.764 | -0.006 [-0.024,+0.013] | +0.033 [+0.011,+0.056] | - |
| C2_own_own | 0.742 | -0.027 [-0.043,-0.010] | +0.012 [-0.007,+0.033] | -0.021 [-0.043,+0.002] |

## Conclusion
Negative. No silent fix closes the gap. Held-out C2 recovers about 0.012 of the 0.039 ORACLE-IMPLICIT gap (CI vs joint IMPLICIT [-0.007,+0.033] includes 0) and stays significantly below joint ORACLE (-0.027 [-0.043,-0.010]). It is still below greedy IMPLICIT (-0.021, CI includes 0). Greedy + silence (0.764) remains the best silent policy; joint ORACLE vs greedy remains +0.006, not significant.
Dev signals: time-based trust (A1, A2, B1, B2) did not help or hurt; own-event-based trust (A3, B3, C2) and own-path fallback (C1) were each about +0.01-0.02 on dev, all within noise. The hypothesis that the partner forecast is the mechanism is only weakly supported (fallback toward greedy-like partner modeling is the direction that helps, consistent with greedy IMPLICIT being robust), not established. Caveat: dev has only 3 clusters, so the dev CIs are wide; variants were not tuned beyond the 8 listed.
