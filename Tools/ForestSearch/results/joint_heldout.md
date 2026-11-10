# Joint vs greedy planner, held-out seeds 200–207 (EXPLORATORY, post-prereg)

5x5, travel 1, events 'both', 300 episodes per seed, cluster bootstrap.

## link ideal

| planner | P ORACLE | P IMPLICIT | P FULL | P INDEP | ORACLE−IMPLICIT | FULL−IMPLICIT |
|---|---|---|---|---|---|---|
| greedy | 0.764 | 0.764 | 0.764 | 0.708 | +0.000 [-0.010,+0.012] | +0.000 [-0.010,+0.011] |
| joint | 0.770 | 0.730 | 0.770 | 0.707 | +0.039 [+0.018,+0.061] | +0.039 [+0.016,+0.061] |

joint ORACLE − greedy IMPLICIT (ideal): +0.006 [-0.012,+0.023]

## link burst30_L10_d1

| planner | P ORACLE | P IMPLICIT | P FULL | P INDEP | ORACLE−IMPLICIT | FULL−IMPLICIT |
|---|---|---|---|---|---|---|
| greedy | 0.764 | 0.764 | 0.738 | 0.708 | +0.000 [-0.010,+0.011] | -0.025 [-0.043,-0.010] |
| joint | 0.770 | 0.730 | 0.742 | 0.707 | +0.039 [+0.017,+0.062] | +0.011 [-0.013,+0.038] |

joint ORACLE − greedy IMPLICIT (burst30_L10_d1): +0.006 [-0.013,+0.024]
