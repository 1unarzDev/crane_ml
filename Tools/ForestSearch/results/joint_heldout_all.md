# Joint vs greedy planner across four held-out cells (EXPLORATORY, post-preregistration)

Events `both`, link ideal only, 300 episodes per seed, strategies ORACLE / IMPLICIT / INDEP.
Paired cluster bootstrap 95% CI (resample seeds/instances, then episodes within; 2000 resamples).

**DEVIATION: the three new cells use seeds 200-204 (5 instances), not 200-207.**
The 40-minute compute cap on a shared 3-slot semaphore did not allow all 8 seeds.
Seeds 200-204 are a clean prefix, completed for all three cells.
The 5x5 travel 1 row is copied from joint_heldout.md and uses seeds 200-207.
CIs for the new cells are therefore wider than for the existing row.

Joint planner = `make_joint_strategies(6, 14)`.
Driver: `/home/lunarz/.cache/research/fs-scripts/joint_heldout_cells2.py` and `/tmp/jh2/worker.py`.
Aggregation: `joint_cells_agg.py`, bootstrap RNG seed 12345.

| cell | seeds | planner | P ORACLE | P IMPLICIT | P INDEP | ORACLE-IMPLICIT |
|---|---|---|---|---|---|---|
| 5x5 travel 1 (existing, copied) | 200-207 | greedy | 0.764 | 0.764 | 0.708 | +0.000 [-0.010,+0.012] |
| 5x5 travel 1 (existing, copied) | 200-207 | joint | 0.770 | 0.730 | 0.707 | +0.039 [+0.018,+0.061] |
| 5x5 travel 0, horizon 16 | 200-204 | greedy | 0.704 | 0.689 | 0.615 | +0.015 [-0.007,+0.038] |
| 5x5 travel 0, horizon 16 | 200-204 | joint | 0.633 | 0.638 | 0.544 | -0.005 [-0.026,+0.017] |
| 6x6 travel 0, horizon 16 | 200-204 | greedy | 0.575 | 0.576 | 0.473 | -0.001 [-0.027,+0.024] |
| 6x6 travel 0, horizon 16 | 200-204 | joint | 0.537 | 0.537 | 0.421 | +0.000 [-0.019,+0.019] |
| 6x6 travel 1, default horizon | 200-204 | greedy | 0.759 | 0.768 | 0.705 | -0.009 [-0.026,+0.007] |
| 6x6 travel 1, default horizon | 200-204 | joint | 0.759 | 0.741 | 0.713 | +0.017 [-0.007,+0.046] |

## Cross-planner contrasts

| cell | joint ORACLE - greedy IMPLICIT | joint IMPLICIT - greedy IMPLICIT |
|---|---|---|
| 5x5 travel 1 (existing, seeds 200-207) | +0.006 [-0.012,+0.023] | -0.034 (point only; no CI, see note) |
| 5x5 travel 0 | -0.057 [-0.091,-0.021] | -0.051 [-0.083,-0.017] |
| 6x6 travel 0 | -0.039 [-0.073,-0.005] | -0.039 [-0.072,-0.007] |
| 6x6 travel 1 | -0.009 [-0.029,+0.011] | -0.027 [-0.053,-0.001] |

Note: joint_heldout.md does not report joint IMPLICIT - greedy IMPLICIT directly.
Point value from its table is 0.730 - 0.764 = -0.034 (no CI computed).

## Reading (exploratory)

- Planner dependence of ORACLE - IMPLICIT is not general. Only the existing cell shows a CI-excluding-zero joint gap (+0.039). 6x6 travel 1 has the same sign (+0.017) with a CI including zero. Both travel-0 cells show about 0 for both planners.
- Joint ORACLE vs greedy IMPLICIT: about zero in both travel-1 cells; significantly negative in both travel-0 cells, where the joint planner is worse than greedy at this setting.
- In every cell, communicating (ORACLE) is no better than silent greedy (IMPLICIT) by more than noise, so "best communicating <= best silent" holds, in the weaker sense that no cell shows a communicating advantage.
