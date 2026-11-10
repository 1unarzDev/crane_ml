# Joint vs greedy planner on all 8 held-out instances (EXPLORATORY, post-preregistration)

Supersedes the 5-seed deviation in `joint_heldout_all.md`. That file is kept unchanged; its 5-seed table is reproduced below for comparison.

Settings are unchanged from `joint_heldout_all.md`. Events are `both` (λ=1), the link is ideal, and each seed has 300 episodes. Strategies are ORACLE, IMPLICIT and INDEP. The joint planner is `make_joint_strategies(6, 14)`.
- Seeds 200–204 of the three new cells come from the archived CSVs `results/data/joint_cells__out__c{1,2,3}_20[0-4].csv.gz`.
- Seeds 205–207 were run for this file with the unmodified `scripts/joint_cells_worker.py 205 208` (`FS_JH2=/tmp/fs_nongreedy/jc8`).
- The new run of c2_205 is byte-identical to the archived `joint_cells__extra__c2_205.csv.gz`, so the pipeline reproduces.
- The 5x5 travel 1 row is recomputed from `joint_heldout_a/b.csv.gz` (link ideal). It reproduces `joint_heldout.md`.

CIs are paired cluster-bootstrap 95% intervals: resample instances, then episodes within each instance, 2000 times, with RNG seed 12345. The code is the same as `scripts/joint_cells_agg.py`, extended to read both sources. Aggregator: `/tmp/fs_nongreedy/scripts/agg_joint8.py`.

## All 8 seeds (200–207) in every cell

| cell | seeds | planner | P ORACLE | P IMPLICIT | P INDEP | ORACLE-IMPLICIT |
|---|---|---|---|---|---|---|
| 5x5 travel 1, default horizon | 200-207 | greedy | 0.764 | 0.764 | 0.708 | +0.000 [-0.010,+0.011] |
| 5x5 travel 1, default horizon | 200-207 | joint | 0.770 | 0.730 | 0.707 | +0.039 [+0.017,+0.061] |
| 5x5 travel 0, horizon 16 | 200-207 | greedy | 0.692 | 0.684 | 0.607 | +0.008 [-0.008,+0.027] |
| 5x5 travel 0, horizon 16 | 200-207 | joint | 0.644 | 0.646 | 0.545 | -0.002 [-0.018,+0.014] |
| 6x6 travel 0, horizon 16 | 200-207 | greedy | 0.579 | 0.579 | 0.492 | +0.000 [-0.020,+0.021] |
| 6x6 travel 0, horizon 16 | 200-207 | joint | 0.550 | 0.549 | 0.440 | +0.001 [-0.013,+0.016] |
| 6x6 travel 1, default horizon | 200-207 | greedy | 0.758 | 0.764 | 0.702 | -0.006 [-0.018,+0.006] |
| 6x6 travel 1, default horizon | 200-207 | joint | 0.757 | 0.739 | 0.701 | +0.018 [-0.003,+0.038] |

### Cross-planner contrasts (8 seeds)

| cell | joint ORACLE - greedy IMPLICIT | joint IMPLICIT - greedy IMPLICIT | joint ORACLE - greedy ORACLE |
|---|---|---|---|
| 5x5 travel 1, default horizon | +0.006 [-0.012,+0.024] | -0.033 [-0.055,-0.012] | +0.005 [-0.012,+0.023] |
| 5x5 travel 0, horizon 16 | -0.040 [-0.074,-0.005] | -0.038 [-0.067,-0.010] | -0.048 [-0.084,-0.010] |
| 6x6 travel 0, horizon 16 | -0.029 [-0.058,-0.000] | -0.030 [-0.058,-0.003] | -0.029 [-0.053,-0.002] |
| 6x6 travel 1, default horizon | -0.007 [-0.023,+0.008] | -0.025 [-0.047,-0.003] | -0.001 [-0.016,+0.014] |

## Previous 5-seed table (seeds 200–204 for the three new cells; copied from joint_heldout_all.md)

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

## Reading (exploratory)

- Every 5-seed reading survives with 8 seeds, and the CIs are narrower.
  - Greedy ORACLE − IMPLICIT is within ±0.01 in all four cells, and every CI includes 0.
  - Joint ORACLE − IMPLICIT is +0.039 [+0.017,+0.061] in 5x5 travel 1 and +0.018 [−0.003,+0.038] in 6x6 travel 1. The travel-1 sign is consistent, but only the 5x5 CI excludes 0.
  - In both travel-0 cells, joint ORACLE − IMPLICIT is about 0.
- Joint IMPLICIT − greedy IMPLICIT is now significantly negative in all four cells, from −0.025 to −0.038. The 5x5 travel 1 contrast was previously a point estimate only; it is now −0.033 [−0.055,−0.012]. Silent agents using the joint planner pay for nominal-plan brittleness everywhere.
- The best communicating team is never better than the best silent team.
  - Joint ORACLE − greedy IMPLICIT is +0.006 [−0.012,+0.024] and −0.007 [−0.023,+0.008] in the travel-1 cells.
  - It is significantly negative in both travel-0 cells, where the joint planner is worse than greedy even when centralised: joint ORACLE − greedy ORACLE is −0.048 and −0.029.
- The 5x5 travel 0 greedy ORACLE − IMPLICIT shrinks from +0.015 (5 seeds) to +0.008 (8 seeds).
