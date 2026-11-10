# Preregistration: confirmatory run (frozen 2026-11-09, before any held-out data)

Code: worktree `research/forest-sar` (uncommitted; sha256 of the .py files is recorded in `confirm_manifest.txt` at launch).
Planner: greedy (`make_all_strategies()`). Robustness to the planner is reported separately and is not part of these hypotheses.

## Design
- **Held-out instances:** grid_instance seeds **200–207** (8 instances), grids **5x5 and 6x6**, travel_scale **0 and 1**.
  - Dev seeds 100–102 are excluded.
  - Horizon is 16 for travel 0 and 2·n for travel 1.
- **Private-event scale:** λ ∈ {0, 1, 2, 3}, applied to the base rates (abort 0.10, h_good 0.30, h_bad 0.15, alpha 0.03).
- **Links:**
  - d0_loss0: ideal
  - d1_loss0: delay 1, no loss
  - d0_burst30: GE, 30% mean loss, mean burst 10, delay 0
  - d1_burst30: GE, 30% mean loss, mean burst 10, delay 1
- **Strategies:** ORACLE, IMPLICIT, SECTOR, INDEP, FULL, HYBRID, HYBRID_P.
- **Episodes:** 300 per (instance, cell), paired by seed across strategies (common random numbers).
- **Metric:** P(confirmed detection by horizon).
- **Uncertainty:** cluster bootstrap with 2000 resamples. Instances are resampled first, then episodes within each instance. Analysis uses `analyze_curve.py` unchanged.

## Hypotheses (judged per grid × travel cell; all cells reported)
- **H1 (communication has little value).** With link d0_loss0, the 95% CI of ORACLE − IMPLICIT lies within [−0.03, +0.03] for every λ.
- **H2 (delayed messages hurt).** Travel 0, links d1_*: the upper CI of HYBRID − IMPLICIT is below 0 for λ ∈ {1, 2, 3}.
- **H3 (mechanism).** Travel 0, d1_loss0: the lower CI of HYBRID_P − HYBRID is above 0 for λ ∈ {1, 2, 3}.
- **H4 (replacement is fragile).** d1_loss0, λ = 0: the upper CI of FULL − IMPLICIT is below 0 for both travel settings.
- **H5 (a default plan matters).** d0_loss0: the upper CI of INDEP − IMPLICIT is below 0 for every λ.

## Decision rules
- If H1 fails in any cell, the headline becomes "communication has value in regime X", and that cell is what we report.
- If H3 fails, the mechanism claim is dropped. H2 is then reported as an unexplained effect.
- No re-running with new seeds and no dropping of cells. Any exploratory follow-up is labeled as such.
