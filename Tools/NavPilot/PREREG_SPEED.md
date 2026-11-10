# E3 preregistration: maximum reliably achievable speed at controlled collision risk

Status: **FROZEN** 2026-11-10 00:59 local, after DEV tuning and before any E3 held-out layout was generated. The held-out plan then runs once, and later
changes are listed under Deviations. Evidence label for every outcome: IMPLEMENTED (2.5D point mass, synthetic
sensors, synthetic forest and scenarios). Nothing here is LIVE_SIM_VALIDATED or PHYSICALLY_CALIBRATED.

## Question
For each navigation method, what is the highest speed setpoint at which the collision probability stays at or below
5 %, and what course speed does the method actually achieve there? The conditions cover stale maps, map-alignment
drift, local-estimator drift, thin branches, sensor occlusion and compute stalls. A second question is when mapping
beats lightweight reactive control.

## Harness and code
`Tools/NavPilot`: unchanged N1/E2 modules plus the new files `stacks/advanced.py`, `experiments/e3_episode.py` (a copy
of the E2-frozen `e2_episode.py` with hooks) and `experiments/e3.py`. The E2-frozen files are not edited.

## Condition families (each held-out layout yields all four; `e3.FAMS`)
| family | scenario | compute tail | stressor |
|---|---|---|---|
| stale | S1 | heavy | prior shows a near-axis gap open; 2 unmapped 2.5-4 cm hard branches span it; 5 % phantom stems; no branches in prior |
| drift | S3 | heavy | prior correct but map alignment drifts 1.2-2.0 m before a wall with a 1.8-2.2 m gap |
| occl | S4 | heavy | stereo 60 % invalid, LiDAR x0.25 points, local drift x10, thin branch in the only 1.8-2.0 m gap, alignment drift 1.0-1.8 m |
| stall | S0 | severe | correct prior, no drift; compute p99 0.56 s, p99.9 2.5 s |

All families use detector `on` (rho 0.2), link up, wind `light_breeze`, 80 s time limit, 10 s stuck rule.

## Methods (`stacks/advanced.py`; every method keeps the E2 safety layer: p99-age stopping rule with simulated
lag/jerk braking plus the 0.15 s stall watchdog)
| id | description | role |
|---|---|---|
| reactive | E2 REACTIVE (newest frames only, 8 m, goal heading) | reference |
| hybrid_conf | E2 HYBRID_CONF with E2-tuned params (E2 budget 8 configs x 8 episodes) | prior work in this track |
| opt_backup_t | SUPER/FASTER-style: optimistic scoring (prior + unknown free), known-free backup stop | **strongest baseline** |
| reactive_plus | short-memory fresh map (no prior), optimistic score + known-free backup, latency compensation | stronger reactive |
| mpc_lc | opt_backup_t + latency-compensated start state + constraint tightening k0 + ks*s | short-horizon MPC |
| ua_map | opt_backup_t + prior weight c_align + prior dilation z*sigma_align + drift-budget memory | uncertainty-aware map reliance |
| pas_opt | opt_backup_t + perception-aware backup set (close-range-confirmed free space near structure) + look-ahead yaw | active perception |
| pas_full | ua_map + mpc_lc + pas | combination |

Tuning: DEV layouts 300-301 only, families x {6, 9} m/s = 16 episodes per config. Each of the six new or tuned methods
gets 5 configs (defaults + 4 seeded random draws over its knobs, `e3.configs`), i.e. an equal budget of 80 episodes.
Selection objective: mean(progress_rate - 40 * 1[collision]). reactive is untuned (no knobs in E2), and hybrid_conf
keeps its E2 tuning.

## Held-out plan (`e3.plan_jobs("heldout")`)
- **Fresh** HELD-OUT layout seeds **90400-90407** (8 whole layouts; splits.json held-out range). E2 uses 90200-90207,
  and E3 does not touch them.
- Speed setpoints V = {2.5, 4, 5.5, 7, 9} m/s (the v_lim given to every method).
- **Compute cut (decided before freeze, from a measured ~8.7 episodes/min on the single fsrun slot obtained).**
  Held-out arms: opt_backup_t, reactive_plus, mpc_lc, ua_map, plus the PAS variant with the higher DEV utility, which
  is **pas_full** (`_pas_pick`). reactive and hybrid_conf are dropped from held-out, because E2 held-out covers them
  at 3 and 7 m/s. pas_opt stays a DEV-only ablation. The grid moved down because DEV showed opt_backup_t colliding
  in 7/7 `stale` episodes already at 6 m/s.
- 5 methods x 4 families x 5 speeds x 8 layouts = 800 episodes. Run seed = layout seed (common random numbers).

## Endpoints (decided now)
For method m and family set F (primary: all four pooled; secondary: each family alone):
1. Collision indicator per episode. At each setpoint v: c(v) collisions out of n(v) episodes.
2. Isotonic (non-decreasing, PAVA, weights n) estimate p(v) of P(collision | v).
3. **v\*_m** = setpoint speed where p(v) crosses 5 %, by linear interpolation on the grid. Censoring: if p(2.5) > 5 %,
   v\* is reported as "<2.5" (value 2.5 in the bootstrap). If p never exceeds 5 %, v\* is reported as ">=9".
4. **A\*_m (primary)** = achieved course speed at v\*: the mean progress rate (x progress / time) of non-collision
   episodes at each setpoint, linearly interpolated at v\*. This is the "maximum reliably achievable speed". It is
   primary because a method that self-limits speed is safe at every setpoint, so its v\* would be uninformative.
5. Uncertainty: cluster bootstrap over layouts (2000 resamples, seed 0, all families and speeds of a layout move
   together), 95 % percentile CIs. Comparisons are paired, using the same resample indices.
6. **Primary test:** dA\* = A\*_m - A\*_opt_backup_t, pooled over families, for each m in {reactive_plus, mpc_lc, ua_map,
   pas_opt, pas_full}. A method "raises the max safe speed" iff the 95 % CI of dA\* excludes 0 and is positive.
   There is no multiplicity correction, and with 5 comparisons a single marginal win is treated as weak.
7. Secondary: dv\* with CIs; the per-family A\* and v\* (with 10 layouts, so per-family CIs will be wide); the
   mapping-vs-reactive contrast reactive_plus vs {opt_backup_t, ua_map} per family; the mechanism table
   (first-detection range of the scenario thin branches, contact kind and speed).
8. Completeness guard: only layouts with all 100 planned episodes enter the analysis. If the run is cut by compute,
   the report uses the complete layouts and the shortfall is stated.

Report: `experiments/e3.py report --plan heldout` -> `results/E3_heldout.md`, `results/raw/e3_heldout_summary.json`.

## Declared assumptions and known limits (from the pre-freeze code review)
- **Self-reported uncertainty channels (E2 convention).** `sigma_loc_rate` is the estimator's own drift rate, a step
  function at the scenario's degraded zone. `sigma_align_rep` is a noisy alignment-quality report derived from the
  true alignment error with a lognormal factor. These are channels a real VIO covariance and a registration module
  would provide, but here they are idealised. Only ua_map, pas_full and hybrid_conf consume them, so any ua_map gain
  in `occl` or `drift` is an upper bound.
- **PAS structural prior.** "Thin branches grow near stems or deadfall" is true by construction of this synthetic
  forest and its scenario blocks. A PAS gain is therefore partly a gain from a correct prior.
- **rmin bias.** The `RangeOccMap.rmin` update keeps the last write among duplicate cells, not the minimum. This biases
  rmin up by at most ~0.15 m, which is conservative for PAS. It is kept unchanged because the methods were tuned with
  it.
- **Coarse structure dilation.** The `d_struct` dilation uses 0.5 m blocks aligned to the window, so the effective
  radius is d_struct + 0 to 0.4 m (Chebyshev).
- **Tightening cap.** Tightening saturates at 0.4 m.
- **Pooled 5 % criterion.** One family can exceed 5 % while the pooled rate stays below it. The per-family tables
  are reported for that reason.
- **Unequal tuning.** reactive is untuned, and hybrid_conf keeps its E2 tuning (64 episodes, a different objective).
  The six E3 methods get 80 DEV episodes each, with ties going to the higher config index.
- **Mechanism metric.** The block-branch first-occupied range uses interior branch samples only (more than 0.4 m from
  any stem surface), checked at planner cycles in the drifting estimate frame. It is descriptive only.
- **Analysis.** Rows are deduplicated by (layout, family, method, setpoint), keeping the first. A layout enters only
  if its key set equals the planned set exactly. If a setpoint has no non-collision episode, A\* carries the previous
  setpoint's achieved value forward.
- **Parity.** e3_episode must reproduce e2_episode exactly for reactive, for hybrid_conf, and for opt_backup_t at
  defaults vs E2 OPT_BACKUP (`results/raw/e3_parity.json`).

## Freeze
- Time: 2026-11-10 00:59 local, before any E3 held-out layout was built, run or logged.
- Code sha256 (concatenated sorted: experiments/e3.py, experiments/e3_episode.py, experiments/scenarios.py,
  navpilot/*.py, stacks/*.py): `6df2c3cf975d2b4ff8e8bea1b88807573773e71b4e49d20aa9ae4e66408a6ba2`
- `results/raw/e3_tuned.json` sha256 `a799993dac489f7c2596c07374566023263b6e926dbaa4d08a0a62b87f00e044`. Tuned params:
  - opt_backup_t: `{"margin": 0.5, "inflate_extra": 0.25, "w_free": 1.0}`
  - reactive_plus: `{"margin": 0.5, "inflate_extra": 0.25, "w_free": 0.6, "react_mem": 0.7, "H": 8.0}`
  - mpc_lc: `{"margin": 0.3, "inflate_extra": 0.15, "w_free": 0.6, "tighten_k0": 0.1, "tighten_ks": 0.01}`
  - ua_map: `{}`
  - pas_opt: `{}`
  - pas_full: `{"margin": 0.3, "inflate_extra": 0.25, "w_free": 0.4, "tighten_k0": 0.05, "tighten_ks": 0.01, "ua_z": 1.0, "drift_tol": 0.15, "R_conf": 4.0, "d_struct": 1.0, "look": 0.0}`
- DEV tuning rows: `results/raw/e3_tune_*.jsonl` (480 episodes). E2-parity check: `results/raw/e3_parity.json` (3/3
  identical).
- Command: `e3.py submit --plan heldout --allow-heldout` (job files run by `experiments/worker.py` under fsrun);
  report: `e3.py report --plan heldout`.

## Deviations
- None yet.
- 2026-11-10 01:01 (execution only, after freeze, before any held-out episode): `e3.submit` now writes held-out job
  files as contiguous layout-ordered chunks instead of interleaved ones, so a compute-cut run leaves whole layouts
  complete. No episode changes. The code sha256 above predates this one-line edit.
- 2026-11-10 01:05: the orchestrator paused the held-out worker child with SIGSTOP (machine oversubscribed). At that
  point 58 of 800 held-out episodes were written (layout 90400, job files 000-001 done, 002 in progress). The second
  queued worker was cancelled to respect the 2-heavy-process cap. Resuming with SIGCONT continues the same frozen
  plan unchanged. Nothing has been read from the held-out rows.
- 2026-11-10 01:10 (analysis code, before any held-out result was read): `curve_stats` now aggregates through a
  per-(method, family, layout, setpoint) index, for speed. The estimator and endpoint definitions are unchanged.
