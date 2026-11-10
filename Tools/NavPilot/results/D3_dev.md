# D3 (DEV only): compute-aware high-speed navigation, why 7 m/s fails and whether (speed, compute) policies raise v*

Status 2026-11-10, DEV only. Evidence label: IMPLEMENTED (NavPilot 2.5D point mass, synthetic sensors/forest/scenarios); nothing here is LIVE_SIM_VALIDATED or PHYSICALLY_CALIBRATED, and DEV numbers are not reportable results. **No E2/E3 held-out layout (90400-90407), no E4 layout (90600-90615) and no .NET/Unity run was used.** Vehicle profile for every number: **v1** (`aos7_v5_f90_6s_v1.json`, NavPilot's pinned default) except the labelled v1-vs-CAD row (section 5). Layouts: diag 310-313, tune 330-331, train 340-341, eval 320-324. Compute: about 2.4 CPU-h (chain: 7208 s user + 913 s sys, plus a 290 s chaos-floor control and report runs), one fsrun slot, no Unity, no .NET.

## 0. Answer in brief
1. **H1 (latency x braking explains 7 m/s failures) is refuted in NavPilot; NAV_LIT P1 / NOVELTY_D3 P1 (perception, not latency) is favoured.** Of 13 baseline collisions (opt_backup_t = p99 governor, 5.5/7/9 m/s, 48 paired episodes) perfect perception removed 12 (92 %, McNemar p = 0.013; 2 new), perfect perception + infinite range 12, prior = truth (map staleness) 10 of 13 (p = 0.18). Zero latency, p50 latency, ideal braking and infinite range each remove 6-7 but also create 5-6 new collisions (net about 0), and latency + braking together removes 6 and creates 2 (McNemar p = 0.29): that churn equals the chaos floor of section 4 (any 1 mm perturbation flips 19 of 120 outcomes), so none of these arms is distinguishable from no effect.
2. **H2 formally holds against the stated baseline, but the gain is not from compute adaptivity.** cc_pas vs the p99 governor opt_backup_t: dv* = +4.42 m/s (95 % CI +2.00, +6.00) at 5 % and +1.95 (+1.60, +6.40) at 1 %. Against the same-belief non-adaptive baseline pas_full (PAS = perception-aware backup set, an E3 method), the pooled 5.5/7/9 m/s collision rate difference is 0.0 pp (-5.6, +5.6) and achieved progress is -0.13 m/s (-0.20, -0.06). A fixed lite level (fix_l2) and a RoboRun-style environment-driven level (rr_pas) do as well (NOVELTY_D3 P3 holds), and a CEM-learned level/delta head (lp_pas) is worse (15 % vs 5 % collisions). **Kill check (gain < 0.5 m/s with a known-free backup present): FIRES.** v* differences among the PAS-family arms are artefacts of single collisions (20 episodes per setpoint; one collision at 4 m/s puts pas_full's v* at 3.70 while cc_pas, with 0 there, gets 7.67), so use the robust table in section 3.
3. **Without a backup** (p99_nb vs cc_nb), the governor lowers collisions 55.0 % -> 46.7 % at equal progress but both are far from safe (v* < 2.5-4 m/s); dv* = 0.00 (0.00, +1.88). Consistent in sign with NOVELTY_D3 P2, not at the >= 1 m/s size, and no CI was computed for the collision-rate difference.
4. **Multi-rate: the 99 % / 2 cm acceptance rule cannot be met by any perturbation in this closed loop.** 100 Hz and 50 Hz plants agree with 200 Hz on 83.3 % and 85.0 % of 120 collision outcomes (mean |d min-clearance| 14 cm); a 1 mm start offset at 200 Hz gives 84.2 % and 14.9 cm. Lower plant rates are statistically indistinguishable from the chaos floor, but the stated rule would reject them (and 200 Hz itself). The rule must be reformulated relative to the floor before the .NET test (section 4, `results/MULTIRATE_NET_SPEC.md`).
5. **Fidelity caveat (from D4's first readout, reported by the orchestrator, not reproduced here):** NavPilot over-states >= 5 m/s failure vs the .NET harness, so every 5-9 m/s statement above is NavPilot-specific (section 6).

## 1. Diagnosis (H1)

### Collision rate by counterfactual arm (opt_backup_t = the p99-age governor; 4 DEV layouts x 4 families = 16 episodes per cell)

| arm | 5.5 m/s | 7 m/s | 9 m/s | pooled | removed (flips) | new collisions |
|---|---|---|---|---|---|---|
| base | 3/16 | 6/16 | 4/16 | 13/48 (27%) | 0 | 0 |
| lat0 | 4/16 | 4/16 | 5/16 | 13/48 (27%) | 6 | 6 |
| lat50 | 2/16 | 6/16 | 4/16 | 12/48 (25%) | 6 | 5 |
| brake | 4/16 | 3/16 | 5/16 | 12/48 (25%) | 7 | 6 |
| lat0+brake | 2/16 | 3/16 | 4/16 | 9/48 (19%) | 6 | 2 |
| perc | 1/16 | 1/16 | 1/16 | 3/48 (6%) | 12 | 2 |
| range | 3/16 | 4/16 | 5/16 | 12/48 (25%) | 6 | 5 |
| perc+range | 2/16 | 0/16 | 2/16 | 4/48 (8%) | 12 | 3 |
| map | 2/16 | 4/16 | 1/16 | 7/48 (15%) | 10 | 4 |

### Attribution: share of baseline collisions removed by each factor (paired episodes; McNemar exact p on flips)

| factor (arms) | 5.5 m/s | 7 m/s | 9 m/s | pooled | McNemar p (pooled) |
|---|---|---|---|---|---|
| tau-tail only (lat50: p50 compute instead of heavy tail) | 2/3 | 2/6 | 2/4 | 6/13 (46%) | 1.000 |
| zero latency (lat0) | 2/3 | 2/6 | 2/4 | 6/13 (46%) | 1.000 |
| ideal braking (brake) | 3/3 | 4/6 | 0/4 | 7/13 (54%) | 1.000 |
| tau-tail x braking envelope (lat0+brake) | 2/3 | 3/6 | 1/4 | 6/13 (46%) | 0.289 |
| perfect perception (perc) | 3/3 | 5/6 | 4/4 | 12/13 (92%) | 0.013 |
| infinite range (range) | 3/3 | 2/6 | 1/4 | 6/13 (46%) | 1.000 |
| occlusion-limited free space (perc+range) | 2/3 | 6/6 | 4/4 | 12/13 (92%) | 0.035 |
| map staleness (map: prior = truth) | 2/3 | 4/6 | 4/4 | 10/13 (77%) | 0.180 |

**Verdict H1 vs NAV_LIT P1 at 7 m/s:** perfect latency removes 33% of baseline collisions (plan H1 needs most of them), perfect perception+range removes 100%, ideal braking 67%, latency+braking together 50%. P1 (perception range, not latency) is favoured: latency removes < 50% and perception > 50% (NOVELTY_D3 P1).
First-detection range of the scenario thin branches, baseline, stale+occl (m, mean over detected branches; n): 5.5 m/s: 10.6 (n=12), 7 m/s: 11.6 (n=12), 9 m/s: 11.7 (n=12)

Baseline collisions: 13 of 48 episodes. Not removed by ANY single counterfactual arm: 0 (0%).
Contact kind of baseline collisions: branch 2, branch/block 5, stem 3, wall 3.
brake_infeasible (NavPilot analogue: no-feasible-command onset <= 1 s before contact at > 1 m/s with true clearance < brake distance): 12 of 13 baseline collisions; with fallback/no-feasible events: 12.

By family (baseline collisions / episodes, then removed by lat0+brake, perc+range, map):

| family | speed | base | lat0+brake | perc+range | map |
|---|---|---|---|---|---|
| stale | 5.5 | 0/4 | 0/4 | 0/4 | 0/4 |
| stale | 7 | 4/4 | 3/4 | 0/4 | 1/4 |
| stale | 9 | 3/4 | 2/4 | 1/4 | 1/4 |
| drift | 5.5 | 1/4 | 1/4 | 1/4 | 1/4 |
| drift | 7 | 0/4 | 0/4 | 0/4 | 1/4 |
| drift | 9 | 0/4 | 1/4 | 0/4 | 0/4 |
| occl | 5.5 | 1/4 | 1/4 | 1/4 | 0/4 |
| occl | 7 | 2/4 | 0/4 | 0/4 | 2/4 |
| occl | 9 | 1/4 | 1/4 | 1/4 | 0/4 |
| stall | 5.5 | 1/4 | 0/4 | 0/4 | 1/4 |
| stall | 7 | 0/4 | 0/4 | 0/4 | 0/4 |
| stall | 9 | 0/4 | 0/4 | 0/4 | 0/4 |

Mean progress rate (m/s) by arm (all episodes): base 3.91, lat0 4.75, lat50 4.89, brake 4.39, lat0+brake 4.83, perc 3.72, range 4.04, perc+range 3.71, map 4.79

Interpretation: the arms are paired (same seeds); "removed" and "new collisions" together show the chaos churn. Only the perception arms (and weakly map) remove more than they create. The verdict line states the 7 m/s shares as the NOVELTY_D3 P1 test: latency removes 33 % (< 50 %), perception + range 100 % (> 50 %): P1 satisfied, plan H1 refuted. Stall family (severe tail p99 0.56 s) collides in 1 of 12 baseline episodes: the p99-age rule already handles heavy tails, which is why latency is not the binding constraint in NavPilot.

## 2. Adaptive-compute policies: model and grounding

### Compute-level model tau(c), q(c) (`stacks/compute_gov.py`, LEVELS)
| level | map frames integrated per sensor per cycle | candidate headings | body-time x | stall x | detector rate x |
|---|---|---|---|---|---|
| 0 | 3 | 48 | 1.00 | 1.00 | 1.00 (10 Hz) |
| 1 | 2 | 24 | 0.83 | 0.80 | 0.50 (5 Hz) |
| 2 | 1 | 12 | 0.66 | 0.60 | 0.25 (2.5 Hz) |

- **tau(c).** Planner compute per cycle is the E3 lognormal body plus Bernoulli-gated Pareto stall (`navpilot.sensors.compute_time`), with common random numbers across levels (`level_compute`); the body is scaled by s_body and the stall by s_stall; the detector GPU share rho (E2 M/G/1-style slowdown) scales with the detector rate. **Grounding:** s_body(level 2) = 0.66 is a wall-time microbench of `CCStack.cycle` on this CPU (600 cycles per level, loaded host); level 1 was interpolated because its microbench was noise (12.1 ms median, above level 0). s_stall and the detector scaling are **assumptions [U]**: no Orin measurement exists. The Orin figures in `Tools/Vision/results/VISION_RESULTS.md` section 6.4 are *estimates* (RT-DETR-R18 640 tile 17-24 ms p50 / p99 x2.3; tiled 6x640 68-93 ms p50, 156-214 ms p99); `Tools/Vision/orin_bench.py` is the ready-to-run measurement and has not been run (no Jetson). The detector tail is what the stall multiplier stands for, and the NavPilot "heavy" tail (p99 160 ms, p99.9 521 ms) is of the same order as the tiled-detector p99 estimate.
- **q(c).** Not assumed: it is what the closed loop produces. Fewer headings (12 vs 48) coarsen the primitive set and fewer map frames per cycle thin the evidence integrated; the lower detector rate lowers person recall (reported as a cost). The extra-clearance knob e_c (D3_DEFAULTS e_q) is a tuned stand-in for the lost free-space quality in the chance constraint.
- **Not modelled (limits):** planner iterations or map resolution as separate knobs (map resolution is fixed at 0.1 m in the frozen map code); any GPU contention beyond the E2 formula. The "interior optimum" of NOVELTY_D3 therefore exists in this model only through the heading/frame/detector trade, and only if the eval table shows it.

### Policies (all keep the E2/E3 safety layer: stall watchdog 0.15 s in the episode)
- **p99 governor** = `opt_backup_t`: StopRule with tau = rolling p99 of the measured perception-to-command age (frozen, from E2/E3); known-free backup stop. This is Falanga 2019 evaluated at a quantile (NOVELTY_D3: a baseline, not a contribution).
- **clear_only**: tau = 0; clearance and braking envelope only.
- **cc_opt / cc_pas (a, analytic chance-constrained governor).** Each cycle, for every compute level c and for the primitives, require d_free >= v tau_c(delta) + D_brake(v) + m + e_c with tau_c(delta) the (1 - delta) quantile of (frame staleness + F_c), where F_c is the empirical per-level compute distribution of the last 100 cycles (cold start: level-0 samples scaled by the level model) and staleness is measured from the same cycle (age minus compute). Pick the level whose best feasible primitive has the highest score minus a per-level penalty lam_c; if nothing is feasible at any level, switch to level 2 (shortest latency to re-plan). Knobs (delta, e_c, lam_c) tuned on DEV layouts 330-331 (5 configs x 16 episodes).
- **lp_pas (b, learned).** A softmax level head and a delta head over 5 features (bias, fraction of feasible primitives at level 0, best free distance, tau_0, current level), 30 weights, trained by cross-entropy-method (CEM, NumPy, 3 iterations x 6 candidates) on DEV layouts 340-341 at 7 and 9 m/s with the utility progress - 40 x collision (`results/raw/d3_lp.json`). Not `Tools/Frontier/train.py` (it does not exist yet).
- **rr_pas** (RoboRun-style): level chosen from the environment only (clutter = share of headings with < 6 m free), p99 speed cap at that level. **fix_l1/fix_l2**: pas_full pinned to a level (the "fixed-c oracle" control of NOVELTY_D3). **p99_nb / cc_nb**: opt_backup_t and cc_opt without the known-free backup (optimistic belief only): the "without backup" arm.
- Recursive feasibility: the NavPilot rule already includes a one-cycle certificate only; the .NET RecursiveFeasibility (D_turn, lateral band, path-following brake) has **no NavPilot implementation**, so it is not a baseline here (noted, not run). SUPER-style backup: `opt_backup_t` is the SUPER/FASTER-style known-free backup stop (E3 labelled it the strongest baseline); a committed backup *trajectory* with path-following brake is not implemented.

### GPU-queue version of the learned policy (spec only; nothing submitted)
- Policy: 2-layer MLP (64-64), inputs = the 5 features above + the last 8 compute times + belief summary (d_free quantiles at 3 speeds), outputs = 3 level logits + 3 delta logits + a speed-cap residual; trained with PPO/REINFORCE-with-baseline against the NavPilot environment (the CCStack evaluator is the actor; env steps on CPU), 4 DEV layout shards x 4 families x {5.5, 7, 9} m/s, CRN seeds, 2-5 M planner cycles. `Tools/Frontier/jobs.py` job JSON: `needs.gpu`, `vram_gb <= 8`, preemptible, checkpoint every 50 k cycles, resume flag; first batch = the 2-job canary. Expected GPU use <= 2 GPU-h; the CPU rollouts dominate, so the gain from a GPU is small until the policy is a larger network. Submit only after the CEM result below shows that a learned level head beats cc_pas on DEV.

## 3. DEV evaluation (H2, kill check)

Complete DEV layouts: [320, 321, 322, 323, 324] (5). Episodes: 1100. Cluster bootstrap over layouts (2000 resamples, seed 0). Only 5 clusters, so CIs are wide and the numbers are DEV-grade.

### Pooled over families, v* at P(collision) <= 5 % (isotonic, E3 code); dv* = method - p99 governor (opt_backup_t), paired

| method | v* (95% CI) | A* m/s (95% CI) | dv* vs p99 gov (95% CI) | dA* vs p99 gov (95% CI) | collisions by setpoint [2.5, 4.0, 5.5, 7.0, 9.0] | achieved by setpoint |
|---|---|---|---|---|---|---|
| opt_backup_t | 3.25 (2.88-5.50) | 1.98 (1.79-2.91) | - | - | 0/20 2/20 4/20 9/20 7/20 | 1.5 2.4 2.9 2.9 3.8 |
| clear_only | 4.00 (3.00-4.21) | 2.36 (1.88-2.47) | +0.75 (-1.33, +1.15) | +0.38 (-0.52, +0.56) | 0/20 1/20 10/20 9/20 12/20 | 1.5 2.4 2.9 3.7 4.1 |
| pas_full | 3.70 (3.25-9.00) | 2.57 (2.25-4.79) | +0.45 (-0.38, +6.00) | +0.60 (+0.32, +2.67) | 0/20 2/20 1/20 1/20 1/20 | 1.8 2.8 3.5 4.1 4.4 |
| cc_opt | 3.25 (2.93-4.75) | 2.03 (1.92-2.60) | +0.00 (-2.25, +1.38) | +0.06 (-0.82, +0.68) | 0/20 2/20 3/20 6/20 5/20 | 1.6 2.5 2.9 3.2 3.7 |
| cc_pas | 7.67 (5.00-9.00) | 4.01 (3.24-4.83) | +4.42 (+2.00, +6.00) | +2.03 (+1.34, +2.59) | 0/20 0/20 1/20 0/20 2/20 | 1.8 2.8 3.4 3.8 4.4 |
| lp_pas | 4.00 (3.00-9.00) | 2.79 (2.13-4.85) | +0.75 (+0.00, +4.62) | +0.81 (+0.27, +2.55) | 0/20 1/20 2/20 4/20 3/20 | 1.9 2.8 3.5 4.3 4.5 |
| rr_pas | >=9.0 (6.25-9.00) | 4.31 (3.40-4.61) | +5.75 (+2.25, +6.12) | +2.33 (+1.07, +2.58) | 0/20 0/20 0/20 1/20 1/20 | 1.8 2.6 3.4 3.9 4.3 |
| fix_l1 | 7.00 (3.25-9.00) | 3.88 (2.10-4.90) | +3.75 (-1.25, +6.00) | +1.90 (-0.40, +2.56) | 0/20 1/20 1/20 1/20 2/20 | 1.7 2.7 3.4 3.9 4.4 |
| fix_l2 | >=9.0 (3.36-9.00) | 3.87 (2.28-4.25) | +5.75 (+0.48, +6.00) | +1.89 (+0.44, +2.38) | 0/20 2/20 1/20 0/20 0/20 | 1.7 2.7 3.3 3.4 3.9 |
| p99_nb | <2.5 (2.50-4.00) | 1.66 (1.56-2.37) | -0.75 (-3.00, -0.12) | -0.32 (-1.12, +0.07) | 2/20 5/20 11/20 12/20 10/20 | 1.7 2.5 3.0 3.5 3.4 |
| cc_nb | 2.50 (2.50-4.50) | 1.64 (1.57-2.55) | -0.75 (-2.75, +1.25) | -0.34 (-0.90, +0.62) | 1/20 2/20 7/20 10/20 11/20 | 1.6 2.4 2.9 3.5 4.4 |

### Pooled over families, v* at P(collision) <= 1 % (isotonic, E3 code); dv* = method - p99 governor (opt_backup_t), paired

| method | v* (95% CI) | A* m/s (95% CI) | dv* vs p99 gov (95% CI) | dA* vs p99 gov (95% CI) | collisions by setpoint [2.5, 4.0, 5.5, 7.0, 9.0] | achieved by setpoint |
|---|---|---|---|---|---|---|
| opt_backup_t | 2.65 (2.58-4.30) | 1.63 (1.54-2.33) | - | - | 0/20 2/20 4/20 9/20 7/20 | 1.5 2.4 2.9 2.9 3.8 |
| clear_only | 2.80 (2.60-4.04) | 1.68 (1.58-2.35) | +0.15 (-0.26, +1.43) | +0.05 (-0.10, +0.70) | 0/20 1/20 10/20 9/20 12/20 | 1.5 2.4 2.9 3.7 4.1 |
| pas_full | 2.74 (2.65-4.60) | 1.93 (1.77-3.07) | +0.09 (+0.00, +0.60) | +0.30 (+0.16, +0.84) | 0/20 2/20 1/20 1/20 1/20 | 1.8 2.8 3.5 4.1 4.4 |
| cc_opt | 2.65 (2.59-4.10) | 1.70 (1.64-2.33) | +0.00 (-1.54, +1.46) | +0.07 (-0.67, +0.76) | 0/20 2/20 3/20 6/20 5/20 | 1.6 2.5 2.9 3.2 3.7 |
| cc_pas | 4.60 (4.20-9.00) | 3.02 (2.82-4.83) | +1.95 (+1.60, +6.40) | +1.39 (+1.18, +2.99) | 0/20 0/20 1/20 0/20 2/20 | 1.8 2.8 3.4 3.8 4.4 |
| lp_pas | 2.80 (2.60-5.80) | 2.07 (1.85-3.91) | +0.15 (+0.00, +1.65) | +0.44 (+0.27, +1.65) | 0/20 1/20 2/20 4/20 3/20 | 1.9 2.8 3.5 4.3 4.5 |
| rr_pas | 5.80 (5.65-9.00) | 3.51 (3.27-4.61) | +3.15 (+1.55, +6.35) | +1.89 (+1.23, +2.93) | 0/20 0/20 0/20 1/20 1/20 | 1.8 2.6 3.4 3.9 4.3 |
| fix_l1 | 2.80 (2.65-9.00) | 1.91 (1.77-4.75) | +0.15 (-1.45, +6.35) | +0.29 (-0.45, +2.99) | 0/20 1/20 1/20 1/20 2/20 | 1.7 2.7 3.4 3.9 4.4 |
| fix_l2 | 2.90 (2.67-9.00) | 2.00 (1.70-4.04) | +0.25 (+0.10, +4.85) | +0.37 (+0.09, +1.77) | 0/20 2/20 1/20 0/20 0/20 | 1.7 2.7 3.3 3.4 3.9 |
| p99_nb | <2.5 (2.50-2.65) | 1.66 (1.55-1.79) | -0.15 (-1.65, -0.08) | +0.03 (-0.72, +0.17) | 2/20 5/20 11/20 12/20 10/20 | 1.7 2.5 3.0 3.5 3.4 |
| cc_nb | <2.5 (2.50-4.10) | 1.64 (1.56-2.40) | -0.15 (-1.55, +1.45) | +0.01 (-0.67, +0.80) | 1/20 2/20 7/20 10/20 11/20 | 1.6 2.4 2.9 3.5 4.4 |

### Backup / no-backup and fixed-compute controls (NOVELTY_D3 P2, P3)

Every method except p99_nb and cc_nb keeps the SUPER/FASTER-style known-free backup stop (opt_backup_t and pas_full are those baselines). p99_nb and cc_nb drop it: feasibility is evaluated on the optimistic belief (unknown = free) only.

| contrast | dv* @5 % (95% CI) | dv* @1 % (95% CI) |
|---|---|---|
| Delta (no backup): cc_nb - p99_nb | +0.00 (+0.00, +1.88) | +0.00 (+0.00, +1.60) |
| Delta_b (with backup): cc_opt - opt_backup_t | +0.00 (-2.25, +1.38) | +0.00 (-1.54, +1.46) |
| Delta_b (with backup + PAS): cc_pas - pas_full | +3.97 (-1.33, +5.00) | +1.86 (+1.50, +6.20) |
| cc_pas - opt_backup_t (p99 governor, H2) | +4.42 (+2.00, +6.00) | +1.95 (+1.60, +6.40) |
| lp_pas - opt_backup_t | +0.75 (+0.00, +4.62) | +0.15 (+0.00, +1.65) |
| RoboRun-style rr_pas - opt_backup_t | +5.75 (+2.25, +6.12) | +3.15 (+1.55, +6.35) |
| cc_pas - rr_pas | -1.33 (-4.00, +2.75) | -1.20 (-4.70, +3.35) |
| cc_pas - fix_l1 (adaptive vs fixed level 1) | +0.67 (-1.00, +5.50) | +1.80 (-1.40, +6.30) |
| cc_pas - fix_l2 (adaptive vs fixed level 2) | -1.33 (-4.00, +5.50) | +1.70 (+0.00, +6.30) |
| fix_l1 - pas_full (compute-only) | +3.30 (-3.83, +5.00) | +0.06 (-1.50, +6.20) |
| fix_l2 - pas_full (compute-only) | +5.30 (-5.30, +5.64) | +0.16 (-0.30, +4.70) |

P3 check (5 % crossing): adaptive cc_pas gain over level 0 (pas_full) = +3.97 m/s; best fixed level (1 or 2) gain = +5.30 m/s, ratio 1.34.

### Robust contrasts: pooled over the 5.5/7/9 m/s setpoints (60 episodes per method; v* from 20 episodes per setpoint is noisy: one collision moves it by metres)

| method | collision | progress m/s | stuck | d collision vs opt_backup_t (95% CI) | d collision vs pas_full (95% CI) | d progress vs pas_full (95% CI) |
|---|---|---|---|---|---|---|
| opt_backup_t | 33.3% | 3.59 | 15.0% | +0.0 pp (+0.0, +0.0) | +28.3 pp (+16.7, +38.9) | -0.37 (-0.64, -0.04) |
| clear_only | 51.7% | 4.15 | 6.7% | +18.3 pp (+12.5, +25.0) | +46.7 pp (+36.1, +55.6) | +0.19 (-0.14, +0.62) |
| pas_full | 5.0% | 3.95 | 1.7% | -28.3 pp (-38.9, -16.7) | +0.0 pp (+0.0, +0.0) | +0.00 (+0.00, +0.00) |
| cc_opt | 23.3% | 3.56 | 20.0% | -10.0 pp (-16.7, -2.8) | +18.3 pp (+11.1, +25.0) | -0.39 (-0.75, -0.04) |
| cc_pas | 5.0% | 3.83 | 3.3% | -28.3 pp (-33.3, -22.2) | +0.0 pp (-5.6, +5.6) | -0.13 (-0.20, -0.06) |
| lp_pas | 15.0% | 4.11 | 5.0% | -18.3 pp (-25.0, -12.5) | +10.0 pp (+0.0, +19.4) | +0.16 (+0.01, +0.28) |
| rr_pas | 3.3% | 3.88 | 0.0% | -30.0 pp (-39.0, -19.4) | -1.7 pp (-5.6, +2.8) | -0.07 (-0.17, +0.03) |
| fix_l1 | 6.7% | 3.87 | 5.0% | -26.7 pp (-33.3, -19.4) | +1.7 pp (-4.2, +8.3) | -0.08 (-0.23, +0.04) |
| fix_l2 | 1.7% | 3.54 | 6.7% | -31.7 pp (-39.0, -22.2) | -3.3 pp (-8.3, +0.0) | -0.41 (-0.78, -0.05) |
| p99_nb | 55.0% | 4.04 | 6.7% | +21.7 pp (+19.4, +25.0) | +50.0 pp (+38.9, +58.3) | +0.09 (-0.22, +0.42) |
| cc_nb | 46.7% | 4.10 | 8.3% | +13.3 pp (+8.3, +19.4) | +41.7 pp (+30.6, +52.8) | +0.15 (-0.12, +0.42) |

### Cost and diagnostics by method (all eval episodes)

| method | collision | stuck | progress m/s | mean compute level | recall (persons) | brake_infeasible | fallback activations | stall-watchdog trips |
|---|---|---|---|---|---|---|---|---|
| opt_backup_t | 22.0% | 19.0% | 2.94 | 0.00 | 0.75 | 20 | 2370 | 2259 |
| clear_only | 32.0% | 16.0% | 3.26 | 0.00 | 0.75 | 30 | 2186 | 2049 |
| pas_full | 5.0% | 4.0% | 3.28 | 0.00 | 0.77 | 5 | 2482 | 2416 |
| cc_opt | 16.0% | 20.0% | 2.94 | 0.20 | 0.74 | 14 | 2282 | 2189 |
| cc_pas | 3.0% | 3.0% | 3.21 | 0.25 | 0.77 | 2 | 2406 | 2338 |
| lp_pas | 10.0% | 5.0% | 3.40 | 0.16 | 0.77 | 10 | 2166 | 2076 |
| rr_pas | 2.0% | 1.0% | 3.22 | 0.17 | 0.75 | 1 | 2630 | 2571 |
| fix_l1 | 5.0% | 9.0% | 3.20 | 1.00 | 0.69 | 5 | 1435 | 1378 |
| fix_l2 | 3.0% | 8.0% | 3.01 | 2.00 | 0.62 | 3 | 1041 | 974 |
| p99_nb | 40.0% | 9.0% | 3.27 | 0.00 | 0.77 | 31 | 2090 | 2000 |
| cc_nb | 31.0% | 13.0% | 3.29 | 0.17 | 0.76 | 22 | 2069 | 1995 |

### Per family v* at 5 % (point estimates; 5 layouts per family, CIs omitted as uninformative)

| family | opt_backup_t | clear_only | pas_full | cc_opt | cc_pas | lp_pas | rr_pas | fix_l1 | fix_l2 | p99_nb | cc_nb |
|---|---|---|---|---|---|---|---|---|---|---|---|
| stale | 4.19 | 4.09 | 6.25 | 4.38 | 7.50 | 4.19 | >=9.0 | 5.88 | >=9.0 | 2.88 | 2.88 |
| drift | 5.88 | 4.19 | 5.12 | 4.38 | >=9.0 | 5.88 | >=9.0 | 3.25 | >=9.0 | 2.88 | 4.38 |
| occl | 2.69 | 2.88 | 3.06 | 2.80 | 4.75 | 3.25 | 5.88 | 7.50 | 3.00 | <2.5 | <2.5 |
| stall | >=9.0 | 7.50 | >=9.0 | 7.50 | >=9.0 | >=9.0 | >=9.0 | >=9.0 | >=9.0 | >=9.0 | 6.25 |

Reading guide for section 3: the first two tables are the E3-protocol v* (isotonic, cluster bootstrap over 5 layouts). They are included for H2 and for continuity but are fragile: each setpoint has 20 episodes, one collision moves v* by metres, and the paired CIs can exclude 0 spuriously (cc_pas - pas_full at 1 %). The robust-contrast table (60 episodes per method pooled over 5.5/7/9 m/s) is the decision basis: PAS-family arms all sit at 2-7 % collisions; cc_pas equals pas_full on collisions and is 0.13 m/s slower; adaptivity adds nothing, a fixed lite level or an environment heuristic does equally well (P3), and the learned head does not beat the analytic one. P4 (full F_c vs p99 quantile) was not tested.

## 4. Multi-rate metamorphic test (NavPilot)

Reference = the 200 Hz eval episodes (same seeds, same code, event tick 5 ms in every arm); the plant is integrated every 1/2/4 ticks (200/100/50 Hz) with a zero-order-hold command and exactly aggregated OU wind noise. Planner, sensors, collision test (50 Hz swept segment) and RNG streams are identical.

| plant | pairs | collision agreement | Wilson 95% | disagreements (ref>x / x>ref) | |dmin_clear| mean / median / p95 (cm), all pairs | same, pairs both non-collision | |d progress_m| mean (m) | verdict (>=99 % and <= 2 cm) |
|---|---|---|---|---|---|---|---|---|
| 100 Hz | 120 | 83.3% | 75.7-88.9% | 6 / 14 | 14.2 / 8.3 / 47.6 | 12.2 / 7.8 / 35.2 | 4.87 | FAIL |
|  ... collision RATE ref 23/120 vs 31/120 (McNemar exact p = 0.12) | | | | | | | | |
| 50 Hz | 120 | 85.0% | 77.5-90.3% | 5 / 13 | 14.0 / 8.7 / 42.6 | 11.8 / 8.7 / 38.2 | 4.57 | FAIL |
|  ... collision RATE ref 23/120 vs 31/120 (McNemar exact p = 0.10) | | | | | | | | |
| CHAOS FLOOR: 200 Hz, start y + 1 mm | 120 | 84.2% | 76.6-89.6% | 6 / 13 | 14.9 / 8.6 / 41.0 | 12.8 / 8.5 / 36.0 | 5.58 | FAIL |
|  ... collision RATE ref 23/120 vs 30/120 (McNemar exact p = 0.17) | | | | | | | | |

Interpretation: the acceptance rule (>= 99 % collision agreement and <= 2 cm min-clearance difference) cannot be satisfied in this closed loop: even a 1 mm initial offset at the reference rate gives 84.2 % agreement and a 14.9 cm mean clearance difference. The 100/50 Hz plants are inside that floor on every column (agreement 83.3 / 85.0 vs 84.2 %; collision-rate shifts 23 -> 31 / 31 vs 23 -> 30 for the 1 mm floor, McNemar p 0.12 / 0.10 / 0.17). **Verdict:** FAIL under the stated rule, but the rule is uninformative; replace it by "agreement and clearance difference not worse than the chaos floor (equivalence margin from the floor's CI) and per-setpoint collision rates within the paired CI", then a lower plant rate is acceptable here (NavPilot only; this also needs the 99 % check on per-run clearance *with the plant-only change*, which the 5 ms event tick fixed in all arms allows). Expected saving in NavPilot: negligible, since planner and sensors (not the plant) dominate the runtime. The .NET spec is in `results/MULTIRATE_NET_SPEC.md` (it adds the chaos-floor control: RunSeed fixed, 1e-9 m perturbation) and notes that the plant rate is not a config knob today.

## 5. Physics profile (v1 vs CAD AOS7), cheap DEV row

DEV layouts [320, 321, 322], 4 families, 5 setpoints; v1 = frozen default (1.383 kg, T/W 6.38), CAD = `aos7_v5_f90_6s.json` (1.843 kg, T/W 4.79). In NavPilot the horizontal authority is tilt-limited (a_h_max = 8.23 m/s^2 for both: the thrust limit is 36 m/s^2 at T/W 4.79), so the profile enters only through drag/mass (quadratic drag term) and nothing else.

| method | profile | v* @5 % | v* @1 % | collisions by setpoint [2.5, 4.0, 5.5, 7.0, 9.0] | achieved |
|---|---|---|---|---|---|
| opt_backup_t | v1 | 2.95 | 2.59 | 0/12 2/12 3/12 7/12 4/12 | 1.6 2.6 3.2 2.9 3.5 |
| opt_backup_t | CAD | 2.95 | 2.59 | 0/12 2/12 5/12 4/12 4/12 | 1.7 2.5 3.1 3.2 3.5 |
| pas_full | v1 | 3.40 | 2.68 | 0/12 2/12 0/12 1/12 1/12 | 1.7 2.7 3.3 3.7 4.0 |
| pas_full | CAD | 3.22 | 2.64 | 0/12 2/12 1/12 1/12 1/12 | 1.9 2.7 3.5 3.8 4.1 |

Reading: no measurable change (opt_backup_t identical, pas_full 3.40 -> 3.22, within one collision). As expected from `navpilot/vehicle.py`: a_h_max is tilt-limited (8.23 m/s^2) for both profiles, so only the mass-scaled quadratic drag differs; the lower T/W and the 1.6x inertia of the CAD build act in the .NET RK4 plant, not here. Do not read this as evidence that v* is insensitive to the CAD profile.

## 6. Why NavPilot may over-state high-speed failure relative to the .NET harness (code-read hypotheses, UNKNOWN until tested)
Orchestrator, from D4's readout: NavPilot twin collides 70/90/60 % at 5/6/8 m/s vs 10/40/40 % for the .NET harness on the same missions; mission rank correlation about 0 or negative. Reference code: `Tools/CraneHarness/navpilot_bridge.py`, `Assets/Scripts/Drones/Autonomy/Core/{AutonomyConfig,Planning,AutonomousMission}.cs`. Ranked by how much I expect each to matter for D3; none has been tested.

| # | difference | NavPilot | .NET harness | expected direction / link to D3 |
|---|---|---|---|---|
| 1 | vertical freedom | fixed altitude, +-0.35 m slab (2.0 m) | `ClimbDeg {-12,0,12}`, altitude band `AltMinAgl 1.0 .. AltMaxAgl 4.5`, `WAlt` term; 3D capsules | .NET can climb or descend around thin branches and oblique limbs that block the 2.5D slab: NavPilot over-states. D3's dominant failure (thin unmapped branches, section 1) is the case where this matters most |
| 2 | prior use | prior enters the belief at w = 1 (opt_backup/pas), stale/phantom priors by construction in S1/S3/S4 | `PriorUse = "route"`: prior used for cost-to-go only; twin prior = stems >= 6 cm, no phantoms, no alignment drift | the E3 families trust a wrong prior in the belief by design; .NET would not. Map staleness removes 77 % of D3 baseline collisions, so this difference alone can move NavPilot rates a lot |
| 3 | scenario construction | adversarial families (unmapped 2.5-4 cm branches across the only gap, 60 % invalid stereo, drift ramps) | forest cross routes (`cross_k`, corridor) | D3 collision rates (27 % baseline) are a property of these families, not forests |
| 4 | braking model | brake table simulated on the point-mass P loop (`tau_track` 0.30 s, `a_h_max` 8.23 m/s^2 tilt-limited incl. drag, jerk 30), dt 5 ms | `StopRule.Brake` from `SimulateBrakeTable` on the RK4 rotor plant + cascade; rollout `TrackTau` 0.33; path brake `BrakeDecel` 7, `RecursiveFeasibility`/`TurnBrakeTable`/`BrakeAlongPath` default off | HARNESS.md expects the point-mass brake to be shorter, i.e. NavPilot should collide less, the opposite of the observed gap: not the cause of the over-statement. The `brake_infeasible` share is similar (12 of 13 NavPilot baseline collisions by my analogue label vs 9 of 11 in .NET), so the single-cycle-certificate failure mode is shared |
| 5 | latency injection | E3 families use heavy/severe tails (p99 160 / 560 ms) plus detector GPU share; the twin uses the nominal tail (p99 47 ms) and no detector | `PlanMedianS 0.025, PlanSigma 0.25, PlanPStall 0.003` (= nominal), depth 45 ms, lidar 65 ms, estimator 10 ms; `WatchdogS 0.15`, `EstimateStaleS 0.10` | D3 imposes a harsher tail than the .NET default; the twin gap cannot be latency (both nominal). D3's finding that latency is not the dominant factor held even with heavy tails |
| 6 | sensing / map | stereo 96-bin 2D z-buffer, free evidence 8 m; LiDAR 720 bins, free to 12 m; map res 0.1 m, carve cap 12 m; per-object per-frame dropout | depth 80x50 3D rays (55 deg vfov), `FreeEvidenceM 8`, lidar 3D (96 deg vfov, range 20 m), `MapRes 0.2`, `MapWindowXY 11` m half-window, `InflateExtra 0.15` + half voxel; per-ray pixel-footprint dropout (`ThinW50Px 3.0`, same) | 2x coarser map inflates obstacles in .NET (more conservative); 11 m planning window caps known-free distance below NavPilot's 12-14 m horizon at 8 m/s |
| 7 | body radius | 0.25 m (twin: 0.2479) | 0.2479 m (`AutonomousMission.BodyRadius`; AUTONOMY_STATUS says 0.268) | negligible |
| 8 | plant and wind | point mass, OU wind `light_breeze` in E3 families (twin: calm) | RK4 rotor plant + cascaded controller, calm in the batches | D3 families add wind the .NET batches do not |
Suggested cheap checks (not run): (a) NavPilot with a +-0.35 m to +-1.0 m slab and a climb primitive; (b) E3 family S0 with `prior as cost-to-go only`; (c) re-run the D4 twin with `MapRes 0.2` and `MapWindowXY 11`.

## 7. Candidates for .NET validation at 5-8 m/s (top 2; spec only, not run)
D3 found no compute-adaptive policy that beats its non-adaptive counterpart, so the two candidates are the two *mechanisms* that DEV supports, each with the control that isolates it. The .NET harness has no compute levels and no PAS, so both need small additions behind default-off flags.
1. **PAS (perception-aware backup set) + recursive feasibility.** NavPilot `pas_full`: collisions 5.0 % vs 33.3 % (opt_backup_t) pooled over 5.5/7/9 m/s, -28.3 pp (-38.9, -16.7). Spec: forest 1337 (manifest of `recfeas_sweep.json`), missions `cross` x10 + `corridor`, planner `map_rhp`, speeds 5, 6, 8, run_seeds 1 and 2, calm wind, `split: dev`. Arms (same pair_key): `ctrl` = `{"RecursiveFeasibility": true, "BrakeAlongPath": true, "TurnBrakeTable": true}`; `pas` = ctrl + new `{"PasEnabled": true, "PasRConfM": 4.0, "PasDStructM": 1.0}` where a cell counts as known-free for the backup stop only if it was observed free from within `PasRConfM` metres or lies farther than `PasDStructM` from any mapped structure (port of `AdvStack._untrusted`). 66 runs per arm, 132 runs, about 1 core-h. Success metric: collision rate and min hard clearance vs `ctrl`, paired; prediction from NavPilot (not transferable by itself): the PAS gain shrinks in .NET because of difference 1 above.
2. **Lite compute level with p99 tau (fix_l2 / rr_pas family).** NavPilot fix_l2: 1.7 % collisions (-3.3 pp vs pas_full, CI -8.3, 0.0) at -0.41 m/s progress; rr_pas 3.3 % at -0.07. Spec: arm `lite` = `pas` + `{"Headings": 12, "PlanMedianS": 0.0165, "PlanStallMinS": 0.024}` (scale 0.66 and 0.60 of the nominal tail; `AgeQuantile` 0.99 as is); optional arm `rr` = `lite` only when fewer than 70 % of the headings have >= 6 m free, which needs a new per-cycle `Headings` switch (RoboRun-style). Same missions/speeds/seeds; 66 runs per arm. Because difference 5 makes the .NET latency nominal already, the expected benefit is small; the arm is mainly a test of whether fewer headings cost clearance.
Escalation rule from the plan stands: only candidates that pass the .NET tier get a Unity batch.

## 8. Files, tests, reproduction
- New (this task; nothing committed): `Tools/NavPilot/stacks/compute_gov.py`, `Tools/NavPilot/experiments/d3_episode.py`, `d3.py`, `test_d3.py`, `Tools/NavPilot/PREREG_E4.md` (DRAFT), `Tools/NavPilot/results/D3_dev.md`, `results/D3_dev_tables.md` (generated), `results/MULTIRATE_NET_SPEC.md`, raw rows `results/raw/d3_{diag,tune,eval,multirate,profile,chaos}.jsonl`, `d3_tuned.json`, `d3_lp.json`, `d3_diag_contaminated*.{json,jsonl}`; README section appended in `Tools/NavPilot/README.md`.
- Tests: `experiments/test_d3.py` (d3_episode == e3_episode bit-for-bit on opt_backup_t and pas_full, cf = (), 200 Hz): PASSED. Smoke run of every counterfactual/policy/plant-rate path before the sweep.
- Reproduce: `fsrun python experiments/d3.py run diag|tune|train|eval|multirate|chaos` (resumable), then `fsrun python experiments/d3.py report`; regenerate this file's tables from `D3_dev_tables.md`.
- **Data integrity note:** the physics profile file was swapped (10:38) while the diag stage was running; the episodes after that read the CAD profile through the old default path. 184 of 432 diag rows were discarded (`d3_diag_contaminated.json` documents the cut; backup kept) and re-run on v1 by the next stage. All reported rows carry `profile` and v1 unless labelled.
- Limits: 5 layouts per policy cell and 4 diag layouts (clusters), 20 episodes per setpoint; CEM is tiny (3 x 6 candidates x 16 episodes); the compute-level model is partly assumed (section 2); NavPilot over-states failure vs .NET (section 6); recursive feasibility is not implemented in NavPilot; the SUPER backup trajectory is represented only by the known-free backup stop.
- Ops notes: (i) fsrun waited about 2 h for a slot (slot churn between chained jobs). (ii) The quick lane (`/tmp/forestsearch-slots/quick`) is currently blocked: a `VBCSCompiler` (dotnet Roslyn server) started from a quick-lane `dotnet build` inherited the lock fd and holds it, so all later `fsrun-quick` jobs queue forever; I did not kill it.
