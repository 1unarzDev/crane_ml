# D4 retrospective: multi-fidelity experiment selection and ranking transfer

_2026-11-10, framework/D4 agent. Logged data only: no simulator was run, no Unity, no new held-out run. Evidence labels follow
Docs/ResearchOrchestration/CONTRACTS.md. Everything numeric below is IMPLEMENTED analysis of existing logs (exploratory) or
**SYNTHETIC-CALIBRATED** (clearly marked; it says nothing about real transfer). Reproduce with the commands in section 9._

## 0. Verdict in five lines

1. **Paired multi-fidelity data are thin but no longer zero.** The ops agent's stage-2 twin now exists: 70 NavPilot 2.5D twin runs paired with
   .NET runs for **one config** (map_rhp, legacy flags), one forest (1337), 10 cross missions x 7 speeds x 1 seed. There is **no cell with
   two fidelities for more than one config**, so a Kendall tau *across configs* (the real question) is **not computable (NOT_RUN)**.
2. What can be computed is rank agreement across **missions** (the 10 routes), per speed: L1(twin) vs L2(.NET) and L2 vs L3(Unity). It is
   **indistinguishable from zero in every band** (n = 10-12 units; CIs about +-0.45; section 3), and the twin disagrees strongly in *level* at
   5-8 m/s (twin collides 60-90 %, .NET 10-40 %).
3. Trusting the cheap tier at face value picks the wrong speed on the real paired data (twin says 2.5 m/s, .NET says 4.0 m/s; regret 1.13
   utility units, tau over speeds 0.33; section 5).
4. A replay simulator and six selectors are implemented and tested. On **synthetic-calibrated** tables, the cost-aware AR1 selector reaches
   mean regret <= 1.0 in 0.25-1.0 h when the cheap tier transfers (tau 0.5-0.9), versus 3 h for halving over seeds and 0.75-1.5 h for "cheap screen then
   top-k at the expensive tier"; when it does not transfer (tau 0.1) AR1 stalls and the cheap screen + top-k wins. None of this is evidence about NavPilot vs .NET (section 6).
5. Exact paired-data request for the ops agent is in section 8 (Tier A fits the planned reduced recfeas budget; about 1.3 .NET core-h).

## 1. Inventory: which cells have two or more fidelities

Fidelities: L1 NavPilot 2.5D point mass; L2 CRANE .NET harness (3D capsules, RK4 plant); L3 CRANE Unity colliders. L0 (analytic envelope) has
no logged rows. Split labels: DEV, **RETRO-HELDOUT** (rows of a frozen held-out run, reused only for retrospective analysis; no tuning on them, no new run).

| Source (loader in mf/data.py) | Fidelity | Split label | n | Configs | Forests | Speeds [m/s] |
|---|---|---|---|---|---|---|
| NavPilot E3 held-out (`e3_heldout_*.jsonl`) | L1 | RETRO-HELDOUT | 800 | 5 stacks | 8 synthetic layouts (90400-90407), 4 families | 2.5, 4, 5.5, 7, 9 |
| NavPilot E3 tune | L1 | DEV | 480 | 6 stacks (5 tune draws each) | layouts 300-301 | 6, 9 |
| NavPilot E2 held-out | L1 | RETRO-HELDOUT | 464 | 7 stacks (map_rhp 112 rows) | 8 layouts (90200-90207) | 3, 4, 7 |
| NavPilot dev sweep (`dev_shard*`) | L1 | DEV | 864 | one stack x branch/sensor/latency conds | 6 dev seeds | 3, 5, 7, 10 |
| NavPilot 2.5D twin of CRANE forest 1337 (ops stage 2) | L1 | DEV | 70 | map_rhp (legacy flags) | 1337 | 2, 2.5, 3, 4, 5, 6, 8 |
| CRANE .NET (`ah_swA/swB/m5a/m5b`, snapshot) | L2 | DEV | 96 | map_rhp (flags off) | 1337 | 2, 2.5, 3, 4, 5, 6, 8 |
| CRANE Unity M5 (`summary.jsonl`, de-duplicated) | L3 | DEV | 48 | map_rhp | 1337 | 2.5, 4, 6 |

The /tmp CRANE rows are volatile, so `data.snapshot()` copies them to `~/.cache/frontier/mf_data/` with a sha256 manifest.

**Cells with results at >= 2 fidelities** (analysis.json, `inventory`):

| Cell definition | Cells | Fidelity pairs, n per fidelity |
|---|---|---|
| strict (config, forest, speed) | 7 | all one config (map_rhp) and forest 1337. L1/L2 at 2, 3, 5, 8 m/s (n 10 / 12); L1/L2/L3 at 2.5 (10 / 24 / 24), 4 (10 / 12 / 12), 6 (10 / 12 / 12) |
| run-level pairs (same mission, seed, speed) | 10 / speed for L1-L2; 10-24 for L2-L3 | |
| strict with **more than one config** | **0** | the NavPilot E2/E3 stacks and layouts (synthetic, 90xxx) never meet CRANE forest 1337 |
| relaxed (config, speed), unpaired scenario | NavPilot E2 `map_rhp` at 3 (n 24) vs .NET at 3 (n 12) only exact-speed overlap; 7 m/s sits between .NET 6 and 8 | |

So the benchmark is: **one config, one forest, 7 speeds, with the config axis missing.** Selector replays over a config set can only be
run on synthetic tables (section 6) or on the 7-speed toy (section 5).

## 2. Statistics used

* Kendall tau-b, `judge.stats.kendall_tau` (DecisionCal, read-only import from `~/worktrees/decisioncal/judge`, fallback in `analyze.py`).
  Unit = mission (10 for the twin pairs, 12 for L2-L3). The per-mission value is the mean over run seeds of: progress fraction, min hard
  clearance, or failure flag.
* CIs: percentile bootstrap over units (2000), **plus** the Fieller (1957) small-sample interval (`atanh tau`, var 0.437 / (n - 4)) and a
  permutation p. With n = 10 and many ties the percentile bootstrap is anti-conservative (it can exclude 0 at tau = -0.5); quote Fieller.
* Noise ceiling (DecisionCal `rank_agreement` idea): seed-to-seed tau of the mission ranking *within* a fidelity. Computable only where there
  are 2 seeds (.NET and Unity at 2.5 m/s); the twin pairs have 1 seed, so **no ceiling exists for L1-L2**.

## 3. Rank agreement (tau-b) between fidelities, per speed band

### 3a. L1 NavPilot twin vs L2 .NET, across 10 missions (config fixed; seed 1)

Reading: tau of the mission ordering. NA = one side constant (all missions identical, tau undefined).

| v [m/s] | progress frac | min hard clearance | failure flag | Fieller 95 % CI (progress / clearance / failure) | L1 vs L2 collision rate |
|---|---|---|---|---|---|
| 2 | NA | -0.38 | NA | - / [-0.73, 0.13] / - | 0.00 vs 0.00 |
| 2.5 | +0.45 | +0.16 | 0.00 | [-0.04, 0.77] / [-0.36, 0.60] / [-0.48, 0.48] | 0.00 vs 0.00 |
| 3 | +0.11 | -0.07 | -0.22 | [-0.39, 0.57] / [-0.53, 0.43] / [-0.64, 0.30] | 0.10 vs 0.00 |
| 4 | NA | +0.16 | NA | - / [-0.36, 0.60] / - | 0.10 vs 0.00 |
| 5 | -0.52 | -0.38 | -0.76 | [-0.80, -0.05] / [-0.73, 0.13] / [-0.91, -0.44] | **0.70 vs 0.10** |
| 6 | -0.23 | -0.11 | +0.33 | [-0.64, 0.29] / [-0.57, 0.39] / [-0.18, 0.70] | **0.90 vs 0.40** |
| 8 | -0.29 | -0.11 | -0.58 | [-0.68, 0.22] / [-0.57, 0.39] / [-0.83, -0.14] | 0.60 vs 0.40 |

No band shows positive agreement. Permutation p-values are 0.06-1.0 (smallest: failure flag at 5 m/s, tau = -0.76, p = 0.06, i.e. the
twin tends to rank the *hard* missions as easy there). This is **mission difficulty transfer at a fixed config**, a weaker proxy than config
ranking, and n = 10 with one seed is far below the power needed (section 3d). The run-level agreement is the clearer statement:
**same success flag in 3/10 runs at 2 m/s, 2/10 at 4, 1/10 at 5, 6/10 at 6, 2/10 at 8**, and the twin gets "stuck" (local minimum) at 2-4 m/s
where .NET succeeds (success 0.2-0.5 vs 0.8-1.0), so the success flag is not comparable at low speed. This matches the ops agent's `fidelity_report.md`
("DIVERGE" at 2-6 m/s on success, 5-6 m/s on collision; "agree" at 8 m/s).

### 3b. L2 .NET vs L3 Unity, across 12 missions

| v [m/s] | progress frac | min hard clearance | failure flag | seed-to-seed ceiling (progress; a = .NET, b = Unity) | collision rate L2 vs L3 |
|---|---|---|---|---|---|
| 2.5 (2 seeds) | -0.19 (p 1.0) | -0.33 (p 0.15) | -0.20 | a = -0.09, b = +0.72 | 0.00 vs 0.00 |
| 4 (1 seed) | NA (all .NET missions succeed) | +0.18 | NA | n/a | 0.00 vs 0.00 (Unity: 1 stuck) |
| 6 (1 seed) | -0.23 | +0.03 | -0.19 | n/a | 0.42 vs 0.17 |

Run-level, same (mission, speed, seed): same success flag 19/24 (2.5), 11/12 (4), **5/12 (6)**; at 6 m/s the .NET harness collides in 5 runs,
Unity in 2, both in 1. The ceiling at 2.5 m/s for the .NET ranking is already -0.09: **at these n the mission ranking inside one fidelity is noise**,
so a low cross-fidelity tau cannot be read as a fidelity gap.

### 3c. L1 internal stability across speed (NavPilot E3 held-out; RETRO-HELDOUT)

Stack ranking (5 stacks) at one speed vs another, cluster bootstrap over the 8 layouts. This is the within-simulator baseline for "how much does
the ordering move with speed".

| speeds [m/s] | tau on collision rate (95 % boot) | tau on safe progress (95 % boot) |
|---|---|---|
| 2.5 vs 4 | -0.53 (collision counts are almost all 0 at 2.5: ties) | +0.40 [0.2, 1.0] |
| 4 vs 5.5 | +0.50 [-0.25, 0.67] | +0.60 [0.0, 0.8] |
| 5.5 vs 7 | +0.45 [-0.20, 0.82] | +0.60 [-0.2, 1.0] |
| 7 vs 9 | +0.60 [-0.12, 0.89] | +0.60 [-0.4, 0.8] |
| 4 vs 7 | +0.22 [-0.43, 0.95] | +0.20 [-0.4, 1.0] |

Over the 20 (stack, family) cells: tau 0.00 (2.5 vs 4), 0.26 (4 vs 5.5), **0.73 (5.5 vs 7), 0.76 (7 vs 9)**, 0.04 (2.5 vs 9). With 5 stacks no interval
excludes 0 except where the lower speeds are degenerate. Within L1 the ordering is stable among the high speeds and not usable at 2.5-4 m/s
(no collisions to rank). Note that within L1 the collision-rate ranking carries information only once collisions happen, so low-speed bands are degenerate for any collision-based tau (a problem for H1's low-speed band).

### 3d. Power, honestly

Monte Carlo (bivariate-normal copula, 150 draws, 200 bootstraps; `run_retro.py --power-only`):

| n units | CI half-width at tau 0.6 | P(CI excludes 0 \| tau 0.6) | CI half-width at tau 0.2 | P(CI excludes 0 \| tau 0.2) |
|---|---|---|---|---|
| 5 | 0.65 | 0.21 | 0.88 | 0.03 |
| 10 | 0.40 | 0.71 | 0.53 | 0.09 |
| 12 | 0.34 | 0.82 | 0.46 | 0.11 |
| 20 | 0.24 | 0.98 | 0.33 | 0.23 |
| 40 | 0.14 | 1.00 | 0.21 | 0.46 |

With about 10 units, the intervals around tau = 0.6 and tau = 0.2 overlap in most draws, so **H1's "tau >= 0.6 at <= 4 m/s, <= 0.3 at >= 6 m/s" cannot be
resolved with 10 configs** (consistent with the literature critic's +-0.45). The prospective test needs either >= 20 configs or a pairwise
design (per-mission paired differences between two configs; section 8).

## 4. Level comparison L1 vs L2 (unpaired scenario, collision rate with Wilson 95 %)

| L1 source | speed | L1 collision | .NET (speeds within 1 m/s) | .NET collision |
|---|---|---|---|---|
| E2 map_rhp (RETRO-HELDOUT) | 3 | 1/24 (0.4-20 %) | 2-4 m/s | 0/60 (0-6 %) |
| E2 map_rhp (RETRO-HELDOUT) | 7 | 54/88 = 61 % (51-71 %) | 6 and 8 | 9/24 = 38 % (21-57 %) |
| NavPilot dev sweep (harsh conds, DEV) | 5 | 147/216 = 68 % (62-74 %) | 4-6 | 7/36 = 19 % (10-35 %) |
| NavPilot twin (same forest, same missions) | 5 / 6 / 8 | 70 % / 90 % / 60 % | same missions | 10 % / 40 % / 40 % |

NavPilot is more pessimistic than .NET by roughly 1.6x at 7-8 m/s (unpaired) and 2-7x at 5-6 m/s on the twin pairs. A level shift alone
does not break rankings; it does break "choose the fastest speed with collision <= 5 %", which is what the real-data replay shows next.

## 5. Real-data replay: choosing a speed limit with the twin (7 configs)

`real_replay.py`: config = speed limit; L1 pools = twin runs (10 per speed), L2 pools = .NET runs on the same 10 missions (seed 1). Run utility = mean_speed x progress
if no collision, else -8. Costs = measured medians (twin 4.5 s, .NET 48 s per run, from `costs.jsonl` and the row `WallS`). The L2 truth is the pool
mean, so this is **in-sample**.

| speed | 2.0 | 2.5 | 3 | 4 | 5 | 6 | 8 |
|---|---|---|---|---|---|---|---|
| L1 twin utility | 1.79 | 2.16 | 1.52 | 2.12 | -4.19 | -6.65 | -2.14 |
| L2 .NET utility | 1.79 | 2.18 | 2.20 | **3.32** | 2.40 | -0.52 | -0.04 |

Twin at face value: 2.5 m/s (regret 1.13 at L2). tau over the 7 speeds 0.33. Mean regret versus compute (150 random replays of query order,
budget 1 h = 75 L2 runs):

| selector | 0.1 h | 0.25 h | 0.5 h | 1 h | P(finds L2 best by 1 h) |
|---|---|---|---|---|---|
| random (L2) | 1.98 | 1.76 | 0.73 | 0.00 | 1.00 (pool exhausted) |
| all-high | 2.80 | 1.55 | 0.35 | 0.00 | 1.00 (pool exhausted) |
| low-only (face value) | 1.13 | 1.13 | 1.13 | 1.13 | 0.00 |
| halving over seeds, L2 only | 2.81 | 1.12 | 1.12 | 1.12 | 0.46 |
| successive halving across fidelities | 1.13 | **0.18** | 0.18 | 0.18 | 0.84 |
| cheap screen + top-3 at L2 | 0.38 | 0.35 | 0.35 | 0.35 | 0.69 |
| AR1-BLR (3 features) | 1.93 | 1.43 | 1.50 | 1.53 | 0.00 |

Honest reading: with 7 configs and 10 runs per pool, "random" and "all-high" simply exhaust the L2 pool (70 runs = 56 min) and win at 1 h by construction;
the informative columns are 0.25-0.5 h. A cheap screen followed by a few L2 runs beat everything at 0.1-0.5 h; halving across fidelities was
next; the AR1 linear model with 3 speed features **failed** here (too few configs, the twin's level shift and sign flips at 5-8 m/s are not linear in speed).
Treat as a feasibility demo of the harness, not a result.

## 6. Replay simulator, selectors and the synthetic-calibrated study

### 6a. Implementation (`mf/replay.py`)

* `Env`: a selector queries `(config, fidelity)`; the environment returns the next *cached* outcome of that pool without replacement and charges
  the tier's cost from `costs.jsonl` (`costs.cost_per_unit`: measured median if present, else the geometric mean of the audit range). Exhausted pools
  return None and cost nothing. Regret is simple regret at L2: max f2 - f2(recommendation), recorded against cumulative compute-hours.
* Selectors: **random** (L2); **all-high** (round-robin L2); **low-only** (cheap tier at face value); **sh-seeds-L2** (Hyperband-style halving over seeds, L2 only);
  **succ-halving** (across fidelities, rungs L1 x n0, L1 x 2 n0, then L2 x 2/4/8/16, tuned over eta in {2, 3, 4} x n0 in {2, 4, 8} on separate tuning seeds);
  **screen+topk** (full cheap screen with 8 samples, then L2 on the top 10: the "near-free cheap tier" strong baseline from the experiment-choice study);
  **AR1-BLR** (cost-aware two-fidelity Bayesian linear model f2 = rho f1 + delta on 14 config features: stack one-hot, speed, speed^2, margin, interactions;
  acquisition = knowledge gradient of m samples per tier divided by their cost; the L1 route acts on f2 only through rho f1).
* Reused, not rewritten: tau-b from DecisionCal `judge.stats`. Cited but not imported: `judge/selection.py` (`rank_agreement`, `split_half_rank`, `pick_cost`),
  `judge/compare.py` (`tier_agreement`), `expchoice/Tools/ExperimentChoice/twostep/twostep.py` and `learner/{acquire,gpmodel}.py`.
  **Not implemented:** exact two-step lookahead over (item, tier) queries and a GP AR1 (the BLR is the cheap variant the task allowed); MF-GP-UCB, MF-MES, BOHB, Marco-style
  entropy search, and the failures-per-hour objective (no `Tools/Adversarial` exists yet): all NOT_RUN. The old crane_ml MF-BO is deliberately excluded.
* Baselines the experiment-choice and DecisionCal sessions asked for: "cheap tier at face value" = `low-only`; "full cheap screen then expensive tier on the top few" =
  `screen+topk` (a one-step proxy for their two-step lookahead, which they found essentially optimal at a near-free cheap tier); noise ceiling = section 2 / `analyze.split_half`.
  Their negative result "on boat twins nothing beat the cheap tier at face value" is a *prior* here: section 5 finds the opposite on this pair of simulators (face value was wrong).

### 6b. The synthetic-calibrated tables (label: SYNTHETIC-CALIBRATED)

* **Real backbone:** per (stack, speed) collision probability and achieved progress of NavPilot E3 held-out (RETRO-HELDOUT rows), linearly interpolated in speed, 5 stacks x 9 speeds x 3 margin levels = **135 configs**.
  The margin knob and its effect are **synthetic**. Run utility = progress if no collision, else -8.
* **L2 transfer (assumed; measured tau(L1, L2) over configs is UNKNOWN):** logit p2 = rho logit p1 + beta + delta(x), a2 = 0.9 a1, beta = -0.35 (level shift toward the observed .NET/NavPilot gap of about 0.6x at 7-8 m/s);
  delta = stack offset + idiosyncratic noise, times a speed gain that is constant, or (speed-divergent) zero below 5 m/s and growing above, as H1 predicts.
  Scenarios span the realised tau between f1 and f2 over configs. The L1 and L2 pools are 40 and 16 cached runs per config; costs are the audit's 1.33 s (L1) and 32 s (L2).
* 60 replications per scenario (new delta draw, pool draw and selector randomness each), budget 6 h.

costs (s/query): L1 1.33, L2 32.0; tuned halving: eta=3, first L1 rung n0=2; reps=60; eps=0.5

| scenario | tau(f1,f2) all / <=4 / >=6 m/s | regret of trusting L1 (face value) |
|---|---|---|
| faithful | 0.78 / 0.83 / 0.77 | 0.12 |
| moderate | 0.47 / 0.60 / 0.40 | 0.89 |
| speed_divergent | 0.50 / 0.92 / 0.28 | 2.08 |
| unfaithful | 0.09 / 0.12 / 0.09 | 3.45 |

**faithful**: mean simple regret at L2 (+- s.e.) vs compute-hours

| selector | 0.25 h | 0.5 h | 1 h | 2 h | 4 h | 6 h | h to median<=eps | h to mean<=1.0 | hours used |
|---|---|---|---|---|---|---|---|---|---|
| random | 1.59 (0.12) | 1.75 (0.12) | 1.70 (0.13) | 1.67 (0.15) | 1.60 (0.14) | 1.42 (0.14) | >6 | >6 | 6.00 |
| all-high | 1.73 (0.14) | 1.69 (0.14) | 1.70 (0.13) | 1.69 (0.14) | 1.33 (0.13) | 1.38 (0.13) | >6 | >6 | 6.00 |
| low-only | 1.14 (0.14) | 0.87 (0.11) | 0.51 (0.08) | 0.35 (0.04) | 0.35 (0.04) | 0.35 (0.04) | 0.75 | 0.5 | 2.00 |
| sh-seeds-L2 | 1.48 (0.02) | 1.48 (0.02) | 1.48 (0.02) | 1.29 (0.13) | 0.98 (0.10) | 0.98 (0.10) | >6 | 3 | 2.19 |
| succ-halving | 1.32 (0.15) | 0.89 (0.11) | 0.77 (0.10) | 0.77 (0.10) | 0.77 (0.10) | 0.77 (0.10) | 0.75 | 0.5 | 0.63 |
| screen+topk | 1.14 (0.14) | 1.02 (0.11) | 0.62 (0.08) | 0.39 (0.05) | 0.39 (0.05) | 0.39 (0.05) | 0.75 | 0.75 | 1.82 |
| AR1-BLR | 0.57 (0.08) | 0.63 (0.08) | 0.49 (0.08) | 0.27 (0.05) | 0.12 (0.02) | 0.13 (0.02) | 0.25 | 0.25 | 6.02 |

**moderate**: mean simple regret at L2 (+- s.e.) vs compute-hours

| selector | 0.25 h | 0.5 h | 1 h | 2 h | 4 h | 6 h | h to median<=eps | h to mean<=1.0 | hours used |
|---|---|---|---|---|---|---|---|---|---|
| random | 2.00 (0.18) | 2.19 (0.18) | 2.31 (0.19) | 2.23 (0.22) | 1.96 (0.19) | 1.44 (0.13) | >6 | >6 | 6.00 |
| all-high | 2.40 (0.21) | 2.26 (0.20) | 2.18 (0.19) | 1.90 (0.19) | 1.31 (0.14) | 1.23 (0.12) | >6 | >6 | 6.00 |
| low-only | 1.78 (0.21) | 1.50 (0.19) | 1.37 (0.18) | 1.23 (0.13) | 1.23 (0.13) | 1.23 (0.13) | >6 | >6 | 2.00 |
| sh-seeds-L2 | 1.79 (0.06) | 1.79 (0.06) | 1.79 (0.06) | 1.32 (0.13) | 0.93 (0.12) | 0.93 (0.12) | >6 | 3 | 2.19 |
| succ-halving | 1.97 (0.24) | 1.08 (0.13) | 1.07 (0.14) | 1.07 (0.14) | 1.07 (0.14) | 1.07 (0.14) | >6 | >6 | 0.63 |
| screen+topk | 1.78 (0.21) | 1.69 (0.18) | 0.90 (0.11) | 0.69 (0.09) | 0.69 (0.09) | 0.69 (0.09) | 1.5 | 1 | 1.82 |
| AR1-BLR | 1.07 (0.13) | 1.00 (0.11) | 0.89 (0.10) | 0.53 (0.06) | 0.49 (0.06) | 0.44 (0.05) | 2 | 0.75 | 6.02 |

**speed_divergent**: mean simple regret at L2 (+- s.e.) vs compute-hours

| selector | 0.25 h | 0.5 h | 1 h | 2 h | 4 h | 6 h | h to median<=eps | h to mean<=1.0 | hours used |
|---|---|---|---|---|---|---|---|---|---|
| random | 2.30 (0.22) | 2.51 (0.24) | 2.83 (0.25) | 2.79 (0.30) | 2.15 (0.26) | 1.27 (0.15) | >6 | >6 | 6.00 |
| all-high | 3.17 (0.27) | 2.89 (0.26) | 2.78 (0.26) | 1.86 (0.23) | 1.37 (0.18) | 1.11 (0.16) | >6 | >6 | 6.00 |
| low-only | 2.72 (0.34) | 2.29 (0.27) | 2.49 (0.30) | 2.27 (0.24) | 2.27 (0.24) | 2.27 (0.24) | >6 | >6 | 2.00 |
| sh-seeds-L2 | 2.48 (0.07) | 2.48 (0.07) | 2.48 (0.07) | 1.35 (0.19) | 0.73 (0.10) | 0.73 (0.10) | >6 | 3 | 2.19 |
| succ-halving | 2.91 (0.37) | 1.41 (0.15) | 1.26 (0.16) | 1.26 (0.16) | 1.26 (0.16) | 1.26 (0.16) | >6 | >6 | 0.63 |
| screen+topk | 2.72 (0.34) | 2.16 (0.23) | 1.09 (0.13) | 0.85 (0.10) | 0.85 (0.10) | 0.85 (0.10) | >6 | 1.5 | 1.82 |
| AR1-BLR | 1.54 (0.17) | 1.21 (0.17) | 0.85 (0.12) | 0.76 (0.13) | 0.59 (0.09) | 0.68 (0.09) | 2 | 1 | 6.02 |

**unfaithful**: mean simple regret at L2 (+- s.e.) vs compute-hours

| selector | 0.25 h | 0.5 h | 1 h | 2 h | 4 h | 6 h | h to median<=eps | h to mean<=1.0 | hours used |
|---|---|---|---|---|---|---|---|---|---|
| random | 2.63 (0.28) | 2.77 (0.30) | 3.27 (0.31) | 2.38 (0.30) | 2.22 (0.29) | 1.51 (0.19) | >6 | >6 | 6.00 |
| all-high | 3.47 (0.34) | 3.32 (0.32) | 2.96 (0.32) | 1.68 (0.21) | 1.36 (0.20) | 1.06 (0.18) | >6 | >6 | 6.00 |
| low-only | 3.53 (0.41) | 3.30 (0.39) | 3.90 (0.41) | 4.01 (0.39) | 4.01 (0.39) | 4.01 (0.39) | >6 | >6 | 2.00 |
| sh-seeds-L2 | 3.48 (0.20) | 3.48 (0.20) | 3.48 (0.20) | 1.20 (0.19) | 0.67 (0.10) | 0.67 (0.10) | 3 | 3 | 2.19 |
| succ-halving | 3.70 (0.44) | 1.61 (0.20) | 1.40 (0.18) | 1.40 (0.18) | 1.40 (0.18) | 1.40 (0.18) | >6 | >6 | 0.63 |
| screen+topk | 3.53 (0.41) | 2.54 (0.29) | 1.39 (0.18) | 0.87 (0.10) | 0.87 (0.10) | 0.87 (0.10) | >6 | 1.5 | 1.82 |
| AR1-BLR | 2.53 (0.26) | 1.94 (0.24) | 1.32 (0.14) | 1.11 (0.12) | 1.15 (0.16) | 1.20 (0.17) | >6 | >6 | 6.02 |


**Reading** (synthetic; s.e. in brackets; mean-regret threshold 1.0 utility unit is about a quarter of the speed-utility range; the grid is 0.25 h wide, so hours are +-0.25):

* When the cheap tier transfers (tau 0.5-0.9: faithful, moderate, speed-divergent), **AR1-BLR reaches mean regret <= 1.0 after 0.25 / 0.75 / 1.0 h**, vs
  halving over seeds at L2 only 3 / 3 / 3 h, the cheap screen + top-k 0.75 / 1.0 / 1.5 h, all-high > 6 h, random > 6 h. That is 33 % / 75 % / 67 % of the compute of the cheap screen + top-k (a reduction of 67 % / 25 % / 33 %); in the faithful case, where trusting L1 is right, tuned halving and low-only need 0.5 h (AR1 50 %).
  The tuned across-fidelity halving **plateaus above 1.0** in the moderate and speed-divergent scenarios (regret 1.07 and 1.26 after 0.63 h, then no further information), so it
  loses to AR1 on regret at >= 1 h and wins only below 0.5 h.
* All-high at 6 h still has mean regret 1.1-1.4: with a 135-config space, 16 runs per config at a collision rate near 20 % cannot be exhausted; compute-to-best for all-high is beyond the budget, so "30 % of all-high" is met trivially here.
* In the **unfaithful** scenario (tau 0.09): trusting L1 has regret 4.0; AR1 stalls at 1.1-1.2 (the linear delta cannot represent idiosyncratic errors); the cheap screen + top-k is best (reaches <= 1.0 at 1.5 h), then halving over seeds (3 h). The right escalation there is to stop trusting L1.
* With measured costs (L1 3.3 s, .NET 48 s) the ordering is unchanged: in the moderate scenario AR1 reaches <= 1.0 at 0.5 h, tuned halving 1 h, screen + top-k 2 h, halving over seeds 4 h; in the speed-divergent scenario AR1 1.5 h, screen + top-k 3 h, halving over seeds 4 h, tuned halving > 6 h (reached 1.04 only after the pool ran out; 40 reps).
* **Kill criterion status (>= 15 % better compute-to-best than tuned halving on the prospective recfeas test):** not testable here. On the synthetic scenarios with tau >= 0.5 AR1
  would pass it against tuned across-fidelity halving and against halving over seeds. Against the cheap-screen baseline the margin is 25-67 % but with +-0.25 h resolution and
  AR1 untuned (halving got a 9-point grid search on the moderate scenario, AR1 got none). It fails when tau is about 0.1. The prospective test must decide.

### 6c. What the synthetic study does not show

It shows the selectors work mechanically and that the ranking depends on the unknown tau and on the cheap screen's cost, not that NavPilot-to-.NET transfer is any of these. The real paired data in
section 3 (tau about 0 over missions, level gaps of 1.6-7x) put the real pair nearer *moderate-to-unfaithful* than faithful, and the twin is not even run on a second config. The backbone is
E3 held-out rows (RETRO-HELDOUT): the selectors were not tuned on them, but the surface is those rows, so the study cannot be presented as a held-out test of anything.

## 7. Claim reframing and prediction status (per NOVELTY_D4.md)

Dropped: "robotics-specific multi-fidelity selection" (Marco 2017, Cutler 2015) and failures-per-hour as a new objective (Shahrooei 2023, Koren 2021). The AR1 + information-per-cost
selector is Kennedy-O'Hagan + MF-MES-style; its value here is the cost model and the replay harness. The remaining claim is a **regime map of rank transfer and a regime-gated escalation rule**:
rank correlation drops when v x delta-tau(L1, L2) exceeds a fraction of the obstacle spacing, and the rule "run L2 only if the posterior probability of a rank reversal at L1 exceeds p".

| Prediction | Status from logged data |
|---|---|
| P1: tau(L1, L2) >= 0.6 at <= 4 m/s, <= 0.3 at >= 6 m/s, non-overlapping CIs | NOT TESTABLE: needs configs; the mission-level tau at <= 4 m/s is +0.45 / +0.11 / +0.16 / -0.07 (CIs include 0), the design cannot distinguish 0.6 from 0.2 (section 3d) |
| P2: transition speed predicted by v x delta-tau | NOT TESTABLE. Measured timing: age p99 at 5 m/s .NET 137 ms vs twin 134 ms, paired difference 4.3 ms [3.6, 5.1] (`fidelity_report.md`): the *latency* mismatch is about 4-7 ms, so a latency-driven transition predicted by v x 5 ms = 3.5 cm at 7 m/s would be far inside the obstacle spacing, yet the twin collides 1.6-7x more. The mismatch is not latency; the margin violation rate differs by 10-20 pp at 5-8 m/s (twin 11.6-21.6 %, .NET 0.9-3.6 %). Candidate cause: dynamics / sensing, not timing. |
| P3: tuned halving over seeds matches the MF selector within 10 % below 4 m/s | NOT TESTED (needs collisions below 4 m/s; none in 97 .NET flights) |
| P4: L1 failures replay as L2 failures >= 50 % below 5 m/s, < 20 % above 6 | Observed at run level (twin, same mission/seed): both collide 0/10 at 2-4 m/s, 0/10 at 5, 4/10 at 6, 1/10 at 8 (discordant 8/10 at 5 and 8). So the replay rate is ~0 below 5 m/s (no collisions to replay) and 25-40 % at 6-8: the *reverse* of P4's shape; n = 10 per speed |

## 8. Exact paired-data request for the ops agent

**What is missing:** a second config with both tiers, so rankings over configs exist. The twin currently covers only the legacy flags; .NET covers legacy (all
speeds, mostly seed 1) and, in the staged plan, recfeas at 5, 6, 8 m/s. Use NavPilot twin = L1 ("low"), .NET staged recfeas / legacy = L2 ("high"). Unit = (config, mission, speed, run seed); missions must be the 10 `cross_k` (the twin's corridor convention; corridors have no twin).
Measured costs: twin 4.5 s per run (median of the 70 stage-2 rows; 13 s for the first ones under load), .NET 48 s per run median (52 s mean; the audit's 32 s is optimistic under current load).

**Tier A (required; fits the planned reduced recfeas budget):**

| Item | Runs | Cost |
|---|---|---|
| .NET: the already planned staged recfeas arms (`recfeas_staged` and `recfeas_legacy`), speeds **5, 6, 8**, stages 1-2 (units 0-15 = 8 + 8 flights per cell) | 2 arms x 3 speeds x 16 = 96 | 96 x 48 s = **1.3 core-h** (0.85 h at the audit's 32 s) |
| NavPilot twin on the **same (cross mission, run seed) units** for both configs: legacy flags (exists at seed 1; needs seed 2 on cross_0-3) and recfeas-equivalent flags | cross units of 0-15 = 14 per cell x 3 speeds x 2 arms = 84 (of which 30 legacy exist) = **54 new** | 54 x 4.5 s = **4 min** |
| Seeds: 1 for cross_0-9, 2 for cross_0-3 (these are the planned units; no extra seeds) | | |

The twin needs the recfeas knobs (RecursiveFeasibility, BrakeAlongPath, TurnBrakeTable) mapped onto `navpilot_bridge.py`'s MAP_RHP stack, or, if NavPilot cannot express them, use
`InflateExtra` (0.25 vs 0.5; shared parameter) as the config axis instead and state it. **What Tier A buys:** 14 x 3 = 42 paired (config-difference) units; the readout is the **sign and size of the paired difference recfeas - legacy** in collision rate and min clearance at L1 vs L2, per speed. Pairwise agreement of two configs, not tau.

**Tier B (needed for a tau at all; needs orchestrator approval because it exceeds the 1 core-h reduction):** add `InflateExtra` in {0.0, 0.5} (0.25 is Tier A) for both flag sets at 6 and 8 m/s, units cross_0-9, seed 1:
.NET 2 values x 2 arms x 2 speeds x 10 = **80 runs, 64 min** (43 min at 32 s); twin 80 runs, 6 min. This gives 6 configs per speed band; the power table says the tau CI half-width is about 0.6, so
**even Tier B only supports "is tau clearly positive over all of 6-8 m/s", not a low-vs-high-band contrast.** For a band contrast, n >= 20 configs per band is needed (about 20 x 10 missions x 2 speeds = 400 .NET runs, 5 core-h): **not recommended** before Tier A shows any positive signal.

**Tier C (conditional, successive halving over seeds):** add seed 2 for a cell only where the paired L1 and L2 differences disagree in sign (expected <= 15 runs, 12 min).

**Log for the selector:** each .NET and twin run's `WallS` / `wall_s` in `costs.jsonl` (ops already does this for the twin), plus the `config` hash, so `costs.cost_per_unit` has measured medians for both tiers.

**Expected total for A + C: about 1.5 .NET core-h and 10 twin minutes (about 1.0 h at the audit's 32 s). A + B + C: about 2.5 .NET core-h.**

## 9. Reproduce, files, caveats

```
PY=~/.cache/research/fs-venv/bin/python
$PY -m unittest discover -s Tools/Frontier -p 'test_*.py'          # framework: 11 tests
$PY -m unittest discover -s Tools/Frontier/mf -p 'test_*.py'       # retrospective: 9 tests
~/.cache/research/fs-scripts/fsrun-quick $PY Tools/Frontier/mf/analyze.py            # inventory + tau (seconds)
~/.cache/research/fs-scripts/fsrun-quick $PY Tools/Frontier/mf/real_replay.py        # 7-speed real replay
~/.cache/research/fs-scripts/fsrun-quick $PY Tools/Frontier/mf/run_retro.py --reps 60 --scenarios moderate --out ~/.cache/frontier/mf/piece_moderate.json   # one scenario per call, ~1.5 min each
$PY Tools/Frontier/mf/report.py piece_*.json                                          # tables
```

Outputs live in `~/.cache/frontier/mf/` (analysis.json, piece_*.json, tables.md) and the data snapshot in `~/.cache/frontier/mf_data/`.

Caveats: (1) DEV-only CRANE rows; one forest; seed 1 for most pairs. (2) The twin's `stuck` outcomes at 2-4 m/s make "success" incomparable with .NET; use collision, clearance and progress. (3) E3 and E2 held-out rows
enter only as RETRO-HELDOUT (inventory, L1 stability, synthetic backbone, level comparison). (4) The NavPilot dev sweep's conditions (harsh sensors and latency tails) differ from the CRANE forest; its rates are context only.
(5) The synthetic AR1 had no tuning and the tuned halving baseline had a 9-point grid on the moderate scenario; a different tuning could change the 25 % margin. (6) The percentile-bootstrap CIs are anti-conservative at n = 10; quote Fieller.
