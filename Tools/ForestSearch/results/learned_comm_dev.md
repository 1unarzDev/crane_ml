# D1 learned decision-aware communication: DEV verdict and results

**DEV ONLY. Not preregistered. [workshop] paper and all existing results untouched.** Held-out seeds (3000-3015, 4000-4015, 5000-5031, 6000-6015) were never used (guarded in code and tests).

## Verdict (kill criterion from the plan and NOVELTY_D1.md)
**The kill criterion FIRES on DEV.** At matched bytes, the learned gate (logistic or MLP, trained on 121,679 paired-CRN rollout labels) is within +/-2 pp (point estimate) of HEALTH_PART at every N, and of the execution-time myopic-VoI gate (G:mv, no learning) at every N (section 3, 3b).

| N | logit - HEALTH_PART | mlp - HEALTH_PART | logit - G:mv | mlp - G:mv | gate CI inside +/-2pp (HEALTH_PART) |
|---|---|---|---|---|---|
| 2 | +0.006 [0.000,+0.014] | +0.006 [0.000,+0.014] | +0.009 [-0.016,+0.037] | +0.009 | yes / yes |
| 3 | +0.005 [0.000,+0.013] | +0.000 [0.000,0.000] | +0.014 [-0.003,+0.034] | +0.009 | yes / yes |
| 5 | +0.013 [-0.005,+0.034] | +0.003 [-0.005,+0.014] | +0.002 [-0.019,+0.020] | -0.008 | no / yes |
| 10 | -0.000 [-0.031,+0.037] | +0.002 [-0.016,+0.029] | -0.007 [-0.037,+0.013] | -0.006 | no / no |

Strict reading: CI-based equivalence is established only at N = 2, 3 (and mlp at N = 5). At N = 5 (logit) and N = 10 the CIs are too wide to exclude a gain of 3 pp (N = 10 has only 6 evaluation instances), so the data say "no detectable gain from learning", not "proven equal". No cell shows a learned gate clearly beating HEALTH_PART or G:mv. Publish as: "a hand-made beacon plus myopic VoI is near-optimal; herding needs a coordinated take-over rule (HEALTH_PART), not a learned gate".

Honest answer to "where does learning beat HEALTH_PART": **nowhere detectable.**
- The label signal is time-critical and simple: the beacon label is positive only right after death (mean +0.05 to +0.15 in the first 2 steps at N = 2, 3, decaying to about 0 later; section 7). A gate that sends once, promptly, is HEALTH. Repeats (BEACON_REP3: every 3 steps) change P(detect) by -0.003, +0.006, +0.006, -0.005 (N = 2, 3, 5, 10; all within noise) at about 5x the bytes.
- Rec and claim labels are noise-level (mean -0.003, -0.001); gates at high lambda correctly stay silent and, in healthy teams, at low lambda they lose 0.02-0.03 (stale-plan hazard).
- Where a gate "beats HEALTH" (N = 10, +0.075 [+0.020,+0.137]) it does so only because every gated team in this study inherits the HEALTH_PART receiver; that gain is the take-over rule, not the gate. The gate vs HEALTH_PART difference is 0.

## Hypotheses, as stated for D1 and in NOVELTY_D1.md predictions
- **H1 (>= 80% of ORACLE gain at <= 5% FULL bytes, N = 2..10): not supported.** Gate fraction of the ORACLE-IMPLICIT gain 0.56 (N = 2), 0.60 (N = 3), 1.30 (N = 5), -0.22 (N = 10) for the logistic gate, all with CIs spanning 0; the ORACLE-IMPLICIT ceilings are +0.056, +0.062, +0.016, -0.017 (N = 2, 3, 5, 10; only N = 3 is significant, and at N = 10 ORACLE is not above IMPLICIT, so fractions are meaningless there). HEALTH_PART reaches 0.44, 0.53, 0.50, (n/a) at the same bytes; so any "80%" is not a learned property.
- **H2 (learned take-over beats HEALTH_PART): not tested as a learned allocation** (the take-over rule was hand-made; only whether to send was learned). The gate cannot beat HEALTH_PART because it uses it.
- **P1 (N = 2 within +/-2 pp of HEALTH): holds.**
- **P2 (rollout labels beat critic labels by >= 5 pp of oracle gain): not supported at the stated size, direction consistent.** Critic (ATOC-style dQ from episode-outcome regression) minus G:mv bytes-matched: -0.025, -0.027, -0.019*, -0.008 at N = 2, 3, 5, 10; the rollout-labelled logit is +0.009, +0.014, +0.002, -0.007 against the same reference. So the critic gate is 2 to 4 pp worse at N = 2..5 (calibration: corr(pred, label) = -0.08 for the critic vs +0.21 / +0.19 for logit / MLP; the critic is confounded because episode outcome does not isolate one message). In oracle-gain units (ceilings 0.056 and 0.062) the critic-to-rollout gap is about 0.6 of the ceiling at N <= 3, but 5 pp is not reached and the paired logit-vs-critic CI was not computed. Delta (1) of NOVELTY_D1 is not dead, but weakly supported only.
- **P3 (myopic VoI reaches >= 90% of the learned gain): partly.** G:mv captures 0.39 / 0.38 / 1.20 / -0.50 of the ceiling vs the logistic gate 0.56 / 0.60 / 1.30 / -0.22; at N = 5 about 92%, at N = 2, 3 about 65-70%; absolute differences are all within +/-2 pp. G:mv uses the joint record history (more information than a real agent) and 4 paired rollouts per decision, so it is an optimistic baseline, and it costs about 8 replays per decision.
- **P4 (herding): half supported.** HEALTH orphan-first-decision index (HI) N = 10 vs N = 5: 0.055 vs 0.063 (not >= 2x), but same-step orphan collisions per episode rise 0.31 -> 1.91 (6x). The take-over rule removes 97% of them (1.91 -> 0.050); the gated team at lambda = 1e-4 gives 0.067. A claim-type message does NOT cut herding: FULL (carries claims) has the highest collisions (1.33 at N = 5, 7.25 at N = 10).
- **P5 (burst vs i.i.d. loss, matched mean 30%): NOT RUN** (no bern30_d1 cell; labels were generated on ideal and burst30_d1 only).

## Costs
- Labels: 121,679 (train 8 instances x 16 cells x 40 (N = 10: 24) behaviour episodes, twice; val 4 instances). Measured 0.021 CPU-s per label (2 paired replays, plus the shared behaviour run): N = 2 0.007, N = 3 0.011, N = 5 0.019, N = 10 0.076 s per label from the first 100-label measurement. 100-label extrapolation to 1e5 was 0.8 CPU-h; actual 2,613 CPU-s (0.73 h) including the 267,753-row critic data set.
- Model fits (logit, MLP, critic) 204 s, REINFORCE gates 187 s + 178 s (two extra runs of 200 iterations), eval 1,697 CPU-s (9 chunks), analysis/diagnostics/tests about 150 s. Total about 5,000 CPU-s = 1.4 CPU-h (budget 3). Logged in `crane-drones/Tools/Frontier/costs.jsonl` (sims learned_comm_labels, learned_comm_train).
- The episode-result cache (`Tools/Frontier/cache.py`) was not used (it appeared after the label run started); data are in `results/learned_comm_data/` (npz labels, models.pkl, eval csv.gz) instead. models.pkl must be loaded from `python -m ForestSearch.learned_comm` (classes are pickled under `__main__`).

## Design and caveats (read before citing)
- A gated team = HYBRID_P reading (records, leases, role rule) + HEALTH_PART reading of death frames. Messages: rec, claim, beacon (offered only to a dead sender; an alive agent's beacon is a no-op for every receiver, not offered). Equivalences are tested: silent gates = IMPLICIT, beacon-once = HEALTH_PART, always-send (healthy) = HYBRID_P, episode for episode on N = 2, 3, 5 (`test_learned_comm.py`).
- Labels are one-step: send or stay silent at one decision, silent afterwards, paired replays with identical random numbers; behaviour policy random sends (p = 0.15 to 0.25). dV in {-1, 0, +1}; about 3% non-zero for beacon. Myopic (Becker 2009): delayed payoffs, for example a claim that prevents a later collision, are undervalued.
- Features include the true mean link loss as "estimated loss rate" (as if from a link-quality estimator), partner silence age (rarely informative in silent-base states), own belief-nominal divergence count, steps since own death (dominant).
- G:mv: 4 posterior-sampled rollouts (SIR over truth given all records; fresh outcome uniforms and link states); sends at most one beacon, only in the first 4 steps after death; dV quantised in steps of 0.25 so the lambda grid collapses to one point.
- RL gate: per-kind Bernoulli logistic, REINFORCE with a paired silent baseline, 200 iterations x 24 episodes. It did not reach the silent baseline (last advantage -0.017 at lambda = 1e-5, -0.129 at 1e-4): under-trained, so it is a weak baseline.
- Critic arm is a regression of episode outcomes, not a full ATOC training; its dQ is used directly instead of the min-max-normalised classifier.
- Evaluation: travel 1, lambda_events 1, greedy planner, k = 1 permanent sensor loss, links ideal and burst30_d1 pooled; 20 episodes per instance; instances: N = 2, 3, 5: 16 (seeds 2100-2115); N = 10: 6 (2100-2105; the remaining chunks were cut for time because the shared quick lane queued 30+ min). N = 10 CIs are therefore wide. Lambda grid of 8 values was fixed before evaluation, but the matched-bytes interpolation on the Pareto hull is post hoc; "below-range"/"above-range" flags mean the gate had no point at HEALTH_PART's bytes (at N = 2 the logistic gate's cheapest point is 18 B vs 10.2 B, so +0.006 is at slightly more bytes than HEALTH_PART).
- Not run: EVENT and DEV_SD (not implemented for N > 2); lookahead (rollout-lite) cells; travel 0 evaluation; bern30_d1; CBBA/auction re-allocation, I2C, VBC, ETCNet baselines.

## Reproduce
`scripts/learned_comm/labels.sh`, `labels2.sh` (fsrun, ~1 h), `python -m ForestSearch.learned_comm train|rl|critic|diagnose|eval|analyze` (see `scripts/learned_comm/eval_chunks.sh`). Tests: `python3 -m pytest Tools/ForestSearch/test_learned_comm.py` (61 passed, 1 strict xfail).

---

# Detailed tables (generated by `learned_comm.py analyze` and `diagnose`)


**DEV ONLY.** Label seeds 2000-2007 (train) and 2008-2011 (validation); evaluation on fresh instances 2100-2115. No held-out seed (3000-3015, 4000-4015, 5000-5031, 6000-6015) is used. Nothing here is preregistered; the confirmatory plan is `PREREG_LEARNEDCOMM.md` (DRAFT).

## 1. Label models (validation split, instances 2008-2011)

| message | n labels | nonzero | mean dV | val MSE const | val MSE logistic | val MSE MLP |
|---|---|---|---|---|---|---|
| rec | 35530 | 715 | -0.0028 | 0.02281 | 0.02271 | 0.02274 |
| claim | 36864 | 540 | -0.0011 | 0.02381 | 0.02374 | 0.02375 |
| beacon | 41555 | 1699 | +0.0169 | 0.03432 | 0.03304 | 0.03310 |

## 2.N2. N = 2: protocols, travel 1, pooled over ideal and burst30_d1 (fresh DEV instances)

### fault = perm1

| protocol | P(detect) | bytes/ep | vs IMPLICIT (paired 95% CI) | frac of ORACLE gain |
|---|---|---|---|---|
| BEACON_ONCE | 0.425 | 10.2 | +0.025 [-0.024,+0.070] | 0.44 |
| BEACON_REP3 | 0.422 | 56.6 | +0.022 [-0.032,+0.070] | 0.39 |
| FULL | 0.412 | 1066.6 | +0.013 [-0.050,+0.073] | 0.22 |
| HB_TO | 0.438 | 145.1 | +0.037 [-0.016,+0.089] | 0.67 |
| HEALTH | 0.425 | 10.2 | +0.025 [-0.024,+0.070] | 0.44 |
| HEALTH_ONE | 0.425 | 10.2 | +0.025 [-0.024,+0.070] | 0.44 |
| HEALTH_PART | 0.425 | 10.2 | +0.025 [-0.024,+0.070] | 0.44 |
| HYBRID_P | 0.420 | 1022.7 | +0.020 [-0.024,+0.066] | 0.36 |
| IMPLICIT | 0.400 | 0.0 | +0.000 [+0.000,+0.000] | 0.00 |
| ORACLE | 0.456 | 0.0 | +0.056 [-0.005,+0.111] | 1.00 |
| G:critic:0 | 0.423 | 1180.7 | +0.023 [-0.024,+0.072] | 0.42 |
| G:critic:0.0001 | 0.423 | 1170.5 | +0.023 [-0.024,+0.072] | 0.42 |
| G:critic:0.0003 | 0.423 | 1074.7 | +0.023 [-0.022,+0.074] | 0.42 |
| G:critic:0.001 | 0.397 | 244.3 | -0.003 [-0.049,+0.044] | -0.06 |
| G:critic:1e-05 | 0.423 | 1180.7 | +0.023 [-0.024,+0.072] | 0.42 |
| G:critic:1e-06 | 0.423 | 1180.7 | +0.023 [-0.024,+0.072] | 0.42 |
| G:critic:3e-05 | 0.423 | 1180.7 | +0.023 [-0.024,+0.072] | 0.42 |
| G:critic:3e-06 | 0.423 | 1180.7 | +0.023 [-0.024,+0.072] | 0.42 |
| G:logit:0 | 0.442 | 411.3 | +0.042 [-0.009,+0.091] | 0.75 |
| G:logit:0.0001 | 0.438 | 167.4 | +0.037 [-0.011,+0.084] | 0.67 |
| G:logit:0.0003 | 0.436 | 63.3 | +0.036 [-0.012,+0.086] | 0.64 |
| G:logit:0.001 | 0.431 | 18.2 | +0.031 [-0.020,+0.080] | 0.56 |
| G:logit:1e-05 | 0.439 | 267.1 | +0.039 [-0.012,+0.087] | 0.69 |
| G:logit:1e-06 | 0.442 | 364.0 | +0.042 [-0.009,+0.091] | 0.75 |
| G:logit:3e-05 | 0.439 | 237.3 | +0.039 [-0.012,+0.087] | 0.69 |
| G:logit:3e-06 | 0.439 | 303.9 | +0.039 [-0.012,+0.087] | 0.69 |
| G:mlp:0 | 0.436 | 898.1 | +0.036 [-0.012,+0.083] | 0.64 |
| G:mlp:0.0001 | 0.442 | 726.6 | +0.042 [-0.007,+0.091] | 0.75 |
| G:mlp:0.0003 | 0.439 | 547.8 | +0.039 [-0.012,+0.084] | 0.69 |
| G:mlp:0.001 | 0.431 | 27.9 | +0.031 [-0.020,+0.080] | 0.56 |
| G:mlp:1e-05 | 0.436 | 897.0 | +0.036 [-0.012,+0.083] | 0.64 |
| G:mlp:1e-06 | 0.436 | 898.0 | +0.036 [-0.012,+0.083] | 0.64 |
| G:mlp:3e-05 | 0.438 | 886.1 | +0.037 [-0.010,+0.084] | 0.67 |
| G:mlp:3e-06 | 0.436 | 897.7 | +0.036 [-0.012,+0.083] | 0.64 |
| G:mv:0 | 0.422 | 6.7 | +0.022 [-0.020,+0.066] | 0.39 |
| G:mv:0.0001 | 0.422 | 6.7 | +0.022 [-0.020,+0.066] | 0.39 |
| G:mv:0.0003 | 0.422 | 6.7 | +0.022 [-0.020,+0.066] | 0.39 |
| G:mv:0.001 | 0.422 | 6.7 | +0.022 [-0.020,+0.066] | 0.39 |
| G:mv:1e-05 | 0.422 | 6.7 | +0.022 [-0.020,+0.066] | 0.39 |
| G:mv:1e-06 | 0.422 | 6.7 | +0.022 [-0.020,+0.066] | 0.39 |
| G:mv:3e-05 | 0.422 | 6.7 | +0.022 [-0.020,+0.066] | 0.39 |
| G:mv:3e-06 | 0.422 | 6.7 | +0.022 [-0.020,+0.066] | 0.39 |
| G:rl:0.0001 | 0.398 | 201.9 | -0.002 [-0.045,+0.041] | -0.03 |
| G:rl:1e-05 | 0.425 | 297.7 | +0.025 [-0.023,+0.077] | 0.44 |

### fault = none

| protocol | P(detect) | bytes/ep | vs IMPLICIT (paired 95% CI) | frac of ORACLE gain |
|---|---|---|---|---|
| BEACON_ONCE | 0.566 | 0.0 | +0.000 [+0.000,+0.000] | n/a |
| BEACON_REP3 | 0.566 | 0.0 | +0.000 [+0.000,+0.000] | n/a |
| FULL | 0.472 | 772.1 | -0.094 [-0.141,-0.037]* | n/a |
| HB_TO | 0.562 | 127.8 | -0.003 [-0.025,+0.016] | n/a |
| HEALTH | 0.566 | 0.0 | +0.000 [+0.000,+0.000] | n/a |
| HEALTH_ONE | 0.566 | 0.0 | +0.000 [+0.000,+0.000] | n/a |
| HEALTH_PART | 0.566 | 0.0 | +0.000 [+0.000,+0.000] | n/a |
| HYBRID_P | 0.550 | 729.1 | -0.016 [-0.041,+0.007] | n/a |
| IMPLICIT | 0.566 | 0.0 | +0.000 [+0.000,+0.000] | n/a |
| ORACLE | 0.541 | 0.0 | -0.025 [-0.056,+0.003] | n/a |
| G:critic:0 | 0.550 | 729.1 | -0.016 [-0.041,+0.007] | n/a |
| G:critic:0.0001 | 0.550 | 729.1 | -0.016 [-0.041,+0.007] | n/a |
| G:critic:0.0003 | 0.544 | 725.2 | -0.022 [-0.045,-0.001]* | n/a |
| G:critic:0.001 | 0.542 | 148.8 | -0.023 [-0.053,+0.002] | n/a |
| G:critic:1e-05 | 0.550 | 729.1 | -0.016 [-0.041,+0.007] | n/a |
| G:critic:1e-06 | 0.550 | 729.1 | -0.016 [-0.041,+0.007] | n/a |
| G:critic:3e-05 | 0.550 | 729.1 | -0.016 [-0.041,+0.007] | n/a |
| G:critic:3e-06 | 0.550 | 729.1 | -0.016 [-0.041,+0.007] | n/a |
| G:logit:0 | 0.566 | 112.1 | +0.000 [+0.000,+0.000] | n/a |
| G:logit:0.0001 | 0.566 | 0.0 | +0.000 [+0.000,+0.000] | n/a |
| G:logit:0.0003 | 0.566 | 0.0 | +0.000 [+0.000,+0.000] | n/a |
| G:logit:0.001 | 0.566 | 0.0 | +0.000 [+0.000,+0.000] | n/a |
| G:logit:1e-05 | 0.566 | 2.0 | +0.000 [+0.000,+0.000] | n/a |
| G:logit:1e-06 | 0.566 | 96.7 | +0.000 [+0.000,+0.000] | n/a |
| G:logit:3e-05 | 0.566 | 0.3 | +0.000 [+0.000,+0.000] | n/a |
| G:logit:3e-06 | 0.567 | 59.7 | +0.002 [+0.000,+0.006] | n/a |
| G:mlp:0 | 0.548 | 643.8 | -0.017 [-0.041,+0.003] | n/a |
| G:mlp:0.0001 | 0.564 | 302.6 | -0.002 [-0.013,+0.006] | n/a |
| G:mlp:0.0003 | 0.566 | 146.2 | +0.000 [-0.006,+0.006] | n/a |
| G:mlp:0.001 | 0.566 | 0.0 | +0.000 [+0.000,+0.000] | n/a |
| G:mlp:1e-05 | 0.547 | 638.9 | -0.019 [-0.042,+0.002] | n/a |
| G:mlp:1e-06 | 0.548 | 643.2 | -0.017 [-0.041,+0.003] | n/a |
| G:mlp:3e-05 | 0.552 | 594.3 | -0.014 [-0.036,+0.005] | n/a |
| G:mlp:3e-06 | 0.548 | 642.4 | -0.017 [-0.041,+0.003] | n/a |
| G:mv:0 | 0.566 | 0.0 | +0.000 [+0.000,+0.000] | n/a |
| G:mv:0.0001 | 0.566 | 0.0 | +0.000 [+0.000,+0.000] | n/a |
| G:mv:0.0003 | 0.566 | 0.0 | +0.000 [+0.000,+0.000] | n/a |
| G:mv:0.001 | 0.566 | 0.0 | +0.000 [+0.000,+0.000] | n/a |
| G:mv:1e-05 | 0.566 | 0.0 | +0.000 [+0.000,+0.000] | n/a |
| G:mv:1e-06 | 0.566 | 0.0 | +0.000 [+0.000,+0.000] | n/a |
| G:mv:3e-05 | 0.566 | 0.0 | +0.000 [+0.000,+0.000] | n/a |
| G:mv:3e-06 | 0.566 | 0.0 | +0.000 [+0.000,+0.000] | n/a |
| G:rl:0.0001 | 0.545 | 109.5 | -0.020 [-0.037,-0.004]* | n/a |
| G:rl:1e-05 | 0.562 | 82.2 | -0.003 [-0.009,+0.000] | n/a |

## 2.N3. N = 3: protocols, travel 1, pooled over ideal and burst30_d1 (fresh DEV instances)

### fault = perm1

| protocol | P(detect) | bytes/ep | vs IMPLICIT (paired 95% CI) | frac of ORACLE gain |
|---|---|---|---|---|
| BEACON_ONCE | 0.508 | 20.6 | +0.033 [-0.001,+0.068] | 0.53 |
| BEACON_REP3 | 0.514 | 105.9 | +0.039 [-0.002,+0.078] | 0.62 |
| FULL | 0.456 | 2879.8 | -0.019 [-0.069,+0.039] | -0.30 |
| HB_TO | 0.484 | 414.0 | +0.009 [-0.037,+0.062] | 0.15 |
| HEALTH | 0.498 | 20.6 | +0.023 [-0.024,+0.069] | 0.38 |
| HEALTH_ONE | 0.498 | 20.6 | +0.023 [-0.010,+0.055] | 0.38 |
| HEALTH_PART | 0.508 | 20.6 | +0.033 [-0.001,+0.068] | 0.53 |
| HYBRID_P | 0.492 | 2633.9 | +0.017 [-0.027,+0.058] | 0.28 |
| IMPLICIT | 0.475 | 0.0 | +0.000 [+0.000,+0.000] | 0.00 |
| ORACLE | 0.537 | 0.0 | +0.062 [+0.010,+0.111]* | 1.00 |
| G:critic:0 | 0.495 | 2924.3 | +0.020 [-0.023,+0.061] | 0.33 |
| G:critic:0.0001 | 0.497 | 2833.2 | +0.022 [-0.022,+0.062] | 0.35 |
| G:critic:0.0003 | 0.484 | 1335.5 | +0.009 [-0.028,+0.047] | 0.15 |
| G:critic:0.001 | 0.472 | 5.4 | -0.003 [-0.013,+0.000] | -0.05 |
| G:critic:1e-05 | 0.495 | 2924.3 | +0.020 [-0.023,+0.061] | 0.33 |
| G:critic:1e-06 | 0.495 | 2924.3 | +0.020 [-0.023,+0.061] | 0.33 |
| G:critic:3e-05 | 0.495 | 2924.3 | +0.020 [-0.023,+0.061] | 0.33 |
| G:critic:3e-06 | 0.495 | 2924.3 | +0.020 [-0.023,+0.061] | 0.33 |
| G:logit:0 | 0.516 | 1182.3 | +0.041 [-0.013,+0.094] | 0.65 |
| G:logit:0.0001 | 0.522 | 179.0 | +0.047 [+0.002,+0.097]* | 0.75 |
| G:logit:0.0003 | 0.528 | 53.8 | +0.053 [+0.013,+0.096]* | 0.85 |
| G:logit:0.001 | 0.512 | 22.3 | +0.037 [+0.004,+0.073]* | 0.60 |
| G:logit:1e-05 | 0.511 | 517.0 | +0.036 [-0.014,+0.091] | 0.57 |
| G:logit:1e-06 | 0.512 | 984.7 | +0.037 [-0.014,+0.093] | 0.60 |
| G:logit:3e-05 | 0.512 | 374.6 | +0.037 [-0.007,+0.091] | 0.60 |
| G:logit:3e-06 | 0.508 | 693.5 | +0.033 [-0.014,+0.088] | 0.53 |
| G:mlp:0 | 0.520 | 2062.4 | +0.045 [-0.002,+0.092] | 0.72 |
| G:mlp:0.0001 | 0.506 | 1226.2 | +0.031 [-0.016,+0.086] | 0.50 |
| G:mlp:0.0003 | 0.492 | 123.8 | +0.017 [-0.023,+0.056] | 0.28 |
| G:mlp:0.001 | 0.508 | 36.0 | +0.033 [-0.001,+0.068] | 0.53 |
| G:mlp:1e-05 | 0.512 | 1900.2 | +0.037 [-0.008,+0.085] | 0.60 |
| G:mlp:1e-06 | 0.522 | 2049.4 | +0.047 [+0.001,+0.094]* | 0.75 |
| G:mlp:3e-05 | 0.512 | 1603.4 | +0.037 [-0.007,+0.091] | 0.60 |
| G:mlp:3e-06 | 0.519 | 2020.4 | +0.044 [-0.002,+0.091] | 0.70 |
| G:mv:0 | 0.498 | 11.8 | +0.023 [-0.005,+0.053] | 0.38 |
| G:mv:0.0001 | 0.498 | 11.8 | +0.023 [-0.005,+0.053] | 0.38 |
| G:mv:0.0003 | 0.498 | 11.8 | +0.023 [-0.005,+0.053] | 0.38 |
| G:mv:0.001 | 0.498 | 11.8 | +0.023 [-0.005,+0.053] | 0.38 |
| G:mv:1e-05 | 0.498 | 11.8 | +0.023 [-0.005,+0.053] | 0.38 |
| G:mv:1e-06 | 0.498 | 11.8 | +0.023 [-0.005,+0.053] | 0.38 |
| G:mv:3e-05 | 0.498 | 11.8 | +0.023 [-0.005,+0.053] | 0.38 |
| G:mv:3e-06 | 0.498 | 11.8 | +0.023 [-0.005,+0.053] | 0.38 |
| G:rl:0.0001 | 0.492 | 345.0 | +0.017 [-0.022,+0.061] | 0.28 |
| G:rl:1e-05 | 0.481 | 546.9 | +0.006 [-0.033,+0.045] | 0.10 |

### fault = none

| protocol | P(detect) | bytes/ep | vs IMPLICIT (paired 95% CI) | frac of ORACLE gain |
|---|---|---|---|---|
| BEACON_ONCE | 0.600 | 0.0 | +0.000 [+0.000,+0.000] | n/a |
| BEACON_REP3 | 0.600 | 0.0 | +0.000 [+0.000,+0.000] | n/a |
| FULL | 0.481 | 2228.8 | -0.119 [-0.163,-0.073]* | n/a |
| HB_TO | 0.588 | 368.9 | -0.013 [-0.030,+0.006] | n/a |
| HEALTH | 0.600 | 0.0 | +0.000 [+0.000,+0.000] | n/a |
| HEALTH_ONE | 0.600 | 0.0 | +0.000 [+0.000,+0.000] | n/a |
| HEALTH_PART | 0.600 | 0.0 | +0.000 [+0.000,+0.000] | n/a |
| HYBRID_P | 0.591 | 2037.6 | -0.009 [-0.042,+0.020] | n/a |
| IMPLICIT | 0.600 | 0.0 | +0.000 [+0.000,+0.000] | n/a |
| ORACLE | 0.591 | 0.0 | -0.009 [-0.042,+0.020] | n/a |
| G:critic:0 | 0.591 | 2037.6 | -0.009 [-0.042,+0.020] | n/a |
| G:critic:0.0001 | 0.591 | 2037.4 | -0.009 [-0.042,+0.020] | n/a |
| G:critic:0.0003 | 0.584 | 719.5 | -0.016 [-0.048,+0.009] | n/a |
| G:critic:0.001 | 0.600 | 0.0 | +0.000 [+0.000,+0.000] | n/a |
| G:critic:1e-05 | 0.591 | 2037.6 | -0.009 [-0.042,+0.020] | n/a |
| G:critic:1e-06 | 0.591 | 2037.6 | -0.009 [-0.042,+0.020] | n/a |
| G:critic:3e-05 | 0.591 | 2037.6 | -0.009 [-0.042,+0.020] | n/a |
| G:critic:3e-06 | 0.591 | 2037.6 | -0.009 [-0.042,+0.020] | n/a |
| G:logit:0 | 0.594 | 483.8 | -0.006 [-0.022,+0.008] | n/a |
| G:logit:0.0001 | 0.600 | 0.0 | +0.000 [+0.000,+0.000] | n/a |
| G:logit:0.0003 | 0.600 | 0.0 | +0.000 [+0.000,+0.000] | n/a |
| G:logit:0.001 | 0.600 | 0.0 | +0.000 [+0.000,+0.000] | n/a |
| G:logit:1e-05 | 0.600 | 14.5 | +0.000 [+0.000,+0.000] | n/a |
| G:logit:1e-06 | 0.592 | 420.7 | -0.008 [-0.020,+0.003] | n/a |
| G:logit:3e-05 | 0.600 | 0.4 | +0.000 [+0.000,+0.000] | n/a |
| G:logit:3e-06 | 0.592 | 195.7 | -0.008 [-0.017,-0.002]* | n/a |
| G:mlp:0 | 0.583 | 1470.3 | -0.017 [-0.039,+0.003] | n/a |
| G:mlp:0.0001 | 0.598 | 479.6 | -0.002 [-0.006,+0.000] | n/a |
| G:mlp:0.0003 | 0.600 | 0.0 | +0.000 [+0.000,+0.000] | n/a |
| G:mlp:0.001 | 0.600 | 0.0 | +0.000 [+0.000,+0.000] | n/a |
| G:mlp:1e-05 | 0.588 | 1149.8 | -0.013 [-0.031,+0.005] | n/a |
| G:mlp:1e-06 | 0.586 | 1435.8 | -0.014 [-0.035,+0.005] | n/a |
| G:mlp:3e-05 | 0.597 | 903.3 | -0.003 [-0.023,+0.014] | n/a |
| G:mlp:3e-06 | 0.586 | 1383.3 | -0.014 [-0.033,+0.003] | n/a |
| G:mv:0 | 0.600 | 0.0 | +0.000 [+0.000,+0.000] | n/a |
| G:mv:0.0001 | 0.600 | 0.0 | +0.000 [+0.000,+0.000] | n/a |
| G:mv:0.0003 | 0.600 | 0.0 | +0.000 [+0.000,+0.000] | n/a |
| G:mv:0.001 | 0.600 | 0.0 | +0.000 [+0.000,+0.000] | n/a |
| G:mv:1e-05 | 0.600 | 0.0 | +0.000 [+0.000,+0.000] | n/a |
| G:mv:1e-06 | 0.600 | 0.0 | +0.000 [+0.000,+0.000] | n/a |
| G:mv:3e-05 | 0.600 | 0.0 | +0.000 [+0.000,+0.000] | n/a |
| G:mv:3e-06 | 0.600 | 0.0 | +0.000 [+0.000,+0.000] | n/a |
| G:rl:0.0001 | 0.577 | 177.0 | -0.023 [-0.044,-0.005]* | n/a |
| G:rl:1e-05 | 0.592 | 227.1 | -0.008 [-0.020,+0.005] | n/a |

## 2.N5. N = 5: protocols, travel 1, pooled over ideal and burst30_d1 (fresh DEV instances)

### fault = perm1

| protocol | P(detect) | bytes/ep | vs IMPLICIT (paired 95% CI) | frac of ORACLE gain |
|---|---|---|---|---|
| BEACON_ONCE | 0.564 | 40.8 | +0.008 [-0.024,+0.039] | 0.50 |
| BEACON_REP3 | 0.570 | 203.6 | +0.014 [-0.021,+0.048] | 0.90 |
| FULL | 0.359 | 9026.8 | -0.197 [-0.263,-0.129]* | -12.60 |
| HB_TO | 0.528 | 1339.9 | -0.028 [-0.083,+0.026] | -1.80 |
| HEALTH | 0.558 | 40.8 | +0.002 [-0.045,+0.045] | 0.10 |
| HEALTH_ONE | 0.569 | 40.8 | +0.013 [-0.016,+0.041] | 0.80 |
| HEALTH_PART | 0.564 | 40.8 | +0.008 [-0.024,+0.039] | 0.50 |
| HYBRID_P | 0.562 | 7547.5 | +0.006 [-0.047,+0.055] | 0.40 |
| IMPLICIT | 0.556 | 0.0 | +0.000 [+0.000,+0.000] | 0.00 |
| ORACLE | 0.572 | 0.0 | +0.016 [-0.039,+0.068] | 1.00 |
| G:critic:0 | 0.562 | 8123.4 | +0.006 [-0.048,+0.058] | 0.40 |
| G:critic:0.0001 | 0.562 | 7522.3 | +0.006 [-0.049,+0.058] | 0.40 |
| G:critic:0.0003 | 0.541 | 1772.1 | -0.016 [-0.072,+0.032] | -1.00 |
| G:critic:0.001 | 0.556 | 0.0 | +0.000 [+0.000,+0.000] | 0.00 |
| G:critic:1e-05 | 0.562 | 8123.4 | +0.006 [-0.048,+0.058] | 0.40 |
| G:critic:1e-06 | 0.562 | 8123.4 | +0.006 [-0.048,+0.058] | 0.40 |
| G:critic:3e-05 | 0.562 | 8082.9 | +0.006 [-0.048,+0.058] | 0.40 |
| G:critic:3e-06 | 0.562 | 8123.4 | +0.006 [-0.048,+0.058] | 0.40 |
| G:logit:0 | 0.583 | 3440.8 | +0.027 [-0.018,+0.070] | 1.70 |
| G:logit:0.0001 | 0.558 | 111.9 | +0.002 [-0.034,+0.033] | 0.10 |
| G:logit:0.0003 | 0.566 | 49.5 | +0.009 [-0.027,+0.039] | 0.60 |
| G:logit:0.001 | 0.577 | 12.8 | +0.020 [+0.000,+0.041] | 1.30 |
| G:logit:1e-05 | 0.564 | 1424.5 | +0.008 [-0.041,+0.054] | 0.50 |
| G:logit:1e-06 | 0.584 | 2952.1 | +0.028 [-0.019,+0.073] | 1.80 |
| G:logit:3e-05 | 0.572 | 427.0 | +0.016 [-0.027,+0.055] | 1.00 |
| G:logit:3e-06 | 0.580 | 1990.8 | +0.023 [-0.022,+0.067] | 1.50 |
| G:mlp:0 | 0.541 | 4634.6 | -0.016 [-0.068,+0.029] | -1.00 |
| G:mlp:0.0001 | 0.553 | 867.8 | -0.003 [-0.045,+0.036] | -0.20 |
| G:mlp:0.0003 | 0.564 | 67.5 | +0.008 [-0.024,+0.039] | 0.50 |
| G:mlp:0.001 | 0.567 | 38.0 | +0.011 [-0.019,+0.037] | 0.70 |
| G:mlp:1e-05 | 0.552 | 3169.6 | -0.005 [-0.055,+0.040] | -0.30 |
| G:mlp:1e-06 | 0.544 | 4541.4 | -0.013 [-0.064,+0.034] | -0.80 |
| G:mlp:3e-05 | 0.550 | 2486.9 | -0.006 [-0.055,+0.038] | -0.40 |
| G:mlp:3e-06 | 0.550 | 4184.7 | -0.006 [-0.058,+0.038] | -0.40 |
| G:mv:0 | 0.575 | 15.2 | +0.019 [+0.000,+0.038] | 1.20 |
| G:mv:0.0001 | 0.575 | 15.2 | +0.019 [+0.000,+0.038] | 1.20 |
| G:mv:0.0003 | 0.575 | 15.2 | +0.019 [+0.000,+0.038] | 1.20 |
| G:mv:0.001 | 0.575 | 15.2 | +0.019 [+0.000,+0.038] | 1.20 |
| G:mv:1e-05 | 0.575 | 15.2 | +0.019 [+0.000,+0.038] | 1.20 |
| G:mv:1e-06 | 0.575 | 15.2 | +0.019 [+0.000,+0.038] | 1.20 |
| G:mv:3e-05 | 0.575 | 15.2 | +0.019 [+0.000,+0.038] | 1.20 |
| G:mv:3e-06 | 0.575 | 15.2 | +0.019 [+0.000,+0.038] | 1.20 |
| G:rl:0.0001 | 0.562 | 544.2 | +0.006 [-0.029,+0.041] | 0.40 |
| G:rl:1e-05 | 0.575 | 883.0 | +0.019 [-0.019,+0.053] | 1.20 |

### fault = none

| protocol | P(detect) | bytes/ep | vs IMPLICIT (paired 95% CI) | frac of ORACLE gain |
|---|---|---|---|---|
| BEACON_ONCE | 0.631 | 0.0 | +0.000 [+0.000,+0.000] | n/a |
| BEACON_REP3 | 0.631 | 0.0 | +0.000 [+0.000,+0.000] | n/a |
| FULL | 0.366 | 7856.4 | -0.266 [-0.334,-0.198]* | n/a |
| HB_TO | 0.583 | 1257.4 | -0.048 [-0.077,-0.020]* | n/a |
| HEALTH | 0.631 | 0.0 | +0.000 [+0.000,+0.000] | n/a |
| HEALTH_ONE | 0.631 | 0.0 | +0.000 [+0.000,+0.000] | n/a |
| HEALTH_PART | 0.631 | 0.0 | +0.000 [+0.000,+0.000] | n/a |
| HYBRID_P | 0.600 | 6635.9 | -0.031 [-0.066,+0.000] | n/a |
| IMPLICIT | 0.631 | 0.0 | +0.000 [+0.000,+0.000] | n/a |
| ORACLE | 0.606 | 0.0 | -0.025 [-0.062,+0.008] | n/a |
| G:critic:0 | 0.600 | 6635.9 | -0.031 [-0.066,+0.000] | n/a |
| G:critic:0.0001 | 0.600 | 6418.9 | -0.031 [-0.066,+0.000] | n/a |
| G:critic:0.0003 | 0.575 | 1598.0 | -0.056 [-0.089,-0.030]* | n/a |
| G:critic:0.001 | 0.631 | 0.0 | +0.000 [+0.000,+0.000] | n/a |
| G:critic:1e-05 | 0.600 | 6635.9 | -0.031 [-0.066,+0.000] | n/a |
| G:critic:1e-06 | 0.600 | 6635.9 | -0.031 [-0.066,+0.000] | n/a |
| G:critic:3e-05 | 0.600 | 6635.9 | -0.031 [-0.066,+0.000] | n/a |
| G:critic:3e-06 | 0.600 | 6635.9 | -0.031 [-0.066,+0.000] | n/a |
| G:logit:0 | 0.633 | 1994.0 | +0.002 [-0.027,+0.027] | n/a |
| G:logit:0.0001 | 0.631 | 0.0 | +0.000 [+0.000,+0.000] | n/a |
| G:logit:0.0003 | 0.631 | 0.0 | +0.000 [+0.000,+0.000] | n/a |
| G:logit:0.001 | 0.631 | 0.0 | +0.000 [+0.000,+0.000] | n/a |
| G:logit:1e-05 | 0.636 | 167.2 | +0.005 [+0.000,+0.016] | n/a |
| G:logit:1e-06 | 0.636 | 1606.1 | +0.005 [-0.017,+0.027] | n/a |
| G:logit:3e-05 | 0.631 | 8.9 | +0.000 [+0.000,+0.000] | n/a |
| G:logit:3e-06 | 0.641 | 610.2 | +0.009 [-0.006,+0.029] | n/a |
| G:mlp:0 | 0.609 | 3623.8 | -0.022 [-0.042,-0.003]* | n/a |
| G:mlp:0.0001 | 0.631 | 0.0 | +0.000 [+0.000,+0.000] | n/a |
| G:mlp:0.0003 | 0.631 | 0.0 | +0.000 [+0.000,+0.000] | n/a |
| G:mlp:0.001 | 0.631 | 0.0 | +0.000 [+0.000,+0.000] | n/a |
| G:mlp:1e-05 | 0.636 | 1908.0 | +0.005 [+0.000,+0.016] | n/a |
| G:mlp:1e-06 | 0.611 | 3526.6 | -0.020 [-0.039,-0.002]* | n/a |
| G:mlp:3e-05 | 0.631 | 951.2 | +0.000 [+0.000,+0.000] | n/a |
| G:mlp:3e-06 | 0.620 | 2987.8 | -0.011 [-0.031,+0.006] | n/a |
| G:mv:0 | 0.631 | 0.0 | +0.000 [+0.000,+0.000] | n/a |
| G:mv:0.0001 | 0.631 | 0.0 | +0.000 [+0.000,+0.000] | n/a |
| G:mv:0.0003 | 0.631 | 0.0 | +0.000 [+0.000,+0.000] | n/a |
| G:mv:0.001 | 0.631 | 0.0 | +0.000 [+0.000,+0.000] | n/a |
| G:mv:1e-05 | 0.631 | 0.0 | +0.000 [+0.000,+0.000] | n/a |
| G:mv:1e-06 | 0.631 | 0.0 | +0.000 [+0.000,+0.000] | n/a |
| G:mv:3e-05 | 0.631 | 0.0 | +0.000 [+0.000,+0.000] | n/a |
| G:mv:3e-06 | 0.631 | 0.0 | +0.000 [+0.000,+0.000] | n/a |
| G:rl:0.0001 | 0.611 | 328.4 | -0.020 [-0.041,-0.002]* | n/a |
| G:rl:1e-05 | 0.631 | 364.1 | +0.000 [+0.000,+0.000] | n/a |

## 2.N10. N = 10: protocols, travel 1, pooled over ideal and burst30_d1 (fresh DEV instances)

### fault = perm1

| protocol | P(detect) | bytes/ep | vs IMPLICIT (paired 95% CI) | frac of ORACLE gain |
|---|---|---|---|---|
| BEACON_ONCE | 0.688 | 86.4 | +0.004 [-0.052,+0.058] | -0.25 |
| BEACON_REP3 | 0.683 | 515.7 | +0.000 [-0.058,+0.054] | -0.00 |
| FULL | 0.425 | 52761.9 | -0.258 [-0.358,-0.163]* | 15.50 |
| HB_TO | 0.583 | 7128.9 | -0.100 [-0.158,-0.046]* | 6.00 |
| HEALTH | 0.613 | 86.4 | -0.071 [-0.146,-0.004]* | 4.25 |
| HEALTH_ONE | 0.675 | 86.4 | -0.008 [-0.033,+0.000] | 0.50 |
| HEALTH_PART | 0.688 | 86.4 | +0.004 [-0.052,+0.058] | -0.25 |
| HYBRID_P | 0.662 | 41623.7 | -0.021 [-0.098,+0.042] | 1.25 |
| IMPLICIT | 0.683 | 0.0 | +0.000 [+0.000,+0.000] | -0.00 |
| ORACLE | 0.667 | 0.0 | -0.017 [-0.092,+0.048] | 1.00 |
| G:critic:0 | 0.662 | 43149.6 | -0.021 [-0.098,+0.042] | 1.25 |
| G:critic:0.0001 | 0.654 | 19734.3 | -0.029 [-0.102,+0.033] | 1.75 |
| G:critic:0.0003 | 0.683 | 0.0 | +0.000 [+0.000,+0.000] | -0.00 |
| G:critic:0.001 | 0.683 | 0.0 | +0.000 [+0.000,+0.000] | -0.00 |
| G:critic:1e-05 | 0.662 | 43094.7 | -0.021 [-0.098,+0.042] | 1.25 |
| G:critic:1e-06 | 0.662 | 43149.6 | -0.021 [-0.098,+0.042] | 1.25 |
| G:critic:3e-05 | 0.667 | 42357.8 | -0.017 [-0.098,+0.050] | 1.00 |
| G:critic:3e-06 | 0.662 | 43149.6 | -0.021 [-0.098,+0.042] | 1.25 |
| G:logit:0 | 0.675 | 20418.6 | -0.008 [-0.071,+0.054] | 0.50 |
| G:logit:0.0001 | 0.688 | 98.1 | +0.004 [-0.037,+0.042] | -0.25 |
| G:logit:0.0003 | 0.675 | 12.2 | -0.008 [-0.033,+0.000] | 0.50 |
| G:logit:0.001 | 0.683 | 0.0 | +0.000 [+0.000,+0.000] | -0.00 |
| G:logit:1e-05 | 0.646 | 4911.3 | -0.037 [-0.087,+0.013] | 2.25 |
| G:logit:1e-06 | 0.667 | 12650.9 | -0.017 [-0.067,+0.029] | 1.00 |
| G:logit:3e-05 | 0.662 | 557.9 | -0.021 [-0.075,+0.025] | 1.25 |
| G:logit:3e-06 | 0.654 | 8811.6 | -0.029 [-0.079,+0.025] | 1.75 |
| G:mlp:0 | 0.604 | 9217.6 | -0.079 [-0.146,-0.021]* | 4.75 |
| G:mlp:0.0001 | 0.692 | 119.7 | +0.008 [-0.042,+0.058] | -0.50 |
| G:mlp:0.0003 | 0.683 | 52.6 | +0.000 [-0.037,+0.033] | -0.00 |
| G:mlp:0.001 | 0.683 | 0.0 | +0.000 [+0.000,+0.000] | -0.00 |
| G:mlp:1e-05 | 0.608 | 8806.4 | -0.075 [-0.144,-0.017]* | 4.50 |
| G:mlp:1e-06 | 0.604 | 9151.6 | -0.079 [-0.146,-0.021]* | 4.75 |
| G:mlp:3e-05 | 0.629 | 7238.9 | -0.054 [-0.108,+0.000] | 3.25 |
| G:mlp:3e-06 | 0.604 | 9095.7 | -0.079 [-0.146,-0.021]* | 4.75 |
| G:mv:0 | 0.692 | 28.8 | +0.008 [-0.017,+0.046] | -0.50 |
| G:mv:0.0001 | 0.692 | 28.8 | +0.008 [-0.017,+0.046] | -0.50 |
| G:mv:0.0003 | 0.692 | 28.8 | +0.008 [-0.017,+0.046] | -0.50 |
| G:mv:0.001 | 0.692 | 28.8 | +0.008 [-0.017,+0.046] | -0.50 |
| G:mv:1e-05 | 0.692 | 28.8 | +0.008 [-0.017,+0.046] | -0.50 |
| G:mv:1e-06 | 0.692 | 28.8 | +0.008 [-0.017,+0.046] | -0.50 |
| G:mv:3e-05 | 0.692 | 28.8 | +0.008 [-0.017,+0.046] | -0.50 |
| G:mv:3e-06 | 0.692 | 28.8 | +0.008 [-0.017,+0.046] | -0.50 |
| G:rl:0.0001 | 0.646 | 1566.8 | -0.037 [-0.098,+0.021] | 2.25 |
| G:rl:1e-05 | 0.679 | 1656.0 | -0.004 [-0.033,+0.017] | 0.25 |

### fault = none

| protocol | P(detect) | bytes/ep | vs IMPLICIT (paired 95% CI) | frac of ORACLE gain |
|---|---|---|---|---|
| BEACON_ONCE | 0.717 | 0.0 | +0.000 [+0.000,+0.000] | n/a |
| BEACON_REP3 | 0.717 | 0.0 | +0.000 [+0.000,+0.000] | n/a |
| FULL | 0.442 | 49131.6 | -0.275 [-0.356,-0.194]* | n/a |
| HB_TO | 0.667 | 6822.0 | -0.050 [-0.098,+0.004] | n/a |
| HEALTH | 0.717 | 0.0 | +0.000 [+0.000,+0.000] | n/a |
| HEALTH_ONE | 0.717 | 0.0 | +0.000 [+0.000,+0.000] | n/a |
| HEALTH_PART | 0.717 | 0.0 | +0.000 [+0.000,+0.000] | n/a |
| HYBRID_P | 0.679 | 38975.1 | -0.037 [-0.100,+0.013] | n/a |
| IMPLICIT | 0.717 | 0.0 | +0.000 [+0.000,+0.000] | n/a |
| ORACLE | 0.667 | 0.0 | -0.050 [-0.113,+0.002] | n/a |
| G:critic:0 | 0.679 | 38975.1 | -0.037 [-0.100,+0.013] | n/a |
| G:critic:0.0001 | 0.688 | 18743.2 | -0.029 [-0.087,+0.025] | n/a |
| G:critic:0.0003 | 0.717 | 0.0 | +0.000 [+0.000,+0.000] | n/a |
| G:critic:0.001 | 0.717 | 0.0 | +0.000 [+0.000,+0.000] | n/a |
| G:critic:1e-05 | 0.679 | 38975.1 | -0.037 [-0.100,+0.013] | n/a |
| G:critic:1e-06 | 0.679 | 38975.1 | -0.037 [-0.100,+0.013] | n/a |
| G:critic:3e-05 | 0.679 | 38857.1 | -0.037 [-0.100,+0.013] | n/a |
| G:critic:3e-06 | 0.679 | 38975.1 | -0.037 [-0.100,+0.013] | n/a |
| G:logit:0 | 0.688 | 16116.8 | -0.029 [-0.077,+0.019] | n/a |
| G:logit:0.0001 | 0.717 | 0.0 | +0.000 [+0.000,+0.000] | n/a |
| G:logit:0.0003 | 0.717 | 0.0 | +0.000 [+0.000,+0.000] | n/a |
| G:logit:0.001 | 0.717 | 0.0 | +0.000 [+0.000,+0.000] | n/a |
| G:logit:1e-05 | 0.717 | 1196.2 | +0.000 [-0.021,+0.021] | n/a |
| G:logit:1e-06 | 0.688 | 9515.2 | -0.029 [-0.081,+0.017] | n/a |
| G:logit:3e-05 | 0.717 | 0.0 | +0.000 [+0.000,+0.000] | n/a |
| G:logit:3e-06 | 0.713 | 3205.5 | -0.004 [-0.029,+0.017] | n/a |
| G:mlp:0 | 0.717 | 4955.4 | +0.000 [+0.000,+0.000] | n/a |
| G:mlp:0.0001 | 0.717 | 0.0 | +0.000 [+0.000,+0.000] | n/a |
| G:mlp:0.0003 | 0.717 | 0.0 | +0.000 [+0.000,+0.000] | n/a |
| G:mlp:0.001 | 0.717 | 0.0 | +0.000 [+0.000,+0.000] | n/a |
| G:mlp:1e-05 | 0.717 | 4728.0 | +0.000 [+0.000,+0.000] | n/a |
| G:mlp:1e-06 | 0.717 | 4869.0 | +0.000 [+0.000,+0.000] | n/a |
| G:mlp:3e-05 | 0.717 | 4546.5 | +0.000 [+0.000,+0.000] | n/a |
| G:mlp:3e-06 | 0.717 | 4833.0 | +0.000 [+0.000,+0.000] | n/a |
| G:mv:0 | 0.717 | 0.0 | +0.000 [+0.000,+0.000] | n/a |
| G:mv:0.0001 | 0.717 | 0.0 | +0.000 [+0.000,+0.000] | n/a |
| G:mv:0.0003 | 0.717 | 0.0 | +0.000 [+0.000,+0.000] | n/a |
| G:mv:0.001 | 0.717 | 0.0 | +0.000 [+0.000,+0.000] | n/a |
| G:mv:1e-05 | 0.717 | 0.0 | +0.000 [+0.000,+0.000] | n/a |
| G:mv:1e-06 | 0.717 | 0.0 | +0.000 [+0.000,+0.000] | n/a |
| G:mv:3e-05 | 0.717 | 0.0 | +0.000 [+0.000,+0.000] | n/a |
| G:mv:3e-06 | 0.717 | 0.0 | +0.000 [+0.000,+0.000] | n/a |
| G:rl:0.0001 | 0.675 | 944.2 | -0.042 [-0.096,+0.004] | n/a |
| G:rl:1e-05 | 0.717 | 900.0 | +0.000 [+0.000,+0.000] | n/a |

## 3. Matched-bytes comparison against HEALTH_PART / HEALTH (perm1, travel 1; the kill check)

Gate curve (lambda sweep) interpolated on its upper-left Pareto hull at HEALTH_PART's mean bytes. Difference = P(gate at matched bytes) - P(reference), paired cluster bootstrap (instances then episodes), 95% CI. `kill` = 95% CI inside +/-0.02.

| N | family | ref | ref bytes | diff vs ref | range flag | within +/-2pp (CI) | point within | frac ORACLE gain (gate / ref) |
|---|---|---|---|---|---|---|---|---|
| 2 | logit | HEALTH_PART | 10.2 | +0.006 [+0.000,+0.014] | below-range | yes | yes | 0.56 / 0.44 |
| 2 | logit | HEALTH | 10.2 | +0.006 [+0.000,+0.014] | below-range | yes | yes | 0.56 / 0.44 |
| 2 | mlp | HEALTH_PART | 10.2 | +0.006 [+0.000,+0.014] | below-range | yes | yes | 0.56 / 0.44 |
| 2 | mlp | HEALTH | 10.2 | +0.006 [+0.000,+0.014] | below-range | yes | yes | 0.56 / 0.44 |
| 2 | rl | HEALTH_PART | 10.2 | -0.027 [-0.059,+0.006] | below-range | no | no | -0.03 / 0.44 |
| 2 | rl | HEALTH | 10.2 | -0.027 [-0.059,+0.006] | below-range | no | no | -0.03 / 0.44 |
| 2 | critic | HEALTH_PART | 10.2 | -0.028 [-0.059,+0.006] | below-range | no | no | -0.06 / 0.44 |
| 2 | critic | HEALTH | 10.2 | -0.028 [-0.059,+0.006] | below-range | no | no | -0.06 / 0.44 |
| 2 | mv | HEALTH_PART | 10.2 | -0.003 [-0.030,+0.020] | above-range | no | yes | 0.39 / 0.44 |
| 2 | mv | HEALTH | 10.2 | -0.003 [-0.030,+0.020] | above-range | no | yes | 0.39 / 0.44 |
| 3 | logit | HEALTH_PART | 20.6 | +0.005 [+0.000,+0.013] | below-range | yes | yes | 0.60 / 0.53 |
| 3 | logit | HEALTH | 20.6 | +0.014 [-0.016,+0.047] | below-range | no | yes | 0.60 / 0.38 |
| 3 | mlp | HEALTH_PART | 20.6 | +0.000 [+0.000,+0.000] | below-range | yes | yes | 0.53 / 0.53 |
| 3 | mlp | HEALTH | 20.6 | +0.009 [-0.020,+0.039] | below-range | no | yes | 0.53 / 0.38 |
| 3 | rl | HEALTH_PART | 20.6 | -0.016 [-0.052,+0.016] | below-range | no | yes | 0.28 / 0.53 |
| 3 | rl | HEALTH | 20.6 | -0.006 [-0.041,+0.028] | below-range | no | yes | 0.28 / 0.38 |
| 3 | critic | HEALTH_PART | 20.6 | -0.036 [-0.069,-0.005]* |  | no | no | -0.05 / 0.53 |
| 3 | critic | HEALTH | 20.6 | -0.026 [-0.071,+0.020] |  | no | no | -0.05 / 0.38 |
| 3 | mv | HEALTH_PART | 20.6 | -0.009 [-0.028,+0.008] | above-range | no | yes | 0.38 / 0.53 |
| 3 | mv | HEALTH | 20.6 | +0.000 [-0.034,+0.033] | above-range | no | yes | 0.38 / 0.38 |
| 5 | logit | HEALTH_PART | 40.8 | +0.013 [-0.005,+0.034] |  | no | yes | 1.30 / 0.50 |
| 5 | logit | HEALTH | 40.8 | +0.019 [-0.014,+0.056] |  | no | yes | 1.30 / 0.10 |
| 5 | mlp | HEALTH_PART | 40.8 | +0.003 [-0.005,+0.014] | above-range | yes | yes | 0.70 / 0.50 |
| 5 | mlp | HEALTH | 40.8 | +0.009 [-0.022,+0.041] | above-range | no | yes | 0.70 / 0.10 |
| 5 | rl | HEALTH_PART | 40.8 | -0.002 [-0.027,+0.026] | below-range | no | yes | 0.40 / 0.50 |
| 5 | rl | HEALTH | 40.8 | +0.005 [-0.027,+0.036] | below-range | no | yes | 0.40 / 0.10 |
| 5 | critic | HEALTH_PART | 40.8 | -0.008 [-0.039,+0.024] |  | no | yes | 0.00 / 0.50 |
| 5 | critic | HEALTH | 40.8 | -0.002 [-0.044,+0.045] |  | no | yes | 0.00 / 0.10 |
| 5 | mv | HEALTH_PART | 40.8 | +0.011 [-0.011,+0.037] | above-range | no | yes | 1.20 / 0.50 |
| 5 | mv | HEALTH | 40.8 | +0.017 [-0.023,+0.061] | above-range | no | yes | 1.20 / 0.10 |
| 10 | logit | HEALTH_PART | 86.4 | -0.000 [-0.031,+0.037] |  | no | yes | -0.22 / -0.25 |
| 10 | logit | HEALTH | 86.4 | +0.075 [+0.020,+0.137]* |  | no | no | -0.22 / 4.25 |
| 10 | mlp | HEALTH_PART | 86.4 | +0.002 [-0.016,+0.029] |  | no | yes | -0.36 / -0.25 |
| 10 | mlp | HEALTH | 86.4 | +0.077 [+0.025,+0.132]* |  | no | no | -0.36 / 4.25 |
| 10 | rl | HEALTH_PART | 86.4 | -0.042 [-0.113,+0.021] | below-range | no | no | 2.25 / -0.25 |
| 10 | rl | HEALTH | 86.4 | +0.033 [-0.033,+0.104] | below-range | no | no | 2.25 / 4.25 |
| 10 | critic | HEALTH_PART | 86.4 | -0.004 [-0.058,+0.052] | above-range | no | yes | -0.00 / -0.25 |
| 10 | critic | HEALTH | 86.4 | +0.071 [+0.004,+0.146]* | above-range | no | no | -0.00 / 4.25 |
| 10 | mv | HEALTH_PART | 86.4 | +0.004 [-0.029,+0.050] | above-range | no | yes | -0.50 / -0.25 |
| 10 | mv | HEALTH | 86.4 | +0.079 [+0.021,+0.150]* | above-range | no | no | -0.50 / 4.25 |

## 3b. Kill check against the execution-time myopic-VoI gate (G:mv) and G3 label source (perm1, travel 1)

G:mv has no tunable price in the useful range (its rollout value is quantised in steps of 1/R = 0.25, so every lambda in the grid gives the same decision); it is a single point. Difference = P(family at G:mv bytes, interpolated) - P(G:mv); critic vs rollout: P(logit at critic-matched bytes) - P(critic).

| N | family | ref | ref bytes | diff | range flag | CI inside +/-2pp |
|---|---|---|---|---|---|---|
| 2 | logit | G:mv | 6.7 | +0.009 [-0.016,+0.037] | below-range | no |
| 2 | mlp | G:mv | 6.7 | +0.009 [-0.016,+0.037] | below-range | no |
| 2 | rl | G:mv | 6.7 | -0.023 [-0.061,+0.014] | below-range | no |
| 2 | critic | G:mv | 6.7 | -0.025 [-0.059,+0.015] | below-range | no |
| 3 | logit | G:mv | 11.8 | +0.014 [-0.003,+0.034] | below-range | no |
| 3 | mlp | G:mv | 11.8 | +0.009 [-0.008,+0.028] | below-range | no |
| 3 | rl | G:mv | 11.8 | -0.006 [-0.047,+0.031] | below-range | no |
| 3 | critic | G:mv | 11.8 | -0.027 [-0.056,+0.001] |  | no |
| 5 | logit | G:mv | 15.2 | +0.002 [-0.019,+0.020] |  | no |
| 5 | mlp | G:mv | 15.2 | -0.008 [-0.032,+0.013] | below-range | no |
| 5 | rl | G:mv | 15.2 | -0.013 [-0.045,+0.019] | below-range | no |
| 5 | critic | G:mv | 15.2 | -0.019 [-0.038,-0.000]* |  | no |
| 10 | logit | G:mv | 28.8 | -0.007 [-0.037,+0.013] |  | no |
| 10 | mlp | G:mv | 28.8 | -0.006 [-0.034,+0.013] |  | no |
| 10 | rl | G:mv | 28.8 | -0.046 [-0.115,+0.021] | below-range | no |
| 10 | critic | G:mv | 28.8 | -0.008 [-0.046,+0.017] | above-range | no |

## 4. Pareto fronts (perm1, travel 1; per-N, pooled over links)

### N = 2

| family | lambda | bytes/ep | P(detect) | vs IMPLICIT |
|---|---|---|---|---|
| logit | 0 | 411.3 | 0.442 | +0.042 |
| logit | 1e-06 | 364.0 | 0.442 | +0.042 |
| logit | 3e-06 | 303.9 | 0.439 | +0.039 |
| logit | 1e-05 | 267.1 | 0.439 | +0.039 |
| logit | 3e-05 | 237.3 | 0.439 | +0.039 |
| logit | 0.0001 | 167.4 | 0.438 | +0.037 |
| logit | 0.0003 | 63.3 | 0.436 | +0.036 |
| logit | 0.001 | 18.2 | 0.431 | +0.031 |
| mlp | 0 | 898.1 | 0.436 | +0.036 |
| mlp | 1e-06 | 898.0 | 0.436 | +0.036 |
| mlp | 3e-06 | 897.7 | 0.436 | +0.036 |
| mlp | 1e-05 | 897.0 | 0.436 | +0.036 |
| mlp | 3e-05 | 886.1 | 0.438 | +0.037 |
| mlp | 0.0001 | 726.6 | 0.442 | +0.042 |
| mlp | 0.0003 | 547.8 | 0.439 | +0.039 |
| mlp | 0.001 | 27.9 | 0.431 | +0.031 |
| rl | 1e-05 | 297.7 | 0.425 | +0.025 |
| rl | 0.0001 | 201.9 | 0.398 | -0.002 |
| critic | 0 | 1180.7 | 0.423 | +0.023 |
| critic | 1e-06 | 1180.7 | 0.423 | +0.023 |
| critic | 3e-06 | 1180.7 | 0.423 | +0.023 |
| critic | 1e-05 | 1180.7 | 0.423 | +0.023 |
| critic | 3e-05 | 1180.7 | 0.423 | +0.023 |
| critic | 0.0001 | 1170.5 | 0.423 | +0.023 |
| critic | 0.0003 | 1074.7 | 0.423 | +0.023 |
| critic | 0.001 | 244.3 | 0.397 | -0.003 |
| mv | 0 | 6.7 | 0.422 | +0.022 |
| mv | 1e-06 | 6.7 | 0.422 | +0.022 |
| mv | 3e-06 | 6.7 | 0.422 | +0.022 |
| mv | 1e-05 | 6.7 | 0.422 | +0.022 |
| mv | 3e-05 | 6.7 | 0.422 | +0.022 |
| mv | 0.0001 | 6.7 | 0.422 | +0.022 |
| mv | 0.0003 | 6.7 | 0.422 | +0.022 |
| mv | 0.001 | 6.7 | 0.422 | +0.022 |
| IMPLICIT | - | 0.0 | 0.400 | +0.000 |
| HEALTH | - | 10.2 | 0.425 | +0.025 |
| HEALTH_PART | - | 10.2 | 0.425 | +0.025 |
| HEALTH_ONE | - | 10.2 | 0.425 | +0.025 |
| HB_TO | - | 145.1 | 0.438 | +0.037 |
| BEACON_REP3 | - | 56.6 | 0.422 | +0.022 |
| FULL | - | 1066.6 | 0.412 | +0.012 |
| HYBRID_P | - | 1022.7 | 0.420 | +0.020 |
| ORACLE | - | 0.0 | 0.456 | +0.056 |

### N = 3

| family | lambda | bytes/ep | P(detect) | vs IMPLICIT |
|---|---|---|---|---|
| logit | 0 | 1182.3 | 0.516 | +0.041 |
| logit | 1e-06 | 984.7 | 0.512 | +0.037 |
| logit | 3e-06 | 693.5 | 0.508 | +0.033 |
| logit | 1e-05 | 517.0 | 0.511 | +0.036 |
| logit | 3e-05 | 374.6 | 0.512 | +0.037 |
| logit | 0.0001 | 179.0 | 0.522 | +0.047 |
| logit | 0.0003 | 53.8 | 0.528 | +0.053 |
| logit | 0.001 | 22.3 | 0.512 | +0.037 |
| mlp | 0 | 2062.4 | 0.520 | +0.045 |
| mlp | 1e-06 | 2049.4 | 0.522 | +0.047 |
| mlp | 3e-06 | 2020.4 | 0.519 | +0.044 |
| mlp | 1e-05 | 1900.2 | 0.512 | +0.037 |
| mlp | 3e-05 | 1603.4 | 0.512 | +0.037 |
| mlp | 0.0001 | 1226.2 | 0.506 | +0.031 |
| mlp | 0.0003 | 123.8 | 0.492 | +0.017 |
| mlp | 0.001 | 36.0 | 0.508 | +0.033 |
| rl | 1e-05 | 546.9 | 0.481 | +0.006 |
| rl | 0.0001 | 345.0 | 0.492 | +0.017 |
| critic | 0 | 2924.3 | 0.495 | +0.020 |
| critic | 1e-06 | 2924.3 | 0.495 | +0.020 |
| critic | 3e-06 | 2924.3 | 0.495 | +0.020 |
| critic | 1e-05 | 2924.3 | 0.495 | +0.020 |
| critic | 3e-05 | 2924.3 | 0.495 | +0.020 |
| critic | 0.0001 | 2833.2 | 0.497 | +0.022 |
| critic | 0.0003 | 1335.5 | 0.484 | +0.009 |
| critic | 0.001 | 5.4 | 0.472 | -0.003 |
| mv | 0 | 11.8 | 0.498 | +0.023 |
| mv | 1e-06 | 11.8 | 0.498 | +0.023 |
| mv | 3e-06 | 11.8 | 0.498 | +0.023 |
| mv | 1e-05 | 11.8 | 0.498 | +0.023 |
| mv | 3e-05 | 11.8 | 0.498 | +0.023 |
| mv | 0.0001 | 11.8 | 0.498 | +0.023 |
| mv | 0.0003 | 11.8 | 0.498 | +0.023 |
| mv | 0.001 | 11.8 | 0.498 | +0.023 |
| IMPLICIT | - | 0.0 | 0.475 | +0.000 |
| HEALTH | - | 20.6 | 0.498 | +0.023 |
| HEALTH_PART | - | 20.6 | 0.508 | +0.033 |
| HEALTH_ONE | - | 20.6 | 0.498 | +0.023 |
| HB_TO | - | 414.0 | 0.484 | +0.009 |
| BEACON_REP3 | - | 105.9 | 0.514 | +0.039 |
| FULL | - | 2879.8 | 0.456 | -0.019 |
| HYBRID_P | - | 2633.9 | 0.492 | +0.017 |
| ORACLE | - | 0.0 | 0.537 | +0.062 |

### N = 5

| family | lambda | bytes/ep | P(detect) | vs IMPLICIT |
|---|---|---|---|---|
| logit | 0 | 3440.8 | 0.583 | +0.027 |
| logit | 1e-06 | 2952.1 | 0.584 | +0.028 |
| logit | 3e-06 | 1990.8 | 0.580 | +0.023 |
| logit | 1e-05 | 1424.5 | 0.564 | +0.008 |
| logit | 3e-05 | 427.0 | 0.572 | +0.016 |
| logit | 0.0001 | 111.9 | 0.558 | +0.002 |
| logit | 0.0003 | 49.5 | 0.566 | +0.009 |
| logit | 0.001 | 12.8 | 0.577 | +0.020 |
| mlp | 0 | 4634.6 | 0.541 | -0.016 |
| mlp | 1e-06 | 4541.4 | 0.544 | -0.013 |
| mlp | 3e-06 | 4184.7 | 0.550 | -0.006 |
| mlp | 1e-05 | 3169.6 | 0.552 | -0.005 |
| mlp | 3e-05 | 2486.9 | 0.550 | -0.006 |
| mlp | 0.0001 | 867.8 | 0.553 | -0.003 |
| mlp | 0.0003 | 67.5 | 0.564 | +0.008 |
| mlp | 0.001 | 38.0 | 0.567 | +0.011 |
| rl | 1e-05 | 883.0 | 0.575 | +0.019 |
| rl | 0.0001 | 544.2 | 0.562 | +0.006 |
| critic | 0 | 8123.4 | 0.562 | +0.006 |
| critic | 1e-06 | 8123.4 | 0.562 | +0.006 |
| critic | 3e-06 | 8123.4 | 0.562 | +0.006 |
| critic | 1e-05 | 8123.4 | 0.562 | +0.006 |
| critic | 3e-05 | 8082.9 | 0.562 | +0.006 |
| critic | 0.0001 | 7522.3 | 0.562 | +0.006 |
| critic | 0.0003 | 1772.1 | 0.541 | -0.016 |
| critic | 0.001 | 0.0 | 0.556 | +0.000 |
| mv | 0 | 15.2 | 0.575 | +0.019 |
| mv | 1e-06 | 15.2 | 0.575 | +0.019 |
| mv | 3e-06 | 15.2 | 0.575 | +0.019 |
| mv | 1e-05 | 15.2 | 0.575 | +0.019 |
| mv | 3e-05 | 15.2 | 0.575 | +0.019 |
| mv | 0.0001 | 15.2 | 0.575 | +0.019 |
| mv | 0.0003 | 15.2 | 0.575 | +0.019 |
| mv | 0.001 | 15.2 | 0.575 | +0.019 |
| IMPLICIT | - | 0.0 | 0.556 | +0.000 |
| HEALTH | - | 40.8 | 0.558 | +0.002 |
| HEALTH_PART | - | 40.8 | 0.564 | +0.008 |
| HEALTH_ONE | - | 40.8 | 0.569 | +0.012 |
| HB_TO | - | 1339.9 | 0.528 | -0.028 |
| BEACON_REP3 | - | 203.6 | 0.570 | +0.014 |
| FULL | - | 9026.8 | 0.359 | -0.197 |
| HYBRID_P | - | 7547.5 | 0.562 | +0.006 |
| ORACLE | - | 0.0 | 0.572 | +0.016 |

### N = 10

| family | lambda | bytes/ep | P(detect) | vs IMPLICIT |
|---|---|---|---|---|
| logit | 0 | 20418.6 | 0.675 | -0.008 |
| logit | 1e-06 | 12650.9 | 0.667 | -0.017 |
| logit | 3e-06 | 8811.6 | 0.654 | -0.029 |
| logit | 1e-05 | 4911.3 | 0.646 | -0.037 |
| logit | 3e-05 | 557.9 | 0.662 | -0.021 |
| logit | 0.0001 | 98.1 | 0.688 | +0.004 |
| logit | 0.0003 | 12.2 | 0.675 | -0.008 |
| logit | 0.001 | 0.0 | 0.683 | +0.000 |
| mlp | 0 | 9217.6 | 0.604 | -0.079 |
| mlp | 1e-06 | 9151.6 | 0.604 | -0.079 |
| mlp | 3e-06 | 9095.7 | 0.604 | -0.079 |
| mlp | 1e-05 | 8806.4 | 0.608 | -0.075 |
| mlp | 3e-05 | 7238.9 | 0.629 | -0.054 |
| mlp | 0.0001 | 119.7 | 0.692 | +0.008 |
| mlp | 0.0003 | 52.6 | 0.683 | +0.000 |
| mlp | 0.001 | 0.0 | 0.683 | +0.000 |
| rl | 1e-05 | 1656.0 | 0.679 | -0.004 |
| rl | 0.0001 | 1566.8 | 0.646 | -0.037 |
| critic | 0 | 43149.6 | 0.662 | -0.021 |
| critic | 1e-06 | 43149.6 | 0.662 | -0.021 |
| critic | 3e-06 | 43149.6 | 0.662 | -0.021 |
| critic | 1e-05 | 43094.7 | 0.662 | -0.021 |
| critic | 3e-05 | 42357.8 | 0.667 | -0.017 |
| critic | 0.0001 | 19734.3 | 0.654 | -0.029 |
| critic | 0.0003 | 0.0 | 0.683 | +0.000 |
| critic | 0.001 | 0.0 | 0.683 | +0.000 |
| mv | 0 | 28.8 | 0.692 | +0.008 |
| mv | 1e-06 | 28.8 | 0.692 | +0.008 |
| mv | 3e-06 | 28.8 | 0.692 | +0.008 |
| mv | 1e-05 | 28.8 | 0.692 | +0.008 |
| mv | 3e-05 | 28.8 | 0.692 | +0.008 |
| mv | 0.0001 | 28.8 | 0.692 | +0.008 |
| mv | 0.0003 | 28.8 | 0.692 | +0.008 |
| mv | 0.001 | 28.8 | 0.692 | +0.008 |
| IMPLICIT | - | 0.0 | 0.683 | +0.000 |
| HEALTH | - | 86.4 | 0.613 | -0.071 |
| HEALTH_PART | - | 86.4 | 0.688 | +0.004 |
| HEALTH_ONE | - | 86.4 | 0.675 | -0.008 |
| HB_TO | - | 7128.9 | 0.583 | -0.100 |
| BEACON_REP3 | - | 515.7 | 0.683 | +0.000 |
| FULL | - | 52761.9 | 0.425 | -0.258 |
| HYBRID_P | - | 41623.7 | 0.662 | -0.021 |
| ORACLE | - | 0.0 | 0.667 | -0.017 |

## 5. Herding index (perm1, episodes where the death precedes detection)

HI = survivors whose FIRST post-death decision targets one of the dead agent's orphan cells / survivors with a post-death decision (ratio of sums, instance-cluster bootstrap CI). coll = same-step identical (cell, view) survivor pairs after the death, per episode.

| N | protocol | HI | 95% CI | coll/ep |
|---|---|---|---|---|
| 2 | IMPLICIT | 0.059 | [0.010,0.114] | 0.000 |
| 2 | HEALTH | 0.116 | [0.060,0.171] | 0.000 |
| 2 | HEALTH_PART | 0.116 | [0.060,0.171] | 0.000 |
| 2 | BEACON_REP3 | 0.118 | [0.062,0.173] | 0.000 |
| 2 | FULL | 0.157 | [0.079,0.241] | 0.000 |
| 2 | ORACLE | 0.150 | [0.095,0.203] | 0.000 |
| 2 | G:logit:0.0001 | 0.118 | [0.069,0.171] | 0.000 |
| 2 | G:mv:0 | 0.104 | [0.049,0.159] | 0.000 |
| 3 | IMPLICIT | 0.082 | [0.049,0.122] | 0.000 |
| 3 | HEALTH | 0.125 | [0.079,0.179] | 0.128 |
| 3 | HEALTH_PART | 0.102 | [0.062,0.145] | 0.002 |
| 3 | BEACON_REP3 | 0.102 | [0.062,0.145] | 0.003 |
| 3 | FULL | 0.143 | [0.087,0.201] | 0.341 |
| 3 | ORACLE | 0.134 | [0.088,0.180] | 0.000 |
| 3 | G:logit:0.0001 | 0.119 | [0.071,0.165] | 0.073 |
| 3 | G:mv:0 | 0.095 | [0.059,0.131] | 0.000 |
| 5 | IMPLICIT | 0.031 | [0.013,0.059] | 0.000 |
| 5 | HEALTH | 0.063 | [0.042,0.092] | 0.305 |
| 5 | HEALTH_PART | 0.037 | [0.016,0.067] | 0.002 |
| 5 | BEACON_REP3 | 0.037 | [0.016,0.067] | 0.002 |
| 5 | FULL | 0.092 | [0.070,0.115] | 1.328 |
| 5 | ORACLE | 0.066 | [0.040,0.100] | 0.013 |
| 5 | G:logit:0.0001 | 0.045 | [0.024,0.076] | 0.030 |
| 5 | G:mv:0 | 0.033 | [0.013,0.062] | 0.002 |
| 10 | IMPLICIT | 0.034 | [0.017,0.057] | 0.000 |
| 10 | HEALTH | 0.055 | [0.033,0.079] | 1.913 |
| 10 | HEALTH_PART | 0.039 | [0.022,0.064] | 0.050 |
| 10 | BEACON_REP3 | 0.039 | [0.022,0.064] | 0.050 |
| 10 | FULL | 0.071 | [0.033,0.113] | 7.250 |
| 10 | ORACLE | 0.064 | [0.043,0.090] | 0.033 |
| 10 | G:logit:0.0001 | 0.038 | [0.021,0.063] | 0.067 |
| 10 | G:mv:0 | 0.035 | [0.017,0.058] | 0.029 |

## 6. Healthy teams (fault none): bytes and P(detect) of the gated protocols

| N | protocol | bytes/ep | vs IMPLICIT |
|---|---|---|---|
| 2 | HYBRID_P | 729.1 | -0.016 [-0.037,+0.006] |
| 2 | FULL | 772.1 | -0.094 [-0.150,-0.037]* |
| 2 | ORACLE | 0.0 | -0.025 [-0.052,+0.003] |
| 2 | G:critic:0 | 729.1 | -0.016 [-0.037,+0.006] |
| 2 | G:critic:0.0001 | 729.1 | -0.016 [-0.037,+0.006] |
| 2 | G:critic:1e-05 | 729.1 | -0.016 [-0.037,+0.006] |
| 2 | G:logit:0 | 112.1 | +0.000 [+0.000,+0.000] |
| 2 | G:logit:0.0001 | 0.0 | +0.000 [+0.000,+0.000] |
| 2 | G:logit:1e-05 | 2.0 | +0.000 [+0.000,+0.000] |
| 2 | G:mlp:0 | 643.8 | -0.017 [-0.041,+0.002] |
| 2 | G:mlp:0.0001 | 302.6 | -0.002 [-0.012,+0.006] |
| 2 | G:mlp:1e-05 | 638.9 | -0.019 [-0.042,+0.000] |
| 2 | G:mv:0 | 0.0 | +0.000 [+0.000,+0.000] |
| 2 | G:mv:0.0001 | 0.0 | +0.000 [+0.000,+0.000] |
| 2 | G:mv:1e-05 | 0.0 | +0.000 [+0.000,+0.000] |
| 2 | G:rl:0.0001 | 109.5 | -0.020 [-0.037,-0.005]* |
| 2 | G:rl:1e-05 | 82.2 | -0.003 [-0.009,+0.000] |
| 3 | HYBRID_P | 2037.6 | -0.009 [-0.038,+0.020] |
| 3 | FULL | 2228.8 | -0.119 [-0.163,-0.073]* |
| 3 | ORACLE | 0.0 | -0.009 [-0.039,+0.022] |
| 3 | G:critic:0 | 2037.6 | -0.009 [-0.038,+0.020] |
| 3 | G:critic:0.0001 | 2037.4 | -0.009 [-0.038,+0.020] |
| 3 | G:critic:1e-05 | 2037.6 | -0.009 [-0.038,+0.020] |
| 3 | G:logit:0 | 483.8 | -0.006 [-0.020,+0.008] |
| 3 | G:logit:0.0001 | 0.0 | +0.000 [+0.000,+0.000] |
| 3 | G:logit:1e-05 | 14.5 | +0.000 [+0.000,+0.000] |
| 3 | G:mlp:0 | 1470.3 | -0.017 [-0.042,+0.006] |
| 3 | G:mlp:0.0001 | 479.6 | -0.002 [-0.008,+0.000] |
| 3 | G:mlp:1e-05 | 1149.8 | -0.013 [-0.032,+0.006] |
| 3 | G:mv:0 | 0.0 | +0.000 [+0.000,+0.000] |
| 3 | G:mv:0.0001 | 0.0 | +0.000 [+0.000,+0.000] |
| 3 | G:mv:1e-05 | 0.0 | +0.000 [+0.000,+0.000] |
| 3 | G:rl:0.0001 | 177.0 | -0.023 [-0.041,-0.005]* |
| 3 | G:rl:1e-05 | 227.1 | -0.008 [-0.022,+0.005] |
| 5 | HYBRID_P | 6635.9 | -0.031 [-0.071,-0.002]* |
| 5 | FULL | 7856.4 | -0.266 [-0.331,-0.194]* |
| 5 | ORACLE | 0.0 | -0.025 [-0.068,+0.009] |
| 5 | G:critic:0 | 6635.9 | -0.031 [-0.071,-0.002]* |
| 5 | G:critic:0.0001 | 6418.9 | -0.031 [-0.071,-0.002]* |
| 5 | G:critic:1e-05 | 6635.9 | -0.031 [-0.071,-0.002]* |
| 5 | G:logit:0 | 1994.0 | +0.002 [-0.031,+0.027] |
| 5 | G:logit:0.0001 | 0.0 | +0.000 [+0.000,+0.000] |
| 5 | G:logit:1e-05 | 167.2 | +0.005 [+0.000,+0.014] |
| 5 | G:mlp:0 | 3623.8 | -0.022 [-0.042,-0.002]* |
| 5 | G:mlp:0.0001 | 0.0 | +0.000 [+0.000,+0.000] |
| 5 | G:mlp:1e-05 | 1908.0 | +0.005 [+0.000,+0.014] |
| 5 | G:mv:0 | 0.0 | +0.000 [+0.000,+0.000] |
| 5 | G:mv:0.0001 | 0.0 | +0.000 [+0.000,+0.000] |
| 5 | G:mv:1e-05 | 0.0 | +0.000 [+0.000,+0.000] |
| 5 | G:rl:0.0001 | 328.4 | -0.020 [-0.041,-0.002]* |
| 5 | G:rl:1e-05 | 364.1 | +0.000 [+0.000,+0.000] |
| 10 | HYBRID_P | 38975.1 | -0.037 [-0.098,+0.025] |
| 10 | FULL | 49131.6 | -0.275 [-0.356,-0.194]* |
| 10 | ORACLE | 0.0 | -0.050 [-0.117,+0.008] |
| 10 | G:critic:0 | 38975.1 | -0.037 [-0.098,+0.025] |
| 10 | G:critic:0.0001 | 18743.2 | -0.029 [-0.092,+0.031] |
| 10 | G:critic:1e-05 | 38975.1 | -0.037 [-0.098,+0.025] |
| 10 | G:logit:0 | 16116.8 | -0.029 [-0.081,+0.023] |
| 10 | G:logit:0.0001 | 0.0 | +0.000 [+0.000,+0.000] |
| 10 | G:logit:1e-05 | 1196.2 | +0.000 [-0.017,+0.021] |
| 10 | G:mlp:0 | 4955.4 | +0.000 [+0.000,+0.000] |
| 10 | G:mlp:0.0001 | 0.0 | +0.000 [+0.000,+0.000] |
| 10 | G:mlp:1e-05 | 4728.0 | +0.000 [+0.000,+0.000] |
| 10 | G:mv:0 | 0.0 | +0.000 [+0.000,+0.000] |
| 10 | G:mv:0.0001 | 0.0 | +0.000 [+0.000,+0.000] |
| 10 | G:mv:1e-05 | 0.0 | +0.000 [+0.000,+0.000] |
| 10 | G:rl:0.0001 | 944.2 | -0.042 [-0.092,+0.000] |
| 10 | G:rl:1e-05 | 900.0 | +0.000 [+0.000,+0.000] |


## 7. Failure analysis on validation labels (instances 2008-2011, never used for fitting)

Calibration of the beacon gates: bin by predicted dV (quintiles); mean realised label per bin (label noise is large: each label is a single paired replay, values in {-1,0,+1}).

| model | bin | n | mean predicted | mean label | - |
|---|---|---|---|---|---|
| logit | 1 | 543 | -0.0008 | +0.0037 | - |
| logit | 2 | 543 | -0.0000 | +0.0000 | - |
| logit | 3 | 542 | +0.0012 | +0.0018 | - |
| logit | 4 | 541 | +0.0123 | +0.0185 | - |
| logit | 5 | 545 | +0.0552 | +0.0642 | - |
| logit | corr(pred,label) | 2714 | +0.206 | | |
| mlp | 1 | 543 | -0.0064 | +0.0000 | - |
| mlp | 2 | 542 | -0.0020 | +0.0000 | - |
| mlp | 3 | 543 | +0.0024 | +0.0074 | - |
| mlp | 4 | 543 | +0.0239 | +0.0147 | - |
| mlp | 5 | 543 | +0.0808 | +0.0663 | - |
| mlp | corr(pred,label) | 2714 | +0.189 | | |
| critic | 1 | 543 | +0.0065 | +0.0424 | - |
| critic | 2 | 542 | +0.0077 | +0.0258 | - |
| critic | 3 | 543 | +0.0094 | +0.0092 | - |
| critic | 4 | 543 | +0.0108 | +0.0055 | - |
| critic | 5 | 543 | +0.0121 | +0.0055 | - |
| critic | corr(pred,label) | 2714 | -0.078 | | |

Where is the beacon label positive? Mean realised beacon label by N, link and steps since death (validation labels):

| N | link | steps since death | n | mean label | share +1 | share -1 |
|---|---|---|---|---|---|---|
| 2 | ideal | 0-1 | 34 | +0.1471 | 0.176 | 0.029 |
| 2 | ideal | early | 43 | +0.1163 | 0.140 | 0.023 |
| 2 | ideal | late | 246 | +0.0203 | 0.033 | 0.012 |
| 2 | burst | 0-1 | 35 | +0.1143 | 0.114 | 0.000 |
| 2 | burst | early | 45 | +0.0222 | 0.044 | 0.022 |
| 2 | burst | late | 246 | +0.0081 | 0.016 | 0.008 |
| 3 | ideal | 0-1 | 43 | +0.1163 | 0.116 | 0.000 |
| 3 | ideal | early | 51 | +0.0392 | 0.039 | 0.000 |
| 3 | ideal | late | 232 | +0.0172 | 0.017 | 0.000 |
| 3 | burst | 0-1 | 42 | +0.1190 | 0.119 | 0.000 |
| 3 | burst | early | 45 | +0.0222 | 0.022 | 0.000 |
| 3 | burst | late | 224 | +0.0045 | 0.004 | 0.000 |
| 5 | ideal | 0-1 | 42 | +0.0476 | 0.071 | 0.024 |
| 5 | ideal | early | 54 | +0.0370 | 0.074 | 0.037 |
| 5 | ideal | late | 252 | -0.0159 | 0.004 | 0.020 |
| 5 | burst | 0-1 | 39 | +0.0256 | 0.051 | 0.026 |
| 5 | burst | early | 46 | +0.0652 | 0.087 | 0.022 |
| 5 | burst | late | 250 | +0.0000 | 0.008 | 0.008 |
| 10 | ideal | 0-1 | 35 | +0.0571 | 0.057 | 0.000 |
| 10 | ideal | early | 54 | +0.0370 | 0.037 | 0.000 |
| 10 | ideal | late | 283 | +0.0000 | 0.004 | 0.004 |
| 10 | burst | 0-1 | 38 | -0.0263 | 0.000 | 0.026 |
| 10 | burst | early | 57 | +0.0175 | 0.018 | 0.000 |
| 10 | burst | late | 278 | +0.0000 | 0.004 | 0.004 |

Sign errors of the logistic beacon gate at lam = 0 (send iff dV_hat > 0): fraction of validation beacon states where it sends yet the realised label is -1 (harm) and where it stays silent though the label is +1 (missed gain):

| N | n | sends | send & label -1 | silent & label +1 | mean label when sends | mean label when silent |
|---|---|---|---|---|---|---|
| 2 | 649 | 0.700 | 0.0123 | 0.0000 | +0.0485 | +0.0000 |
| 3 | 637 | 0.692 | 0.0000 | 0.0031 | +0.0363 | +0.0102 |
| 5 | 683 | 0.624 | 0.0176 | 0.0000 | +0.0094 | +0.0000 |
| 10 | 745 | 0.509 | 0.0040 | 0.0000 | +0.0106 | +0.0000 |

rec / claim labels: mean realised label rec +0.0008, claim -0.0066 on validation (noise level: |mean| < 0.01, rare +/-1). Table 6 shows the logistic and MLP gates send 0 bytes in healthy teams at lambda = 1e-4, so both rec and claim gates are silent at that price; at lambda <= 1e-5 they send records and claims and lose 0.02-0.03 P(detect) (the stale-plan hazard of the [workshop] paper).
