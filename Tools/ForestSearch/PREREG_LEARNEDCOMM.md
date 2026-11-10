# PREREG_LEARNEDCOMM (DRAFT, NOT FROZEN)

Status: DRAFT written after the DEV study (`results/learned_comm_dev.md`). It is not frozen; it must be frozen (sha256 manifest of this file, `learned_comm.py`, `test_learned_comm.py`, the trained-model pickle and the lambda grid) by the user/orchestrator before ANY held-out run. Nothing below has been run on held-out seeds.

**DEV outcome (results/learned_comm_dev.md): the DEV kill criterion fired** (learned gate within +/-2 pp of HEALTH_PART and of the myopic-VoI gate at every N, point estimates). This held-out run is therefore only worth spending if the orchestrator wants a CI-level equivalence test (N = 5, 10 were underpowered on DEV: N = 10 had 6 instances). With 32 instances the G1/G2 equivalence bounds become testable.

## Question (rewritten after NOVELTY_D1.md)
In decentralised search with permanent agent loss and bursty links, how far does a cheap decision-aware gate close the gap to ORACLE communication, as a function of N, link loss and burstiness, compared with the 12 B beacon (HEALTH), the coordinated beacon (HEALTH_PART), an execution-time myopic-VoI gate (no learning) and critic-labelled (ATOC-style) gates? "Gate trained on counterfactual labels" is NOT claimed as new (ATOC 2018, I2C, CMVC, MUTE). The claimed contribution is the regime map and the takeover/herding analysis.

## Seeds (held-out, run ONCE)
- Instance seeds 5000-5031 (32 instances) per cell. Episode seeds 0..E-1, fault seed 10000+ep (as `nscale`).
- DEV seeds used for labels, validation, critic data and model choice: 2000-2011 (+ evaluation DEV 2100-2115). The held-out run must not retrain or re-pick lambda; the lambda per N is frozen from the DEV matched-bytes rule below.
- Never touched: 3000-3015, 4000-4015, 6000-6015 (other studies).

## Design
- Cells: N in {2, 3, 5, 10}; travel 1 (primary) and 0; link in {ideal, burst30_d1} (+ bern30_d1 for P5); fault in {none, perm1}; lambda_events 1; greedy planner; horizons from `nbeacon.HZ`.
- Protocols: IMPLICIT, HEALTH, HEALTH_PART(near), HEALTH_ONE(low), HB_TO, BEACON_REP3, FULL, HYBRID_P, ORACLE, G:logit, G:mlp, G:rl, G:critic, G:mv (execution-time myopic VoI), each gate at the single frozen lambda* per (family, N) plus the full lambda sweep for the Pareto figure. Lookahead (rollout-lite planner) in at most two cells, descriptive.
- Not run (stated): EVENT and DEV_SD (not implemented for N > 2 in `nagent`/`nscale`).
- Episodes per instance: E = 30 (N <= 5), 20 (N = 10). Budget <= 3 CPU-h.

## Metrics
1. Primary: P(detect) at matched bytes. For each gate family, the lambda sweep is interpolated on its upper-left Pareto hull at HEALTH_PART's mean bytes (perm1, pooled over links); difference vs HEALTH_PART and vs HEALTH.
2. Fraction of the ORACLE-IMPLICIT gain captured at matched bytes (reported only when that ceiling > 0.005).
3. Herding index at N = 5, 10: first post-death survivor decisions that target the dead agent's orphan cells / survivors with such a decision; same-step orphan-pair collisions per episode.
4. Bytes per episode, healthy cells (gates must not send in healthy teams beyond the DEV level).

## Hypotheses
- G1 (kill-aware): the best learned gate (logit or mlp, rollout-labelled) differs from HEALTH_PART at matched bytes by more than 2 pp at some N (95% CI of the paired difference excludes the interval [-0.02, +0.02] on the favourable side). If at every N the 95% CI lies inside [-0.02, +0.02], G1 FAILS and the direction is killed: report "hand-made beacon is near-optimal".
- G2: same test against the execution-time myopic-VoI gate (G:mv). Inside +/-2 pp at every N: killed ("learning is a compression of myopic VoI").
- G3 (label source): rollout-labelled gate beats the critic-labelled gate by >= 5 pp of oracle gain at <= 5% of FULL's bytes (N >= 5). < 2 pp: the label-source delta is dead.
- G4 (herding): HEALTH herding index at N = 10 >= 2x that at N = 5; the best gate's herding index is not above HEALTH_PART's.
- G5 (burstiness): at matched mean loss 30%, every gate loses >= 2x more ORACLE gain under burst30_d1 than under bern30_d1.

## Analysis
Paired cluster bootstrap (instances, then episodes within instance), B = 2000, 95% CIs; instance-level t as a cross-check; Holm across the four N within each of G1/G2. A hypothesis passes only if the Holm-adjusted instance-t p < 0.05 AND the bootstrap interval meets the stated bound. Every cell is reported, including failures.

## Stop rules
One run, no reruns with new seeds, no dropped cells. If compute runs out, unfinished cells are NOT RUN and the families INCOMPLETE. All follow-ups are labelled exploratory.

## Escalation
Only if G1 AND G2 both pass on held-out: a two-drone harness mission with injected agent loss (CRANE), as per the plan.
