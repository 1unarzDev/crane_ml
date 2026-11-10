# Health-beacon claim: adversarial robustness test

Code: ../beacon_robustness.py (new file; it subclasses loss_baselines.LEpisode and edits no existing file). Tests: ../test_beacon_robustness.py
(42 pass, plus 2 strict-xfail must-fail tests). DEV tuning: seeds 500-503 x 200 episodes (beacon_tuning.md/.json). Final tables: FRESH seeds
800-811 (never used before; checked by grep), 200 episodes per cell, paired by episode index (every strategy sees the same truth, fault
and link draws), 95% paired cluster bootstrap (instances, then episodes; B = 2000; falsify.boot). `*` = CI excludes 0. Raw per-episode
outcomes: beacon_final_A_raw.npy / beacon_final_B_raw.npy. Full per-cell tables: beacon_final_A.md and beacon_final_B.md (reproduced below).

Claim tested: "under permanent agent loss, a single ~12-byte HEALTH frame recovers essentially all of full messaging's gain over a silent
shared plan (IMPLICIT), and unlike ignoring the partner (INDEP) it costs nothing when nobody fails."

## Strategies added
Silent (0 bytes). All were tuned on DEV only, with two picks per family: _L = best mean gain over R7a-d; _B = best 0.5*(R7 mean + healthy mean).
- TAKEOVER(f,w): IMPLICIT until T = f*H, then score = max(score_IMPLICIT, w * score_assuming_partner_died_at_T). _L: f=0.25, w=1; _B: f=0.25, w=0.5.
- REDUNDANT(rho): planned overlap, P(detect) = (1-rho) pd_IMPLICIT + rho pd_INDEP. Both picks have rho = 0.5.
- BAYES(pi): Bayes-mixture myopic planner that knows the shape of the R7 loss model (one random agent, time ~ U[1, H/2]) and conditions on its
  own survival; pi = prior loss probability. Both picks have pi = 1.0, i.e. it is told a loss is certain. This is the most generous silent baseline.
- SECSWEEP(f): static SECTOR split until f*H, then an unrestricted sweep using own looks plus the partner's nominal sector looks before T. _L: f=0.25; _B: f=0.75.

Messaging:
- HEALTH, HEARTBEAT: exactly as in loss_baselines.py (the tests assert episode-for-episode reproduction).
- HB_TO(P=3, K=7): every agent with a working radio sends a status frame every 3 steps, plus an immediate frame on sensor death or recovery.
  Silence for more than 7 steps => infer death at the last-alive stamp + 1. An alive frame cancels an inferred death; a recovery frame closes
  an outage. P was chosen on DEV (burst-link total-loss gain + burst-link healthy delta).

Fault kinds added:
- perm: identical to falsify's 'loss'.
- transient: sensor outage of length L.
- intermittent: abort probability 0.5 for L steps.
- total: the radio dies with the sensor, so nothing is transmitted after t_dead.
- beacon_lost: the first dead-status frame from the lost agent is dropped.

Links: d0_loss0 (ideal), d1_burst30 (30% bursty loss, mean burst 10, delay 1), d1_loss0, d3_loss0 (new: delay 3).

Scope cuts (75-min compute cap; shared CPU slots made runs slower than planned):
- Part B was run only on builder A (5x5, t0, H16). perm_early, perm_mid, trans_s, trans_l and builder D were dropped from the final run;
  the code supports them, and R7c is the early-loss case.
- N = 3 was skipped: nagent.py is a separate engine with no fault model.
- The characterization-fault fixtures (worktrees/*-characterization, test/characterization-faults) were searched. They are MHSeals hardware
  fixtures and do not apply to this simulator.

## Compact table A: d vs IMPLICIT, ideal link, fresh seeds 800-811

Bytes per episode are given in parentheses where nonzero.

| strategy | R7a | R7b | R7c | R7d | loss mean | R0 | H-b | H-d | R3a | healthy mean |
|---|---|---|---|---|---|---|---|---|---|---|
| ORACLE | +0.065* | +0.072* | +0.091* | +0.034* | +0.066 | -0.003 | -0.004 | -0.003 | +0.010 | +0.000 |
| HYBRID_P (0.8-1.9 kB) | +0.051* | +0.060* | +0.067* | +0.030* | +0.052 | -0.003 | -0.004 | -0.003 | +0.010 | +0.000 |
| HEALTH (9-11 B under loss, 0 B healthy) | +0.049* | +0.061* | +0.085* | +0.028* | +0.056 | 0 | 0 | 0 | 0 | 0 |
| HEARTBEAT (49-160 B; 33-139 B healthy) | +0.049* | +0.061* | +0.085* | +0.028* | +0.056 | 0 | 0 | 0 | 0 | 0 |
| HB_TO (79-257 B; 61-231 B healthy) | +0.049* | +0.061* | +0.085* | +0.028* | +0.056 | 0 | 0 | 0 | 0 | 0 |
| INDEP | +0.015 | +0.024 | +0.058* | -0.002 | +0.024 | -0.076* | -0.094* | -0.093* | -0.109* | -0.093 |
| TAKEOVER_L | +0.036* | +0.032* | +0.053* | +0.010 | +0.033 | -0.036* | -0.055* | -0.058* | -0.029* | -0.045 |
| TAKEOVER_B | +0.005 | +0.020* | +0.021* | +0.007 | +0.013 | -0.005 | -0.009 | -0.005 | -0.003 | -0.006 |
| REDUNDANT (rho .5) | +0.016 | +0.030* | +0.047* | +0.008 | +0.025 | -0.022 | -0.040* | -0.022* | -0.022* | -0.027 |
| BAYES (pi 1) | +0.029* | +0.048* | +0.055* | +0.018 | +0.038 | -0.045* | -0.049* | -0.033* | -0.028* | -0.039 |
| SECSWEEP_L | +0.035* | +0.047* | +0.064* | +0.011 | +0.039 | -0.047* | -0.077* | -0.069* | -0.052* | -0.061 |
| SECSWEEP_B | -0.007 | +0.003 | +0.023 | +0.001 | +0.005 | -0.003 | -0.012 | -0.001 | -0.005 | -0.005 |

HEALTH minus the silent hedges under loss (ideal link):
- Every silent hedge is below HEALTH in every R7 regime (point estimates).
- The CI of (hedge - HEALTH) excludes 0 for TAKEOVER_L in R7b/c/d, BAYES in R7a/c, and REDUNDANT in R7a/b/c.
- In single regimes the strongest silent arms come within noise of HEALTH: TAKEOVER_L R7a -0.013 [-0.031,+0.005]; BAYES R7b
  -0.013 [-0.033,+0.007] and R7d -0.011 [-0.034,+0.011]; SECSWEEP_L in R7a/b/d. Each of those arms loses 0.03-0.08 on every healthy
  cell (CIs exclude 0).

HEALTH against the reference strategies:
- vs HYBRID_P: within noise in R7a/b/d; HEALTH is better in R7c (+0.019 [+0.001,+0.036]).
- vs ORACLE: within noise except R7a (ORACLE +0.017 [+0.003,+0.032]).
- HEALTH recovers 96-127% of HYBRID_P's gain and 75-93% of ORACLE's.

Bursty link (d1_burst30), d vs IMPLICIT, R7a-d:
- HEALTH: +0.035* / +0.046* / +0.055* / +0.021*.
- HEARTBEAT: +0.042 / +0.060 / +0.069 / +0.021.
- HB_TO: +0.048 / +0.058 / +0.083 / +0.025.

On this link HB_TO pays a healthy-team cost because it falsely infers deaths: R0 -0.015*, H-b -0.018*, H-d -0.021*, R3a -0.022*. HEALTH
and HEARTBEAT stay at exactly 0 on healthy cells, because they never act on silence.

## Compact table B: other loss / abort conditions

Builder A = 5x5 t0 H16. Values are d vs IMPLICIT; bytes per episode in parentheses.

| condition | link | HEALTH | HEARTBEAT | HB_TO | TAKEOVER_L | BAYES | INDEP |
|---|---|---|---|---|---|---|---|
| perm, R7a (t 1-8), reference | ideal | +0.049* (9) | +0.049* (49) | +0.049* (79) | +0.036* | +0.029* | +0.015 |
| perm_late (t 9-12) | ideal | +0.003 (5) | +0.003 (40) | +0.003 (68) | -0.022* | -0.021* | -0.065* |
| perm_late | burst30 / d1 / d3 | +0.000 / +0.002 / +0.000 | +0.001 / +0.002 / +0.000 | -0.008 / +0.002 / +0.000 | | | |
| transient, L=4 | ideal | -0.028* (9) | -0.028* (47) | -0.019* (81) | -0.032* | -0.028* | -0.075* |
| transient, L=4 | burst30 | -0.019* | -0.022* | -0.021* | | | |
| intermittent (p=0.5, 8 steps) | ideal | -0.037* (10) | -0.037* (48) | -0.018* (94) | -0.023* | -0.029* | -0.077* |
| intermittent | burst30 | -0.032* | -0.035* | -0.023* | | | |
| TOTAL (radio+sensor) | ideal | +0.000 (0) | +0.000 (24) | +0.041* (44) | +0.036* | +0.029* | +0.015 |
| TOTAL | burst30 / d1 / d3 | +0.000 / +0.000 / +0.000 | +0.000 x3 | +0.040* / +0.041* / +0.041* | | | |
| beacon_lost (first frame) | ideal | +0.000 (9) | +0.046* (49) | +0.047* (79) | +0.036* | +0.029* | +0.015 |
| beacon_lost | burst30 | +0.000 | +0.036* | +0.047* | | | |

TOTAL and beacon_lost use the same instances, victims and death times as R7a, so their silent rows equal R7a's. Delay alone (d1, d3)
changes nothing material for any beacon variant in perm_late or TOTAL.

## Answers

(1) Is there a silent baseline that matches HEALTH under loss without a healthy-team cost?

No. Four families and eight DEV-tuned arms were tested, including a Bayes planner that is told the loss-time distribution and that a loss
is certain. Every arm sits on the same trade-off:
- The arms that come within noise of HEALTH in some R7 regimes (TAKEOVER_L, BAYES, SECSWEEP_L) lose 0.029-0.077 on every healthy cell,
  with CIs excluding 0.
- The arms that are nearly free on healthy teams (TAKEOVER_B, SECSWEEP_B; healthy mean -0.006 / -0.005) recover only 0.005-0.013 of
  HEALTH's 0.056 mean loss gain.
- On the balanced DEV objective HEALTH scores +0.026 and the best silent arm +0.007.

The "costs nothing when nobody fails" half holds structurally. HEALTH never transmits or acts unless something fails, so on healthy cells it
is identical to IMPLICIT episode-for-episode (tested), on every link.

(2) Where HEALTH fails:
- TOTAL loss (radio dies with the sensor): HEALTH = IMPLICIT exactly (0 B, gain 0). The plain HEARTBEAT also gains 0, because it only
  retransmits a beacon and never acts on silence. HB_TO (silence timeout) restores +0.040 to +0.041* on all four links, about 84% of
  HEALTH's non-total gain on the same faults, for 44 B/episode. Its price is false death inferences on a bursty link, which cost
  0.015-0.022* on healthy teams (0 on ideal or pure-delay links). A silent TAKEOVER_L also gets +0.036 under total loss, but costs 0.045
  on healthy teams.
- Lost beacon: one dropped frame turns HEALTH into IMPLICIT (gain 0 by construction). HEARTBEAT (+0.046*, ideal) and HB_TO (+0.047*) fix
  it. On the 30% bursty link HEALTH still keeps +0.035 in R7a, because most single frames get through.
- Late loss (t in 9-12 of H16): no strategy adds value. HEALTH gains +0.003 (n.s.), and the silent hedges are harmful (-0.02*).
- Transient outages and intermittent degradation: HEALTH as specified sends its beacon at the first bad step and the partner is then
  dropped for good. This is harmful: -0.028* for a 4-step outage and -0.037* for intermittent degradation, vs IMPLICIT. HEARTBEAT is
  identical. HB_TO's recovery frames halve the damage but stay negative (-0.019*, -0.018*): once the partner's plan has diverged from
  nominal, going back to the nominal prediction is wrong. Under non-permanent faults the best strategy measured is plain IMPLICIT.
  HEALTH relies on the assumption that the agent instantly and correctly diagnoses its loss as permanent.
- Delay of 1 or 3 steps on a lossless link: no effect.

(3) Narrowest true statement:

The claim holds when an agent is permanently lost early or mid-mission, keeps its radio, correctly self-diagnoses the loss as permanent, and
the single frame arrives. Under those conditions one 12-byte HEALTH frame recovers essentially all of full messaging's gain over IMPLICIT
(here 96-127% of HYBRID_P's and 75-93% of ORACLE's, at about 1% of the bytes), and it is exactly free on a healthy team. No silent default
plan tested reaches that gain without a healthy-team cost of 0.03-0.08.

The single frame is fragile:
- Covering total loss needs periodic status frames with a silence timeout: +0.04 at about 44-80 B/episode, at the cost of false alarms on
  bursty links (-0.02).
- A one-shot "I am dead" beacon hurts (-0.03 to -0.04) when the fault is transient or intermittent.

## Recommended paper wording

"Under permanent agent loss with a working radio, a single 12-byte health frame sent at the moment of failure recovers essentially all of
the gain that full record-sharing achieves over the silent shared plan (fresh instances: +0.028 to +0.085 vs. +0.030 to +0.067). Because it
is only sent on failure, it changes nothing for a healthy team. No silent hedge we tried (time-triggered takeover, planned overlap, a Bayes
planner that knows the loss distribution, sector-then-sweep) matches it without losing 0.03-0.08 detection probability when nobody fails.
The result has limits. If the radio fails with the sensor, or the single frame is lost, the beacon has no effect at all. Periodic status
frames with a silence timeout recover most of the gain (+0.04 for about 44-80 bytes per episode) but raise false alarms on bursty links
(-0.02 on healthy teams). When the outage is transient or intermittent, a one-shot 'I am lost' message is harmful (-0.03 to -0.04), so the
beacon is only worth sending when the failure is known to be permanent."

---

## DEV tuning (seeds 500-503)

DEV tuning seeds [500, 501, 502, 503] episodes 200: mean d vs IMPLICIT (ideal link unless noted)

| strategy | R7a | R7b | R7c | R7d | R0 | H-b (5x5 t1 H30) | H-d (8x8 t1 H40) | R3a | total(A) | total(D) | burst R7 mean | burst healthy mean | loss mean | healthy mean | balanced |
|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|
| IMPLICIT | +0.000 | +0.000 | +0.000 | +0.000 | +0.000 | +0.000 | +0.000 | +0.000 | +0.000 | +0.000 | +0.000 | +0.000 | +0.000 | +0.000 | +0.000 |
| HEALTH | +0.039 | +0.025 | +0.120 | +0.025 | +0.000 | +0.000 | +0.000 | +0.000 | +0.000 | +0.000 | +0.036 | +0.000 | +0.052 | +0.000 | +0.026 |
| TAKEOVER_f0.25_w0.5 | +0.004 | +0.015 | +0.005 | +0.028 | -0.001 | +0.001 | +0.000 | +0.003 | +0.004 | +0.028 | +0.013 | +0.001 | +0.013 | +0.001 | +0.007 |
| TAKEOVER_f0.25_w1.0 | +0.033 | +0.018 | +0.059 | +0.028 | -0.045 | -0.052 | -0.043 | -0.025 | +0.033 | +0.028 | +0.034 | -0.041 | +0.034 | -0.041 | -0.004 |
| TAKEOVER_f0.5_w0.5 | +0.000 | +0.000 | +0.003 | +0.018 | +0.000 | +0.000 | +0.000 | +0.001 | +0.000 | +0.018 | +0.005 | +0.000 | +0.005 | +0.000 | +0.003 |
| TAKEOVER_f0.5_w1.0 | +0.010 | +0.007 | +0.019 | +0.028 | -0.018 | -0.013 | -0.010 | -0.007 | +0.010 | +0.028 | +0.016 | -0.012 | +0.016 | -0.012 | +0.002 |
| TAKEOVER_f0.75_w0.5 | +0.000 | +0.001 | +0.001 | +0.000 | +0.001 | +0.001 | +0.000 | +0.000 | +0.000 | +0.000 | +0.001 | +0.001 | +0.001 | +0.001 | +0.001 |
| TAKEOVER_f0.75_w1.0 | +0.000 | +0.001 | +0.000 | +0.001 | -0.003 | +0.001 | -0.003 | -0.001 | +0.000 | +0.001 | +0.001 | -0.001 | +0.001 | -0.001 | -0.000 |
| REDUNDANT_r0.1 | +0.000 | +0.003 | +0.019 | -0.001 | +0.001 | -0.019 | -0.003 | +0.001 | +0.000 | -0.001 | +0.005 | -0.005 | +0.005 | -0.005 | +0.000 |
| REDUNDANT_r0.25 | -0.004 | +0.004 | +0.030 | -0.006 | -0.010 | -0.029 | -0.025 | -0.005 | -0.004 | -0.006 | +0.006 | -0.017 | +0.006 | -0.017 | -0.006 |
| REDUNDANT_r0.5 | +0.013 | +0.021 | +0.069 | +0.018 | -0.024 | -0.020 | -0.031 | -0.025 | +0.013 | +0.018 | +0.030 | -0.025 | +0.030 | -0.025 | +0.003 |
| BAYES_pi0.1 | +0.000 | +0.000 | +0.000 | +0.001 | +0.001 | +0.000 | +0.001 | +0.000 | +0.000 | +0.001 | +0.000 | +0.001 | +0.000 | +0.001 | +0.000 |
| BAYES_pi0.25 | -0.001 | +0.004 | +0.001 | +0.005 | -0.004 | -0.004 | -0.003 | +0.000 | -0.001 | +0.005 | +0.002 | -0.003 | +0.002 | -0.003 | -0.000 |
| BAYES_pi0.5 | -0.001 | +0.004 | +0.005 | +0.021 | -0.007 | -0.022 | +0.005 | -0.003 | -0.001 | +0.021 | +0.007 | -0.007 | +0.007 | -0.007 | +0.000 |
| BAYES_pi1.0 | +0.041 | +0.035 | +0.070 | +0.039 | -0.043 | -0.058 | -0.034 | -0.028 | +0.041 | +0.039 | +0.046 | -0.040 | +0.046 | -0.040 | +0.003 |
| SECSWEEP_f0.25 | -0.001 | +0.013 | +0.058 | +0.001 | -0.077 | -0.107 | -0.058 | -0.048 | -0.001 | +0.001 | +0.018 | -0.072 | +0.018 | -0.072 | -0.027 |
| SECSWEEP_f0.5 | +0.004 | +0.003 | +0.048 | -0.020 | -0.035 | -0.056 | -0.025 | -0.019 | +0.004 | -0.020 | +0.008 | -0.034 | +0.008 | -0.034 | -0.013 |
| SECSWEEP_f0.75 | -0.001 | -0.003 | +0.036 | -0.028 | -0.005 | -0.019 | +0.001 | -0.013 | -0.001 | -0.028 | +0.001 | -0.009 | +0.001 | -0.009 | -0.004 |
| SECSWEEP_f1.01 | -0.014 | -0.018 | +0.021 | -0.025 | -0.005 | -0.007 | +0.005 | -0.016 | -0.014 | -0.025 | -0.009 | -0.006 | -0.009 | -0.006 | -0.007 |
| HB_TO_P2 | +0.039 | +0.025 | +0.120 | +0.025 | +0.000 | +0.000 | +0.000 | +0.000 | +0.048 | +0.033 | +0.049 | -0.021 | +0.052 | +0.000 | +0.026 |
| HB_TO_P3 | +0.039 | +0.025 | +0.120 | +0.025 | +0.000 | +0.000 | +0.000 | +0.000 | +0.058 | +0.029 | +0.054 | -0.020 | +0.052 | +0.000 | +0.026 |
| HB_TO_P5 | +0.039 | +0.025 | +0.120 | +0.025 | +0.000 | +0.000 | +0.000 | +0.000 | +0.020 | +0.020 | +0.049 | -0.017 | +0.052 | +0.000 | +0.026 |

TAKEOVER: loss-tuned = TAKEOVER_f0.25_w1.0, balanced = TAKEOVER_f0.25_w0.5

REDUNDANT: loss-tuned = REDUNDANT_r0.5, balanced = REDUNDANT_r0.5

BAYES: loss-tuned = BAYES_pi1.0, balanced = BAYES_pi1.0

SECSWEEP: loss-tuned = SECSWEEP_f0.25, balanced = SECSWEEP_f0.75
HB_TO: chosen HB_TO_P3 (max burst-link total-loss gain + burst healthy delta: HB_TO_P2 +0.009, HB_TO_P3 +0.020, HB_TO_P5 -0.002)

---

FRESH seeds [800, 801, 802, 803, 804, 805, 806, 807, 808, 809, 810, 811], 200 episodes/cell, paired by episode index (same truth, fault, link draws), 95% cluster bootstrap (instances then episodes, B=2000). * = CI excludes 0.


## Part A: silent hedges vs HEALTH, R7 (loss) and healthy cells


### R7a

| cell | link | strategy | P(detect) | d vs IMPLICIT | d vs HEALTH | bytes/ep |
|---|---|---|---|---|---|---|
| R7a | d0_loss0 | ORACLE | 0.585 | +0.065 [+0.043,+0.087]* | +0.017 [+0.003,+0.032]* | 0.0 |
| R7a | d0_loss0 | IMPLICIT | 0.520 | - | -0.049 [-0.068,-0.028]* | 0.0 |
| R7a | d0_loss0 | INDEP | 0.535 | +0.015 [-0.013,+0.044] | -0.033 [-0.058,-0.010]* | 0.0 |
| R7a | d0_loss0 | SECTOR | 0.508 | -0.012 [-0.039,+0.014] | -0.060 [-0.086,-0.034]* | 0.0 |
| R7a | d0_loss0 | HYBRID_P | 0.571 | +0.051 [+0.030,+0.072]* | +0.002 [-0.015,+0.018] | 946.0 |
| R7a | d0_loss0 | HEALTH | 0.569 | +0.049 [+0.028,+0.068]* | - | 8.9 |
| R7a | d0_loss0 | HEARTBEAT | 0.569 | +0.049 [+0.028,+0.068]* | +0.000 [+0.000,+0.000] | 48.7 |
| R7a | d0_loss0 | HB_TO | 0.569 | +0.049 [+0.028,+0.068]* | +0.000 [+0.000,+0.000] | 78.9 |
| R7a | d0_loss0 | TAKEOVER_L | 0.556 | +0.036 [+0.018,+0.054]* | -0.013 [-0.031,+0.005] | 0.0 |
| R7a | d0_loss0 | TAKEOVER_B | 0.525 | +0.005 [-0.003,+0.014] | -0.044 [-0.063,-0.024]* | 0.0 |
| R7a | d0_loss0 | REDUNDANT_L | 0.536 | +0.016 [-0.009,+0.041] | -0.033 [-0.060,-0.006]* | 0.0 |
| R7a | d0_loss0 | BAYES_L | 0.549 | +0.029 [+0.012,+0.048]* | -0.020 [-0.038,-0.001]* | 0.0 |
| R7a | d0_loss0 | SECSWEEP_L | 0.555 | +0.035 [+0.008,+0.060]* | -0.014 [-0.037,+0.009] | 0.0 |
| R7a | d0_loss0 | SECSWEEP_B | 0.513 | -0.007 [-0.032,+0.017] | -0.055 [-0.080,-0.031]* | 0.0 |
| R7a | d1_burst30 | HYBRID_P | 0.547 | +0.027 [+0.004,+0.049]* | -0.008 [-0.025,+0.010] | 971.4 |
| R7a | d1_burst30 | HEALTH | 0.555 | +0.035 [+0.016,+0.052]* | - | 8.9 |
| R7a | d1_burst30 | HEARTBEAT | 0.562 | +0.042 [+0.023,+0.060]* | +0.007 [-0.000,+0.015] | 49.0 |
| R7a | d1_burst30 | HB_TO | 0.568 | +0.048 [+0.027,+0.068]* | +0.013 [+0.002,+0.025]* | 79.2 |

### R7b

| cell | link | strategy | P(detect) | d vs IMPLICIT | d vs HEALTH | bytes/ep |
|---|---|---|---|---|---|---|
| R7b | d0_loss0 | ORACLE | 0.534 | +0.072 [+0.046,+0.097]* | +0.010 [-0.002,+0.024] | 0.0 |
| R7b | d0_loss0 | IMPLICIT | 0.463 | - | -0.061 [-0.089,-0.035]* | 0.0 |
| R7b | d0_loss0 | INDEP | 0.487 | +0.024 [-0.004,+0.052] | -0.037 [-0.061,-0.013]* | 0.0 |
| R7b | d0_loss0 | SECTOR | 0.459 | -0.004 [-0.035,+0.031] | -0.065 [-0.095,-0.035]* | 0.0 |
| R7b | d0_loss0 | HYBRID_P | 0.522 | +0.060 [+0.036,+0.083]* | -0.002 [-0.019,+0.016] | 1172.4 |
| R7b | d0_loss0 | HEALTH | 0.524 | +0.061 [+0.035,+0.089]* | - | 9.8 |
| R7b | d0_loss0 | HEARTBEAT | 0.524 | +0.061 [+0.035,+0.089]* | +0.000 [+0.000,+0.000] | 102.3 |
| R7b | d0_loss0 | HB_TO | 0.524 | +0.061 [+0.035,+0.089]* | +0.000 [+0.000,+0.000] | 169.2 |
| R7b | d0_loss0 | TAKEOVER_L | 0.495 | +0.032 [+0.014,+0.051]* | -0.029 [-0.053,-0.007]* | 0.0 |
| R7b | d0_loss0 | TAKEOVER_B | 0.483 | +0.020 [+0.011,+0.031]* | -0.041 [-0.067,-0.015]* | 0.0 |
| R7b | d0_loss0 | REDUNDANT_L | 0.492 | +0.030 [+0.008,+0.054]* | -0.031 [-0.057,-0.005]* | 0.0 |
| R7b | d0_loss0 | BAYES_L | 0.510 | +0.048 [+0.029,+0.069]* | -0.013 [-0.033,+0.007] | 0.0 |
| R7b | d0_loss0 | SECSWEEP_L | 0.510 | +0.047 [+0.016,+0.079]* | -0.014 [-0.038,+0.010] | 0.0 |
| R7b | d0_loss0 | SECSWEEP_B | 0.465 | +0.003 [-0.028,+0.035] | -0.059 [-0.086,-0.030]* | 0.0 |
| R7b | d1_burst30 | HYBRID_P | 0.507 | +0.044 [+0.020,+0.066]* | -0.002 [-0.021,+0.015] | 1227.6 |
| R7b | d1_burst30 | HEALTH | 0.509 | +0.046 [+0.024,+0.068]* | - | 9.8 |
| R7b | d1_burst30 | HEARTBEAT | 0.523 | +0.060 [+0.035,+0.087]* | +0.014 [+0.005,+0.024]* | 102.5 |
| R7b | d1_burst30 | HB_TO | 0.520 | +0.058 [+0.031,+0.086]* | +0.012 [-0.002,+0.025] | 170.0 |

### R7c

| cell | link | strategy | P(detect) | d vs IMPLICIT | d vs HEALTH | bytes/ep |
|---|---|---|---|---|---|---|
| R7c | d0_loss0 | ORACLE | 0.559 | +0.091 [+0.059,+0.122]* | +0.006 [-0.005,+0.017] | 0.0 |
| R7c | d0_loss0 | IMPLICIT | 0.468 | - | -0.085 [-0.115,-0.053]* | 0.0 |
| R7c | d0_loss0 | INDEP | 0.525 | +0.058 [+0.027,+0.091]* | -0.028 [-0.053,-0.003]* | 0.0 |
| R7c | d0_loss0 | SECTOR | 0.491 | +0.023 [-0.010,+0.057] | -0.062 [-0.088,-0.035]* | 0.0 |
| R7c | d0_loss0 | HYBRID_P | 0.534 | +0.067 [+0.034,+0.095]* | -0.019 [-0.036,-0.001]* | 1013.6 |
| R7c | d0_loss0 | HEALTH | 0.553 | +0.085 [+0.053,+0.115]* | - | 10.3 |
| R7c | d0_loss0 | HEARTBEAT | 0.553 | +0.085 [+0.053,+0.115]* | +0.000 [+0.000,+0.000] | 53.1 |
| R7c | d0_loss0 | HB_TO | 0.553 | +0.085 [+0.053,+0.115]* | +0.000 [+0.000,+0.000] | 83.1 |
| R7c | d0_loss0 | TAKEOVER_L | 0.520 | +0.053 [+0.032,+0.074]* | -0.033 [-0.060,-0.004]* | 0.0 |
| R7c | d0_loss0 | TAKEOVER_B | 0.488 | +0.021 [+0.006,+0.038]* | -0.065 [-0.099,-0.031]* | 0.0 |
| R7c | d0_loss0 | REDUNDANT_L | 0.514 | +0.047 [+0.018,+0.077]* | -0.039 [-0.065,-0.014]* | 0.0 |
| R7c | d0_loss0 | BAYES_L | 0.522 | +0.055 [+0.030,+0.081]* | -0.030 [-0.059,-0.002]* | 0.0 |
| R7c | d0_loss0 | SECSWEEP_L | 0.531 | +0.064 [+0.030,+0.095]* | -0.022 [-0.043,+0.000] | 0.0 |
| R7c | d0_loss0 | SECSWEEP_B | 0.490 | +0.023 [-0.011,+0.057] | -0.062 [-0.088,-0.036]* | 0.0 |
| R7c | d1_burst30 | HYBRID_P | 0.507 | +0.040 [+0.014,+0.064]* | -0.016 [-0.036,+0.003] | 1036.5 |
| R7c | d1_burst30 | HEALTH | 0.523 | +0.055 [+0.030,+0.077]* | - | 10.3 |
| R7c | d1_burst30 | HEARTBEAT | 0.536 | +0.069 [+0.040,+0.095]* | +0.013 [+0.004,+0.022]* | 54.0 |
| R7c | d1_burst30 | HB_TO | 0.550 | +0.083 [+0.050,+0.113]* | +0.028 [+0.012,+0.043]* | 83.7 |

### R7d

| cell | link | strategy | P(detect) | d vs IMPLICIT | d vs HEALTH | bytes/ep |
|---|---|---|---|---|---|---|
| R7d | d0_loss0 | ORACLE | 0.349 | +0.034 [+0.009,+0.058]* | +0.005 [-0.004,+0.015] | 0.0 |
| R7d | d0_loss0 | IMPLICIT | 0.315 | - | -0.028 [-0.050,-0.006]* | 0.0 |
| R7d | d0_loss0 | INDEP | 0.313 | -0.002 [-0.023,+0.020] | -0.030 [-0.050,-0.011]* | 0.0 |
| R7d | d0_loss0 | SECTOR | 0.313 | -0.002 [-0.026,+0.020] | -0.030 [-0.055,-0.006]* | 0.0 |
| R7d | d0_loss0 | HYBRID_P | 0.345 | +0.030 [+0.010,+0.051]* | +0.002 [-0.010,+0.014] | 1939.8 |
| R7d | d0_loss0 | HEALTH | 0.343 | +0.028 [+0.006,+0.050]* | - | 10.5 |
| R7d | d0_loss0 | HEARTBEAT | 0.343 | +0.028 [+0.006,+0.050]* | +0.000 [+0.000,+0.000] | 159.6 |
| R7d | d0_loss0 | HB_TO | 0.343 | +0.028 [+0.006,+0.050]* | +0.000 [+0.000,+0.000] | 256.9 |
| R7d | d0_loss0 | TAKEOVER_L | 0.325 | +0.010 [-0.007,+0.029] | -0.018 [-0.035,-0.002]* | 0.0 |
| R7d | d0_loss0 | TAKEOVER_B | 0.323 | +0.007 [-0.001,+0.017] | -0.021 [-0.043,+0.003] | 0.0 |
| R7d | d0_loss0 | REDUNDANT_L | 0.323 | +0.008 [-0.009,+0.028] | -0.020 [-0.045,+0.005] | 0.0 |
| R7d | d0_loss0 | BAYES_L | 0.333 | +0.018 [-0.001,+0.036] | -0.011 [-0.034,+0.011] | 0.0 |
| R7d | d0_loss0 | SECSWEEP_L | 0.326 | +0.011 [-0.011,+0.031] | -0.018 [-0.036,+0.000] | 0.0 |
| R7d | d0_loss0 | SECSWEEP_B | 0.316 | +0.001 [-0.023,+0.023] | -0.027 [-0.051,-0.004]* | 0.0 |
| R7d | d1_burst30 | HYBRID_P | 0.333 | +0.018 [+0.001,+0.037]* | -0.003 [-0.016,+0.010] | 2010.3 |
| R7d | d1_burst30 | HEALTH | 0.336 | +0.021 [+0.003,+0.041]* | - | 10.5 |
| R7d | d1_burst30 | HEARTBEAT | 0.336 | +0.021 [+0.001,+0.040]* | +0.000 [-0.009,+0.010] | 159.9 |
| R7d | d1_burst30 | HB_TO | 0.340 | +0.025 [+0.003,+0.048]* | +0.004 [-0.009,+0.017] | 257.2 |

### R0

| cell | link | strategy | P(detect) | d vs IMPLICIT | d vs HEALTH | bytes/ep |
|---|---|---|---|---|---|---|
| R0 | d0_loss0 | ORACLE | 0.681 | -0.003 [-0.019,+0.013] | -0.003 [-0.019,+0.013] | 0.0 |
| R0 | d0_loss0 | IMPLICIT | 0.684 | - | +0.000 [+0.000,+0.000] | 0.0 |
| R0 | d0_loss0 | INDEP | 0.608 | -0.076 [-0.102,-0.052]* | -0.076 [-0.102,-0.052]* | 0.0 |
| R0 | d0_loss0 | SECTOR | 0.684 | +0.000 [-0.020,+0.020] | +0.000 [-0.020,+0.020] | 0.0 |
| R0 | d0_loss0 | HYBRID_P | 0.681 | -0.003 [-0.019,+0.013] | -0.003 [-0.019,+0.013] | 835.3 |
| R0 | d0_loss0 | HEALTH | 0.684 | +0.000 [+0.000,+0.000] | - | 0.0 |
| R0 | d0_loss0 | HEARTBEAT | 0.684 | +0.000 [+0.000,+0.000] | +0.000 [+0.000,+0.000] | 35.6 |
| R0 | d0_loss0 | HB_TO | 0.684 | +0.000 [+0.000,+0.000] | +0.000 [+0.000,+0.000] | 64.1 |
| R0 | d0_loss0 | TAKEOVER_L | 0.647 | -0.036 [-0.053,-0.020]* | -0.036 [-0.053,-0.020]* | 0.0 |
| R0 | d0_loss0 | TAKEOVER_B | 0.678 | -0.005 [-0.012,+0.001] | -0.005 [-0.012,+0.001] | 0.0 |
| R0 | d0_loss0 | REDUNDANT_L | 0.662 | -0.022 [-0.042,+0.000] | -0.022 [-0.042,+0.000] | 0.0 |
| R0 | d0_loss0 | BAYES_L | 0.639 | -0.045 [-0.065,-0.025]* | -0.045 [-0.065,-0.025]* | 0.0 |
| R0 | d0_loss0 | SECSWEEP_L | 0.637 | -0.047 [-0.070,-0.023]* | -0.047 [-0.070,-0.023]* | 0.0 |
| R0 | d0_loss0 | SECSWEEP_B | 0.680 | -0.003 [-0.024,+0.017] | -0.003 [-0.024,+0.017] | 0.0 |
| R0 | d1_burst30 | HYBRID_P | 0.673 | -0.010 [-0.025,+0.004] | -0.010 [-0.025,+0.004] | 845.3 |
| R0 | d1_burst30 | HEALTH | 0.684 | +0.000 [+0.000,+0.000] | - | 0.0 |
| R0 | d1_burst30 | HEARTBEAT | 0.684 | +0.000 [+0.000,+0.000] | +0.000 [+0.000,+0.000] | 35.6 |
| R0 | d1_burst30 | HB_TO | 0.669 | -0.015 [-0.025,-0.005]* | -0.015 [-0.025,-0.005]* | 64.8 |

### H-b (5x5 t1 H30)

| cell | link | strategy | P(detect) | d vs IMPLICIT | d vs HEALTH | bytes/ep |
|---|---|---|---|---|---|---|
| H-b (5x5 t1 H30) | d0_loss0 | ORACLE | 0.657 | -0.004 [-0.019,+0.010] | -0.004 [-0.019,+0.010] | 0.0 |
| H-b (5x5 t1 H30) | d0_loss0 | IMPLICIT | 0.661 | - | +0.000 [+0.000,+0.000] | 0.0 |
| H-b (5x5 t1 H30) | d0_loss0 | INDEP | 0.568 | -0.094 [-0.113,-0.074]* | -0.094 [-0.113,-0.074]* | 0.0 |
| H-b (5x5 t1 H30) | d0_loss0 | SECTOR | 0.648 | -0.013 [-0.035,+0.008] | -0.013 [-0.035,+0.008] | 0.0 |
| H-b (5x5 t1 H30) | d0_loss0 | HYBRID_P | 0.657 | -0.004 [-0.019,+0.011] | -0.004 [-0.019,+0.011] | 804.6 |
| H-b (5x5 t1 H30) | d0_loss0 | HEALTH | 0.661 | +0.000 [+0.000,+0.000] | - | 0.0 |
| H-b (5x5 t1 H30) | d0_loss0 | HEARTBEAT | 0.661 | +0.000 [+0.000,+0.000] | +0.000 [+0.000,+0.000] | 82.5 |
| H-b (5x5 t1 H30) | d0_loss0 | HB_TO | 0.661 | +0.000 [+0.000,+0.000] | +0.000 [+0.000,+0.000] | 143.2 |
| H-b (5x5 t1 H30) | d0_loss0 | TAKEOVER_L | 0.606 | -0.055 [-0.074,-0.037]* | -0.055 [-0.074,-0.037]* | 0.0 |
| H-b (5x5 t1 H30) | d0_loss0 | TAKEOVER_B | 0.652 | -0.009 [-0.023,+0.003] | -0.009 [-0.023,+0.003] | 0.0 |
| H-b (5x5 t1 H30) | d0_loss0 | REDUNDANT_L | 0.622 | -0.040 [-0.061,-0.018]* | -0.040 [-0.061,-0.018]* | 0.0 |
| H-b (5x5 t1 H30) | d0_loss0 | BAYES_L | 0.612 | -0.049 [-0.067,-0.032]* | -0.049 [-0.067,-0.032]* | 0.0 |
| H-b (5x5 t1 H30) | d0_loss0 | SECSWEEP_L | 0.584 | -0.077 [-0.107,-0.053]* | -0.077 [-0.107,-0.053]* | 0.0 |
| H-b (5x5 t1 H30) | d0_loss0 | SECSWEEP_B | 0.650 | -0.012 [-0.034,+0.007] | -0.012 [-0.034,+0.007] | 0.0 |
| H-b (5x5 t1 H30) | d1_burst30 | HYBRID_P | 0.647 | -0.014 [-0.024,-0.005]* | -0.014 [-0.024,-0.005]* | 810.0 |
| H-b (5x5 t1 H30) | d1_burst30 | HEALTH | 0.661 | +0.000 [+0.000,+0.000] | - | 0.0 |
| H-b (5x5 t1 H30) | d1_burst30 | HEARTBEAT | 0.661 | +0.000 [+0.000,+0.000] | +0.000 [+0.000,+0.000] | 82.5 |
| H-b (5x5 t1 H30) | d1_burst30 | HB_TO | 0.644 | -0.018 [-0.029,-0.007]* | -0.018 [-0.029,-0.007]* | 145.9 |

### H-d (8x8 t1 H40)

| cell | link | strategy | P(detect) | d vs IMPLICIT | d vs HEALTH | bytes/ep |
|---|---|---|---|---|---|---|
| H-d (8x8 t1 H40) | d0_loss0 | ORACLE | 0.466 | -0.003 [-0.015,+0.008] | -0.003 [-0.015,+0.008] | 0.0 |
| H-d (8x8 t1 H40) | d0_loss0 | IMPLICIT | 0.469 | - | +0.000 [+0.000,+0.000] | 0.0 |
| H-d (8x8 t1 H40) | d0_loss0 | INDEP | 0.376 | -0.093 [-0.124,-0.060]* | -0.093 [-0.124,-0.060]* | 0.0 |
| H-d (8x8 t1 H40) | d0_loss0 | SECTOR | 0.470 | +0.001 [-0.017,+0.018] | +0.001 [-0.017,+0.018] | 0.0 |
| H-d (8x8 t1 H40) | d0_loss0 | HYBRID_P | 0.466 | -0.003 [-0.015,+0.008] | -0.003 [-0.015,+0.008] | 1363.2 |
| H-d (8x8 t1 H40) | d0_loss0 | HEALTH | 0.469 | +0.000 [+0.000,+0.000] | - | 0.0 |
| H-d (8x8 t1 H40) | d0_loss0 | HEARTBEAT | 0.469 | +0.000 [+0.000,+0.000] | +0.000 [+0.000,+0.000] | 139.1 |
| H-d (8x8 t1 H40) | d0_loss0 | HB_TO | 0.469 | +0.000 [+0.000,+0.000] | +0.000 [+0.000,+0.000] | 231.2 |
| H-d (8x8 t1 H40) | d0_loss0 | TAKEOVER_L | 0.411 | -0.058 [-0.081,-0.035]* | -0.058 [-0.081,-0.035]* | 0.0 |
| H-d (8x8 t1 H40) | d0_loss0 | TAKEOVER_B | 0.465 | -0.005 [-0.015,+0.006] | -0.005 [-0.015,+0.006] | 0.0 |
| H-d (8x8 t1 H40) | d0_loss0 | REDUNDANT_L | 0.447 | -0.022 [-0.043,-0.003]* | -0.022 [-0.043,-0.003]* | 0.0 |
| H-d (8x8 t1 H40) | d0_loss0 | BAYES_L | 0.436 | -0.033 [-0.053,-0.014]* | -0.033 [-0.053,-0.014]* | 0.0 |
| H-d (8x8 t1 H40) | d0_loss0 | SECSWEEP_L | 0.400 | -0.069 [-0.098,-0.041]* | -0.069 [-0.098,-0.041]* | 0.0 |
| H-d (8x8 t1 H40) | d0_loss0 | SECSWEEP_B | 0.468 | -0.001 [-0.019,+0.017] | -0.001 [-0.019,+0.017] | 0.0 |
| H-d (8x8 t1 H40) | d1_burst30 | HYBRID_P | 0.453 | -0.016 [-0.030,-0.002]* | -0.016 [-0.030,-0.002]* | 1372.0 |
| H-d (8x8 t1 H40) | d1_burst30 | HEALTH | 0.469 | +0.000 [+0.000,+0.000] | - | 0.0 |
| H-d (8x8 t1 H40) | d1_burst30 | HEARTBEAT | 0.469 | +0.000 [+0.000,+0.000] | +0.000 [+0.000,+0.000] | 139.1 |
| H-d (8x8 t1 H40) | d1_burst30 | HB_TO | 0.448 | -0.021 [-0.036,-0.007]* | -0.021 [-0.036,-0.007]* | 233.5 |

### R3a

| cell | link | strategy | P(detect) | d vs IMPLICIT | d vs HEALTH | bytes/ep |
|---|---|---|---|---|---|---|
| R3a | d0_loss0 | ORACLE | 0.745 | +0.010 [-0.004,+0.025] | +0.010 [-0.004,+0.025] | 0.0 |
| R3a | d0_loss0 | IMPLICIT | 0.734 | - | +0.000 [+0.000,+0.000] | 0.0 |
| R3a | d0_loss0 | INDEP | 0.625 | -0.109 [-0.140,-0.077]* | -0.109 [-0.140,-0.077]* | 0.0 |
| R3a | d0_loss0 | SECTOR | 0.730 | -0.005 [-0.022,+0.013] | -0.005 [-0.022,+0.013] | 0.0 |
| R3a | d0_loss0 | HYBRID_P | 0.745 | +0.010 [-0.004,+0.025] | +0.010 [-0.004,+0.025] | 773.6 |
| R3a | d0_loss0 | HEALTH | 0.734 | +0.000 [+0.000,+0.000] | - | 0.0 |
| R3a | d0_loss0 | HEARTBEAT | 0.734 | +0.000 [+0.000,+0.000] | +0.000 [+0.000,+0.000] | 33.1 |
| R3a | d0_loss0 | HB_TO | 0.734 | +0.000 [+0.000,+0.000] | +0.000 [+0.000,+0.000] | 60.5 |
| R3a | d0_loss0 | TAKEOVER_L | 0.705 | -0.029 [-0.042,-0.016]* | -0.029 [-0.042,-0.016]* | 0.0 |
| R3a | d0_loss0 | TAKEOVER_B | 0.732 | -0.003 [-0.008,+0.003] | -0.003 [-0.008,+0.003] | 0.0 |
| R3a | d0_loss0 | REDUNDANT_L | 0.712 | -0.022 [-0.040,-0.006]* | -0.022 [-0.040,-0.006]* | 0.0 |
| R3a | d0_loss0 | BAYES_L | 0.706 | -0.028 [-0.044,-0.013]* | -0.028 [-0.044,-0.013]* | 0.0 |
| R3a | d0_loss0 | SECSWEEP_L | 0.682 | -0.052 [-0.072,-0.030]* | -0.052 [-0.072,-0.030]* | 0.0 |
| R3a | d0_loss0 | SECSWEEP_B | 0.729 | -0.005 [-0.022,+0.013] | -0.005 [-0.022,+0.013] | 0.0 |
| R3a | d1_burst30 | HYBRID_P | 0.717 | -0.017 [-0.030,-0.004]* | -0.017 [-0.030,-0.004]* | 806.4 |
| R3a | d1_burst30 | HEALTH | 0.734 | +0.000 [+0.000,+0.000] | - | 0.0 |
| R3a | d1_burst30 | HEARTBEAT | 0.734 | +0.000 [+0.000,+0.000] | +0.000 [+0.000,+0.000] | 33.1 |
| R3a | d1_burst30 | HB_TO | 0.713 | -0.022 [-0.032,-0.013]* | -0.022 [-0.032,-0.013]* | 61.2 |

FRESH seeds [800, 801, 802, 803, 804, 805, 806, 807, 808, 809, 810, 811], 200 episodes/cell, paired by episode index (same truth, fault, link draws), 95% cluster bootstrap (instances then episodes, B=2000). * = CI excludes 0.


## Part B: loss and abort conditions x links (builders A = 5x5 t0 H16, D = 8x8 t1 H40)


### A / perm_late: {'kind': 'perm', 't_lo': 9, 't_hi': 12}

| link | strategy | P(detect) | d vs IMPLICIT | d vs HEALTH | bytes/ep |
|---|---|---|---|---|---|
| d0_loss0 | ORACLE | 0.643 | +0.007 [-0.014,+0.027] | +0.004 [-0.014,+0.022] | 0.0 |
| d0_loss0 | IMPLICIT | 0.636 | - | -0.003 [-0.013,+0.008] | 0.0 |
| d0_loss0 | INDEP | 0.571 | -0.065 [-0.091,-0.040]* | -0.068 [-0.094,-0.043]* | 0.0 |
| d0_loss0 | HEALTH | 0.639 | +0.003 [-0.008,+0.013] | - | 5.4 |
| d0_loss0 | HEARTBEAT | 0.639 | +0.003 [-0.008,+0.013] | +0.000 [+0.000,+0.000] | 40.1 |
| d0_loss0 | HB_TO | 0.639 | +0.003 [-0.008,+0.013] | +0.000 [+0.000,+0.000] | 68.2 |
| d0_loss0 | TAKEOVER_L | 0.615 | -0.022 [-0.038,-0.005]* | -0.024 [-0.042,-0.007]* | 0.0 |
| d0_loss0 | BAYES_L | 0.615 | -0.021 [-0.037,-0.005]* | -0.024 [-0.040,-0.007]* | 0.0 |
| d1_burst30 | HEALTH | 0.637 | +0.000 [-0.009,+0.010] | - | 5.4 |
| d1_burst30 | HEARTBEAT | 0.637 | +0.001 [-0.008,+0.010] | +0.000 [+0.000,+0.002] | 40.1 |
| d1_burst30 | HB_TO | 0.628 | -0.008 [-0.022,+0.006] | -0.008 [-0.019,+0.002] | 68.8 |
| d1_loss0 | HEALTH | 0.638 | +0.002 [-0.009,+0.012] | - | 5.4 |
| d1_loss0 | HEARTBEAT | 0.638 | +0.002 [-0.009,+0.012] | +0.000 [+0.000,+0.000] | 40.1 |
| d1_loss0 | HB_TO | 0.638 | +0.002 [-0.009,+0.012] | +0.000 [+0.000,+0.000] | 68.2 |
| d3_loss0 | HEALTH | 0.636 | +0.000 [-0.010,+0.010] | - | 5.4 |
| d3_loss0 | HEARTBEAT | 0.636 | +0.000 [-0.010,+0.010] | +0.000 [+0.000,+0.000] | 40.1 |
| d3_loss0 | HB_TO | 0.636 | +0.000 [-0.010,+0.010] | +0.000 [+0.000,+0.000] | 68.3 |

### A / trans_m: {'kind': 'transient', 't_lo': 1, 't_hi': 8, 'L': 4}

| link | strategy | P(detect) | d vs IMPLICIT | d vs HEALTH | bytes/ep |
|---|---|---|---|---|---|
| d0_loss0 | ORACLE | 0.653 | +0.002 [-0.020,+0.023] | +0.030 [+0.009,+0.050]* | 0.0 |
| d0_loss0 | IMPLICIT | 0.651 | - | +0.028 [+0.014,+0.044]* | 0.0 |
| d0_loss0 | INDEP | 0.576 | -0.075 [-0.102,-0.049]* | -0.047 [-0.069,-0.025]* | 0.0 |
| d0_loss0 | HEALTH | 0.623 | -0.028 [-0.044,-0.014]* | - | 8.9 |
| d0_loss0 | HEARTBEAT | 0.623 | -0.028 [-0.044,-0.014]* | +0.000 [+0.000,+0.000] | 47.1 |
| d0_loss0 | HB_TO | 0.632 | -0.019 [-0.034,-0.005]* | +0.010 [+0.000,+0.019] | 81.2 |
| d0_loss0 | TAKEOVER_L | 0.619 | -0.032 [-0.047,-0.016]* | -0.004 [-0.020,+0.013] | 0.0 |
| d0_loss0 | BAYES_L | 0.623 | -0.028 [-0.044,-0.010]* | +0.000 [-0.015,+0.017] | 0.0 |
| d1_burst30 | HEALTH | 0.632 | -0.019 [-0.034,-0.005]* | - | 8.9 |
| d1_burst30 | HEARTBEAT | 0.629 | -0.022 [-0.038,-0.007]* | -0.003 [-0.008,+0.002] | 47.0 |
| d1_burst30 | HB_TO | 0.630 | -0.021 [-0.039,-0.003]* | -0.002 [-0.016,+0.012] | 81.2 |

### A / intermit: {'kind': 'intermit', 't_lo': 1, 't_hi': 4, 'L': 8, 'p_bad': 0.5}

| link | strategy | P(detect) | d vs IMPLICIT | d vs HEALTH | bytes/ep |
|---|---|---|---|---|---|
| d0_loss0 | ORACLE | 0.653 | +0.001 [-0.024,+0.024] | +0.038 [+0.016,+0.061]* | 0.0 |
| d0_loss0 | IMPLICIT | 0.652 | - | +0.037 [+0.019,+0.055]* | 0.0 |
| d0_loss0 | INDEP | 0.575 | -0.077 [-0.102,-0.051]* | -0.040 [-0.064,-0.018]* | 0.0 |
| d0_loss0 | HEALTH | 0.615 | -0.037 [-0.055,-0.019]* | - | 9.6 |
| d0_loss0 | HEARTBEAT | 0.615 | -0.037 [-0.055,-0.019]* | +0.000 [+0.000,+0.000] | 48.4 |
| d0_loss0 | HB_TO | 0.634 | -0.018 [-0.032,-0.003]* | +0.019 [+0.008,+0.030]* | 94.0 |
| d0_loss0 | TAKEOVER_L | 0.628 | -0.023 [-0.040,-0.007]* | +0.013 [-0.001,+0.028] | 0.0 |
| d0_loss0 | BAYES_L | 0.623 | -0.029 [-0.045,-0.011]* | +0.008 [-0.009,+0.024] | 0.0 |
| d1_burst30 | HEALTH | 0.620 | -0.032 [-0.048,-0.016]* | - | 9.6 |
| d1_burst30 | HEARTBEAT | 0.617 | -0.035 [-0.052,-0.018]* | -0.003 [-0.007,+0.002] | 48.4 |
| d1_burst30 | HB_TO | 0.628 | -0.023 [-0.040,-0.007]* | +0.009 [-0.004,+0.022] | 94.2 |

### A / total: {'kind': 'perm', 't_lo': 1, 't_hi': 8, 'total': True}

| link | strategy | P(detect) | d vs IMPLICIT | d vs HEALTH | bytes/ep |
|---|---|---|---|---|---|
| d0_loss0 | ORACLE | 0.585 | +0.065 [+0.043,+0.087]* | +0.065 [+0.043,+0.087]* | 0.0 |
| d0_loss0 | IMPLICIT | 0.520 | - | +0.000 [+0.000,+0.000] | 0.0 |
| d0_loss0 | INDEP | 0.535 | +0.015 [-0.013,+0.044] | +0.015 [-0.013,+0.044] | 0.0 |
| d0_loss0 | HEALTH | 0.520 | +0.000 [+0.000,+0.000] | - | 0.0 |
| d0_loss0 | HEARTBEAT | 0.520 | +0.000 [+0.000,+0.000] | +0.000 [+0.000,+0.000] | 23.7 |
| d0_loss0 | HB_TO | 0.561 | +0.041 [+0.023,+0.061]* | +0.041 [+0.023,+0.061]* | 43.8 |
| d0_loss0 | TAKEOVER_L | 0.556 | +0.036 [+0.018,+0.054]* | +0.036 [+0.018,+0.054]* | 0.0 |
| d0_loss0 | BAYES_L | 0.549 | +0.029 [+0.012,+0.048]* | +0.029 [+0.012,+0.048]* | 0.0 |
| d1_burst30 | HEALTH | 0.520 | +0.000 [+0.000,+0.000] | - | 0.0 |
| d1_burst30 | HEARTBEAT | 0.520 | +0.000 [+0.000,+0.000] | +0.000 [+0.000,+0.000] | 23.7 |
| d1_burst30 | HB_TO | 0.560 | +0.040 [+0.022,+0.059]* | +0.040 [+0.022,+0.059]* | 43.9 |
| d1_loss0 | HEALTH | 0.520 | +0.000 [+0.000,+0.000] | - | 0.0 |
| d1_loss0 | HEARTBEAT | 0.520 | +0.000 [+0.000,+0.000] | +0.000 [+0.000,+0.000] | 23.7 |
| d1_loss0 | HB_TO | 0.561 | +0.041 [+0.023,+0.061]* | +0.041 [+0.023,+0.061]* | 43.8 |
| d3_loss0 | HEALTH | 0.520 | +0.000 [+0.000,+0.000] | - | 0.0 |
| d3_loss0 | HEARTBEAT | 0.520 | +0.000 [+0.000,+0.000] | +0.000 [+0.000,+0.000] | 23.7 |
| d3_loss0 | HB_TO | 0.561 | +0.041 [+0.023,+0.061]* | +0.041 [+0.023,+0.061]* | 43.8 |

### A / beacon_lost: {'kind': 'perm', 't_lo': 1, 't_hi': 8, 'drop_first': True}

| link | strategy | P(detect) | d vs IMPLICIT | d vs HEALTH | bytes/ep |
|---|---|---|---|---|---|
| d0_loss0 | ORACLE | 0.585 | +0.065 [+0.043,+0.087]* | +0.065 [+0.043,+0.087]* | 0.0 |
| d0_loss0 | IMPLICIT | 0.520 | - | +0.000 [+0.000,+0.000] | 0.0 |
| d0_loss0 | INDEP | 0.535 | +0.015 [-0.013,+0.044] | +0.015 [-0.013,+0.044] | 0.0 |
| d0_loss0 | HEALTH | 0.520 | +0.000 [+0.000,+0.000] | - | 8.9 |
| d0_loss0 | HEARTBEAT | 0.566 | +0.046 [+0.025,+0.067]* | +0.046 [+0.025,+0.067]* | 48.8 |
| d0_loss0 | HB_TO | 0.567 | +0.047 [+0.027,+0.067]* | +0.047 [+0.027,+0.067]* | 78.8 |
| d0_loss0 | TAKEOVER_L | 0.556 | +0.036 [+0.018,+0.054]* | +0.036 [+0.018,+0.054]* | 0.0 |
| d0_loss0 | BAYES_L | 0.549 | +0.029 [+0.012,+0.048]* | +0.029 [+0.012,+0.048]* | 0.0 |
| d1_burst30 | HEALTH | 0.520 | +0.000 [+0.000,+0.000] | - | 8.9 |
| d1_burst30 | HEARTBEAT | 0.556 | +0.036 [+0.019,+0.053]* | +0.036 [+0.019,+0.053]* | 49.1 |
| d1_burst30 | HB_TO | 0.567 | +0.047 [+0.027,+0.067]* | +0.047 [+0.027,+0.067]* | 79.1 |
