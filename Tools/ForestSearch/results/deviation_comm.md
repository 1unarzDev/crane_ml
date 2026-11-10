# Event-triggered prediction-deviation baseline (DEVIATION; ConTaCT / Roth et al. style)

Requested by REREVIEW_senior.md item 1. Code: ../deviation_comm.py (new file; subclasses beacon_robustness.BEpisode; no existing
file edited). Tests: ../test_deviation_comm.py (all pass, plus 3 strict-xfail must-fail tests). Raw per-episode outcomes, bytes and
frame logs (JSON) and the run scripts: crane_ml/Docs/ResearchOrchestration/AGENT_HANDOFFS/deviation_comm_raw/.
All comparisons are paired by episode index: every strategy sees the same truth, outcome uniforms, fault and link draws. CIs are 95%
cluster bootstraps (falsify.boot: instances first, then episodes; B = 2000; rng seed 1). `*` = CI excludes 0. "Pooled" rows treat
(cell, instance) pairs as clusters. Per-cell CIs are not multiplicity-corrected.

## Protocol

- **Common ground (CG).** Each agent keeps a copy of "what my partner can predict about both of us". It starts as the shared default
  plan. When a CG agent is idle, CG predicts its next task with the same greedy rule as the real planner, using only common-ground
  looks: outcomes default to NULL unless a message corrected them, and the other agent's CG tasks in flight count as virtual looks.
  With no messages, CG is exactly nominal_plan() (tested).
- **Planning.** A real agent plans on its own true records plus its CG model of the partner. This is IMPLICIT, with the static
  default plan replaced by the CG re-plan.
- **Frames.** A frame is 12 B: a 4 B header plus two 4 B items. An item is `task(seq, cell, view, start, finish)`,
  `rec(seq, cell, view, outcome)`, or `idle(t0)`.
  - The sender applies its own frame to its CG copy at t + delay, assuming delivery (no acks).
  - The receiver applies the frame on delivery.
  - On lossless links the two copies stay identical (tested at delay 0, 1 and 3).
- **Triggers.**
  - DEV_TASK: send `task` iff the agent's actual new task differs from CG's prediction of it.
  - DEV_INFO: in every step, compute the partner's greedy argmax from CG with and without my not-yet-common records (exact, same
    score). Send the records iff the argmax changes.
  - DEV: both triggers. A task frame's spare slot carries the newest pending record at no extra cost.
  - DEV_SD: DEV plus self-diagnosis, the same assumption HEALTH makes. While its sensor is dead, the agent's actual plan is "no look",
    so it sends `idle(t)` iff CG still predicts looks for it. A later real task re-declares it.
  - _B<b>: at most b bytes per agent per episode. Once spent, the agent is silent and its partner predicts it from CG.
- **Prop 1 equivalence (tested).** With no private events, DEV, DEV_TASK, DEV_INFO, DEV_SD and the budgeted variants send 0 bytes and
  are episode-for-episode IMPLICIT. This was checked on d0, d1 and d1_burst30, travel 0 and 1. With budget 0 they are IMPLICIT even
  with events.
- **Observed identities (not theorems; tested on subsets).**
  - On the ideal link, DEV = ORACLE in 19199/19200 healthy Part A episodes at 300 episodes (and 6400/6400 in a 100-episode pilot).
  - DEV_SD = ORACLE in 4800/4800 loss episodes (R7a-d).
  - DEV = HYBRID_P in 4797/4800 loss episodes.
  - Why: on a zero-delay lossless link, every deviation that could change the partner's greedy argmax is common knowledge before the
    partner next decides. The info test cannot see the partner's own private records, so the identity is not guaranteed.

Compute cap: a single process on shared, saturated CPU slots within a 100-min budget, and DEV costs 3-4x IMPLICIT per episode.
- Part B: 100 episodes per cell (not 200). DEV_TASK and DEV_INFO were run on the ideal link only.
- Part C: 100 episodes, ideal link only.
- Part A: 300 episodes as specified, but only ORACLE, IMPLICIT and DEV (both links). HYBRID / HYBRID_P are quoted from confirm.md
  (same seeds 200-207, same 300 paired episodes). Our ORACLE - IMPLICIT means reproduce confirm.md cell for cell.
- The fsrun slots were saturated for more than 15 min, so the 300-episode Part A ran directly under nice -n 19 (one process).
- Budget sweep: done on the loss cells (DEV_SD_B12, with DEV_B24 code present), not on Part A.

## Compact table A: healthy held-out confirmatory cells (seeds 200-207, 300 episodes, hints and aborts at lambda 1 and 3)

d vs IMPLICIT. HYBRID / HYBRID_P (d1_loss0) are from confirm.md: same instances, same paired episodes.

| cell | ORACLE (ideal) | DEV ideal (B/ep) | DEV d1_loss0 (B/ep) | HYBRID d1_loss0 | HYBRID_P d1_loss0 |
|---|---|---|---|---|---|
| 5x5 t0 lam1 | +0.008 [-0.009,+0.027] | +0.008 [-0.009,+0.027] (27) | +0.003 [-0.013,+0.021] (53) | -0.043* | -0.005 |
| 5x5 t0 lam3 | +0.010 [-0.009,+0.026] | +0.010 [-0.009,+0.026] (78) | +0.010 [-0.005,+0.024] (147) | -0.049* | -0.006 |
| 5x5 t1 lam1 | +0.000 [-0.010,+0.011] | +0.000 [-0.010,+0.011] (35) | -0.007 [-0.018,+0.003] (47) | -0.015* | -0.011 |
| 5x5 t1 lam3 | -0.005 [-0.019,+0.009] | -0.005 [-0.019,+0.009] (103) | -0.003 [-0.016,+0.010] (149) | -0.010 | -0.008 |
| 6x6 t0 lam1 | +0.000 [-0.021,+0.022] | +0.000 [-0.021,+0.022] (31) | +0.001 [-0.018,+0.022] (60) | -0.048* | -0.003 |
| 6x6 t0 lam3 | -0.005 [-0.023,+0.012] | -0.005 [-0.023,+0.012] (90) | -0.003 [-0.017,+0.013] (170) | -0.063* | -0.009 |
| 6x6 t1 lam1 | -0.006 [-0.018,+0.006] | -0.005 [-0.017,+0.006] (52) | -0.008 [-0.020,+0.004] (64) | -0.007 | -0.002 |
| 6x6 t1 lam3 | -0.003 [-0.015,+0.010] | -0.003 [-0.015,+0.010] (149) | +0.004 [-0.009,+0.017] (198) | -0.005 | -0.004 |
| pooled (64 clusters) | +0.000 [-0.005,+0.006] | +0.000 [-0.005,+0.006] (71) | -0.000 [-0.006,+0.005] (111) | | |

The largest DEV-IMPLICIT upper CI in any cell is +0.027. Bytes: 27-149 B/ep on the ideal link and 47-198 B/ep at delay 1, against
IMPLICIT's 0 and HYBRID's 870-1680.

## Answers

**(1) Does DEVIATION reduce to the beacon under loss?** Only once the agent self-diagnoses its failure, which is the assumption HEALTH
already makes. With that assumption it does, almost exactly. Without it, DEVIATION recovers about HYBRID_P's gain for about 1/8 of
HYBRID_P's bytes.

- **DEV_SD (self-diagnosing).** The protocol emits the beacon automatically. The lost agent sends 0.98-1.04 frames after death, the
  first at a median of 0-1 steps (HEALTH: 1 frame at t_dead). Content: one `idle(t)` item, i.e. "drop my predicted looks from t", which
  is HEALTH's semantics; with no other events, DEV_SD is HEALTH episode for episode (tested).
  - With hints and aborts present, it also sends about 2 deviation frames from the healthy partner and 0.2-0.6 from the victim before
    death. That gives 32 B/ep against HEALTH's 10.
  - Pooled R7a-d, ideal link: +0.065 [+0.046,+0.084] vs IMPLICIT, i.e. +0.007 [-0.002,+0.017] over HEALTH (n.s.). It equals ORACLE
    in 4800/4800 episodes.
  - On d1_burst30: +0.043, identical to HEALTH (+0.000 [-0.011,+0.011]), at 45 B.
  - So HEALTH is the deviation protocol restricted to its one decision-relevant event. It is derived, not hand-picked.
- **DEV with no self-diagnosis.** Each post-death look aborts, so every re-plan deviates and the dead agent streams 8.5-22 frames
  (17-29 on the bursty link).
  - Gain: +0.049 [+0.032,+0.067] = HYBRID_P (4797/4800 identical episodes), at 158 B against 1266 B. It is -0.009 [-0.020,+0.004]
    vs HEALTH.
  - On the bursty link it collapses to +0.010 [-0.005,+0.026], -0.032* vs HEALTH, because lost task frames leave the partner with a
    stale model.
- **Byte budget.** A 12 B per-agent budget (one frame) is a poor beacon: in 13-32% of episodes the frame was spent earlier on a hint
  or abort deviation. Gain is +0.045 (-0.012* vs HEALTH) at 18 B.
- **Part C.**
  - Total loss: no frame can be sent, and every DEV variant equals IMPLICIT within noise (-0.013 n.s.). Only the HB_TO silence
    timeout helps (+0.040*).
  - Lost first frame: DEV_SD has no retransmission and gains nothing (-0.007). The naive stream is partly robust (+0.025, n.s.).
  - Transient and intermittent faults: DEV_SD's recovery declaration removes most of HEALTH's harm (-0.011 and -0.007 vs IMPLICIT, n.s.;
    +0.038* over HEALTH when intermittent). This is the one condition where the derived protocol is strictly better than the
    hand-made beacon.

**(2) On healthy teams with hints, does DEVIATION beat silence?** No.
- On the 8 held-out cells (300 paired episodes, lambda 1 and 3) DEV ranges from -0.005 to +0.010 vs IMPLICIT, and no CI excludes 0.
  The largest upper bound is +0.027. Pooled: +0.000 [-0.005,+0.006] at 71 B/ep.
- On the ideal link, DEV reproduces centralized greedy ORACLE in 19199/19200 episodes. So, for a greedy team, a principled
  event-triggered protocol cannot do better than the Oracle, and the Oracle does not beat silence.
- This sharpens the null. Within these models the "silence is enough" result does not come from a weak messaging protocol: the best
  event-triggered protocol, which sends exactly the information that would change the partner's choice, adds +0.000 [-0.005,+0.006].

**(3) Under delay, does DEVIATION suffer the stale-plan hazard?** Not on lossless delay-1 links.
- DEV at d1_loss0 ranges from -0.008 to +0.010 per cell (pooled -0.000 [-0.006,+0.005]). On the same cells HYBRID loses 0.043-0.063
  at travel 0, and HYBRID_P loses 0.002-0.011.
- Reason: both agents re-plan on a common-ground model that is updated at the same (known) delayed time. In effect, the
  priority-order repair is built into the deviation semantics.
- The hazard does return when the link also loses frames. On d1_burst30 under agent loss, naive DEV keeps only +0.010 of HYBRID_P's
  +0.033, because with no acks the sender's CG copy diverges from the receiver's.
  - DEV_SD avoids this only because it needs one frame (+0.043).
  - The cost in bytes is that DEV sends about 1.6x more at delay 1 (111 vs 71 B/ep): more deviations become visible before the CG
    copy catches up.

## Suggested paper wording

"Following Roth et al. and ConTaCT, we added an event-triggered baseline. Each agent maintains the common-ground model its partner
uses to predict it (the default plan, re-planned on everything already sent). It transmits a 12-byte frame only when its next task
deviates from that prediction, or when its private evidence would change the partner's greedy choice. With no private events it is
silent and identical to Implicit, a corollary of Prop. 1. On the eight held-out healthy cells it reproduces the centralized greedy
Oracle almost exactly (19,199/19,200 episodes) and gains +0.000 [-0.005, +0.006] over silence (per-cell maxima +0.010, upper CI
+0.027), at 27-149 bytes per episode. On a delay-1 link it shows no stale-plan loss (-0.000 [-0.006, +0.005]), unlike layered Hybrid
(-0.04 to -0.06). Under permanent agent loss, an agent that knows its sensor is dead emits exactly one deviation frame at failure
('my predicted looks will not happen') and matches the health beacon (+0.065 vs +0.058; +0.043 vs +0.043 on the bursty link). The
beacon is therefore the event-triggered protocol's only decision-relevant message under loss, not a hand-picked heuristic. Without
self-diagnosis the same protocol streams one frame per aborted look and recovers Hybrid-P's gain (+0.049) at about 1/8 of its bytes,
but loses it on a bursty delayed link (+0.010). Neither variant helps when the radio dies with the sensor; there only a silence timeout
works."

Caveats to state:
- Parts B and C use 100 episodes per cell, not 200.
- The DEV = ORACLE identity is empirical. The info trigger cannot see the partner's private records; 1/19200 episodes differ.
- Self-diagnosis is an assumption, and it is shared with HEALTH.
- Frames are a generous compact encoding (two 4 B items per 12 B frame); HYBRID_P's encoding is 12 + 8 per record + 8 per lease.

---

## Compact table B: fresh agent-loss regimes R7a-d (seeds 800-811, 100 episodes per cell), pooled over the 4 regimes

d vs IMPLICIT, pooled over 48 (regime, instance) clusters, with bytes per episode. Per-regime tables are in Part B below.

| strategy | ideal: d vs IMPLICIT | ideal: d vs HEALTH | ideal B/ep | d1_burst30: d vs IMPLICIT | d1_burst30: d vs HEALTH | burst B/ep |
|---|---|---|---|---|---|---|
| ORACLE | +0.065 [+0.046,+0.084]* | +0.007 [-0.002,+0.017] | 0 | (link-free) | | |
| HEALTH | +0.058 [+0.040,+0.075]* | - | 9.9 | +0.043 [+0.027,+0.058]* | - | 9.9 |
| HYBRID_P | +0.049 [+0.032,+0.067]* | -0.008 [-0.020,+0.004] | 1266 | +0.033 [+0.017,+0.049]* | -0.009 [-0.022,+0.003] | 1304 |
| DEV | +0.049 [+0.032,+0.067]* | -0.009 [-0.020,+0.004] | 158 | +0.010 [-0.005,+0.026] | -0.032 [-0.046,-0.019]* | 248 |
| DEV_TASK | +0.039 [+0.022,+0.056]* | -0.019 [-0.031,-0.005]* | 165 | | | |
| DEV_INFO | +0.046 [+0.029,+0.064]* | -0.012 [-0.024,-0.000]* | 109 | | | |
| DEV_SD | +0.065 [+0.046,+0.084]* | +0.007 [-0.002,+0.017] | 32 | +0.043 [+0.024,+0.061]* | +0.000 [-0.011,+0.011] | 45 |
| DEV_SD_B12 | +0.045 [+0.028,+0.062]* | -0.012 [-0.024,-0.001]* | 18 | +0.032 [+0.016,+0.048]* | -0.011 [-0.023,+0.000] | 18 |

Lost-agent frame counts (episodes that reach the failure time; ranges over R7a-d):

| strategy | frames after death, ideal | frames after death, burst | lag of 1st post-death frame (median steps) | frames before death | partner's frames |
|---|---|---|---|---|---|
| DEV (no self-diagnosis) | 8.5-22.0 | 17-29 | 1-3 | 0.15-0.56 | 1.6-2.0 |
| DEV_INFO | 8.2-10.1 | - | 1-7 | 0.17-0.36 | 0.7-2.2 |
| DEV_SD | 0.98-1.04 | 0.98-1.03 | 0-1 | 0.15-0.56 | 1.6-2.0 |
| DEV_SD_B12 | 0.63-0.79 (vs 0.91-0.95 for DEV_SD: in 13-32% of episodes the single frame was already spent on an earlier hint or abort deviation) | same | 0-1 | 0.14-0.34 | 0.7-0.8 |
| HEALTH (reference) | exactly 1 at t_dead | 1 | 0 | 0 | 0 |

## Compact table C: other faults (builder A = 5x5 t0 H16, seeds 800-811, 100 episodes, ideal link)

d vs IMPLICIT, with bytes per episode in parentheses.

| condition | HEALTH | HB_TO(P3) | DEV | DEV_SD | DEV_SD_B12 |
|---|---|---|---|---|---|
| transient L=4 | -0.028* (9) | -0.019 (81) | -0.009 (55) | -0.011 (41) | -0.023 (17) |
| intermittent | -0.045* (10) | -0.026 (94) | -0.014 (56) | -0.007 (58); +0.038* vs HEALTH | -0.033* (17) |
| TOTAL (radio dies) | 0.000 (0) | +0.040* (43) | -0.013 (21) | -0.013 (21) | +0.003 (11) |
| first post-failure frame lost | 0.000 (9) | +0.037* (79) | +0.025 (98) | -0.007 (30) | +0.003 (17) |
| late loss (t 9-12) | -0.001 (6) | -0.001 (68) | -0.012 (51) | -0.001 (30) | -0.007 (15) |

# Full per-cell tables

## Part A: seeds 200-207, 300 episodes/cell


### 5x5|t0|lam1

| link | strategy | P(detect) | d vs IMPLICIT | d vs ORACLE | bytes/ep |
|---|---|---|---|---|---|
| d0_loss0 | ORACLE | 0.692 | +0.008 [-0.009,+0.027] | - | 0.0 |
| d0_loss0 | IMPLICIT | 0.684 | - | -0.008 [-0.027,+0.009] | 0.0 |
| d0_loss0 | DEV | 0.692 | +0.008 [-0.009,+0.027] | +0.000 [+0.000,+0.000] | 27.2 |
| d1_loss0 | DEV | 0.687 | +0.003 [-0.013,+0.021] | -0.005 [-0.015,+0.005] | 52.7 |

### 5x5|t0|lam3

| link | strategy | P(detect) | d vs IMPLICIT | d vs ORACLE | bytes/ep |
|---|---|---|---|---|---|
| d0_loss0 | ORACLE | 0.703 | +0.010 [-0.009,+0.026] | - | 0.0 |
| d0_loss0 | IMPLICIT | 0.693 | - | -0.010 [-0.026,+0.009] | 0.0 |
| d0_loss0 | DEV | 0.703 | +0.010 [-0.009,+0.026] | +0.000 [+0.000,+0.000] | 77.9 |
| d1_loss0 | DEV | 0.703 | +0.010 [-0.005,+0.024] | +0.000 [-0.009,+0.010] | 146.8 |

### 5x5|t1|lam1

| link | strategy | P(detect) | d vs IMPLICIT | d vs ORACLE | bytes/ep |
|---|---|---|---|---|---|
| d0_loss0 | ORACLE | 0.764 | +0.000 [-0.010,+0.011] | - | 0.0 |
| d0_loss0 | IMPLICIT | 0.764 | - | -0.000 [-0.011,+0.010] | 0.0 |
| d0_loss0 | DEV | 0.764 | +0.000 [-0.010,+0.011] | +0.000 [+0.000,+0.000] | 35.0 |
| d1_loss0 | DEV | 0.756 | -0.007 [-0.018,+0.003] | -0.008 [-0.016,+0.000] | 47.3 |

### 5x5|t1|lam3

| link | strategy | P(detect) | d vs IMPLICIT | d vs ORACLE | bytes/ep |
|---|---|---|---|---|---|
| d0_loss0 | ORACLE | 0.788 | -0.005 [-0.019,+0.009] | - | 0.0 |
| d0_loss0 | IMPLICIT | 0.793 | - | +0.005 [-0.009,+0.019] | 0.0 |
| d0_loss0 | DEV | 0.788 | -0.005 [-0.019,+0.009] | +0.000 [+0.000,+0.000] | 103.3 |
| d1_loss0 | DEV | 0.790 | -0.003 [-0.016,+0.010] | +0.002 [-0.005,+0.010] | 149.2 |

### 6x6|t0|lam1

| link | strategy | P(detect) | d vs IMPLICIT | d vs ORACLE | bytes/ep |
|---|---|---|---|---|---|
| d0_loss0 | ORACLE | 0.579 | +0.000 [-0.021,+0.022] | - | 0.0 |
| d0_loss0 | IMPLICIT | 0.579 | - | +0.000 [-0.022,+0.021] | 0.0 |
| d0_loss0 | DEV | 0.579 | +0.000 [-0.021,+0.022] | +0.000 [+0.000,+0.000] | 30.9 |
| d1_loss0 | DEV | 0.580 | +0.001 [-0.018,+0.022] | +0.001 [-0.010,+0.012] | 59.9 |

### 6x6|t0|lam3

| link | strategy | P(detect) | d vs IMPLICIT | d vs ORACLE | bytes/ep |
|---|---|---|---|---|---|
| d0_loss0 | ORACLE | 0.595 | -0.005 [-0.023,+0.012] | - | 0.0 |
| d0_loss0 | IMPLICIT | 0.600 | - | +0.005 [-0.012,+0.023] | 0.0 |
| d0_loss0 | DEV | 0.595 | -0.005 [-0.023,+0.012] | +0.000 [+0.000,+0.000] | 90.2 |
| d1_loss0 | DEV | 0.598 | -0.003 [-0.017,+0.013] | +0.002 [-0.013,+0.018] | 170.2 |

### 6x6|t1|lam1

| link | strategy | P(detect) | d vs IMPLICIT | d vs ORACLE | bytes/ep |
|---|---|---|---|---|---|
| d0_loss0 | ORACLE | 0.758 | -0.006 [-0.018,+0.006] | - | 0.0 |
| d0_loss0 | IMPLICIT | 0.764 | - | +0.006 [-0.006,+0.018] | 0.0 |
| d0_loss0 | DEV | 0.759 | -0.005 [-0.017,+0.006] | +0.000 [+0.000,+0.002] | 51.7 |
| d1_loss0 | DEV | 0.756 | -0.008 [-0.020,+0.004] | -0.003 [-0.012,+0.008] | 63.5 |

### 6x6|t1|lam3

| link | strategy | P(detect) | d vs IMPLICIT | d vs ORACLE | bytes/ep |
|---|---|---|---|---|---|
| d0_loss0 | ORACLE | 0.788 | -0.003 [-0.015,+0.010] | - | 0.0 |
| d0_loss0 | IMPLICIT | 0.791 | - | +0.003 [-0.010,+0.015] | 0.0 |
| d0_loss0 | DEV | 0.788 | -0.003 [-0.015,+0.010] | +0.000 [+0.000,+0.000] | 149.2 |
| d1_loss0 | DEV | 0.795 | +0.004 [-0.009,+0.017] | +0.006 [-0.006,+0.019] | 197.9 |

### Part A pooled over cells (clusters = cell x instance)

| link | strategy | d vs IMPLICIT | d vs ORACLE | mean bytes/ep |
|---|---|---|---|---|
| d0_loss0 | ORACLE | +0.000 [-0.005,+0.006] | - | 0.0 |
| d0_loss0 | IMPLICIT | - | +0.000 [-0.006,+0.005] | 0.0 |
| d0_loss0 | DEV | +0.000 [-0.005,+0.006] | +0.000 [+0.000,+0.000] | 70.7 |
| d1_loss0 | DEV | -0.000 [-0.006,+0.005] | -0.000 [-0.004,+0.004] | 110.9 |

## Part B: seeds 800-811, 100 episodes/cell


### R7a

| link | strategy | P(detect) | d vs IMPLICIT | d vs HEALTH | d vs HYBRID_P | bytes/ep |
|---|---|---|---|---|---|---|
| d0_loss0 | ORACLE | 0.583 | +0.058 [+0.025,+0.091]* | +0.017 [-0.007,+0.040] | +0.012 [-0.006,+0.030] | 0.0 |
| d0_loss0 | IMPLICIT | 0.525 | - | -0.041 [-0.071,-0.013]* | -0.046 [-0.078,-0.013]* | 0.0 |
| d0_loss0 | HEALTH | 0.566 | +0.041 [+0.013,+0.071]* | - | -0.005 [-0.028,+0.018] | 8.9 |
| d0_loss0 | HYBRID_P | 0.571 | +0.046 [+0.013,+0.078]* | +0.005 [-0.018,+0.028] | - | 935.2 |
| d0_loss0 | DEV | 0.570 | +0.045 [+0.012,+0.077]* | +0.004 [-0.019,+0.028] | -0.001 [-0.003,+0.000] | 95.7 |
| d0_loss0 | DEV_TASK | 0.550 | +0.025 [-0.006,+0.058] | -0.016 [-0.043,+0.013] | -0.021 [-0.044,+0.002] | 96.6 |
| d0_loss0 | DEV_INFO | 0.570 | +0.045 [+0.012,+0.077]* | +0.004 [-0.019,+0.028] | -0.001 [-0.003,+0.000] | 88.4 |
| d0_loss0 | DEV_SD | 0.583 | +0.058 [+0.025,+0.091]* | +0.017 [-0.007,+0.040] | +0.012 [-0.006,+0.030] | 29.6 |
| d0_loss0 | DEV_SD_B12 | 0.561 | +0.036 [+0.003,+0.068]* | -0.005 [-0.033,+0.023] | -0.010 [-0.035,+0.015] | 16.7 |
| d1_burst30 | HEALTH | 0.557 | +0.032 [+0.003,+0.062]* | - | +0.006 [-0.018,+0.028] | 8.9 |
| d1_burst30 | HYBRID_P | 0.551 | +0.026 [-0.004,+0.058] | -0.006 [-0.028,+0.018] | - | 962.9 |
| d1_burst30 | DEV | 0.527 | +0.003 [-0.026,+0.031] | -0.029 [-0.053,-0.004]* | -0.023 [-0.048,+0.002] | 191.1 |
| d1_burst30 | DEV_SD | 0.562 | +0.037 [+0.007,+0.071]* | +0.006 [-0.016,+0.028] | +0.012 [-0.018,+0.038] | 49.7 |
| d1_burst30 | DEV_SD_B12 | 0.549 | +0.024 [-0.005,+0.055] | -0.007 [-0.038,+0.022] | -0.002 [-0.037,+0.030] | 17.0 |

Lost-agent frames (episodes that reached the failure): 
| link | strategy | frames before death | frames after death | P(any post-death frame) | median lag of 1st (steps) | partner frames |
|---|---|---|---|---|---|---|
| d0_loss0 | DEV | 0.35 | 8.53 | 0.87 | 1 | 1.75 |
| d0_loss0 | DEV_TASK | 0.39 | 8.27 | 0.87 | 1 | 2.04 |
| d0_loss0 | DEV_INFO | 0.36 | 8.18 | 0.86 | 1 | 1.27 |
| d0_loss0 | DEV_SD | 0.35 | 1.04 | 0.91 | 0 | 1.79 |
| d0_loss0 | DEV_SD_B12 | 0.31 | 0.63 | 0.63 | 0 | 0.81 |
| d1_burst30 | DEV | 0.73 | 17.08 | 0.88 | 1 | 3.12 |
| d1_burst30 | DEV_SD | 0.73 | 1.03 | 0.91 | 0 | 3.50 |
| d1_burst30 | DEV_SD_B12 | 0.32 | 0.62 | 0.62 | 0 | 0.82 |

### R7b

| link | strategy | P(detect) | d vs IMPLICIT | d vs HEALTH | d vs HYBRID_P | bytes/ep |
|---|---|---|---|---|---|---|
| d0_loss0 | ORACLE | 0.539 | +0.083 [+0.048,+0.120]* | +0.009 [-0.010,+0.029] | +0.020 [-0.003,+0.043] | 0.0 |
| d0_loss0 | IMPLICIT | 0.456 | - | -0.074 [-0.108,-0.040]* | -0.063 [-0.102,-0.027]* | 0.0 |
| d0_loss0 | HEALTH | 0.530 | +0.074 [+0.040,+0.108]* | - | +0.011 [-0.016,+0.037] | 9.9 |
| d0_loss0 | HYBRID_P | 0.519 | +0.063 [+0.027,+0.102]* | -0.011 [-0.037,+0.016] | - | 1191.5 |
| d0_loss0 | DEV | 0.519 | +0.063 [+0.027,+0.102]* | -0.011 [-0.037,+0.016] | +0.000 [+0.000,+0.000] | 156.8 |
| d0_loss0 | DEV_TASK | 0.512 | +0.056 [+0.022,+0.089]* | -0.018 [-0.043,+0.007] | -0.007 [-0.028,+0.015] | 170.7 |
| d0_loss0 | DEV_INFO | 0.517 | +0.062 [+0.026,+0.100]* | -0.013 [-0.037,+0.013] | -0.002 [-0.014,+0.011] | 106.7 |
| d0_loss0 | DEV_SD | 0.539 | +0.083 [+0.048,+0.120]* | +0.009 [-0.010,+0.029] | +0.020 [-0.003,+0.043] | 28.9 |
| d0_loss0 | DEV_SD_B12 | 0.514 | +0.058 [+0.024,+0.092]* | -0.016 [-0.036,+0.005] | -0.005 [-0.037,+0.024] | 17.3 |
| d1_burst30 | HEALTH | 0.515 | +0.059 [+0.029,+0.091]* | - | +0.013 [-0.013,+0.041] | 9.9 |
| d1_burst30 | HYBRID_P | 0.502 | +0.046 [+0.011,+0.081]* | -0.013 [-0.041,+0.013] | - | 1245.9 |
| d1_burst30 | DEV | 0.482 | +0.027 [-0.003,+0.055] | -0.033 [-0.061,-0.006]* | -0.019 [-0.043,+0.005] | 237.2 |
| d1_burst30 | DEV_SD | 0.514 | +0.058 [+0.022,+0.096]* | -0.001 [-0.021,+0.021] | +0.013 [-0.017,+0.044] | 35.3 |
| d1_burst30 | DEV_SD_B12 | 0.500 | +0.044 [+0.013,+0.075]* | -0.015 [-0.034,+0.004] | -0.002 [-0.029,+0.026] | 17.4 |

Lost-agent frames (episodes that reached the failure): 
| link | strategy | frames before death | frames after death | P(any post-death frame) | median lag of 1st (steps) | partner frames |
|---|---|---|---|---|---|---|
| d0_loss0 | DEV | 0.25 | 13.93 | 0.88 | 3 | 1.55 |
| d0_loss0 | DEV_TASK | 0.31 | 14.80 | 0.88 | 3 | 1.98 |
| d0_loss0 | DEV_INFO | 0.17 | 9.07 | 0.83 | 5 | 1.46 |
| d0_loss0 | DEV_SD | 0.25 | 0.98 | 0.92 | 1 | 1.58 |
| d0_loss0 | DEV_SD_B12 | 0.22 | 0.73 | 0.73 | 1 | 0.72 |
| d1_burst30 | DEV | 0.34 | 21.35 | 0.88 | 3 | 2.07 |
| d1_burst30 | DEV_SD | 0.34 | 0.98 | 0.92 | 1 | 2.08 |
| d1_burst30 | DEV_SD_B12 | 0.22 | 0.73 | 0.73 | 1 | 0.71 |

### R7c

| link | strategy | P(detect) | d vs IMPLICIT | d vs HEALTH | d vs HYBRID_P | bytes/ep |
|---|---|---|---|---|---|---|
| d0_loss0 | ORACLE | 0.555 | +0.088 [+0.044,+0.131]* | +0.005 [-0.015,+0.024] | +0.029 [+0.002,+0.054]* | 0.0 |
| d0_loss0 | IMPLICIT | 0.467 | - | -0.083 [-0.123,-0.042]* | -0.059 [-0.098,-0.019]* | 0.0 |
| d0_loss0 | HEALTH | 0.550 | +0.083 [+0.042,+0.123]* | - | +0.024 [-0.003,+0.051] | 10.3 |
| d0_loss0 | HYBRID_P | 0.526 | +0.059 [+0.019,+0.098]* | -0.024 [-0.051,+0.003] | - | 1009.0 |
| d0_loss0 | DEV | 0.526 | +0.059 [+0.019,+0.098]* | -0.024 [-0.051,+0.003] | +0.000 [-0.003,+0.003] | 117.9 |
| d0_loss0 | DEV_TASK | 0.521 | +0.054 [+0.016,+0.092]* | -0.029 [-0.057,+0.000] | -0.005 [-0.025,+0.014] | 115.1 |
| d0_loss0 | DEV_INFO | 0.525 | +0.058 [+0.018,+0.098]* | -0.025 [-0.052,+0.002] | -0.001 [-0.003,+0.000] | 107.8 |
| d0_loss0 | DEV_SD | 0.555 | +0.088 [+0.044,+0.131]* | +0.005 [-0.015,+0.024] | +0.029 [+0.002,+0.054]* | 30.7 |
| d0_loss0 | DEV_SD_B12 | 0.534 | +0.068 [+0.028,+0.108]* | -0.016 [-0.036,+0.003] | +0.008 [-0.024,+0.039] | 17.9 |
| d1_burst30 | HEALTH | 0.518 | +0.052 [+0.022,+0.084]* | - | +0.020 [-0.010,+0.048] | 10.3 |
| d1_burst30 | HYBRID_P | 0.498 | +0.032 [-0.002,+0.066] | -0.020 [-0.048,+0.010] | - | 1029.9 |
| d1_burst30 | DEV | 0.472 | +0.005 [-0.036,+0.046] | -0.047 [-0.078,-0.017]* | -0.027 [-0.058,+0.003] | 229.8 |
| d1_burst30 | DEV_SD | 0.516 | +0.049 [+0.007,+0.098]* | -0.003 [-0.030,+0.026] | +0.018 [-0.020,+0.055] | 50.5 |
| d1_burst30 | DEV_SD_B12 | 0.508 | +0.042 [+0.003,+0.083]* | -0.010 [-0.033,+0.012] | +0.010 [-0.025,+0.044] | 18.1 |

Lost-agent frames (episodes that reached the failure): 
| link | strategy | frames before death | frames after death | P(any post-death frame) | median lag of 1st (steps) | partner frames |
|---|---|---|---|---|---|---|
| d0_loss0 | DEV | 0.15 | 9.57 | 0.88 | 1 | 1.69 |
| d0_loss0 | DEV_TASK | 0.16 | 8.98 | 0.87 | 1 | 1.98 |
| d0_loss0 | DEV_INFO | 0.15 | 9.58 | 0.88 | 1 | 0.71 |
| d0_loss0 | DEV_SD | 0.15 | 1.03 | 0.92 | 0 | 1.77 |
| d0_loss0 | DEV_SD_B12 | 0.14 | 0.79 | 0.79 | 0 | 0.79 |
| d1_burst30 | DEV | 0.29 | 19.08 | 0.88 | 1 | 2.82 |
| d1_burst30 | DEV_SD | 0.29 | 1.03 | 0.91 | 0 | 3.52 |
| d1_burst30 | DEV_SD_B12 | 0.13 | 0.79 | 0.79 | 0 | 0.81 |

### R7d

| link | strategy | P(detect) | d vs IMPLICIT | d vs HEALTH | d vs HYBRID_P | bytes/ep |
|---|---|---|---|---|---|---|
| d0_loss0 | ORACLE | 0.367 | +0.031 [-0.004,+0.066] | -0.001 [-0.015,+0.013] | +0.003 [-0.013,+0.017] | 0.0 |
| d0_loss0 | IMPLICIT | 0.336 | - | -0.032 [-0.063,-0.001]* | -0.028 [-0.059,+0.002] | 0.0 |
| d0_loss0 | HEALTH | 0.367 | +0.032 [+0.001,+0.063]* | - | +0.003 [-0.013,+0.020] | 10.5 |
| d0_loss0 | HYBRID_P | 0.364 | +0.028 [-0.002,+0.059] | -0.003 [-0.020,+0.013] | - | 1927.1 |
| d0_loss0 | DEV | 0.364 | +0.028 [-0.002,+0.059] | -0.003 [-0.020,+0.013] | +0.000 [+0.000,+0.000] | 259.6 |
| d0_loss0 | DEV_TASK | 0.357 | +0.021 [-0.011,+0.052] | -0.011 [-0.029,+0.007] | -0.007 [-0.021,+0.005] | 278.9 |
| d0_loss0 | DEV_INFO | 0.354 | +0.018 [-0.009,+0.047] | -0.013 [-0.031,+0.005] | -0.010 [-0.022,+0.003] | 132.7 |
| d0_loss0 | DEV_SD | 0.367 | +0.031 [-0.004,+0.066] | -0.001 [-0.015,+0.013] | +0.003 [-0.013,+0.017] | 38.4 |
| d0_loss0 | DEV_SD_B12 | 0.356 | +0.020 [-0.008,+0.047] | -0.012 [-0.029,+0.005] | -0.008 [-0.027,+0.012] | 19.9 |
| d1_burst30 | HEALTH | 0.364 | +0.028 [-0.000,+0.057] | - | -0.002 [-0.021,+0.017] | 10.5 |
| d1_burst30 | HYBRID_P | 0.366 | +0.030 [+0.002,+0.058]* | +0.002 [-0.017,+0.021] | - | 1975.4 |
| d1_burst30 | DEV | 0.343 | +0.007 [-0.018,+0.034] | -0.021 [-0.040,-0.001]* | -0.022 [-0.043,-0.001]* | 335.1 |
| d1_burst30 | DEV_SD | 0.362 | +0.027 [-0.004,+0.058] | -0.002 [-0.017,+0.013] | -0.003 [-0.022,+0.016] | 43.5 |
| d1_burst30 | DEV_SD_B12 | 0.352 | +0.017 [-0.009,+0.043] | -0.012 [-0.028,+0.004] | -0.013 [-0.035,+0.009] | 20.0 |

Lost-agent frames (episodes that reached the failure): 
| link | strategy | frames before death | frames after death | P(any post-death frame) | median lag of 1st (steps) | partner frames |
|---|---|---|---|---|---|---|
| d0_loss0 | DEV | 0.56 | 21.98 | 0.93 | 3 | 1.99 |
| d0_loss0 | DEV_TASK | 0.64 | 23.36 | 0.94 | 2 | 2.37 |
| d0_loss0 | DEV_INFO | 0.26 | 10.10 | 0.87 | 7 | 2.19 |
| d0_loss0 | DEV_SD | 0.56 | 0.98 | 0.95 | 1 | 1.98 |
| d0_loss0 | DEV_SD_B12 | 0.34 | 0.63 | 0.63 | 1 | 0.83 |
| d1_burst30 | DEV | 0.63 | 28.72 | 0.94 | 2 | 2.33 |
| d1_burst30 | DEV_SD | 0.63 | 0.99 | 0.95 | 1 | 2.38 |
| d1_burst30 | DEV_SD_B12 | 0.34 | 0.63 | 0.63 | 1 | 0.84 |

### Part B pooled over cells (clusters = cell x instance)

| link | strategy | d vs IMPLICIT | d vs HEALTH | d vs HYBRID_P | mean bytes/ep |
|---|---|---|---|---|---|
| d0_loss0 | ORACLE | +0.065 [+0.046,+0.084]* | +0.007 [-0.002,+0.017] | +0.016 [+0.005,+0.026]* | 0.0 |
| d0_loss0 | IMPLICIT | - | -0.058 [-0.075,-0.040]* | -0.049 [-0.067,-0.032]* | 0.0 |
| d0_loss0 | HEALTH | +0.058 [+0.040,+0.075]* | - | +0.008 [-0.004,+0.020] | 9.9 |
| d0_loss0 | HYBRID_P | +0.049 [+0.032,+0.067]* | -0.008 [-0.020,+0.004] | - | 1265.7 |
| d0_loss0 | DEV | +0.049 [+0.032,+0.067]* | -0.009 [-0.020,+0.004] | -0.000 [-0.001,+0.001] | 157.5 |
| d0_loss0 | DEV_TASK | +0.039 [+0.022,+0.056]* | -0.019 [-0.031,-0.005]* | -0.010 [-0.020,-0.000]* | 165.3 |
| d0_loss0 | DEV_INFO | +0.046 [+0.029,+0.064]* | -0.012 [-0.024,-0.000]* | -0.003 [-0.008,+0.001] | 108.9 |
| d0_loss0 | DEV_SD | +0.065 [+0.046,+0.084]* | +0.007 [-0.002,+0.017] | +0.016 [+0.005,+0.026]* | 31.9 |
| d0_loss0 | DEV_SD_B12 | +0.045 [+0.028,+0.062]* | -0.012 [-0.024,-0.001]* | -0.004 [-0.019,+0.010] | 18.0 |
| d1_burst30 | HEALTH | +0.043 [+0.027,+0.058]* | - | +0.009 [-0.003,+0.022] | 9.9 |
| d1_burst30 | HYBRID_P | +0.033 [+0.017,+0.049]* | -0.009 [-0.022,+0.003] | - | 1303.5 |
| d1_burst30 | DEV | +0.010 [-0.005,+0.026] | -0.032 [-0.046,-0.019]* | -0.023 [-0.036,-0.010]* | 248.3 |
| d1_burst30 | DEV_SD | +0.043 [+0.024,+0.061]* | +0.000 [-0.011,+0.011] | +0.010 [-0.005,+0.025] | 44.8 |
| d1_burst30 | DEV_SD_B12 | +0.032 [+0.016,+0.048]* | -0.011 [-0.023,+0.000] | -0.002 [-0.017,+0.014] | 18.1 |

## Part C: seeds 800-811, 100 episodes/cell


### trans_m

| link | strategy | P(detect) | d vs IMPLICIT | d vs HEALTH | bytes/ep |
|---|---|---|---|---|---|
| d0_loss0 | IMPLICIT | 0.652 | - | +0.028 [+0.003,+0.051]* | 0.0 |
| d0_loss0 | HEALTH | 0.624 | -0.028 [-0.051,-0.003]* | - | 8.9 |
| d0_loss0 | HB_TO | 0.632 | -0.019 [-0.042,+0.002] | +0.008 [-0.003,+0.020] | 80.7 |
| d0_loss0 | DEV | 0.642 | -0.009 [-0.043,+0.021] | +0.018 [-0.008,+0.045] | 54.7 |
| d0_loss0 | DEV_SD | 0.641 | -0.011 [-0.043,+0.017] | +0.017 [-0.008,+0.043] | 40.8 |
| d0_loss0 | DEV_SD_B12 | 0.628 | -0.023 [-0.054,+0.004] | +0.004 [-0.021,+0.030] | 16.6 |

Lost-agent frames (episodes that reached the failure): 
| link | strategy | frames before death | frames after death | P(any post-death frame) | median lag of 1st (steps) | partner frames |
|---|---|---|---|---|---|---|
| d0_loss0 | DEV | 0.35 | 3.97 | 0.87 | 1 | 1.69 |
| d0_loss0 | DEV_SD | 0.35 | 2.40 | 0.91 | 0 | 1.69 |
| d0_loss0 | DEV_SD_B12 | 0.31 | 0.63 | 0.63 | 0 | 0.80 |

### intermit

| link | strategy | P(detect) | d vs IMPLICIT | d vs HEALTH | bytes/ep |
|---|---|---|---|---|---|
| d0_loss0 | IMPLICIT | 0.656 | - | +0.045 [+0.018,+0.073]* | 0.0 |
| d0_loss0 | HEALTH | 0.611 | -0.045 [-0.073,-0.018]* | - | 9.7 |
| d0_loss0 | HB_TO | 0.630 | -0.026 [-0.052,+0.001] | +0.019 [+0.003,+0.036]* | 93.9 |
| d0_loss0 | DEV | 0.642 | -0.014 [-0.048,+0.017] | +0.031 [+0.000,+0.063] | 56.4 |
| d0_loss0 | DEV_SD | 0.649 | -0.007 [-0.045,+0.028] | +0.038 [+0.008,+0.069]* | 58.1 |
| d0_loss0 | DEV_SD_B12 | 0.623 | -0.033 [-0.065,-0.004]* | +0.012 [-0.015,+0.036] | 17.0 |

Lost-agent frames (episodes that reached the failure): 
| link | strategy | frames before death | frames after death | P(any post-death frame) | median lag of 1st (steps) | partner frames |
|---|---|---|---|---|---|---|
| d0_loss0 | DEV | 0.24 | 3.91 | 0.86 | 1 | 1.55 |
| d0_loss0 | DEV_SD | 0.24 | 4.09 | 0.90 | 0 | 1.55 |
| d0_loss0 | DEV_SD_B12 | 0.24 | 0.68 | 0.68 | 0 | 0.76 |

### total

| link | strategy | P(detect) | d vs IMPLICIT | d vs HEALTH | bytes/ep |
|---|---|---|---|---|---|
| d0_loss0 | IMPLICIT | 0.525 | - | +0.000 [+0.000,+0.000] | 0.0 |
| d0_loss0 | HEALTH | 0.525 | +0.000 [+0.000,+0.000] | - | 0.0 |
| d0_loss0 | HB_TO | 0.565 | +0.040 [+0.016,+0.066]* | +0.040 [+0.016,+0.066]* | 43.4 |
| d0_loss0 | DEV | 0.512 | -0.013 [-0.048,+0.019] | -0.013 [-0.048,+0.019] | 20.7 |
| d0_loss0 | DEV_SD | 0.512 | -0.013 [-0.048,+0.019] | -0.013 [-0.048,+0.019] | 20.7 |
| d0_loss0 | DEV_SD_B12 | 0.527 | +0.003 [-0.032,+0.033] | +0.003 [-0.032,+0.033] | 11.1 |

Lost-agent frames (episodes that reached the failure): 
| link | strategy | frames before death | frames after death | P(any post-death frame) | median lag of 1st (steps) | partner frames |
|---|---|---|---|---|---|---|
| d0_loss0 | DEV | 0.35 | 0.00 | 0.00 | nan | 1.82 |
| d0_loss0 | DEV_SD | 0.35 | 0.00 | 0.00 | nan | 1.82 |
| d0_loss0 | DEV_SD_B12 | 0.31 | 0.00 | 0.00 | nan | 0.81 |

### beacon_lost

| link | strategy | P(detect) | d vs IMPLICIT | d vs HEALTH | bytes/ep |
|---|---|---|---|---|---|
| d0_loss0 | IMPLICIT | 0.525 | - | +0.000 [+0.000,+0.000] | 0.0 |
| d0_loss0 | HEALTH | 0.525 | +0.000 [+0.000,+0.000] | - | 8.9 |
| d0_loss0 | HB_TO | 0.562 | +0.037 [+0.008,+0.068]* | +0.037 [+0.008,+0.068]* | 79.1 |
| d0_loss0 | DEV | 0.550 | +0.025 [-0.007,+0.058] | +0.025 [-0.007,+0.058] | 97.9 |
| d0_loss0 | DEV_SD | 0.518 | -0.007 [-0.039,+0.023] | -0.007 [-0.039,+0.023] | 29.8 |
| d0_loss0 | DEV_SD_B12 | 0.527 | +0.003 [-0.032,+0.033] | +0.003 [-0.032,+0.033] | 16.8 |

Lost-agent frames (episodes that reached the failure): 
| link | strategy | frames before death | frames after death | P(any post-death frame) | median lag of 1st (steps) | partner frames |
|---|---|---|---|---|---|---|
| d0_loss0 | DEV | 0.35 | 8.75 | 0.87 | 1 | 1.76 |
| d0_loss0 | DEV_SD | 0.35 | 1.04 | 0.91 | 0 | 1.81 |
| d0_loss0 | DEV_SD_B12 | 0.31 | 0.63 | 0.63 | 0 | 0.81 |

### perm_late

| link | strategy | P(detect) | d vs IMPLICIT | d vs HEALTH | bytes/ep |
|---|---|---|---|---|---|
| d0_loss0 | IMPLICIT | 0.633 | - | +0.001 [-0.016,+0.018] | 0.0 |
| d0_loss0 | HEALTH | 0.632 | -0.001 [-0.018,+0.016] | - | 5.5 |
| d0_loss0 | HB_TO | 0.632 | -0.001 [-0.018,+0.016] | +0.000 [+0.000,+0.000] | 67.6 |
| d0_loss0 | DEV | 0.622 | -0.012 [-0.042,+0.018] | -0.011 [-0.036,+0.013] | 50.5 |
| d0_loss0 | DEV_SD | 0.632 | -0.001 [-0.030,+0.027] | +0.000 [-0.024,+0.024] | 29.9 |
| d0_loss0 | DEV_SD_B12 | 0.626 | -0.007 [-0.036,+0.018] | -0.007 [-0.033,+0.018] | 15.1 |

Lost-agent frames (episodes that reached the failure): 
| link | strategy | frames before death | frames after death | P(any post-death frame) | median lag of 1st (steps) | partner frames |
|---|---|---|---|---|---|---|
| d0_loss0 | DEV | 1.15 | 4.94 | 0.92 | 1 | 2.09 |
| d0_loss0 | DEV_SD | 1.15 | 1.05 | 0.94 | 0 | 2.19 |
| d0_loss0 | DEV_SD_B12 | 0.73 | 0.25 | 0.25 | 0 | 0.93 |
