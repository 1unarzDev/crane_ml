# Agent-loss baselines: INDEP, HEALTH beacon, HEARTBEAT (reviewer request)

Code: ../loss_baselines.py (new; subclass LEpisode of falsify.FEpisode, no existing file edited). Tests: ../test_loss_baselines.py.
Regimes R7a-d exactly as falsify.py (same builders, fault params, fault seed 10000+ep, base event rates). Seeds 504-511 x 200 episodes
(falsify.py confirm stage). Links: d0_loss0 (ideal) for all strategies; d1_burst30 (30% bursty loss, mean burst 10, delay 1) for the
sending strategies (HYBRID_P, HEALTH, HEARTBEAT); ORACLE/IMPLICIT/INDEP send nothing so their numbers are link-independent.
Paired by episode index; 95% cluster bootstrap (instances, then episodes), B=2000, falsify.boot. The ORACLE/IMPLICIT/INDEP/HYBRID_P
ideal-link rows reproduce falsify.md's confirm table exactly.

Strategies
- HEALTH: IMPLICIT + one frame (HEADER_B = 12 B, no payload records) sent when the agent's sensor dies, carrying the death stamp. On
  receipt the partner drops the dead agent's nominal tasks that start at/after the stamp (not counted as done, not predicted in flight).
- HEARTBEAT: HEALTH + a 1-bit status frame from each agent every 5 steps (dead status carries the stamp). One 12 B frame each time,
  so it pays for itself only when the link drops beacons.
- Assumptions: instant self-diagnosis of sensor death; radio stays up (same as falsify.py's loss model; a full radio+sensor loss is NOT
  tested); the stamp is free inside the frame. Bytes/episode averages over all episodes, including ones detected before the death
  (hence ~9-10 B, not 12 B).
- delivered/sent: for HYBRID_P the ratio is meaningless (msgs_delivered counts per send, msgs_sent per frame); ignore it there.

## Verdicts

(a) INDEP vs "communication detects failure": the reviewer is right as far as these data go. In R7c INDEP gains +0.051 [+0.015,+0.088] vs
IMPLICIT, and HYBRID_P - INDEP = +0.004 [-0.023,+0.032]. In all four regimes on the ideal link both HYBRID_P - INDEP and HEALTH - INDEP
have CIs that include 0 (best case R7b HEALTH +0.032 [-0.001,+0.064]). So the data cannot separate "communication detects failure" from
"stop relying on a dead partner". Caveat in the other direction: INDEP is not a usable policy elsewhere (R0 in falsify.md: INDEP - IMPLICIT = -0.096
without loss); the R7 gain over IMPLICIT is only the removal of a harmful assumption, and the loss regimes alone do not show that
messaging adds anything beyond that.

(b) HEALTH isolates the value of one failure message: it recovers essentially all of HYBRID_P's R7 gain at about 1% of the bytes
(ideal link, d vs IMPLICIT: R7a +0.043 vs +0.042, R7b +0.031 vs +0.028, R7c +0.066 vs +0.054, R7d +0.030 vs +0.029; ~10 B vs 1.0-2.0 kB per
episode) and all four HEALTH CIs exclude 0. Point estimates are within noise of ORACLE (R7c +0.066 vs +0.074). Hence the R7 counterexample
is a failure-notification effect: one frame is enough, and the full-record HYBRID_P protocol is not needed for it. On the lossy link a
single beacon is fragile (HEALTH CIs include 0 in R7a, R7b: +0.022, +0.014; delivered 886/1214 etc.); HEARTBEAT retransmission restores
+0.026..+0.049 (3 of 4 CIs exclude 0, R7b lower bound +0.000) for 50-160 B. HYBRID_P on the burst link is weaker than HEALTH in every regime but R7d
(differences within noise).

## Compact table (d vs IMPLICIT, ideal link d0_loss0; bytes/episode)

| regime | ORACLE | INDEP | HYBRID_P (bytes) | HEALTH (bytes) | HEARTBEAT (bytes) |
|---|---|---|---|---|---|
| R7a | +0.050 [+0.016,+0.083] | +0.020 [-0.014,+0.054] | +0.042 [+0.006,+0.077] (988) | +0.043 [+0.011,+0.073] (9.1) | +0.043 [+0.011,+0.073] (50) |
| R7b | +0.039 [+0.013,+0.066] | -0.001 [-0.033,+0.033] | +0.028 [+0.001,+0.054] (1171) | +0.031 [+0.007,+0.055] (9.5) | +0.031 [+0.007,+0.055] (101) |
| R7c | +0.074 [+0.032,+0.111] | +0.051 [+0.015,+0.088] | +0.054 [+0.019,+0.089] (1047) | +0.066 [+0.034,+0.098] (10.6) | +0.066 [+0.034,+0.098] (55) |
| R7d | +0.031 [+0.003,+0.061] | +0.023 [-0.004,+0.055] | +0.029 [+0.001,+0.059] (1966) | +0.030 [+0.005,+0.058] (10.3) | +0.030 [+0.005,+0.058] (158) |

d1_burst30, d vs IMPLICIT: HYBRID_P / HEALTH / HEARTBEAT = R7a +0.019 / +0.022 / +0.033; R7b +0.013 / +0.014 / +0.026; R7c +0.022 / +0.030 / +0.049;
R7d +0.026 / +0.024 / +0.032 (CIs in the raw output below).

## Tests (../test_loss_baselines.py)
- HEALTH and HEARTBEAT equal IMPLICIT episode-for-episode (detected, ttd, looks, redundant, aborts, hints) when no loss occurs, on all 4 links, R0 and R4a;
  HEALTH sends 0 bytes then.
- Beacon costs 0 or exactly one frame under loss.
- Non-vacuity: HEALTH differs from IMPLICIT under loss (R7c).
- MUST-FAIL (strict xfail): "HEALTH equals IMPLICIT under loss" is asserted and expected to fail; if the beacon ever became inert this xfail would flip to an error.
- Inertness control: on a link that drops everything, HEALTH equals IMPLICIT even under loss.

## Raw output

seeds [504, 505, 506, 507, 508, 509, 510, 511] episodes 200 (paired by episode index; B=2000 cluster bootstrap)


### R7a: agent loss t0 H16


link d0_loss0

| strategy | P(detect) | d vs IMPLICIT | d vs INDEP | bytes/episode | delivered/sent |
|---|---|---|---|---|---|
| ORACLE | 0.551 | +0.050 [+0.016,+0.083] | +0.030 [+0.004,+0.054] | 0.0 | - |
| IMPLICIT | 0.501 | - | -0.020 [-0.054,+0.014] | 0.0 | - |
| INDEP | 0.521 | +0.020 [-0.014,+0.054] | - | 0.0 | - |
| HYBRID_P | 0.542 | +0.042 [+0.006,+0.077] | +0.022 [-0.004,+0.046] | 987.6 | 65412/35242 |
| HEALTH | 0.543 | +0.043 [+0.011,+0.073] | +0.022 [-0.004,+0.048] | 9.1 | 1214/1214 |
| HEARTBEAT | 0.543 | +0.043 [+0.011,+0.073] | +0.022 [-0.004,+0.048] | 50.1 | 6678/6678 |

link d1_burst30

| strategy | P(detect) | d vs IMPLICIT | d vs INDEP | bytes/episode | delivered/sent |
|---|---|---|---|---|---|
| HYBRID_P | 0.519 | +0.019 [-0.011,+0.051] | -0.001 [-0.028,+0.028] | 999.7 | 46616/35624 |
| HEALTH | 0.522 | +0.022 [-0.002,+0.046] | +0.002 [-0.028,+0.033] | 9.1 | 886/1214 |
| HEARTBEAT | 0.533 | +0.033 [+0.004,+0.060] | +0.013 [-0.016,+0.043] | 50.3 | 4738/6708 |

### R7b: agent loss t1 H30


link d0_loss0

| strategy | P(detect) | d vs IMPLICIT | d vs INDEP | bytes/episode | delivered/sent |
|---|---|---|---|---|---|
| ORACLE | 0.516 | +0.039 [+0.013,+0.066] | +0.040 [+0.010,+0.068] | 0.0 | - |
| IMPLICIT | 0.477 | - | +0.001 [-0.033,+0.033] | 0.0 | - |
| INDEP | 0.476 | -0.001 [-0.033,+0.033] | - | 0.0 | - |
| HYBRID_P | 0.505 | +0.028 [+0.001,+0.054] | +0.029 [-0.005,+0.061] | 1171.3 | 77082/40806 |
| HEALTH | 0.508 | +0.031 [+0.007,+0.055] | +0.032 [-0.001,+0.064] | 9.5 | 1270/1270 |
| HEARTBEAT | 0.508 | +0.031 [+0.007,+0.055] | +0.032 [-0.001,+0.064] | 101.0 | 13472/13472 |

link d1_burst30

| strategy | P(detect) | d vs IMPLICIT | d vs INDEP | bytes/episode | delivered/sent |
|---|---|---|---|---|---|
| HYBRID_P | 0.489 | +0.013 [-0.013,+0.037] | +0.013 [-0.024,+0.049] | 1244.7 | 56856/43083 |
| HEALTH | 0.491 | +0.014 [-0.007,+0.036] | +0.015 [-0.019,+0.048] | 9.5 | 869/1270 |
| HEARTBEAT | 0.502 | +0.026 [+0.000,+0.051] | +0.026 [-0.008,+0.060] | 101.4 | 9385/13526 |

### R7c: agent 0 loss early (t in 1..4) t0 H16


link d0_loss0

| strategy | P(detect) | d vs IMPLICIT | d vs INDEP | bytes/episode | delivered/sent |
|---|---|---|---|---|---|
| ORACLE | 0.527 | +0.074 [+0.032,+0.111] | +0.023 [-0.010,+0.052] | 0.0 | - |
| IMPLICIT | 0.453 | - | -0.051 [-0.088,-0.015] | 0.0 | - |
| INDEP | 0.504 | +0.051 [+0.015,+0.088] | - | 0.0 | - |
| HYBRID_P | 0.507 | +0.054 [+0.019,+0.089] | +0.004 [-0.023,+0.032] | 1046.6 | 68820/37126 |
| HEALTH | 0.519 | +0.066 [+0.034,+0.098] | +0.016 [-0.013,+0.044] | 10.6 | 1410/1410 |
| HEARTBEAT | 0.519 | +0.066 [+0.034,+0.098] | +0.016 [-0.013,+0.044] | 55.4 | 7390/7390 |

link d1_burst30

| strategy | P(detect) | d vs IMPLICIT | d vs INDEP | bytes/episode | delivered/sent |
|---|---|---|---|---|---|
| HYBRID_P | 0.476 | +0.022 [-0.009,+0.053] | -0.028 [-0.067,+0.011] | 1065.6 | 49040/37719 |
| HEALTH | 0.483 | +0.030 [+0.007,+0.053] | -0.021 [-0.057,+0.016] | 10.6 | 946/1410 |
| HEARTBEAT | 0.502 | +0.049 [+0.020,+0.078] | -0.001 [-0.036,+0.032] | 55.9 | 5201/7456 |

### R7d: agent loss 8x8 t1 H40


link d0_loss0

| strategy | P(detect) | d vs IMPLICIT | d vs INDEP | bytes/episode | delivered/sent |
|---|---|---|---|---|---|
| ORACLE | 0.338 | +0.031 [+0.003,+0.061] | +0.008 [-0.012,+0.031] | 0.0 | - |
| IMPLICIT | 0.306 | - | -0.023 [-0.055,+0.004] | 0.0 | - |
| INDEP | 0.329 | +0.023 [-0.004,+0.055] | - | 0.0 | - |
| HYBRID_P | 0.336 | +0.029 [+0.001,+0.059] | +0.006 [-0.017,+0.031] | 1966.1 | 126129/65489 |
| HEALTH | 0.336 | +0.030 [+0.005,+0.058] | +0.007 [-0.015,+0.028] | 10.3 | 1374/1374 |
| HEARTBEAT | 0.336 | +0.030 [+0.005,+0.058] | +0.007 [-0.015,+0.028] | 158.4 | 21125/21125 |

link d1_burst30

| strategy | P(detect) | d vs IMPLICIT | d vs INDEP | bytes/episode | delivered/sent |
|---|---|---|---|---|---|
| HYBRID_P | 0.333 | +0.026 [+0.004,+0.052] | +0.003 [-0.019,+0.025] | 2026.4 | 92131/67361 |
| HEALTH | 0.331 | +0.024 [+0.007,+0.043] | +0.001 [-0.022,+0.022] | 10.3 | 984/1374 |
| HEARTBEAT | 0.338 | +0.032 [+0.011,+0.058] | +0.009 [-0.014,+0.029] | 158.4 | 15008/21119 |
