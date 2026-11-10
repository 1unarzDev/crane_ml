# PREREG_BELIEF: search, verify and report in belief space under correlated repeat-view detections (DRAFT)

**Status: DRAFT. Not frozen.** No held-out run before the orchestrator freezes this file and records its sha256 in
`Docs/DroneVisuals/frontier/SEEDS.md` (the `Tools/Frontier/cache.py` guard refuses `split="heldout"` otherwise). Every
number in `Tools/BeliefSVR/results/` is DEV-only (MissionSim dev layouts 0-1999: tuning 0-23, sweeps 100-219) and exploratory.

Simulators: `exact.py` (exact expectimax, deterministic) and `Tools/BeliefSVR/bs_sim.py` (belief-level mission simulator on
MissionSim layouts, evidence label IMPLEMENTED; nothing LIVE_SIM_VALIDATED or PHYSICALLY_CALIBRATED). The detector anchor is the
NOMAD-calibrated `Tools/Vision/obs_model.py`; the repeat-look ICC, the cross-view independence, the decoy fire rate per view and
the background alert rate are PRIOR values until the Vision rendered rerun measures them.

## 1. Claim (rewritten after `frontier/NOVELTY_D2.md`)

NOT claimed: that independence-assuming fusion is overconfident (Koopman 1956; Julier-Uhlmann 1997), that decorrelated
viewpoints help (Velez et al., JAIR 2012), or that a second agent can verify (Kress et al. 2008; Jin 2006).
Claimed: the structure of the three-way verification choice (repeat look / own independent view / partner view) under travel and
communication cost and a false-report cost, characterised EXACTLY on small instances (minimal examples, thresholds in rho, view
change cost s and partner delay d) and tested at scale for where that structure changes time to confirmed report.

## 2. Hypotheses (each can kill the claim; thresholds are set from DEV, see section 8)

- **H0 (sanity, reported, not tested as a finding).** Reports made at nominal posterior p by the independence belief have realised
  precision below p; the correlation-aware belief is calibrated (reliability slope within 0.15 of 1 on 5 bins).
- **H1 (decision structure, exact).** On the pre-declared instance family, the share of non-first same-cell looks that are
  independent-view rises monotonically in rho and falls in s (P3), and the optimal partner-vs-solo switch delay d*(s, pP) exists
  within the horizon for s = 0.
- **H2 (scale, per regime).** At the measured rho and measured f, in the VERIFICATION-HEAVY regime the correlation-aware policy
  with view/partner choice has J = T_cens + 300 s x FR lower than the best of {fixed-k, IND_BAYES, deflated SPRT, CI threshold}
  by at least 10 % (threshold from DEV effect size and cluster count, section 8); in the COVERAGE-bound regime all arms tie within
  +-5 % (RoleSpec consistency, P4).
- **H3 (regime).** The gain of H2 is larger in the verification-heavy regime than in the coverage-bound regime
  (difference in gain, cluster bootstrap CI excludes 0).
- **Kill conditions (either folds the work into a calibration note).** (K1) FR inflation of the independence belief at matched
  time-to-report < 1.3 at the measured rho (DEV: see DEV.md section 6). (K2) The ESS-deflated SPRT or covariance-intersection
  threshold is within 5 % of the best policy's J at matched false reports.

## 3. Arms (one simulator, one budget, common random numbers across arms)

COVER (coverage sweep, report first alert), FIXK (k = 1..4), SPRT, SPRT_DEFL (ESS-deflated), SPRT_DEFL_ALT (deflated, alternating
views), CI_THR (covariance-intersection style), IND_BAYES (rb = 0, theta grid incl. 0.95-0.999), CORR (rb = rho, view class by
KL information per second), CORR_SAMEVIEW, CORR_PARTNER, POMCP-lite (reported separately), ORACLE (labelled). Thresholds
(theta in {0.2,0.35,0.5,0.7,0.9} (+0.95,0.99,0.999 for IND_BAYES), SPRT A in {1,2,3,4.5,6}, k in {1..4}) are fixed grids
reported as frontiers; the headline compares the best grid point per family at lambda = 300 s per false report (and 100, 900).
Not implemented, stated as limitations: Velez-style correlated path planner, Kress greedy index, Jin Confirm assignment,
Sandino POMDP-inspect, a UCT tree POMCP, learned policies (designed only, DEV.md section 9).

## 4. Conditions and seeds

- HELD-OUT layouts **92000-92031** (32 layouts, in the global held-out block 90000-99999; disjoint from RoleSpec 91000-91019,
  NavPilot E2 90200-90207 and E3 90400-90407), R = 4 replicates per layout (rep re-draws latents, decoys and look noises).
- rho in {0, 0.3, 0.6, 0.8} plus the Vision-measured rho (and its CI endpoints) once the rendered rerun exists;
  f in {low, measured, high}; regimes {coverage-bound, verification-heavy}; partner delay d0 in {10, 25, 60, 120} s on the measured cell.
- Exact: the instance families of `bs_exact_experiments.py` (deterministic, no seeds).

## 5. Statistics

Unit = layout; missions paired across arms by (layout, rep); 95 % cluster bootstrap over layouts (2000 resamples) for all
differences; best grid point per family is re-selected inside every resample (selection optimism applies equally to all families).
Holm across the primary family {H2 (verification-heavy, measured), H2 (coverage-bound, measured), H3, K1, K2}; everything else
descriptive. Censoring: unreported missions contribute D to T_cens; success rate is reported separately.

## 6. Metrics

Time to confirmed (correct) report T_cens (misses = D), false reports per mission and P(>= 1), misses, verification actions per
true report, verification time share, miss-cause labels (route, visibility, inference, association, communication),
report-posterior reliability, CPU per mission.

## 7. Known limitations

Binary alerts only (the detector confidence score is not used by the belief); cross-view independence assumed; one target;
reports are instantaneous (no comms loss); the correlation structure is a Gaussian copula calibrated to the Vision ICC at one
reference marginal; decoy per-view fire probability = Vision persistent fire (0.8) x MissionSim per-view visibility (0.35).

## 8. DEV observations used to finalise this draft (exploratory, not evidence)

See `results/DEV.md`. Numbers for thresholds in H2 and the cluster count are filled from there before freezing.
