# NOVELTY_D2: search, verify and report in belief space under correlated repeat-view detections

Literature critic, 2026-11-10.
- **[V]:** primary abstract or record checked.
- **[S]:** snippet only.
- **UNVERIFIED:** not confirmed.

This brief does not repeat what `mission/LITREVIEW.md` (Schedl, Chung, Feldman, Waharte) and `LIT_ROLES.md` (Jin/Polycarpou search–confirm, Nathan 2025, Senthilnath 2024) already cover.

**Verdict: partially novel; H1 is not a contribution.** Three of the plan's points are already published:
- Independence-assuming fusion becomes overconfident under correlation. This is textbook: Koopman 1956, Julier–Uhlmann 1997.
- Planning viewpoints to get decorrelated observations was done by Vélez et al., JAIR 2012.
- A second agent verifying positives was done by Kress et al. 2008 (perfect verifier) and by Jin 2006 (Confirm tasks).

The open part is the explicit **three-way verify choice** under travel, communication and time costs in a two-agent SAR team, with a *measured* forest ρ and a characterised regime.

## Closest prior work

| # | Work (year) | What it already does | What it leaves |
|---|---|---|---|
| 1 | **Vélez, Hemann, Huang, Posner & Roy**, JAIR 44, 2012 ([arXiv 1401.4612](https://arxiv.org/abs/1401.4612)) [V] | Learns a detector model with *spatial correlation between measurements*: successive views from the same or nearby poses are not independent. Plans paths that maximise expected detection performance. Simulation plus real robot. | Single robot. Indoor doors and text. No partner, no report and no false-report constraint. **The single biggest threat.** |
| 2 | **Atanasov et al.**, T-RO 30(5), 2014 ([DOI 10.1109/TRO.2014.2320795](https://doi.org/10.1109/TRO.2014.2320795)) [V] | Active M-ary hypothesis testing with viewpoint control, solved as a point-based POMDP. | Single sensor. Observations are conditionally independent given the viewpoint (**UNVERIFIED** detail). |
| 3 | **Naghshvar & Javidi**, Ann. Statist. 41(6), 2013 ([arXiv 1203.4626](https://arxiv.org/abs/1203.4626)) [V] | Active sequential hypothesis testing: choose the most informative sensing action, then stop and declare. Gives asymptotic optimality and error exponents. | Assumes i.i.d. observations given the action. Correlation across repeated actions is outside the model. |
| 4 | **Kress, Lin & Szechtman**, Math. Meth. OR 68, 2008 ([Springer](https://link.springer.com/article/10.1007/s00186-007-0197-2)) [V] | Discrete search with false positives. A *second, perfect* sensor investigates positives, and a greedy index policy is optimal. | The verifier is perfect and has no travel cost. No correlation. |
| 5 | **Sandino et al.**, Remote Sensing 13, 2021 ([DOI 10.3390/rs13214481](https://doi.org/10.3390/rs13214481)) [S] | A bushland lost-person UAV task posed as a POMDP. On detection, the UAV inspects to confirm or reject. False positives from trees and grass are rejected. 92.5–100% accuracy. | Single UAV. Inspection observations are treated as independent (**UNVERIFIED**). |
| 6 | **"Semantic Segmentation as the Detector for Search Problems"**, Research Square preprint, 2026 ([rs-10382705](https://www.researchsquare.com/article/rs-10382705/v1)) [S; the page body did not load] | Per the snippet: a *correlated* Bayesian belief update for repeated multi-altitude looks, using a generalized binomial distribution, validated on the MESSI UAV dataset. | Authors and planning content are **UNVERIFIED**. If it plans actions, it is a direct competitor. **Read it before writing.** |
| 7 | **Julier & Uhlmann**, ACC 1997 ([DOI 10.1109/ACC.1997.609105](https://doi.org/10.1109/ACC.1997.609105)) [V] | Covariance intersection gives consistent fusion under *unknown* correlation. It is the standard cheap fix for double counting. | A fusion rule, not a decision. |
| 8 | **SPRT under dependence**: "SPRT and CUSUM in hidden Markov models", Ann. Statist. 31(3), 2003 ([Project Euclid](https://projecteuclid.org/journals/annals-of-statistics/volume-31/issue-3/SPRT-and-CUSUM-in-hidden-Markov-models/10.1214/aos/1056562468.pdf)) [S; the author, Fuh, is **UNVERIFIED**]; an AR(1)-modified SPRT for leak detection, Comput. Chem. Eng. 2010 ([ScienceDirect](https://www.sciencedirect.com/science/article/abs/pii/S0098135410002048)) [S] | Error approximations for SPRT on Markov-dependent data. The correlation-corrected SPRT cuts false alarms compared with the i.i.d. SPRT. | **H1 is already shown for SPRT in another domain.** |
| 9 | **Schedl et al.**, Sci. Robot. 2021; **Nathan et al.**, 2025 (from LITREVIEW/LIT_ROLES) | Multi-view integration under canopy, with adaptive re-check of uncertain detections. | No explicit correlation model and no false-report constraint. |

## What's incremental

- **H1 ("independence over-reports")** follows mechanically from equicorrelation. With k looks at pairwise ρ:
  - The effective sample size is n_eff = k / (1 + (k−1)ρ).
  - An independence-assuming log-likelihood ratio is inflated by 1 + (k−1)ρ, which is **2.2× at k = 3, ρ = 0.6**.
  - A reviewer will call ≥ 2× over-reporting a calibration identity, not a finding. Report it as a measured confirmation, not a result.
- A "correlation-aware threshold" is effectively covariance intersection or a deflated-likelihood SPRT, both known.
- "Choose an independent viewpoint" is the core idea of Vélez 2012.
- POMCP or DESPOT on a SAR POMDP has been done (Sandino 2021; Shrinking POMCP, arXiv [2411.12967](https://arxiv.org/abs/2411.12967) [S]).

## Strongest defensible claim (rewritten)

> For a two-drone team under a false-report constraint P(FR) ≤ α, the policy that decides *between* a repeat look, an independent self-view, and a partner view (paying travel and communication costs) has a simple structure determined by ρ, the false-positive rate f and the relative travel cost.
>
> We characterise that structure exactly on small instances (MissionExact) and show it governs time-to-confirmed-report only in the verification-bound regime. Every single-agent rule that is merely calibrated (CI, deflated SPRT) loses there by a measurable margin.

The candidate structure (to prove or refute):
- A repeat look is worth at most 1/ρ − 1 ≈ 0.67 extra independent equivalents in total at ρ = 0.6.
- So a threshold in (ρ, travel cost) separates "repeat" from "move or call the partner".

The contribution is: the decision structure, the regime map, and the measured forest ρ. "Correlation matters" is not the contribution.

## Baselines a reviewer will demand

- Fixed-k confirmation.
- Independent-view Bayes threshold.
- Wald SPRT.
- An **effective-sample-size-deflated SPRT and a covariance-intersection threshold**. These are the cheap fixes, and must be matched at false-report rate.
- A Vélez-style correlated-observation path planner, single agent.
- The Kress greedy index, with the partner as an imperfect verifier.
- A Jin-style nearest-free Confirm assignment.
- A Sandino-style POMDP inspect.
- POMCP.
- A coverage-optimal sweep.

## Contradicting evidence

- RoleSpec null result: the coverage-bound regime shows no gain.
- Nathan 2025: role-free swarms reach 95.9% recall under canopy.
- If ρ ≈ 0.6 is estimated on latent *visibility* rather than on *detector output*, the detector-level correlation could be lower. The 16 vision reruns must estimate ρ on detector scores. The ρ CI half-width from 16 sequences will be wide (expect ±0.15 or more; UNVERIFIED until measured).

## Falsifiable predictions

- **P1.** At matched time-to-report and ρ = 0.6, the independence-assuming false-report inflation equals 1 + (k̄−1)ρ within 20%, where k̄ is the mean number of looks before reporting. A large deviation means the copula model is wrong.
- **P2.** At matched false reports, the deflated SPRT recovers ≥ 70% of the full POMDP policy's gain in time to confirmed report. If it recovers ≥ 90%, the viewpoint-choice claim collapses into a calibration note.
- **P3.** The fraction of independent and partner verifications chosen by the exact policy rises monotonically with ρ. It is ≈ 0 at ρ = 0 and dominant at ρ ≥ 0.6.
- **P4.** In the coverage-bound regime (low target density, f ≤ 2%), all policies tie within CI, consistent with RoleSpec.

## Kill condition

Kill the direction if, at the measured ρ (DEV), either condition holds:
- false-report inflation is < 1.3× (as in the plan), **or**
- the deflated SPRT / covariance-intersection baseline is within 5% of the best policy's time to confirmed report at matched false reports.

In either case, fold it into RoleSpec as a calibration note: "deflate by n_eff".
