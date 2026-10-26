# PREREGISTRATION (DRAFT — freeze before any comparative run)

**Sharpened question:** With ≤5 real trials, does a calibrated transient actuator + delivery-timing model with posterior uncertainty (a) raise sim→real rank correlation of Nav2 configurations on held-out real sites and (b) support abstention with bounded failure rate? Baselines:
- multi-fidelity BO with an uncalibrated sim
- adaptive domain randomization (DR) over a steady-state model
- CURE
- EIDA-style command→response fitting

Platforms: land and water.

Predictions (full list in AGENT_HANDOFFS/I1-literature-novelty.md):
- P1: Rank correlation improves by ≥0.2, with the confidence interval excluding 0.
- P2: The advantage shows at ≤5 trials and shrinks by 20 trials.
- P3: The timing-model-only ablation hurts the aggressive configurations most.
- P4: A steady-state model with randomized delay must NOT match the calibrated model. If it does, the transient claim is falsified.
- P5: Abstention keeps the accepted-recommendation failure rate within target at ≥50% coverage.
- P6: The water platform's gain has the same sign as the land platform's.
- P7: Sensor-noise fidelity adds about 0.

Held-out rule: held-out means a different site/session, never a later slice of the same run.
Prior against: Truong CoRL'22 and AdaptSim CoRL'23 (fidelity is not necessary or sufficient). Kadian RA-L'20 SUPPORTS the premise: tuning the simulator raised the sim-real correlation from 0.18 to 0.844 (Pearson, not Spearman; corrected 2026-10-21 per AGENT_HANDOFFS/I1-memo-review.md). The CRANE gap must therefore be narrower than 'calibration improves sim-real ranking'. A null result is plausible and will be reported.
Source status (2026-10-21): EIDA and APPLV are verified arXiv preprints (no venue); Huang ICRA'24 is verified (arXiv:2311.01457); MOSAIC-SV is UNVERIFIED (do not cite); the Förster thesis exists but its 1.282e-8 coefficient is UNVERIFIED.
