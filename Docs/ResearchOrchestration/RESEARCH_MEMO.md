# CRANE Research Memo (draft rev 2, 2026-10-21)

Revised per AGENT_HANDOFFS/I1-memo-review.md ("Review"). CRANE claims cite E#/D#; literature cites the source (E6, §2); otherwise **ASSUMPTION**. Profile rules are PR# (not RISK_REGISTER R#).

**Bottom line.** Nothing in CRANE is PHYSICALLY_CALIBRATED, and no physical dataset exists (E45). All evidence comes from Python fixtures, Unity EditMode/PlayMode tests, or live simulation. No research prediction has been tested. The CRANE↔Nav2 comparative binding is NOT_RUN until the preregistration is frozen (D18).

## 1. Hypothesis and predictions

**Hypothesis.** With ≤ 5 real trials per method (the trial definition is still to be frozen, §5), *transient* actuator and delivery-timing structure with posterior uncertainty beats steady-state calibration, randomized delay, and multi-fidelity BO on two things:
- (a) sim→real rank correlation of classical Nav2 configurations on held-out real sites;
- (b) calibrated abstention.

It is tested on a wheeled robot and a surface vessel. The predictions below follow the literature handoff (I1-literature-novelty.md §4). Differences from the PREREGISTRATION draft are flagged and must be reconciled before freeze.

- **P1.** Rank correlation (calibrated-transient minus steady-state) rises by ≥ 0.2, with a bootstrap 95% CI excluding 0. The test uses ≥ 20 configurations, analysed within site. The coefficient (Spearman vs Pearson) must be fixed before freeze, because Kadian's SRCC is Pearson.
- **P2.** Simple regret beats MF-BO at B ≤ 5, and the advantage shrinks by B = 20.
- **P3.** A timing-only ablation degrades aggressive configurations more than conservative ones.
- **P4.** A steady-state model with delay/gain randomization does **not** match calibrated-transient rank correlation within 0.05. If it does, the transient claim is falsified.
- **P5.** At ≥ 50% coverage, the failure rate of accepted recommendations is ≤ α = 0.10, and the risk–coverage curve dominates GP-variance and uncalibrated-ensemble abstainers.
- **P6.** The water gain has the same sign as the land gain.
- **P7 (restored; missing from the PREREGISTRATION draft).** Under an induced shift (surface change, payload, +50–100 ms latency), the calibrated model's uncertainty and abstention rate rise *before* failures. Uncalibrated baselines do not show this.
- **P8.** Calibrated sensor-noise fidelity adds at most a small gain beyond actuation and timing. The PREREGISTRATION draft calls this "P7: adds about 0", which is stronger than the source; keep the weaker form.

## 2. Prior art and the novelty gap

**Already shown in the literature** (E6):
- planner-parameter tuning: APPLR ICRA'21, APPL RAS'22, CURE T-RO'25;
- actuator/latency models for transfer: Tan RSS'18, Hwangbo Sci. Robot.'19, Isaac Lab arXiv:2511.04831;
- adaptive DR: SimOpt ICRA'19, BayesSim RSS'19, DROPO RAS'23;
- multi-fidelity/safe/meta BO: Marco ICRA'17, SafeOpt ICML'15, MetaBO ICLR'20;
- USV grey-box identification: Fossen 2021, Morel Ocean Eng.'25.

**Kadian et al., RA-L 2020 supports the premise and pre-empts it** (Review §A1; arXiv:1912.06321). Tuning simulator parameters (sliding, actuation noise) raised sim–real correlation for success from 0.18 to 0.844 across 9 PointNav models in one scanned lab. That correlation is Pearson. So **"calibration improves sim–real ranking" is not an open question.**

**Cautions against fidelity for its own sake:**
- Truong CoRL'22: lower fidelity transferred better.
- AdaptSim CoRL'23: task-driven adaptation beats dynamics matching.
- CURE: transfers causal structure without calibrating the sim.

**What may remain open.** Whether *transient* actuator/delivery-timing structure specifically beats steady-state calibration, randomized delay, and MF-BO for classical Nav2 configuration pools at ≤ 5 real trials, together with calibrated abstention, on land and water. OOF-01 (§4) shows steady-state models are already adequate at steady speed. **This is plausibly a modest increment, and a null result is plausible.**

**Source status** (checked at primary sources 2026-10-21; matches PREREGISTRATION):
- EIDA (arXiv:2610.01219) and APPLV (arXiv:2603.08862): verified preprints, no venue.
- Huang et al., ICRA'24 (arXiv:2311.01457): verified. Page numbers come from a secondary citation.
- Förster thesis (ETH 2015, DOI 10.3929/ethz-b-000214143): exists. Its 1.282e-8 coefficient is **UNVERIFIED** (D15).
- MOSAIC-SV: **UNVERIFIED**, not cited.
- LLM-based Nav2 tuning: no primary source found.

## 3. What CRANE has established (none physical)

| Claim (scope) | Evidence | Label |
|---|---|---|
| crane_hardware/crane_nav ports pass 73 + 75 tests with identity gate and domain guard. This came after regressions (E14, E18), and source mode still skips the launch tests | E19, E13 | FIXTURE_VALIDATED |
| Legacy V1–V3 reproduce in C# (E15: fake body, no PhysX; E34: only the added-mass sign is wrong) | E15, E34 | UNITY_EDITMODE_EXECUTED |
| Adversarial identification suite: 78 pass, 1 xfail. Constants 0.1 and 25 are unjustified, and the post-fix suite has not been independently re-reviewed | E39 (E33 predates fixes) | FIXTURE_VALIDATED (sim-truth) |
| Sign-split decoupling fitter improves R17 in its design regimes. Not adoptable as-is | E43, **downgraded by E49** | FIXTURE_VALIDATED |
| Underwater 6-DOF port matches the oracle (1.5e-13). Oracle and port share authors, and a shared defect was mirrored (E31, fixed E37). PlayMode bounds are self-set at 1.5× measured (E37). No independent verifier of the port | E31, E37, E44 | UNITY_EDITMODE / PLAYMODE_EXECUTED |
| Terrain slope-traction math matches the oracle. Not wired to a robot | E35 | UNITY_EDITMODE_EXECUTED |
| Aerial v2: frame conversion matches the spec in EditMode, and the explicit gyro term reproduces the tumble flip (0.116%). Runtime plant and refusal NOT_RUN; Start() has no runtime test; never run against ArduPilot SITL | E38, E40, E50 | UNITY_EDITMODE / PLAYMODE_EXECUTED |
| Profile schema v2: all 8 profiles are UNCALIBRATED_PRIOR. 2 of the 8 come from uncommitted user WIP (campus TurtleBot3, aerial reference quad). BlueROV2's evidenceLabel is UNKNOWN. **No surface-vessel profile exists**, although the vessel is a claim embodiment | E45; Review §B (L48) | FIXTURE_VALIDATED |
| Baseline competence: GP-BO, TPE and CMA-ES beat random (p at the 9.1e-5 floor of the normal-approximation Mann–Whitney test). **MF-BO and contextual GP have no competence test**, so D21 is unmet for them. Fake-worker results are not CRANE evidence | E47, D21 | FIXTURE_VALIDATED |
| Live calibration runs 2026-10-19: every case failed | E3 | LIVE_SIM (negative) |
| Aerial validation on a stale build, sign-only gate | E28 | LIVE_SIM (weak) |

## 4. Negative and surprising results

- **OOF-01** (D11, E39). With thrust ∝ cmd² and quadratic drag, an affine steady-state fit extrapolates within 5.5% (expected > 15%). Only a through-origin gain fails (24.7%). Consequence: P4 must be tested on transient or aggressive manoeuvres.
- **R17 root cause** (E43, E49, D19). The upstream decoupler applies a large-signal secant inverse as a small-signal correction.
  - For every row, κ(n) = n·κ(1). κ = n holds on surge rows **only when every thruster has the same reverse ratio**; with per-thruster ratios, surge κ was 0.40 and 0.72 at n = 2 (E49).
  - **A post-decoupling residual hides faults.** weak_rr and angled_fr fall below 0.087 after decoupling, so faults must be detected from the identified plant (D19).
  - E49 also found:
    - no abstention under deadband;
    - homogeneity certified beyond the tested range;
    - 19/100 near-threshold fits worse than identity;
    - thruster-fault vs hull-coupling misattribution.
- **`reference`** (E17, D10). The metric is **mean-dominated steady sideslip**: mean 0.0519, SD 0.0124, about 3.9° at 0.77 m/s, partly turn-induced (slope −0.85 s). The *pass/fail decision* against 0.05 is noise-level; the phenomenon is not. On reverse rows the fitted decoupling raised it (surge− 0.0116 → 0.0534). That is consistent with one sign-averaged matrix (`decoupling_negative: null`), i.e. R17. This is a hypothesis, not tested (I1-reference-diagnosis §6).
- **Legacy defects** (E11, E15, E34, E30): V1 added-mass sign; V2 Coriolis; V3 drag ratio 1.0 not 4; V4 campus gyro sign; V5a/c; E30 viscosity units (also the boat on main). Main-branch water runs are not evidence-grade (R14); frozen (D8).
- **PhysX gaps.**
  - *Flat patches* longer than about 18–25 mm get about 2× μ with the default setting (E41, E42).
  - *Rounded contacts* in static/breakaway tests: sphere, capsule and mesh tire 1.00–1.04× (E42); articulation 4-wheel 0.95–1.01× (E41, single author). **Drive-torque traction is INCONCLUSIVE** (E42, R26).
  - *Gyro:* Rigidbody has no ω×Iω term (E48, D20). The ArticulationBody and the explicit −ω×Iω both **gain energy**: +2.4% at dt 0.005 and +10% at dt 0.02 (E48; I1-verify-r17-gyro.md).
- **Unidentifiability** (sim-truth fixtures):
  - *Lag vs stale timestamp:* the logs are identical after stamp alignment, so the split abstains. The abstention is unconditional by construction (ID-07; E39, E33).
  - *Motor τ vs gyro delay:* Fisher correlation < −0.99 under 0.2–0.9 Hz excitation.
  - *τ vs IMU low-pass:* identical traces. An RPM channel separates both pairs. Author-only (E27; E32 did not cover this). **ASSUMPTION** (untested): the same holds on land.
  - *Wheel scale vs slope slip:* fit wheel scale on flat ground only (D12, E22, E26).
  - *Thruster fault vs hull coupling:* hull couplings are unflagged or misattributed (E49). The proposed discriminators (non-homogeneity, coast-down) are **untested**.

## 5. Evaluation design

**Baselines.**
- Present in Tools/Experiment: default, random, Sobol, CMA-ES, TPE, GP-BO, contextual GP, MF-BO (D18). Only GP-BO, TPE and CMA-ES have shown competence (E47).
- Still to build, each needing competence evidence first (D21):
  - SafeOpt;
  - MetaBO;
  - APPLR-style parameter policy;
  - adaptive-DR posterior over a steady-state model with delay randomization;
  - CURE-style pruning;
  - EIDA-style response fit;
  - expert-tuned Nav2 configuration (none exists);
  - abstainers: GP variance, ensemble disagreement, split-conformal.
- AdaptSim is prior art, **not a baseline**. **ASSUMPTION** to review: its idea is covered by the adaptive-DR arm.

**Equal budget** (D18). Charges come from worker-reported cost, and invalid, stale and loophole trials are still charged. Sim and real budgets are reported separately, with each baseline's tuning budget. A "trial" must be defined per method, and **calibration sessions must be charged to every method that uses them** (Review §D7).

**Splits** (D18; PR12 in E45). Data is split by whole site or session, never by later slices of the same session. Held-out contexts stay hidden until `freeze()`.

**Abstention** (D18 design). RECOMMEND only if all of these hold, otherwise ABSTAIN with one of 7 reasons:
- the valid fraction is sufficient;
- constraint risk ≤ α;
- the lower bound of improvement over default > 0;
- no loophole was found;
- calibration state matches the claim.

This is exercised only on the fake worker so far (E47).

**Outcome metric.** Undefined: which navigation score is ranked, and what counts as a P5 "failure". **ASSUMPTION**: success / time-to-goal / collisions, with failure = collision or timeout. Freeze this first.

## 6. Threats to validity

1. **The literature prior** (Kadian) already gives calibration a benefit, so the increment may be small or null. MF-BO may absorb the gap (R7).
2. **The simulator is its own truth.** Fixtures recover invented or default constants (E39). PhysX checks compare against the team's own oracle (E37, E44, E50). Oracle and port are co-authored, and a defect was mirrored (E31).
3. **Delivery timing is untested in Unity** (E5 NOT_RUN). The only timing evidence is fixture identifiability (E39, E27).
   - **All timing results come from a single machine** (31 GB, ≤ 2 Unity instances; D14). Nothing shows they survive a change of host, real-time factor or load.
   - Freshness defaults to `latest`, and unstamped thruster commands hide transport latency (truth-inventory defect TI-D20, I1-truth-inventory.md:87).
4. **Uncommitted WIP as provenance.** The land-robot profile and the V4 fix depend on uncommitted user files (E45, D8). No surface-vessel v2 profile exists.
5. **PhysX gaps inherited asymmetrically.** Drive traction is unmeasured (E42), integrators gain energy (E48), and the frozen legacy plants keep these gaps (D20).
6. **Forking paths.** The preregistration is unfrozen and the predictions have already drifted (§1). Correct for multiplicity across predictions × baselines × domains.
7. **Finite-pool replay** supports pool selection only; selection and scoring use different repeats. The characterization plant is not on main (E16, R13).

## 7. Physical data needed

**Power plan: pilot plus simulation**, replacing the closed-form figures.
- **Pilot per domain:** 1 fit site plus 2 held-out sites, about 10 configurations × 3 repeats per site.
- **The pilot estimates:** r12 (correlation between the two sim rankings), run-to-run reliability of the navigation score, between-site variance, and the regret distribution.
- **Then:** fit a generative mixed model, simulate the bootstrap/permutation tests as preregistered, and pick sizes for 80% power at one-sided α = 0.05, Bonferroni-adjusted across baselines.

The reviewer's corrected ranges are **planning estimates, not evidence** (Review §C):
- **P1: K ≈ 59–92 effective units.** This uses Steiger's dependent-correlation variance, baseline correlations from 0.5 down to Kadian's 0.18, and attenuation at reliability 0.8. It is highly sensitive to r12 (K ≈ 23 at r12 = 0.9).
  - **Site clustering is explicit:** compute the correlation within each site (≥ 20 configurations) and meta-analyse across sites. Pooled (configuration, site) units inflate the correlation through between-site difficulty.
- **P2: about 27 contexts at d = 0.5** (≈ 41 with Bonferroni; ≈ 70 at d = 0.3).
  - Shared-table cost: 27 × 15 configurations × 3 repeats ≈ **1,215 held-out runs per domain**, up to about 1,850 at 41 contexts. One repeat per cell is reserved for scoring.
  - Cheaper alternative: 8 configurations × 3 repeats ≈ 650 runs, at the cost of a narrower regret range, and the pool is then too small for P1's ≥ 20 configurations.
  - The B = 20 shrinkage clause is an interaction and needs its own simulation.
- **P5: about 200 accepted recommendations** for a Clopper–Pearson upper bound ≤ 0.10 at a true rate of 0.05 (n = 60 gives 0.19 power). That is about 400 instances at 50% coverage.
  - Recommendations cluster by context, so effective n approaches the number of contexts; analyse per context. Dominance over the other abstainers needs its own test.
- **P3:** about 4× the P1 sample, so exploratory. **P8:** a bound only.

**Land (wheeled robot; platform not yet named, ASSUMPTION).**
- **Sites:** ≥ 2 fit sites plus enough held-out sites for P1 (expected 3–5, set by the pilot) across surfaces, plus one surveyed slope. ≥ 2 sessions per site on different days. Calibration and evaluation sessions are disjoint.
- **Sensors:** wheel encoders; motor RPM/current; hardware-stamped IMU; LiDAR and depth with driver stamps; sender-stamped `cmd_vel`; mocap indoors, RTK GNSS with dual-antenna heading outdoors (accuracy stated); PTP/chrony with offsets logged.
- **Excitation:** v/ω steps, chirps and multisines at ≥ 2 amplitudes, both signs, into saturation; coast-downs; injected 0/50/100 ms delays; wheel scale on flat ground only (D12); ≥ 2 h static IMU; static LiDAR/depth target captures.
- **Volume:** pilot about 90 runs; main campaign per the P2 table (≈ 1,200–1,850 runs).

**Water (surface vessel; platform, permits and safety case not addressed, ASSUMPTION).**
- **Sites:** the previous draft had **only one held-out site**, which cannot test P1 across sites, P2 or P6. This needs ≥ 3 held-out sites or sessions spanning wind and current, set by the pilot.
- **Sensors:** sender-stamped per-thruster PWM; RPM/current; bollard test per thruster (forward and reverse curves, deadband); RTK GNSS with heading plus AHRS; anemometer; ADCP or drift buoys; synchronized clocks.
- **Excitation:** steps at both signs and ≥ 2 amplitudes with cross-axis probes (D19); 10/10 zig-zags and turning circles; coast-downs and stop tests (E49); deadband sweeps; ≥ 10 `reference` holds (D10).
- **Volume:** as for land.

**Per-session quality gates (ASSUMPTION, to be frozen):** clock-offset bound, RTK fixed solution, logged weather, and a stopping rule for degraded sessions.

## 8. Claims CRANE must not make

- PHYSICALLY_CALIBRATED or "sim-to-real validated" without held-out physical data (E45), including the `calibrated-*` directories (R11).
- "Calibration improves sim–real ranking" as a CRANE finding (Kadian shows it).
- "Higher fidelity ⇒ better transfer" as a general law.
- "First" in any of these: actuator-model simulator, sim-to-real navigation tuning, or Fossen USV simulator (Isaac Lab, APPLR/CURE, VRX).
- LLM/RL Nav2 tuning as novelty.
- That the steady-state model fails at steady speed (OOF-01).
- That a post-decoupling pass means "no fault" (D19).
- Unique lag/timestamp, τ/IMU, scale/slip or fault/hull splits without RPM, coast-down or flat-ground data.
- Any delivery-timing fidelity claim before E5 runs, or from a single host.
- Underwater results as HDRP-validated (D13). Aerial runtime as working (E38, E40).
- Main-branch water or fake-worker results as evidence (R14, E47).
- MF-BO or contextual-GP comparisons before their competence tests (D21).
- Numbers from UNVERIFIED sources: MOSAIC-SV, Förster's 1.282e-8, Peng 2018's timestep range.
