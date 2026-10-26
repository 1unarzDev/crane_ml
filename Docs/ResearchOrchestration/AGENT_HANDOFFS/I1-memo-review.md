# I1 — Independent critic review of RESEARCH_MEMO.md (draft 2026-10-21)

Reviewer: independent CRANE critic. I did not write the memo. Read-only except this file.
Line numbers (Lnn) refer to `RESEARCH_MEMO.md` as of 2026-10-21 01:20.

**Verdict: MAJOR REVISION.** The memo is careful about labels (nothing is called PHYSICALLY_CALIBRATED, E43 is flagged as downgraded by E49). It has three problems: (1) it misattributes its key piece of "contrary" prior art, (2) its §7 data budget is internally inconsistent by about 9×, and (3) its power calculations are optimistic in ways that change the required sample sizes materially.

---

## A. Ranked required corrections

1. **Kadian RA-L'20 is misattributed (L25, L131; also in PREREGISTRATION L19).** The paper's abstract reports that tuning simulator parameters *raised* SRCC_Succ from 0.18 to 0.844 (9 PointNav models, one scanned lab, LoCoBot). So it is evidence *for* the premise that calibration improves rank predictivity, not against it. It is also the closest prior art for P1. Kadian's SRCC is a Pearson correlation of per-agent metrics, not Spearman, so "SRCC" in L12 is a redefinition. Required: move Kadian to prior art that supports or pre-empts the claim, and narrow the gap (L35) to (i) classical Nav2 configuration pools, (ii) *physically structured transient actuation + delivery timing* as the tuned component, (iii) abstention, and (iv) ≥2 embodiments. Also state that Kadian already showed tuning the sim (sliding, actuation noise) moves SRCC.
2. **The §7 run budget is inconsistent with P2 (L107, L118, L124).** P2 needs about 27 independent (site, session) contexts, and offline replay needs the *full* configurations × repeats table in every context. That is 27 × 15 × 3 ≈ **1,215 held-out runs per domain**, not 135. "Reuses the P1 runs" (L107) holds only if P1 itself covers 27 contexts. L118 admits that more sessions are needed but does not carry the cost into the total. The water design, "≥ 1 held out entirely" (L121), gives one held-out site, which cannot support P1 across sites or P2 with 27 contexts. Both totals need to be recomputed. Alternatively, shrink the configuration pool per context and say what that costs.
3. **The P1 power calculation is optimistic (L105). See §C.** It uses 2(1−r12) instead of Steiger's dependent-correlation variance, applies the Spearman factor 1.06 to the SE instead of the variance, ignores attenuation, ignores clustering of the (configuration, site) units, and assumes a baseline SRCC of 0.5 (Kadian's default was 0.18). A realistic K is about 60–90 rather than 45. It also uses 15 configurations against the literature handoff's P1 requirement of "≥20 configs" (I1-literature-novelty.md:126).
4. **P5's arithmetic tests the wrong threshold (L16 vs L108).** P5 states failure ≤ α = 0.10. The calculation shows an upper bound < 0.15. To show a Clopper–Pearson upper bound < 0.10 at a true rate of 0.05 with 80% power takes about **200 accepted recommendations (≈400 instances at 50% coverage)**, not 60/120. Recommendations drawn from ≤27 contexts are clustered, so the effective n is closer to the number of contexts. The calculation also ignores the second half of P5, risk–coverage dominance over two abstainers.
5. **The delivery-timing half of the hypothesis has no executed evidence, and the memo doesn't say so.** E5 (Unity timing/PWM tests) is **NOT_RUN**. The `latest` default and unstamped commands (L99) are noted, but the threat list never states that timing fidelity is untested in Unity. The only timing-like evidence is fixture identifiability (ID-07, I1-aerial). All timing results would also come from a single host (31 GB, ≤2 concurrent Unity instances per D14). They are not stated in a form that would survive a change of machine, real-time factor or load.
6. **The `reference` result is characterised wrongly (L66–67).** I1-reference-diagnosis.md:38,141 shows the metric is *mean-dominated steady sideslip* (mean 0.0519, SD 0.0124), not noise. On the reverse rows, the fitted decoupling *raised* it (surge− 0.0116 → 0.0534; sway− 0.0152 → 0.0507), consistent with one sign-averaged matrix (`decoupling_negative: null`). What is noise-level is the *pass/fail decision* against 0.05, not the phenomenon. Link it to R17.
7. **Some "established" table rows overstate scope (§3).**
   - L47: "bridge frames fixed" is EditMode parity against the spec only. The runtime plant and the refusal are NOT_RUN (E38), Start() has no runtime test (E40), and the bridge was never run against ArduPilot SITL.
   - L45: the oracle and the port share authorship. A shared oracle defect was found and *mirrored* into the port (E31). The PlayMode bounds are self-set at 1.5× measured (E37). None of E31/E37/E44 has an independent verifier; E29 verified the legacy Sub, not the port.
   - L43: E39 records unjustified constants (geometry threshold 0.1, sign-gap 25). The post-fix suite was re-run but never re-reviewed independently (E33 predates the fixes).
   - L72: "Wheels … get 1.00×" is static/breakaway only. Drive-torque traction was INCONCLUSIVE (E42), the articulation wheels gave 0.95–1.01× (E41), and E36/E41 are single-author.
   - L73: omits that ArticulationBody and the explicit −ω×Iω both *gain energy* (+2.4% at dt 0.005, +10% at dt 0.02; I1-verify-r17-gyro.md:73,89).
   - L63: "κ = n" holds only when the reverse ratio is the same on every thruster. In general κ(n) = n·κ(1) (I1-verify-r17-gyro.md:21–22).
8. **Claims with no evidence ID, or a wrong one.**
   - L76: "The structure carries over to land" has no evidence.
   - L76: the Fisher correlation < −0.99 is for (τ_motor, gyro *delay*) under 0.2–0.9 Hz excitation. The identical traces are for τ vs IMU *low-pass* (I1-aerial.md:123–124); the memo merges the two. These are author-only (E27); E32 did not verify identifiability.
   - L78: "Non-homogeneity and coast-down tests are the discriminators" is a *proposal* from the verifier, never tested.
   - L82: "Hand/expert-tuned Nav2 defaults are included". No expert-tuned baseline exists in Tools/Experiment.
   - L87–88: freeze(), the R5/R12 overlap rules and the 7 abstain reasons need E45/E47/D18 cited. R5 is the promotion rule; only R12 forbids overlap. "R#" also collides with RISK_REGISTER IDs.
   - L99: "truth inventory D20" collides with DECISIONS D20 (gyro). Cite I1-truth-inventory.md:87.
9. **Numbers that don't reproduce.**
   - L83: "10 reps, each p ≈ 2e-5". The repo's `mannwhitney_less` (Tools/Experiment/crane_experiment/stats.py:21, normal approximation with continuity correction) cannot go below 9.1e-5 at 10 vs 10; the exact minimum is 5.4e-6. Either the replicate count or the p-value is wrong.
   - L82–83: competence (E47) is shown only for GP-BO, TPE and CMA-ES. **MF-BO, the P2 comparator, and contextual GP have no benchmark competence test**, which the D21 gate requires.
10. **Predictions changed against the source without saying so (L17, L13).** The handoff's P7 (shift detection) was dropped, and P8 was renamed P7 and strengthened from "≤ small gain" to "≈0". The memo also marks EIDA, APPLV and Huang as VERIFIED, while PREREGISTRATION L22 still lists them as unverified. Reconcile both before freezing. P3's "K ≥ 40 per aggressiveness group" (L109, 80 total) contradicts "≈4× the P1 sample" (≈180).
11. **Missing baselines.** AdaptSim (task-driven adaptation) is cited as contrary evidence (L25) but is not a baseline (L82). The handoff also requires SafeOpt, MetaBO, APPLR and split-conformal abstention (I1-literature-novelty.md:144–155), and the memo neither includes them nor justifies dropping them.

## B. Evidence-ID audit (per row)

| Line | Cited | Verdict |
|---|---|---|
| L5 | E45 | OK |
| L41 | E19, E13 | OK. Does not mention that E14/E18 regressions preceded E19, or that source-mode skips remain |
| L42 | E15, E34 | OK. E15 used a fake IPhysicsBody with no PhysX; E34 fixed the dt dependence. Should say "no PhysX" |
| L43 | E39 | Label OK; scope incomplete (A7) |
| L44 | E43 ↓ E49 | OK. The downgrade is noted |
| L45 | E31, E37, E44 | Label OK; circularity and single-author not noted (A7) |
| L46 | E35 | OK |
| L47 | E38, E40, E50 | Overstated: runtime NOT_RUN (A7) |
| L48 | E45 | Calibration state OK. Omits that **2/8 profiles depend on uncommitted user WIP** (aerial crane-reference-quad; campus TurtleBot3 sourced from uncommitted `CraneCampusRobotModel.cs`), that BlueROV2 has evidenceLabel UNKNOWN, and that **no surface-vessel profile exists** among the 8, although the vessel is one of the two claim embodiments |
| L49, L83 | E47 | Fake worker correctly excluded; p-value and MF-BO gaps (A9) |
| L50 | E3 | OK |
| L51 | E28 | OK |
| L53 | E24, E29 | OK ("prior, not measurement") |
| L57–60 | D11, E39 | OK (5.5% / 15% / 24.7% match Tools/Adversarial/README.md:66–68) |
| L61–65 | E43, E49, D19 | OK apart from the κ caveat. 0.0077 / 0.0063 < 0.087 matches I1-verify-r17-gyro.md:47–48 |
| L66–67 | E17, D10 | Numbers match; the interpretation is wrong (A6) |
| L68–70 | E11, E15, E34, E30 | OK |
| L72 | E41, E42, D17 | Overgeneralised to "wheels" (A7) |
| L73 | E48, D20, E44, E50 | OK apart from the energy-gain omission |
| L75 | E39, E33 | OK |
| L76 | E27/E32 | E32 does not cover this; author-only fixture |
| L77 | D12, E22, E26 | OK |
| L78 | E49 | "discriminators" is untested |

No inflation of the forms checked for: no fixture result is stated as LIVE_SIM, no sim-truth as calibration, and E43 is not cited without E49. The residual inflation is single-author or co-authored-oracle results presented without that qualifier (E27, E31/E37/E44, E36/E41, E39 post-fix).

## C. Power re-derivation

Common: one-sided α = 0.05, power 0.80, z = 1.645 + 0.842 = **2.487** (memo 2.49: OK).

**P1.** The memo assumes ρ_ss = 0.5 → ρ_cal = 0.7, Fisher-z difference 0.318 (OK), r12 = 0.7, and SE = 1.06·√(2(1−r12)/(K−3)). Solving gives K = 44.2 → 45. **The arithmetic is consistent with its own formula.** The assumptions are optimistic:
- *Dependent-correlation variance.* Steiger (1980) for correlations sharing the real variable gives Var(z1−z2) = 2(1−c)/(K−3), with c = 0.571 at r̄ = 0.6, r12 = 0.7. That is 0.858 vs the memo's 0.6. With the Spearman variance factor 1.06 applied to the *variance*: **K ≈ 59**.
- *Baseline SRCC.* At Kadian's default 0.18 → 0.38 (Δz = 0.218): **K ≈ 92**.
- *Attenuation.* At reliability 0.8, the observed correlations shrink by √0.8 (0.447 → 0.626). That gives K ≈ 68 under the memo's formula, or **≈ 84** under Steiger. The memo demands reliability 0.8 but does not inflate K for it.
- *Sensitivity to r12.* At r12 = 0.9, K ≈ 23. The result depends strongly on an r12 that nobody has estimated. A pilot is needed.
- *Clustering.* The 45 units are 15 configurations × 3 sites. Pooling across sites lets between-site difficulty, which a sim of each site also reproduces, inflate SRCC. The effective n is nearer 15 configurations per site, with only 3 site replicates. Compute SRCC within each site and meta-analyse, or use a mixed model.
- *Reliability.* "≈3 repeats from E17-like SD" (L106) is unsupported. E17 is a boat cross-axis calibration metric, not a navigation score.
- The preregistered test is a bootstrap CI, not Fisher-z. State that the Fisher-z figure is only an approximation.

**P2.** Paired d = 0.5 gives n = (2.487/0.5)² + 1.4 ≈ **26 → 27: arithmetic OK**. Where it is optimistic:
- d = 0.5 has no source. At d = 0.3, n ≈ 70.
- Regret is zero-inflated and heavy-tailed, so a paired t approximation overstates power. Use a Wilcoxon or permutation test and simulate.
- Four or more baselines need a multiplicity correction. Bonferroni at α = 0.0125 gives n ≈ 41.
- The "advantage shrinks by B = 20" clause is an interaction, and no power is given for it.
- Replay leakage: if selection samples and the scoring "truth" come from the same repeats in the table, regret is biased low. Hold out a repeat for scoring.
- The cost of 27 contexts in runs (A2).

**P5.** Clopper–Pearson one-sided 95%, true p = 0.05: at n = 60, power for UB < 0.15 is **0.82: OK for the bound the memo chose**. But:
- P5's own α is 0.10. For UB < 0.10: n = 60 gives 0.19, n = 150 gives 0.66, n ≈ 200 gives 0.80.
- If the true rate is 0.07, n = 60 against 0.15 gives only 0.59.
- Clustering by context and the untested dominance claim (A4) are further gaps.

**Power verdict: the arithmetic is correct given the stated inputs, but the inputs are optimistic and partly inconsistent.** P1 needs about 60–90 units with within-site analysis. P2 needs 27–40 contexts, and therefore about 1,200+ runs per domain. P5 needs about 200 accepted recommendations if α = 0.10. Replace the closed-form figures with a simulation-based power analysis from a pilot that estimates r12, run-to-run reliability and the regret distribution.

## D. Threats to validity missing from §6

1. **Simulator is its own truth.** Every "PlayMode via PhysX" check compares PhysX against the team's own oracle (E37, E44, E50). The fixtures recover parameters of plants whose constants were invented or are simulator defaults: adversarial thresholds 0.1 and 25 (E39), the x500 SIMULATOR_DEFAULT (I1-aerial.md:77), fault magnitudes. §6.2 mentions circularity but not invented constants.
2. **Co-authored oracle and port** (E31 mirrored defect), which makes parity partly tautological.
3. **Delivery timing is untested** in Unity (E5 NOT_RUN), and all timing comes from a single machine (D14).
4. **Uncommitted user WIP as provenance.** The campus land-robot profile and the aerial reference quad come from uncommitted files (Tools/Profiles/migrated/terrain__crane-campus-turtlebot3-waffle-prior.json:133). The V4 gyro fix waits on the same WIP (D8). The land embodiment therefore has no reproducible committed plant, and no surface-vessel v2 profile exists.
5. **PhysX gaps beyond friction and gyro.** Drive-torque traction is unmeasured (E42), and the integrators gain energy at dt 0.02. The legacy boat, aerial and ground plants are frozen *with* these omissions (D20), so any "steady-state" baseline built on them inherits the gaps asymmetrically.
6. **Garden of forking paths.** The preregistration is not frozen, but the memo already re-specifies predictions (A10). There is no multiplicity plan across P1–P7 × baselines × 2 domains.
7. **Hypothesis budget vs data budget.** "≤ 5 real trials" (L9) sits beside 20–45 min fit sessions plus hundreds of evaluation runs. Define what counts as a "trial" for each method, and charge every method for the calibration session (D18 equal budget).
8. **Outcome metric undefined.** Which real navigation score is ranked (success, time, SPL, collisions), and what counts as a "failure" for P5, are not specified.

## E. §7 "physical data needed": concreteness and feasibility

**Concrete and good:** the sensor list (sender stamps, RPM/current, PTP offsets, RTK with dual-antenna heading), excitation design tied to D12/D19/E49, injected 0/50/100 ms delays, Allan variance, bollard tests, and `reference` holds ≥ 10 (D10).

**Not feasible or not concrete as written:**
- Run totals are off by about 9× (A2). Water has a single held-out site.
- Neither the physical robot nor the vessel is named. Their availability, cost, safety case and permits (open water, RTK base, ADCP) are not addressed.
- Indoor mocap cannot cover the outdoor slope or dirt sites. Specify RTK as the ground truth there, with its accuracy.
- There is no pilot phase to estimate r12, reliability and regret variance before the main campaign.
- There is no stopping rule or data-quality gate per session (clock offset bound, GNSS fix type).
- Aerial and underwater are correctly excluded (L126).

Sources: [Kadian et al., arXiv:1912.06321](https://www.alphaxiv.org/abs/1912.06321); [Semantic Scholar entry](https://www.semanticscholar.org/paper/Sim2Real-Predictivity:-Does-Evaluation-in-Predict-Kadian-Truong/8f1f19bfddf103e4e3ac74610f84d944496406b3).
