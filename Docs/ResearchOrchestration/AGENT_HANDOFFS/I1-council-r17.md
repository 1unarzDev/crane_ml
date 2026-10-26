# I1 council: R17 (the fitted decoupling makes rows worse), systems critic

Date: 2026-10-21. Evidence label: **FIXTURE_VALIDATED** (adv_plant 3-DOF numerical fixture only; no Unity, no ROS, no physical data).

Rules followed:
- MHSeals was read only, through `git show`: mhseals_testing@a645657 `mhseals_testing/analysis/characterization.py`, and mhseals_hardware@26c59f7 `velocity_law.py` / `thruster_mixer.py` (worktree HEAD).
- Nothing was imported from or copied out of MHSeals. The baseline was re-derived from the documented model.
- New files exist only under `Tools/Adversarial/decoupling/`. `matrix.json`, `adv_*.py` and the other tests are untouched; `adv_plant`/`adv_faults` are imported only.
- Nothing was committed or pushed.

Files:
- `Tools/Adversarial/decoupling/conftest.py`: sys.path for `python -I`.
- `dec_sim.py`: plant regimes (subclasses of `adv_plant.VesselModel` / `adv_faults.FaultedVessel`), the X effort mixer (asserted against `adv_plant.column`), settled holds, verification rows. The verification metric is the mhseals probe `effort_acceptance` metric.
- `dec_upstream.py`: the upstream-equivalent baseline. It fits single-axis + neutral samples to `v = b + G e (+ drift)` with ridge 1e-8, then `C = G_raw^-1 diag(G_raw)`, with `decoupling_negative = None`.
- `dec_opfit.py`: the proposed fitter.
- `r17_proof.py`: the fixture check of the algebra (`python -I Tools/Adversarial/decoupling/r17_proof.py`).
- `test_decoupling_r17.py`: 31 tests.

Result: `python -I -m pytest Tools/Adversarial/decoupling -q` gives **31 passed in 89 s**.

## 1. Root cause

**The violated assumption.** The upstream decoupler inverts a *secant* (large-signal) velocity-gain matrix. At run time it applies that inverse as a *small-signal correction riding on the operating point*.

That is correct only if the steady map e -> v(e) is all three of:
- (i) additive across axes;
- (ii) odd, with one gain for both signs;
- (iii) homogeneous of degree 1 in amplitude.

Upstream knows (i) fails for its *data*: it drops mixed samples because squared thrust makes them non-additive. But `C e` *is* a mixed command, so the decoupler still relies on (i) at run time. The task's guess (one linear map on mixed-sign data) is right in direction but incomplete. Three separate mechanisms are at work.

**Algebra.** Plant at steady state: `D(v) = W(e) = B K_sigma phi(M e)`.
- `phi(t) = t|t|^(n-1)`.
- `K_sigma` holds per-thruster gains, with reverse gain x rho.
- In the X layout |M| = 1.

Take row (j, s, a), with leak `y = v_x(s a e_j)` and incremental Jacobian `J = dv_x/de_x` at that operating point. Upstream column: `c_j = e_j + eps`, with `eps = -G_xx^-1 G_xj`. The first-order residual is `r = y + J s a eps = (1 - kappa) y`.

With linear drag, every quantity is a matrix of the form D^-1 B K M:
- Fitting on +-a samples with an intercept gives `G = abar^(n-1) D^-1 B Kbar M` plus the intercept, with `Kbar = k (1+rho)/2` and `abar = sum a^3 / sum a^2`.
- The true incremental gain is `J = n a^(n-1) D^-1 [B K_(j,s) M]_xx`.
- The true leak is `y = s a^n D^-1 [B K_(j,s) M_:j]_x`.

1. **Surge rows** (all thrusters share a sign, so `K_(1,s)` is proportional to `Kbar`). This gives `J s a eps = -n y` *exactly*, so **kappa = n**. With n = 2 the correction is twice too large and the residual is about -y: no improvement for either sign. Linear thrust (n = 1) is exact.
2. **Sway/yaw rows with rho != 1.** The sign pattern mixes forward and reverse thrusters, so `y(+a) != -y(-a)`. The leak has an **even** part. A slope fitted on +-a sees only the odd part, and one column per axis cannot cancel an even leak. Fixing it needs columns of opposite sign for +e and -e, which only a sign-split C can express. One sign improves while the other worsens.
3. **Quadratic drag.** Cross leaks are small-signal (slope 1/d1), but G_xx carries the cross axes' own large-signal secants 1/(d1 + d2|v_k|). The correction is over-scaled on *both* signs.

Linear thrust, rho = 1 and linear drag satisfy (i) to (iii) up to Coriolis, so the fit works there.

**Fixture check** (`r17_proof.py`): kappa is measured with +-p probes at the +-0.4 rows, and compared with the simulated after/before ratio.

| regime | surge+ / surge- kappa | first-order \|1-kappa\| vs simulated after/before |
|---|---|---|
| lin_lin_sym | 0.993 / 0.993 | match (yaw rows 0.22 / 0.22: a Coriolis residual) |
| lin_lin_asym | 1.04 / 0.98 | sway kappa 0.16 / -0.002 and yaw -0.21 / 0.004: the even leak goes uncorrected |
| quad_lin | **2.06 / 2.04** | all rows match within 0.06 (e.g. yaw+ 1.92 vs 1.93) |
| quad_quad | 1.76 / 1.72 | first order is only indicative; simulated surge 1.77 / 1.59, yaw 1.67 / 1.28 |

**What the baseline reproduces at verify +-0.4** (settled holds, so this is not transient carry-over). Values are translation cross before -> after.
- **quad_lin** (the R17 case): surge+ 0.0096 -> 0.0106, surge- 0.0023 -> 0.0030, sway+ 0.095 -> 0.072, **sway- 0.079 -> 0.090**, **yaw+ 0.068 -> 0.131**, yaw- 0.083 -> 0.023.
- **quad_quad:** surge and yaw rows are worse on both signs.
- **lin_lin_sym:** every row improves.

Correction to the verifier's description: in a settled fixture, the "reverse rows worse, forward rows better" pattern from I1-verify is **not universal**.
- What holds: surge rows get no better on either sign (kappa = 2), and the sway/yaw change is sign-asymmetric.
- Which sign loses depends on the row: here sway- and yaw+.

## 2. Candidate fitters

| option | what it needs | identifiable from steady holds? | abstains when | verdict |
|---|---|---|---|---|
| (a) force-space fit through an identified thrust curve | thrust curve + drag to turn velocity into force | **No.** Thrust exponent and drag shape trade off (D11: quad/quad gives v ~ cmd). Needs transient/pulse data with a known mass. | the exponent-vs-d2 Fisher information is rank-deficient | Its *insight* is used: the null condition lives in force space and needs no drag. |
| (b) sign-split piecewise-linear C | one column per (axis, sign) | yes, **if** the columns come from incremental Jacobians | a sign was never excited | Right *form* (exact under homogeneous thrust). The naive per-sign G refit fails (I1-verify section 5; also mutant M1 below). |
| (c) speed-scheduled C | columns at each amplitude | yes, inside the envelope | it would extrapolate outside the measured amplitudes | Fallback only, used when homogeneity is rejected. |
| (d) joint thrust + drag + coupling with uncertainty | everything | partly. The scale, the symmetric "X-angle" mode and the exponent/d2 split are not identifiable. | rank-deficient | Too heavy for the decoupler. A drag-free *actuator* sub-model is used for the fault report. |

**Chosen: (b) with operating-point Jacobians, (c) as a guarded fallback, and an (a)/(d)-style force-space actuator ID for faults.**

Key fact: with diagonal hull drag, zero steady cross velocity happens if and only if the cross *wrench* is zero, and Coriolis vanishes at that null point. If each thruster curve is homogeneous per sign, f(lambda t) = lambda^n f(t) (true for CRANE `Thruster.cs`, K s|s| with backK), then the null command scales with amplitude: `e*(s a) = s a c_j^s`. So a sign-split *linear* C is **exact at every speed, for any exponent n and any drag law**. That is why it extrapolates to held-out speeds. Homogeneity is an assumption, so it is *tested* (across two or more amplitudes), not assumed.

## 3. Implementation (`dec_opfit.py`)

Procedure:
- Neutral holds.
- For each axis, sign and amplitude (default 0.1 and 0.2): a base hold plus +-p probes (p = 0.25a) on both cross axes.
- Two chord-Newton confirmation holds at the proposed command.
- All holds are logged as **raw effort**, with the active decoupling bypassed.
- That makes 86 holds; the upstream procedure uses 14.

Fit:
- Shared world-frame drift, estimated from heading change within holds.
- The leak is measured against the neutral bias.
- `J` comes from central differences.
- The column is `e_j + J^-1(-y)/(s a)`, followed by chord refinement.
- Sigma comes from a 200-draw Monte Carlo.
- Homogeneity test: |c(a1) - c(a2)| <= max(0.01, 3 sigma).
  - If it passes: `mode=homogeneous`. Export `C_pos`/`C_neg`, which equals mhseals_hardware `decouple(e, decoupling, decoupling_negative)` (tested).
  - Otherwise: `mode=scheduled` with `valid_range`. No matrix export, and `in_envelope()` is false outside it.

Abstention reasons, each returned with no decoupling:
- `no_reverse_excitation`, `missing_axis_sign_excitation`
- `no_cross_probe_excitation` (upstream-style data: secant gains only)
- `homogeneity_untestable_single_amplitude`
- `jacobian_unidentified` (smin <= 3 sigma)
- `insufficient_precision` (column sigma >= 0.02)
- `inconsistent_probe_sizes`, `probe_not_on_operating_point`, `no_neutral_hold`
- `cannot_divide_out_active_decoupling`

**Fault report (separate from decoupling).**
- The identified null columns are explained by per-thruster strength and mount-yaw deviations of the *design* actuator. This uses a force-space Newton (no drag needed) and Levenberg-Marquardt restricted to the observable subspace (rank 6 of 8; the common scale and the symmetric X-angle mode are removed).
- It flags strength deviations above 10%, mount yaw above 5 deg, or an unexplained residual above 0.02, each beyond 3 sigma.
- These tolerances are *declared* assumptions, not derived from data.

## 4. Test results (31 passed, 89 s)

**R17 reproduced by the baseline:**
- lin_lin_sym: every row is at most 0.25x of before.
- quad_lin: sway- and surge- worse, sway+ better, yaw +/- sign-asymmetric.
- quad_quad: surge and yaw worse on both signs.

**Mechanism:**
- kappa = n on surge rows (n = 1, 1, 2).
- The first-order prediction matches simulation within 0.1 under linear drag.
- The even leak is invisible to the fit (lin_lin_asym).

**New fitter, all 4 regimes:** fit at 0.1/0.2, verify at the held-out 0.15, 0.3, 0.4 and 0.6.
- Every row ends at most max(0.15x before, 0.003).
- It is never worse than identity or upstream.
- No false fault flag (RR 0.95 is within tolerance).
- Worst translation row at 0.4/0.6, before -> upstream -> new: quad_lin 0.222 -> 0.212 -> 0.000; quad_quad 0.402 -> 0.303 -> 0.016.

**Noise** (0.01 m/s, 2 seeds): rows fall to the noise floor (at most 0.03) and the fit reports no fault.

**Faults detected after decoupling:**
- weak_rr: `RR_strength`, ratio 0.598 against a true 0.6.
- angled_fr: `FR_mount_yaw` (11.5 deg shown, which is the true 15 deg minus the unobservable symmetric mode); also flagged under noise.
- hull_drag and combined_current: flagged.
- Masking test: after decoupling, weak_rr and angled_fr fall **below the D10 tolerance of 0.087**. A residual-only gate would pass them; the report still flags them.

**Must-abstain cases:**
- no reverse excitation;
- single-axis data only;
- a single amplitude;
- 0.5 m/s noise;
- an unlogged active decoupling.
- Contrast: the upstream baseline silently returns a C from forward-only data.

**Non-homogeneous thrust** (deadband-edge curve): the fit goes to `scheduled`, valid only on [0.1, 0.2], and 0.4 is out of envelope.

**Ad-hoc mutants** (not committed):
- sign-averaged columns: worst ratio 1.09, so the test fails as it should;
- 2x (secant-sized) correction: 1.00, fails;
- no confirmation step: still passes on quad_lin (one Newton step is enough there). The confirmation matters on quad_quad: 0.0195 -> 0.007 at sway+0.4.

## 5. Residual risks

- **Fixture only.** The X layout and ±45 deg axes are fixture values; pre-fit cross values are about 5x CRANE's. No live CRANE run has been done.
- **Homogeneity.** It holds for CRANE `Thruster.cs`, but not for a real T200 (deadband, curve shape). On hardware, expect `scheduled` mode, which needs an envelope covering the operating speeds.
- **Mixed commands.** A sign-split C is exact only for single-axis commands. Mixed closed-loop commands are not additive under squared thrust. Not tested.
- **Fault attribution is not identifiable.** hull_drag aliases into actuator parameters (FL 0.65 and others), so it is detected but mislabelled. The common strength scale and the X-angle mode are unobservable from steady holds. combined_current's angles are not recovered.
- **Precision.** At 0.05 m/s noise the fit abstains (`insufficient_precision`), so it needs longer windows or quieter odometry. The tolerances (0.01, 0.02, 10%, 5 deg) are declared, not frozen.
- **Cost.** 86 holds against upstream's 14, roughly 20 minutes live.
- **Current in yaw rows** relies on within-hold heading change to separate drift.
- **Runtime.** `pytest Tools/Adversarial` now also collects these tests (+89 s).

## 6. What crane_testing must change versus upstream

1. **Excitation.** Add cross-axis probes on every operating point, use both signs, use at least 2 amplitudes, and add confirmation holds. Do not drop mixed samples; probes are exactly the mixed data the decoupling needs.
2. **Fit.** Compute per-(axis, sign) null columns from operating-point Jacobians. Never compute `G^-1 diag(G)` from a secant fit. Export `decoupling` + `decoupling_negative` only when the homogeneity test passes. Otherwise emit a schedule with an envelope, or refuse.
3. **Abstention.** Use the reasons in section 3. Never fall back to a sign-averaged C or to identity silently.
4. **Raw-effort logging.** Log characterization holds as raw effort. Upstream divides out C linearly, which is impossible for a sign-split C; without raw logging the fit must abstain.
5. **Gating.** Detect faults from the identified plant (fault report), not from the post-decoupling residual. A good decoupling hides actuator faults from the residual gate (tested). Pair this with D10's tolerance and a "no worse than identity" check per row.
6. **Next steps.** Register the R17 tests in `matrix.json` (left to the agent who owns it). Then run a LIVE `reference` + faults pass in CRANE before any default change.

## 7. Round 2: fixes after the independent review (I1-verify-r17-gyro.md)

Edits stay inside `Tools/Adversarial/decoupling/`:
- rewritten `dec_opfit.py`;
- `dec_sim.py` gains `quad_knee`, plus deadband/geometry/hull options in `Regime.builder`;
- updated `test_decoupling_r17.py`;
- new `test_decoupling_r17_round2.py`.

The reviewer's scripts were re-run against the live code through a shim, `/tmp/crane-r17-fix/rerun/` (outputs in `out/`). The shim sets FIT to the new default and records refused rows instead of crashing on them. **Suite: 48 passed in 235 s** (31 kept, some assertions adapted, plus 17 new).

Design changes:
- **Fit amplitudes:** at least 3 amplitudes, default 0.1/0.3/0.6.
- **Probe:** a fixed absolute probe (0.25 x the lowest amplitude). The old probe proportional to amplitude pushed the cross axes out of their small-signal range at 0.6.
- **Refinement:** Broyden quasi-Newton refinement over 3 confirmation holds. It updates only on a statistically significant secant mismatch, because a pure chord step diverged at 0.6.
- **Acceptance gate:** per-row, at the held-out midpoint amplitudes with 8 s windows. A row fails if after - before > 1 sigma + 1e-4; any failure means ABSTAIN `worse_than_identity_on_verification`, and the fault report is kept.
- **Homogeneity:** checked on every pair of amplitudes plus a curvature check. `certified_envelope` is the tested range, and `decoupler` raises `OutOfEnvelope` outside it.
- **Excitation gate:** a ladder at 0.5x and 0.75x the lowest amplitude gives a lower bound on the deadband. ABSTAIN `INSUFFICIENT_EXCITATION` if the response is at or below the noise floor, or if the deadband reaches the lowest probe excursion.
- **Fault report, in wrench space:** H0, sparse thruster (1 or 2 thrusters), full thruster, hull (odd in v, shared across signs), and thruster + hull.
  - Hypotheses are accepted by chi^2 at p = 0.001.
  - The thruster-vs-hull decision uses 2 ln LR with |dchi^2| < 10, giving `thruster-or-hull: ambiguous`. In that case thruster flags appear only as `candidate_flags`.
  - If only the full thruster model explains the data (all four thrusters moving), or nothing does, the report says `MODEL_GEOMETRY_SUSPECT`.
- **Mixed-command gate:** checks uncommanded-axis leak plus superposition error. Worse than identity means mixed commands pass through undecoupled, with the reason recorded.

Break status against the reviewer's cases:

| case | status |
|---|---|
| (d) noise near threshold | **CLOSED.** 0 of 180 accepted fits are worse than identity (worst after/before 0.177). The abstain band is still sharp, at 0.027-0.029. |
| (e) held-out / knee | **CLOSED.** Out-of-envelope rows are REFUSED (72). The knee goes to `scheduled`, and inside the envelope the rows are at most 0.004x identity. |
| (b) deadband 0.12 / 0.08 | **CLOSED:** INSUFFICIENT_EXCITATION, and no coupling flag. RR-only deadband 0.08 is homogeneous and correct inside the envelope, but its partial deadband is not detected (OPEN, minor). |
| (b) split exponent | **CLOSED:** `scheduled`, no WORSE rows inside the envelope. |
| (c) wrong geometry | **CLOSED:** `MODEL_GEOMETRY_SUSPECT` + ambiguous, with 0 thruster flags. |
| (f) hull couplings undetected or misattributed | **CLOSED:** Nvv/Nuu/Nv and the combined case become `hull`; Yuu and drag_offset become ambiguous (+ geometry suspect). |
| (f) thruster faults | Now **ambiguous** (correct thruster *candidate*: RR 0.600, FR 8.0 / 15.0 deg). With steady probes the hull hypothesis explains them within the model floor. A coast-down or transient test is needed to attribute them (**OPEN**, by design). |
| (a) mixed commands | **DOCUMENTED, NOT FIXED.** The mixed gate disables decoupling in every fixture regime, so mixed commands pass through and keep the identity leak. In quad_quad the decoupled uncommanded leak is smaller, but superposition is worse. A mixed-aware decoupler is OPEN. |

Changes to existing tests:
- `FIT_AMPS` becomes 0.1/0.3/0.6, and held-out rows move to 0.15/0.2/0.4/0.5, inside the envelope.
- Fault tests accept thruster flags as candidates.
- The noise-abstain test also accepts INSUFFICIENT_EXCITATION.
- The non-homogeneous test now expects refusal outside [0.1, 0.6].

Cost: about 230 holds per characterization, roughly 8 s of simulation time in the fixture.

## 8. Round 3: separating thruster faults from hull coupling (coast-down)

**Identifiability.** Linearised at the design point, computed by `dec_opfit.identifiability_ranks` and asserted for all 4 regimes. Parameters:
- theta_A: per-thruster log-strength and mount yaw (8);
- theta_H: hull cross coupling h1 v + h2 v|v| per (driving axis, cross axis) (12). drag offset, Yuu, Nuu, Nv and Nvv are all points in this family.

| excitation | rank actuator | rank hull | joint rank | verdict |
|---|---|---|---|---|
| steady null columns (round 2) | 6 (the common scale and the symmetric X-angle mode are unobservable) | 12 | **12** (< 18) | all 6 actuator directions lie inside the hull span: aliased, hence AMBIGUOUS |
| + coast-down from the null point, thrust off | sensitivity 0 | 12 | **18** | separable |
| single-thruster pulses (not implemented) | 8, including scale, if mass is known | 0 at v = 0 | — | separable, but needs 8 rest/pulse cycles (about 50 s) and differentiated odometry |
| transient steps (not implemented) | mixed | mixed | — | needs a dynamic drag model; redundant once the coast is in place |

Why the coast-down has zero actuator sensitivity: it starts at the identified null command. Spool-down scales every thruster force by the same factor (homogeneous), so the actuator cross wrench stays zero; only hull terms driven by the coasting axis act.
- Hull pairs on different cross axes are not confounded with each other: drag_offset is surge->yaw, Yuu is surge->sway.
- Hull subtypes on the *same* pair (drag_offset vs Nuu) differ only in speed shape. They would need the coast's speed sweep to separate; they are reported only as `hull`.
- A thruster+hull mixture: the coast proves the hull, but the actuator stays a candidate (`actuator_not_excluded`).

**Protocol cost.** A 2 s thrust-off coast is appended to the last confirmation hold of each of the 6 top-amplitude operating points. That is **12 s of extra hold time**, with no extra settling.

**Decision rule.**
- `hull_seen`: some cross response grows more than COAST_TOL = 0.005 + 3 sigma beyond its pre-coast residual.
- `decisive`: a hull is seen, or a power check passes. The power check requires that every material steady deviation, if it were a hull, would move at least 10% of its quasi-static cross velocity |r| W_j / d1_x.
- Hull seen: `hull` (or AMBIGUOUS / geometry suspect if HH fails).
- Decisive with no hull seen: `thruster` (sparse) or MODEL_GEOMETRY_SUSPECT (multi-thruster).
- Not decisive: the round-2 likelihood-ratio ambiguity logic.

**Results** (re-run of the reviewer's cases in `/tmp/crane-r17-fix/rerun/out/*_r3.txt`):
- **Thruster faults:** weak_rr -> `thruster ['RR_strength']`, ratio 0.600. angled_fr 8 / 15 deg -> `['FR_mount_yaw']`, 8.0 / 15.0 deg. Both also hold with 0.01 noise.
- **Hull couplings:** Nvv ±8, Nuu, Yuu, Nv, the combined case and drag_offset all give `hull` with no thruster flags.
- **Geometry:** a wrong geometry (mild or strong) gives `MODEL_GEOMETRY_SUSPECT` alone.
- **No false flags:** the nominal plant (with and without noise) and the deadband/knee cases are `none`.
- **combined_current:** `hull`, with the actuator as a candidate only.

**Tests:**
- `test_decoupling_r17_round3.py` (18 tests).
- Round-2 ambiguity tests now run the steady-only protocol (`coast_down=False`), where the ambiguity is still true.

**Open:**
- actuator attribution when a hull is also present (a mixture) needs pulses or a quantitative coast model;
- hull subtype attribution;
- no live data.
