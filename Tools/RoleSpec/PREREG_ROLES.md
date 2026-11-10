# PREREG_ROLES: role specialisation vs a tuned role-free shared-revisit planner (DRAFT)

**Status: DRAFT. Not frozen.** The orchestrator finalises this after the literature review
(Docs/DroneVisuals/mission/LIT_ROLES.md). No held-out run may happen before it is frozen. Every number in
Tools/RoleSpec/results/ is DEV-only (seeds 0-199) and exploratory.

Simulator: Tools/RoleSpec (controlled NumPy model, evidence label IMPLEMENTED; nothing is LIVE_SIM_VALIDATED or
PHYSICALLY_CALIBRATED). The detector anchor is the NOMAD-calibrated Tools/Vision/obs_model.py at the slow speed.
The speed-detection trade-off, decoys and false-alert rates are controlled knobs, not calibrated quantities.

## 1. Claim being tested (narrow, from LIT_ROLES section 4)

The claim is NOT "first adaptive recon/verify team". Prior work covers that: Lee 2021, Jin/Polycarpou 2003-06,
Zhu 2023, Kareem 2026. The claim is an empirical regime map. For physically identical drones, explicit role
commitment (recon vs verify, with hysteresis) improves time to CONFIRMED-AND-REPORTED detection over a tuned
role-free shared-revisit planner (c) only in a characterisable regime, and the map shows where the advantage
reverses.

## 2. Arms (all use one shared scorer, sensing model, battery, speeds, comms and reporting rule)

| Arm | Implementation | Literature anchor |
|---|---|---|
| (a) HOMO_FAST | shared scorer, FAST passes only | B1 |
| (b) HOMO_THOROUGH | shared scorer, SLOW passes only | B1 |
| (c) HOMO_SHARED_REVISIT | Bayesian revisit map (FP + miss + correlated-view model), per-cell speed choice, revisits by expected gain per unit time, intent sharing | B2 (Bourgault 2003, Waharte 2010, Chung 2012) |
| (d) SPEC_FIXED(r) | r = recon fraction; ratios 1:1, 2:1, 4:1 (r = 0.5, 0.67, 0.8) | B4 (Lee 2021) |
| (e) SPEC_ADAPTIVE | response threshold on the unverified-candidate backlog vs uncovered prior mass (local stimulus range), hysteresis h; a marginal-value variant is tuned inside the same budget | B5 (Krieger 2000) |
| B3_JIN_CONFIRM | dynamic Confirm task, nearest free UAV | Jin 2006 |
| B6_AUCTION | greedy single-round auction among currently connected drones | Dias 2006, Giacomossi 2026 |
| B7_SOLO | per-drone detect-then-confirm, no drone-drone comms, static sectors | Senthilnath 2024 |
| B9 NBV | every verifying pass picks the view class with the highest expected high-alert probability (shared scorer), in ALL arms | Strand 2025 |
| ORACLE (labelled) | knows survivor cells; upper bound on time to confirmed | - |

B8 (Nathan 2025 confidence-driven, role-free) is not implemented separately. Arm (c) is the closest role-free,
confidence-driven policy available without AOS imaging. This is stated as a limitation.

Budget matching: the same battery (Wh), speeds {v_fast, v_slow}, transit speed, sensing tables, confirmation
threshold tau = 0.95, comms realisation (common random numbers), reporting/rendezvous rule and the
`switch_s` penalty for every arm.

**Tuning: an equal budget of K = 10 configurations per family**, the default plus a seeded random sample of its
space. Tuning uses DEV seeds 0-99 on a mixture of all main-grid cells, with objective J = AUC_rep - 0.25 x false
reports. The tuned parameters are frozen in `results/tuned.json` before any held-out run.

Design change made during DEV (recorded so reviewers can see it): in the first tuning pass, B7_SOLO (no comms,
hard pre-launch sectors) beat the best (c) (J 0.333 vs 0.290), so (c) was not strong enough. The shared scorer
now has a soft sector preference `sector_w` (gain outside the drone's pre-launch band is scaled by 1 - sector_w),
and every family except B7 can tune it. SPEC_FIXED recon drones partition the map among themselves, and
verifiers roam. Every family was then retuned with the same K.

Role-switching penalty: `switch_s` is the hover time added whenever a drone changes speed mode between passes
(altitude or camera reconfiguration). It applies to every arm, including (c)'s per-cell speed choice. Policy
hysteresis `h` is a separate, tunable commitment parameter.

## 3. Hypotheses (from the LIT_ROLES falsifiers; each can kill the claim)

The primary metric for H1-H3 and H5 is AUC_rep: the mean over survivors of 1 - min(t_reported_confirmed, D)/D.

- **H1 (P1).** At N = 2, (e) does not beat (c). Supported if the 95 % CI of (e) - (c) includes 0 or is < 0 in
  every N = 2 cell.
- **H2 (P2).** The (e) - (c) gain grows from N = 2 to N = 5 and then saturates or reverses at N = 10 (fixed
  area per drone). Test: the contrast gain(N=5) - gain(N=2) > 0, and gain(N=10) - gain(N=5) <= 0 within CI.
- **H3 (P3).** The best verifier fraction 1 - r* of (d) rises with N and with the false-positive rate
  (fa_scale 1 vs 3, clutter 0.05 vs 0.12). r* is chosen per cell on DEV. The held-out test is a sign test of the
  r-sweep curves' argmax over cells.
- **H4 (P4).** With no speed-detection trade-off (trade = none), (a) >= (b) and (d), (e) are within CI of (a).
- **H5 (P5).** Under outages, the drop from ideal to outage is larger for (c) than for (e) (threshold rule).
  Test: the difference-in-differences [(c)_ideal - (c)_outage] - [(e)_ideal - (e)_outage] > 0.
- **H6 (P6).** With unlimited time (D = 3600 s, 400 Wh), recall_rep of (b) >= that of (e) (non-inferiority
  margin 0.02).

Secondary metrics, reported for every arm and cell: recall at deadline (reported), time to first detection, time
to confirmed (onboard), time to reported, false confirmed reports, energy, duplicated effort (same cell-view
looked at by a different drone), number of mode switches, and the failure label per mission (`success`,
`never_searched`, `missed_on_pass`, `detected_unconfirmed`, `confirmed_unreported`, `+false_confirm`).

## 4. Cells

- Main grid (32 cells): N in {2, 3, 5, 10} (area 30 cells of 40 m per drone) x occlusion {low, high} x
  trade-off {weak, strong} x comms {ideal, outage}.
- Side cells at N in {3, 5, 10}, high occlusion, strong trade-off, outage: trade = none (H4); fa_scale 3 +
  clutter 0.12 (H3); switch_s = 20 s; unlimited time (H6).
- Ratio sweep for H3: r in {0.2, 0.34, 0.5, 0.67, 0.8} at N in {3, 5, 10} x {ideal, outage} x fa_scale {1, 3}.

## 5. Seeds and replication

HELD-OUT layout seeds **5000-5019** (20 layouts; refused by `rolespec.world.guard` unless `allow_heldout`),
with R = 5 replicates per layout (rep 0-4: view latents, decoys, outcome uniforms and channels re-drawn). DEV
seeds 0-199 are never reported as evidence. Note for the orchestrator: 5000-5019 lies inside the global
splits.json `train` range (2000-69999). RoleSpec does no learning, so the RoleSpec-local guard is the binding
held-out guard. The alternative is to move the range into the global held-out block 90000-99999 before freezing.

## 6. Statistics

- The unit is a layout. Missions are paired across arms by (layout, rep) through common random numbers.
- Estimate: the mean paired difference. 95 % CI by paired cluster bootstrap over layouts (2000 resamples;
  `missionsim.stats.cluster_boot_diff`). Two-sided bootstrap p.
- Multiplicity: Holm across the pre-declared primary family. That family is H1 (one test per N = 2 cell,
  8 cells), H2 (2 contrasts), H4 (2), H5 (1 per N, pooled over occlusion and trade-off) and H6 (1 per N).
  H3 is a direction test reported with its own Holm family. Everything else is descriptive.
- Censoring: an unconfirmed or unreported survivor contributes D to the time metrics. Recall is reported
  separately.

## 7. Known limitations (to state with any result)

- The posterior treats cells as independent; there is no AOS imaging and no altitude model, since speed is the
  only fidelity knob.
- Comms is slot-level flooding within connected components, with no bandwidth limit.
- The decoy and false-alert numbers are PRIOR values.
- The ORACLE is an upper bound on time to confirmed, not to reported.

## 8. DEV observations relevant to finalising this draft (exploratory, not evidence)

- On 12 of 32 main cells (ideal comms, N in {2, 3, 5}, 8 dev layouts each), no specialised arm beat the tuned
  (c). The pooled (e)-(c) on AUC_rep was N=2 -0.062 [-0.108, -0.016], N=3 -0.029 [-0.078, +0.025] and
  N=5 -0.113 [-0.171, -0.042]. B3_JIN_CONFIRM and B7_SOLO were within CI of (c).
- About 75-80 % of failures are coverage failures (`missed_on_pass`, `never_searched`). The main grid is
  therefore coverage-bound, and H1/H2 risk being decided by that regime choice. Recommendation: before freezing,
  add verification-bound cells (fa_scale 3, clutter 0.12, the switch penalty, long deadline; the `side` grid)
  and a candidate-density factor. Otherwise a null result says nothing about where roles could help.
- The tuning-set best J (dev 0-31, mixture of all main cells) was: (c) 0.367, B3 0.343, B7 0.333, (e) 0.331,
  B6 0.314, (b) 0.303, (a) 0.295, (d) 0.290. The (e) winner used the marginal rule with h = 0, which sits close
  to (c) by construction.

## Orchestrator decisions before freezing (2026-11-10), still DRAFT
1. **Held-out seeds** move from 5000–5019 (inside the global `train` split) to **91000–91019**, in the global HELD-OUT range and disjoint from NavPilot E2 (90200–90207) and E3 (90400–90407).
2. **Add VERIFICATION-BOUND cells** using the calibrated detector's false-alert and decoy rates from Tools/Vision/obs_model.py (false alerts up to 0.5 per cell pass, persistent decoys at ~0.8 per pass), alongside the current coverage-bound cells. The dev finding (no specialization gain) currently covers the coverage-bound regime only (75–80% of failures are search misses).
3. **Run the side grid** (higher false-positive rate and clutter, switching penalty > 0, long deadline) on DEV before freezing.
4. **Freeze only after (2)–(3) run on DEV** and the hypotheses are restated per regime: coverage-bound vs verification-bound.
