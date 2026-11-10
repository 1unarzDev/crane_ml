# Preregistration: team-size scaling of the value of communication

Frozen 2026-11-10, before ANY dev-grid or held-out data from this study was run.

## Provenance
- **Code:** worktree `crane-forest-sar`, `Tools/ForestSearch/nscale.py`, uncommitted.
- **Manifest:** sha256 of this file, nscale.py, test_nscale.py and every imported module is recorded in `results/nscale_manifest.txt`, written at freeze time.
- **Params file:** `results/nscale_heldout_params.json` is generated afterwards by the frozen rule below, from dev seeds only. Its sha256 is then appended to the manifest.
- **Integrity check:** `nscale.py heldout` refuses to run if nscale.py, this file or the params file does not match the manifest.

## Deviation from the requested order (stated up front)
The requested order was "dev exploration, then prereg". The shared CPU-slot queue was saturated: about 20 waiters on 3 slots, and nothing of mine had run after 50 minutes. So the design is frozen here first, as fixed rules that a dev scan on seeds 1000–1003 resolves mechanically. That fixes the horizon per N and the episode count.

Consequences:
- No dev effect size can influence a hypothesis, margin or cell.
- The dev grid (seeds 1000–1003, `results/nscale_dev.md`) is descriptive only and may run after the held-out run.
- The "expected" statements below come from earlier, disjoint studies: 2-agent PREREG/confirm (seeds 200–207), beacon_final (800–811) and the nagent pilot (400–403).

## Question
Does the value of communication (VoC) scale consistently across N ∈ {2, 3, 5, 10} searchers? Conditions varied: event rate, travel, sensor loss and link (delay/loss).

## Design (fixed)
- **Instances:** `grid_instance(w, h, seed, prior=blobs, travel_scale=travel, **events(λ))`.
  - Held-out seeds **3000–3015**: 16 instances per cell, fresh, unused anywhere before.
  - The same seed gives a different grid per N, so instances are independent across N.
- **Grid per N (about 12 cells per agent):** N=2 → 5×5, N=3 → 6×6, N=5 → 8×8, N=10 → 11×11.
  - Starts: the 4 corners (nagent), then farthest-point sampling.
- **Environment:**
  - Private-event scale λ ∈ {1, 3}, as in event_curve.events.
  - travel_scale ∈ {1, 0}. Travel 1 is primary and runs first; travel 0 is the no-travel environment.
- **Links** (per directed pair; a broadcast is N−1 independent sends):
  - `ideal`
  - `d1`: delay 1, no loss
  - `burst30_d1`: Gilbert–Elliott, 30% mean loss, mean burst 10, delay 1
- **Fault:**
  - `none`
  - `perm1`: exactly one agent, chosen uniformly, permanently loses its sensor at t ~ U[1, H/2]. Its looks abort; its radio stays up.
- **Planner:** greedy only.
  - Rollout-lite (k=3, budget 8) is judged *not cheap* under the current CPU contention, so it is excluded from the confirmatory set.
  - The dev scan reports its cost and its P(detect) at dev seeds, descriptively.
  - k = ⌈N/5⌉ loss is not in the confirmatory set; it differs from k=1 only at N=10.
- **Horizon rule (dev seeds 1000–1003, 20 episodes):**
  - For each (N, travel), H is the candidate with P(detect | IMPLICIT, healthy, λ=1, ideal) closest to 0.60.
  - Candidates: travel 1 → {16, 24, 32, 40}; travel 0 → {8, 12, 16, 20}.
  - Ties go to the first (shortest) candidate.
- **Episode rule** (`nscale.py params --budget 90 --travels 1,0 --planners greedy`):
  - E (episodes per instance for N < 10) is the largest multiple of 10 in [40, 200] that fits the budget. N = 10 uses E10 = max(40, ⌊0.6E⌋).
  - The budget check uses dev ms/episode (λ=3, burst link, perm1) to estimate total CPU, which must be ≤ 90 CPU-min.
  - If nothing fits, E = 40 and E10 = 30.
  - Episode seeds are 0..E−1, with fault seed 10000 + episode.
  - Common random numbers: the same truth, outcome, link and fault draws are used for every protocol.
- **Protocols:**
  - **ORACLE:** told when a partner is dead.
  - **INDEP**.
  - **IMPLICIT:** the reference.
  - **HYBRID:** FULL messaging layered on the nominal plan.
  - **HYBRID_P:** adds the priority-order role rule.
  - **HEALTH:** one 12 B death frame per peer.
  - **HB_TO:** a 12 B status frame to each peer every P=3 steps and on change; timeout K=7. These are the 2-agent tuned values, not retuned.
  - DEVIATION (deviation_comm.py) is 2-agent only, so it is not included.

## Metrics
- **Primary:** P(confirmed detection by H).
- **Secondary (reported, not tested):** TTD/H, censored at H; bytes per episode; same-step collisions per episode.
- **VoC:** a paired difference against IMPLICIT.

## Analysis (`nscale.py analyze`, unchanged after freeze; B = 2000)
- **Cluster bootstrap:** instances are resampled first, then episodes within each instance.
- **Instance-level t:** computed on the 16 per-instance means (df = 15).
- **Holm:** applied across the 4 values of N within each hypothesis family. A family is one (travel, λ[, link]) combination.
- **A test passes at a given N only if both criteria hold:**
  - the Holm-adjusted instance-t p < 0.05;
  - the bootstrap interval meets the stated bound.
- **Family verdicts:** a family PASSES only if every N passes. A family with any missing N is INCOMPLETE.

### Hypotheses
- **S1 (healthy teams: silence is enough at every N).**
  - Test: fault none; ORACLE − IMPLICIT is within ±0.03.
  - Method: TOST, where p_TOST is the larger of the two one-sided instance-t p-values; the 90% bootstrap CI must lie inside (−0.03, 0.03).
  - Expected: PASS at N = 2 and 3. At N = 10 the alternative we expect is ORACLE − IMPLICIT slightly *negative* rather than positive. The nagent pilot gave −0.019 at N=3 with travel. Centralised greedy is not better than a shared plan, and HYBRID can fail TOST from noise alone.
- **S2 (HEALTH recovers ≥ 80% of ORACLE's loss gain at every N).**
  - Test: fault perm1. Define D = (HEALTH − IMPLICIT) − 0.8 (ORACLE − IMPLICIT).
  - Primary (non-inferiority): one-sided H1 D > −0.01 (instance t, Holm), and the bootstrap 5th percentile of D > −0.01.
  - Strict version (reported): 5th percentile > 0.
  - Judged per (travel, λ, link).
  - Expected: PASS on ideal and d1. The 2-agent recovery was 75–100%. On burst30_d1 a single frame can be lost, and 2-agent recovery was about 55–75%, so we expect FAIL there, at least at N = 2.
- **S3a (the stale-plan penalty grows with N).**
  - Test: fault none, links d1 and burst30_d1. Penalty_N = IMPLICIT − HYBRID.
  - Slope s: OLS of penalty_N on log2 N; 95% bootstrap CI, resampled within N.
  - PASS iff the lower 95% bound of s > 0.
  - Expected: PASS at travel 0 (pilot penalty 0.07 → 0.16 for N = 2 → 4). Uncertain or FAIL at travel 1, where the pilot showed no trend.
- **S3b (HYBRID_P stays within 0.015 of IMPLICIT).**
  - Test: fault none, d1 and burst30_d1. TOST ±0.015 with Holm; the 90% bootstrap CI must lie inside ±0.015.
  - Expected: the point estimates fall within ±0.015, but TOST at ±0.015 with 16 instances is low-powered, so FAIL from width alone is likely at some N. A FAIL from width is reported as such and is not read as evidence of a penalty.
- **S4 (VoC under loss is one lost agent's share, ≈ 1/N).**
  - Test: fault perm1, VoC_N = ORACLE − IMPLICIT. Weighted-LS fit VoC_N = c·N^−b, weights 1/bootstrap variance; CI of b by bootstrap within N.
  - PASS iff the 95% CI of b contains 1 and its lower bound > 0.
  - Expected: b ≈ 1, i.e. N·VoC roughly flat.

### Step 5 (descriptive)
- **Fits:** y_N = c·N^−b and y_N = a + s·log2 N, with bootstrap CIs.
- **Contrasts:** every contrast vs IMPLICIT, in every condition.
- **Interpretation:** "generalizes" means the hypothesis verdict and the sign agree across both travels, both λ and all links. This is not a test.

## Stop rules and reporting
- **One run only:** held-out seeds 3000–3015 are run once with the frozen code. No reruns with new seeds, no dropped cells, no changes to H or E.
- **If a job crashes,** it is restarted with identical arguments. Runs are deterministic (tested), so the outputs are identical.
- **Task order:** travel 1 with N = 2, 3, 5, 10, then travel 0 with N = 2, 3, 5, 10.
- **If compute runs out:** finished tasks are analysed and reported. Unfinished ones are listed as NOT RUN, and the affected families are reported as INCOMPLETE, never as PASS.
- **Every cell is reported:** §1 of `results/nscale_heldout.md` covers all cells, including failures and results against our claims. Any follow-up is labelled exploratory.
