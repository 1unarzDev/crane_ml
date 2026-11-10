# EXPLORATORY: can a COORDINATED health beacon restore the beacon's benefit at larger N?

**Status: exploratory follow-up to `nscale_heldout.md`. Nothing here is preregistered or Holm-adjusted.** One lam (1), greedy planner, k = 1 permanent sensor loss, 16 instances per cell, 80 episodes per instance (diagnostic: 12). Success criteria below were fixed in the session before the fresh run, not in a frozen file.

## Provenance
- Code: `nbeacon.py` (new; imports the frozen `nscale.py`, not edited), tests `test_nbeacon.py` (20 pass, 5 strict must-fail xfail). Manifest: `results/nbeacon_manifest.txt` (sha256 of nbeacon.py, test file, frozen-rule file; written before the fresh run).
- Dev seeds 1000-1015 for all tuning (`data/nbeacon_dev_*.csv.gz`, E = 60, diagnostic E = 12). Rule frozen in `nbeacon_frozen.json`: per family, the variant with the highest mean (X - IMPLICIT) over the perm1 dev cells at N in {5, 10}. Result: **HEALTH_PART = near, HEALTH_ONE = low**.
- Fresh seeds 6000-6015, run ONCE (`data/nbeacon_fresh_*.csv.gz`; the CLI refuses to rerun). Run via `fsrun`, single process. Fresh seeds were grepped for earlier use: none.
- Horizons from PREREG_SCALING.md Amendment 1: travel 1 -> 24 (N = 3, 5), 32 (N = 10); travel 0 -> 12 (N = 3, 5), 16 (N = 10). (`nscale.HORIZON` holds older 30/16 values; they are not used.)
- Links: ideal and burst30_d1 (Gilbert-Elliott, 30% mean loss, burst 10, delay 1). Cells: N in {3, 5, 10} x travel {0, 1} x {healthy, perm1} x link. Healthy cells are link-free for the beacon protocols.
- Fixture search: `worktrees/mhseals_hardware-characterization` and branch `test/characterization-faults` hold only a hardware config (MHSeals, read-only, untouched); no usable fault fixtures. No hardware was run.

## Protocols (same 12 B death frame; no extra messages, verified by test)
- **HEALTH_PART:** on the frame, the orphaned nominal tasks of the dead agent (start >= stamp) are split by a deterministic rule from the default plan; a receiver drops from its belief only the orphans assigned to itself, the rest stay counted as covered.
  - near (frozen): nearest survivor by Manhattan distance from its default-plan position at the orphan's start; ties go to the lighter load, then lower index. Others tried on dev: nearbal (capacity cap), rr (round robin in priority order).
- **HEALTH_ONE:** all orphans go to one survivor. low (frozen) = lowest index. Tried on dev: near.
- A receiver assumes every other survivor got the frame. Under loss that is false, so an assignee that missed the frame never takes its cells.

## 1. Duplicated-effort diagnosis of plain HEALTH: CONFIRMED (and refined)
Diagnostic episodes run the full horizon with the target removed (no detection, no early-stop truncation), perm1, pooled over travel x link. Fresh-seed values per survivor (full tables in section 3):

| N=10, per survivor | IMPLICIT | HEALTH | HEALTH_PART | HEALTH_ONE |
|---|---|---|---|---|
| looks on the dead agent's orphan cells | 0.02 | 3.59 | 0.74 | 0.34 |
| DUPLICATE looks on orphan cells | 0.00 | **2.92** | 0.09 | 0.04 |
| duplicate looks, any cell | 1.86 | 4.03 | 1.82 | 1.84 |
| own nominal tasks skipped (nobody looked there) | 1.50 | **4.80** | 2.20 | 1.82 |
| same-step orphan collisions (pairs per EPISODE) | 0.00 | **20.4** | 0.00 | 0.00 |
| survivors whose first post-death choice is an orphan | 0.004 | 0.090 | 0.018 | 0.005 |

- The rush is real and grows with N. HEALTH duplicates 0.0 -> 1.2 (N=3), 1.6 (N=5), 2.9 (N=10) orphan looks per survivor, with 0.7, 2.6, 20 same-step collisions per episode. Paired HEALTH - IMPLICIT for duplicate looks is positive with CI excluding 0 at every N (N=10: +2.17 [+2.05,+2.28]).
- Refinement: HEALTH covers the orphans (6.0 distinct orphan cells of 10.9 at N=10, about the same as PART's 5.8) but abandons survivors' own planned cells (4.8 vs 1.5 skipped). So the cost is duplication plus abandoned own work. Duplicated looks on non-orphan cells rise too (4.03 vs 1.86).
- PART removes almost all orphan duplication (2.92 -> 0.09) while covering as many distinct orphans as HEALTH; ONE covers only 2.7 orphans.
- Caveat: diagnostic episodes have no target, so choices differ from the main episodes (no hints). The mechanism is shown as a planning behaviour, not as a decomposition of the detection loss.

## 2. Does coordination restore a benefit? (fresh seeds; perm1 pooled over the 4 travel x link cells)

| N | ORACLE-IMPL (ceiling) | HEALTH-IMPL | HEALTH_PART-IMPL | HEALTH_ONE-IMPL | PART-HEALTH | ONE-HEALTH |
|---|---|---|---|---|---|---|
| 3 | +0.038 [+0.021,+0.054]* | +0.016 [-0.002,+0.032] | +0.026 [+0.010,+0.042]* | +0.020 [+0.008,+0.031]* | +0.011 [-0.002,+0.024] | +0.004 [-0.008,+0.016] |
| 5 | +0.032 [+0.012,+0.052]* | -0.000 [-0.019,+0.017] | **+0.022 [+0.010,+0.037]*** | +0.017 [+0.006,+0.029]* | +0.023 [+0.012,+0.034]* | +0.018 [+0.002,+0.031]* |
| 10 | +0.014 [-0.001,+0.029] | -0.040 [-0.057,-0.021]* | +0.009 [-0.001,+0.021] | +0.008 [+0.002,+0.015]* | +0.049 [+0.037,+0.062]* | +0.048 [+0.032,+0.064]* |

Criteria fixed before the fresh run: C1 = PART - IMPLICIT pooled 95% CI lower bound > 0 at the given N; C2 = PART - HEALTH > 0 likewise.
- **N = 5: C1 and C2 both met.** PART gains +0.022 over IMPLICIT, about 70% of ORACLE's +0.032 (ratio of means, no CI), where plain HEALTH gained nothing. 12 of 16 instances positive (instance-t p = 0.0015, unadjusted). ONE also gains (+0.017).
- **N = 10: C2 met, C1 NOT met for PART.** The harm is removed (-0.040 -> +0.009, a swing of +0.049), but PART - IMPLICIT = +0.009 [-0.001,+0.021] includes 0 (12/16 instances positive, p = 0.083). ONE's +0.008 [+0.002,+0.015] has CI excluding 0 and 11/16 instances positive (p = 0.010), but it is not the chosen headline and is 1 of 2 tried; treat as suggestive. The ceiling itself is small and not significant at N=10 (ORACLE - IMPLICIT +0.014 [-0.001,+0.029]), so there is little to restore.
- Split by link (pooled over travel): ideal N=5 PART +0.027 [+0.011,+0.046], burst N=5 +0.018 [+0.003,+0.033]; N=10 ideal +0.011 [-0.003,+0.025], burst +0.008 [-0.004,+0.019]. Bursty loss slightly weakens PART, consistent with assignees missing the frame; not separately tested.
- Dev agreed in sign: PART - IMPLICIT +0.021 (N=5), +0.005 (N=10), HEALTH -0.054 at N=10.
- Healthy cells: 36/36 (cell, protocol) pairs identical to IMPLICIT episode for episode.

## What does NOT hold / limits
- Not shown: that PART gives a significant gain over silence at N=10; only that it removes the HEALTH harm and brings the team to IMPLICIT level or marginally above.
- Dev tuning did not separate the variants (scores within 0.005); the choice of near over nearbal (dev favoured nearbal at N=10) and low over near is not robust.
- Simple HEALTH_ONE (one taker, the others ignore the death) performs about as well as the partition. So the gain comes mostly from stopping the herd, not from clever assignment.
- lam = 1 only, greedy only, k = 1, no rollout planner, no ORACLE-style loss of the radio. Other lam (3) was not run. 16 instances. No multiplicity correction; per-cell stars are unadjusted.
- The 2-agent recovery (75-100% of the ORACLE gain) is not matched at N >= 5 by either variant: about 70% at N = 5 and 65% at N = 10 in the pooled ratios, but at N = 10 the denominator is not distinguishable from 0.

## Tests
`test_nbeacon.py`: N = 2 equivalence (every PART/ONE rule == HEALTH, ideal/d1/burst, travel 0/1, 25 episodes each); no events => equals IMPLICIT with 0 bytes at N = 3, 5, 10; plain HEALTH == nscale HEALTH; determinism; assignment is a deterministic partition of the orphans (never the dead agent, one taker for ONE, caps respected); no extra bytes; diagnostic metric toy case. Strict xfail (must fail): PART == IMPLICIT under loss; PART == HEALTH at N=5; ONE == PART at N=10; diagnostic episodes detect; a mutant with the assignee test inverted == PART. All 5 fail as required.

## Dev tuning table (seeds 1000-1015, perm1, mean X - IMPLICIT)
| variant | N=3 | N=5 | N=10 | score (N=5,10) |
|---|---|---|---|---|
| HEALTH | +0.0081 | -0.0052 | -0.0542 | -0.0297 |
| HEALTH_ONE:low | +0.0253 | +0.0206 | +0.0016 | +0.0111 |
| HEALTH_ONE:near | +0.0245 | +0.0156 | +0.0013 | +0.0085 |
| HEALTH_PART:near | +0.0276 | +0.0214 | +0.0047 | +0.0130 |
| HEALTH_PART:nearbal | +0.0273 | +0.0177 | +0.0068 | +0.0122 |
| HEALTH_PART:rr | +0.0279 | +0.0133 | +0.0029 | +0.0081 |

---

## 3. FRESH full output (seeds 6000-6015)
### FRESH (seeds 6000-6015, run once) per-cell results (lam=1, greedy)

P(detect) of IMPLICIT and paired differences vs IMPLICIT, mean [95% cluster-bootstrap CI]; * = CI excludes 0. p = unadjusted instance-level t.

| travel | fault | link | N | P IMPL | ORACLE-IMPL | HEALTH-IMPL | HEALTH_PART:near-IMPL | HEALTH_ONE:low-IMPL | HEALTH_PART:near-HEALTH | HEALTH_ONE:low-HEALTH |
|---|---|---|---|---|---|---|---|---|---|---|
| 0 | none | ideal | 3 | 0.633 | +0.016 [-0.012,+0.044] | +0.000 [+0.000,+0.000] | +0.000 [+0.000,+0.000] | +0.000 [+0.000,+0.000] | +0.000 [+0.000,+0.000] | +0.000 [+0.000,+0.000] |
| 0 | none | ideal | 5 | 0.634 | +0.002 [-0.031,+0.038] | +0.000 [+0.000,+0.000] | +0.000 [+0.000,+0.000] | +0.000 [+0.000,+0.000] | +0.000 [+0.000,+0.000] | +0.000 [+0.000,+0.000] |
| 0 | none | ideal | 10 | 0.688 | -0.005 [-0.031,+0.020] | +0.000 [+0.000,+0.000] | +0.000 [+0.000,+0.000] | +0.000 [+0.000,+0.000] | +0.000 [+0.000,+0.000] | +0.000 [+0.000,+0.000] |
| 0 | none | burst30_d1 | 3 | 0.633 | +0.016 [-0.012,+0.044] | +0.000 [+0.000,+0.000] | +0.000 [+0.000,+0.000] | +0.000 [+0.000,+0.000] | +0.000 [+0.000,+0.000] | +0.000 [+0.000,+0.000] |
| 0 | none | burst30_d1 | 5 | 0.634 | +0.002 [-0.031,+0.038] | +0.000 [+0.000,+0.000] | +0.000 [+0.000,+0.000] | +0.000 [+0.000,+0.000] | +0.000 [+0.000,+0.000] | +0.000 [+0.000,+0.000] |
| 0 | none | burst30_d1 | 10 | 0.688 | -0.005 [-0.031,+0.020] | +0.000 [+0.000,+0.000] | +0.000 [+0.000,+0.000] | +0.000 [+0.000,+0.000] | +0.000 [+0.000,+0.000] | +0.000 [+0.000,+0.000] |
| 0 | perm1 | ideal | 3 | 0.529 | +0.034 [+0.004,+0.066]* | +0.002 [-0.032,+0.034] | +0.034 [+0.005,+0.062]* | +0.025 [-0.002,+0.052] | +0.032 [+0.009,+0.056]* | +0.023 [+0.002,+0.046]* |
| 0 | perm1 | ideal | 5 | 0.549 | +0.040 [+0.005,+0.076]* | -0.020 [-0.054,+0.014] | +0.028 [+0.001,+0.057]* | +0.024 [+0.001,+0.047]* | +0.048 [+0.023,+0.073]* | +0.044 [+0.019,+0.069]* |
| 0 | perm1 | ideal | 10 | 0.664 | +0.012 [-0.015,+0.038] | -0.076 [-0.115,-0.038]* | +0.011 [-0.010,+0.031] | +0.009 [-0.003,+0.023] | +0.087 [+0.059,+0.116]* | +0.085 [+0.052,+0.122]* |
| 0 | perm1 | burst30_d1 | 3 | 0.529 | +0.034 [+0.004,+0.066]* | +0.012 [-0.018,+0.041] | +0.020 [-0.005,+0.045] | +0.023 [-0.002,+0.047] | +0.009 [-0.011,+0.028] | +0.011 [-0.006,+0.029] |
| 0 | perm1 | burst30_d1 | 5 | 0.549 | +0.040 [+0.005,+0.076]* | +0.008 [-0.020,+0.035] | +0.016 [-0.008,+0.041] | +0.015 [-0.005,+0.034] | +0.009 [-0.014,+0.031] | +0.007 [-0.014,+0.027] |
| 0 | perm1 | burst30_d1 | 10 | 0.664 | +0.012 [-0.015,+0.038] | -0.044 [-0.075,-0.013]* | +0.009 [-0.007,+0.026] | +0.009 [-0.002,+0.022] | +0.053 [+0.031,+0.076]* | +0.052 [+0.025,+0.081]* |
| 1 | none | ideal | 3 | 0.590 | -0.010 [-0.034,+0.013] | +0.000 [+0.000,+0.000] | +0.000 [+0.000,+0.000] | +0.000 [+0.000,+0.000] | +0.000 [+0.000,+0.000] | +0.000 [+0.000,+0.000] |
| 1 | none | ideal | 5 | 0.594 | -0.009 [-0.032,+0.016] | +0.000 [+0.000,+0.000] | +0.000 [+0.000,+0.000] | +0.000 [+0.000,+0.000] | +0.000 [+0.000,+0.000] | +0.000 [+0.000,+0.000] |
| 1 | none | ideal | 10 | 0.666 | +0.000 [-0.023,+0.024] | +0.000 [+0.000,+0.000] | +0.000 [+0.000,+0.000] | +0.000 [+0.000,+0.000] | +0.000 [+0.000,+0.000] | +0.000 [+0.000,+0.000] |
| 1 | none | burst30_d1 | 3 | 0.590 | -0.010 [-0.034,+0.013] | +0.000 [+0.000,+0.000] | +0.000 [+0.000,+0.000] | +0.000 [+0.000,+0.000] | +0.000 [+0.000,+0.000] | +0.000 [+0.000,+0.000] |
| 1 | none | burst30_d1 | 5 | 0.594 | -0.009 [-0.032,+0.016] | +0.000 [+0.000,+0.000] | +0.000 [+0.000,+0.000] | +0.000 [+0.000,+0.000] | +0.000 [+0.000,+0.000] | +0.000 [+0.000,+0.000] |
| 1 | none | burst30_d1 | 10 | 0.666 | +0.000 [-0.023,+0.024] | +0.000 [+0.000,+0.000] | +0.000 [+0.000,+0.000] | +0.000 [+0.000,+0.000] | +0.000 [+0.000,+0.000] | +0.000 [+0.000,+0.000] |
| 1 | perm1 | ideal | 3 | 0.463 | +0.041 [+0.016,+0.068]* | +0.024 [-0.002,+0.051] | +0.030 [+0.002,+0.056]* | +0.018 [-0.001,+0.037] | +0.005 [-0.013,+0.024] | -0.006 [-0.030,+0.016] |
| 1 | perm1 | ideal | 5 | 0.526 | +0.023 [-0.007,+0.054] | +0.000 [-0.026,+0.024] | +0.027 [+0.006,+0.046]* | +0.016 [+0.001,+0.032]* | +0.027 [+0.012,+0.043]* | +0.016 [-0.004,+0.037] |
| 1 | perm1 | ideal | 10 | 0.627 | +0.017 [-0.009,+0.045] | -0.027 [-0.052,-0.002]* | +0.011 [-0.007,+0.028] | +0.008 [-0.004,+0.021] | +0.037 [+0.016,+0.059]* | +0.034 [+0.010,+0.059]* |
| 1 | perm1 | burst30_d1 | 3 | 0.463 | +0.041 [+0.016,+0.068]* | +0.026 [+0.003,+0.048]* | +0.022 [-0.002,+0.045] | +0.013 [-0.004,+0.031] | -0.004 [-0.025,+0.016] | -0.013 [-0.030,+0.004] |
| 1 | perm1 | burst30_d1 | 5 | 0.526 | +0.023 [-0.007,+0.054] | +0.010 [-0.014,+0.034] | +0.019 [+0.001,+0.037]* | +0.013 [+0.001,+0.027]* | +0.009 [-0.006,+0.024] | +0.003 [-0.016,+0.022] |
| 1 | perm1 | burst30_d1 | 10 | 0.627 | +0.017 [-0.009,+0.045] | -0.013 [-0.034,+0.009] | +0.006 [-0.009,+0.020] | +0.008 [-0.001,+0.019] | +0.019 [+0.001,+0.037]* | +0.020 [-0.002,+0.041] |

Healthy-cell check: 36/36 (cell, protocol) pairs have detections identical to IMPLICIT episode for episode (beacon silent when nobody dies).

### FRESH (seeds 6000-6015, run once) perm1 pooled over travel {0,1} x link {ideal, burst30_d1} (4 cells, equal weight)

Mean paired difference [95% bootstrap CI; instances resampled jointly across the 4 cells]. Healthy cells are not pooled in (the beacon protocols are silent there; see the healthy rows above).

| N | ORACLE-IMPL | HEALTH-IMPL | HEALTH_PART:near-IMPL | HEALTH_ONE:low-IMPL | HEALTH_PART:near-HEALTH | HEALTH_ONE:low-HEALTH | recovery of ORACLE gain (PART) |
|---|---|---|---|---|---|---|---|
| 3 | +0.038 [+0.021,+0.054]* | +0.016 [-0.002,+0.032] | +0.026 [+0.010,+0.042]* | +0.020 [+0.008,+0.031]* | +0.011 [-0.002,+0.024] | +0.004 [-0.008,+0.016] | 0.70 |
| 5 | +0.032 [+0.012,+0.052]* | -0.000 [-0.019,+0.017] | +0.022 [+0.010,+0.037]* | +0.017 [+0.006,+0.029]* | +0.023 [+0.012,+0.034]* | +0.018 [+0.002,+0.031]* | 0.71 |
| 10 | +0.014 [-0.001,+0.029] | -0.040 [-0.057,-0.021]* | +0.009 [-0.001,+0.021] | +0.008 [+0.002,+0.015]* | +0.049 [+0.037,+0.062]* | +0.048 [+0.032,+0.064]* | 0.65 |

### Duplicated-effort diagnostic (perm1, target removed, full horizon)

Per-episode counts after the death, averaged over episodes and instances; per survivor = / (N-1). Pooled over travel {0,1} x link {ideal, burst30_d1} (4 cells, equal weight); orphans = the dead agent's nominal tasks with start >= death stamp.

**N = 3** (2 survivors; mean orphan tasks per episode = 8.18)

| metric (per survivor) | IMPLICIT | HEALTH | HEALTH_PART:near | HEALTH_ONE:low |
|---|---|---|---|---|
| decisions after death | 8.495 [8.263,8.699] | 8.389 [8.210,8.556] | 8.424 [8.252,8.596] | 8.451 [8.242,8.646] |
| dup looks (any) | 1.190 [1.075,1.321] | 2.001 [1.891,2.117] | 1.123 [1.032,1.232] | 1.137 [1.036,1.257] |
| looks on orphan (c,v) | 0.016 [0.005,0.028] | 3.023 [2.827,3.203] | 1.917 [1.797,2.040] | 1.460 [1.337,1.577] |
| dup looks on orphans | 0.000 [0.000,0.000] | 1.245 [1.137,1.352] | 0.191 [0.159,0.225] | 0.152 [0.130,0.176] |
| distinct orphans covered | 0.031 [0.010,0.056] | 3.555 [3.341,3.729] | 3.451 [3.250,3.656] | 2.617 [2.396,2.833] |
| own nominal tasks skipped | 1.171 [1.109,1.234] | 3.946 [3.764,4.135] | 2.974 [2.824,3.124] | 2.477 [2.349,2.614] |
| survivors rushing to orphan first | 0.008 [0.000,0.018] | 0.122 [0.100,0.146] | 0.059 [0.041,0.078] | 0.033 [0.016,0.049] |
| same-step orphan collisions | 0.000 [0.000,0.000] | 0.741 [0.658,0.824] | 0.004 [0.000,0.014] | 0.000 [0.000,0.000] |

Paired HEALTH - IMPLICIT per survivor: dup_post: +0.811 [+0.710,+0.911]*; orph_dup: +1.245 [+1.143,+1.349]*; own_missed: +2.775 [+2.589,+2.952]*

**N = 5** (4 survivors; mean orphan tasks per episode = 8.18)

| metric (per survivor) | IMPLICIT | HEALTH | HEALTH_PART:near | HEALTH_ONE:low |
|---|---|---|---|---|
| decisions after death | 8.529 [8.374,8.673] | 8.387 [8.256,8.517] | 8.480 [8.338,8.622] | 8.506 [8.360,8.642] |
| dup looks (any) | 1.153 [1.099,1.212] | 2.426 [2.330,2.524] | 1.137 [1.089,1.188] | 1.137 [1.086,1.189] |
| looks on orphan (c,v) | 0.008 [0.001,0.021] | 2.537 [2.418,2.652] | 0.997 [0.935,1.056] | 0.637 [0.581,0.689] |
| dup looks on orphans | 0.003 [0.000,0.008] | 1.598 [1.501,1.693] | 0.117 [0.095,0.138] | 0.073 [0.059,0.088] |
| distinct orphans covered | 0.023 [0.001,0.056] | 3.757 [3.602,3.878] | 3.518 [3.331,3.703] | 2.255 [2.068,2.428] |
| own nominal tasks skipped | 1.131 [1.087,1.178] | 3.510 [3.391,3.627] | 2.088 [2.013,2.162] | 1.704 [1.637,1.775] |
| survivors rushing to orphan first | 0.001 [0.000,0.004] | 0.103 [0.090,0.116] | 0.029 [0.022,0.036] | 0.012 [0.007,0.017] |
| same-step orphan collisions | 0.000 [0.000,0.000] | 2.637 [2.430,2.837] | 0.000 [0.000,0.000] | 0.000 [0.000,0.000] |

Paired HEALTH - IMPLICIT per survivor: dup_post: +1.273 [+1.196,+1.349]*; orph_dup: +1.595 [+1.504,+1.687]*; own_missed: +2.379 [+2.266,+2.489]*

**N = 10** (9 survivors; mean orphan tasks per episode = 10.88)

| metric (per survivor) | IMPLICIT | HEALTH | HEALTH_PART:near | HEALTH_ONE:low |
|---|---|---|---|---|
| decisions after death | 11.304 [11.126,11.473] | 10.997 [10.846,11.147] | 11.263 [11.095,11.427] | 11.266 [11.093,11.430] |
| dup looks (any) | 1.857 [1.739,1.986] | 4.028 [3.902,4.169] | 1.824 [1.718,1.945] | 1.837 [1.722,1.958] |
| looks on orphan (c,v) | 0.016 [0.011,0.022] | 3.592 [3.459,3.738] | 0.737 [0.708,0.767] | 0.341 [0.308,0.378] |
| dup looks on orphans | 0.002 [0.000,0.004] | 2.920 [2.800,3.051] | 0.094 [0.080,0.109] | 0.039 [0.031,0.047] |
| distinct orphans covered | 0.130 [0.089,0.177] | 6.046 [5.849,6.260] | 5.788 [5.603,5.983] | 2.719 [2.484,2.978] |
| own nominal tasks skipped | 1.502 [1.465,1.538] | 4.803 [4.661,4.962] | 2.199 [2.146,2.253] | 1.822 [1.778,1.864] |
| survivors rushing to orphan first | 0.004 [0.002,0.007] | 0.090 [0.081,0.099] | 0.018 [0.013,0.023] | 0.005 [0.002,0.009] |
| same-step orphan collisions | 0.000 [0.000,0.000] | 20.402 [19.423,21.364] | 0.000 [0.000,0.000] | 0.003 [0.000,0.010] |

Paired HEALTH - IMPLICIT per survivor: dup_post: +2.171 [+2.054,+2.283]*; orph_dup: +2.918 [+2.799,+3.047]*; own_missed: +3.300 [+3.172,+3.430]*



---

## 4. DEV full output (seeds 1000-1015; all variants; descriptive)
### DEV (seeds 1000-1015) per-cell results (lam=1, greedy)

P(detect) of IMPLICIT and paired differences vs IMPLICIT, mean [95% cluster-bootstrap CI]; * = CI excludes 0. p = unadjusted instance-level t.

| travel | fault | link | N | P IMPL | ORACLE-IMPL | HEALTH-IMPL | HEALTH_PART:near-IMPL | HEALTH_PART:nearbal-IMPL | HEALTH_PART:rr-IMPL | HEALTH_ONE:low-IMPL | HEALTH_ONE:near-IMPL | HEALTH_PART:near-HEALTH | HEALTH_PART:nearbal-HEALTH | HEALTH_PART:rr-HEALTH | HEALTH_ONE:low-HEALTH | HEALTH_ONE:near-HEALTH |
|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|
| 0 | perm1 | ideal | 3 | 0.533 | +0.049 [+0.015,+0.083]* | +0.015 [-0.026,+0.052] | +0.048 [+0.019,+0.077]* | +0.046 [+0.018,+0.075]* | +0.041 [+0.005,+0.075]* | +0.037 [+0.006,+0.069]* | +0.043 [+0.010,+0.072]* | +0.033 [+0.006,+0.062]* | +0.031 [+0.004,+0.060]* | +0.026 [+0.001,+0.051]* | +0.023 [-0.002,+0.049] | +0.028 [+0.006,+0.050]* |
| 0 | perm1 | ideal | 5 | 0.573 | +0.026 [-0.014,+0.069] | -0.032 [-0.069,+0.003] | +0.020 [-0.006,+0.045] | +0.018 [-0.008,+0.044] | +0.008 [-0.022,+0.039] | +0.013 [-0.011,+0.037] | +0.009 [-0.011,+0.032] | +0.052 [+0.025,+0.082]* | +0.050 [+0.024,+0.078]* | +0.041 [+0.018,+0.064]* | +0.045 [+0.021,+0.071]* | +0.042 [+0.014,+0.070]* |
| 0 | perm1 | ideal | 10 | 0.671 | +0.002 [-0.026,+0.031] | -0.089 [-0.125,-0.054]* | +0.013 [-0.008,+0.033] | +0.016 [-0.005,+0.036] | +0.013 [-0.007,+0.033] | -0.002 [-0.019,+0.017] | +0.001 [-0.018,+0.019] | +0.101 [+0.067,+0.134]* | +0.104 [+0.071,+0.139]* | +0.101 [+0.067,+0.138]* | +0.086 [+0.050,+0.124]* | +0.090 [+0.058,+0.122]* |
| 0 | perm1 | burst30_d1 | 3 | 0.533 | +0.049 [+0.015,+0.083]* | +0.022 [-0.009,+0.053] | +0.034 [+0.009,+0.060]* | +0.034 [+0.009,+0.060]* | +0.028 [-0.002,+0.057] | +0.028 [-0.001,+0.055] | +0.035 [+0.010,+0.058]* | +0.013 [-0.010,+0.036] | +0.013 [-0.010,+0.036] | +0.006 [-0.016,+0.026] | +0.006 [-0.011,+0.024] | +0.014 [-0.003,+0.031] |
| 0 | perm1 | burst30_d1 | 5 | 0.573 | +0.026 [-0.014,+0.069] | -0.006 [-0.035,+0.024] | +0.019 [-0.004,+0.042] | +0.017 [-0.007,+0.040] | +0.014 [-0.008,+0.036] | +0.015 [-0.004,+0.034] | +0.008 [-0.008,+0.028] | +0.025 [+0.001,+0.049]* | +0.023 [+0.000,+0.045] | +0.020 [+0.001,+0.039]* | +0.021 [+0.000,+0.045] | +0.015 [-0.007,+0.036] |
| 0 | perm1 | burst30_d1 | 10 | 0.671 | +0.002 [-0.026,+0.031] | -0.052 [-0.081,-0.025]* | +0.010 [-0.007,+0.029] | +0.011 [-0.007,+0.030] | +0.003 [-0.015,+0.022] | -0.001 [-0.016,+0.016] | +0.001 [-0.016,+0.017] | +0.062 [+0.035,+0.093]* | +0.064 [+0.037,+0.093]* | +0.055 [+0.028,+0.084]* | +0.051 [+0.024,+0.078]* | +0.053 [+0.029,+0.079]* |
| 1 | perm1 | ideal | 3 | 0.511 | +0.025 [-0.016,+0.069] | -0.010 [-0.053,+0.033] | +0.015 [-0.019,+0.049] | +0.016 [-0.019,+0.052] | +0.023 [-0.009,+0.057] | +0.018 [-0.017,+0.054] | +0.013 [-0.022,+0.048] | +0.025 [+0.000,+0.051] | +0.026 [+0.001,+0.052]* | +0.033 [+0.001,+0.065]* | +0.028 [+0.005,+0.049]* | +0.023 [-0.002,+0.046] |
| 1 | perm1 | ideal | 5 | 0.501 | +0.054 [+0.024,+0.084]* | +0.001 [-0.035,+0.039] | +0.025 [-0.003,+0.053] | +0.020 [-0.007,+0.047] | +0.017 [-0.004,+0.038] | +0.027 [+0.006,+0.049]* | +0.021 [-0.005,+0.047] | +0.024 [+0.002,+0.047]* | +0.019 [-0.007,+0.044] | +0.016 [-0.018,+0.046] | +0.026 [-0.005,+0.058] | +0.020 [-0.004,+0.047] |
| 1 | perm1 | ideal | 10 | 0.634 | -0.003 [-0.031,+0.026] | -0.047 [-0.074,-0.022]* | -0.003 [-0.021,+0.014] | +0.001 [-0.013,+0.016] | -0.002 [-0.019,+0.013] | +0.006 [-0.004,+0.018] | +0.003 [-0.008,+0.016] | +0.044 [+0.024,+0.065]* | +0.048 [+0.025,+0.072]* | +0.045 [+0.023,+0.069]* | +0.053 [+0.029,+0.079]* | +0.050 [+0.025,+0.075]* |
| 1 | perm1 | burst30_d1 | 3 | 0.511 | +0.025 [-0.016,+0.069] | +0.006 [-0.029,+0.045] | +0.014 [-0.013,+0.041] | +0.014 [-0.013,+0.041] | +0.020 [-0.005,+0.047] | +0.018 [-0.010,+0.045] | +0.007 [-0.016,+0.031] | +0.007 [-0.020,+0.030] | +0.007 [-0.018,+0.030] | +0.014 [-0.013,+0.039] | +0.011 [-0.009,+0.032] | +0.001 [-0.026,+0.025] |
| 1 | perm1 | burst30_d1 | 5 | 0.501 | +0.054 [+0.024,+0.084]* | +0.017 [-0.011,+0.046] | +0.022 [-0.002,+0.045] | +0.017 [-0.007,+0.040] | +0.015 [+0.000,+0.031] | +0.028 [+0.011,+0.048]* | +0.024 [+0.004,+0.044]* | +0.005 [-0.011,+0.023] | +0.000 [-0.020,+0.019] | -0.002 [-0.029,+0.022] | +0.011 [-0.011,+0.031] | +0.007 [-0.013,+0.027] |
| 1 | perm1 | burst30_d1 | 10 | 0.634 | -0.003 [-0.031,+0.026] | -0.029 [-0.054,-0.006]* | -0.001 [-0.017,+0.014] | -0.001 [-0.016,+0.014] | -0.002 [-0.016,+0.009] | +0.003 [-0.005,+0.013] | +0.000 [-0.010,+0.010] | +0.028 [+0.009,+0.048]* | +0.028 [+0.007,+0.049]* | +0.027 [+0.006,+0.050]* | +0.032 [+0.011,+0.056]* | +0.029 [+0.008,+0.051]* |

Healthy-cell check: 0/0 (cell, protocol) pairs have detections identical to IMPLICIT episode for episode (beacon silent when nobody dies).

### DEV (seeds 1000-1015) perm1 pooled over travel {0,1} x link {ideal, burst30_d1} (4 cells, equal weight)

Mean paired difference [95% bootstrap CI; instances resampled jointly across the 4 cells]. Healthy cells are not pooled in (the beacon protocols are silent there; see the healthy rows above).

| N | ORACLE-IMPL | HEALTH-IMPL | HEALTH_PART:near-IMPL | HEALTH_PART:nearbal-IMPL | HEALTH_PART:rr-IMPL | HEALTH_ONE:low-IMPL | HEALTH_ONE:near-IMPL | HEALTH_PART:near-HEALTH | HEALTH_PART:nearbal-HEALTH | HEALTH_PART:rr-HEALTH | HEALTH_ONE:low-HEALTH | HEALTH_ONE:near-HEALTH | recovery of ORACLE gain (PART) |
|---|---|---|---|---|---|---|---|---|---|---|---|---|---|
| 3 | +0.037 [+0.014,+0.059]* | +0.008 [-0.016,+0.031] | +0.028 [+0.010,+0.044]* | +0.027 [+0.010,+0.045]* | +0.028 [+0.010,+0.044]* | +0.025 [+0.008,+0.043]* | +0.024 [+0.008,+0.041]* | +0.020 [+0.005,+0.034]* | +0.019 [+0.005,+0.033]* | +0.020 [+0.005,+0.034]* | +0.017 [+0.004,+0.030]* | +0.016 [+0.004,+0.029]* | 0.75 |
| 5 | +0.040 [+0.018,+0.062]* | -0.005 [-0.024,+0.015] | +0.021 [+0.006,+0.036]* | +0.018 [+0.003,+0.033]* | +0.013 [-0.001,+0.028] | +0.021 [+0.009,+0.033]* | +0.016 [+0.003,+0.029]* | +0.027 [+0.014,+0.039]* | +0.023 [+0.010,+0.036]* | +0.018 [+0.003,+0.033]* | +0.026 [+0.013,+0.038]* | +0.021 [+0.008,+0.034]* | 0.53 |
| 10 | -0.001 [-0.016,+0.016] | -0.054 [-0.071,-0.037]* | +0.005 [-0.003,+0.013] | +0.007 [-0.002,+0.016] | +0.003 [-0.008,+0.013] | +0.002 [-0.008,+0.011] | +0.001 [-0.008,+0.010] | +0.059 [+0.045,+0.073]* | +0.061 [+0.046,+0.076]* | +0.057 [+0.040,+0.074]* | +0.056 [+0.041,+0.070]* | +0.055 [+0.040,+0.070]* | n/a (ORACLE gain ~0) |

### Duplicated-effort diagnostic (perm1, target removed, full horizon)

Per-episode counts after the death, averaged over episodes and instances; per survivor = / (N-1). Pooled over travel {0,1} x link {ideal, burst30_d1} (4 cells, equal weight); orphans = the dead agent's nominal tasks with start >= death stamp.

**N = 3** (2 survivors; mean orphan tasks per episode = 8.13)

| metric (per survivor) | IMPLICIT | HEALTH | HEALTH_PART:near | HEALTH_ONE:low |
|---|---|---|---|---|
| decisions after death | 8.510 [8.355,8.677] | 8.445 [8.282,8.618] | 8.468 [8.321,8.627] | 8.467 [8.299,8.641] |
| dup looks (any) | 1.154 [1.062,1.251] | 2.034 [1.949,2.121] | 1.087 [1.021,1.154] | 1.111 [1.033,1.191] |
| looks on orphan (c,v) | 0.034 [0.016,0.056] | 3.183 [3.039,3.329] | 1.839 [1.733,1.959] | 1.595 [1.482,1.705] |
| dup looks on orphans | 0.003 [0.000,0.007] | 1.293 [1.208,1.384] | 0.184 [0.149,0.221] | 0.168 [0.146,0.193] |
| distinct orphans covered | 0.062 [0.031,0.099] | 3.780 [3.616,3.953] | 3.310 [3.131,3.494] | 2.854 [2.648,3.051] |
| own nominal tasks skipped | 1.154 [1.085,1.217] | 3.990 [3.848,4.127] | 2.870 [2.764,2.986] | 2.540 [2.428,2.652] |
| survivors rushing to orphan first | 0.014 [0.005,0.025] | 0.128 [0.109,0.150] | 0.068 [0.054,0.084] | 0.037 [0.023,0.053] |
| same-step orphan collisions | 0.000 [0.000,0.000] | 0.784 [0.688,0.879] | 0.003 [0.000,0.010] | 0.000 [0.000,0.000] |

Paired HEALTH - IMPLICIT per survivor: dup_post: +0.880 [+0.790,+0.966]*; orph_dup: +1.290 [+1.202,+1.379]*; own_missed: +2.836 [+2.699,+2.964]*

**N = 5** (4 survivors; mean orphan tasks per episode = 8.04)

| metric (per survivor) | IMPLICIT | HEALTH | HEALTH_PART:near | HEALTH_ONE:low |
|---|---|---|---|---|
| decisions after death | 8.464 [8.308,8.617] | 8.327 [8.177,8.474] | 8.434 [8.284,8.575] | 8.436 [8.288,8.585] |
| dup looks (any) | 1.244 [1.161,1.338] | 2.514 [2.416,2.614] | 1.233 [1.159,1.319] | 1.220 [1.142,1.307] |
| looks on orphan (c,v) | 0.014 [0.002,0.036] | 2.616 [2.502,2.733] | 0.999 [0.935,1.068] | 0.664 [0.607,0.723] |
| dup looks on orphans | 0.003 [0.000,0.010] | 1.647 [1.553,1.745] | 0.123 [0.104,0.145] | 0.079 [0.064,0.096] |
| distinct orphans covered | 0.044 [0.008,0.105] | 3.878 [3.740,4.033] | 3.501 [3.302,3.728] | 2.341 [2.146,2.555] |
| own nominal tasks skipped | 1.124 [1.069,1.181] | 3.559 [3.428,3.693] | 2.077 [1.990,2.165] | 1.721 [1.642,1.797] |
| survivors rushing to orphan first | 0.003 [0.000,0.007] | 0.109 [0.092,0.129] | 0.027 [0.020,0.034] | 0.014 [0.006,0.027] |
| same-step orphan collisions | 0.000 [0.000,0.000] | 2.634 [2.417,2.861] | 0.000 [0.000,0.000] | 0.000 [0.000,0.000] |

Paired HEALTH - IMPLICIT per survivor: dup_post: +1.270 [+1.185,+1.363]*; orph_dup: +1.644 [+1.553,+1.742]*; own_missed: +2.434 [+2.324,+2.544]*

**N = 10** (9 survivors; mean orphan tasks per episode = 10.86)

| metric (per survivor) | IMPLICIT | HEALTH | HEALTH_PART:near | HEALTH_ONE:low |
|---|---|---|---|---|
| decisions after death | 11.320 [11.129,11.512] | 11.026 [10.878,11.193] | 11.272 [11.088,11.463] | 11.273 [11.083,11.462] |
| dup looks (any) | 2.045 [1.891,2.238] | 4.203 [4.064,4.344] | 1.999 [1.852,2.182] | 2.014 [1.868,2.188] |
| looks on orphan (c,v) | 0.023 [0.014,0.034] | 3.691 [3.540,3.830] | 0.757 [0.716,0.799] | 0.361 [0.335,0.390] |
| dup looks on orphans | 0.003 [0.001,0.005] | 3.016 [2.880,3.141] | 0.102 [0.086,0.119] | 0.049 [0.041,0.058] |
| distinct orphans covered | 0.180 [0.104,0.279] | 6.081 [5.887,6.289] | 5.895 [5.626,6.163] | 2.805 [2.607,3.009] |
| own nominal tasks skipped | 1.467 [1.439,1.502] | 4.853 [4.706,4.992] | 2.197 [2.138,2.256] | 1.795 [1.755,1.840] |
| survivors rushing to orphan first | 0.006 [0.003,0.009] | 0.089 [0.081,0.097] | 0.018 [0.014,0.022] | 0.006 [0.003,0.010] |
| same-step orphan collisions | 0.000 [0.000,0.000] | 20.845 [19.918,21.876] | 0.001 [0.000,0.007] | 0.000 [0.000,0.000] |

Paired HEALTH - IMPLICIT per survivor: dup_post: +2.158 [+2.010,+2.299]*; orph_dup: +3.013 [+2.881,+3.147]*; own_missed: +3.385 [+3.243,+3.519]*

