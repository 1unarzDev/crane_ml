# Seed ledger (frontier studies)

_Written 2026-11-10 by the D4/framework agent from the PREREG files and code of Tools/* (this worktree) and
crane-forest-sar Tools/ForestSearch. Machine-checked by `Tools/Frontier/cache.py`: a `split="heldout"` call is refused
unless the sha256 of its `prereg=` file appears **on a line that also contains the file's name** in section 3 below._

**Namespaces.** Seeds are integers, but three seed spaces exist and must not be confused:
1. **Drone global splits** (`Tools/DroneData/splits.json`): dev 0-1999, train 2000-69999, val 70000-79999, test 80000-89999,
   heldout 90000-99999. Used by NavPilot (layout seeds), MissionSim (layout seeds), RoleSpec (layout seeds), the CRANE
   harness (forest and run seeds).
2. **ForestSearch grid-instance seeds** (`crane-forest-sar`, no global guard; guarded per study by preregistration text
   and `confirm_manifest.txt`/`nscale.py heldout` checks).
3. **Run seeds** inside a layout (NavPilot run seed = layout seed under CRN; CRANE harness run seeds 1, 2, ...).

## 1. Existing ranges

| Study | Namespace | DEV / tuning | Held-out / confirmatory | Status | Source |
|---|---|---|---|---|---|
| NavPilot E2 | 1 | layouts 100-101 (tune), seeds 0-5 (early sweep `dev_shard*`) | layouts 90200-90207 | held-out run, report written | Tools/NavPilot/results/E2_HELDOUT_SPEC.md |
| NavPilot E3 | 1 | layouts 300-301 (tune/smoke) | layouts 90400-90407 (PREDICTIONS says 90400-90409; 8 were run, 800 rows) | held-out run complete 800/800 | Tools/NavPilot/PREREG_SPEED.md |
| MissionSim E1/E3/P7/ablations | 1 | dev 0-1999 | none yet | val split 70000-70011 used (reports) | Tools/MissionSim/results/*.md |
| RoleSpec | 1 | seeds 0-199 | draft 5000-5019, recommended move to 91000-91019 | PREREG_ROLES.md is DRAFT, not frozen | Tools/RoleSpec/PREREG_ROLES.md |
| CRANE harness / autonomy M5 | 1, 3 | forest 1337, run seeds 1-2 | none | dev only | Docs/DroneVisuals/HARNESS.md, AUTONOMY_STATUS.md |
| ForestSearch confirmatory | 2 | 100-102 | 200-207 | frozen, run | PREREG.md |
| ForestSearch beacon_final / robustness | 2 | tuning 500-503 | fresh 800-811 (exploratory) | run | beacon_robustness.md |
| ForestSearch reviews, controls | 2 | - | 300-309, 504-511, 600-603, 700/701, 900-907 (exploratory, post hoc) | run | REVIEW_*.md, mechanism2.md |
| ForestSearch nagent pilot | 2 | 400-403 | - | run | STATUS.md |
| ForestSearch nscale | 2 | dev 1000-1003 | 3000-3015 | frozen, run | PREREG_SCALING.md |
| ForestSearch predictor | 2 | dev 1000-1011 | 4000-4015 | run | predictor.md |
| ForestSearch nbeacon | 2 | dev 1000-1015 | fresh 6000-6015 (refused without `nbeacon_frozen.json`) | code written, **held-out not run** (check) | nbeacon.py lines 28, 505-511 |

## 2. New held-out ranges (this program)

| Direction | Range | Namespace | Prereg file (to be frozen before the run) | Status |
|---|---|---|---|---|
| D1 learned communication | 5000-5031 | 2 (ForestSearch instances) | PREREG_LEARNEDCOMM.md | NOT FROZEN |
| D2 belief-space search | 92000-92031 | 1 (MissionSim layouts, inside heldout) | PREREG_BELIEF.md | NOT FROZEN |
| D3 compute-aware navigation (E4) | 90600-90615 (moved from 6000-6015, see item 1-2) | 1 (NavPilot forests) | PREREG_E4.md | NOT FROZEN |
| D4 multi-fidelity selection | none (retrospective, logged data only) | - | - | n/a |

## 3. Frozen prereg files (the sha256 list the cache guard reads)

Format: `| file name | sha256 | note |`. A frozen prereg is added here by the orchestrator at freeze time. The sha is of the
**file as it stood when it was added to this ledger**; later edits change the sha and the guard then refuses.

| File | sha256 | Note |
|---|---|---|
| PREREG_SPEED.md | efbe73b0d0d0c7717a151a2fbd54277aacfc47f448fe5908a7df6c93d39d460f | NavPilot E3, file sha 2026-11-10 (the in-file code sha 6df2c3cf... is a different quantity; deviations were appended after freeze) |
| E2_HELDOUT_SPEC.md | 119392866fd5c5625166c56a17d0de3238273f0f4f21f6c585581c8f8ff82dc0 | NavPilot E2 |
| PREREG.md | b7e856f02fcdc502f390bb0e5021b78e949035787f6435d2909bf0b5338ac20a | ForestSearch confirmatory (crane-forest-sar) |
| PREREG_SCALING.md | 52736413d41dd1e49ed7aeafde4421c55adca6bcac994b877843113ab58ed8a1 | ForestSearch nscale (crane-forest-sar) |
| PREREG_LEARNEDCOMM.md | PENDING | D1, not frozen |
| PREREG_BELIEF.md | PENDING | D2, not frozen |
| PREREG_E4.md | PENDING | D3, not frozen |
| PREREG_ROLES.md | PENDING | RoleSpec, DRAFT |

## 4. Overlaps and hazards (flagged)

1. **6000-6015 collision (numeric, cross-namespace).** D3's E4 held-out forests (NavPilot layouts 6000-6015) equal the
   ForestSearch nbeacon "fresh seeds 6000-6015" (grid instances). Different generators, so no data leaks, but a log
   line "seed 6003" is ambiguous. Prefer disambiguating by study name in every manifest. If nbeacon's held-out is later
   run, nothing in the two studies conflicts; it just must be labelled.
2. **6000-6015 and 5000-5031 sit in the global `train` split** (2000-69999), not in the held-out block. NavPilot's and
   MissionSim's guards (`splits.py`) will NOT refuse them, so for D3 the guard is only the prereg discipline
   (and `cache.py`). Same hazard that RoleSpec flagged for its draft 5000-5019. Recommendation: move D3 to
   a layout range inside 90000-99999 (e.g. 90600-90615) before freezing PREREG_E4.md. D1 is ForestSearch-namespace and
   unaffected by splits.json.
3. **D1 5000-5031 vs RoleSpec draft 5000-5019.** Numerically overlapping, different namespaces (ForestSearch instances vs
   RoleSpec layouts), and RoleSpec's recommended move to 91000-91019 removes the overlap. Do not freeze RoleSpec at 5000-5019.
4. **E3 range disagreement.** PREDICTIONS.md Q1 says E3 held-out 90400-90409; the run and the frozen PREREG use 90400-90407.
   90408-90409 were never run. Treat 90408-90409 as burned for new studies anyway.
5. **ForestSearch dev overlap (benign).** nscale dev 1000-1003, predictor dev 1000-1011 and nbeacon dev 1000-1015 share
   dev seeds; all are DEV and descriptive only.
6. **Disjoint from the new ranges (checked):** D2 92000-92031 does not touch NavPilot E2 (90200-90207), E3 (90400-90407), RoleSpec's
   91000-91019 or MissionSim val 70000-70011. D1 5000-5031 and D3 6000-6015 do not touch any ForestSearch range above except item 1.

## 5. Use by D4

D4 reads logged rows only. The NavPilot E3 held-out rows (layouts 90400-90407) enter the **retrospective** ranking-transfer
analysis, labelled `RETRO-HELDOUT`: no new held-out run happens and no tuning uses them; they cannot be treated as a fresh
held-out test of any selector.
