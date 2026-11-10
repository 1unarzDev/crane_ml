# Tools/RoleSpec: controlled simulator for recon/verify role specialisation in multi-drone forest SAR

Evidence label: **IMPLEMENTED** (DEV phase). This is a synthetic, controlled mission-level model. Nothing here is
LIVE_SIM_VALIDATED or PHYSICALLY_CALIBRATED, and no held-out run has been made. The draft preregistration is in
`PREREG_ROLES.md`; the literature context is in `Docs/DroneVisuals/mission/LIT_ROLES.md`.

| Module | Content |
|---|---|
| `rolespec/world.py` | `Scenario` knobs: N, area per drone, occlusion, trade-off strength, comms, clutter, FP rate, switch penalty, deadline, battery. A seeded layout holds the canopy field, prior heatmap, M ~ U{1,2,3} survivors drawn from the prior, per-(cell, view) person-visibility latents and decoys, common-random-number outcome uniforms keyed by (cell, view, k-th look), and per-pair Gilbert-Elliott channels (ForestSearch `links.GE` parametrisation) with range limits. Split guard: `Tools/DroneData/splits.py`, plus the RoleSpec held-out range 5000-5019. |
| `rolespec/sensing.py` | Per-pass outcome tables {none, low, high} for 6 latent states (person good/bad view x decoy). P(view good) comes from the MissionSim NavPilot sight-line table. P(detect \| good) at v_slow comes from the calibrated `Tools/Vision/obs_model.py`. The fast pass uses exp(-kappa) (none / weak / strong) or the calibrated model at v_fast. |
| `rolespec/belief.py` | Revisit map: per-(cell, view) sufficient statistic of log-likelihoods for each latent state, giving the cell posterior and the per-view posterior over visibility and decoy state (the "informativeness of past passes"). Knowledge is prefix watermarks of each origin's log, which gives idempotent store-and-forward merging. |
| `rolespec/sim.py` | Event-driven tasks and 5 s comms slots: flooding within connected components, a no-a2a mode for B7, rendezvous/courier reporting, energy and RTB, and the switch penalty. Metrics and the per-mission failure label. |
| `rolespec/policies.py` | One shared scorer (expected high-alert mass plus alpha x P(cross tau), per unit time, with intent-claim deconfliction and next-best-view over 4 view classes = B9). Arms: HOMO_FAST (a), HOMO_THOROUGH (b), HOMO_SHARED_REVISIT (c, B2), SPEC_FIXED(r) (d, B4), SPEC_ADAPTIVE (e, B5 threshold or marginal), B3_JIN_CONFIRM, B6_AUCTION, B7_SOLO, ORACLE (labelled). |
| `experiments.py` | `tune` (equal budget K = 10 configurations per family), `explore` (grids `main`, `side`, `ratio`), `report` (paired cluster bootstrap, Holm, EXPLORATORY). |
| `tests/test_rolespec.py` | Determinism, budget matching, no truth in planner input, no future information (perturbing future outcomes and channels leaves earlier decisions unchanged), equivalences (SPEC_FIXED r = 1 with verify-by-self == (c); SPEC_ADAPTIVE marginal, h = 0, broad roles == (c)), split guard, ideal-comms reporting. |

```
F=/home/lunarz/.cache/research/fs-scripts/fsrun; PY=/home/lunarz/.cache/research/fs-venv/bin/python
cd Tools/RoleSpec
$F $PY -m unittest discover -s tests -p 'test_*.py'
$F $PY experiments.py tune --seeds 0:32
$F $PY experiments.py explore --grid main --seeds 100:108 --out results/explore_main_s1.jsonl --shard 1 --nshard 2
$F $PY experiments.py report --inp results/explore_main_s1.jsonl --fail
```

Scheduling helper (dev only): `scratch/worker.sh` holds one fsrun slot and runs the jobs in `scratch/queue/` in
order. Outputs go to `scratch/done/`.

DEV status (2026-11-10): 10/10 tests pass. Equal-budget tuning is done (`results/tuned.json`; the per-config
logs are `results/tuned_a2.json`, `results/tuned_b2.json` and `results/tuned_a.json` for B7). The exploration run is
PAUSED by the orchestrator after 12 of 32 main cells (`results/explore_main_s1.jsonl`). The queue still holds
the outage half (`04_explore_main_2`), `05_explore_side` and `06_explore_ratio`. Readout:
`python dev_hypotheses.py --main results/explore_main_s1.jsonl,results/explore_main_s2.jsonl --side
results/explore_side.jsonl --ratio results/explore_ratio.jsonl`.
