# E2 held-out evaluation spec (FROZEN before the held-out run)

Status: **FROZEN** at 2026-11-09 22:41 local, before any held-out layout was generated, run or logged. The held-out plan runs
once. Any later change to code, parameters or analysis is reported as a deviation below, never applied silently.

Evidence label for all outcomes: IMPLEMENTED (2.5D point mass, synthetic sensors, synthetic forest and scenarios).
Nothing here is LIVE_SIM_VALIDATED or PHYSICALLY_CALIBRATED.

## Code and data
- Harness: `Tools/NavPilot` (navpilot/*, unchanged from N1) + `Tools/NavPilot/stacks/{core,policies}.py` +
  `Tools/NavPilot/experiments/{scenarios,e2_episode,e2}.py`. Code sha256 (concatenated, sorted paths): `d54f73ba5ba15aa1ac5b1180e28882508d4c271cd30cf45fcf606df41b45d804 (experiments/e2.py, experiments/e2_episode.py, experiments/scenarios.py, navpilot/__init__.py, navpilot/episode.py, navpilot/forest.py, navpilot/logio.py, navpilot/planner.py, navpilot/sensors.py, navpilot/vehicle.py, stacks/__init__.py, stacks/core.py, stacks/policies.py)`.
- Tuned parameters: `Tools/NavPilot/results/raw/e2_tuned.json` (sha256 `48074af7bb855a97a9d384a0db61628d5ba580af1c7199a6b8d5016b7f0e5ab9`), selected on DEV layouts 100-101
  only (layouts 100-101 x S1-S4 = 8 episodes per config; v 7 m/s, heavy tail, detector on). Equal budgets: 8 configurations x 8 episodes for HYBRID_CONF (defaults +
  seeded random search over 9 parameters) and for HYBRID_FIXED (grid: signal {conf, agree} x theta {0.35, 0.6} x
  v_fb {0.5, 0.8}). Objective per episode: 1[success] - 2 x 1[collision] - 0.005 x time_s, averaged.
  - HYBRID_CONF: `{} (empty = CONF_DEFAULT in stacks/policies.py: s_align 1.0, a0 0.35, a1 0.65, o0 0.4, k_loc 0.15, H_min 8, v_floor 0.35, mem_min 0.5 s, m_loc 0.4, age_growth 0.05)`
  - HYBRID_FIXED (global, B7c): `{"signal": "conf", "theta": 0.35, "v_fb": 0.5}`
  - HYBRID_FIXED per environment (B7c "tuned per environment", same 8 x 8 budget, best per family): `{"S1": {"signal": "agree", "theta": 0.6, "v_fb": 0.8}, "S2": {"signal": "agree", "theta": 0.6, "v_fb": 0.8}, "S3": {"signal": "conf", "theta": 0.6, "v_fb": 0.8}, "S4": {"signal": "conf", "theta": 0.6, "v_fb": 0.5}}; S0 (untuned control) uses the global switch`

## Layouts and pairing
- HELD-OUT layout seeds 90200-90207 (8 whole layouts; splits.json held-out range), `--allow-heldout`.
  Wind `light_breeze` (a DEV wind value: the held-out axis here is layout, not disturbance).
- Each layout yields S0-S4 by deterministic construction (`scenarios.build(seed, family)`). Run seed = layout seed for
  every stack and condition (paired, common random numbers per stream).

## Arms (plan `heldout` = `e2.plan_jobs("heldout")`, 8 layouts x 58 episodes = 464 episodes)
- A main: S0-S4 x {reactive, map_rhp, opt_backup, hybrid_conf, hybrid_fixed, hybrid_fixed_env}, plus map_rhp_n1
  (N1 safety layer reference) on S1 and S3; v 7, heavy compute tail, detector on (rho 0.2), link up.
- B speed sweep: S0, S1, S3 x v 3 x {map_rhp, opt_backup, hybrid_conf, hybrid_fixed_env} (v 7 from A).
- C detector/network: S1, S3 x map_rhp x {detector isolated, detector heavy, network loss}; hybrid_conf x detector heavy.
- D mission ablation: S0, S1, S3 x hybrid_conf x {uniform 4 m/s, detector-aware schedule beta 0.5} (fast = A).

## Primary endpoints and tests (decided now)
1. **Primary (sharpest claim candidate):** S3 (alignment drift) and S1 (stale shortcut), success and collision,
   HYBRID_CONF minus each of {map_rhp, opt_backup, hybrid_fixed, hybrid_fixed_env}. Paired by (layout, family), 95%
   cluster bootstrap over layouts (2000 resamples, seed 0). A "win" requires the CI to exclude 0; overlapping 0 = tie.
2. P8/T3: S0 at v 3 m/s, collision rate HYBRID_CONF vs hybrid_fixed_env: P8 holds if the CI covers 0 at 3 m/s AND a
   win (CI excludes 0 in hybrid_conf's favour on success or collision) appears in S1 or S3.
3. Secondary: S2 reactive vs map stacks (success, wasted travel); S4 collisions and contact speed; age p50/p95/p99 by
   detector condition; network-loss null for map_rhp; ablation contrasts (uniform4 - fast, detaware - fast,
   detaware - uniform4) on collision, near misses, recall, rep@20s, rep@30s, time.
- No multiplicity correction; the secondary endpoints are descriptive. With 8 layouts a 95% CI half-width on a
  rate difference is ~+-0.25-0.3: differences under ~25 pts will read as ties.

## Deviations
- None at freeze. (Compute: the machine was at load ~58 on 20 cores with the 3 shared fsrun slots contended, so the plan was cut before freezing to 8 held-out layouts, a 3 m/s-only speed arm, and HYBRID_CONF-only ablation; the separate DEV comparison run was skipped and the DEV tuning rows serve as the DEV check.)
- 2026-11-09 23:16 (after launch, execution only): shard partitions 0 and 1 were still queued for a shared fsrun slot,
  so they were re-queued as one process (`e2.py run --plan heldout --allow-heldout --shards 0,1 --nshards 3`) that
  runs the same jobs in layout order. Episodes are deterministic per job, so results do not change. Additions to
  `experiments/e2.py` after freeze: `--shards` (execution) and a report filter that uses only layouts with every
  planned job present (analysis guard against a partial run). Neither changes any episode. The code sha256 above
  predates these two additions.
- 2026-11-09 23:26, at the agent's 150-min cap: only partition 2 (154 of 464 jobs, about 1/3 of every layout) had
  finished. Partition 0+1 (310 jobs) was still waiting for a slot and remains queued, so the held-out run will finish
  unattended under this frozen spec. No layout was complete, so the paired cluster-bootstrap endpoints are NOT_RUN at
  the cap. Complete them with `fsrun python Tools/NavPilot/experiments/e2.py report --plan heldout`
  (-> results/E2_heldout.md) once `results/raw/e2_heldout_shard01.jsonl` has 310 rows. Do not re-run held-out.
