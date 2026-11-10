# OPS LOG (frontier compute queue, ops agent, 2026-11-10)

Rules in force: fsrun / fsrun-quick only, one fsrun job of mine at a time, no Unity, no GPU-queue submissions, never signal processes I did not
start (RoleSpec 2079546 untouched; the orchestrator resumed and later re-stopped it). All outputs under `~/.cache/frontier/crane/`.
Memory gate (`heavy status`) checked before each job: 10.7 GB and 13.3 GB available (>= 8 GB).
At 09:42 the orchestrator cut fsrun to ONE slot (plus fsrun-quick) because of the pilot load; my later job waits in the flock lottery behind
other agents' jobs (see "Queue state").

## 1. run_pending.sh stages 2 + 3 (NavPilot 2.5D twin, then CRANE side verification + fidelity report): DONE
- Command: `fsrun Tools/CraneHarness/run_pending_stages.sh "2 3"` (new wrapper; it extracts the build/header and the chosen stage blocks of the
  untouched `run_pending.sh`, redirects BIN and OUT to the cache, and swaps `navpilot_bridge.py` for `navpilot_bridge_timed.py`, a wrapper that
  adds `wall_s`/`cpu_s` per row and appends a cost row).
- Start 2026-11-10 09:28:42, stage 2 ended 09:35:04 (wall 370 s summed over runs, cpu 349 s, median 4.5 s/run), stage 3 ended 09:39:29. Total wall 10.8 min.
- Stage 3 = 6/6 lead rows bit-identical through the harness (`test_lead_swA.txt`, `test_lead_swB.txt`), then `fidelity.py`.
- Outputs (`~/.cache/frontier/crane/harness/`):
  - `navpilot_fidelity.jsonl`: 70 rows (cross_0..9 x speeds 2, 2.5, 3, 4, 5, 6, 8, seed 1): outcome, success, collision, min_clear, time_s, mean_speed, age p50/p99,
    margin_viol_frac, nofeasible_events, pair_key, cluster, wall_s, cpu_s.
  - `crane_lead_rows.jsonl`: the lead's .NET rows (ah_swA, ah_swB, ah_m5a); `fidelity_report.md`: paired report; `stands/`: exported stands.
  - Cost rows: `Tools/Frontier/costs.jsonl` (`kind: run`, sim `navpilot_2p5d_twin`, fidelity `low`).
- The twin cannot express the recfeas flags (RecursiveFeasibility, BrakeAlongPath, TurnBrakeTable are .NET `AutonomyConfig` only; NavPilot MAP_RHP has its own
  braking-feasibility check and no InflateExtra knob; `r_body` is both inflation and the collision truth, so it is not a clean shared axis).
  The twin's seed-2 legacy runs on cross_0-3 (12 runs) are in the recfeas driver below.

## 2. Reduced recfeas sweep (Tier A, orchestrator-approved): QUEUED
- Specs (derived from the originals, which are unchanged): `Tools/CraneHarness/specs/recfeas_{staged,legacy}_s1.json` (units 0-7) and `_s2.json`, `_s2_p1.json`
  (units 8-15), made by `Tools/Frontier/ops/recfeas.py gen`. Speeds 5, 6, 8 only. Unit k = (mission `[cross_0..9, corridor, corridor_rev][k % 12]`, run seed `1 + k // 12`);
  stages 1-2 = units 0-15 (the 14 cross units are the twin's units). Cell = speed. Stage 3 (32 seeds) is NOT approved, so no successive halving beyond stage 2;
  the per-cell halving rule is still evaluated and printed as information.
- Command: `fsrun python Tools/Frontier/ops/recfeas_driver.py`, which (a) runs the 12 twin seed-2 runs, (b) runs stages 1-2, fix then legacy, each
  through `timed.py` (wall, CPU, max RSS), (c) pairs by `pair_key` with a mission-cluster bootstrap (95% CI on the collision-rate difference fix minus legacy),
  (d) enforces a 1.3 core-h cap (4680 s CPU) by projecting each stage's cost, (e) runs `run_pending_stages.sh "1 5"` (12 counterfactual replays, attrib relabel;
  stage 4 dev_dr_demo deferred), (f) appends the results below via `oplog_append.py`.
- Outputs: `~/.cache/frontier/crane/recfeas/<spec name>/`, `paired_after_stage{1,2}.md`, `<spec name>.time.json`; the twin seed-2 rows in
  `~/.cache/frontier/crane/harness/navpilot_twin_seed2.jsonl`; costs in `Tools/Frontier/costs.jsonl` (`kind: run`, sim `crane_dotnet_harness`, fidelity `high`;
  `cpu_s` = per-run wall x the job's CPU/wall ratio).
- The first submission (stage 1 only, pid 1350720) never started and was cancelled by me (it was my own process) and replaced by the driver above.

## 3. run_pending.sh stages 1 and 5: QUEUED (inside the driver above; stage 4 deferred)
- Outputs under `~/.cache/frontier/crane/harness/`: `cf_*` dirs, `attrib2_v{5,6,8}`, `attrib2_summary.md`, `cf_*_summary.md`.

## 4. Docs
- `Docs/DroneVisuals/RESUME.md`: E3 held-out done (`Tools/NavPilot/results/E3_heldout.md`); PID 2096101 gone; MissionSim reports done
  (`Tools/MissionSim/results/{E1,E3,P7,ablations}.md`); `BuildManifestOnly` exists, never run; new section 5 "Frontier program".
- `Docs/DroneVisuals/HARNESS.md` line 105 corrected the same way.

## 5. GPU canary (spec only, NOT submitted): BLOCKED
- Files: `Tools/Frontier/canary/{canary_train.py, make_jobs.py, jobs.json}`. Two `rc-job/1` bodies (SPEC.md 2.1): job 1 trains a tiny MLP and stops at its 30 s
  checkpoint (`--stop-after-ckpt`, simulating preemption); job 2 `--resume`s from it and finishes the 60 s budget. Target host: harmony (<= 8 GB VRAM; the job
  asks for 1 GB). Submit command, once unblocked: `research submit -p drones Tools/Frontier/canary/jobs.json`.
- Local CPU check of the train/checkpoint/resume logic (fsrun-quick, 6 s budget, ckpt at 3 s): resumed from step 58, replay of the first resumed batch matches the
  checkpoint's recorded loss exactly (abs err 0.0), final loss below first, pass = true.
- BLOCKED on a user decision: no PyTorch training backend exists on any worker (offered kinds: fastsim, unity, llm-batch, ald-shard, command). `kind: command` is
  assumed in `jobs.json` and the worker tag `host:harmony` is an assumption; both need confirming. Also the checkpoint dir must persist between job 1 and job 2
  (same host / shared scratch); `$JOB_SCRATCH` is a placeholder.
