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

**Finding for D4 (orchestrator decision 2026-11-10):** the twin cannot express the recfeas config axis, so the cheap simulator cannot screen this fix. NavPilot is not
modified (E2/E3 frozen). The D4 readout is (a) the level gap twin-legacy vs .NET-legacy per speed on matched units, with seed-2 twin units as a noise ceiling, and
(b) the .NET recfeas minus legacy effect per speed with CIs. The driver is kept as ONE fsrun job (orchestrator decision).

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

## Profile pin (orchestrator FYI, 2026-11-10)
The live AOS7 profile became the CAD one (1.843 kg); the specs for this queue are pinned to `Tools/Aerial/profiles/aos7_v5_f90_6s_v1.json`. Checked that
`recfeas.py gen` copies the pin into all generated `recfeas_{staged,legacy}_s1/_s2/_s2_p1` specs (verified by grep). A CAD-profile rerun of the >= 5 m/s sweep follows after the pilot.

## Auto-appended by Tools/Frontier/ops/oplog_append.py (end of the recfeas driver job)

- `pending_1_5`: start 2026-11-10 13:02:31, end 2026-11-10 13:14:15, wall 704 s, cpu nan s, max RSS nan MB, rc 0; out `/home/lunarz/.cache/frontier/crane/recfeas/pending_1_5/`
- `recfeas_legacy_s1`: start 2026-11-10 12:47:19, end 2026-11-10 12:53:04, wall 345 s, cpu 342 s, max RSS 397 MB, rc 0; out `/home/lunarz/.cache/frontier/crane/recfeas/recfeas_legacy_s1/`
- `recfeas_legacy_s2`: start 2026-11-10 12:57:26, end 2026-11-10 12:59:18, wall 111 s, cpu 111 s, max RSS 359 MB, rc 0; out `/home/lunarz/.cache/frontier/crane/recfeas/recfeas_legacy_s2/`
- `recfeas_legacy_s2_p1`: start 2026-11-10 12:59:18, end 2026-11-10 13:02:31, wall 193 s, cpu 192 s, max RSS 402 MB, rc 0; out `/home/lunarz/.cache/frontier/crane/recfeas/recfeas_legacy_s2_p1/`
- `recfeas_staged_s1`: start 2026-11-10 12:42:54, end 2026-11-10 12:47:19, wall 265 s, cpu 263 s, max RSS 430 MB, rc 0; out `/home/lunarz/.cache/frontier/crane/recfeas/recfeas_staged_s1/`
- `recfeas_staged_s2`: start 2026-11-10 12:53:05, end 2026-11-10 12:54:55, wall 110 s, cpu 110 s, max RSS 408 MB, rc 0; out `/home/lunarz/.cache/frontier/crane/recfeas/recfeas_staged_s2/`
- `recfeas_staged_s2_p1`: start 2026-11-10 12:54:55, end 2026-11-10 12:57:26, wall 151 s, cpu 151 s, max RSS 410 MB, rc 0; out `/home/lunarz/.cache/frontier/crane/recfeas/recfeas_staged_s2_p1/`

### paired_after_stage1.md

| speed | pairs | fix coll | legacy coll | diff (pp) | 95% CI (pp) | fix succ | legacy succ | decision |
|---|---|---|---|---|---|---|---|---|
| 5 | 8 | 0 | 1 | -12.5 | [-37.5, +0.0] | 8 | 6 | CONTINUE |
| 6 | 8 | 0 | 3 | -37.5 | [-75.0, +0.0] | 8 | 4 | CONTINUE |
| 8 | 8 | 1 | 3 | -25.0 | [-75.0, +25.0] | 7 | 5 | CONTINUE |


### paired_after_stage2.md

| speed | pairs | fix coll | legacy coll | diff (pp) | 95% CI (pp) | fix succ | legacy succ | decision |
|---|---|---|---|---|---|---|---|---|
| 5 | 15 | 0 | 2 | -13.3 | [-33.3, +0.0] | 15 | 12 | CONTINUE |
| 6 | 15 | 1 | 6 | -33.3 | [-66.7, +0.0] | 14 | 8 | CONTINUE |
| 8 | 15 | 1 | 6 | -33.3 | [-61.5, +0.0] | 14 | 9 | CONTINUE |


### D4 readout (a): level gap, twin-legacy minus .NET-legacy collision rate, matched cross units (mission-cluster bootstrap 95% CI)

| speed | matched units | twin coll | .NET coll | gap (pp) | 95% CI (pp) | twin seed1-vs-seed2 collision disagreements (cross_0-3) |
|---|---|---|---|---|---|---|
| 5 | 14 | 10 | 1 | +64.3 | [+20.0, +100.0] | 0 of 4 |
| 6 | 14 | 12 | 5 | +50.0 | [+21.4, +80.0] | 0 of 4 |
| 8 | 14 | 9 | 6 | +21.4 | [-21.4, +68.8] | 1 of 4 |

Readout (b) = the paired_after_stage tables above (.NET recfeas minus legacy).
