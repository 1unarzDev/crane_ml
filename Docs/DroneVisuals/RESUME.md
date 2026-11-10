# RESUME PLAYBOOK: restart every workstream from a fresh session

_Written 2026-11-10 before a usage cutoff. Read this first, then `STATUS.md` (drone sprint) and
`~/worktrees/crane-forest-sar/Tools/ForestSearch/STATUS.md` ([workshop] paper)._

## 0. Ground rules (must hold)

**Repos and branches** (pushes authorized for both):
- `~/worktrees/crane-forest-sar` on `research/forest-sar`: [workshop] paper (ForestSearch).
- `~/worktrees/crane-drones` on `research/drone-sim`: CRANE drone simulator and the SAR studies.
- MHSeals repos are READ-ONLY. Never push main. Never command hardware.

**Compute:** innovation is shared. The modular pilot (28 trials of about 40 min) is expected to end around 2026-11-11 04:00 CST; the cluster session will message. Until then: one pre-approved Unity job only, via `UNITY_SHARE_SLOT=1 UNITY_NICE=15 UNITY_MAX_LOAD=22 UNITY_TIMEOUT_S=1500 unity-drones --render ...` (the iter4 forest render batch). The modular pilot campaign was running about 14 h from ~01:00 on 11-10.
- **Unity:** only via `~/.cache/research/fs-scripts/unity-drones [--render] LOG ...`. It enforces a project flock, waits until no heavy unity slot is held machine-wide and MemAvailable ≥ 8 GiB, runs at nice 10, and does a post-grant race check. `/tmp/crane-drones-unity-hold`, if present, blocks it.
- **CPU:** `~/.cache/research/fs-scripts/fsrun CMD` (long jobs; slots reduced to 2 during the pilot; restore to 3 by editing `for i in 1 2` → `1 2 3`). `fsrun-quick CMD` is one extra slot, 10-minute timeout, short jobs only.
- **Python venvs:**
  - `~/.cache/research/fs-venv` (numpy, matplotlib)
  - `~/.cache/research/vision-venv`
  - TinyTeX: `~/.cache/research/tinytex/.TinyTeX/bin/x86_64-linux`
- **Agent rules** for all drone agents: `Docs/DroneVisuals/AGENT_RULES.md` (rules 1–10). Point every new agent at it.
- **Coordinate with the cluster session.** Peer `research-cluster-35`, reachable via ListAgents/SendMessage, owns queue policy and memory warnings. Check before raising load.

## 1. Paused processes (SIGSTOP; state kept). Resume ONLY after the pilot finishes or the cluster OKs it

```
kill -CONT 1339976   # RoleSpec dev explore (side grid; queue in Tools/RoleSpec/scratch/queue). Re-paused 11-10 09:42 at cluster request. Also restore fsrun to "1 2 3".
```

_Update 2026-11-10 (ops agent): PID 2096101 (NavPilot E3 held-out) is gone because the run finished: 800/800 rows, report at
`Tools/NavPilot/results/E3_heldout.md`. 2079546 was resumed by the orchestrator after the cluster session's OK and holds an fsrun slot._

If the PIDs are gone (reboot), restart from the scripts in section 3. The NavPilot E2 held-out (pid 2009703) was still running.

## 2. Scheduled checkpoints

Session cron jobs die with the session. Recreate them with CronCreate if needed. Sprint clock T0 = 2026-11-09 21:15 CST.

| Checkpoint | Time (CST) |
|---|---|
| h24 | 11-10 21:17 |
| h36 | 11-11 09:17 |
| h48 | 11-11 21:17 |
| h60 | 11-12 09:17 |
| h72 | 11-12 21:17 |

- **Novelty brief every 12 h:** write to `Docs/DroneVisuals/briefs/hNN.md`.
- **Demo pictures and study report every 24 h:** a third-person and an onboard still from the same seeded run, plus a depth/LiDAR overlay and an image review.
- **h72 deliverables:**
  - a polished demo sequence
  - a reproducible seeded flight (DONE: `Docs/DroneVisuals/flights/seed1337_run1`)
  - the autonomy design note (`AUTONOMY_DESIGN.md`)
  - [workshop]: PDF, source, figures, REPRODUCE.md, RESPONSE_LOG.md

## 3. Workstreams: state, next command, relaunch brief

### A. [workshop] paper (`crane-forest-sar`, `Tools/ForestSearch/`), DEADLINE Nov 20 AoE; the USER submits, never us
- **State:** submittable draft at commit f2baea9 or later. Build with `paper/build_pdflatex.sh`. Open objections are in `paper/RESPONSE_LOG.md`.
- **Pending:**
  - Scaling study (`nscale.py`, `PREREG_SCALING.md`, held-out seeds 3000–3015).
  - DRD/coordination-regret predictor (`predictor.py`, held-out seeds 4000–4015).
  - Check their files and results. Integrate only if they finish by about Nov 16 and survive a quick re-review; otherwise make them appendix or follow-up material.
- **Relaunch brief:** "Integrate results/nscale_heldout.md and results/predictor.md into paper/main.tex if they pass, keep body ≤8 pp, build clean, Type 1 fonts, update provenance.md and RESPONSE_LOG.md, then run one fresh reviewer and a cold reader on the abstract."
- **Later (post-deadline):** learned event-triggered comm vs beacon/periodic/DEVIATION at matched bytes; CRANE replication.

### B. CRANE autonomy (shared infrastructure, TOP PRIORITY): `Assets/Scripts/Drones/Autonomy`, `Docs/DroneVisuals/AUTONOMY_STATUS.md`
- **State:** M0–M5 done in the .NET harness; 0 collisions at ≤4 m/s (48 flights). The Unity batch was running; its results land in `/tmp/autonomy_unity/m5/`.
- **Collision cause** (from the harness): brake_infeasible.
- **Fix in progress by the lead:**
  - recursive feasibility: only accept a path from whose endpoint a safe stop is guaranteed
  - the fallback brake follows the last validated path
  - a turning brake table
  - reconcile the body radius: 0.268 m vs 0.2479 m
- **Paired sweep, ready to launch** (default-off fix vs legacy; ~1.5 h each, single core):
  - `dotnet $BIN/CraneExperiment.dll run --spec Tools/CraneHarness/specs/recfeas_sweep.json --out $OUT/recfeas_sweep`
  - `dotnet $BIN/CraneExperiment.dll run --spec Tools/CraneHarness/specs/recfeas_sweep_legacy.json --out $OUT/recfeas_sweep_legacy`
  - Then a Unity compile check of the new Core.
  - M5 (Unity) and M6 are DONE; stills are in Docs/DroneVisuals/autonomy_m6.
- **Next:**
  - re-run the speed sweep via the harness
  - M6 rendered flight
  - multi-forest validation: `BuildDroneForest.BuildManifestOnly` exists in the tree but has NEVER been run (no manifest produced by it); first run needs a Unity slot (pilot-gated)
  - PX4 SITL seam
  - two-drone missions
- **Relaunch brief:** "You lead CRANE autonomy. Read AUTONOMY_STATUS.md, HARNESS.md, AGENT_RULES.md. Implement the feasibility/fallback fix if not done, unit-test in the .NET harness (`Assets/Scripts/Drones/Autonomy/Harness~`), then re-run the speed sweep spec with CraneExperiment via fsrun, then M6 render via `unity-drones --render`. No commits; report."

### C. CRANE harness, DR and fidelity: `Assets/Scripts/Drones/Harness`, `Tools/CraneHarness`, `HARNESS.md`
- **Next:**
  - `fsrun Tools/CraneHarness/run_pending.sh` (~75 min): fidelity NavPilot-vs-CRANE, ablations, relabel
  - Unity compile check: `unity-drones /tmp/harness_compile.log -quit`

### D. Navigation research (NavPilot): `Tools/NavPilot`
- **E2:** held-out report: `fsrun python Tools/NavPilot/experiments/e2.py report --plan heldout` → `results/E2_heldout.md`. Do NOT re-run held-out.
- **E3 (max safe speed, `PREREG_SPEED.md` frozen):** **DONE** (800/800 rows; report `Tools/NavPilot/results/E3_heldout.md`). Still to do: fill the Q1–Q4 outcome rows in `Docs/DroneVisuals/mission/PREDICTIONS.md`.
- **Lead candidate to transfer to CRANE:** `pas_full`, i.e. trust free space for the braking stop only if confirmed at close range near structure.

### E. Mission SAR (two-drone find/verify/report): `Tools/MissionSim`, `Tools/MissionExact`
- **MissionSim:** DONE 2026-11-10: reports `Tools/MissionSim/results/{E1,E3,P7,ablations}.md` (val shards complete). Reference commands, only if a rerun is needed:
  - `fsrun $PY experiments.py report --exp E1,E3,ABL,P7 --split val`
  - `fsrun $PY experiments.py logs --seed 7` (decision logs for the demo)
  - If success ≤5%, tune difficulty on DEV first.
- **MissionExact:** check `Tools/MissionExact/results/MINIMAL_EXAMPLES.md`. If missing, relaunch with the brief "find minimal examples where coverage-optimal loses on time-to-confirmed-report; delay sweep where solo verify switches to partner verify."
- The calibrated observation model is in `Tools/Vision/obs_model.py` (`load(prefer_vision=True)`).

### F. Role specialization: `Tools/RoleSpec`, `PREREG_ROLES.md` (DRAFT)
- **Dev finding:** in the coverage-bound regime no specialization gain.
- **Orchestrator decisions before freeze** (at the end of PREREG_ROLES.md):
  - add verification-bound cells with the calibrated false-positive and decoy rates
  - run the side grid on DEV
  - freeze on held-out seeds 91000–91019
- Literature: `Docs/DroneVisuals/mission/LIT_ROLES.md` (claim = regime map, not "first adaptive team").

### G. Vision: `Tools/Vision`
- **Done:** RT-DETR-R18, NOMAD site-level held-out, obs_model, Orin estimate plus `orin_bench.py`.
- **Next:** run the 64 rendered-rerun jobs (`rerun_jobs.json`) after the demo logs the person's screen bbox and visibility per frame in `frames.jsonl` (~15 lines in `Assets/Scripts/Theater/Aerial/DemoSession.cs`).

### H. Visuals (lower priority than autonomy): `Assets/Drones/Environment`
- **State:** forest iteration 3 agent mid-run; its uncommitted edits are in Environment/**, Profiles and the scene.
- **Iteration 3 goals:** fix aerial far-terrain cliff streaks, speckled crowns from above, target-rig occlusion, grass, the dusk grade. `BuildDroneForest.BuildManifestOnly` was added (never run yet).
- **Relaunch brief if lost:** "Finish forest iteration 3 per Docs/DroneVisuals/STATUS.md: render seed 1337 rigs into environment/iter3 plus a second aerial angle, composites vs iter2, and run BuildManifestOnly once to test it. Rules: AGENT_RULES.md."
- **Deferred until after the pilot:**
  - TreeGen comparison render (command in `Docs/DroneVisuals/treegen/README.md`, density 0.5)
  - the full 69 s demo render
  - the two-drone decision-card demo from the MissionSim logs

## 4. Committing hygiene
- Commit only compiled, finished work. Never commit `.vscode/settings.json`, `Tools/NavPilot/results/raw`, `Tools/RoleSpec/scratch`, or `Assets/Drones/Environment/Generated` / `TreeGen/Imported` (gitignored).
- **research/forest-sar is published ONLY via `~/.cache/research/fs-scripts/publish-forest-sar --push`** (double-blind filter: no paper/title/venue; direct pushes are blocked by a dead pushRemote).
- **`git add` with a missing path aborts the whole add,** so add paths individually or check that they exist.
- Commit messages carry NO Claude attribution trailer (no Co-Authored-By / "Generated with"): the user asked on 11-10 (relayed by the DecisionCal session) to remove Claude attribution from crane_ml. A history rewrite is being coordinated by that session; do not rewrite or force-push yourself.

## 5. Frontier program (compute-efficient research, started 2026-11-10)

- Plan: `/home/lunarz/.claude/plans/steady-yawning-crayon.md` (section 1 = compute audit and revised queue; section 2 = `Tools/Frontier/`;
  section 3 = directions D1-D4 with 10 steps each).
- Working docs: `Docs/DroneVisuals/frontier/` (`OPS_LOG.md` = every ops job with command, times, cost, outputs; `SEEDS.md` = seed ledger).
- Costs: `Tools/Frontier/costs.jsonl` (`{sim, fidelity, config, seed, wall_s, cpu_s}`), results cache under `~/.cache/frontier/` (never /tmp).
- Ops tooling: `Tools/CraneHarness/run_pending_stages.sh "2 3"` runs chosen stages of `run_pending.sh` without editing it (outputs under
  `~/.cache/frontier/crane/harness`, binary `~/.cache/frontier/crane/bin`); `Tools/Frontier/ops/recfeas.py` makes staged recfeas specs and the
  paired analysis; `Tools/Frontier/canary/` is the GPU canary (spec only, NOT submitted; blocked on a user decision about a PyTorch backend).
- Compute: only fsrun/fsrun-quick, one job at a time per agent; no Unity until the pilot ends.

## 5. Post-pilot queue (decided 11-10 ~13:50; the pilot is expected to end around 11-11 04:00 CST, and research-cluster-35 will message)
Restore fsrun to 3 slots and resume RoleSpec (`kill -CONT 1339976`). Then run Unity one job at a time through `unity-drones`:
1. Forest iteration 5: `-executeMethod Crane.Drones.Environment.Iter4.IterFourRender.RenderBatchFocused`. The log goes to `~/.cache/drones-env/iter5_render.log` and the frames to `environment/iter5/`. It covers season-kit diagnostics, night, stars and the defect views.
2. AOS 7 V5 URDF import and rotor-spinner check, per `Tools/DroneCAD/aos7v5/UNITY_CHECK.md`.
3. The M6 autonomous-flight render, then the full 69 s demo, then 16 vision reruns.

.NET (fsrun):
- Recfeas fix ≤ 4 m/s regression check on the CAD profile.
- Recfeas fix vs legacy at 5–8 m/s on the CAD profile.
- If both are clean, switch `RecursiveFeasibility`/`BrakeAlongPath` to default-on.

Evidence so far: on v1, the fix had 2 collisions vs 14 for legacy across 45 pairs at 5/6/8 m/s (`frontier/recfeas/`).
