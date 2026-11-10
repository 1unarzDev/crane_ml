# Drone visuals and fidelity: status record

_Started 2026-11-10. Branch `research/drone-sim` (from origin/main 0e337cd). Worktree ~/worktrees/crane-drones._

## Goal (user)
- Highly accurate, high-fidelity drone physics.
- Recorded demos.
- Iterate lighting, post-processing, drone models and demo visuals until they are at least RoboBoat quality.

## Rules
- **Unity:** only via `~/.cache/research/fs-scripts/unity-drones` (innovation's `heavy unity`; one Editor at a time, memory-capped, granted by the coordinator).
- **Renders:** demo renders run locally. The cluster theater-render backend is offline, and cluster unity players are prebuilt, so a new player needs the user's approval.
- **Commits:** the orchestrator commits; agents don't. Pushing this branch is authorized (user: "commit and push everything").
- **Licenses:** asset licenses are recorded per folder (CC0 / CC-BY only).

## Loop
Each iteration runs build agents (physics / model / environment / demo director). A visual critic then compares captures against the RoboBoat reference stills, and the next iteration targets the critic's top gaps. Physics changes must keep the parity tests against `Tools/Aerial/aerial_model.py` green.

## Iteration 1 (launched)
| Agent | Scope |
|---|---|
| D1 | Pipeline: import, EditMode tests, headless capture, RoboBoat reference media |
| D2 | Physics: aerial-v2 port, ≥1 kHz sub-stepped plant, AOS 7 V5 profile with provenance, controller, parity tests |
| D3 | AOS 7 V5 quad prefab: HDRP materials, prop blur, sensor frames |
| D4 | Drone Forest scene: lighting presets, post-processing, CC0 assets |

Next: D5 Theater aerial plug-in and demo director; D6 visual critic.

## 72-hour sprint (from 2026-11-10)
**Deliverables:**
1. One polished demo sequence.
2. One reproducible seeded flight with synchronized logs.
3. A short autonomy design note.

**Iteration protocol:**
- Fixed seed, the same scene each time.
- Render onboard (sensor profile) and third-person (cinematic profile).
- Record flight and sensor logs.
- The critic reviews renders and logs together.
- Fix the most consequential issue, then rerun the same seed to verify.

**Separation:** cinematic Volume and camera settings never touch the onboard sensor settings.

**Data:**
- Synchronized logs.
- Forest seeds split into train / val / held-out. Held-out layouts are never rendered during development.

**Agents added:**
- D5: Theater aerial plug-in and demo director.
- D6: synchronized logging, seeded flight and dataset splits.
- A1: autonomy design note (astro_dock read-only reference).
- T1: tree-growth generator, designed first, then implemented.

## Sprint addendum (user, 2026-11-09 21:29)
**Sprint clock:** starts 2026-11-09 21:15 CST.

**Checkpoints:** h12 = 11-10 09:17, h24 = 11-10 21:17, h36 = 11-11 09:17, h48 = 11-11 21:17, h60 = 11-12 09:17, h72 = 11-12 21:17. Session cron jobs are scheduled for each.

| Hours | Deliverable |
|---|---|
| 12, 24, 36, 48, 60, 72 | Novelty brief (briefs/hNN.md): closest competing result, strongest supportable claim, experiment most likely to overturn it. Covers the coordination paper (health-beacon robustness) and the navigation study (latency-aware speed vs foliage-aware sensing vs detection-aware search). |
| 24, 48, 72 | Demo pictures: a third-person and an onboard view from the same seeded run, plus a depth/LiDAR overlay. Image review covering scale, branch geometry, lighting, camera realism, and agreement with sensor and collision data. Study report with [workshop] and flight reported separately. |
| 72 | [workshop] final PDF, source, figures, reproducibility instructions, reviewer-response log. Submission is left to the user. |

**Navigation study approach:** explore broadly, then test selectively. Cheap pilots locate the conventional baseline's failures. Then choose between a learned proposal, a speed policy and a bounded residual, compared at matched observations, safety limits and compute. Whole forest layouts are held out.

**Agents added:**
- W1: health-beacon robustness: strong silent baselines; total, transient and timed loss; links.
- N1: navigation pilot harness plus baseline failure map.

**Queued:** idea agents after N1 reports; the [workshop] review round after writing pass 3.

## Mission-SAR addendum (user, 2026-11-10)
Two-drone find, verify and report under GPS denial, incomplete maps and radio outages. The full brief is in memory and summarized below.

**Workstreams:**
- M1: literature (primary sources)
- M2: exact small-graph search, verify and report
- M3: mission simulator with outage experiments 1 and 3
- M4: vision benchmark and calibrated observation model
- M5: venue fit ([workshop] / Robust AI / WoMAPF)
- Later: N2 navigation stack comparison (experiment 2) on N1's harness; demo replay with visible decisions

**Primary metric:** confirmed location delivered within deadline and tolerance.

**Predictions log:** Docs/DroneVisuals/mission/PREDICTIONS.md. Each cycle records what was predicted and what happened.

## Pending follow-ups (2026-11-09 23:20)
- **Compute contention:** innovation load is about 50 on 20 cores with 3 GB free (other sessions are running too). The 3-slot semaphore stays at 3; do not raise it.
- **M3 MissionSim:** E1/E3/ablation/P7 shards are queued (`fsrun ./run_all.sh {0,1,2} 3 val`).
  - After they finish:
    - `fsrun $PY experiments.py report --exp E1,E3,ABL,P7 --split val`
    - `fsrun $PY experiments.py logs --seed 7` (decision logs for the demo)
  - Risk: missions may be too hard (old smoke runs had 0–3 successes out of 20). The deadline is now 720 s and the battery 75 Wh, untested. If success stays at 5% or below, tune difficulty on DEV before drawing any conclusion.
- **N2 nav E2:** held-out spec frozen. 154/464 held-out episodes are done; the remaining 310 are queued (PID 1570417).
  - When `Tools/NavPilot/results/raw/e2_heldout_shard01.jsonl` has 310 rows: `fsrun python Tools/NavPilot/experiments/e2.py report --plan heldout` writes results/E2_heldout.md. Do NOT re-run held-out.
  - Also run the untested `experiments/test_e2.py` and the 17 NavPilot tests.
  - Provisional signal (n = 2–3): at 7 m/s every stack hits the S1 thin branch. The outcome hinges on first-detection range, not map trust, which refutes P9 as framed. P8 (3 m/s) is undecided.

## Redirect (user, 2026-11-10): navigation and CRANE autonomy (no agent sprawl; existing work redirected)
- **CRANE drone-autonomy workstream** (lead agent): shared infrastructure that the SAR papers depend on.
  - Milestones:
    - M0 architecture (in-process lockstep C# modules behind the AUTONOMY_DESIGN interfaces)
    - M1 sensors: depth, an L2-like LiDAR
    - M2 estimator with a drift model
    - M3 mapping
    - M4 port of NavPilot's MAP_RHP plus safety layer and REACTIVE
    - M5 ≥20 autonomous DEV flights plus a speed sweep
    - M6 a rendered autonomous flight for h24
  - End-to-end autonomy comes before ambitious comparisons.
- **Navigation research lead** (continues NavPilot/E2):
  - Primary outcome: MAX RELIABLY ACHIEVABLE SPEED at controlled collision risk, with bootstrap CIs, preregistered as PREREG_SPEED.md on fresh held-out layouts.
  - Literature review first (SUPER, Bubble, FASTER, EGO, Loquercio, Zhang NMI'25, pixel-motion, Falanga, safety filters, perception-aware planning, chance-constrained MPC).
  - Methods: stronger reactive control, latency-compensated MPC, uncertainty-aware map reliance, active perception (targeting first-detection range), and an optional learned policy with a backup safety filter.
  - Transfers the best method to CRANE once M5 holds.
- **E2 held-out:** still queued; the orchestrator reports it.

## Redirect (user, 2026-11-10): vision scope and CRANE as shared infrastructure
- **Vision (existing agent, re-scoped):** mission-relevant perception.
  - Sweeps: speed/blur, geometry, occlusion, resolution, compute contention.
  - Compare single-frame vs temporal vs multi-frame evidence accumulation vs adaptive inference.
  - Calibrated detection maps and coverage-confidence maps (feeding RoleSpec and MissionSim).
  - Scene-level held-out splits; Jetson estimate plus a ready-to-run Orin script.
- **CRANE autonomy/infrastructure team (high priority):**
  - The lead runs M0–M6, plus M2b (FC/PX4 seam), M4b (safety recovery), and the progression low-speed → high-speed → multi-drone.
  - After M0, ONE infrastructure agent joins for: the automated experiment harness (synced ground truth, sensors, decisions, comms events, failure replays), domain randomization, and fidelity validation against NumPy models.
- **Priority order:** autonomy and validation > the scientific studies > visuals. Visual work is held at forest iteration 3 plus the decision-card demo later.
- **Memory incident (2026-11-10 ~00:00):** the TreeGenCompare render (about 9 GB with import workers) stalled the modular pilot for 40 min and was stopped. Defer large renders (TreeGen comparison, the full 69 s demo) until the pilot finishes, about 15 h out; the cluster session will notify. Small Editor jobs that pass the gates are OK. A manual hold file, /tmp/crane-drones-unity-hold, now also blocks the wrapper if ever needed.
- **Vision results:** detector recall is low and strongly size/occlusion dependent; false alerts persist (decoys); repeat-view correlation is about 0.6, which supports [workshop]'s correlated-visibility model (to be cited as external support).
  - TODO for the next demo/autonomy agent: write the person's screen bbox and visibility per frame into the demo `frames.jsonl` (about 15 lines in DemoSession), so the 64 rendered-rerun jobs (Tools/Vision/rerun_jobs.json) can run after the pilot.
- **CPU incident (2026-11-10):** load ~40 on 20 threads slowed the modular pilot below RTF 0.5. For the pilot's duration (~14 h):
  - The fsrun semaphore is reduced to 2 slots; Unity runs at nice 10.
  - PAUSED with SIGSTOP; resume with `kill -CONT 2079546 2096101` after the pilot (2096872 had already exited; 2096101 = NavPilot e3, nav lead):
    - 2079546: RoleSpec explore
    - 2096872: CraneHarness attribution (dotnet)
  - Agents must not run heavy jobs outside fsrun.
- **Nav E3 (max safe speed):** prereg frozen (PREREG_SPEED.md, layouts 90400–90407). Held-out is paused at 58/800 (pid 2096101, SIGSTOP).
  - After the pilot: `kill -CONT 2096101` (about 85 min on one slot).
  - Then: `fsrun python Tools/NavPilot/experiments/e3.py report --plan heldout` writes results/E3_heldout.md. Fill the Q1–Q4 outcome rows in PREDICTIONS.md.
  - DEV signal: pas_full (trust free space for the braking stop only if confirmed at close range near structure) gave 0/16 collisions vs 4/16 for opt_backup_t.
  - Mechanism finding: thin branches enter the map at 8–19 m, then are erased by free-space clearing.
- **RoleSpec:** dev, partial (12/32 cells). Coverage-bound regime: no recon/verify arm beats the tuned shared-revisit baseline (N=2: −0.06; N=5: −0.11).
  - Paused (pid 2079546) with its queue (outage half, side cells, r-sweep; ~35 CPU-min).
  - After the pilot: resume, add verification-bound cells (calibrated FP/decoys), then freeze the prereg on held-out seeds 91000–91019.
- **CRANE harness:** committed. Collision cause for ≥5 m/s: brake_infeasible, 9/11 (planner fallback). Fix (recursive feasibility plus fallback following the last safe path) handed to the autonomy lead.
  - After the pilot:
    - `fsrun Tools/CraneHarness/run_pending.sh` (~75 min): fidelity sweep, ablations, refined relabel.
    - A Unity compile check, `unity-drones /tmp/harness_compile.log -quit`.
    - Add new forest seeds once BuildManifestOnly lands.
