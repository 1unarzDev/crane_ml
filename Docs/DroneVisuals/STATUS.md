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
