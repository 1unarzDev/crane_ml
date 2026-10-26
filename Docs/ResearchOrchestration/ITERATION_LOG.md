# ITERATION LOG

## I1 — 2026-10-20 — Discovery
Repo state at start:
- crane_ml main @ 976ff1e (12 modified, many untracked fidelity docs/tools, all user work and left untouched). Worktrees: crane_ml-roboboat-pwm (roboboat-thruster-pwm @ bf82744), worktrees/crane-characterization (test/characterization-faults @ dccb3e6), plus several worktrees/roboboat-* worktrees.
- astro_dock main @ f6afe37 (devcontainer/README modified). Submodule src/ros_tcp_endpoint @ 3c3d405.
- astro_dock/src: mhseals_nav a73098e, mhseals_hardware 26c59f7 (modified), mhseals_testing f3f5d95 (modified). All have MHSeals origins and are READ-ONLY.
- Characterization fault worktrees: worktrees/mhseals_{hardware,nav,testing}-characterization (testing on test/characterization-faults @ a645657; nav @ 04a9f4a detached; hardware @ 26c59f7 detached).
- Claude Code 2.1.306; no prior .claude/ config in crane_ml.
Delegated in parallel: truth inventory, literature/novelty, adversarial test design.
- Adversarial design returned (≈55 tests, 12 mutants). Proposed `Tools/Adversarial/`. Holding implementation until truth inventory lands, because its model constants and defects may change the fixture model.
- Literature returned; drafted PREREGISTRATION.md. Awaiting truth inventory.
- Truth inventory returned (20 defects, 10 hypotheses). Launched independent defect verifier + hydro-strategy council. Sent production-defect mutants (MUT-13..16) to the adversarial implementer.
- crane_hardware ported and re-verified by the orchestrator. Builds use astro:core-local (jazzy), because clock-local has no ROS. Next: domain guard + crane_nav.
- Hydro council returned option D (frozen legacy + flagged hull-am-v1, plus a separate reference diagnosis). Waiting on the defect verifier before acting.
- Verifier confirmed V1–V5. D8 is final. Baseline doc corrected. Underwater specialist told that Robosub Pool uses defective GeneralDynamics.
- crane_nav ported. The orchestrator found 12 task-test regressions vs upstream (only 1 upstream failure) and returned them to the implementer.
- C# invariants executed in Unity batchmode, so V1–V3 now reproduce in C#. The planar Coriolis-power test passes only by construction (weak) and is queued for verifier review.
- Reference diagnosis: noise-level failure, criterion retired (D10). The live N≥10 repeats are NOT_RUN and queued for Phase E. crane_nav: import regression fixed; 2 source-mode launch tests returned.
- crane_ros gate green (supersedes E14/E18). The crane_testing port is deferred until R17 (decoupling fitter) is reproduced in Tools/Adversarial.
- Terrain verified (2 confirmed, 2 partial). Fixes sent to the terrain agent. Rover Apache-2.0 notice + license added.
- Underwater verified. Viscosity bug is shared with the surface boat GeneralDynamics path (main only), added to the D8 hull-am-v1 gate list.
- Adversarial review: fixes dispatched to the suite author and the C# invariant author; R17 fitter-design council launched (Tools/Adversarial/decoupling/).
- Underwater: quaternion fixed, tank PlayMode passes. Next underwater: CG-offset restoring case, sensors (pressure/DVL), and independent review.
- matrix.json gains DEC-01..20 (31 decoupling tests, marked slow). Fast suite: 78 pass/1 xfail.
- Memo review MAJOR. The orchestrator fixed the PREREGISTRATION Kadian misattribution and the E47 p-values. D22 re-scopes the novelty gap. Memo revision and exact-MW/MF-BO competence tests dispatched.
- I1 closing report written (REPORT_I1.md). All offline suites green on orchestrator re-runs. Remaining I1 item: C# weak energy-test demotion (agent still running).
