# CONTRACTS (immutable boundaries)

- Editable: `crane_ml`; CRANE-owned packages (`crane_nav`, `crane_hardware`, `crane_testing`) in a CRANE-owned workspace or `astro_dock` with provenance.
- `MHSeals/*` repos are read-only references: no writes, pushes, PRs, forks or mirrors. No exceptions for subagents.
- No pushes to any remote without explicit user authorization.
- Never command live hardware; use isolated ROS domains/simulated topics only.
- Never overwrite uncommitted user work, force-reset, or recursively update pinned submodules.
- Do not alter RoboBoat course geometry, water dynamics, boat coefficients, thruster mounts, or validated controller defaults silently; defects go to isolated fixtures/profiles.
- Preserve timestamp/episode provenance, fixed timestep, bounded queues, action freshness, reset isolation, replay reproducibility.
- HDRP water is rendering-coupled; no graphics-free aquatic claims without validation. `train-cpu` stays nonaquatic if code rejects water.
- Preserve licenses, third-party notices and attribution in any port.
- Evidence labels: IMPLEMENTED / FIXTURE_VALIDATED / LIVE_SIM_VALIDATED / PHYSICALLY_CALIBRATED / UNKNOWN / NOT_RUN.
- Research predictions and baselines: see PREREGISTRATION.md (DRAFT until frozen by user/orchestrator before comparative runs).
