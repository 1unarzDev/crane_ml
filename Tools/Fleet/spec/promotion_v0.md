# Platform ↔ fleet-game coupling: physical costs and scenario promotion (v0, 2026-11-10)

Unifying objective (user, 2026-11-10): capability-changing physical reconfiguration and cooperative specialization —
how modular robots form assemblies that unlock capabilities, adapt to changing task requirements, and decide when
cooperation is worth its physical and opportunity costs. Strategic coalition formation must be tested against physical
feasibility; CRANE-derived docking and transport costs must inform the higher-level planners.

## 1. Physical cost table (CRANE → fleet game, capability planner, policies)
File `physical_costs_<source>_<version>.json`, one per source (`fastmodel` now, `crane` once CRANE runs exist), each
entry keyed by (interface pair type, approach geometry class, assembly shape class, floor, payload class, failure state):
- docking: success probability per attempt (magnet capture / both hooks / false engagement), time distribution
  (quantiles), retries, energy; dependence on starting relative-pose uncertainty and feedback source.
- transport: speed and acceleration limits per assembly shape and payload; doorway transit time vs clearance;
  separation (undock) time; tracking error growth with module count.
- provenance: source, profile hash, CRANE commit, number of runs, seeds; and the fast-model value for comparison.
Consumers must read costs from this table (no hard-coded docking times) and record the table hash per run.

## 2. Scenario promotion (fleet game → CRANE)
A promoted case = a fleet-game scene instance + the decisions a policy made (assignments, coalitions, rendezvous cell,
face, shape, route) translated into a CRANE scenario (map in the shared scenario format, rovers from the shared
configuration generator, mission as a task list). CRANE executes it CLOSED-LOOP (no truth-pose steering) under the same
seed, and the comparison records: predicted vs executed feasibility (did the team form? dock? pass the doorway?),
predicted vs executed time and energy, and the first point of divergence with a failure replay.
Promotion is selective: only decisive cases (where the fleet-game decision depends on feasibility or on a cost near a
threshold) and a random sample for calibration.

## 3. Disagreement log
Every promoted case writes a row to `disagreements.jsonl` (scene, policy, predicted, executed, divergence step, cause
class: geometry / docking / traction / perception / planner / simulator artefact). Disagreement is a result, not noise.
