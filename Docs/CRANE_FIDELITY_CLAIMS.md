# CRANE-Fidelity claims register

This register separates supported statements from claims that remain blocked. It is updated
alongside evidence rather than inferred from simulator capability.

| Claim area | Supported now | Required before a physical claim |
| --- | --- | --- |
| Simulator repeatability | Nominal CAL-02 command response is repeatable across three independent Unity runs with exact parity residuals in retained reports. | None for this narrow simulation claim; it must not be relabeled as real-to-real repeatability. |
| Runtime timing provenance | Acquisition, completion, receive, and application metadata seams exist; async camera/delayed LiDAR tests pass; replay headers preserve seed and calibration-profile hashes. | Synchronized device clocks and measured publication, reception, and physical actuation events. |
| Plant/actuator calibration | Contract, fit-only profile artifact, split validator, runtime loader, and synthetic fixtures exist. | CRANE actuator commands, feedback, independent pose, repeated trials, and held-out trajectory/event residuals. |
| LiDAR/RGB/depth calibration | Engineering-prior LiDAR and geometric/rendered camera models remain available; public sensor audits exercise ingestion. | VLP-16 or equivalent and ZED 2i measurements with target/material/condition truth and held-out sensor-distribution residuals. |
| Detection calibration | Existing detector path is an explicit geometry/confidence heuristic. | Versioned detector outputs, annotations, confidence, false/missed detections, 3-D error, and inference timing. |
| Procedural generalization | Deterministic benchmark infrastructure and existing fixtures are available. | Topology/disjoint route and dynamic-event splits with equal-budget task results and invalid-layout accounting. |
| Sim-to-real transfer | No physical transfer improvement is claimed. | Paired closed-loop navigation or docking on the identified platform under held-out commands/routes/conditions. |

The intended scope is sufficient fidelity for local ROS 2 navigation, obstacle avoidance, recovery,
and docking under tested land or surface-water conditions. Universal high-fidelity or cross-domain
physical-equivalence claims are unsupported.
