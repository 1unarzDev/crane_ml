## Modular fleet (CRANE)

These figures cover the modular ground fleet: CAD v1 rovers, physical-load runs in CRANE, findings and test scenes. **Simulation only.** CAD is at concept/datasheet level. Frame notes and critiques are in [`Docs/Fleet/media/README.md`](../../tree/research/fleet/Docs/Fleet/media) on `research/fleet`.

<table>
  <tr>
    <td width="33%"><img src="Docs/Fleet/media/cad_v1/compare_lifter.jpg" alt="Lifter concept beside CAD v1"><br><sub><b>Design.</b> CAD v1 lifter: concept (left) and parametric CAD (right). 342 × 260 mm chassis, mecanum wheels, reciprocal docks (D87).</sub></td>
    <td width="33%"><img src="Docs/Fleet/media/cad_v1/lifter_places_box_small_on_cargo.jpg" alt="Lifter placing box_small on a cargo deck"><br><sub><b>Design.</b> 170 mm lift sets the 2 kg box side-on onto a cargo deck. Static CAD check; rating 2.0 kg at SF 1.51 (D90).</sub></td>
    <td width="33%"><img src="Docs/Fleet/media/runs/box_long6kg_seed1_t68.8_capacity_crop.jpg" alt="Lifter refusing an over-capacity box"><br><sub><b>Run.</b> 6 kg box over the 5 kg rating: the lift stalls, the library refuses with Capacity and returns the box (load4_box_long6_1, seed 1; D82).</sub></td>
  </tr>
  <tr>
    <td><img src="Docs/Fleet/media/findings/crane_load_batches.jpg" alt="CRANE physical-load batch outcomes"><br><sub><b>Finding.</b> Physical load: box_small 8/10 delivered; 6 kg refused 10/10; 5 kg end-gripped tips 3/3 (D82, D86).</sub></td>
    <td><img src="Docs/Fleet/media/findings/robust_feasibility_info.jpg" alt="Effect of feasibility information"><br><sub><b>Finding.</b> Feasibility information helps larger held-out fleets once the action set is fixed (D84, revising D72).</sub></td>
    <td><img src="Docs/Fleet/media/scenes/scenes_v1_index.jpg" alt="Nine modular-robotics test scene families"><br><sub><b>Scenes.</b> Scenes v1: nine test families (narrow door, lateral slot, wheel failure, ...), fastsim top-down previews (D85).</sub></td>
  </tr>
</table>

**Latest findings (simulation only):**
- **CRANE physical load (D82, D86).** Results:
  - box_small (2 kg, within rating) delivered 8/10 (Wilson 95 % 0.49–0.94);
  - box_long at 6 kg was refused 10/10 with an explicit Capacity failure;
  - end-gripped box_long at 5 kg tipped 3/3.

  There were 0 false successes. The earlier 10/10 (D77) used a kinematic carry and is not physical evidence.
- **Capability information (D84 revises D72).** Feasibility information raises servable delivery on larger held-out fleets: +0.108 [0.067, 0.154] at 4 jobs and +0.040 [0.022, 0.058] at 16. The earlier null came from the action set. On train-like families it is still null.
- **Fault identification (D83).** D74's kill of the belief-space planner is suspended because of an adapter bug. In the E2 rerun with the executor fixed, the parameter oracle does not beat nominal: −0.035 [−0.081, +0.012].
- **Execution-aware MAPF (D81, exploratory).** 14 of 15 UD-TAPF plans were invalid because of docking constraints. At high drift, failures trace to tracking and localisation (60 % each).
- **CAD v1 (D87, D89, D90).** The lifter stroke is now 170 mm, with 0.24 m jaws, rated 2.0 kg at SF 1.51 (under 1 % margin). It sets loads side-on only.
