## Modular fleet (CRANE)

These pictures show the modular mecanum ground fleet in CRANE: the CAD v1 rover design, physical-load runs and the modular-robotics test scenes. **Simulation only;** the CAD is at concept/datasheet level, not measured. Frame notes are in [`Docs/Fleet/media/README.md`](Docs/Fleet/media/README.md).

<table>
  <tr>
    <td width="33%"><img src="Docs/Fleet/media/cad_v1/compare_scout.jpg" alt="Scout concept beside CAD v1"><br><sub><b>Design.</b> Scout: concept (left) and parametric CAD v1 (right), with a lidar, a depth camera and a proxy arm.</sub></td>
    <td width="33%"><img src="Docs/Fleet/media/cad_v1/compare_lifter.jpg" alt="Lifter concept beside CAD v1"><br><sub><b>Design.</b> Lifter: 342 × 260 mm chassis, mecanum wheels, reciprocal hook-and-pin docks.</sub></td>
    <td width="33%"><img src="Docs/Fleet/media/cad_v1/compare_cargo.jpg" alt="Cargo concept beside CAD v1"><br><sub><b>Design.</b> Cargo rover with a raised deck on standoffs.</sub></td>
  </tr>
  <tr>
    <td><img src="Docs/Fleet/media/cad_v1/lifter_places_box_small_on_cargo.jpg" alt="Lifter placing a box on a cargo deck"><br><sub><b>Design.</b> The 170 mm lift sets a 2 kg box side-on onto a cargo deck (static CAD check).</sub></td>
    <td><img src="Docs/Fleet/media/runs/box_long6kg_seed1_t68.8_capacity_crop.jpg" alt="Lifter refusing an over-capacity box"><br><sub><b>Run.</b> A box over the lift rating: the lift stalls, the autonomy refuses it as over capacity and returns it.</sub></td>
    <td><img src="Docs/Fleet/media/scenes/scenes_v1_index.jpg" alt="Modular-robotics test scene families"><br><sub><b>Scenes.</b> Test scene families (narrow doors, a strafe-only shelf slot, wheel failure, ...) that compile to fastsim and CRANE.</sub></td>
  </tr>
</table>

