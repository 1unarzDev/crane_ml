# cad_iter2 critique (payload deck added, tapered pitched props, first details)

Changes from iter 1: Jetson, L2, radio, USB-Eth, antennas moved to a 2 mm carbon payload deck on five gold
standoffs at z = 48 mm (above the prop plane); only Cube + Mini carrier stay on the frame upper plate inside the free
rhombus; props are tapered, pitched, CW/CCW mirrored; battery slid forward (x = +22 mm, limited by the OAK-D bracket) to
bring the CoM x to -2.8 mm; arm root moved back to (20, 14) so the OAK-D bracket clears the front arms.
Numbers: mass 1.843 kg, CoM (-2.8, 0, +17.3) mm, prop-disc clearance min +4.2 mm (all violations gone).

Remaining differences from the concept
1. Height: with the L2 (65 mm) on the payload deck the whole build is ~135 mm tall (battery bottom to antenna ~330 mm); the
   concept looks lower because it is an AI render that ignores the prop discs. Accepted: this is what the geometry forces.
2. Battery lead boot collides with the OAK-D housing (6.5 mm) and the power leads pierce the pack by 1.8 mm: move the boot to
   the rear face and stop the leads on the pack top.
3. Missing vs concept: black bells (they render as bare aluminium), battery straps around the pack, gold standoff
   spacers under Cube/Jetson, Jetson 40-pin header pins, camera-cage tabs, OAK lens bezels are flat, USB-Eth cable.
4. L2 is a square base plus cylinder; concept shows an upright pill. Keep (dimension-accurate 75 x 75 x 65 box envelope).
5. Plates are solid slabs (no honeycomb cut-outs); mass is rescaled to the datasheet 325 g anyway.
6. Render framing still leaves 35% empty border.
