# cad_iter4: first real-parts assembly (pass 1 of 3)

Built from: Luxonis OAK-D S2 enclosure STEP (MIT) as the camera mesh; Cube / Mini carrier STEP-measured dimensions and hole
pattern; Unitree L2 manual p.8 (24 mm base, 44.5 mm scan centre, 4 x M3 on a 51 mm circle); Holybro ESC spec (30.5 x 30.5 holes);
AOS 7 V5 vendor figures (250 g, 19 x 19 motor pattern, stacks); real standoffs (22 mm stack, 20 mm and 2 x 22 mm deck, 5 mm Cube,
10 mm Jetson, 6 mm radio). Renders: iso, top, front, side, closeup (stack and deck), deck_top (deck + payload only).

Issues found by the pass (each fixed in the same pass or noted for 5/6)
Orientation
1. PROPELLERS: the previous model's blade chord was mirrored, so the pitched leading edge pointed down (blades would push air up).
   Fixed in `blade_solid`: for CCW the leading edge is +y and pitched up, CW is the mirror; `test_model.py` now checks LE z > TE z per rotor.
2. Jetson carrier modelled 100 mm along x (it is 100 mm across the vehicle with the long edge on y): dimensions transposed. Fixed; ports now
   on the aft edge (reachable from behind, next to the adapter), 40-pin header on the forward edge.
3. Cube: arrow on the lid points +x (forward), JST strip on the +y (left) side. OK. L2 LED on the front right. OK.
4. OAK-D: lenses face +x, STEP z mapped to vehicle x, checked in `front.png` (three bezels forward). OK. Motor leads leave each motor
   towards the centre. OK.
Interfaces
5. Cube carrier edge was 0.5 mm from the front prop discs: moved back to x = 44 mm (clearance 3.8 mm); deck front edge cut back to x = 20 so the
   Cube (top z = 59) does not run into the deck.
6. L2, Jetson, radio and adapter overlapped (3-4 mm): L2 to x = -20, Jetson to x = -106, radio/adapter further aft, rear deck lengthened to x = -212.
7. Power leads cut through arm_3: rerouted vertically at (6, -26) through new slots in the top and bottom plates.
8. Adapter bands penetrated the deck: bands now go through slots.
Standoffs: deck standoffs and hole positions defined, lengths 20 / 2 x 22 / 10 / 6 / 5 mm. Not yet verified by a test (added in pass 6).
Open after pass 4: deck outline plain; USB stack hit a Jetson screw (moved the ports). CoM x = -11 mm (battery at its forward limit).
Numbers: mass 1.791 kg, CoM (-11.2, 0, 19.4) mm, 0 interferences, prop clearance +3.8 mm.
