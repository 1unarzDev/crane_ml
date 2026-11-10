# cad_iter1 critique (blocks, single deck, layout copied from the reference)

Numbers: mass 1.794 kg, CoM z +10.6 mm, battery pushed to x = +31 mm to balance a rear-heavy deck.
Renders: iso (reference camera angle: nose image-left, from the front-left, 30 deg elevation), top, front, side.

What differs from the concept / what is wrong
1. LAYOUT IS NOT BUILDABLE. The automated prop-disc check (build.py `prop_clearance`) reports the Jetson body (-46 mm),
   the USB-Ethernet adapter (-51 mm), the radio carrier/module (-62/-64 mm) and both antennas inside the swept
   discs of the rear 7 in props. The reference render puts everything on one deck at the arm plane; with a 184 mm lateral
   motor spacing and 177.8 mm props only a small rhombus around the centre is free at prop height (z = 36..44 mm).
   Fix (iteration 2): move Jetson, L2, radio, USB-Eth and antennas onto a second "payload deck" on gold standoffs
   above the prop plane; keep only the Cube + Mini carrier on the frame upper plate, inside the free rhombus.
2. Props are flat rods; the reference has tapered, pitched three-blade props (CW/CCW mirrored). Needed for inertia and look.
3. Everything is a block: no L2 head/window, no Jetson heatsink/fan/ports, no ESC capacitors/XT60, no Cube connectors,
   no OAK lenses. The OAK-D bracket is a flat plate. Add in iteration 2/3.
4. Arms: root starts at x = 34 mm and looks thin next to the reference. Plates are fine for a first pass.
5. Interference list (AABB): 25 hits, mostly false positives from rotated arm AABBs and intended pass-throughs
   (standoffs through ESC holes, power leads through plate slots). Replace by an exact mesh test with an allow-list.
6. Render framing: model fills only 40% of the frame; metals render black without an environment map (lower metallic).
