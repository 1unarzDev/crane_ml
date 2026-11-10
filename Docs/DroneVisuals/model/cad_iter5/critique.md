# cad_iter5: deck and payload mounting (pass 2 of 3)

Changes: deck outline redesigned (stepped taper, 45-degree chamfers like the top plate), cable pass-through window under the L2,
two windows under the Jetson module (bridge kept for the centre-line standoff), pill slots along the edges, slot pairs for the
adapter bands; new `under` render (battery, straps, motor screws, OAK bracket) and `deck_top` render (deck + payload).
Looked at: closeup, side, under, deck_top.
Findings
* Deck now reads as a matching carbon plate (chamfers, pill slots). The windows are hidden by the L2 and Jetson from above, so they are
  checked in the under view and by the automated hole test in pass 6.
* Side view confirms the layer stack: 22 mm stack standoffs, 20 mm and 2 x 22 mm deck standoffs, 5 mm Cube spacers, 10 mm nylon
  under the Jetson, 6 mm under the radio. Rear standoff stands on the top plate and passes the upper-plate level beyond its tail.
* Under view: motor screws on the 19 x 19 pattern under each pad, straps through the plate slots clear of the arm roots, OAK-D
  gripped by the gold cheeks and screwed to the nose.
Remaining: rx and buck had a 0.5 / 3 mm air gap above the top plate (floating); radio carrier had no holes for its spacers; no automated check that a standoff meets a hole at both ends.
Numbers: mass 1.783 kg, CoM (-10.6, 0, 19.2) mm, 0 interferences.
