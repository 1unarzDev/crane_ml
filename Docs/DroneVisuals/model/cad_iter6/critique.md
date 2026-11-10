# cad_iter6: automated interface checks (pass 3 of 3, final)

Added to `build.py`: `interface_report` (every standoff/spacer ends on plates or a coupler, the plate has a hole on its axis and
material around it, tested on the watertight B-rep meshes) and `connectivity` (every body reaches the frame within 0.35 mm).
Findings and fixes
* The centre-line deck standoff at (-100, 0) stood inside the new Jetson window (ring test: 0 of 8 points on material): windows split in two
  with a bridge. Now 24 standoffs/spacers checked, 0 bad.
* rx and buck lowered onto the top plate; radio carrier given its 4 spacer holes; 0 floating bodies.
* Orientation re-checked in front/top/closeup: OAK-D lenses forward, Cube arrow forward, JST left, Jetson ports aft, L2 upright on the deck, props CCW/CW per
  PX4 order with the leading edge up (tested), motor leads towards the centre.
No orientation, interface or standoff issues remain in the checks or in the renders.
Accepted differences from the concept (see SPEC.md): deck above the props, Cube at the front centre, L2 is a cylinder with a window band (envelope exact),
no honeycomb, no Tattu artwork, battery plain, no ESC grommets, no GNSS.
Numbers: mass 1.784 kg, CoM (-10.7, 0, 19.3) mm, Ixx 6.75e-3, Iyy 1.717e-2, Izz 1.672e-2, Ixz 3.24e-3 kg m^2, 0 interferences,
prop clearance +3.8 mm, 24/24 interfaces, 0 floating.
