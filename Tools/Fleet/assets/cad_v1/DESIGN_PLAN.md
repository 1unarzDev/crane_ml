# CRANE modular mecanum fleet: v1 design plan (image planning)

Inputs are three concept renders: `images/ref/modular1.jpg` (scout), `modular2.jpg` (lifter) and `modular3.jpg` (JPEG copies) (cargo). They are copies; the originals in the repo root were not changed. Output is the v1 design: `Tools/Fleet/spec/fleet_profile_v1.json`, `cad_params_v1.json` and the generated assets in `~/.cache/crane-fleet/assets_v1/`. The v0 profile, the v0 generator and `assets_v0/` were not changed.

**Iteration 5 supersedes the width below:** W = 0.26 m (drivetrain-driven; the image's COTS scale says 0.17 m, see `images/width_overlay.jpg` and DESIGN_LOG iteration 5). Wheels have a 9-spoke star hub; motors are Pololu 25D 47:1 HP; masses are 4.94 / 4.57 / 5.08 kg; lifter options (a)/(b) are in the profile.

**Read this first.** The concept images are AI-style renders. They give proportions, not dimensions, and they are not geometrically consistent. In modular1, for example, the side-face pockets and the side-face dock span give side/front foreshortening ratios of 0.74 and 0.45. Every number below is an estimate. Its uncertainty is given in its row, and the profile records it per value in `estimate`.

## 0. View geometry: the images are 3/4 front-RIGHT views

In all three images, the face that carries the forward sensor (the D435, the gripper, the front Arducam) is on the image right, and the face with the second dock is on the image left. A camera that sees the front face on its right and a side face on its left stands at the robot's front-right. The visible side is therefore the robot's **right** side (-y). Features that the brief calls "left" because they are on the image left (the LiDAR at the "rear-left" and the Orange Pi bay) are, by this geometry, on the robot's right side. v1 places them at -y and renders the comparison views from azimuth -30 deg. The handoff lists this as conflict C0. Mirroring the layout would not change any engineering check except the sign of the COM y offset.

## 1. Feature inventory (numbered, per image)

Locations are given in the robot frame (x forward, y left). "Prop." means proportion, measured on the same face as the item.

### modular1.png, scout
| # | feature | role | location | prop. / estimate |
|---|---|---|---|---|
| S1 | chamfered octagonal dark-grey body | shared chassis | whole body | L/W ~2.0-2.2 (dock-normalised); side height ~0.6 x W |
| S2 | top bevel between walls and top plate, screwed top plate (perimeter M3 heads) | shell | top edge | bevel ~12-14 % of side height |
| S3 | vertical corner chamfers, wheels tucked into the corner wells | chassis | 4 corners | front flat face ~0.80 x width |
| S4 | mecanum wheels, chrome star side plate (8 windows), ~9-10 black rollers | drive | corners | diameter ~1.2-1.25 x side height |
| S5 | front dock: two orange C-brackets ("[" "]"), J-hook in the left (viewer) pocket, horizontal pin in the right pocket, 2 round magnets inside the brackets, 2 small square pads at top/bottom centre | reciprocal dock | front face | dock span 0.90 x flat face; pockets ~31 x 34 mm; magnets ~11 mm |
| S6 | identical dock on the visible side face, between the wheels | dock | right side (-y) | same module |
| S7 | Intel RealSense D435 on a low bracket at the front edge | depth / stereo | top, front, centred | 90 mm = 0.65 x front flat face |
| S8 | arm turntable on a dark square base plate behind the D435 | arm base | top, centreline, x ~ +0.075 | turntable ~58 mm |
| S9 | ~6-DOF arm: yaw, shoulder, elbow, wrist pitch, wrist roll, gripper; beige links, black servos, orange accents | manipulator | on S8 | servo bodies ~45 mm (STS3215-like); reach ~0.25-0.30 m |
| S10 | RPLIDAR C1 on a raised block with orange accent and cable | 2-D LiDAR | rear corner, visible side (-y) | riser ~1.1 x C1 height |
| S11 | Orange Pi 5 Pro with heatsink + fan in an open top bay | compute | top, -y side, middle-rear | 89 x 56 mm |
| S12 | Ethernet service port (RJ45 panel) | service | top, front-right corner | ~22 x 18 mm |
| S13 | cable runs (black/red/orange) | wiring | top | not modelled (collision envelope only) |
| S14 | inset: internal bay with Raspberry Pi Pico + Adafruit ISM330DHCX (#4502), STEMMA cable, cable handle | MCU + IMU | inside the body | bay ~100 x 50 mm |
| S15 | inset: reciprocal dock, two faces facing each other: hook (viewer-left) + pin (viewer-right) on both | dock concept | - | confirms 1 hook + 1 pin per face |
| S16 | inset: arm folded for travel, the forearm lying forward at about LiDAR height | travel pose | top | fold inside the footprint |

### modular2.png, lifter
| # | feature | role | location | prop. / estimate |
|---|---|---|---|---|
| L1 | same chassis, wheels, top plate | shared | - | same as S1-S4 |
| L2 | Orange Pi 4 Pro with heatsink + fan in an open top bay | compute | top, rear, -y side | 89 x 56 mm |
| L3 | Arducam OV9281 board on a short slotted mast, lens forward | camera | top, near the front, slightly -y | board 38 mm; mast ~ one board high |
| L4 | AprilTag on the top plate | fiducial | top, front half, +y of the mast | plate ~88 mm |
| L5 | front low-lift gripper: top block, twin vertical rails, upper cross bar with orange stripe, carriage bar with orange stripe, two L-shaped jaws whose horizontal prongs run forward at floor level, black grip pads on the inner faces | gripper | front face | rails ~58 mm apart, ~67 mm tall; jaws ~105 mm outer |
| L6 | front face keeps both C-brackets and one magnet (viewer-right), no hook/pin | partial dock | front face | - |
| L7 | dock on the visible side face | dock | right side | as S5 |
| L8 | inset: tagged cardboard box (~80 mm cube) held in the jaws | payload | front | ~ jaw width |
| L9 | cable grommets | wiring | top | not modelled |

### modular3.png, cargo
| # | feature | role | location | prop. / estimate |
|---|---|---|---|---|
| C1 | same chassis | shared | - | - |
| C2 | raised flat deck: dark plate with screwed rim and a slightly inset mat, chamfered corners | cargo deck | top | covers the body footprint |
| C3 | 4 polished standoffs with collars | deck support | deck corners | ~12 mm diameter, ~35-40 mm gap |
| C4 | vented electronics box in the deck gap (perforated front panel) | electronics | under the deck | ~0.55 x deck length |
| C5 | AprilTag on a corner flag at deck level | fiducial | far corner (+y, rear) | plate ~62 mm |
| C6 | Arducam on a bracket at the front-face top edge, below the deck | camera | front, centred | board 38 mm |
| C7 | full docks on the front and the visible side | dock | front, right | as S5 |
| C8 | inset: carton on the deck, the flag stays visible | payload | deck | box ~0.55 x deck length |

## 2. Absolute scale (COTS cross-check)

Datasheet dimensions (URLs in `fleet_profile_v1.json`): RealSense D435 90 x 25 x 25 mm, 72-75 g, IR baseline 50 mm; RPLIDAR C1 55.6 x 55.6 x 41.3 mm, 110 g; Orange Pi 5 Pro and 4 Pro 89 x 56 mm (62 g / 58 g); Raspberry Pi Pico 51 x 21 mm; Arducam OV9281 USB board (B0332) 38 x 38 mm, HFOV 70 deg with the stock 2.8 mm M12 lens; Adafruit ISM330DHCX #4502 is the standard 1.0 x 0.7 in STEMMA QT board (not confirmed: the fab print could not be read); DFRobot FIT0768 97 mm mecanum, 44.9 mm wide, 9 rollers at 45 deg, ~150 g; Feetech STS3215 45.2 x 24.7 x 35 mm, 55 g.

Method: a ruler only counts if it lies in, or parallel to, the face being measured. That removes the unknown foreshortening.

| ruler (image) | measured | gives |
|---|---|---|
| D435, 250 px, parallel to the scout front face | flat front face 385 px, dock span 345 px, full front 480 px | flat face 129-139 mm, dock 115-124 mm, W 165-173 mm (range = 0-7 % perspective correction) |
| Arducam board, 107 px, in the cargo front plane | dock span 310 px, flat face 360 px, front 445 px | dock 110 mm, flat face 128 mm, W ~160-170 mm |
| Arducam board, 105 px, lifter mast (further back) | flat front 335 px | flat face <= 121 mm |
| wheel = 97 mm (vertical extent, same depth as the face) | side height / wheel = 0.75-0.80 | side height 73-78 mm |
| Arducam board height (vertical, cargo) | face height 230 px | 85 mm |
| D435 vs wheel (scout, with the left-face ellipse ratio) | wheel 180 px | D435 would be 93 mm (vs 90): the wheel and D435 scales agree within 3 % |
| Pico 230 px / ISM330 135 px (inset) | bay 490 x 255 px | internal bay ~92-109 x 41-57 mm (the inset's own proportions disagree by 30 %) |

Length cannot be measured directly, because the side faces are foreshortened by an unknown amount. Two estimates agree: (a) taking the dock span as the same on front and side gives side/front foreshortening 0.41-0.52 and L = 296-368 mm; (b) a full 124 mm dock must fit between two 97 mm wheels tucked into corner wells behind a 6 mm front wall, which forces L >= 350 mm. The pocket-width ratio (0.74) would give L ~250 mm, but then no side dock fits, so it was rejected. The deck in modular3 looks almost square in projection. That conflicts with every face-based estimate and is treated as an image inconsistency.

**Chosen (final, after iteration 2; mating plane to mating plane):** L = 0.342 m (+-15 %), W = 0.176 m (+-8 %), H = 0.085 m underside to the top of the top plate (+-10 %), ground clearance 0.022 m (+-8 mm). Iteration 1 used 0.358 x 0.180 with a 124 mm dock. The renders read clearly longer than the concept, and the side-face wheel ruler gives a side dock of only 86-105 mm, so the dock was reduced to 110 mm on every face (see DESIGN_LOG). The final L/W = 1.94 against the concept impression of ~1.45 is deliberate: a full dock must fit between two tucked 97 mm wheels.

## 3. Decisions

**Chassis.** Walls from z = 0 to 0.074, a 9 mm 45 deg bevel, then a 2 mm aluminium top plate (octagon corners 24 mm). Vertical corner chamfer 16 mm (iteration 2, from the 0.80 flat-face ratio). Printed PETG shell with 3 mm walls (mass from the solid).

**Wheels.** 97 mm x 45 mm, 9 rollers (datasheet class). lx = 0.109 and ly = 0.062, so the wheel tread stays 13.5 mm behind the front mating plane and the outer wheel face stays 3.5 mm inside the side mating plane. Wheels therefore never touch when docked front/rear or side/side. The validator measured 0 mm^3 of non-dock overlap.

**Dock faces per role.** The front-dock conflict was resolved this way:
| role | front | rear | left | right |
|---|---|---|---|---|
| scout | dock | dock | dock | dock |
| lifter | gripper face: brackets + 1 magnet only, **not** a dock (concept L6) | dock (assumed, same chassis) | dock | dock |
| cargo | dock | dock | dock | dock |

**Reciprocal hook / pin / magnet geometry (face frame, x out, z up):**
- Mating plane x = 0 is the outer surface of the orange C-brackets (|y| 30-55 mm, z +-28 mm, 7 mm bars, 4 mm proud of the face plate; dock span 110 mm) and of the two steel pads (8 x 6 mm at z +-33.5 mm). Identical brackets butt against each other; they are the contact datum, not a funnel.
- Hook pocket at y = -22 mm and pin pocket at y = +22 mm, both 24 x 38 x 24 mm deep (rounded r = 4).
- Catch pin: 6 mm steel, axis y, at x = -10 mm, z = 0, spanning the +y pocket (supported at both walls).
- J-hook: 6 mm rod, pivot axis +y at (x -20, y -22, z +6.5) mm. The arm runs 30 mm forward and the crook (centreline radius 6.5 mm) wraps 105 deg over and behind the partner pin; engage point (30, 0, -6.5) mm in the hook frame. q = 0 is closed; q = -0.30 rad lifts the tip 2.9 mm clear of the pin and stays inside the 38 mm pocket. A 180 deg J would jam on the pin when it rotates, so the wrap was cut back to 105 deg (deliberate, see the log).
- Rz(pi) maps -y onto +y, so each hook lands on the partner's pin: **one hook per face, and two hooks per mated connection.**
- Magnets: 10 x 3 mm discs at y = +-41 mm, z = -4 mm, inside the brackets, faces 0.5 mm behind the mating plane (1 mm apart when mated), opposite poles (+y N out, -y S out).
- Lateral capture is +-9 mm (24 mm pocket around the 6 mm hook) and equals the profile guide_half_width. There is no funnel chamfer.

**Sensing conflict: D435 vs OV9281 stereo (v0).** The concept clearly shows a D435, so v1 uses it. The scout "stereo pair" becomes the D435 infra1/infra2 imagers (global shutter, 50 mm baseline, datasheet). Depth, infra1, infra2 and colour frames follow the realsense-ros naming. The v0 rear OV9281 is dropped (it is not in the concept). The lifter and cargo keep one OV9281 each (HFOV 70 deg datasheet, was 80 deg in v0). The v0 ToF sensors are dropped (not in the concept). Open question: re-add one for box-presence sensing in the jaws?

**Tags.** There are no face tags: the dock fills the face, and the concept shows none. Top tags: lifter 70 mm (concept), cargo 50 mm on the corner flag (concept), scout 50 mm (added, not in the concept).

**Deck.** 0.334 x 0.168 m, 4 mm inside the mating planes so decks never touch. Top at 0.150 m above ground. The 39 mm gap is set by the 38 mm Arducam board under the deck edge. Standoffs are 12 mm. The rear-left standoff moved inboard to clear the flag notch.

**Arm (proxy).** Turntable at x = +0.070; shoulder 26 mm above the turntable; upper arm 100, forearm 95, wrist 30, gripper 50 mm (reach 0.275 m, +-25 %). The upper arm and forearm sit side by side (30 mm elbow offset), so the arm folds flat. The folded arm is under the LiDAR scan plane and inside the footprint (checked). Limits: yaw +-pi, shoulder +-1.75, elbow -0.3..3.2, wrist pitch +-1.8, roll +-pi, jaw 0..0.9 rad.

**Lifter gripper envelope.** Rails 58 mm apart, 8 mm ahead of the front plane. Lift travel 45 mm. Jaw clear opening 40-120 mm. Prongs 70 mm long, 3 mm above the floor at lift = 0. The front face is not a dock, so the jaws may stand proud of the mating plane.

## 4. Conflicts with the v0 profile (and the choice)
| # | v0 | concept | v1 choice | why |
|---|---|---|---|---|
| C0 | - | brief says LiDAR "rear-left" | rear-right (-y) | image geometry, sec. 0 |
| C1 | 2 hooks per face (A upper +y, B lower -y), 2 pins | 1 J-hook + 1 pin per face | 1 + 1, mated connection = 2 hooks | concept; the same engagement count, with simpler faces |
| C2 | scout stereo OV9281 pair + rear OV9281 | D435 | D435 (infra pair as stereo) | concept; a single calibrated unit |
| C3 | body 0.42 x 0.30 x 0.10 | ~0.25-0.30 x 0.17 x 0.085 (impression) | 0.342 x 0.176 x 0.085 | COTS-scaled |
| C4 | side faces at y = +-0.20, outside the body ("mounting concept?") | docks on the side walls | y = +-0.088 on the walls | concept, which answers the v0 open question |
| C5 | face tags 0.05 | none | none | no room on the dock face |
| C6 | lift travel 0.15, opening 0.30, capacity 5 kg | low lift, small jaws | 0.045 / 0.12 / 1.0 kg | concept + stability check |
| C7 | deck 0.40 x 0.30 at 0.14 m | deck on standoffs | 0.334 x 0.168 at 0.150 m | concept |
| C8 | base mass 6 kg | small rover | 2.79 kg chassis; 3.51 / 3.14 / 3.43 kg per role | computed from solids |
| C9 | payload box_small 0.30 x 0.20, 2 kg for the lifter | 80 mm box | profile payloads unchanged; lifter rated 1 kg | box_small tips the lifter (check 4c) |
| C10 | wheel torque 1.2 N m | - | 0.177 N m stall (Pololu micro gearmotor) | only micro motors fit between the wheel inner faces (79 mm) |
