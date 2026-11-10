# v1 design log (render-and-compare iterations)

Each iteration does the same three things: build (`build_v1.py`), render the concept-matched views and dock/pair close-ups (`render_v1.py`), and compare them with the references. Comparison sheets are in `images/iterN/compare_<role>.jpg`. The full render sets are in `~/.cache/crane-fleet/assets_v1/previews/iterN/`. Validator and engineering checks run on every build (`checks_v1.py`).

## Iteration 1: first parametric build (L 0.358, W 0.180, dock span 124 mm)
Validator: 0 FAIL on the first pass, every mate at gap 0.0 mm, hook-to-pin 0.00 mm. The WARNs are `guide_width` (by design, no funnel) and two degenerate triangles per hook.

Discrepancies (concept vs render):
1. **View mirrored.** I rendered from the front-left. All three concepts put the forward face on the image right, so they are 3/4 front-RIGHT views and the visible side is the robot's right (-y). Fix: azimuth -27..-30 deg, and the -y/+y layout is mirrored (LiDAR, Orange Pi bays, Ethernet port, lifter mast, cargo flag).
2. **Proportion.** The render reads about 2.0:1 (L/W); the concepts read about 1.45:1. With the wheel as an in-plane ruler on the side face, the concept side length is 245-300 mm. With the D435/Arducam as rulers on the front face, the front dock is 110-124 mm. The concept draws the side dock smaller (86-105 mm) than the front dock. Fix: one identical 110 mm dock on every face (reciprocity needs identical faces), L 0.358 -> 0.342 and W 0.180 -> 0.176. The residual (L about 15 % longer than the concept impression) is **deliberate**: a full dock between two 97 mm wheels that are tucked behind a 6 mm front wall needs L >= 0.34 m.
3. Front flat face was 0.88 x W; the concept shows 0.80. Fix: vertical corner chamfer 10 -> 16 mm.
4. Top bevel was 6 mm; the concept shows ~12 % of side height. Fix: walls 77 -> 74 mm, bevel 6 -> 9 mm.
5. Lifter jaws looked long and spread. Fix: prongs 70 -> 60 mm.
6. Scout arm concept pose: the forearm drooped. Fix: new concept pose.
7. Cargo flag was on the rear-right; with the mirrored geometry the concept's far corner is rear-left (+y). Fix: moved, and the deck notch and the displaced standoff moved with it.
8. Dock shrink made the 12 mm magnets cut into the bracket bars. Fix: 10 mm discs at y = +-41 mm.

## Iteration 2: L 0.342, W 0.176, dock span 110 mm, mirrored layout
Masses: scout 3.51 kg, lifter 3.14 kg, cargo 3.43 kg. Validator 0 FAIL. All engineering checks pass except one: the folded arm top was 1.0 mm above the assumed LiDAR scan plane.

Discrepancies:
1. Wheels read as big grey discs. In the concept a smaller chrome star plate sits inside a ring of fat black rollers. Fix: plate 0.80 -> 0.66 x roller circle, smaller windows on a 16.5 mm pitch circle, roller fill 0.92 -> 0.96.
2. The arm looked too compact compared with the concept, where the elbow is high and the forearm reaches forward over the D435. Fix: concept pose j = (-0.30, -0.20, 1.25, 0.95, 0, 0.55).
3. The cargo deck merged visually with the body (both near-black). Fix: deck plate colour 0.27 grey (the concept deck is lighter than the body).
4. The hook-section render dropped the partner pin, because it filtered whole triangles. Fix: proper plane clipping (`render_v1.py`).
5. Folded arm 1 mm above the scan plane. Fix: shoulder 30 -> 26 mm above the turntable.
6. Kept (deliberate): the body is longer than the concept (see iteration 1 #2). The cargo flag sits inside the footprint, 2 mm below the deck top, so it does not collide with docked neighbours or payload overhang. In the concept it cantilevers out at deck level. The cargo Arducam lens is recessed so it stays behind the mating plane; in the concept it sticks out past the face.

## Iteration 3: wheels, arm pose, shoulder height, section renders
Changes: wheel star plate 0.66, roller fill 0.96, arm concept pose, shoulder 26 mm, deck colour, plane-clipped sections, and zero-area pole triangles removed from the tessellation. That last change cleared the `watertight.degenerate` WARN.
Checks: validator 0 FAIL (the only WARN left is `guide_width`), dock / stability / arm-fold checks all pass; folded arm top 0.149 m against a scan plane of 0.152 m.
URDF test: two failures, both test or format bugs, not design issues. (a) The hook-joint filter caught the fixed `hook_*_engage` joints. (b) URDF rpy was written with `%.6g`, which left about 4e-6 rad of error against the manifest. Fix: test filter, URDF written at `%.10g`.

Discrepancies:
1. Scout arm: the concept forearm reaches toward the image right (+y, forward); mine was yawed toward the camera. Fix: concept pose yaw +0.60 rad.
2. The hook section now shows the partner pin inside the crook. The validator measures a 0.00 mm engage error, so no change.
3. Remaining items are listed below as deliberate or below the uncertainty.

## Iteration 4: final
The only change is the arm concept pose (render only; no geometry change). Rebuilt with STEP export; validator 0 FAIL, all engineering checks pass, all 6 URDF tests pass. Final sheets: `images/compare_{scout,lifter,cargo}.jpg` (= `images/iter4/`); final renders in `~/.cache/crane-fleet/assets_v1/previews/final/` (role views from iteration 4, docked pairs from iteration 3, whose geometry is identical).

### Remaining discrepancies (stopping criterion)
| # | discrepancy | status |
|---|---|---|
| 1 | v1 body L/W 1.94 vs concept impression ~1.45 | deliberate: an identical 110 mm reciprocal dock between two tucked 97 mm wheels needs L >= 0.34 m. The concept draws its side dock about 20 % smaller than its front dock |
| 2 | dock span 110 mm vs front-face estimate 115-124 mm | below the uncertainty (compromise with the side-face ruler at 86-105 mm) |
| 3 | cargo flag inside the footprint and 2 mm below the deck top; the concept cantilevers it out at deck level | deliberate: no collision with docked neighbours or payload overhang |
| 4 | cargo Arducam lens recessed behind the mating plane | deliberate: docking clearance |
| 5 | no face tags | deliberate: the concept shows none, and the dock leaves no room (open question) |
| 6 | lifter mast camera pitched 20 deg down (concept ~5-10 deg) | deliberate: the box top must stay in view |
| 7 | hook crook wraps 105 deg instead of a full J | deliberate: a 180 deg J jams on the pin when it rotates open |
| 8 | arm links are box envelopes (no rounded servo brackets, no cables) | proxy by design |
| 9 | cables, grommets, the RJ45 cable and the Arducam ribbon are not modelled | cosmetic; belongs in the collision envelope later |
| 10 | wheel 9 rollers (datasheet) vs ~10 drawn | below the uncertainty |
| 11 | lifter jaw / rail sizes, cargo standoff diameter, deck thickness | within +-20 % image uncertainty |

## Iteration 5: orchestrator review (footprint, wheels, drivetrain, lifter, acquisition)
The review pointed at three problems: v1 reads as a long narrow box (L/W 1.94) where the concepts are near-square and squat; the wheels read as grey discs; and the 0.18 N m micro motors are not credible.

1. **Footprint / width.** I re-measured with in-plane COTS rulers (`images/width_overlay.jpg`): the D435 against the scout front edge (250 px vs 430 px top edge, 490 px with corners), the Arducam on the lifter mast against the lifter front (105 vs 340 px; rails 160 px = 58 mm), and the Arducam against the cargo front (107 vs 360 px). **All three put the concept width at 0.16-0.18 m**, so the image's own COTS scale really is narrow. But the same scale makes the side faces 0.24-0.30 m, which cannot hold a reciprocal dock between two 97 mm wheels and leaves 79 mm for the motors. The concept is internally inconsistent at COTS scale. **Decision: W = 0.26 m**, set by the drivetrain (two 25D x 67L gearmotors face to face need ~160 mm between the wheel inner faces). This also restores the concept's near-square, squat look: L/W 1.32 (concept 1.1-1.4), cargo deck 0.334 x 0.252, H/W 0.33. L stays 0.342 (dock-between-wheels); H stays 0.085 (dock/wheel rulers). Cost: the D435, Orange Pis and Arducams look about 1/3 smaller relative to the body than drawn. That is deliberate, documented in the profile `body_size_lwh.estimate`, and open question 2.
   Knock-on changes: ly 0.062 -> 0.104, side faces at y = +-0.130, corner chamfer 22 mm, top-plate corners 34 mm, wheel well at y >= 0.0795, layouts re-spaced (LiDAR (-0.122, -0.072), Orange Pi bays at y -0.105..-0.049, tag (-0.122, +0.072)), cargo standoffs (+-0.130, +-0.100), ebox 0.20 x 0.15, flag (-0.131, +0.090).
2. **Wheels.** The flat chrome plates are replaced by a 9-spoke silver star hub (one spoke and boss per roller) recessed 6 mm, so the 9 dark barrel rollers (fill 0.96, 45 deg, X pattern unchanged) bulge past it, plus silver roller end caps (the silver dots around the rim in the concept). The concept draws 8 spokes and ~10 rollers; 9/9 follows the DFRobot datasheet wheel.
3. **Drivetrain.** Pololu 47:1 25Dx67L HP 12 V with 48 CPR encoder (#4845, datasheet): stall 1.47 N m, 220 rpm (no-load 1.12 m/s), 2249 counts/rev, 101 g. Battery 3S2P (0.30 kg). Masses recomputed from the solids: **base chassis 4.22 kg; scout 4.94, lifter 4.57, cargo 5.08 kg.** Steady driving at 0.5 m/s^2 plus rolling needs 0.06-0.12 N m (4-8 % of stall, cargo with 0-5 kg). Sills (quasi-static, one front wheel pivoting on the edge):
   - 1 cm: 0.36-0.73 N m (24-50 % stall), but the push needed (18-38 N) exceeds rear traction at mu 0.6 (15-30 N), so this is marginal without front-wheel edge grip or a rubber floor.
   - 2 cm: 0.48-0.97 N m, push 33-68 N: marginal to no.
   - 3 cm: no; it needs a ramp or larger wheels.
   The motor is no longer the limit: the 97 mm wheel and its 9 mm rollers are. Deck capacity 3 -> 5 kg (provisional, sill-limited).
4. **Lifter vs payloads** (no silent cut). The profile now carries both options:
   - (a) follow the concept (recommended): new payload `tag_box` (0.08 m cube, design 0.5 kg). Pitch stability allows 2.27 kg at safety 1.5 with no ballast; the lift/jaw mechanism rating stays 1.0 kg (provisional). box_small and larger travel on cargo decks.
   - (b) widen the jaws to 0.21 m clear (0.236 m outer, < 0.26 body) and add a 1.07 kg counterweight at x = -0.150, giving a 5.64 kg lifter that carries box_small (2 kg, gripped across 0.20 m) at safety 1.5. Lift load 19.6 N; carriage and lead screw must be re-sized.
   Recommendation: (a). The user decides.
5. **Unchanged:** reciprocal dock, D435 stereo, right-side layout (a choice, from the front-right view geometry), no face tags. **Partner acquisition proposal** (profile `docking.partner_acquisition_proposal`): top tags are seen almost edge-on by forward cameras at docking range, so they serve only the overhead camera. Far field: overhead/fleet map. Mid field: a 30 mm tag on a 45 deg facet of the top bevel above each dock (local bevel 9 -> 25 mm). Near field: odometry, magnet capture (+-9 mm) and hook sensing (as TagDockingController does now). Alternative: tags on widened C-brackets.
Checks after iteration 5: validator 0 FAIL (only `guide_width` WARNs), dock/stability/arm-fold pass, 6/6 URDF tests pass.
- Two cargo with box_long: tip 38.3 deg / 0.79 g (was 22.9 deg).
- Scout arm stretched with 0.2 kg: margins 81-97 mm.
- Lifter with tag_box: margin 75 mm.

## Iteration 6: user decisions (2026-11-10)
1. **Width 0.26 m kept** (342 x 260). The electronics (D435, Orange Pis, Arducams) appear about 1/3 smaller relative to the body than in the concept. This is by design: the width is set by the drivetrain.
2. **Lifter option (b) is the default.**
   - Jaws: clear opening 0.06-0.21 m, 160 mm forks, 120 mm pads, carriage bar 0.244 m.
   - Battery moved above the rear motors (centre x -0.104; it cannot go further back because the cavity ends at -0.136 and the motors fill the space below). This saves 0.12 kg of steel.
   - Counterweight: **1.25 kg** steel in two blocks above the rear motors (centroid x -0.106). The iteration-5 estimate of 1.07 kg assumed x = -0.150 and the short concept forks; with the battery centred it would be 1.37 kg.
   - Lifter mass **6.07 kg**. With box_small (2 kg, gripped across 0.20 m, 0.30 m deep) at full stroke (45 mm): pitch safety 1.54, static margin 32 mm, tip-free deceleration 3.8 m/s^2.
   - **Lift rating 2.0 kg** (limit 2.05 kg at safety 1.5 for box_small geometry; 3.78 kg for an 80 mm tag_box).
   - tag_box stays as an optional payload type.
3. **Dock acquisition:** a 30 mm tag36h11 on a 45 deg facet of a printed wedge (`dock_tag_mounts`) above every dock face.
   - Geometry: wedge top 18 mm above the top plate. Ids = role base + face index (scout 10-13, lifter 21-23, cargo 30-33, +10 for cargo_2). Lateral offsets clear the D435, Orange Pi bay, LiDAR, cargo camera and flag. The top contact pad moved 3.5 mm down.
   - Visibility check (`checks_v1.py tag_visibility`): every observer camera (scout D435 colour, lifter mast OV9281, cargo OV9281) against every dock tag at 0.3 / 0.5 / 1.0 / 1.5 m, aligned. **132/132 visible**: in FOV, view angle 40-45 deg, no occlusion. Border edge 58-93 px at 0.3 m, 19-29 px at 1.0 m, 12.9-19.4 px at 1.5 m. The OV9281 at 1.5 m is marginal for reliable tag36h11 decoding.
   - Staged scheme: far = overhead / fleet map; mid (0.3-1.5 m) = bevel tags; close (<0.3 m) = odometry + magnet capture + hook sensing. The lifter's camera faces forward but it docks rear/side, so it acquires front-on, turns, and closes on odometry.
Checks: validator 0 FAIL (WARN only `guide_width`), dock / stability / arm-fold / tag visibility pass, 6/6 URDF tests. Final sheets: `images/compare_*.jpg` (= iter6).

## Iteration 7: user decision D89 (taller lift, so the lifter can load cargo decks)
1. **Stroke 45 -> 170 mm.** Rails extended to z = 0.192. The upper cross bar moved above the carriage's top position: at the old 45 mm stroke it already overlapped the carriage by 3 mm, which no check covered. The top block is on the rail tops. At full stroke the gripped box underside is 0.170 m above ground = deck top + 20 mm; the minimum for the 15 mm requirement is 165 mm. URDF `lift_joint` limit 0..0.17.
2. **Jaw clear opening 0.06-0.24 m** (20 mm per side on box_small's 0.20 m width). Carriage bar 0.258 m and thinner jaw sections keep the jaw outer width at 0.258 m, inside the 0.26 m body. Grip pads are now 48 mm tall and run the full 160 mm prong length; they end 1 mm above the box underside and 4-5 mm below the prong bottoms. That is what lets the forks clear a deck after set-down.
3. **Stability at full height with box_small** (2 kg, 0.30 m deep):
   - Lifter mass 6.22 kg (counterweight 1.25 kg steel unchanged, battery rearward).
   - Pitch safety **1.51**, static margin 30 mm, combined COM 0.120 m high, tip-free braking up to **2.45 m/s^2** (requirement 1.0).
   - **Lift rating stays 2.0 kg** (limit at safety 1.5 = 2.01 kg): headroom is under 1 %. +0.1 kg of counterweight would buy margin, but the steel blocks above the rear motors hold at most ~1.27 kg, so that would need denser material (lead/tungsten) or larger blocks.
4. **Deck placement check** (`checks_v1.py deck_placement`: whole-lifter mesh overlap against the cargo at carry, set-down and released poses):
   - **Side-on onto one deck:** box centred (0 offset); 0 mm^3 overlap in all three poses; box 20 mm above the deck in transit; forks 5 mm above the deck while withdrawing. The 0.30 m box overhangs the 0.252 m deck by 24 mm per side and covers 3.6 cm^2 of the flag notch (flag 2 mm below the deck top: no contact, tag occluded).
   - **Side-on at the junction of an inline pair:** box centred on the coupling, 0 mm^3 overlap, forks 5 mm clear. It covers 52 cm^2 of the second cargo's flag notch: the flag is occluded and that corner of the box is unsupported, but the box is still carried by both decks.
   - **End-on:** feasible within 6 mm of centre, but jaws touch the cargo's front by 4-8 mm^3 at set-down. Use the side-on approach.
5. The mast camera moved to (0.090, -0.085) so the taller column stays out of its view. Bevel-tag visibility is still 132/132.
Checks: validator 0 FAIL (only `guide_width` WARNs), all engineering checks pass, 6/6 URDF tests.
