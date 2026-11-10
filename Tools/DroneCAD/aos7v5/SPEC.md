# AOS 7 V5 search-and-rescue quad: concept, spec and reconciliation (real-parts rebuild, passes 4-6)

Status: IMPLEMENTED + FIXTURE_VALIDATED for geometry/mass bookkeeping (`test_model.py`, 67 checks); NOT PHYSICALLY_CALIBRATED.
Nothing was weighed or measured on the real airframe. Each dimension is `DATASHEET` (vendor-published), `DERIVED` (computed or
measured from a vendor STEP whose numbers are facts) or `ESTIMATE` (my guess; measure it). Vendor CAD status per part: `VENDOR_CAD.md`.

One parameter file, `params.py`, drives `build.py` (CAD, meshes, URDF, sidecar); `spec_table.py` regenerates the tables below;
`test_model.py` verifies; `render.py` renders.

## 1. Concept (from `Docs/DroneVisuals/model/reference_concept.png`, layout only)
Stretched-X carbon frame, two-level stack on gold standoffs, Cube Orange + Mini carrier at the front, L2 upright at the centre,
Jetson at the rear, 2.5GbE adapter at the side, pMDDL900 + two antennas at the rear, OAK-D S2 at the nose in a gold bracket,
Tattu 6S 3500 slung under the frame with straps, 4-in-1 ESC in the stack. The picture is an AI render and not buildable as drawn.

## 2. Frame
iFlight page: wheelbase 313 mm, L254 x W184 x H36, arm 8, plates 2 / 2.5 / 2 mm. Manufacturer (aos-rc.com/designs/aos-7-v5):
"250 g including hardware", stacks 30.5 / 25.5 / 20 mm, 28 x 28 mm M3 GoPro pattern, 19 mm motor pattern, 155 deg FOV with no
props in view. The iFlight shop lists "Weight 325g"; I use the manufacturer's 250 g (the shop figure is probably the boxed kit with
TPU parts and straps). If your as-built frame weighs 325 g, change `FRAME["mass_g"]`: the total becomes 1.859 kg.
sqrt(254^2 + 184^2) = 313.6, so the motors sit on a 254 x 184 rectangle (stretched X; rotors at +-0.127, +-0.092 m).
There is no public STEP/DXF of the frame (aos-rc.com build resources list 3D-print STLs only), so plates and arms are modelled from
the dimensions, with real hole patterns: 19 x 19 M3 per motor (aligned with the arm), 30.5 x 30.5 stack, battery-strap slots.

## 3. Layers, standoffs and fasteners (z in mm from the arm mid-plane)
| layer | z | held by |
|---|---|---|
| bottom plate | -6.5 .. -4 | straps through 4 slots (x = -4 and +58, y = +-23.3, clear of the arm roots), foam pad + pack below |
| arms | -4 .. 4 | 4 x M3 socket screws per motor from below (19 x 19 pattern) |
| top plate | 4 .. 6 | frame |
| ESC (30.5 x 30.5) | 9 .. 16 | the 4 inner stack standoffs pass through its holes (grommets omitted) |
| 6 stack standoffs, M3 hex alu, **22 mm** | 6 .. 28 | top plate to upper plate (this gives the vendor H36: 36.5 built) |
| upper plate | 28 .. 30 | stack standoffs, M3 screws on top |
| Cube carrier on **5 mm** alu spacers | 30 .. 35 | 4 holes (26.4 x 30 pattern, measured from the Mini carrier STEP) in the upper plate |
| 4 deck standoffs, M3 alu, **20 mm** | 30 .. 50 | upper plate holes (14, +-24) and (-58, +-26) |
| 1 deck standoff, **2 x 22 mm stacked** | 6 .. 50 | top-plate hole (-100, 0), inside the free rhombus between the rear prop discs |
| payload deck, 2 mm carbon | 50 .. 52 | the 5 standoffs |
| Jetson on 4 x **10 mm** nylon spacers | 52 .. 62 | 86 x 58 holes (from memory, verify) |
| L2 base on the deck | 52 .. 117 | 4 x M3 from below on the 51 mm bolt circle (manual p.8) |
| radio carrier on 4 x 6 mm nylon spacers | 52 .. 58 | 27 x 42 holes (ESTIMATE) |
| USB-Ethernet adapter | 52 .. 65 | two hook-and-loop bands through deck slots |
| OAK-D S2 | -26 .. 1 | gold bracket: 3 mm top strap + 2 cheeks, 4 x M3 through the top-plate nose |
Why a second deck above the props: with 177.8 mm props on motors 184 mm apart, the only free area at prop height is a small
rhombus (e.g. |y| < 7 mm at x = -100). Everything wide must sit above z = 42; the deck standoffs stand inside the rhombus
(`prop_clearance_min_mm` = 3.8 mm, checked in `build.py`). The top-plate nose is cut back to x = 124 so it does not shade the
OAK-D's upper field of view (ESTIMATE of the real nose).

## 4. Component table (generated)
<!-- AUTO:table -->
| component | link(s) | dims (mm) | dims source | mass g | CoM x, y, z (mm, FLU from frame centre) | mass source |
|---|---|---|---|---|---|---|
| AOS 7 V5 frame kit: 4 arms, bottom/top/upper plates, 6 stack standoffs + hardware, 2 battery straps | base_link | motor rect 254 x 184, arm 8, plates 2.5 / 2 / 2, H36, 19x19 motor pattern, 30.5 stack | DATASHEET iFlight + aos-rc.com; plate outlines and hole positions ESTIMATE (no public CAD) | 250.0 | -0.9, 0.0, 3.0 | 250 g DATASHEET aos-rc.com ('including hardware'); iFlight shop lists 325 g (see section 2) |
| Payload deck (2 mm carbon, z 50..52) + 5 alu standoffs (4 x 20 mm, 1 x 2 x 22 mm) + 4 nylon Jetson spacers (10 mm) + 4 nylon radio spacers (6 mm) + screws | payload_deck_link | 232 x 116 max, chamfered/tapered, lightening slots | ESTIMATE (own design) | 76.5 | -96.4, 0.1, 50.1 | volume x density: carbon 1.6, alu 2.7, nylon 1.14, steel 7.85 g/cc |
| Motor stator/base/cable (x4); F90 2806.5 KV1300 stand-in for the user's 'Avenger' | motor_0..3 | 33.4 dia x 34.7 high incl. bell | DATASHEET T-Motor/ProgressiveRC | 106.4 | 4 x see URDF (rotor hubs z = +40, motors z = +11.5) | 46.6 g incl. cable DATASHEET (stator/bell split ESTIMATE) |
| Bell (20 g) + HQ 7040 tri-blade prop 9 g (x4), spin about +z | rotor_0..3 | 177.8 mm dia; blade shape ESTIMATE | DATASHEET diameter | 116.0 | 4 x see URDF (rotor hubs z = +40, motors z = +11.5) | bell split ESTIMATE; prop 9 g ESTIMATE |
| Holybro Tekko32 F4 Metal 65 A 4-in-1 ESC | esc_link | 43 x 44 x 7 (+ 2 caps, XT60); 30.5 x 30.5 M4 holes | DATASHEET Holybro spec V1.8; height ESTIMATE | 15.8 | -6.5, -3.9, 12.7 | 15.8 g DATASHEET |
| Cube Orange + Mini carrier + 4 x 5 mm alu spacers | cube_carrier_link | case 38.25 x 38.25 x 22.3; carrier 38.5 x 47.4 x 1.7; holes 26.4 x 30 (approx.) | case DATASHEET (PX4 docs); carrier and holes measured from the CubePilot STEP (not redistributed) | 56.0 | 44.0, -3.0, 47.1 | Cube 40 g ESTIMATE; carrier 15 g ESTIMATE; spacers volume x 2.7 |
| Unitree L2 LiDAR (upright, base on the deck) | l2_link | 75 x 75 x 65, base 24, scan centre 44.5, 4 x M3 on a 51 mm circle | DATASHEET Unitree manual p.8 | 230.0 | -20.0, -0.0, 86.3 | 230 g DATASHEET |
| Jetson Orin Nano Super dev kit (carrier long axis across the vehicle, ports aft) | jetson_link | 100 x 79 x 21, holes 86 x 58 | NVIDIA, recalled from memory: VERIFY (CAD needs a login) | 176.0 | -111.5, 0.1, 70.7 | 176 g ESTIMATE (dev kit class) |
| Cable Matters USB-to-2.5GbE adapter + hook-and-loop bands | usb_eth_link | 60 x 22 x 13 | ESTIMATE | 25.5 | -181.9, -38.0, 58.5 | 25 g ESTIMATE + 0.5 g bands |
| Microhard pMDDL900 OEM + carrier + 2 antennas | radio_link | module 32 x 30 x 6; carrier 35 x 50 x 8; antennas 140 x 8 dia | ESTIMATE | 42.0 | -195.0, 30.5, 92.6 | module 7 g DATASHEET; carrier 15 g, antennas 2 x 10 g ESTIMATE |
| FrSky receiver (model unknown) | rx_link | 16 x 11 x 5 | ESTIMATE | 5.0 | -45.4, 12.6, 8.8 | 5 g ESTIMATE |
| OAK-D S2 enclosure (real Luxonis STEP, MIT) + lens bezels | oak_d_link | 97 x 29.5 x 23.1, baseline 75 | DATASHEET Luxonis STEP / datasheet | 91.0 | 108.1, -0.0, -13.7 | 91 g DATASHEET |
| Gold aluminium OAK-D bracket (top strap + 2 cheeks) + 4 x M3 screws | oak_bracket_link | 3 mm plate | ESTIMATE | 15.0 | 109.1, 0.0, -3.8 | 15 g ESTIMATE |
| Tattu 6S 22.2 V 3500 mAh + 3 mm foam pad (strapped under the bottom plate) | battery_link | 140 x 45 x 40 | ESTIMATE (vendor page not fetchable) | 524.0 | 23.2, -0.7, -29.3 | 520 g + 4 g pad ESTIMATE |
| XT60 + power leads, 12 V buck for L2, 4 motor-lead bundles | harness_link | bundles | ESTIMATE | 55.0 | -5.3, 3.7, 9.3 | 55 g ESTIMATE |
| **total** | | | | **1784.2** | **-10.7, 0.0, 19.3** | |
<!-- /AUTO:table -->

## 5. All-up mass and reconciliation
New CAD total **1.784 kg** (iteration-3 model 1.843 kg; CRANE v1 profile 1.383 kg). CoM (-10.7, 0.0, +19.3) mm from the frame centre.
Inertia about the CoM (kg m^2): Ixx 6.75e-3, Iyy 1.717e-2, Izz 1.672e-2, Ixz 3.24e-3.
Why CoM x is -10.7 mm: the pack is already as far forward as it can go (x = +24; the OAK-D bracket is in front of it), and the
Jetson (176 g at x = -111), L2 (230 g at x = -20), radio and adapter are all aft of the motor centre. The proposed fix is on the real
drone: mount the Jetson further forward over the stack, or use a shorter pack. Until then the plant trims 8.5% differential rotor speed
at hover (front rotors slower), which is realistic for this layout but eats thrust headroom on the rear rotors.
<!-- AUTO:recon -->
| item | CRANE v1 profile g | CAD g | delta g | why |
|---|---|---|---|---|
| frame kit (+ standoffs, straps) | 170.0 | 250.0 | +80.0 | vendor 250 g incl. hardware vs 170 g guessed |
| payload deck + standoffs + spacers | 0.0 | 76.5 | +76.5 | new: needed above the prop plane |
| motors + props | 222.4 | 222.4 | -0.0 | unchanged (186.4 + 36) |
| ESC | 15.8 | 15.8 | +0.0 | unchanged |
| Cube + carrier + spacers | 95.0 | 56.0 | -39.0 | Cube 40 g (v1 guessed 75 g) |
| Jetson | 140.0 | 176.0 | +36.0 | dev kit class 176 g |
| USB-Ethernet adapter | 0.0 | 25.5 | +25.5 | not in the v1 budget |
| radio + antennas | 42.0 | 42.0 | +0.0 | same |
| RC receiver | 5.0 | 5.0 | +0.0 | same |
| OAK-D S2 + bracket | 91.0 | 106.0 | +15.0 | +15 g bracket |
| L2 | 230.0 | 230.0 | -0.0 | same |
| battery + pad | 290.0 | 524.0 | +234.0 | 3500 mAh pack vs assumed 1800 mAh / 290 g |
| harness / regulator / misc | 82.0 | 55.0 | -27.0 | v1 82 g bucket incl. GNSS; CAD 55 g, no GNSS modelled |
| **total** | **1383.2** | **1784.2** | **+401.0** | |
<!-- /AUTO:recon -->
Changes vs the iteration-3 model (-59 g): frame 332 -> 250 g (-82, manufacturer figure), deck 48 -> 76 g (+28, now with real
standoffs/spacers/screws), Cube + carrier 65 -> 56 g, Jetson/L2/OAK/battery unchanged.
Not modelled: GNSS/compass (no part named; about 30 g if fitted), ESC capacitor bank, grommets, prop guards, landing gear.

## 6. Proposed change to CRANE's AOS7 profile (NOT applied; the live profile is the iteration-3 CAD one)
`proposed_profile_aos7_cad.json` (id `aos7-v5-f90-1300kv-6s-cad`). Differences from the live profile come from this pass only
(mass 1.843 -> 1.784 kg, inertia, CoM, rotor z, tau, drag). Run `test_model.py` for the table: hover thrust fraction 0.202 (live
0.209, v1 0.157), T/W 4.95 (4.79, 6.38), trim speed spread 8.5% (2.4%, 0).
Rotor mapping (identity): PX4 motor 1..4 = URDF `rotor_0..3` = CRANE M1..M4 = FR (CCW), RL (CCW), FL (CW), RR (CW).

## 7. Motor: "Avenger" is unresolved (question for the user)
The user says "Avenger motor". BrotherHobby's Avenger line (brotherhobby.com, checked 2026-11-10) lists 2810 V5 910KV, 2812 V5 910KV,
2816 V5 760KV, 3110-6220 sizes and "SE 2807 1300KV"; there is **no Avenger 2806.5**, and the product pages carry no thrust
table in their HTML (specs load by script). No Avenger data was used: the thrust/torque numbers are still the T-Motor F90 2806.5
KV1300 + HQ 7040 table (6S), now labelled `ASSUMED` in the sidecar. Question: which exact motor and KV is on the drone (label on the
bell), and which propeller? With the answer, only `MOTOR`/`PROP` in `params.py` and the table in `Tools/Aerial/fit_aos7_profile.py` change.

## 8. Dimensions still marked ESTIMATE (please measure on the real drone)
Battery mass/size (520 g, 140 x 45 x 40) and internal resistance; Jetson mass as mounted and its hole pattern (86 x 58, from memory)
and height; Cube + carrier mass (40 + 15 g) and the carrier hole pattern (approx. from the STEP); as-built frame mass (250 vs 325 g)
and real CoM (balance on a string: x, z); deck height/standoffs you actually use; radio carrier mass/outline/holes and antenna mounts;
USB-Ethernet adapter size/mass; harness mass (55 g); FrSky model; GNSS; prop mass (9 g), bell split (20 g); L2 bolt-circle angles
(read off a figure) and its heading (which side the cable leaves); L2 IMU offset; OAK-D bracket and its height; top-plate nose.

## 9. Passes (images + `critique.md` in `Docs/DroneVisuals/model/cad_iterN/`)
Passes 1-3 were the first model (blocks, payload deck, details). Passes 4-6 rebuild it from real parts and hole patterns:
4. Real OAK-D enclosure, Cube/carrier/L2/ESC from measured dims and hole patterns, standoffs with real lengths, strap slots,
   motor 19x19 patterns, motor lead bundles. 6 problems found and fixed (critique.md).
5. Deck outline and lightening redesigned, windows under the L2 and Jetson, rx/radio/adapter fastening.
6. Automated interface and floating-part checks added; last standoff in a window found and fixed. No orientation, interface or
   standoff issues remain in the checks (24 standoffs/spacers, 0 floating bodies, 0 interferences).

## 10. Rebuild and test
```
export TMPDIR=~/.cache/cad-tmp; PY=~/.cache/research/cad-venv/bin/python
cd Tools/DroneCAD/aos7v5
python3 fetch_vendor_cad.py            # optional: downloads vendor CAD into the cache (never executed); build falls back without it
$PY build.py --out-dir out --model-dir ../../../Docs/DroneVisuals/model --urdf-dir "../../../Assets/Robot Descriptions/aos7v5"   # ~35 s
$PY render.py ../../../Docs/DroneVisuals/model ../../../Docs/DroneVisuals/model/cad_iter6 ../../../Docs/DroneVisuals/model/reference_concept.png
$PY spec_table.py ; $PY test_model.py   # ~20 s
```
