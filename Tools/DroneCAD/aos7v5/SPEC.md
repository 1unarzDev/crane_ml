# AOS 7 V5 search-and-rescue quad: concept, spec and reconciliation

Status: IMPLEMENTED + FIXTURE_VALIDATED for geometry/mass bookkeeping (`test_model.py`); NOT PHYSICALLY_CALIBRATED.
Nothing here was weighed or measured on the real airframe. Every dimension is a vendor datasheet number (`DATASHEET`),
derived (`DERIVED`) or my engineering guess (`ESTIMATE`). Geometry is modelled from scratch; no vendor CAD was downloaded
(`Assets/Drones/Models/LICENSES.md`).

All numbers come from one file, `params.py`. `build.py` makes CAD, meshes, URDF and the sidecar; `spec_table.py` regenerates
the tables below; `test_model.py` verifies.

## 1. Concept (from `Docs/DroneVisuals/model/reference_concept.png`, layout only)
True/stretched-X carbon frame (AOS 7 V5, 7 in, 3-blade props, T-Motor F90), two-level stack on gold standoffs, Cube Orange +
Mini carrier at the front, Unitree L2 upright at the centre, Jetson Orin Nano Super dev kit with fan heatsink at the rear,
Cable Matters 2.5GbE adapter at the side, Microhard pMDDL900 with two antennas at the rear, OAK-D S2 at the nose in a gold
bracket, Tattu 6S 3500 mAh slung under the frame with straps, 4-in-1 ESC in the stack. The picture is an AI render: it is
not a dimension source, and it is not physically buildable as drawn (section 4).

## 2. Frame (what I could verify)
iFlight AOS 7 V5 page (fetched 2026-11-10): wheelbase 313 mm; frame L254 x W184 x H36 mm; arm 8 mm; top plate 2 mm;
bottom plate 2.5 mm; upper plate 2 mm; stack 20x20 / 30x30; motor mount 19x19 M3; **kit weight 325 g**.
sqrt(254^2 + 184^2) = 313.6 mm, so the motors sit on a **254 x 184 mm rectangle: a stretched X** (longer fore-aft).
Consequences for CRANE: the current profile models a true-X square of 318 mm wheelbase (rotors at +-0.11243 m). The CAD
puts rotors at (+-0.127, +-0.092) m. Roll arm 92 mm, pitch arm 127 mm, so pitch authority is larger than roll authority.
Plate outlines and arm widths are ESTIMATE (only the thicknesses and the motor rectangle are published).
Arm centre plane is z = 0; prop plane z = +40 mm (arm top 4 + motor 34.7 + 1.3 seat, DERIVED; matches the CRANE profile).

## 3. Component table (generated)
<!-- AUTO:table -->
| component | link(s) | dims (mm) | dims source | mass g | CoM x, y, z (mm, FLU from frame centre) | mass source |
|---|---|---|---|---|---|---|
| AOS 7 V5 frame kit (arms, 3 plates, hardware, straps) + 6 gold stack standoffs | base_link | motor rect 254 x 184, arm 8, top 2, bottom 2.5, upper 2, H36 | DATASHEET iFlight AOS 7 V5 page; plate outlines ESTIMATE | 332.2 | 0.5, 0.0, 2.7 | 325 g kit DATASHEET (iFlight 'Weight: 325g'); standoffs 7.2 g ESTIMATE |
| Payload deck (2 mm carbon, z = 48 mm) + 5 gold standoffs | payload_deck_link | approx. 187 x 104 (+ tabs), 2 thick | ESTIMATE (my addition, needed for prop clearance) | 48.5 | -66.6, 1.2, 47.8 | ESTIMATE (volume x 1.6 g/cc x 0.6 cut-out factor) |
| F90 2806.5 KV1300 stator/base/cable (x4) | motor_0..3 | 33.4 dia x 34.7 high incl. bell | DATASHEET T-Motor/ProgressiveRC | 106.4 | 4 x see URDF (rotor hubs z = +40, motors z = +11.5) | 46.6 g incl. cable DATASHEET (split stator/bell ESTIMATE) |
| Bell (20 g) + HQ 7040 tri-blade prop 9 g (x4), spin about +z | rotor_0..3 | 177.8 mm dia; blade shape ESTIMATE | DATASHEET diameter | 116.0 | 4 x see URDF (rotor hubs z = +40, motors z = +11.5) | bell split ESTIMATE; prop 9 g ESTIMATE |
| Holybro Tekko32 F4 Metal 65 A 4-in-1 ESC | esc_link | 43 x 44 x 7 (+ 2 caps, XT60) | DATASHEET Holybro/GetFPV; height ESTIMATE | 15.8 | -5.4, -3.3, 12.8 | 15.8 g DATASHEET |
| Cube Orange + Mini carrier | cube_carrier_link | case 38.25 x 38.25 x 22.3; carrier 45 x 40 x 1.6 | case DATASHEET (earlier fetch); carrier ESTIMATE | 65.0 | 50.8, -0.0, 43.8 | Cube 40 g ESTIMATE; carrier 25 g ESTIMATE |
| Unitree L2 LiDAR (upright) | l2_link | 75 x 75 x 65 | DATASHEET Unitree manual | 230.0 | -9.0, 0.0, 84.5 | 230 g DATASHEET |
| Jetson Orin Nano Super dev kit (heatsink + fan) | jetson_link | 100 x 79 x 21 | DATASHEET NVIDIA (from memory, re-check) | 176.0 | -101.2, 7.0, 63.6 | 176 g ESTIMATE (dev kit class) |
| Cable Matters USB-to-2.5GbE adapter | usb_eth_link | 60 x 22 x 13 | ESTIMATE | 25.0 | -65.6, 62.6, 56.6 | 25 g ESTIMATE |
| Microhard pMDDL900 OEM + carrier + 2 antennas | radio_link | module 32 x 30 x 6; carrier 50 x 35 x 8; antennas 140 x 8 dia | ESTIMATE | 42.0 | -133.3, -50.0, 90.1 | module 7 g DATASHEET; carrier 15 g, antennas 2 x 10 g ESTIMATE |
| FrSky receiver (model unknown) | rx_link | 16 x 11 x 5 | ESTIMATE | 5.0 | -49.4, 12.6, 9.2 | 5 g ESTIMATE |
| OAK-D S2 + gold aluminium bracket | oak_d_link | 97 x 29.5 x 22.9 (W x H x D), baseline 75 | DATASHEET Luxonis; bracket ESTIMATE | 106.0 | 108.3, 0.0, -9.4 | 91 g DATASHEET; bracket 15 g ESTIMATE |
| Tattu 6S 22.2 V 3500 mAh (slung under bottom plate) | battery_link | 140 x 45 x 40 | ESTIMATE (vendor page not fetchable) | 520.0 | 21.2, -2.4, -29.4 | 520 g ESTIMATE |
| XT60 + power leads, 12 V buck for L2, signal wiring | harness_link | bundles | ESTIMATE | 55.0 | -14.5, 8.5, 10.8 | 55 g ESTIMATE |
| **total** | | | | **1842.9** | **-3.0, 0.0, 17.3** | |
<!-- /AUTO:table -->

## 4. Layout decision: payload deck above the prop plane
With 177.8 mm props on motors 184 mm apart laterally, the discs nearly touch at the centre line. At prop height
(z = 36..44 mm) only a small rhombus around the centre is free: e.g. at x = -60 mm the free band is |y| < 34 mm, and beyond
|x| = 127 mm it is |y| < 3 mm. The concept's Jetson (100 x 79), radio and antennas cut the rear discs by 46..64 mm
(measured by the automated check on iteration 1). So:
* Cube + Mini carrier stay on the frame upper plate at x = +50 mm (inside the rhombus, clearance +4.2 mm to the front discs).
* Jetson, L2, radio, USB-Ethernet adapter and antennas sit on a second 2 mm carbon deck at z = 48 mm on five gold standoffs
  placed inside the rhombus. This is the concept's "second level on gold standoffs", moved up by 38 mm.
* `build.py` asserts the clearance (`prop_clearance_min_mm >= 1`) and an exact mesh interference test with a short allow-list
  (standoffs through board holes, leads through plate slots, bracket bolted through the plate).
* The battery is slid forward until it meets the OAK-D bracket (x = +22 mm) and still leaves the CoM 3.0 mm behind the motor
  centre. This is reported, not hidden: equal-speed hover then has a 3.6 rad/s^2 pitch-up error, so the trim speeds differ by 2.4%.

## 5. All-up mass and reconciliation with CRANE's 1.383 kg profile
CAD total **1.843 kg** (+0.460 kg, +33%). CoM (-3.0, 0.0, +17.3) mm from the frame centre.
Inertia about the CoM (kg m^2): Ixx 6.935e-3, Iyy 1.5050e-2, Izz 1.5213e-2, Ixz 2.309e-3 (CRANE: 5.98e-3, 8.31e-3, 9.49e-3,
2.1e-4). Iyy/Izz grow most because the Jetson, radio and antennas sit 100..140 mm aft and 50..190 mm up, and because the pack
is large.
<!-- AUTO:recon -->
| item | CRANE profile g | CAD g | delta g | why |
|---|---|---|---|---|
| frame kit + 6 stack standoffs | 170.0 | 332.2 | +162.2 | kit is 325 g (DATASHEET); CRANE assumed 170 g |
| payload deck + 5 standoffs | 0.0 | 48.5 | +48.5 | needed above the prop plane (see section 4) |
| motors + props | 222.4 | 222.4 | -0.0 | unchanged (186.4 + 36) |
| ESC | 15.8 | 15.8 | +0.0 | unchanged |
| Cube + carrier | 95.0 | 65.0 | -30.0 | CRANE used a 70-80 g class guess; Cube Orange is lighter |
| Jetson | 140.0 | 176.0 | +36.0 | full dev kit incl. big carrier and heatsink |
| USB-Ethernet adapter | 0.0 | 25.0 | +25.0 | not in the CRANE budget |
| radio + antennas | 42.0 | 42.0 | +0.0 | same |
| RC receiver | 5.0 | 5.0 | +0.0 | same |
| OAK-D S2 + bracket | 91.0 | 106.0 | +15.0 | +15 g gold bracket |
| L2 | 230.0 | 230.0 | -0.0 | same |
| battery | 290.0 | 520.0 | +230.0 | 3500 mAh pack vs assumed 1800 mAh / 290 g |
| harness / regulator / misc | 82.0 | 55.0 | -27.0 | CRANE 82 g bucket incl. GNSS; CAD 55 g, no GNSS modelled |
| **total** | **1383.2** | **1842.9** | **+459.7** | |
<!-- /AUTO:recon -->
The largest items: pack +230 g (the profile assumed an 1800 mAh pack; the user's is 3500 mAh), frame +162 g with its standoffs (the profile guessed
170 g; the vendor says 325 g for the kit), the payload deck +48 g (needed for geometry), Jetson +36 g.
Not modelled: GNSS/compass (no part named; add about 30 g if fitted), ESC capacitor bank, prop guards, landing gear.

## 6. Proposed change to CRANE's AOS7 profile (NOT applied)
`proposed_profile_aos7_cad.json` (same schema, id `aos7-v5-f90-1300kv-6s-cad`) differs from
`Tools/Aerial/profiles/aos7_v5_f90_6s.json` as follows. `kf`, `km`, `omega_max`, radius, motor kv/R are unchanged (same F90 table).

| parameter | current | proposed | reason |
|---|---|---|---|
| mass | 1.3832 kg | 1.8429 kg | CAD sum, section 5 |
| inertia | diag 5.98e-3, 8.31e-3, 9.49e-3; Ixz 2.1e-4 | diag 6.94e-3, 1.505e-2, 1.521e-2; Ixz 2.31e-3 | CAD |
| rotor xy | +-0.11243 (true-X, 318 mm) | +-0.130/-0.124 x +-0.092 (254 x 184) | datasheet motor rectangle; shifted by the 3 mm CoM offset |
| rotor z (rel. CoM) | +0.0248 | +0.0227 | CoM is 17.3 mm above the frame centre |
| tau_up = tau_down | 33.8 ms | 25.9 ms | lower rotor inertia (CAD 2.20e-5 vs 2.80e-5 kg m^2); higher hover speed |
| rotor_inertia | 2.80e-5 | 2.20e-5 | CAD blade shape |
| rotor_drag lambda1 | 1.49e-4 | 1.72e-4 | 0.35 1/s x mass / (4 w_h) |
| drag_quadratic | 8.58e-3 | 1.18e-2 | CAD silhouette (props excluded): 0.0161 m^2 front, 0.0224 m^2 side, Cd 1 ESTIMATE |
| battery capacity / R | 1.8 Ah / 30 mOhm | 3.5 Ah / 60 mOhm | user's 3500 mAh 25C-class pack; R ESTIMATE |
| derived T/W (nominal voltage) | 6.38 | 4.79 | heavier |
| hover rotor speed / thrust fraction | 813 rad/s / 0.157 | 939 rad/s (8965 rpm) / 0.209 | heavier |
| hover ESC throttle (datasheet table) | 0.30 | 0.34 | heavier |
| body radius (tip to CoM) | 0.2479 m | 0.2481 m | essentially unchanged |

Rotor mapping (identity): PX4 motor 1..4 = URDF `rotor_0..3` = CRANE M1..M4 = FR (CCW), RL (CCW), FL (CW), RR (CW);
spin +1 = CCW seen from above = positive rotation about +z.

## 7. Dimensions still marked ESTIMATE (please measure on the real drone)
Highest value first: battery mass and size (Tattu 6S 3500: I assumed 520 g, 140 x 45 x 40); Jetson dev kit mass as mounted
(176 g) and heatsink height; Cube + carrier mass (40 + 25 g) and carrier outline; the actual frame weight as built (325 g is the
kit, with the camera cage and TPU you may not use); where the CoM really is (balance on a string: CoM x/z); payload deck
height and standoff positions; radio carrier mass/outline and antenna mounts; the USB-Ethernet adapter size/mass;
harness mass (55 g); FrSky receiver model; whether a GNSS unit is fitted; prop mass (9 g) and blade shape; motor bell
mass split (20 g); battery internal resistance (60 mOhm); OAK-D mounting height and the L2 scan-origin/IMU offsets
(`scan_origin_z = 40 mm`, `imu_offset`). A thrust-stand run of one F90 + prop at 6S would replace `kq`, `tau` and the 8%
thrust-fit residual.

## 8. Iterations (images and critiques in `Docs/DroneVisuals/model/cad_iterN/`)
1. Blocks, single deck, layout copied from the concept: 1.794 kg; prop-disc check fails for Jetson/radio/adapter/antennas.
2. Payload deck, tapered pitched props, exact interference test, battery solved for CoM (limited by the OAK-D): 1.843 kg,
   clearance +4.2 mm, 2 minor clashes (battery boot vs OAK-D, power leads vs pack).
3. Detail pass (black bells, straps, spacers, header pins, cable, lens bezels), clashes fixed: 0 interferences,
   watertight meshes, 1.843 kg (details carry fixed masses).

## 9. Rebuild and test
```
export TMPDIR=~/.cache/cad-tmp; PY=~/.cache/research/cad-venv/bin/python
cd Tools/DroneCAD/aos7v5
$PY build.py --out-dir out --model-dir ../../../Docs/DroneVisuals/model --urdf-dir "../../../Assets/Robot Descriptions/aos7v5"
$PY render.py ../../../Docs/DroneVisuals/model ../../../Docs/DroneVisuals/model/cad_iter3 ../../../Docs/DroneVisuals/model/reference_concept.png
$PY spec_table.py ; $PY test_model.py
```
(`build.py` takes about 10 s, `test_model.py` about 15 s: below the one-minute threshold, so run directly with `nice`.)
