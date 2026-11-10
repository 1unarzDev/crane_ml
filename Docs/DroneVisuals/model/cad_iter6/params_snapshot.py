"""Single parameter file for the AOS 7 V5 search-and-rescue quad CAD (units: mm, g, degrees).

Frame: x forward, y left, z up (REP-103 FLU).  Origin = frame centre: x/y at the motor-rectangle
centre, z = mid-plane of the 8 mm arms.  Every number carries a status:
  DATASHEET  vendor-published number (url in SRC)
  DERIVED    computed here from other parameters
  ESTIMATE   my engineering estimate; the user should measure it on the real drone
build.py, test_model.py and the generated SPEC table read ONLY this file.
"""

ITER = 6                 # passes 1-3: first model (cad_iter1..3); passes 4-6: real-parts rebuild (cad_iter4..6), see cad_iterN/critique.md
LAYOUT = "payload_deck"
DETAIL = 3

SRC = {
    "AOS7": "https://shop.iflight.com/AOS-7-V5-Frame-Kit-Pro2311 (spec block, fetched 2026-11-10); https://www.aos-rc.com/designs/aos-7-v5 (manufacturer: 250 g including hardware, stacks 30.5/25.5/20 mm, 28x28 M3 GoPro pattern, 19 mm motor pattern)",
    "F90": "https://www.progressiverc.com/products/t-motor-f90-2806-5-brushless-motor ; https://store.tmotor.com/product/f90-fpv-motor.html",
    "L2": "Unitree 4D LiDAR L2 User Manual p.8 (installation dimensions figure): 75 x 75 x 65, base 24 mm, scan centre 44.5 mm, 4 x M3 x 6 deep on a 51 mm bolt circle; https://oss-global-cdn.unitree.com/static/Unitree%204D%20LiDAR%20L2%20User%20Manual.pdf",
    "OAK": "Luxonis depthai-hardware DM9098_OAK-D-S2 enclosure STEP (MIT): 97 x 29.5 x 23.1 mm; https://www.mouser.com/catalog/specsheets/Luxonis_4-13-2022_OAK-D-S2_AF%20&%20FF%20Datasheet.pdf ; https://shop.luxonis.com/products/oak-d-s2",
    "ESC": "https://holybro.com/products/tekko32-f4-metal-4in1-65a-esc-65a ; https://www.getfpv.com/holybro-tekko32-f4-metal-65a-4-in-1-esc.html",
    "CUBE": "CubePilot Cube STEP (docs.cubepilot.org/3d-printing/cubepilot-product-3d-model): bbox 38.2 x 38.2 x 22.1 mm, measured; PX4 docs 38.25 x 38.25 x 22.3",
    "MINI": "CubePilot Mini Carrier Board STEP (same page): PCB 38.5 x 47.4 x 1.7 mm and hole pattern measured from the STEP (dimensions are facts; the STEP is not redistributed)",
    "JETSON": "NVIDIA Jetson Orin Nano Developer Kit (100 x 79 x 21 mm and the 86 x 58 mm hole pattern are quoted from memory; NVIDIA CAD needs a login, so NOT verified)",
    "MICROHARD": "https://www.microhardcorp.com/brochures/pMDDL900.Brochure.Rev.1.2.1.pdf (OEM ~7 g)",
    "TATTU": "https://genstattu.com (6S 3500 mAh 25C class; product page not fetchable 2026-11-10)",
    "PROP": "HQProp 7040 3-blade, 7 in diameter (as used in the T-Motor F90 KV1300 thrust table)",
}

# ---------------- frame (AOS 7 V5 kit; no public CAD exists, so the plates are modelled from the published dimensions) ----------------
FRAME = dict(
    motor_dx=127.0, motor_dy=92.0,            # DATASHEET: L254 x W184 motor rectangle -> 313.6 mm diagonal (wheelbase 313)
    arm_t=8.0, top_t=2.0, bot_t=2.5, upper_t=2.0,   # DATASHEET thicknesses
    stack_gap=22.0,                            # DATASHEET H36 total => top-plate-top to upper-plate-bottom = 22 mm standoffs (36 - 6.5 - 2 - 2 - ... = 36.5 built)
    height=36.0,
    mass_g=250.0,                              # DATASHEET manufacturer aos-rc.com: "250g including hardware" (iFlight shop lists 325 g: packaging/TPU/straps; 250 used, see SPEC.md)
    carbon_density=1.6, alu_density=2.81,      # g/cc, ESTIMATE
    arm_width_root=26.0, arm_width_tip=22.0, arm_pad_r=17.0,   # ESTIMATE
    arm_root=(20.0, 14.0),                     # ESTIMATE arm root point (|x|, |y|)
    motor_pattern=19.0,                        # DATASHEET 19 x 19 mm, M3
    stack_pattern=30.5,                        # DATASHEET stack 30.5 x 30.5 (also 25.5 / 20 positions)
    bottom_plate=[(-84, -30), (-70, -40), (70, -40), (96, -22), (96, 22), (70, 40), (-70, 40), (-84, 30)],   # ESTIMATE outline
    top_plate=[(-150, -16), (-120, -40), (-80, -46), (60, -46), (100, -30), (140, -22), (140, 22), (100, 30), (60, 46), (-80, 46), (-120, 40), (-150, 16)],   # ESTIMATE
    upper_plate=[(-70, -30), (-45, -42), (40, -42), (78, -22), (78, 22), (40, 42), (-45, 42), (-70, 30)],      # ESTIMATE
    stack_standoffs=[(15.25, 15.25), (15.25, -15.25), (-15.25, 15.25), (-15.25, -15.25), (62.0, 0.0), (-62.0, 0.0)],   # 30.5 pattern + 2 outer
    strap_x=(-35.0, 35.0), strap_slot=(16.0, 3.2), strap_slot_y=27.0, strap_w=14.0,   # battery strap slots in the bottom plate (ESTIMATE positions, kit has 2 straps)
)
STANDOFF = dict(od=5.5, id=2.9, rho_alu=2.7, rho_nylon=1.14)   # M3 hex standoff, ESTIMATE (standard 5.5 mm AF)

MOTOR = dict(                                  # T-Motor F90 2806.5 KV1300 used as a stand-in until the Avenger question is answered (SPEC.md section 7)
    r_bell=16.7, height=34.7, mass_g=46.6,    # DATASHEET 33.4 dia x 34.7, 46.6 g incl. cable
    bell_mass_g=20.0, bell_h=18.7, bell_wall=1.2,       # ESTIMATE split
    stator_r=14.0, base_r=16.0, base_h=5.0,   # ESTIMATE visual split
    kv=1300, resistance_ohm=0.076,
)
PROP = dict(
    diameter=177.8, blades=3, mass_g=9.0,      # DATASHEET diameter (7 in); mass ESTIMATE (HQ 7x4x3 ~9 g)
    plane_z=40.0,                              # DERIVED = arm top 4 + motor 34.7 + 1.3 seat
    hub_r=11.0, hub_h=5.0, chord_peak=15.5, chord_root=13.0, chord_tip=6.0, thickness=1.8, pitch_deg=18.0,   # ESTIMATE shape
    nut_r=4.5, nut_h=6.0,
)

# ---------------- payload components (x, y, z = centre of the box; dims x,y,z) ----------------
ESC = dict(dims=(43.0, 44.0, 7.0), pos=(0.0, 0.0, 12.5), mass_g=15.8,    # dims DATASHEET(Holybro: 43 x 44, 30.5 x 30.5 M4 holes w/ M3 grommets); mass DATASHEET; height ESTIMATE
           cap_r=5.0, cap_len=20.0, xt60=(14.0, 8.0, 8.0))
CUBE = dict(dims=(38.25, 38.25, 22.3), mass_g=40.0,                      # case DATASHEET (PX4 docs, STEP 38.2 x 38.2 x 22.1); mass ESTIMATE
            connector_h=5.6,                                             # DERIVED from the Cube STEP: DF17 connector zone below the orange case
            carrier_dims=(38.5, 47.4, 1.7), carrier_mass_g=15.0,         # DERIVED from the Mini carrier STEP (x width, y length, thickness); mass ESTIMATE
            carrier_holes=[(-13.2, 10.5), (13.2, 10.5), (-13.2, -19.5), (13.2, -19.5)],   # DERIVED from the Mini carrier STEP, relative to the PCB centre (approximate)
            jst_strip_y=(13.7, 21.4),                                    # DERIVED: connector strip on the +y end of the carrier (STEP frame y)
            pos_xy=(44.0, 0.0), standoff_h=5.0)                          # ESTIMATE placement; 5 mm spacers
L2 = dict(dims=(75.0, 75.0, 65.0), mass_g=230.0, pos_xy=(-20.0, 0.0),    # DATASHEET dims & mass (manual p.8)
          base_h=24.0, scan_origin_z=44.5, head_r=33.0,                  # DATASHEET base 24 mm, scan centre 44.5 mm above the base; head radius ESTIMATE
          hole_circle_d=51.0, hole_angles_deg=[22.5, 67.5, 202.5, 292.5],   # DATASHEET 4 x M3 on a 51 mm circle; angles read off the manual figure (ESTIMATE)
          imu_offset=(-0.007, 0.0, 0.0127))                              # ESTIMATE IMU offset from the scan origin
JETSON = dict(dims=(100.0, 79.0, 21.0), mass_g=176.0, pos_xy=(-106.0, 0.0),   # dims NVIDIA (memory); mass ESTIMATE (dev kit class)
              hole_pattern=(86.0, 58.0), standoff_h=10.0, pcb_t=1.6)     # hole pattern from memory (verify); 10 mm nylon spacers clear the underside M.2 slot
ETH = dict(dims=(60.0, 22.0, 13.0), mass_g=25.0, pos_xy=(-179.0, -38.0))   # ESTIMATE (Cable Matters USB-to-2.5GbE adapter)
RADIO = dict(module_dims=(32.0, 30.0, 6.0), module_mass_g=7.0,           # module DATASHEET mass ~7 g; outline ESTIMATE
             carrier_dims=(35.0, 50.0, 8.0), carrier_mass_g=15.0, pos_xy=(-172.0, 30.0), standoff_h=6.0, hole_pattern=(27.0, 42.0),   # ESTIMATE
             antenna=dict(r=4.0, length=140.0, mass_g=10.0, tilt_out_deg=14.0, tilt_back_deg=12.0, base_xy=((-205.0, 44.0), (-205.0, 18.0))))   # ESTIMATE
RX = dict(dims=(16.0, 11.0, 5.0), mass_g=5.0, pos=(-44.0, 12.0, 8.6))    # ESTIMATE (FrSky model not given; R-XSR class)
OAK = dict(dims=(23.1, 97.0, 29.5), mass_g=91.0,                          # DATASHEET 97 x 29.5 x 23.1 (STEP) and 91 g
           baseline_mm=75.0,                                              # DATASHEET stereo baseline
           pos=(108.05, 0.0, -10.75),                                     # DERIVED: lens plane at x=+119.6, top flush under top plate
           bracket_mass_g=15.0, bracket_t=3.0, bracket_screws=[(104.0, -12.0), (104.0, 12.0), (116.0, -12.0), (116.0, 12.0)])   # ESTIMATE gold aluminium bracket, 4 x M3 through the top plate nose
BATTERY = dict(dims=(140.0, 45.0, 40.0), mass_g=520.0,                    # Tattu 6S 3500 mAh 25C class: ESTIMATE dims and mass
               pad_gap=3.0, pad_mass_g=4.0, strap_mass_g=0.0,
               capacity_ah=3.5, v_nominal=22.2, cells=6,                  # label (user)
               x_solved=True, x_bounds=(-40.0, 24.0), resistance_ohm=0.060)
HARNESS = dict(mass_g=55.0, pos=(-10.0, 0.0, 14.0))                      # ESTIMATE XT60/power leads, L2 12 V buck, signal wiring, motor leads outside the motors
DECK = dict(
    z0=50.0, t=2.0,
    front_standoffs=[(14.0, 24.0), (14.0, -24.0), (-58.0, 26.0), (-58.0, -26.0)], front_len=20.0,   # on the upper plate (z=30): 20 mm
    rear_standoffs=[(-100.0, 0.0)], rear_len=44.0,            # on the top plate (z=6): 2 x 22 mm stacked = 44 mm; inside the free rhombus between the rear prop discs
    outline_half=[(20, 22), (8, 42), (-44, 42), (-56, 56), (-62, 58), (-146, 58), (-156, 52), (-204, 52), (-212, 44)],   # +y half, mirrored; 45-degree chamfers + a stepped taper like the frame top plate (ESTIMATE design)
    standoff_od=5.5, lightening_rho=1.6)
SPIN = {0: +1, 1: +1, 2: -1, 3: -1}     # CCW seen from above = +1 (PX4/ArduCopter quad-X)
ROTOR_NAMES = {0: "front_right", 1: "rear_left", 2: "front_left", 3: "rear_right"}
ROTOR_SIGN = {0: (+1, -1), 1: (-1, +1), 2: (+1, +1), 3: (-1, -1)}      # (sign x, sign y) of the motor in FLU

AIR = dict(rho=1.225, cd_body=1.0)       # frontal-area drag, ESTIMATE Cd
