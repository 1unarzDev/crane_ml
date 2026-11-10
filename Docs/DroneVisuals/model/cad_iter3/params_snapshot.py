"""Single parameter file for the AOS 7 V5 search-and-rescue quad CAD (units: mm, g, degrees).

Frame: x forward, y left, z up (REP-103 FLU).  Origin = frame centre: x/y at the motor-rectangle
centre, z = mid-plane of the 8 mm arms.  Every number carries a status:
  DATASHEET  vendor-published number (url in SRC)
  DERIVED    computed here from other parameters
  ESTIMATE   my engineering estimate; the user should measure it on the real drone
build.py, test_model.py and the generated SPEC table read ONLY this file.
"""

ITER = 3                 # 1 = blocks, single deck; 2 = payload deck; 3 = detailed (final).  See cad_iterN/critique.md
LAYOUT = "payload_deck" if ITER >= 2 else "single_deck"
DETAIL = ITER

SRC = {
    "AOS7": "https://shop.iflight.com/AOS-7-V5-Frame-Kit-Pro2311 (spec block, fetched 2026-11-10)",
    "F90": "https://www.progressiverc.com/products/t-motor-f90-2806-5-brushless-motor ; https://store.tmotor.com/product/f90-fpv-motor.html",
    "L2": "https://oss-global-cdn.unitree.com/static/Unitree%204D%20LiDAR%20L2%20User%20Manual.pdf",
    "OAK": "https://www.mouser.com/catalog/specsheets/Luxonis_4-13-2022_OAK-D-S2_AF%20&%20FF%20Datasheet.pdf ; https://shop.luxonis.com/products/oak-d-s2",
    "ESC": "https://holybro.com/products/tekko32-f4-metal-4in1-65a-esc-65a ; https://www.getfpv.com/holybro-tekko32-f4-metal-65a-4-in-1-esc.html",
    "CUBE": "https://docs.cubepilot.org/user-guides/autopilot/the-cube-module-overview (page moved; case size from earlier fetch, mass not published there)",
    "JETSON": "NVIDIA Jetson Orin Nano Developer Kit datasheet (100 x 79 x 21 mm; quoted from memory, not re-fetched)",
    "MICROHARD": "https://www.microhardcorp.com/brochures/pMDDL900.Brochure.Rev.1.2.1.pdf (OEM ~7 g)",
    "TATTU": "https://genstattu.com (6S 3500 mAh 25C class; product page not fetchable 2026-11-10)",
    "PROP": "HQProp 7040 3-blade, 7 in diameter (as used in the T-Motor F90 KV1300 thrust table)",
}

# ---------------- frame (AOS 7 V5 kit, DATASHEET unless noted) ----------------
FRAME = dict(
    motor_dx=127.0, motor_dy=92.0,            # DATASHEET: L254 x W184 motor rectangle -> 313.6 mm diagonal (wheelbase 313)
    arm_t=8.0, top_t=2.0, bot_t=2.5, upper_t=2.0,   # DATASHEET thicknesses
    height=36.0,                               # DATASHEET H36 (bottom plate underside to upper plate top)
    mass_g=325.0,                              # DATASHEET "Weight: 325g" for the kit (plates, arms, cage, TPU, screws, straps)
    carbon_density=1.6, alu_density=2.81,      # g/cc, ESTIMATE (carbon laminate; 7075)
    arm_width_root=26.0, arm_width_tip=22.0, arm_pad_r=17.0,   # ESTIMATE
    arm_root=(20.0, 14.0),                     # ESTIMATE arm root point (|x|, |y|)
    bottom_plate=[(-84, -30), (-70, -40), (70, -40), (96, -22), (96, 22), (70, 40), (-70, 40), (-84, 30)],   # ESTIMATE outline
    top_plate=[(-150, -16), (-120, -40), (-80, -46), (60, -46), (100, -30), (140, -22), (140, 22), (100, 30), (60, 46), (-80, 46), (-120, 40), (-150, 16)],   # ESTIMATE
    upper_plate=[(-70, -30), (-45, -42), (40, -42), (78, -22), (78, 22), (40, 42), (-45, 42), (-70, 30)],      # ESTIMATE
    lightening=0.72,                           # ESTIMATE fraction of plate area left after cut-outs (volume only; mass is rescaled to 325 g)
    stack_standoff=dict(r=2.5, positions=[(15.25, 15.25), (15.25, -15.25), (-15.25, 15.25), (-15.25, -15.25), (62, 0), (-62, 0)],
                        mass_g_each=1.2),      # ESTIMATE; M3 hex aluminium, gold anodised
)

MOTOR = dict(                                  # T-Motor F90 2806.5 KV1300
    r_bell=16.7, height=34.7, mass_g=46.6,    # DATASHEET 33.4 dia x 34.7, 46.6 g incl. cable
    bell_mass_g=20.0, bell_h=18.7, bell_wall=1.2,       # ESTIMATE split (existing profile used 18 g bell)
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
ESC = dict(dims=(43.0, 44.0, 7.0), pos=(0.0, 0.0, 12.5), mass_g=15.8,    # dims DATASHEET(Holybro/GetFPV); mass DATASHEET; height ESTIMATE
           cap_r=5.0, cap_len=20.0, xt60=(14.0, 8.0, 8.0))
CUBE = dict(dims=(38.25, 38.25, 22.3), mass_g=40.0,                      # case dims DATASHEET(earlier fetch); mass ESTIMATE
            carrier_dims=(45.0, 40.0, 1.6), carrier_mass_g=25.0,         # ESTIMATE (Mini carrier outline/mass unpublished)
            pos_xy=(50.0, 0.0), standoff_h=3.0, header_h=4.0)            # ESTIMATE placement; must stay clear of prop discs
L2 = dict(dims=(75.0, 75.0, 65.0), mass_g=230.0, pos_xy=(-9.0, 0.0),    # DATASHEET dims & mass; placement ESTIMATE
          scan_origin_z=40.0, imu_offset=(-0.007, 0.0, 0.0127))          # ESTIMATE frame heights/offsets above L2 base
JETSON = dict(dims=(100.0, 79.0, 21.0), mass_g=176.0, pos_xy=(-100.0, 10.0),   # dims DATASHEET(NVIDIA, memory); mass ESTIMATE (dev kit ~176 g)
              standoff_h=3.0)
ETH = dict(dims=(60.0, 22.0, 13.0), mass_g=25.0, pos_xy=(-64.0, 63.0))   # ESTIMATE (Cable Matters USB-to-2.5GbE adapter, dims/mass unpublished here)
RADIO = dict(module_dims=(32.0, 30.0, 6.0), module_mass_g=7.0,           # module DATASHEET mass ~7 g; outline ESTIMATE
             carrier_dims=(50.0, 35.0, 8.0), carrier_mass_g=15.0, pos_xy=(-112.0, -50.0),   # ESTIMATE
             antenna=dict(r=4.0, length=140.0, mass_g=10.0, tilt_out_deg=14.0, tilt_back_deg=12.0))   # ESTIMATE
RX = dict(dims=(16.0, 11.0, 5.0), mass_g=5.0, pos=(-48.0, 12.0, 9.0))    # ESTIMATE (FrSky model not given; R-XSR class)
OAK = dict(dims=(22.9, 97.0, 29.5), mass_g=91.0,                          # DATASHEET 97 x 29.5 x 22.9 (x depth, y width, z height) and 91 g
           baseline_mm=75.0,                                              # DATASHEET stereo baseline
           pos=(108.05, 0.0, -10.75),                                     # DERIVED: lens plane at x=+119.5, top flush under top plate (z=4)
           bracket_mass_g=15.0, bracket_t=3.0)                            # ESTIMATE gold aluminium bracket
BATTERY = dict(dims=(140.0, 45.0, 40.0), mass_g=520.0,                    # Tattu 6S 3500 mAh 25C class: ESTIMATE dims and mass
               pad_gap=3.0, strap_mass_g=0.0,                             # straps are in the 325 g frame kit ("2 x Battery Strap")
               capacity_ah=3.5, v_nominal=22.2, cells=6,                  # DATASHEET (label)
               x_solved=True, x_bounds=(-40.0, 22.0), resistance_ohm=0.060)   # x chosen so CoM x = 0; R ESTIMATE (6 x 8 mOhm + leads)
HARNESS = dict(mass_g=55.0, pos=(-10.0, 0.0, 14.0))                      # ESTIMATE XT60/power leads, L2 12 V buck, signal wiring
DECK = dict(z0=48.0, t=2.0, mass_g=None,                                  # payload deck plate (ESTIMATE; own carbon plate, not in the kit)
            outline_rects=[(-162.0, 25.0, -52.0, 52.0), (-150.0, -80.0, -70.0, -52.0), (-100.0, -30.0, 52.0, 76.0)],   # x0,x1,y0,y1 ESTIMATE
            standoff_xy=[(-100.0, 0.0), (-55.0, 24.0), (-55.0, -24.0), (22.0, 24.0), (22.0, -24.0)],
            standoff_r=2.5, standoff_mass_g=1.1, lightening=0.6, deck_clear_of_props_z=44.5)

SPIN = {0: +1, 1: +1, 2: -1, 3: -1}     # CCW seen from above = +1 (PX4/ArduCopter quad-X)
ROTOR_NAMES = {0: "front_right", 1: "rear_left", 2: "front_left", 3: "rear_right"}
ROTOR_SIGN = {0: (+1, -1), 1: (-1, +1), 2: (+1, +1), 3: (-1, -1)}      # (sign x, sign y) of the motor in FLU

AIR = dict(rho=1.225, cd_body=1.0)       # frontal-area drag, ESTIMATE Cd
