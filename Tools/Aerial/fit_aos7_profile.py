"""Build profiles/aos7_v5_f90_6s.json from cited datasheet numbers (numpy only, no network).

Run:  python -I Tools/Aerial/fit_aos7_profile.py [--check]
Every derived number is recomputed here so the provenance in the JSON is reproducible.
Provenance labels: MEASURED_DATASHEET (vendor-published number), LITERATURE (published model or
coefficient), ESTIMATED (engineering estimate; method stated in the locator).
"""

import json
import math
import sys
from pathlib import Path

import numpy as np

HERE = Path(__file__).resolve().parent
OUT = HERE / "profiles" / "aos7_v5_f90_6s.json"

TMOTOR_URL = "https://store.tmotor.com/product/f90-fpv-motor.html"
TMOTOR_ARCHIVE = "http://web.archive.org/web/20250524125822/https://store.tmotor.com/product/f90-fpv-motor.html"
TMOTOR_SHA = "9a66a6b911f63098355bdb318ea5288691295acb3e6c645200b65e14dbf2d97a"

# T-Motor F90 2806.5 KV1300, prop HQ 7040 (7x4x3), bench supply ~24 V (6S), ambient 27 C.
# columns: throttle, thrust_g, voltage_V, current_A, rpm, power_W   (MEASURED_DATASHEET)
F90_KV1300_HQ7040 = [
    (0.30, 353.46, 24.24, 2.14, 8192, 51.97),
    (0.35, 478.74, 24.22, 3.31, 9342, 80.15),
    (0.40, 596.38, 24.19, 4.55, 10488, 110.13),
    (0.45, 758.30, 24.15, 6.71, 11716, 162.08),
    (0.50, 947.57, 24.10, 8.83, 12801, 212.90),
    (0.55, 1131.80, 24.06, 11.29, 13891, 271.71),
    (0.60, 1290.19, 24.01, 14.27, 14807, 342.61),
    (0.70, 1585.74, 23.90, 20.64, 16402, 493.17),
    (0.80, 1865.32, 23.78, 27.73, 17739, 659.40),
    (0.90, 2210.20, 23.63, 36.73, 18865, 867.82),
    (1.00, 2360.61, 23.48, 45.11, 19617, 1059.16),
]
F90 = dict(kv=1300, weight_g=46.6, resistance_ohm=0.076, idle_current_a_10v=0.95, poles="12N14P")

G = 9.80665
RHO = 1.225
PROP_DIAMETER = 7.0 * 0.0254          # HQ 7040: 7 in
ETA_SHAFT = 0.80                      # ESTIMATED motor+ESC efficiency (shaft / electrical)
DRAG_PER_MASS = 0.35                  # ESTIMATED aggregate rotor drag [1/s]; see locator
WHEELBASE = 0.318                     # ESTIMATED motor-to-motor diagonal [m]

# (name, mass_kg, position FLU [m] of the component centre, box size [m] (x,y,z) or None for a
#  point mass, status, source key, locator)
COMPONENTS = [
    ("frame_center_plates", 0.095, (0.0, 0.0, 0.0), (0.16, 0.08, 0.03), "ESTIMATED", None,
     "AOS 7 V5 bare-frame mass not retrievable (aos-rc.com is a JS-only Wix site). 170 g total frame is the "
     "midpoint of 150-190 g typical for 7-inch 5 mm-arm carbon frames; split 95 g plates/standoffs + 4 x 18.75 g arms."),
    ("frame_arms", 0.075, None, None, "ESTIMATED", None, "four 18.75 g rods from hub to motor, see frame_center_plates"),
    ("motors", 4 * F90["weight_g"] / 1000.0, None, None, "MEASURED_DATASHEET", "TMOTOR_F90",
     "F90 KV1300 Weight (Incl. Cable) 46.6 g x 4, at motor positions, z = +0.017 m (half of 34.7 mm height)"),
    ("props", 4 * 0.0090, None, None, "ESTIMATED", None,
     "HQ 7x4x3 propeller mass ~9 g each (typical 7-inch polycarbonate tri-blade 8-10 g); z = +0.040 m"),
    ("esc_4in1", 0.0158, (0.0, 0.0, -0.004), None, "MEASURED_DATASHEET", "HOLYBRO_65A",
     "Tekko32 F4 Metal 4in1 65A ESC Weight 15.8 g, bottom of stack"),
    ("cube_orange_plus", 0.075, (0.0, 0.0, 0.022), (0.038, 0.038, 0.022), "ESTIMATED", None,
     "CubePilot Cube Orange/Orange+ module, 38.25 x 38.25 x 22.3 mm; vendor mass not retrievable from "
     "docs.cubepilot.org (no weight published on the pages fetched); 70-80 g aluminium-cased cube class"),
    ("cube_mini_carrier", 0.020, (0.0, 0.0, 0.008), (0.045, 0.045, 0.010), "ESTIMATED", None,
     "CubePilot Mini Carrier Board (https://docs.cubepilot.org/user-guides/carrier-boards/mini-carrier-board); "
     "no published mass; small 4-layer PCB + connectors 15-25 g"),
    ("jetson_orin_nano_super", 0.140, (-0.055, 0.0, 0.040), (0.100, 0.080, 0.030), "ESTIMATED", None,
     "Orin Nano (Super mode) module (69.6 x 45 mm SO-DIMM) + compact carrier + heatsink/fan; NVIDIA dev kit "
     "(~176 g with large heatsink) minus oversized carrier; vendor module datasheet requires login"),
    ("l2_regulator", 0.012, (0.030, 0.0, 0.0), None, "ESTIMATED", None,
     "dedicated 12 V regulated branch for the Unitree L2 (6S -> 12 V, 3-5 A buck class 8-15 g)"),
    ("microhard_pmddl900_oem", 0.007, (-0.080, 0.0, 0.005), None, "MEASURED_DATASHEET", "MICROHARD_PMDDL900",
     "pMDDL900 brochure Rev 1.2.1: Weight OEM ~7 grams"),
    ("microhard_carrier", 0.015, (-0.080, 0.0, 0.0), (0.050, 0.035, 0.010), "ESTIMATED", None,
     "interface/carrier board for the OEM module (USB/Ethernet + power), no mass published"),
    ("radio_antennas", 0.020, (-0.100, 0.0, -0.030), None, "ESTIMATED", None,
     "two 900 MHz dipoles for the 2x2 MIMO pMDDL900, ~10 g each incl. pigtail"),
    ("frsky_receiver", 0.005, (-0.060, 0.0, 0.005), None, "ESTIMATED", None,
     "FrSky receiver, model not specified by owner; R-XSR/Archer RS class 1.5-6 g incl. antennas"),
    ("oak_d_s2", 0.091, (0.110, 0.0, 0.010), (0.023, 0.097, 0.030), "MEASURED_DATASHEET", "LUXONIS_OAKDS2",
     "OAK-D S2 Weight 91 g, Dimensions 97 x 29.5 x 22.9 mm; front-mounted facing +x"),
    ("unitree_l2", 0.230, (0.0, 0.0, 0.085), (0.075, 0.075, 0.065), "MEASURED_DATASHEET", "UNITREE_L2",
     "Unitree 4D LiDAR L2 Weight 230 g; top-mounted above the compute stack"),
    ("battery_6s_lipo", 0.290, (0.0, 0.0, -0.040), (0.110, 0.040, 0.040), "ESTIMATED", None,
     "6S 1800 mAh 100C LiPo, 270-300 g class (pack choice not specified by owner); bottom-slung; x solved so the CG "
     "lies on the motor centroid (balance_battery)"),
    ("wiring_mounts_misc", 0.070, (0.0, 0.0, 0.010), (0.12, 0.08, 0.06), "ESTIMATED", None,
     "XT60 + 12 AWG leads, GNSS, standoffs, TPU mounts, straps, Jetson BEC (receiver/radio/regulator itemized)"),
]


def fit_rotor():
    rows = np.array(F90_KV1300_HQ7040, dtype=float)
    thrust = rows[:, 1] * 1e-3 * G
    omega = rows[:, 4] * 2.0 * math.pi / 60.0
    power = rows[:, 5]
    # least squares T = kf w^2 (through origin), relative weighting so the hover region counts
    x = omega ** 2
    w = 1.0 / thrust
    kf = float(np.sum(w * w * x * thrust) / np.sum(w * w * x * x))
    resid = thrust / (kf * x) - 1.0
    # shaft torque Q = eta * P / w; kq by the same relative least squares, km = kq / kf
    torque = ETA_SHAFT * power / omega
    wq = 1.0 / torque
    kq = float(np.sum(wq * wq * x * torque) / np.sum(wq * wq * x * x))
    km = kq / kf
    omega_max = float(omega[-1])
    v_at_max = float(rows[-1, 2])
    ct = kf / (RHO * (1.0 / (2 * math.pi)) ** 2 * PROP_DIAMETER ** 4)
    return dict(kf=kf, km=km, kq=kq, omega_max=omega_max, v_at_max=v_at_max,
                thrust_rel_resid_max=float(np.max(np.abs(resid))),
                thrust_rel_resid_rms=float(np.sqrt(np.mean(resid ** 2))),
                ct=ct, thrust_max_n=float(kf * omega_max ** 2),
                power_rel_resid_max=float(np.max(np.abs(kq * omega ** 3 / ETA_SHAFT / power - 1.0))))


def motor_positions():
    a = WHEELBASE / 2.0 / math.sqrt(2.0)
    # ArduCopter/PX4 QUAD-X order: FR, RL, FL, RR; FR & RL CCW (+1), FL & RR CW (-1)
    return [("front_right", (a, -a), 1), ("rear_left", (-a, a), 1),
            ("front_left", (a, a), -1), ("rear_right", (-a, -a), -1)]


def balance_battery():
    """Builders slide the pack until the CG sits on the motor centroid; solve its x for that."""
    moment = 0.0
    for name, m, pos, *_ in COMPONENTS:
        if pos is not None and name != "battery_6s_lipo":
            moment += m * pos[0]
    for i, c in enumerate(COMPONENTS):
        if c[0] == "battery_6s_lipo":
            x = -moment / c[1]
            COMPONENTS[i] = (c[0], c[1], (x, c[2][1], c[2][2])) + c[3:]
            return x
    raise KeyError("battery")


def mass_properties():
    """Point masses, boxes and rods -> total mass, CoM and inertia about the CoM (FLU)."""
    balance_battery()
    items = []   # (m, pos, local inertia 3x3 about own centre)
    motors = motor_positions()
    for name, m, pos, box, *_ in COMPONENTS:
        if name == "frame_arms":
            for _, (x, y), _ in motors:
                length = math.hypot(x, y)
                u = np.array([x, y, 0.0]) / length
                mi = m / 4.0
                i_rod = mi * length ** 2 / 12.0 * (np.eye(3) - np.outer(u, u))
                items.append((mi, np.array([x / 2, y / 2, 0.0]), i_rod))
        elif name == "motors":
            for _, (x, y), _ in motors:
                r = 0.0167
                i_cyl = (m / 4) * np.diag([r * r / 4 + 0.0347 ** 2 / 12] * 2 + [r * r / 2])
                items.append((m / 4, np.array([x, y, 0.017]), i_cyl))
        elif name == "props":
            for _, (x, y), _ in motors:
                rp = PROP_DIAMETER / 2
                i_disc = (m / 4) * np.diag([rp * rp / 6, rp * rp / 6, rp * rp / 3])
                items.append((m / 4, np.array([x, y, 0.040]), i_disc))
        else:
            if box is None:
                local = np.zeros((3, 3))
            else:
                bx, by, bz = box
                local = m / 12.0 * np.diag([by * by + bz * bz, bx * bx + bz * bz, bx * bx + by * by])
            items.append((m, np.array(pos, dtype=float), local))
    mass = sum(i[0] for i in items)
    com = sum(i[0] * i[1] for i in items) / mass
    inertia = np.zeros((3, 3))
    for m, p, local in items:
        d = p - com
        inertia += local + m * (np.dot(d, d) * np.eye(3) - np.outer(d, d))
    return mass, com, inertia


def motor_time_constant(kq, omega_hover):
    """Linearised electromechanical lag of a voltage-driven BLDC + prop around hover:
    J dw/dt = Kt (V - Ke w)/R - kq w^2  ->  tau = J / (Kt Ke / R + 2 kq w_h)."""
    kt = 60.0 / (2.0 * math.pi * F90["kv"])
    rp = PROP_DIAMETER / 2
    j_prop = 0.0090 * rp * rp / 3.0            # 3 blades as uniform rods (ESTIMATED 9 g)
    j_bell = 0.018 * 0.0155 ** 2               # ~18 g bell as a thin ring r = 15.5 mm (ESTIMATED)
    j = j_prop + j_bell
    tau = j / (kt * kt / F90["resistance_ohm"] + 2.0 * kq * omega_hover)
    return tau, j, kt


def build():
    rot = fit_rotor()
    mass, com, inertia = mass_properties()
    kf, km = rot["kf"], rot["km"]
    omega_hover = math.sqrt(mass * 9.81 / (4 * kf))
    tau, j_rotor, kt = motor_time_constant(rot["kq"], omega_hover)
    rows = np.array(F90_KV1300_HQ7040, dtype=float)
    hover_g = mass * 1000.0 / 4.0 * 9.81 / G
    # linear interpolation of throttle vs thrust; below 30% extrapolate along the first segment
    thr, tg = rows[:, 0], rows[:, 1]
    if hover_g < tg[0]:
        hover_throttle = float(thr[0] + (hover_g - tg[0]) * (thr[1] - thr[0]) / (tg[1] - tg[0]))
    else:
        hover_throttle = float(np.interp(hover_g, tg, thr))
    rotor_drag = DRAG_PER_MASS * mass / (4.0 * omega_hover)
    radius = PROP_DIAMETER / 2
    rotors = []
    for name, (x, y), spin in motor_positions():
        p = np.array([x, y, 0.040]) - com
        rotors.append({"name": name, "position": [round(float(v), 6) for v in p], "spin": spin,
                       "source": "TMOTOR_F90",
                       "locator": f"QUAD-X {name}; wheelbase {WHEELBASE} m (ESTIMATED); prop plane z=+0.040 m; "
                                  "relative to the computed CoM"})
    I = [[round(float(v), 7) for v in row] for row in inertia]
    p = {
        "mass": {"value": round(mass, 4), "unit": "kg", "status": "ESTIMATED", "source": None,
                 "locator": "sum of the itemized component list in 'massBudget' (datasheet items + estimates)"},
        "inertia": {"value": I, "unit": "kg m^2", "status": "ESTIMATED", "source": None,
                    "locator": "parallel-axis sum over the component layout in 'massBudget' "
                               "(boxes, rods, point motors, disc props) about the CoM"},
        "kf": {"value": kf, "unit": "N/(rad/s)^2", "status": "MEASURED_DATASHEET", "source": "TMOTOR_F90",
               "locator": f"relative least-squares T = kf w^2 over the 11-row F90 KV1300 + HQ7040 6S table; "
                          f"max rel residual {rot['thrust_rel_resid_max']:.3f}, rms {rot['thrust_rel_resid_rms']:.3f}; "
                          f"C_T = {rot['ct']:.3f}"},
        "km": {"value": km, "unit": "m", "status": "ESTIMATED", "source": "TMOTOR_F90",
               "locator": f"Q = eta P_elec / w with eta = {ETA_SHAFT} (ESTIMATED motor+ESC efficiency); kq fit by "
                          "relative least squares, km = kq / kf. Electrical power is datasheet; the split is estimated."},
        "tau_up": {"value": round(tau, 4), "unit": "s", "status": "ESTIMATED", "source": "TMOTOR_F90",
                   "locator": f"tau = J/(Kt Ke/R + 2 kq w_hover); J = {j_rotor:.3e} kg m^2 (prop 3 rods 9 g + bell ring 18 g, "
                              f"ESTIMATED), Kt = 60/(2 pi Kv) = {kt:.5f}, R = 76 mOhm (datasheet). Cross-check: PX4 gz x500 "
                              "uses 12.5 ms up / 25 ms down (SIMULATOR_DEFAULT)."},
        "tau_down": {"value": round(tau, 4), "unit": "s", "status": "ESTIMATED", "source": None,
                     "locator": "same linearisation; AM32 complementary-PWM (active braking) makes the small-signal lag "
                                "symmetric. Current-limited large steps are not modelled."},
        "omega_max": {"value": rot["omega_max"], "unit": "rad/s", "status": "MEASURED_DATASHEET", "source": "TMOTOR_F90",
                      "locator": "100% throttle 19617 rpm at 23.48 V (HQ7040)"},
        "rotor_drag": {"value": rotor_drag, "unit": "N s/(m rad)", "status": "ESTIMATED", "source": "FAESSLER2018",
                       "locator": f"aggregate linear rotor-drag {DRAG_PER_MASS} 1/s x mass, split over 4 rotors at hover "
                                  "speed (F = -|w| lambda1 v_perp). Order of magnitude from the identified rotor-drag "
                                  "coefficients of ~1 kg quadrotors in FAESSLER2018; not identified on this airframe."},
        "rolling_moment": {"value": 0.0, "unit": "N s/rad", "status": "ESTIMATED", "source": None,
                           "locator": "neglected (gz-sim term; no data for 7-inch props)"},
        "rotor_radius": {"value": radius, "unit": "m", "status": "MEASURED_DATASHEET", "source": "TMOTOR_F90",
                         "locator": "HQ 7040 test propeller, 7 in diameter"},
        "drag_linear": {"value": [0.0, 0.0, 0.0], "unit": "N s/m", "status": "ESTIMATED", "source": None,
                        "locator": "linear airframe drag is carried by rotor_drag"},
        "drag_quadratic": {"value": round(0.5 * RHO * 1.0 * 0.014, 5), "unit": "N s^2/m^2", "status": "ESTIMATED",
                           "source": None,
                           "locator": "0.5 rho Cd A with Cd 1.0 (bluff body) and A 0.014 m^2 (frame + L2 + battery + OAK "
                                      "frontal area), rotation-consistent |v| v form"},
        "rotor_inertia": {"value": j_rotor, "unit": "kg m^2", "status": "ESTIMATED", "source": None,
                          "locator": "prop + bell, see tau_up"},
        "battery_v_full": {"value": 25.2, "unit": "V", "status": "LITERATURE", "source": "LIPO_STD",
                           "locator": "6S x 4.20 V"},
        "battery_v_empty": {"value": 21.0, "unit": "V", "status": "ESTIMATED", "source": "LIPO_STD",
                            "locator": "6S x 3.50 V linear-OCV endpoint (real LiPo OCV is non-linear)"},
        "battery_v_nominal": {"value": rot["v_at_max"], "unit": "V", "status": "MEASURED_DATASHEET", "source": "TMOTOR_F90",
                              "locator": "bench voltage at which omega_max was measured; omega limit scales with V/V_nom (capped 1)"},
        "battery_resistance": {"value": 0.030, "unit": "ohm", "status": "ESTIMATED", "source": None,
                               "locator": "6 cells x ~4 mOhm (100C 1800 mAh class) + leads/connector ~6 mOhm"},
        "battery_capacity_ah": {"value": 1.8, "unit": "Ah", "status": "ESTIMATED", "source": None,
                                "locator": "assumed 6S 1800 mAh pack, see massBudget"},
        "battery_efficiency": {"value": ETA_SHAFT, "unit": "-", "status": "ESTIMATED", "source": None,
                               "locator": "same eta used to split datasheet electrical power into shaft torque, so the "
                                          f"model reproduces the datasheet power column (max rel err {rot['power_rel_resid_max']:.3f})"},
    }
    budget = []
    for name, m, pos, box, status, src, loc in COMPONENTS:
        budget.append({"item": name, "mass_kg": round(m, 4), "status": status, "source": src, "locator": loc,
                       "position_flu": None if pos is None else list(pos), "box": None if box is None else list(box)})
    doc = {
        "schema": "crane-aerial-vehicle-profile-v1",
        "id": "aos7-v5-f90-1300kv-6s",
        "description": "Owner airframe: AOS 7 V5 7-inch frame, T-Motor F90 2806.5 KV1300 (the 6S 7-inch variant), 7-inch "
                       "tri-blade props (HQ 7040 used by the vendor thrust table), Holybro Tekko32 F4 Metal 65A 4in1 ESC, "
                       "6S LiPo, Jetson Orin Nano, OAK-D S2, Unitree L2. Generated by Tools/Aerial/fit_aos7_profile.py.",
        "evidenceLabel": "IMPLEMENTED (not PHYSICALLY_CALIBRATED: no flight log or thrust-stand data of this airframe)",
        "frames": {"body": "FLU", "world": "ENU z-up", "spin": "+1 = CCW seen from above"},
        "rotorOrder": "ArduCopter/PX4 QUAD-X servo order: FR, RL, FL, RR",
        "derived": {"com_offset_from_frame_center_flu": [round(float(v), 5) for v in com],
                    "hover_rotor_speed": omega_hover, "hover_rpm": omega_hover * 60 / (2 * math.pi),
                    "thrust_to_weight": 4 * rot["thrust_max_n"] / (mass * 9.81),
                    "thrust_max_per_rotor_n": rot["thrust_max_n"], "ct": rot["ct"],
                    "hover_thrust_per_rotor_g": hover_g,
                    "hover_throttle_datasheet": hover_throttle,
                    "hover_throttle_note": "ESC throttle fraction interpolated from the F90 table (extrapolated "
                                           "below 30%); the model's thrust fraction at hover is (w_h/w_max)^2",
                    "hover_thrust_fraction_model": (omega_hover / rot["omega_max"]) ** 2,
                    "thrust_to_weight_flag": "OK (>= 2.5)" if 4 * rot["thrust_max_n"] / (mass * 9.81) >= 2.5
                                             else "LOW (< 2.5): stopping distance / speed envelope degraded"},
        "sources": {
            "TMOTOR_F90": {"url": TMOTOR_URL, "archive": TMOTOR_ARCHIVE, "sha256_of_archived_html": TMOTOR_SHA,
                           "retrieved": "2026-11-09", "license": "vendor web page; numeric measurements cited as facts",
                           "note": "Specification and Test Data tables, F90 KV1300 + HQ 7040; live URL now redirects, values "
                                   "read from the 2025-05-24 Wayback snapshot"},
            "HOLYBRO_65A": {"url": "https://holybro.com/products/tekko32-f4-metal-4in1-65a-esc-65a", "retrieved": "2026-11-09",
                            "license": "vendor web page; facts only"},
            "LUXONIS_OAKDS2": {"url": "https://shop.luxonis.com/products/oak-d-s2", "retrieved": "2026-11-09",
                               "license": "vendor web page; facts only"},
            "UNITREE_L2": {"url": "https://www.unitree.com/L2/", "retrieved": "2026-11-09",
                           "license": "vendor web page; facts only"},
            "FAESSLER2018": {"url": "https://doi.org/10.1109/LRA.2017.2776353",
                             "citation": "M. Faessler, A. Franchi, D. Scaramuzza, Differential Flatness of Quadrotor Dynamics "
                                         "Subject to Rotor Drag for Accurate Tracking of High-Speed Trajectories, IEEE RA-L 3(2), 2018",
                             "license": "paper; model form and order of magnitude cited"},
            "CHEESEMAN1955": {"url": "https://reports.aerade.cranfield.ac.uk/handle/1826.2/3590",
                              "citation": "I. C. Cheeseman, W. E. Bennett, The Effect of the Ground on a Helicopter Rotor in "
                                          "Forward Flight, ARC R&M 3021, 1955", "license": "report; model cited"},
            "MICROHARD_PMDDL900": {"url": "https://www.microhardcorp.com/brochures/pMDDL900.Brochure.Rev.1.2.1.pdf",
                                   "sha256": "ab021935e4fbe1778e3c9c9f8012b6572426f08052c9286d3d30a442ff1646f7",
                                   "retrieved": "2026-11-09", "license": "vendor brochure; facts only"},
            "MIL_DRYDEN": {"url": "http://everyspec.com/MIL-SPECS/MIL-SPECS-MIL-F/MIL-F-8785C_5295/",
                           "citation": "MIL-F-8785C (1980) / MIL-HDBK-1797 Dryden low-altitude turbulence model",
                           "license": "US government specification"},
            "LIPO_STD": {"url": "https://en.wikipedia.org/wiki/Lithium_polymer_battery",
                         "license": "CC BY-SA; standard 4.20 V/cell full charge"},
        },
        "parameters": p,
        "rotors": rotors,
        "massBudget": budget,
        "notes": [
            "Ground effect uses Cheeseman-Bennett (CHEESEMAN1955) with rotor_radius; turbulence is Dryden (MIL_DRYDEN) "
            "in CraneDrydenTurbulence; neither has an airframe-specific parameter.",
            "Rotor positions are relative to the computed CoM; the CoM offset from the frame centre is in 'derived'.",
        ],
    }
    return doc


def main():
    doc = build()
    text = json.dumps(doc, indent=1) + "\n"
    if "--check" in sys.argv:
        if OUT.read_text(encoding="utf-8") != text:
            print("profile out of date; rerun fit_aos7_profile.py")
            return 1
        print("profile up to date")
        return 0
    OUT.write_text(text, encoding="utf-8")
    d = doc["derived"]
    print(f"wrote {OUT}")
    print(f"mass {doc['parameters']['mass']['value']} kg, inertia diag "
          f"{[doc['parameters']['inertia']['value'][i][i] for i in range(3)]}")
    print(f"kf {doc['parameters']['kf']['value']:.4e} km {doc['parameters']['km']['value']:.4f} "
          f"tau {doc['parameters']['tau_up']['value']} hover {d['hover_rpm']:.0f} rpm T/W {d['thrust_to_weight']:.2f} "
          f"com {d['com_offset_from_frame_center_flu']}")
    print(f"hover {d['hover_thrust_per_rotor_g']:.0f} g/rotor, datasheet throttle {d['hover_throttle_datasheet']:.3f}, "
          f"model thrust fraction {d['hover_thrust_fraction_model']:.3f}, T/W flag {d['thrust_to_weight_flag']}")
    for b in doc["massBudget"]:
        print(f"  {b['item']:24s} {b['mass_kg']*1000:7.1f} g  {b['status']}")
    print(doc["parameters"]["kf"]["locator"])
    print(doc["parameters"]["battery_efficiency"]["locator"])
    return 0


if __name__ == "__main__":
    sys.exit(main())
