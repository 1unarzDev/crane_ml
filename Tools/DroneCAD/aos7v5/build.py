#!/usr/bin/env python3
"""Parametric CAD for the AOS 7 V5 SAR quad.  build123d solids (mm) -> mass properties -> STEP, GLB, OBJ,
per-link STL, URDF and the multirotor sidecar.   Usage (venv ~/.cache/research/cad-venv):
    python build.py [--out-dir DIR] [--iter N] [--no-urdf]
Everything is driven by params.py.  Output SI (m, kg) except the STEP file (mm).
"""
import argparse, json, math, os, sys
from dataclasses import dataclass, field
from pathlib import Path

import numpy as np
import trimesh
from build123d import (Box, Cylinder, Compound, Face, Plane, Pos, Rot, Solid, Vector, Wire, export_step)

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))
import params as P  # noqa: E402

MAT = {  # name -> (rgba, metallic, roughness)
    "carbon": ((0.07, 0.07, 0.08, 1), 0.3, 0.45), "orange": ((0.95, 0.38, 0.04, 1), 0.5, 0.35),
    "gold": ((0.80, 0.60, 0.16, 1), 0.9, 0.3), "black": ((0.03, 0.03, 0.035, 1), 0.1, 0.4),
    "alu": ((0.68, 0.68, 0.70, 1), 0.9, 0.35), "pcb": ((0.05, 0.08, 0.07, 1), 0.2, 0.5),
    "copper": ((0.72, 0.38, 0.14, 1), 0.9, 0.3), "yellow": ((0.92, 0.72, 0.05, 1), 0.2, 0.5),
    "grey": ((0.45, 0.45, 0.47, 1), 0.1, 0.6), "white": ((0.9, 0.9, 0.9, 1), 0.0, 0.6),
    "green": ((0.1, 0.9, 0.25, 1), 0.0, 0.3), "glass": ((0.02, 0.03, 0.04, 1), 0.1, 0.1),
    "prop": ((0.12, 0.12, 0.13, 1), 0.0, 0.4), "beige": ((0.85, 0.8, 0.65, 1), 0.0, 0.6),
    "lipo": ((0.05, 0.05, 0.06, 1), 0.0, 0.55),
}


@dataclass
class Body:
    name: str
    shape: object
    mat: str
    group: str = "all"
    density_rel: float = 1.0
    visual: object = None              # optional trimesh (vehicle frame, mm) used for the visual export instead of the shape (shape = mass/collision proxy)


@dataclass
class Link:
    name: str
    anchor: tuple                       # mm, in base frame
    bodies: list = field(default_factory=list)
    group_mass: dict = field(default_factory=dict)     # group -> g (distributed by volume*density_rel)
    auto_density: dict = field(default_factory=dict)   # group -> g/cc: mass computed from the B-rep volume (standoffs, plates, screws)
    parent: str = "base_link"
    joint: str = "fixed"
    axis: tuple = (0, 0, 1)
    rpy: tuple = (0, 0, 0)
    collision: list = field(default_factory=list)      # (kind, size tuple, centre abs mm, yaw rad)


# ------------------------------------------------------------ geometry helpers
def box(sx, sy, sz, c):
    return Pos(*c) * Box(sx, sy, sz)


def cyl(r, h, c, axis="z"):
    s = Cylinder(r, h)
    if axis == "x":
        s = Rot(0, 90, 0) * s
    elif axis == "y":
        s = Rot(90, 0, 0) * s
    return Pos(*c) * s


def tube_between(p0, p1, r):
    p0, p1 = np.array(p0, float), np.array(p1, float)
    d = p1 - p0
    L = np.linalg.norm(d)
    u = d / L
    zax = np.array([0, 0, 1.0])
    axis = np.cross(zax, u)
    ang = math.degrees(math.acos(np.clip(zax @ u, -1, 1)))
    s = Cylinder(r, L)
    if np.linalg.norm(axis) > 1e-9:
        from build123d import Axis
        s = s.rotate(Axis((0, 0, 0), tuple(axis)), ang)
    return Pos(*((p0 + p1) / 2)) * s


def prism(pts, z0, z1):
    f = Face(Wire.make_polygon([Vector(x, y, z0) for x, y in pts], close=True))
    return Solid.extrude(f, Vector(0, 0, z1 - z0))


def holes(shape, pts, r, z0, z1):
    """Cut vertical round holes (r mm) through [z0, z1] at xy points."""
    for x, y in pts:
        shape = shape - cyl(r, z1 - z0 + 2.0, (x, y, (z0 + z1) / 2))
    return shape


def slot(shape, x, y, length, width, z0, z1, ang_deg=0.0):
    """Cut a rounded slot (pill), long axis at ang_deg, through [z0, z1]."""
    h = z1 - z0 + 2.0
    s = Pos(x, y, (z0 + z1) / 2) * Rot(0, 0, ang_deg) * (Box(max(length - width, 0.1), width, h)
        + Pos(-(length - width) / 2, 0, 0) * Cylinder(width / 2, h) + Pos((length - width) / 2, 0, 0) * Cylinder(width / 2, h))
    return shape - s


def standoff(x, y, z0, z1, mat="gold", od=None):
    od = od or P.STANDOFF["od"]
    return cyl(od / 2, z1 - z0, (x, y, (z0 + z1) / 2)) - cyl(P.STANDOFF["id"] / 2, z1 - z0 + 2, (x, y, (z0 + z1) / 2))


def screw_head(x, y, z, up=True, r=2.7, h=1.8):
    return cyl(r, h, (x, y, z + (h / 2 if up else -h / 2)))


# ------------------------------------------------------------ assembly
F, M, PR = P.FRAME, P.MOTOR, P.PROP
MX, MY = F["motor_dx"], F["motor_dy"]
ARM_TOP = F["arm_t"] / 2
Z_BOT0 = -F["arm_t"] / 2 - F["bot_t"]
Z_TOP1 = ARM_TOP + F["top_t"]                        # 6
Z_UPPER0 = Z_TOP1 + F["stack_gap"]                    # 28
Z_UPPER1 = Z_UPPER0 + F["upper_t"]                    # 30
D = P.DECK
DECK_BOT, DECK_TOP = D["z0"], D["z0"] + D["t"]
assert abs(Z_UPPER1 + D["front_len"] - DECK_BOT) < 1e-6 and abs(Z_TOP1 + D["rear_len"] - DECK_BOT) < 1e-6, "deck standoff lengths do not reach the deck"
PAYLOAD = True
BATT_TOP = Z_BOT0 - P.BATTERY["pad_gap"]
OAK_Z = 1.0 - P.OAK["dims"][2] / 2 - 0.0          # housing top at z=1: 3 mm bracket strap up to the top-plate underside (z=4)
OAK_Z = 1.0 - P.OAK["dims"][2] / 2 if False else (1.0 - 29.5 / 2)
CUBE_Z0 = Z_UPPER1 + P.CUBE["standoff_h"]


def motor_xy(i):
    sx, sy = P.ROTOR_SIGN[i]
    return sx * MX, sy * MY


def arm_polygon(sx, sy):
    rx, ry = F["arm_root"]
    root = np.array([sx * rx, sy * ry])
    tip = np.array([sx * MX, sy * MY])
    d = (tip - root) / np.linalg.norm(tip - root)
    n = np.array([-d[1], d[0]])
    w0, w1 = F["arm_width_root"] / 2, F["arm_width_tip"] / 2
    pts = [root + n * w0, tip + n * w1, tip - n * w1, root - n * w0]
    return [tuple(p) for p in pts]


def motor_screw_pts(i):
    """19 x 19 mm M3 pattern aligned with the arm direction (DATASHEET pattern, ESTIMATE orientation)."""
    sx, sy = P.ROTOR_SIGN[i]
    x, y = motor_xy(i)
    rx, ry = F["arm_root"]
    d = np.array([x - sx * rx, y - sy * ry]); d /= np.linalg.norm(d)
    n = np.array([-d[1], d[0]])
    h = F["motor_pattern"] / 2
    return [(x + a * h * d[0] + b * h * n[0], y + a * h * d[1] + b * h * n[1]) for a in (-1, 1) for b in (-1, 1)]


def stack_pts():
    return F["stack_standoffs"]


def build_frame():
    L = Link("base_link", (0, 0, 0))
    z4 = ARM_TOP
    cd = F["carbon_density"]
    for i in range(4):
        sx, sy = P.ROTOR_SIGN[i]
        arm = prism(arm_polygon(sx, sy), -z4, z4) + cyl(F["arm_pad_r"], F["arm_t"], (sx * MX, sy * MY, 0))
        arm = holes(arm, motor_screw_pts(i), 1.6, -z4, z4)
        arm = holes(arm, [(sx * MX, sy * MY)], 5.0, -z4, z4)
        # lightening holes along the arm (kit arms are 8 mm solid carbon; these are modest Ø7 relief holes)
        rx, ry = F["arm_root"]
        for t in (0.30, 0.5, 0.7):
            arm = holes(arm, [(sx * (rx + t * (MX - rx)), sy * (ry + t * (MY - ry)))], 3.5, -z4, z4)
        L.bodies.append(Body(f"arm_{i}", arm, "carbon", "kit", cd))
    bot = prism(F["bottom_plate"], Z_BOT0, -z4)
    bot = holes(bot, [(x, y) for x, y in stack_pts()[:4]], 1.6, Z_BOT0, -z4)
    for xs in F["strap_x"]:
        for sy in (-1, 1):
            bot = slot(bot, xs, sy * 23.3, F["strap_slot"][0], F["strap_slot"][1], Z_BOT0, -z4, 0)
    for x, y in [(-60, 0), (80, 0), (30, 0)]:
        bot = slot(bot, x, y, 22, 9, Z_BOT0, -z4, 0)
    bot = slot(bot, 6.0, -26.0, 12, 8, Z_BOT0, -z4, 0)
    L.bodies.append(Body("bottom_plate", bot, "carbon", "kit", cd))
    top = prism(F["top_plate"], z4, Z_TOP1)
    top = holes(top, stack_pts(), 1.6, z4, Z_TOP1)
    top = holes(top, [(x, y) for x, y in D["rear_standoffs"]], 1.6, z4, Z_TOP1)
    top = holes(top, P.OAK["bracket_screws"], 1.6, z4, Z_TOP1)
    top = slot(top, 6.0, -26.0, 12, 8, z4, Z_TOP1, 0)
    for x, y in [(-112, 17), (-112, -17), (-132, 0)]:
        top = slot(top, x, y, 14, 7, z4, Z_TOP1, 90 if y == 0 else 0)
    L.bodies.append(Body("top_plate", top, "carbon", "kit", cd))
    cx, cy = P.CUBE["pos_xy"]
    up = prism(F["upper_plate"], Z_UPPER0, Z_UPPER1)
    up = holes(up, stack_pts(), 1.6, Z_UPPER0, Z_UPPER1)
    up = holes(up, D["front_standoffs"], 1.6, Z_UPPER0, Z_UPPER1)
    up = holes(up, [(cx + dx, cy + dy) for dx, dy in P.CUBE["carrier_holes"]], 1.6, Z_UPPER0, Z_UPPER1)
    for x, y in [(-30, 0), (-8, 0), (-40, 30), (-40, -30)]:
        up = slot(up, x, y, 12, 7, Z_UPPER0, Z_UPPER1, 0)
    up = slot(up, 60, 0, 14, 6, Z_UPPER0, Z_UPPER1, 0) if False else up
    L.bodies.append(Body("upper_plate", up, "carbon", "kit", cd))
    for k, (x, y) in enumerate(stack_pts()):
        L.bodies.append(Body(f"stack_standoff_{k}", standoff(x, y, Z_TOP1, Z_UPPER0), "gold", "kit", 1.0))
        L.bodies.append(Body(f"stack_screw_{k}", screw_head(x, y, Z_UPPER1), "alu", "kit", 1.0))
    # kit hardware: motor screws from below (M3 socket heads under the arm pads)
    for i in range(4):
        for k, (x, y) in enumerate(motor_screw_pts(i)):
            L.bodies.append(Body(f"motor_screw_{i}_{k}", screw_head(x, y, -z4, up=False, r=2.6, h=2.0), "alu", "kit", 1.0))
    # kit battery straps: loop around the pack and through the bottom-plate slots (run on the plate top between the slots)
    pw = P.BATTERY["dims"][1]
    zb = BATT_TOP - P.BATTERY["dims"][2]
    for xs in F["strap_x"]:
        for sy in (-1, 1):
            L.bodies.append(Body(f"strap_{int(xs)}_{sy}", box(F["strap_w"], 1.6, BATT_TOP - zb + 0.0 + 6.5 + 4.0 - 4.0, (xs, sy * 23.3, (zb + -4.0) / 2 - 0.0)), "black", "kit", 0.5))
        L.bodies.append(Body(f"strap_bot_{int(xs)}", box(F["strap_w"], 2 * 23.3 + 1.6, 1.6, (xs, 0, zb - 0.8)), "black", "kit", 0.5))
        L.bodies.append(Body(f"strap_top_{int(xs)}", box(F["strap_w"], 2 * 23.3 + 1.6, 1.6, (xs, 0, -z4 + 0.8)), "black", "kit", 0.5))
    # 7075 camera-cage tabs at the nose (inside the kit mass)
    L.group_mass = {"kit": F["mass_g"]}
    L.collision = [("box", (200, 80, 12.5), (-10, 0, -0.25), 0.0)]
    for i in range(4):
        sx, sy = P.ROTOR_SIGN[i]
        L.collision.append(("box", (math.hypot(MX - 34, MY - 14), 22, 8), (sx * (MX + 34) / 2, sy * (MY + 14) / 2, 0),
                            math.atan2(sy * (MY - 14), sx * (MX - 34))))
    return L


def blade_solid(spin, k):
    """One tri-blade-prop blade.  Chord runs along y; for CCW (spin=+1) the leading edge is on +y and is pitched UP so the blade
    pushes air down (thrust +z); CW is the mirror image."""
    R = PR["diameter"] / 2
    rs = np.linspace(PR["hub_r"], R, 9)

    def chord(r):
        t = (r - PR["hub_r"]) / (R - PR["hub_r"])
        if t < 0.6:
            return PR["chord_root"] + (PR["chord_peak"] - PR["chord_root"]) * (t / 0.6)
        return PR["chord_peak"] + (PR["chord_tip"] - PR["chord_peak"]) * ((t - 0.6) / 0.4) ** 1.3
    le = [(r, spin * 0.35 * chord(r)) for r in rs]
    te = [(r, -spin * 0.65 * chord(r)) for r in rs[::-1]]
    th = PR["thickness"]
    s = prism(le + te, -th / 2, th / 2)
    s = Rot(spin * PR["pitch_deg"], 0, 0) * s
    return Rot(0, 0, 120.0 * k) * s


def build_rotor(i):
    x, y = motor_xy(i)
    spin = P.SPIN[i]
    z_prop = PR["plane_z"]
    st = Link(f"motor_{i}", (x, y, ARM_TOP))
    st.bodies.append(Body("mount_base", cyl(M["base_r"], M["base_h"], (x, y, ARM_TOP + M["base_h"] / 2)), "alu", "all"))
    z_w0 = ARM_TOP + M["base_h"]
    z_bell0 = ARM_TOP + M["height"] - M["bell_h"]
    st.bodies.append(Body("windings", cyl(M["stator_r"], z_bell0 - z_w0, (x, y, (z_w0 + z_bell0) / 2)), "copper", "all"))
    st.group_mass = {"all": M["mass_g"] - M["bell_mass_g"]}
    st.collision = [("cyl", (M["base_r"], M["height"]), (x, y, ARM_TOP + M["height"] / 2), 0.0)]
    ro = Link(f"rotor_{i}", (x, y, z_prop), parent=f"motor_{i}", joint="continuous")
    bell = cyl(M["r_bell"], M["bell_h"], (x, y, z_bell0 + M["bell_h"] / 2))
    inner = cyl(M["r_bell"] - M["bell_wall"], M["bell_h"] - 2.0, (x, y, z_bell0 + (M["bell_h"] - 2.0) / 2 - 0.01))
    ro.bodies.append(Body("bell", bell - inner, "black", "bell"))
    ro.bodies.append(Body("shaft_nut", cyl(PR["nut_r"], PR["nut_h"], (x, y, z_prop + PR["hub_h"] / 2 + PR["nut_h"] / 2 - 1.0)), "alu", "bell"))
    ro.bodies.append(Body("hub", cyl(PR["hub_r"], PR["hub_h"], (x, y, z_prop)), "prop", "prop"))
    for k in range(PR["blades"]):
        ro.bodies.append(Body(f"blade_{k}", Pos(x, y, z_prop) * blade_solid(spin, k), "prop", "prop"))
    ro.group_mass = {"bell": M["bell_mass_g"], "prop": PR["mass_g"]}
    return st, ro


def build_esc():
    d = P.ESC
    L = Link("esc_link", d["pos"])
    x, y, z = d["pos"]
    pcb = box(d["dims"][0], d["dims"][1], 1.6, (x, y, z - d["dims"][2] / 2 + 0.8))
    pcb = holes(pcb, stack_pts()[:4], 3.1, z - 4, z - 2)
    L.bodies.append(Body("pcb", pcb, "pcb"))
    L.bodies.append(Body("fets", box(24.8, 34.0, d["dims"][2] - 1.6, (x, y, z + 0.8)), "black"))
    for sy in (-1, 1):
        L.bodies.append(Body(f"cap_{sy}", cyl(d["cap_r"], d["cap_len"], (x - d["dims"][0] / 2 - 4, y + sy * 8, z + 1), "y"), "black"))
    L.bodies.append(Body("xt60", box(*d["xt60"], (x + 6, y - d["dims"][1] / 2 - 5, z + 2)), "yellow"))
    L.bodies.append(Body("xt60b", box(*d["xt60"], (x - 6, y - d["dims"][1] / 2 - 5, z + 2)), "yellow"))
    L.group_mass = {"all": d["mass_g"]}
    L.collision = [("box", d["dims"], d["pos"], 0.0)]
    return L


def build_cube():
    c = P.CUBE
    x, y = c["pos_xy"]
    zb = CUBE_Z0
    L = Link("cube_carrier_link", (x, y, zb))
    cx, cy, cz = c["carrier_dims"]
    pcb = box(cx, cy, cz, (x, y, zb + cz / 2))
    pcb = holes(pcb, [(x + dx, y + dy) for dx, dy in c["carrier_holes"]], 1.74, zb, zb + cz)
    L.bodies.append(Body("carrier_pcb", pcb, "pcb", "carrier"))
    dx, dy, dz = c["dims"]
    ch = c["connector_h"]
    cube_dy = -4.6                                    # cube is centred towards the -y end of the carrier (STEP), JST strip at +y
    L.bodies.append(Body("header", box(30.0, 30.0, ch, (x, y + cube_dy, zb + cz + ch / 2)), "black", "carrier"))
    L.bodies.append(Body("cube_case", box(dx, dy, dz - ch, (x, y + cube_dy, zb + cz + ch + (dz - ch) / 2)), "orange", "cube"))
    for k, sx in enumerate((-9.0, 9.0)):
        L.bodies.append(Body(f"jst_{k}", box(16.0, 7.7, 7.5, (x + sx, y + 19.6, zb + cz + 3.75)), "beige", "carrier"))
    L.bodies.append(Body("cube_label", box(22, 14, 0.3, (x, y + cube_dy, zb + cz + dz + 0.15)), "white", "cube"))
    # forward arrow on the cube lid (+x = vehicle forward)
    a = Face(Wire.make_polygon([Vector(x + 11, y + cube_dy, 0), Vector(x + 1, y + cube_dy + 6, 0), Vector(x + 1, y + cube_dy - 6, 0)], close=True))
    L.bodies.append(Body("cube_arrow", Pos(0, 0, zb + cz + dz + 0.3) * Solid.extrude(a, Vector(0, 0, 0.3)), "black", "cube"))
    for k, (hx, hy) in enumerate(c["carrier_holes"]):
        L.bodies.append(Body(f"spacer_{k}", standoff(x + hx, y + hy, zb - c["standoff_h"], zb, "alu", od=5.0), "gold", "spacer"))
        L.bodies.append(Body(f"spacer_screw_{k}", screw_head(x + hx, y + hy, zb + cz, r=2.4, h=1.6), "alu", "spacer"))
    L.group_mass = {"carrier": c["carrier_mass_g"], "cube": c["mass_g"]}
    L.auto_density = {"spacer": 2.7}
    L.collision = [("box", (cx, cy, cz + dz - 0), (x, y, zb + (cz + dz) / 2), 0.0)]
    imu = (x, y + cube_dy, zb + cz + ch + (dz - ch) / 2)
    return L, imu


def l2_origin():
    return P.L2["pos_xy"][0], P.L2["pos_xy"][1], DECK_TOP


def l2_holes():
    r = P.L2["hole_circle_d"] / 2
    x, y, _ = l2_origin()
    return [(x + r * math.cos(math.radians(a)), y + r * math.sin(math.radians(a))) for a in P.L2["hole_angles_deg"]]


def build_l2():
    d = P.L2
    x, y, zb = l2_origin()
    L = Link("l2_link", (x, y, zb))
    bh = d["base_h"]
    L.bodies.append(Body("base", cyl(d["dims"][0] / 2, bh, (x, y, zb + bh / 2)), "black"))
    L.bodies.append(Body("head", cyl(d["head_r"], d["dims"][2] - bh - 4, (x, y, zb + bh + (d["dims"][2] - bh - 4) / 2)), "black"))
    L.bodies.append(Body("window", cyl(d["head_r"] + 0.4, 27, (x, y, zb + d["scan_origin_z"])), "glass"))
    L.bodies.append(Body("cap", cyl(d["head_r"] - 2, 4, (x, y, zb + d["dims"][2] - 2)), "black"))
    L.bodies.append(Body("led", cyl(2.5, 1.2, (x + 26, y - 20, zb + bh - 3.0)), "green"))
    L.group_mass = {"all": d["mass_g"]}
    L.collision = [("box", d["dims"], (x, y, zb + d["dims"][2] / 2), 0.0)]
    return L


def build_jetson():
    d = P.JETSON
    x, y = d["pos_xy"]
    zb = DECK_TOP + d["standoff_h"]
    L = Link("jetson_link", (x, y, zb))
    dlong, dshort, dz = d["dims"]                       # carrier 100 along y (across the vehicle), 79 along x
    t = d["pcb_t"]
    hp = d["hole_pattern"]                              # 86 x 58: 86 along y (long axis across the vehicle), 58 along x
    pts = [(x + sa * hp[1] / 2, y + sb * hp[0] / 2) for sa in (-1, 1) for sb in (-1, 1)]
    pcb = holes(Pos(x, y, zb + t / 2) * Box(dshort, dlong, t), pts, 1.6, zb, zb + t)
    L.bodies.append(Body("carrier_pcb", pcb, "pcb"))
    L.bodies.append(Body("module_riser", box(45, 69.6, 7.0, (x + 0.0, y + 0.0, zb + t + 3.5)), "pcb"))
    L.bodies.append(Body("heatsink", box(56, 70, 7.4, (x, y, zb + t + 7.0 + 3.7)), "black"))
    L.bodies.append(Body("fan", cyl(19.0, 6.0, (x, y, zb + t + 14.4 + 3.0)), "black"))
    # ports on the aft (-x) long edge, header on the forward (+x) edge
    L.bodies.append(Body("usb_stack", box(17.0, 28.0, 15.5, (x - dshort / 2 + 6.0, y - 22.0, zb + t + 7.75)), "alu"))
    L.bodies.append(Body("usb_stack2", box(17.0, 28.0, 15.5, (x - dshort / 2 + 6.0, y + 8.0, zb + t + 7.75)), "alu"))
    L.bodies.append(Body("rj45", box(16.0, 15.0, 13.5, (x - dshort / 2 + 6.0, y + 30.0, zb + t + 6.75)), "alu"))
    L.bodies.append(Body("header40", box(5.0, 51.0, 8.5, (x + dshort / 2 - 4.0, y + 4.0, zb + t + 4.25)), "black"))
    L.bodies.append(Body("header_pins", box(2.6, 50.0, 6.0, (x + dshort / 2 - 4.0, y + 4.0, zb + t + 8.5 + 3.0)), "gold"))
    L.group_mass = {"all": d["mass_g"]}
    L.collision = [("box", (dshort, dlong, dz + 1), (x, y, zb + dz / 2), 0.0)]
    return L


def build_eth():
    d = P.ETH
    x, y = d["pos_xy"]
    zb = DECK_TOP
    L = Link("usb_eth_link", (x, y, zb))
    L.bodies.append(Body("adapter", box(*d["dims"], (x, y, zb + d["dims"][2] / 2)), "black"))
    L.bodies.append(Body("label", box(34, 14, 0.3, (x, y, zb + d["dims"][2] + 0.15)), "grey"))
    L.bodies.append(Body("rj_end", box(14, 12, 10, (x - 36, y, zb + 6)), "grey"))
    L.bodies.append(Body("cable", tube_between((x + 30, y, zb + 6), (x + 36, y + 0.0, zb + 6), 2.6), "grey"))
    hw, hz = d["dims"][1] / 2, d["dims"][2]
    for sx in (-14.0, 14.0):                                # hook-and-loop band: over the adapter, down through two deck slots, under the deck
        yb = hw + 0.6
        band = box(5.0, 1.2, hz + 1.2 + 2.0, (x + sx, y + yb, zb + (hz + 1.2 - 2.0) / 2)) + box(5.0, 1.2, hz + 1.2 + 2.0, (x + sx, y - yb, zb + (hz + 1.2 - 2.0) / 2)) \
            + box(5.0, 2 * yb + 1.2, 1.2, (x + sx, y, zb + hz + 0.6)) + box(5.0, 2 * yb + 1.2, 1.2, (x + sx, y, zb - 2.0 - 0.6))
        L.bodies.append(Body(f"band_{int(sx)}", band, "black", "band"))
    L.group_mass = {"all": d["mass_g"], "band": 0.5}
    L.collision = [("box", d["dims"], (x, y, zb + d["dims"][2] / 2), 0.0)]
    return L


def build_radio():
    d = P.RADIO
    x, y = d["pos_xy"]
    zb = DECK_TOP + d["standoff_h"]
    L = Link("radio_link", (x, y, zb))
    cx, cy, cz = d["carrier_dims"]
    mx, my, mz = d["module_dims"]
    hp = d["hole_pattern"]
    carrier = holes(box(cx, cy, cz, (x, y, zb + cz / 2)), [(x + sa * hp[0] / 2, y + sb * hp[1] / 2) for sa in (-1, 1) for sb in (-1, 1)], 1.3, zb, zb + cz)
    L.bodies.append(Body("carrier", carrier, "black", "carrier"))
    L.bodies.append(Body("module", box(mx, my, mz, (x - 4, y, zb + cz + mz / 2)), "alu", "module"))
    a = d["antenna"]
    gm = {"carrier": d["carrier_mass_g"], "module": d["module_mass_g"]}
    for k, (bx, by) in enumerate(a["base_xy"]):
        base = np.array([bx, by, DECK_TOP])
        sy = 1 if by > 0 else -1
        t_out, t_back = math.radians(a["tilt_out_deg"]), math.radians(a["tilt_back_deg"])
        u = np.array([-math.sin(t_back), sy * math.sin(t_out) * (1 if by > y else -1) , math.cos(t_out) * math.cos(t_back)])
        u /= np.linalg.norm(u)
        L.bodies.append(Body(f"sma_{k}", cyl(3.6, 9, (bx, by, DECK_TOP + 4.5)), "gold", f"ant{k}"))
        L.bodies.append(Body(f"antenna_{k}", tube_between(base + np.array([0, 0, 9]) + u * 0, base + np.array([0, 0, 9]) + u * (a["length"]), a["r"]), "black", f"ant{k}"))
        gm[f"ant{k}"] = a["mass_g"]
    L.group_mass = gm
    L.collision = [("box", (cx + 6, cy, cz + mz), (x - 3, y, zb + (cz + mz) / 2), 0.0)]
    return L


def build_rx():
    d = P.RX
    L = Link("rx_link", d["pos"])
    x, y, z = d["pos"]
    L.bodies.append(Body("rx", box(*d["dims"], (x, y, z)), "black"))
    L.bodies.append(Body("rx_ant", tube_between((x - 8, y, z), (x - 8 - 30, y + 20, z + 6), 0.7), "black"))
    L.group_mass = {"all": d["mass_g"]}
    return L


def oak_visual_mesh():
    """MIT-licensed Luxonis enclosure mesh (decimated from DM9098S2_enclosure.stp) in the vehicle frame, or None if absent."""
    p = HERE / "vendor_derived" / "oak_d_s2_enclosure_stepframe.stl"
    if not p.exists():
        return None
    m = trimesh.load(p)
    # STEP frame: x = width (97, -4..93), y = height (-26.5..3), z = depth (front face +z at 10.1) -> vehicle: x fwd = z, y left = x, z up = y
    v = m.vertices
    ox, oy, oz = P.OAK["pos"]
    face = ox + P.OAK["dims"][0] / 2
    nv = np.c_[v[:, 2] - 10.1 + face, v[:, 0] - 44.5, v[:, 1] + 11.75 + OAK_Z]
    out = trimesh.Trimesh(nv, m.faces, process=False)
    return out


def build_oak():
    d = P.OAK
    ox, oy, _ = d["pos"]
    oz = OAK_Z
    L = Link("oak_d_link", (ox, oy, oz))
    dx, dy, dz = d["dims"]
    L.bodies.append(Body("housing", box(dx, dy, dz, (ox, oy, oz)), "black", "cam", visual=oak_visual_mesh()))
    face = ox + dx / 2
    for sy, nm in ((1, "left"), (-1, "right"), (0, "rgb")):
        L.bodies.append(Body(f"lens_{nm}", cyl(5.5 if sy else 6.5, 1.2, (face + 0.1, oy + sy * d["baseline_mm"] / 2, oz + 2.0), "x"), "glass", "cam"))
    L.group_mass = {"cam": d["mass_g"]}
    L.collision = [("box", (dx + 6, dy + 2 * d["bracket_t"], dz + 8), (ox + 1, oy, oz), 0.0)]
    return L


def build_oak_bracket():
    d = P.OAK
    ox, oy, _ = d["pos"]
    oz = OAK_Z
    t = d["bracket_t"]
    dx, dy, dz = d["dims"]
    L = Link("oak_bracket_link", (ox, oy, oz), parent="base_link")
    top_z0 = oz + dz / 2                                  # housing top (z=1) .. plate underside (z=4)
    bot_z = oz - dz / 2 - 2.0
    h = Z_TOP1 - 2.0 - bot_z                              # cheeks run up to the plate underside (z=4)
    cheek_h = 4.0 - bot_z
    for sy, nm in ((1, "l"), (-1, "r")):
        L.bodies.append(Body(f"cheek_{nm}", box(dx + 6, t, cheek_h, (ox + 1, oy + sy * (dy / 2 + t / 2), bot_z + cheek_h / 2)), "gold", "brk"))
    strap = box(dx + 6, dy + 2 * t, 4.0 - top_z0, (ox + 1, oy, (4.0 + top_z0) / 2))
    strap = holes(strap, d["bracket_screws"], 1.6, top_z0, 4.0)
    L.bodies.append(Body("top_strap", strap, "gold", "brk"))
    for k, (sx, sy) in enumerate(d["bracket_screws"]):
        L.bodies.append(Body(f"screw_{k}", screw_head(sx, sy, Z_TOP1, r=2.6, h=1.8), "alu", "brk"))
    L.group_mass = {"brk": d["bracket_mass_g"]}
    L.collision = []
    return L


def build_battery(px, py):
    d = P.BATTERY
    dx, dy, dz = d["dims"]
    zc = BATT_TOP - dz / 2
    L = Link("battery_link", (px, py, zc))
    L.bodies.append(Body("pack", box(dx, dy, dz, (px, py, zc)), "lipo", "pack"))
    L.bodies.append(Body("pad", box(dx - 6, dy - 6, d["pad_gap"], (px, py, BATT_TOP + d["pad_gap"] / 2)), "grey", "pad"))
    L.bodies.append(Body("lead_boot", box(18, 16, 9, (px - dx / 2 - 8, py, BATT_TOP - 14)), "yellow", "pack"))
    L.group_mass = {"pack": d["mass_g"], "pad": d["pad_mass_g"]}
    L.collision = [("box", d["dims"], (px, py, zc), 0.0)]
    return L


def build_harness():
    d = P.HARNESS
    x, y, z = d["pos"]
    L = Link("harness_link", d["pos"])
    L.bodies.append(Body("power_leads", tube_between((6.0, -26.0, 12.0), (6.0, -26.0, Z_BOT0 - 0.5), 3.0), "black"))
    L.bodies.append(Body("buck", box(20, 14, 6, (-30.0, 28.0, Z_TOP1 + 3.0)), "black"))
    for i in range(4):                                     # motor lead bundles: along the arm, from the motor base towards the centre (3 wires in a sleeve)
        sx, sy = P.ROTOR_SIGN[i]
        mx, my = motor_xy(i)
        dvec = np.array([-sx * MX + sx * 24.0, -sy * MY + sy * 16.0]); dvec /= np.linalg.norm(dvec)
        p0 = np.array([mx, my, ARM_TOP + 4.4]) + np.r_[dvec * (M["base_r"] + 1.0), 0]
        p1 = np.array([sx * 24.0, sy * 16.0, 11.5])
        L.bodies.append(Body(f"motor_leads_{i}", tube_between(p0, p1, 2.2), "black"))
    L.group_mass = {"all": d["mass_g"]}
    return L


def deck_outline():
    half = D["outline_half"]
    return half + [(x, -y) for x, y in reversed(half)]


def build_deck():
    L = Link("payload_deck_link", (0, 0, DECK_BOT))
    z0, z1 = DECK_BOT, DECK_TOP
    plate = prism(deck_outline(), z0, z1)
    plate = holes(plate, [(x, y) for x, y in D["front_standoffs"] + D["rear_standoffs"]], 1.6, z0, z1)
    plate = holes(plate, l2_holes(), 1.6, z0, z1)
    jx, jy = P.JETSON["pos_xy"]
    plate = holes(plate, [(jx + sa * 29.0, jy + sb * 43.0) for sa in (-1, 1) for sb in (-1, 1)], 1.6, z0, z1)
    rx, ry = P.RADIO["pos_xy"]
    hp = P.RADIO["hole_pattern"]
    plate = holes(plate, [(rx + sa * hp[0] / 2, ry + sb * hp[1] / 2) for sa in (-1, 1) for sb in (-1, 1)], 1.3, z0, z1)
    plate = holes(plate, [tuple(a) for a in P.RADIO["antenna"]["base_xy"]], 3.2, z0, z1)
    ex, ey = P.ETH["pos_xy"]
    for sx in (-14.0, 14.0):
        for sy in (-1, 1):
            plate = slot(plate, ex + sx, ey + sy * (P.ETH["dims"][1] / 2 + 0.6), 6.5, 2.4, z0, z1, 0)
    # lightening: windows under the L2 (cable pass-through) and the Jetson module, plus pill slots along the edges, the same
    # language as the frame plates (45-degree chamfers + pill cut-outs).  Everything stays 6+ mm from every mounting hole.
    lx, ly, _ = l2_origin()
    plate = plate - cyl(13.0, D["t"] + 2, (lx, ly, (z0 + z1) / 2))
    plate = slot(plate, jx + 2.0, jy, 30.0, 52.0, z0, z1, 0) if False else plate
    for sy in (-1, 1):                                   # two windows under the module, leaving a bridge for the centre-line standoff
        plate = slot(plate, jx + 4.0, jy + sy * 24.0, 22.0, 24.0, z0, z1, 90)
    for x, y, ln, w, a in [(-6, 38, 22, 5, 0), (-6, -38, 22, 5, 0), (-100, 54, 36, 5, 0), (-100, -54, 36, 5, 0), (-136, 54, 12, 5, 0), (-136, -54, 12, 5, 0),
                           (-46, 52, 10, 5, 45), (-46, -52, 10, 5, 45), (-190, 0, 14, 7, 90), (-150, 0, 14, 7, 90), (-196, -22, 12, 6, 0), (-196, 22, 12, 6, 0),
                           (-170, -8, 22, 6, 0), (-170, 8, 10, 4, 0)]:
        try:
            plate = slot(plate, x, y, ln, w, z0, z1, a)
        except Exception:
            pass
    L.bodies.append(Body("deck_plate", plate, "carbon", "plate", 1.0))
    for k, (x, y) in enumerate(D["front_standoffs"]):
        L.bodies.append(Body(f"deck_standoff_f{k}", standoff(x, y, Z_UPPER1, z0), "gold", "standoff"))
    for k, (x, y) in enumerate(D["rear_standoffs"]):
        L.bodies.append(Body(f"deck_standoff_r{k}_lo", standoff(x, y, Z_TOP1, Z_TOP1 + 22.0), "gold", "standoff"))
        L.bodies.append(Body(f"deck_standoff_r{k}_hi", standoff(x, y, Z_TOP1 + 22.0, z0), "gold", "standoff"))
    # Jetson carrier: 4 x 10 mm nylon spacers on the 86 x 58 pattern; L2: 4 x M3 screws from below; radio: 4 x 6 mm nylon spacers
    for k, (sa, sb) in enumerate([(a, b) for a in (-1, 1) for b in (-1, 1)]):
        L.bodies.append(Body(f"jetson_spacer_{k}", standoff(jx + sa * 29.0, jy + sb * 43.0, z1, z1 + P.JETSON["standoff_h"], "carbon", od=5.0), "white", "nylon"))
        L.bodies.append(Body(f"jetson_screw_{k}", screw_head(jx + sa * 29.0, jy + sb * 43.0, z1 + P.JETSON["standoff_h"] + P.JETSON["pcb_t"], r=2.6, h=1.6), "alu", "screw"))
        L.bodies.append(Body(f"radio_spacer_{k}", standoff(rx + sa * hp[0] / 2, ry + sb * hp[1] / 2, z1, z1 + P.RADIO["standoff_h"], "carbon", od=4.0), "white", "nylon"))
    for k, (hx, hy) in enumerate(l2_holes()):
        L.bodies.append(Body(f"l2_screw_{k}", screw_head(hx, hy, z0, up=False, r=2.6, h=1.8), "alu", "screw"))
    for k, (x, y) in enumerate(D["front_standoffs"] + D["rear_standoffs"]):
        L.bodies.append(Body(f"deck_screw_{k}", screw_head(x, y, z1, r=2.6, h=1.8), "alu", "screw"))
    L.auto_density = {"plate": P.FRAME["carbon_density"], "standoff": 2.7, "nylon": 1.14, "screw": 7.85}
    xs = [p[0] for p in deck_outline()]
    ys = [p[1] for p in deck_outline()]
    L.collision = [("box", (max(xs) - min(xs), max(ys) - min(ys), D["t"]), ((max(xs) + min(xs)) / 2, (max(ys) + min(ys)) / 2, z0 + D["t"] / 2), 0.0)]
    return L


# ------------------------------------------------------------ mass properties
_MESH_CACHE = {}


def body_mesh(b, tol=0.05):
    key = (id(b), tol)
    if key not in _MESH_CACHE:
        _MESH_CACHE[key] = _body_mesh(b, tol)
    return _MESH_CACHE[key]


def _body_mesh(b, tol=0.05):
    v, t = b.shape.tessellate(tol, 0.1)
    m = trimesh.Trimesh(np.array([(p.X, p.Y, p.Z) for p in v]), np.array(t), process=True)
    m.merge_vertices()
    return m


def link_mass_props(link):
    """B-rep mass properties (OCC), SI.  Returns dict(mass, com(abs m), I_com 3x3 kg m^2)."""
    vol = {}
    for b in link.bodies:
        vol[b.name] = b.shape.volume * b.density_rel
    groups = {}
    for b in link.bodies:
        groups.setdefault(b.group, 0.0)
        groups[b.group] += vol[b.name]
    for g_, dens in link.auto_density.items():      # mass from the B-rep volume (mm3 -> cm3) x density
        link.group_mass[g_] = groups.get(g_, 0.0) / 1000.0 * dens
    items = []
    for b in link.bodies:
        gm = link.group_mass.get(b.group)
        if gm is None:
            raise KeyError(f"{link.name}: no mass for group {b.group}")
        m_kg = gm * 1e-3 * vol[b.name] / groups[b.group]
        v_mm3 = b.shape.volume
        c = np.array(list(b.shape.center(__import__('build123d').CenterOf.MASS))) * 1e-3
        Iv = np.array(b.shape.matrix_of_inertia, float)      # mm^5 for unit density
        I = Iv * (m_kg / (v_mm3 * 1e-9)) * 1e-15              # (kg/m^3) * m^5
        items.append((m_kg, c, I, b))
    mass = sum(i[0] for i in items)
    com = sum(i[0] * i[1] for i in items) / mass
    I = np.zeros((3, 3))
    for m, c, Il, _ in items:
        d = c - com
        I += Il + m * (d @ d * np.eye(3) - np.outer(d, d))
    return dict(mass=mass, com=com, I=I, body_mass={i[3].name: i[0] for i in items})


def build_all(battery_xy=(0.0, 0.0)):
    links = [build_frame(), build_deck()]
    for i in range(4):
        st, ro = build_rotor(i)
        links += [st, ro]
    links.append(build_esc())
    cube, cube_imu = build_cube()
    links += [cube, build_l2(), build_jetson(), build_eth(), build_radio(), build_rx(), build_oak(), build_oak_bracket(), build_harness(),
              build_battery(*battery_xy)]
    return links, cube_imu


def assemble():
    bx, by = 0.0, 0.0
    for _ in range(3):      # fixed-point: slide the pack until the CoM sits on the motor-rectangle centre (bounded)
        links, cube_imu = build_all((bx, by))
        props = {l.name: link_mass_props(l) for l in links}
        m, com, _ = total_props(props)
        bx = float(np.clip(bx - com[0] * 1e3 * m / props["battery_link"]["mass"], *P.BATTERY["x_bounds"]))
        by = float(np.clip(by - com[1] * 1e3 * m / props["battery_link"]["mass"], -10, 10))
    links, cube_imu = build_all((bx, by))
    props = {l.name: link_mass_props(l) for l in links}
    return links, props, cube_imu, (bx, by)


def total_props(props):
    m = sum(p["mass"] for p in props.values())
    com = sum(p["mass"] * p["com"] for p in props.values()) / m
    I = np.zeros((3, 3))
    for p in props.values():
        d = p["com"] - com
        I += p["I"] + p["mass"] * (d @ d * np.eye(3) - np.outer(d, d))
    return m, com, I


# ------------------------------------------------------------ geometry checks
def prop_clearance(links, margin=0.0):
    """Min xy distance from each motor axis to every non-motor/prop body that crosses the prop plane.  Returns list."""
    zlo, zhi = P.PROP["plane_z"] - 5.0, P.PROP["plane_z"] + 5.0
    R = P.PROP["diameter"] / 2
    from scipy.spatial import ConvexHull
    out = []
    centres = [motor_xy(i) for i in range(4)]
    for l in links:
        if l.name.startswith(("motor_", "rotor_", "base_link")):
            continue
        for b in l.bodies:
            m = body_mesh(b, 0.2)
            if m.bounds[1][2] < zlo or m.bounds[0][2] > zhi:
                continue
            xy = m.vertices[:, :2]
            try:
                h = ConvexHull(xy)
                poly = xy[h.vertices]
            except Exception:
                poly = xy
            dmin = 1e9
            for c in centres:
                d = point_polygon_dist(np.array(c), poly)
                dmin = min(dmin, d)
            out.append((l.name, b.name, dmin - R))
    return out


def point_polygon_dist(p, poly):
    n = len(poly)
    inside = True
    best = 1e9
    sign = None
    for i in range(n):
        a, b = poly[i], poly[(i + 1) % n]
        ab = b - a
        t = np.clip(((p - a) @ ab) / (ab @ ab + 1e-12), 0, 1)
        best = min(best, np.linalg.norm(p - (a + t * ab)))
        cr = ab[0] * (p - a)[1] - ab[1] * (p - a)[0]
        if sign is None:
            sign = cr >= 0
        elif (cr >= 0) != sign:
            inside = False
    return 0.0 if inside else best


ALLOWED_PAIRS = [("stack_standoff", "esc_link"), ("power_leads", "plate"), ("power_leads", "esc_link"), ("signal_bundle", "stack_standoff"),
                 ("deck_standoff", "jetson_link"), ("deck_standoff", "l2_link"), ("deck_standoff", "radio_link"),
                 ("spacer", "plate"), ("lead_boot", "plate"), ("cage_tab", "plate"), ("strap", "battery"), ("sma_", "antenna"),
                 ("harness_link", "esc_link"), ("harness_link", "stack_standoff"), ("harness_link", "plate"), ("bracket", "plate")]


def _allowed(a, b):
    ka, kb = a.lower(), b.lower()
    for p, q in ALLOWED_PAIRS:
        if (p in ka and q in kb) or (p in kb and q in ka):
            return True
    return False


def interferences(links, tol=0.6):
    """Exact-ish: AABB pre-filter, then surface samples of one body inside the other by > tol mm (trimesh signed distance)."""
    items = []
    for l in links:
        for b in l.bodies:
            items.append((l.name, b.name, body_mesh(b, 0.3)))
    hits = []
    for i in range(len(items)):
        for j in range(i + 1, len(items)):
            la, na, ma = items[i]; lb, nb, mb = items[j]
            if la == lb:
                continue
            pen = np.minimum(ma.bounds[1], mb.bounds[1]) - np.maximum(ma.bounds[0], mb.bounds[0])
            if not np.all(pen > tol):
                continue
            worst = 0.0
            for src, dst in ((ma, mb), (mb, ma)):
                pts = np.vstack([src.vertices, src.sample(300)])
                inside = (pts > dst.bounds[0] - 1e-6).all(1) & (pts < dst.bounds[1] + 1e-6).all(1)
                if not inside.any():
                    continue
                sd = trimesh.proximity.signed_distance(dst, pts[inside])
                worst = max(worst, float(sd.max()))
            if worst > tol and not _allowed(la + "/" + na, lb + "/" + nb):
                hits.append((la + "/" + na, lb + "/" + nb, round(worst, 1)))
    return hits


PLATE_NAMES = ("top_plate", "upper_plate", "bottom_plate", "deck_plate", "carrier_pcb", "carrier")


def interface_report(links):
    """Every standoff/spacer must (a) end on a plate at both ends and (b) have a hole in that plate at its axis and material around it."""
    bodies = [(l.name, b) for l in links for b in l.bodies]
    plates = [(ln, b, body_mesh(b, 0.2)) for ln, b in bodies if b.name in PLATE_NAMES]
    out = []
    for ln, b in bodies:
        if not (("standoff" in b.name or "spacer" in b.name) and "screw" not in b.name):
            continue
        m = body_mesh(b, 0.2)
        c = (m.bounds[0] + m.bounds[1]) / 2
        z0, z1 = m.bounds[0][2], m.bounds[1][2]
        ends = []
        for zend, side in ((z0, "bottom"), (z1, "top")):
            hit = None
            for pln, pb, pm in plates:
                pz0, pz1 = pm.bounds[0][2], pm.bounds[1][2]
                touching = abs(pz1 - zend) < 0.05 if side == "bottom" else abs(pz0 - zend) < 0.05
                inside_xy = (pm.bounds[0][0] <= c[0] <= pm.bounds[1][0]) and (pm.bounds[0][1] <= c[1] <= pm.bounds[1][1])
                if touching and inside_xy:
                    zm = (pz0 + pz1) / 2
                    ring = [(c[0] + 4.2 * math.cos(a), c[1] + 4.2 * math.sin(a), zm) for a in np.linspace(0, 2 * math.pi, 8, endpoint=False)]
                    try:
                        centre_in = bool(pm.contains([[c[0], c[1], zm]])[0])
                        ring_in = int(np.sum(pm.contains(ring)))
                    except Exception:
                        centre_in, ring_in = None, None
                    hit = dict(plate=pln + "/" + pb.name, centre_is_hole=(centre_in is False), ring_material=ring_in)
                    break
            if hit is None:                      # stacked standoffs: the joint is a coupler, not a plate
                for oln, ob in bodies:
                    if ob is not b and "standoff" in ob.name and "screw" not in ob.name:
                        om = body_mesh(ob, 0.2)
                        oc = (om.bounds[0] + om.bounds[1]) / 2
                        if abs(oc[0] - c[0]) < 0.05 and abs(oc[1] - c[1]) < 0.05 and (abs(om.bounds[1][2] - zend) < 0.05 if side == "bottom" else abs(om.bounds[0][2] - zend) < 0.05):
                            hit = dict(plate="coupler:" + oln + "/" + ob.name, centre_is_hole=True, ring_material=None)
            ends.append(hit)
        out.append(dict(part=ln + "/" + b.name, xy=[round(float(c[0]), 2), round(float(c[1]), 2)], length_mm=round(float(z1 - z0), 2), bottom=ends[0], top=ends[1]))
    return out


def connectivity(links, gap=0.35):
    """Coarse no-floating-parts check: AABBs (grown by gap) form a graph; everything must reach base_link."""
    nodes = []
    for l in links:
        for b in l.bodies:
            m = body_mesh(b, 0.3)
            nodes.append((l.name, b.name, m.bounds[0] - gap, m.bounds[1] + gap))
    n = len(nodes)
    adj = [[] for _ in range(n)]
    lo = np.array([x[2] for x in nodes]); hi = np.array([x[3] for x in nodes])
    for i in range(n):
        ov = np.all((lo[i] <= hi) & (hi[i] >= lo), axis=1)
        for j in np.nonzero(ov)[0]:
            if j != i:
                adj[i].append(int(j))
    seen = {i for i, x in enumerate(nodes) if x[0] == "base_link" and x[1].startswith(("bottom_plate", "top_plate"))}
    stack = list(seen)
    while stack:
        i = stack.pop()
        for j in adj[i]:
            if j not in seen:
                seen.add(j); stack.append(j)
    return [nodes[i][0] + "/" + nodes[i][1] for i in range(n) if i not in seen]


# ------------------------------------------------------------ exports
def link_local_mesh(link, mat):
    ms = []
    for b in link.bodies:
        if b.mat == mat:
            m = b.visual.copy() if b.visual is not None else body_mesh(b)
            ms.append(m)
    if not ms:
        return None
    m = trimesh.util.concatenate(ms)
    m.apply_translation(-np.array(link.anchor, float))
    m.apply_scale(1e-3)
    return m


def write_obj(path, scene_meshes, mtl_name):
    lines = [f"mtllib {mtl_name}", "o aos7v5"]
    off = 1
    for name, mat, m in scene_meshes:
        lines.append(f"g {name}\nusemtl {mat}")
        for v in m.vertices:
            lines.append("v %.6f %.6f %.6f" % tuple(v))
        for f in m.faces:
            lines.append("f %d %d %d" % (f[0] + off, f[1] + off, f[2] + off))
        off += len(m.vertices)
    Path(path).write_text("\n".join(lines) + "\n")
    mtl = []
    for k, (rgba, met, rough) in MAT.items():
        mtl.append(f"newmtl {k}\nKd {rgba[0]:.3f} {rgba[1]:.3f} {rgba[2]:.3f}\nKs 0.2 0.2 0.2\nNs {int((1-rough)*200)}\nd 1.0\n")
    (Path(path).parent / mtl_name).write_text("\n".join(mtl))


def export_visual(links, outdir, urdf_mesh_dir):
    outdir.mkdir(parents=True, exist_ok=True)
    urdf_mesh_dir.mkdir(parents=True, exist_ok=True)
    scene = trimesh.Scene()
    all_meshes = []
    files = {}
    for l in links:
        files[l.name] = []
        mats = sorted({b.mat for b in l.bodies})
        for mat in mats:
            m = link_local_mesh(l, mat)
            fname = f"{l.name}__{mat}.stl"
            m.export(urdf_mesh_dir / fname)
            files[l.name].append((mat, fname))
            wm = m.copy()
            wm.apply_translation(np.array(l.anchor, float) * 1e-3)
            rgba, met, rough = MAT[mat]
            wm.visual = trimesh.visual.TextureVisuals(material=trimesh.visual.material.PBRMaterial(
                baseColorFactor=[int(c * 255) for c in rgba], metallicFactor=min(met, 0.35), roughnessFactor=rough))
            scene.add_geometry(wm, node_name=f"{l.name}/{mat}")
            all_meshes.append((f"{l.name}_{mat}", mat, wm))
    scene.export(outdir / "aos7v5_cad.glb")
    write_obj(outdir / "aos7v5_cad.obj", all_meshes, "aos7v5_cad.mtl")
    return files


def write_step(links, path):
    comps = []
    for l in links:
        for b in l.bodies:
            s = b.shape
            s.label = f"{l.name}/{b.name}"
            comps.append(s)
    export_step(Compound(comps), str(path))


def silhouette_area(links, axis):
    """Frontal projected area (m^2) of everything except props, rasterised at 1 mm."""
    import matplotlib
    matplotlib.use("Agg")
    from matplotlib.path import Path as MPath
    res = 1.0
    meshes = [body_mesh(b, 0.3) for l in links if not l.name.startswith("rotor_") for b in l.bodies]
    allv = np.vstack([m.vertices for m in meshes])
    ia, ib = {"x": (1, 2), "y": (0, 2)}[axis]
    lo, hi = allv[:, [ia, ib]].min(0) - 2, allv[:, [ia, ib]].max(0) + 2
    nx, ny = int((hi - lo)[0] / res) + 1, int((hi - lo)[1] / res) + 1
    grid = np.zeros((ny, nx), bool)
    gx, gy = np.meshgrid(lo[0] + (np.arange(nx) + 0.5) * res, lo[1] + (np.arange(ny) + 0.5) * res)
    pts = np.c_[gx.ravel(), gy.ravel()]
    for m in meshes:
        tri = m.vertices[m.faces][:, :, [ia, ib]]
        for t in tri:
            a = t.min(0); b = t.max(0)
            ix0 = max(int((a[0] - lo[0]) / res), 0); ix1 = min(int((b[0] - lo[0]) / res) + 1, nx)
            iy0 = max(int((a[1] - lo[1]) / res), 0); iy1 = min(int((b[1] - lo[1]) / res) + 1, ny)
            if ix1 <= ix0 or iy1 <= iy0:
                continue
            sub = np.c_[gx[iy0:iy1, ix0:ix1].ravel(), gy[iy0:iy1, ix0:ix1].ravel()]
            inside = MPath(t).contains_points(sub, radius=1e-9)
            grid[iy0:iy1, ix0:ix1] |= inside.reshape(iy1 - iy0, ix1 - ix0)
    return grid.sum() * res * res * 1e-6


# ------------------------------------------------------------ URDF
def fmt(v):
    return " ".join("%.6g" % x for x in v)


def inertia_xml(I):
    return (f'<inertia ixx="{I[0,0]:.6e}" ixy="{I[0,1]:.6e}" ixz="{I[0,2]:.6e}" '
            f'iyy="{I[1,1]:.6e}" iyz="{I[1,2]:.6e}" izz="{I[2,2]:.6e}"/>')


def write_urdf(path, links, props, files, imu_abs, bat_xy):
    anchors = {l.name: np.array(l.anchor, float) * 1e-3 for l in links}
    out = ['<?xml version="1.0"?>',
           f'<!-- AOS 7 V5 search-and-rescue quad, generated by Tools/DroneCAD/aos7v5/build.py (iteration {P.ITER}). Do not edit by hand:',
           '     change Tools/DroneCAD/aos7v5/params.py and rebuild.  Geometry is original (modelled from published dimensions); GPL-3.0.',
           '     Frames: REP-103 FLU, base_link origin = frame centre (motor-rectangle centre, arm mid-plane).  Masses in kg, inertias about each link CoM.',
           '     Rotors follow PX4 quad-X numbering: rotor_0 = motor 1 front-right CCW, rotor_1 = motor 2 rear-left CCW, rotor_2 = motor 3 front-left CW,',
           '     rotor_3 = motor 4 rear-right CW  (CRANE M1..M4 = FR, RL, FL, RR, the same order).  Physics coefficients: aos7v5_multirotor.yaml -->',
           '<robot name="aos7v5">']
    for k, (rgba, *_ ) in MAT.items():
        out.append(f'  <material name="{k}"><color rgba="{fmt(rgba)}"/></material>')
    def mesh_visuals(l):
        for mat, fn in files[l.name]:
            out.append(f'    <visual><geometry><mesh filename="package://aos7v5/meshes/visual/{fn}"/></geometry><material name="{mat}"/></visual>')
    def collision(l):
        for kind, size, c, yaw in l.collision:
            loc = (np.array(c) * 1e-3 - anchors[l.name])
            org = f'<origin xyz="{fmt(loc)}" rpy="0 0 {yaw:.6g}"/>'
            if kind == "box":
                out.append(f'    <collision>{org}<geometry><box size="{fmt(np.array(size)*1e-3)}"/></geometry></collision>')
            else:
                out.append(f'    <collision>{org}<geometry><cylinder radius="{size[0]*1e-3:.6g}" length="{size[1]*1e-3:.6g}"/></geometry></collision>')
    def inertial(l):
        p = props[l.name]
        loc = p["com"] - anchors[l.name]
        out.append(f'    <inertial><origin xyz="{fmt(loc)}" rpy="0 0 0"/><mass value="{p["mass"]:.6f}"/>{inertia_xml(p["I"])}</inertial>')
    for l in links:
        out.append(f'  <link name="{l.name}">')
        inertial(l)
        mesh_visuals(l)
        collision(l)
        out.append('  </link>')
        if l.name == "base_link":
            continue
        parent_anchor = anchors[l.parent]
        org = anchors[l.name] - parent_anchor
        if l.joint == "continuous":
            out.append(f'  <joint name="{l.name}_joint" type="continuous"><parent link="{l.parent}"/><child link="{l.name}"/>'
                       f'<origin xyz="{fmt(org)}" rpy="0 0 0"/><axis xyz="0 0 1"/><dynamics damping="0" friction="0"/></joint>')
        else:
            out.append(f'  <joint name="{l.name}_joint" type="fixed"><parent link="{l.parent}"/><child link="{l.name}"/><origin xyz="{fmt(org)}" rpy="0 0 0"/></joint>')
    # sensor / reference frames (massless).  name, parent, abs xyz (mm), rpy
    oak = P.OAK
    ox, oy, _ = oak["pos"]
    oz = OAK_Z
    face = ox + oak["dims"][0] / 2
    half_b = oak["baseline_mm"] / 2
    optical = "-1.5707963 0 -1.5707963"
    l2 = P.L2
    frames = [
        ("oak_d_left_camera_frame", "oak_d_link", (face, oy + half_b, oz), "0 0 0"),
        ("oak_d_right_camera_frame", "oak_d_link", (face, oy - half_b, oz), "0 0 0"),
        ("oak_d_rgb_camera_frame", "oak_d_link", (face, oy, oz), "0 0 0"),
        ("oak_d_left_camera_optical_frame", "oak_d_left_camera_frame", (face, oy + half_b, oz), optical),
        ("oak_d_right_camera_optical_frame", "oak_d_right_camera_frame", (face, oy - half_b, oz), optical),
        ("oak_d_rgb_camera_optical_frame", "oak_d_rgb_camera_frame", (face, oy, oz), optical),
        ("l2_lidar_frame", "l2_link", (l2["pos_xy"][0], l2["pos_xy"][1], DECK_TOP + l2["scan_origin_z"]), "0 0 0"),
        ("l2_imu_frame", "l2_link", (l2["pos_xy"][0] + l2["imu_offset"][0] * 1e3, l2["pos_xy"][1] + l2["imu_offset"][1] * 1e3, DECK_TOP + l2["scan_origin_z"] + l2["imu_offset"][2] * 1e3), "0 0 0"),
        ("cube_imu_link", "cube_carrier_link", imu_abs, "0 0 0"),
    ]
    abs_pos = {l.name: anchors[l.name] for l in links}
    for name, parent, pos, rpy in frames:
        p = np.array(pos, float) * 1e-3
        abs_pos[name] = p
        out.append(f'  <link name="{name}"/>')
        out.append(f'  <joint name="{name}_joint" type="fixed"><parent link="{parent}"/><child link="{name}"/><origin xyz="{fmt(p - abs_pos[parent])}" rpy="{rpy}"/></joint>')
    # tags the CRANE URDF sync reads: sensors named by the Surface profile rules (lidar|laser, camera, imu); propellers by name
    out.append('  <gazebo reference="l2_lidar_frame"><sensor name="unitree_l2" type="gpu_lidar"><update_rate>10</update_rate></sensor></gazebo>')
    out.append('  <gazebo reference="oak_d_rgb_camera_optical_frame"><sensor name="oak_d_rgb" type="camera"><update_rate>30</update_rate></sensor></gazebo>')
    out.append('  <gazebo reference="oak_d_left_camera_optical_frame"><sensor name="oak_d_left" type="camera"><update_rate>30</update_rate></sensor></gazebo>')
    out.append('  <gazebo reference="oak_d_right_camera_optical_frame"><sensor name="oak_d_right" type="camera"><update_rate>30</update_rate></sensor></gazebo>')
    out.append('  <gazebo reference="cube_imu_link"><sensor name="cube_imu" type="imu"><update_rate>250</update_rate></sensor></gazebo>')
    out.append('</robot>')
    Path(path).write_text("\n".join(out) + "\n")


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--out-dir", default=str(HERE / "out"))
    ap.add_argument("--urdf-dir", default=None, help="default: <repo>/Assets/Robot Descriptions/aos7v5 (only for the final build)")
    ap.add_argument("--model-dir", default=None, help="GLB/OBJ/STEP/report dir")
    ap.add_argument("--no-urdf", action="store_true")
    a = ap.parse_args()
    out = Path(a.out_dir)
    out.mkdir(parents=True, exist_ok=True)
    links, props, cube_imu, bat_xy = assemble()
    m, com, I = total_props(props)
    model_dir = Path(a.model_dir) if a.model_dir else out
    model_dir.mkdir(parents=True, exist_ok=True)
    urdf_dir = Path(a.urdf_dir) if a.urdf_dir else out / "urdf"
    mesh_dir = urdf_dir / "meshes" / "visual"
    files = export_visual(links, model_dir, mesh_dir)
    write_step(links, model_dir / "aos7v5.step")
    # convex-hull collision proxy of the whole airframe without props (for engines that want one mesh)
    hull = trimesh.util.concatenate([body_mesh(bd, 0.2) for l in links if not l.name.startswith("rotor_") for bd in l.bodies]).convex_hull
    hull.apply_scale(1e-3)
    (urdf_dir / "meshes" / "collision").mkdir(parents=True, exist_ok=True)
    hull.export(urdf_dir / "meshes" / "collision" / "aos7v5_hull.stl")
    clear = prop_clearance(links)
    hits = interferences(links)
    iface = interface_report(links)
    floating = connectivity(links)
    ax, ay = silhouette_area(links, "x"), silhouette_area(links, "y")
    report = dict(iter=P.ITER, layout=P.LAYOUT, mass_kg=m, com_m=com.tolist(), inertia_com=I.tolist(), battery_xy_mm=bat_xy,
                  links={n: dict(mass_kg=p["mass"], com_m=p["com"].tolist()) for n, p in props.items()},
                  prop_clearance_mm=[(a_, b_, round(float(c_), 2)) for a_, b_, c_ in clear],
                  prop_clearance_min_mm=min([c_ for *_, c_ in clear], default=None),
                  interferences=hits, interfaces=iface, floating_bodies=floating, frontal_area_x_m2=ax, frontal_area_y_m2=ay,
                  bbox_mm=[(np.min([body_mesh(b, 0.3).bounds[0] for l in links for b in l.bodies], axis=0)).tolist(),
                           (np.max([body_mesh(b, 0.3).bounds[1] for l in links for b in l.bodies], axis=0)).tolist()])
    Path(model_dir / "cad_report.json").write_text(json.dumps(report, indent=1))
    if not a.no_urdf:
        (urdf_dir / "NOTICE.md").write_text("# aos7v5 assets\n\nAll geometry is generated by Tools/DroneCAD/aos7v5/build.py from published dimensions (GPL-3.0), except:\n\n"
            "* `meshes/visual/oak_d_link__black.stl` contains a decimated copy of the Luxonis OAK-D S2 enclosure (DM9098S2_enclosure.stp,\n"
            "  https://github.com/luxonis/depthai-hardware, MIT License, Copyright (c) 2020 luxonis). License text: Tools/DroneCAD/aos7v5/vendor_derived/LICENSE.luxonis-depthai-hardware.MIT.\n\n"
            "No other vendor CAD is redistributed; see Tools/DroneCAD/aos7v5/VENDOR_CAD.md.\n")
        write_urdf(urdf_dir / "aos7v5.urdf", links, props, files, cube_imu, bat_xy)
        import physics_sidecar
        physics_sidecar.write(urdf_dir / "aos7v5_multirotor.yaml", links, props, report)
    print(f"iter {P.ITER} [{P.LAYOUT}]: mass {m:.4f} kg, CoM(mm) {np.round(com*1e3,2)}, diag I {np.round(np.diag(I),6)}")
    print("battery xy mm", np.round(bat_xy, 1), "frontal areas m2", round(ax, 4), round(ay, 4))
    bad = [c for c in clear if c[2] < 1.0]
    print("prop-disc clearance min mm:", report["prop_clearance_min_mm"], "violations(<1 mm):", [(c[0], c[1], round(c[2], 1)) for c in bad])
    bad_if = [r["part"] for r in iface if any(e is None or not e["centre_is_hole"] or (e["ring_material"] is not None and e["ring_material"] < 5) for e in (r["bottom"], r["top"]))]
    print(f"standoff/spacer interfaces: {len(iface)} checked, {len(bad_if)} bad {bad_if[:6]}; floating bodies: {len(floating)} {floating[:6]}")
    print("interferences (AABB, >0.6 mm):", len(hits))
    for h in hits[:20]:
        print("  ", h)


if __name__ == "__main__":
    main()
