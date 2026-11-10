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


@dataclass
class Link:
    name: str
    anchor: tuple                       # mm, in base frame
    bodies: list = field(default_factory=list)
    group_mass: dict = field(default_factory=dict)     # group -> g (distributed by volume*density_rel)
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
        s = Rot(*(0, 0, 0)) * s
        from build123d import Axis
        s = s.rotate(Axis((0, 0, 0), tuple(axis)), ang)
    return Pos(*((p0 + p1) / 2)) * s


def prism(pts, z0, z1):
    f = Face(Wire.make_polygon([Vector(x, y, z0) for x, y in pts], close=True))
    return Solid.extrude(f, Vector(0, 0, z1 - z0))


def rect_union_prism(rects, z0, z1):
    solids = [prism([(a, c), (b, c), (b, d), (a, d)], z0, z1) for a, b, c, d in rects]
    return solids


# ------------------------------------------------------------ assembly
F, M, PR = P.FRAME, P.MOTOR, P.PROP
MX, MY = F["motor_dx"], F["motor_dy"]
ARM_TOP = F["arm_t"] / 2
Z_BOT0 = -F["arm_t"] / 2 - F["bot_t"]
Z_UPPER1 = Z_BOT0 + F["height"]
Z_UPPER0 = Z_UPPER1 - F["upper_t"]
PAYLOAD = P.LAYOUT == "payload_deck"
DECK_TOP = (P.DECK["z0"] + P.DECK["t"]) if PAYLOAD else Z_UPPER1


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


def build_frame():
    L = Link("base_link", (0, 0, 0))
    z4 = ARM_TOP
    for i in range(4):
        sx, sy = P.ROTOR_SIGN[i]
        L.bodies.append(Body(f"arm_{i}", prism(arm_polygon(sx, sy), -z4, z4), "carbon", "kit", F["carbon_density"]))
        L.bodies.append(Body(f"motor_pad_{i}", cyl(F["arm_pad_r"], F["arm_t"], (sx * MX, sy * MY, 0)), "carbon", "kit", F["carbon_density"]))
    L.bodies.append(Body("bottom_plate", prism(F["bottom_plate"], Z_BOT0, -z4), "carbon", "kit", F["carbon_density"] * F["lightening"]))
    L.bodies.append(Body("top_plate", prism(F["top_plate"], z4, z4 + F["top_t"]), "carbon", "kit", F["carbon_density"] * F["lightening"]))
    L.bodies.append(Body("upper_plate", prism(F["upper_plate"], Z_UPPER0, Z_UPPER1), "carbon", "kit", F["carbon_density"] * F["lightening"]))
    so = F["stack_standoff"]
    for k, (x, y) in enumerate(so["positions"]):
        z0 = z4 + F["top_t"]
        L.bodies.append(Body(f"stack_standoff_{k}", cyl(so["r"], Z_UPPER0 - z0, (x, y, (z0 + Z_UPPER0) / 2)), "gold", "standoff", 1.0))
    L.group_mass = {"kit": F["mass_g"], "standoff": so["mass_g_each"] * len(so["positions"])}
    if P.DETAIL >= 3:
        # 7075 camera cage tabs (inside the 325 g kit): small alu blocks on the nose, screws at plate corners
        L.bodies.append(Body("cage_tab_l", box(14, 3, 14, (136, 24, -1)), "alu", "kit", F["alu_density"]))
        L.bodies.append(Body("cage_tab_r", box(14, 3, 14, (136, -24, -1)), "alu", "kit", F["alu_density"]))
        for x in (-35.0, 35.0):   # kit battery straps seen around the pack
            L.bodies.append(Body(f"strap_{int(x)}", box(14, 49, 1.6, (x, 0, Z_BOT0 - 0.8)), "black", "kit", 0.5))
    L.collision = [("box", (200, 80, 12.5), (-10, 0, -0.25), 0.0)]
    for i in range(4):
        sx, sy = P.ROTOR_SIGN[i]
        L.collision.append(("box", (math.hypot(MX - 34, MY - 14), 22, 8), (sx * (MX + 34) / 2, sy * (MY + 14) / 2, 0),
                            math.atan2(sy * (MY - 14), sx * (MX - 34))))
    return L


def blade_solid(spin, k):
    R = PR["diameter"] / 2
    rs = np.linspace(PR["hub_r"], R, 9)
    def chord(r):
        t = (r - PR["hub_r"]) / (R - PR["hub_r"])
        if t < 0.6:
            return PR["chord_root"] + (PR["chord_peak"] - PR["chord_root"]) * (t / 0.6)
        return PR["chord_peak"] + (PR["chord_tip"] - PR["chord_peak"]) * ((t - 0.6) / 0.4) ** 1.3
    le = [(r, -0.35 * chord(r)) for r in rs]
    te = [(r, 0.65 * chord(r)) for r in rs[::-1]]
    pts = le + te
    if spin < 0:
        pts = [(x, -y) for x, y in pts]
    th = PR["thickness"]
    s = prism(pts, -th / 2, th / 2)
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
    z_top = ARM_TOP + M["height"]
    bell = cyl(M["r_bell"], M["bell_h"], (x, y, z_bell0 + M["bell_h"] / 2))
    inner = cyl(M["r_bell"] - M["bell_wall"], M["bell_h"] - 2.0, (x, y, z_bell0 + (M["bell_h"] - 2.0) / 2 - 0.01))
    ro.bodies.append(Body("bell", bell - inner, "alu" if P.DETAIL < 3 else "black", "bell"))
    ro.bodies.append(Body("shaft_nut", cyl(PR["nut_r"], PR["nut_h"], (x, y, z_prop + PR["hub_h"] / 2 + PR["nut_h"] / 2 - 1.0)), "alu", "bell"))
    ro.bodies.append(Body("hub", cyl(PR["hub_r"], PR["hub_h"], (x, y, z_prop)), "prop", "prop"))
    nb = PR["blades"]
    for k in range(nb):
        if P.DETAIL == 1:   # iteration 1: flat rectangular rods
            ang = 2 * math.pi * k / nb
            c = np.array([math.cos(ang), math.sin(ang)])
            R = PR["diameter"] / 2
            mid = (R + PR["hub_r"]) / 2
            sol = Rot(0, 0, math.degrees(ang)) * box(R - PR["hub_r"], 12, 1.8, (mid, 0, 0))
            sol = Pos(x, y, z_prop) * sol
        else:
            sol = Pos(x, y, z_prop) * blade_solid(spin, k)
        ro.bodies.append(Body(f"blade_{k}", sol, "prop", "prop"))
    ro.group_mass = {"bell": M["bell_mass_g"], "prop": PR["mass_g"]}
    return st, ro


def build_esc():
    d = P.ESC
    L = Link("esc_link", d["pos"])
    x, y, z = d["pos"]
    L.bodies.append(Body("pcb", box(d["dims"][0], d["dims"][1], 1.6, (x, y, z - d["dims"][2] / 2 + 0.8)), "pcb"))
    L.bodies.append(Body("fets", box(d["dims"][0] - 8, d["dims"][1] - 8, d["dims"][2] - 1.6, (x, y, z + 0.8)), "black"))
    if P.DETAIL >= 2:
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
    zb = Z_UPPER1 + c["standoff_h"]
    L = Link("cube_carrier_link", (x, y, zb))
    cx, cy, cz = c["carrier_dims"]
    L.bodies.append(Body("carrier_pcb", box(cx, cy, cz, (x, y, zb + cz / 2)), "pcb", "carrier"))
    zcube0 = zb + cz + c["header_h"]
    dx, dy, dz = c["dims"]
    L.bodies.append(Body("header", box(dx - 8, dy - 8, c["header_h"], (x, y, zb + cz + c["header_h"] / 2)), "black", "carrier"))
    L.bodies.append(Body("cube_case", box(dx, dy, dz, (x, y, zcube0 + dz / 2)), "orange", "cube"))
    if P.DETAIL >= 2:
        for k in (-1, 0, 1):
            L.bodies.append(Body(f"jst_{k}", box(6, 8, 6, (x + cx / 2 - 5, y + k * 11, zb + cz + 3)), "beige", "carrier"))
        L.bodies.append(Body("cube_label", box(22, 14, 0.3, (x, y, zcube0 + dz + 0.15)), "white", "cube"))
    if P.DETAIL >= 3:
        for sx in (-1, 1):
            for sy in (-1, 1):
                L.bodies.append(Body(f"spacer_{sx}{sy}", cyl(2.2, c["standoff_h"], (x + sx * (cx / 2 - 4), y + sy * (cy / 2 - 4), zb - c["standoff_h"] / 2)), "gold", "carrier"))
    L.group_mass = {"carrier": c["carrier_mass_g"], "cube": c["mass_g"]}
    L.collision = [("box", (cx, cy, zcube0 + dz - zb), (x, y, (zb + zcube0 + dz) / 2), 0.0)]
    imu = (x, y, zcube0 + dz / 2)
    return L, imu


def build_l2():
    d = P.L2
    x, y = d["pos_xy"]
    zb = DECK_TOP
    L = Link("l2_link", (x, y, zb))
    dx, dy, dz = d["dims"]
    if P.DETAIL == 1:
        L.bodies.append(Body("l2_block", box(dx, dy, dz, (x, y, zb + dz / 2)), "black"))
    else:
        L.bodies.append(Body("base", box(dx, dy, 22, (x, y, zb + 11)), "black"))
        L.bodies.append(Body("head", cyl(dx / 2 - 2, dz - 22, (x, y, zb + 22 + (dz - 22) / 2)), "black"))
        L.bodies.append(Body("window", cyl(dx / 2 - 0.5, 28, (x, y, zb + 22 + 24)), "glass"))
        L.bodies.append(Body("led", cyl(3, 1, (x + dx / 2 - 9, y + 18, zb + 11 + 12.5)), "green"))
    L.group_mass = {"all": d["mass_g"]}
    L.collision = [("box", d["dims"], (x, y, zb + dz / 2), 0.0)]
    return L


def build_jetson():
    d = P.JETSON
    x, y = d["pos_xy"]
    zb = DECK_TOP + d["standoff_h"]
    L = Link("jetson_link", (x, y, zb))
    dx, dy, dz = d["dims"]
    L.bodies.append(Body("carrier_pcb", box(dx, dy, 1.6, (x, y, zb + 0.8)), "pcb"))
    if P.DETAIL == 1:
        L.bodies.append(Body("body", box(dx, dy, dz - 1.6, (x, y, zb + 1.6 + (dz - 1.6) / 2)), "black"))
    else:
        hs = (70.0, 56.0, dz - 1.6 - 9.0)
        L.bodies.append(Body("heatsink", box(*hs, (x + 4, y - 4, zb + 1.6 + 9.0 + hs[2] / 2)), "black"))
        L.bodies.append(Body("module_riser", box(60, 46, 9.0, (x + 4, y - 4, zb + 1.6 + 4.5)), "pcb"))
        L.bodies.append(Body("fan", cyl(22, 3.0, (x + 4, y - 4, zb + dz + 1.5)), "black"))
        L.bodies.append(Body("usb_stack", box(18, 28, 15.5, (x - dx / 2 + 12, y + dy / 2 - 16, zb + 1.6 + 7.75)), "alu"))
        L.bodies.append(Body("rj45", box(16, 15, 13.5, (x - dx / 2 + 12, y - dy / 2 + 12, zb + 1.6 + 6.75)), "alu"))
        L.bodies.append(Body("header40", box(51, 5, 8.5, (x + 8, y - dy / 2 + 3, zb + 1.6 + 4.25)), "black"))
    if P.DETAIL >= 3:
        L.bodies.append(Body("header_pins", box(50, 2.5, 6, (x + 8, y - dy / 2 + 3, zb + 1.6 + 8.5 + 3)), "gold"))
        for k in range(4):
            sxm = -1 if k < 2 else 1
            L.bodies.append(Body(f"jet_spacer_{k}", cyl(2.5, d["standoff_h"], (x + (1 if k % 2 else -1) * (dx / 2 - 4), y + sxm * (dy / 2 - 4), zb - d["standoff_h"] / 2)), "gold"))
    L.group_mass = {"all": d["mass_g"]}
    L.collision = [("box", (dx, dy, dz + 3), (x, y, zb + dz / 2 + 1.5), 0.0)]
    return L


def build_eth():
    d = P.ETH
    x, y = d["pos_xy"]
    zb = DECK_TOP
    L = Link("usb_eth_link", (x, y, zb))
    L.bodies.append(Body("adapter", box(*d["dims"], (x, y, zb + d["dims"][2] / 2)), "black"))
    if P.DETAIL >= 2:
        L.bodies.append(Body("label", box(34, 14, 0.3, (x, y, zb + d["dims"][2] + 0.15)), "grey"))
        L.bodies.append(Body("cable", tube_between((x + 30, y, zb + 6), (x + 55, y - 20, zb + 14), 2.6), "grey"))
        L.bodies.append(Body("rj_end", box(14, 12, 10, (x - 36, y, zb + 6)), "grey"))
    L.group_mass = {"all": d["mass_g"]}
    L.collision = [("box", d["dims"], (x, y, zb + d["dims"][2] / 2), 0.0)]
    return L


def build_radio():
    d = P.RADIO
    x, y = d["pos_xy"]
    zb = DECK_TOP
    L = Link("radio_link", (x, y, zb))
    cx, cy, cz = d["carrier_dims"]
    mx, my, mz = d["module_dims"]
    L.bodies.append(Body("carrier", box(cx, cy, cz, (x, y, zb + cz / 2)), "black", "carrier"))
    L.bodies.append(Body("module", box(mx, my, mz, (x - 4, y, zb + cz + mz / 2)), "alu", "module"))
    a = d["antenna"]
    gm = {"carrier": d["carrier_mass_g"], "module": d["module_mass_g"]}
    for k, sy in enumerate((-1, 1)):
        base = np.array([x - cx / 2 - 3, y + sy * 12, zb + cz - 1])
        t_out, t_back = math.radians(a["tilt_out_deg"]), math.radians(a["tilt_back_deg"])
        u = np.array([-math.sin(t_back), sy * math.sin(t_out), math.cos(t_out) * math.cos(t_back)])
        u /= np.linalg.norm(u)
        L.bodies.append(Body(f"antenna_{k}", tube_between(base + u * 8, base + u * (8 + a["length"]), a["r"]), "black", f"ant{k}"))
        if P.DETAIL >= 2:
            L.bodies.append(Body(f"sma_{k}", cyl(3.6, 9, tuple(base + np.array([0, 0, 3])), "z"), "gold", f"ant{k}"))
        gm[f"ant{k}"] = a["mass_g"]
    L.group_mass = gm
    L.collision = [("box", (cx + 6, cy, cz + mz), (x - 3, y, zb + (cz + mz) / 2), 0.0)]
    return L


def build_rx():
    d = P.RX
    L = Link("rx_link", d["pos"])
    x, y, z = d["pos"]
    L.bodies.append(Body("rx", box(*d["dims"], (x, y, z)), "black"))
    if P.DETAIL >= 2:
        L.bodies.append(Body("rx_ant", tube_between((x - 8, y, z), (x - 8 - 30, y + 20, z + 6), 0.7), "black"))
    L.group_mass = {"all": d["mass_g"]}
    return L


def build_oak():
    d = P.OAK
    x, y, z = d["pos"]
    L = Link("oak_d_link", (x, y, z))
    dx, dy, dz = d["dims"]
    L.bodies.append(Body("housing", box(dx, dy, dz, (x, y, z)), "black", "cam"))
    t = d["bracket_t"]
    L.bodies.append(Body("bracket_l", box(dx + 6, t, dz + 8, (x + 1, y + dy / 2 + t / 2, z)), "gold", "brk"))
    L.bodies.append(Body("bracket_r", box(dx + 6, t, dz + 8, (x + 1, y - dy / 2 - t / 2, z)), "gold", "brk"))
    if P.DETAIL >= 2:
        L.bodies.append(Body("bracket_top", box(dx + 6, dy + 1.0, t, (x + 1, y, z + dz / 2 + 4 - t / 2)), "gold", "brk"))
        for sy, nm in ((1, "left"), (-1, "right"), (0, "rgb")):
            L.bodies.append(Body(f"lens_{nm}", cyl(5.5 if sy else 6.5, 1.2, (x + dx / 2 + 0.6, y + sy * d["baseline_mm"] / 2, z), "x"), "glass", "cam"))
    L.group_mass = {"cam": d["mass_g"], "brk": d["bracket_mass_g"]}
    L.collision = [("box", (dx + 6, dy + 2 * t, dz + 8), (x + 1, y, z), 0.0)]
    return L


def build_battery(px, py):
    d = P.BATTERY
    dx, dy, dz = d["dims"]
    ztop = Z_BOT0 - d["pad_gap"]
    zc = ztop - dz / 2
    L = Link("battery_link", (px, py, zc))
    L.bodies.append(Body("pack", box(dx, dy, dz, (px, py, zc)), "lipo"))
    if P.DETAIL >= 2:
        L.bodies.append(Body("lead_boot", box(18, 16, 9, (px - dx / 2 - 8, py, ztop - 14)), "yellow"))
    L.group_mass = {"all": d["mass_g"]}
    L.collision = [("box", d["dims"], (px, py, zc), 0.0)]
    return L


def build_harness():
    d = P.HARNESS
    x, y, z = d["pos"]
    L = Link("harness_link", d["pos"])
    L.bodies.append(Body("power_leads", tube_between((x, y, z - 4), (x + 25, y, Z_BOT0 - P.BATTERY["pad_gap"] + 4.0), 3.5), "black"))
    L.bodies.append(Body("buck", box(20, 14, 6, (x - 20, y + 22, z - 4)), "black"))
    L.bodies.append(Body("signal_bundle", tube_between((x - 20, y - 18, z + 2), (x + 20, y + 18, z + 6), 3.0), "grey"))
    L.group_mass = {"all": d["mass_g"]}
    return L


def build_deck():
    d = P.DECK
    L = Link("payload_deck_link", (0, 0, d["z0"]))
    z0, z1 = d["z0"], d["z0"] + d["t"]
    for k, r in enumerate(rect_union_prism(d["outline_rects"], z0, z1)):
        L.bodies.append(Body(f"deck_plate_{k}", r, "carbon", "plate", P.FRAME["carbon_density"] * d["lightening"]))
    vol_cc = sum(b.shape.volume for b in L.bodies) / 1000.0
    plate_g = vol_cc * P.FRAME["carbon_density"] * d["lightening"]
    for k, (x, y) in enumerate(d["standoff_xy"]):
        L.bodies.append(Body(f"deck_standoff_{k}", cyl(d["standoff_r"], z0 - Z_UPPER1, (x, y, (z0 + Z_UPPER1) / 2)), "gold", "standoff"))
    L.group_mass = {"plate": plate_g, "standoff": d["standoff_mass_g"] * len(d["standoff_xy"])}
    xs = [r[0] for r in d["outline_rects"]] + [r[1] for r in d["outline_rects"]]
    ys = [r[2] for r in d["outline_rects"]] + [r[3] for r in d["outline_rects"]]
    L.collision = [("box", (max(xs) - min(xs), max(ys) - min(ys), d["t"]), ((max(xs) + min(xs)) / 2, (max(ys) + min(ys)) / 2, z0 + d["t"] / 2), 0.0)]
    return L


# ------------------------------------------------------------ mass properties
def body_mesh(b, tol=0.05):
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
    links = [build_frame()]
    if PAYLOAD:
        links.append(build_deck())
    for i in range(4):
        st, ro = build_rotor(i)
        links += [st, ro]
    links.append(build_esc())
    cube, cube_imu = build_cube()
    links += [cube, build_l2(), build_jetson(), build_eth(), build_radio(), build_rx(), build_oak(), build_harness(),
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


# ------------------------------------------------------------ exports
def link_local_mesh(link, mat):
    ms = []
    for b in link.bodies:
        if b.mat == mat:
            m = body_mesh(b)
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
    ox, oy, oz = oak["pos"]
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
    ax, ay = silhouette_area(links, "x"), silhouette_area(links, "y")
    report = dict(iter=P.ITER, layout=P.LAYOUT, mass_kg=m, com_m=com.tolist(), inertia_com=I.tolist(), battery_xy_mm=bat_xy,
                  links={n: dict(mass_kg=p["mass"], com_m=p["com"].tolist()) for n, p in props.items()},
                  prop_clearance_mm=[(a_, b_, round(float(c_), 2)) for a_, b_, c_ in clear],
                  prop_clearance_min_mm=min([c_ for *_, c_ in clear], default=None),
                  interferences=hits, frontal_area_x_m2=ax, frontal_area_y_m2=ay,
                  bbox_mm=[(np.min([body_mesh(b, 0.3).bounds[0] for l in links for b in l.bodies], axis=0)).tolist(),
                           (np.max([body_mesh(b, 0.3).bounds[1] for l in links for b in l.bodies], axis=0)).tolist()])
    Path(model_dir / "cad_report.json").write_text(json.dumps(report, indent=1))
    if not a.no_urdf:
        write_urdf(urdf_dir / "aos7v5.urdf", links, props, files, cube_imu, bat_xy)
        import physics_sidecar
        physics_sidecar.write(urdf_dir / "aos7v5_multirotor.yaml", links, props, report)
    print(f"iter {P.ITER} [{P.LAYOUT}]: mass {m:.4f} kg, CoM(mm) {np.round(com*1e3,2)}, diag I {np.round(np.diag(I),6)}")
    print("battery xy mm", np.round(bat_xy, 1), "frontal areas m2", round(ax, 4), round(ay, 4))
    bad = [c for c in clear if c[2] < 1.0]
    print("prop-disc clearance min mm:", report["prop_clearance_min_mm"], "violations(<1 mm):", [(c[0], c[1], round(c[2], 1)) for c in bad])
    print("interferences (AABB, >0.6 mm):", len(hits))
    for h in hits[:20]:
        print("  ", h)


if __name__ == "__main__":
    main()
