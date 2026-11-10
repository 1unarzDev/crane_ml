"""Parametric v1 parts and role kits (build123d solids) for the concept-derived mecanum fleet.

One shared chassis module (octagonal body, tucked mecanum wheels, reciprocal docks) plus role kits:
  scout  : RPLIDAR C1 on a rear-left riser, RealSense D435, Orange Pi 5 Pro bay, 5-DOF arm + gripper
  lifter : low-lift front gripper (rails, carriage, two L-jaws), Arducam mast, top AprilTag, Orange Pi 4 Pro
  cargo  : raised deck on 4 standoffs, vented electronics box, corner tag flag, front Arducam
All numbers come from Tools/Fleet/spec/fleet_profile_v1.json and cad_v1/cad_params_v1.json.
"""
import json
import math
import os

import numpy as np

import cadlib as C
from cadlib import Item, Link, T, box, cyl, cyl_between, place, pv
import geom as g

HERE = os.path.dirname(os.path.abspath(__file__))
REPO = os.path.abspath(os.path.join(HERE, "..", "..", "..", ".."))
PROFILE = os.path.join(REPO, "Tools", "Fleet", "spec", "fleet_profile_v1.json")
PARAMS = os.path.join(HERE, "cad_params_v1.json")

FACE_ORDER = ["front", "rear", "left", "right"]


class Cfg:
    def __init__(self, profile=PROFILE, params=PARAMS):
        self.profile_path, self.params_path = profile, params
        self.prof = json.load(open(profile))
        self.par = json.load(open(params))["groups"]
        self.used = set()

    def p(self, dotted):
        node = self.prof
        for k in dotted.split("."):
            node = node[k]
        return pv(node)

    def d(self, dotted):
        grp, key = dotted.split(".")
        self.used.add(dotted)
        return pv(self.par[grp][key])


def face_T(c, name):
    x, y, z, yaw = c.p(f"docking.face_frames.{name}")
    return T(x, y, z, R=g.rotz(yaw))


# ------------------------------------------------------------------ chassis
def chassis_dims(c):
    L, W, H = c.p("chassis.body_size_lwh")
    bp = c.d("chassis.bracket_proud")
    return dict(L=L, W=W, H=H, hx=L / 2 - bp, hy=W / 2 - bp, bp=bp, hw=c.d("chassis.wall_height"),
                bev=c.d("chassis.top_bevel_inset"), tp=c.d("chassis.top_plate_thickness"),
                cc=c.d("chassis.corner_chamfer"), tc=c.d("chassis.top_corner_chamfer"),
                r=c.p("chassis.wheel.radius"), w=c.p("chassis.wheel.width"),
                lx=c.p("chassis.wheel.half_wheelbase_lx"), ly=c.p("chassis.wheel.half_track_ly"),
                clr=c.p("chassis.ground_clearance"))


def dock_cutters(c, face, kind):
    """Pocket cutters (face frame -> base) for one face. kind: 'dock' | 'none'."""
    if kind != "dock":
        return []
    bp = c.d("chassis.bracket_proud")
    yc, wy, hh = c.d("dock.pocket_y_center"), c.d("dock.pocket_width"), c.d("dock.pocket_half_height")
    dep, rr = c.d("dock.pocket_depth"), c.d("dock.pocket_corner_radius")
    Tf = face_T(c, face)
    out = []
    for s in (-1, 1):
        cut = C.rounded_rect_prism_x(s * yc, 0.0, wy, 2 * hh, rr, -bp - dep, 0.02)
        out.append(place(cut, Tf))
    return out


def bracket_solid(c, s):
    yi, yo = c.d("dock.bracket_y_inner"), c.d("dock.bracket_y_outer")
    hz, bar = c.d("dock.bracket_half_height"), c.d("dock.bracket_bar")
    ch = min(c.d("dock.bracket_chamfer"), bar - 0.002)
    yb = yo - bar
    pts = [(yi, hz), (yo - ch, hz), (yo, hz - ch), (yo, -hz + ch), (yo - ch, -hz), (yi, -hz), (yi, -hz + bar), (yb, -hz + bar), (yb, hz - bar), (yi, hz - bar)]
    pts = [(s * a, b) for a, b in pts]
    if s < 0:
        pts = pts[::-1]
    return C.polygon_prism_x(pts, -c.d("chassis.bracket_proud"), 0.0)


def dock_items(c, face, kind, shell):
    """Brackets, magnets, pads and the catch pin of one face (base-frame items for chassis_body)."""
    Tf = face_T(c, face)
    bp = c.d("chassis.bracket_proud")
    items = []
    for s in (-1, 1):
        items.append(Item(place(bracket_solid(c, s), Tf), "guide", f"guide_{face}", density="petg_wall", label=f"bracket_{face}_{s:+d}"))
    mr, my, mz = c.d("dock.magnet_radius"), c.d("dock.magnet_y"), c.d("dock.magnet_z")
    rec = c.d("dock.magnet_recess")
    mags = (-1, 1) if kind == "dock" else (1,)
    for s in mags:
        m = cyl(mr, bp - rec, "x", (-(bp + rec) / 2, s * my, mz))
        items.append(Item(place(m, Tf), "magnet", f"magnet_{face}", density="ndfeb", label=f"magnet_{face}_{s:+d}"))
    if kind == "dock":
        py, pz = c.d("dock.pad_size_yz")
        for s in (-1, 1):
            zc = s * c.d("dock.pad_z")
            items.append(Item(place(box([-bp, -py / 2, zc - pz / 2], [0, py / 2, zc + pz / 2]), Tf), "pin", f"pads_{face}", density="steel", label=f"pad_{face}_{s:+d}"))
        # catch pin across the +y pocket, supported at both pocket walls, + collar at the outer wall
        yc, wy = c.d("dock.pocket_y_center"), c.d("dock.pocket_width")
        pr, px = c.d("dock.pin_radius"), c.d("dock.pin_x")
        y0, y1 = yc - wy / 2 - 0.001, yc + wy / 2 + 0.001
        items.append(Item(place(cyl_between((px, y0, 0), (px, y1, 0), pr), Tf), "pin", f"pin_{face}", density="steel", label=f"pin_{face}"))
        cr = c.d("dock.pin_collar_radius")
        items.append(Item(place(cyl_between((px, y1 - 0.004, 0), (px, y1, 0), cr), Tf), "pin", f"pin_{face}", density="steel", label=f"pin_collar_{face}"))
    return items


def tag_wedge_geom(c, role, face):
    """45 deg tag facet above a dock face, in the face frame. Returns (solid, T_tag in base frame)."""
    H = c.p("chassis.body_size_lwh")[2]
    zc = c.p(f"docking.face_frames.{face}")[2]
    bp = c.d("chassis.bracket_proud")
    z0 = c.d("dock_tag.facet_lower_z") - zc
    s = c.d("dock_tag.facet_slant") / math.sqrt(2)
    dep, wid = c.d("dock_tag.wedge_depth"), c.d("dock_tag.wedge_width")
    yo = c.d("dock_tag.offsets_face_y")[role][face]
    zb = H - zc - 0.015
    pts = [(-bp, zb), (-bp, z0), (-bp - s, z0 + s), (-dep, z0 + s), (-dep, zb)]
    from build123d import Polyline, make_face, extrude
    w = Polyline(*[(x, yo - wid / 2, z) for x, z in pts], close=True)
    wedge = extrude(make_face(w), wid, dir=(0, 1, 0))
    r2 = 1 / math.sqrt(2)
    R = np.array([[0, r2, -r2], [1, 0, 0], [0, -r2, -r2]], float)      # columns: x_t=(0,1,0), y_t=(r2,0,-r2), z_t=(-r2,0,-r2)
    ctr = np.array([-bp - s / 2, yo, z0 + s / 2]) + 0.0004 * np.array([r2, 0, r2])
    Tt = face_T(c, face) @ g.make_T(R, ctr)
    return place(wedge, face_T(c, face)), Tt


def dock_tag_link(c, role, faces):
    """Printed 45 deg tag wedges above each dock face (separate static part so chassis_body keeps the profile height)."""
    L_ = Link("dock_tag_mounts", role="body")
    for f in faces:
        L_.add(Item(tag_wedge_geom(c, role, f)[0], f"shell_{role}", f"wedge_{f}", density="petg_wall", label=f"tag_wedge_{f}"))
    return L_


def pin_position_face(c):
    return [c.d("dock.pin_x"), c.d("dock.pocket_y_center"), 0.0]


def chassis_body(c, role, face_kinds, bays=()):
    """face_kinds: {face: 'dock' | 'brackets' | 'none'}; bays: [(lo, hi)] boxes cut from the top (open bays)."""
    D = chassis_dims(c)
    shell = f"shell_{role}"
    hx, hy, H = D["hx"], D["hy"], D["H"]
    body = C.prism_xy(C.octagon(hx, hy, D["cc"]), 0.0, D["hw"])
    top_o = C.octagon(hx - D["bev"], hy - D["bev"], D["tc"])
    body = body + C.loft_octagons(C.octagon(hx, hy, D["cc"]), D["hw"], top_o, H - D["tp"])
    # wheel wells (open to the side and the floor)
    gap = c.d("chassis.wheel_gap")
    x0, x1 = D["lx"] - D["r"] - gap, D["lx"] + D["r"] + gap
    yin, ztop = c.d("chassis.well_inner_y"), c.d("chassis.well_top_z")
    for sx in (1, -1):
        for sy in (1, -1):
            lo = [min(sx * x0, sx * x1), min(sy * yin, sy * 0.3), -0.1]
            hi = [max(sx * x0, sx * x1), max(sy * yin, sy * 0.3), ztop]
            body = body - box(lo, hi)
    # interior cavity (printed shell, 3 mm walls)
    wt = c.d("chassis.wall_thickness")
    xi = hx - c.d("dock.pocket_depth") - 0.007
    body = body - box([-xi, -(yin - 0.0035), wt], [xi, yin - 0.0035, 0.074])
    ys = hy - c.d("dock.pocket_depth") - 0.007
    body = body - box([-(x0 - wt), -ys, wt], [x0 - wt, ys, 0.074])
    for f, k in face_kinds.items():
        for cut in dock_cutters(c, f, k):
            body = body - cut
    plate = C.prism_xy(top_o, H - D["tp"], H)
    for lo, hi in bays:
        body = body - box(lo, hi)
        plate = plate - box(lo, hi)
    items = [Item(body, shell, "shell", density="petg_wall", label="printed_shell"),
             Item(plate, "top_plate", "top_plate", density="aluminium", label="top_plate")]
    # perimeter screws on the top plate
    sr = c.d("chassis.screw_head_radius")
    n = int(c.d("chassis.screw_count_per_long_side"))
    ix, iy = hx - D["bev"] - 0.007, hy - D["bev"] - 0.006
    pts = [(x, s * iy) for x in np.linspace(-ix + 0.02, ix - 0.02, n) for s in (1, -1)] + [(s * ix, y) for y in (-0.035, 0.035) for s in (1, -1)]
    for k, (x, y) in enumerate(pts):
        if any(lo[0] - 0.004 <= x <= hi[0] + 0.004 and lo[1] - 0.004 <= y <= hi[1] + 0.004 for lo, hi in bays):
            continue
        items.append(Item(cyl(sr, 0.0012, "z", (x, y, H + 0.0006)), "screw", "screws", density="steel", label=f"screw_{k}"))
    for f, k in face_kinds.items():
        if k in ("dock", "brackets"):
            items += dock_items(c, f, k, shell)
    L_ = Link("chassis_body", role="body")
    L_.add(*items)
    return L_


# ------------------------------------------------------------------ wheels
def wheel_link(c, name, sx, sy):
    D = chassis_dims(c)
    r, w = D["r"], D["w"]
    n = int(c.p("chassis.wheel.roller_count"))
    alpha = c.d("wheel.roller_angle")
    fill = c.d("wheel.roller_fill")
    s_ = math.sin(math.pi / n) * math.sin(alpha)
    rr = fill * r * s_ / (1 + fill * s_)
    Rc = r - rr
    Lr = (w - 2 * rr * math.sin(alpha)) / math.cos(alpha)
    sign = -sx * sy
    taper = c.d("wheel.roller_taper")
    prof = [(t, rr * math.sqrt(max(1 - taper * (2 * t / Lr) ** 2, 0.0))) for t in np.linspace(-Lr / 2, Lr / 2, 7)]
    base_roller = C.revolve_profile_z(prof)
    items = []
    hub = cyl(c.d("wheel.hub_radius_frac") * r, 0.9 * w, "y")
    items.append(Item(hub, "wheel_core", "hub", label="hub_core"))
    # silver star hub (one spoke per roller), recessed so the dark rollers bulge past it (concept)
    rec = c.d("wheel.star_recess")
    for s in (-1, 1):
        yc_ = s * (w / 2 - rec)
        star = cyl(0.0125, 0.004, "y", (0, yc_, 0))
        for i in range(n):
            th = -math.pi / 2 + 2 * math.pi * i / n
            r0, r1 = 0.008, Rc - 0.004
            spoke = place(box([r0, -0.0015, -0.003], [r1, 0.0015, 0.003]), T(0, yc_, 0, R=g.roty(-th)))
            star = star + spoke + cyl(0.0042, 0.004, "y", ((Rc - 0.004) * math.cos(th), yc_, (Rc - 0.004) * math.sin(th)))
        star = star - cyl(0.0045, 0.01, "y", (0, yc_, 0))
        items.append(Item(star, "wheel_hub", "hub", label=f"star_hub_{s:+d}"))
        cap = cyl(0.0085, 0.003, "y", (0, s * (w / 2 - rec + 0.0035), 0))
        items.append(Item(cap, "axle_cap", "hub", label=f"axle_cap_{s:+d}"))
    for i in range(n):
        th = -math.pi / 2 + 2 * math.pi * i / n
        tang = np.array([-math.sin(th), 0.0, math.cos(th)])
        u = math.sin(alpha) * tang + math.cos(alpha) * sign * np.array([0, 1.0, 0])
        ctr = Rc * np.array([math.cos(th), 0.0, math.sin(th)])
        items.append(Item(place(base_roller, T(*ctr, R=g.R_align_z(u))), "roller", "roller_%02d" % i, label="roller_%02d" % i))
        for e in (-1, 1):
            items.append(Item(place(cyl(0.0026, 0.003, "z", (0, 0, e * (Lr / 2 + 0.001))), T(*ctr, R=g.R_align_z(u))), "axle_cap", "roller_caps", label="roller_cap_%02d" % i))
    # mass: datasheet wheel mass distributed by volume
    vols = [it.props()[3] for it in [Item(x.solid, "", mass=1.0) for x in items]]
    tot = sum(vols)
    for it, v in zip(items, vols):
        it.mass = c.d("masses.wheel") * v / tot
    L_ = Link(f"wheel_{name}", joint={"type": "continuous", "axis": [0, 1, 0], "origin": T(sx * D["lx"], sy * D["ly"], r - D["clr"]), "authored": 0.0},
              role="wheel", contacts_ground=True)
    L_.add(*items)
    L_.info = dict(sign=sign, roller_radius=rr, roller_circle=Rc, roller_length=Lr)
    return L_


# ------------------------------------------------------------------ hooks
def hook_link(c, face):
    bp = c.d("chassis.bracket_proud")
    yc = c.d("dock.pocket_y_center")
    px = c.d("dock.pin_x")
    hr, clr = c.d("dock.hook_rod_radius"), c.d("dock.hook_clearance")
    pr = c.d("dock.pin_radius")
    rc = pr + clr + hr                         # crook centreline radius about the pin
    xp = c.d("dock.hook_pivot_x")
    reach = -px - xp                           # partner pin sits at face x = -pin_x when mated
    pivot = T(xp, -yc, rc)                     # hook at face -y, arm centreline rc above the pin
    Tp = face_T(c, face) @ pivot
    items = [Item(cyl(c.d("dock.hook_boss_radius"), c.d("dock.hook_boss_width"), "y"), "hook", "boss", density="steel", label="boss"),
             Item(cyl_between((0, 0, 0), (reach, 0, 0), hr), "hook", "arm", density="steel", label="arm")]
    a1 = c.d("dock.hook_crook_end_deg")
    items.append(Item(C.torus_arc((reach, 0, -rc), rc, 90, a1, hr), "hook", "crook", density="steel", label="crook"))
    tip = (reach + rc * math.cos(math.radians(a1)), 0, -rc + rc * math.sin(math.radians(a1)))
    items.append(Item(C.Pos(*tip) * C.Sphere(hr), "hook", "crook", density="steel", label="tip"))
    op = c.p("docking.hook_open_angle")
    L_ = Link(f"hook_{face}_a", joint={"type": "revolute", "axis": [0, 1, 0], "origin": Tp, "limits": [-op, 0.0], "authored": 0.0},
              role="hook")
    L_.add(*items)
    L_.frames["engage"] = T(reach, 0, -rc)
    L_.info = dict(face=face, open_angle=-op, engage=[reach, 0.0, -rc], tip=tip)
    return L_


# ------------------------------------------------------------------ COTS proxies
def sbc_items(c, x0, y0, z0, mass_key, label):
    """Orange Pi 89 x 56 board (long side along x) with ports, heatsink and fan; (x0,y0) = min corner."""
    t = 0.0016
    it = [Item(box([x0, y0, z0], [x0 + 0.089, y0 + 0.056, z0 + t]), "pcb_blue", "pcb", mass=c.d(f"masses.{mass_key}") * 0.75, label=label + "_pcb")]
    xe = x0 + 0.089
    for k, (yy, wy, hz, mat) in enumerate([(0.004, 0.016, 0.0135, "port_metal"), (0.022, 0.014, 0.015, "port_metal"), (0.038, 0.014, 0.015, "port_metal")]):
        it.append(Item(box([xe - 0.018, y0 + yy, z0 + t], [xe + 0.002, y0 + yy + wy, z0 + t + hz]), mat, "ports", mass=c.d(f"masses.{mass_key}") * 0.08, label=f"{label}_port{k}"))
    it.append(Item(box([x0 + 0.003, y0 + 0.050, z0 + t], [x0 + 0.054, y0 + 0.055, z0 + t + 0.0085]), "header", "header", mass=0.001, label=label + "_gpio"))
    hx, hy = x0 + 0.030, y0 + 0.024
    hs = box([hx - 0.020, hy - 0.020, z0 + t + 0.002], [hx + 0.020, hy + 0.020, z0 + t + 0.004])
    for k in range(9):
        xf = hx - 0.018 + k * 0.0045
        hs = hs + box([xf, hy - 0.020, z0 + t + 0.004], [xf + 0.0012, hy + 0.020, z0 + t + 0.012])
    it.append(Item(hs, "heatsink", "heatsink", mass=c.d("masses.heatsink_fan") * 0.7, label=label + "_heatsink"))
    fan = box([hx - 0.015, hy - 0.015, z0 + t + 0.012], [hx + 0.015, hy + 0.015, z0 + t + 0.019]) - cyl(0.0125, 0.01, "z", (hx, hy, z0 + t + 0.0175))
    fan = fan + cyl(0.006, 0.006, "z", (hx, hy, z0 + t + 0.0155))
    it.append(Item(fan, "fan", "fan", mass=c.d("masses.heatsink_fan") * 0.3, label=label + "_fan"))
    return it


def arducam_items(c, Tcam, label="arducam"):
    """OV9281 module (38 x 38 board, M12 lens) in a camera frame Tcam (x = boresight). Returns items + lens-front offset."""
    m = c.d("masses.arducam")
    bs = pv(c.prof["sensors"]["mono_camera_ov9281"]["board_size"])[0]
    it = [Item(place(box([-0.0016, -bs / 2, -bs / 2], [0, bs / 2, bs / 2]), Tcam), "pcb_black", "board", mass=m * 0.3, label=label + "_board"),
          Item(place(box([0, -0.0085, -0.0085], [0.007, 0.0085, 0.0085]), Tcam), "sensor_body", "holder", mass=m * 0.2, label=label + "_holder"),
          Item(place(cyl(0.0075, 0.012, "x", (0.013, 0, 0)), Tcam), "lens_barrel", "lens", mass=m * 0.45, label=label + "_lens"),
          Item(place(cyl(0.0052, 0.0008, "x", (0.0194, 0, 0)), Tcam), "lens", "lens", mass=m * 0.05, label=label + "_glass")]
    for sy in (-1, 1):
        for sz in (-1, 1):
            it.append(Item(place(cyl(0.0021, 0.0015, "x", (0.0008, sy * (bs / 2 - 0.0035), sz * (bs / 2 - 0.0035))), Tcam), "screw", "board", density="steel", label=label + "_screw"))
    return it, 0.0198


def tag_quad(size, tag_id, Tt):
    plate = size * 10.0 / 8.0
    p = plate / 2
    corners = [(-p, -p, 0), (-p, p, 0), (p, p, 0), (p, -p, 0)]
    uvs = [(0, 1), (0, 0), (1, 0), (1, 1)]
    mat = "tag36h11_%05d" % tag_id
    q = g.quad_mesh(corners, uvs, mat, "plate").transformed(Tt)
    return q, plate, mat


R_TAG_TOP = np.array([[0, -1, 0], [-1, 0, 0], [0, 0, -1]], float)


def internals_link(c, role, faces_with_hooks, extra=()):
    """Hidden internal components (mass only + designer visibility): battery, motors, MCU, IMU, drivers, hook servos."""
    D = chassis_dims(c)
    it = []
    if role == "lifter":
        lo, hi = c.d("lifter.battery_box")
        it.append(Item(box(lo, hi), "internal", "battery", mass=c.d("masses.battery"), label="battery_3s2p_18650_rear"))
        cwm = c.d("lifter.counterweight_mass")
        for k, (lo, hi) in enumerate(c.d("lifter.counterweight_boxes")):
            it.append(Item(box(lo, hi), "internal", "counterweight", mass=cwm / 2, label=f"counterweight_steel_{k}"))
    else:
        it.append(Item(box([-0.050, -0.032, 0.004], [0.015, 0.024, 0.023]), "internal", "battery", mass=c.d("masses.battery"), label="battery_3s2p_18650"))
    for sx in (1, -1):
        for sy in (1, -1):
            x = sx * D["lx"]
            yin_w = D["ly"] - D["w"] / 2                    # wheel inner face
            y0, y1 = sy * (yin_w - 0.004), sy * (yin_w - 0.004 - 0.067)   # 25D x 67L gearmotor, coupling 4 mm
            it.append(Item(cyl_between((x, y0, D["r"] - D["clr"]), (x, y1, D["r"] - D["clr"]), 0.0122), "internal", "motors",
                           mass=c.d("masses.gearmotor"), label=f"gearmotor_{'F' if sx > 0 else 'R'}{'L' if sy > 0 else 'R'}"))
    px, py, pz = c.d("scout.pico_xyz")
    it.append(Item(box([px - 0.0255, py - 0.0105, pz], [px + 0.0255, py + 0.0105, pz + 0.001]), "internal", "mcu", mass=c.d("masses.pico"), label="raspberry_pi_pico"))
    ix, iy, iz = c.d("scout.imu_xyz")
    it.append(Item(box([ix - 0.0127, iy - 0.0089, iz], [ix + 0.0127, iy + 0.0089, iz + 0.0016]), "internal", "imu", mass=c.d("masses.imu_board"), label="ism330dhcx_4502"))
    it.append(Item(box([0.030, -0.030, 0.004], [0.090, 0.000, 0.020]), "internal", "electronics", mass=c.d("masses.electronics_misc"), label="drivers_power_wiring"))
    for f in faces_with_hooks:
        Tf = face_T(c, f)
        it.append(Item(place(box([-0.050, -0.036, -0.011], [-0.028, -0.012, 0.011]), Tf), "internal", "hook_servos", mass=c.d("masses.hook_servo"), label=f"hook_servo_{f}"))
    for e in extra:
        it.append(e)
    L_ = Link("internals", role="envelope", visible=False, hide_group="internals")
    L_.add(*it)
    return L_


# ------------------------------------------------------------------ role kits
def scout_links(c):
    D = chassis_dims(c)
    H = D["H"]
    links = []
    ox, oy = c.d("scout.opi_xy_min")
    bd = c.d("scout.bay_depth")
    bay = ([ox - 0.003, oy - 0.003, H - bd], [ox + 0.092, oy + 0.059, H + 0.01])
    body = chassis_body(c, "scout", {f: "dock" for f in FACE_ORDER}, bays=[bay])
    links.append(body)
    comp = Link("compute", role="sensor")
    comp.add(*sbc_items(c, ox, oy, H - bd + 0.001, "opi5pro", "orange_pi_5_pro"))
    links.append(comp)
    # service ethernet port
    ex, ey = c.d("scout.ethernet_xy")
    sp = Link("service_port", role="sensor")
    sp.add(Item(box([ex - 0.012, ey - 0.010, H], [ex + 0.012, ey + 0.010, H + 0.003]), "sensor_body", "panel", mass=0.006, label="rj45_panel"),
           Item(box([ex - 0.007, ey - 0.006, H + 0.001], [ex + 0.007, ey + 0.006, H + 0.0035]), "port_metal", "jack", mass=0.004, label="rj45_jack"))
    links.append(sp)
    # D435
    dx, dy, dz = c.d("scout.d435_center")
    sz = c.p("sensors.realsense_d435_body.size_lwh")
    md = c.d("masses.d435")
    hous = box([dx - sz[0] / 2, dy - (sz[1] - sz[2]) / 2, dz - sz[2] / 2], [dx + sz[0] / 2, dy + (sz[1] - sz[2]) / 2, dz + sz[2] / 2])
    for s in (-1, 1):
        hous = hous + cyl(sz[2] / 2, sz[0], "x", (dx, dy + s * (sz[1] - sz[2]) / 2, dz))
    d435 = Link("d435", role="sensor")
    d435.add(Item(hous, "d435_shell", "housing", mass=md * 0.93, label="d435_housing"))
    front = dx + sz[0] / 2
    d435.add(Item(box([front - 0.0005, dy - 0.040, dz - 0.0085], [front + 0.0008, dy + 0.040, dz + 0.0085]), "sensor_body", "glass", mass=md * 0.03, label="d435_glass"))
    lens_y = {"infra1": 0.0175, "infra2": -0.0325, "color": 0.0325, "projector": -0.0075}
    for k, yy in lens_y.items():
        d435.add(Item(cyl(0.0042 if k != "projector" else 0.0035, 0.0008, "x", (front + 0.0012, dy + yy, dz)), "lens", "lenses", mass=md * 0.01, label=f"d435_{k}"))
    d435.add(Item(box([dx - 0.008, dy - 0.015, H], [dx + 0.008, dy + 0.015, dz - sz[2] / 2]), "metal", "bracket", density="aluminium", label="d435_bracket"))
    for s in (-1, 1):
        d435.add(Item(box([dx - 0.006, dy + s * 0.030 - 0.003, dz + sz[2] / 2 - 0.0002], [dx + 0.006, dy + s * 0.030 + 0.003, dz + sz[2] / 2 + 0.0004]), "sensor_body", "vents", mass=0.0002, label="d435_vent"))
    d435.frames["camera_link"] = T(front - 0.0042, dy + lens_y["infra1"], dz)
    d435.frames["infra1"] = T(front - 0.0042, dy + lens_y["infra1"], dz)
    d435.frames["infra2"] = T(front - 0.0042, dy + lens_y["infra2"], dz)
    d435.frames["color"] = T(front - 0.0042, dy + lens_y["color"], dz)
    links.append(d435)
    # LiDAR riser ("mast") + C1
    lxp, lyp = c.d("scout.lidar_xy")
    mw = c.d("scout.lidar_mount")
    mh = c.p("roles.scout.mast_height")
    riser = C.chamfer_box_yz([lxp - mw[0] / 2, lyp - mw[1] / 2, H], [lxp + mw[0] / 2, lyp + mw[1] / 2, H + mh], 0.004)
    riser = riser - box([lxp - 0.012, lyp + mw[1] / 2 - 0.006, H + 0.008], [lxp + 0.012, lyp + mw[1] / 2 + 0.01, H + mh - 0.008])
    mast = Link("mast", role="body")
    mast.add(Item(riser, "shell_scout", "riser", density="petg_infill", label="lidar_riser"),
             Item(box([lxp + mw[0] / 2 - 0.0005, lyp - 0.010, H + 0.010], [lxp + mw[0] / 2 + 0.001, lyp + 0.010, H + mh - 0.010]), "accent", "accent", density="petg_wall", label="riser_accent"))
    links.append(mast)
    ld, lh = c.p("sensors.lidar_2d.size")[0], c.p("sensors.lidar_2d.size")[2]
    z0 = H + mh
    base_h = 0.016
    lb = Link("lidar_base", role="sensor")
    lb.add(Item(cyl(ld / 2, base_h, "z", (lxp, lyp, z0 + base_h / 2)), "lidar", "base", mass=c.d("masses.rplidar_c1") * 0.6, label="c1_base"))
    links.append(lb)
    hh = lh - base_h
    head = Link("lidar_head", role="sensor", joint={"type": "continuous", "axis": [0, 0, 1], "origin": T(lxp, lyp, z0 + base_h + hh / 2), "authored": 0.0})
    head.add(Item(cyl(ld / 2 - 0.002, hh * 0.45, "z", (0, 0, -hh * 0.275)), "lidar_window", "window", mass=c.d("masses.rplidar_c1") * 0.15, label="c1_window"),
             Item(cyl(ld / 2 - 0.0015, hh * 0.55, "z", (0, 0, hh * 0.225)), "lidar", "head", mass=c.d("masses.rplidar_c1") * 0.25, label="c1_head"))
    links.append(head)
    scan_z = z0 + base_h + hh * 0.45 * 0.5 - 0.0 + 0.0       # middle of the window band
    lidar_frame = T(lxp, lyp, z0 + base_h + (hh * 0.45) / 2)
    # top tag (added for overhead localisation)
    tx, ty = c.d("scout.tag_xy")
    links += arm_links(c, H)
    return links, dict(lidar_frame=lidar_frame, tag_xy=(tx, ty), faces={f: "dock" for f in FACE_ORDER})


def arm_links(c, H):
    ax, ay = c.d("scout.arm_base_xy")
    bp = c.d("scout.arm_base_plate")
    tr_, th = c.d("scout.turntable")
    sh = c.d("scout.shoulder_height")
    Lu, Lf, Lw, Lg = c.d("scout.upper_arm"), c.d("scout.forearm"), c.d("scout.wrist"), c.d("scout.gripper")
    sec_y, sec_x = c.d("scout.arm_link_section")
    oy = c.d("scout.elbow_offset_y") / 2
    ms = c.d("masses.sts3215")
    fold = c.d("scout.folded_pose")
    out = []
    base = Link("arm_base", role="body")
    base.add(Item(box([ax - bp[0] / 2, ay - bp[1] / 2, H], [ax + bp[0] / 2, ay + bp[1] / 2, H + bp[2]]), "sensor_body", "plate", density="aluminium", label="arm_base_plate"),
             Item(cyl(tr_, th / 2, "z", (ax, ay, H + bp[2] + th / 4)), "arm_shell", "turntable", density="petg_infill", label="turntable_lower"),
             Item(cyl(tr_ + 0.0005, th / 4, "z", (ax, ay, H + bp[2] + th / 2 + th / 8)), "accent", "turntable", density="petg_wall", label="turntable_ring"))
    for sx in (-1, 1):
        for sy in (-1, 1):
            base.add(Item(cyl(0.0025, 0.0012, "z", (ax + sx * (bp[0] / 2 - 0.006), ay + sy * (bp[1] / 2 - 0.006), H + bp[2] + 0.0006)), "screw", "screws", density="steel", label="plate_screw"))
    out.append(base)
    j1z = H + bp[2] + th
    servo = lambda: box([-sec_x / 2 + 0.0026, -0.01235, -0.0226], [sec_x / 2 - 0.0026, 0.01235, 0.0226])   # STS3215 45.2 x 24.7 x 35 (35 along x)
    # link 1: turntable top + shoulder servo (stator) on +y
    l1 = Link("arm_link_1", parent="arm_base", role="arm",
              joint={"type": "revolute", "axis": [0, 0, 1], "origin": T(ax, ay, j1z), "limits": [-math.pi, math.pi], "authored": fold["arm_joint_1"]})
    l1.add(Item(cyl(tr_ - 0.002, th / 2, "z", (0, 0, -th / 4)), "arm_shell", "turntable", density="petg_infill", label="turntable_upper"),
           Item(place(box([-0.016, -0.0124, -0.0226], [0.016, 0.0124, 0.0226]), T(0, oy, sh)), "servo", "servo", mass=ms, label="shoulder_servo"),
           Item(box([-0.018, oy + 0.0128, 0.0], [0.018, oy + 0.0168, sh + 0.008]), "arm_shell", "bracket", density="petg_infill", label="shoulder_bracket"))
    out.append(l1)
    # link 2: upper arm on +y
    l2 = Link("arm_link_2", parent="arm_link_1", role="arm",
              joint={"type": "revolute", "axis": [0, 1, 0], "origin": T(0, 0, sh), "limits": [-1.75, 1.75], "authored": fold["arm_joint_2"]})
    l2.add(Item(box([-sec_x / 2 + 0.003, oy - sec_y / 2, 0.012], [sec_x / 2 - 0.003, oy + sec_y / 2, Lu - 0.012]), "arm_shell", "link", density="petg_infill", label="upper_arm"),
           Item(box([sec_x / 2 - 0.0035, oy - 0.004, 0.025], [sec_x / 2 - 0.0025, oy + 0.004, Lu - 0.025]), "accent", "accent", density="petg_wall", label="upper_arm_accent"),
           Item(place(box([-0.0226, -0.0124, -0.016], [0.0226, 0.0124, 0.016]), T(0, -oy, Lu)), "servo", "servo", mass=ms, label="elbow_servo"))
    out.append(l2)
    l3 = Link("arm_link_3", parent="arm_link_2", role="arm",
              joint={"type": "revolute", "axis": [0, 1, 0], "origin": T(0, 0, Lu), "limits": [-0.3, 3.2], "authored": fold["arm_joint_3"]})
    l3.add(Item(box([-sec_x / 2 + 0.004, -oy - sec_y / 2, 0.020], [sec_x / 2 - 0.004, -oy + sec_y / 2, Lf - 0.014]), "arm_shell", "link", density="petg_infill", label="forearm"),
           Item(box([-sec_x / 2 + 0.0025, -oy - 0.004, 0.030], [-sec_x / 2 + 0.0035, -oy + 0.004, Lf - 0.025]), "accent", "accent", density="petg_wall", label="forearm_accent"),
           Item(place(box([-0.0124, -0.0124, -0.016], [0.0124, 0.0124, 0.016]), T(0, -oy, Lf)), "servo", "servo", mass=ms, label="wrist_pitch_servo"))
    out.append(l3)
    l4 = Link("arm_link_4", parent="arm_link_3", role="arm",
              joint={"type": "revolute", "axis": [0, 1, 0], "origin": T(0, 0, Lf), "limits": [-1.8, 1.8], "authored": fold["arm_joint_4"]})
    l4.add(Item(box([-0.011, -oy - 0.011, 0.012], [0.011, -oy + 0.011, Lw]), "servo", "servo", mass=ms, label="wrist_roll_servo"))
    out.append(l4)
    l5 = Link("arm_link_5", parent="arm_link_4", role="arm",
              joint={"type": "revolute", "axis": [0, 0, 1], "origin": T(0, -oy, Lw), "limits": [-math.pi, math.pi], "authored": fold["arm_joint_5"]})
    l5.add(Item(box([-0.010, -0.012, 0.0], [0.010, 0.012, 0.018]), "servo", "servo", mass=ms, label="gripper_servo"),
           Item(box([-0.009, 0.004, 0.018], [0.009, 0.010, Lg]), "arm_shell", "finger", density="petg_infill", label="fixed_finger"),
           Item(box([-0.008, 0.0035, Lg - 0.012], [0.008, 0.0045, Lg]), "sensor_body", "finger", density="rubber", label="fixed_pad"))
    out.append(l5)
    jaw = Link("arm_gripper_jaw", parent="arm_link_5", role="gripper",
               joint={"type": "revolute", "axis": [1, 0, 0], "origin": T(0, -0.004, 0.018), "limits": [0.0, 0.9], "authored": fold["arm_gripper_joint"]})
    jaw.add(Item(box([-0.008, -0.006, 0.0], [0.008, 0.0, Lg - 0.018]), "arm_shell", "finger", density="petg_infill", label="moving_jaw"),
            Item(box([-0.007, 0.0, Lg - 0.030], [0.007, 0.001, Lg - 0.018]), "sensor_body", "finger", density="rubber", label="moving_pad"))
    jaw.frames["tcp"] = T(0, 0.004, Lg - 0.018 - 0.004)
    out.append(jaw)
    return out


def lifter_links(c):
    D = chassis_dims(c)
    H, hx = D["H"], D["hx"]
    links = []
    ox, oy = c.d("lifter.opi_xy_min")
    bd = c.d("scout.bay_depth")
    bay = ([ox - 0.003, oy - 0.003, H - bd], [ox + 0.092, oy + 0.059, H + 0.01])
    faces = {"front": "brackets", "rear": "dock", "left": "dock", "right": "dock"}
    links.append(chassis_body(c, "lifter", faces, bays=[bay]))
    comp = Link("compute", role="sensor")
    comp.add(*sbc_items(c, ox, oy, H - bd + 0.001, "opi4pro", "orange_pi_4_pro"))
    links.append(comp)
    # camera mast + Arducam
    mx, my = c.d("lifter.mast_xy")
    mh = c.d("lifter.mast_height")
    mast = Link("camera_mast", role="body")
    mast.add(Item(box([mx - 0.012, my - 0.010, H], [mx + 0.012, my + 0.010, H + 0.003]), "metal", "foot", density="aluminium", label="mast_foot"),
             Item(box([mx - 0.008, my - 0.007, H + 0.003], [mx - 0.002, my + 0.007, H + mh]), "metal", "column", density="aluminium", label="mast_column"),
             Item(box([mx - 0.0075, my - 0.003, H + 0.010], [mx - 0.0015, my + 0.003, H + mh - 0.008]), "sensor_body", "slot", density="aluminium", label="mast_slot"))
    links.append(mast)
    pitch = c.p("roles.lifter.mast_camera_pitch")
    Tcam = T(mx, my, H + mh + 0.008) @ g.make_T(g.roty(pitch))
    cam = Link("front_camera", role="sensor")
    its, lens_front = arducam_items(c, Tcam)
    cam.add(*its)
    cam.frames["cam_front"] = Tcam @ T(lens_front, 0, 0)
    links.append(cam)
    # lift column: top block, rails, lead screw, upper cross bar, bottom bracket
    rx, ry = c.d("lifter.rail_x"), c.d("lifter.rail_y")
    rz0, rz1 = c.d("lifter.rail_z")
    rr = c.d("lifter.rail_radius")
    tb = c.d("lifter.top_block")
    col = Link("lift_column", role="body")
    col.add(Item(box([rx - tb[0] / 2 - 0.006, -tb[1] / 2, rz1 - 0.004], [rx + tb[0] / 2 - 0.006, tb[1] / 2, rz1 - 0.004 + tb[2]]), "sensor_body", "top_block", density="petg_infill", label="lift_top_block"))
    col.add(Item(box([rx - 0.006, -0.012, rz1 - 0.003], [rx + 0.010, 0.012, rz1 + 0.012]), "internal", "motor", mass=c.d("masses.lift_motor"), label="lift_motor", render=False))
    for s in (-1, 1):
        col.add(Item(cyl_between((rx, s * ry, rz0), (rx, s * ry, rz1), rr), "pin", "rails", density="steel", label=f"rail_{s:+d}"))
        col.add(Item(box([rx - 0.007, s * ry - 0.008, rz0 - 0.006], [rx + 0.007, s * ry + 0.008, rz0]), "metal", "rail_foot", density="aluminium", label=f"rail_foot_{s:+d}"))
        col.add(Item(box([hx - 0.001, s * ry - 0.006, rz0 - 0.006], [rx - 0.007, s * ry + 0.006, rz0 + 0.004]), "metal", "rail_foot", density="aluminium", label=f"rail_arm_{s:+d}"))
    col.add(Item(cyl_between((rx, 0, rz0), (rx, 0, rz1), 0.0022), "metal", "lead_screw", density="steel", label="lead_screw"))
    zb0, zb1 = rz1 - 0.011, rz1 - 0.001                                 # upper cross bar above the carriage's top position
    col.add(Item(box([rx - 0.006, -ry - 0.006, zb0], [rx + 0.007, ry + 0.006, zb1]), "sensor_body", "cross_bar", density="aluminium", label="upper_cross_bar"),
            Item(box([rx + 0.007, -0.020, zb0 + 0.003], [rx + 0.0085, 0.020, zb1 - 0.003]), "accent", "accent", density="petg_wall", label="upper_bar_accent"))
    for s in (-1, 1):
        col.add(Item(cyl(0.0022, 0.0015, "x", (rx + 0.0075, s * 0.025, (zb0 + zb1) / 2)), "screw", "screws", density="steel", label="bar_screw"))
    links.append(col)
    cb = c.d("lifter.carriage_bar")
    z0 = c.d("lifter.carriage_z0")
    trv = c.p("roles.lifter.lift_travel")
    car = Link("fork_carriage", role="lift", joint={"type": "prismatic", "axis": [0, 0, 1], "origin": T(rx, 0, z0), "limits": [0.0, trv], "authored": 0.0})
    car.add(Item(box([-cb[0] / 2, -cb[1] / 2 - 0.022, 0], [cb[0] / 2, cb[1] / 2 + 0.022, cb[2]]), "sensor_body", "bar", density="aluminium", label="carriage_bar"),
            Item(box([cb[0] / 2, -0.022, 0.004], [cb[0] / 2 + 0.0015, 0.022, 0.008]), "accent", "accent", density="petg_wall", label="carriage_accent"))
    for s in (-1, 1):
        car.add(Item(cyl(0.0055, cb[2] + 0.004, "z", (0, s * ry, cb[2] / 2)), "metal", "bushing", density="aluminium", label=f"bushing_{s:+d}"))
        car.add(Item(cyl(0.0025, 0.0015, "x", (cb[0] / 2 + 0.0008, s * 0.030, cb[2] / 2)), "screw", "screws", density="steel", label="carriage_screw"))
    links.append(car)
    pr = c.d("lifter.jaw_prong")
    up = c.d("lifter.jaw_upright")
    pad = c.d("lifter.jaw_pad")
    omax = c.p("roles.lifter.gripper_opening")
    omin = c.d("lifter.gripper_min_opening")
    xg = cb[0] / 2
    fingers = []
    for nm, s in (("finger_left", 1), ("finger_right", -1)):
        f = Link(nm, parent="fork_carriage", role="gripper",
                 joint={"type": "prismatic", "axis": [0, -s, 0], "origin": T(xg, s * omax / 2, 0), "limits": [0.0, (omax - omin) / 2], "authored": 0.0})
        pz0 = c.d("lifter.jaw_pad_z0")
        lo_y, hi_y = sorted((0.0, s * up[1]))
        f.add(Item(box([0, lo_y, pz0], [up[0], hi_y, pz0 + up[2]]), "metal", "upright", density="aluminium", label="jaw_upright"))
        lo_y, hi_y = sorted((0.0, s * pr[1]))
        prong = box([up[0] - 0.001, lo_y, -0.013], [up[0] + pr[0], hi_y, -0.013 + pr[2]])
        f.add(Item(prong, "metal", "prong", density="aluminium", label="jaw_prong"))
        f.add(Item(box([up[0] - 0.001, lo_y, pz0], [up[0] + pr[0] - 0.004, hi_y, -0.013]), "metal", "prong", density="aluminium", label="jaw_skirt"))
        lo_y, hi_y = sorted((0.0, -s * pad[1]))
        f.add(Item(box([up[0] - 0.002, lo_y, pz0], [up[0] + pr[0] - 0.004, hi_y, pz0 + pad[2]]), "rubber_pad", "pad", density="rubber", label="jaw_pad"))
        f.frames["pad_face"] = T(up[0] + (up[0] + pr[0]) / 2, -s * pad[1], pz0 + pad[2] / 2)
        links.append(f)
    tx, ty = c.d("lifter.tag_xy")
    return links, dict(tag_xy=(tx, ty), faces=faces)


def cargo_links(c):
    D = chassis_dims(c)
    H, hx, hy = D["H"], D["hx"], D["hy"]
    faces = {f: "dock" for f in FACE_ORDER}
    links = [chassis_body(c, "cargo", faces)]
    dl, dw = c.p("roles.cargo.deck_size_lw")
    dtop = c.p("roles.cargo.deck_height") - D["clr"]
    dt = c.d("cargo.deck_thickness")
    ch = c.d("cargo.deck_corner_chamfer")
    notch = c.d("cargo.flag_notch")
    deck_s = C.prism_xy(C.octagon(dl / 2, dw / 2, ch), dtop - dt, dtop)
    deck_s = deck_s - box([-dl / 2 - 0.01, dw / 2 - notch, dtop - dt - 0.01], [-dl / 2 + notch, dw / 2 + 0.01, dtop + 0.01])
    deck = Link("deck", role="deck")
    deck.add(Item(deck_s, "deck_plate", "deck", density="acm", label="deck_plate"))
    inner = C.prism_xy(C.octagon(dl / 2 - 0.010, dw / 2 - 0.010, ch), dtop, dtop + 0.0004)
    inner = inner - box([-dl / 2 - 0.01, dw / 2 - notch - 0.010, dtop - 0.01], [-dl / 2 + notch + 0.010, dw / 2 + 0.01, dtop + 0.01])
    deck.add(Item(inner, "deck_mat", "mat", density="rubber", label="deck_mat"))
    sx_, sy_ = c.d("cargo.standoff_xy")
    sr = c.d("cargo.standoff_radius")
    so = [(sx_, sy_), (sx_, -sy_), (-sx_, -sy_), (-dl / 2 + notch + 0.012, sy_)]
    for k, (x, y) in enumerate(so):
        deck.add(Item(cyl(sr, dtop - dt - H, "z", (x, y, (H + dtop - dt) / 2)), "metal", "standoffs", density="aluminium", label=f"standoff_{k}"),
                 Item(cyl(sr + 0.0015, 0.003, "z", (x, y, H + 0.0015)), "metal", "standoffs", density="aluminium", label=f"standoff_collar_{k}"),
                 Item(cyl(0.0028, 0.0012, "z", (x, y, dtop + 0.0006)), "screw", "screws", density="steel", label=f"deck_screw_{k}"))
    for k, (x, y) in enumerate([(0.0, dw / 2 - 0.006), (0.0, -dw / 2 + 0.006), (dl / 2 - 0.006, 0.0), (-dl / 2 + 0.006, 0.0), (0.085, dw / 2 - 0.006), (-0.085, dw / 2 - 0.006), (0.085, -dw / 2 + 0.006)]):
        deck.add(Item(cyl(0.0028, 0.0012, "z", (x, y, dtop + 0.0006)), "screw", "screws", density="steel", label=f"deck_rim_screw_{k}"))
    links.append(deck)
    eb = c.d("cargo.ebox")
    ebox = Link("electronics_box", role="body")
    ebs = box([-eb[0] / 2, -eb[1] / 2, H + 0.003], [eb[0] / 2, eb[1] / 2, H + 0.003 + eb[2]])
    ebs = ebs - box([-eb[0] / 2 + 0.0025, -eb[1] / 2 + 0.0025, H + 0.0055], [eb[0] / 2 - 0.0025, eb[1] / 2 - 0.0025, H + eb[2]])
    ebox.add(Item(ebs, "shell_cargo", "box", density="petg_wall", label="ebox_shell"))
    for sgn in (-1, 1):
        vent = box([sgn * eb[0] / 2 - 0.0006, -eb[1] / 2 + 0.008, H + 0.008], [sgn * eb[0] / 2 + 0.0006, eb[1] / 2 - 0.008, H + eb[2] - 0.003])
        ebox.add(Item(vent, "vent", "vent", density="petg_wall", label=f"vent_{sgn:+d}"))
    for sgn in (-1, 1):
        vent = box([-eb[0] / 2 + 0.010, sgn * eb[1] / 2 - 0.0006, H + 0.008], [eb[0] / 2 - 0.010, sgn * eb[1] / 2 + 0.0006, H + eb[2] - 0.003])
        ebox.add(Item(vent, "vent", "vent", density="petg_wall", label=f"vent_side_{sgn:+d}"))
    ebox.add(Item(box([-0.080, -0.040, H + 0.006], [0.050, 0.040, H + 0.007]), "internal", "x", mass=0.035, label="cargo_controller_board", render=False))
    links.append(ebox)
    # tag flag (inside the footprint, 2 mm below the deck top)
    fx, fy = c.d("cargo.flag_xy")
    fl = Link("tag_flag_mount", role="body")
    fp = notch - 0.004
    fl.add(Item(cyl(0.005, dtop - 0.005 - H, "z", (fx, fy, (H + dtop - 0.005) / 2)), "metal", "post", density="aluminium", label="flag_post"),
           Item(box([fx - fp / 2, fy - fp / 2, dtop - 0.005], [fx + fp / 2, fy + fp / 2, dtop - 0.002]), "sensor_body", "flag", density="aluminium", label="flag_plate"))
    links.append(fl)
    # front Arducam on a bracket
    camx, camz = c.d("cargo.camera_x"), c.d("cargo.camera_z")
    Tcam = T(camx, 0.0, camz)
    cam = Link("front_camera", role="sensor")
    its, lens_front = arducam_items(c, Tcam)
    cam.add(*its)
    cam.add(Item(box([camx - 0.020, -0.012, H], [camx - 0.0016, 0.012, H + 0.002]), "metal", "bracket", density="aluminium", label="cam_bracket_foot"),
            Item(box([camx - 0.004, -0.012, H], [camx - 0.0016, 0.012, camz - 0.006]), "metal", "bracket", density="aluminium", label="cam_bracket"))
    cam.frames["cam_front"] = Tcam @ T(lens_front, 0, 0)
    links.append(cam)
    return links, dict(tag_xy=(fx, fy), tag_z=dtop - 0.002 + 0.0003, faces=faces)
