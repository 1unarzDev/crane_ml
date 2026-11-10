#!/usr/bin/env python3
"""Generate PROVISIONAL rover geometry + manifests from the fleet profile.

    python gen_provisional.py [--profile PATH] [--defaults PATH] [--out DIR] [--roles scout lifter cargo payloads]

Every dimension comes from fleet_profile_v0.json or, where the profile is silent, from
generator_defaults.json (each entry provisional + a question for the designer). Nothing is hard-coded
here except tessellation-free maths. Output (default ~/.cache/crane-fleet/assets_v0):
  <role>/<part>.obj, <role>/<role>.mtl, <role>/<role>.manifest.json, textures/tag36h11_*.png
"""
import argparse
import hashlib
import json
import math
import os
import sys

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
import geom as g  # noqa: E402
import apriltag36h11  # noqa: E402

REPO = os.path.abspath(os.path.join(HERE, "..", "..", ".."))
DEFAULT_PROFILE = os.path.join(REPO, "Tools", "Fleet", "spec", "fleet_profile_v0.json")
DEFAULT_OUT = os.path.expanduser("~/.cache/crane-fleet/assets_v0")
SRC_REF = "Tools/Fleet/assets/gen_provisional.py"

ROLE_COLOR = {"scout": (0.16, 0.42, 0.86), "lifter": (0.93, 0.52, 0.12), "cargo": (0.28, 0.62, 0.36), "payloads": (0.7, 0.7, 0.7)}
MATERIALS = {
    "shell_dark": dict(base_color=[0.13, 0.14, 0.16], specular=0.2, shininess=30),
    "guide": dict(base_color=[0.98, 0.84, 0.10], specular=0.2, shininess=30, note="funnel / chamfer lead-in faces of the docking pocket"),
    "pocket": dict(base_color=[0.05, 0.05, 0.06]),
    "hook": dict(base_color=[0.90, 0.16, 0.12], specular=0.3, shininess=40),
    "pin": dict(base_color=[0.78, 0.78, 0.80], specular=0.6, shininess=80),
    "wheel_hub": dict(base_color=[0.10, 0.10, 0.11]),
    "roller": dict(base_color=[0.86, 0.86, 0.80], specular=0.2),
    "axle_cap": dict(base_color=[0.55, 0.56, 0.58], specular=0.5, shininess=60),
    "sensor_body": dict(base_color=[0.08, 0.08, 0.09]),
    "lens": dict(base_color=[0.02, 0.03, 0.08], specular=0.9, shininess=120),
    "lidar": dict(base_color=[0.20, 0.21, 0.24], specular=0.3),
    "lidar_window": dict(base_color=[0.55, 0.05, 0.05], specular=0.5),
    "metal": dict(base_color=[0.60, 0.62, 0.65], specular=0.5, shininess=60),
    "arm_envelope": dict(base_color=[0.70, 0.25, 0.85], alpha=0.30, note="reach envelope proxy, hidden by default"),
    "deck": dict(base_color=[0.46, 0.47, 0.50], specular=0.1),
    "marker": dict(base_color=[0.95, 0.95, 0.95], alpha=0.6),
    "foot": dict(base_color=[0.10, 0.10, 0.10]),
}


# ------------------------------------------------------------------ config
def pv(x):
    return x["value"] if isinstance(x, dict) and "value" in x else x


class Cfg:
    def __init__(self, profile_path, defaults_path):
        self.profile_path = profile_path
        self.prof = json.load(open(profile_path))
        self.dfl = json.load(open(defaults_path))["groups"]
        self.defaults_path = defaults_path
        self.used = set()

    def p(self, dotted):
        node = self.prof
        for k in dotted.split("."):
            node = node[k]
        return pv(node)

    def d(self, dotted):
        grp, key = dotted.split(".")
        self.used.add(dotted)
        return pv(self.dfl[grp][key])


def sha(path):
    return hashlib.sha256(open(path, "rb").read()).hexdigest()


def rel_to_repo(path):
    try:
        return os.path.relpath(path, REPO)
    except ValueError:
        return path


def tr(x=0, y=0, z=0):
    return g.make_T(None, [x, y, z])


def Tz(yaw, x=0, y=0, z=0):
    return g.make_T(g.rotz(yaw), [x, y, z])


class Part:
    def __init__(self, name, mesh, parent="base_link", T=None, joint=None, visible=True, kind="solid",
                 hide_group=None, role="body", contacts_ground=False):
        self.name, self.mesh, self.parent = name, mesh, parent
        self.T = np.eye(4) if T is None else T
        self.joint, self.visible, self.kind = joint, visible, kind
        self.hide_group, self.role, self.contacts_ground = hide_group, role, contacts_ground

    def entry(self):
        e = {"name": self.name, "file": self.name + ".obj", "parent": self.parent, "origin": g.pose_of(self.T),
             "materials": list(self.mesh.mats), "kind": self.kind, "default_visible": self.visible,
             "role_in_assembly": self.role}
        if self.joint:
            e["joint"] = self.joint
        if self.hide_group:
            e["hideable_group"] = self.hide_group
        if self.contacts_ground:
            e["contacts_ground"] = True
        return e


# ------------------------------------------------------------------ geometry
def wheel_mesh(c, sign, shell_name="roller"):
    r = c.p("chassis.wheel.radius")
    w = c.p("chassis.wheel.width")
    n = c.p("chassis.wheel.roller_count")
    alpha = c.d("wheel.roller_angle")
    fill = c.d("wheel.roller_fill")
    seg_h, seg_r = c.d("tessellation.wheel_hub_segments"), c.d("tessellation.roller_segments")
    nst = c.d("tessellation.roller_profile_stations")
    s_ = math.sin(math.pi / n) * math.sin(alpha)
    rr = fill * r * s_ / (1 + fill * s_)            # roller max radius
    Rc = r - rr                                      # roller centre circle
    L = (w - 2 * rr * math.sin(alpha)) / math.cos(alpha)
    to_y = g.make_T(g.rotx(-math.pi / 2))
    m = g.Mesh()
    hub = g.cylinder(c.d("wheel.hub_radius_frac") * r, 0.9 * w, seg_h, "wheel_hub", "hub")
    m.append(hub.transformed(to_y))
    tp = c.d("wheel.end_plate_thickness")
    for sgn in (-1, 1):
        plate = g.cylinder(c.d("wheel.end_plate_radius_frac") * Rc, tp, seg_h, "wheel_hub", "hub")
        m.append(plate.transformed(tr(0, sgn * (w / 2 - tp / 2), 0) @ to_y))
        cap = g.cylinder(c.d("wheel.axle_cap_radius"), c.d("wheel.axle_cap_length"), 16, "axle_cap", "hub")
        m.append(cap.transformed(tr(0, sgn * (w / 2 + c.d("wheel.axle_cap_length") / 2 - 0.001), 0) @ to_y))
    taper = c.d("wheel.roller_taper")
    ts = np.linspace(-L / 2, L / 2, nst)
    prof = [(t, rr * math.sqrt(max(1 - taper * (2 * t / L) ** 2, 0.0))) for t in ts]
    base = g.lathe(prof, seg_r, shell_name, "roller_00")
    for i in range(n):
        th = -math.pi / 2 + 2 * math.pi * i / n          # i=0 is the ground-contact roller (bottom)
        tang = np.array([-math.sin(th), 0.0, math.cos(th)])
        u = math.sin(alpha) * tang + math.cos(alpha) * sign * np.array([0, 1.0, 0])
        ctr = Rc * np.array([math.cos(th), 0.0, math.sin(th)])
        T = g.make_T(g.R_align_z(u), ctr)
        piece = base.transformed(T)
        piece.groups = ["roller_%02d" % i]
        m.append(piece)
    return m, dict(roller_radius=rr, roller_length=L, roller_circle=Rc)


def hook_mesh(c, mirrored):
    L, aw, t = c.d("dock.hook_arm_length"), c.d("dock.hook_arm_width"), c.d("dock.hook_thickness")
    pd, reach, boss = c.d("dock.hook_prong_depth"), c.d("dock.hook_prong_reach"), c.d("dock.hook_boss_radius")
    arm = g.box_mesh([0, -aw / 2, -t / 2], [L, aw / 2, t / 2], "hook",
                     chamfers=[((L, aw / 2, 0), (0, 1, 0), (1, 0, 0), 0.002), ((L, -aw / 2, 0), (0, -1, 0), (1, 0, 0), 0.002)])
    prong = g.box_mesh([L - pd, -aw / 2 - reach, -t / 2], [L, aw / 2 - 0.002, t / 2], "hook",
                       chamfers=[((L, -aw / 2 - reach, 0), (1, 0, 0), (0, -1, 0), min(0.6 * reach, 0.006))])
    bs = g.cylinder(boss, t + 0.004, 20, "pin", "boss")
    m = g.Mesh()
    for part in (arm, prong, bs):
        m.append(part)
    if mirrored:
        m = m.transformed(np.diag([1, -1, 1, 1.0]))
    return m


def dock_geometry(c):
    """Derived dock numbers shared by mesh + manifest."""
    s = c.p("docking.hook_lateral_spacing")
    L = c.d("dock.hook_arm_length")
    op = c.d("dock.hook_open_angle")
    pd = c.d("dock.pocket_depth")
    aw, pr, clr = c.d("dock.hook_arm_width"), c.d("dock.pin_radius"), c.d("dock.engage_clearance")
    prongd = c.d("dock.hook_prong_depth")
    pivot_x = -(pd - c.d("dock.hook_pivot_inset"))
    crook_local = np.array([L - prongd - pr - clr, -(aw / 2 + pr + clr), 0.0])   # hook A (y>0)
    pw = s / 2 + L * math.sin(op) + c.d("dock.pocket_side_margin")
    return dict(s=s, L=L, op=op, pd=pd, pivot_x=pivot_x, crook_local=crook_local, pw=pw,
                ph=c.d("dock.pocket_half_height"), dz=c.d("dock.hook_level_dz"), pr=pr,
                t=c.d("dock.hook_thickness"))


def dock_module_mesh(c, shell_mat, zlo, zhi):
    """Docking end block in the face frame: x in [-D,0], origin on the mating plane at face centre."""
    dk = dock_geometry(c)
    W = c.p("chassis.body_size_lwh")[1]
    D = c.d("chassis.dock_module_depth")
    pd, pw, ph = dk["pd"], dk["pw"], dk["ph"]
    ce, cc = c.d("chassis.edge_chamfer"), c.d("chassis.corner_chamfer")
    rail_h = zhi - ph
    cf = min(c.p("docking.guide_half_width"), c.d("dock.guide_chamfer_max_frac_of_rail") * rail_h)
    cte = min(ce, max(rail_h - cf - 0.002, 0.001))
    tm = {"guide": "guide", "chamfer": shell_mat, "+x": shell_mat, "-x": shell_mat, "+y": shell_mat, "-y": shell_mat, "+z": shell_mat, "-z": shell_mat}
    longedges = lambda x: [((x, W / 2, zhi), (0, 1, 0), (0, 0, 1), ce), ((x, -W / 2, zhi), (0, -1, 0), (0, 0, 1), ce),
                           ((x, W / 2, zlo), (0, 1, 0), (0, 0, -1), ce), ((x, -W / 2, zlo), (0, -1, 0), (0, 0, -1), ce)]
    vcorners = [((0, W / 2, 0), (1, 0, 0), (0, 1, 0), cc), ((0, -W / 2, 0), (1, 0, 0), (0, -1, 0), cc)]
    m = g.Mesh()
    uv = c.d("chassis.uv_scale")
    m.append(g.box_mesh([-D, -W / 2, zlo], [-pd, W / 2, zhi], "pocket", "dock", longedges(-D), tm, uv))
    top = [((0, 0, ph), (1, 0, 0), (0, 0, -1), cf, "guide"), ((0, 0, zhi), (1, 0, 0), (0, 0, 1), cte)] + vcorners + longedges(0)
    bot = [((0, 0, -ph), (1, 0, 0), (0, 0, 1), cf, "guide"), ((0, 0, zlo), (1, 0, 0), (0, 0, -1), ce)] + vcorners + longedges(0)
    m.append(g.box_mesh([-pd, -W / 2, ph], [0, W / 2, zhi], shell_mat, "dock", top, tm, uv))
    m.append(g.box_mesh([-pd, -W / 2, zlo], [0, W / 2, -ph], shell_mat, "dock", bot, tm, uv))
    for sg in (1, -1):
        e = [((0, sg * pw, 0), (1, 0, 0), (0, -sg, 0), cf, "guide"), ((0, sg * W / 2, 0), (1, 0, 0), (0, sg, 0), cc),
             ((0, sg * W / 2, zhi), (0, sg, 0), (0, 0, 1), ce), ((0, sg * W / 2, zlo), (0, sg, 0), (0, 0, -1), ce)]
        lo, hi = (pw, W / 2) if sg > 0 else (-W / 2, -pw)
        m.append(g.box_mesh([-pd, lo, zlo], [0, hi, zhi], shell_mat, "dock", e, tm, uv))
    # catch pins (face frame): upper-level pin at y=-yc, lower-level pin at y=+yc (see spec)
    xc = dk["pivot_x"] + dk["crook_local"][0]
    yc = dk["s"] / 2 + dk["crook_local"][1]
    dz, t = dk["dz"], dk["t"]
    seg = c.d("tessellation.pin_segments")
    pins = []
    for name, y, lvl in (("pin_hi", -yc, +1), ("pin_lo", +yc, -1)):
        if lvl > 0:
            z0, z1 = dz - t / 2 - 0.003, ph
        else:
            z0, z1 = -ph, -dz + t / 2 + 0.003
        pm = g.cylinder(dk["pr"], z1 - z0, seg, "pin", "pins").transformed(tr(-xc, y, (z0 + z1) / 2))
        m.append(pm)
        pins.append({"name": name, "position": [round(-xc, 9), round(y, 9), round(lvl * dz, 9)], "radius": dk["pr"]})
    return m, dict(cf=cf, pins=pins, xc=xc, yc=yc)


def chassis_core(c, shell_mat, faces_T):
    L, W, H = c.p("chassis.body_size_lwh")
    D = c.d("chassis.dock_module_depth")
    ce = c.d("chassis.edge_chamfer")
    tp = c.d("chassis.top_plate_thickness")
    lx = c.p("chassis.wheel.half_wheelbase_lx")
    r = c.p("chassis.wheel.radius")
    xwell = lx - r - c.d("chassis.wheel_well_margin")
    xe = L / 2 - D
    inset = c.d("chassis.wheel_well_inset")
    uv = c.d("chassis.uv_scale")
    m = g.Mesh()
    tm = {"chamfer": shell_mat}
    m.append(g.box_mesh([-xwell, -W / 2, 0], [xwell, W / 2, H - tp], shell_mat, "core",
                        [((0, W / 2, 0), (0, 1, 0), (0, 0, -1), ce), ((0, -W / 2, 0), (0, -1, 0), (0, 0, -1), ce)], tm, uv))
    for sx in (1, -1):
        lo, hi = sorted((sx * xwell, sx * xe))
        m.append(g.box_mesh([lo, -W / 2 + inset, 0], [hi, W / 2 - inset, H - tp], "shell_dark", "core",
                            [((0, W / 2 - inset, 0), (0, 1, 0), (0, 0, -1), ce), ((0, -W / 2 + inset, 0), (0, -1, 0), (0, 0, -1), ce)], tm, uv))
    m.append(g.box_mesh([-xe, -W / 2, H - tp], [xe, W / 2, H], shell_mat, "core",
                        [((0, W / 2, H), (0, 1, 0), (0, 0, 1), ce), ((0, -W / 2, H), (0, -1, 0), (0, 0, 1), ce),
                         ((0, W / 2, H - tp), (0, 1, 0), (0, 0, -1), ce)], tm, uv))
    return m


def wrap_tag_plate(size, tag_id, name):
    plate = size * 10.0 / 8.0
    p = plate / 2
    corners = [(-p, -p, 0), (-p, p, 0), (p, p, 0), (p, -p, 0)]        # TL, BL, BR, TR in the tag frame
    uvs = [(0, 1), (0, 0), (1, 0), (1, 1)]
    mat = "tag36h11_%05d" % tag_id
    return g.quad_mesh(corners, uvs, mat, "plate"), plate, mat


R_TAG_FACE = np.array([[0, 0, -1], [1, 0, 0], [0, -1, 0]], float)   # columns x_t=(0,1,0) y_t=(0,0,-1) z_t=(-1,0,0)
R_TAG_TOP = np.array([[0, -1, 0], [-1, 0, 0], [0, 0, -1]], float)    # columns x_t=(0,-1,0) y_t=(-1,0,0) z_t=(0,0,-1)


def quat_cam_pose(T_link):
    return g.pose_of(T_link), g.pose_of(T_link @ g.make_T(
        np.array([[0, 0, 1], [-1, 0, 0], [0, -1, 0]], float)))


def camera_mesh(c, size, name="cam"):
    sx, sy, sz = size
    m = g.box_mesh([-sx, -sy / 2, -sz / 2], [0, sy / 2, sz / 2], "sensor_body", "body")
    lens = g.cylinder(min(sy, sz) * 0.32, 0.008, c.d("tessellation.cylinder_segments"), "lens", "lens")
    m.append(lens.transformed(g.make_T(g.roty(math.pi / 2), [0.004, 0, 0])))
    return m


def inertia_box(mass, size):
    x, y, z = size
    return np.diag([mass / 12 * (y * y + z * z), mass / 12 * (x * x + z * z), mass / 12 * (x * x + y * y)])


def combine(components):
    """components: [(mass, com(3), I_about_own_com (3x3))] -> (mass, com, I_about_total_com)."""
    M = sum(m for m, _, _ in components)
    com = sum(m * np.array(cm, float) for m, cm, _ in components) / M
    I = np.zeros((3, 3))
    for m, cm, Ic in components:
        d = np.array(cm, float) - com
        I += Ic + m * (d @ d * np.eye(3) - np.outer(d, d))
    return M, com, I


# ------------------------------------------------------------------ role builder
def build_role(c, role):
    prof = c.prof
    L, W, H = c.p("chassis.body_size_lwh")
    clr = c.p("chassis.ground_clearance")
    r = c.p("chassis.wheel.radius")
    ww = c.p("chassis.wheel.width")
    lx, ly = c.p("chassis.wheel.half_wheelbase_lx"), c.p("chassis.wheel.half_track_ly")
    roles = prof["roles"][role]
    shell = "shell_" + role
    materials = dict(MATERIALS)
    materials[shell] = dict(base_color=list(ROLE_COLOR[role]), specular=0.25, shininess=35)
    parts, notes = [], {}
    dk = dock_geometry(c)
    faces = {}
    for fname, fd in prof["docking"]["face_frames"].items():
        if fname == "note":
            continue
        x, y, z, yaw = pv(fd)
        faces[fname] = Tz(yaw, x, y, z)
    active = prof["docking"]["active_faces_default"]
    zc = pv(prof["docking"]["face_frames"]["front"])[2]

    # chassis body ---------------------------------------------------------
    body = chassis_core(c, shell, faces)
    pins_by_face = {}
    for fname in active:
        mod, info = dock_module_mesh(c, shell, -zc, H - zc)
        body.append(mod.transformed(faces[fname]))
        pins_by_face[fname] = info
    parts.append(Part("chassis_body", body, role="body"))

    # hooks ---------------------------------------------------------------
    hooks_manifest, pivots, hooks_by_face = [], [], {}
    for fname in active:
        for hid, sg in (("a", 1), ("b", -1)):
            name = f"hook_{fname}_{hid}"
            Tp = faces[fname] @ tr(dk["pivot_x"], sg * dk["s"] / 2, sg * dk["dz"])
            hm = hook_mesh(c, mirrored=(sg < 0))
            lo, hi = sorted((0.0, sg * c.d("dock.hook_open_angle")))
            parts.append(Part(name, hm, T=Tp, role="hook",
                              joint={"type": "revolute", "axis": [0, 0, 1], "limits": [lo, hi], "authored_value": 0.0, "actuated_ref": name}))
            pivots.append({"name": f"pivot_{fname}_{hid}", "face": fname, "parent": "base_link", "origin": g.pose_of(Tp)})
            crook = dk["crook_local"] * np.array([1, sg, 1])
            hooks_manifest.append({
                "name": name, "face": fname, "part": name, "pivot": f"pivot_{fname}_{hid}", "axis": [0, 0, 1],
                "closed_angle": 0.0, "open_angle": sg * c.d("dock.hook_open_angle"), "authored_angle": 0.0,
                "engage_point": [round(float(v), 9) for v in crook], "level_dz": sg * dk["dz"],
                "rated_load": c.p("docking.hook_rated_load"), "actuation_time": c.p("docking.hook_actuation_time")})
            hooks_by_face.setdefault(fname, []).append(name)

    # wheels --------------------------------------------------------------
    wheel_items = []
    wm_info = None
    for name, (sx, sy) in {"FL": (1, 1), "FR": (1, -1), "RL": (-1, 1), "RR": (-1, -1)}.items():
        sign = -sx * sy                                  # X convention: s = -sign(x*y)
        wm, wm_info = wheel_mesh(c, sign)
        T = tr(sx * lx, sy * ly, r - clr)
        parts.append(Part(f"wheel_{name}", wm, T=T, role="wheel", contacts_ground=True,
                          joint={"type": "continuous", "axis": [0, 1, 0], "authored_value": 0.0}))
        a = [round(math.sin(c.d("wheel.roller_angle")), 9), round(sign * math.cos(c.d("wheel.roller_angle")), 9)]
        wheel_items.append({"name": name, "position": [sx * lx, sy * ly, r - clr], "frame": g.pose_of(T), "axis": [0, 1, 0],
                            "radius": r, "width": ww, "roller_count": c.p("chassis.wheel.roller_count"),
                            "roller_angle": c.d("wheel.roller_angle"), "roller_angle_sign": sign,
                            "ground_roller_axis_xy": a, "render_part": f"wheel_{name}", "collision_shape": f"wheel_{name}_col"})

    # tags ----------------------------------------------------------------
    ids = c.d("tags.id_base")[role]
    tags, tag_ids = [], {}
    face_order = ["front", "rear", "left", "right"]
    for fname in active:
        tid = ids + face_order.index(fname)
        mesh, plate, mat = wrap_tag_plate(c.p("sensors.apriltag.face_tag_size"), tid, f"tag_{fname}")
        Tt = faces[fname] @ tr(-dk["pd"] + c.d("dock.tag_recess"), 0, 0) @ g.make_T(R_TAG_FACE)
        parts.append(Part(f"tag_{fname}", mesh, T=Tt, kind="quad", role="tag"))
        nrm = (Tt[:3, :3] @ np.array([0, 0, -1.0])).tolist()
        tags.append({"name": f"tag_{fname}", "family": "tag36h11", "id": tid, "size": c.p("sensors.apriltag.face_tag_size"),
                     "plate_size": plate, "parent": "base_link", "origin": g.pose_of(Tt), "outward_normal": [round(v, 9) for v in nrm],
                     "render_part": f"tag_{fname}", "texture": f"../textures/tag36h11_{tid:05d}.png",
                     "instance_id_offset_per_unit": c.d("tags.id_offset_second_cargo") if role == "cargo" else 0})
        tag_ids[mat] = tid
    top_z = H
    if role == "cargo":
        top_z = c.p("roles.cargo.deck_height") - clr
    tx, ty = c.d("tags.top_tag_offset_xy")[role]
    tid = ids + 4
    mesh, plate, mat = wrap_tag_plate(c.p("sensors.apriltag.top_tag_size"), tid, "tag_top")
    Tt = tr(tx, ty, top_z + c.d("tags.top_tag_standoff")) @ g.make_T(R_TAG_TOP)
    parts.append(Part("tag_top", mesh, T=Tt, kind="quad", role="tag"))
    tags.append({"name": "tag_top", "family": "tag36h11", "id": tid, "size": c.p("sensors.apriltag.top_tag_size"), "plate_size": plate,
                 "parent": "base_link", "origin": g.pose_of(Tt), "outward_normal": [0.0, 0.0, 1.0], "render_part": "tag_top",
                 "texture": f"../textures/tag36h11_{tid:05d}.png", "may_be_occluded_by_payload": role == "cargo",
                 "instance_id_offset_per_unit": c.d("tags.id_offset_second_cargo") if role == "cargo" else 0})
    tag_ids[mat] = tid
    # side faces: marker only ----------------------------------------------
    mk = c.d("dock.marker_size")
    for fname in ("left", "right"):
        mesh = g.box_mesh([-0.003, -mk[0] / 2, -mk[1] / 2], [0, mk[0] / 2, mk[1] / 2], "marker", "marker")
        parts.append(Part(f"dock_marker_{fname}", mesh, T=faces[fname], visible=False, role="marker", hide_group="side_dock_markers"))

    sensors, collision, actuated_extra = [], [], {}
    box = lambda n, parent, T, size: {"name": n, "type": "box", "parent": parent, "origin": g.pose_of(T), "size": list(size)}
    cyl = lambda n, parent, T, rad, h: {"name": n, "type": "cylinder", "parent": parent, "origin": g.pose_of(T), "radius": rad, "height": h}
    sd = c.d("sensors.camera_body_size")
    tofs = c.d("sensors.tof_body_size")
    cam_p = prof["sensors"]["mono_camera_ov9281"]
    def cam_entry(name, parent, T, render_part=None):
        po, oo = quat_cam_pose(T)
        e = {"name": name, "type": "camera", "parent": parent, "origin": po, "optical_origin": oo,
             "optical_axis_convention": "optical frame: x right, y down, z = optical axis (forward); link frame: x = optical axis, y left, z up",
             "params": {"resolution": pv(cam_p["resolution"]), "hfov": pv(cam_p["hfov"]), "rate_hz": pv(cam_p["rate_hz"]),
                        "latency": pv(cam_p["latency"]), "shutter": cam_p["shutter"], "encoding": cam_p["encoding"]}}
        if render_part:
            e["render_part"] = render_part
        return e
    tof_p = prof["sensors"]["range_tof"]
    def tof_entry(name, parent, T, render_part=None):
        e = {"name": name, "type": "tof", "parent": parent, "origin": g.pose_of(T),
             "params": {"max_range": pv(tof_p["max_range"]), "fov": pv(tof_p["fov"]), "rate_hz": pv(tof_p["rate_hz"])}}
        if render_part:
            e["render_part"] = render_part
        return e
    imu_p = prof["sensors"]["imu"]
    sensors.append({"name": "imu", "type": "imu", "parent": "base_link", "origin": g.pose_of(tr(*c.d("sensors.imu_position"))),
                    "params": {"rate_hz": pv(imu_p["rate_hz"]), "gyro_noise_density": pv(imu_p["gyro_noise_density"]),
                               "gyro_bias_instability": pv(imu_p["gyro_bias_instability"])}})
    body_col = box("chassis_col", "base_link", tr(0, 0, H / 2), (L, W, H))
    collision.append(body_col)
    for wi in wheel_items:
        collision.append(cyl(wi["collision_shape"], f"wheel_{wi['name']}", g.make_T(g.rotx(-math.pi / 2)), r, ww))

    added = roles["added_mass"]["value"]
    comps = [(c.p("chassis.base_mass"), c.p("chassis.com_offset_xyz"), np.diag(c.p("chassis.inertia_diag")))]
    optional = []
    actuated = {"hooks": hooks_manifest}
    ext_notes = {}

    # ---- role specific ------------------------------------------------------
    seg = c.d("tessellation.cylinder_segments")
    if role == "scout":
        mh = roles["mast_height"]["value"]
        mx = c.d("scout.mast_x")
        mtop = H + mh
        bt = c.d("scout.mast_base_height")
        mast = g.cylinder(c.d("scout.mast_base_radius"), bt, seg, "metal", "mast_base").transformed(tr(mx, 0, H + bt / 2))
        mast.append(g.cylinder(c.d("scout.mast_radius"), mh - bt, seg, "metal", "mast").transformed(tr(mx, 0, H + bt + (mh - bt) / 2)))
        parts.append(Part("mast", mast, role="body"))
        base = prof["sensors"]["stereo_baseline"]["value"]
        bar_l, bar_d, bar_h = base + c.d("scout.stereo_bar_margin"), c.d("scout.stereo_bar_depth"), c.d("scout.stereo_bar_height")
        zbar = mtop - c.d("scout.stereo_bar_below_mast_top")
        bar = g.box_mesh([0, -bar_l / 2, -bar_h / 2], [bar_d, bar_l / 2, bar_h / 2], "sensor_body", "bar",
                         [((bar_d, bar_l / 2, 0), (1, 0, 0), (0, 1, 0), 0.004), ((bar_d, -bar_l / 2, 0), (1, 0, 0), (0, -1, 0), 0.004)])
        for sg in (1, -1):
            lens = g.cylinder(c.d("scout.lens_radius"), c.d("scout.lens_length"), 20, "lens", "lens")
            bar.append(lens.transformed(g.make_T(g.roty(math.pi / 2), [bar_d + c.d("scout.lens_length") / 2 - 0.002, sg * base / 2, 0])))
        Tbar = tr(mx - bar_d / 2, 0, zbar)
        parts.append(Part("stereo_bar", bar, T=Tbar, role="sensor"))
        for nm, sg in (("cam_left", 1), ("cam_right", -1)):
            Tc = Tbar @ tr(bar_d + c.d("scout.lens_length") - 0.002, sg * base / 2, 0)
            sensors.append(cam_entry(nm, "base_link", Tc, "stereo_bar"))
        pr, ph = c.d("scout.puck_radius"), c.d("scout.puck_height")
        bf = c.d("scout.puck_base_frac")
        pk = g.cylinder(pr * 1.0, ph * bf, c.d("tessellation.puck_segments"), "lidar", "puck_base").transformed(tr(0, 0, ph * bf / 2))
        parts.append(Part("lidar_base", pk, T=tr(mx, 0, mtop), role="sensor"))
        head = g.cylinder(pr * 0.92, ph * (1 - bf), c.d("tessellation.puck_segments"), "lidar", "puck_head")
        win = g.cylinder(pr * 0.93, ph * (1 - bf) * 0.35, c.d("tessellation.puck_segments"), "lidar_window", "window")
        head.append(win)
        parts.append(Part("lidar_head", head, T=tr(mx, 0, mtop + ph * bf + ph * (1 - bf) / 2), role="sensor",
                          joint={"type": "continuous", "axis": [0, 0, 1], "authored_value": 0.0}))
        lp = prof["sensors"]["lidar_2d"]
        sensors.append({"name": "lidar", "type": "lidar2d", "parent": "base_link", "origin": g.pose_of(tr(mx, 0, mtop + ph * bf + ph * (1 - bf) / 2)),
                        "params": {"max_range": pv(lp["max_range"]), "rate_hz": pv(lp["rate_hz"]), "points_per_scan": pv(lp["points_per_scan"]),
                                   "fov": 2 * math.pi, "scan_plane": "xy of this frame, +z up, angle counter-clockwise from +x"},
                        "render_part": "lidar_head"})
        actuated["lidar_spin"] = {"part": "lidar_head", "axis": [0, 0, 1], "rate_hz": pv(lp["rate_hz"])}
        rs = c.d("scout.rear_camera_size")
        rc = camera_mesh(c, rs)
        Trc = Tz(math.pi, c.d("scout.rear_camera_x"), 0, H + rs[2] / 2 - 0.0)
        parts.append(Part("rear_camera", rc, T=Trc, role="sensor"))
        sensors.append(cam_entry("cam_rear", "base_link", Trc @ tr(0.004 + 0.0, 0, 0), "rear_camera"))
        # arm pedestal + envelope
        ax = c.d("scout.arm_pedestal_x")
        ped = g.cylinder(c.d("scout.arm_pedestal_radius"), c.d("scout.arm_pedestal_height"), seg, "metal", "pedestal")
        parts.append(Part("arm_pedestal", ped, T=tr(ax, 0, H + c.d("scout.arm_pedestal_height") / 2), role="body"))
        reach = roles["arm_proxy"]["reach"]["value"]
        ew, eh = c.d("scout.arm_envelope_width"), c.d("scout.arm_envelope_height")
        env = g.box_mesh([0, -ew / 2, 0], [reach, ew / 2, eh], "arm_envelope", "envelope")
        z0 = H + c.d("scout.arm_pedestal_height") + c.d("scout.arm_envelope_height_above_top")
        parts.append(Part("arm_proxy_envelope", env, T=tr(ax, 0, z0), visible=False, hide_group="arm_envelope", role="envelope"))
        actuated["arm_proxy"] = {"part": "arm_proxy_envelope", "mass": roles["arm_proxy"]["mass"]["value"], "reach": reach,
                                 "payload": roles["arm_proxy"]["payload"]["value"], "default_visible": False,
                                 "modelled_as": roles["arm_proxy"]["modelled_as"]}
        collision.append(cyl("mast_col", "base_link", tr(mx, 0, H + mh / 2), c.d("scout.mast_radius") * 2, mh))
        collision.append(cyl("lidar_col", "base_link", tr(mx, 0, mtop + ph / 2), pr, ph))
        collision.append(box("stereo_bar_col", "base_link", Tbar @ tr(bar_d / 2, 0, 0), (bar_d, bar_l, bar_h)))
        fp = c.d("scout.mast_added_mass_footprint")
        hb = min(c.d("mass_model.scout_added_box_height"), mh)
        comps.append((added, [mx, 0, H + hb / 2], inertia_box(added, [fp[0], fp[1], hb])))
        am = roles["arm_proxy"]["mass"]["value"]
        ab = c.d("mass_model.arm_proxy_box")
        acom = [ax + reach / 2, 0, H + c.d("scout.arm_pedestal_height") + ab[2] / 2]
        optional.append({"name": "arm_proxy", "mass": am, "com_xyz": acom, "enabled_by_default": False,
                         "inertia_tensor_about_own_com": inertia_box(am, [reach, ab[1], ab[2]]).tolist()})
        top_height = mtop + ph
    elif role == "lifter":
        cx = c.d("lifter.column_x")
        csz = c.d("lifter.column_size")
        ch = c.d("lifter.column_height_above_top")
        col = g.box_mesh([-csz[0] / 2, -csz[1] / 2, 0], [csz[0] / 2, csz[1] / 2, ch], "metal", "column",
                         [((csz[0] / 2, csz[1] / 2, 0), (1, 0, 0), (0, 1, 0), 0.005), ((csz[0] / 2, -csz[1] / 2, 0), (1, 0, 0), (0, -1, 0), 0.005)])
        parts.append(Part("lift_column", col, T=tr(cx, 0, H), role="body"))
        collision.append(box("column_col", "base_link", tr(cx, 0, H + ch / 2), (csz[0], csz[1], ch)))
        # carriage (moves up with the lift); frame origin = face plane + standoff, plate bottom at lift = 0
        face_x = L / 2
        cpt, cph, cpw = c.d("lifter.carriage_plate_thickness"), c.d("lifter.carriage_plate_height"), c.d("lifter.carriage_plate_width")
        zbot = -clr + c.d("lifter.carriage_low_bottom_above_ground")
        standoff = dk["pivot_x"] + dk["L"] + c.d("lifter.carriage_standoff")   # clear the closed hook tips
        Tcar = tr(face_x + standoff, 0, zbot)
        plate = g.box_mesh([0, -cpw / 2, 0], [cpt, cpw / 2, cph], shell, "carriage",
                           [((cpt, cpw / 2, 0), (1, 0, 0), (0, 1, 0), 0.006), ((cpt, -cpw / 2, 0), (1, 0, 0), (0, -1, 0), 0.006)])
        shoe_h = c.d("lifter.carriage_shoe_height")
        shoe_len = (face_x + standoff) - (cx + csz[0] / 2 - 0.0)
        zshoe = (H + 0.0 + 0.5 * shoe_h + 0.002) - zbot                 # at lift=0 the shoe rides just above the body top
        shoe = g.box_mesh([-shoe_len, -csz[1] / 2 + 0.01, zshoe - shoe_h / 2], [0.001, csz[1] / 2 - 0.01, zshoe + shoe_h / 2], "metal", "shoe")
        fl, ft, fm = c.d("lifter.fork_plate_length"), c.d("lifter.fork_plate_thickness"), c.d("lifter.fork_plate_margin")
        open_max = roles["gripper_opening"]["value"]
        fw = open_max + 2 * (c.d("lifter.finger_thickness") + fm)
        fork = g.box_mesh([cpt - 0.001, -fw / 2, 0], [cpt + fl, fw / 2, ft], shell, "fork",
                          [((cpt + fl, fw / 2, 0), (1, 0, 0), (0, 1, 0), 0.006), ((cpt + fl, -fw / 2, 0), (1, 0, 0), (0, -1, 0), 0.006),
                           ((cpt + fl, 0, ft), (1, 0, 0), (0, 0, 1), 0.004)])
        car = g.Mesh()
        for part in (plate, shoe, fork):
            car.append(part)
        trv = roles["lift_travel"]["value"]
        parts.append(Part("fork_carriage", car, T=Tcar, role="lift",
                          joint={"type": "prismatic", "axis": [0, 0, 1], "limits": [0.0, trv], "authored_value": 0.0, "actuated_ref": "lift"}))
        collision.append(box("carriage_plate_col", "fork_carriage", tr(cpt / 2, 0, cph / 2), (cpt, cpw, cph)))
        collision.append(box("fork_plate_col", "fork_carriage", tr(cpt + fl / 2, 0, ft / 2), (fl, fw, ft)))
        # fingers: authored fully open; inner faces at +-open_max/2
        fth, fln, fht = c.d("lifter.finger_thickness"), c.d("lifter.finger_length"), c.d("lifter.finger_height")
        f0 = cpt + c.d("lifter.finger_start_from_plate")
        fingers = []
        for nm, sg in (("finger_left", 1), ("finger_right", -1)):
            fm_ = g.box_mesh([0, 0, 0], [fln, fth, fht], "metal", "finger",
                             [((fln, 0, fht), (1, 0, 0), (0, 0, 1), 0.006)])
            if sg < 0:
                fm_ = fm_.transformed(tr(0, -fth, 0))
            Tf = tr(f0, sg * open_max / 2, ft)
            mn_open = c.d("lifter.gripper_min_opening")
            travel = (open_max - mn_open) / 2
            parts.append(Part(nm, fm_, parent="fork_carriage", T=Tf, role="gripper",
                              joint={"type": "prismatic", "axis": [0, -sg, 0], "limits": [0.0, travel], "authored_value": 0.0, "actuated_ref": "gripper"}))
            collision.append(box(nm + "_col", nm, tr(fln / 2, sg * fth / 2, fht / 2), (fln, fth, fht)))
            fingers.append({"part": nm, "axis": [0, -sg, 0], "travel": [0.0, travel]})
        actuated["lift"] = {"part": "fork_carriage", "axis": [0, 0, 1], "range": [0.0, trv], "authored_value": 0.0, "docking_value": trv,
                            "speed": roles["lift_speed"]["value"], "capacity": roles["lift_capacity"]["value"],
                            "note": "At lift=0 the carriage plate blocks the front docking pocket; raise to docking_value before mating the front face."}
        actuated["gripper"] = {"fingers": fingers, "opening_range": [c.d("lifter.gripper_min_opening"), open_max], "authored_opening": open_max,
                               "close_time": roles["gripper_close_time"]["value"],
                               "note": "opening = clear gap between finger inner faces; each finger moves (open_max-opening)/2 along its axis"}
        # camera + tof on the carriage plate
        zc_ = c.d("lifter.camera_dz_above_plate_bottom")
        Tcam = Tcar @ tr(cpt, 0, zc_)
        rcm = camera_mesh(c, sd)
        parts.append(Part("front_camera", rcm, parent="fork_carriage", T=tr(cpt, 0, zc_), role="sensor"))
        sensors.append(cam_entry("cam_front", "fork_carriage", tr(cpt + 0.004, 0, zc_), "front_camera"))
        tm_ = g.box_mesh([-tofs[0], -tofs[1] / 2, -tofs[2] / 2], [0, tofs[1] / 2, tofs[2] / 2], "sensor_body", "body")
        Ttof = tr(cpt, c.d("lifter.tof_offset_y"), zc_)
        parts.append(Part("front_tof", tm_, parent="fork_carriage", T=Ttof, role="sensor"))
        sensors.append(tof_entry("tof_front", "fork_carriage", Ttof, "front_tof"))
        lb = c.d("mass_model.lifter_added_box")
        comps.append((added, [cx + 0.02, 0, H + lb[2] / 2], inertia_box(added, lb)))
        top_height = H + ch
        ext_notes["front"] = ["fork_carriage"]
    else:  # cargo
        dl, dw = roles["deck_size_lw"]["value"]
        dh_ground = roles["deck_height"]["value"]
        dtop = dh_ground - clr                        # deck top in base frame; profile deck_height is above the GROUND, ground is z=-clearance
        dt, lh, lw = c.d("cargo.deck_thickness"), c.d("cargo.lip_height"), c.d("cargo.lip_width")
        deck = g.Mesh()
        deck.append(g.box_mesh([-dl / 2, -dw / 2, dtop - dt], [dl / 2, dw / 2, dtop], "deck", "deck", [((dl / 2, dw / 2, dtop), (0, 1, 0), (0, 0, 1), 0.003)]))
        for (lo, hi) in (([-dl / 2, -dw / 2, dtop], [dl / 2, -dw / 2 + lw, dtop + lh]), ([-dl / 2, dw / 2 - lw, dtop], [dl / 2, dw / 2, dtop + lh]),
                         ([-dl / 2, -dw / 2 + lw, dtop], [-dl / 2 + lw, dw / 2 - lw, dtop + lh]), ([dl / 2 - lw, -dw / 2 + lw, dtop], [dl / 2, dw / 2 - lw, dtop + lh])):
            deck.append(g.box_mesh(lo, hi, shell, "lip"))
        parts.append(Part("deck", deck, role="deck"))
        collision.append(box("deck_col", "base_link", tr(0, 0, dtop - dt / 2), (dl, dw, dt)))
        collision.append(box("deck_lip_col", "base_link", tr(0, 0, dtop + lh / 2), (dl, dw, lh)))
        fx = c.d("cargo.camera_x")
        zt = dtop + lh + 0.0
        Tcam = Tz(0, fx, 0, zt + sd[2] / 2)
        rcm = camera_mesh(c, sd)
        parts.append(Part("front_camera", rcm, T=Tcam, role="sensor"))
        sensors.append(cam_entry("cam_front", "base_link", Tcam @ tr(0.004, 0, 0), "front_camera"))
        tm_ = g.box_mesh([-tofs[0], -tofs[1] / 2, -tofs[2] / 2], [0, tofs[1] / 2, tofs[2] / 2], "sensor_body", "body")
        Ttof = tr(fx, c.d("cargo.tof_offset_y"), zt + tofs[2] / 2)
        parts.append(Part("front_tof", tm_, T=Ttof, role="sensor"))
        sensors.append(tof_entry("tof_front", "base_link", Ttof, "front_tof"))
        comps.append((added, [0, 0, dtop - dt / 2], inertia_box(added, [dl, dw, dt])))
        actuated["lift"] = None
        top_height = dtop + lh
    # lifter/cargo/scout front sensors for non-scout done above
    if role == "scout":
        actuated["lift"] = None

    Mtot, com, I = combine(comps)
    mass = {"provenance": "provisional", "source_file": SRC_REF, "mass": Mtot, "com_xyz": [round(float(v), 9) for v in com],
            "inertia_tensor": [[round(float(v), 9) for v in row] for row in I], "frame": "base_link axes, about the centre of mass",
            "derivation": [{"component": n, "mass": m_, "com_xyz": list(map(float, cm)), "inertia_about_own_com": Ic.tolist()}
                           for n, (m_, cm, Ic) in zip(["chassis_base"] + ["added_mass"], comps)],
            "optional_masses": optional,
            "note": "Base chassis from profile (mass, com, inertia_diag) plus the role's added mass as a solid box (parallel-axis combined). Provisional."}

    # docking faces manifest ---------------------------------------------------------
    face_entries = []
    for fname in ["front", "rear", "left", "right"]:
        act = fname in active
        e = {"name": fname, "active": act, "geometry": "modelled" if act else "marker_only", "origin": g.pose_of(faces[fname]),
             "hooks": hooks_by_face.get(fname, []), "catch_pins": [], "guide": {}}
        if act:
            info = pins_by_face[fname]
            e["pocket"] = {"half_width": dk["pw"], "half_height": dk["ph"], "depth": dk["pd"], "back_wall_x": -dk["pd"]}
            e["catch_pins"] = [{"name": f"{fname}_{p['name']}", "position": p["position"], "radius": p["radius"]} for p in info["pins"]]
            e["guide"] = {"type": "funnel", "axis": "face +x (centreline of the pocket)", "chamfer_width": info["cf"],
                          "flare_deg": c.d("dock.guide_flare_deg"), "profile_half_width": c.p("docking.guide_half_width"),
                          "profile_acceptance_half_angle": c.p("docking.guide_acceptance_half_angle"),
                          "material_slot": "guide"}
            e["tag"] = f"tag_{fname}"
            if role == "lifter" and fname == "front":
                e["obstructed_by"] = ["fork_carriage"]
                e["note"] = "blocked unless the lift is raised to actuated.lift.docking_value"
        else:
            e["note"] = "available face: frame only; no pocket/hooks modelled (placed at y=+-0.20, outside body and wheel envelope)"
        face_entries.append(e)

    # render section ------------------------------------------------------------------
    used_mats = []
    for p_ in parts:
        for mname in p_.mesh.mats:
            if mname not in used_mats:
                used_mats.append(mname)
    mats_out = {}
    for mname in used_mats:
        if mname in tag_ids:
            mats_out[mname] = {"base_color": [1, 1, 1], "specular": 0.0, "texture": f"../textures/tag36h11_{tag_ids[mname]:05d}.png",
                               "note": f"tag36h11 id {tag_ids[mname]}"}
        elif mname in materials:
            mats_out[mname] = materials[mname]
        else:
            mats_out[mname] = {"base_color": [0.6, 0.6, 0.6]}
    manifest = {
        "schema": "crane.fleet_asset_manifest/v0", "asset_id": f"{role}_provisional_v0", "role": role,
        "created": "generated", "description": prof["roles"][role]["description"],
        "profile": {"profile_id": prof["profile_id"], "path": rel_to_repo(c.profile_path), "sha256": sha(c.profile_path),
                    "version_note": "bump profile_id when replacing provisional values with CAD values"},
        "units": {"length": "m", "mass": "kg", "angle": "rad"},
        "conventions": CONVENTIONS,
        "base_frame": {"name": "base_link", "description": "origin at the centre of the body underside (footprint centre, z = body bottom); ground plane is z = -ground_clearance",
                       "ground_clearance": clr, "ground_z": -clr},
        "render": {"provenance": "provisional", "source_file": SRC_REF, "note": "generated placeholder shapes", "mtl_file": f"{role}.mtl",
                   "materials": mats_out, "parts": [p_.entry() for p_ in parts]},
        "collision": {"provenance": "provisional", "source_file": SRC_REF, "note": "primitives; cylinder axis = shape-frame z", "shapes": collision},
        "mass_properties": mass,
        "wheels": {"provenance": "provisional", "source_file": SRC_REF, "convention": WHEEL_CONVENTION, "items": wheel_items,
                   "note": "ground_roller_axis_xy = (sin(roller_angle), s*cos(roller_angle)) is the roller axis at the contact patch, oriented forward"},
        "sensors": {"provenance": "provisional", "source_file": SRC_REF, "frames": sensors},
        "attachments": {"provenance": "provisional", "source_file": SRC_REF, "mating_convention": MATING_CONVENTION,
                        "docking_faces": face_entries, "hook_pivots": pivots, "tags": tags},
        "actuated": dict(provenance="provisional", source_file=SRC_REF, **actuated),
        "generator_defaults": {"file": rel_to_repo(c.defaults_path), "sha256": sha(c.defaults_path), "keys_used": sorted(c.used)},
    }
    return manifest, parts, materials_for_mtl(mats_out), tag_ids


WHEEL_CONVENTION = {
    "name": "mecanum_X",
    "frames": "wheel frame: origin at the axle centre, axes parallel to base_link; spin axis +y for ALL wheels; positive spin rolls the rover forward (+x)",
    "ground_roller_axis": "a_i = (sin(alpha), s_i*cos(alpha)) in base xy (alpha = angle between roller axis and wheel axle, 45 deg), oriented forward",
    "sign_rule": "s_i = -sign(x_i * y_i): FL -1, FR +1, RL +1, RR -1 (FL and RR parallel, FR and RL parallel)",
    "kinematics": "w_i = (vx + k_i*vy + (lx+ly)*kz_i*wz)/r with k = (-1,+1,+1,-1) and kz = (-1,+1,-1,+1) for (FL,FR,RL,RR); equivalently vx=r/4*sum(w), vy=r/4*(-wFL+wFR+wRL-wRR), wz=r/(4(lx+ly))*(-wFL+wFR-wRL+wRR)",
    "verify": "(1) all four wheels forward -> +x; (2) wFL=wRR=-w, wFR=wRL=+w -> pure strafe LEFT (+y); (3) the roller touching the ground in each wheel mesh, viewed from above, has its axis along a_i: FL and RR drawn '/' (rising to the right... forward-right), FR and RL '\\\\' when forward is up on the page; the TOP-visible rollers show the mirror image (the 'X' seen from above). validate_manifest.py recomputes (3) from the OBJ roller groups."
}
MATING_CONVENTION = ("Face frame: origin on the mating plane at the face centre, +x = outward normal (pointing away from the rover), +z up, +y = z cross x. "
                     "Two faces are mated when T_world_B*F_B = T_world_A*F_A*Rz(pi): origins coincide, outward normals are opposite, z stays up, y flips. "
                     "Faces are hermaphroditic: each has two hooks (A at face y>0 on the upper level +dz, B at y<0 on the lower level -dz) and two catch pins "
                     "(pin_hi at y=-yc upper level, pin_lo at y=+yc lower level) so that A's hook A closes on the opposing face's pin_hi and so on. "
                     "A hook's engage_point (hook frame, closed) must coincide with the opposing catch pin position when mated.")
CONVENTIONS = {
    "handedness": "right", "axes": "x_forward_y_left_z_up",
    "pose": "xyz [m]; rpy [rad] extrinsic roll-pitch-yaw, R = Rz(yaw) Ry(pitch) Rx(roll); poses give child frame in parent frame; OBJ vertices are in the PART frame",
    "obj": "OBJ vertices are metres in the part frame, right-handed z-up, triangle winding counter-clockwise seen from outside; v/vt/vn present; vt origin bottom-left",
    "unity": "Unity (left-handed, y up, z forward): (x_u,y_u,z_u) = (-y, z, x); reverse triangle winding (a,b,c)->(a,c,b); yaw psi (left turn) -> Unity euler y = -psi; quaternion (x,y,z,w) -> (y,-z,-x,w). Worked example: hook pivot (0.21,0.06,0.06) -> (-0.06,0.06,0.21).",
    "camera_optical": "optical frame x right, y down, z forward (OpenCV/ROS optical); link frame x forward, y left, z up",
    "tag_frame": "AprilTag3 detector convention: x right, y down (as seen by a viewer), z INTO the tag; outward_normal gives the viewer-side normal",
}


def materials_for_mtl(mats_out):
    return {k: {kk: vv for kk, vv in v.items() if kk != "note"} for k, v in mats_out.items()}


# ------------------------------------------------------------------ payloads
def build_payloads(c):
    pl = c.prof["payloads"]
    chamfer = c.d("payload.chamfer")
    foot_sz = c.d("payload.foot_size")
    inset = c.d("payload.foot_inset")
    fg = c.d("payload.friction_group")
    parts, items = [], []
    foot_h = 0.003
    materials = {"payload_box": dict(base_color=[0.72, 0.55, 0.32]), "payload_tray": dict(base_color=[0.35, 0.50, 0.70]),
                 "payload_long": dict(base_color=[0.78, 0.72, 0.30]), "foot": MATERIALS["foot"]}
    matname = {"box_small": "payload_box", "box_long": "payload_long", "tray_wide": "payload_tray"}
    for name, d in pl.items():
        l, w, h = pv(d["size_lwh"])
        mass = pv(d["mass"])
        ce = chamfer
        body = g.box_mesh([-l / 2, -w / 2, foot_h], [l / 2, w / 2, h], matname[name], "body",
                          [((l / 2, 0, h), (1, 0, 0), (0, 0, 1), ce), ((-l / 2, 0, h), (-1, 0, 0), (0, 0, 1), ce),
                           ((0, w / 2, h), (0, 1, 0), (0, 0, 1), ce), ((0, -w / 2, h), (0, -1, 0), (0, 0, 1), ce),
                           ((l / 2, w / 2, 0), (1, 0, 0), (0, 1, 0), ce), ((l / 2, -w / 2, 0), (1, 0, 0), (0, -1, 0), ce),
                           ((-l / 2, w / 2, 0), (-1, 0, 0), (0, 1, 0), ce), ((-l / 2, -w / 2, 0), (-1, 0, 0), (0, -1, 0), ce)])
        patches = []
        if name == "box_small":
            f = c.d("payload.base_patch_frac")
            fl_, fw_ = l * f, w * f
            body.append(g.box_mesh([-fl_ / 2, -fw_ / 2, 0], [fl_ / 2, fw_ / 2, foot_h + 0.001], "foot", "feet"))
            patches.append({"name": "base", "center": [0, 0, 0], "size": [fl_, fw_], "friction_group": fg})
        else:
            xs = [-1, 1]
            ys = [-1, 1] if name == "tray_wide" else [0]
            for i, sx in enumerate(xs):
                for j, sy in enumerate(ys):
                    cx = sx * (l / 2 - inset)
                    cy = sy * (w / 2 - inset) if sy else 0.0
                    body.append(g.box_mesh([cx - foot_sz[0] / 2, cy - foot_sz[1] / 2, 0], [cx + foot_sz[0] / 2, cy + foot_sz[1] / 2, foot_h + 0.001], "foot", "feet"))
                    patches.append({"name": f"foot_{i}{j}", "center": [cx, cy, 0], "size": list(foot_sz), "friction_group": fg})
        parts.append(Part(f"payload_{name}", body, parent="payload_frame", role="payload"))
        I = inertia_box(mass, [l, w, h])
        items.append({"name": name, "frame": "payload_frame: origin at the centre of the bottom face; x along length, y along width, z up",
                      "size_lwh": [l, w, h], "mass": mass, "com_xyz": [0, 0, h / 2], "inertia_tensor": I.tolist(),
                      "render_part": f"payload_{name}",
                      "collision": {"type": "box", "size": [l, w, h], "origin": g.pose_of(tr(0, 0, h / 2))},
                      "contact_patches": patches,
                      "grasp_faces": [{"name": "side_pos_y", "normal": [0, 1, 0], "center": [0, w / 2, h / 2], "size": [l, h]},
                                      {"name": "side_neg_y", "normal": [0, -1, 0], "center": [0, -w / 2, h / 2], "size": [l, h]}],
                      "needs": pv(d["needs"]) if not isinstance(d["needs"], dict) else d["needs"]})
    mats_out = {k: v for k, v in materials.items()}
    manifest = {
        "schema": "crane.fleet_asset_manifest/v0", "asset_id": "payloads_provisional_v0", "role": "payloads", "created": "generated",
        "description": "payload types carried by cargo decks",
        "profile": {"profile_id": c.prof["profile_id"], "path": rel_to_repo(c.profile_path), "sha256": sha(c.profile_path)},
        "units": {"length": "m", "mass": "kg", "angle": "rad"}, "conventions": CONVENTIONS,
        "base_frame": {"name": "payload_frame", "description": "origin at the centre of the bottom face of the payload", "ground_clearance": 0.0, "ground_z": 0.0},
        "render": {"provenance": "provisional", "source_file": SRC_REF, "mtl_file": "payloads.mtl", "materials": mats_out,
                   "parts": [p_.entry() for p_ in parts]},
        "collision": {"provenance": "provisional", "source_file": SRC_REF,
                      "shapes": [{"name": f"{it['name']}_col", "type": "box", "parent": "payload_frame", "origin": it["collision"]["origin"], "size": it["size_lwh"]} for it in items]},
        "mass_properties": {"provenance": "provisional", "source_file": SRC_REF, "mass": items[0]["mass"], "com_xyz": items[0]["com_xyz"],
                            "inertia_tensor": items[0]["inertia_tensor"], "note": "first payload only; per-payload values are in payloads.items"},
        "payloads": {"provenance": "provisional", "source_file": SRC_REF, "items": items},
        "generator_defaults": {"file": rel_to_repo(c.defaults_path), "sha256": sha(c.defaults_path), "keys_used": sorted(c.used)},
    }
    return manifest, parts, mats_out


# ------------------------------------------------------------------ output
def write_textures(out, tag_ids_all):
    tdir = os.path.join(out, "textures")
    os.makedirs(tdir, exist_ok=True)
    for tid in sorted(tag_ids_all):
        apriltag36h11.write_png(os.path.join(tdir, f"tag36h11_{tid:05d}.png"), apriltag36h11.tag_image(tid, 32))


def emit(out, role, manifest, parts, mtl_mats, crease):
    d = os.path.join(out, role)
    os.makedirs(d, exist_ok=True)
    g.write_mtl(os.path.join(d, f"{role}.mtl"), mtl_mats)
    for p_ in parts:
        g.write_obj(os.path.join(d, p_.name + ".obj"), [(p_.name, p_.mesh)], f"{role}.mtl",
                    header=f"{role}/{p_.name} - PROVISIONAL generated geometry, metres, part frame, x fwd y left z up", crease_deg=crease)
    with open(os.path.join(d, f"{role}.manifest.json"), "w") as f:
        json.dump(manifest, f, indent=1)
    return os.path.join(d, f"{role}.manifest.json")


def main(argv=None):
    ap = argparse.ArgumentParser()
    ap.add_argument("--profile", default=DEFAULT_PROFILE)
    ap.add_argument("--defaults", default=os.path.join(HERE, "generator_defaults.json"))
    ap.add_argument("--out", default=DEFAULT_OUT)
    ap.add_argument("--roles", nargs="+", default=["scout", "lifter", "cargo", "payloads"])
    a = ap.parse_args(argv)
    all_tags = set()
    paths = []
    for role in a.roles:
        c = Cfg(a.profile, a.defaults)
        if role == "payloads":
            man, parts, mats = build_payloads(c)
            tags = {}
        else:
            man, parts, mats, tags = build_role(c, role)
        all_tags |= set(tags.values())
        paths.append(emit(a.out, role, man, parts, mats, c.d("tessellation.crease_angle_deg")))
        print(f"{role}: {len(parts)} parts, {sum(len(p.mesh) for p in parts)} triangles")
    # cargo_2 uses offset ids: write those textures too
    off = Cfg(a.profile, a.defaults).d("tags.id_offset_second_cargo")
    all_tags |= {t + off for t in all_tags if 30 <= t < 40}
    write_textures(a.out, all_tags)
    print("manifests:", *paths, sep="\n  ")
    return paths


if __name__ == "__main__":
    main()
