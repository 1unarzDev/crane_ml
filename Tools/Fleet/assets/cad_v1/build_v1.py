#!/usr/bin/env python3
"""Build the v1 concept-derived fleet assets: STEP per part, OBJ per part, manifests, URDFs.

    python build_v1.py [--out ~/.cache/crane-fleet/assets_v1] [--roles scout lifter cargo payloads] [--no-step]

Reads Tools/Fleet/spec/fleet_profile_v1.json + cad_v1/cad_params_v1.json. Writes the derived mass numbers
(base_mass, added_mass, arm mass) back into the v1 profile (provenance 'concept', derived from solids) before
the manifests are written, so manifest and profile agree (validator profile.* / mass.total).
v0 (gen_provisional.py, fleet_profile_v0.json, assets_v0/) is not touched.
"""
import argparse
import copy
import hashlib
import json
import math
import os
import shutil
import sys

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
import cadlib as C  # noqa: E402
from cadlib import Item, Link, T  # noqa: E402
import parts_v1 as P  # noqa: E402
import geom as g  # noqa: E402
import apriltag36h11  # noqa: E402

REPO = P.REPO
DEFAULT_OUT = os.path.expanduser("~/.cache/crane-fleet/assets_v1")
SRC_REF = "Tools/Fleet/assets/cad_v1/build_v1.py"
URDF_DIR = os.path.join(HERE, "urdf")
PKG = "crane_fleet_v1"

MATERIALS = {
    "guide": dict(base_color=[0.98, 0.45, 0.06], specular=0.35, shininess=40, note="orange C-shaped guide/contact brackets of the reciprocal dock"),
    "magnet": dict(base_color=[0.80, 0.80, 0.82], specular=0.7, shininess=90),
    "hook": dict(base_color=[0.82, 0.82, 0.84], specular=0.8, shininess=100, note="J-hook, polished steel"),
    "pin": dict(base_color=[0.78, 0.78, 0.80], specular=0.7, shininess=90),
    "top_plate": dict(base_color=[0.20, 0.21, 0.23], specular=0.25, shininess=30),
    "screw": dict(base_color=[0.62, 0.63, 0.66], specular=0.6, shininess=70),
    "accent": dict(base_color=[0.98, 0.45, 0.06], specular=0.3, shininess=40),
    "wheel_hub": dict(base_color=[0.80, 0.80, 0.82], specular=0.8, shininess=100, note="chrome side plates with 8 windows"),
    "wheel_core": dict(base_color=[0.25, 0.25, 0.27]),
    "axle_cap": dict(base_color=[0.70, 0.70, 0.72], specular=0.6, shininess=60),
    "roller": dict(base_color=[0.11, 0.11, 0.12], specular=0.15, shininess=20),
    "sensor_body": dict(base_color=[0.07, 0.07, 0.08], specular=0.2),
    "lens": dict(base_color=[0.05, 0.08, 0.10], specular=0.9, shininess=120),
    "lens_barrel": dict(base_color=[0.10, 0.10, 0.11], specular=0.3),
    "d435_shell": dict(base_color=[0.72, 0.73, 0.75], specular=0.6, shininess=70),
    "lidar": dict(base_color=[0.13, 0.13, 0.14], specular=0.3),
    "lidar_window": dict(base_color=[0.05, 0.05, 0.06], specular=0.8, shininess=100),
    "pcb_blue": dict(base_color=[0.10, 0.28, 0.62]),
    "pcb_black": dict(base_color=[0.08, 0.08, 0.08], note="Arducam board (orange silkscreen edge omitted)"),
    "port_metal": dict(base_color=[0.75, 0.76, 0.78], specular=0.6, shininess=70),
    "header": dict(base_color=[0.15, 0.15, 0.15]),
    "heatsink": dict(base_color=[0.78, 0.79, 0.81], specular=0.5, shininess=50),
    "fan": dict(base_color=[0.06, 0.06, 0.07]),
    "metal": dict(base_color=[0.66, 0.67, 0.69], specular=0.5, shininess=60),
    "arm_shell": dict(base_color=[0.84, 0.80, 0.74], specular=0.2, shininess=25, note="concept beige printed arm links"),
    "servo": dict(base_color=[0.09, 0.09, 0.10]),
    "rubber_pad": dict(base_color=[0.06, 0.06, 0.06]),
    "deck_mat": dict(base_color=[0.21, 0.21, 0.22]),
    "deck_plate": dict(base_color=[0.27, 0.28, 0.30], specular=0.3, shininess=35),
    "vent": dict(base_color=[0.05, 0.05, 0.06]),
    "internal": dict(base_color=[0.30, 0.75, 0.35], alpha=0.5, note="hidden internal components (battery, motors, MCU, IMU, drivers, servos)"),
}
SHELL = dict(base_color=[0.17, 0.18, 0.20], specular=0.2, shininess=30, note="concept dark-grey printed body (all roles)")

WHEEL_CONVENTION = {
    "name": "mecanum_X",
    "frames": "wheel frame: origin at the axle centre, axes parallel to base_link; spin axis +y for ALL wheels; positive spin rolls the rover forward (+x)",
    "ground_roller_axis": "a_i = (sin(alpha), s_i*cos(alpha)) in base xy (alpha = angle between roller axis and wheel axle, 45 deg), oriented forward",
    "sign_rule": "s_i = -sign(x_i * y_i): FL -1, FR +1, RL +1, RR -1 (FL and RR parallel, FR and RL parallel)",
    "kinematics": "w_i = (vx + k_i*vy + (lx+ly)*kz_i*wz)/r with k = (-1,+1,+1,-1) and kz = (-1,+1,-1,+1) for (FL,FR,RL,RR)",
    "verify": "top-visible rollers form an X (FL '\\\\', FR '/', RL '/', RR '\\\\' with forward up); validate_manifest.py recomputes the contact-roller lean from the OBJ groups",
}
MATING_CONVENTION = ("Face frame: origin on the mating plane at the face centre, +x = outward normal, +z up, +y = z cross x. "
                     "Two faces are mated when T_world_B*F_B = T_world_A*F_A*Rz(pi). v1 faces are RECIPROCAL: one J-hook at face -y (pivot axis +y, "
                     "closes downwards over the partner's horizontal pin) and one horizontal catch pin at face +y; Rz(pi) maps -y onto +y, so each "
                     "hook lands on the partner's pin. A mated connection therefore has two hooks, one per rover. Magnets at y=+-0.041 (opposite "
                     "poles), orange C-brackets and the two steel pads are the contact datum on the mating plane.")
CONVENTIONS = {
    "handedness": "right", "axes": "x_forward_y_left_z_up",
    "pose": "xyz [m]; rpy [rad] extrinsic roll-pitch-yaw, R = Rz(yaw) Ry(pitch) Rx(roll); poses give child frame in parent frame; OBJ vertices are in the PART frame",
    "obj": "OBJ vertices are metres in the part frame, right-handed z-up, triangle winding counter-clockwise seen from outside; v/vt/vn present; vt origin bottom-left",
    "unity": "Unity (left-handed, y up, z forward): (x_u,y_u,z_u) = (-y, z, x); reverse triangle winding (a,b,c)->(a,c,b); yaw psi (left turn) -> Unity euler y = -psi; quaternion (x,y,z,w) -> (y,-z,-x,w).",
    "camera_optical": "optical frame x right, y down, z forward (OpenCV/ROS optical); link frame x forward, y left, z up",
    "tag_frame": "AprilTag3 detector convention: x right, y down (as seen by a viewer), z INTO the tag; outward_normal gives the viewer-side normal",
}
R_OPT = np.array([[0, 0, 1], [-1, 0, 0], [0, -1, 0]], float)


def sha(path):
    return hashlib.sha256(open(path, "rb").read()).hexdigest()


def rel(path):
    return os.path.relpath(path, REPO)


# ------------------------------------------------------------------ assembly
class Role:
    def __init__(self, c, role):
        self.c, self.role = c, role
        builder = {"scout": P.scout_links, "lifter": P.lifter_links, "cargo": P.cargo_links}[role]
        links, info = builder(c)
        self.info = info
        self.faces = info["faces"]
        self.hook_faces = [f for f, k in self.faces.items() if k == "dock"]
        for name, (sx, sy) in {"FL": (1, 1), "FR": (1, -1), "RL": (-1, 1), "RR": (-1, -1)}.items():
            links.append(P.wheel_link(c, name, sx, sy))
        for f in self.hook_faces:
            links.append(P.hook_link(c, f))
        extra = []
        if role == "scout":
            ax, ay = c.d("scout.arm_base_xy")
            H = c.p("chassis.body_size_lwh")[2]
            extra.append(Item(C.box([ax - 0.0175, ay - 0.0124, H - 0.050], [ax + 0.0175, ay + 0.0124, H - 0.005]), "internal", "servo", mass=c.d("masses.sts3215"), label="arm_yaw_servo"))
        if role == "lifter":
            extra.append(Item(C.box([0.155, -0.012, 0.030], [0.170, 0.012, 0.050]), "internal", "servo", mass=c.d("masses.jaw_servo"), label="jaw_servo_drive"))
        links.append(P.internals_link(c, role, self.hook_faces, extra))
        links.append(P.dock_tag_link(c, role, self.hook_faces))
        # top tag (quad)
        ids = {"scout": 10, "lifter": 20, "cargo": 30}[role]
        self.tag_id = ids + 4
        size = c.p(f"roles.{role}.tag_sizes.tag_top")
        tx, ty = info["tag_xy"]
        tz = info.get("tag_z", c.p("chassis.body_size_lwh")[2] + 0.0006)
        self.tag_T = T(tx, ty, tz, R=P.R_TAG_TOP)
        q, plate, mat = P.tag_quad(size, self.tag_id, self.tag_T)
        tl = Link("tag_top", role="tag", kind="quad")
        tl.quads.append(q)
        links.append(tl)
        self.tag = dict(size=size, plate=plate, mat=mat)
        self.face_tags = {}
        fsz = c.p("sensors.apriltag.face_tag_size")
        for f in self.hook_faces:
            tid = ids + P.FACE_ORDER.index(f)
            Tt = P.tag_wedge_geom(c, role, f)[1]
            qf, pl_, mt = P.tag_quad(fsz, tid, Tt)
            lf = Link(f"tag_{f}", role="tag", kind="quad")
            lf.quads.append(qf)
            links.append(lf)
            self.face_tags[f] = dict(id=tid, T=Tt, size=fsz, plate=pl_, mat=mt)
        self.links = {l.name: l for l in links}
        self.order = [l.name for l in links]

    # kinematics -------------------------------------------------------
    def world_T(self, name, state=None):
        state = state or {}
        if name == "base_link":
            return np.eye(4)
        l = self.links[name]
        parent = self.world_T(l.parent, state) if l.parent != "base_link" else np.eye(4)
        if not l.joint:
            return parent
        q = state.get(name, l.joint.get("authored", 0.0))
        return parent @ l.joint["origin"] @ C.joint_motion(l.joint, q)

    def mass_table(self, state=None):
        rows, comps = [], []
        for n in self.order:
            l = self.links[n]
            m, com, I, items = l.mass_props()
            if m <= 0:
                continue
            Tw = self.world_T(n, state)
            mw, cw, Iw = C.transform_inertia(m, com, I, Tw)
            comps.append((mw, cw, Iw))
            rows.append({"component": n, "mass": float(m), "com_xyz": [float(v) for v in cw], "inertia_about_own_com": Iw.tolist(),
                         "items": [{"item": r["item"], "material": r["material"], "rule": r["rule"], "mass": round(r["mass"], 6)} for r in items]})
        M, com, I = C.combine(comps)
        return M, com, I, rows


# ------------------------------------------------------------------ profile update
def base_chassis_mass(roles):
    """Shared chassis = cargo's body (4 docks) + wheels + 4 hooks + internals (incl. 4 hook servos)."""
    r = roles["cargo"]
    comps = []
    for n in r.order:
        if n in ("chassis_body", "internals") or n.startswith("wheel_") or n.startswith("hook_"):
            l = r.links[n]
            m, com, I, _ = l.mass_props()
            comps.append(C.transform_inertia(m, com, I, r.world_T(n)))
    return C.combine(comps)


def update_profile(c, roles, path):
    prof = json.load(open(path))
    Mb, comb, Ib = base_chassis_mass(roles)
    src = "Tools/Fleet/assets/cad_v1/build_v1.py (solids x densities + datasheet COTS masses)"
    prof["chassis"]["base_mass"] = {"value": round(float(Mb), 4), "provenance": "concept", "source": src,
                                    "estimate": "shared chassis: printed PETG shell, aluminium top plate, 4 docks, wheels, 4 hooks, battery, motors, MCU, IMU, drivers; +-20 % (materials and wall thickness not chosen)"}
    prof["chassis"]["com_offset_xyz"] = {"value": [round(float(v), 4) for v in comb], "provenance": "concept", "source": src}
    prof["chassis"]["inertia_diag"] = {"value": [round(float(v), 5) for v in np.diag(Ib)], "provenance": "concept", "source": src,
                                       "note": "about the chassis COM; full tensors in the manifests"}
    for name, r in roles.items():
        M, com, I, _ = r.mass_table()
        prof["roles"][name]["added_mass"] = {"value": round(float(M - Mb), 4), "provenance": "concept", "source": src,
                                             "estimate": f"role total {M:.3f} kg minus base chassis {Mb:.3f} kg"}
        if name == "scout":
            am = sum(r.links[n].mass_props()[0] for n in r.order if n.startswith("arm_"))
            prof["roles"]["scout"]["arm_proxy"]["mass"] = {"value": round(float(am), 4), "provenance": "concept", "source": src,
                                                          "estimate": "moving arm links: 5 x STS3215 (55 g datasheet) + printed links at 35 % effective density; the 6th (yaw) servo sits inside the body (internals, +0.055 kg)"}
    json.dump(prof, open(path, "w"), indent=1)
    c.prof = prof
    return Mb


# ------------------------------------------------------------------ manifest
def pose(Tm):
    return g.pose_of(Tm)


def materials_for(role, used, tag_mat, tag_id):
    out = {}
    for m in used:
        if m.startswith("tag36h11_"):
            tid = int(m.split("_")[1])
            out[m] = {"base_color": [1, 1, 1], "specular": 0.0, "texture": f"../textures/tag36h11_{tid:05d}.png", "note": f"tag36h11 id {tid}"}
        elif m.startswith("shell_"):
            out[m] = dict(SHELL)
        else:
            out[m] = dict(MATERIALS.get(m, {"base_color": [0.6, 0.6, 0.6]}))
    return out


def collision_shapes(r):
    c, role = r.c, r.role
    L, W, H = c.p("chassis.body_size_lwh")
    rw, ww = c.p("chassis.wheel.radius"), c.p("chassis.wheel.width")
    boxs = lambda n, parent, Tm, size: {"name": n, "type": "box", "parent": parent, "origin": pose(Tm), "size": [float(v) for v in size]}
    cyls = lambda n, parent, Tm, rad, h: {"name": n, "type": "cylinder", "parent": parent, "origin": pose(Tm), "radius": float(rad), "height": float(h)}
    out = [boxs("chassis_col", "base_link", T(0, 0, H / 2), (L - 0.0, W, H))]
    for w in ("FL", "FR", "RL", "RR"):
        out.append(cyls(f"wheel_{w}_col", f"wheel_{w}", T(R=g.rotx(-math.pi / 2)), rw, ww))
    for f in r.hook_faces:
        hl = r.links[f"hook_{f}_a"]
        out.append(boxs(f"hook_{f}_a_col", hl.name, T(0.017, 0, -0.003), (0.042, 0.008, 0.016)))
    def bb_box(name, link, parent="base_link", pad=0.0):
        mesh = C.link_mesh(r.links[link], 2e-3, 0.8)
        lo, hi = mesh.bbox()
        return boxs(name, parent, T(*((lo + hi) / 2)), (hi - lo) + 2 * pad)
    if role == "scout":
        out += [bb_box("d435_col", "d435"), bb_box("lidar_mount_col", "mast")]
        lx_, ly_ = c.d("scout.lidar_xy")
        mh = c.p("roles.scout.mast_height")
        lh = c.p("sensors.lidar_2d.size")[2]
        out.append(cyls("lidar_col", "base_link", T(lx_, ly_, H + mh + lh / 2), c.p("sensors.lidar_2d.size")[0] / 2, lh))
        out.append(bb_box("compute_col", "compute"))
        out.append(bb_box("arm_base_col", "arm_base"))
        for n in ("arm_link_1", "arm_link_2", "arm_link_3", "arm_link_4", "arm_link_5", "arm_gripper_jaw"):
            out.append(bb_box(f"{n}_col", n, parent=n))
    elif role == "lifter":
        out += [bb_box("lift_column_col", "lift_column"), bb_box("camera_mast_col", "camera_mast"), bb_box("front_camera_col", "front_camera"),
                bb_box("compute_col", "compute")]
        for n in ("fork_carriage", "finger_left", "finger_right"):
            out.append(bb_box(f"{n}_col", n, parent=n))
    else:
        dl, dw = c.p("roles.cargo.deck_size_lw")
        dtop = c.p("roles.cargo.deck_height") - c.p("chassis.ground_clearance")
        dt = c.d("cargo.deck_thickness")
        out.append(boxs("deck_col", "base_link", T(0, 0, dtop - dt / 2), (dl, dw, dt)))
        sx_, sy_ = c.d("cargo.standoff_xy")
        out.append(bb_box("electronics_box_col", "electronics_box"))
        out.append(bb_box("tag_flag_col", "tag_flag_mount"))
        out.append(bb_box("front_camera_col", "front_camera"))
    return out


def link_state_local(r, name):
    """Link pose relative to its parent at the authored state (manifest 'origin')."""
    l = r.links[name]
    return l.origin_authored()


def sensor_entries(r):
    c, role = r.c, r.role
    prof = c.prof
    out = []
    imu_p = prof["sensors"]["imu"]
    out.append({"name": "imu", "type": "imu", "parent": "base_link", "origin": pose(T(*c.d("scout.imu_xyz"))),
                "params": {"rate_hz": C.pv(imu_p["rate_hz"]), "gyro_noise_density": C.pv(imu_p["gyro_noise_density"]),
                           "gyro_bias_instability": C.pv(imu_p["gyro_bias_instability"]), "model": imu_p.get("model"), "urdf_link": "imu_link"}})
    def cam(name, Tm, key, urdf_link, urdf_opt, render_part):
        sp = prof["sensors"][key]
        e = {"name": name, "type": "camera", "parent": "base_link", "origin": pose(Tm), "optical_origin": pose(Tm @ g.make_T(R_OPT)),
             "optical_axis_convention": "optical frame: x right, y down, z = optical axis (forward); link frame: x = optical axis, y left, z up",
             "params": {"profile_sensor": key, "resolution": C.pv(sp["resolution"]), "hfov": C.pv(sp["hfov"]), "rate_hz": C.pv(sp["rate_hz"]),
                        "latency": C.pv(sp["latency"]), "shutter": sp.get("shutter"), "encoding": sp.get("encoding"),
                        "urdf_link": urdf_link, "urdf_optical_frame": urdf_opt},
             "render_part": render_part}
        return e
    if role == "scout":
        d = r.links["d435"]
        out.append(cam("camera_depth", d.frames["infra1"], "realsense_d435_depth", "camera_depth_frame", "camera_depth_optical_frame", "d435"))
        out.append(cam("camera_infra1", d.frames["infra1"], "realsense_d435_infra", "camera_infra1_frame", "camera_infra1_optical_frame", "d435"))
        out.append(cam("camera_infra2", d.frames["infra2"], "realsense_d435_infra", "camera_infra2_frame", "camera_infra2_optical_frame", "d435"))
        out.append(cam("camera_color", d.frames["color"], "realsense_d435_color", "camera_color_frame", "camera_color_optical_frame", "d435"))
        lp = prof["sensors"]["lidar_2d"]
        out.append({"name": "lidar", "type": "lidar2d", "parent": "base_link", "origin": pose(r.info["lidar_frame"]),
                    "params": {"model": lp.get("model"), "max_range": C.pv(lp["max_range"]), "rate_hz": C.pv(lp["rate_hz"]), "points_per_scan": C.pv(lp["points_per_scan"]),
                               "fov": 2 * math.pi, "scan_plane": "xy of this frame, +z up, angle counter-clockwise from +x", "urdf_link": "laser",
                               "scan_height_note": "assumed at the middle of the C1 window band; not in the datasheet"},
                    "render_part": "lidar_head"})
    else:
        f = r.links["front_camera"].frames["cam_front"]
        out.append(cam("cam_front", f, "mono_camera_ov9281", "front_camera_link", "front_camera_optical_frame", "front_camera"))
    return out


def face_entries(r):
    c, role = r.c, r.role
    out = []
    yc, wy, hh = c.d("dock.pocket_y_center"), c.d("dock.pocket_width"), c.d("dock.pocket_half_height")
    for f in P.FACE_ORDER:
        k = r.faces.get(f, "none")
        e = {"name": f, "active": k == "dock", "geometry": "modelled" if k == "dock" else "marker_only", "origin": pose(P.face_T(c, f)),
             "hooks": [f"hook_{f}_a"] if k == "dock" else [], "catch_pins": [], "guide": {}}
        guide = {"type": "reciprocal: C-brackets + pocket capture + magnets", "axis": "face +x", "material_slot": "guide",
                 "chamfer_width": 0.0, "capture_half_width": c.p("docking.guide_half_width"),
                 "profile_half_width": c.p("docking.guide_half_width"), "profile_acceptance_half_angle": c.p("docking.guide_acceptance_half_angle"),
                 "brackets": {"y_inner": c.d("dock.bracket_y_inner"), "y_outer": c.d("dock.bracket_y_outer"), "half_height": c.d("dock.bracket_half_height"),
                              "proud": c.d("chassis.bracket_proud"), "role": "contact datum on the mating plane (identical brackets butt against each other)"},
                 "magnets": [{"position": [-c.d("dock.magnet_recess"), s * c.d("dock.magnet_y"), c.d("dock.magnet_z")], "radius": c.d("dock.magnet_radius"),
                              "pole_out": "N" if s > 0 else "S"} for s in ((-1, 1) if k == "dock" else ((1,) if k == "brackets" else ()))],
                 "contact_pads": [{"position": [0.0, 0.0, s * c.d("dock.pad_z")], "size_yz": c.d("dock.pad_size_yz")} for s in (-1, 1)] if k == "dock" else [],
                 "note": "No funnel: lateral capture = (pocket width - hook rod)/2 = 10 mm, vertical +-(pocket half-height - hook) ; chamfer_width 0 keeps the validator's guide_width WARN honest."}
        if k == "dock":
            e["pocket"] = {"hook_pocket": {"y": -yc, "width": wy, "half_height": hh, "depth": c.d("dock.pocket_depth")},
                           "pin_pocket": {"y": yc, "width": wy, "half_height": hh, "depth": c.d("dock.pocket_depth")},
                           "back_wall_x": -c.d("chassis.bracket_proud") - c.d("dock.pocket_depth")}
            e["catch_pins"] = [{"name": f"{f}_pin", "position": [round(v, 9) for v in P.pin_position_face(c)], "radius": c.d("dock.pin_radius"),
                                "axis": [0, 1, 0]}]
            e["guide"] = guide
            e["tag"] = f"tag_{f}"
        else:
            e["guide"] = guide if k == "brackets" else {}
            if k == "brackets":
                e["note"] = "lifter gripper face (concept): guide brackets + one magnet only, no hook/pin pocket; not a docking face"
                e["obstructed_by"] = ["lift_column", "fork_carriage", "finger_left", "finger_right"]
            else:
                e["note"] = "inactive face"
        out.append(e)
    return out


def build_manifest(r, mesh_info, profile_path, used_keys):
    c, role = r.c, r.role
    prof = c.prof
    L, W, H = c.p("chassis.body_size_lwh")
    clr = c.p("chassis.ground_clearance")
    parts = []
    for n in r.order:
        l = r.links[n]
        if n not in mesh_info:
            continue
        e = {"name": n, "file": n + ".obj", "parent": l.parent, "origin": pose(link_state_local(r, n)), "materials": mesh_info[n],
             "kind": l.kind, "default_visible": l.visible, "role_in_assembly": l.role}
        if l.joint:
            j = {"type": l.joint["type"], "axis": list(l.joint["axis"]), "authored_value": l.joint.get("authored", 0.0)}
            if "limits" in l.joint:
                j["limits"] = list(l.joint["limits"])
            e["joint"] = j
        if l.hide_group:
            e["hideable_group"] = l.hide_group
        if l.contacts_ground:
            e["contacts_ground"] = True
        parts.append(e)
    used = []
    for e in parts:
        for m in e["materials"]:
            if m not in used:
                used.append(m)
    mats = materials_for(role, used, r.tag["mat"], r.tag_id)
    # wheels
    wheel_items = []
    for w in ("FL", "FR", "RL", "RR"):
        l = r.links[f"wheel_{w}"]
        s = l.info["sign"]
        a = c.d("wheel.roller_angle")
        Tw = l.joint["origin"]
        wheel_items.append({"name": w, "position": [round(float(v), 9) for v in Tw[:3, 3]], "frame": pose(Tw), "axis": [0, 1, 0],
                            "radius": c.p("chassis.wheel.radius"), "width": c.p("chassis.wheel.width"), "roller_count": int(c.p("chassis.wheel.roller_count")),
                            "roller_angle": a, "roller_angle_sign": s, "ground_roller_axis_xy": [round(math.sin(a), 9), round(s * math.cos(a), 9)],
                            "render_part": f"wheel_{w}", "collision_shape": f"wheel_{w}_col"})
    # mass
    M, com, I, rows = r.mass_table()
    mass = {"provenance": "provisional", "source_file": SRC_REF, "mass": round(float(M), 6), "com_xyz": [round(float(v), 6) for v in com],
            "inertia_tensor": [[round(float(v), 8) for v in row] for row in I], "frame": "base_link axes, about the centre of mass",
            "derivation": rows, "optional_masses": [],
            "note": ("Computed from the v1 solids (OCC volume properties) with material densities (cadlib.DENSITY) and datasheet COTS masses where "
                     "available; whole rover at the authored pose (hooks closed, arm folded, lift down, jaws open). Concept-derived, not measured: +-20 %.")}
    # hooks / pivots
    hooks, pivots = [], []
    for f in r.hook_faces:
        l = r.links[f"hook_{f}_a"]
        pivots.append({"name": f"pivot_{f}_a", "face": f, "parent": "base_link", "origin": pose(l.joint["origin"])})
        hooks.append({"name": l.name, "face": f, "part": l.name, "pivot": f"pivot_{f}_a", "axis": [0, 1, 0], "closed_angle": 0.0,
                      "open_angle": l.info["open_angle"], "authored_angle": 0.0, "engage_point": [round(v, 9) for v in l.info["engage"]],
                      "level_dz": round(c.d("dock.pin_radius") + c.d("dock.hook_clearance") + c.d("dock.hook_rod_radius"), 9),
                      "rated_load": C.pv(prof["docking"]["hook_rated_load"]), "actuation_time": C.pv(prof["docking"]["hook_actuation_time"])})
    tags = [{"name": "tag_top", "family": "tag36h11", "id": r.tag_id, "size": r.tag["size"], "plate_size": r.tag["plate"], "parent": "base_link",
             "origin": pose(r.tag_T), "outward_normal": [0.0, 0.0, 1.0], "render_part": "tag_top", "texture": f"../textures/tag36h11_{r.tag_id:05d}.png",
             "may_be_occluded_by_payload": role == "cargo", "instance_id_offset_per_unit": 10 if role == "cargo" else 0}]
    for f, ft in r.face_tags.items():
        nrm = (ft["T"][:3, :3] @ np.array([0, 0, -1.0])).tolist()
        tags.append({"name": f"tag_{f}", "family": "tag36h11", "id": ft["id"], "size": ft["size"], "plate_size": ft["plate"], "parent": "base_link",
                     "origin": pose(ft["T"]), "outward_normal": [round(v, 9) for v in nrm], "render_part": f"tag_{f}",
                     "texture": f"../textures/tag36h11_{ft['id']:05d}.png", "instance_id_offset_per_unit": 10 if role == "cargo" else 0})
    actuated = {"provenance": "provisional", "source_file": SRC_REF, "hooks": hooks, "lift": None, "gripper": None, "lidar_spin": None, "arm_proxy": None,
                "note": "Hook q: 0 = closed (crook down behind the partner pin), open_angle -0.30 rad lifts the tip (rotation about the face +y axis)."}
    if role == "lifter":
        trv = c.p("roles.lifter.lift_travel")
        actuated["lift"] = {"part": "fork_carriage", "axis": [0, 0, 1], "range": [0.0, trv], "authored_value": 0.0, "docking_value": 0.0,
                            "speed": c.p("roles.lifter.lift_speed"), "capacity": c.p("roles.lifter.lift_capacity"),
                            "note": "lift=0: jaw prongs 3 mm above the ground. The front face is not a dock in v1, so docking_value is irrelevant (0)."}
        omax, omin = c.p("roles.lifter.gripper_opening"), c.d("lifter.gripper_min_opening")
        actuated["gripper"] = {"fingers": [{"part": "finger_left", "axis": [0, -1, 0], "travel": [0.0, (omax - omin) / 2]},
                                           {"part": "finger_right", "axis": [0, 1, 0], "travel": [0.0, (omax - omin) / 2]}],
                               "opening_range": [omin, omax], "authored_opening": omax, "close_time": c.p("roles.lifter.gripper_close_time"),
                               "note": "opening = clear gap between the jaw pads; each jaw moves (omax-opening)/2"}
    if role == "scout":
        actuated["lidar_spin"] = {"part": "lidar_head", "axis": [0, 0, 1], "rate_hz": C.pv(prof["sensors"]["lidar_2d"]["rate_hz"])}
        ap = prof["roles"]["scout"]["arm_proxy"]
        joints = []
        for n in ("arm_link_1", "arm_link_2", "arm_link_3", "arm_link_4", "arm_link_5", "arm_gripper_jaw"):
            j = r.links[n].joint
            joints.append({"part": n, "joint": {"arm_link_1": "arm_joint_1", "arm_link_2": "arm_joint_2", "arm_link_3": "arm_joint_3", "arm_link_4": "arm_joint_4",
                                                 "arm_link_5": "arm_joint_5", "arm_gripper_jaw": "arm_gripper_joint"}[n],
                           "type": j["type"], "axis": j["axis"], "limits": j["limits"], "folded": j["authored"]})
        actuated["arm_proxy"] = {"part": "arm_link_1", "mass": C.pv(ap["mass"]), "reach": C.pv(ap["reach"]), "payload": C.pv(ap["payload"]),
                                 "default_visible": True, "joints": joints, "concept_pose": c.d("scout.concept_pose"), "folded_pose": c.d("scout.folded_pose"),
                                 "modelled_as": ap["modelled_as"], "proxy": True}
    man = {
        "schema": "crane.fleet_asset_manifest/v0", "asset_id": f"{role}_concept_v1", "role": role, "created": "generated",
        "description": prof["roles"][role]["description"],
        "profile": {"profile_id": prof["profile_id"], "path": rel(profile_path), "sha256": sha(profile_path),
                    "version_note": "v1 concept-derived design; v0 (mecanum_fleet_provisional_v0) unchanged"},
        "units": {"length": "m", "mass": "kg", "angle": "rad"}, "conventions": CONVENTIONS,
        "base_frame": {"name": "base_link", "description": "origin at the centre of the body underside (footprint centre, z = body bottom); ground plane is z = -ground_clearance",
                       "ground_clearance": clr, "ground_z": -clr},
        "render": {"provenance": "provisional", "source_file": SRC_REF, "mtl_file": f"{role}.mtl", "materials": mats, "parts": parts,
                   "note": "Concept-derived parametric CAD (build123d/OpenCASCADE), tessellated. Section provenance stays 'provisional' (schema enum) because the "
                           "geometry is estimated from concept renders, not from a released CAD model; per-value provenance (concept/datasheet) is in fleet_profile_v1.json. STEP per part in step/."},
        "collision": {"provenance": "provisional", "source_file": SRC_REF, "note": "primitives; cylinder axis = shape-frame z; wheel cylinders use the true 45 mm width", "shapes": collision_shapes(r)},
        "mass_properties": mass,
        "wheels": {"provenance": "provisional", "source_file": SRC_REF, "convention": WHEEL_CONVENTION, "items": wheel_items,
                   "note": "DFRobot FIT0768-class 97 mm wheel, 9 rollers at 45 deg; ground_roller_axis_xy = (sin a, s cos a)"},
        "sensors": {"provenance": "provisional", "source_file": SRC_REF, "frames": sensor_entries(r)},
        "attachments": {"provenance": "provisional", "source_file": SRC_REF, "mating_convention": MATING_CONVENTION,
                        "docking_faces": face_entries(r), "hook_pivots": pivots, "tags": tags,
                        "note": "Top tag per rover (cargo: corner flag) + one 30 mm dock-acquisition tag per dock face on a 45 deg bevel facet above the dock (user decision 2026-11-10)."},
        "actuated": actuated,
        "generator_defaults": {"file": rel(c.params_path), "sha256": sha(c.params_path), "keys_used": sorted(used_keys)},
    }
    return man


# ------------------------------------------------------------------ URDF
def urdf_for(r, mesh_dir_rel, mesh_files, profile_path):
    c, role = r.c, r.role
    clr = c.p("chassis.ground_clearance")
    fmt = lambda v: " ".join("%.10g" % (0.0 if abs(x) < 1e-12 else x) for x in v)
    def origin(Tm):
        p = g.pose_of(Tm)
        return f'<origin xyz="{fmt(p["xyz"])}" rpy="{fmt(p["rpy"])}"/>'
    def inertial(m, com, I):
        I = (I + I.T) / 2
        return (f'<inertial><origin xyz="{fmt(com)}" rpy="0 0 0"/><mass value="{m:.6g}"/>'
                f'<inertia ixx="{I[0,0]:.6g}" ixy="{I[0,1]:.6g}" ixz="{I[0,2]:.6g}" iyy="{I[1,1]:.6g}" iyz="{I[1,2]:.6g}" izz="{I[2,2]:.6g}"/></inertial>')
    def visuals(name):
        out = []
        if not r.links[name].visible or r.links[name].kind == "quad":
            return out
        for mat, f in mesh_files.get(name, []):
            col = (MATERIALS.get(mat) or (SHELL if mat.startswith("shell_") else {"base_color": [0.6, 0.6, 0.6]}))["base_color"]
            out.append(f'<visual name="{name}__{mat}"><geometry><mesh filename="package://{PKG}/{mesh_dir_rel}/{f}"/></geometry>'
                       f'<material name="{mat}"><color rgba="{fmt(col)} 1"/></material></visual>')
        return out
    def collisions(link_name, static_parent=False):
        out = []
        for s in collision_shapes(r):
            if s["parent"] != link_name:
                continue
            Tm = g.T_from_pose(s["origin"])
            if s["type"] == "box":
                geo = f'<box size="{fmt(s["size"])}"/>'
            else:
                geo = f'<cylinder radius="{s["radius"]:.6g}" length="{s["height"]:.6g}"/>'
            out.append(f'<collision name="{s["name"]}">{origin(Tm)}<geometry>{geo}</geometry></collision>')
        return out
    static = [n for n in r.order if not r.links[n].joint and r.links[n].parent == "base_link"]
    moving = [n for n in r.order if n not in static]
    comps = []
    for n in static:
        m, com, I, _ = r.links[n].mass_props()
        if m > 0:
            comps.append((m, com, I))
    Mb, cb, Ib = C.combine(comps)
    L = []
    L.append('<?xml version="1.0"?>')
    L.append(f'<!-- crane fleet v1 concept rover: {role}. GENERATED by {SRC_REF} from {rel(profile_path)} - do not edit by hand.\n'
             '     Frames: REP-103 (x forward, y left, z up); REP-105 base_footprint -> base_link; camera optical frames x right, y down, z forward.\n'
             '     base_link origin = centre of the body underside (same as the asset manifest); ground is z = -ground_clearance.\n'
             '     MECANUM: wheels are plain continuous joints about +y with cylinder collisions of the true 45 mm width. There are no roller\n'
             '     links. Roller behaviour must come from the simulator: (a) kinematic: a mecanum drive plugin/controller with the X-pattern\n'
             '     inverse kinematics w_i = (vx + k_i vy + (lx+ly) kz_i wz)/r, k=(-1,+1,+1,-1), kz=(-1,+1,-1,+1) for FL,FR,RL,RR\n'
             '     (gz-sim MecanumDrive system, ros2_control mecanum_drive_controller); (b) dynamic: anisotropic friction per wheel with\n'
             '     mu2 ~ 0 along the ground-contact roller axis a_i=(sin45, s_i cos45), s=(-1,+1,+1,-1) (FL,FR,RL,RR), as CRANE does with its\n'
             '     ArticulationBody rover. The roller sign of each wheel is repeated in its <link> comment.\n'
             '     The arm is a PROXY (link lengths and limits from the concept image, STS3215-class servos). -->')
    L.append(f'<robot name="crane_{role}_v1">')
    L.append('  <link name="base_footprint"/>')
    L.append(f'  <joint name="base_footprint_joint" type="fixed"><parent link="base_footprint"/><child link="base_link"/><origin xyz="0 0 {clr:.6g}" rpy="0 0 0"/></joint>')
    L.append('  <link name="base_link">')
    L.append('    ' + inertial(Mb, cb, Ib))
    for n in static:
        for v in visuals(n):
            L.append('    ' + v.replace("<visual", "<visual").replace("<geometry>", "<origin xyz=\"0 0 0\" rpy=\"0 0 0\"/><geometry>", 1))
    for cl in collisions("base_link"):
        L.append('    ' + cl)
    L.append('  </link>')
    jname = {"arm_link_1": "arm_joint_1", "arm_link_2": "arm_joint_2", "arm_link_3": "arm_joint_3", "arm_link_4": "arm_joint_4",
             "arm_link_5": "arm_joint_5", "arm_gripper_jaw": "arm_gripper_joint", "fork_carriage": "lift_joint"}
    for n in moving:
        l = r.links[n]
        m, com, I, _ = l.mass_props()
        lname = n + "_link" if n.startswith("wheel_") or n.startswith("hook_") or n == "lidar_head" else n
        comment = ""
        if n.startswith("wheel_"):
            comment = f'  <!-- {n}: roller_angle_sign s={l.info["sign"]:+d}, ground roller axis ({math.sin(math.pi/4):.3f}, {l.info["sign"]*math.cos(math.pi/4):+.3f}) in base xy -->'
            L.append(comment)
        L.append(f'  <link name="{lname}">')
        if m > 0:
            L.append('    ' + inertial(m, com, I))
        for v in visuals(n):
            L.append('    ' + v.replace("<geometry>", "<origin xyz=\"0 0 0\" rpy=\"0 0 0\"/><geometry>", 1))
        for cl in collisions(n):
            L.append('    ' + cl)
        L.append('  </link>')
        j = l.joint
        parent = l.parent
        plink = "base_link" if parent == "base_link" or (parent in r.links and not r.links[parent].joint) else (parent + "_link" if parent.startswith(("wheel_", "hook_")) or parent == "lidar_head" else parent)
        # static parents (e.g. arm_base) are merged into base_link: fold their (identity) frame into the origin
        jo = j["origin"]
        jn = jname.get(n, n + "_joint")
        lim = ""
        if j["type"] in ("revolute", "prismatic"):
            eff, vel = (20.0, 0.05) if j["type"] == "prismatic" else (2.0, 2.0)
            if n.startswith("hook_"):
                eff, vel = (0.5, 1.0)
            if n.startswith("finger_"):
                eff, vel = (20.0, 0.04)
            lim = f'<limit lower="{j["limits"][0]:.6g}" upper="{j["limits"][1]:.6g}" effort="{eff}" velocity="{vel}"/>'
        elif j["type"] == "continuous":
            lim = '<limit effort="0.177" velocity="23.0"/>' if n.startswith("wheel_") else ""
        mimic = '<mimic joint="finger_left_joint" multiplier="1" offset="0"/>' if n == "finger_right" else ""
        L.append(f'  <joint name="{jn}" type="{j["type"]}"><parent link="{plink}"/><child link="{lname}"/>{origin(jo)}'
                 f'<axis xyz="{fmt(j["axis"])}"/>{lim}{mimic}</joint>')
    # fixed frames ---------------------------------------------------
    def fixed(name, parent, Tm, note=None):
        if note:
            L.append(f'  <!-- {note} -->')
        L.append(f'  <link name="{name}"/>')
        L.append(f'  <joint name="{name}_joint" type="fixed"><parent link="{parent}"/><child link="{name}"/>{origin(Tm)}</joint>')
    for s in sensor_entries(r):
        Tl = g.T_from_pose(s["origin"])
        if s["type"] == "imu":
            fixed("imu_link", "base_link", Tl, "Adafruit ISM330DHCX (#4502) in the internal bay")
        elif s["type"] == "lidar2d":
            fixed("laser", "base_link", Tl, "RPLIDAR C1 scan frame (scan plane = xy)")
        else:
            lk, op = s["params"]["urdf_link"], s["params"]["urdf_optical_frame"]
            if role == "scout" and s["name"] == "camera_depth":
                fixed("camera_link", "base_link", Tl, "RealSense D435: camera_link at the depth origin (left imager), realsense-ros convention")
                fixed(lk, "camera_link", np.eye(4))
            elif role == "scout":
                fixed(lk, "camera_link", np.linalg.inv(r.links["d435"].frames["infra1"]) @ Tl)
            else:
                fixed(lk, "base_link", Tl, "Arducam OV9281 (lens front)")
            fixed(op, lk, g.make_T(R_OPT))
    fixed("tag_top_link", "base_link", r.tag_T, f"AprilTag tag36h11 id {r.tag_id} (size {r.tag['size']} m), AprilTag3 frame: z into the tag")
    for f, ft in r.face_tags.items():
        fixed(f"tag_{f}_link", "base_link", ft["T"], f"dock-acquisition tag {f}: tag36h11 id {ft['id']}, {ft['size']} m, on the 45 deg bevel facet above the dock (z into the tag)")
    for f in P.FACE_ORDER:
        fixed(f"dock_{f}_frame", "base_link", P.face_T(c, f), f"docking face {f}: x out of the face, z up ({r.faces.get(f, 'none')})")
        if r.faces.get(f) == "dock":
            fixed(f"dock_{f}_pin", f"dock_{f}_frame", T(*P.pin_position_face(c)), f"catch pin {f} (axis y of this frame)")
            fixed(f"hook_{f}_a_engage", f"hook_{f}_a_link", r.links[f"hook_{f}_a"].frames["engage"], "hook crook centre (engage point, hook closed)")
    if role == "scout":
        fixed("arm_tcp", "arm_link_5", T(0, 0.004, c.d("scout.gripper") - 0.006), "arm tool centre point between the jaws (proxy)")
    L.append('</robot>')
    return "\n".join(L) + "\n"


# ------------------------------------------------------------------ payloads (reuse v0 generator code path, v1 profile)
def build_payloads(out, profile_path):
    """v0 payload generator (unchanged code) for the shared payloads + v1 additions (tag_box) generated here."""
    sys.path.insert(0, os.path.dirname(HERE))
    import gen_provisional as gp
    prof = json.load(open(profile_path))
    extra = {k: prof["payloads"].pop(k) for k in list(prof["payloads"]) if k not in ("box_small", "box_long", "tray_wide")}
    tmp = os.path.join(out, "_work", "profile_for_v0_payload_generator.json")
    os.makedirs(os.path.dirname(tmp), exist_ok=True)
    json.dump(prof, open(tmp, "w"), indent=1)
    c0 = gp.Cfg(tmp, os.path.join(os.path.dirname(HERE), "generator_defaults.json"))
    man, parts, mats = gp.build_payloads(c0)
    man["asset_id"] = "payloads_concept_v1"
    man["profile"] = {"profile_id": prof["profile_id"], "path": rel(profile_path), "sha256": sha(profile_path),
                      "version_note": "box_small/box_long/tray_wide from gen_provisional.build_payloads (unchanged code); tag_box added by build_v1.py"}
    mats["payload_tag_box"] = dict(base_color=[0.72, 0.55, 0.34])
    for name, d in extra.items():
        l, w, h = C.pv(d["size_lwh"])
        mass = C.pv(d["mass"])
        body = g.box_mesh([-l / 2, -w / 2, 0.0], [l / 2, w / 2, h], "payload_tag_box", "body")
        parts.append(gp.Part(f"payload_{name}", body, parent="payload_frame", role="payload"))
        I = gp.inertia_box(mass, [l, w, h])
        man["payloads"]["items"].append({"name": name, "frame": "payload_frame: origin at the centre of the bottom face; x along length, y along width, z up",
                                         "size_lwh": [l, w, h], "mass": mass, "com_xyz": [0, 0, h / 2], "inertia_tensor": I.tolist(), "render_part": f"payload_{name}",
                                         "collision": {"type": "box", "size": [l, w, h], "origin": g.pose_of(T(0, 0, h / 2))},
                                         "contact_patches": [{"name": "base", "center": [0, 0, 0], "size": [l, w], "friction_group": "cardboard"}],
                                         "grasp_faces": [{"name": "side_pos_y", "normal": [0, 1, 0], "center": [0, w / 2, h / 2], "size": [l, h]},
                                                         {"name": "side_neg_y", "normal": [0, -1, 0], "center": [0, -w / 2, h / 2], "size": [l, h]}],
                                         "needs": d["needs"] + " (v1 payload type from the concept modular2 tagged box; side-face tag not modelled)"})
        man["collision"]["shapes"].append({"name": f"{name}_col", "type": "box", "parent": "payload_frame", "origin": g.pose_of(T(0, 0, h / 2)), "size": [l, w, h]})
        man["render"]["parts"].append(parts[-1].entry())
    man["render"]["materials"] = mats
    for sec in ("render", "collision", "mass_properties", "payloads"):
        man[sec]["source_file"] = SRC_REF
    return man, parts, mats, c0


# ------------------------------------------------------------------ main
def main(argv=None):
    ap = argparse.ArgumentParser()
    ap.add_argument("--out", default=DEFAULT_OUT)
    ap.add_argument("--roles", nargs="+", default=["scout", "lifter", "cargo", "payloads"])
    ap.add_argument("--no-step", action="store_true")
    ap.add_argument("--profile", default=P.PROFILE)
    a = ap.parse_args(argv)
    if os.path.abspath(a.out) == os.path.abspath(os.path.expanduser("~/.cache/crane-fleet/assets_v0")):
        raise SystemExit("refusing to write into assets_v0")
    c = P.Cfg(a.profile)
    roles = {n: Role(c, n) for n in ("scout", "lifter", "cargo")}
    update_profile(c, roles, a.profile)
    paths = []
    os.makedirs(os.path.join(a.out, "meshes"), exist_ok=True)
    tag_ids = set()
    for n in [x for x in a.roles if x != "payloads"]:
        r = roles[n]
        d = os.path.join(a.out, n)
        os.makedirs(d, exist_ok=True)
        os.makedirs(os.path.join(d, "step"), exist_ok=True)
        mdir = os.path.join(a.out, "meshes", n)
        os.makedirs(mdir, exist_ok=True)
        mesh_info, mesh_files = {}, {}
        ntri = 0
        for ln in r.order:
            l = r.links[ln]
            mesh = C.link_mesh(l)
            if len(mesh) == 0:
                continue
            g.write_obj(os.path.join(d, ln + ".obj"), [(ln, mesh)], f"{n}.mtl",
                        header=f"{n}/{ln} - v1 concept-derived CAD (build123d), metres, part frame, x fwd y left z up", crease_deg=35.0)
            mesh_info[ln] = list(mesh.mats)
            ntri += len(mesh)
            files = []
            for mi, mat in enumerate(mesh.mats):
                sub = g.Mesh()
                sel = mesh.M == mi
                sub.P = mesh.P[sel]
                fn = f"{ln}__{mat}.stl"
                C.write_stl(os.path.join(mdir, fn), sub)
                files.append((mat, fn))
            mesh_files[ln] = files
            if not a.no_step and l.items:
                C.export_step_link(l, os.path.join(d, "step", ln + ".step"))
        man = build_manifest(r, mesh_info, a.profile, c.used)
        mats = {k: {kk: vv for kk, vv in v.items() if kk != "note"} for k, v in man["render"]["materials"].items()}
        g.write_mtl(os.path.join(d, f"{n}.mtl"), mats)
        mp = os.path.join(d, f"{n}.manifest.json")
        json.dump(man, open(mp, "w"), indent=1)
        paths.append(mp)
        tag_ids.add(r.tag_id)
        tag_ids |= {ft["id"] for ft in r.face_tags.values()}
        urdf = urdf_for(r, f"meshes/{n}", mesh_files, a.profile)
        os.makedirs(URDF_DIR, exist_ok=True)
        open(os.path.join(URDF_DIR, f"{n}.urdf"), "w").write(urdf)
        os.makedirs(os.path.join(a.out, "urdf"), exist_ok=True)
        open(os.path.join(a.out, "urdf", f"{n}.urdf"), "w").write(urdf)
        print(f"{n}: {len(mesh_info)} parts, {ntri} triangles, mass {man['mass_properties']['mass']:.3f} kg, COM {man['mass_properties']['com_xyz']}")
    if "payloads" in a.roles:
        man, parts, mats, c0 = build_payloads(a.out, a.profile)
        d = os.path.join(a.out, "payloads")
        os.makedirs(d, exist_ok=True)
        g.write_mtl(os.path.join(d, "payloads.mtl"), mats)
        for p_ in parts:
            g.write_obj(os.path.join(d, p_.name + ".obj"), [(p_.name, p_.mesh)], "payloads.mtl", header=f"payloads/{p_.name} - generated, metres")
        mp = os.path.join(d, "payloads.manifest.json")
        json.dump(man, open(mp, "w"), indent=1)
        paths.append(mp)
    tdir = os.path.join(a.out, "textures")
    os.makedirs(tdir, exist_ok=True)
    for tid in sorted(tag_ids | {t + 10 for t in tag_ids if 30 <= t < 40}):
        apriltag36h11.write_png(os.path.join(tdir, f"tag36h11_{tid:05d}.png"), apriltag36h11.tag_image(tid, 32))
    open(os.path.join(a.out, "package.xml"), "w").write(
        f'<?xml version="1.0"?>\n<package format="3"><name>{PKG}</name><version>0.1.0</version>'
        '<description>CRANE fleet v1 concept-derived rover meshes (generated)</description><maintainer email="none@example.com">crane</maintainer>'
        '<license>GPL-3.0-only</license><buildtool_depend>ament_cmake</buildtool_depend></package>\n')
    json.dump({"generated_by": SRC_REF, "profile": rel(a.profile), "profile_sha256": sha(a.profile), "params_sha256": sha(c.params_path)},
              open(os.path.join(a.out, "BUILD_INFO.json"), "w"), indent=1)
    print("manifests:", *paths, sep="\n  ")
    return paths


if __name__ == "__main__":
    main()
