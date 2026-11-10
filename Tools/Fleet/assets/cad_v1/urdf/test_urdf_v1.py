#!/usr/bin/env python3
"""URDF checks for the v1 rovers (run with pytest or as a script).

* parses with yourdfpy (if installed) and with a plain ElementTree parser
* joint tree: one root (base_footprint), every link reached once, no cycles, joint types/limits as expected
* inertias: mass > 0, positive definite, triangle inequality
* every mesh filename resolves (package://crane_fleet_v1/ -> $CRANE_FLEET_V1_PKG or ~/.cache/crane-fleet/assets_v1)
* wheel collisions are cylinders of the true wheel width and radius (no spheres)
* manifest <-> URDF consistency: same total mass and COM at the authored joint state, wheel/hook/face/tag/sensor frames coincide
"""
import json
import math
import os
import sys
import xml.etree.ElementTree as ET

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
ROLES = ("scout", "lifter", "cargo")
PKG = "crane_fleet_v1"
PKG_ROOT = os.environ.get("CRANE_FLEET_V1_PKG", os.path.expanduser("~/.cache/crane-fleet/assets_v1"))


def rpy_R(r, p, y):
    cr, sr, cp, sp, cy, sy = math.cos(r), math.sin(r), math.cos(p), math.sin(p), math.cos(y), math.sin(y)
    Rx = np.array([[1, 0, 0], [0, cr, -sr], [0, sr, cr]])
    Ry = np.array([[cp, 0, sp], [0, 1, 0], [-sp, 0, cp]])
    Rz = np.array([[cy, -sy, 0], [sy, cy, 0], [0, 0, 1]])
    return Rz @ Ry @ Rx


def origin_T(el):
    T = np.eye(4)
    o = el.find("origin") if el is not None else None
    if o is not None:
        xyz = [float(v) for v in o.get("xyz", "0 0 0").split()]
        rpy = [float(v) for v in o.get("rpy", "0 0 0").split()]
        T[:3, :3] = rpy_R(*rpy)
        T[:3, 3] = xyz
    return T


def axis_motion(axis, typ, q):
    a = np.asarray(axis, float)
    a = a / np.linalg.norm(a)
    T = np.eye(4)
    if typ in ("revolute", "continuous"):
        K = np.array([[0, -a[2], a[1]], [a[2], 0, -a[0]], [-a[1], a[0], 0]])
        T[:3, :3] = np.eye(3) + math.sin(q) * K + (1 - math.cos(q)) * K @ K
    elif typ == "prismatic":
        T[:3, 3] = a * q
    return T


class Urdf:
    def __init__(self, path):
        self.path = path
        self.root = ET.parse(path).getroot()
        self.links = {l.get("name"): l for l in self.root.findall("link")}
        self.joints = {j.get("name"): j for j in self.root.findall("joint")}
        self.child_joint = {j.find("child").get("link"): j for j in self.joints.values()}

    def T(self, link, q=None):
        q = q or {}
        if link not in self.child_joint:
            return np.eye(4)
        j = self.child_joint[link]
        parent = j.find("parent").get("link")
        typ = j.get("type")
        M = np.eye(4)
        if typ != "fixed":
            ax = [float(v) for v in j.find("axis").get("xyz").split()]
            name = j.get("name")
            val = q.get(name, 0.0)
            mim = j.find("mimic")
            if mim is not None:
                val = q.get(mim.get("joint"), 0.0) * float(mim.get("multiplier", 1)) + float(mim.get("offset", 0))
            M = axis_motion(ax, typ, val)
        return self.T(parent, q) @ origin_T(j) @ M

    def inertials(self):
        out = {}
        for n, l in self.links.items():
            i = l.find("inertial")
            if i is None:
                continue
            m = float(i.find("mass").get("value"))
            c = origin_T(i)[:3, 3]
            e = i.find("inertia").attrib
            I = np.array([[float(e["ixx"]), float(e["ixy"]), float(e["ixz"])], [float(e["ixy"]), float(e["iyy"]), float(e["iyz"])],
                          [float(e["ixz"]), float(e["iyz"]), float(e["izz"])]])
            out[n] = (m, c, I)
        return out


def resolve(fn):
    if fn.startswith(f"package://{PKG}/"):
        return os.path.join(PKG_ROOT, fn[len(f"package://{PKG}/"):])
    return os.path.join(HERE, fn)


def load(role):
    return Urdf(os.path.join(HERE, f"{role}.urdf"))


def manifest(role):
    return json.load(open(os.path.join(PKG_ROOT, role, f"{role}.manifest.json")))


def authored_q(man, u):
    """URDF joint values equal to the manifest's authored joint values."""
    q = {}
    for p in man["render"]["parts"]:
        j = p.get("joint")
        if not j:
            continue
        n = p["name"]
        for jn, je in u.joints.items():
            if je.find("child").get("link") in (n, n + "_link"):
                q[jn] = j.get("authored_value", 0.0)
    return q


# ------------------------------------------------------------------ tests
def test_parse_yourdfpy():
    try:
        import yourdfpy
    except ImportError:
        return
    for r in ROLES:
        robot = yourdfpy.URDF.load(os.path.join(HERE, f"{r}.urdf"), build_scene_graph=True, load_meshes=False,
                                   filename_handler=lambda fn: resolve(fn))
        assert robot.base_link == "base_footprint", robot.base_link
        assert len(robot.robot.links) == len(load(r).links)


def test_tree():
    for r in ROLES:
        u = load(r)
        roots = [n for n in u.links if n not in u.child_joint]
        assert roots == ["base_footprint"], (r, roots)
        for n in u.links:                      # every link reaches the root without cycles
            seen, cur = set(), n
            while cur in u.child_joint:
                assert cur not in seen, (r, "cycle", n)
                seen.add(cur)
                cur = u.child_joint[cur].find("parent").get("link")
            assert cur == "base_footprint"
        types = {j.get("name"): j.get("type") for j in u.joints.values()}
        for w in ("FL", "FR", "RL", "RR"):
            assert types[f"wheel_{w}_joint"] == "continuous"
            ax = u.joints[f"wheel_{w}_joint"].find("axis").get("xyz").split()
            assert [float(v) for v in ax] == [0, 1, 0]
        hooks = [n for n in types if n.startswith("hook_") and n.endswith("_a_joint")]
        assert hooks and all(types[h] == "revolute" for h in hooks), (r, hooks)
        for jn, j in u.joints.items():
            if j.get("type") in ("revolute", "prismatic"):
                lim = j.find("limit")
                assert lim is not None and float(lim.get("lower")) < float(lim.get("upper")), (r, jn)
        if r == "lifter":
            assert types["lift_joint"] == "prismatic" and types["finger_left_joint"] == "prismatic" and types["finger_right_joint"] == "prismatic"
        if r == "scout":
            for k in range(1, 6):
                assert types[f"arm_joint_{k}"] == "revolute"
            assert types["arm_gripper_joint"] == "revolute"
        names = set(u.links)
        need = {"base_link", "imu_link", "tag_top_link", "dock_front_frame", "dock_rear_frame", "dock_left_frame", "dock_right_frame"}
        need |= {"laser", "camera_link", "camera_depth_optical_frame", "camera_color_optical_frame", "camera_infra1_optical_frame", "camera_infra2_optical_frame"} if r == "scout" else {"front_camera_link", "front_camera_optical_frame"}
        assert need <= names, (r, need - names)


def test_inertia_positive_definite():
    for r in ROLES:
        for n, (m, c, I) in load(r).inertials().items():
            assert m > 0, (r, n)
            w = np.linalg.eigvalsh(I)
            assert w.min() > 0, (r, n, w)
            a, b, cc = np.sort(w)
            assert a + b >= cc * (1 - 1e-6), (r, n, w)


def test_meshes_resolve():
    for r in ROLES:
        u = load(r)
        for m in u.root.iter("mesh"):
            p = resolve(m.get("filename"))
            assert os.path.exists(p), (r, m.get("filename"), p)


def test_wheel_collision_cylinders():
    for r in ROLES:
        u = load(r)
        man = manifest(r)
        w0 = man["wheels"]["items"][0]
        for w in ("FL", "FR", "RL", "RR"):
            l = u.links[f"wheel_{w}_link"]
            cols = l.findall("collision")
            assert cols, (r, w)
            for c in cols:
                geo = c.find("geometry")
                assert geo.find("sphere") is None, "wheel collision must not be a sphere"
                cy = geo.find("cylinder")
                assert cy is not None
                assert abs(float(cy.get("length")) - w0["width"]) < 1e-6 and abs(float(cy.get("radius")) - w0["radius"]) < 1e-6
                R = origin_T(c)[:3, :3]
                assert abs(abs(R[:, 2] @ np.array([0, 1.0, 0])) - 1) < 1e-6, "cylinder axis must be the wheel axle (+y)"


def test_manifest_consistency():
    for r in ROLES:
        u = load(r)
        man = manifest(r)
        q = authored_q(man, u)
        Tb = u.T("base_link")
        comps = []
        for n, (m, c, I) in u.inertials().items():
            Tw = np.linalg.inv(Tb) @ u.T(n, q)
            comps.append((m, (Tw @ np.append(c, 1))[:3]))
        M = sum(m for m, _ in comps)
        com = sum(m * c for m, c in comps) / M
        mp = man["mass_properties"]
        assert abs(M - mp["mass"]) < 1e-4 * mp["mass"], (r, M, mp["mass"])
        assert np.abs(com - np.array(mp["com_xyz"])).max() < 1e-4, (r, com, mp["com_xyz"])
        assert abs(Tb[2, 3] - man["base_frame"]["ground_clearance"]) < 1e-9
        inv = np.linalg.inv(Tb)
        def same(link, xyz, tol=1e-6, q_=None):
            p = (inv @ u.T(link, q_ or q))[:3, 3]
            assert np.abs(p - np.array(xyz)).max() < tol, (r, link, p, xyz)
        for w in man["wheels"]["items"]:
            same(f"wheel_{w['name']}_link", w["position"])
        for f in man["attachments"]["docking_faces"]:
            same(f"dock_{f['name']}_frame", f["origin"]["xyz"])
            for pin in f["catch_pins"]:
                F = u.T(f"dock_{f['name']}_frame")
                pw = (inv @ F @ np.append(pin["position"], 1))[:3]
                same(f"dock_{f['name']}_pin", pw)
        for h in man["attachments"]["hook_pivots"]:
            same(h["name"].replace("pivot_", "hook_") + "_link", h["origin"]["xyz"])
        for t in man["attachments"]["tags"]:
            same(f"{t['name']}_link", t["origin"]["xyz"])
        for s in man["sensors"]["frames"]:
            lk = s.get("params", {}).get("urdf_link")
            if lk:
                same(lk, s["origin"]["xyz"])
                Tl = inv @ u.T(lk)
                from_pose = rpy_R(*s["origin"]["rpy"])
                assert np.abs(Tl[:3, :3] - from_pose).max() < 1e-6, (r, lk)
            op = s.get("params", {}).get("urdf_optical_frame")
            if op:
                To = inv @ u.T(op)
                assert np.abs(To[:3, 3] - np.array(s["optical_origin"]["xyz"])).max() < 1e-6
                assert np.abs(To[:3, :3] - rpy_R(*s["optical_origin"]["rpy"])).max() < 1e-6, (r, op)
                assert np.allclose(To[:3, 2], Tl[:3, 0], atol=1e-6), "optical z must equal the link boresight (x)"


if __name__ == "__main__":
    fails = 0
    for name, fn in sorted(globals().items()):
        if name.startswith("test_") and callable(fn):
            try:
                fn()
                print("PASS", name)
            except Exception as e:  # noqa: BLE001
                fails += 1
                print("FAIL", name, type(e).__name__, e)
    sys.exit(1 if fails else 0)
