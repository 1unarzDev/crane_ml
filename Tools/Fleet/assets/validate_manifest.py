#!/usr/bin/env python3
"""Validate a crane.fleet_asset_manifest/v0 (provisional or CAD-derived).

    python validate_manifest.py MANIFEST [MANIFEST ...] [--profile P] [--report out.json]
                                [--mate "A.json:front,B.json:rear"] [--no-mate] [--tol key=value ...]

Checks (ids are stable; every check is PASS / WARN / FAIL):
 schema, files, units, scale, profile, wheels_ground, symmetry, roller_handedness, mass, bbox, watertight,
 actuated, mating.  Exit status 1 if any FAIL.
"""
import argparse
import json
import math
import os
import sys

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
import geom as g  # noqa: E402
import minischema  # noqa: E402
from rover_model import RoverModel  # noqa: E402

REPO = os.path.abspath(os.path.join(HERE, "..", "..", ".."))
SCHEMA_PATH = os.path.join(HERE, "fleet_asset_manifest_v0.schema.json")
TOL = dict(pos=2e-3, ang=0.02, ground=1e-3, bbox=0.035, body=3e-3, mass_rel=0.10, inertia_factor=3.0,
           gap_over=0.010, gap_under=0.005, vol_body=2e-6, vol_hook=1e-6, vol_other=1e-6, engage=None,
           scale_lo=0.4, scale_hi=4.0, sym=2e-3, cell=0.002)
SKIP_ROLES = {"tag", "envelope", "marker"}


def pv(x):
    return x["value"] if isinstance(x, dict) and "value" in x else x


class Report:
    def __init__(self, path):
        self.path = path
        self.checks = []
        self.mating = []

    def add(self, cid, level, msg, measured=None, expected=None, tol=None):
        e = {"id": cid, "level": level, "message": msg}
        if measured is not None:
            e["measured"] = measured
        if expected is not None:
            e["expected"] = expected
        if tol is not None:
            e["tol"] = tol
        self.checks.append(e)

    def ok(self, cid, msg, **k):
        self.add(cid, "PASS", msg, **k)

    def warn(self, cid, msg, **k):
        self.add(cid, "WARN", msg, **k)

    def fail(self, cid, msg, **k):
        self.add(cid, "FAIL", msg, **k)

    def check(self, cid, cond, msg_ok, msg_bad, level_bad="FAIL", **k):
        self.add(cid, "PASS" if cond else level_bad, msg_ok if cond else msg_bad, **k)
        return cond

    def counts(self):
        c = {"PASS": 0, "WARN": 0, "FAIL": 0}
        for e in self.checks:
            c[e["level"]] += 1
        return c

    def overall(self):
        return "FAIL" if self.counts()["FAIL"] else "PASS"

    def to_json(self):
        return {"schema": "crane.fleet_asset_validation/v0", "manifest": self.path, "overall": self.overall(),
                "counts": self.counts(), "checks": self.checks, "mating": self.mating}

    def summary(self):
        c = self.counts()
        lines = [f"== {self.path}", f"   overall {self.overall()}   PASS {c['PASS']}  WARN {c['WARN']}  FAIL {c['FAIL']}"]
        for e in self.checks:
            if e["level"] != "PASS":
                lines.append(f"   [{e['level']}] {e['id']}: {e['message']}")
        for m in self.mating:
            lines.append("   mate " + m["label"] + ": " + m["summary"])
        return "\n".join(lines)


def active_faces_for(prof, role):
    """Per-role active faces (profile v1 key roles.<role>.active_faces) or the fleet default (v0)."""
    r = prof.get("roles", {}).get(role, {})
    return list(pv(r["active_faces"])) if "active_faces" in r else list(prof["docking"]["active_faces_default"])


def expected_tag_size(prof, role, tag_name):
    """v1 profiles may give per-role tag sizes (roles.<role>.tag_sizes.{tag_name}); otherwise the v0 rule."""
    ts = prof.get("roles", {}).get(role, {}).get("tag_sizes")
    if ts and tag_name in ts:
        return pv(ts[tag_name])
    return pv(prof["sensors"]["apriltag"]["top_tag_size" if tag_name == "tag_top" else "face_tag_size"])


def _wrap(angle):
    return (angle + math.pi) % (2 * math.pi) - math.pi


def safe(rep, cid, fn, *a, **k):
    try:
        return fn(*a, **k)
    except Exception as e:  # noqa: BLE001
        rep.fail(cid, f"check crashed: {type(e).__name__}: {e}")


# ------------------------------------------------------------------- helpers
def resolve(path, mdir):
    for base in (mdir, REPO, os.getcwd()):
        p = os.path.join(base, path)
        if os.path.exists(p):
            return p
    return None


def visible_parts(model, skip_roles=SKIP_ROLES):
    out = []
    for n, p in model.parts.items():
        if p.get("role_in_assembly") in skip_roles or not p.get("default_visible", True):
            continue
        out.append(n)
    return out


def authored_state(model):
    st = {}
    for n, p in model.parts.items():
        j = p.get("joint")
        if j:
            st[n] = j.get("authored_value", 0.0)
    return st


def collision_aabb(model):
    lo, hi = np.full(3, np.inf), np.full(3, -np.inf)
    for s in model.m["collision"]["shapes"]:
        T = model.frame_T(s["parent"], authored_state(model)) @ g.T_from_pose(s["origin"])
        if s["type"] == "box":
            h = np.array(s["size"]) / 2
            pts = np.array([[sx * h[0], sy * h[1], sz * h[2]] for sx in (-1, 1) for sy in (-1, 1) for sz in (-1, 1)])
        elif s["type"] == "cylinder":
            r, h = s["radius"], s["height"] / 2
            pts = np.array([[r * c, r * s_, z] for c, s_ in ((1, 0), (-1, 0), (0, 1), (0, -1), (.7071, .7071), (-.7071, .7071), (.7071, -.7071), (-.7071, -.7071)) for z in (-h, h)])
        elif s["type"] == "sphere":
            r = s["radius"]
            pts = np.array([[sx * r, sy * r, sz * r] for sx in (-1, 1) for sy in (-1, 1) for sz in (-1, 1)]) * 0.577
            pts = np.concatenate([pts, np.eye(3) * r, -np.eye(3) * r])
        else:
            f = resolve(s.get("file", ""), model.dir)
            if not f:
                continue
            mm, _ = g.read_obj(f)
            pts = mm.P.reshape(-1, 3)
        w = pts @ T[:3, :3].T + T[:3, 3]
        lo, hi = np.minimum(lo, w.min(0)), np.maximum(hi, w.max(0))
    return lo, hi


def render_aabb(model, state=None):
    lo, hi = np.full(3, np.inf), np.full(3, -np.inf)
    st = state if state is not None else authored_state(model)
    for n in visible_parts(model):
        m = model.world_mesh(n, st)
        if len(m) == 0:
            continue
        l, h = m.bbox()
        lo, hi = np.minimum(lo, l), np.maximum(hi, h)
    return lo, hi


def top_surface_z(mesh):
    """z of the upward-facing surface with the largest total area (the 'deck top')."""
    n, a = mesh.face_normals()
    up = (n[:, 2] / np.maximum(a, 1e-30) > 0.99) & (a > 1e-9)
    if not up.any():
        return None
    z = mesh.P[up].mean(1)[:, 2]
    area = a[up] / 2
    keys = np.round(z / 5e-4).astype(int)
    best, bz = 0, None
    for k in np.unique(keys):
        s = area[keys == k].sum()
        if s > best:
            best, bz = s, z[keys == k].mean()
    return float(bz)


# ------------------------------------------------------------------ checks
def check_schema(rep, man):
    schema = json.load(open(SCHEMA_PATH))
    errs = minischema.validate(man, schema)
    if errs:
        rep.fail("schema", f"{len(errs)} schema errors; first: {errs[0]}", measured=errs[:10])
    else:
        rep.ok("schema", "manifest conforms to crane.fleet_asset_manifest/v0")
    return not errs


def check_provenance_and_sources(rep, model, mdir):
    sections = ["render", "collision", "mass_properties", "wheels", "sensors", "attachments", "actuated", "payloads"]
    for s in sections:
        if s not in model.m:
            continue
        sec = model.m[s]
        src = sec.get("source_file")
        if not src:
            rep.fail(f"files.source.{s}", f"section '{s}' has no source_file")
        elif resolve(src, mdir) is None:
            rep.fail(f"files.source.{s}", f"section '{s}' source_file '{src}' not found (searched manifest dir, repo root)")
        else:
            rep.ok(f"files.source.{s}", f"{s}: source_file exists ({sec.get('provenance')})")


def check_files(rep, model):
    mdir = model.dir
    m = model.m
    missing = []
    for p in m["render"]["parts"]:
        f = os.path.join(mdir, p["file"])
        if not os.path.exists(f):
            missing.append(p["file"])
    mtl = m["render"].get("mtl_file")
    if mtl and not os.path.exists(os.path.join(mdir, mtl)):
        missing.append(mtl)
    for k, v in m["render"]["materials"].items():
        if v.get("texture") and not os.path.exists(os.path.normpath(os.path.join(mdir, v["texture"]))):
            missing.append(v["texture"])
    for s in m["collision"]["shapes"]:
        if s["type"] == "convex_mesh" and (not s.get("file") or not os.path.exists(os.path.join(mdir, s["file"]))):
            missing.append(s.get("file", "<convex_mesh without file>"))
    for t in m.get("attachments", {}).get("tags", []):
        if t.get("texture") and not os.path.exists(os.path.normpath(os.path.join(mdir, t["texture"]))):
            missing.append(t["texture"])
    rep.check("files.exist", not missing, "all referenced files exist", f"missing files: {missing[:8]}", measured=missing)
    # OBJ contents
    bad = []
    declared = set(m["render"]["materials"])
    for p in m["render"]["parts"]:
        f = os.path.join(mdir, p["file"])
        if not os.path.exists(f):
            continue
        try:
            mesh, info = g.read_obj(f)
        except Exception as e:  # noqa: BLE001
            bad.append(f"{p['file']}: unreadable ({e})")
            continue
        if len(mesh) == 0:
            bad.append(f"{p['file']}: no faces")
        if info["n_vn"] == 0:
            bad.append(f"{p['file']}: no normals")
        if info["n_vt"] == 0:
            bad.append(f"{p['file']}: no UVs")
        used = set(mesh.mats)
        if not used <= set(p["materials"]):
            bad.append(f"{p['file']}: uses materials {sorted(used - set(p['materials']))} not listed in part.materials")
        if not set(p["materials"]) <= declared:
            bad.append(f"{p['name']}: material slots {sorted(set(p['materials']) - declared)} not declared in render.materials")
        parents = set(model.parts) | {model.base}
        if p["parent"] not in parents:
            bad.append(f"{p['name']}: parent '{p['parent']}' unknown")
    rep.check("files.obj_contents", not bad, "OBJ files readable with normals, UVs and declared material slots", "; ".join(bad[:6]), measured=bad)
    # collision parents
    parents = set(model.parts) | {model.base}
    badc = [s["name"] for s in m["collision"]["shapes"] if s["parent"] not in parents]
    rep.check("files.collision_parents", not badc, "collision shape parents exist", f"unknown parents: {badc}")


def check_scale(rep, model, prof, tol):
    L, W, H = pv(prof["chassis"]["body_size_lwh"])
    ref = max(L, W, H)
    lo, hi = render_aabb(model)
    ext = hi - lo
    if model.m["role"] == "payloads":
        ref = max(max(pv(d["size_lwh"])) for d in prof["payloads"].values())
        ratio = float(ext.max() / ref)
        rep.check("scale.assembly", 0.3 <= ratio <= 5, f"largest payload extent {ext.max():.3f} m", f"extent {ext.max():.4f} m vs reference {ref} m (ratio {ratio:.2f}): unit error?", measured=ratio)
        return
    longest = float(ext.max())
    ratio = longest / ref
    ok = tol["scale_lo"] <= ratio <= tol["scale_hi"] * 1.5
    rep.check("scale.assembly", ok, f"rover extent {ext.round(3).tolist()} m is {ratio:.2f}x the profile body ({ref} m)",
              f"rover extent {ext.round(4).tolist()} m is {ratio:.3g}x the profile body: probable mm/m unit mistake",
              measured=ext.tolist(), expected=ref)
    absurd = []
    for n in model.parts:
        m = model.mesh(n)
        if len(m) == 0:
            continue
        e = (m.bbox()[1] - m.bbox()[0]).max()
        if e > 5.0 or e < 1e-4:
            absurd.append((n, float(e)))
    rep.check("scale.parts", not absurd, "every part is between 0.1 mm and 5 m", f"parts with absurd size: {absurd[:5]}", measured=absurd)
    r = model.m.get("wheels", {}).get("items", [{}])[0].get("radius")
    if r is not None:
        rep.check("scale.wheel_radius", 0.01 <= r <= 0.5, f"wheel radius {r} m plausible", f"wheel radius {r} m implausible (mm vs m?)", measured=r)


def check_profile(rep, model, prof, tol):
    m = model.m
    role = m["role"]
    ch = prof["chassis"]
    L, W, H = pv(ch["body_size_lwh"])
    clr = pv(ch["ground_clearance"])
    r, ww = pv(ch["wheel"]["radius"]), pv(ch["wheel"]["width"])
    lx, ly = pv(ch["wheel"]["half_wheelbase_lx"]), pv(ch["wheel"]["half_track_ly"])
    if role == "payloads":
        bad = []
        items = {it["name"]: it for it in m["payloads"]["items"]}
        for k, d in prof["payloads"].items():
            it = items.get(k)
            if not it:
                bad.append(f"{k} missing")
                continue
            if np.abs(np.array(it["size_lwh"]) - np.array(pv(d["size_lwh"]))).max() > tol["pos"]:
                bad.append(f"{k} size {it['size_lwh']} vs {pv(d['size_lwh'])}")
            if abs(it["mass"] - pv(d["mass"])) > tol["mass_rel"] * pv(d["mass"]):
                bad.append(f"{k} mass {it['mass']} vs {pv(d['mass'])}")
        rep.check("profile.payloads", not bad, "payload sizes and masses match the profile", "; ".join(bad), measured=bad)
        return
    rep.check("profile.ground_clearance", abs(m["base_frame"]["ground_clearance"] - clr) < tol["pos"] and abs(m["base_frame"]["ground_z"] + clr) < tol["pos"],
              f"ground clearance {clr} m", f"manifest clearance {m['base_frame']['ground_clearance']} / ground_z {m['base_frame']['ground_z']} vs profile {clr}")
    # wheels
    bad = []
    for w in m["wheels"]["items"]:
        sx = 1 if w["name"][0] == "F" else -1
        sy = 1 if w["name"][1] == "L" else -1
        exp = np.array([sx * lx, sy * ly, r - clr])
        d = np.abs(np.array(w["position"]) - exp).max()
        if d > tol["pos"]:
            bad.append(f"{w['name']} position {w['position']} vs {exp.tolist()} (err {d*1000:.1f} mm)")
        if abs(w["radius"] - r) > tol["pos"]:
            bad.append(f"{w['name']} radius {w['radius']} vs {r}")
        if abs(w["width"] - ww) > tol["pos"]:
            bad.append(f"{w['name']} width {w['width']} vs {ww}")
        if w["roller_count"] != pv(ch["wheel"]["roller_count"]):
            bad.append(f"{w['name']} roller_count {w['roller_count']} vs {pv(ch['wheel']['roller_count'])}")
    rep.check("profile.wheels", not bad, "wheel positions, radius, width and roller count match the profile", "; ".join(bad), measured=bad, tol=tol["pos"])
    # body
    st = authored_state(model)
    if "chassis_body" in model.parts:
        lo, hi = model.world_mesh("chassis_body", st).bbox()
        e = hi - lo
        err = [abs(e[0] - L), abs(e[1] - W)]
        rep.check("profile.body_size", max(err) < tol["body"], f"chassis_body footprint {e[0]:.3f} x {e[1]:.3f} m",
                  f"chassis_body footprint {e[0]:.4f} x {e[1]:.4f} m vs profile {L} x {W}", measured=e.tolist(), expected=[L, W, H], tol=tol["body"])
        rep.check("profile.body_clearance", abs((lo[2] - m["base_frame"]["ground_z"]) - clr) < tol["body"],
                  f"body underside {lo[2] - m['base_frame']['ground_z']:.4f} m above ground",
                  f"body underside is {lo[2] - m['base_frame']['ground_z']:.4f} m above the ground plane, profile clearance {clr}",
                  measured=float(lo[2] - m["base_frame"]["ground_z"]), expected=clr)
        rep.check("profile.body_height", abs((hi[2] - lo[2]) - H) < tol["body"] or role != "scout" and False or abs((hi[2] - lo[2]) - H) < tol["body"],
                  f"body height {hi[2]-lo[2]:.3f} m", f"body height {hi[2]-lo[2]:.4f} vs profile {H}", level_bad="WARN")
    else:
        rep.warn("profile.body_size", "no part named 'chassis_body'; body footprint not checked")
    # faces
    att = m["attachments"]
    bad = []
    s = pv(prof["docking"]["hook_lateral_spacing"])
    for f in att["docking_faces"]:
        pf = pv(prof["docking"]["face_frames"][f["name"]])
        T = g.T_from_pose(f["origin"])
        yaw = g.R_to_rpy(T[:3, :3])[2]
        pos_err = np.abs(T[:3, 3] - np.array(pf[:3])).max()
        yaw_err = abs(_wrap(yaw - pf[3]))
        act_exp = f["name"] in active_faces_for(prof, role)
        if pos_err > tol["pos"]:
            bad.append(f"{f['name']} position {T[:3,3].round(4).tolist()} vs {pf[:3]} (err {pos_err*1000:.1f} mm)")
        if yaw_err > tol["ang"]:
            bad.append(f"{f['name']} yaw {yaw:.3f} vs {pf[3]:.3f}")
        if f["active"] != act_exp:
            bad.append(f"{f['name']} active={f['active']} vs profile default {act_exp}")
        if f["geometry"] == "modelled":
            n_h = len(f["hooks"])
            if n_h != pv(prof["docking"]["hook_count"]):
                bad.append(f"{f['name']} has {n_h} hooks vs hook_count {pv(prof['docking']['hook_count'])}")
            ys = []
            for hn in f["hooks"]:
                pivs = [p for p in att["hook_pivots"] if p["name"] == next((h["pivot"] for h in model.m["actuated"]["hooks"] if h["name"] == hn), "")]
                if pivs:
                    Tf = np.linalg.inv(T) @ g.T_from_pose(pivs[0]["origin"])
                    ys.append(Tf[1, 3])
            if len(ys) == 2 and abs(abs(ys[0] - ys[1]) - s) > tol["pos"]:
                bad.append(f"{f['name']} hook spacing {abs(ys[0]-ys[1]):.4f} vs {s}")
    rep.check("profile.faces", not bad, "docking-face poses, active set and hook spacing match the profile", "; ".join(bad), measured=bad, tol=tol["pos"])
    # tags
    bad = []
    for t in att["tags"]:
        exp = expected_tag_size(prof, role, t["name"])
        if abs(t["size"] - exp) > 1e-4:
            bad.append(f"{t['name']} size {t['size']} vs {exp}")
        if t.get("plate_size") and abs(t["plate_size"] - t["size"] * 10 / 8) > 1e-4:
            bad.append(f"{t['name']} plate_size {t['plate_size']} != size*10/8")
    rep.check("profile.tags", not bad, "tag sizes match the profile (black-border edge)", "; ".join(bad))
    # sensors
    cams = [s_ for s_ in m["sensors"]["frames"] if s_["type"] == "camera"]
    bad = []
    for c in cams:
        p = c.get("params", {})
        # v1: a camera names its profile sensor entry (params.profile_sensor); v0 cameras are all OV9281
        mp = prof["sensors"][p.get("profile_sensor", "mono_camera_ov9281")]
        if abs(p.get("hfov", 0) - pv(mp["hfov"])) > 1e-3:
            bad.append(f"{c['name']} hfov {p.get('hfov')} vs {pv(mp['hfov'])}")
        if list(p.get("resolution", [])) != list(pv(mp["resolution"])):
            bad.append(f"{c['name']} resolution")
    req = prof["roles"][role].get("required_sensor_frames")
    if req is not None:
        # v1: the profile lists the sensor frames each role must provide (name -> type)
        have = {s_["name"]: s_["type"] for s_ in m["sensors"]["frames"]}
        for nm, typ in pv(req).items():
            if have.get(nm) != typ:
                bad.append(f"required sensor frame {nm} ({typ}) missing or wrong type ({have.get(nm)})")
        sb = prof["sensors"].get("stereo_baseline")
        pair = pv(prof["roles"][role].get("stereo_pair", [])) or []
        if sb is not None and len(pair) == 2:
            A_ = [c for c in cams if c["name"] == pair[0]]
            B_ = [c for c in cams if c["name"] == pair[1]]
            if A_ and B_:
                pa_ = g.T_from_pose(A_[0]["origin"])[:3, 3]
                pb_ = g.T_from_pose(B_[0]["origin"])[:3, 3]
                bl = float(np.linalg.norm(pa_ - pb_))
                if abs(bl - pv(sb)) > 1e-3:
                    bad.append(f"stereo baseline {bl:.4f} vs {pv(sb)}")
    elif role == "scout":
        L_, R_ = [c for c in cams if c["name"] == "cam_left"], [c for c in cams if c["name"] == "cam_right"]
        if L_ and R_:
            bl = abs(L_[0]["origin"]["xyz"][1] - R_[0]["origin"]["xyz"][1])
            if abs(bl - pv(prof["sensors"]["stereo_baseline"])) > 1e-3:
                bad.append(f"stereo baseline {bl:.4f} vs {pv(prof['sensors']['stereo_baseline'])}")
        else:
            bad.append("scout needs cam_left and cam_right")
        if not any(s_["type"] == "lidar2d" for s_ in m["sensors"]["frames"]):
            bad.append("scout needs a lidar2d frame")
    elif req is None:
        if not any(s_["type"] == "tof" for s_ in m["sensors"]["frames"]) or not cams:
            bad.append("front camera and ToF expected")
    if not any(s_["type"] == "imu" for s_ in m["sensors"]["frames"]):
        bad.append("imu frame missing")
    rep.check("profile.sensors", not bad, "sensor set and camera parameters match the profile", "; ".join(bad), measured=bad)
    # role-specific
    rp = prof["roles"][role]
    if role == "cargo":
        dp = model.mesh("deck") if "deck" in model.parts else None
        if dp is not None and len(dp):
            z = top_surface_z(model.world_mesh("deck", st))
            # profile deck_height is the deck TOP above the GROUND (generator_defaults cargo.deck_height_reference)
            exp = m["base_frame"]["ground_z"] + pv(rp["deck_height"])
            rep.check("profile.deck_height", z is not None and abs(z - exp) < tol["pos"], f"deck top {z:.4f} m in base (= {pv(rp['deck_height'])} m above ground)",
                      f"deck top {z} vs expected {exp:.4f} in base frame (profile deck_height {pv(rp['deck_height'])} above ground)", measured=z, expected=exp, tol=tol["pos"])
            lo, hi = dp.transformed(model.frame_T("deck", st)).bbox()
            dl, dw = pv(rp["deck_size_lw"])
            rep.check("profile.deck_size", abs((hi[0] - lo[0]) - dl) < tol["body"] and abs((hi[1] - lo[1]) - dw) < tol["body"],
                      f"deck footprint {(hi[0]-lo[0]):.3f} x {(hi[1]-lo[1]):.3f}", f"deck footprint {(hi[0]-lo[0]):.4f} x {(hi[1]-lo[1]):.4f} vs {dl} x {dw}")
        else:
            rep.fail("profile.deck_height", "cargo manifest has no part named 'deck'")
    if role == "lifter":
        lift = m["actuated"].get("lift")
        gr = m["actuated"].get("gripper")
        ok = lift is not None and abs((lift["range"][1] - lift["range"][0]) - pv(rp["lift_travel"])) < tol["pos"]
        rep.check("profile.lift_travel", ok, f"lift travel {pv(rp['lift_travel'])} m", f"lift travel {lift and lift['range']} vs profile {pv(rp['lift_travel'])}")
        ok = gr is not None and abs(gr["opening_range"][1] - pv(rp["gripper_opening"])) < tol["pos"]
        rep.check("profile.gripper_opening", ok, f"gripper max opening {pv(rp['gripper_opening'])} m", f"gripper opening {gr and gr['opening_range']} vs profile {pv(rp['gripper_opening'])}")
    if role == "scout":
        if "mast" in model.parts:
            hi = model.world_mesh("mast", st).bbox()[1][2]
            exp = H + pv(rp["mast_height"]) + m["base_frame"].get("ground_z", 0) * 0
            rep.check("profile.mast_height", abs(hi - exp) < 5e-3, f"mast top {hi:.3f} m = body top + {pv(rp['mast_height'])}", f"mast top {hi:.4f} vs body top + mast_height = {exp:.4f}", measured=float(hi), expected=exp)
        else:
            rep.warn("profile.mast_height", "no part named 'mast'")
        ap = m["actuated"].get("arm_proxy")
        rep.check("profile.arm_proxy", ap is not None and abs(ap["reach"] - pv(rp["arm_proxy"]["reach"]["value"] if "value" in rp["arm_proxy"]["reach"] else rp["arm_proxy"]["reach"])) < tol["pos"],
                  "arm proxy reach matches", "arm proxy missing or wrong reach")


def check_wheels_ground(rep, model, prof, tol):
    st = authored_state(model)
    gz = model.m["base_frame"]["ground_z"]
    bad, meas = [], {}
    for w in model.m["wheels"]["items"]:
        z_axle_bottom = w["position"][2] - w["radius"]
        e1 = abs(z_axle_bottom - gz)
        msg = None
        if e1 > tol["ground"]:
            msg = f"{w['name']}: axle z - radius = {z_axle_bottom:.4f} but ground plane is {gz:.4f} (err {e1*1000:.2f} mm)"
        rp = model.parts.get(w["render_part"])
        if rp is not None:
            mesh = model.world_mesh(w["render_part"], st)
            if len(mesh):
                zmin = float(mesh.bbox()[0][2])
                meas[w["name"]] = zmin
                if abs(zmin - gz) > tol["ground"]:
                    msg = (msg + "; " if msg else f"{w['name']}: ") + f"wheel mesh lowest point z={zmin:.4f} vs ground {gz:.4f} (err {(zmin-gz)*1000:.2f} mm)"
        if msg:
            bad.append(msg)
    rep.check("wheels_ground", not bad, "all four wheels touch z = ground plane within tolerance (axle height and mesh)", "; ".join(bad), measured=meas, tol=tol["ground"])


def check_symmetry(rep, model, tol):
    m = model.m
    W = {w["name"]: w for w in m["wheels"]["items"]}
    bad = []
    for a, b in (("FL", "FR"), ("RL", "RR")):
        pa, pb = np.array(W[a]["position"]), np.array(W[b]["position"])
        if abs(pa[0] - pb[0]) > tol["sym"] or abs(pa[1] + pb[1]) > tol["sym"] or abs(pa[2] - pb[2]) > tol["sym"]:
            bad.append(f"wheels {a}/{b} not mirrored about y=0: {pa.tolist()} vs {pb.tolist()}")
        if W[a]["roller_angle_sign"] != -W[b]["roller_angle_sign"]:
            bad.append(f"wheels {a}/{b} must have opposite roller_angle_sign")
    for a, b in (("FL", "RL"), ("FR", "RR")):
        pa, pb = np.array(W[a]["position"]), np.array(W[b]["position"])
        if abs(pa[0] + pb[0]) > tol["sym"] or abs(pa[1] - pb[1]) > tol["sym"]:
            bad.append(f"wheels {a}/{b} not mirrored about x=0")
        if W[a]["roller_angle_sign"] != -W[b]["roller_angle_sign"]:
            bad.append(f"wheels {a}/{b} must have opposite roller_angle_sign (diagonal pairs share the sign)")
    com = m["mass_properties"]["com_xyz"]
    if abs(com[1]) > 5e-3:
        bad.append(f"COM y offset {com[1]*1000:.1f} mm (expected ~0)")
    I = np.array(m["mass_properties"]["inertia_tensor"])
    if abs(I[0, 1]) > 0.05 * np.trace(I) / 3 or abs(I[1, 2]) > 0.05 * np.trace(I) / 3:
        bad.append("inertia tensor has large Ixy/Iyz products for a left/right symmetric rover")
    for f in m["attachments"]["docking_faces"]:
        if f["geometry"] != "modelled":
            continue
        T = g.T_from_pose(f["origin"])
        ys = []
        for hn in f["hooks"]:
            h = next((h for h in m["actuated"]["hooks"] if h["name"] == hn), None)
            if h:
                piv = next(p for p in m["attachments"]["hook_pivots"] if p["name"] == h["pivot"])
                P = np.linalg.inv(T) @ g.T_from_pose(piv["origin"])
                ys.append((P[0, 3], P[1, 3]))
        if len(ys) == 2 and (abs(ys[0][0] - ys[1][0]) > tol["sym"] or abs(ys[0][1] + ys[1][1]) > tol["sym"]):
            bad.append(f"face {f['name']}: hooks not symmetric about the face centreline: {ys}")
        if len(ys) == 2:
            hs = [h for h in m["actuated"]["hooks"] if h["name"] in f["hooks"]]
            if abs(hs[0]["open_angle"] + hs[1]["open_angle"]) > 1e-6:
                bad.append(f"face {f['name']}: hook open angles are not mirrored")
    if "chassis_body" in model.parts:
        lo, hi = model.world_mesh("chassis_body", authored_state(model)).bbox()
        if abs(lo[1] + hi[1]) > tol["sym"]:
            bad.append(f"chassis_body not centred in y (min {lo[1]:.4f}, max {hi[1]:.4f})")
    rep.check("symmetry", not bad, "left/right and fore/aft symmetry holds (wheels, hooks, COM, body)", "; ".join(bad), measured=bad, tol=tol["sym"])


def check_handedness(rep, model, tol):
    st = authored_state(model)
    bad, info = [], {}
    for w in model.m["wheels"]["items"]:
        x, y = w["position"][0], w["position"][1]
        exp = -1 if x * y > 0 else 1
        s = w["roller_angle_sign"]
        if s != exp:
            bad.append(f"{w['name']}: roller_angle_sign {s} but the X convention requires {exp} at (x={x:+.3f}, y={y:+.3f})")
        a = w.get("ground_roller_axis_xy")
        if a is not None:
            ang = math.atan2(a[1], a[0])
            if (a[1] >= 0) != (s > 0):
                bad.append(f"{w['name']}: ground_roller_axis_xy {a} inconsistent with roller_angle_sign {s}")
        if abs(abs(w["roller_angle"]) - math.pi / 4) > 0.02:
            bad.append(f"{w['name']}: roller_angle {w['roller_angle']:.3f} rad is not 45 deg")
        ax = np.array(w["axis"], float)
        if abs(ax[1]) < 0.99 or ax[1] < 0:
            bad.append(f"{w['name']}: spin axis {w['axis']} must be +y (positive spin = forward)")
        # mesh
        part = model.parts.get(w["render_part"])
        if part is None:
            continue
        mesh = model.world_mesh(w["render_part"], st)
        groups = [i for i, n in enumerate(mesh.groups) if n.startswith("roller")]
        if not groups:
            info[w["name"]] = "no roller groups"
            continue
        best = None
        for gi in groups:
            pts = mesh.P[mesh.G == gi].reshape(-1, 3)
            ctr = pts.mean(0)
            if best is None or ctr[2] < best[0]:
                best = (ctr[2], pts)
        pts = best[1]
        u, sv, vt = np.linalg.svd(pts - pts.mean(0), full_matrices=False)
        axis = vt[0]
        if axis[0] < 0:
            axis = -axis
        s_mesh = 1 if axis[1] > 0 else -1
        # angle to the wheel axle (base y)
        ang = math.degrees(math.acos(min(1, abs(axis[1]))))
        info[w["name"]] = {"mesh_sign": s_mesh, "axis_xy": axis[:2].round(3).tolist(), "angle_to_axle_deg": round(ang, 1)}
        if s_mesh != exp:
            bad.append(f"{w['name']}: roller MESH at the contact patch leans s={s_mesh} (axis {axis[:2].round(3).tolist()}), X convention needs {exp}")
        if abs(ang - 45) > 6:
            bad.append(f"{w['name']}: roller mesh axis is {ang:.1f} deg to the axle, expected 45")
    rep.check("roller_handedness", not bad, "manifest signs and roller meshes follow the X convention (FL/RR -1, FR/RL +1)", "; ".join(bad), measured=info)


def check_mass(rep, model, prof, tol):
    m = model.m
    mp = m["mass_properties"]
    I = np.array(mp["inertia_tensor"], float)
    mass = mp["mass"]
    bad = []
    if I.shape != (3, 3):
        rep.fail("mass.tensor", "inertia tensor must be 3x3")
        return
    if np.abs(I - I.T).max() > 1e-6 * max(1, np.abs(I).max()):
        bad.append("inertia tensor not symmetric")
    w = np.linalg.eigvalsh((I + I.T) / 2)
    if w.min() <= 0:
        bad.append(f"inertia tensor not positive definite (eigenvalues {w.round(5).tolist()})")
    a, b, c = np.sort(w)
    if a + b < c * (1 - 1e-6):
        bad.append(f"triangle inequality violated: {a:.5f}+{b:.5f} < {c:.5f}")
    rep.check("mass.physical", not bad, f"inertia positive definite, principal moments {w.round(4).tolist()} satisfy the triangle inequalities", "; ".join(bad), measured=w.tolist())
    # box consistency
    if m["role"] == "payloads":
        return
    L, W, H = pv(prof["chassis"]["body_size_lwh"])
    Ib = np.diag([mass / 12 * (W * W + H * H), mass / 12 * (L * L + H * H), mass / 12 * (L * L + W * W)])
    F = tol["inertia_factor"]
    ratio = np.diag(I) / np.diag(Ib)
    ok = np.all(ratio > 1 / F) and np.all(ratio < F)
    rep.check("mass.box_consistency", ok, f"diag(I)/diag(I_box) = {ratio.round(2).tolist()} within a factor {F}",
              f"inertia diag differs from a {L}x{W}x{H} box of the same mass by more than a factor {F}: ratios {ratio.round(2).tolist()}", measured=ratio.tolist(), expected=np.diag(Ib).tolist())
    rp = prof["roles"][m["role"]]
    exp = pv(prof["chassis"]["base_mass"]) + pv(rp["added_mass"])
    rep.check("mass.total", abs(mass - exp) <= tol["mass_rel"] * exp, f"total mass {mass:.3f} kg = base + added ({exp:.3f})",
              f"total mass {mass:.3f} kg vs profile base+added {exp:.3f} kg (>{tol['mass_rel']*100:.0f}% off; bump the profile if CAD is right)", measured=mass, expected=exp)
    lo, hi = collision_aabb(model)
    com = np.array(mp["com_xyz"])
    inside = np.all(com >= lo - 0.01) and np.all(com <= hi + 0.01)
    rep.check("mass.com_inside", inside, "centre of mass lies inside the collision envelope", f"COM {com.tolist()} outside collision AABB {lo.round(3).tolist()}..{hi.round(3).tolist()}")
    # solid-body bound: a body of this mass inside the collision AABB cannot have a principal moment above the AABB-filling box
    ext = hi - lo
    Imax = np.diag([mass / 12 * (ext[1] ** 2 + ext[2] ** 2), mass / 12 * (ext[0] ** 2 + ext[2] ** 2), mass / 12 * (ext[0] ** 2 + ext[1] ** 2)])
    rep.check("mass.within_envelope", np.all(np.diag(I) <= 3 * np.diag(Imax) + 1e-9), "inertia no larger than the collision envelope can support",
              f"inertia diag {np.diag(I).round(4).tolist()} exceeds what the collision envelope allows ({np.diag(Imax).round(4).tolist()})")


def check_bbox(rep, model, tol):
    lo_r, hi_r = render_aabb(model)
    lo_c, hi_c = collision_aabb(model)
    d = np.concatenate([lo_r - lo_c, hi_c - hi_r])    # >0 means render sticks out further than collision (first 3 = low side)
    d_lo = lo_c - lo_r
    d_hi = hi_r - hi_c
    worst = max(np.abs(d_lo).max(), np.abs(d_hi).max())
    ok = worst <= tol["bbox"]
    names = ["x-", "y-", "z-", "x+", "y+", "z+"]
    diffs = dict(zip(names, np.concatenate([d_lo, d_hi]).round(4).tolist()))
    rep.check("bbox.render_vs_collision", ok, f"render and collision bounding boxes agree within {tol['bbox']*1000:.0f} mm (worst {worst*1000:.1f} mm)",
              f"render vs collision bounding boxes differ by up to {worst*1000:.1f} mm (tolerance {tol['bbox']*1000:.0f} mm): {diffs}",
              level_bad="WARN", measured=diffs, tol=tol["bbox"])


def check_watertight(rep, model):
    bad, neg, degen = [], [], []
    for n, p in model.parts.items():
        mesh = model.mesh(n)
        if len(mesh) == 0:
            continue
        if p.get("kind") == "quad":
            continue
        c = mesh.closedness()
        if c["unbalanced"]:
            bad.append(f"{n}({c['unbalanced']} open edges)")
        if c["degenerate"]:
            degen.append(f"{n}({c['degenerate']})")
        if mesh.signed_volume() < 0:
            neg.append(n)
    rep.check("watertight", not bad, "all solid parts are closed (every edge balanced)", f"not watertight: {bad[:8]}", level_bad="WARN", measured=bad)
    rep.check("watertight.degenerate", not degen, "no degenerate triangles", f"degenerate triangles: {degen[:6]}", level_bad="WARN")
    rep.check("winding", not neg, "solid parts have outward (positive-volume) winding", f"inside-out or mirrored meshes (negative signed volume): {neg}")


def check_actuated(rep, model, prof, tol):
    m = model.m
    bad = []
    for h in m["actuated"]["hooks"]:
        if h["part"] not in model.parts:
            bad.append(f"hook {h['name']}: part '{h['part']}' missing")
            continue
        j = model.parts[h["part"]].get("joint")
        if not j or j["type"] not in ("revolute", "continuous"):
            bad.append(f"hook {h['name']}: part has no revolute joint")
            continue
        lo, hi = j["limits"]
        for k in ("closed_angle", "open_angle"):
            if not (lo - 1e-9 <= h[k] <= hi + 1e-9):
                bad.append(f"hook {h['name']}: {k} {h[k]} outside joint limits {j['limits']}")
        if abs(h["open_angle"] - h["closed_angle"]) < 0.1:
            bad.append(f"hook {h['name']}: open/closed angles too close")
        if not any(p["name"] == h["pivot"] for p in m["attachments"]["hook_pivots"]):
            bad.append(f"hook {h['name']}: pivot frame '{h['pivot']}' missing")
    for p in m["render"]["parts"]:
        j = p.get("joint")
        if j and j["type"] in ("revolute", "prismatic") and ("limits" not in j or "axis" not in j):
            bad.append(f"part {p['name']}: joint needs axis and limits")
    rep.check("actuated", not bad, "actuated parts reference existing parts/pivots with consistent limits", "; ".join(bad), measured=bad)


# ------------------------------------------------------------------ mating
def docking_state(model):
    st = authored_state(model)
    lift = model.m["actuated"].get("lift")
    if lift:
        st[lift["part"]] = lift["docking_value"]
    return st


def mate_check(rep, A, fa, B, fb, prof, tol, label=None):
    label = label or f"{A.m['asset_id']}.{fa} <-> {B.m['asset_id']}.{fb}"
    stA, stB = docking_state(A), docking_state(B)
    TA = np.eye(4)
    FA, FB = A.face_T(fa), B.face_T(fb)
    TB = TA @ FA @ g.make_T(g.rotz(math.pi)) @ g.inv_T(FB)
    out = {"label": label, "A": A.m["asset_id"], "B": B.m["asset_id"], "face_A": fa, "face_B": fb}
    # protrusion beyond the mating plane
    def protrusion(model, face_T, T_w, st):
        if "chassis_body" not in model.parts:
            return 0.0
        mesh = model.world_mesh("chassis_body", st, T_w)
        pts = mesh.P.reshape(-1, 3)
        local = (pts - (T_w @ face_T)[:3, 3]) @ (T_w @ face_T)[:3, :3]
        return float(local[:, 0].max())
    pA, pB = protrusion(A, FA, TA, stA), protrusion(B, FB, TB, stB)
    gap = -(pA + pB)
    out["body_protrusion_beyond_face_mm"] = [round(pA * 1000, 2), round(pB * 1000, 2)]
    out["face_plane_gap_mm"] = round(gap * 1000, 2)
    # overlaps
    def group(model, T_w, st):
        body, hooks, other = g.Mesh(), g.Mesh(), g.Mesh()
        for n, p in model.parts.items():
            if not p.get("default_visible", True) or p.get("role_in_assembly") in SKIP_ROLES:
                continue
            mesh = model.world_mesh(n, st, T_w)
            r = p.get("role_in_assembly")
            if r == "hook":
                hooks.append(mesh)
            elif n == "chassis_body":
                body.append(mesh)
            else:
                other.append(mesh)
        return body, hooks, other
    bA, hA, oA = group(A, TA, stA)
    bB, hB, oB = group(B, TB, stB)
    cell = tol["cell"]
    ov = {
        "body_vs_body": g.overlap_volume(bA, bB, cell),
        "hooks_vs_hooks": g.overlap_volume(hA, hB, cell),
        "hooksA_vs_bodyB": g.overlap_volume(hA, bB, cell),
        "hooksB_vs_bodyA": g.overlap_volume(hB, bA, cell),
        "other_vs_all": g.overlap_volume(oA, bB, cell) + g.overlap_volume(oA, oB, cell) + g.overlap_volume(oA, hB, cell)
                        + g.overlap_volume(oB, bA, cell) + g.overlap_volume(oB, hA, cell),
    }
    out["overlap_mm3"] = {k: round(v * 1e9, 1) for k, v in ov.items()}
    # engagement
    def pins_world(model, face, T_w):
        f = model.face(face)
        T = T_w @ model.face_T(face)
        return [(p["name"], (T @ np.append(p["position"], 1))[:3]) for p in f["catch_pins"]]
    def hooks_world(model, face, T_w, st):
        res = []
        for h in model.m["actuated"]["hooks"]:
            if h["face"] != face:
                continue
            piv = next(p for p in model.m["attachments"]["hook_pivots"] if p["name"] == h["pivot"])
            Tp = model.frame_T(h["part"], {**st, h["part"]: h["closed_angle"]})
            res.append((h["name"], (T_w @ Tp @ np.append(h["engage_point"], 1))[:3]))
        return res
    eng = []
    for (hn, hp) in hooks_world(A, fa, TA, stA):
        d = min((np.linalg.norm(hp - pp), pn) for pn, pp in pins_world(B, fb, TB)) if pins_world(B, fb, TB) else (float("inf"), "-")
        eng.append({"hook": hn, "nearest_pin": d[1], "distance_mm": round(d[0] * 1000, 3)})
    for (hn, hp) in hooks_world(B, fb, TB, stB):
        d = min((np.linalg.norm(hp - pp), pn) for pn, pp in pins_world(A, fa, TA)) if pins_world(A, fa, TA) else (float("inf"), "-")
        eng.append({"hook": hn, "nearest_pin": d[1], "distance_mm": round(d[0] * 1000, 3)})
    out["hook_engagement"] = eng
    # guide
    gA, gB = A.face(fa).get("guide", {}), B.face(fb).get("guide", {})
    ax_err = math.degrees(math.acos(np.clip(-(TA @ FA)[:3, 0] @ (TB @ FB)[:3, 0] * -1, -1, 1)))
    org_off = np.linalg.norm((TA @ FA)[:3, 3] - (TB @ FB)[:3, 3])
    out["guide"] = {"axis_antiparallel_error_deg": round(abs(180 - math.degrees(math.acos(np.clip((TA @ FA)[:3, 0] @ (TB @ FB)[:3, 0], -1, 1)))), 4),
                    "origin_offset_mm": round(org_off * 1000, 4),
                    "chamfer_width_mm": [round(gA.get("chamfer_width", 0) * 1000, 2), round(gB.get("chamfer_width", 0) * 1000, 2)],
                    "profile_guide_half_width_mm": round(pv(prof["docking"]["guide_half_width"]) * 1000, 2)}
    etol = tol["engage"] if tol["engage"] is not None else pv(prof["docking"]["hook_position_tolerance"])
    worst = max([e["distance_mm"] for e in eng], default=float("inf")) / 1000
    out["summary"] = (f"gap {out['face_plane_gap_mm']} mm, overlaps(mm3) {out['overlap_mm3']}, worst hook-to-pin {worst*1000:.2f} mm, "
                      f"guide axis err {out['guide']['axis_antiparallel_error_deg']} deg")
    rep.mating.append(out)
    tag = f"mating.{fa}_{fb}"
    rep.check(tag + ".gap", -tol["gap_under"] <= gap <= tol["gap_over"], f"face-plane gap {gap*1000:.2f} mm ({label})",
              f"face-plane gap {gap*1000:.2f} mm outside [-{tol['gap_under']*1000:.0f}, {tol['gap_over']*1000:.0f}] mm ({label}); negative = bodies overlap", measured=gap)
    rep.check(tag + ".body_overlap", ov["body_vs_body"] <= tol["vol_body"], f"no body interpenetration ({ov['body_vs_body']*1e9:.0f} mm3)",
              f"bodies interpenetrate by {ov['body_vs_body']*1e9:.0f} mm3 ({label})", measured=ov["body_vs_body"])
    hv = max(ov["hooks_vs_hooks"], ov["hooksA_vs_bodyB"], ov["hooksB_vs_bodyA"])
    rep.check(tag + ".hook_clearance", hv <= tol["vol_hook"], f"hooks clear of the opposing hooks/body/pins (max overlap {hv*1e9:.0f} mm3)",
              f"hooks collide with the opposing face by {hv*1e9:.0f} mm3 ({label}): {out['overlap_mm3']}", measured=hv)
    rep.check(tag + ".hook_engagement", worst <= etol, f"every hook engage point is within {worst*1000:.2f} mm of an opposing catch pin (tolerance {etol*1000:.1f} mm)",
              f"hook engage points miss the opposing catch pins by up to {worst*1000:.2f} mm (tolerance {etol*1000:.1f} mm) ({label})", measured=worst, tol=etol)
    rep.check(tag + ".guide_alignment", out["guide"]["axis_antiparallel_error_deg"] < 0.5 and out["guide"]["origin_offset_mm"] < 1.0, "guide axes are collinear and antiparallel",
              f"guide axes misaligned: {out['guide']}")
    gmin = min(out["guide"]["chamfer_width_mm"]) / 1000
    rep.check(tag + ".guide_width", gmin >= pv(prof["docking"]["guide_half_width"]) - 1e-6,
              f"guide chamfer {gmin*1000:.1f} mm covers the profile guide half-width", f"guide chamfer {gmin*1000:.1f} mm is narrower than the profile guide_half_width {pv(prof['docking']['guide_half_width'])*1000:.1f} mm",
              level_bad="WARN", measured=gmin)
    rep.check(tag + ".other_parts", ov["other_vs_all"] <= tol["vol_other"], "no other parts collide when mated (docking pose)",
              f"non-dock parts collide with the neighbour by {ov['other_vs_all']*1e9:.0f} mm3 ({label})", level_bad="WARN", measured=ov["other_vs_all"])


def default_mates(rep, model, prof, tol):
    faces = [f["name"] for f in model.m["attachments"]["docking_faces"] if f["geometry"] == "modelled"]
    if "front" in faces and "rear" in faces:
        mate_check(rep, model, "front", model, "rear", prof, tol)
        mate_check(rep, model, "front", model, "front", prof, tol)
    elif len(faces) >= 1:
        mate_check(rep, model, faces[0], model, faces[0], prof, tol)
    else:
        rep.fail("mating.faces", "no modelled docking faces to mate")


# ------------------------------------------------------------------ driver
def validate(manifest_path, profile_path=None, tol_over=None, mates=None, do_mate=True):
    tol = dict(TOL)
    tol.update(tol_over or {})
    rep = Report(os.path.abspath(manifest_path))
    try:
        man = json.load(open(manifest_path))
    except Exception as e:  # noqa: BLE001
        rep.fail("schema", f"cannot read JSON: {e}")
        return rep
    ok = check_schema(rep, man)
    if not ok:
        # schema failure: only continue if the structure is usable
        need = ("render", "collision", "mass_properties", "base_frame", "role", "profile")
        if not all(k in man for k in need):
            return rep
    model = RoverModel(manifest_path, man)
    pp = profile_path or resolve(man.get("profile", {}).get("path", ""), model.dir)
    if not pp or not os.path.exists(pp):
        rep.fail("files.profile", f"profile '{man.get('profile', {}).get('path')}' not found; pass --profile")
        return rep
    prof = json.load(open(pp))
    rep.ok("files.profile", f"profile {prof.get('profile_id')} loaded")
    if man.get("profile", {}).get("profile_id") != prof.get("profile_id"):
        rep.fail("profile.id", f"manifest refers to profile_id {man['profile'].get('profile_id')} but profile file is {prof.get('profile_id')} (bump both together)")
    for s in ("render", "collision", "mass_properties", "wheels", "sensors", "attachments", "actuated", "payloads"):
        if s in man and man[s].get("provenance") not in ("provisional", "cad", "measured"):
            rep.fail(f"provenance.{s}", f"section {s} has invalid provenance")
    safe(rep, "files.sources", check_provenance_and_sources, rep, model, model.dir)
    safe(rep, "files", check_files, rep, model)
    safe(rep, "scale", check_scale, rep, model, prof, tol)
    safe(rep, "profile", check_profile, rep, model, prof, tol)
    safe(rep, "mass", check_mass, rep, model, prof, tol)
    safe(rep, "bbox", check_bbox, rep, model, tol)
    safe(rep, "watertight", check_watertight, rep, model)
    if man["role"] != "payloads":
        safe(rep, "wheels_ground", check_wheels_ground, rep, model, prof, tol)
        safe(rep, "symmetry", check_symmetry, rep, model, tol)
        safe(rep, "roller_handedness", check_handedness, rep, model, tol)
        safe(rep, "actuated", check_actuated, rep, model, prof, tol)
        if do_mate:
            if mates:
                for spec in mates:
                    (pa, fa), (pb, fb) = (s.rsplit(":", 1) for s in spec.split(","))
                    A = model if os.path.abspath(pa) == model.path else RoverModel(pa)
                    B = model if os.path.abspath(pb) == model.path else RoverModel(pb)
                    safe(rep, "mating", mate_check, rep, A, fa, B, fb, prof, tol)
            else:
                safe(rep, "mating", default_mates, rep, model, prof, tol)
    return rep


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("manifests", nargs="+")
    ap.add_argument("--profile")
    ap.add_argument("--report", help="JSON report path (a list of reports if several manifests)")
    ap.add_argument("--mate", action="append", help="'A.manifest.json:front,B.manifest.json:rear' (repeatable)")
    ap.add_argument("--no-mate", action="store_true")
    ap.add_argument("--tol", nargs="*", default=[], help="key=value overrides, e.g. bbox=0.05")
    a = ap.parse_args(argv)
    over = {}
    for kv in a.tol:
        k, v = kv.split("=")
        over[k] = float(v)
    reps = [validate(p, a.profile, over, a.mate, not a.no_mate) for p in a.manifests]
    for r in reps:
        print(r.summary())
    if a.report:
        os.makedirs(os.path.dirname(os.path.abspath(a.report)), exist_ok=True)
        json.dump([r.to_json() for r in reps] if len(reps) > 1 else reps[0].to_json(), open(a.report, "w"), indent=1)
    return 1 if any(r.overall() == "FAIL" for r in reps) else 0


if __name__ == "__main__":
    sys.exit(main())
