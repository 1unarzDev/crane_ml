#!/usr/bin/env python3
"""Render the v1 assets from viewpoints matching the concept images, plus dock close-ups, docked pairs and
reference-vs-render comparison sheets. Uses the numpy rasteriser of ../render_preview.py (no GPU, no Unity).

    python render_v1.py --assets ~/.cache/crane-fleet/assets_v1 --out <dir> [--sheets <dir for JPEG sheets>] [--only roles|pairs|sheets]
"""
import argparse
import json
import math
import os
import sys

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.dirname(HERE))
sys.path.insert(0, HERE)
import geom as g  # noqa: E402
import render_preview as RP  # noqa: E402
from rover_model import RoverModel  # noqa: E402

REF = {"scout": "modular1.jpg", "lifter": "modular2.jpg", "cargo": "modular3.jpg"}
# camera direction (azimuth from +x towards +y, elevation) estimated from the reference images
VIEW = {"scout": (-27.0, 27.0, 30.0), "lifter": (-30.0, 28.0, 30.0), "cargo": (-30.0, 25.0, 30.0)}
BG = (0.86, 0.86, 0.88)


def cam_for(objs, az, el, fov, scale=1.15, target_shift=(0, 0, 0)):
    lo, hi = RP.bbox_of(objs)
    c = (lo + hi) / 2 + np.asarray(target_shift, float)
    R = float(np.linalg.norm(hi - lo)) / 2
    d = np.array([math.cos(math.radians(el)) * math.cos(math.radians(az)), math.cos(math.radians(el)) * math.sin(math.radians(az)), math.sin(math.radians(el))])
    dist = R / math.tan(math.radians(fov / 2)) * scale
    return RP.Camera(c + d * dist, c, fov_deg=fov), c


def render(objs, cam, size, gz, c, ss=2):
    RP_bg = RP.render.__defaults__
    return RP.render(objs, cam, size, ss, bg=BG, ground_z=gz, ground_half=1.6, ground_center=(c[0], c[1]))


def ortho(objs, view, size, gz):
    lo, hi = RP.bbox_of(objs)
    c = (lo + hi) / 2
    ext = hi - lo
    cams = {"top": RP.Camera(c + [0, 0, 5], c, up=(1, 0, 0), ortho_scale=1), "side": RP.Camera(c + [0, 5, 0], c, ortho_scale=1),
            "front": RP.Camera(c + [5, 0, 0], c, ortho_scale=1)}
    cam = cams[view]
    w, h = {"top": (ext[1], ext[0]), "side": (ext[0], ext[2]), "front": (ext[1], ext[2])}[view]
    s = min(size[0] * 0.86 / w, size[1] * 0.86 / h) * 2
    cam2 = RP.Camera(cam.eye, cam.eye + cam.f, cam.u, ortho_scale=s)
    cam2.eye, cam2.f, cam2.r, cam2.u = cam.eye, cam.f, cam.r, cam.u
    return RP.render(objs, cam2, size, 2, bg=BG, ground_z=gz if view != "top" else None, ground_center=(c[0], c[1]))


def label(img, text, size=22):
    from PIL import Image, ImageDraw, ImageFont
    im = Image.fromarray(img) if not hasattr(img, "save") else img
    d = ImageDraw.Draw(im)
    try:
        f = ImageFont.truetype("/usr/share/fonts/noto/NotoSans-Regular.ttf", size)
    except Exception:  # noqa: BLE001
        f = ImageFont.load_default()
    d.rectangle([0, 0, im.width, size + 10], fill=(255, 255, 255))
    d.text((8, 4), text, fill=(0, 0, 0), font=f)
    return im


def save(path, arr):
    from PIL import Image
    os.makedirs(os.path.dirname(path), exist_ok=True)
    (Image.fromarray(arr) if not hasattr(arr, "save") else arr).save(path)


def box_obj(lo, hi, color, T=None, name="box"):
    m = g.box_mesh(lo, hi, name, "b")
    if T is not None:
        m = m.transformed(T)
    m.smooth_normals(30)
    return (m, {name: {"base_color": list(color), "specular": 0.1}}, "")


def load(assets):
    return {r: RoverModel(os.path.join(assets, r, f"{r}.manifest.json")) for r in ("scout", "lifter", "cargo")}


def concept_state(m, role):
    st = {}
    if role == "scout":
        names = {"arm_joint_1": "arm_link_1", "arm_joint_2": "arm_link_2", "arm_joint_3": "arm_link_3", "arm_joint_4": "arm_link_4",
                 "arm_joint_5": "arm_link_5", "arm_gripper_joint": "arm_gripper_jaw"}
        for k, v in m.m["actuated"]["arm_proxy"]["concept_pose"].items():
            st[names[k]] = v
    if role == "lifter":
        st["fork_carriage"] = 0.0
        for f in m.m["actuated"]["gripper"]["fingers"]:
            st[f["part"]] = f["travel"][1] * 0.45
    return st


def role_views(models, out, size=(1100, 760)):
    res = {}
    for role, m in models.items():
        gz = m.m["base_frame"]["ground_z"]
        st = concept_state(m, role)
        objs = RP.collect(m, None, st)
        az, el, fov = VIEW[role]
        cam, c = cam_for(objs, az, el, fov)
        im = render(objs, cam, size, gz, c)
        save(os.path.join(out, f"{role}_ref_view.png"), im)
        res[role] = im
        folded = RP.collect(m, None, {})
        cam2, c2 = cam_for(folded, az, el, fov)
        save(os.path.join(out, f"{role}_authored.png"), render(folded, cam2, size, gz, c2))
        for v in ("top", "side", "front"):
            save(os.path.join(out, f"{role}_{v}.png"), ortho(folded, v, (900, 600), gz))
        # back-right view (faces not visible in the concept)
        cam3, c3 = cam_for(folded, az + 180, el, fov)
        save(os.path.join(out, f"{role}_rear_right.png"), render(folded, cam3, size, gz, c3))
        # dock close-ups
        faces = [f["name"] for f in m.m["attachments"]["docking_faces"] if f["geometry"] == "modelled"]
        for face in faces[:2] + (["front"] if role == "lifter" else []):
            T = m.face_T(face)
            o, n, yv, zv = T[:3, 3], T[:3, 0], T[:3, 1], T[:3, 2]
            for lab, sts in (("closed", {}), ("open", {h["part"]: h["open_angle"] for h in m.m["actuated"]["hooks"]})):
                s2 = dict(st if role == "lifter" else {})
                s2.update(sts)
                ob = RP.collect(m, None, s2)
                camd = RP.Camera(o + n * 0.30 + yv * 0.10 + zv * 0.12, o - n * 0.01, fov_deg=36)
                save(os.path.join(out, f"{role}_dock_{face}_{lab}.png"), render(ob, camd, (900, 640), gz, o))
        # internals visible
        ob = RP.collect(m, None, {}, show_hidden=True, skip=("chassis_body",))
        cam4, c4 = cam_for(ob, az, el + 15, fov)
        save(os.path.join(out, f"{role}_internals.png"), render(ob, cam4, (900, 640), gz, c4))
    return res


def mate_T(A, fa, B, fb):
    return A.face_T(fa) @ g.make_T(g.rotz(math.pi)) @ g.inv_T(B.face_T(fb))


def pair_views(models, assets, out, size=(1200, 760)):
    pl = RoverModel(os.path.join(assets, "payloads", "payloads.manifest.json"))
    pairs = [("cargo", "front", "cargo", "rear", "box_long"), ("lifter", "right", "cargo", "left", None), ("scout", "front", "cargo", "rear", None),
             ("lifter", "rear", "cargo", "front", None)]
    for ra, fa, rb, fb, payload in pairs:
        A, B = models[ra], models[rb]
        TB = mate_T(A, fa, B, fb)
        sa = concept_state(A, ra) if ra == "lifter" else {}
        objs = RP.collect(A, None, sa) + RP.collect(B, TB, {})
        if payload:
            it = next(i for i in pl.m["payloads"]["items"] if i["name"] == payload)
            dtop = B.m["base_frame"]["ground_z"] + json.load(open(os.path.join(os.path.dirname(HERE), "..", "spec", "fleet_profile_v1.json")))["roles"]["cargo"]["deck_height"]["value"]
            mid = (np.eye(4)[:3, 3] + TB[:3, 3]) / 2
            Tp = g.make_T(None, [mid[0], mid[1], dtop])
            objs += RP.collect(pl, Tp, {}, skip=[p for p in pl.parts if p != f"payload_{payload}"])
        gz = A.m["base_frame"]["ground_z"]
        cam, c = cam_for(objs, 30, 28, 30, 1.05)
        tag = f"docked_{ra}_{fa}_to_{rb}_{fb}"
        save(os.path.join(out, tag + "_persp.png"), render(objs, cam, size, gz, c))
        save(os.path.join(out, tag + "_top.png"), ortho(objs, "top", (1000, 640), gz))
        # cutaway at the dock: everything above the face centre removed, view from above-side
        F = A.face_T(fa)
        zc = float(F[2, 3] + 0.002)
        ob = RP.collect(A, None, sa, clip_z=zc) + RP.collect(B, TB, {}, clip_z=zc)
        o, n, yv = F[:3, 3], F[:3, 0], F[:3, 1]
        camc = RP.Camera(o + np.array([0, 0, 0.16]) + yv * 0.10 - n * 0.02, o + np.array([0, 0, -0.005]), fov_deg=34)
        save(os.path.join(out, tag + "_cutaway.png"), render(ob, camc, (1000, 700), gz, o))
        # section through the hook plane (y of A's hook) seen from the side
        hk = [h for h in A.m["actuated"]["hooks"] if h["face"] == fa]
        if hk:
            P = A.frame_T(hk[0]["part"], {})[:3, 3]
            camh = RP.Camera(P + yv * 0.0 + np.cross(np.array([0, 0, 1.0]), n) * -0.0 + np.array([0, 0, 0]) + (np.cross(n, [0, 0, 1]) * 0.25), P, fov_deg=18)
            ob2 = []
            nrm = np.cross(n, [0, 0, 1.0])
            nrm = nrm / np.linalg.norm(nrm)
            Rn = g.R_align_z(nrm)                 # z. = (p - P).nrm; keep z. <= 0 (far side of the hook plane)
            Tf = g.make_T(Rn.T, -Rn.T @ P)
            for mdl, Tw, s_ in ((A, None, sa), (B, TB, {})):
                for nm, p in mdl.parts.items():
                    if not p.get("default_visible", True):
                        continue
                    mm = mdl.world_mesh(nm, s_, Tw).transformed(Tf)
                    sub = RP.clip_below(mm, 0.0).transformed(g.inv_T(Tf))
                    if len(sub):
                        sub.smooth_normals(40)
                        ob2.append((sub, mdl.m["render"]["materials"], mdl.dir))
            save(os.path.join(out, tag + "_hook_section.png"), render(ob2, camh, (900, 640), gz, P))


def payload_views(models, assets, out, size=(1100, 760)):
    """lifter holding the 80 mm concept box; cargo with box_small."""
    L = models["lifter"]
    st = concept_state(L, "lifter")
    st["fork_carriage"] = 0.02
    gr = L.m["actuated"]["gripper"]
    omax = gr["opening_range"][1]
    q = (omax - 0.08) / 2
    for f in gr["fingers"]:
        st[f["part"]] = q
    objs = RP.collect(L, None, st)
    Tc = L.frame_T("fork_carriage", st)
    x0 = Tc[0, 3] + 0.022
    zb = Tc[2, 3] - 0.013 + 0.001
    objs.append(box_obj([x0, -0.04, zb], [x0 + 0.08, 0.04, zb + 0.08], (0.72, 0.55, 0.34), name="cardboard"))
    cam, c = cam_for(objs, 30, 30, 30)
    save(os.path.join(out, "lifter_box_pickup.png"), render(objs, cam, size, L.m["base_frame"]["ground_z"], c))
    # lifter placing box_small on a cargo deck, side-on, at full stroke (D89)
    Cg = models["cargo"]
    pl_ = RoverModel(os.path.join(assets, "payloads", "payloads.manifest.json"))
    st2 = {"fork_carriage": L.m["actuated"]["lift"]["range"][1]}
    for f in gr["fingers"]:
        st2[f["part"]] = (omax - 0.20) / 2
    Tc0 = L.frame_T("fork_carriage", {"fork_carriage": 0.0})
    bx = Tc0[0, 3] + 0.021 + 0.15
    TL = g.make_T(g.rotz(-math.pi / 2), [0.0, bx, 0.0])
    Tcw = TL @ L.frame_T("fork_carriage", st2)
    zb = Tcw[2, 3] - 0.018
    Tbox = TL @ g.make_T(None, [bx, 0.0, 0.0]) @ g.make_T(g.rotz(0.0))
    Tbox[2, 3] = zb
    objs = RP.collect(L, TL, st2) + RP.collect(Cg, None, {}) + RP.collect(pl_, Tbox, {}, skip=[p for p in pl_.parts if p != "payload_box_small"])
    cam, c = cam_for(objs, -40, 24, 30)
    save(os.path.join(out, "lifter_places_box_small_on_cargo.png"), render(objs, cam, size, L.m["base_frame"]["ground_z"], c))

    pl = RoverModel(os.path.join(assets, "payloads", "payloads.manifest.json"))
    dtop = Cg.m["base_frame"]["ground_z"] + 0.150
    objs = RP.collect(Cg, None, {}) + RP.collect(pl, g.make_T(None, [0.02, 0.0, dtop]), {}, skip=[p for p in pl.parts if p != "payload_box_small"])
    cam, c = cam_for(objs, 30, 26, 30)
    save(os.path.join(out, "cargo_loaded_box_small.png"), render(objs, cam, size, Cg.m["base_frame"]["ground_z"], c))


def sheets(models, out, sheet_dir, repo_ref_dir, max_kb=300):
    from PIL import Image
    os.makedirs(sheet_dir, exist_ok=True)
    for role in ("scout", "lifter", "cargo"):
        ref = Image.open(os.path.join(repo_ref_dir, REF[role])).convert("RGB")
        ren = Image.open(os.path.join(out, f"{role}_ref_view.png")).convert("RGB")
        extra = [os.path.join(out, f) for f in ({"scout": ["scout_authored.png", "scout_dock_front_closed.png"],
                                                  "lifter": ["lifter_places_box_small_on_cargo.png", "lifter_dock_left_closed.png"],
                                                  "cargo": ["cargo_loaded_box_small.png", "cargo_dock_front_closed.png"]}[role])]
        H = 520
        ref_s = ref.resize((int(ref.width * H / ref.height), H))
        ren_s = ren.resize((int(ren.width * H / ren.height), H))
        sm = [Image.open(p).convert("RGB") for p in extra if os.path.exists(p)]
        sm = [s.resize((int(s.width * (H // 2) / s.height), H // 2)) for s in sm]
        W = ref_s.width + ren_s.width + (max(s.width for s in sm) if sm else 0)
        sheet = Image.new("RGB", (W, H + 34), (255, 255, 255))
        sheet.paste(label(ref_s.copy(), f"concept {REF[role]}"), (0, 34))
        sheet.paste(label(ren_s.copy(), f"v1 CAD render ({role}, concept-matched view)"), (ref_s.width, 34))
        y = 34
        for s, p in zip(sm, extra):
            sheet.paste(label(s.copy(), os.path.basename(p)[:-4], 16), (ref_s.width + ren_s.width, y))
            y += s.height
        label(sheet, f"{role}: concept (left) vs v1 parametric CAD (right) - proportions estimated from the image, scale from COTS parts", 20)
        q = 85
        path = os.path.join(sheet_dir, f"compare_{role}.jpg")
        while True:
            sheet.save(path, quality=q, optimize=True)
            if os.path.getsize(path) <= max_kb * 1024 or q < 30:
                break
            if q > 50:
                q -= 10
            else:
                sheet = sheet.resize((int(sheet.width * 0.85), int(sheet.height * 0.85)))
        print("sheet", path, os.path.getsize(path) // 1024, "KB")


def main(argv=None):
    ap = argparse.ArgumentParser()
    ap.add_argument("--assets", default=os.path.expanduser("~/.cache/crane-fleet/assets_v1"))
    ap.add_argument("--out", required=True)
    ap.add_argument("--sheets", default=None)
    ap.add_argument("--only", choices=["roles", "pairs", "payloads", "sheets"], default=None)
    a = ap.parse_args(argv)
    models = load(a.assets)
    if a.only in (None, "roles"):
        role_views(models, a.out)
    if a.only in (None, "payloads"):
        payload_views(models, a.assets, a.out)
    if a.only in (None, "pairs"):
        pair_views(models, a.assets, a.out)
    if a.sheets and a.only in (None, "sheets", "roles"):
        sheets(models, a.out, a.sheets, os.path.join(HERE, "images", "ref"))
    print("renders in", a.out)


if __name__ == "__main__":
    main()
