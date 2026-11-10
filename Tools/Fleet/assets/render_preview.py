#!/usr/bin/env python3
"""Software rasteriser + matplotlib contact sheets for judging meshes before Unity.

    python render_preview.py [--assets DIR] [--out DIR]

Writes per role: <role>_persp.png, <role>_top.png, <role>_side.png, <role>_front.png, <role>_dock_closeup.png,
<role>_sheet.png (all views), and docked_*.png (two rovers face to face). Hidden parts (arm envelope, side
markers) are drawn only in the *_envelope.png variant.
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
from rover_model import RoverModel  # noqa: E402

DEFAULT_ASSETS = os.path.expanduser("~/.cache/crane-fleet/assets_v0")
LIGHTS = [(np.array([0.4, -0.5, 0.8]), 0.65), (np.array([-0.6, 0.4, 0.35]), 0.25)]
AMBIENT = 0.32


def look_at(eye, target, up=(0, 0, 1)):
    eye, target, up = (np.asarray(v, float) for v in (eye, target, up))
    f = target - eye
    f /= np.linalg.norm(f)
    r = np.cross(f, up)
    r /= np.linalg.norm(r)
    u = np.cross(r, f)
    return eye, f, r, u


class Camera:
    def __init__(self, eye, target, up=(0, 0, 1), fov_deg=None, ortho_scale=None):
        self.eye, self.f, self.r, self.u = look_at(eye, target, up)
        self.fov, self.scale = fov_deg, ortho_scale  # scale: pixels per metre at 1x

    def project(self, P, W, H, focal=None):
        d = P - self.eye
        xc, yc, zc = d @ self.r, d @ self.u, d @ self.f
        if self.scale is not None:
            s = self.scale
            return np.stack([W / 2 + s * xc, H / 2 - s * yc, zc], -1), np.ones_like(zc)
        fpx = (W / 2) / math.tan(math.radians(self.fov) / 2)
        zs = np.maximum(zc, 1e-6)
        return np.stack([W / 2 + fpx * xc / zs, H / 2 - fpx * yc / zs, zc], -1), 1.0 / zs


def load_tex(path, cache={}):
    if path not in cache:
        from PIL import Image
        cache[path] = np.array(Image.open(path).convert("RGB"))
    return cache[path]


def clip_below(m, zc):
    """Cut a mesh at z = zc and keep what lies below (no cap; the renderer is two-sided)."""
    tris, uvs, mats, grps = [], [], [], []
    for t in range(len(m)):
        P, UV = m.P[t], m.UV[t]
        if P[:, 2].max() <= zc:
            tris.append(P); uvs.append(UV); mats.append(m.M[t]); grps.append(m.G[t]); continue
        if P[:, 2].min() >= zc:
            continue
        poly, puv = [], []
        for i in range(3):
            a, b, ua, ub = P[i], P[(i + 1) % 3], UV[i], UV[(i + 1) % 3]
            ina, inb = a[2] <= zc, b[2] <= zc
            if ina:
                poly.append(a); puv.append(ua)
            if ina != inb:
                f = (zc - a[2]) / (b[2] - a[2])
                poly.append(a + f * (b - a)); puv.append(ua + f * (ub - ua))
        for k in range(1, len(poly) - 1):
            tris.append(np.array([poly[0], poly[k], poly[k + 1]])); uvs.append(np.array([puv[0], puv[k], puv[k + 1]]))
            mats.append(m.M[t]); grps.append(m.G[t])
    o = m.copy()
    o.P = np.array(tris).reshape(-1, 3, 3); o.UV = np.array(uvs).reshape(-1, 3, 2)
    o.M = np.array(mats, int); o.G = np.array(grps, int); o.S = np.zeros(len(tris), int); o.N = None
    return o


def _subset(m, keep):
    o = m.copy()
    o.P, o.UV, o.M, o.G, o.S = m.P[keep], m.UV[keep], m.M[keep], m.G[keep], m.S[keep]
    o.N = None if m.N is None else m.N[keep]
    return o


def collect(model, T_world=None, state=None, show_hidden=False, skip=(), clip_z=None):
    objs = []
    for name, p in model.parts.items():
        if name in skip:
            continue
        if not p.get("default_visible", True) and not show_hidden:
            continue
        m = model.world_mesh(name, state or {}, T_world)
        if clip_z is not None and len(m):
            m = clip_below(m, clip_z)
        if m.N is None or len(m) == 0:
            m.smooth_normals(40)
        objs.append((m, model.m["render"]["materials"], model.dir))
    return objs


def render(objs, cam, size=(900, 700), ss=2, bg=(0.93, 0.94, 0.96), ground_z=None, ground_half=2.4, ground_center=(0, 0)):
    W, H = size[0] * ss, size[1] * ss
    zbuf = np.full((H, W), np.inf, np.float32)
    img = np.zeros((H, W, 3), np.float32)
    img[:] = bg
    items = list(objs)
    if ground_z is not None:
        cx, cy = ground_center
        h = ground_half
        n = 36
        xs = np.linspace(cx - h, cx + h, n + 1)
        ys = np.linspace(cy - h, cy + h, n + 1)
        tri = []
        for i in range(n):
            for j in range(n):
                p00, p10, p11, p01 = ([xs[i], ys[j], ground_z], [xs[i + 1], ys[j], ground_z], [xs[i + 1], ys[j + 1], ground_z], [xs[i], ys[j + 1], ground_z])
                tri += [[p00, p10, p11], [p00, p11, p01]]
        gm = g.Mesh.from_tris(np.array(tri), "__ground__")
        gm.smooth_normals()
        items.insert(0, (gm, {"__ground__": {"base_color": [0.80, 0.82, 0.80]}}, ""))
    trans = []
    for mesh, mats, mdir in items:
        for pass_ in (0, 1):
            ids_all = np.arange(len(mesh))
            alpha = np.array([mats.get(mesh.mats[k], {}).get("alpha", 1.0) for k in mesh.M]) if len(mesh) else np.zeros(0)
            sel = ids_all[(alpha >= 0.999) if pass_ == 0 else (alpha < 0.999)]
            if len(sel) == 0:
                continue
            if pass_ == 1:
                trans.append((mesh, mats, mdir, sel))
                continue
            _draw(img, zbuf, mesh, mats, mdir, sel, cam, W, H, True)
    for mesh, mats, mdir, sel in trans:
        _draw(img, zbuf, mesh, mats, mdir, sel, cam, W, H, False)
    out = img.reshape(size[1], ss, size[0], ss, 3).mean((1, 3))
    return (np.clip(out, 0, 1) * 255).astype(np.uint8)


def _draw(img, zbuf, mesh, mats, mdir, sel, cam, W, H, write_z):
    scr, invw = cam.project(mesh.P, W, H)
    for t in sel:
        s = scr[t]
        if np.any(s[:, 2] <= 1e-4):
            continue
        x0, x1 = int(max(math.floor(s[:, 0].min()), 0)), int(min(math.ceil(s[:, 0].max()), W - 1))
        y0, y1 = int(max(math.floor(s[:, 1].min()), 0)), int(min(math.ceil(s[:, 1].max()), H - 1))
        if x1 < x0 or y1 < y0:
            continue
        a, b, c = s[0, :2], s[1, :2], s[2, :2]
        den = (b[1] - c[1]) * (a[0] - c[0]) + (c[0] - b[0]) * (a[1] - c[1])
        if abs(den) < 1e-12:
            continue
        xs, ys = np.meshgrid(np.arange(x0, x1 + 1) + 0.5, np.arange(y0, y1 + 1) + 0.5)
        l1 = ((b[1] - c[1]) * (xs - c[0]) + (c[0] - b[0]) * (ys - c[1])) / den
        l2 = ((c[1] - a[1]) * (xs - c[0]) + (a[0] - c[0]) * (ys - c[1])) / den
        l3 = 1 - l1 - l2
        inside = (l1 >= -1e-6) & (l2 >= -1e-6) & (l3 >= -1e-6)
        if not inside.any():
            continue
        w = np.stack([l1, l2, l3], -1)
        if cam.scale is None:
            iw = invw[t]
            wp = w * iw
            wp = wp / np.maximum(wp.sum(-1, keepdims=True), 1e-12)
        else:
            wp = w
        z = (w * s[:, 2]).sum(-1)
        sub = zbuf[y0:y1 + 1, x0:x1 + 1]
        ok = inside & (z < sub - 1e-5)
        if not ok.any():
            continue
        n = (wp[..., None] * mesh.N[t][None, None]).sum(2)
        nn = np.linalg.norm(n, axis=-1, keepdims=True)
        n = n / np.maximum(nn, 1e-12)
        facing = (n * cam.f).sum(-1, keepdims=True) > 0           # two-sided: flip towards the viewer
        n = np.where(facing, -n, n)
        mat = mats.get(mesh.mats[mesh.M[t]], {"base_color": [0.6, 0.6, 0.6]})
        col = np.array(mat["base_color"], np.float32)
        colimg = np.broadcast_to(col, n.shape).copy()
        if mat.get("texture"):
            tex = load_tex(os.path.normpath(os.path.join(mdir, mat["texture"])))
            uv = (wp[..., None] * mesh.UV[t][None, None]).sum(2)
            th, tw = tex.shape[:2]
            ix = np.clip((uv[..., 0] * tw).astype(int), 0, tw - 1)
            iy = np.clip(((1 - uv[..., 1]) * th).astype(int), 0, th - 1)
            colimg = tex[iy, ix].astype(np.float32) / 255.0
        if mesh.mats[mesh.M[t]] == "__ground__":
            wpos = (wp[..., None] * mesh.P[t][None, None]).sum(2)
            chk = ((np.floor(wpos[..., 0] / 0.1) + np.floor(wpos[..., 1] / 0.1)) % 2)[..., None]
            colimg = colimg * (0.93 + 0.07 * chk)
        lam = AMBIENT + sum(wt * np.clip((n * (ld / np.linalg.norm(ld))).sum(-1, keepdims=True), 0, 1) for ld, wt in LIGHTS)
        spec = mat.get("specular", 0.1)
        hdir = LIGHTS[0][0] / np.linalg.norm(LIGHTS[0][0]) - cam.f
        hdir = hdir / np.linalg.norm(hdir)
        sp = spec * np.clip((n * hdir).sum(-1, keepdims=True), 0, 1) ** max(mat.get("shininess", 20), 4)
        shaded = np.clip(colimg * lam + sp * 0.6, 0, 1)
        alpha = mat.get("alpha", 1.0)
        reg = img[y0:y1 + 1, x0:x1 + 1]
        if alpha >= 0.999:
            reg[ok] = shaded[ok]
        else:
            reg[ok] = reg[ok] * (1 - alpha) + shaded[ok] * alpha
        if write_z:
            sub[ok] = z[ok]


def bbox_of(objs):
    lo = np.full(3, np.inf)
    hi = np.full(3, -np.inf)
    for m, _, _ in objs:
        l, h = m.bbox()
        lo, hi = np.minimum(lo, l), np.maximum(hi, h)
    return lo, hi


def views(objs, ground_z):
    lo, hi = bbox_of(objs)
    c = (lo + hi) / 2
    ext = hi - lo
    R = float(np.linalg.norm(ext)) / 2
    v = {}
    d = np.array([1.25, -1.55, 0.85])
    d = d / np.linalg.norm(d) * R / math.tan(math.radians(34 / 2)) * 1.3
    v["persp"] = Camera(c + d, c, fov_deg=34)
    v["top"] = Camera(c + [0, 0, 5], c, up=(1, 0, 0), ortho_scale=1)
    v["side"] = Camera(c + [0, -5, 0], c, ortho_scale=1)
    v["front"] = Camera(c + [5, 0, 0], c, ortho_scale=1)
    return v, c, ext


def ortho_scale(view, ext, size):
    w, h = {"top": (ext[1], ext[0]), "side": (ext[0], ext[2]), "front": (ext[1], ext[2])}[view]
    return min(size[0] * 0.86 / w, size[1] * 0.86 / h)


def render_views(objs, ground_z, size=(900, 700), which=("persp", "top", "side", "front")):
    vs, c, ext = views(objs, ground_z)
    out = {}
    for k in which:
        cam = vs[k]
        if k != "persp":
            cam.scale = ortho_scale(k, ext, size) * 2 / 2 * 1.0
            cam.scale *= 1  # per-pixel scale at 1x; render() applies supersampling by W,H scaling below
        out[k] = cam
    return out, c, ext


def render_view(objs, cam, size, ground_z, gc, ss=2):
    if cam.scale is not None:
        cam2 = Camera(cam.eye, cam.eye + cam.f, cam.u, ortho_scale=cam.scale * ss)
        cam2.eye, cam2.f, cam2.r, cam2.u = cam.eye, cam.f, cam.r, cam.u
        return render(objs, cam2, size, ss, ground_z=ground_z, ground_center=gc)
    return render(objs, cam, size, ss, ground_z=ground_z, ground_center=gc)


def save(path, arr):
    from PIL import Image
    os.makedirs(os.path.dirname(path), exist_ok=True)
    Image.fromarray(arr).save(path)


def sheet(path, images, titles, ncols=3, note=None):
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    n = len(images)
    nr = int(math.ceil(n / ncols))
    fig, axs = plt.subplots(nr, ncols, figsize=(5.2 * ncols, 4.2 * nr + (0.4 if note else 0)), squeeze=False)
    for ax in axs.ravel():
        ax.axis("off")
    for ax, im, t in zip(axs.ravel(), images, titles):
        ax.imshow(im)
        ax.set_title(t, fontsize=10)
    if note:
        fig.suptitle(note, fontsize=9)
    fig.tight_layout()
    fig.savefig(path, dpi=110)
    plt.close(fig)


def closeup_dock(model, face, T_world, state, ground_z, size=(900, 700), dist=0.45, hide=()):
    T = model.face_T(face)
    if T_world is not None:
        T = T_world @ T
    origin, nrm = T[:3, 3], T[:3, 0]
    eye = origin + nrm * dist + np.array([0.0, 0.0, 0.0]) + T[:3, 1] * 0.18 + T[:3, 2] * 0.14
    cam = Camera(eye, origin + np.array([0, 0, -0.01]) - nrm * 0.02, fov_deg=40)
    return cam


def main(argv=None):
    ap = argparse.ArgumentParser()
    ap.add_argument("--assets", default=DEFAULT_ASSETS)
    ap.add_argument("--out", default=None)
    ap.add_argument("--only", choices=["roles", "docked"], default=None)
    a = ap.parse_args(argv)
    out = a.out or os.path.join(a.assets, "previews")
    size = (900, 700)
    models = {}
    for role in ("scout", "lifter", "cargo"):
        models[role] = RoverModel(os.path.join(a.assets, role, f"{role}.manifest.json"))
    for role, mdl in ([] if a.only == "docked" else models.items()):
        gz = mdl.m["base_frame"]["ground_z"]
        state = {}
        for h in mdl.m["actuated"]["hooks"]:
            state[h["part"]] = h["open_angle"] * 0.0          # closed
        objs = collect(mdl, None, state)
        cams, c, ext = render_views(objs, gz, size)
        imgs, titles = [], []
        for k, cam in cams.items():
            im = render_view(objs, cam, size, gz, (c[0], c[1]))
            save(os.path.join(out, f"{role}_{k}.png"), im)
            imgs.append(im)
            titles.append(f"{role} {k}")
        # open-hook perspective to show the swing
        st2 = dict(state)
        for h in mdl.m["actuated"]["hooks"]:
            st2[h["part"]] = h["open_angle"]
        o2 = collect(mdl, None, st2)
        # closeups of rear (+ front) docking pocket with hooks closed and open
        for face in ("front", "rear"):
            for lab, st in (("closed", state), ("open", st2)):
                ob = collect(mdl, None, st)
                cam = closeup_dock(mdl, face, None, st, gz, dist=0.34)
                im = render_view(ob, cam, size, gz, (c[0], c[1]))
                save(os.path.join(out, f"{role}_dock_{face}_{lab}.png"), im)
                if face == "front":
                    imgs.append(im)
                    titles.append(f"{role} front dock, hooks {lab}")
        if role == "scout":
            ob = collect(mdl, None, state, show_hidden=True)
            cam = cams["persp"]
            im = render_view(ob, cam, size, gz, (c[0], c[1]))
            save(os.path.join(out, f"{role}_envelope.png"), im)
            imgs.append(im)
            titles.append("scout with arm-proxy envelope shown")
        if role == "lifter":
            st3 = dict(state)
            st3["fork_carriage"] = mdl.m["actuated"]["lift"]["range"][1]
            for fg in mdl.m["actuated"]["gripper"]["fingers"]:
                st3[fg["part"]] = fg["travel"][1] * 0.8
            ob = collect(mdl, None, st3)
            im = render_view(ob, cams["persp"], size, gz, (c[0], c[1]))
            save(os.path.join(out, f"{role}_raised_gripping.png"), im)
            imgs.append(im)
            titles.append("lifter: lift raised, fingers closing")
        sheet(os.path.join(out, f"{role}_sheet.png"), imgs, titles, 3, f"{role}: PROVISIONAL geometry generated from fleet_profile_v0 (not CAD)")
    # docked pairs: A.front <-> B.rear
    pairs = [] if a.only == "roles" else [("scout", "cargo"), ("cargo", "cargo"), ("lifter", "cargo")]
    for ra, rb in pairs:
        A, B = models[ra], models[rb]
        TA = np.eye(4)
        if ra == "lifter":
            sa = {"fork_carriage": A.m["actuated"]["lift"]["docking_value"]}
        else:
            sa = {}
        TB = TA @ A.face_T("front") @ g.make_T(g.rotz(math.pi)) @ g.inv_T(B.face_T("rear"))
        objs = collect(A, TA, sa) + collect(B, TB, {})
        lo, hi = bbox_of(objs)
        c = (lo + hi) / 2
        ext = hi - lo
        gz = A.m["base_frame"]["ground_z"]
        R = float(np.linalg.norm(ext)) / 2
        d = np.array([1.2, -1.7, 0.9])
        d = d / np.linalg.norm(d) * R / math.tan(math.radians(32 / 2)) * 1.1
        cp = Camera(c + d, c, fov_deg=32)
        ct = Camera(c + [0, 0, 5], c, up=(1, 0, 0), ortho_scale=min(size[0] * 0.88 / ext[0], size[1] * 0.88 / ext[1]))
        im1 = render_view(objs, cp, size, gz, (c[0], c[1]), ss=2)
        im2 = render_view(objs, ct, size, gz, (c[0], c[1]), ss=2)
        cc = (TA @ A.face_T("front"))[:3, 3]
        cam3 = Camera(cc + np.array([-0.0, -0.40, 0.22]), cc, fov_deg=38)
        im3 = render_view(objs, cam3, size, gz, (c[0], c[1]), ss=2)
        # cutaway: remove everything above the pocket ceiling, look straight down at the mating plane
        zc_cut = float(TA[2, 3] + (TA @ A.face_T("front"))[2, 3] + 0.026)
        objs_c = collect(A, TA, sa, clip_z=zc_cut) + collect(B, TB, {}, clip_z=zc_cut)
        cam4 = Camera(cc + np.array([-0.16, -0.20, 0.30]), cc + np.array([0, 0, -0.02]), fov_deg=34)
        im4 = render_view(objs_c, cam4, size, gz, (c[0], c[1]), ss=2)
        tag = f"docked_{ra}_front_to_{rb}_rear"
        save(os.path.join(out, tag + "_cutaway.png"), im4)
        save(os.path.join(out, tag + "_persp.png"), im1)
        save(os.path.join(out, tag + "_top.png"), im2)
        save(os.path.join(out, tag + "_closeup.png"), im3)
        sheet(os.path.join(out, tag + "_sheet.png"), [im1, im2, im3, im4], ["perspective", "top (ortho)", "closeup at mating plane", "cutaway: top removed, hooks closed on pins"], 2,
              f"{ra}.front <-> {rb}.rear, hooks closed (PROVISIONAL)")
    print("previews in", out)


if __name__ == "__main__":
    main()
