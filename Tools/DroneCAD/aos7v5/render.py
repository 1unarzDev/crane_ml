#!/usr/bin/env python3
"""Headless offscreen renders (pyrender/EGL) of aos7v5_cad.glb: iso (reference camera angle), top, front, side and a
side-by-side with reference_concept.png.   python render.py <model_dir> <out_dir> [reference.png]"""
import os, sys
os.environ.setdefault("PYOPENGL_PLATFORM", "egl")
from pathlib import Path
import numpy as np, trimesh, pyrender
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib import image as mpimg

BG = np.array([0.80, 0.70, 0.72, 1.0])


def look_at(eye, target, up):
    f = target - eye; f /= np.linalg.norm(f)
    r = np.cross(f, up); r /= np.linalg.norm(r)
    u = np.cross(r, f)
    m = np.eye(4); m[:3, 0] = r; m[:3, 1] = u; m[:3, 2] = -f; m[:3, 3] = eye
    return m


def render(scene_tm, eye_dir, up, size, ortho=None, fov=30.0, fit=1.0):
    sc = pyrender.Scene.from_trimesh_scene(scene_tm, bg_color=BG, ambient_light=[0.35, 0.33, 0.34])
    b = scene_tm.bounds; c = b.mean(0); rad = np.linalg.norm(b[1] - b[0]) / 2
    d = np.array(eye_dir, float); d /= np.linalg.norm(d)
    if ortho:
        cam = pyrender.OrthographicCamera(xmag=ortho[0], ymag=ortho[1], znear=0.01, zfar=5)
        eye = c + d * 1.5
    else:
        cam = pyrender.PerspectiveCamera(yfov=np.radians(fov), znear=0.01, zfar=10)
        eye = c + d * (rad * fit / np.sin(np.radians(fov) / 2))
    pose = look_at(eye, c, np.array(up, float))
    sc.add(cam, pose=pose)
    for dirv, inten in (([-0.5, -0.6, -1.0], 3.0), ([0.8, 0.3, -0.5], 1.5), ([0, 0.5, 1], 0.8)):
        l = pyrender.DirectionalLight(color=np.ones(3), intensity=inten)
        # light looking along dirv
        z = -np.array(dirv, float); z /= np.linalg.norm(z)
        pose_l = look_at(c + z, c, np.array([0, 0, 1.0]) if abs(z[2]) < 0.99 else np.array([1.0, 0, 0]))
        sc.add(l, pose=pose_l)
    r = pyrender.OffscreenRenderer(*size)
    col, _ = r.render(sc, flags=pyrender.RenderFlags.SKIP_CULL_FACES)
    r.delete()
    return col


def main():
    mdir, odir = Path(sys.argv[1]), Path(sys.argv[2])
    ref = Path(sys.argv[3]) if len(sys.argv) > 3 else None
    odir.mkdir(parents=True, exist_ok=True)
    scene = trimesh.load(mdir / "aos7v5_cad.glb")
    b = scene.bounds
    ext = b[1] - b[0]
    out = {}
    # reference: nose to image-left, viewed from the front-left, ~30 deg elevation (AI render: perspective ~ 30 deg)
    az, el = np.radians(62), np.radians(30)
    out["iso"] = render(scene, [np.cos(az) * np.cos(el), np.sin(az) * np.cos(el), np.sin(el)], [0, 0, 1], (1670, 940), fov=30, fit=0.74)
    out["top"] = render(scene, [0, 0, 1], [1, 0, 0], (900, 1000), ortho=(ext[1] / 2 * 1.08, ext[0] / 2 * 1.1))
    out["front"] = render(scene, [1, 0, 0], [0, 0, 1], (1000, 700), ortho=(ext[1] / 2 * 1.1, ext[2] / 2 * 1.2))
    out["side"] = render(scene, [0, 1, 0], [0, 0, 1], (1100, 700), ortho=(ext[0] / 2 * 1.05, ext[2] / 2 * 1.3))
    for k, v in out.items():
        mpimg.imsave(odir / f"{k}.png", v)
    if ref and ref.exists():
        fig, ax = plt.subplots(1, 2, figsize=(20, 5.8))
        ax[0].imshow(mpimg.imread(ref)); ax[0].set_title("reference_concept.png (AI render, layout only)")
        ax[1].imshow(out["iso"]); ax[1].set_title(f"CAD ({odir.name}) same view")
        for a in ax: a.axis("off")
        plt.tight_layout(); plt.savefig(odir / "side_by_side.png", dpi=80); plt.close()
    print("rendered", list(out), "->", odir)


if __name__ == "__main__":
    main()
