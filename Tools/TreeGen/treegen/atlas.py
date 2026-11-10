# SPDX-License-Identifier: GPL-3.0-or-later
"""Procedural 2x2 leaf atlas (RGBA PNG) matching the leaf-card UV convention in CONTRACT.md section 7.

Tile 0 (u 0-0.5, v 0-0.5): broadleaf spray (beech-like ovate leaves on a sprig)
Tile 1 (u 0.5-1, v 0-0.5): needle spray (spruce)
Tile 2 (u 0-0.5, v 0.5-1): small broadleaf spray (birch, serrate-ish deltoid leaves)
Tile 3 (u 0.5-1, v 0.5-1): dense foliage clump (LOD2 crossed quads)

v = 0 is the bottom of the image (Unity/OpenGL convention): every card hangs from its twig at v = 0, so
each sprig starts at the bottom centre of its tile and grows upward. Colours are mid greens; the
material tint per species multiplies them. Transparent texels carry the mean leaf colour so mipmaps
do not darken the alpha-clipped edges. Drawn with PIL at 2x and downsampled. Deterministic per seed.
"""
from __future__ import annotations

import math

import numpy as np


def _leaf_poly(cx, cy, ang, length, width, n=18, serrate=0.0, deltoid=0.0):
    pts = []
    for side in (1, -1):
        rng_t = np.linspace(0, 1, n) if side == 1 else np.linspace(1, 0, n)
        for t in rng_t:
            # ovate outline: widest at ~40% of the length; deltoid shifts the widest point to the base
            w = width * math.sin(math.pi * t ** (0.8 - 0.45 * deltoid)) * (1 - 0.15 * t)
            if serrate:
                w *= 1 + serrate * (0.5 + 0.5 * math.sin(t * 40))
            x, y = t * length, side * w * 0.5
            pts.append((cx + x * math.cos(ang) - y * math.sin(ang), cy + x * math.sin(ang) + y * math.cos(ang)))
    return pts


def _col(rng, base, var=0.12):
    c = np.clip(np.array(base) * (1 + rng.normal(0, var, 3)) + rng.normal(0, 0.03), 0, 1)
    return tuple(int(255 * v) for v in c) + (255,)


def make_atlas(path, size=1024, seed=5):
    from PIL import Image, ImageDraw
    rng = np.random.default_rng(seed)
    S = size * 2
    T = S // 2
    img = Image.new("RGBA", (S, S), (0, 0, 0, 0))
    d = ImageDraw.Draw(img)

    def tile_origin(idx):
        col, row = idx % 2, idx // 2
        # image row 0 is the top; v = 0 is the bottom, so tile row 0 sits in the lower half
        return col * T, S - (row + 1) * T

    def to_img(idx, x, y):           # tile-local (x right, y up, 0..1) -> image px
        ox, oy = tile_origin(idx)
        return ox + x * T, oy + (1 - y) * T

    stem = (88, 70, 46, 255)
    # ---- tile 0: beech spray
    for idx, base, n_leaf, L, W, ser, delt in ((0, (0.30, 0.50, 0.17), 9, 0.30, 0.17, 0.0, 0.0),
                                                (2, (0.40, 0.58, 0.20), 13, 0.20, 0.15, 0.08, 0.7)):
        pts = [(0.5 + 0.03 * math.sin(k * 0.9), k / 10 * 0.92) for k in range(11)]
        d.line([to_img(idx, *p) for p in pts], fill=stem, width=max(2, T // 120))
        for k in range(n_leaf):
            t = 0.12 + 0.82 * k / max(n_leaf - 1, 1)
            px, py = 0.5 + 0.03 * math.sin(t * 9), t * 0.92
            side = 1 if k % 2 == 0 else -1
            ang = math.radians(90 - side * (55 + rng.normal(0, 8)) - (25 if idx == 2 else 0) * side * 0)
            if k == n_leaf - 1:
                ang = math.radians(90 + rng.normal(0, 5))
            sc = (0.75 + 0.35 * math.sin(math.pi * t)) * (1 + rng.normal(0, 0.08))
            poly = _leaf_poly(px, py, ang, L * sc, W * sc, serrate=ser, deltoid=delt)
            ip = [to_img(idx, x, y) for x, y in poly]
            d.polygon(ip, fill=_col(rng, base))
            # midrib
            tip = (px + L * sc * 0.9 * math.cos(ang), py + L * sc * 0.9 * math.sin(ang))
            d.line([to_img(idx, px, py), to_img(idx, *tip)], fill=_col(rng, np.array(base) * 0.75, 0.05),
                   width=max(1, T // 300))
    # ---- tile 1: needle spray (planar, both sides of a central twig plus two side shoots)
    idx = 1
    nb = (0.16, 0.32, 0.15)
    axes = [((0.5, 0.0), (0.5, 0.97)), ((0.5, 0.35), (0.2, 0.8)), ((0.5, 0.45), (0.8, 0.88))]
    for (x0, y0), (x1, y1) in axes:
        d.line([to_img(idx, x0, y0), to_img(idx, x1, y1)], fill=stem, width=max(2, T // 140))
        L = math.hypot(x1 - x0, y1 - y0)
        ux, uy = (x1 - x0) / L, (y1 - y0) / L
        for k in range(int(L * 90)):
            t = k / (L * 90)
            px, py = x0 + ux * L * t, y0 + uy * L * t
            for side in (1, -1):
                a = math.atan2(uy, ux) + side * math.radians(55 + rng.normal(0, 10))
                nl = 0.07 * (1 - 0.4 * t) * (1 + rng.normal(0, 0.1))
                d.line([to_img(idx, px, py), to_img(idx, px + nl * math.cos(a), py + nl * math.sin(a))],
                       fill=_col(rng, nb, 0.15), width=max(2, T // 220))
    # ---- tile 3: clump (many overlapping leaves inside a soft disc)
    idx = 3
    for k in range(260):
        r = 0.42 * math.sqrt(rng.random())
        th = rng.uniform(0, 2 * math.pi)
        px, py = 0.5 + r * math.cos(th), 0.5 + r * math.sin(th)
        poly = _leaf_poly(px, py, rng.uniform(0, 2 * math.pi), 0.11, 0.065)
        shade = 0.75 + 0.5 * (py - 0.1)
        d.polygon([to_img(idx, x, y) for x, y in poly], fill=_col(rng, np.array((0.25, 0.45, 0.15)) * shade))
    img = img.resize((size, size), Image.LANCZOS)
    a = np.asarray(img).astype(np.float32)
    alpha = a[..., 3:4] / 255.0
    solid = alpha[..., 0] > 0.5
    mean = a[solid][:, :3].mean(0) if solid.any() else np.array([60, 100, 40.0])
    rgb = np.where(alpha > 0.02, a[..., :3] / np.maximum(alpha, 1e-3), mean)
    out = np.concatenate([np.clip(rgb, 0, 255), a[..., 3:4]], -1).astype(np.uint8)
    Image.fromarray(out, "RGBA").save(path)
    return path
