# SPDX-License-Identifier: GPL-3.0-or-later
"""Bark tube meshes, leaf cards and LODs from a Skeleton.

Junctions follow the modular_tree "manifold" idea in spirit (no code ported): every lateral chain
starts with a collar ring centred on the parent node, so the child tube is rooted inside the parent
tube with no visible gap; tips are closed with an apex vertex and the stem base with a fan, so each
tube is closed. Bark UVs: U wraps an integer number of times (round(2*pi*r/bark_tile)), V = arc length
/ bark_tile, i.e. constant texel density along the axis and seam-free around it.
"""
from __future__ import annotations

import numpy as np

from .glb import MeshPart
from .rng import stream

LOD_SPECS = {
    0: {"min_r": 0.0, "k_max": 14, "k_min": 4, "seg_len": 0.035, "leaf_keep": 1.0, "leaf_scale": 1.0,
        "clumps": False},
    1: {"min_r": 0.01, "k_max": 8, "k_min": 4, "seg_len": 0.06, "leaf_keep": 0.4, "leaf_scale": 1.55,
        "clumps": False},
    2: {"min_r": 0.03, "k_max": 6, "k_min": 3, "seg_len": 0.12, "leaf_keep": 0.0, "leaf_scale": 1.0,
        "clumps": True},
}


def _norm(v):
    return v / np.maximum(np.linalg.norm(v, axis=-1, keepdims=True), 1e-12)


def _transport_frames(T):
    """Parallel-transported normals along tangents T (m,3)."""
    m = len(T)
    N = np.zeros_like(T)
    a = np.array([1.0, 0, 0]) if abs(T[0, 0]) < 0.9 else np.array([0, 1.0, 0])
    N[0] = _norm(a - (a @ T[0]) * T[0])
    for i in range(1, m):
        n = N[i - 1] - (N[i - 1] @ T[i]) * T[i]
        ln = np.linalg.norm(n)
        N[i] = n / ln if ln > 1e-9 else _norm(np.cross(T[i], [0.0, 0, 1.0]) + 1e-6)
    return N


def bark_mesh(sk, lod=0, bark_tile=0.5, chains=None):
    spec = LOD_SPECS[lod]
    if chains is None:
        chains = sk.chains()
    P_all, N_all, UV_all, T_all = [], [], [], []
    nv = 0
    pos, rad = sk.pos, sk.radius
    for ch in chains:
        if len(ch) < 2:
            continue
        # truncate at the first node thinner than this LOD keeps
        r = rad[ch].copy()
        if spec["min_r"] > 0:
            # the collar uses the child radius, not the parent's
            thin = np.nonzero(r[1:] < spec["min_r"])[0]
            if len(thin):
                ch = ch[: thin[0] + 1]
                r = r[: thin[0] + 1]
            if len(ch) < 2:
                continue
        is_lateral = sk.parent[ch[1]] == ch[0] and not sk.is_main[ch[1]]
        if is_lateral:
            r[0] = r[1]
        p = pos[ch]
        seg = np.diff(p, axis=0)
        sl = np.linalg.norm(seg, axis=1)
        if np.all(sl < 1e-9):
            continue
        tseg = _norm(seg)
        T = np.empty_like(p)
        T[0] = tseg[0]
        T[-1] = tseg[-1]
        if len(p) > 2:
            T[1:-1] = _norm(tseg[:-1] + tseg[1:])
        Nf = _transport_frames(T)
        Bf = np.cross(T, Nf)
        rmed = float(np.median(r))
        k = int(np.clip(np.ceil(2 * np.pi * rmed / spec["seg_len"]), spec["k_min"], spec["k_max"]))
        wraps = max(1, int(round(2 * np.pi * rmed / bark_tile)))
        th = np.linspace(0, 2 * np.pi, k + 1)          # duplicate seam column
        c, s = np.cos(th), np.sin(th)
        radial = c[None, :, None] * Nf[:, None, :] + s[None, :, None] * Bf[:, None, :]   # (m,k+1,3)
        rings = p[:, None, :] + r[:, None, None] * radial
        arc = np.concatenate([[0.0], np.cumsum(sl)])
        u = np.broadcast_to((th / (2 * np.pi) * wraps)[None, :], (len(p), k + 1))
        v = np.broadcast_to((arc / bark_tile)[:, None], (len(p), k + 1))
        m = len(p)
        P_all.append(rings.reshape(-1, 3))
        N_all.append(radial.reshape(-1, 3))
        UV_all.append(np.stack([u, v], -1).reshape(-1, 2))
        i = np.arange(m - 1)[:, None]
        j = np.arange(k)[None, :]
        a = nv + i * (k + 1) + j
        b = a + 1
        cc = b + (k + 1)
        d = a + (k + 1)
        tri = np.concatenate([np.stack([a, b, cc], -1).reshape(-1, 3), np.stack([a, cc, d], -1).reshape(-1, 3)])
        T_all.append(tri)
        nv += m * (k + 1)
        # apex cap at the tip
        tip = p[-1] + T[-1] * r[-1] * 1.5
        P_all.append(tip[None])
        N_all.append(T[-1][None])
        UV_all.append(np.array([[0.5 * wraps, arc[-1] / bark_tile + r[-1] / bark_tile]]))
        ring0 = nv - (k + 1)
        jj = np.arange(k)
        T_all.append(np.stack([ring0 + jj, ring0 + jj + 1, np.full(k, nv)], -1))
        nv += 1
        # base fan on the stem (laterals are rooted inside their parent)
        if sk.parent[ch[0]] < 0:
            base = p[0] - T[0] * 0.0
            P_all.append(base[None])
            N_all.append(-T[0][None])
            UV_all.append(np.array([[0.5 * wraps, 0.0]]))
            ring0 = nv - (m * (k + 1) + 1)
            T_all.append(np.stack([ring0 + jj + 1, ring0 + jj, np.full(k, nv)], -1))
            nv += 1
    if not P_all:
        return MeshPart("bark", np.zeros((0, 3)), np.zeros((0, 3)), np.zeros((0, 2)), np.zeros((0, 3)), {})
    return MeshPart("bark", np.concatenate(P_all), np.concatenate(N_all), np.concatenate(UV_all),
                    np.concatenate(T_all), {})


# ---------------------------------------------------------------- leaves
ATLAS = 2   # 2x2 atlas: 0 broadleaf, 1 needle spray, 2 small broadleaf, 3 clump


def _atlas_rect(idx):
    col, row = idx % ATLAS, idx // ATLAS
    return np.array([col / ATLAS, row / ATLAS]), 1.0 / ATLAS


def leaf_anchor_points(sk, species):
    lf = species["leaf"]
    final = int(sk.meta.get("start_season", 0)) + int(sk.meta.get("seasons", 0)) - 1
    age = final - sk.birth.astype(np.int64)
    ok = sk.alive & (sk.radius <= float(lf["max_radius"])) & (age < int(lf["max_age"])) & (sk.parent >= 0)
    return np.nonzero(ok)[0]


def leaf_cards(sk, species, forest_seed, tree_id, keep=1.0, scale=1.0):
    """Quads (two triangles, double-sided material) around young thin nodes. Own RNG stream."""
    lf = species["leaf"]
    rng = stream(forest_seed, tree_id, "leaves")
    ids = leaf_anchor_points(sk, species)
    cnt = rng.poisson(float(lf["cards_per_node"]), len(ids))
    sel = rng.random(len(ids)) < keep
    cnt = np.where(sel, cnt, 0)
    rep = np.repeat(ids, cnt)
    n = len(rep)
    if n == 0:
        return MeshPart("leaves", np.zeros((0, 3)), np.zeros((0, 3)), np.zeros((0, 2)), np.zeros((0, 3)), {}), np.zeros((0, 3))
    par = sk.parent[rep]
    axis = _norm(sk.pos[rep] - sk.pos[par])
    t = rng.random(n)
    centre = sk.pos[par] + (sk.pos[rep] - sk.pos[par]) * t[:, None]
    out = centre.copy()
    out[:, 2] = 0
    outward = _norm(out + 1e-6)
    nrm = _norm(0.55 * np.array([0, 0, 1.0]) + 0.35 * outward + 0.35 * rng.standard_normal((n, 3)))
    size = float(lf["card_size"]) * scale * np.exp(rng.normal(0, 0.15, n))
    aspect = float(lf.get("aspect", 1.0))
    # card long axis follows the twig, projected into the card plane
    long = axis - (axis * nrm).sum(1, keepdims=True) * nrm
    long = _norm(long + 1e-6 * rng.standard_normal((n, 3)))
    side = np.cross(nrm, long)
    off = long * (0.5 * size)[:, None]       # card hangs from the twig
    c = centre + off
    hl = (0.5 * size)[:, None] * long
    hs = (0.5 * size * aspect)[:, None] * side
    quad = np.stack([c - hl - hs, c - hl + hs, c + hl + hs, c + hl - hs], 1)
    o, w = _atlas_rect(int(lf.get("atlas_index", 0)))
    uvq = o + w * np.array([[0, 0], [1, 0], [1, 1], [0, 1.0]])
    P = quad.reshape(-1, 3)
    N = np.repeat(nrm, 4, axis=0)
    UV = np.tile(uvq, (n, 1))
    base = (np.arange(n) * 4)[:, None]
    tri = np.concatenate([base + np.array([0, 1, 2]), base + np.array([0, 2, 3])], 1).reshape(-1, 3)
    return MeshPart("leaves", P, N, UV, tri, {}), c


def leaf_clumps(centres, species, voxel=1.6):
    """LOD2 foliage: three crossed quads per occupied voxel of leaf centres (atlas tile 3)."""
    if len(centres) == 0:
        return MeshPart("leaves", np.zeros((0, 3)), np.zeros((0, 3)), np.zeros((0, 2)), np.zeros((0, 3)), {}), np.zeros((0, 4))
    key = np.floor(centres / voxel).astype(np.int64)
    uk, inv, counts = np.unique(key, axis=0, return_inverse=True, return_counts=True)
    inv = inv.reshape(-1)
    cen = np.zeros((len(uk), 3))
    for ax in range(3):
        cen[:, ax] = np.bincount(inv, weights=centres[:, ax]) / counts
    keep = counts >= 3
    cen = cen[keep]
    cnt = counts[keep]
    s = voxel * np.clip(0.55 + 0.03 * cnt, 0.6, 1.1)
    Ps, Ns, UVs, Ts = [], [], [], []
    o, w = _atlas_rect(3)
    uvq = o + w * np.array([[0, 0], [1, 0], [1, 1], [0, 1.0]])
    axes = [(np.array([1.0, 0, 0]), np.array([0, 0, 1.0])), (np.array([0, 1.0, 0]), np.array([0, 0, 1.0])),
            (np.array([1.0, 0, 0]), np.array([0, 1.0, 0]))]
    nv = 0
    for a, b in axes:
        h = s[:, None] * 0.5
        q = np.stack([cen - h * a - h * b, cen + h * a - h * b, cen + h * a + h * b, cen - h * a + h * b], 1)
        Ps.append(q.reshape(-1, 3))
        Ns.append(np.repeat(np.cross(a, b)[None], len(cen) * 4, 0))
        UVs.append(np.tile(uvq, (len(cen), 1)))
        base = nv + (np.arange(len(cen)) * 4)[:, None]
        Ts.append(np.concatenate([base + np.array([0, 1, 2]), base + np.array([0, 2, 3])], 1).reshape(-1, 3))
        nv += len(cen) * 4
    vol = np.concatenate([cen, (s * 0.6)[:, None]], 1)
    return MeshPart("leaves", np.concatenate(Ps), np.concatenate(Ns), np.concatenate(UVs), np.concatenate(Ts), {}), vol


def build_lods(sk, species, forest_seed, tree_id, lods=(0, 1, 2), with_leaves=True):
    bark_mat = {"name": f"{species['name']}_bark", "color": species["bark_color"], "double_sided": False,
                "texture_hint": f"bark/{species['name']}"}
    leaf_mat = {"name": f"{species['name']}_leaf", "color": species["leaf_color"], "double_sided": True,
                "alpha_mask": True, "texture_hint": f"leaf_atlas/{species['name']}"}
    chains = sk.chains()
    out = {}
    full_leaves, centres = (leaf_cards(sk, species, forest_seed, tree_id) if with_leaves
                            else (None, np.zeros((0, 3))))
    foliage_volumes = np.zeros((0, 4))
    for lod in lods:
        spec = LOD_SPECS[lod]
        bark = bark_mesh(sk, lod, float(species.get("bark_tile_m", 0.5)), chains)
        bark.material = bark_mat
        leaves = None
        if with_leaves:
            if spec["clumps"]:
                leaves, foliage_volumes = leaf_clumps(centres, species)
            elif spec["leaf_keep"] >= 1.0:
                leaves = full_leaves
            else:
                leaves, _ = leaf_cards(sk, species, forest_seed, tree_id, spec["leaf_keep"], spec["leaf_scale"])
            leaves.material = leaf_mat
        out[lod] = (bark, leaves)
    if with_leaves and len(foliage_volumes) == 0 and len(centres):
        _, foliage_volumes = leaf_clumps(centres, species)
    return out, centres, foliage_volumes
