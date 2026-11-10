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


def _empty(name):
    return MeshPart(name, np.zeros((0, 3)), np.zeros((0, 3)), np.zeros((0, 2)), np.zeros((0, 3)), {})


def _quads(c, long, nrm, size, aspect, atlas_index):
    """Cards hanging from point c along `long` (card plane normal `nrm`). Returns P, N, UV, tri."""
    n = len(c)
    long = long - (long * nrm).sum(1, keepdims=True) * nrm
    long = _norm(long + 1e-9)
    side = np.cross(nrm, long)
    hl = (0.5 * size)[:, None] * long
    hs = (0.5 * size * aspect)[:, None] * side
    m = c + hl
    quad = np.stack([m - hl - hs, m - hl + hs, m + hl + hs, m + hl - hs], 1)
    o, w = _atlas_rect(int(atlas_index))
    uvq = o + w * np.array([[0, 0], [1, 0], [1, 1], [0, 1.0]])
    base = (np.arange(n) * 4)[:, None]
    tri = np.concatenate([base + np.array([0, 1, 2]), base + np.array([0, 2, 3])], 1).reshape(-1, 3)
    return quad.reshape(-1, 3), np.repeat(nrm, 4, axis=0), np.tile(uvq, (n, 1)), tri, m


def twig_sprays(sk, species, forest_seed, tree_id, keep=1.0, scale=1.0, with_twigs=True):
    """Weber-Penn style terminal sprays (Weber & Penn 1995, the last stem level plus leaves; parameter
    names follow friggog/tree-gen, no code ported). Every young thin internode gets `twigs_per_node`
    short twigs at `twig_down_deg` from its axis, spread by `twig_rotate_deg` phyllotaxis and bent by
    `twig_gravity` (birch: pendulous). Each twig carries `leaves_per_twig` cards in alternating pairs
    plus one terminal card, and the internode itself keeps `cards_per_node` direct cards.
    Visual only: twigs are thinner than colliders' soft_min_d (4 mm) and never enter the skeleton.
    Own RNG streams ("leaves" for direct cards, "twigs" for sprays), so topology hashes are untouched.
    Returns (twig tube MeshPart, leaf MeshPart, leaf card centres)."""
    lf = species["leaf"]
    ids = leaf_anchor_points(sk, species)
    rng = stream(forest_seed, tree_id, "leaves")
    cnt = rng.poisson(float(lf["cards_per_node"]), len(ids))
    sel = rng.random(len(ids)) < keep
    cnt = np.where(sel, cnt, 0)
    up = np.array([0, 0, 1.0])
    aspect = float(lf.get("aspect", 1.0))
    atlas = int(lf.get("atlas_index", 0))
    Ps, Ns, UVs, Ts, Cs = [], [], [], [], []
    nv = 0

    def add(P, N, UV, T, C):
        nonlocal nv
        Ps.append(P); Ns.append(N); UVs.append(UV); Ts.append(T + nv); Cs.append(C)
        nv += len(P)

    # ---- direct cards on the internode (legacy behaviour, same RNG draws as schema 1)
    rep = np.repeat(ids, cnt)
    n = len(rep)
    if n:
        par = sk.parent[rep]
        axis = _norm(sk.pos[rep] - sk.pos[par])
        t = rng.random(n)
        centre = sk.pos[par] + (sk.pos[rep] - sk.pos[par]) * t[:, None]
        out = centre.copy()
        out[:, 2] = 0
        outward = _norm(out + 1e-6)
        nrm = _norm(0.55 * up + 0.35 * outward + 0.35 * rng.standard_normal((n, 3)))
        size = float(lf["card_size"]) * scale * np.exp(rng.normal(0, 0.15, n))
        add(*_quads(centre, axis + 1e-6 * rng.standard_normal((n, 3)), nrm, size, aspect, atlas))
    # ---- twig sprays
    tw = stream(forest_seed, tree_id, "twigs")
    k_tw = int(lf.get("twigs_per_node", 0))
    twig_parts = None
    if k_tw > 0 and len(ids):
        sel_t = tw.random(len(ids)) < keep
        tid_ = np.repeat(ids[sel_t], k_tw)
        m = len(tid_)
        par = sk.parent[tid_]
        a, b = sk.pos[par], sk.pos[tid_]
        axis = _norm(b - a)
        slot = np.tile(np.arange(k_tw), int(sel_t.sum()))
        t = np.clip((slot + 0.5) / k_tw + tw.normal(0, 0.12, m), 0.05, 1.0)
        base = a + (b - a) * t[:, None]
        ref = np.where(np.abs(axis[:, 2:3]) < 0.9, up, np.array([1.0, 0, 0]))
        e1 = _norm(np.cross(axis, ref))
        e2 = np.cross(axis, e1)
        rot = np.radians(float(lf.get("twig_rotate_deg", 140.0)))
        az = rot * slot + tw.uniform(0, 2 * np.pi, m)
        if lf.get("twig_planar", False):
            # conifer sprays lie flat: left/right of the shoot in its horizontal plane
            lr = _norm(np.cross(axis, up) + 1e-6)
            side = lr * np.where(slot % 2 == 0, 1.0, -1.0)[:, None]
        else:
            side = np.cos(az)[:, None] * e1 + np.sin(az)[:, None] * e2
        down = np.radians(float(lf.get("twig_down_deg", 45.0))) + tw.normal(0, 0.15, m)
        d = _norm(np.cos(down)[:, None] * axis + np.sin(down)[:, None] * side
                  - float(lf.get("twig_gravity", 0.1)) * up)
        L = float(lf.get("twig_len", 0.2)) * scale ** 0.5 * np.exp(tw.normal(0, 0.2, m))
        tip = base + d * L[:, None]
        # bend: mid point sags/droops with gravity
        mid = 0.5 * (base + tip) - (float(lf.get("twig_gravity", 0.1)) * 0.25 * L)[:, None] * up
        # twig tubes: triangular prisms base->mid->tip, radius 1.5 mm -> 0.8 mm (LOD0 only)
        if with_twigs:
            twig_parts = _twig_tubes(base, mid, tip, 0.0018, 0.0009, float(species.get("bark_tile_m", 0.5)))
        # leaves along the twig: pairs at fractions, plus a terminal card
        npt = int(lf.get("leaves_per_twig", 4))
        if npt > 0:
            fr = (np.arange(npt) + 1.0) / (npt + 0.5)
            ti = np.repeat(np.arange(m), npt)
            f = np.tile(fr, m)
            u = f[:, None]
            # quadratic bezier position and tangent
            P = (1 - u) ** 2 * base[ti] + 2 * u * (1 - u) * mid[ti] + u ** 2 * tip[ti]
            T = _norm(2 * (1 - u) * (mid[ti] - base[ti]) + 2 * u * (tip[ti] - mid[ti]))
            alt = np.where(np.tile(np.arange(npt), m) % 2 == 0, 1.0, -1.0)[:, None]
            sd = _norm(np.cross(T, up) + 1e-6) * alt
            ld = np.radians(float(lf.get("leaf_down_deg", 55.0)))
            long = _norm(np.cos(ld) * T + np.sin(ld) * sd - float(lf.get("leaf_droop", 0.2)) * up)
            k = len(P)
            nrm = _norm(0.7 * up + 0.3 * np.cross(long, T) * alt + 0.3 * tw.standard_normal((k, 3)))
            size = float(lf.get("spray_card_size", lf["card_size"])) * scale * np.exp(tw.normal(0, 0.15, k))
            add(*_quads(P, long, nrm, size, aspect, atlas))
            # terminal card continues the twig
            nrm = _norm(0.7 * up + 0.3 * tw.standard_normal((m, 3)))
            size = float(lf.get("spray_card_size", lf["card_size"])) * scale * np.exp(tw.normal(0, 0.15, m))
            add(*_quads(tip, d, nrm, size, aspect, atlas))
    if twig_parts is None:
        twig_parts = _empty("twigs")
    if not Ps:
        return twig_parts, _empty("leaves"), np.zeros((0, 3))
    return (twig_parts, MeshPart("leaves", np.concatenate(Ps), np.concatenate(Ns), np.concatenate(UVs),
                                 np.concatenate(Ts), {}), np.concatenate(Cs))


def _twig_tubes(base, mid, tip, r0, r1, bark_tile):
    """Two-ring triangular prisms base->mid->tip collapsed to one segment (6 tris per twig)."""
    m = len(base)
    if m == 0:
        return _empty("twigs")
    end = 0.5 * (mid + tip)                                   # follows the sag a little
    T = _norm(end - base)
    ref = np.where(np.abs(T[:, 2:3]) < 0.9, np.array([0, 0, 1.0]), np.array([1.0, 0, 0]))
    N = _norm(np.cross(T, ref))
    B = np.cross(T, N)
    th = np.array([0, 2 * np.pi / 3, 4 * np.pi / 3])
    radial = np.cos(th)[None, :, None] * N[:, None, :] + np.sin(th)[None, :, None] * B[:, None, :]   # (m,3,3)
    P = np.stack([base[:, None, :] + r0 * radial, end[:, None, :] + r1 * radial], 1)               # (m,2,3,3)
    L = np.linalg.norm(end - base, axis=1)
    UV = np.stack([np.broadcast_to(th / (2 * np.pi), (m, 2, 3)),
                   np.broadcast_to((np.stack([np.zeros(m), L], 1) / bark_tile)[:, :, None], (m, 2, 3))], -1)
    tri = []
    for j in range(3):
        a, b = j, (j + 1) % 3
        tri += [[a, b, b + 3], [a, b + 3, a + 3]]
    tri = (np.arange(m) * 6)[:, None, None] + np.array(tri)[None]
    return MeshPart("twigs", P.reshape(-1, 3), np.repeat(radial[:, None], 2, 1).reshape(-1, 3), UV.reshape(-1, 2),
                    tri.reshape(-1, 3), {})


def leaf_cards(sk, species, forest_seed, tree_id, keep=1.0, scale=1.0):
    """Leaf cards (direct cards + twig-spray cards, double-sided material). Returns (MeshPart, centres)."""
    _, leaves, centres = twig_sprays(sk, species, forest_seed, tree_id, keep, scale, with_twigs=False)
    return leaves, centres


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


def merge_parts(name, parts):
    parts = [p for p in parts if not p.empty]
    off = np.cumsum([0] + [len(p.pos) for p in parts])[:-1]
    return MeshPart(name, np.concatenate([p.pos for p in parts]), np.concatenate([p.nrm for p in parts]),
                    np.concatenate([p.uv for p in parts]),
                    np.concatenate([p.tri.astype(np.int64) + o for p, o in zip(parts, off)]), {})


def build_lods(sk, species, forest_seed, tree_id, lods=(0, 1, 2), with_leaves=True):
    bark_mat = {"name": f"{species['name']}_bark", "color": species["bark_color"], "double_sided": False,
                "texture_hint": f"bark/{species['name']}"}
    leaf_mat = {"name": f"{species['name']}_leaf", "color": species["leaf_color"], "double_sided": True,
                "alpha_mask": True, "texture_hint": f"leaf_atlas/{species['name']}"}
    chains = sk.chains()
    out = {}
    if with_leaves:
        twigs, full_leaves, centres = twig_sprays(sk, species, forest_seed, tree_id)
    else:
        twigs, full_leaves, centres = _empty("twigs"), None, np.zeros((0, 3))
    foliage_volumes = np.zeros((0, 4))
    for lod in lods:
        spec = LOD_SPECS[lod]
        bark = bark_mesh(sk, lod, float(species.get("bark_tile_m", 0.5)), chains)
        if lod == 0 and not twigs.empty:
            bark = merge_parts("bark", [bark, twigs])
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
