# SPDX-License-Identifier: GPL-3.0-or-later
"""Dead-wood variants derived from a grown skeleton: standing snag and fallen log.

Both use the tree's own "variant" RNG stream, so the base tree's growth is unaffected. Fine twigs are
removed (pipe radii guarantee child radius <= parent radius, so a radius threshold keeps the graph
parent-closed). All nodes become dead (alive = 0).
"""
from __future__ import annotations

import numpy as np

from .rng import stream


def _final_season(sk):
    return int(sk.meta.get("start_season", 0)) + int(sk.meta.get("seasons", 0)) - 1


def _shorten_laterals(sk, keep, rng, max_frac, max_len):
    mc = sk.main_child()
    starts = np.nonzero((sk.parent >= 0) & ~sk.is_main & keep)[0]
    for s in starts:
        chain = []
        c = int(s)
        while c >= 0 and keep[c]:
            chain.append(c)
            c = int(mc[c])
        if not chain:
            continue
        n_keep = max(1, int(np.ceil(len(chain) * rng.uniform(0.1, max_frac))))
        L = np.concatenate([[0], np.cumsum(np.linalg.norm(np.diff(sk.pos[[sk.parent[s]] + chain], axis=0), axis=1))])
        n_keep = min(n_keep, int(np.searchsorted(L[1:], max_len * rng.uniform(0.4, 1.0))) + 1)
        for i in chain[n_keep:]:
            keep[i] = False
    # drop descendants of removed nodes
    for i in range(1, sk.n):
        if not keep[sk.parent[i]]:
            keep[i] = False
    return keep


def make_snag(sk, forest_seed, tree_id, min_r=0.02, break_frac=(0.45, 0.85), stub_frac=0.45, stub_len=2.5):
    rng = stream(forest_seed, tree_id, "variant")
    H = sk.pos[:, 2].max()
    hb = H * rng.uniform(*break_frac)
    keep = sk.radius >= min_r
    keep[0] = True
    stem_above = (sk.order == 0) & (sk.pos[:, 2] > hb)
    keep &= ~stem_above
    for i in range(1, sk.n):
        if not keep[sk.parent[i]]:
            keep[i] = False
    keep = _shorten_laterals(sk, keep, rng, stub_frac, stub_len)
    out = sk.subset(keep)
    out.alive[:] = False
    out.dead_since[:] = _final_season(sk)
    out.meta = dict(sk.meta, variant="snag", break_height=float(hb), variant_min_r=min_r)
    return out


def make_log(sk, forest_seed, tree_id, min_r=0.03, stub_len=0.6):
    rng = stream(forest_seed, tree_id, "variant")
    snag = make_snag(sk, forest_seed, tree_id, min_r=min_r, break_frac=(0.5, 0.9), stub_frac=0.25,
                     stub_len=stub_len)
    p = snag.pos.copy()
    stem = np.nonzero(snag.order == 0)[0]
    top = p[stem[np.argmax(p[stem, 2])]]
    az = rng.uniform(0, 2 * np.pi)
    target = np.array([np.cos(az), np.sin(az), np.tan(np.radians(rng.uniform(-3, 3)))])
    target /= np.linalg.norm(target)
    src = top / max(np.linalg.norm(top), 1e-9)
    v = np.cross(src, target)
    c = float(src @ target)
    vx = np.array([[0, -v[2], v[1]], [v[2], 0, -v[0]], [-v[1], v[0], 0]])
    R = np.eye(3) + vx + vx @ vx * (1.0 / (1.0 + c))
    # roll about the log axis so stubs splay randomly
    p = p @ R.T
    lift = float(np.max(snag.radius - p[:, 2]))
    p[:, 2] += lift
    snag.pos = p
    snag.meta = dict(snag.meta, variant="fallen_log", log_azimuth_deg=float(np.degrees(az)))
    return snag
