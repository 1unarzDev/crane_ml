# SPDX-License-Identifier: GPL-3.0-or-later
"""Structural metrics of a skeleton: height, DBH, crown ratio, branching angles, diameters, volumes."""
from __future__ import annotations

import numpy as np

WYTHAM_DCLASS_MM = [0, 25, 50, 75, 100, 200, 500, 1000, 1e9]


def height(sk):
    return float(sk.pos[:, 2].max() - sk.pos[0, 2])


def dbh(sk, at=1.3):
    """Stem diameter at breast height (m) by interpolating order-0 internodes; nan if shorter."""
    stem = np.nonzero((sk.order == 0) & (sk.parent >= 0))[0]
    za = sk.pos[sk.parent[stem], 2]
    zb = sk.pos[stem, 2]
    hit = np.nonzero((za <= at) & (zb > at))[0]
    if len(hit) == 0:
        return float("nan")
    i = hit[0]
    t = (at - za[i]) / max(zb[i] - za[i], 1e-9)
    ra = sk.radius[sk.parent[stem[i]]]
    rb = sk.radius[stem[i]]
    return float(2 * (ra + t * (rb - ra)))


def crown_base_height(sk):
    tips = sk.tips() & sk.alive & (sk.order >= 1)
    if not tips.any():
        return float("nan")
    return float(np.percentile(sk.pos[tips, 2], 2))


def crown_ratio(sk):
    H = height(sk)
    cbh = crown_base_height(sk)
    return float((H - cbh) / H) if H > 0 and np.isfinite(cbh) else float("nan")


def crown_radius(sk):
    al = sk.alive
    r = np.hypot(sk.pos[al, 0] - sk.pos[0, 0], sk.pos[al, 1] - sk.pos[0, 1])
    return float(np.percentile(r, 98)) if len(r) else 0.0


def branching_angles(sk):
    """Dict order -> array of angles (deg) between a lateral's first internode and its parent axis."""
    lat = np.nonzero((sk.parent >= 0) & ~sk.is_main)[0]
    p = sk.parent[lat]
    dc = sk.pos[lat] - sk.pos[p]
    gp = sk.parent[p]
    dp = np.where((gp >= 0)[:, None], sk.pos[p] - sk.pos[np.maximum(gp, 0)], np.array([0, 0, 1.0]))
    cosang = (dc * dp).sum(1) / np.maximum(np.linalg.norm(dc, axis=1) * np.linalg.norm(dp, axis=1), 1e-12)
    ang = np.degrees(np.arccos(np.clip(cosang, -1, 1)))
    out = {}
    for o in np.unique(sk.order[lat]):
        out[int(o)] = ang[sk.order[lat] == o]
    return out


def branch_base_diameters(sk, alive_only=True):
    lat = np.nonzero((sk.parent >= 0) & ~sk.is_main)[0]
    if alive_only:
        lat = lat[sk.alive[lat]]
    return 2 * sk.radius[lat], sk.order[lat]


def volume_by_diameter_class(sk, edges_mm=WYTHAM_DCLASS_MM):
    kid = np.nonzero(sk.parent >= 0)[0]
    L = np.linalg.norm(sk.pos[kid] - sk.pos[sk.parent[kid]], axis=1)
    r = sk.radius[kid]
    vol = np.pi * r * r * L
    d_mm = 2000 * r
    idx = np.clip(np.searchsorted(edges_mm, d_mm, side="right") - 1, 0, len(edges_mm) - 2)
    return np.bincount(idx, weights=vol, minlength=len(edges_mm) - 1)


def pipe_exponent_fit(sk, radius=None):
    """Median per-junction exponent n solving r_p^n = sum r_c^n over junctions with >= 2 children."""
    r = sk.radius if radius is None else radius
    indptr, ind = sk.children_csr()
    nk = np.diff(indptr)
    js = np.nonzero(nk >= 2)[0]
    ns = []
    for j in js:
        ch = ind[indptr[j]:indptr[j + 1]]
        rp, rc = r[j], r[ch]
        if rp <= 0 or np.any(rc <= 0) or rc.max() >= rp:
            continue
        lo, hi = 0.5, 6.0
        f = lambda n: (rc ** n).sum() - rp ** n  # noqa: E731
        if f(lo) * f(hi) > 0:
            continue
        for _ in range(40):
            mid = 0.5 * (lo + hi)
            if f(lo) * f(mid) <= 0:
                hi = mid
            else:
                lo = mid
        ns.append(0.5 * (lo + hi))
    return float(np.median(ns)) if ns else float("nan"), np.asarray(ns)


def summary(sk):
    ang = branching_angles(sk)
    d, o = branch_base_diameters(sk)
    return {
        "height_m": round(height(sk), 3),
        "dbh_m": round(dbh(sk), 4),
        "crown_base_m": round(crown_base_height(sk), 3),
        "crown_ratio": round(crown_ratio(sk), 3),
        "crown_radius_m": round(crown_radius(sk), 3),
        "nodes": sk.n,
        "dead_nodes": int((~sk.alive).sum()),
        "max_order": int(sk.order.max()),
        "branch_angle_median_deg_by_order": {k: round(float(np.median(v)), 1) for k, v in ang.items() if len(v)},
        "n_branches_by_order": {int(k): int((o == k).sum()) for k in np.unique(o)},
    }
