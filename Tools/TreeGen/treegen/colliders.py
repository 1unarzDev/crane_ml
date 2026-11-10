# SPDX-License-Identifier: GPL-3.0-or-later
"""Capsule-chain colliders and soft-obstacle lists from the full-resolution skeleton (never LOD'd).

A segment (internode parent->child, diameter 2*radius[child]) is HARD if its diameter >= the threshold
for its height: `band_min_d` (default 2 cm) where the segment overlaps the band [band_lo, band_hi]
(default 1-4 m above the tree base), else `default_min_d` (5 cm). Consecutive hard segments on one
axis are merged into a capsule while every interior node stays within `merge_tol_frac * r` of the
capsule axis and the capsule stays shorter than `max_capsule_len`. Capsule radius = max radius in the
run, so the capsule always contains the branch axis. Everything thinner (>= soft_min_d) goes to the
soft-obstacle list as merged segments with the same rule.
"""
from __future__ import annotations

import numpy as np

from .skeleton import to_unity

DEFAULTS = {"band_lo": 1.0, "band_hi": 4.0, "band_min_d": 0.02, "default_min_d": 0.05,
            "soft_min_d": 0.004, "merge_tol_frac": 0.25, "max_capsule_len": 2.5}


def segment_threshold(za, zb, cfg):
    lo, hi = np.minimum(za, zb), np.maximum(za, zb)
    in_band = (hi >= cfg["band_lo"]) & (lo <= cfg["band_hi"])
    return np.where(in_band, cfg["band_min_d"], cfg["default_min_d"])


def hard_mask(sk, cfg=None):
    """Per node (child end of its internode): True if that internode must be covered by a capsule."""
    cfg = {**DEFAULTS, **(cfg or {})}
    kid = np.nonzero(sk.parent >= 0)[0]
    za = sk.pos[sk.parent[kid], 2]
    zb = sk.pos[kid, 2]
    thr = segment_threshold(za, zb, cfg)
    m = np.zeros(sk.n, dtype=bool)
    m[kid] = 2 * sk.radius[kid] >= thr
    return m


def _point_seg_dist(p, a, b):
    ab = b - a
    t = np.clip(((p - a) * ab).sum(-1) / np.maximum((ab * ab).sum(-1), 1e-18), 0, 1)
    return np.linalg.norm(p - (a + t[..., None] * ab), axis=-1)


def _merge_runs(sk, chains, mask, cfg):
    """Yield (node_ids list) runs of consecutive masked internodes along each chain."""
    out = []
    pos, rad = sk.pos, sk.radius
    for ch in chains:
        run = []
        for idx in range(1, len(ch)):
            c = int(ch[idx])
            if not mask[c]:
                if len(run) > 1:
                    out.append(run)
                run = []
                continue
            if not run:
                run = [int(ch[idx - 1]), c]
                continue
            cand = run + [c]
            a, b = pos[cand[0]], pos[c]
            r = rad[cand[1:]].max()
            inner = pos[cand[1:-1]]
            ok = np.linalg.norm(b - a) <= cfg["max_capsule_len"]
            if ok and len(inner):
                ok = _point_seg_dist(inner, a[None], b[None]).max() <= cfg["merge_tol_frac"] * r
            # do not swallow a strong taper into one fat capsule
            ok = ok and rad[cand[1]] <= 1.35 * rad[c] + 1e-9
            if ok:
                run = cand
            else:
                out.append(run)
                run = [run[-1], c]
        if len(run) > 1:
            out.append(run)
    return out


def build_colliders(sk, cfg=None):
    cfg = {**DEFAULTS, **(cfg or {})}
    chains = sk.chains()
    hard = hard_mask(sk, cfg)
    kid = np.nonzero(sk.parent >= 0)[0]
    soft = np.zeros(sk.n, dtype=bool)
    soft[kid] = (~hard[kid]) & (2 * sk.radius[kid] >= cfg["soft_min_d"])
    caps = []
    for run in _merge_runs(sk, chains, hard, cfg):
        a = sk.pos[run[0]]
        b = sk.pos[run[-1]]
        r = float(sk.radius[run[1:]].max())
        caps.append((a, b, r, run[0], run[-1], bool(sk.alive[run[1:]].all())))
    softs = []
    for run in _merge_runs(sk, chains, soft, {**cfg, "merge_tol_frac": 1.0, "max_capsule_len": 3.0}):
        softs.append((sk.pos[run[0]], sk.pos[run[-1]], float(sk.radius[run[1:]].max()), run[0], run[-1],
                      bool(sk.alive[run[1:]].all())))
    return caps, softs, cfg


def _v3(p):
    u = to_unity(p)
    return {"x": round(float(u[0]), 5), "y": round(float(u[1]), 5), "z": round(float(u[2]), 5)}


def colliders_json(sk, caps, softs, cfg, foliage_volumes=None) -> dict:
    def cap_entry(c):
        a, b, r, i0, i1, alive = c
        au, bu = to_unity(a), to_unity(b)
        L = float(np.linalg.norm(bu - au))
        axis = (bu - au) / L if L > 1e-9 else np.array([0, 1.0, 0])
        return {"a": _v3(a), "b": _v3(b), "radius": round(r, 5),
                "center": {"x": round(float((au[0] + bu[0]) / 2), 5), "y": round(float((au[1] + bu[1]) / 2), 5),
                           "z": round(float((au[2] + bu[2]) / 2), 5)},
                "axis": {"x": round(float(axis[0]), 6), "y": round(float(axis[1]), 6), "z": round(float(axis[2]), 6)},
                "height": round(L + 2 * r, 5), "node_a": int(i0), "node_b": int(i1), "dead": not alive}

    fv = []
    if foliage_volumes is not None:
        for c in foliage_volumes:
            fv.append({"center": _v3(c[:3]), "radius": round(float(c[3]), 4)})
    return {
        "schema": "crane.treegen.colliders",
        "schema_version": 1,
        "frame": "unity_lh_yup_m_tree_local",
        "tree_id": sk.meta.get("tree_id"),
        "species": sk.meta.get("species"),
        "skeleton_hash": sk.topology_hash(),
        "thresholds": {k: cfg[k] for k in DEFAULTS},
        "capsules": [cap_entry(c) for c in caps],
        "soft_obstacles": [cap_entry(c) for c in softs],
        "foliage_volumes": fv,
    }


def coverage_report(sk, caps, cfg=None, samples=3, eps=1e-6):
    """Fraction of hard internodes whose axis sample points lie inside some capsule."""
    cfg = {**DEFAULTS, **(cfg or {})}
    hard = hard_mask(sk, cfg)
    ids = np.nonzero(hard)[0]
    if len(ids) == 0:
        return 1.0, np.zeros(0, dtype=np.int64)
    A = np.array([c[0] for c in caps]) if caps else np.zeros((0, 3))
    B = np.array([c[1] for c in caps]) if caps else np.zeros((0, 3))
    R = np.array([c[2] for c in caps]) if caps else np.zeros(0)
    t = np.linspace(0, 1, samples)
    pa = sk.pos[sk.parent[ids]]
    pb = sk.pos[ids]
    covered = np.ones(len(ids), dtype=bool)
    for tt in t:
        p = pa + (pb - pa) * tt
        ok = np.zeros(len(ids), dtype=bool)
        for i in range(0, len(A), 512):
            d = _point_seg_dist(p[:, None, :], A[None, i:i + 512], B[None, i:i + 512])
            ok |= (d <= R[None, i:i + 512] + eps).any(1)
        covered &= ok
    return float(covered.mean()), ids[~covered]
