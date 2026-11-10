# SPDX-License-Identifier: GPL-3.0-or-later AND BSD-3-Clause
"""Space colonization (Runions et al. 2007) for conifer crowns and as an alternative bud environment.

`remove_matched_atts` and `space_colonization` are ported from Infinigen,
src/infinigen/assets/objects/trees/tree.py (Copyright (C) 2023, Princeton University, BSD 3-Clause,
author Alejandro Newell). Changes: the global np.random calls are replaced by an explicit
np.random.Generator, the TreeVertices class by plain arrays, and bpy/helper imports are removed.
The original BSD-3-Clause notice is reproduced in THIRD_PARTY_NOTICES.md (section "Infinigen trees").

`marker_environment` and `remove_occupied` (the SOTM bud-environment variant used by growth.py,
Palubicki 2009 section 4.1) are CRANE code under GPL-3.0-or-later.
"""
from __future__ import annotations

import warnings

import numpy as np


def compute_dists(atts: np.ndarray, vtxs: np.ndarray):
    deltas = vtxs[None, :, :] - atts[:, None, :]
    deltas = -deltas  # direction from vertex to attractor (atts - vtxs), as Infinigen's helper
    dists = np.linalg.norm(deltas, axis=-1)
    return dists, deltas


# ---------------------------------------------------------------- Infinigen port (BSD-3-Clause)
def remove_matched_atts(atts, vtxs, dist_thr, curr_min, curr_match, idx_offset=0, prev_deltas=None):
    dists, deltas = compute_dists(atts, vtxs)
    if prev_deltas is not None:
        deltas = np.append(prev_deltas, deltas, axis=1)

    min_dist = dists.min(1)
    closest = dists.argmin(1)
    to_keep = min_dist > dist_thr

    atts = atts[to_keep]
    min_dist = min_dist[to_keep]
    closest = closest[to_keep]
    deltas = deltas[to_keep]
    curr_min = curr_min[to_keep]
    curr_match = curr_match[to_keep]

    to_update = min_dist < curr_min
    curr_min[to_update] = min_dist[to_update]
    curr_match[to_update] = closest[to_update] + idx_offset

    return atts, deltas, curr_min, curr_match


def space_colonization(vtxs, parents, atts, rng: np.random.Generator, D=0.1, d=10.0, s=0.1,
                       pull_dir=None, dir_rand=0.1, mag_rand=0.15, n_steps=200):
    """Grow new vertices from `vtxs` toward attractors `atts`.

    Returns (vtxs, parents) with the new vertices appended. Parameters as in Infinigen:
    D step length, d initial match distance, s kill distance (> D), pull_dir growth bias.
    """
    vtxs = np.asarray(vtxs, dtype=np.float64).reshape(-1, 3)
    parents = list(parents)
    atts = np.asarray(atts, dtype=np.float64).reshape(-1, 3)
    if len(atts) == 0:
        return vtxs, np.asarray(parents)
    curr_min = np.zeros(len(atts)) + d
    curr_match = -np.ones(len(atts)).astype(int)
    atts, deltas, curr_min, curr_match = remove_matched_atts(atts, vtxs, s, curr_min, curr_match)

    if np.all(curr_match == -1):
        warnings.warn("Space colonization attractor matching failed, all curr_match == -1")
        return vtxs, np.asarray(parents)

    for _ in range(n_steps):
        new_vtxs = []
        new_parents = []
        matched_vtxs = np.unique(curr_match)
        for n_idx in matched_vtxs:
            if n_idx != -1:
                new_dir = deltas[curr_match == n_idx, n_idx].mean(0)
                new_dir = new_dir / np.linalg.norm(new_dir)
                if pull_dir is not None:
                    new_dir += pull_dir
                    new_dir = new_dir / np.linalg.norm(new_dir)
                new_dir += rng.standard_normal(3) * dir_rand
                tmp_D = D * np.exp(rng.standard_normal() * mag_rand)
                n0 = vtxs[n_idx]
                new_vtxs.append(n0 + tmp_D * new_dir)
                new_parents.append(int(n_idx))
        if not new_vtxs:
            break
        idx_offset = len(vtxs)
        new_vtxs = np.stack(new_vtxs, 0)
        vtxs = np.concatenate([vtxs, new_vtxs], 0)
        parents += new_parents
        atts, deltas, curr_min, curr_match = remove_matched_atts(
            atts, new_vtxs, s, curr_min, curr_match, idx_offset, deltas)
        if atts.shape[0] == 0:
            break
    return vtxs, np.asarray(parents)


# ---------------------------------------------------------------- CRANE SOTM environment (GPL-3.0)
def remove_occupied(markers: np.ndarray, node_pos: np.ndarray, r_occ: float, chunk: int = 2048):
    if len(markers) == 0 or len(node_pos) == 0:
        return markers
    keep = np.ones(len(markers), dtype=bool)
    r2 = r_occ * r_occ
    for i in range(0, len(node_pos), chunk):
        p = node_pos[i:i + chunk]
        d2 = ((markers[:, None, :] - p[None, :, :]) ** 2).sum(-1)
        keep &= d2.min(1) > r2
    return markers[keep]


def marker_environment(markers, bud_pos, bud_dir, r_perc, cos_perc, chunk=1024):
    """Each marker is claimed by the closest bud whose perception cone (radius r_perc, half-angle
    acos(cos_perc)) contains it. Returns per-bud marker count and unit mean direction (0 if none)."""
    B = len(bud_pos)
    cnt = np.zeros(B)
    vsum = np.zeros((B, 3))
    if B == 0 or len(markers) == 0:
        return cnt, vsum
    r2 = r_perc * r_perc
    for i in range(0, len(markers), chunk):
        m = markers[i:i + chunk]
        delta = m[:, None, :] - bud_pos[None, :, :]
        d2 = (delta ** 2).sum(-1)
        dist = np.sqrt(np.maximum(d2, 1e-18))
        cosang = (delta * bud_dir[None, :, :]).sum(-1) / dist
        ok = (d2 < r2) & (cosang >= cos_perc)
        d2m = np.where(ok, d2, np.inf)
        j = np.argmin(d2m, axis=1)
        claimed = np.isfinite(d2m[np.arange(len(m)), j])
        jj = j[claimed]
        u = delta[np.nonzero(claimed)[0], jj] / dist[np.nonzero(claimed)[0], jj][:, None]
        cnt += np.bincount(jj, minlength=B)
        for ax in range(3):
            vsum[:, ax] += np.bincount(jj, weights=u[:, ax], minlength=B)
    nrm = np.linalg.norm(vsum, axis=1, keepdims=True)
    v = np.where(nrm > 1e-12, vsum / np.maximum(nrm, 1e-12), 0.0)
    return cnt, v


def crown_markers(rng: np.random.Generator, shape: str, height: float, base: float, radius: float,
                  n: int) -> np.ndarray:
    """Attractor cloud filling a crown envelope (tree-local Z-up). shape: 'cone' | 'ellipsoid'."""
    pts = []
    need = n
    while need > 0:
        c = rng.uniform([-radius, -radius, base], [radius, radius, height], size=(need * 3, 3))
        t = (c[:, 2] - base) / max(height - base, 1e-6)
        rr = np.hypot(c[:, 0], c[:, 1])
        if shape == "cone":
            inside = rr <= radius * (1.0 - t) ** 0.9
        else:
            inside = (rr / radius) ** 2 + ((t - 0.5) / 0.5) ** 2 <= 1.0
        c = c[inside][:need]
        pts.append(c)
        need -= len(c)
    return np.concatenate(pts, 0)
