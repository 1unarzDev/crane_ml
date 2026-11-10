# SPDX-License-Identifier: GPL-3.0-or-later
"""Shared shadow-propagation voxel light field (Palubicki et al. 2009, section 4.2).

Each bud adds shadow a*b^-k to the voxels of a pyramid k layers below it (k = 0..q, half-width k).
Exposure Q = max(C - s + a, 0) / C (the +a cancels the bud's own contribution). One grid is shared by
every tree in a stand, which is what makes neighbouring trees compete for light.

Implementation: the per-voxel bud count is box-filtered per depth with a summed-area table, so cost is
O(q * voxels) independent of bud count.
"""
from __future__ import annotations

import numpy as np


def _fib_dirs(n: int) -> np.ndarray:
    i = np.arange(n) + 0.5
    phi = np.arccos(1 - 2 * i / n)
    th = np.pi * (1 + 5 ** 0.5) * i
    return np.stack([np.cos(th) * np.sin(phi), np.sin(th) * np.sin(phi), np.cos(phi)], 1)


class ShadowGrid:
    def __init__(self, lo, hi, voxel=0.5, a=0.08, b=1.8, q=6, C=1.0, perception_angle_deg=80.0,
                 perception_dist=2.0, n_dirs=48):
        lo = np.asarray(lo, dtype=np.float64)
        hi = np.asarray(hi, dtype=np.float64)
        self.voxel = float(voxel)
        self.lo = lo
        self.dims = np.maximum(np.ceil((hi - lo) / voxel).astype(int), 1)
        self.a, self.b, self.q, self.C = float(a), float(b), int(q), float(C)
        self.cos_perc = float(np.cos(np.radians(perception_angle_deg)))
        self.perc_dist = float(perception_dist) * self.voxel
        self.dirs = _fib_dirs(n_dirs)
        self.shadow = np.zeros(tuple(self.dims), dtype=np.float64)

    # -------------------------------------------------------------- indexing
    def _idx(self, p):
        return np.floor((p - self.lo) / self.voxel).astype(np.int64)

    def _lookup(self, p):
        ijk = self._idx(p)
        # horizontally clamp to the edge voxels so escaping branches gain no free light
        ijk[..., 0] = np.clip(ijk[..., 0], 0, self.dims[0] - 1)
        ijk[..., 1] = np.clip(ijk[..., 1], 0, self.dims[1] - 1)
        inside = (ijk[..., 2] >= 0) & (ijk[..., 2] < self.dims[2])
        out = np.zeros(p.shape[:-1], dtype=np.float64)
        ii = ijk[inside]
        out[inside] = self.shadow[ii[:, 0], ii[:, 1], ii[:, 2]]
        # below the ground plane counts as fully shaded so V never points into the soil
        out[ijk[..., 2] < 0] = 1e9
        return out

    # -------------------------------------------------------------- update
    def compute(self, bud_pos: np.ndarray, weights: np.ndarray | None = None) -> None:
        nx, ny, nz = self.dims
        src = np.zeros(nx * ny * nz, dtype=np.float64)
        if len(bud_pos):
            ijk = self._idx(bud_pos)
            ijk[:, 0] = np.clip(ijk[:, 0], 0, nx - 1)
            ijk[:, 1] = np.clip(ijk[:, 1], 0, ny - 1)
            ok = (ijk[:, 2] >= 0) & (ijk[:, 2] < nz)
            flat = (ijk[ok, 0] * ny + ijk[ok, 1]) * nz + ijk[ok, 2]
            w = None if weights is None else weights[ok]
            src = np.bincount(flat, weights=w, minlength=nx * ny * nz).astype(np.float64)
        src = src.reshape(nx, ny, nz)
        sat = np.zeros((nx + 1, ny + 1, nz), dtype=np.float64)
        sat[1:, 1:, :] = src.cumsum(0).cumsum(1)
        shadow = np.zeros_like(src)
        xs = np.arange(nx)
        ys = np.arange(ny)
        for k in range(self.q + 1):
            if k >= nz:
                break
            x0 = np.clip(xs - k, 0, nx)
            x1 = np.clip(xs + k + 1, 0, nx)
            y0 = np.clip(ys - k, 0, ny)
            y1 = np.clip(ys + k + 1, 0, ny)
            win = (sat[x1][:, y1] - sat[x0][:, y1] - sat[x1][:, y0] + sat[x0][:, y0])
            # voxel at layer z is shaded by sources at layer z + k
            shadow[:, :, : nz - k] += self.a * self.b ** (-k) * win[:, :, k:]
        self.shadow = shadow

    # -------------------------------------------------------------- queries
    def exposure(self, p: np.ndarray) -> np.ndarray:
        s = self._lookup(p)
        return np.maximum(self.C - s + self.a, 0.0) / self.C

    def best_direction(self, p: np.ndarray, d: np.ndarray) -> np.ndarray:
        """Unit vector V toward the least-shaded sample inside each bud's perception cone."""
        if len(p) == 0:
            return np.zeros((0, 3))
        cand = p[:, None, :] + self.perc_dist * self.dirs[None, :, :]
        s = self._lookup(cand.reshape(-1, 3)).reshape(len(p), -1)
        cosang = d @ self.dirs.T
        s = np.where(cosang >= self.cos_perc, s, np.inf)
        # tie-break deterministically toward the bud's own heading
        s = s - 1e-6 * cosang
        j = np.argmin(s, axis=1)
        v = self.dirs[j]
        none = ~np.isfinite(s[np.arange(len(p)), j])
        v[none] = d[none]
        return v
