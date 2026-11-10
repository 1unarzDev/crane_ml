"""Instance generators."""
from __future__ import annotations

import numpy as np

from .model import Instance


def grid_instance(w: int, h: int, *, seed: int = 0, prior: str = "blobs",
                  g_range=(0.3, 0.9), q=0.8, oblique_gain=0.0,
                  h_good=0.3, h_bad=0.15, alpha=0.02, abort=0.0,
                  horizon: int | None = None, travel_scale: float = 1.0,
                  starts=None, name=None) -> Instance:
    """w x h grid of cells with two view classes.

    View 0 (along-track) has visibility g ~ U(g_range); view 1 (oblique) is
    drawn independently and shifted by ``oblique_gain`` (canopy-dependent in
    practice).  Travel time is Manhattan distance * travel_scale (rounded).
    """
    rng = np.random.default_rng(seed)
    n = w * h
    xy = np.array([(x, y) for y in range(h) for x in range(w)], float)
    if prior == "uniform":
        p = np.ones(n)
    elif prior == "blobs":
        p = np.full(n, 0.05)
        for _ in range(max(1, n // 12)):
            c = xy[rng.integers(n)]
            s = rng.uniform(0.8, 2.0)
            p += rng.uniform(0.5, 1.5) * np.exp(-((xy - c) ** 2).sum(1) / (2 * s * s))
    else:
        raise ValueError(prior)
    g = rng.uniform(*g_range, size=(n, 2))
    g[:, 1] = np.clip(g[:, 1] + oblique_gain, 0.0, 1.0)
    qq = np.full((n, 2), q)
    D = np.abs(xy[:, None, :] - xy[None, :, :]).sum(-1)
    dist = np.rint(D * travel_scale).astype(int)
    if starts is None:
        starts = (0, n - 1)
    return Instance(prior=p, g=g, q=qq, h_good=h_good, h_bad=h_bad,
                    alpha=alpha, abort=abort, dist=dist, starts=starts,
                    horizon=horizon or 2 * n, coords=xy,
                    name=name or f"grid{w}x{h}-s{seed}")
