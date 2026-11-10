# SPDX-License-Identifier: GPL-3.0-or-later
"""Headless matplotlib previews: radius-scaled skeleton line plots (side, front, top, 3D)."""
from __future__ import annotations

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402
import numpy as np  # noqa: E402
from matplotlib.collections import LineCollection  # noqa: E402
from mpl_toolkits.mplot3d.art3d import Line3DCollection  # noqa: E402


def _segments(sk, offset=(0, 0, 0), min_r=0.0):
    a, b, ra, rb, kid = sk.segment_arrays()
    m = rb >= min_r
    off = np.asarray(offset, dtype=float)
    return a[m] + off, b[m] + off, rb[m], sk.alive[kid][m]


def _colors(alive, base=(0.30, 0.22, 0.15), dead=(0.6, 0.55, 0.5)):
    c = np.where(alive[:, None], np.array(base)[None], np.array(dead)[None])
    return np.concatenate([c, np.ones((len(c), 1))], 1)


def _lw(r, px_per_m):
    return np.clip(2 * r * px_per_m, 0.15, 40)


def render_trees(items, path, title="", leaves=None, min_r=0.0, dpi=110):
    """items: list of (skeleton, offset_xyz_internal). Writes a 4-panel PNG."""
    fig = plt.figure(figsize=(16, 9))
    axs = [fig.add_subplot(1, 4, 1), fig.add_subplot(1, 4, 2), fig.add_subplot(1, 4, 3),
           fig.add_subplot(1, 4, 4, projection="3d")]
    allp = np.concatenate([sk.pos + np.asarray(o) for sk, o in items], 0)
    lo, hi = allp.min(0), allp.max(0)
    span = max(hi[2] - min(lo[2], 0), hi[0] - lo[0], hi[1] - lo[1], 1.0)
    px_per_m = 9 * dpi * 0.8 / span * 0.72
    for sk, off in items:
        a, b, r, alive = _segments(sk, off, min_r)
        col = _colors(alive)
        lw = _lw(r, px_per_m)
        for ax, (i, j) in zip(axs[:3], [(0, 2), (1, 2), (0, 1)]):
            seg = np.stack([a[:, [i, j]], b[:, [i, j]]], 1)
            ax.add_collection(LineCollection(seg, linewidths=lw, colors=col, capstyle="round"))
        seg3 = np.stack([a, b], 1)
        axs[3].add_collection3d(Line3DCollection(seg3, linewidths=np.clip(lw * 0.6, 0.1, 20), colors=col))
    if leaves is not None and len(leaves):
        for ax, (i, j) in zip(axs[:3], [(0, 2), (1, 2), (0, 1)]):
            ax.scatter(leaves[:, i], leaves[:, j], s=0.4, c=[(0.25, 0.5, 0.15, 0.25)], lw=0)
    names = ["side (X-Z)", "front (Y-Z)", "top (X-Y)"]
    for ax, nm, (i, j) in zip(axs[:3], names, [(0, 2), (1, 2), (0, 1)]):
        ax.set_xlim(lo[i] - 1, hi[i] + 1)
        ax.set_ylim((min(lo[j], 0) - 0.5) if j == 2 else lo[j] - 1, hi[j] + 1)
        ax.set_aspect("equal")
        ax.set_title(nm)
        ax.grid(alpha=0.3)
    ax3 = axs[3]
    c = (lo + hi) / 2
    h = span / 2
    ax3.set_xlim(c[0] - h, c[0] + h)
    ax3.set_ylim(c[1] - h, c[1] + h)
    ax3.set_zlim(0, 2 * h)
    ax3.set_box_aspect((1, 1, 1))
    ax3.view_init(elev=12, azim=35)
    ax3.set_title("3D")
    fig.suptitle(title)
    fig.tight_layout()
    fig.savefig(path, dpi=dpi)
    plt.close(fig)
