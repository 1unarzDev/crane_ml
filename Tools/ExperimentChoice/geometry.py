"""Shared 2-D geometry for the experiment-choice track (EXPLORATORY).

Conventions used by every module in this directory:

* World frame: x runs along the bay depth (back wall inner face at x = 0, door wall at
  x in [d, d + t]); y is lateral. Heading theta is measured from +x, counter-clockwise.
* A robot pose (x, y, theta) is the pose of its ROTATION CENTRE. The footprint is a
  rectangle of length L (along heading) and width W. ``axle_offset`` a is the distance
  the rotation centre sits BEHIND the geometric centre, so the nose is L/2 + a ahead of
  the pose point and the tail L/2 - a behind it. a = 0 is the default everywhere.
* Boxes are axis-aligned, rows of (xmin, xmax, ymin, ymax) in metres.

numpy only.
"""
from __future__ import annotations

import numpy as np

_TOL = 1e-9


def wrap(a):
    """Wrap angle(s) to (-pi, pi]."""
    return np.arctan2(np.sin(a), np.cos(a))


def rect_corners(x, y, th, L, W, a=0.0):
    """Corners of the footprint, shape (..., 4, 2), order front-left, front-right, rear-right, rear-left."""
    x, y, th = np.broadcast_arrays(np.asarray(x, float), np.asarray(y, float), np.asarray(th, float))
    c, s = np.cos(th)[..., None], np.sin(th)[..., None]
    bx = np.array([L / 2 + a, L / 2 + a, -(L / 2 - a), -(L / 2 - a)])
    by = np.array([W / 2, -W / 2, -W / 2, W / 2])
    X = x[..., None] + c * bx - s * by
    Y = y[..., None] + s * bx + c * by
    return np.stack([X, Y], axis=-1)


def rect_box_overlap(x, y, th, L, W, boxes, a=0.0, tol=_TOL):
    """Separating-axis test, oriented rectangle against axis-aligned boxes.

    Returns a bool array of shape (N, M): True where the footprint at pose n overlaps box m
    with positive penetration (> tol). Touching is not a collision.
    """
    x = np.atleast_1d(np.asarray(x, float))
    y = np.atleast_1d(np.asarray(y, float))
    th = np.atleast_1d(np.asarray(th, float))
    x, y, th = np.broadcast_arrays(x, y, th)
    boxes = np.asarray(boxes, float).reshape(-1, 4)
    cor = rect_corners(x, y, th, L, W, a)                      # (N, 4, 2)
    rxmin, rxmax = cor[..., 0].min(-1)[:, None], cor[..., 0].max(-1)[:, None]
    rymin, rymax = cor[..., 1].min(-1)[:, None], cor[..., 1].max(-1)[:, None]
    bxmin, bxmax, bymin, bymax = (boxes[None, :, k] for k in range(4))
    ok = (np.minimum(rxmax, bxmax) - np.maximum(rxmin, bxmin) > tol)
    ok &= (np.minimum(rymax, bymax) - np.maximum(rymin, bymin) > tol)
    c, s = np.cos(th)[:, None], np.sin(th)[:, None]
    bcx, bcy = (bxmin + bxmax) / 2, (bymin + bymax) / 2
    bhx, bhy = (bxmax - bxmin) / 2, (bymax - bymin) / 2
    # rectangle long axis u = (c, s): rectangle spans [p.u - (L/2 - a), p.u + (L/2 + a)]
    pu = (x[:, None] * c + y[:, None] * s)
    bu = bcx * c + bcy * s
    br = bhx * np.abs(c) + bhy * np.abs(s)
    ok &= (np.minimum(pu + (L / 2 + a), bu + br) - np.maximum(pu - (L / 2 - a), bu - br) > tol)
    # rectangle short axis n = (-s, c)
    pn = (-x[:, None] * s + y[:, None] * c)
    bn = -bcx * s + bcy * c
    brn = bhx * np.abs(s) + bhy * np.abs(c)
    ok &= (np.minimum(pn + W / 2, bn + brn) - np.maximum(pn - W / 2, bn - brn) > tol)
    return ok


def rect_hits_boxes(x, y, th, L, W, boxes, a=0.0, tol=_TOL):
    """True per pose if the footprint overlaps any box."""
    return rect_box_overlap(x, y, th, L, W, boxes, a, tol).any(axis=1)


def raycast(origins, angles, boxes, max_range):
    """Ranges of rays against axis-aligned boxes (slab method).

    origins (N, 2); angles (K,) or (N, K) world-frame; boxes (M, 4). Returns (N, K) ranges,
    equal to ``max_range`` where nothing is hit within range.
    """
    origins = np.asarray(origins, float).reshape(-1, 2)
    angles = np.asarray(angles, float)
    if angles.ndim == 1:
        angles = np.broadcast_to(angles, (origins.shape[0], angles.shape[0]))
    boxes = np.asarray(boxes, float).reshape(-1, 4)
    dx, dy = np.cos(angles), np.sin(angles)
    dx = np.where(np.abs(dx) < 1e-12, 1e-12, dx)[..., None]
    dy = np.where(np.abs(dy) < 1e-12, 1e-12, dy)[..., None]
    ox, oy = origins[:, 0][:, None, None], origins[:, 1][:, None, None]
    tx1, tx2 = (boxes[:, 0] - ox) / dx, (boxes[:, 1] - ox) / dx
    ty1, ty2 = (boxes[:, 2] - oy) / dy, (boxes[:, 3] - oy) / dy
    tmin = np.maximum(np.minimum(tx1, tx2), np.minimum(ty1, ty2))
    tmax = np.minimum(np.maximum(tx1, tx2), np.maximum(ty1, ty2))
    hit = tmax >= np.maximum(tmin, 0.0)
    t = np.where(hit, np.maximum(tmin, 0.0), np.inf).min(axis=-1)
    return np.minimum(t, max_range)


def points_in_rect(px, py, x, y, th, L, W, a=0.0, margin=0.0):
    """Which points lie inside the footprint inflated by ``margin``.

    px, py (..., K); x, y, th (...,) broadcastable against px[..., 0]. Returns bool (..., K).
    """
    c, s = np.cos(th)[..., None], np.sin(th)[..., None]
    rx, ry = px - np.asarray(x)[..., None], py - np.asarray(y)[..., None]
    u = rx * c + ry * s
    n = -rx * s + ry * c
    return (u < L / 2 + a + margin) & (u > -(L / 2 - a) - margin) & (np.abs(n) < W / 2 + margin)
