"""Inputs offered to the learners, and the no-learning rules (EXPLORATORY).

  raw     (w, e, d) in metres
  slack   (w - W, e, d - L) in metres: raw parameters shifted by the footprint (used for transfer only;
          on one footprint it is the same model as raw)
  rawdim  raw plus an explicit robot-dimension vector (L, W, v_max, omega_max): the hardware-conditioned
          alternative to mechanics features. Constant when all training data come from one footprint.
  mech    dimensionless mechanics features, all taken from mechanics.py:
          d / 2 rho, e / W, w / W, log(1 + min(minimum reversals, 50)) / log 51, 1[d >= 2 rho]
  barn    BARN-style difficulty metrics computed from the map along a reference path:
          closest-wall distance (mean, min), dispersion (mean), characteristic dimension (mean, min)

Rules (recommend 'A' = the no-reverse arm or 'B' = the reverse-capable arm; no data used):
  rule_2rho     B if d < 2 rho else A
  rule_stretch  A if one forward stretch can make the offset (mechanics.min_stretches_exact <= 1,
                centre start; true in every roomy bay), else B
  always_A, always_B
"""
from __future__ import annotations

import math
import os
import sys

import numpy as np

_HERE = os.path.dirname(os.path.abspath(__file__))
_PARENT = os.path.dirname(_HERE)
for _p in (_PARENT, _HERE):
    if _p not in sys.path:
        sys.path.insert(0, _p)

import geometry as G          # noqa: E402
import mechanics as M         # noqa: E402

REV_CAP = 50
MECH_NAMES = ["d_over_2rho", "e_over_W", "w_over_W", "log_min_reversals", "roomy"]
BARN_NAMES = ["closest_mean", "closest_min", "dispersion_mean", "chardim_mean", "chardim_min"]


def raw(course):
    return [course.w, abs(course.e), course.d]


def slack(course, robot):
    return [course.w - robot.W, abs(course.e), course.d - robot.L]


def dims(robot):
    """Explicit robot-dimension vector (hardware-conditioned input): L, W and the speed limits."""
    return [robot.L, robot.W, robot.v_max, robot.omega_max]


def rawdim(course, robot):
    return raw(course) + dims(robot)


def mech(course, robot):
    L, W = robot.L, robot.W
    e, d = abs(course.e), course.d
    rev = M.min_reversals_exact(e, d, L, W, "centre")
    rev = REV_CAP if rev == M.INF else min(rev, REV_CAP)
    return [d / (2 * M.rho(L, W)), e / W, course.w / W, math.log1p(rev) / math.log1p(REV_CAP),
            1.0 if M.regime(d, L, W) == "roomy" else 0.0]


# --------------------------------------------------------------------------- BARN-style metrics

def reference_path(course, step=0.05):
    """Start -> sideways to the door axis at mid-depth -> straight through the door to the goal."""
    sx, sy = course.start[0], course.start[1]
    gx, gy = course.goal
    pts = [(sx, sy), (sx, gy), (gx, gy)]
    out = [np.array([[sx, sy]])]
    for (x0, y0), (x1, y1) in zip(pts[:-1], pts[1:]):
        n = int(math.ceil(math.hypot(x1 - x0, y1 - y0) / step))
        if n:
            u = np.arange(1, n + 1)[:, None] / n
            out.append(np.array([[x0, y0]]) + u * np.array([[x1 - x0, y1 - y0]]))
    return np.concatenate(out)


def closest_wall_distance(points, boxes):
    """Exact distance from each point to the nearest axis-aligned box. (N,)"""
    px, py = points[:, 0][:, None], points[:, 1][:, None]
    dx = np.maximum(np.maximum(boxes[None, :, 0] - px, px - boxes[None, :, 1]), 0.0)
    dy = np.maximum(np.maximum(boxes[None, :, 2] - py, py - boxes[None, :, 3]), 0.0)
    return np.hypot(dx, dy).min(axis=1)


def barn(course, n_rays=16, radius=1.0, max_range=6.0):
    """Simple versions of three BARN metrics (Perille et al. 2020), averaged along the reference path.

    closest-wall distance: exact distance to the nearest wall.
    dispersion: number of free/occupied alternations around a ring of n_rays rays, a ray being
      'occupied' when it hits a wall within ``radius``.
    characteristic dimension: the smallest wall-to-wall extent through the point over n_rays / 2 axes.
    """
    boxes = course.boxes()
    pts = reference_path(course)
    cw = closest_wall_distance(pts, boxes)
    ang = np.arange(n_rays) * (2 * math.pi / n_rays)
    rng = G.raycast(pts, ang, boxes, max_range)
    occ = rng < radius
    disp = (occ != np.roll(occ, 1, axis=1)).sum(axis=1)
    half = n_rays // 2
    cd = (rng[:, :half] + rng[:, half:]).min(axis=1)
    return [float(cw.mean()), float(cw.min()), float(disp.mean()), float(cd.mean()), float(cd.min())]


# --------------------------------------------------------------------------- matrices and scaling

def matrix(kind, courses, robot):
    f = {"raw": lambda c: raw(c), "slack": lambda c: slack(c, robot), "mech": lambda c: mech(c, robot),
         "rawdim": lambda c: rawdim(c, robot),
         "barn": lambda c: barn(c)}[kind]
    return np.array([f(c) for c in courses], float)


class Scaler:
    """Affine map to [0, 1] per column, fixed from a reference matrix (the pool); constant columns map to 0."""

    def __init__(self, ref):
        self.lo = ref.min(axis=0)
        span = ref.max(axis=0) - self.lo
        self.span = np.where(span > 1e-12, span, 1.0)

    def __call__(self, X):
        return (np.asarray(X, float) - self.lo) / self.span


# --------------------------------------------------------------------------- no-learning rules

def rule_2rho(course, robot):
    return "B" if course.d < 2 * M.rho(robot.L, robot.W) else "A"


def rule_stretch(course, robot):
    return "A" if M.min_stretches_exact(abs(course.e), course.d, robot.L, robot.W, "centre") <= 1 else "B"


RULES = {"rule_2rho": rule_2rho, "rule_stretch": rule_stretch,
         "always_A": lambda c, r: "A", "always_B": lambda c, r: "B"}
