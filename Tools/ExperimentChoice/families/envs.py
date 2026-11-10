"""Procedural environment families with one interface (EXPLORATORY).

Every family maps integer-centimetre parameters to a ``FamilyCourse``: walls as axis-aligned
boxes, a start pose, a goal point and a stable id. A ``FamilyCourse`` offers what the surrogate
and the exporters of course.py use (``walls()``, ``boxes()``, ``start``, ``goal``, ``course_id``,
``params()``), so ``course.to_boxes``, ``to_occupancy_grid``, ``pgm_bytes``, ``map_yaml`` and
``to_sdf`` work unchanged. Each family also has a frozen admissible box with a seeded sampler and
an analytic feasibility statement, ``feasibility(course, robot)``, whose status is recorded in
``FAMILIES[name].status`` and tested against the brute-force lattice of cspace.py.

Frame: as geometry.py (start heading +x in every family).

Feasibility here is GEOMETRIC: 'fits' = a collision-free motion exists. In open free space this
does not depend on the drive type, because a pivot-capable differential drive (and a car) can
follow any path of the open free configuration space arbitrarily closely; the drive type changes
the number of stretches, not the verdict. Statements about a restricted controller class
(forward-only) are given separately and say so.
"""
from __future__ import annotations

import hashlib
import json
import math
import os
from dataclasses import dataclass

import numpy as np

from . import _PARENT, LABEL  # noqa: F401
from . import cspace as CS
from .robotspace import as_config
import course as C
import geometry as G
import mechanics as M

SCHEMA = "expchoice.family_course/v0"
WALL = C.WALL
GOAL_TOLERANCE = C.GOAL_TOLERANCE


def _cm(v) -> int:
    return int(round(float(v) * 100))


@dataclass(frozen=True)
class FamilyCourse:
    family: str
    params_cm: tuple            # sorted ((name, int), ...); the hashed content
    start_cm: tuple             # (x_cm, y_cm, heading mrad)
    goal_cm: tuple              # (x_cm, y_cm)
    wall_m: tuple               # ((name, xmin, xmax, ymin, ymax), ...) metres; derived, not hashed
    legacy_id: str = ""         # offset_doorway keeps the id of course.Course

    def __getattr__(self, name):                       # course.d, course.w ... in metres
        if name.startswith("_"):
            raise AttributeError(name)
        for k, v in object.__getattribute__(self, "params_cm"):
            if k == name + "_cm":
                return v / 100.0
            if k == name:
                return v
        raise AttributeError(name)

    @property
    def p(self) -> dict:
        return dict(self.params_cm)

    @property
    def start(self):
        return (self.start_cm[0] / 100.0, self.start_cm[1] / 100.0, self.start_cm[2] / 1000.0)

    @property
    def goal(self):
        return (self.goal_cm[0] / 100.0, self.goal_cm[1] / 100.0)

    def params(self) -> dict:
        d = {"schema": SCHEMA, "family": self.family, "start_cm_mrad": list(self.start_cm),
             "goal_cm": list(self.goal_cm), "wall_cm": _cm(WALL)}
        d.update(self.p)
        return d

    @property
    def course_id(self) -> str:
        if self.legacy_id:
            return self.legacy_id
        canon = json.dumps(self.params(), sort_keys=True, separators=(",", ":"))
        return hashlib.sha256(canon.encode()).hexdigest()[:16]

    def walls(self) -> list:
        return [{"name": n, "xmin": a, "xmax": b, "ymin": c, "ymax": d} for n, a, b, c, d in self.wall_m]

    def boxes(self) -> np.ndarray:
        return C.to_boxes(self)


def _course(family, params, start_cm, goal_cm, walls, legacy_id=""):
    wall_m = tuple((n, round(a / 100.0, 4), round(b / 100.0, 4), round(c / 100.0, 4), round(d / 100.0, 4))
                   for n, a, b, c, d in walls if b > a and d > c)
    return FamilyCourse(family, tuple(sorted((k, int(v)) for k, v in params.items())),
                        tuple(int(v) for v in start_cm), tuple(int(v) for v in goal_cm), wall_m, legacy_id)


def _room(x0, x1, y0, y1, w=None):
    """Four outer walls (cm) around the rectangle [x0, x1] x [y0, y1]."""
    w = _cm(WALL) if w is None else w
    return [("outer_back", x0 - w, x0, y0 - w, y1 + w), ("outer_front", x1, x1 + w, y0 - w, y1 + w),
            ("outer_neg", x0 - w, x1 + w, y0 - w, y0), ("outer_pos", x0 - w, x1 + w, y1, y1 + w)]


def _gate(prefix, x0, x1, y_lo, y_hi, centre, width):
    """Wall slab x0..x1 across y_lo..y_hi with an opening (cm; half-centimetres allowed)."""
    return [(prefix + "_neg", x0, x1, y_lo, centre - width / 2.0), (prefix + "_pos", x0, x1, centre + width / 2.0, y_hi)]


def _start_free(course, robot):
    return not bool(CS.hits(*course.start, robot.parts(), course.boxes())[0])


def _res(verdict, reason, **kw):
    out = {"verdict": verdict, "reason": reason}
    out.update(kw)
    return out


# =========================================================================== (a) offset doorway

DOORWAY_BOX = dict(C.ADMISSIBLE_BOX_V0)


def make_offset_doorway(w, e, d, B=5.0, t=0.20, start=None, goal=None) -> FamilyCourse:
    """Wraps course.make_course; the id is the id of the wrapped course."""
    return wrap_legacy(C.make_course(w, e, d, B=B, t=t, start=start, goal=goal))


def wrap_legacy(c) -> FamilyCourse:
    walls = [(b["name"], b["xmin"] * 100, b["xmax"] * 100, b["ymin"] * 100, b["ymax"] * 100) for b in c.walls()]
    return _course("offset_doorway", {"w_cm": c.w_cm, "e_cm": c.e_cm, "d_cm": c.d_cm, "B_cm": c.B_cm, "t_cm": c.t_cm},
                   c.start_cm, c.goal_cm, walls, legacy_id=c.course_id)


def to_legacy(course) -> "C.Course":
    p = course.p
    return C.Course(p["w_cm"], p["e_cm"], p["d_cm"], p["B_cm"], p["t_cm"], tuple(course.start_cm), tuple(course.goal_cm))


def doorway_feasibility(course, robot):
    """mechanics.certificate on the wrapped course (see the parent README for its status)."""
    cert = dict(M.certificate(to_legacy(course), robot))
    if cert["verdict"] == "does_not_fit" and robot.drive == "differential" and robot.W >= course.w > robot.L:
        # mechanics.certificate speaks about heading-first passage; sideways passage through an
        # opening wider than L is geometrically possible for a pivot drive (at a large stretch count).
        cert = _res("undetermined", "W >= w but L < w: no heading-first passage; a sideways shimmy is not excluded")
    cert["binding"] = {"d_over_2rho": course.d / (2 * robot.rho), "eps_over_W": (course.d - robot.L) / robot.W,
                       "e_over_W": abs(course.e) / robot.W, "w_over_W": course.w / robot.W}
    return cert


# =========================================================================== (b) chicane

CHICANE_APPROACH = 250      # cm, depth of the room before the first gate
CHICANE_EXIT = 250
CHICANE_BOX = {
    "name": "chicane_box_v0",
    "w1_cm": [80, 120], "w2_cm": [80, 120],
    "e1_cm": [-100, 100], "e2_cm": [-100, 100],
    "s_cm": [60, 260],             # chamber depth between the two walls
    "t_cm": [5, 40],
    "B_cm": [500, 500],
    "side_clearance_cm": 60,
}


def make_chicane(w1, e1, w2, e2, s, t=0.20, B=5.0) -> FamilyCourse:
    v = {"w1_cm": _cm(w1), "e1_cm": _cm(e1), "w2_cm": _cm(w2), "e2_cm": _cm(e2), "s_cm": _cm(s), "t_cm": _cm(t), "B_cm": _cm(B)}
    return _chicane(v)


def _chicane(v):
    A, t, s, B = CHICANE_APPROACH, v["t_cm"], v["s_cm"], v["B_cm"]
    x1, x2 = A, A + t + s
    xend = x2 + t + CHICANE_EXIT
    walls = _room(0, xend, -B / 2, B / 2)
    walls += _gate("gate1", x1, x1 + t, -B / 2, B / 2, v["e1_cm"], v["w1_cm"])
    walls += _gate("gate2", x2, x2 + t, -B / 2, B / 2, v["e2_cm"], v["w2_cm"])
    return _course("chicane", v, (A // 2, 0, 0), (x2 + t + _cm(C.GOAL_BEYOND), v["e2_cm"]), walls)


def _chicane_ok(v, box):
    lim = v["B_cm"] / 2 - box["side_clearance_cm"]
    return abs(v["e1_cm"]) + v["w1_cm"] / 2 <= lim and abs(v["e2_cm"]) + v["w2_cm"] / 2 <= lim


def chicane_band(course):
    """Widest straight band that passes both openings: (width, direction phi, offset c).

    The band is {p : |n.p - c| <= width/2}, n = (-sin phi, cos phi). Exact for the eight jamb
    corners (support of two point sets), maximised over phi numerically.
    """
    A, t, s = CHICANE_APPROACH / 100.0, course.t, course.s
    xs = [A, A + t, A + t + s, A + 2 * t + s]
    up = np.array([[xs[0], course.e1 + course.w1 / 2], [xs[1], course.e1 + course.w1 / 2],
                   [xs[2], course.e2 + course.w2 / 2], [xs[3], course.e2 + course.w2 / 2]])
    lo = up - np.array([[0, course.w1], [0, course.w1], [0, course.w2], [0, course.w2]])

    def width(phi):
        n = np.array([-math.sin(phi), math.cos(phi)])
        return float((up @ n).min() - (lo @ n).max())

    grid = np.linspace(-1.4, 1.4, 2801)
    k = int(np.argmax([width(p) for p in grid]))
    a, b = grid[max(k - 1, 0)], grid[min(k + 1, grid.size - 1)]
    gr = (math.sqrt(5) - 1) / 2
    for _ in range(60):
        c_, d_ = b - gr * (b - a), a + gr * (b - a)
        if width(c_) > width(d_):
            b = d_
        else:
            a = c_
    phi = 0.5 * (a + b)
    n = np.array([-math.sin(phi), math.cos(phi)])
    return width(phi), phi, 0.5 * float((up @ n).min() + (lo @ n).max())


def chicane_feasibility(course, robot):
    L, W, rc = robot.L, robot.W, robot.r_circ
    w1, w2, s, B = course.w1, course.w2, course.s, course.B
    de = abs(course.e2 - course.e1)
    bw, phi, c0 = chicane_band(course)
    x_in, x_out = CHICANE_APPROACH / 100.0 - rc, CHICANE_APPROACH / 100.0 + 2 * course.t + s + rc
    band_ends = all(abs((c0 + x * math.sin(phi)) / math.cos(phi)) <= B / 2 - rc for x in (x_in, x_out))
    slack = max(w1 - W, 0.0) / 2 + max(w2 - W, 0.0) / 2
    shift = max(de - slack, 0.0)
    binding = {"s_over_L": s / L, "s_over_2rho": s / (2 * robot.rho), "shift_over_W": shift / W,
               "band_over_W": bw / W, "band_angle": phi}
    if min(L, W) >= min(w1, w2):
        return _res("does_not_fit", "both footprint dimensions are at least the narrower gate", binding=binding)
    A = CHICANE_APPROACH / 100.0
    side = min(B / 2 - abs(course.e1), B / 2 - abs(course.e2))
    roomy_ends = A >= 2 * rc and side >= rc
    if W < min(w1, w2) and roomy_ends and s >= 2 * rc:
        return _res("fits", "roomy chamber (s >= pivot disc): pass gate 1 straight, pivot, shift, pivot, pass gate 2",
                    stretches_strip=1, binding=binding)
    if W < min(w1, w2) and roomy_ends and s > L and robot.is_rect:
        n = M.min_stretches_exact(shift, s, L, W, "rear")
        return _res("fits", "tight chamber (L < s < 2 rho): strip shimmy inside the chamber; the gate openings "
                            "only add room, so the strip count is an upper estimate", stretches_strip=n, binding=binding)
    if bw > W and roomy_ends and band_ends:
        return _res("fits", "straight band of width %.3f through both gates at %.1f deg" % (bw, math.degrees(phi)),
                    stretches_strip=1, binding=binding)
    return _res("undetermined", "s <= L and no straight band: a curved straddling pass is not analysed", binding=binding)


# =========================================================================== (c) L-bend

LBEND_IN = 300              # cm from the back wall to the inner corner
LBEND_OUT = 300             # cm of outgoing corridor beyond the incoming one
CHAMFER_STEP = 2            # cm, staircase step of the chamfer
LBEND_BOX = {
    "name": "lbend_box_v0",
    "a_cm": [80, 200],             # incoming corridor width
    "b_cm": [80, 200],             # outgoing corridor width
    "c_cm": [0, 40],               # inner-corner chamfer leg (multiple of CHAMFER_STEP)
    "step": {"c_cm": CHAMFER_STEP},
}


def make_lbend(a, b, c=0.0) -> FamilyCourse:
    return _lbend({"a_cm": _cm(a), "b_cm": _cm(b), "c_cm": _cm(c)})


def _lbend(v):
    a, b, c = v["a_cm"], v["b_cm"], v["c_cm"]
    if c % CHAMFER_STEP or c < 0:
        raise ValueError("chamfer must be a non-negative multiple of %d cm" % CHAMFER_STEP)
    X1, top, w = LBEND_IN, a + LBEND_OUT, _cm(WALL)
    walls = [("outer_back", -w, 0, -w, a + w), ("outer_bottom", -w, X1 + b + w, -w, 0),
             ("outer_right", X1 + b, X1 + b + w, 0, top + w), ("outer_top", X1 - w, X1 + b, top, top + w),
             ("inner_block", -w, X1 - c, a, top + w), ("inner_block_corner", X1 - c, X1, a + c, top + w)]
    for k in range(c // CHAMFER_STEP):                    # staircase inscribed in the chamfered solid
        x0 = X1 - c + k * CHAMFER_STEP
        walls.append(("inner_step_%02d" % k, x0, x0 + CHAMFER_STEP, a + (k + 1) * CHAMFER_STEP, a + c))
    return _course("lbend", v, (100, a / 2 if a % 2 == 0 else (a - 1) / 2, 0), (X1 + b // 2, top - 100), walls)


def lbend_admissible_headings(course, robot, n=1440):
    """(thetas, admissible) on [0, 2 pi): can the footprint be placed at this heading in the bend?

    Exact per heading for an unbounded bend: the inner block is closed under moves toward -x, +y,
    so if any placement at a heading is free then so is the one pushed into the outer corner
    (largest x, smallest y). That placement is tested with the exact overlap test. Holds for any
    footprint shape and any chamfer; the outer corner must be square.
    """
    parts = robot.parts()
    th = 2 * math.pi * np.arange(n) / n
    lo_x, hi_x, lo_y, hi_y = CS.extents(th, parts)
    X1b = LBEND_IN / 100.0 + course.b
    inner = np.array([[m["xmin"], m["xmax"], m["ymin"], m["ymax"]] for m in course.walls() if m["name"].startswith("inner")])
    adm = ~CS.hits(X1b - hi_x, -lo_y, th, parts, inner)
    return th, adm, (hi_x - lo_x), (hi_y - lo_y)


def lbend_closed_form(theta, a, b, L, W):
    """Sharp corner, centred rectangle, 0 <= theta <= pi/2 (heading from the incoming corridor):
    admissible iff  a cos + b sin - L sin cos >= W  or  L sin + W cos <= a  or  L cos + W sin <= b.  Exact."""
    c, s = np.cos(theta), np.sin(theta)
    return (a * c + b * s - L * s * c >= W) | (L * s + W * c <= a) | (L * c + W * s <= b)


def lbend_corner_margin(a, b, L, W, n=2001):
    """min over 0..pi/2 of the largest of the three slacks of ``lbend_closed_form`` (metres; >= 0 iff passable)."""
    th = np.linspace(0.0, math.pi / 2, n)
    c, s = np.cos(th), np.sin(th)
    return float(np.max([a * c + b * s - L * s * c - W, a - (L * s + W * c), b - (L * c + W * s)], axis=0).min())


def lbend_feasibility(course, robot):
    th, adm, xext, _ = lbend_admissible_headings(course, robot)
    n = th.size
    binding = {"corner_margin_over_W": lbend_corner_margin(course.a, course.b, robot.L, robot.W) / robot.W,
               "a_over_W": course.a / robot.W, "b_over_W": course.b / robot.W, "c_over_W": course.c / robot.W}
    if not adm[0]:
        return _res("does_not_fit", "footprint does not fit the incoming corridor at heading 0", binding=binding)
    n_pos = n_neg = 0                                     # admissible steps from heading 0, each way round
    while n_pos < n - 1 and adm[n_pos + 1]:
        n_pos += 1
    while n_neg < n - 1 and adm[(-n_neg - 1) % n]:
        n_neg += 1
    fits_out = xext <= course.b + 1e-9                    # heading at which the footprint fits the outgoing corridor
    step = 2 * math.pi / n
    cands = [k * step for k in range(n_pos + 1) if fits_out[k]][:1]
    cands += [-k * step for k in range(n_neg + 1) if fits_out[(-k) % n]][:1]
    if cands:
        target = min(cands, key=abs)                      # signed, not wrapped: the turn that is actually made
        return _res("fits", "headings admissible from 0 to one that fits the outgoing corridor (turn %.1f deg)"
                    % math.degrees(target), binding=binding, min_turn=abs(target), target_heading=target)
    return _res("does_not_fit", "no admissible run of headings links the incoming to the outgoing corridor", binding=binding)


def lbend_corner_path(course, robot, n=1500):
    """The constructive motion behind a 'fits' verdict, as dense poses (xs, ys, ths) for a swept check:
    translate to the outer corner at heading 0, turn to the target heading while staying pushed into
    the outer corner, then translate along the outgoing corridor. None if the verdict is not 'fits'."""
    rc = as_config(robot)
    f = lbend_feasibility(course, rc)
    if f["verdict"] != "fits" or not _start_free(course, rc):
        return None
    parts = rc.parts()
    X1b = LBEND_IN / 100.0 + course.b
    th = np.linspace(0.0, f["target_heading"], n)
    lo_x, hi_x, lo_y, hi_y = CS.extents(th, parts)
    cx, cy = X1b - hi_x, -lo_y
    sx, sy, _ = course.start
    u = np.linspace(0.0, 1.0, n)
    y_end = course.goal[1]
    xs = np.concatenate([sx + (cx[0] - sx) * u, np.full(n, cx[0]), cx, np.full(n, cx[-1])])
    ys = np.concatenate([np.full(n, sy), sy + (cy[0] - sy) * u, cy, cy[-1] + (y_end - cy[-1]) * u])
    ths = np.concatenate([np.zeros(2 * n), th, np.full(n, th[-1])])
    return xs, ys, ths


# =========================================================================== (d) dead-end bay

DEADEND_CROSS_LEN = 600     # cm, length of the cross corridor
DEADEND_START_BACK = 100    # cm, start pose this far from the closed end
DEADEND_BOX = {
    "name": "dead_end_box_v0",
    "Bw_cm": [80, 220],            # bay width
    "D_cm": [200, 400],            # bay depth
    "Cw_cm": [120, 300],           # cross-corridor width at the bay mouth
    "g_cm": [-200, 200],           # exit (goal) position along the cross corridor
}


def make_dead_end(Bw, D, Cw, g) -> FamilyCourse:
    return _dead_end({"Bw_cm": _cm(Bw), "D_cm": _cm(D), "Cw_cm": _cm(Cw), "g_cm": _cm(g)})


def _dead_end(v):
    Bw, D, Cw, H, w = v["Bw_cm"], v["D_cm"], v["Cw_cm"], DEADEND_CROSS_LEN, _cm(WALL)
    walls = [("cross_far", -Cw - w, -Cw, -H / 2 - w, H / 2 + w), ("cross_end_neg", -Cw, 0, -H / 2 - w, -H / 2),
             ("cross_end_pos", -Cw, 0, H / 2, H / 2 + w), ("bay_side_neg", 0, D + w, -H / 2 - w, -Bw / 2.0),
             ("bay_side_pos", 0, D + w, Bw / 2.0, H / 2 + w), ("bay_end", D, D + w, -Bw / 2.0, Bw / 2.0)]
    return _course("dead_end", v, (D - DEADEND_START_BACK, 0, 0), (-(Cw // 2), v["g_cm"]), walls)


def dead_end_alpha(Bw, L, W):
    """Largest |heading| any motion can reach inside a bay of width Bw: asin(Bw / 2 rho) - atan(W / L). Exact.
    (mechanics.alpha_max with the roles of L and W exchanged.) pi/2 means unconfined."""
    return M.alpha_max(Bw, W, L)


def dead_end_feasibility(course, robot):
    L, W, rc, two_rho = robot.L, robot.W, robot.r_circ, 2 * robot.rho
    Bw, Cw, D = course.Bw, course.Cw, course.D
    lo_y, hi_y = (float(v[0]) for v in CS.extents(0.0, robot.parts())[2:])
    turn = Bw >= two_rho
    alpha = dead_end_alpha(Bw, L, W) if robot.is_rect and Bw >= W else float("nan")
    nose = max(p[0] + p[2] / 2 for p in robot.parts())
    pivot_at_start = Bw >= 2 * rc and DEADEND_START_BACK / 100.0 >= rc and D - DEADEND_START_BACK / 100.0 >= 0
    out = {"binding": {"Bw_over_2rho": Bw / two_rho, "Bw_over_W": Bw / W, "Cw_over_2rho": Cw / two_rho,
                       "alpha_bay": alpha},
           "reverse_out_free": bool(hi_y <= Bw / 2 and -lo_y <= Bw / 2 and nose <= DEADEND_START_BACK / 100.0),
           "turnaround_in_bay": bool(turn),
           # a controller that never reverses: x of the rotation centre cannot decrease while |heading| < pi/2
           "forward_only_exit": "no" if not turn else ("yes" if pivot_at_start else "undetermined")}
    if not out["reverse_out_free"]:
        out.update(_res("does_not_fit", "start pose does not fit the bay"))
    elif Cw >= 2 * rc:
        out.update(_res("fits", "reverse straight out until the pivot disc is in the cross corridor, pivot, drive to the exit"))
    else:
        out.update(_res("undetermined", "cross corridor narrower than the pivot disc: the T-junction turn is not analysed"))
    return out


# =========================================================================== (e) occluded doorway

OCC_START_X = 100
OCC_BOX = {
    "name": "occluded_doorway_box_v0",
    "w_cm": [80, 120], "e_cm": [0, 150], "t_cm": [5, 40],
    "d_cm": [320, 450],            # bay depth (roomy on purpose: the occluder must not change the fit)
    "B_cm": [600, 600],
    "ow_cm": [40, 140],            # occluder size across the bay
    "ot_cm": [10, 40],             # occluder size along the bay
    "ox_cm": [120, 200],           # free depth between the occluder and the door wall
    "oy_cm": [-50, 150],           # occluder centre across the bay
    "side_clearance_cm": 60,
    "require_hidden": True,        # the opening must be fully hidden from the start point
}


def make_occluded_doorway(w, e, d, ow, ot, ox, oy, t=0.20, B=6.0) -> FamilyCourse:
    return _occluded({"w_cm": _cm(w), "e_cm": _cm(e), "d_cm": _cm(d), "t_cm": _cm(t), "B_cm": _cm(B),
                      "ow_cm": _cm(ow), "ot_cm": _cm(ot), "ox_cm": _cm(ox), "oy_cm": _cm(oy)})


def _occluded(v):
    base = C.make_course(v["w_cm"] / 100, v["e_cm"] / 100, v["d_cm"] / 100, B=v["B_cm"] / 100, t=v["t_cm"] / 100,
                         start=(OCC_START_X / 100, 0.0, 0.0))
    walls = [(b["name"], b["xmin"] * 100, b["xmax"] * 100, b["ymin"] * 100, b["ymax"] * 100) for b in base.walls()]
    x1 = v["d_cm"] - v["ox_cm"]
    walls.append(("occluder", x1 - v["ot_cm"], x1, v["oy_cm"] - v["ow_cm"] / 2.0, v["oy_cm"] + v["ow_cm"] / 2.0))
    return _course("occluded_doorway", v, base.start_cm, base.goal_cm, walls)


def _interval_minus(lo, hi, a, b):
    """Length of [lo, hi] minus [a, b]."""
    return (hi - lo) - max(0.0, min(hi, b) - max(lo, a))


def occluder_visibility(course, sensor_xy=None):
    """Fraction of the door opening (its near edge) seen from a point: exact for a point sensor.

    The opening subtends [atan2(e - w/2 - y, d - x), atan2(e + w/2 - y, d - x)]; the occluder, a
    convex box in front of the wall, shadows the angular interval spanned by its corners.
    """
    sx, sy = sensor_xy if sensor_xy is not None else course.start[:2]
    d, e, w = course.d, course.e, course.w
    lo, hi = math.atan2(e - w / 2 - sy, d - sx), math.atan2(e + w / 2 - sy, d - sx)
    x1 = d - course.ox
    x0 = x1 - course.ot
    if x0 <= sx:
        raise ValueError("sensor is not in front of the occluder")
    ang = [math.atan2(y - sy, x - sx) for x in (x0, x1) for y in (course.oy - course.ow / 2, course.oy + course.ow / 2)]
    vis = _interval_minus(lo, hi, min(ang), max(ang))
    return {"opening_interval": (lo, hi), "shadow_interval": (min(ang), max(ang)),
            "visible_fraction": max(vis, 0.0) / (hi - lo), "hidden": vis <= 1e-12}


def _occluded_ok(v, box):
    if v["e_cm"] + v["w_cm"] / 2 > v["B_cm"] / 2 - box["side_clearance_cm"]:
        return False
    if v["d_cm"] - v["ox_cm"] - v["ot_cm"] < OCC_START_X + 120:        # room between start and occluder
        return False
    if abs(v["oy_cm"]) + v["ow_cm"] / 2 > v["B_cm"] / 2 - 150:         # a passage on either side
        return False
    return (not box.get("require_hidden")) or occluder_visibility(_occluded(v))["hidden"]


def occluded_feasibility(course, robot):
    """The occluder leaves the doorway verdict unchanged when a pivot disc can go around it."""
    rc, W = robot.r_circ, robot.W
    d, B = course.d, course.B
    x0 = d - course.ox - course.ot
    side = max(B / 2 - (course.oy + course.ow / 2), (course.oy - course.ow / 2) + B / 2)
    sens = robot.sensor["mount"]
    vis = occluder_visibility(course, (course.start[0] + sens[0], course.start[1] + sens[1]))
    binding = {"visible_fraction": vis["visible_fraction"], "gap_behind_over_2r": course.ox / (2 * rc),
               "gap_side_over_2r": side / (2 * rc), "w_over_W": course.w / W}
    if min(robot.L, W) >= course.w:
        return _res("does_not_fit", "both footprint dimensions are at least the door width", binding=binding)
    disc = (course.ox >= 2 * rc and side >= 2 * rc and course.start[0] >= rc and x0 - course.start[0] >= rc
            and B / 2 - abs(course.e) >= rc)
    if W < course.w and disc:
        return _res("fits", "pivot disc passes beside and behind the occluder; then a roomy doorway (W < w)",
                    binding=binding, fit_unchanged=True)
    return _res("undetermined", "a gap around the occluder is narrower than the pivot disc", binding=binding,
                fit_unchanged=False)


# =========================================================================== (f) cluttered passage

CLUTTER_FIRST = 250         # cm, first station
CLUTTER_TAIL = 250          # cm after the last station
CLUTTER_BOX = {
    "name": "cluttered_box_v0",
    "gmin_cm": [70, 150],          # width of the narrowest gap that has to be used
    "K": [2, 5],                   # number of stations
    "pitch_cm": [120, 260],        # distance between stations
    "ps_cm": [20, 40],             # pillar size along the corridor
    "layout": [0, 9999],           # seed of the lateral arrangement
    "Bc_cm": [300, 300],
}


def make_cluttered(gmin, K, pitch, ps=0.30, layout=0, Bc=3.0) -> FamilyCourse:
    return _cluttered({"gmin_cm": _cm(gmin), "K": int(K), "pitch_cm": _cm(pitch), "ps_cm": _cm(ps),
                       "layout": int(layout), "Bc_cm": _cm(Bc)})


def _h(layout, k, salt, n):
    return int(hashlib.sha256(("%d:%d:%s" % (layout, k, salt)).encode()).hexdigest()[:8], 16) % n


def clutter_stations(v):
    """Per station: three gaps (cm, from -y to +y) around two pillars; the widest is G_k and
    min_k G_k = gmin by construction (one designated station has G = gmin)."""
    Bc, K, lay = v["Bc_cm"], v["K"], v["layout"]
    star = _h(lay, 0, "star", K)
    out = []
    for k in range(K):
        G_k = v["gmin_cm"] + (0 if k == star else 10 + _h(lay, k, "extra", 51))
        G_k = min(G_k, Bc - 100)
        cap = min(0.8 * G_k, (Bc - G_k - 40) / 2.0)
        others = [int(cap * (30 + _h(lay, k, "g%d" % i, 71)) / 100.0) for i in range(2)]
        slot = _h(lay, k, "slot", 3)
        gaps = others[:]
        gaps.insert(slot, G_k)
        out.append({"gaps_cm": gaps, "widest_cm": G_k, "pillar_cm": (Bc - sum(gaps)) / 2.0})
    return out


def _cluttered(v):
    Bc, K, ps = v["Bc_cm"], v["K"], v["ps_cm"]
    length = CLUTTER_FIRST + (K - 1) * v["pitch_cm"] + CLUTTER_TAIL
    walls = _room(0, length, -Bc / 2.0, Bc / 2.0)
    for k, st in enumerate(clutter_stations(v)):
        xc = CLUTTER_FIRST + k * v["pitch_cm"]
        y = -Bc / 2.0 + st["gaps_cm"][0]
        walls.append(("pillar_%d_a" % k, xc - ps / 2.0, xc + ps / 2.0, y, y + st["pillar_cm"]))
        y += st["pillar_cm"] + st["gaps_cm"][1]
        walls.append(("pillar_%d_b" % k, xc - ps / 2.0, xc + ps / 2.0, y, y + st["pillar_cm"]))
    return _course("cluttered", v, (100, 0, 0), (length - 100, 0), walls)


def cluttered_feasibility(course, robot):
    p = course.p
    L, W, rc = robot.L, robot.W, robot.r_circ
    gmin, free_depth = course.gmin, course.pitch - course.ps
    binding = {"gmin_over_W": gmin / W, "gmin_over_2rho": gmin / (2 * robot.rho),
               "free_depth_over_2rho": free_depth / (2 * robot.rho), "stations": p["K"]}
    if min(L, W) > gmin:
        return _res("does_not_fit", "every gap of the narrowest station is narrower than both footprint dimensions",
                    binding=binding)
    if gmin >= 2 * rc and free_depth >= 2 * rc and min(CLUTTER_FIRST - 100 - course.ps / 2 * 100, 100) / 100.0 >= rc:
        return _res("fits", "the pivot disc passes every station and can cross over between stations", binding=binding)
    likely = W < gmin and free_depth >= 2 * robot.rho
    return _res("undetermined", "between the disc bound and the width bound", binding=binding,
                heuristic="likely_fits" if likely else "unknown")


# =========================================================================== registry

@dataclass(frozen=True)
class Family:
    name: str
    box: dict
    build: object               # dict of integer parameters -> FamilyCourse
    admissible: object          # (dict, box) -> bool
    feasibility_fn: object
    binding: str                # the mechanical quantity that decides
    statement: str              # the analytic feasibility statement
    status: str                 # exact / first-order / heuristic, with scope

    def keys(self):
        return [k for k, v in self.box.items() if isinstance(v, list)]

    def box_hash(self) -> str:
        return hashlib.sha256(json.dumps(self.box, sort_keys=True, separators=(",", ":")).encode()).hexdigest()

    def sample(self, n, seed) -> list:
        """n courses drawn uniformly (integers) from the frozen box; deterministic in (seed, box)."""
        if self.name == "offset_doorway":
            return [wrap_legacy(c) for c in C.sample_courses(n, seed, self.box)]
        rng = np.random.Generator(np.random.PCG64(int(seed)))
        steps = self.box.get("step", {})
        out = []
        while len(out) < n:
            v = {}
            for k in self.keys():
                st = steps.get(k, 1)
                v[k] = self.box[k][0] + st * int(rng.integers(0, (self.box[k][1] - self.box[k][0]) // st + 1))
            if self.admissible(v, self.box):
                out.append(self.build(v))
        return out

    def feasibility(self, course, robot) -> dict:
        """Analytic statement for one (course, robot). Non-rectangular footprints: 'fits' verdicts
        use the bounding box (still valid), 'does_not_fit' verdicts are downgraded unless the
        statement handles the shape itself (L-bend)."""
        rc = as_config(robot)
        if not _start_free(course, rc):
            return {"verdict": "does_not_fit", "reason": "start pose collides", "family": self.name, "status": "exact"}
        out = dict(self.feasibility_fn(course, rc))
        if out["verdict"] == "does_not_fit" and not rc.is_rect and self.name != "lbend":
            out["verdict"], out["reason"] = "undetermined", "bounding box does not fit; composite shape not analysed"
        out["family"], out["status"] = self.name, self.status
        return out


FAMILIES = {f.name: f for f in (
    Family("offset_doorway", DOORWAY_BOX, lambda v: wrap_legacy(C.make_course(*(v[k] / 100 for k in ("w_cm", "e_cm", "d_cm", "B_cm", "t_cm")))),
           lambda v, box: v["e_cm"] + v["w_cm"] / 2 <= v["B_cm"] / 2 - box["side_clearance_cm"], doorway_feasibility,
           "pivot room d / 2 rho; sideways shift e under heading confinement alpha_max(d)",
           "mechanics.certificate: d >= 2 rho -> turn, shift, turn (no reversal); L < d < 2 rho -> shimmy with at least "
           "ceil(e / Delta(d)) stretches; W >= w -> no heading-first passage",
           "exact in the roomy and plain-strip cases (parent README); side-wall conditions unverified; "
           "ceil(e/Delta) minimality is lattice evidence only"),
    Family("chicane", CHICANE_BOX, _chicane, _chicane_ok, chicane_feasibility,
           "sideways shift |e2 - e1| under heading confinement in the chamber of depth s (s / L, s / 2 rho); "
           "width of the straight band through both gates",
           "fits if s >= pivot disc; fits if L < s (strip shimmy, count from Delta(s)); fits if a straight band of "
           "width > W passes both openings; does not fit if min(L, W) >= min(w1, w2); otherwise undetermined",
           "sufficient conditions exact; band width exact; necessity only via the gate width; stretch count "
           "first-order (the openings add room); s <= L without a band is undetermined"),
    Family("lbend", LBEND_BOX, _lbend, lambda v, box: True, lbend_feasibility,
           "swept-corner clearance: min over heading of a cos + b sin - L sin cos - W",
           "a heading is admissible iff the footprint pushed into the outer corner clears the inner block; the "
           "bend is passable iff the admissible run through heading 0 contains a heading that fits the outgoing "
           "corridor. Sharp corner, centred rectangle: a cos + b sin - L sin cos >= W or L sin + W cos <= a or "
           "L cos + W sin <= b",
           "exact both ways for an unbounded bend with a square outer corner, any footprint shape, staircase "
           "chamfer as built (headings sampled at 0.25 deg); stretch count not derived"),
    Family("dead_end", DEADEND_BOX, _dead_end, lambda v, box: abs(v["g_cm"]) <= DEADEND_CROSS_LEN / 2 - 100, dead_end_feasibility,
           "turn-around room Bw / 2 rho (decides whether a robot that never reverses can leave)",
           "reverse-out is free iff the footprint fits the bay at heading 0; headings inside the bay are confined "
           "to asin(Bw / 2 rho) - atan(W / L); a forward-only pivot robot cannot leave if Bw < 2 rho; the task "
           "fits if the cross corridor holds the pivot disc",
           "exact (heading confinement, forward-only impossibility, reverse-out); the T-junction turn for a "
           "narrow cross corridor is undetermined"),
    Family("occluded_doorway", OCC_BOX, _occluded, _occluded_ok, occluded_feasibility,
           "visibility: fraction of the door opening seen from the sensor at the start; fit is the roomy doorway's",
           "visible fraction = |opening interval minus occluder shadow| / |opening interval|; verdict unchanged "
           "by the occluder if every gap around it holds the pivot disc",
           "visibility exact for a point sensor (near edge of the opening, range and field of view not included); "
           "fit-unchanged is an exact sufficient condition"),
    Family("cluttered", CLUTTER_BOX, _cluttered, lambda v, box: True, cluttered_feasibility,
           "minimum gap gmin / W and cross-over room (pitch - ps) / 2 rho",
           "does not fit if min(L, W) > gmin; fits if gmin and the depth between stations hold the pivot disc; "
           "in between only a heuristic (W < gmin and depth >= 2 rho)",
           "heuristic in the middle range; the two outer bounds are exact (the lower one rests on the planar "
           "door lemma, quoted, not re-derived)"),
)}


def sample_all(n_per_family, seed) -> list:
    """n courses from every family; family k uses seed + k (order of FAMILIES)."""
    out = []
    for k, fam in enumerate(FAMILIES.values()):
        out += fam.sample(n_per_family, int(seed) + k)
    return out


# =========================================================================== manifest and export

def manifest(course, robot) -> dict:
    """Truth manifest (JSON-serialisable) for any family."""
    rc = as_config(robot)
    fam = FAMILIES[course.family]
    return {
        "schema": SCHEMA, "label": LABEL, "course_id": course.course_id, "family": course.family,
        "parameters": course.params(),
        "frame": "x along the start heading; y lateral; heading from +x (geometry.py)",
        "walls": course.walls(),
        "start": dict(zip(("x", "y", "theta"), course.start)),
        "goal": {"x": course.goal[0], "y": course.goal[1], "tolerance": GOAL_TOLERANCE},
        "robot": rc.to_dict(),
        "binding_quantity": fam.binding, "statement": fam.statement, "statement_status": fam.status,
        "feasibility": fam.feasibility(course, rc),
        "admissible_box": {"name": fam.box["name"], "sha256": fam.box_hash()},
    }


def manifest_json(course, robot) -> str:
    return json.dumps(manifest(course, robot), sort_keys=True, indent=1, default=float)


def write_course(out_dir, course, robot) -> dict:
    """Manifest, map (PGM + YAML) and SDF through the exporters of course.py."""
    os.makedirs(out_dir, exist_ok=True)
    stem = "%s_%s" % (course.family, course.course_id)
    occ, origin = C.to_occupancy_grid(course)
    paths = {k: os.path.join(out_dir, stem + ext) for k, ext in
             (("manifest", ".json"), ("pgm", ".pgm"), ("yaml", ".yaml"), ("sdf", ".sdf"))}
    with open(paths["manifest"], "w", encoding="utf-8") as f:
        f.write(manifest_json(course, robot))
    with open(paths["pgm"], "wb") as f:
        f.write(C.pgm_bytes(occ))
    with open(paths["yaml"], "w", encoding="utf-8") as f:
        f.write(C.map_yaml(stem + ".pgm", origin))
    with open(paths["sdf"], "w", encoding="utf-8") as f:
        f.write(C.to_sdf(course, world_name=stem))
    return paths
