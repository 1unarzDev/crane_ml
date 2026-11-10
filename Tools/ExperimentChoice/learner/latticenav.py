"""Independent controller pair for the offset doorway (EXPLORATORY, SURROGATE, not Nav2).

Why this exists: the original differential stand-ins in surrogate.py ('rotate_then_follow',
'stroke_shimmy'; called the CONSTRUCTED pair from here on) make the pilot circular. A is
written to stall when it cannot pivot and B carries two hand-set speed handicaps, so the
winner flip at d = 2 rho and the size of the roomy-bay gap are properties of the code.

This file adds the INDEPENDENT pair. Nothing in it refers to rho, alpha_max, stretch gains,
cycle counts or any other quantity from mechanics.py. Both arms are the same program:

  * occupancy on a 0.05 m grid from lidar hits (unknown = free, no clearing);
  * a state-lattice A* over (x, y, heading) with 32 headings (integer lattice directions up to
    (3, 1)), footprint collision checking against the occupancy with the footprint padded by
    half a cell, in-place rotation between neighbouring headings wherever the padded footprint
    permits it, cost = time at the speed limits;
  * a segment follower (rotate to heading, drive the straight piece, stop-in-time speed
    limit against the current scan);
  * one recovery: back up 0.15 m (or as far as the scan allows), re-plan; at most 6 per episode.

They differ in ONE setting:

  A' = 'lattice_forward'   forward motion primitives only            (allow_reverse = False)
  B' = 'lattice_reverse'   forward and reverse primitives, reverse cost x 2  (allow_reverse = True)

Speed, acceleration, margins, recovery and every other constant are identical.

Consequence worth knowing before reading results: B' can do everything A' can, so with equal
speeds A' can only win through execution accidents or through B' preferring a reversing plan
that turns out slower when driven. Whether a two-sided reversal exists here is an outcome.

numpy only. Registers itself in surrogate.EXTERNAL_CONTROLLERS on import.
"""
from __future__ import annotations

import heapq
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
import surrogate as S         # noqa: E402

LATTICE_VERSION = 1           # part of the truth-store key; bump on any behaviour change
ARM_FORWARD, ARM_REVERSE = "lattice_forward", "lattice_reverse"
INDEPENDENT_ARMS = (ARM_FORWARD, ARM_REVERSE)

# Generic settings (values in the range of Nav2 defaults; none derived from the mechanics).
LATTICE_CFG = {
    "res": 0.05,               # m, lattice and occupancy cell (Nav2 costmap default)
    "pad": 0.025,              # m, footprint padding in the planner (half a cell)
    "reverse_penalty": 2.0,    # cost multiplier on reverse primitives (Smac default)
    "goal_tol": 0.10,          # m, planner goal radius (the scenario's own tolerance is 0.25 m)
    "max_expansions": 40000,   # A* budget, then the plan fails ("planner timeout")
    "backup_dist": 0.15,       # m, recovery back-up (Nav2 BackUp default)
    "backup_speed": 0.15,      # m/s
    "max_recoveries": 6,       # Nav2 default number_of_retries
    "max_plans": 60,
    "k_heading": 2.0,          # 1/s
    "k_cross": 1.5,            # rad/m
    "max_steer": 0.15,         # rad, cross-track correction of the heading target
    "max_corr_rate": 0.5,      # rad/s while driving a straight piece
    "off_path": 0.12,          # m, cross-track error that forces a re-plan
}

# 8 lattice directions per quadrant, sorted by angle; rotated copies give 32 headings.
_QUAD = [(1, 0), (3, 1), (2, 1), (3, 2), (1, 1), (2, 3), (1, 2), (1, 3)]
DIRS = []
for _q in range(4):
    for _a, _b in _QUAD:
        for _ in range(_q):
            _a, _b = -_b, _a
        DIRS.append((_a, _b))
NH = len(DIRS)                                   # 32
ANGLES = [math.atan2(b, a) for a, b in DIRS]
STEP_LEN = [math.hypot(a, b) for a, b in DIRS]   # in cells


def _intermediate(a, b):
    """Cells checked between the ends of a straight primitive (a, b)."""
    n = max(abs(a), abs(b))
    return [(int(round(a * k / n)), int(round(b * k / n))) for k in range(1, n)]


def footprint_kernel(theta, L, W, pad, res):
    """Bool (2K+1, 2K+1): cell offsets whose centre lies inside the padded footprint at heading theta."""
    K = int(math.ceil((0.5 * math.hypot(L, W) + pad) / res)) + 1
    off = np.arange(-K, K + 1) * res
    X, Y = np.meshgrid(off, off, indexing="ij")
    c, s = math.cos(theta), math.sin(theta)
    u, n = X * c + Y * s, -X * s + Y * c
    return (np.abs(u) <= L / 2 + pad + 1e-9) & (np.abs(n) <= W / 2 + pad + 1e-9)


class LatticeGrid:
    """Occupancy from lidar hits plus configuration-space obstacle masks per heading."""

    BORDER = 4

    def __init__(self, xlim, ylim, robot, cfg):
        self.cfg, self.res = cfg, cfg["res"]
        self.x0, self.y0 = xlim[0], ylim[0]
        self.nx = int(math.ceil((xlim[1] - xlim[0]) / self.res))
        self.ny = int(math.ceil((ylim[1] - ylim[0]) / self.res))
        self.nc = self.nx * self.ny
        self.occ = np.zeros((self.nx, self.ny), bool)
        half = NH // 2
        # masks[0:16]: footprint at heading k (k and k + 16 coincide); masks[16:32]: footprint
        # half-way between headings k and k + 1 (checked by the rotation primitive).
        mids = [0.5 * (ANGLES[k] + (ANGLES[k + 1] if k + 1 < NH else ANGLES[0] + 2 * math.pi)) for k in range(half)]
        thetas = ANGLES[:half] + mids
        self.kern = np.stack([footprint_kernel(th, robot.L, robot.W, cfg["pad"], self.res) for th in thetas])
        self.K = self.kern.shape[1] // 2
        self.masks = np.zeros((2 * half, self.nx, self.ny), bool)
        b = self.BORDER
        self.masks[:, :b, :] = True
        self.masks[:, -b:, :] = True
        self.masks[:, :, :b] = True
        self.masks[:, :, -b:] = True
        self.version = 0

    def cell(self, x, y):
        return int(math.floor((x - self.x0) / self.res)), int(math.floor((y - self.y0) / self.res))

    def centre(self, i, j):
        return self.x0 + (i + 0.5) * self.res, self.y0 + (j + 0.5) * self.res

    def add_points(self, px, py):
        i = np.floor((px - self.x0) / self.res).astype(np.int64)
        j = np.floor((py - self.y0) / self.res).astype(np.int64)
        ok = (i >= 0) & (i < self.nx) & (j >= 0) & (j < self.ny)
        i, j = i[ok], j[ok]
        new = ~self.occ[i, j]
        if not new.any():
            return False
        flat = np.unique(i[new] * self.ny + j[new])
        K, nx, ny = self.K, self.nx, self.ny
        for f in flat.tolist():
            ci, cj = divmod(f, ny)
            self.occ[ci, cj] = True
            i0, i1, j0, j1 = max(ci - K, 0), min(ci + K + 1, nx), max(cj - K, 0), min(cj + K + 1, ny)
            self.masks[:, i0:i1, j0:j1] |= self.kern[:, i0 - ci + K:i1 - ci + K, j0 - cj + K:j1 - cj + K]
        self.version += 1
        return True

    def blocked(self, k, i, j):
        return bool(self.masks[k % (NH // 2), i, j])


class LatticePlanner:
    """A* over (heading, cell). States are integers s = k * nc + i * ny + j."""

    def __init__(self, grid, robot, goal_xy, cfg):
        self.g, self.cfg = grid, cfg
        res, ny = grid.res, grid.ny
        self.t_lin = [STEP_LEN[k] * res / robot.v_max for k in range(NH)]
        self.dcell = [a * ny + b for a, b in DIRS]
        self.inter = [[a * ny + b for a, b in _intermediate(*DIRS[k])] for k in range(NH)]
        self.t_rot = [abs(G.wrap(ANGLES[(k + 1) % NH] - ANGLES[k])) / robot.omega_max for k in range(NH)]
        ii, jj = np.meshgrid(np.arange(grid.nx), np.arange(grid.ny), indexing="ij")
        dist = np.hypot(grid.x0 + (ii + 0.5) * res - goal_xy[0], grid.y0 + (jj + 0.5) * res - goal_xy[1])
        self.goal = (dist <= cfg["goal_tol"]).ravel().tobytes()
        self.h = (np.maximum(dist - cfg["goal_tol"], 0.0) / robot.v_max).ravel().tolist()
        self.last_expansions = 0

    def snap(self, x, y, th):
        """Nearest collision-free lattice state to a continuous pose (within one cell / one heading), or None."""
        g = self.g
        i, j = g.cell(x, y)
        k0 = min(range(NH), key=lambda k: abs(G.wrap(ANGLES[k] - th)))
        best, bd = None, 1e9
        for dk in (0, 1, -1):
            k = (k0 + dk) % NH
            for di in (0, 1, -1):
                for dj in (0, 1, -1):
                    a, b = i + di, j + dj
                    if not (0 <= a < g.nx and 0 <= b < g.ny) or g.blocked(k, a, b):
                        continue
                    cx, cy = g.centre(a, b)
                    dd = math.hypot(cx - x, cy - y) + 0.2 * abs(G.wrap(ANGLES[k] - th))
                    if dd < bd:
                        best, bd = k * g.nc + a * g.ny + b, dd
            if best is not None and dk == 0:
                break
        return best

    def plan(self, start, allow_reverse):
        """Returns a list of (state, move) with move in 'S' (start), 'F', 'R', 'T' (turn), or None."""
        if start is None:
            return None
        g = self.g
        nc, half = g.nc, NH // 2
        m = g.masks.reshape(2 * half, nc)
        blk = np.concatenate([m[:half], m[:half]]).tobytes()          # indexed by state
        mid = m[half:].tobytes()                                      # indexed by (k % 16) * nc + c
        goal, h, t_lin, t_rot, dcell, inter = self.goal, self.h, self.t_lin, self.t_rot, self.dcell, self.inter
        rev_pen = self.cfg["reverse_penalty"]
        cap = self.cfg["max_expansions"]
        best = {start: 0.0}
        parent = {}
        heap = [(h[start % nc], 0.0, start)]
        pop, push = heapq.heappop, heapq.heappush
        nexp, found = 0, None
        while heap:
            _, gc, s = pop(heap)
            if gc > best[s]:
                continue
            k, c = divmod(s, nc)
            if goal[c]:
                found = s
                break
            nexp += 1
            if nexp > cap:
                break
            base = s - c
            # straight primitives
            for sign in ((1, -1) if allow_reverse else (1,)):
                c2 = c + sign * dcell[k]
                s2 = base + c2
                if blk[s2]:
                    continue
                free = True
                for o in inter[k]:
                    if blk[base + c + sign * o]:
                        free = False
                        break
                if not free:
                    continue
                ng = gc + (t_lin[k] if sign > 0 else rev_pen * t_lin[k])
                if ng < best.get(s2, 1e18):
                    best[s2] = ng
                    parent[s2] = (s, "F" if sign > 0 else "R")
                    push(heap, (ng + h[c2], ng, s2))
            # in-place rotation to the neighbouring headings
            k2 = k + 1 if k + 1 < NH else 0
            s2 = k2 * nc + c
            if not blk[s2] and not mid[(k % half) * nc + c]:
                ng = gc + t_rot[k]
                if ng < best.get(s2, 1e18):
                    best[s2] = ng
                    parent[s2] = (s, "T")
                    push(heap, (ng + h[c], ng, s2))
            k2 = k - 1 if k > 0 else NH - 1
            s2 = k2 * nc + c
            if not blk[s2] and not mid[(k2 % half) * nc + c]:
                ng = gc + t_rot[k2]
                if ng < best.get(s2, 1e18):
                    best[s2] = ng
                    parent[s2] = (s, "T")
                    push(heap, (ng + h[c], ng, s2))
        self.last_expansions = nexp
        if found is None:
            return None
        out = [(found, None)]
        while out[-1][0] != start:
            p, mv = parent[out[-1][0]]
            out[-1] = (out[-1][0], mv)
            out.append((p, None))
        out[-1] = (start, "S")
        out.reverse()
        return out

    def path_valid(self, path):
        g = self.g
        nc, half = g.nc, NH // 2
        m = g.masks.reshape(2 * half, nc)
        for s, _ in path:
            k, c = divmod(s, nc)
            if m[k % half, c]:
                return False
        return True

    def segments(self, path):
        """Compress a lattice path into rotate / straight segments for the follower."""
        g = self.g
        segs = []
        for s, mv in path[1:]:
            k, c = divmod(s, g.nc)
            i, j = divmod(c, g.ny)
            if mv == "T":
                if segs and segs[-1]["kind"] == "rot":
                    segs[-1]["target"] = ANGLES[k]
                else:
                    segs.append({"kind": "rot", "target": ANGLES[k]})
            else:
                d = 1 if mv == "F" else -1
                x, y = g.centre(i, j)
                if segs and segs[-1]["kind"] == "lin" and segs[-1]["k"] == k and segs[-1]["dir"] == d:
                    segs[-1]["x"], segs[-1]["y"] = x, y
                else:
                    segs.append({"kind": "lin", "k": k, "th": ANGLES[k], "dir": d, "x": x, "y": y})
        return segs


class LatticeFollower:
    """Controller interface of surrogate.Controller; sees lidar ranges and odometry only."""

    def __init__(self, kind, robot, start, goal, cfg):
        if kind not in INDEPENDENT_ARMS:
            raise ValueError(kind)
        if abs(robot.axle_offset) > 1e-9:
            raise NotImplementedError("rotation centre must be the footprint centre")
        self.kind, self.robot, self.goal = kind, robot, np.array(goal, float)
        self.allow_reverse = kind == ARM_REVERSE
        self.cfg = dict(LATTICE_CFG)
        for key, val in cfg.items():                      # sensitivity runs only: cfg={"lattice_pad": 0.0}
            if key.startswith("lattice_"):
                if key[8:] not in LATTICE_CFG:
                    raise KeyError(key)
                self.cfg[key[8:]] = val
        self.scfg = cfg                                   # margins shared with the constructed pair
        self.hl, self.hw = robot.L / 2, robot.W / 2
        xs, ys = (start[0], goal[0]), (start[1], goal[1])
        self.grid = LatticeGrid((min(xs) - 1.0, max(xs) + 1.2), (min(ys) - 2.3, max(ys) + 2.3), robot, self.cfg)
        self.planner = LatticePlanner(self.grid, robot, goal, self.cfg)
        self.pts = np.zeros((0, 2))
        self.path, self.segs = None, []
        self.need_plan = True
        self.recovery = None
        self.failed_key = None
        self.anchor, self.t_anchor = np.array(start, float), 0.0
        self.aborted = False
        self.abort_reason = ""
        self.n_plans = self.n_strokes = self.n_reversals = self.n_backups = self.n_recoveries = 0
        self.n_plan_fail = 0
        self.expansions = 0
        self._last_dir = 0
        self._seg_started = False

    # -- observations
    def on_scan(self, ranges, odom, t):
        K = ranges.size
        ang = odom[2] + np.arange(K) * (2 * math.pi / K)
        ok = ranges < self.robot.lidar["max_range"] - 0.05
        self.pts = np.stack([odom[0] + ranges[ok] * np.cos(ang[ok]), odom[1] + ranges[ok] * np.sin(ang[ok])], axis=1)
        if self.grid.add_points(self.pts[:, 0], self.pts[:, 1]) and self.path is not None and not self.need_plan:
            if not self.planner.path_valid(self.path[self.path_pos:]):
                self.need_plan = True

    def _body_points(self, odom):
        c, s = math.cos(odom[2]), math.sin(odom[2])
        rx, ry = self.pts[:, 0] - odom[0], self.pts[:, 1] - odom[1]
        return rx * c + ry * s, -rx * s + ry * c

    def _free(self, u, n, direction):
        c = self.scfg
        return float(S.free_straight(direction * u, n, self.hl, self.hw, c["margin_front"], c["margin_side"]))

    def _free_rot(self, u, n, sign):
        c = self.scfg
        return S.free_rotation(u, n, self.hl, self.hw, c["margin_front"], c["margin_hard"], sign)

    def _speed(self, limit, room, acc):
        if room <= 0:
            return 0.0
        return min(limit, math.sqrt(2 * self.scfg["decel_frac"] * acc * room), 0.6 * room / S.DT)

    def _abort(self, why):
        self.aborted, self.abort_reason = True, why

    def _progress(self, odom, t):
        if (math.hypot(odom[0] - self.anchor[0], odom[1] - self.anchor[1]) > S.STUCK_MOVE
                or abs(G.wrap(odom[2] - self.anchor[2])) > S.STUCK_TURN):
            self.anchor, self.t_anchor = np.array(odom, float), t
        if t - self.t_anchor > S.STUCK_S:
            self._abort("no motion")

    def _count_stroke(self, direction):
        self.n_strokes += 1
        if self._last_dir and direction != self._last_dir:
            self.n_reversals += 1
        self._last_dir = direction

    def _recover(self, u, n):
        """Back up and re-plan. Counts against max_recoveries whether or not there is room to move."""
        self.n_recoveries += 1
        self.path, self.segs = None, []
        if self.n_recoveries > self.cfg["max_recoveries"]:
            self._abort("recoveries exhausted")
            return
        back = min(self._free(u, n, -1), self.cfg["backup_dist"])
        if back >= 0.02:
            self.recovery = {"s": back, "x0": None}
            self.n_backups += 1
            self._count_stroke(-1)
        self.need_plan = True

    def _plan(self, odom, u, n):
        start = self.planner.snap(odom[0], odom[1], odom[2])
        key = (self.grid.version, start)
        if key == self.failed_key:                      # same map, same state: the search would fail again
            self.n_plan_fail += 1
            self._recover(u, n)
            return
        self.n_plans += 1
        if self.n_plans > self.cfg["max_plans"]:
            self._abort("plan budget")
            return
        path = self.planner.plan(start, self.allow_reverse)
        self.expansions += self.planner.last_expansions
        if path is None:
            self.failed_key = key
            self.n_plan_fail += 1
            self._recover(u, n)
            return
        self.path, self.path_pos = path, 0
        self.segs = self.planner.segments(path)
        k0 = path[0][0] // self.grid.nc
        if abs(G.wrap(ANGLES[k0] - odom[2])) > 0.02 and (not self.segs or self.segs[0]["kind"] != "rot"):
            self.segs.insert(0, {"kind": "rot", "target": ANGLES[k0]})
        self.need_plan = False
        self._seg_started = False

    # -- main entry
    def command(self, odom, twist, t):
        self._progress(odom, t)
        if self.aborted or self.pts.shape[0] == 0:
            return 0.0, 0.0, 0.0
        r = self.robot
        u, n = self._body_points(odom)
        moving = abs(twist[0]) > 0.02 or abs(twist[2]) > 0.03
        if self.recovery is not None:
            rec = self.recovery
            if rec["x0"] is None:
                if moving:
                    return 0.0, 0.0, 0.0
                rec["x0"], rec["y0"] = odom[0], odom[1]
            rem = rec["s"] - math.hypot(odom[0] - rec["x0"], odom[1] - rec["y0"])
            room = min(rem, self._free(u, n, -1))
            if rem < 0.005 or room < 0.002:
                self.recovery = None
                return 0.0, 0.0, 0.0
            return -self._speed(self.cfg["backup_speed"], room, r.acc), 0.0, 0.0
        if self.need_plan or not self.segs:
            if moving:
                return 0.0, 0.0, 0.0
            self._plan(odom, u, n)
            return 0.0, 0.0, 0.0
        seg = self.segs[0]
        if seg["kind"] == "rot":
            err = float(G.wrap(seg["target"] - odom[2]))
            if abs(err) < 0.01:
                self.segs.pop(0)
                return 0.0, 0.0, 0.0
            if abs(twist[0]) > 0.02:
                return 0.0, 0.0, 0.0
            sign = 1 if err > 0 else -1
            free = self._free_rot(u, n, sign)
            room = min(abs(err), free)
            if room < 0.004:
                if abs(err) > 0.03:
                    self._recover(u, n)
                else:
                    self.segs.pop(0)
                return 0.0, 0.0, 0.0
            return 0.0, 0.0, sign * self._speed(r.omega_max, room, r.ang_acc)
        # straight piece
        c, s = math.cos(seg["th"]), math.sin(seg["th"])
        rx, ry = seg["x"] - odom[0], seg["y"] - odom[1]
        d = seg["dir"]
        rem = d * (rx * c + ry * s)
        off = -(-rx * s + ry * c)                         # robot left of the line: positive
        herr = float(G.wrap(seg["th"] - odom[2]))
        if rem < 0.01:
            self.segs.pop(0)
            self._seg_started = False
            return 0.0, 0.0, 0.0
        if abs(off) > self.cfg["off_path"] or abs(herr) > 0.35:
            self.need_plan = True
            return 0.0, 0.0, 0.0
        if not self._seg_started:
            if abs(twist[2]) > 0.05:
                return 0.0, 0.0, 0.0
            self._seg_started = True
            self._count_stroke(d)
        room = min(rem, self._free(u, n, d))
        if room < 0.002:
            if rem > 0.03:
                self._recover(u, n)
            else:
                self.segs.pop(0)
                self._seg_started = False
            return 0.0, 0.0, 0.0
        v = d * self._speed(r.v_max, room, r.acc)
        steer = max(-self.cfg["max_steer"], min(self.cfg["max_steer"], self.cfg["k_cross"] * off))
        want = float(G.wrap(seg["th"] - d * steer - odom[2]))
        om = max(-self.cfg["max_corr_rate"], min(self.cfg["max_corr_rate"], self.cfg["k_heading"] * want))
        if abs(om) > 1e-3 and self._free_rot(u, n, 1 if om > 0 else -1) < abs(om) * 0.3:
            om = 0.0
        return v, 0.0, om


S.EXTERNAL_CONTROLLERS[ARM_FORWARD] = LatticeFollower
S.EXTERNAL_CONTROLLERS[ARM_REVERSE] = LatticeFollower
