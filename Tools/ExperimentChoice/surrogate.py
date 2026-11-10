"""Deterministic numpy kinematic surrogate of the offset-doorway scenario (EXPLORATORY, SURROGATE).

What it is: a batched kinematic simulator (dt 0.05 s, velocity and acceleration limits, exact
footprint-vs-box collision, timeout), a 2-D lidar with counter-based seeded noise (paired runs
share the noise), and hand-written controller STAND-INS that see only lidar ranges and
odometry. It is not Nav2 and nothing here has been compared with Nav2; see README.md.

Controller stand-ins (names are behaviours, not Nav2 plugins):
  differential  A = 'rotate_then_follow'  rotate in place to the path heading, then pure pursuit
                                          forward; never reverses; aborts when it cannot move.
                B = 'stroke_shimmy'       reverse-capable; when it cannot simply follow, picks
                                          greedy rotate-then-straight strokes (forward or
                                          reverse) from lidar clearances, which yields K-turns
                                          and shimmies. Runs at reduced speed (declared constants).
  mecanum       A = 'holonomic'           translates toward the path carrot at fixed heading.
                B = 'rotate_then_follow'  the differential-style follower (no lateral motion).

Score: J = time to goal if reached, else T_CAP + a fixed penalty (constants below).

CLI:  python3 -I surrogate.py grid --robot compact_diff      (resumable JSONL in ~/.cache/expchoice/)
      python3 -I surrogate.py summarize --robot compact_diff
"""
from __future__ import annotations

import argparse
import heapq
import json
import math
import os
import sys
import time

import numpy as np

try:
    from . import course as C, geometry as G, mechanics as M, robots as R
except ImportError:
    sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
    import course as C
    import geometry as G
    import mechanics as M
    import robots as R

LABEL = "exploratory-surrogate"
SURROGATE_VERSION = 2     # bump when controller logic or defaults change (part of the grid resume key)

# ---- explicit constants ------------------------------------------------------------------
DT = 0.05                 # s, simulation and control step
T_CAP = 120.0             # s, score cap and simulation timeout
PEN_COLLISION = 60.0      # s, added to T_CAP on collision
PEN_TIMEOUT = 30.0        # s, added to T_CAP on timeout or controller abort ('stuck')
SCAN_EVERY = 2            # ticks between lidar scans (10 Hz)
ODOM_SIGMA = 0.01         # fractional noise on each odometry increment
STUCK_S = 8.0             # s without odometric motion before a controller aborts
STUCK_MOVE = 0.03         # m
STUCK_TURN = 0.05         # rad

DEFAULT_CFG = {
    "margin_front": 0.02,     # m kept between footprint and lidar points along the motion
    "margin_side": 0.01,      # m lateral band beside the swept footprint
    "margin_hard": 0.005,     # m, used for points already inside the soft margin
    "decel_frac": 0.7,        # fraction of the acceleration limit used when planning stops
    "lookahead": 0.5,         # m, carrot distance along the planned path
    "rot_enter": 0.6,         # rad, heading error that triggers rotate-to-heading
    "rot_exit": 0.08,         # rad
    "grid_res": 0.05,         # m, planner grid (Nav2 costmap default)
    "replan_period": 3.0,     # s
    "B_speed_scale": 0.75,    # stroke_shimmy: fraction of v_max      (DESIGNED handicap, see README)
    "B_rot_scale": 0.6,       # stroke_shimmy: fraction of omega_max  (DESIGNED handicap, see README)
    "B_min_gain": 0.0015,     # m, smallest progress a stroke (or a pair of strokes) must promise
    "B_direct_gain": 0.02,    # m, a single stroke gaining this much is taken without lookahead
    "B_rot_cost": 0.02,       # m per rad, stroke score penalty
    "B_rev_cost": 0.002,      # m, stroke score penalty for reversing
    "B_stroke_max": 1.0,      # m
    "B_pursuit_angle": 0.6,   # rad, below this heading error B follows the path like A does
    "B_backup": 0.4,          # m, straight back-up when no stroke plan helps
    "holo_heading_gain": 1.5,
}

ARMS = {"differential": ("rotate_then_follow", "stroke_shimmy"),
        "mecanum": ("holonomic", "rotate_then_follow")}

# Controllers defined outside this file (learner/latticenav.py registers the independent pair here).
# name -> factory(kind, robot, start, goal, cfg) returning an object with the Controller interface
# (on_scan, command, aborted, n_plans, n_strokes, n_reversals, n_backups). The built-in stand-ins
# are untouched by this hook, so SURROGATE_VERSION is unchanged.
EXTERNAL_CONTROLLERS = {}

RUNNING, SUCCESS, COLLISION, TIMEOUT, STUCK = 0, 1, 2, 3, 4
STATUS_NAMES = {RUNNING: "running", SUCCESS: "success", COLLISION: "collision", TIMEOUT: "timeout", STUCK: "stuck"}


def score(status, t):
    """J for one episode."""
    if status == SUCCESS:
        return min(float(t), T_CAP)
    if status == COLLISION:
        return T_CAP + PEN_COLLISION
    return T_CAP + PEN_TIMEOUT


# ---- counter-based noise ------------------------------------------------------------------

def stream_normals(seed, stream, counter, n):
    """n standard normals that depend only on (seed, stream, counter): Philox is counter-based,
    so two arms with the same seed read identical noise at the same scan/tick index."""
    bg = np.random.Philox(key=[int(seed) & 0xFFFFFFFFFFFFFFFF, int(stream)], counter=[int(counter), 0, 0, 0])
    return np.random.Generator(bg).standard_normal(n)


# ---- lidar-based free-motion queries (body frame: u ahead, n left) -------------------------

def free_straight(u, n, half_len, half_wid, m_front, m_side):
    """Distance the footprint can advance along +u before a point comes within m_front of its
    leading face. Only points ahead (u > 0) inside the lateral band count. (..., P) -> (...)."""
    rel = (np.abs(n) < half_wid + m_side) & (u > 0)
    gap = np.where(rel, np.maximum(u - (half_len + m_front), 0.0), np.inf)
    return gap.min(axis=-1) if gap.shape[-1] else np.full(gap.shape[:-1], np.inf)


def free_rotation(u, n, half_len, half_wid, margin, margin_hard, sign):
    """Largest in-place rotation (rad, same sense as ``sign``) before any point enters the
    footprint inflated by ``margin``. Points already inside the inflated footprint are tested
    against ``margin_hard`` instead, so sensor noise cannot freeze the robot. Exact (analytic).
    """
    if u.size == 0:
        return math.pi
    if sign < 0:
        n = -n
    r = np.hypot(u, n)
    psi = np.arctan2(n, u)
    soft = (np.abs(u) < half_len + margin) & (np.abs(n) < half_wid + margin)
    a = np.where(soft, half_len + margin_hard, half_len + margin)
    b = np.where(soft, half_wid + margin_hard, half_wid + margin)
    if np.any((np.abs(u) < a) & (np.abs(n) < b)):
        return 0.0
    r = np.maximum(r, 1e-9)
    A = np.arccos(np.minimum(1.0, a / r))
    Bq = np.arcsin(np.minimum(1.0, b / r))
    ok = A < Bq                                   # this radius crosses the rectangle at all
    two_pi = 2 * math.pi
    best = np.full(r.shape, math.pi)
    for hi in (Bq, math.pi - A, -math.pi + Bq, -A):          # upper ends of the four inside arcs
        best = np.minimum(best, np.where(ok, np.mod(psi - hi, two_pi), math.pi))
    return float(best.min())


# ---- planner on the observed occupancy -----------------------------------------------------

def _disc(radius_cells):
    k = int(math.ceil(radius_cells))
    yy, xx = np.mgrid[-k:k + 1, -k:k + 1]
    return (xx * xx + yy * yy) <= radius_cells * radius_cells + 1e-9


class NavGrid:
    """Occupancy from lidar hits (unknown = free) with incremental inflation and an A* planner."""

    def __init__(self, xlim, ylim, res, r_lethal):
        self.res, self.x0, self.y0 = res, xlim[0], ylim[0]
        self.nx = int(math.ceil((xlim[1] - xlim[0]) / res))
        self.ny = int(math.ceil((ylim[1] - ylim[0]) / res))
        self.occ = np.zeros((self.nx, self.ny), bool)
        self.radii = [r_lethal, r_lethal - res, r_lethal + 0.10, r_lethal + 0.20]   # lethal, fallback, soft rings
        self.discs = [_disc(max(r, 0.0) / res) for r in self.radii]
        self.masks = [np.zeros((self.nx, self.ny), bool) for _ in self.radii]
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
        cells = np.unique(np.stack([i[new], j[new]], axis=1), axis=0)
        self.occ[cells[:, 0], cells[:, 1]] = True
        for disc, mask in zip(self.discs, self.masks):
            k = disc.shape[0] // 2
            for ci, cj in cells:
                i0, i1, j0, j1 = max(ci - k, 0), min(ci + k + 1, self.nx), max(cj - k, 0), min(cj + k + 1, self.ny)
                mask[i0:i1, j0:j1] |= disc[i0 - ci + k:i1 - ci + k, j0 - cj + k:j1 - cj + k]
        self.version += 1
        return True

    def _nearest_free(self, c, lethal, reach=8):
        i, j = min(max(c[0], 0), self.nx - 1), min(max(c[1], 0), self.ny - 1)
        if not lethal[i, j]:
            return i, j
        best, bd = None, 1e9
        for di in range(-reach, reach + 1):
            for dj in range(-reach, reach + 1):
                a, b = i + di, j + dj
                if 0 <= a < self.nx and 0 <= b < self.ny and not lethal[a, b] and di * di + dj * dj < bd:
                    best, bd = (a, b), di * di + dj * dj
        return best

    def plan(self, start_xy, goal_xy):
        """A* (8-connected, octile) from start to goal. Returns (path Kx2 in metres, used_fallback) or (None, _)."""
        for level in (0, 1):
            lethal = self.masks[level]
            s = self._nearest_free(self.cell(*start_xy), lethal)
            g = self._nearest_free(self.cell(*goal_xy), lethal)
            if s is None or g is None:
                continue
            cost = (1.0 + 3.0 * self.masks[2] + 1.0 * self.masks[3]).tolist()
            leth = lethal.tolist()
            nx, ny = self.nx, self.ny
            gi, gj = g
            best = {s: 0.0}
            parent = {}
            heap = [(0.0, 0.0, s)]
            closed = set()
            found = False
            while heap:
                _, gc, cur = heapq.heappop(heap)
                if cur in closed:
                    continue
                closed.add(cur)
                if cur == g:
                    found = True
                    break
                ci, cj = cur
                for di, dj, w in ((1, 0, 1.0), (-1, 0, 1.0), (0, 1, 1.0), (0, -1, 1.0),
                                  (1, 1, 1.41421356), (1, -1, 1.41421356), (-1, 1, 1.41421356), (-1, -1, 1.41421356)):
                    a, b = ci + di, cj + dj
                    if a < 0 or b < 0 or a >= nx or b >= ny or leth[a][b]:
                        continue
                    if di and dj and (leth[ci + di][cj] or leth[ci][cj + dj]):
                        continue                               # no corner cutting
                    ng = gc + w * cost[a][b]
                    nb = (a, b)
                    if ng < best.get(nb, 1e18):
                        best[nb] = ng
                        parent[nb] = cur
                        dx, dy = abs(a - gi), abs(b - gj)
                        heapq.heappush(heap, (ng + (dx + dy) - 0.58578644 * min(dx, dy), ng, nb))
            if not found:
                continue
            cells = [g]
            while cells[-1] != s:
                cells.append(parent[cells[-1]])
            cells.reverse()
            pts = np.array([self.centre(i, j) for i, j in cells], float)
            pts[-1] = goal_xy if not lethal[self.cell(*goal_xy)] else pts[-1]
            return pts, bool(level)
        return None, False

    def path_blocked(self, pts, level):
        i = np.clip(np.floor((pts[:, 0] - self.x0) / self.res).astype(int), 0, self.nx - 1)
        j = np.clip(np.floor((pts[:, 1] - self.y0) / self.res).astype(int), 0, self.ny - 1)
        return bool(self.masks[level][i, j][1:-1].any())


# ---- controller stand-ins ------------------------------------------------------------------

class Controller:
    """Sees lidar ranges, odometry pose and odometry twist. Never sees truth."""

    def __init__(self, kind, robot, start, goal, cfg):
        if abs(robot.axle_offset) > 1e-9:
            raise NotImplementedError("surrogate controllers assume the rotation centre at the footprint centre")
        self.kind, self.robot, self.cfg, self.goal = kind, robot, cfg, np.array(goal, float)
        self.hl, self.hw = robot.L / 2, robot.W / 2
        xs, ys = (start[0], goal[0]), (start[1], goal[1])
        self.grid = NavGrid((min(xs) - 1.0, max(xs) + 1.2), (min(ys) - 2.3, max(ys) + 2.3), cfg["grid_res"],
                            r_lethal=robot.W / 2)
        self.pts = np.zeros((0, 2))
        self.path, self.path_level, self.path_idx, self.t_plan = None, 0, 0, -1e9
        self.mode = "drive"
        self.prims = []
        self.ref_heading = start[2]
        self.anchor, self.t_anchor = np.array(start, float), 0.0
        self.aborted = False
        self.n_plans = 0
        self.n_strokes = 0
        self.n_reversals = 0
        self.n_backups = 0
        self._last_dir = 0
        slow = kind == "stroke_shimmy"
        self.v_lim = robot.v_max * (cfg["B_speed_scale"] if slow else 1.0)
        self.w_lim = robot.omega_max * (cfg["B_rot_scale"] if slow else 1.0)

    # -- observations
    def on_scan(self, ranges, odom, t):
        K = ranges.size
        ang = odom[2] + np.arange(K) * (2 * math.pi / K)
        ok = ranges < self.robot.lidar["max_range"] - 0.05
        self.pts = np.stack([odom[0] + ranges[ok] * np.cos(ang[ok]), odom[1] + ranges[ok] * np.sin(ang[ok])], axis=1)
        self.grid.add_points(self.pts[:, 0], self.pts[:, 1])
        need = self.path is None or t - self.t_plan > self.cfg["replan_period"]
        if not need and t - self.t_plan > 0.5:
            need = self.grid.path_blocked(self.path, self.path_level) or self._path_dist(odom) > 0.35
        if need:
            self.path, self.path_level = self.grid.plan((odom[0], odom[1]), tuple(self.goal))
            self.path_idx, self.t_plan = 0, t
            self.n_plans += 1

    def _path_dist(self, odom):
        d = np.hypot(self.path[:, 0] - odom[0], self.path[:, 1] - odom[1])
        return float(d.min())

    def _carrot(self, odom):
        if self.path is None:
            return None
        p = self.path
        d = np.hypot(p[:, 0] - odom[0], p[:, 1] - odom[1])
        lo = self.path_idx
        k = lo + int(np.argmin(d[lo:lo + 40]))
        self.path_idx = k
        far = np.flatnonzero(d[k:] >= self.cfg["lookahead"])
        return p[k + far[0]] if far.size else p[-1]

    def _body_points(self, odom):
        c, s = math.cos(odom[2]), math.sin(odom[2])
        rx, ry = self.pts[:, 0] - odom[0], self.pts[:, 1] - odom[1]
        return rx * c + ry * s, -rx * s + ry * c

    def _free(self, u, n, direction):
        c = self.cfg
        if direction == "fwd":
            return float(free_straight(u, n, self.hl, self.hw, c["margin_front"], c["margin_side"]))
        if direction == "rev":
            return float(free_straight(-u, n, self.hl, self.hw, c["margin_front"], c["margin_side"]))
        if direction == "left":
            return float(free_straight(n, u, self.hw, self.hl, c["margin_front"], c["margin_side"]))
        return float(free_straight(-n, u, self.hw, self.hl, c["margin_front"], c["margin_side"]))

    def _free_rot(self, u, n, sign):
        c = self.cfg
        return free_rotation(u, n, self.hl, self.hw, c["margin_front"], c["margin_hard"], sign)

    def _speed(self, limit, room, acc):
        """Speed that can still stop within ``room`` (and creeps the last millimetres)."""
        if room <= 0:
            return 0.0
        return min(limit, math.sqrt(2 * self.cfg["decel_frac"] * acc * room), 0.6 * room / DT)

    # -- stuck bookkeeping (odometry only)
    def _progress(self, odom, t):
        if (math.hypot(odom[0] - self.anchor[0], odom[1] - self.anchor[1]) > STUCK_MOVE
                or abs(G.wrap(odom[2] - self.anchor[2])) > STUCK_TURN):
            self.anchor, self.t_anchor = np.array(odom, float), t
        if t - self.t_anchor > STUCK_S:
            self.aborted = True

    # -- main entry
    def command(self, odom, twist, t):
        self._progress(odom, t)
        carrot = self._carrot(odom)
        if carrot is None or self.pts.shape[0] == 0:
            return 0.0, 0.0, 0.0
        u, n = self._body_points(odom)
        dx, dy = carrot[0] - odom[0], carrot[1] - odom[1]
        dist_c = math.hypot(dx, dy)
        beta = float(G.wrap(math.atan2(dy, dx) - odom[2]))
        if self.kind == "holonomic":
            return self._holonomic(odom, u, n, dx, dy, dist_c)
        if self.kind == "rotate_then_follow":
            return self._rotate_then_follow(odom, u, n, beta, dist_c)
        return self._stroke_shimmy(odom, twist, u, n, beta, dist_c)

    def _pursuit(self, odom, u, n, beta, dist_c, fwd):
        r = self.robot
        kappa = 2 * math.sin(beta) / max(dist_c, 0.1)
        v = min(self._speed(self.v_lim, fwd, r.acc), self.w_lim / max(abs(kappa), 1e-6))
        om = v * kappa
        if om != 0.0 and self._free_rot(u, n, 1 if om > 0 else -1) < abs(om) * 0.3:
            om = 0.0
        return v, 0.0, om

    def _rotate_cmd(self, u, n, err):
        sign = 1 if err > 0 else -1
        room = min(abs(err), self._free_rot(u, n, sign))
        return sign * self._speed(self.w_lim, room if room > 0.004 else 0.0, self.robot.ang_acc)

    def _rotate_then_follow(self, odom, u, n, beta, dist_c):
        c = self.cfg
        fwd = self._free(u, n, "fwd")
        if self.mode == "rotate" and abs(beta) < c["rot_exit"]:
            self.mode = "drive"
        elif self.mode == "drive" and (abs(beta) > c["rot_enter"] or (fwd < 0.02 and abs(beta) > c["rot_exit"])):
            self.mode = "rotate"
        if self.mode == "rotate":
            return 0.0, 0.0, self._rotate_cmd(u, n, beta)
        return self._pursuit(odom, u, n, beta, dist_c, fwd)

    def _holonomic(self, odom, u, n, dx, dy, dist_c):
        r, c = self.robot, self.cfg
        cth, sth = math.cos(odom[2]), math.sin(odom[2])
        bu, bn = (dx * cth + dy * sth) / max(dist_c, 1e-6), (-dx * sth + dy * cth) / max(dist_c, 1e-6)
        scale = min(self.v_lim / max(abs(bu), 1e-6), r.vy_max / max(abs(bn), 1e-6))
        vx, vy = scale * bu, scale * bn
        vx = math.copysign(min(abs(vx), self._speed(self.v_lim, self._free(u, n, "fwd" if vx > 0 else "rev"), r.acc)), vx)
        vy = math.copysign(min(abs(vy), self._speed(r.vy_max, self._free(u, n, "left" if vy > 0 else "right"), r.acc)), vy)
        err = float(G.wrap(self.ref_heading - odom[2]))
        om = max(-self.w_lim, min(self.w_lim, c["holo_heading_gain"] * err))
        if om != 0.0 and self._free_rot(u, n, 1 if om > 0 else -1) < abs(om) * 0.3:
            om = 0.0
        return vx, vy, om

    # -- reverse-capable stroke controller
    def _strokes(self, u, n, cu, cn, both_lengths):
        """Candidate strokes from one state: rotate by delta in place, then drive straight.

        (u, n) lidar points and (cu, cn) the carrot, all in the body frame. Returns arrays
        delta, direction, length, gain (reduction of the distance to the carrot), cost, and the
        rotated points/carrot per heading candidate for the caller's lookahead.
        """
        c = self.cfg
        fp, fn = min(self._free_rot(u, n, +1), math.pi), min(self._free_rot(u, n, -1), math.pi)
        beta, dist = math.atan2(cn, cu), math.hypot(cu, cn)
        fr = (0.25, 0.5, 0.75, 1.0)
        deltas = [0.0] + [f * fp for f in fr] + [-f * fn for f in fr]
        if -fn < beta < fp:
            deltas.append(beta)
        dl = np.array(deltas)
        cd, sd = np.cos(dl)[:, None], np.sin(dl)[:, None]
        uu, nn = u[None, :] * cd + n[None, :] * sd, -u[None, :] * sd + n[None, :] * cd
        ccu, ccn = dist * np.cos(beta - dl), dist * np.sin(beta - dl)
        out = []
        for direction in (1, -1):
            free = free_straight(direction * uu, nn, self.hl, self.hw, c["margin_front"], c["margin_side"])
            full = np.minimum(free, c["B_stroke_max"])
            capped = np.minimum(full, np.maximum(direction * ccu, 0.0))
            for length in ((capped, full) if both_lengths else (capped,)):
                gain = dist - np.hypot(ccu - direction * length, ccn)
                cost = c["B_rot_cost"] * np.abs(dl) + (c["B_rev_cost"] if direction < 0 else 0.0)
                out.append((dl, np.full(dl.size, direction), length, gain, cost, np.arange(dl.size)))
        d_, dir_, s_, g_, c_, k_ = (np.concatenate(z) for z in zip(*out))
        return d_, dir_, s_, g_, c_, k_, uu, nn, ccu, ccn

    def _choose_stroke(self, odom, u, n, beta, dist_c):
        """Greedy stroke choice with a two-stroke lookahead when no single stroke helps much.

        Returns (score, delta, direction, length) or None. Uses lidar points and the carrot only.
        """
        c = self.cfg
        near = (u * u + n * n) < (c["B_stroke_max"] + 0.8) ** 2
        u, n = u[near], n[near]
        cu, cn = dist_c * math.cos(beta), dist_c * math.sin(beta)
        d1, dir1, s1, g1, c1, k1, uu, nn, ccu, ccn = self._strokes(u, n, cu, cn, both_lengths=True)
        sc = np.where(g1 > c["B_min_gain"], g1 - c1, -np.inf)
        k = int(np.argmax(sc))
        if g1[k] >= c["B_direct_gain"]:
            return float(sc[k]), float(d1[k]), int(dir1[k]), float(s1[k])
        best = (float(sc[k]), float(d1[k]), int(dir1[k]), float(s1[k])) if np.isfinite(sc[k]) else None
        seen = set()
        for j in range(d1.size):
            if s1[j] < 0.005:
                continue
            sig = (int(k1[j]), int(dir1[j]), round(float(s1[j]), 4))
            if sig in seen:
                continue
            seen.add(sig)
            h = k1[j]
            u2, n2 = uu[h] - dir1[j] * s1[j], nn[h]
            _, _, _, g2, c2, _, _, _, _, _ = self._strokes(u2, n2, ccu[h] - dir1[j] * s1[j], ccn[h], both_lengths=False)
            tot_gain = g1[j] + g2
            tot = np.where(tot_gain > c["B_min_gain"], tot_gain - c1[j] - c2, -np.inf)
            m = int(np.argmax(tot))
            if np.isfinite(tot[m]) and (best is None or tot[m] > best[0]):
                best = (float(tot[m]), float(d1[j]), int(dir1[j]), float(s1[j]))
        return best

    def _stroke_shimmy(self, odom, twist, u, n, beta, dist_c):
        r, c = self.robot, self.cfg
        moving = abs(twist[0]) > 0.02 or abs(twist[2]) > 0.03
        while self.prims:
            p = self.prims[0]
            if p["kind"] == "rot":
                err = float(G.wrap(p["target"] - odom[2]))
                om = self._rotate_cmd(u, n, err) if abs(err) > 0.004 else 0.0
                if om == 0.0:
                    self.prims.pop(0)
                    continue
                if abs(twist[0]) > 0.02:
                    return 0.0, 0.0, 0.0                       # finish stopping first
                return 0.0, 0.0, om
            done = (odom[0] - p["x0"]) * math.cos(p["th"]) + (odom[1] - p["y0"]) * math.sin(p["th"])
            rem = p["s"] - p["dir"] * done
            room = min(rem, self._free(u, n, "fwd" if p["dir"] > 0 else "rev"))
            if rem < 0.003 or room < 0.002:
                self.prims.pop(0)
                continue
            if abs(twist[2]) > 0.03:
                return 0.0, 0.0, 0.0
            return p["dir"] * self._speed(self.v_lim, room, r.acc), 0.0, 0.0
        fwd = self._free(u, n, "fwd")
        if abs(beta) < c["B_pursuit_angle"] and fwd > 0.10:
            self._last_dir = 1
            return self._pursuit(odom, u, n, beta, dist_c, fwd)
        if moving:
            return 0.0, 0.0, 0.0
        best = self._choose_stroke(odom, u, n, beta, dist_c)
        if best is None:
            # No one- or two-stroke plan gets closer: back straight up (cf. a back-up recovery).
            back = min(self._free(u, n, "rev"), c["B_backup"])
            if back < 0.02:
                return 0.0, 0.0, 0.0
            best = (0.0, 0.0, -1, back)
            self.n_backups += 1
        _, delta, direction, s = best
        th = float(G.wrap(odom[2] + delta))
        if abs(delta) > 0.004:
            self.prims.append({"kind": "rot", "target": th})
        self.prims.append({"kind": "lin", "dir": direction, "s": s, "x0": odom[0], "y0": odom[1], "th": th})
        self.n_strokes += 1
        if self._last_dir and direction != self._last_dir:
            self.n_reversals += 1
        self._last_dir = direction
        return 0.0, 0.0, 0.0


# ---- batched simulator ---------------------------------------------------------------------

class BatchSim:
    """N environments on one course. Kinematics, collision and lidar are batched numpy."""

    def __init__(self, course, robot, seeds):
        self.course, self.robot = course, robot
        self.boxes = course.boxes()
        self.seeds = np.asarray(seeds, np.int64)
        N = self.seeds.size
        self.pose = np.tile(np.array(course.start, float), (N, 1))
        self.odom = self.pose.copy()
        self.vel = np.zeros((N, 3))                       # body vx, body vy, yaw rate
        self.status = np.full(N, RUNNING)
        self.t_end = np.zeros(N)
        self.tick = 0
        self.path_len = np.zeros(N)
        self.min_clear_note = None
        lid = robot.lidar
        self.K, self.max_range, self.sigma = int(lid["n_beams"]), float(lid["max_range"]), float(lid["noise_sigma"])
        self.beam = np.arange(self.K) * (2 * math.pi / self.K)
        self.lim = np.array([robot.v_max, robot.vy_max, robot.omega_max])
        self.acc = np.array([robot.acc, robot.acc, robot.ang_acc])
        if G.rect_hits_boxes(self.pose[:1, 0], self.pose[:1, 1], self.pose[:1, 2], robot.L, robot.W, self.boxes).any():
            self.status[:] = COLLISION                     # start pose does not fit

    @property
    def running(self):
        return self.status == RUNNING

    def scan(self):
        """Noisy ranges (N, K). Noise depends on (seed, scan index) only."""
        true = G.raycast(self.pose[:, :2], self.pose[:, 2:3] + self.beam[None, :], self.boxes, self.max_range)
        idx = self.tick // SCAN_EVERY
        noise = np.stack([stream_normals(s, 1, idx, self.K) for s in self.seeds])
        return np.where(true < self.max_range, np.maximum(true + self.sigma * noise, 0.0), self.max_range)

    def step(self, cmd):
        run = self.running
        cmd = np.clip(np.asarray(cmd, float), -self.lim, self.lim)
        dv = np.clip(cmd - self.vel, -self.acc * DT, self.acc * DT)
        self.vel = np.where(run[:, None], self.vel + dv, 0.0)
        th_mid = self.pose[:, 2] + 0.5 * self.vel[:, 2] * DT
        c, s = np.cos(th_mid), np.sin(th_mid)
        dxy = np.stack([(self.vel[:, 0] * c - self.vel[:, 1] * s) * DT, (self.vel[:, 0] * s + self.vel[:, 1] * c) * DT], 1)
        self.pose[:, :2] += dxy
        self.pose[:, 2] = G.wrap(self.pose[:, 2] + self.vel[:, 2] * DT)
        self.path_len += np.hypot(dxy[:, 0], dxy[:, 1])
        nz = np.stack([stream_normals(sd, 2, self.tick, 3) for sd in self.seeds])
        inc = self.vel * DT * (1.0 + ODOM_SIGMA * nz)
        om_mid = self.odom[:, 2] + 0.5 * inc[:, 2]
        co, so = np.cos(om_mid), np.sin(om_mid)
        self.odom[:, 0] += inc[:, 0] * co - inc[:, 1] * so
        self.odom[:, 1] += inc[:, 0] * so + inc[:, 1] * co
        self.odom[:, 2] = G.wrap(self.odom[:, 2] + inc[:, 2])
        self.tick += 1
        t = self.tick * DT
        hit = G.rect_hits_boxes(self.pose[:, 0], self.pose[:, 1], self.pose[:, 2], self.robot.L, self.robot.W, self.boxes)
        gx, gy = self.course.goal
        at_goal = np.hypot(self.pose[:, 0] - gx, self.pose[:, 1] - gy) < C.GOAL_TOLERANCE
        for mask, code in ((run & hit, COLLISION), (run & ~hit & at_goal, SUCCESS)):
            self.status[mask], self.t_end[mask] = code, t
        late = self.running & (t >= T_CAP - 1e-9)
        self.status[late], self.t_end[late] = TIMEOUT, t

    def abort(self, k):
        if self.status[k] == RUNNING:
            self.status[k], self.t_end[k] = STUCK, self.tick * DT


def run_batch(course, robot, arms, seeds, cfg=None, trace=False):
    """Run every (arm, seed) pair on one course. Returns a list of per-episode dicts
    (arm, seed, status, time, J, path_len, n_plans, n_strokes, n_reversals[, trace])."""
    cfg = dict(DEFAULT_CFG, **(cfg or {}))
    pairs = [(a, int(s)) for a in arms for s in seeds]
    sim = BatchSim(course, robot, [s for _, s in pairs])
    ctls = [EXTERNAL_CONTROLLERS.get(a, Controller)(a, robot, course.start, course.goal, cfg) for a, _ in pairs]
    traces = [[] for _ in pairs] if trace else None
    cmd = np.zeros((len(pairs), 3))
    while sim.running.any():
        t = sim.tick * DT
        run = sim.running
        if sim.tick % SCAN_EVERY == 0:
            ranges = sim.scan()
            for k in np.flatnonzero(run):
                ctls[k].on_scan(ranges[k], sim.odom[k], t)
        for k in np.flatnonzero(run):
            cmd[k] = ctls[k].command(sim.odom[k], sim.vel[k], t)
            if ctls[k].aborted:
                sim.abort(k)
            if trace:
                traces[k].append((t, *sim.pose[k]))
        sim.step(cmd)
    out = []
    for k, (a, s) in enumerate(pairs):
        st = int(sim.status[k])
        rec = {"arm": a, "seed": s, "status": STATUS_NAMES[st], "time": round(float(sim.t_end[k]), 3),
               "J": round(score(st, sim.t_end[k]), 3), "path_len": round(float(sim.path_len[k]), 4),
               "n_plans": ctls[k].n_plans, "n_strokes": ctls[k].n_strokes, "n_reversals": ctls[k].n_reversals, "n_backups": ctls[k].n_backups}
        if trace:
            rec["trace"] = np.array(traces[k])
        out.append(rec)
    return out


def paired_gap(course, robot, seeds, cfg=None, arms=None):
    """J_A and J_B per seed with common random numbers (same lidar and odometry noise streams).

    Returns dict(arm_A, arm_B, seeds, J_A, J_B, gap = J_A - J_B, status_A, status_B, ...).
    gap > 0 means B (the second arm) is better.
    """
    arm_a, arm_b = arms or ARMS[robot.drive]          # arms: optional explicit pair (see EXTERNAL_CONTROLLERS)
    res = run_batch(course, robot, (arm_a, arm_b), seeds, cfg)
    S = len(seeds)
    a, b = res[:S], res[S:]
    ja, jb = [r["J"] for r in a], [r["J"] for r in b]
    return {"label": LABEL, "arm_A": arm_a, "arm_B": arm_b, "seeds": [int(s) for s in seeds],
            "J_A": ja, "J_B": jb, "gap": [round(x - y, 3) for x, y in zip(ja, jb)],
            "status_A": [r["status"] for r in a], "status_B": [r["status"] for r in b],
            "strokes_B": [r["n_strokes"] for r in b], "reversals_B": [r["n_reversals"] for r in b],
            "backups_B": [r["n_backups"] for r in b],
            "path_len_A": [r["path_len"] for r in a], "path_len_B": [r["path_len"] for r in b]}


# ---- dense grid CLI --------------------------------------------------------------------------

CACHE_DIR = os.path.expanduser("~/.cache/expchoice")
SMALL_GRID = {"eps_over_W": [0.06, 0.12, 0.18, 0.25, 0.32, 0.38, 0.44, 0.50, 0.56, 0.62, 0.70, 0.85, 1.0],
              "e_over_W": [0.1, 0.25, 0.5, 1.0, 1.5, 2.0]}


def grid_path(robot_name, tag="small_v2"):
    return os.path.join(CACHE_DIR, "grid_%s_%s.jsonl" % (robot_name, tag))


def _key(course, robot_name, seeds, cfg):
    return "v%d|%s|%s|%s|%s" % (SURROGATE_VERSION, course.course_id, robot_name, ",".join(str(s) for s in seeds),
                            json.dumps(cfg or {}, sort_keys=True))


def run_grid(robot_name, eps_list, e_list, seeds, out_path, w_over_W=1.3, t=0.20, B=5.0, cfg=None,
             budget_s=None, log=print):
    """Evaluate paired_gap on the (eps/W, e/W) grid; append one JSON line per course; skip done ones."""
    robot = R.load_robot(robot_name)
    os.makedirs(os.path.dirname(out_path), exist_ok=True)
    done = set()
    if os.path.exists(out_path):
        with open(out_path, "r", encoding="utf-8") as f:
            for line in f:
                try:
                    done.add(json.loads(line)["key"])
                except (ValueError, KeyError):
                    continue                                   # torn last line from an interrupted run
    t0 = time.time()
    n_new = 0
    for eps_w in eps_list:
        for e_w in e_list:
            course = C.course_from_ratios(robot, eps_w, e_w, w_over_W=w_over_W, t=t, B=B)
            key = _key(course, robot_name, seeds, cfg)
            if key in done:
                continue
            if budget_s is not None and time.time() - t0 > budget_s:
                log("budget reached; rerun to resume")
                return n_new
            t1 = time.time()
            feat = M.features(course, robot)
            gap = paired_gap(course, robot, seeds, cfg)
            rec = {"key": key, "label": LABEL, "course_id": course.course_id, "robot": robot_name,
                   "requested": {"eps_over_W": eps_w, "e_over_W": e_w, "w_over_W": w_over_W},
                   "parameters": course.params(), "features": feat, "result": gap,
                   "constants": {"DT": DT, "T_CAP": T_CAP, "PEN_COLLISION": PEN_COLLISION, "PEN_TIMEOUT": PEN_TIMEOUT},
                   "cfg_overrides": cfg or {}, "surrogate_version": SURROGATE_VERSION, "sources": R.source_hashes(), "wall_s": None}
            rec["wall_s"] = round(time.time() - t1, 2)
            with open(out_path, "a", encoding="utf-8") as f:
                f.write(json.dumps(rec, sort_keys=True) + "\n")
                f.flush()
                os.fsync(f.fileno())
            done.add(key)
            n_new += 1
            log("eps/W=%.2f e/W=%.2f  A=%s  B=%s  (%.1fs)" % (
                feat["eps_over_W"], feat["e_over_W"], gap["J_A"], gap["J_B"], rec["wall_s"]))
    return n_new


def load_grid(path):
    rows = []
    with open(path, "r", encoding="utf-8") as f:
        for line in f:
            try:
                rows.append(json.loads(line))
            except ValueError:
                continue
    return rows


def classify(rec, margin):
    """Four-way label of one course: A wins, B wins, indifferent, both fail."""
    r = rec["result"]
    ok_a = [s == "success" for s in r["status_A"]]
    ok_b = [s == "success" for s in r["status_B"]]
    if not any(ok_a) and not any(ok_b):
        return "both_fail"
    g = float(np.mean(r["gap"]))
    if abs(g) < margin:
        return "indifferent"
    return "B_wins" if g > 0 else "A_wins"


def summarize(rows, margin=None):
    """Text map and headline numbers for a grid (EXPLORATORY)."""
    if not rows:
        return "no rows"
    robot = R.load_robot(rows[0]["robot"])
    sds = [float(np.std(r["result"]["gap"], ddof=1)) for r in rows if len(r["result"]["gap"]) > 1]
    both_ok = [float(np.std(r["result"]["gap"], ddof=1)) for r in rows if len(r["result"]["gap"]) > 1
               and all(s == "success" for s in r["result"]["status_A"] + r["result"]["status_B"])]
    noise = float(np.median(both_ok)) if both_ok else float("nan")
    if margin is None:
        margin = max(1.0, 2 * noise if both_ok else 1.0)
    eps_vals = sorted({round(r["features"]["eps_over_W"], 3) for r in rows})
    e_vals = sorted({round(r["features"]["e_over_W"], 3) for r in rows})
    cell = {(round(r["features"]["eps_over_W"], 3), round(r["features"]["e_over_W"], 3)): r for r in rows}
    sym = {"A_wins": "A", "B_wins": "B", "indifferent": "=", "both_fail": "x"}
    two_rho_eps = (2 * robot.rho - robot.L) / robot.W
    lines = ["EXPLORATORY SURROGATE grid, robot %s, %d courses, seeds per course %d" % (
        rows[0]["robot"], len(rows), len(rows[0]["result"]["seeds"])),
        "legend: A = %s wins, B = %s wins, '=' |mean gap| < %.2f s, x = both fail; d = 2 rho at eps/W = %.3f" % (
            rows[0]["result"]["arm_A"], rows[0]["result"]["arm_B"], margin, two_rho_eps),
        "rows e/W (top = largest), columns eps/W", "        " + " ".join("%5.2f" % v for v in eps_vals)]
    for e in reversed(e_vals):
        lines.append("e/W %4.2f " % e + " ".join("    %s" % (sym[classify(cell[(x, e)], margin)] if (x, e) in cell else " ")
                                                  for x in eps_vals))
    lines.append("mean gap J_A - J_B (s):")
    for e in reversed(e_vals):
        lines.append("e/W %4.2f " % e + " ".join("%5.0f" % np.mean(cell[(x, e)]["result"]["gap"]) if (x, e) in cell else "     "
                                                  for x in eps_vals))
    lines.append("B strokes (mean) / predicted four-flow cycles:")
    for e in reversed(e_vals):
        lines.append("e/W %4.2f " % e + " ".join(
            "%3.0f/%-3s" % (np.mean(cell[(x, e)]["result"]["strokes_B"]),
                            str(cell[(x, e)]["features"]["predicted_cycles"])[:3]) if (x, e) in cell else "       "
            for x in eps_vals))
    labels = [classify(r, margin) for r in rows]
    fits = [r["features"]["certificate"]["verdict"] == "fits" for r in rows]
    n_fit = sum(fits)
    n_bf = sum(1 for lab, f in zip(labels, fits) if f and lab == "both_fail")
    stat = {}
    for arm in ("A", "B"):
        for r in rows:
            for s in r["result"]["status_" + arm]:
                stat[(arm, s)] = stat.get((arm, s), 0) + 1
    lines += [
        "labels: " + ", ".join("%s %d" % (k, labels.count(k)) for k in ("A_wins", "B_wins", "indifferent", "both_fail")),
        "episode statuses: " + ", ".join("%s:%s %d" % (a, s, c) for (a, s), c in sorted(stat.items())),
        "courses certified 'fits': %d of %d; of those, both stand-ins fail on %d (%.0f%%)" % (
            n_fit, len(rows), n_bf, 100.0 * n_bf / max(n_fit, 1)),
        "seed-to-seed sd of the gap: median %.2f s over all courses, median %.2f s where all runs succeed (max %.2f s)" % (
            float(np.median(sds)) if sds else float("nan"), noise, max(sds) if sds else float("nan")),
    ]
    return "\n".join(lines)


def _floats(text):
    return [float(v) for v in text.split(",") if v]


def main(argv=None):
    ap = argparse.ArgumentParser(description="EXPLORATORY numpy surrogate of the offset doorway")
    sub = ap.add_subparsers(dest="cmd", required=True)
    g = sub.add_parser("grid", help="dense (eps/W, e/W) grid for one robot, resumable JSONL")
    g.add_argument("--robot", default="compact_diff")
    g.add_argument("--eps", type=_floats, default=SMALL_GRID["eps_over_W"])
    g.add_argument("--e", type=_floats, default=SMALL_GRID["e_over_W"])
    g.add_argument("--seeds", type=int, default=3)
    g.add_argument("--seed0", type=int, default=1000)
    g.add_argument("--w-over-W", type=float, default=1.3)
    g.add_argument("--tag", default="small_v2")
    g.add_argument("--budget-s", type=float, default=540.0)
    s = sub.add_parser("summarize")
    s.add_argument("--robot", default="compact_diff")
    s.add_argument("--tag", default="small_v2")
    s.add_argument("--margin", type=float, default=None)
    a = ap.parse_args(argv)
    path = grid_path(a.robot, a.tag)
    if a.cmd == "grid":
        seeds = list(range(a.seed0, a.seed0 + a.seeds))
        n = run_grid(a.robot, a.eps, a.e, seeds, path, w_over_W=a.w_over_W, budget_s=a.budget_s)
        print("wrote %d new rows to %s" % (n, path))
    else:
        print(summarize(load_grid(path), a.margin))
    return 0


if __name__ == "__main__":
    sys.exit(main())
