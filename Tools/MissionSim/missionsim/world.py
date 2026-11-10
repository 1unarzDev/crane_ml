# SPDX-License-Identifier: GPL-3.0-or-later
"""Scenario family generator: seeded forests, canopy density, persons, decoys, blockages, RF field and forecast.

TRUTH lives in `World`. Planners receive only `PublicInfo` (prior map, lost-person prior, link FORECAST, base
position, deadline) plus what they observe during the mission. The forecast is built from the (possibly stale) prior
map with no knowledge of the true shadowing field, so it is uncertain and differs from the truth.

Splits: a layout seed fixes the forest, RF truth and person; dev / val / held-out are whole layouts, guarded by
Tools/DroneData/splits.py (dev 0-1999, val 70000-79999, HELD-OUT 90000-99999). Held-out layouts raise
HeldoutRefused unless allow_heldout=True.

Grid: NX x NY cells of CELL m, world ENU metres, base station outside the left edge.
"""
import json
import math
import os
import sys
from dataclasses import dataclass, field, asdict

import numpy as np

from . import obs_model as _obs

_HERE = os.path.dirname(os.path.abspath(__file__))
_DRONEDATA = os.path.normpath(os.path.join(_HERE, "..", "..", "DroneData"))
_NAVPILOT = os.path.normpath(os.path.join(_HERE, "..", "..", "NavPilot"))
if _DRONEDATA not in sys.path:
    sys.path.insert(0, _DRONEDATA)
import splits as _splits  # noqa: E402  (Tools/DroneData/splits.py)

HeldoutRefused = _splits.HeldoutRefused
CELL = 20.0
NX = NY = 10
NC = NX * NY
DENSITY_MEAN = {"sparse": 0.25, "medium": 0.5, "dense": 0.75}
STREAMS = {"canopy": 1, "block": 2, "prior": 3, "person": 4, "decoy": 5, "rf": 6, "mapnoise": 7, "align": 8,
           "comms": 9, "obs": 10, "drift": 11, "nearmiss": 12, "e1": 13}


def rng(seed, stream, sub=0):
    return np.random.Generator(np.random.PCG64(np.random.SeedSequence([int(seed) & 0xFFFFFFFF, STREAMS[stream], sub])))


def guard_layout(seed, allow_heldout=False):
    return _splits.enforce(_splits.load(), int(seed), allow_heldout=allow_heldout)


def cell_xy(c):
    i, j = divmod(int(c), NY)
    return np.array([(i + 0.5) * CELL, (j + 0.5) * CELL])


def xy_cell(p):
    i = int(np.clip(p[0] // CELL, 0, NX - 1))
    j = int(np.clip(p[1] // CELL, 0, NY - 1))
    return i * NY + j


CENTERS = np.array([cell_xy(c) for c in range(NC)])
NEIGH = [[] for _ in range(NC)]
for _c in range(NC):
    _i, _j = divmod(_c, NY)
    for _di in (-1, 0, 1):
        for _dj in (-1, 0, 1):
            if (_di or _dj) and 0 <= _i + _di < NX and 0 <= _j + _dj < NY:
                NEIGH[_c].append((_i + _di) * NY + _j + _dj)


def vmax(rho):
    return 7.0 - 5.0 * rho


def r_eff(rho):
    """Effective person-sighting radius under canopy (m)."""
    return 12.0 - 7.0 * rho


def scan_time(rho, v):
    s = 2.0 * r_eff(rho)
    n = math.ceil(CELL / s)
    return (n * CELL + (n - 1) * s) / max(min(v, vmax(rho)), 0.5)


def power_w(v, hover=False):
    """Electrical power of the 7-inch quad (W) [U prior]: induced + parasitic; hover/slow re-view near P_hover."""
    return 330.0 + 2.5 * v * v if not hover else 345.0


def near_miss_rate(rho, v):
    """Expected near-miss events per metre flown (proxy; not a collision model)."""
    return 2e-4 * rho * (v / 4.0) ** 2


# ------------------------------------------------------------------ visibility calibration (NavPilot stands)
def vis_calibration(cache_dir=None):
    """P(view good) vs stems/ha by sampling sight lines in NavPilot dev stands; cached JSON; fallback None."""
    cache_dir = cache_dir or os.path.normpath(os.path.join(_HERE, "..", "cache"))
    path = os.path.join(cache_dir, "vis_table.json")
    if os.path.exists(path):
        with open(path) as f:
            d = json.load(f)
        return np.array(d["stems_per_ha"]), np.array(d["p_good"])
    try:
        if _NAVPILOT not in sys.path:
            sys.path.insert(0, _NAVPILOT)
        from navpilot import forest as npf
        from navpilot.sensors import target_visibility
    except Exception:
        return None
    xs, ys = [], []
    for fam, sph in npf.STEM_DENSITY.items():
        vals = []
        for seed in (11, 12, 13):
            st = npf.generate_stand(seed, npf.family_spec(stem=fam, branch="low"), length_m=60.0, half_width_m=12.0)
            w = npf.World(st)
            r = np.random.default_rng(seed)
            for _ in range(150):
                tx, ty = r.uniform(10, 50), r.uniform(-8, 8)
                ang = r.uniform(0, 2 * math.pi)
                rr = r.uniform(3.0, 10.0)
                vals.append(target_visibility(w, tx + rr * math.cos(ang), ty + rr * math.sin(ang), tx, ty, 0.8) >= 0.6)
        xs.append(sph)
        ys.append(float(np.mean(vals)))
    os.makedirs(cache_dir, exist_ok=True)
    with open(path, "w") as f:
        json.dump({"source": "navpilot.forest stands, seeds 11-13, 150 sight lines each, range 3-10 m, vis>=0.6",
                   "stems_per_ha": xs, "p_good": ys}, f, indent=1)
    return np.array(xs), np.array(ys)


# ------------------------------------------------------------------ scenario parameters
@dataclass
class Scenario:
    density: str = "medium"
    deadline_s: float = 720.0
    err_tol_m: float = 10.0
    battery_wh: float = 75.0
    horizon_s: float = 780.0
    radio_mult: float = 1.0       # foliage attenuation multiplier (radio ablation)
    drift_mult: float = 1.0       # odometry drift multiplier (drift ablation)
    stale: float = 0.0            # 0 fresh .. 1 stale prior map (unknown blockages, landmark offsets, canopy error)
    correlated: bool = False      # dense foliage degrades radio, drift and map together
    e1: bool = False              # person placed in a true dead zone at the forecast-link boundary
    e1_forced: bool = False       # mission starts at the glimpse (drone 1 at the person's cell)
    e1_conf: float = 0.8          # forced glimpse confidence
    e1_prior: str = "high"        # prior mass on the glimpse cell: high | low
    asym_outage: tuple = ()       # ((src, dst, t0, t1), ...) forced-bad directed links (E3)
    weak_target: bool = False     # E3: person harder to see (partial evidence)
    prior_mismatch: float = 0.15  # fraction of truth draws from a flat prior instead of the published prior
    fa_mult: float = 1.0          # background false-alert rate multiplier (public: the detector is characterised)
    decoy_mult: float = 1.0       # decoy (person-like object) detection multiplier
    align_mult: float = 1.0       # map-alignment heading uncertainty multiplier

    def key(self):
        return json.dumps(asdict(self), sort_keys=True)


@dataclass
class PublicInfo:
    """Everything a planner may know before launch."""
    p0: np.ndarray
    rho_map: np.ndarray
    blocked_map: np.ndarray
    clearings_map: dict           # cell -> believed landmark xy
    forecast_up: np.ndarray       # dB margin forecast drone->base per cell
    forecast_down: np.ndarray
    forecast_sigma: float
    base_xy: np.ndarray
    start_cells: tuple
    deadline_s: float
    err_tol_m: float
    battery_wh: float
    horizon_s: float
    m0_a2a: float
    alpha_db_per_m: float
    fa_mult: float = 1.0


class World:
    """TRUTH. Never handed to a non-oracle planner."""

    def __init__(self, layout_seed, sc: Scenario, obs=None, allow_heldout=False):
        self.split = guard_layout(layout_seed, allow_heldout=allow_heldout)
        self.seed = int(layout_seed)
        self.sc = sc
        self.obs = obs or _obs.load(vis_table=vis_calibration())
        rmean = DENSITY_MEAN[sc.density]
        # canopy density: smooth bumps around the family mean
        rc = rng(layout_seed, "canopy")
        X, Y = CENTERS[:, 0], CENTERS[:, 1]
        f = np.zeros(NC)
        for _ in range(7):
            cx, cy, s, a = rc.uniform(0, NX * CELL), rc.uniform(0, NY * CELL), rc.uniform(25, 70), rc.normal(0, 0.25)
            f += a * np.exp(-((X - cx) ** 2 + (Y - cy) ** 2) / (2 * s * s))
        rho = np.clip(rmean + f - f.mean(), 0.05, 0.95)
        ncl = 3
        clear_cells = rc.choice(NC, ncl, replace=False)
        rho[clear_cells] = 0.05
        self.rho = rho
        self.base_xy = np.array([-10.0, rc.uniform(50, 150)])
        start = xy_cell(np.array([5.0, self.base_xy[1]]))
        j = start % NY
        self.start_cells = (start, min(NY - 1, j + 1) if j < NY - 1 else j - 1)
        self.start_cells = (start, start + 1 if j < NY - 1 else start - 1)
        # blockages (thickets, deadfall)
        rb = rng(layout_seed, "block")
        stale = sc.stale + (0.5 * (rmean - 0.5) if sc.correlated else 0.0)
        pblk = 0.05 + 0.10 * rho
        blk = rb.random(NC) < pblk
        for c in self.start_cells:
            blk[c] = False
        blk[clear_cells] = False
        self.blocked = self._connect(blk, self.start_cells[0], rb)
        # prior map: incomplete (missing blockages) and stale (phantom blockages, landmark offsets, canopy error)
        rm = rng(layout_seed, "mapnoise")
        u1, u2 = rm.random(NC), rm.random(NC)
        miss = 0.35 + 0.5 * min(max(stale, 0.0), 1.0)
        bmap = self.blocked & (u1 >= miss)
        phantom = (~self.blocked) & (u2 < 0.02 + 0.06 * stale)
        for c in self.start_cells:
            phantom[c] = False
        bmap = bmap | phantom
        self.blocked_map = bmap
        self.rho_map = np.clip(rho + rm.normal(0, 0.06 + 0.15 * stale, NC), 0.05, 0.95)
        sig_lm = 0.8 + 6.0 * stale
        self.clear_cells = [int(c) for c in clear_cells]
        self.landmark_err = {c: rm.normal(0, sig_lm, 2) for c in self.clear_cells}
        clearings_map = {c: CENTERS[c] + self.landmark_err[c] for c in self.clear_cells}
        # lost-person prior: hotspots (trail line, clearings) over a floor, zero on mapped blockages
        rp = rng(layout_seed, "prior")
        a0 = rp.uniform(0, math.pi)
        px0, py0 = rp.uniform(60, 140), rp.uniform(60, 140)
        dline = np.abs((X - px0) * math.sin(a0) - (Y - py0) * math.cos(a0))
        p = 0.25 + np.exp(-dline ** 2 / (2 * 25.0 ** 2))
        for c in self.clear_cells:
            p += 0.8 * np.exp(-np.sum((CENTERS - CENTERS[c]) ** 2, 1) / (2 * 30.0 ** 2))
        p[bmap] = 0.0
        p[list(self.start_cells)] *= 0.2
        self.p0 = p / p.sum()
        # RF truth: log-distance + foliage integral + correlated shadowing, uplink asymmetry
        rr = rng(layout_seed, "rf")
        self.alpha = 0.22 * sc.radio_mult * (1.0 + (0.6 * (rmean - 0.5) if sc.correlated else 0.0))
        self.m0 = 44.0
        self.m0_a2a = 38.0
        sh = np.zeros(NC)
        for _ in range(8):
            cx, cy, s, a = rr.uniform(0, NX * CELL), rr.uniform(0, NY * CELL), rr.uniform(20, 50), rr.normal(0, 5.0)
            sh += a * np.exp(-((X - cx) ** 2 + (Y - cy) ** 2) / (2 * s * s))
        self.shadow = sh
        self.asym = rr.uniform(2.0, 6.0)
        self.margin_down = self._margin_field(self.rho, self.alpha) + sh
        self.margin_up = self.margin_down - self.asym - rr.normal(0, 1.5, NC)
        # forecast: prior-map canopy, nominal alpha (planners do not know radio_mult), no shadowing
        self.forecast_down = self._margin_field(self.rho_map, 0.22)
        self.forecast_up = self.forecast_down - 4.0
        self.forecast_sigma = 6.0
        # person
        rq = rng(layout_seed, "person")
        if sc.e1 or sc.e1_forced:
            tc = self._e1_cell(rq)
        else:
            pt = (1 - sc.prior_mismatch) * self.p0 + sc.prior_mismatch * (~self.blocked) / (~self.blocked).sum()
            pt[self.blocked] = 0
            tc = int(rq.choice(NC, p=pt / pt.sum()))
        if sc.e1_forced:
            pm = self.p0.copy()
            pm[tc] = (0.06 if sc.e1_prior == "high" else 0.004)
            others = np.arange(NC) != tc
            pm[others] *= (1 - pm[tc]) / pm[others].sum()
            self.p0 = pm
        self.target_cell = tc
        self.target_xy = CENTERS[tc] + rq.uniform(-0.4 * CELL, 0.4 * CELL, 2)
        occl = rq.beta(2.0, 4.0) + (0.25 if sc.weak_target else 0.0)
        self.person_occl = float(min(occl, 0.95))
        g = self.obs.p_view_good(self.rho[tc], self.person_occl)
        self.view_good = rq.random(4) < g
        if sc.e1_forced:
            self.view_good[0] = True  # the glimpse view (approach from W) was good enough to glimpse
        # decoys: persistent person-like objects with their own per-view latent states
        rd = rng(layout_seed, "decoy")
        has = rd.random(NC) < (0.06 + 0.10 * self.rho)
        has[tc] = False
        self.decoys = {int(c): (CENTERS[c] + rd.uniform(-0.4 * CELL, 0.4 * CELL, 2), rd.random(4) < 0.35)
                       for c in np.nonzero(has)[0]}
        # alignment / drift parameters
        self.drift_rate = sc.drift_mult * (0.004 + 0.02 * self.rho) * (1.0 + (0.8 * (rmean - 0.5) if sc.correlated else 0.0))
        self.public = PublicInfo(p0=self.p0.copy(), rho_map=self.rho_map.copy(), blocked_map=self.blocked_map.copy(),
                                 clearings_map=clearings_map, forecast_up=self.forecast_up.copy(),
                                 forecast_down=self.forecast_down.copy(), forecast_sigma=self.forecast_sigma,
                                 base_xy=self.base_xy.copy(), start_cells=self.start_cells, deadline_s=sc.deadline_s,
                                 err_tol_m=sc.err_tol_m, battery_wh=sc.battery_wh, horizon_s=sc.horizon_s,
                                 m0_a2a=self.m0_a2a, alpha_db_per_m=0.22, fa_mult=sc.fa_mult * (0.5 + 0.5 * sc.decoy_mult))

    # -------------------------------------------------------------- helpers
    def _connect(self, blk, start, r):
        blk = blk.copy()
        for _ in range(200):
            seen = np.zeros(NC, bool)
            st = [start]
            seen[start] = True
            while st:
                c = st.pop()
                for n in NEIGH[c]:
                    if not seen[n] and not blk[n]:
                        seen[n] = True
                        st.append(n)
            bad = np.nonzero(~seen & ~blk)[0]
            if len(bad) == 0:
                return blk
            c = int(bad[0])
            cand = [n for n in NEIGH[c] if blk[n]]
            blk[cand[int(r.integers(len(cand)))]] = False
        return blk

    def _rho_integral(self, rho, a, b, step=5.0):
        d = float(np.hypot(*(b - a)))
        n = max(2, int(d / step))
        s = (np.arange(n) + 0.5) / n
        pts = a[None, :] + s[:, None] * (b - a)[None, :]
        inside = (pts[:, 0] >= 0) & (pts[:, 0] < NX * CELL) & (pts[:, 1] >= 0) & (pts[:, 1] < NY * CELL)
        ii = np.clip((pts[:, 0] // CELL).astype(int), 0, NX - 1)
        jj = np.clip((pts[:, 1] // CELL).astype(int), 0, NY - 1)
        return float(np.sum(rho[ii * NY + jj] * inside) * d / n)

    def _margin_field(self, rho, alpha):
        out = np.zeros(NC)
        for c in range(NC):
            d = max(float(np.hypot(*(CENTERS[c] - self.base_xy))), 10.0)
            out[c] = self.m0 - 20 * math.log10(d / 10.0) - alpha * self._rho_integral(rho, self.base_xy, CENTERS[c])
        return out

    def a2a_margin(self, pa, pb):
        d = max(float(np.hypot(*(pb - pa))), 5.0)
        return self.m0_a2a - 20 * math.log10(d / 10.0) - self.alpha * self._rho_integral(self.rho, pa, pb)

    def _e1_cell(self, rq):
        """A truly dead cell whose forecast is near the link boundary and which has a connected 8-neighbour."""
        best = []
        for c in range(NC):
            if self.blocked[c] or c in self.start_cells:
                continue
            if self.margin_up[c] < -1.0 and any(self.forecast_up[n] > 2.0 and not self.blocked[n] for n in NEIGH[c]):
                best.append(c)
        if not best:  # fall back to the deadest reachable cell
            m = np.where(self.blocked, np.inf, self.margin_up)
            return int(np.argmin(m))
        return int(best[int(rq.integers(len(best)))])

    def summary(self):
        return {"seed": self.seed, "split": self.split, "dead_frac_up": float(np.mean(self.margin_up < 0)),
                "forecast_dead_frac": float(np.mean(self.forecast_up < 0)), "target_cell": self.target_cell,
                "target_margin_up": float(self.margin_up[self.target_cell]), "n_blocked": int(self.blocked.sum()),
                "n_blocked_unmapped": int((self.blocked & ~self.blocked_map).sum()), "n_decoys": len(self.decoys)}


# ------------------------------------------------------------------ travel graph
def travel_matrix(blocked, rho, v_cap=6.0):
    """All-pairs shortest travel time (s) over the 8-connected free-cell graph (Floyd-Warshall, numpy)."""
    D = np.full((NC, NC), np.inf)
    np.fill_diagonal(D, 0.0)
    vm = np.minimum(vmax(rho), v_cap)
    for c in range(NC):
        if blocked[c]:
            continue
        for n in NEIGH[c]:
            if blocked[n]:
                continue
            d = float(np.hypot(*(CENTERS[n] - CENTERS[c])))
            D[c, n] = d / (0.5 * (vm[c] + vm[n]))
    for k in range(NC):
        D = np.minimum(D, D[:, k:k + 1] + D[k:k + 1, :])
    return np.minimum(D, 1e4)  # cells disconnected in the believed map: effectively unreachable


def next_hop(D, blocked, rho, cur, goal, v_cap=6.0):
    if cur == goal:
        return cur
    vm = np.minimum(vmax(rho), v_cap)
    best, bn = np.inf, cur
    for n in NEIGH[cur]:
        if blocked[n]:
            continue
        e = float(np.hypot(*(CENTERS[n] - CENTERS[cur]))) / (0.5 * (vm[cur] + vm[n]))
        if e + D[n, goal] < best:
            best, bn = e + D[n, goal], n
    return bn
