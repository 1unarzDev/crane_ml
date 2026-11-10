# SPDX-License-Identifier: GPL-3.0-or-later
"""Controlled world for role specialisation in multi-drone forest SAR (DEV model, IMPLEMENTED only).

A layout seed + a `Scenario` fix everything that is TRUTH: canopy field, lost-person prior heatmap, M in {1,2,3}
survivors drawn from the prior, per-(cell, view) person visibility latents (correlated repeat views, ForestSearch /
MissionSim convention), per-(cell, view) persistent decoys, the per-pass outcome uniforms (common random numbers:
keyed on (cell, view, k-th look), so two policies that make the same look see the same outcome), and the per-pair
Gilbert-Elliott channel states. Planners only ever receive `PublicInfo`.

Seeds: global guard Tools/DroneData/splits.py (dev 0-1999). In addition the RoleSpec HELD-OUT range 5000-5019
(PREREG_ROLES.md) is refused here unless allow_heldout=True; dev work uses 0-199 only.
"""
import math
import os
import sys
from dataclasses import dataclass, asdict, replace

import numpy as np

from . import sensing as S

_HERE = os.path.dirname(os.path.abspath(__file__))
_DRONEDATA = os.path.normpath(os.path.join(_HERE, "..", "..", "DroneData"))
if _DRONEDATA not in sys.path:
    sys.path.insert(0, _DRONEDATA)
import splits as _splits  # noqa: E402

HeldoutRefused = _splits.HeldoutRefused
ROLESPEC_HELDOUT = (5000, 5019)
STREAMS = {"canopy": 1, "prior": 2, "surv": 3, "views": 4, "decoy": 5, "obs": 6, "comms": 7, "policy": 8}


def guard(seed, allow_heldout=False):
    seed = int(seed)
    split = _splits.enforce(_splits.load(), seed, allow_heldout=allow_heldout)
    if ROLESPEC_HELDOUT[0] <= seed <= ROLESPEC_HELDOUT[1] and not allow_heldout:
        raise HeldoutRefused("seed %d is RoleSpec HELD-OUT (PREREG_ROLES.md); refusing" % seed)
    return split


def rng(seed, stream, *sub):
    return np.random.Generator(np.random.PCG64(np.random.SeedSequence(
        [int(seed) & 0xFFFFFFFF, STREAMS[stream]] + [int(x) & 0xFFFFFFFF for x in sub])))


@dataclass(frozen=True)
class Scenario:
    n_drones: int = 3
    cells_per_drone: int = 30      # area scales with N
    cell_m: float = 40.0
    occlusion: str = "low"         # low | high
    trade: str = "weak"            # speed-detection trade-off strength: none | weak | strong | calibrated
    comms: str = "ideal"           # ideal | outage
    deadline_s: float = 1200.0
    battery_wh: float = 140.0
    v_fast: float = 9.0
    v_slow: float = 3.0
    v_transit: float = 9.0
    clutter: float = 0.05          # P(persistent decoy per (cell, view))
    fa_scale: float = 1.0          # multiplies the background false-alert rate (false-positive-rate knob)
    switch_s: float = 0.0          # physical mode-switch penalty (s of hover) whenever a drone changes speed mode
                                   # between passes (altitude / camera reconfiguration); applies to EVERY arm
    tau_conf: float = 0.95         # confirmation threshold (posterior), shared by all policies
    slot_s: float = 5.0            # comms slot
    # outage comms (ignored when comms == ideal)
    base_range_frac: float = 0.55  # base radio range as a fraction of the region diagonal
    a2a_range_m: float = 220.0
    ge_mean_loss: float = 0.35
    ge_mean_burst_s: float = 60.0

    def key(self):
        k = "N%d_%s_%s_%s" % (self.n_drones, self.occlusion, self.trade, self.comms)
        d = Scenario()
        for f in ("clutter", "fa_scale", "switch_s", "deadline_s", "battery_wh"):
            if getattr(self, f) != getattr(d, f):
                k += "_%s%g" % (f, getattr(self, f))
        return k

    def as_dict(self):
        return asdict(self)


OCCL = {"low": (0.30, 0.25), "high": (0.70, 0.55)}   # (canopy mean, mean person-level occlusion)
V = 4


def side_for(sc):
    return int(math.ceil(math.sqrt(sc.cells_per_drone * sc.n_drones)))


def smooth_field(r, n, scale, mean, sd):
    z = r.normal(size=(n, n))
    k = np.fft.fftfreq(n)
    kx, ky = np.meshgrid(k, k, indexing="ij")
    f = np.real(np.fft.ifft2(np.fft.fft2(z) * np.exp(-(kx ** 2 + ky ** 2) * (scale ** 2) * 2 * math.pi ** 2)))
    f = (f - f.mean()) / (f.std() + 1e-9)
    return np.clip(mean + sd * f, 0.05, 0.95)


@dataclass
class PublicInfo:
    """Everything a planner may know before launch."""
    n: int
    side: int
    cell_m: float
    centers: np.ndarray       # (C, 2)
    canopy: np.ndarray        # (C,) known canopy map (prior map; assumed correct in this controlled model)
    prior: np.ndarray         # (C,) lost-person heatmap, sums to 1
    p0: np.ndarray            # (C,) prior P(>= 1 survivor in cell) for M ~ U{1,2,3} drawn with replacement
    base_xy: np.ndarray
    tables: "S.Tables"
    sc: Scenario


class World:
    def __init__(self, seed, sc: Scenario, allow_heldout=False, rep=0):
        """rep: replicate index; re-draws view latents, decoys, outcome uniforms and channels on the same layout
        (forest, prior, survivors), so missions cluster by layout."""
        guard(seed, allow_heldout)
        self.seed, self.sc, self.rep = int(seed), sc, int(rep)
        n = side_for(sc)
        C = n * n
        self.side, self.C = n, C
        tag = (sc.n_drones, 0 if sc.occlusion == "low" else 1)
        ii, jj = np.meshgrid(np.arange(n), np.arange(n), indexing="ij")
        centers = np.stack([(ii.ravel() + 0.5) * sc.cell_m, (jj.ravel() + 0.5) * sc.cell_m], 1)
        cm, po = OCCL[sc.occlusion]
        canopy = smooth_field(rng(seed, "canopy", *tag), n, 2.0, cm, 0.15).ravel()
        # prior: 2-3 Gaussian blobs (last-known point, trail, drainage) + 15 % uniform floor
        r = rng(seed, "prior", *tag)
        nb = int(r.integers(2, 4))
        pr = np.full(C, 0.15 / C)
        w = r.dirichlet(np.ones(nb)) * 0.85
        for b in range(nb):
            mu = r.uniform(0.15, 0.85, 2) * n
            s = r.uniform(0.10, 0.22) * n
            d2 = (ii.ravel() + 0.5 - mu[0]) ** 2 + (jj.ravel() + 0.5 - mu[1]) ** 2
            g = np.exp(-0.5 * d2 / s ** 2)
            pr += w[b] * g / g.sum()
        pr /= pr.sum()
        p0 = 1.0 - np.mean([(1.0 - pr) ** m for m in (1, 2, 3)], axis=0)
        base = np.array([-0.5 * sc.cell_m, 0.5 * n * sc.cell_m])
        tables = S.Tables(canopy, po, sc)
        self.pub = PublicInfo(sc.n_drones, n, sc.cell_m, centers, canopy, pr, p0, base, tables, sc)
        # ---------------- truth
        r = rng(seed, "surv", *tag)
        M = int(r.integers(1, 4))
        self.survivor_cells = [int(c) for c in r.choice(C, size=M, replace=True, p=pr)]
        self.present = np.zeros(C, bool)
        self.present[self.survivor_cells] = True
        rv = rng(seed, "views", *tag, rep)
        self.view_good = rv.random((C, V)) < tables.g[:, None]          # only used where present
        rd = rng(seed, "decoy", *tag, rep)
        self.decoy = rd.random((C, V)) < sc.clutter
        self.K_MAX = 24
        self.u_obs = rng(seed, "obs", *tag, rep).random((C, V, self.K_MAX))
        self.looks = np.zeros((C, V), int)
        H = int(sc.deadline_s / sc.slot_s) + 2
        self.n_nodes = sc.n_drones + 1
        rc = rng(seed, "comms", *tag, rep)
        p_bg = sc.slot_s / sc.ge_mean_burst_s
        p_gb = sc.ge_mean_loss * p_bg / (1.0 - sc.ge_mean_loss)
        self.ge_bad = np.zeros((self.n_nodes, self.n_nodes, H), bool)
        for a in range(self.n_nodes):
            for b in range(a + 1, self.n_nodes):
                st = rc.random() < sc.ge_mean_loss
                u = rc.random(H)
                for t in range(H):
                    self.ge_bad[a, b, t] = st
                    st = (u[t] >= p_bg) if st else (u[t] < p_gb)
                self.ge_bad[b, a] = self.ge_bad[a, b]
        self.base_range = sc.base_range_frac * math.hypot(n, n) * sc.cell_m

    # ------------------------------------------------------------- truth-side sensing
    def look(self, c, v, s):
        """Outcome (0 none / 1 low alert / 2 high alert) of one pass over cell c from view v at speed index s."""
        k = self.looks[c, v]
        self.looks[c, v] += 1
        u = self.u_obs[c, v, min(k, self.K_MAX - 1)] if k < self.K_MAX else rng(self.seed, "obs", c, v, k, self.rep).random()
        st = S.state_index(bool(self.present[c]), bool(self.view_good[c, v]), bool(self.decoy[c, v]))
        cdf = self.pub.tables.cdf[c, st, s]
        return int(np.searchsorted(cdf, u, side="right"))

    def link_up(self, a, b, pa, pb, t):
        if self.sc.comms == "ideal":
            return True
        ti = min(int(t // self.sc.slot_s), self.ge_bad.shape[2] - 1)
        if self.ge_bad[a, b, ti]:
            return False
        rng_m = self.base_range if (a == 0 or b == 0) else self.sc.a2a_range_m
        return float(np.hypot(*(pa - pb))) <= rng_m


def scenario_grid(Ns=(2, 3, 5, 10), occl=("low", "high"), trade=("weak", "strong"), comms=("ideal", "outage"), **kw):
    out = []
    for n in Ns:
        for o in occl:
            for tr in trade:
                for cm in comms:
                    out.append(Scenario(n_drones=n, occlusion=o, trade=tr, comms=cm, **kw))
    return out


def with_(sc, **kw):
    return replace(sc, **kw)
