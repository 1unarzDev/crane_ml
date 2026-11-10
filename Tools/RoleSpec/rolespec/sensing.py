# SPDX-License-Identifier: GPL-3.0-or-later
"""Per-pass sensing tables: speed-detection trade-off, correlated views, decoys, background false alerts.

Evidence label: IMPLEMENTED. Anchors (reused, not re-fitted):
  * stand visibility P(view good) vs canopy: MissionSim NavPilot sight-line table (Tools/MissionSim/cache/vis_table.json)
    times a person-level occlusion factor (MissionSim `p_view_good` convention);
  * P(detect | good view) at the SLOW speed: the calibrated detector, Tools/Vision/obs_model.py
    `ObsModel.expected_pd(rho, v_slow, r_eff)` (NOMAD-calibrated RT-DETR, beta-binomial within-pass correlation),
    falling back to MissionSim's placeholder when the Vision model is absent;
  * the FAST pass: pd_fast = pd_slow * exp(-kappa) with kappa set by the controlled trade-off strength
    (none 0, weak 0.3, strong 1.2), or 'calibrated' = the Vision model evaluated at v_fast (blur + fewer frames).
  * swath 2 r_eff(rho), r_eff = 12 - 7 rho m (MissionSim), power 330 + 2.5 v^2 W (MissionSim power_w).
Decoy / false-alert / confidence-bin numbers are PRIOR (controlled knobs, not calibrated).

Outcome alphabet per pass: 0 = nothing, 1 = low-confidence alert, 2 = high-confidence alert. Each source (person,
decoy, background) emits an outcome; the pass reports the max. Planners use exactly these tables (the model is
correctly specified per (cell, view); the only approximation is cell independence of the posterior).

Latent states per (cell, view), K = 6:
  0 present, view good, decoy    1 present, good, no decoy    2 present, bad, decoy    3 present, bad, no decoy
  4 absent, decoy                5 absent, no decoy
"""
import json
import math
import os
import sys

import numpy as np

_HERE = os.path.dirname(os.path.abspath(__file__))
_VIS_TABLE = os.path.normpath(os.path.join(_HERE, "..", "..", "MissionSim", "cache", "vis_table.json"))
_VISION = os.path.normpath(os.path.join(_HERE, "..", "..", "Vision"))
_MS = os.path.normpath(os.path.join(_HERE, "..", "..", "MissionSim"))

FAST, SLOW = 0, 1
KAPPA = {"none": 0.0, "weak": 0.3, "strong": 1.2}
H_TRUE = 0.75      # P(high | person detected on a good view)        PRIOR
GLIMPSE = 0.10     # bad view: P(glimpse) = GLIMPSE * pd               (MissionSim placeholder convention)
H_GLIMPSE = 0.20
P_DECOY = (0.40, 0.50)   # per pass (fast, slow): a person-like decoy fires  PRIOR (blur lowers it)
H_DECOY = 0.40
FA_BASE = 0.03     # background false alert per pass, x (1 + rho), x 1.5 when fast   PRIOR
H_FA = 0.15
K = 6
H1 = np.array([0, 1, 2, 3])
H0 = np.array([4, 5])

_vision_model = None
_vis_tab = None


def _vision():
    global _vision_model
    if _vision_model is None:
        try:
            if _VISION not in sys.path:
                sys.path.insert(0, _VISION)
            import obs_model as vom  # Tools/Vision/obs_model.py
            _vision_model = vom.ObsModel()
        except Exception:  # pragma: no cover
            if _MS not in sys.path:
                sys.path.insert(0, _MS)
            from missionsim import obs_model as mom
            _vision_model = mom.PlaceholderObsModel()
    return _vision_model


def stand_visibility(rho):
    global _vis_tab
    if _vis_tab is None:
        try:
            with open(_VIS_TABLE) as f:
                d = json.load(f)
            _vis_tab = (np.array(d["stems_per_ha"]), np.array(d["p_good"]))
        except Exception:  # pragma: no cover
            _vis_tab = False
    sph = 400.0 + 1000.0 * np.asarray(rho)
    if _vis_tab is False:
        return np.exp(-sph / 1e4 * 1.75)
    return np.interp(sph, *_vis_tab)


def r_eff(rho):
    return 12.0 - 7.0 * np.asarray(rho)


def power_w(v):
    return 330.0 + 2.5 * v * v


_pd_cache = {}


def pd_good(rho, v):
    """Calibrated P(detect | good view) for one pass at speed v (cached on rounded canopy)."""
    key = (round(float(rho), 2), float(v))
    if key not in _pd_cache:
        m = _vision()
        rr = key[0]
        _pd_cache[key] = float(m.expected_pd(rr, v, float(r_eff(rr))))
    return _pd_cache[key]


def state_index(present, good, decoy):
    if present:
        return (0 if decoy else 1) if good else (2 if decoy else 3)
    return 4 if decoy else 5


def _src(p, h):
    return np.stack([1.0 - p, p * (1.0 - h), p * h], -1)


class Tables:
    def __init__(self, canopy, person_occl, sc):
        rho = np.asarray(canopy, float)
        C = rho.size
        self.C = C
        self.g = np.clip(stand_visibility(rho) * (1.0 - 0.7 * person_occl), 0.02, 0.98)
        self.q = float(sc.clutter)
        pd_s = np.array([pd_good(r, sc.v_slow) for r in rho])
        if sc.trade == "calibrated":
            pd_f = np.array([pd_good(r, sc.v_fast) for r in rho])
        else:
            pd_f = pd_s * math.exp(-KAPPA[sc.trade])
        self.pd = np.stack([pd_f, pd_s], 1)                    # (C, S)
        self.speeds = np.array([sc.v_fast, sc.v_slow])
        swath = 2.0 * r_eff(rho)
        path = sc.cell_m * sc.cell_m / swath
        self.scan_t = path[:, None] / self.speeds[None, :]     # (C, S) seconds per pass
        self.scan_e = self.scan_t * power_w(self.speeds)[None, :] / 3600.0   # Wh per pass
        # per-source outcome distributions (C, S, 3)
        person_good = _src(self.pd, H_TRUE)
        person_bad = _src(GLIMPSE * self.pd, H_GLIMPSE)
        decoy = _src(np.broadcast_to(np.array(P_DECOY)[None, :], (C, 2)), H_DECOY)
        fa = sc.fa_scale * FA_BASE * (1.0 + rho)[:, None] * np.array([1.5, 1.0])[None, :]
        bg = _src(fa, H_FA)
        none = np.zeros_like(bg)
        none[..., 0] = 1.0

        def comb(*srcs):
            cdf0 = np.prod([s[..., 0] for s in srcs], 0)
            cdf1 = np.prod([s[..., 0] + s[..., 1] for s in srcs], 0)
            return np.stack([cdf0, cdf1 - cdf0, 1.0 - cdf1], -1)

        P = np.zeros((C, K, 2, 3))
        P[:, 0] = comb(person_good, decoy, bg)
        P[:, 1] = comb(person_good, bg)
        P[:, 2] = comb(person_bad, decoy, bg)
        P[:, 3] = comb(person_bad, bg)
        P[:, 4] = comb(decoy, bg)
        P[:, 5] = comb(bg)
        P = np.clip(P, 1e-12, 1.0)
        self.P = P / P.sum(-1, keepdims=True)                  # (C, K, S, O)
        self.logP = np.log(self.P)
        self.cdf = np.cumsum(self.P, -1)
        self.cdf[..., -1] = 1.0 + 1e-12
        g, q = self.g, self.q
        w = np.zeros((C, K))
        w[:, 0], w[:, 1], w[:, 2], w[:, 3] = g * q, g * (1 - q), (1 - g) * q, (1 - g) * (1 - q)
        w[:, 4], w[:, 5] = q, 1 - q
        self.logw = np.log(np.clip(w, 1e-300, 1.0))            # within-hypothesis state priors
