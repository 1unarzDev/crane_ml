# SPDX-License-Identifier: GPL-3.0-or-later
"""Swappable person-observation model (mission level, one call per scan pass or review look).

PLACEHOLDER. Evidence label: IMPLEMENTED (synthetic priors, not calibrated). The vision agent will provide a
calibrated model at Tools/Vision/obs_model.py; `load()` uses it when it exists and `prefer_vision=True`
(it must expose a class `ObsModel` with the same methods as `PlaceholderObsModel`).

Structure (shared by truth and planners, but planners only call the `expected_*` / `log_lr` methods):
  * V = 4 view classes (approach from W, S, E, N). Each person has a latent per-view visibility state drawn once per
    mission (good w.p. g(rho, person occlusion)); repeated looks from the same view share it, so a bad view stays bad
    (correlated repeat views, ForestSearch convention).
  * Per-pass detection on a good view: per-frame probability from the NavPilot RGB model
    (`navpilot.sensors.detection_p`: pixel height x blur) when importable, else a compatible closed form, then
    aggregated over at most N_EFF effectively independent frames (frames inside one pass are correlated).
  * Bad view: rare low-confidence glimpse. False alerts: background rate per scanned area plus persistent decoys
    (person-like logs) that have their own per-view latent states (correlated false alerts).
  * Every alert carries a confidence in [0, 1] and a relative-position measurement with range/speed-dependent noise.
"""
import math
import os
import sys

import numpy as np

_HERE = os.path.dirname(os.path.abspath(__file__))
_NAVPILOT = os.path.normpath(os.path.join(_HERE, "..", "..", "NavPilot"))
_VISION = os.path.normpath(os.path.join(_HERE, "..", "..", "Vision"))

try:  # reuse the NavPilot per-frame person detector (pixel height x visibility x motion blur)
    if _NAVPILOT not in sys.path:
        sys.path.insert(0, _NAVPILOT)
    from navpilot.sensors import detection_p as _np_detection_p  # noqa: E402
    FRAME_MODEL = "navpilot.sensors.detection_p"
except Exception:  # pragma: no cover - fallback keeps the module self-contained
    _np_detection_p = None
    FRAME_MODEL = "closed-form fallback"

V = 4
N_EFF = 3.0          # effectively independent frames per pass
FRAME_HZ = 10.0
PERSON_SIZE_M = 0.8  # lying / crouching person, as in NavPilot targets


def _frame_p(range_m, speed):
    if _np_detection_p is not None:
        return _np_detection_p(range_m, PERSON_SIZE_M, 1.0, speed, 0.0)
    f = 0.5 * 640 / math.tan(math.radians(33.0))
    r = max(range_m, 0.3)
    h = f * PERSON_SIZE_M / r
    blur = f * (abs(speed) / r) * 0.008
    return 0.9 / (1.0 + math.exp(-(h - 18.0) / 5.0)) * math.exp(-blur / 6.0)


def _beta_pdf(c, a, b):
    c = min(max(c, 1e-4), 1 - 1e-4)
    return math.exp((a - 1) * math.log(c) + (b - 1) * math.log(1 - c) + math.lgamma(a + b) - math.lgamma(a) - math.lgamma(b))


class PlaceholderObsModel:
    name = "placeholder-v1"
    frame_model = FRAME_MODEL
    V = V
    # confidence distributions (truth); planners use the same family for their likelihood ratio
    CONF_TRUE = (6.0, 2.0)
    CONF_GLIMPSE = (2.5, 2.5)
    CONF_DECOY = (3.0, 3.0)
    CONF_FA = (2.0, 4.0)

    def __init__(self, vis_table=None):
        # vis_table: (stems_per_ha array, P(view good) array), from NavPilot sight-line sampling (world.vis_calibration)
        self.vis_table = vis_table

    # ------------------------------------------------------------- truth-side
    def p_view_good(self, rho, person_occl):
        """P(a given view class of the person is good): stand visibility x person-level occlusion (0..1)."""
        return float(np.clip(self.stand_visibility(rho) * (1.0 - 0.7 * person_occl), 0.02, 0.98))

    def stand_visibility(self, rho):
        sph = 400.0 + 1000.0 * rho
        if self.vis_table is not None:
            xs, ys = self.vis_table
            return float(np.interp(sph, xs, ys))
        return float(math.exp(-sph / 1e4 * 0.25 * 7.0))  # Beer-Lambert at ~7 m

    def p_detect_pass(self, range_m, speed, rho, r_eff):
        """P(detect | good view) for one pass at closest-approach range and speed."""
        pf = _frame_p(range_m, speed)
        chord = 2.0 * math.sqrt(max(r_eff * r_eff - min(range_m, r_eff) ** 2, 1.0))
        n_frames = chord / max(speed, 0.3) * FRAME_HZ
        n = min(N_EFF, max(1.0, n_frames / 10.0))
        return float(1.0 - (1.0 - pf) ** n)

    def p_glimpse_bad(self, p_good_pass):
        return 0.10 * p_good_pass

    def p_false_alert(self, rho, speed, area_m2):
        return min(0.5, 0.012 * (1.0 + 2.0 * rho) * (1.0 + speed / 6.0) * area_m2 / 400.0)

    def p_decoy_detect(self, speed):
        return 0.55 * math.exp(-max(speed - 2.0, 0.0) / 6.0)

    def sample_conf(self, kind, quality, u):
        """Inverse-CDF-free draw from a Beta using a numpy Generator `u` (kind: true/glimpse/decoy/fa)."""
        a, b = {"true": self.CONF_TRUE, "glimpse": self.CONF_GLIMPSE, "decoy": self.CONF_DECOY,
                "fa": self.CONF_FA}[kind]
        if kind == "true":
            a = 2.0 + 4.0 * quality
        return float(u.beta(a, b))

    def loc_sigma(self, range_m, speed):
        """1-sigma relative-position measurement noise (m), isotropic."""
        return 0.3 + 0.08 * range_m + 0.10 * speed

    # ------------------------------------------------------------- planner-side
    def log_lr(self, c):
        """log f(conf | person) / f(conf | not person); the not-person density mixes decoys and background."""
        f1 = 0.85 * _beta_pdf(c, 2.0 + 4.0 * 0.8, 2.0) + 0.15 * _beta_pdf(c, *self.CONF_GLIMPSE)
        f0 = 0.6 * _beta_pdf(c, *self.CONF_FA) + 0.4 * _beta_pdf(c, *self.CONF_DECOY)
        return math.log(max(f1, 1e-9) / max(f0, 1e-9))

    def expected_g(self, rho):
        return self.p_view_good(rho, 1.0 / 3.0)

    def expected_pd(self, rho, speed, r_eff, mode="scan"):
        if mode == "review":
            return self.p_detect_pass(4.0, speed, rho, r_eff)
        d = 0.25 * 2.0 * r_eff  # mean lateral offset is half the half-spacing
        return self.p_detect_pass(math.hypot(d, 2.0), speed, rho, r_eff)


def load(prefer_vision=False, vis_table=None):
    """Return the calibrated Tools/Vision model when asked and present, else the placeholder."""
    path = os.path.join(_VISION, "obs_model.py")
    if prefer_vision and os.path.exists(path):
        import importlib.util
        spec = importlib.util.spec_from_file_location("crane_vision_obs_model", path)
        mod = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(mod)
        return mod.ObsModel()
    return PlaceholderObsModel(vis_table=vis_table)
