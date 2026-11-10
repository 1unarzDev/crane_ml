# SPDX-License-Identifier: GPL-3.0-or-later
"""Tests for Tools/Vision/obs_model.py: MissionSim API compatibility, monotonicity, correlated-look aggregation,
projection-error closed form vs Monte Carlo, and the recorded held-out calibration. Run: pytest -q Tools/Vision"""
import inspect
import json
import math
import os
import sys

import numpy as np
import pytest

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
sys.path.insert(0, os.path.join(HERE, '..', 'MissionSim'))

import obs_model as om  # noqa: E402

M = om.ObsModel()


def test_missionsim_api_compatible():
    from missionsim import obs_model as ph
    P = ph.PlaceholderObsModel
    for name, fn in inspect.getmembers(P, inspect.isfunction):
        if name.startswith('_'):
            continue
        assert hasattr(om.ObsModel, name), name
        want = list(inspect.signature(fn).parameters)
        got = list(inspect.signature(getattr(om.ObsModel, name)).parameters)
        assert got[:len(want)] == want, (name, want, got)
    for attr in ('name', 'frame_model', 'V', 'CONF_TRUE', 'CONF_GLIMPSE', 'CONF_DECOY', 'CONF_FA'):
        assert hasattr(M, attr)
    loaded = ph.load(prefer_vision=True)
    assert type(loaded).__name__ == 'ObsModel' and loaded.name == M.name


def test_missionsim_calls_return_probabilities():
    rng = np.random.default_rng(0)
    for rho in (0.0, 0.5, 1.0):
        for v in (0.5, 3.0, 7.0):
            r_eff = 12.0 - 7.0 * rho
            for f in (M.expected_g(rho), M.expected_pd(rho, v, r_eff), M.expected_pd(rho, v, r_eff, 'review'),
                      M.p_false_alert(rho, v, 400.0), M.p_decoy_detect(v), M.p_view_good(rho, 0.3),
                      M.p_glimpse_bad(0.5), M.p_detect_pass(5.0, v, rho, r_eff)):
                assert 0.0 <= f <= 1.0
            assert M.loc_sigma(8.0, v) > 0
    for kind in ('true', 'glimpse', 'decoy', 'fa'):
        c = M.sample_conf(kind, 0.7, rng)
        assert M.tau <= c <= 1.0
        assert math.isfinite(M.log_lr(c))


def test_p_detect_monotone():
    hs = [4, 8, 16, 32, 64, 128, 256]
    p = [M.p_detect(h) for h in hs]
    assert all(b >= a - 1e-9 for a, b in zip(p, p[1:]))
    assert M.p_detect(64, visible=1.0) >= M.p_detect(64, visible=0.3)
    assert M.p_detect(32, blur_px=0) >= M.p_detect(32, blur_px=24)
    assert M.p_detect(0.5) == 0.0


def test_p_any_beta_binomial():
    p = 0.3
    assert M.p_any(p, 1, 0.4) == pytest.approx(p, abs=1e-9)
    assert M.p_any(p, 5, 0.0) == pytest.approx(1 - 0.7 ** 5)
    assert M.p_any(p, 5, 0.999) == pytest.approx(p, abs=0.01)
    seq = [M.p_any(p, n, 0.3) for n in (1, 2, 4, 8, 16)]
    assert all(b > a for a, b in zip(seq, seq[1:]))
    # Monte Carlo check of the closed form
    rng = np.random.default_rng(1); rho = 0.3; s = (1 - rho) / rho
    P = rng.beta(p * s, (1 - p) * s, 200000)
    hits = rng.random((200000, 4)) < P[:, None]
    assert M.p_any(p, 4, rho) == pytest.approx(hits.any(1).mean(), abs=0.005)


def test_loc_error_matches_monte_carlo_projection():
    H, D, h_px = 30.0, 40.0, 30.0
    along, cross = M.loc_error(H, D, h_px, alt_sigma_m=0.3)
    rng = np.random.default_rng(2); n = 200000
    s_pix = M.centre_sigma_px(h_px) / om.F_PX
    el0 = math.atan2(H, D)
    el = el0 + rng.normal(0, s_pix, n) + rng.normal(0, math.radians(om.ATT_SIGMA_DEG), n)
    az = rng.normal(0, math.radians(om.YAW_SIGMA_DEG), n)
    Hn = H + rng.normal(0, 0.3, n)
    g = Hn / np.tan(el)
    x, y = g * np.cos(az), g * np.sin(az) + math.hypot(H, D) * rng.normal(0, s_pix, n)
    assert np.std(x) == pytest.approx(along, rel=0.1)
    assert np.std(y) == pytest.approx(cross, rel=0.1)


def test_blur_and_geometry():
    assert om.blur_px(10.0, om.F_PX * 0.01, 0.01) == pytest.approx(10.0, rel=1e-6)  # f*v*t/R = 10 px
    assert om.apparent_height_px(1.7, 30.0, 0.0) == pytest.approx(om.F_PX * 1.7 / 30.0)
    assert om.apparent_height_px(1.7, 30.0, 90.0) == pytest.approx(om.F_PX * 0.3 / 30.0)


def test_recorded_heldout_split_is_disjoint_and_beats_marginal():
    # the pre-declared 4 -> 5 location split: better log loss than the marginal rate, but NOT well calibrated
    # (ECE ~0.10: held-out scenes are easier); documented in results/VISION_RESULTS.md
    v = json.load(open(om.PARAMS_PATH))['validation_heldout_locations']
    assert set(v['heldout_locations']).isdisjoint(v['fit_locations'])
    assert v['model']['logloss'] < v['marginal']['logloss']


def test_scene_draw_spreads_detectability():
    rng = np.random.default_rng(3)
    ps = [om.ObsModel.draw_scene(rng).p_detect(40.0) for _ in range(40)]
    assert np.std(ps) > 0.02
    assert om.ObsModel(scene_offset=1.0).p_detect(40.0) > om.ObsModel(scene_offset=-1.0).p_detect(40.0)


def test_recorded_lolo_calibration():
    v = json.load(open(om.PARAMS_PATH))['validation_lolo']
    assert v['model_no_scene']['logloss'] < v['marginal']['logloss']
    assert v['model_no_scene']['logloss'] < v['placeholder']['logloss']
    assert v['model_no_scene']['ece'] < 0.05


# ---------------------------------------------------------------- SearchMaps (map-update API)
def _maps():
    return om.new_maps((0, 0, 60, 60), cell_m=2.0, prior=1e-3)


def test_maps_negative_looks_raise_coverage_and_lower_posterior():
    S = _maps()
    p0 = S.posterior().copy()
    ok = S.update((30, 0), 10.0, math.pi / 2, 30.0, detections=[])
    assert ok.any()
    assert (S.coverage()[ok] > 0).all() and (S.posterior()[ok] < p0[ok]).all()
    assert (S.coverage()[~ok] == 0).all()


def test_maps_same_view_repeats_cover_less_than_new_views():
    A, B = _maps(), _maps()
    for _ in range(4):
        A.update((30, 0), 10.0, math.pi / 2, 30.0)
    B.update((30, 0), 10.0, math.pi / 2, 30.0); B.update((0, 30), 10.0, 0.0, 30.0)
    B.update((30, 60), 10.0, -math.pi / 2, 30.0); B.update((60, 30), 10.0, math.pi, 30.0)
    iy, ix = np.argmin(abs(A.ys - 30)), np.argmin(abs(A.xs - 30))
    assert B.coverage()[iy, ix] > A.coverage()[iy, ix]
    assert B.view_classes()[iy, ix] > A.view_classes()[iy, ix]


def test_maps_detection_creates_candidate_with_separate_covariance():
    S = _maps()
    for k in range(3):
        d = S.project_detection((30, 0), 10.0, math.pi / 2, 30.0, 0.0, 0.85, h_px=40)
        S.update((30, 0), 10.0, math.pi / 2, 30.0, detections=[d])
    c = S.candidates(p_min=0.05)
    assert c, 'no candidate'
    best = max(c, key=lambda x: x['confidence'])
    assert abs(best['x'] - 30) < 4 and abs(best['y'] - 30) < 4
    assert 0 < best['confidence'] <= 1 and np.linalg.det(np.array(best['cov'])) > 0
    # a single low-score hit is weaker evidence than a high-score hit
    S1, S2 = _maps(), _maps()
    S1.update((30, 0), 10.0, math.pi / 2, 30.0, detections=[S1.project_detection((30, 0), 10.0, math.pi / 2, 30.0, 0.0, 0.55)])
    S2.update((30, 0), 10.0, math.pi / 2, 30.0, detections=[S2.project_detection((30, 0), 10.0, math.pi / 2, 30.0, 0.0, 0.95)])
    assert S2.posterior().max() > S1.posterior().max()
