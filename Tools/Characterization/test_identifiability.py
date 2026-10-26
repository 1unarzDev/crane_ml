"""Identifiability: simulator-truth recovery on the fixture, for the estimators the gate keeps.

Kept from the owner's adversarial suite: ID-01..08 (mixer, current/bias split, latency) and
ID-11/12 (drag terms, thrust-vs-drag scale). Each case asserts its expected outcome directly:
PASS cases recover the truth, ABSTAIN cases must abstain with the named reason.
"""
import math
from dataclasses import replace
from functools import lru_cache

import numpy as np
import pytest

import estimators as E
import excitation as X
import faults as F
import plant as P

SEEDS = tuple(range(100, 120))  # 20 fixed seeds
EST = E.Estimators()


def make(cfg=None, dt=P.DT):
    if cfg is not None:
        cfg.validate()
    return P.VesselModel(dt=dt) if cfg is None else F.FaultedVessel(cfg, dt=dt)


@lru_cache(maxsize=None)
def pulses(cfg):
    return tuple(X.all_pulses(lambda: make(cfg)))


def experiments(cfg, seed):
    out = []
    for j, (cmd, log) in enumerate(pulses(cfg)):
        obs = F.make_observations(log, cfg, seed * 1000 + j)  # independent noise per pulse
        W, s = EST.measure_pulse_wrench(obs, X.REST_S, X.REST_S + X.HOLD_S)
        out.append((cmd, W, s))
    return out


def test_id01_signs_and_columns():
    cfg = F.FaultConfig(signs=(1, -1, 1, 1))
    for seed in SEEDS:
        est = EST.estimate_mixer(experiments(cfg, seed))
        assert est["status"] == E.OK, est
        assert tuple(est["signs"]) == cfg.signs
        cos = []
        for i, s in enumerate(P.NOMINAL_THRUSTERS):
            te = np.array(P.column(replace(s, mount_yaw_deg=cfg.angles[i]))) * cfg.signs[i]
            ee = np.array(P.column(replace(s, mount_yaw_deg=est["angles_deg"][i]))) * est["signs"][i]
            cos.append(float(te @ ee / (np.linalg.norm(te) * np.linalg.norm(ee))))
        assert min(cos) >= 0.99, cos


def test_id02_mount_and_strength():
    cfg = F.FaultConfig(angles=(0, 15, 0, 0), strengths=(1, 1, 0.6, 1))
    worst_a = worst_s = 0.0
    for seed in SEEDS:
        est = EST.estimate_mixer(experiments(cfg, seed))
        assert est["status"] == E.OK, est
        worst_a = max(worst_a, max(abs(np.array(est["angles_deg"]) - np.array(cfg.angles))))
        worst_s = max(worst_s, max(abs(np.array(est["strengths"]) / np.array(cfg.strengths) - 1)))
    assert worst_a <= 1.5 and worst_s <= 0.03, (worst_a, worst_s)


def test_id03_surge_only_is_unidentifiable():
    cfg = F.FaultConfig(angles=(0, 15, 0, 0), strengths=(1, 1, 0.6, 1))
    exps = []
    for c in (0.3, 0.6, -0.3, -0.6):
        log = X.mixed_pulse_log(lambda: make(cfg), (c, c, c, c))
        obs = F.make_observations(log, cfg, 1)
        W, s = EST.measure_pulse_wrench(obs, X.REST_S, X.REST_S + X.HOLD_S)
        exps.append(((c, c, c, c), W, s))
    res = EST.estimate_mixer(exps)
    assert res["status"] == E.ABSTAIN and res["reason"] == "rank_deficient_fisher", res
    assert res["rank"] < 8


def test_id01_id02_chi2_is_calibrated_at_truth():
    """With independent per-pulse noise and the propagated sigma, chi2 ~ dof (not 100s)."""
    cfg = F.FaultConfig(angles=(0, 15, 0, 0), strengths=(1, 1, 0.6, 1))
    ratios = []
    for seed in SEEDS:
        est = EST.estimate_mixer(experiments(cfg, seed))
        assert est["status"] == E.OK
        ratios.append(est["chi2"] / est["dof"])
    assert 0.6 <= np.mean(ratios) <= 1.4 and max(ratios) <= 2.0, (np.mean(ratios), max(ratios))


HEADINGS = np.radians(np.arange(64) * 360.0 / 64)


@lru_cache(maxsize=None)
def drift_mean(cfg, headings_key, ramp):
    heads = np.array(headings_key)
    return X.hold_drift(lambda: make(cfg), heads, ramp_hold=ramp,
                        hold_s=1.0 if not ramp else 120.0 / len(heads))


def drift_noisy(cfg, heads, seed, bias, ramp=False):
    mean = drift_mean(cfg, tuple(heads), ramp)
    rng = np.random.default_rng(seed)
    return mean[:, :2] + np.array(bias) + rng.normal(0, 0.02, (len(heads), 2)), mean[:, 2]


def test_id04_current_bias_separated():
    cfg = F.FaultConfig(current=(0.10, 0.15))
    bias = (0.02, -0.01)
    ce, be = [], []
    for seed in SEEDS:
        d, r = drift_noisy(cfg, HEADINGS, seed, bias)
        est = EST.separate_current_bias(HEADINGS, d, 0.02, yaw_rate=r)
        assert est["status"] == E.OK, est
        ce.append(np.max(np.abs(np.array(est["current"]) - cfg.current)))
        be.append(np.max(np.abs(np.array(est["bias_xy"]) - bias)))
    assert max(ce) <= 0.01 and np.median(be) <= 0.01 and max(be) <= 0.02, (max(ce), np.median(be), max(be))


@pytest.mark.parametrize("span_deg,n,reason", [
    (20.0, 64, "poor_heading_geometry"),
    (20.0, 640, "poor_heading_geometry"),      # more samples do NOT buy identifiability
    (0.0, 1, "rank_deficient_heading_diversity")])
def test_id05_narrow_heading_abstains(span_deg, n, reason):
    cfg = F.FaultConfig(current=(0.10, 0.15))
    heads = np.radians(np.linspace(0, span_deg, n))
    reasons = set()
    for seed in SEEDS:
        d, _ = drift_noisy(cfg, heads, seed, (0.02, -0.01))
        est = EST.separate_current_bias(heads, d, 0.02)
        assert est["status"] == E.ABSTAIN, est
        reasons.add(est["reason"])
    assert reasons == {reason}, reasons


def test_id05b_three_spread_headings_are_identifiable_and_not_a_crash():
    cfg = F.FaultConfig(current=(0.10, 0.15))
    bias = (0.02, -0.01)
    for deg in ([0, 120, 240], [0, 90, 180, 270], [0, 72, 144, 216, 288]):
        heads = np.radians(deg)
        errs, outs = [], set()
        for seed in SEEDS:
            d, _ = drift_noisy(cfg, heads, seed, bias)
            # precision is a separate, noise-dependent gate: relax it here to test geometry only
            est = EST.separate_current_bias(heads, d, 0.02, max_std=0.05)
            outs.add(est["status"])
            if est["status"] == E.OK:
                errs.append(np.max(np.abs(np.array(est["current"]) - cfg.current)) / max(est["std"][:2]))
        assert outs == {E.OK}, (deg, outs)
        assert max(errs) <= 4.0, errs   # 4 predicted sigma
    # at 3 headings and sigma 0.02 the PRECISION gate (not geometry) stops the default tolerance
    d, _ = drift_noisy(cfg, np.radians([0, 120, 240]), 1, bias)
    est = EST.separate_current_bias(np.radians([0, 120, 240]), d, 0.02)
    assert est["status"] == E.ABSTAIN and est["reason"] == "insufficient_precision"


def test_two_headings_never_pass():
    """Two headings give 4 equations for 4 unknowns: dof 0 abstains UNDERDETERMINED, never OK."""
    heads = np.radians([0.0, 90.0])
    d = np.array([[0.1, 0.0], [0.0, -0.1]])
    est = EST.separate_current_bias(heads, d, 0.002, max_std=1.0)
    assert est["status"] == E.ABSTAIN and est["reason"] == E.UNDERDETERMINED, est


RAMP = 0.2


def _ramp_cfg():
    return F.FaultConfig(current=(RAMP * math.cos(math.radians(30)), RAMP * math.sin(math.radians(30))),
                         current_ramp_s=120.0)


@pytest.mark.parametrize("shuffled", [False, True], ids=["sequential", "shuffled"])
def test_id06_ramping_current_is_flagged_or_abstained(shuffled):
    cfg = _ramp_cfg()
    heads = HEADINGS[np.random.default_rng(7).permutation(len(HEADINGS))] if shuffled else HEADINGS
    reasons = set()
    for seed in SEEDS:
        d, _ = drift_noisy(cfg, heads, seed, (0.02, -0.01), ramp=True)
        est = EST.separate_current_bias(heads, d, 0.02)
        assert est["status"] == E.ABSTAIN, est
        reasons.add(est["reason"])
    assert reasons <= {"poor_fit_chi2", "residual_not_white"}, reasons
    if shuffled:
        assert reasons == {"poor_fit_chi2"}   # whiteness cannot see it in shuffled order; chi2 does


def test_id04_whiteness_false_alarm_rate_on_stationary_current():
    """The p<0.001 whiteness gate must not abstain on clean stationary data (all 20 seeds)."""
    cfg = F.FaultConfig(current=(0.10, 0.15))
    reasons = []
    for seed in SEEDS:
        d, _ = drift_noisy(cfg, HEADINGS, seed, (0.0, 0.0))
        est = EST.separate_current_bias(HEADINGS, d, 0.02)
        reasons.append(est.get("reason"))
    assert all(r is None for r in reasons), reasons


@lru_cache(maxsize=None)
def latency_logs(dead=0.16):
    prog = X.step_chirp_program()
    a = X.wrench_program(lambda: make(F.FaultConfig(dead_time_s=dead)), prog, 14.0)[0]
    b = X.wrench_program(lambda: make(F.FaultConfig(obs_delay_s=dead)), prog, 14.0)[0]
    nom = X.wrench_program(lambda: make(F.FaultConfig()), prog, 14.0)[0]
    return a, b, nom


def test_id07_total_latency_and_unidentifiable_split():
    a, b, nom = latency_logs()
    obs_b = F.make_observations(b, F.FaultConfig(obs_delay_s=0.16), 0, sigma_v=0.0, sigma_r=0.0)
    assert np.max(np.abs(obs_b["u"] - a.u)) <= 1e-9
    assert np.max(np.abs(obs_b["r"] - a.r)) <= 1e-9
    split_status, errs = set(), []
    for seed in SEEDS:
        for log, cfg in ((a, F.FaultConfig()), (b, F.FaultConfig(obs_delay_s=0.16))):
            obs = F.make_observations(log, cfg, seed)
            r = EST.estimate_latency(nom, obs)
            assert r["status"] == E.OK, r
            errs.append(abs(r["total_latency_s"] - 0.16))
            split_status.add(r["split"]["status"])
            assert r["split"]["reason"] == "fisher_rank_deficient" and not r["fisher"]["identifiable"]
            assert r["fisher"]["cond"] > 1e6
    assert max(errs) <= 0.010, max(errs)
    assert split_status == {"UNIDENTIFIABLE"}


def test_id08_split_with_truth_clock():
    a, b, nom = latency_logs()
    worst = 0.0
    for seed in SEEDS:
        for log, cfg, (dead, lag) in ((a, F.FaultConfig(), (0.16, 0.0)),
                                      (b, F.FaultConfig(obs_delay_s=0.16), (0.0, 0.16))):
            obs = F.make_observations(log, cfg, seed)
            r = EST.estimate_latency(nom, obs, truth_log=log)
            assert r["split"]["status"] == E.OK, r
            assert r["fisher"]["identifiable"] and r["fisher"]["cond"] < 1e3, r["fisher"]
            worst = max(worst, abs(r["split"]["actuator_dead_time_s"] - dead),
                        abs(r["split"]["obs_stamp_lag_s"] - lag))
    assert worst <= 0.010, worst


@lru_cache(maxsize=None)
def hold_cache_plain(fx):
    u, v, r, al = X.steady_state(lambda: make(), (fx, 0.0, 0.0), seconds=20.0)
    return (u, v, r, al.steady_wrench[0])


def steady_drag_points(speeds_fx, seed):
    rng = np.random.default_rng(seed)
    v, f = [], []
    for fx in speeds_fx:
        u, _, _, al = hold_cache_plain(fx)
        v.append(u + rng.normal(0, 0.02 / 10))
        f.append(al)
    return np.array(v), np.array(f)


def test_id11_single_speed_drag_abstains_and_constant_gain_is_fine():
    v, f = steady_drag_points([58.65] * 6, 1)
    two = EST.fit_drag(v, f, 0.002)
    one = EST.fit_drag(v, f, 0.002, model="linear")
    assert one["status"] == E.OK  # constant-gain drag IS identifiable from one speed (and wrong)
    assert two["status"] == E.ABSTAIN and two["reason"] == "unidentifiable_drag_terms", two
    assert np.ptp(v) < 0.02


def drag_fit_sigma(speeds, sigma_v):
    """First-order covariance of the (a, b) fit when only v carries noise (F exact):
    cov(theta) = sigma_v^2 (X'X)^-1 X' diag(F'^2) X (X'X)^-1, F'(v) = a + 2 b |v|."""
    v = np.asarray(speeds, float)
    X_ = np.column_stack([v, v * np.abs(v)])
    Fp = P.DRAG_SURGE[0] + 2 * P.DRAG_SURGE[1] * np.abs(v)
    G = np.linalg.inv(X_.T @ X_)
    cov = sigma_v ** 2 * G @ X_.T @ np.diag(Fp ** 2) @ X_ @ G
    return np.sqrt(np.diag(cov)) / np.array(P.DRAG_SURGE)   # relative std (lin, quad)


def test_id11_multi_speed_recovers_both_coefficients():
    """Tolerance = 4 x the derived 1-sigma (not a round number)."""
    fx = [5.0, 20.0, 40.0, 60.0, 90.0, 120.0]
    v0, _ = steady_drag_points(fx, 0)
    sig = drag_fit_sigma(v0, 0.002)
    errs = []
    for seed in SEEDS:
        v, f = steady_drag_points(fx, seed)
        est = EST.fit_drag(v, f, 0.002)
        assert est["status"] == E.OK, est
        errs.append((abs(est["coeffs"][0] / P.DRAG_SURGE[0] - 1), abs(est["coeffs"][1] / P.DRAG_SURGE[1] - 1)))
    e = np.max(np.array(errs), axis=0)
    assert e[0] <= 4 * sig[0] and e[1] <= 4 * sig[1], (e, sig)
    rms = np.sqrt(np.mean(np.square(np.array(errs)), axis=0))
    assert np.all(rms <= 1.6 * sig) and np.all(rms >= 0.4 * sig), (rms, sig)


def _scale_rows(seed, transient):
    cfg = F.FaultConfig(strengths=(0.8,) * 4, drag=(1.25, 1.25, 1.25))
    if not transient:
        v, f = [], []
        rng = np.random.default_rng(seed)
        for fx in (20.0, 40.0, 60.0, 80.0, 100.0):
            u, _, _, _ = X.steady_state(lambda: make(cfg), (fx, 0.0, 0.0), seconds=20.0)
            v.append(u + rng.normal(0, 0.002))
            f.append(make().steady_wrench(make().allocate((fx, 0, 0)).commands)[0])
        v = np.array(v)
        D = np.array([-P.drag(x, P.DRAG_SURGE) for x in v])
        sB = np.array([(P.DRAG_SURGE[0] + 2 * P.DRAG_SURGE[1] * x) * 0.002 for x in v])
        return np.array(f), D, np.zeros(len(v)), sB
    prog = lambda t: (60.0 if (1.0 <= t < 4.0 or 8.0 <= t < 10.0) else 0.0, 0.0, 0.0)
    true_log = _scale_logs(cfg, prog)
    nom_log = _scale_logs(None, prog)
    obs = F.make_observations(true_log, cfg, seed)
    t = obs["t"]
    A = np.concatenate([[0], np.cumsum(0.5 * (nom_log.wrench[1:, 0] + nom_log.wrench[:-1, 0]) * np.diff(nom_log.t))])
    Dm = np.array([-P.drag(x, P.DRAG_SURGE) for x in obs["u"]])
    B = np.concatenate([[0], np.cumsum(0.5 * (Dm[1:] + Dm[:-1]) * np.diff(t))])
    sB = np.sqrt(np.cumsum((np.array([P.DRAG_SURGE[0] + 2 * P.DRAG_SURGE[1] * abs(x) for x in obs["u"]])
                            * 0.02 * P.DT) ** 2))
    y = P.MASS * obs["u"]
    idx = np.arange(25, len(t), 10)
    return A[idx], B[idx], y[idx], sB[idx]


_SCALE_LOGS = {}


def _scale_logs(cfg, prog):
    if cfg not in _SCALE_LOGS:
        _SCALE_LOGS[cfg] = X.wrench_program(lambda: make(cfg), prog, 12.0)[0]
    return _SCALE_LOGS[cfg]


def test_id12_steady_only_is_ratio_only_and_transient_separates():
    A, B, y, sB = _scale_rows(1, transient=False)
    steady = EST.fit_thrust_drag_scale(A, B, y, sB)
    assert steady["status"] == E.ABSTAIN and steady["reason"] == "only_ratio_identifiable", steady
    worst = 0.0
    for seed in SEEDS:
        A, B, y, sB = _scale_rows(seed, transient=True)
        est = EST.fit_thrust_drag_scale(A, B, y, sB)
        assert est["status"] == E.OK, est
        worst = max(worst, abs(est["thrust_scale"] / 0.8 - 1), abs(est["drag_scale"] / 1.25 - 1))
    assert worst <= 0.05, worst


def test_degenerate_helpers_never_pass():
    with pytest.raises(ValueError):
        E.chi2_crit(0, 2.33)
    assert E.ljung_box(np.arange(10.0), 5)["reason"] == E.DEGENERATE      # n <= 2h
    assert E.ljung_box(np.ones(50), 5)["reason"] == E.DEGENERATE          # zero variance
    with pytest.raises(ValueError):
        E.ljung_box_q(np.arange(10.0), 5)
    t = np.arange(100) * 0.02
    r = np.full(100, 0.01)
    g = EST.gyro_bias(t, r, np.ones(100, bool), np.zeros(100), 0.005, starts_at_rest=True)
    assert g["reason"] == E.DEGENERATE
