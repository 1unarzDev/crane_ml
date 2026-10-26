"""Gate checks (design section 4): every reason code and signature on small synthetic inputs, the
degenerate cases, and the pure-Python fault-injection closure (design section 7)."""
import copy
import math
from functools import lru_cache

import numpy as np
import pytest

import faults as F
import gate as G
import plant as P
import synthetic as S


def gate(inp, **kw):
    kw.setdefault("yaml_path", "hardware.yaml")
    return G.run_gate(inp["yaml"], inp["provenance"], inp["run"], inp["snapshot"], **kw)


def base_inputs(**kw):
    return S.analytic_inputs(**kw)


def reasons_of(rep, scope_prefix=None, check=None):
    return {c["reason"] for c in rep["checks"] if c["status"] != "OK"
            and (scope_prefix is None or c["scope"].startswith(scope_prefix))
            and (check is None or c["check"] == check)}


def status(rep, name):
    return rep["parameters"][name]["status"]


def sigs(rep, code=None):
    return [s for s in rep["signatures"] if code is None or s["signature"] == code]


# ------------------------------------------------------------------ clean recovery
def test_clean_inputs_are_characterized_and_recover_the_profile():
    rep = gate(base_inputs())
    assert rep["characterized"] and rep["summary"] == "characterized", rep["reasons"]
    for name in G.DIRECTIONS:
        axis, suf = name[:-1], ("Pos" if name.endswith("+") else "Neg")
        p = rep["parameters"][name]
        assert p["status"] == G.IDENTIFIED
        assert abs(p["D1"] / P.IDEAL[axis]["d1" + suf] - 1) < 1e-9
    meff = P.effective_mass(P.IDEAL)
    for j, axis in enumerate(G.AXES):
        p = rep["parameters"][f"meff.{axis}"]
        assert p["status"] == G.IDENTIFIED and abs(p["meff"] / meff[j] - 1) < 1e-9
    assert [s["signature"] for s in rep["signatures"]] == ["SCALE_UNANCHORED"]
    assert not reasons_of(rep)


# ------------------------------------------------------------------ provenance
def test_sim_derived_by_path_and_by_missing_hardware_hash():
    rep = gate(base_inputs(), yaml_path="/x/mhseals_testing/config/sim/hardware_crane.yaml")
    assert all(status(rep, n) == G.PRIOR and "SIM_DERIVED" in rep["parameters"][n]["reasons"] for n in G.DIRECTIONS)
    inp = base_inputs()
    del inp["provenance"]["directions"]["sway-"]["hardware"]["hash"]
    rep = gate(inp)
    assert rep["parameters"]["sway-"]["reasons"] == ["SIM_DERIVED"]
    assert status(rep, "sway+") == G.IDENTIFIED


def test_provenance_missing():
    inp = base_inputs()
    del inp["provenance"]["directions"]["surge+"]["fit"]["session"]
    rep = gate(inp)
    assert rep["parameters"]["surge+"]["reasons"] == ["PROVENANCE_MISSING"] and not rep["characterized"]
    inp = base_inputs()
    del inp["provenance"]["software"]
    rep = gate(inp)
    assert all("PROVENANCE_MISSING" in rep["parameters"][n]["reasons"] for n in G.DIRECTIONS)
    rep = G.run_gate(base_inputs()["yaml"], None, base_inputs()["run"], None, yaml_path="hardware.yaml")
    assert all({"PROVENANCE_MISSING", "SIM_DERIVED"} <= set(rep["parameters"][n]["reasons"]) for n in G.DIRECTIONS)
    inp = base_inputs()
    inp["snapshot"] = None
    assert "PROVENANCE_MISSING" in gate(inp)["parameters"]["yaw+"]["reasons"]


def test_calibration_invalid_against_the_fit_time_snapshot():
    inp = base_inputs()
    hw = copy.deepcopy(inp["snapshot"]["value"]["effective"])
    hw["reversed"] = [True, False, False, False]
    inp["snapshot"] = S.snapshot(hw)
    rep = gate(inp)
    assert all("CALIBRATION_INVALID" in rep["parameters"][n]["reasons"] for n in G.DIRECTIONS)
    ev = next(c for c in rep["checks"] if c.get("reason") == "CALIBRATION_INVALID")["evidence"]
    assert ev["recorded"] != ev["snapshot"] and ev["fit_config"] == "c0007"


@pytest.mark.parametrize("change", ["muted", "channel_map"])
def test_needs_verify_caps_at_provisional(change):
    inp = base_inputs()
    if change == "muted":
        inp["provenance"]["directions"]["yaw-"]["gate"]["muted"] = True
    else:
        hw = copy.deepcopy(inp["snapshot"]["value"]["effective"])
        hw["channel_map"] = [2, 1, 3, 4]
        inp["snapshot"] = S.snapshot(hw)
    rep = gate(inp)
    assert status(rep, "yaw-") == G.PROVISIONAL and rep["parameters"]["yaw-"]["reasons"] == ["NEEDS_VERIFY"]
    assert not rep["characterized"]


def test_battery_is_recorded_not_rejected():
    inp = base_inputs()
    for d in inp["provenance"]["directions"].values():
        d["battery_v"] = 14.6
    rep = gate(inp)
    assert rep["characterized"]
    assert {n["direction"] for n in rep["notes"] if n["note"] == "BATTERY_OFF_VREF"} == set(G.DIRECTIONS)


# ------------------------------------------------------------------ conditions
@pytest.mark.parametrize("field,value", [("verdict", "marginal"), ("ceiling", 0.85), ("drift_trust", 0.7)])
def test_conditions_poor(field, value):
    inp = base_inputs()
    inp["run"]["conditions"][field] = value
    rep = gate(inp)
    assert rep["summary"] == "prior" and all("CONDITIONS_POOR" in rep["parameters"][n]["reasons"] for n in G.DIRECTIONS)


@pytest.mark.parametrize("state", ["trending", "changed"])
def test_drift_unsteady(state):
    inp = base_inputs()
    inp["run"]["conditions"]["state"] = state
    assert all(gate(inp)["parameters"][n]["reasons"] == ["DRIFT_UNSTEADY"] for n in G.DIRECTIONS)


@pytest.mark.parametrize("cond", [None, {"verdict": "unknown"}])
def test_conditions_unknown_caps_at_provisional(cond):
    inp = base_inputs()
    inp["run"]["conditions"] = cond
    rep = gate(inp)
    assert not rep["characterized"] and rep["summary"] == "partial"
    assert all(status(rep, n) == G.PROVISIONAL and rep["parameters"][n]["reasons"] == ["CONDITIONS_UNKNOWN"]
               for n in rep["parameters"])


# ------------------------------------------------------------------ per hold
def _hold_index(inp, name, amp):
    return next(i for i, h in enumerate(inp["run"]["holds"])
                if h["label"] == name and abs(max(abs(c) for c in h["command"]) - amp) < 1e-9)


def _mutate_hold(why, inp, i):
    h = inp["run"]["holds"][i]
    s = h["samples"]
    t = np.array(s["t"])
    win = (t >= h["start"] + 1.0) & (t <= h["end"])
    if why == "override":
        h["override"] = True
    elif why == "rejected_by_audit":
        inp["run"]["holds_rejected"].append({"command": h["command"], "start": h["start"], "end": h["end"],
                                             "ok": False, "coverage": 0.2})
    elif why == "no_samples":
        del h["samples"]
    elif why in ("too_few_samples", "gap_exceeds_limit", "coverage"):
        if why == "gap_exceeds_limit":
            keep = ~(win & (t > h["start"] + 2.0) & (t < h["start"] + 2.5))
        else:  # every third sample (0.12 s apart, no gap): 17 samples in 2 s, or 1/3 coverage in 9 s
            keep = ~win | (np.arange(len(t)) % 3 == 0)
        for k in list(s):
            s[k] = np.array(s[k])[keep].tolist()
    elif why == "flat":
        for c in G.CHANNELS:
            arr = np.array(s[c])
            arr[win] = arr[win][0]
            s[c] = arr.tolist()
    elif why == "frozen":
        for c in G.CHANNELS:
            arr = np.array(s[c])
            k = np.where(win)[0][10]
            arr[k:k + 4] = arr[k]
            s[c] = arr.tolist()
    elif why == "clock_skew":
        s["t_sensor"] = (t * (1.0 + 0.01) - t[0] * 0.01).tolist()
    return h


@pytest.mark.parametrize("why", ["override", "rejected_by_audit", "no_samples", "too_few_samples",
                                 "gap_exceeds_limit", "flat", "frozen", "coverage", "clock_skew"])
def test_hold_invalid(why):
    inp = base_inputs(hold=10.0) if why == "coverage" else base_inputs()
    i = _hold_index(inp, "sway+", 0.5)
    _mutate_hold(why, inp, i)
    rep = gate(inp)
    hold_reasons = reasons_of(rep, f"hold:{i}")
    assert hold_reasons == {f"HOLD_INVALID:{why}"}, hold_reasons
    assert all(c["status"] == "OK" for c in rep["checks"] if c["scope"].startswith("hold:") and c["scope"] != f"hold:{i}")
    # the remaining two amplitudes still fit, exactly: capped at provisional
    assert status(rep, "sway+") == G.PROVISIONAL and rep["parameters"]["sway+"]["reasons"] == ["UNDERDETERMINED"]


def test_clock_checked_and_ok_when_sensor_stamps_agree():
    inp = base_inputs()
    i = _hold_index(inp, "surge+", 0.75)
    s = inp["run"]["holds"][i]["samples"]
    s["t_sensor"] = (np.array(s["t"]) + 0.003).tolist()
    rep = gate(inp)
    assert rep["characterized"]
    assert next(c for c in rep["checks"] if c["scope"] == f"hold:{i}")["evidence"]["clock"]["checked"]


# ------------------------------------------------------------------ per direction
def test_low_snr():
    rep = gate(base_inputs(amplitudes=(0.75, 0.5, 0.03)))
    assert all("LOW_SNR" in rep["parameters"][n]["reasons"] for n in G.DIRECTIONS)


def test_neg_coeff_recomputed_from_holds():
    """Speeds following v ~ e^0.6 need d1 < 0; the YAML keeps a term-dropped refit (d1 = 0)."""
    law = lambda name, e: 1.2 * e ** 0.6
    inp = base_inputs(speed_law=law)
    v = np.array([law("", a) for a in (0.75, 0.5, 0.25)])
    e2 = np.array([0.75, 0.5, 0.25]) ** 2
    d2 = float((v * v) @ e2 / ((v * v) @ (v * v)))
    for name in G.DIRECTIONS:
        inp["yaml"]["characterization"]["speed_curves"][name] = {"d1": 0.0, "d2": d2}
    rep = gate(inp)
    assert all("NEG_COEFF" in rep["parameters"][n]["reasons"] for n in G.DIRECTIONS)
    ev = next(c for c in rep["checks"] if c.get("reason") == "NEG_COEFF")["evidence"]
    assert ev["d1"] < 0 and ev["yaml"]["d1"] == 0.0


def test_underdetermined_two_speeds_provisional_one_speed_prior():
    rep = gate(base_inputs(amplitudes=(0.75, 0.25)))
    assert all(status(rep, n) == G.PROVISIONAL and rep["parameters"][n]["reasons"] == ["UNDERDETERMINED"]
               for n in G.DIRECTIONS)
    assert not rep["characterized"]
    rep = gate(base_inputs(amplitudes=(0.75,)))
    assert all(status(rep, n) == G.PRIOR and "UNDERDETERMINED" in rep["parameters"][n]["reasons"] for n in G.DIRECTIONS)


def test_ill_conditioned():
    rep = gate(base_inputs(amplitudes=(0.5, 0.501, 0.502)))
    assert all("ILL_CONDITIONED" in rep["parameters"][n]["reasons"] for n in G.DIRECTIONS)


def test_degenerate_zero_residual_variance():
    rep = gate(base_inputs(exact_means=True))
    assert all("DEGENERATE" in rep["parameters"][n]["reasons"] for n in G.DIRECTIONS)
    assert all(c["evidence"]["why"] == "zero residual variance" for c in rep["checks"]
               if c.get("reason") == "DEGENERATE")


def test_degenerate_ljung_box_too_few_after_decimation():
    """Strongly correlated within-hold noise decimates to n <= 2h: abstain, never a pass."""
    def ar(rng, n, rho=0.98):
        x = np.zeros(n)
        e = rng.normal(0, 0.01, n)
        for i in range(1, n):
            x[i] = rho * x[i - 1] + e[i]
        return x
    rep = gate(base_inputs(hold_noise=ar))
    lb = [c for c in rep["checks"] if c["check"] == "ljung_box" and c["status"] != "OK"]
    assert lb and all(c["reason"] == "DEGENERATE" and c["evidence"]["why"] == "ljung_box_n_le_2h" for c in lb)
    assert all(status(rep, n) == G.PRIOR for n in G.DIRECTIONS)


def test_misfit_chi2():
    """Speeds off the e^2 = d1 v + d2 v^2 family; the YAML carries the holds' own fit."""
    amps = (1.0, 0.8, 0.6, 0.4, 0.2)
    law = lambda name, e: 0.5 * e + 0.6 * e ** 3
    inp = base_inputs(amplitudes=amps, speed_law=law, noise=0.002)
    v = np.array([law("", a) for a in amps])
    d = np.linalg.lstsq(np.column_stack([v, v * v]), np.array(amps) ** 2, rcond=None)[0]
    assert d[0] > 0 and d[1] > 0
    for name in G.DIRECTIONS:
        inp["yaml"]["characterization"]["speed_curves"][name] = {"d1": float(d[0]), "d2": float(d[1])}
    rep = gate(inp)
    whys = {c["evidence"].get("why") for c in rep["checks"] if c.get("reason") == "MISFIT"}
    assert whys == {"chi2"}, whys
    assert all(rep["parameters"][n]["reasons"] == ["MISFIT"] for n in G.DIRECTIONS)


def test_misfit_ljung_box():
    k = lambda rng, n: 0.01 * np.cos(0.5 * math.pi * np.arange(n)) + rng.normal(0, 0.002, n)
    rep = gate(base_inputs(hold_noise=k))
    whys = {c["evidence"].get("why") for c in rep["checks"] if c.get("reason") == "MISFIT"}
    assert whys == {"ljung_box"}
    assert all(rep["parameters"][n]["reasons"] == ["MISFIT"] for n in G.DIRECTIONS)


def test_misfit_when_yaml_curve_is_not_from_these_holds():
    inp = base_inputs()
    inp["yaml"]["characterization"]["speed_curves"]["yaw-"]["d1"] *= 1.6
    rep = gate(inp)
    assert rep["parameters"]["yaw-"]["reasons"] == ["MISFIT"] and not rep["characterized"]


@pytest.mark.parametrize("record", [{"converged": False}, {"low_ok": False}, {"dead": True}, None])
def test_not_converged(record):
    inp = base_inputs()
    if record is None:
        del inp["run"]["directions"]["sway+"]
    else:
        inp["run"]["directions"]["sway+"].update(record)
    rep = gate(inp)
    assert rep["parameters"]["sway+"]["reasons"] == ["NOT_CONVERGED"]
    assert rep["characterized"]  # sway does not gate `characterized`


def test_characterized_needs_surge_and_yaw_both_signs():
    inp = base_inputs()
    inp["run"]["directions"]["yaw-"]["converged"] = False
    rep = gate(inp)
    assert not rep["characterized"] and rep["summary"] == "partial"


# ------------------------------------------------------------------ consistency
def _scaled_minus(axis, factor, **kw):
    curves = S.ideal_curves()
    for k in ("d1", "d2"):
        curves[axis + "-"][k] *= factor
    return base_inputs(curves=curves, **kw)


def test_asymmetry_implausible():
    rep = gate(_scaled_minus("surge", 3.0))
    assert "ASYMMETRY_IMPLAUSIBLE" in rep["parameters"]["surge+"]["reasons"]
    assert "ASYMMETRY_IMPLAUSIBLE" in rep["parameters"]["surge-"]["reasons"]
    assert "REVERSE_INCONSISTENT" not in reasons_of(rep)  # equal counts on the default mixer
    assert status(rep, "sway+") == G.IDENTIFIED


def _reversed_inputs(factor):
    """FL and FR reversed: surge runs 4 forward / 0 reverse (sway, yaw stay 2/2); K_r = 40.2 makes
    surge k+ != k-."""
    base = copy.deepcopy(P.IDEAL)
    for t in base["thrusters"]:
        t["kReverse"] = 40.2
    hw = S.hardware(reversed_=[True, True, False, False])
    curves = S.ideal_curves(base, hw)
    for k in ("d1", "d2"):
        curves["surge-"][k] *= factor
    inp = base_inputs(curves=curves, mu=S.ideal_mu(base, hw))
    inp["provenance"], inp["snapshot"] = S.provenance(hw), S.snapshot(hw)
    return inp, base, hw


def test_reverse_inconsistent_only_where_counts_differ():
    inp, base, hw = _reversed_inputs(1.0)
    g = G.axis_gains(base, hw)
    assert (g["surge+"]["forward"], g["surge+"]["reverse"]) == (4, 0)
    assert g["surge+"]["k"] != g["surge-"]["k"]
    assert (g["sway+"]["forward"], g["sway+"]["reverse"]) == (2, 2)
    rep = gate(inp, base_profile=base)
    assert rep["characterized"], rep["reasons"]   # d ratio = k-/k+ exactly: consistent
    inp, base, hw = _reversed_inputs(3.0)
    rep = gate(inp, base_profile=base)
    assert "REVERSE_INCONSISTENT" in rep["parameters"]["surge+"]["reasons"]
    assert "ASYMMETRY_IMPLAUSIBLE" not in rep["parameters"]["surge+"]["reasons"]


def test_axis_gains_default_mixer_equal_counts():
    g = G.axis_gains(P.IDEAL, S.hardware())
    for name in G.DIRECTIONS:
        assert (g[name]["forward"], g[name]["reverse"]) == (2, 2)
    assert abs(g["surge+"]["k"] - 4 * 51.5 * math.cos(math.radians(45))) < 1e-9
    assert abs(g["yaw+"]["k"] / (4 * 51.5) - 0.61 * math.cos(math.radians(45))) < 1e-9  # yaw arm 0.431 m


@pytest.mark.parametrize("factor", [5.0, 0.4])
def test_t95_implausible(factor):
    inp = base_inputs()
    inp["yaml"]["characterization"]["t95"]["surge"] *= factor
    rep = gate(inp)
    assert rep["parameters"]["meff.surge"]["reasons"] == ["T95_IMPLAUSIBLE"]
    assert status(rep, "surge+") == G.IDENTIFIED   # D does not depend on t95


def test_lag_dominated():
    rep = gate(base_inputs(), tau_est=1.0)
    assert all(rep["parameters"][f"meff.{a}"]["reasons"] == ["LAG_DOMINATED"] for a in G.AXES)
    base = copy.deepcopy(P.IDEAL)
    for t in base["thrusters"]:
        t["tauT"] = 0.3
    rep = gate(base_inputs(), base_profile=base)
    assert "LAG_DOMINATED" in rep["parameters"]["meff.yaw"]["reasons"]


def test_mu_fit_with_lags_recovers_truth():
    d1, d2, e2, mu = 0.07, 0.13, 0.5, 0.22
    lags = (0.1, 0.04, 0.05)
    t95 = G.t95_model(mu, e2, d1, d2, *lags)
    assert t95 > G.t95_closed(mu, e2, d1, d2)
    assert abs(G.fit_mu(t95, e2, d1, d2, lags, 0.2) / mu - 1) < 1e-4
    assert abs(G.fit_mu(G.t95_closed(mu, e2, d1, d2), e2, d1, d2, (0.0, 0.0, 0.0), 0.2) / mu - 1) < 1e-12


def test_t95_closed_matches_the_plant():
    """The gate's t95 model is the plant's: rest to 95 % at RK4, normalized by k."""
    k = G.axis_gains(P.IDEAL)["surge+"]["k"]
    d1, d2 = P.IDEAL["surge"]["d1Pos"] / k, P.IDEAL["surge"]["d2Pos"] / k
    mu = P.effective_mass(P.IDEAL)[0] / k
    e = 0.6
    rows = P.reference_run(P.IDEAL, lambda t: (k * e * e, 0.0, 0.0), 12.0, sample_dt=0.005)
    vinf = P.steady_speed(d1, d2, e * e)
    tr = next(a[0] + (0.95 * vinf - a[4]) * (b[0] - a[0]) / (b[4] - a[4])
              for a, b in zip(rows, rows[1:]) if b[4] >= 0.95 * vinf > a[4])
    assert abs(G.t95_closed(mu, e * e, d1, d2) / tr - 1) < 1e-4


# ------------------------------------------------------------------ signatures
def test_signature_thrust_or_hull_asym_cross_and_sign():
    inp = base_inputs()
    inp["run"]["directions"]["surge+"].update(response=[0.02, 0.05], low_response=[0.01, 0.04])
    rep = gate(inp)
    hit = [s for s in sigs(rep, "THRUST_OR_HULL_ASYM") if s.get("direction") == "surge+"]
    assert len(hit) == 1 and hit[0]["cross_axis"] == "yaw"
    rep = gate(_scaled_minus("sway", 1.2))
    assert [s["axis"] for s in sigs(rep, "THRUST_OR_HULL_ASYM")] == ["sway"]
    assert status(rep, "sway-") == G.IDENTIFIED  # a signature never changes a parameter


def test_signature_prop_drag_or_k():
    inp = base_inputs()
    curves = inp["yaml"]["characterization"]["speed_curves"]
    mu = S.ideal_mu()["surge"]
    for extra, expect in ((1.0, False), (1.5, True)):
        t = np.arange(0.0, 6.0, 0.02)
        v, out = 1.2, []
        for _ in t:
            out.append(v)
            v -= 0.02 * extra * G.curve_value(curves["surge+"], v) / mu
        run = copy.deepcopy(inp["run"])
        run["coastdowns"] = [{"axis": "surge", "sign": 1, "start": 0.0, "end": 6.0,
                              "samples": {"t": t.tolist(), "vx": out, "vy": [0.0] * len(t), "wz": [0.0] * len(t)}}]
        rep = G.run_gate(inp["yaml"], inp["provenance"], run, inp["snapshot"], yaml_path="hardware.yaml")
        assert bool(sigs(rep, "PROP_DRAG_OR_K")) == expect, sigs(rep)


def test_signature_gyro_bias():
    rep = gate(base_inputs(gyro_bias=0.02))
    assert sigs(rep, "GYRO_BIAS") and abs(sigs(rep, "GYRO_BIAS")[0]["bias_r"] - 0.02) < 0.005
    assert not sigs(gate(base_inputs()), "GYRO_BIAS")


def test_signature_environment_drift_never_enters_d():
    clean, drift = gate(base_inputs()), gate(base_inputs(current=(0.2, -0.1)))
    env = sigs(drift, "ENVIRONMENT_DRIFT")
    assert env and env[0]["separated"] and np.allclose(env[0]["current_world"], (0.2, -0.1), atol=0.01)
    assert not sigs(clean, "ENVIRONMENT_DRIFT")
    for name in G.DIRECTIONS:
        a, b = clean["parameters"][name], drift["parameters"][name]
        assert a["status"] == b["status"] == G.IDENTIFIED
    for c0, c1 in zip(clean["checks"], drift["checks"]):
        if c0["check"] == "direction":
            assert np.allclose(c0["evidence"]["speeds"], c1["evidence"]["speeds"], rtol=1e-9, atol=1e-12)


def test_signature_scale_unanchored_unless_anchored():
    inp = base_inputs()
    assert sigs(gate(inp), "SCALE_UNANCHORED")
    inp["run"]["scale_anchor"] = True
    assert not sigs(gate(inp), "SCALE_UNANCHORED")


def test_signature_estimator_lag_feeds_lag_prior():
    inp = base_inputs()
    t = np.arange(0.0, 20.0, 0.02)
    ref = np.sin(2 * math.pi * 0.5 * t) + 0.5 * np.sin(2 * math.pi * 1.3 * t)
    rng = np.random.default_rng(3)
    obs = np.interp(t - 0.12, t, ref) + rng.normal(0, 0.01, len(t))
    inp["run"]["lag_streams"] = {"t_raw": t.tolist(), "raw": ref.tolist(), "t_odom": t.tolist(),
                                 "odom": obs.tolist(), "sigma": 0.01}
    # the boat's t95 as an odom with that lag would have measured it
    char = inp["yaml"]["characterization"]
    mu, delay = S.ideal_mu(), None
    for a in G.AXES:
        c = char["speed_curves"][a + "+"]
        char["t95"][a] = G.t95_model(mu[a], 0.75 ** 2, c["d1"], c["d2"], 0.0, 0.0, 0.12)
    rep = gate(inp)
    lag = sigs(rep, "ESTIMATOR_LAG")
    assert lag and abs(lag[0]["delay_s"] - 0.12) <= 0.005
    delay = lag[0]["delay_s"]
    meff = P.effective_mass(P.IDEAL)
    for j, a in enumerate(G.AXES):
        p = rep["parameters"][f"meff.{a}"]
        if delay >= 0.2 * char["t95"][a]:
            assert p["reasons"] == ["LAG_DOMINATED"], (a, p)
        else:  # mu is fitted with the measured lag held: M_eff comes back (to the delay's 1 ms grid)
            assert p["status"] == G.IDENTIFIED and abs(p["meff"] / meff[j] - 1) < 0.02, (a, p)


# ------------------------------------------------------------------ fault-injection closure (design 7)
@lru_cache(maxsize=None)
def survey(cfg=None):
    inp = S.fixture_survey(cfg, seed=12)
    return inp, gate(inp)


def _d_not_moved(clean, faulted):
    """Every D the faulted run would write is the prior (abstained) or the clean run's value."""
    moved = []
    for name in G.DIRECTIONS:
        p, c = faulted["parameters"][name], clean["parameters"][name]
        if p["status"] == G.PRIOR:
            continue
        for k in ("D1", "D2"):
            if abs(p[k] / c[k] - 1) > 0.02:
                moved.append((name, k, p[k], c[k]))
    return moved


def test_closure_clean_fixture_is_characterized():
    inp, rep = survey()
    assert rep["characterized"] and not rep["reasons"], rep["reasons"]
    assert [s["signature"] for s in rep["signatures"]] == ["SCALE_UNANCHORED"]
    # recovered d1/d2 equal the fixture's D/k (fixture geometry) within 5 %
    k_fix = {"surge": 4 * 51.5 * math.cos(math.radians(45)), "sway": 4 * 51.5 * math.sin(math.radians(45)),
             "yaw": 4 * 51.5 * P.column(P.NOMINAL_THRUSTERS[1])[2]}
    for name in G.DIRECTIONS:
        axis = name[:-1]
        d = rep["parameters"][name]
        D = {"surge": P.DRAG_SURGE, "sway": P.DRAG_SWAY, "yaw": P.DRAG_YAW}[axis]
        assert abs(d["d1"] * k_fix[axis] / D[0] - 1) < 0.05 and abs(d["d2"] * k_fix[axis] / D[1] - 1) < 0.05, (name, d)


def test_closure_weak_thruster_is_attributed_not_absorbed():
    clean = survey()[1]
    rep = survey(F.FaultConfig(strengths=(1.0, 1.0, 0.6, 1.0)))[1]
    assert sigs(rep, "THRUST_OR_HULL_ASYM")
    assert not _d_not_moved(clean, rep)
    assert not rep["characterized"] and "NOT_CONVERGED" in rep["parameters"]["surge+"]["reasons"]


def test_closure_current_is_environment_not_d():
    clean = survey()[1]
    rep = survey(F.FaultConfig(current=(0.1, 0.15)))[1]
    env = sigs(rep, "ENVIRONMENT_DRIFT")
    assert env and not sigs(clean, "ENVIRONMENT_DRIFT")
    if env[0]["separated"]:
        assert np.allclose(env[0]["current_world"], (0.1, 0.15), atol=0.02)
    assert not _d_not_moved(clean, rep)
    assert all(status(rep, n) == G.IDENTIFIED for n in ("surge+", "surge-", "sway+", "sway-"))


def test_closure_gyro_bias_is_reported_and_cancels():
    clean = survey()[1]
    rep = survey(F.FaultConfig(body_bias=(0.0, 0.0, 0.02)))[1]
    g = sigs(rep, "GYRO_BIAS")
    assert g and abs(g[0]["bias_r"] - 0.02) < 0.002 and not sigs(clean, "GYRO_BIAS")
    assert not _d_not_moved(clean, rep) and rep["characterized"]
