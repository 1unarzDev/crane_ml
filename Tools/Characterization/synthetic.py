"""Synthetic gate inputs for the tests: an analytic survey (exact steady responses plus seeded
noise) and a fixture survey (plant.VesselModel / faults.FaultedVessel driven like the offline
characterization). Both return {yaml, provenance, run, snapshot} dicts in the gate's input contract.
"""
from __future__ import annotations

import copy
import math

import numpy as np

import faults as F
import gate as G
import plant as P

SHA = "64d18b203e3062aa33734951336cc678ec1045ca"  # mhseals_testing HEAD read for the design


def hardware(matrix=None, reversed_=None, epoch=0):
    return {"thruster_matrix": matrix or copy.deepcopy(G.DEFAULT_MATRIX),
            "reversed": reversed_ or [False, False, False, False], "hardware_epoch": epoch,
            "channel_map": [1, 2, 3, 4], "pwm_channels": [0, 1, 2, 3], "reverse_thrust_ratio": 1.0}


def snapshot(hw):
    return {"schema": "mhseals.config_version/1", "subsystem": "hardware", "sha": G._canonical(hw)[:12],
            "value": {"overrides": {}, "effective": hw}, "meta": {}}


def provenance(hw, battery_v=16.1, muted=False, conditions=None):
    dirs = {}
    for i, name in enumerate(G.DIRECTIONS):
        dirs[name] = {"fit": {"id": "f20261025T120000", "time": "2026-10-25T12:00:00+00:00",
                              "session": "s0001", "run": "r0001", "config": "c0007"},
                      "hardware": {"hash": G.dependency_hash(hw, G.DEP_INVALID), "epoch": hw.get("hardware_epoch", 0),
                                   "verify": G.dependency_hash(hw, G.DEP_VERIFY)},
                      "gate": {"muted": muted}, "battery_v": battery_v, "dead": False}
        if conditions:
            dirs[name]["conditions"] = conditions
    return {"schema": "mhseals.calibration/1", "software": {"mhseals_testing": SHA}, "directions": dirs}


GOOD_CONDITIONS = {"verdict": "good", "reason": "", "ceiling": 0.95, "drift_trust": 0.9, "state": "steady",
                   "drift": [0.0, 0.0], "se": [0.005, 0.005]}


def ideal_curves(profile=None, hw=None):
    """d = D/k for the base profile through the mapping of design section 3."""
    profile = profile or P.IDEAL
    gains = G.axis_gains(profile, hw)
    out = {}
    for name in G.DIRECTIONS:
        axis, suf = name[:-1], ("Pos" if name.endswith("+") else "Neg")
        k = gains[name]["k"]
        out[name] = {"d1": profile[axis]["d1" + suf] / k, "d2": profile[axis]["d2" + suf] / k}
    return out


def ideal_mu(profile=None, hw=None):
    profile = profile or P.IDEAL
    gains = G.axis_gains(profile, hw)
    meff = P.effective_mass(profile)
    return {a: meff[j] / gains[a + "+"]["k"] for j, a in enumerate(G.AXES)}


def analytic_inputs(curves=None, amplitudes=(0.75, 0.5, 0.25), noise=0.01, rate=25.0, seed=1,
                    settle=6.0, hold=3.0, current=(0.0, 0.0), gyro_bias=0.0, mu=None, speed_law=None,
                    exact_means=False, hold_noise=None):
    """Exact steady responses v(e) of the curves (or of ``speed_law(name, e)``) plus white noise.

    exact_means: the noise is demeaned inside the gate's baseline and hold windows, so every
    response equals the curve exactly (a noise-free fit with noisy samples).
    hold_noise(rng, n): replaces the main channel's noise inside the hold window (e.g. AR noise)."""
    rng = np.random.default_rng(seed)
    hw = hardware()
    curves = copy.deepcopy(curves or ideal_curves(hw=hw))
    mu = mu or ideal_mu(hw=hw)
    dt = 1.0 / rate
    t = 0.0
    neutral_len = 25.0  # above conditions.yaml gyro_bias.min_s (20 s)
    tn = np.arange(0.0, neutral_len, dt)
    neutral = [{"start": 0.0, "end": float(tn[-1]), "samples": {
        "t": tn.tolist(), "vx": (current[0] + rng.normal(0, noise, len(tn))).tolist(),
        "vy": (current[1] + rng.normal(0, noise, len(tn))).tolist(),
        "wz": (gyro_bias + rng.normal(0, noise, len(tn))).tolist(), "yaw": np.zeros(len(tn)).tolist()}}]
    t = neutral_len
    holds = []
    n_holds = len(amplitudes) * len(G.DIRECTIONS)
    for li, amp in enumerate(amplitudes):
        for di, name in enumerate(G.DIRECTIONS):
            j, s = G.parse_direction(name)
            psi = 2 * math.pi * (li * len(G.DIRECTIONS) + di) / n_holds
            cb = (math.cos(psi) * current[0] + math.sin(psi) * current[1],
                  -math.sin(psi) * current[0] + math.cos(psi) * current[1])
            v = speed_law(name, amp) if speed_law else P.steady_speed(curves[name]["d1"], curves[name]["d2"], amp * amp)
            ts = np.arange(t, t + settle + hold, dt)
            start, end = t + settle, float(ts[-1])
            on = ts >= start
            chans = {"vx": np.full(len(ts), cb[0]), "vy": np.full(len(ts), cb[1]), "wz": np.full(len(ts), gyro_bias)}
            chans[G.CHANNELS[j]] = chans[G.CHANNELS[j]] + np.where(on, s * v, 0.0)
            samples = {"t": ts.tolist(), "yaw": np.full(len(ts), psi).tolist()}
            win = (ts >= start + 1.0) & (ts <= end)
            base = (ts >= start - settle + min(2.0, 0.5 * settle)) & (ts < start)
            for c in G.CHANNELS:
                nz = rng.normal(0, noise, len(ts))
                if hold_noise is not None and c == G.CHANNELS[j]:
                    nz[win] = hold_noise(rng, int(win.sum()))
                if exact_means:
                    nz[win] -= nz[win].mean()
                    nz[base] -= nz[base].mean()
                samples[c] = (chans[c] + nz).tolist()
            cmd = [0.0, 0.0, 0.0]
            cmd[j] = s * amp
            holds.append({"command": cmd, "start": start, "end": end, "label": name, "samples": samples})
            t = end + dt
    directions = {n: {"column": [0.0, 0.0], "response": [0.0, 0.0], "low_response": [0.0, 0.0],
                      "converged": True, "low_ok": True} for n in G.DIRECTIONS}
    hi = max(amplitudes)
    t95 = {a: G.t95_closed(mu[a], hi * hi, curves[a + "+"]["d1"], curves[a + "+"]["d2"]) for a in G.AXES}
    vmax = {a: max(P.steady_speed(curves[a + s]["d1"], curves[a + s]["d2"], hi * hi) for s in "+-") for a in G.AXES}
    char = {"version": 2, "source": "mhseals_testing session", "identified_at": "2026-10-25T12:30:00+00:00",
            "sample_count": 1000, "speed_curves": curves, "t95": t95, "wave_floor": [noise, noise, noise],
            "motion_limits": {"velocity": vmax, "measured_at_effort": [hi, hi, hi]}}
    run = {"procedure": "characterization", "verdict": "FITTED", "directions": directions, "holds": holds,
           "conditions": copy.deepcopy(GOOD_CONDITIONS), "holds_rejected": [], "neutral": neutral,
           "settings": {"transient_s": 1.0, "coast_s": 2.0, "settle_s": settle}}
    return {"yaml": {"reverse_thrust_ratio": 1.0, "characterization": char}, "provenance": provenance(hw),
            "run": run, "snapshot": snapshot(hw)}


# ------------------------------------------------------------------ fixture survey (closure)
FIXTURE_MATRIX = [[1, -1, -1], [1, 1, 1], [1, -1, 1], [1, 1, -1]]  # rows FL FR RR RL of plant.NOMINAL_THRUSTERS
SURVEY = {"settle_s": 30.0, "coast_s": 20.0, "transient_s": 4.0, "hold_s": 8.0, "neutral_s": 20.0}
TOL = G.TOLERANCE


def _speed_curve(points):
    """mhseals_testing analysis.thrust.speed_curve: e^2 = d1 v + d2 v^2, a term dropped if negative."""
    v = np.array([max(p[0], 1e-6) for p in points])
    e2 = np.array([p[1] ** 2 for p in points])
    X = np.column_stack([v, v * v])
    d = np.linalg.lstsq(X, e2, rcond=None)[0]
    if d[0] < 0 or d[1] < 0:
        basis = v if d[1] < 0 else v * v
        c = float(basis @ e2 / (basis @ basis))
        d = (c, 0.0) if d[1] < 0 else (0.0, c)
    return {"d1": float(d[0]), "d2": float(d[1])}


def fixture_survey(cfg: F.FaultConfig | None = None, amplitudes=(0.9, 0.6, 0.3), seed=11, sigma_v=0.02,
                   sigma_r=0.005):
    """Run the offline survey schedule on the fixture and build the gate inputs from it."""
    cfg = cfg or F.FaultConfig()
    S = SURVEY
    m = F.FaultedVessel(cfg) if not cfg.is_neutral else P.VesselModel()
    cx, cy = cfg.current
    m.reset(state=(0.0, 0.0, 0.0, cx, cy, 0.0))  # start at water-relative rest, heading 0
    sched = []  # (t_on, t_off, name, amp)
    t = S["neutral_s"]
    for amp in amplitudes:
        for name in G.DIRECTIONS:
            t += S["settle_s"]
            sched.append((t, t + S["hold_s"], name, amp))
            t += S["hold_s"]
    t_end = t + S["settle_s"]
    dt = m.dt

    def cmd_fn(k, tt, mod):
        tt = k * dt
        for on, off, name, amp in sched:
            if on <= tt < off:
                j, s = G.parse_direction(name)
                return [row[j] * s * amp for row in FIXTURE_MATRIX]
        return [0.0, 0.0, 0.0, 0.0]
    log = m.run(int(round(t_end / dt)), cmd_fn)
    obs = F.make_observations(log, cfg, seed, sigma_v=sigma_v, sigma_r=sigma_r)
    ts, yaw = obs["t"], np.asarray(log.psi)
    data = {"vx": obs["u"], "vy": obs["v"], "wz": obs["r"]}

    def cut(t0, t1):
        sel = (ts >= t0) & (ts <= t1)
        out = {"t": ts[sel].tolist(), "yaw": yaw[sel].tolist()}
        out.update({c: data[c][sel].tolist() for c in G.CHANNELS})
        return out

    holds, deltas = [], {}
    for on, off, name, amp in sched:
        j, s = G.parse_direction(name)
        cmd = [0.0, 0.0, 0.0]
        cmd[j] = s * amp
        holds.append({"command": cmd, "start": on, "end": off, "label": name,
                      "samples": cut(on - S["settle_s"], off)})
        base = (ts >= on - S["settle_s"] + S["coast_s"]) & (ts < on)
        win = (ts >= on + S["transient_s"]) & (ts <= off)
        deltas[(name, amp)] = np.array([data[c][win].mean() - data[c][base].mean() for c in G.CHANNELS])
    hi, lo = max(amplitudes), min(amplitudes)
    curves, directions = {}, {}
    for name in G.DIRECTIONS:
        j, s = G.parse_direction(name)
        curves[name] = _speed_curve([(s * deltas[(name, a)][j], a) for a in amplitudes])
        cross = [k for k in range(3) if k != j]

        def ratios(dv):
            main = max(s * dv[j], 1e-6)
            return [float(dv[k] / main) for k in cross]
        r_hi, r_lo = ratios(deltas[(name, hi)]), ratios(deltas[(name, lo)])
        tol = TOL[name[:-1]]
        directions[name] = {"column": [0.0, 0.0], "response": r_hi, "low_response": r_lo,
                            "converged": all(abs(r) <= tl for r, tl in zip(r_hi, tol)),
                            "low_ok": all(abs(r) <= tl for r, tl in zip(r_lo, tol))}
    t95 = {}
    for axis in G.AXES:
        j = G.AXES.index(axis)
        on, off = next((a, b) for a, b, n, amp in sched if n == axis + "+" and amp == hi)
        before = (ts >= on - S["settle_s"] + S["coast_s"]) & (ts < on)
        during = (ts >= on) & (ts <= off)
        y, tt = data[G.CHANNELS[j]][during], ts[during]
        base = data[G.CHANNELS[j]][before].mean()
        steady = y[-len(y) // 3:].mean()
        frac = (y - base) / (steady - base)
        t95[axis] = float(tt[np.argmax(frac >= 0.95)] - on)
    neutral_res = {c: [] for c in G.CHANNELS}
    for on, off, name, amp in sched:
        base = (ts >= on - S["settle_s"] + S["coast_s"]) & (ts < on)
        for c in G.CHANNELS:
            neutral_res[c].extend((data[c][base] - data[c][base].mean()).tolist())
    wave_floor = [float(np.std(neutral_res[c], ddof=1)) for c in ("wz", "vx", "vy")]
    vel = {axis: float(max(abs(deltas[(n, hi)][G.AXES.index(axis)]) for n in (axis + "+", axis + "-")))
           for axis in G.AXES}
    char = {"version": 2, "source": "mhseals_testing session", "identified_at": "2026-10-25T12:30:00+00:00",
            "sample_count": int(len(ts)), "speed_curves": curves, "t95": t95, "wave_floor": wave_floor,
            "motion_limits": {"velocity": vel, "measured_at_effort": [hi, hi, hi]}}
    hw = hardware(matrix=copy.deepcopy(FIXTURE_MATRIX))
    neutral = [{"start": 0.0, "end": S["neutral_s"], "samples": cut(0.0, S["neutral_s"])}]
    run = {"procedure": "characterization", "verdict": "FITTED", "directions": directions, "holds": holds,
           "conditions": copy.deepcopy(GOOD_CONDITIONS), "holds_rejected": [], "neutral": neutral,
           "settings": {"transient_s": S["transient_s"], "coast_s": S["coast_s"], "settle_s": S["settle_s"]}}
    return {"yaml": {"characterization": char}, "provenance": provenance(hw), "run": run, "snapshot": snapshot(hw)}
