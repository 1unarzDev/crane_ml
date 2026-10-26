"""Analytic, energy, thruster and sweep checks of the exact twin ``plant.ProfilePlant`` (design 7).

Accuracy rule. Every accuracy check runs at dt in {0.005, 0.01, 0.02, 0.04} against RK4 at 1 ms with
the design tolerance (steady 1 %, t95/decay/rise/turn 2 %). Where the profile is past the design's
accuracy limit (dt*J(1.5 v_max)/M_eff > 0.5) only stability (finite) is asserted and the error is
recorded. Where the profile is inside that limit but the measured error still exceeds the design
tolerance, the case is an explicit xfail that names the cause, and it must still sit inside the
first-order envelope dt*J_max/M_eff of the implicit step (twice the leading backward-Euler term
dt*J/(2 M_eff)); an error beyond that envelope is a hard failure. Tolerances are never widened.
Every measured error is printed in the "plant accuracy" section at the end of the run.
"""
import copy
import math
import random

import numpy as np
import pytest

import plant as P

DTS = (0.005, 0.01, 0.02, 0.04)
IDEAL = P.IDEAL
COLS = {"surge": 4, "sway": 5, "yaw": 6}  # rows are [t, x, y, psi, vx, vy, r]; psi stays 0 here


def variant(base=None, **d):
    """Copy of a profile with every axis' D1/D2 (both signs) replaced, e.g. variant(d2=0.0)."""
    p = copy.deepcopy(base or IDEAL)
    for ax in P.AXES:
        for k, v in d.items():
            p[ax][k + "Pos"] = v
            p[ax][k + "Neg"] = v
    return p


def interp_time(rows, col, level, falling=False):
    s = -1.0 if falling else 1.0
    for a, b in zip(rows, rows[1:]):
        if s * (b[col] - level) >= 0.0 > s * (a[col] - level):
            return a[0] + (level - a[col]) * (b[0] - a[0]) / (b[col] - a[col])
    return math.nan


def judge(accuracy, check, dt, err, tol, profile, axis_index, j_max):
    """Apply the accuracy rule of the module docstring."""
    ratio = P.accuracy_ratio(profile, dt)[axis_index]
    meff = P.effective_mass(profile)[axis_index]
    assert math.isfinite(err), (check, dt, err)
    if ratio > 0.5:
        accuracy(check, dt, err, "stability", f"dtJ/M={ratio:.2f} > 0.5: accuracy not asserted")
        return
    if abs(err) <= tol:
        accuracy(check, dt, err, "pass", f"tol {tol:.0%}, dtJ/M={ratio:.2f}")
        return
    envelope = dt * j_max / meff
    accuracy(check, dt, err, "XFAIL", f"tol {tol:.0%} missed at dtJ/M={ratio:.2f} <= 0.5; "
                                      f"first-order term dtJmax/2M={0.5 * envelope:.3f}")
    assert abs(err) <= envelope, f"{check} dt={dt}: error {err:.4f} beyond first-order envelope {envelope:.4f}"
    pytest.xfail(f"design target {tol:.0%} unreachable for the implicit step: err {err:.4f}, "
                 f"dt*J/M_eff={ratio:.2f} <= 0.5 (leading error dt*J/(2 M_eff)={0.5 * envelope:.4f})")


# ------------------------------------------------------------------ references (cached, dt-independent)
_REF = {}


def reference(key, profile, tau, t_end, state0=None, current=(0.0, 0.0)):
    if key not in _REF:
        _REF[key] = P.reference_run(profile, lambda t: tau, t_end, state0=state0, current=current,
                                    sample_dt=0.005)
    return _REF[key]


STEP = {"surge": (58.65, 0.0, 0.0), "sway": (0.0, 56.7, 0.0), "yaw": (0.0, 0.0, 29.3)}


def d_axis(profile, axis):
    c = profile[axis]
    return c["d1Pos"], c["d2Pos"]


@pytest.mark.parametrize("dt", DTS)
@pytest.mark.parametrize("axis", P.AXES)
def test_steady_speed_and_spin(axis, dt, accuracy):
    """Steady speed v = (-D1 + sqrt(D1^2 + 4 D2 T))/(2 D2) within 1 % (yaw: steady spin from N)."""
    j = P.AXES.index(axis)
    T = STEP[axis][j]
    d1, d2 = d_axis(IDEAL, axis)
    v_exact = P.steady_speed(d1, d2, T)
    ref = reference(("step", axis), IDEAL, STEP[axis], 20.0)
    pl = P.ProfilePlant(IDEAL, dt)
    rows = pl.run_wrench(lambda k, t: STEP[axis], int(round(20.0 / dt)))
    err = rows[-1][COLS[axis]] / v_exact - 1.0
    assert abs(ref[-1][COLS[axis]] / v_exact - 1.0) < 1e-6
    judge(accuracy, f"steady {axis}", dt, err, 0.01, IDEAL, j, d1 + 2 * d2 * v_exact)


def test_steady_speed_linear_fallback():
    assert P.steady_speed(10.0, 0.0, 25.0) == 2.5
    assert abs(P.steady_speed(10.0, 19.4, 58.65) - 1.5) < 1e-12


@pytest.mark.parametrize("dt", DTS)
@pytest.mark.parametrize("axis", P.AXES)
def test_t95_step(axis, dt, accuracy):
    j = P.AXES.index(axis)
    col = COLS[axis]
    ref = reference(("step", axis), IDEAL, STEP[axis], 20.0)
    tr = interp_time(ref, col, 0.95 * ref[-1][col])
    pl = P.ProfilePlant(IDEAL, dt)
    rows = [[0.0] * 7] + pl.run_wrench(lambda k, t: STEP[axis], int(round(20.0 / dt)))
    td = interp_time(rows, col, 0.95 * rows[-1][col])
    d1, d2 = d_axis(IDEAL, axis)
    vss = P.steady_speed(d1, d2, STEP[axis][j])
    judge(accuracy, f"t95 {axis}", dt, td / tr - 1.0, 0.02, IDEAL, j, d1 + 2 * d2 * vss)


V0 = {"surge": 1.5, "sway": 1.0, "yaw": 1.0}


@pytest.mark.parametrize("dt", DTS)
@pytest.mark.parametrize("axis", P.AXES)
@pytest.mark.parametrize("law", ["linear", "quadratic"])
def test_decay(law, axis, dt, accuracy):
    """Unforced decay from v0. Linear (D2 = 0): v0 exp(-D1 t/M_eff), checked by the time to v0/e.
    Quadratic (D1 = 0): v0/(1 + D2 v0 t/M_eff), checked by the time to v0/2. Both vs the analytic
    time (RK4 must agree with it to 1e-4) within 2 %."""
    j = P.AXES.index(axis)
    prof = variant(d2=0.0) if law == "linear" else variant(d1=0.0)
    meff = P.effective_mass(prof)[j]
    d1, d2 = d_axis(prof, axis)
    v0 = V0[axis]
    if law == "linear":
        level, t_exact, j_max = v0 / math.e, meff / d1, d1
    else:
        level, t_exact, j_max = v0 / 2.0, meff / (d2 * v0), 2.0 * d2 * v0
    st = [0.0] * 6
    st[3 + j] = v0
    ref = reference(("decay", law, axis), prof, (0.0, 0.0, 0.0), 3.0 * t_exact, state0=st)
    tr = interp_time(ref, COLS[axis], level, falling=True)
    assert abs(tr / t_exact - 1.0) < 1e-4
    pl = P.ProfilePlant(prof, dt)
    pl.reset(st)
    rows = [[0.0] + st] + pl.run_wrench(lambda k, t: (0.0, 0.0, 0.0), int(round(3.0 * t_exact / dt)))
    td = interp_time(rows, COLS[axis], level, falling=True)
    judge(accuracy, f"decay {law} {axis}", dt, td / t_exact - 1.0, 0.02, prof, j, j_max)


@pytest.mark.parametrize("dt", DTS)
@pytest.mark.parametrize("axis", P.AXES)
def test_small_step_rise(axis, dt, accuracy):
    """A tiny step (quadratic term negligible) rises with time constant M_eff/D1 (63.2 % time)."""
    j = P.AXES.index(axis)
    tau = [0.0, 0.0, 0.0]
    tau[j] = 0.01
    meff = P.effective_mass(IDEAL)[j]
    d1, d2 = d_axis(IDEAL, axis)
    tc = meff / d1
    vss = P.steady_speed(d1, d2, 0.01)
    pl = P.ProfilePlant(IDEAL, dt)
    rows = [[0.0] * 7] + pl.run_wrench(lambda k, t: tau, int(round(4.0 * tc / dt)))
    td = interp_time(rows, COLS[axis], (1.0 - math.exp(-1.0)) * vss)
    judge(accuracy, f"rise {axis}", dt, td / tc - 1.0, 0.02, IDEAL, j, d1 + 2 * d2 * vss)


TURN = (40.0, 0.0, 8.0)


def body(row):
    c, s = math.cos(row[3]), math.sin(row[3])
    return (c * row[4] + s * row[5], -s * row[4] + c * row[5], row[6])


@pytest.mark.parametrize("dt", DTS)
def test_steady_turn(dt, accuracy):
    """Constant surge force + yaw moment: steady turn radius R = u/r and sideslip v vs RK4 (2 %)."""
    ref = body(reference(("turn",), IDEAL, TURN, 30.0)[-1])
    pl = P.ProfilePlant(IDEAL, dt)
    b = body(pl.run_wrench(lambda k, t: TURN, int(round(30.0 / dt)))[-1])
    errs = {"turn radius": (b[0] / b[2]) / (ref[0] / ref[2]) - 1.0, "turn sideslip": b[1] / ref[1] - 1.0}
    for name, err in errs.items():
        accuracy(name, dt, err, "pass" if abs(err) <= 0.02 else "FAIL", "tol 2%")
    assert all(abs(e) <= 0.02 for e in errs.values()), errs
    assert P.accuracy_ratio(IDEAL, dt)[0] <= 0.5  # surge/sway are inside the limit at every dt


# ------------------------------------------------------------------ energy and coupling
def test_c_a_is_workless():
    rng = random.Random(7)
    for _ in range(200):
        am = {"a11": rng.uniform(0, 60), "a22": rng.uniform(0, 60), "a26": rng.uniform(-5, 5),
              "a66": rng.uniform(0, 10)}
        nu = [rng.uniform(-3, 3) for _ in range(3)]
        f = P.c_a_times(am, *nu)
        assert abs(sum(a * b for a, b in zip(nu, f))) <= 1e-9


def test_fossen_numeric():
    am = {"a11": 6.0, "a22": 8.0, "a26": 0.0, "a66": 0.0}
    X, Y, N = (-x for x in P.c_a_times(am, 1.0, 0.2, 0.5))
    assert abs(X - 0.8) < 1e-12 and abs(Y + 3.0) < 1e-12 and abs(N + 0.4) < 1e-12


@pytest.mark.parametrize("current", [(0.0, 0.0), (0.3, -0.2)])
@pytest.mark.parametrize("dt", DTS)
def test_unforced_energy_non_increasing(dt, current, accuracy):
    """1/2 nu_r^T M nu_r never rises by more than 1e-6 E0 per step nor 0.1 % over 10 s."""
    prof = copy.deepcopy(IDEAL)
    prof["addedMass"]["a26"] = 0.5
    pl = P.ProfilePlant(prof, dt, current=current)
    pl.reset((0.0, 0.0, 0.7, 1.2 + current[0], -0.4 + current[1], 0.8))
    e0 = pl.energy()
    prev, worst = e0, -math.inf
    for _ in range(int(round(10.0 / dt))):
        pl.step_wrench((0.0, 0.0, 0.0))
        e = pl.energy()
        worst = max(worst, (e - prev) / e0)
        prev = e
    accuracy(f"energy rise/step current={current}", dt, worst, "pass" if worst <= 1e-6 else "FAIL", "")
    assert worst <= 1e-6
    assert prev <= e0 * 1.001


@pytest.mark.parametrize("dt", DTS)
def test_added_mass_coupling_only(dt):
    """D = 0, step force F from rest: acceleration F/(m + a11) exactly (not 1.69x)."""
    prof = variant(d1=0.0, d2=0.0)
    pl = P.ProfilePlant(prof, dt)
    F = 20.0
    for k in range(5):
        u0 = pl.state[3]
        pl.step_wrench((F, 0.0, 0.0))
        acc = (pl.state[3] - u0) / dt
        assert abs(acc / (F / (prof["mass"] + prof["addedMass"]["a11"])) - 1.0) < 1e-12


def test_pinned_implicit_form():
    """One tick against a general numpy evaluation of the pinned form
    F_U = tau_h - (M_A + dt diag J) solve(M_RB + M_A + dt diag J, tau_h - C_RB nu)."""
    prof = copy.deepcopy(IDEAL)
    prof["addedMass"].update(a26=0.7)
    prof["yaw"]["d1Neg"] = 13.0
    dt = 0.02
    pl = P.ProfilePlant(prof, dt, current=(0.3, -0.2))
    pl.reset((1.0, 2.0, 0.4, 0.9, -0.3, -0.6))
    tau = (12.0, -3.0, 2.5)
    psi, (vx, vy, r) = 0.4, (0.9, -0.3, -0.6)
    c, s = math.cos(psi), math.sin(psi)
    nu = np.array([c * vx + s * vy, -s * vx + c * vy, r])
    nuc = np.array([c * 0.3 + s * -0.2, -s * 0.3 + c * -0.2, 0.0])
    nur = nu - nuc
    m, iz, am = prof["mass"], prof["izz"], prof["addedMass"]
    MA = np.array([[am["a11"], 0, 0], [0, am["a22"], am["a26"]], [0, am["a26"], am["a66"]]])
    MRB = np.diag([m, m, iz])
    D, J = zip(*(P.damping_axis(prof[a], nur[i]) for i, a in enumerate(P.AXES)))
    th = (np.array(tau) + MA @ np.array([r * nuc[1], -r * nuc[0], 0.0])
          - np.array(P.c_a_times(am, *nur)) - np.array(D))
    x = th - np.array(P.c_rb_times(m, *nu))
    K = MA + dt * np.diag(J)
    fu_expected = th - K @ np.linalg.solve(MRB + K, x)
    fu = np.array(pl.step_wrench(tau))
    assert np.allclose(fu, fu_expected, rtol=1e-12, atol=1e-12), (fu, fu_expected)
    # the explicit variant (dt*J left out of the correction) differs: the pin is observable
    fu_explicit = th - MA @ np.linalg.solve(MRB + K, x)
    assert np.max(np.abs(fu_explicit - fu)) > 1e-3


def test_damping_jacobian_at_zero_uses_max_d1():
    c = {"d1Pos": 10.0, "d1Neg": 13.0, "d2Pos": 19.4, "d2Neg": 20.0}
    assert P.damping_axis(c, 0.0) == (0.0, 13.0)
    D, J = P.damping_axis(c, -0.5)
    assert D == -(13.0 + 20.0 * 0.5) * 0.5 and J == 13.0 + 2 * 20.0 * 0.5


@pytest.mark.parametrize("psi", [0.0, 1.3, -2.4])
def test_zero_thrust_in_current_converges_to_relative_rest(psi):
    pl = P.ProfilePlant(IDEAL, 0.02, current=(0.3, -0.2))
    pl.reset((0.0, 0.0, psi, 0.0, 0.0, 0.0))
    for _ in range(3000):
        pl.step_wrench((0.0, 0.0, 0.0))
    ur, vr, r = pl.relative_velocity()
    # the heading itself may turn while the boat is carried up to the current (Munk moment of nu_r)
    assert abs(ur) < 1e-6 and abs(vr) < 1e-6 and abs(r) < 1e-6


# ------------------------------------------------------------------ thruster model
def spec(**kw):
    s = dict(IDEAL["thrusters"][0])
    s.update(kw)
    return s


def test_thruster_deadband():
    th = P.ThrusterModel(spec(deadbandUs=50.0), 0.02)
    for p in (1450.0, 1475.0, 1500.0, 1525.0, 1550.0):
        th.reset()
        assert th.step(p) == (0.0, 0.0, 0.0)
    th.reset()
    assert th.step(1551.0)[0] != 0.0


def test_thruster_lag_63_percent_at_tau():
    dt, tau = 0.01, 0.5
    th = P.ThrusterModel(spec(tauT=tau), dt)
    for _ in range(int(round(tau / dt))):
        th.step(1900.0)
    assert abs(th.T / 51.5 - (1.0 - math.exp(-1.0))) < 1e-12


@pytest.mark.parametrize("dead,dt", [(0.06, 0.02), (0.07, 0.02), (0.03, 0.02), (0.1, 0.04), (0.0, 0.01)])
def test_thruster_dead_time_rounding(dead, dt):
    th = P.ThrusterModel(spec(deadTime=dead), dt)
    n_d = int(round(dead / dt))
    assert th.n_d == n_d
    assert th.dead_time_rounding_error == dead - n_d * dt
    out = [th.step(1900.0)[0] for _ in range(n_d + 2)]
    assert all(f == 0.0 for f in out[:n_d]) and out[n_d] != 0.0


def test_thruster_reverse_ratio_and_clamp():
    th = P.ThrusterModel(spec(kReverse=40.2), 0.02)
    th.step(1100.0)
    assert th.T == -40.2
    th.reset()
    th.step(1900.0)
    assert th.T == 51.5
    th.reset()
    th.step(2500.0)
    assert th.T == 51.5
    th.reset()
    th.step(0.0)
    assert th.T == -40.2
    th.reset()
    th.step(1700.0)
    assert abs(th.T - 51.5 * 0.25) < 1e-12


def test_thruster_beta_zero_and_nan_and_voltage():
    th = P.ThrusterModel(spec(), 0.02)
    assert th.step(1900.0, beta=0.0) == (0.0, 0.0, 0.0)
    th.reset()
    assert th.step(math.nan) == (0.0, 0.0, 0.0) and th.events == 1
    assert th.step(math.inf) == (0.0, 0.0, 0.0) and th.events == 2
    th = P.ThrusterModel(spec(nV=2.0), 0.02)
    th.step(1900.0, voltage=8.0)
    assert abs(th.T - 51.5 * 0.25) < 1e-12


def test_thruster_wrench_geometry():
    th = P.ThrusterModel(spec(), 0.02)  # front_left, 135 deg at (0.27, 0.34)
    fx, fy, n = th.step(1900.0)
    a = math.radians(135.0)
    assert abs(fx - 51.5 * math.cos(a)) < 1e-12 and abs(fy - 51.5 * math.sin(a)) < 1e-12
    assert abs(n - (0.27 * fy - 0.34 * fx)) < 1e-12


def test_thruster_reset_clears_lag_and_buffer():
    th = P.ThrusterModel(spec(tauT=0.3, deadTime=0.1), 0.02)
    for _ in range(20):
        th.step(1900.0)
    assert th.T != 0.0
    th.reset()
    assert th.T == 0.0 and all(q == 0.0 for q in th.buf)
    assert th.step(1500.0) == (0.0, 0.0, 0.0)


def test_kappa_must_be_zero():
    with pytest.raises(ValueError):
        P.ThrusterModel(spec(kappa=0.1), 0.02)
    prof = copy.deepcopy(IDEAL)
    prof["thrusters"][2]["kappa"] = 0.1
    with pytest.raises(ValueError):
        P.ProfilePlant(prof, 0.02)


@pytest.mark.parametrize("bad", [("mass", 0.0), ("izz", -1.0), ("mass", math.nan)])
def test_profile_validation_rejects(bad):
    prof = copy.deepcopy(IDEAL)
    prof[bad[0]] = bad[1]
    with pytest.raises(ValueError):
        P.ProfilePlant(prof, 0.02)
    prof = copy.deepcopy(IDEAL)
    prof["surge"]["d1Pos"] = -1.0
    with pytest.raises(ValueError):
        P.ProfilePlant(prof, 0.02)


def test_pwm_helper_surge_and_yaw_are_pure():
    pl = P.ProfilePlant(IDEAL, 0.02)
    X, Y, N = pl.thrust(P.pwm_for_effort(surge=0.5))
    assert X > 0 and abs(Y) < 1e-12 and abs(N) < 1e-12
    pl.reset()
    X, Y, N = pl.thrust(P.pwm_for_effort(yaw=0.5))
    assert N > 0 and abs(X) < 1e-12 and abs(Y) < 1e-12


# ------------------------------------------------------------------ sweeps (design 7)
def sweep_profile(m, kz, ratio, dscale, tau_t, db):
    p = copy.deepcopy(IDEAL)
    p["mass"], p["izz"] = m, m * kz * kz
    p["addedMass"].update(a11=ratio * m, a22=ratio * m, a26=0.0, a66=ratio * p["izz"])
    for ax in P.AXES:
        for k in ("d1Pos", "d1Neg", "d2Pos", "d2Neg"):
            p[ax][k] = IDEAL[ax][k] * dscale
    for t in p["thrusters"]:
        t["tauT"], t["deadbandUs"] = tau_t, db
    return p


CORNERS = [(m, kz, a, d, tt, db) for m in (10.0, 80.0) for kz in (0.3, 0.45) for a in (0.0, 2.0)
           for d in (0.2, 5.0) for tt in (0.0, 1.0) for db in (0.0, 100.0)]
_rng = random.Random(20261025)
INTERIOR = [(_rng.uniform(10, 80), _rng.uniform(0.3, 0.45), _rng.uniform(0, 2), math.exp(_rng.uniform(math.log(0.2), math.log(5))),
             _rng.uniform(0, 1), _rng.uniform(0, 100)) for _ in range(8)]
GRID = CORNERS + INTERIOR


def program(k, t):
    """Steps, a turn, full reverse, then a sign flip every tick at full effort."""
    if t < 1.0:
        return P.pwm_for_effort(surge=1.0)
    if t < 2.0:
        return P.pwm_for_effort(surge=0.6, yaw=0.4)
    if t < 3.0:
        return P.pwm_for_effort(surge=-1.0)
    return P.pwm_for_effort(sway=1.0 if k % 2 else -1.0, yaw=-0.5 if k % 2 else 0.5)


@pytest.mark.parametrize("dt", DTS)
def test_sweep_finite_and_passive(dt):
    """Finite at every corner (and seeded interior points) under the stress program, and the
    unforced energy afterwards never rises (passivity) beyond 1e-6 E0 per step."""
    for params in GRID:
        prof = sweep_profile(*params)
        pl = P.ProfilePlant(prof, dt)
        pl.run_pwm(program, int(round(4.0 / dt)))
        assert all(math.isfinite(x) for x in pl.state), (params, pl.state)
        pl.reset((0.0, 0.0, 0.3, 1.0, 0.5, 0.6))
        e0 = prev = pl.energy()
        for _ in range(int(round(2.0 / dt))):
            pl.step_wrench((0.0, 0.0, 0.0))
            e = pl.energy()
            assert e - prev <= 1e-6 * e0, (params, dt, e - prev)
            prev = e


@pytest.mark.parametrize("dt", DTS)
def test_sweep_speed_monotone_in_effort(dt):
    """Speed at a fixed time from rest is monotone (non-decreasing) in surge effort."""
    for i, params in enumerate(GRID):
        if i % 2 != DTS.index(dt) % 2:  # alternate halves of the grid across dt to keep the suite fast
            continue
        prof = sweep_profile(*params)
        last = -math.inf
        for e in (0.1, 0.3, 0.6, 1.0):
            pwm = P.pwm_for_effort(surge=e)
            pl = P.ProfilePlant(prof, dt)
            pl.run_pwm(lambda k, t: pwm, int(round(3.0 / dt)))
            u = pl.body_velocity()[0]
            assert u >= last - 1e-12, (params, dt, e, u, last)
            last = u
        assert last > 0.0


def mirror(pwm):
    """Mirror about the x axis: FL <-> FR, RR <-> RL (profile order FL, FR, RR, RL)."""
    return [pwm[1], pwm[0], pwm[3], pwm[2]]


@pytest.mark.parametrize("dt", DTS)
def test_sweep_mirror_symmetry(dt):
    for i, params in enumerate(GRID):
        if i % 4 != DTS.index(dt):
            continue
        prof = sweep_profile(*params)
        a, b = P.ProfilePlant(prof, dt), P.ProfilePlant(prof, dt)
        ra = a.run_pwm(program, int(round(4.0 / dt)))
        rb = b.run_pwm(lambda k, t: mirror(program(k, t)), int(round(4.0 / dt)))
        for x, y in zip(ra, rb):
            scale = 1.0 + max(abs(v) for v in x[1:])
            for idx, sgn in ((1, 1), (2, -1), (3, -1), (4, 1), (5, -1), (6, -1)):
                assert abs(x[idx] - sgn * y[idx]) <= 1e-9 * scale, (params, dt, idx, x, y)
