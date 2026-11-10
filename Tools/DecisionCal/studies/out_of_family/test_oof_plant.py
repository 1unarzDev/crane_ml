"""Mechanism tests: off => bit-identical to plant.Plant; on => the intended effect at the intended magnitude;
batch = singles. Fast (< 1 min)."""
import numpy as np
import pytest

import hidden
import mission
import probes
from oof_plant import (HI, MECHS, CommandLink, MechPlant, MechSpec, advance_thrust, at_level, latency_stats,
                       use_mech)
from plant import B, DT, Plant, PlantSpec


def _quiet(n=1, **kw):
    """A clean in-family boat: no noise, no environment, unit thrust map."""
    base = dict(mass=30.0, izz=4.1, a11=1.5, a22=15.0, a66=1.23, d1=[10, 15, 10], d2=[19.4, 41.7, 19.3],
                k_fwd=51.5, k_rev=51.5)
    base.update(kw)
    return PlantSpec.build(n, **base)


def _cmds(T, n, seed=0):
    rng = np.random.default_rng(seed)
    q = rng.uniform(-1, 1, (T // 10 + 1, n, 4))
    return np.repeat(q, 10, 0)[:T]           # held 0.2 s (an even number of ticks)


def _fly(plant, Q):
    out = []
    for k in range(len(Q)):
        plant.step(Q[k])
        pose, vel, gyro = plant.measure()
        out.append(np.concatenate([plant.st, pose, vel, gyro[:, None], plant.thr.T, plant.q_applied], 1))
    return np.asarray(out)


# ---- off => identical -----------------------------------------------------------------------

def test_off_is_bit_identical_to_plant():
    spec = hidden.sample_hidden(3, 4142, 'timing')
    seeds = np.array([11, 12, 13], np.uint64)
    Q = _cmds(600, 3)
    a = _fly(Plant(spec, seeds, t_max=13), Q)
    b = _fly(MechPlant(spec, MechSpec.off(3), seeds, t_max=13), Q)
    assert np.array_equal(a, b)


def test_off_rows_identical_inside_a_mixed_batch():
    spec = hidden.sample_hidden(2, 4142, 'timing')
    seeds = np.array([5, 6], np.uint64)
    Q = _cmds(500, 2)
    mech = MechSpec.concat([MechSpec.off(1), at_level(1, 1.0, 'all', 1)])
    a = _fly(Plant(spec.take([0]), seeds[:1], t_max=11), Q[:, :1])
    b = _fly(MechPlant(spec, mech, seeds, t_max=11), Q)
    assert np.array_equal(a[:, 0], b[:, 0])
    assert not np.array_equal(b[:, 1], _fly(Plant(spec.take([1]), seeds[1:], t_max=11), Q[:, 1:])[:, 0])


@pytest.mark.parametrize('mech', MECHS)
def test_batch_equals_singles(mech):
    spec = hidden.sample_hidden(3, 4142, 'timing')
    seeds = np.array([21, 22, 23], np.uint64)
    m = at_level(3, 1.0, [mech], 3)
    Q = _cmds(400, 3)
    allb = _fly(MechPlant(spec, m, seeds, t_max=9), Q)
    for i in range(3):
        one = _fly(MechPlant(spec.take([i]), m.take([i]), seeds[i:i + 1], t_max=9), Q[:, i:i + 1])
        assert np.array_equal(allb[:, i], one[:, 0]), mech


def test_mission_through_use_mech_off_matches_plain_mission():
    spec = hidden.sample_hidden(1, 4142, 'timing').repeat(2)
    seeds = np.array([1, 2], np.uint64)
    a, _ = mission.run(spec, [0, 3], seeds, t_max=20.0)
    with use_mech(MechSpec.off(1)):
        b, _ = mission.run(spec, [0, 3], seeds, t_max=20.0)
    assert mission.Plant is Plant and probes.Plant is Plant
    for k in a:
        assert np.array_equal(np.nan_to_num(a[k], nan=-1), np.nan_to_num(b[k], nan=-1)), k


# ---- link -------------------------------------------------------------------------------------

def test_ideal_link_equals_dead_time_for_held_commands():
    spec = _quiet(1, dead_time=0.14)
    m = MechSpec.build(1, jitter_mean=1e-9)                # on, but jitter rounds to 0 ticks
    Q = _cmds(400, 1)
    a = _fly(Plant(spec, [3], t_max=9), Q)
    b = _fly(MechPlant(spec, m, [3], t_max=9), Q)
    assert np.array_equal(a, b)


def test_link_jitter_and_drops_magnitudes():
    n = 40
    seeds = np.arange(n, dtype=np.uint64) + 100
    m = MechSpec.build(n, jitter_mean=0.03, drop_p=0.05)
    link = CommandLink(n, np.full(n, 5), m, seeds, record=True)
    for k in range(6000):
        link.push(np.zeros((n, 4)), k)
    assert abs(np.mean(link.log_dropped) - 0.05) < 0.01
    idx = np.asarray(link.log_idx)
    assert (np.diff(idx, axis=0) >= 0).all()                 # never goes back to an older frame
    st = latency_stats(_quiet(1, dead_time=0.10), m.take([0]), seeds[:8])
    # mean extra ~ E[round(Exp(1.5 ticks))] plus hold after drops: 0.02-0.06 s above the dead time
    assert 0.02 < st['cmd_mean'] - 0.10 < 0.06, st
    assert st['cmd_p90'] > st['cmd_mean']


def test_link_outages_cover_expected_fraction():
    n = 30
    seeds = np.arange(n, dtype=np.uint64) + 7
    m = MechSpec.build(n, outage_rate=0.1, outage_mean=0.3)
    link = CommandLink(n, np.zeros(n, int), m, seeds, record=True)
    for k in range(30000):
        link.push(np.zeros((n, 4)), k)
    frac = np.mean(link.log_dropped)
    expect = 0.1 * 0.3 / (1 + 0.1 * 0.3)
    assert abs(frac - expect) < 0.4 * expect, (frac, expect)


def test_ideal_link_latency_stats_equal_spec():
    spec = _quiet(1, dead_time=0.13, odom_latency=0.07)
    st = latency_stats(spec, MechSpec.off(1), [1, 2])
    assert abs(st['L_mean'] - 0.20) < 1e-12 and abs(st['L_p90'] - 0.20) < 1e-12


# ---- prop-hull ----------------------------------------------------------------------------------

def test_advance_thrust_magnitudes():
    k = 51.5
    assert np.isclose(advance_thrust(k, k, 1.0, 0.25), 0.75 * k)          # full thrust, 1 m/s: 25% loss
    assert np.isclose(advance_thrust(0.25 * k, k, 1.0, 0.25), 0.25 * k * 0.5)   # quarter thrust: J doubles
    assert np.isclose(advance_thrust(-k, k, 1.0, 0.25), -0.75 * k)        # reverse, moving along its thrust
    assert advance_thrust(k, k, -1.0, 0.25) > k                           # moving against the thrust: gain
    assert advance_thrust(0.0, k, 1.0, 0.25) == 0.0


def test_prophull_lift_moment_value():
    spec = _quiet(1)
    m = MechSpec.build(1, n_uv=3.0, y_uv=6.0)
    p = MechPlant(spec, m, [1], state0=[[0, 0, 0, 1.0, 0.2, 0]], t_max=2)
    p.step(np.zeros(4))
    q = Plant(spec, [1], state0=[[0, 0, 0, 1.0, 0.2, 0]], t_max=2)
    q.step(np.zeros(4))
    d = p.wrench[0] - q.wrench[0]
    assert np.allclose(d, [0.0, -6.0 * 1.0 * 0.2, 3.0 * 1.0 * 0.2])


def test_prophull_reduces_top_speed():
    spec = _quiet(1)
    m = MechSpec.build(1, adv_loss=0.25)
    q = np.array([-1.0, -1.0, 1.0, 1.0]) * np.sign(B[0])                  # full ahead
    a, b = Plant(spec, [1], t_max=31), MechPlant(spec, m, [1], t_max=31)
    for _ in range(1500):
        a.step(q)
        b.step(q)
    assert b.st[0, 3] < 0.95 * a.st[0, 3]


# ---- battery ------------------------------------------------------------------------------------

def test_battery_sag_full_load():
    from nlc.sim.vessel import wind_wrench
    spec = _quiet(1)
    m = MechSpec.build(1, sag_full=0.12, sag_tau=20.0)
    p = MechPlant(spec, m, [1], t_max=200)
    for _ in range(int(150 / DT)):
        p.step(np.ones(4))
    assert np.isclose(p.sag[0], 0.12, rtol=0.01)              # all four at full for >> sag_tau
    st0 = p.st.copy()
    p.step(np.ones(4))
    g = (1 - p.sag[0]) ** 1.25
    assert 0.84 < g < 0.86
    expect = (p.thr.T * g) @ B.T + wind_wrench(p.P, st0, np.zeros((1, 2)), 1)
    assert np.allclose(p.wrench, expect)


def test_battery_sag_partial_load_is_smaller():
    spec = _quiet(1)
    m = MechSpec.build(1, sag_full=0.12, sag_tau=20.0)
    p = MechPlant(spec, m, [1], t_max=200)
    for _ in range(int(150 / DT)):
        p.step(np.array([1.0, 0, 0, 0]))                      # one thruster at full
    assert np.isclose(p.sag[0], 0.03, rtol=0.02)


# ---- waves --------------------------------------------------------------------------------------

def test_waves_force_and_wobble_amplitude():
    spec = _quiet(1)
    m = MechSpec.build(1, wave_Y=12.0, wave_N=1.5, head_wobble=0.035, wave_period=2.0)
    p, r = MechPlant(spec, m, [9], t_max=61), Plant(spec, [9], t_max=61)
    dY, dN, wob = [], [], []
    for _ in range(3000):
        p.step(np.zeros(4))
        r.st = p.st.copy()
        r.step(np.zeros(4))
        dY.append(p.wrench[0, 1] - r.wrench[0, 1])
        dN.append(p.wrench[0, 2] - r.wrench[0, 2])
        wob.append(p.measure()[0][0, 2] - p.st[0, 2])
    assert abs(np.std(dY) - 12.0 / np.sqrt(2)) < 0.15 * 12.0 / np.sqrt(2)
    assert abs(np.std(dN) - 1.5 / np.sqrt(2)) < 0.15 * 1.5 / np.sqrt(2)
    assert abs(np.std(wob) - 0.035 / np.sqrt(2)) < 0.2 * 0.035 / np.sqrt(2)


# ---- thruster lag ---------------------------------------------------------------------------------

def _rise_time(spec, mech, q, frac=0.632):
    p = MechPlant(spec, mech, [1], t_max=10)
    target = None
    for k in range(400):
        p.step(np.full(4, q))
        if target is None:
            target = p.thrust_of(np.full((1, 4), q))[0, 0]
        if abs(p.thr.T[0, 0]) >= frac * abs(target):
            return (k + 1) * DT
    return np.inf


def test_thrlag_level_and_direction_dependence():
    spec = _quiet(1, tau_up=0.06, tau_down=0.06)
    base_full = _rise_time(spec, MechSpec.off(1), 1.0)
    m = MechSpec.build(1, lag_low_ratio=3.0, lag_rev_ratio=1.6)
    full, low = _rise_time(spec, m, 1.0), _rise_time(spec, m, 0.3)
    rev = _rise_time(spec, m, -1.0)
    assert abs(base_full - 0.06) <= 0.021
    assert full > 1.4 * base_full                              # spins up through the slow low-thrust region
    assert low / _rise_time(spec, MechSpec.off(1), 0.3) > 2.0  # near zero thrust tau -> 3x
    assert rev > 1.4 * full


# ---- estimator --------------------------------------------------------------------------------

def test_estimator_holds_gps_and_lags_velocity():
    spec = _quiet(1)
    m = MechSpec.build(1, gps_period=0.2, vel_tau=0.3)
    p = MechPlant(spec, m, [4], t_max=31)
    r = Plant(spec, [4], t_max=31)
    q = np.array([-1.0, -1.0, 1.0, 1.0]) * np.sign(B[0]) * 0.6
    xs, us, ut = [], [], []
    for _ in range(500):
        p.step(q)
        r.step(q)
        pose, vel, _ = p.measure()
        xs.append(pose[0, 0])
        us.append(vel[0, 0])
        ut.append(p.st[0, 3])
    xs, us, ut = map(np.asarray, (xs, us, ut))
    assert np.array_equal(p.st, r.st)                        # dynamics untouched
    changes = np.flatnonzero(np.diff(xs) != 0)
    assert np.all(np.diff(changes) == 10)                     # position held at 5 Hz
    k = np.arange(100, 300)
    lag = np.argmin([np.abs(us[k] - ut[k - j]).mean() for j in range(40)]) * DT
    assert 0.3 <= lag <= 0.5, lag                             # tau 0.3 + mean ZOH age ~0.1 s
    assert abs(us[-1] - ut[-1]) < 0.02                        # converges at steady speed


# ---- multipath ----------------------------------------------------------------------------------

def test_multipath_steps():
    spec = _quiet(30)
    seeds = np.arange(30, dtype=np.uint64) + 3
    m = MechSpec.build(30, mp_rate=1 / 60, mp_size=0.6, mp_dur=3.0)
    p = MechPlant(spec, m, seeds, t_max=600)
    off = np.array([p.mp_offset(t) for t in np.arange(0, 600, 0.5)])     # [T, N, 2]
    mag = np.hypot(off[..., 0], off[..., 1])
    frac = (mag > 1e-9).mean()
    assert 0.02 < frac < 0.09, frac                            # ~ rate x duration = 0.05
    single = mag[(mag > 1e-9)]
    assert np.isclose(np.median(single), 0.6, atol=1e-9)


def test_levels_scale_and_zero_is_off():
    z = at_level(4, 0.0, 'all', 1)
    assert not any(z.on(mm).any() for mm in MECHS)
    h = at_level(4, 1.0, 'all', 1)
    assert all(h.on(mm).all() for mm in MECHS)
    assert np.allclose(h.adv_loss, HI['prophull']['adv_loss'])
    assert np.allclose(at_level(4, 0.5, 'all', 1).sag_full, 0.06)
