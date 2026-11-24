"""Gate 1 for the aerial kernel: every backend against domains/drones/model.py's step (the port of the drones'
aerial_model.step) over scripted rotor commands, the dt order, and batch invariance of the braking scenario.

Run from the crane root: python3 -m pytest engine/tests/test_aerial.py
"""

import copy
import importlib.util

import numpy as np
import pytest

from domains.drones import braking, card, model
from engine.kernels import aerial as A

HAS = {'numba': importlib.util.find_spec('numba') is not None, 'jax': importlib.util.find_spec('jax') is not None}
GATE = 1e-10
DT = 0.004

# (card, vehicle options, pack options, start height): the x500 (no battery) with ground effect near the pad and
# ground contact, the crazyflie with its battery, the x500 on the zoh_first motor order, and an x500 variant with
# rotor inertia and quadratic body drag (both zero on the shipped cards) started fast and spinning, so the
# gyroscopic torque, the rotors' angular momentum and the quadratic drag all act.
def spinning_x500():
    doc = copy.deepcopy(card.read('x500'))
    doc['parameters']['rotor_inertia'] = {'value': 0.0, 'unit': 'kg m^2', 'status': 'VARIANT'}
    return card.override(doc, {'rotor_inertia': 4e-4, 'drag_quadratic': 0.35})


CASES = {
    'x500-ground': ('x500', {'ground_effect': True}, {'contact': True}, 0.4, None),
    'crazyflie-battery': ('crazyflie21', {}, {}, 5.0, None),
    'x500-zoh': ('x500', {}, {'motor_scheme': 'zoh_first'}, 5.0, None),
    'x500-gyro-quadratic-drag': (spinning_x500, {}, {}, 30.0, ((9.0, -4.0, 2.0), (2.5, -1.8, 3.0))),
}


def program(w_h, n, rng):
    """Rotor targets: smooth differential sweeps, then random steps held 40 steps (tau up and down both)."""
    k = np.arange(n)[:, None]
    smooth = w_h * (1.0 + 0.12 * np.sin(0.011 * k * np.arange(1, 5) + np.arange(4)))
    steps = np.repeat(rng.uniform(0.6, 1.25, (n // 40 + 1, 4)) * w_h, 40, axis=0)[:n]
    return np.where((k % 400) < 250, smooth, steps)


def reference(v, s, us, wind, contact, scheme):
    out = []
    for u in us:
        s = model.step(v, s, u, DT, wind_world=wind, contact=contact, motor_scheme=scheme)
        out.append(np.concatenate([s.p, s.v, s.q, s.omega, [s.soc, s.t], s.rotor_speed]))
    return np.array(out)


def run(backend, p, x0, us, wind):
    if backend == 'numpy':
        x, out = x0[None], []
        for u in us:
            x = A.step(x, u[None], p[None], DT, wind[None])
            out.append(x[0])
        return np.array(out)
    if backend == 'numba':
        x, out = x0.copy(), []
        for u in us:
            A.nb_step(x, u, p, DT, wind)
            out.append(x.copy())
        return np.array(out)
    from engine.jaxconf import jax, jnp
    f = jax.jit(lambda x, u: A.step(x, u, jnp.asarray(p[None]), DT, jnp.asarray(wind[None]), jnp))

    def body(x, u):
        x = f(x, u[None])
        return x, x[0]
    _, xs = jax.lax.scan(body, jnp.asarray(x0[None]), jnp.asarray(us))
    return np.asarray(xs)


@pytest.mark.parametrize('backend', ['numpy', 'numba', 'jax'])
def test_every_backend_is_model_step_over_scripted_commands(backend):
    """Gate 1: 1,200 steps (4.8 s) per case (four cases, every kernel term non-zero in at least one), every state component within 1e-10 of model.step (float64)."""
    if backend != 'numpy' and not HAS[backend]:
        pytest.skip(f'{backend} is not installed (the [fast] extra / jax)')
    rng = np.random.default_rng(7)
    for name, (ref, vopt, popt, z0, moving) in CASES.items():
        v = card.vehicle(ref() if callable(ref) else ref, **vopt)
        w_h = model.hover_rotor_speed(v)
        s0 = model.initial_state(v, p=(0.0, 0.0, z0), rotor_speed=w_h)
        if moving:
            s0.v, s0.omega = np.array(moving[0]), np.array(moving[1])
        us = program(w_h, 1200, rng)
        wind = np.array([1.5, -0.8, 0.3])
        want = reference(v, s0, us, wind, popt.get('contact', False), popt.get('motor_scheme', 'exact_stage'))
        p = A.pack(v, **popt)
        x0 = A.init(p[None], pos=s0.p, vel=s0.v, rotor_speed=w_h)[0]
        x0[A.RATE:A.RATE + 3] = s0.omega
        got = run(backend, p, x0, us, wind)
        err = float(np.abs(got - want).max())
        assert err <= GATE, (backend, name, err)


def test_padding_and_rk4_order():
    """A quad padded to six rotors is the same map; with a held command the error against dt/8 falls ~16x per
    halving (RK4 with exact motor stages)."""
    v = card.vehicle('x500')
    w_h = model.hover_rotor_speed(v)
    p4, p6 = A.pack(v), A.pack(v, rotors=6)
    u = w_h * np.array([1.08, 0.97, 1.03, 0.95])
    x4 = A.init(p4[None], pos=(0, 0, 5), rotor_speed=w_h * 0.9)
    x6 = A.init(p6[None], pos=(0, 0, 5), rotor_speed=w_h * 0.9)
    for _ in range(200):
        x4 = A.step(x4, u[None], p4[None], DT)
        x6 = A.step(x6, np.r_[u, 300.0, 300.0][None], p6[None], DT)
    assert np.array_equal(x4[0, :A.BODY + 4], x6[0, :A.BODY + 4]) and not x6[0, A.BODY + 4:].any()

    def final(dt):
        x = A.init(p4[None], pos=(0, 0, 5), rotor_speed=w_h * 0.9)
        for _ in range(int(round(0.8 / dt))):
            x = A.step(x, u[None], p4[None], dt)
        return x[0, :13]
    fine = final(0.0005)
    errors = [float(np.abs(final(dt) - fine).max()) for dt in (0.008, 0.004, 0.002)]
    ratios = [a / b for a, b in zip(errors, errors[1:])]
    assert all(10.0 < r < 24.0 for r in ratios), (errors, ratios)


def test_braking_is_batch_invariant_and_numba_is_numpy():
    """A stop's numbers do not depend on its batch: env i alone = env i among 24 (bit for bit), on numpy and
    numba; numba = numpy bit for bit; and they are physical (all stop; farther from a higher v0)."""
    seeds = list(range(24))
    variant = {'mass': 2.7, 'tau_up': 0.03, 'tau_down': 0.06}
    batch = braking.stops('x500', 8.0, seeds, variant, backend='numpy')
    for i in (0, 17):
        alone = braking.stops('x500', 8.0, [seeds[i]], variant, backend='numpy')
        assert all(np.array_equal(alone[k][0], batch[k][i], equal_nan=True) for k in braking.METRICS)
    if HAS['numba']:
        fast = braking.stops('x500', 8.0, seeds, variant, backend='numba')
        alone = braking.stops('x500', 8.0, [seeds[5]], variant, backend='numba')
        for k in braking.METRICS:
            assert np.array_equal(fast[k], batch[k], equal_nan=True), k
            assert np.array_equal(alone[k][0], fast[k][5], equal_nan=True), k
    assert (batch['outcome'] == 'STOPPED').all()
    slow = braking.stops('x500', 4.0, seeds[:4], variant, backend='numpy')
    assert (slow['stop_distance_m'] < batch['stop_distance_m'][:4]).all()
