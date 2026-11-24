"""Gate 1 for the 6-DOF kernel: every backend against the reference ``Simulator.step`` (engine/kernels/
marine6_ref.py) over a scripted wrench program, and batch invariance (env i at E=1 is env i at E=1024).

Run from the crane root: python3 -m pytest engine/tests/test_marine6.py
"""

from pathlib import Path

import numpy as np
import pytest

import engine  # noqa: F401  (imports numba's coverage support clear of Tools/RoboBoat/coverage.py)

pytest.importorskip('numba', reason='the engine compiles with numba (the [fast] extra); its numpy backend too')

from engine.kernels import marine6 as K, marine6_ref as REF

PROFILE = Path(__file__).resolve().parents[2] / 'domains' / 'sub' / 'profiles' / 'generic_rov8.json'
CURRENT = (0.12, -0.07, 0.03)


def program(steps, seed=0):
    """Piecewise-constant body wrenches (held 25 steps) with a strong yaw and roll phase (> 180 deg turns),
    plus a small external wrench every step."""
    g = np.random.default_rng(seed)
    tau = np.repeat(g.uniform(-60.0, 60.0, (steps // 25 + 1, 6)) * [1, 1, 1, 0.3, 0.3, 0.3], 25, axis=0)[:steps]
    tau[steps // 3:steps // 2, 5] = 20.0
    tau[steps // 2:steps // 2 + 60, 3] = 6.0
    return tau, g.uniform(-4.0, 4.0, (steps, 6))


def reference(dt, steps, pos=(5.0, 7.0, 1.5), rpy=(0.1, -0.2, 0.7)):
    """The reference's states [steps, S] (thrust padded) and the kernel inputs [steps, U] it was driven with."""
    data, params, thr = K.load(PROFILE)
    n, nsub = len(thr), K.substeps(thr, dt)
    sim = REF.Simulator(params, current_n=np.array(CURRENT), thrusters=thr)
    lo, hi = [-t.max_reverse for t in thr], [t.max_forward for t in thr]
    tau, ext = program(steps)
    s = REF.State(np.array(pos), REF.euler_to_quat(*rpy), np.zeros(6), np.zeros(n))
    xs, us = [], []
    for k in range(steps):
        f = REF.allocate(sim._T, tau[k], lo, hi)
        for _ in range(nsub):
            s = sim.step(s, dt / nsub, ext[k], f)
        xs.append(np.concatenate([s.pos, s.q, s.nu, s.thrust, np.zeros(K.NT - n)]))
        us.append(np.concatenate([ext[k], f, np.zeros(K.NT - n)]))
    return np.array(xs), np.array(us), tau


def run_numpy(p, x0, us, dt):
    x = x0[None]
    for u in us:
        x = K.advance(x, u[None], p[None], dt)
        yield x[0]


def run_numba(p, x0, us, dt):
    x, w = x0.copy(), K.workspace()
    for u in us:
        K.nb_advance(x, u, p, dt, w)
        yield x.copy()


def run_jax(p, x0, us, dt):
    from engine.jaxconf import jax, jnp
    n = int(p[K.NSUB])
    with jax.default_device(jax.devices('cpu')[0]):      # the gate is the arithmetic, not a shared GPU's memory
        pj = jnp.asarray(p[None])

        def body(x, u):
            x = K.advance(x, u[None], pj, dt, jnp, nmax=n)
            return x, x[0]
        _, xs = jax.jit(lambda x, us: jax.lax.scan(body, x, us))(jnp.asarray(x0[None]), jnp.asarray(us))
    yield from np.asarray(xs)


RUNNERS = {'numpy': run_numpy, 'numba': run_numba, 'jax': run_jax}


@pytest.mark.parametrize('backend', RUNNERS)
@pytest.mark.parametrize('dt', [0.02, 0.05])
def test_every_backend_is_the_reference_step(backend, dt):
    """600 control periods (one RK4 step at 0.02 s, two substeps at 0.05 s: the lag bound), a NED current,
    lagged and saturating thrusters, turns past 180 deg: every state within 1e-10 of Simulator.step, and the
    allocation within 1e-12 of REF.allocate."""
    if backend == 'jax':
        pytest.importorskip('jax')
    want, us, tau = reference(dt, 600)
    p = K.pack(PROFILE, dt, CURRENT)
    assert int(p[K.NSUB]) == (1 if dt == 0.02 else 2)
    x0 = K.init(p[None], (5.0, 7.0, 1.5), (0.1, -0.2, 0.7))[0]
    got = np.array(list(RUNNERS[backend](p, x0, us, dt)))
    assert np.abs(got - want).max() <= 1e-10, (backend, dt, np.abs(got - want).max())
    yaw = np.unwrap([REF.quat_to_euler(q)[2] for q in want[:, K.QUAT:K.QUAT + 4]])
    assert np.ptp(yaw) > np.pi                                       # the program does turn past 180 deg
    f = np.zeros(K.NT)
    for t, u in zip(tau[::50], us[::50]):
        K.nb_allocate(t, p, f)
        assert np.abs(f - u[6:]).max() <= 1e-12 and np.abs(K.allocate(t[None], p[None])[0] - u[6:]).max() <= 1e-12


def test_batch_invariance_and_per_env_streams():
    """numpy: env i of a 1024-env batch (each env its own current and pose) is env i run alone, bit for bit;
    numba rows equal numpy to 1e-12. The per-env normal draws depend on (key, counter) only."""
    dt, E = 0.02, 1024
    g = np.random.default_rng(1)
    P = K.with_current(np.repeat(K.pack(PROFILE, dt)[None], E, axis=0), g.uniform(-0.2, 0.2, (E, 3)))
    X = K.init(P, g.uniform(2.0, 10.0, (E, 3)), g.uniform(-0.5, 0.5, (E, 3)))
    U = np.zeros((E, K.U))
    U[:, :6] = g.uniform(-3.0, 3.0, (E, 6))
    pick = [0, 517, 1023]
    single = {e: X[e:e + 1].copy() for e in pick}
    xb = {e: X[e].copy() for e in pick}
    w = K.workspace()
    for k in range(150):
        U[:, 6:] = K.allocate(np.where(np.arange(6) < 3, 30.0, 2.0) * np.sin(0.05 * k + np.arange(E)[:, None]), P)
        X = K.advance(X, U, P, dt)
        for e in pick:
            single[e] = K.advance(single[e], U[e:e + 1], P[e:e + 1], dt)
            K.nb_advance(xb[e], U[e], P[e], dt, w)
    for e in pick:
        assert (single[e][0] == X[e]).all(), e
        assert np.abs(xb[e] - X[e]).max() <= 1e-12, e
    keys = [K.key(s, 'thrust_noise') for s in (0, 1)]
    draws = np.array([K.normal(keys[0], n) for n in range(20000)])
    assert abs(draws.mean()) < 0.03 and abs(draws.std() - 1.0) < 0.03
    assert K.normal(keys[0], 7) == K.normal(keys[0], 7) != K.normal(keys[1], 7)


def test_vecdynamics_interface_runs_the_reference():
    """marine6_vec.VecDynamics (the sub platform's batched-plant interface, its shim target): three envs with
    their own currents, 200 steps of allocated wrenches, each within 1e-10 of Simulator.step."""
    from engine.kernels.marine6_vec import VecDynamics
    data, params, thr = K.load(PROFILE)
    dyn = VecDynamics(params, thr)
    cur = np.array([[0.1, 0.0, 0.0], [0.0, -0.2, 0.05], [0.0, 0.0, 0.0]])
    sims = [REF.Simulator(params, current_n=c, thrusters=thr) for c in cur]
    s = [REF.State(np.array([5.0, 7.0, 1.5]), REF.euler_to_quat(0.1 * i, -0.2, 0.7), np.zeros(6), np.zeros(len(thr)))
         for i in range(3)]
    pos, q = np.array([x.pos for x in s]), np.array([x.q for x in s])
    nu, th = np.zeros((3, 6)), np.zeros((3, len(thr)))
    tau, ext = program(200, seed=2)
    for k in range(200):
        f = dyn.allocate(np.repeat(tau[k][None], 3, axis=0))
        pos, q, nu, th, _ = dyn.step(pos, q, nu, th, 0.02, np.repeat(ext[k][None], 3, axis=0), f, cur)
        s = [sim.step(x, 0.02, ext[k], fi) for sim, x, fi in zip(sims, s, f)]
    want = np.array([np.concatenate([x.pos, x.q, x.nu, x.thrust]) for x in s])
    assert np.abs(np.concatenate([pos, q, nu, th], axis=1) - want).max() <= 1e-10
