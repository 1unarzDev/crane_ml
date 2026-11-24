"""Gate 1, kernel exactness: the boat kernels against the twin (Tools/Characterization/plant.py) on
every backend, the dead-time ring buffer and lag, and the dt convergence against the RK4 reference.

Run from the crane root: python3 -m pytest engine/tests/test_kernels.py
"""

import json
import math

import numpy as np
import pytest

from domains.boat.plant import ROOT, characterization
from engine.kernels import first_order as F, profile as K
from fastsim import config
from fastsim.plant import FirstOrderPlant, Limits

CHAR = ROOT / 'Tools' / 'Characterization'
CASES = json.loads((CHAR / 'parity' / 'cases.json').read_text())
PROFILE = json.loads((CHAR / CASES['profileFile']).read_text())


def run_numpy(p, pwm, dt):
    x = K.init(p[None])
    for u in pwm:
        x = K.step(x, u[None], p[None], dt)
        yield x[0]


def run_numba(p, pwm, dt):
    x = K.init(p[None])[0]
    for u in pwm:
        K.nb_step(x, u, p, dt)
        yield x


def run_jax(p, pwm, dt):
    from engine.jaxconf import jax, jnp
    step = jax.jit(lambda x, u: K.step(x, u, jnp.asarray(p[None]), dt, jnp))

    def body(x, u):
        x = step(x, u[None])
        return x, x[0]
    _, xs = jax.lax.scan(body, jnp.asarray(K.init(p[None])), jnp.asarray(pwm))
    yield from np.asarray(xs)


RUNNERS = {'numpy': run_numpy, 'numba': run_numba, 'jax': run_jax}


@pytest.mark.parametrize('backend', RUNNERS)
def test_every_backend_is_the_twin_on_the_parity_cases(backend):
    """parity/cases.json (the C# MarinePlanarModel gate): every sampled state within 1e-12 in float64."""
    for case in CASES['cases']:
        dt = case['dt']
        p = K.pack(PROFILE, dt, (case['currentX'], case['currentY']))
        pwm = np.array(case['pwm']).reshape(-1, 4)
        want = dict(zip(case['expectedTicks'], range(len(case['expectedTicks']))))
        worst = 0.0
        for k, x in enumerate(RUNNERS[backend](p, pwm, dt)):
            if k + 1 in want:
                j = want[k + 1]
                worst = max(worst, float(np.abs(x[:6] - case['expected'][6 * j:6 * j + 6]).max()))
        assert worst <= 1e-12, (backend, case['name'], worst)


def test_dead_time_and_lag_are_the_twins_ring_buffer():
    """Thrusters with dead time (3 and 5 ticks), lag, deadband and offset: numpy and numba equal the
    twin bit for bit, jax within 1e-12, over a PWM program that flips every few ticks."""
    prof = json.loads(json.dumps(PROFILE))
    for i, t in enumerate(prof['thrusters']):
        t.update(deadTime=0.06 if i % 2 else 0.1, tauT=0.05 * (i + 1), deadbandUs=12.0, offsetUs=4.0 * i - 6.0)
    dt = 0.02
    rng = np.random.default_rng(3)
    pwm = np.repeat(rng.uniform(1150.0, 1850.0, (100, 4)), 7, axis=0)
    twin = characterization('ProfilePlant')(prof, dt)
    states = []
    for u in pwm:
        twin.step_pwm(list(u))
        states.append(list(twin.state))
    p = K.pack(prof, dt)
    for backend, run in RUNNERS.items():
        got = np.array([x[:6].copy() for x in run(p, pwm, dt)])
        err = float(np.abs(got - np.array(states)).max())
        assert err == 0.0 if backend != 'jax' else err <= 1e-12, (backend, err)


def test_the_kernel_converges_at_first_order_in_dt():
    """Symplectic Euler: the error against reference_run (RK4, h = 1 ms) halves as dt halves."""
    plant = characterization('reference_run')
    pwm = characterization('pwm_for_effort')(surge=0.7, yaw=0.3, profile=PROFILE)
    tau = characterization('ProfilePlant')(PROFILE, 0.02).thrust(pwm)
    ref = np.array(plant(PROFILE, lambda t: tau, 4.0, h=1e-3)[-1][1:])
    errors = []
    for dt in (0.04, 0.02, 0.01, 0.005):
        p = K.pack(PROFILE, dt)
        *_, x = run_numpy(p, np.tile(pwm, (int(round(4.0 / dt)), 1)), dt)
        errors.append(float(np.abs(x[:6] - ref).max()))
    ratios = [a / b for a, b in zip(errors, errors[1:])]
    assert all(1.8 < r < 2.2 for r in ratios), (errors, ratios)


def test_the_T0_kernel_is_FirstOrderPlant_with_dead_time():
    cfg = config.load(ROOT / 'domains' / 'boat' / 'fastsim_t0.yaml')[0]
    lim = Limits.from_dict(cfg['limits'])
    spec = {'tau_s': cfg['vehicle']['tau_s'], 'dead_time_s': [0.1, 0.0, 0.25], 'speed_scale': 0.9}
    plant = FirstOrderPlant(lim, **spec)
    p = F.pack(spec, lim, 0.05)
    x, xb = F.init(p[None]), F.init(p[None])[0]
    for k in range(2000):
        t = k * 0.05
        u = np.array((1.2 if t % 30 < 16 else -0.3, 0.3 * math.sin(t), 0.6 if 2 < t % 10 < 8 else -0.4))
        plant.step(tuple(u), 0.05)
        x = F.step(x, u[None], p[None], 0.05)
        F.nb_step(xb, u, p, 0.05)
        assert tuple(x[0, :3]) == plant.pose and tuple(x[0, 3:6]) == plant.vel and (x[0] == xb).all(), k
