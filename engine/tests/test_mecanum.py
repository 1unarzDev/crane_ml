"""Gate 1 for the modular kernel (engine/kernels/mecanum.py): every backend against the port of the modular truth
stepper (domains/modular/truth.py) on a scripted program, batch invariance, dt convergence, and the port against
the truth model itself when its checkout is named.

Provenance of the reference: Tools/Fleet/capplan/truth/{model.py, stepper.py} on the modular platform branch,
ported (domains/modular/truth.py says what differs). Upstream check: CRANE_MODULAR_TRUTH=<dir holding capplan>.

Run from the crane root: python3 -m pytest engine/tests/test_mecanum.py
"""

import os
import subprocess
import sys
import textwrap
from pathlib import Path

import numpy as np
import pytest

pytest.importorskip('numba', reason='the engine compiles with numba (the [fast] extra); its numpy backend too')

from domains.modular import assembly as AS, profile as PR, truth as TR
from engine.kernels import mecanum as K, mecanum_run as R

ROOT = Path(__file__).resolve().parents[2]
CHANNELS = ('cmd', 'pose', 'twist', 'duty', 'speed', 'counts', 'gz', 'ax', 'ay')
# (assembly, card variant, faults (mode, gain, onset_s) per wheel, latency steps)
CASES = [('mecanum-1', 'plated', ([0, 1, 3, 0], [1, 0.5, 1, 1], [0, 0, 0.5, 0]), 2),
         ('mecanum-pair', 'plateless', ([2, 0, 0, 4, 0, 0, 1, 0], [1, 1, 1, 1, 1, 1, 0.4, 1], [0] * 8), 0),
         ('diff-caster', None, ([0, 2, 0, 0], [1] * 4, [0, 0.8, 0, 0]), 1)]


def program(T=60, seed=0):
    """A scripted 50 Hz twist program: sweeps, a step, a reversal and a stop (every regime of the slip model)."""
    t = np.arange(T) * 0.02
    r = np.random.default_rng(seed).uniform(0.5, 1.5, 3)
    return np.stack([0.35 * np.sin(r[0] * 2 * t), 0.25 * (t > 0.3) - 0.5 * (t > 0.9), 0.8 * np.cos(r[2] * 3 * t)], -1) \
        * (t < 1.1)[:, None]


def case(i):
    name, variant, (mode, gain, onset), delay = CASES[i]
    built = AS.build(name, PR.load(f'example#{variant}' if variant else 'example'))
    faults = {'mode': mode, 'gain': gain, 'onset': onset}
    return built, K.pack(built['wheels'], built['body'], faults), TR.Assembly(built, mode, gain, onset), delay


def test_every_backend_is_the_truth_port_on_a_scripted_program():
    """numpy and numba = the port bit for bit (float64); JAX float64 within 1e-8; with every fault mode, an onset
    mid-run, command latency, mecanum, conventional wheels and casters, one and two modules."""
    worst = {}
    for i in range(len(CASES)):
        built, p, A, delay = case(i)
        tl = program(seed=i)
        ref = TR.run(A, tl, delay=delay)
        assert np.isfinite(ref['pose']).all() and np.abs(ref['twist']).max() > 0.05
        backends = ['numpy', 'numba'] + (['jax'] if _have_jax() else [])
        for be in backends:
            out = R.replay(p[None], tl[None], delay=[delay], backend=be, dtype='float64')
            for k in CHANNELS:
                d = float(np.abs(out[k][0] - ref[k]).max())
                worst[be] = max(worst.get(be, 0.0), d if k != 'counts' else 0.0)
                if be == 'jax':
                    assert d <= (1.0 if k == 'counts' else 1e-8), (CASES[i][0], be, k, d)
                else:
                    assert d == 0.0, (CASES[i][0], be, k, d)
    print('gate 1 worst |kernel - port|:', worst)


def test_an_env_reads_only_its_own_rows():
    """Env i alone (E=1) = env i in a mixed batch (three assemblies, other faults, other timelines, other latency)."""
    rows, tls, delays = [], [], []
    for e in range(9):
        _, p, _, delay = case(e % 3)
        rows.append(p)
        tls.append(program(seed=10 + e))
        delays.append((delay + e) % 4)
    P = K.stack(rows)
    lengths = np.array([60, 45, 60, 30, 60, 52, 60, 60, 20])
    for be in ('numpy', 'numba'):
        big = R.replay(P, np.array(tls), lengths=lengths, delay=delays, backend=be)
        for e in (0, 4, 8):
            one = R.replay(P[e:e + 1], np.array(tls)[e:e + 1], lengths=lengths[e:e + 1], delay=delays[e:e + 1],
                           backend=be, ticks=60)
            for k in CHANNELS:
                assert np.array_equal(one[k][0], big[k][e]), (be, e, k)
    # closed loop (Ticks) = replay of the same commands
    plant = R.Ticks(P, delay=delays, ticks=60, backend='numba')
    for j in range(60):
        x = plant.tick(np.array([tl[j] if j < L else np.zeros(3) for tl, L in zip(tls, lengths)]))
    assert np.array_equal(x[:, 0:3], big['pose'][:, -1])


def test_dt_convergence_is_first_order():
    """Linearly-implicit Euler: halving the plant step about halves the error against a fine reference, on a
    smooth program (from 5 ms down; the 50 Hz tick is fixed)."""
    built, _, _, _ = case(0)
    p = K.pack(built['wheels'], built['body'])
    t = np.arange(40) * 0.02
    tl = np.stack([0.3 * np.sin(3 * t), 0.2 * np.sin(2 * t), 0.5 * np.cos(3 * t)], -1)
    final = {n: R.replay(p[None], tl[None], dt=0.02 / n, nsub=n, backend='numba')['pose'][0, -1]
             for n in (4, 8, 16, 32, 512)}
    e = [np.linalg.norm(final[n] - final[512]) for n in (4, 8, 16, 32)]
    assert e[0] < 2e-3 and all(1.6 < a / b < 2.6 for a, b in zip(e, e[1:])), e


def test_physics_limits():
    """High friction and stiff slip: the achieved twist is the commanded one (ideal mecanum kinematics); a braked
    wheel stops the robot sooner than a coasting one; a locked wheel's encoder never counts."""
    prof = PR.load('example', {'floor.mu': 3.0, 'wheel.K': 20.0, 'wheel.tau_stall_Nm': 20.0})
    b = AS.build('mecanum-1', prof)
    p = K.pack(b['wheels'], b['body'])
    want = np.array([0.2, -0.15, 0.4])
    out = R.replay(p[None], np.tile(want, (100, 1))[None], backend='numba')
    assert np.allclose(out['twist'][0, -1], want, rtol=0.01, atol=1e-3), out['twist'][0, -1]
    b = AS.build('mecanum-1', PR.load('example'))
    rows = [K.pack(b['wheels'], b['body'], {'mode': [m] * 4, 'onset': [1.0] * 4}) for m in (K.COAST, K.BRAKE, K.LOCKED)]
    tl = np.tile([0.4, 0.0, 0.0], (100, 1))
    tl[50:] = 0.0
    out = R.replay(K.stack(rows), np.array([tl] * 3), backend='numba')
    stop = [int(np.argmax(np.abs(out['twist'][e, 50:, 0]) < 0.01)) for e in range(3)]
    assert stop[1] <= stop[0] and out['twist'][0, 52, 0] > out['twist'][1, 52, 0], stop
    assert np.all(np.diff(out['counts'][2, 55:], axis=0) == 0)


def _have_jax():
    try:
        import jax  # noqa: F401
        return True
    except ImportError:
        return False


@pytest.mark.skipif(not os.environ.get('CRANE_MODULAR_TRUTH'), reason='CRANE_MODULAR_TRUTH names no truth checkout')
def test_the_port_is_the_truth_model():
    """The port against capplan.truth.model.simulate_assembly itself (one module, every per-wheel jitter fixed to
    the card's values, loads static, compliance off): ok, weak, dead_free and dead_locked wheels."""
    probe = textwrap.dedent(f"""
        import sys
        sys.dont_write_bytecode = True
        sys.path[:0] = [{os.environ['CRANE_MODULAR_TRUTH']!r}, {str(ROOT)!r}]
        import numpy as np
        from capplan import library as L, modgraph as MG
        from capplan.truth import model as M
        from capplan.truth.randomize import World
        from domains.modular import assembly as AS, profile as PR, truth as TR
        prof = PR.load('example')
        asm = AS.load('mecanum-1'); t = asm['types']['mecanum']; wp, fl = prof['wheel'], prof['floor']
        L.register('crane_check', L._t('crane_check', L.body(*t['size'], 0.14, prof['module']['mass_kg']['mecanum']),
            [L.wheel('wheel_mecanum', w['x'], w['y'], 0, w['hand'], radius=t['wheel_radius'],
                     phi_max=wp['phi_max_rad_s'], tau_max=wp['tau_stall_Nm']) for w in t['wheels']]))
        g = MG.ModuleGraph([MG.Module('M1', 'crane_check')])
        w = World(0, 'isotropic', dict(mu_long=fl['mu'], kappa_lat=fl['kappa_lat'], mu_jitter=1.0, lat_jitter=1.0,
            roller_mu_f=wp['roller_mu_f'], mu_rb=wp['mu_rb'], roller_dev_deg=0.0, r_f=1.0, s_x0=wp['s_x0'],
            alpha0=wp['alpha0'], c_rr=wp['c_rr'], k_v=wp['k_v'], K_v=wp['K'], tau_stall=wp['tau_stall_Nm'],
            w_nl=wp['w_nl_rad_s'], b_visc=wp['b_visc'], tau_c=wp['tau_c'], tau_bd=wp['tau_bd'], mass_f=1.0,
            com_dx=0.0, com_dy=0.0, inertia_f=prof['module']['inertia_f']))
        b = AS.build(asm, prof)
        tt = np.arange(400) * 0.005
        cmd = np.stack([0.3 * np.sin(2 * tt), 0.25 * (tt > 0.4), 0.6 * np.cos(3 * tt)], -1)
        worst = np.zeros(4)
        for health, mode in ((None, [0, 0, 0, 0]), ({{('M1', 0): ('weak', 0.5)}}, [1, 0, 0, 0]),
                             ({{('M1', 1): 'dead_free'}}, [0, 2, 0, 0]), ({{('M1', 2): 'dead_locked'}}, [0, 0, 4, 0])):
            A = M.build(g, health, w)
            P = TR.Assembly(b, mode, [0.5, 1, 1, 1])
            cmds = np.array([TR.firmware(P, u) for u in cmd])
            tr = M.simulate_assembly(A, cmds, dt=0.005, load_every=10 ** 9, compliance=False)
            pose, xi, got = (0.0, 0.0, 0.0), np.zeros(3), []
            for k in range(len(cmds)):
                pose, xi, _, om = TR.plant_step(P, pose, xi, cmds[k], k * 0.005, 0.005)
                got.append((pose, xi, om))
            worst = np.maximum(worst, [np.abs(A.N0 - P.N).max(),
                                       np.abs(np.array([q[0] for q in got]) - tr['pose']).max(),
                                       np.abs(np.array([q[1] for q in got]) - tr['xi']).max(),
                                       np.abs(np.array([q[2] for q in got]) - tr['om']).max()])
        print(' '.join(f'{{v:.3g}}' for v in worst))
    """)
    r = subprocess.run([sys.executable, '-B', '-c', probe], capture_output=True, text=True, timeout=300, cwd=ROOT)
    assert r.returncode == 0, r.stderr[-2000:]
    n, pose, twist, speed = map(float, r.stdout.split())
    assert n < 1e-9 and pose < 1e-9 and twist < 1e-8 and speed < 1e-7, r.stdout
