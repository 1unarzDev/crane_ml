"""Gate 2 and batch invariance: the batched episode against the scalar fastsim path.

Run from the crane root: python3 -m pytest engine/tests/test_episode.py
"""

import json
import math
from types import SimpleNamespace

import numpy as np
import pytest

pytest.importorskip('numba', reason='the engine compiles with numba (the [fast] extra); its numpy backend too')

from domains.boat import mission as BM
from domains.boat.adapter import Mission
from domains.boat.plant import ROOT
from engine import boat as BB, decoupled, equivalence as EQ
from engine.episode import Setup, run
from engine.kernels import profile as PK
from engine.ops import numpy_ops
from fastsim import config
from fastsim.batch import adapter_of
from fastsim.run import simulate
from fastsim.scenario import ScenarioSource
from fastsim.world import World

BOAT = ROOT / 'domains' / 'boat'


def load(tier, overrides=None):
    cfg, base = config.load(BOAT / f'fastsim_{tier.lower()}.yaml', overrides)
    return cfg, base, adapter_of(cfg, base)


def backends():
    out = ['numpy', 'numba']
    try:
        import jax  # noqa: F401
        out.append('jax')
    except ImportError:
        pass
    return out


def test_this_hosts_blas_is_the_order_the_reference_reproduces():
    """The scalar plant's ``pinv @ tau`` is OpenBLAS dgemv: fma(b2, t2, fma(b0, t0, b1 t1)) per row."""
    cfg = config.load(BOAT / 'fastsim_t1.yaml')[0]
    q = PK.pack_loop(json.loads((ROOT / cfg['vehicle']['profile']).read_text()))
    B = q[PK.LX['B00']:PK.LX['B00'] + 12].reshape(4, 3)
    tau = np.random.default_rng(0).normal(0, 50, (20000, 3))
    from engine import exact
    assert (np.array([B @ t for t in tau]) == exact.matvec_blas(B, tau)).all()


@pytest.mark.parametrize('tier', ['T1', 'T0'])
def test_the_float64_batch_is_the_scalar_path_run_for_run(tier, tmp_path):
    """Seeds 0-49 in one batch: outcome, sim time, flags, contacts, idle, clearance, every event,
    boat.json and every track row equal to fastsim.run.simulate's, on each backend in float64."""
    cfg, base, adapter = load(tier)
    seeds = list(range(50))
    ScenarioSource(**cfg['scenarios'], base=base).generate(seeds)
    ref = EQ.scalar(cfg, base, seeds, tmp_path, workers=8, track=True)
    setup = Setup(cfg, base, adapter, seeds)
    for backend in backends():
        got = EQ.records(run(setup, backend, record=1), track=True)
        assert EQ.exact(ref, got) == {}, backend
    assert all(r['boat']['success'] for r in ref.values())


def test_an_env_is_the_same_alone_and_in_a_batch_of_4096():
    """Env i's whole final state (plant, goal, mission, scoring, logs) at E=1 equals its row at E=4096,
    bit for bit on numpy and numba; per-env draws (a random current) come from the env's own stream.
    JAX compiles each batch shape on its own and XLA's GPU kernels round differently between them
    (3.6e-14 m after 17 s here): its records are equal and its state within 1e-9."""
    cfg, base, adapter = load('T1', {'time_limit_s': 12.0})
    seeds = list(range(4096))
    ScenarioSource(**cfg['scenarios'], base=base).generate(seeds)
    big = Setup(cfg, base, adapter, seeds, randomize={'current_sd': 0.05})
    picks = [0, 1777, 4095]
    for backend in backends():
        out = run(big, backend, record=1)
        S = out.S
        for i in picks:
            small = run(Setup(cfg, base, adapter, [seeds[i]], randomize={'current_sd': 0.05}), backend, record=1)
            one = small.S
            assert EQ.records(small, True)[seeds[i]] == EQ.records(out, True)[seeds[i]], (backend, i)
            if backend == 'jax':
                assert np.abs(one['V']['x'][0] - S['V']['x'][i]).max() < 1e-9
                continue

            def rows(tree, e):
                if isinstance(tree, dict):
                    return {k: rows(v, e) for k, v in tree.items() if k not in ('step',)}
                return np.asarray(tree)[e]
            a, b = rows(one, 0), rows(S, i)
            flat_a, flat_b = _flat(a), _flat(b)
            for k in flat_a:
                assert _same_padded(flat_a[k], flat_b[k]), (backend, i, k)
            assert one['M']['outcome'][0] == S['M']['outcome'][i]


def _same_padded(x, y):
    """Equal on the common prefix of every axis; beyond it (a wider batch's padding) only zeros."""
    if x.ndim == 0:
        return x == y
    common = tuple(slice(0, min(a, b)) for a, b in zip(x.shape, y.shape))
    if not np.array_equal(x[common], y[common]):
        return False
    for big in (x, y):
        rest = big.copy()
        rest[common] = 0
        if rest.any():
            return False
    return True


def _flat(tree, prefix=''):
    out = {}
    for k, v in tree.items():
        if isinstance(v, dict):
            out.update(_flat(v, f'{prefix}{k}.'))
        else:
            out[prefix + k] = np.asarray(v)
    return out


def test_decoupled_mode_at_E1_is_the_scalar_path(tmp_path):
    """The boat's own scalar Mission on the batched plant and tracker: the same run directory files."""
    for tier in ('T1', 'T0'):
        cfg, base, adapter = load(tier)
        source = ScenarioSource(**cfg['scenarios'], base=base)
        [(summary, run_dir)] = decoupled.run(Setup(cfg, base, adapter, [3], mode='decoupled'), tmp_path / f'd{tier}')
        ref = simulate(source.load(3), adapter, cfg, 3, tmp_path / f'r{tier}', base)
        for name in ('track.jsonl', 'sim_events.jsonl', 'boat.json', 'states.jsonl'):
            assert (run_dir / name).read_text() == (tmp_path / f'r{tier}' / name).read_text(), (tier, name)
        same = {k for k in ref if ref[k] == summary.get(k)}
        assert set(ref) - same == {'identity', 'plant'}, set(ref) - same  # the engine's own blocks


def test_the_fairway_planner_is_mission_route():
    """The batched detour tree (numpy and numba) against Mission.route(here, 'fairway') from 2000
    points round each committed course, bit for bit."""
    cfg, base, adapter = load('T1')
    source = ScenarioSource(**cfg['scenarios'], base=base)
    rng = np.random.default_rng(11)
    for seed in range(5):
        s = source.load(seed)
        w = World(s)
        plans = BB.BoatPlans.pack(adapter, [s], [w])
        m = Mission(SimpleNamespace(world=w), BM.Plan.from_truth(s.raw))
        lo, hi = w.xy.min(axis=0) - 5, w.xy.max(axis=0) + 5
        here = rng.uniform(lo, hi, (2000, 2))
        A = {k: np.repeat(v, len(here), axis=0) for k, v in plans.A.items()}
        route, n = BB.fairway(numpy_ops(), A, here[:, 0].copy(), here[:, 1].copy(), BB.FAIRWAY_R)
        buf = np.zeros((BB.FAIRWAY_R, 3))
        for j, (x, y) in enumerate(here):
            want, _ = m.route((float(x), float(y)), 'fairway')
            assert [tuple(r) for r in route[j, :n[j]].tolist()] == [tuple(p) for p in want], (seed, j)
            k = BB.nb_fairway(A['obst'][0], len(w.ids), A['mouths'][0], A['mouth_keep'][0], A['approach'][0],
                              float(x), float(y), buf)
            assert [tuple(r) for r in buf[:k].tolist()] == [tuple(p) for p in want], (seed, j, 'numba')


def test_the_run_identity_names_backend_dtype_and_substeps():
    cfg, base, adapter = load('T1', {'time_limit_s': 1.0})
    setup = Setup(cfg, base, adapter, [1])
    ids = {(b, d, f): run(setup, b, d, fastmath=f).identity(0)['run_sha256']
           for b, d, f in (('numpy', 'float64', False), ('numba', 'float64', False), ('numba', 'float64', True))}
    assert len(set(ids.values())) == 3
    assert run(setup, 'numba').identity(0) == run(setup, 'numba').identity(0)
    assert setup.identity('numba', 'float64')['substeps'] == {'policy': 'fixed_per_card', 'n': 1, 'tick_s': 0.02}
    assert math.isclose(setup.dt, 0.02)
