"""The sub domain: `crane run --domain sub --tier T1 --engine batched` end to end (crane.run/1 in NED/FRD with a
6-DOF trajectory, resume, --card, the one-line refusals), the mission's rules and numbers against independent
recomputations, and its batch invariance under per-env draws.

Run from the crane root: python3 -m pytest domains/sub
"""

import copy
import json
import math
from pathlib import Path

import numpy as np
import pytest

import engine  # noqa: F401  (imports numba's coverage support clear of Tools/RoboBoat/coverage.py)

pytest.importorskip('numba', reason='the engine compiles with numba (the [fast] extra)')

from crane import cli, record as crec
from fastsim import config
from fastsim.batch import adapter_of
from theater import traj as ctraj

from domains.sub import mission as M

YAML = Path(__file__).resolve().parents[1] / 'fastsim_t1.yaml'
PROFILE = YAML.parent / 'profiles' / 'generic_rov8.json'


def test_crane_run_sub_T1_batched(tmp_path, capsys):
    out, sc = tmp_path / 'runs', tmp_path / 'courses'
    argv = ['run', '--domain', 'sub', '--tier', 'T1', '--engine', 'batched', '--seeds', '0:6', '--out', str(out),
            '--set', f'scenarios.dir={sc}', '--record', '2', '--allow-dirty']
    assert cli.main(['--json', *argv]) == 0
    doc = json.loads(capsys.readouterr().out)
    assert doc['n'] == 6 and doc['outcomes'] == {'SUCCEEDED': 6}, doc['outcomes']
    rec = crec.read_run(out / 'base' / 's3')
    m = rec.meta
    assert (m['domain'], m['tier'], m['kind'], m['backend']) == ('sub', 'T1', 'dynamic', 'engine/numba')
    assert m['frame'] == {'world': 'ned', 'origin': 'scenario', 'body': 'frd'}
    head = ctraj.read_header(rec.file('traj'))
    assert head['pose_dims'] == '6dof' and head['frame']['type'] == 'ned' and head['frame']['body'] == 'frd'
    z = rec.traj.columns['auv.z']
    assert min(z) >= 0.0 and max(z) > 1.0                                  # NED: depth is +z
    kinds = [e['kind'] for e in rec.events]
    assert kinds.count('gate_passed') == 3 and kinds[-1] == 'hold_done'
    assert rec.metrics['sub']['gates_passed'] == 3 and rec.metrics['sub']['success'] is True
    before = (out / 'base' / 's3' / 'run.json').stat().st_mtime_ns
    assert cli.main(['--json', *argv]) == 0                                   # the same command resumes
    capsys.readouterr()
    assert (out / 'base' / 's3' / 'run.json').stat().st_mtime_ns == before
    assert cli.main([*argv[:6], 'fastsim', *argv[7:]]) == 2                  # scalar engine: refused, fix named
    assert '--engine batched' in capsys.readouterr().err
    assert cli.main([*argv, '--backend', 'jax']) == 2
    assert '--backend numba' in capsys.readouterr().err
    # --card binds the profile (config/domains/sub.yaml card.keys): another file is another run identity
    card = tmp_path / 'rov8-copy.json'
    card.write_bytes(PROFILE.read_bytes())
    assert cli.main([*argv, '--seeds', '0:1', '--card', str(card)]) == 2           # same --out: refused
    assert 'holds another run' in capsys.readouterr().err
    assert cli.main(['--json', *argv, '--seeds', '0:1', '--out', str(tmp_path / 'card'), '--card', str(card)]) == 0
    m1 = crec.read_run(Path(json.loads(capsys.readouterr().out)['runs'][0]['out'])).meta
    assert m1['card']['ref'] == str(card)
    assert cli.main([*argv, '--card', str(tmp_path / 'none.json'), '--dry-run']) == 2
    assert 'no such file' in capsys.readouterr().err


def test_mission_rules_and_numbers(tmp_path):
    """A gate exactly as wide as the vehicle is crossed but missed (FAILED, gates_missed 1); a course that is
    only a hold at the start (deadband off) ends after hold_s with the effort the reference allocation needs to
    cancel the restoring force at rest; path_m is the length of the every-step track; the deadband is the profile's
    thrust curve at its deadband fraction."""
    cfg, base = config.load(YAML, {'scenarios': {'dir': str(tmp_path)}, 'track_hz': 50.0})
    adapter = adapter_of(cfg, base)
    normal = M.Setup(adapter, cfg, base, [0])
    sc = normal.scenarios[0]
    narrow = copy.deepcopy(sc)
    narrow.raw['gates'][1]['width_m'] = 2 * normal.half[2]
    hold = copy.deepcopy(sc)
    s = hold.raw['start']
    hold.raw.update(gates=[], finish={'x': s[0], 'y': s[1], 'z': hold.raw['start_depth_m'], 'yaw': s[2]})
    st = M.Setup(adapter, cfg, base, [0, 0, 0], [sc, narrow, hold])
    st.P[2, M.K.DB:M.K.DB + M.K.NT] = 0.0          # the hold's ~0.25 N per thruster is inside the deadband
    out = M.run(st, record=1)
    ok, miss, held = (out.numbers(e) for e in range(3))
    assert out.outcome(0) == 'SUCCEEDED' and (ok['gates_passed'], ok['gates_missed']) == (3, 0)
    assert out.outcome(1) == 'FAILED' and (miss['gates_passed'], miss['gates_missed']) == (2, 1)
    assert [e['gate'] for e in out.events(1) if e['kind'] == 'gate_missed'] == [sc.raw['gates'][1]['id']]
    # path: the every-step track's length
    xyz = np.array([r[1:4] for r in out.track(0)])
    assert math.isclose(ok['path_m'], np.linalg.norm(np.diff(xyz, axis=0), axis=1).sum(), abs_tol=2e-4)
    # effort: the hold's thrust is the reference allocation of g(eta) at rest, level, for hold_s
    from engine.kernels import marine6_profile as VP, marine6_ref as ref
    data = json.loads(PROFILE.read_text())
    params, thr = VP.to_params(data), VP.to_thrusters(data)
    q = ref.euler_to_quat(0.0, 0.0, s[2])
    f = ref.allocate(ref.allocation_matrix(thr), ref.restoring(params, q),
                     -np.array([t.max_reverse for t in thr]), np.array([t.max_forward for t in thr]))
    assert out.outcome(2) == 'SUCCEEDED' and held['time_s'] == pytest.approx(adapter.mission['hold_s'], abs=0.1)
    assert held['effort_Ns'] == pytest.approx(np.abs(f).sum() * held['time_s'], rel=0.05)
    assert held['hold_rms_m'] < 2e-3 and held['path_m'] < 2e-3     # the net-weight feedforward: it stays put
    # deadband: the odd polynomial thrust curve at deadbandFraction
    th = data['thrusters']
    c9, c7, c5, c3, c1 = th['thrustCurve']['value']['coefficients_V9_V7_V5_V3_V1']
    v = th['deadbandFraction']['value']
    db = abs(np.polyval([c9, 0, c7, 0, c5, 0, c3, 0, c1, 0], v))
    assert db > 0.1 and np.allclose(normal.P[0, M.K.DB:M.K.DB + len(thr)], db, rtol=1e-12)


def test_mission_env_is_the_same_at_any_batch_size(tmp_path):
    """With a randomized current and thrust noise (each env's own streams), env i of a 1024-env batch equals
    env i run alone: outcome, events, numbers and the final state, bit for bit."""
    over = {'scenarios': {'dir': str(tmp_path)}, 'vehicle': {'current_ned': [0.15, 0.1, 0.0],
                                                             'randomize': {'current_sd': 0.1}, 'thrust_noise_sd': 0.05}}
    cfg, base = config.load(YAML, over)
    adapter = adapter_of(cfg, base)
    seeds = list(range(1024))
    big = M.run(M.Setup(adapter, cfg, base, seeds))
    assert np.unique(big.setup.P[:, M.K.CUR]).size == 1024                  # every env its own current
    for e in (0, 511, 1023):
        one = M.run(M.Setup(adapter, cfg, base, [seeds[e]], [big.setup.scenarios[e]]))
        assert one.outcome(0) == big.outcome(e) and one.events(0) == big.events(e), e
        assert one.numbers(0) == big.numbers(e) and (one.S['x'][0] == big.S['x'][e]).all(), e
    assert sum(big.outcome(e) == 'SUCCEEDED' for e in range(1024)) > 900


def test_a_study_runs_the_sub_with_a_card(tmp_path, capsys):
    """A study arm's card: binds the sub's profile (card.keys) and its cells run on the batched engine."""
    card = tmp_path / 'rov8-copy.json'
    card.write_bytes(PROFILE.read_bytes())
    where = {'scenarios.dir': str(tmp_path / 'courses')}
    spec = tmp_path / 'sub.yaml'
    spec.write_text(json.dumps({'schema': 'crane.study/1', 'name': 'sub_card', 'domain': 'sub', 'tier': 'T1',
                                'arms': {'base': {'set': where}, 'card': {'card': str(card), 'set': where}},
                                'seeds': {'dev': '0:2'}}))
    assert cli.main(['study', 'run', str(spec), '--out', str(tmp_path / 'out')]) == 0
    capsys.readouterr()
    import csv
    rows = list(csv.DictReader((tmp_path / 'out' / 'rows' / 'main.csv').open()))
    assert sorted((r['arm'], r['outcome'], bool(r['card_sha'])) for r in rows) == \
        [('base', 'SUCCEEDED', False)] * 2 + [('card', 'SUCCEEDED', True)] * 2
    assert {r['effort_Ns'] for r in rows if r['arm'] == 'base'} == {r['effort_Ns'] for r in rows if r['arm'] == 'card'}
