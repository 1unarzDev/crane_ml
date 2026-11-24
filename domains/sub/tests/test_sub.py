"""The sub domain: `crane run --domain sub --tier T1 --engine batched` end to end (crane.run/1 in NED/FRD with a
6-DOF trajectory, resume, the one-line refusals), and the mission's batch invariance under per-env draws.

Run from the crane root: python3 -m pytest domains/sub
"""

import json
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
