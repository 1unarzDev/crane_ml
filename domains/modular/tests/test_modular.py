"""The modular domain end to end: logged-timeline replay through `crane run` into a crane.data dataset of 50 Hz
channels with labels, logged faults, the route task with its footprint corridor, and the one-line refusals.

Run from the crane root: python3 -m pytest domains/modular
"""

import json

import numpy as np
import pytest

pytest.importorskip('numba', reason='the modular kernel compiles with numba (the [fast] extra)')

from crane import cli
from crane.data import rows as R
from domains.modular import batch as B
from engine.kernels import mecanum as K


def timelines(path, M=3, T=150, **extra):
    t = np.arange(T) / 50
    cmd = np.stack([0.3 * np.sin(t[None] * (1 + np.arange(M)[:, None])), 0.15 * np.ones((M, T)),
                    0.5 * np.cos(t[None]) * np.ones((M, 1))], -1)
    np.savez(path, cmd=cmd, length=np.array([T, T - 50, T][:M]), id=np.array([f'log{m}' for m in range(M)]),
             hz=50.0, **extra)
    return cmd


def test_replay_writes_every_env_s_50hz_channels_with_labels(tmp_path, capsys):
    cmd = timelines(tmp_path / 'tl.npz')
    out = tmp_path / 'run'
    code = cli.main(['--json', 'run', '--domain', 'modular', '--replay', str(tmp_path / 'tl.npz'), '--seeds', '0:4',
                     '--card', 'example#plated', '--set', 'faults.p_healthy=0', '--set', 'faults.p_double=1',
                     '--set', 'dr=true', '--out', str(out)])
    doc = json.loads(capsys.readouterr().out)
    assert code == 0 and doc['n'] == 12 and doc['channels']['hz'] == 50.0 and doc['sim_s'] == 3.0
    rows = sorted(R.read_rows(out / 'obs'), key=lambda r: (r['timeline'], r['seed']))
    assert len(rows) == 12 and [r['seed'] for r in rows[:4]] == [0, 1, 2, 3]
    names = doc['channels']['channels']
    assert names[:3] == ['cmd.vx', 'cmd.vy', 'cmd.wz'] and 'counts[3]' in names and names[-3:] == ['imu.gz', 'imu.ax', 'imu.ay']
    r = rows[4]                                                     # timeline 1 (100 ticks), seed 0
    assert r['obs'].shape == (100, 14) and r['label.variant'] == 'plated' and r['label.latency_steps'] == 2
    assert np.allclose(r['obs'][:, :3], cmd[1, :100], atol=1e-6)    # the logged command, tick for tick
    assert sum(m != 'ok' for m in r['label.fault_mode']) == 2 and set(r['label.dr']) == {'floor.mu', 'scale.stall',
                                                                                         'wheel.s_x0', 'wheel.tau_bd'}
    # the same seed draws the same faults and DR on every timeline (common random numbers across arms)
    assert rows[1]['label.dr'] == rows[5]['label.dr'] and rows[0]['label.dr'] != rows[1]['label.dr']
    counts = r['obs'][:, 7:11]
    assert np.all(np.isfinite(r['obs'])) and np.abs(np.diff(counts, axis=0)).max() > 0


def test_logged_faults_and_the_kernel_agree(tmp_path, capsys):
    mode = np.array([[0, 0, 0, 0], [4, 0, 0, 0], [0, 2, 3, 0]])
    timelines(tmp_path / 'tl.npz', mode=mode, gain=np.ones((3, 4)))
    out = tmp_path / 'run'
    assert cli.main(['--json', 'run', '--domain', 'modular', '--replay', str(tmp_path / 'tl.npz'), '--seeds', '7',
                     '--out', str(out), '--set', 'record.obs.channels=[counts, vel]']) == 0
    capsys.readouterr()
    rows = sorted(R.read_rows(out / 'obs'), key=lambda r: r['timeline'])
    assert [r['label.fault_mode'] for r in rows][1] == ['locked', 'ok', 'ok', 'ok']
    assert np.all(rows[1]['obs'][:, 0] == 0) and np.any(rows[0]['obs'][:, 0] != 0)   # a locked wheel never counts
    tl = B.timelines(tmp_path / 'tl.npz', 50.0)
    assert tl['length'].tolist() == [150, 100, 150] and K.MODES[int(tl['mode'][2][2])] == 'brake'


def test_the_route_task_and_its_footprint_corridor(tmp_path, capsys):
    got = {}
    for robot in ('mecanum-1', 'mecanum-pair'):
        assert cli.main(['--json', 'run', '--domain', 'modular', '--robot', robot, '--seeds', '0:2',
                         '--out', str(tmp_path / robot)]) == 0
        capsys.readouterr()
        got[robot] = list(R.read_rows(tmp_path / robot / 'metrics'))
    for robot, rows in got.items():
        assert all(r['outcome'] == 'SUCCEEDED' and 10 < r['time_s'] < 40 for r in rows), (robot, rows)
        assert abs(rows[0]['path_m'] - 8.0) < 0.6 and rows[0]['corridor_out_s'] == 0.0
    # the pair's footprint (0.60 m long) comes nearer the corridor edge than one module's
    assert got['mecanum-pair'][0]['corridor_excess_m'] > got['mecanum-1'][0]['corridor_excess_m']


def test_refusals_are_one_line_naming_the_fix(tmp_path, capsys):
    np.savez(tmp_path / 'bad.npz', cmd=np.zeros((2, 10, 3)), hz=100.0)
    cases = [(['run', '--domain', 'modular', '--replay', str(tmp_path / 'bad.npz')], 'resample the timelines'),
             (['run', '--domain', 'modular', '--card', 'example#wet', '--seeds', '0'], 'variants: plated, plateless'),
             (['run', '--domain', 'modular', '--set', 'card.wheel.grip=1', '--seeds', '0'], 'keys: latency_s'),
             (['run', '--domain', 'modular', '--set', 'faults.modes.melted=1', '--replay', str(tmp_path / 'bad.npz')],
              None),
             (['run', '--domain', 'toy', '--replay', str(tmp_path / 'bad.npz'), '--seeds', '0'], '--domain modular')]
    for argv, fix in cases:
        assert cli.main(argv) == 2, argv
        err = capsys.readouterr().err.strip().splitlines()
        assert len(err) == 1 and (fix is None or fix in err[0]), err
