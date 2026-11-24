"""crane.run/1: the record round trip, frames and clocks, and the fastsim wrapper.

Run from the crane root: python3 -m pytest tests/test_record.py
"""

import json
import math
from pathlib import Path
import sys

import numpy as np
import pytest

ROOT = Path(__file__).resolve().parents[1]
sys.path[:0] = [str(ROOT), str(ROOT / 'fastsim' / 'tests')]

from crane import frames, record  # noqa: E402
from theater import traj as ctraj  # noqa: E402

ENU = {'world': 'enu', 'origin': 'pool', 'body': 'flu'}
NED = {'world': 'ned', 'origin': 'pool', 'body': 'frd'}
SIM = {'source': 'sim', 't0': 0.0, 'time_multiplier': None}


def a_meta(run_id='r1', **over):
    return {**record.meta(run_id, domain='toy', tier='T2', kind='unity', frame=ENU, clock=SIM, seed=3,
                          timing={'t_src': 'sim', 'injected_delay_s': 0.12, 'derate': [1.0, 0.8]}), **over}


def a_traj(hz=20.0, n=41, t0=0.0, agent='robot'):
    ts = [i / hz for i in range(n)]
    cols = {f'{agent}.x': [math.cos(0.3 * t) for t in ts], f'{agent}.y': [0.5 * t for t in ts],
            f'{agent}.z': [-0.2] * n, f'{agent}.yaw': [frames.wrap(1.5 * t).item() for t in ts],
            f'{agent}.pitch': [0.05] * n, f'{agent}.roll': [-0.1] * n}
    data = ctraj.encode({'pose_dims': '6dof', 'robot': {'model': 'm', 'agents': [agent]},
                         'frame': {'type': 'enu', 'origin': 'pool'}}, cols,
                        [{'t': 0.5, 'kind': 'gate', 'id': 'g1'}], hz=hz, t0_s=t0)
    return data, cols


def test_write_read_round_trip_with_the_trajectory(tmp_path):
    data, cols = a_traj()
    m = a_meta()
    written = record.write_run(tmp_path / 'run', m, traj=data, events=[{'t': 0.5, 'kind': 'gate'}],
                               metrics={'score': 0.7})
    r = record.read_run(tmp_path / 'run')
    assert r.meta == m and r.metrics == {'score': 0.7} and r.events == [{'t': 0.5, 'kind': 'gate'}]
    assert r._loaded is False                          # the trajectory is read on first use
    assert r.traj.header['frame'] == {'type': 'enu', 'origin': 'pool', 'body': 'flu'}
    assert r.traj.ticks == 41 and r.traj.events[0]['kind'] == 'gate'
    assert max(abs(a - b) for a, b in zip(r.traj.columns['robot.y'], cols['robot.y'])) <= 0.005 + 1e-9   # 1 cm quantum
    assert written.meta['timing']['derate'] == [1.0, 0.8]
    # a Traj object writes too; an event-kind run has none; a run dir is written once
    record.write_run(tmp_path / 'again', a_meta('r2'), traj=r.traj)
    assert record.read_run(tmp_path / 'again').traj.columns['robot.x'] == r.traj.columns['robot.x']
    ev = record.write_run(tmp_path / 'ev', a_meta('r3', tier='T0', kind='event'), events=[{'t': 1, 'kind': 'done'}])
    assert ev.traj is None and record.read_run(tmp_path / 'ev').events[0]['kind'] == 'done'
    with pytest.raises(record.RecordError, match='event-kind'):
        record.write_run(tmp_path / 'ev2', a_meta('r4', tier='T0', kind='event'), traj=data)
    with pytest.raises(FileExistsError):
        record.write_run(tmp_path / 'run', m)
    with pytest.raises(record.RecordError, match='trajectory is in'):   # the ctraj says pool, the meta dock
        record.write_run(tmp_path / 'x', a_meta('r5', frame={**ENU, 'origin': 'dock'}), traj=data)


def test_validate_names_a_missing_frame_or_clock():
    m = a_meta()
    for key in ('frame', 'clock'):
        with pytest.raises(record.RecordError, match=f'{key} is required'):
            record.validate({k: v for k, v in m.items() if k != key})
    with pytest.raises(record.RecordError, match='frame.body'):
        record.validate({**m, 'frame': {'world': 'enu', 'origin': 'pool'}})
    with pytest.raises(record.RecordError, match='time_multiplier'):
        record.validate({**m, 'clock': {'source': 'wall', 't0': 1.7e9}})
    with pytest.raises(record.RecordError, match='a unity run is T2'):
        record.validate({**m, 'tier': 'T1'})


def test_enu_ned_round_trip_on_known_poses():
    # facing north in ENU (yaw +90 deg) is yaw 0 in NED; facing east is +90 in NED; nose up stays nose up
    enu = {'x': np.array([1.0, 2.0]), 'y': np.array([3.0, -1.0]), 'z': np.array([0.5, -2.0]),
           'yaw': np.array([math.pi / 2, 0.0]), 'pitch': np.array([0.2, -0.3]), 'roll': np.array([0.1, 0.4])}
    ned = frames.convert(enu, ENU, NED)
    assert np.allclose(ned['x'], [3.0, -1.0]) and np.allclose(ned['y'], [1.0, 2.0]) and np.allclose(ned['z'], [-0.5, 2.0])
    assert np.allclose(ned['yaw'], [0.0, math.pi / 2]) and np.allclose(ned['pitch'], [-0.2, 0.3])
    assert np.allclose(ned['roll'], [0.1, 0.4])
    back = frames.convert(ned, NED, ENU)
    for k in enu:
        assert np.allclose(frames.wrap(back[k] - enu[k]), 0.0, atol=1e-12), k
    # a planar pose: yaw only, and a change that would tilt it is refused
    p = frames.convert({'x': 1.0, 'y': 2.0, 'yaw': 0.3}, ENU, NED)
    assert set(p) == {'x', 'y', 'yaw'} and math.isclose(p['yaw'], math.pi / 2 - 0.3)
    with pytest.raises(frames.FrameError, match='tilt'):
        frames.convert({'x': 1.0, 'y': 2.0, 'yaw': 0.3}, ENU, {**NED, 'body': 'flu'})
    # different origins: refused, unless a transform says where one origin sits in the other
    with pytest.raises(frames.FrameError, match='origins differ'):
        frames.convert(enu, ENU, {**NED, 'origin': 'dock'})
    moved = frames.convert({'x': 1.0, 'y': 0.0, 'yaw': 0.0}, ENU, {**ENU, 'origin': 'dock'},
                           {'from': 'pool', 'to': 'dock', 'xyz': [10.0, 0.0, 0.0], 'yaw': math.pi / 2})
    assert np.allclose([moved['x'], moved['y'], moved['yaw']], [10.0, 1.0, math.pi / 2])


def test_to_common_and_planar_projection_of_a_6dof_run(tmp_path):
    data, cols = a_traj()
    r = record.write_run(tmp_path / 'u', a_meta(), traj=data)
    ned = frames.to_common(r, NED)
    assert ned.frame == NED and np.allclose(ned.columns['robot.x'], r.traj.columns['robot.y'])
    with pytest.raises(frames.FrameError, match='origins differ'):
        frames.to_common(r, {**NED, 'origin': 'world'})
    flat = frames.project_planar(r)
    assert flat.pose_dims == 'planar' and set(flat.columns) == {'robot.x', 'robot.y', 'robot.yaw'}
    assert 'projected_planar_from_6dof' in flat.notes
    # the kept yaw is the heading of the body x axis on the ground plane, despite pitch and roll
    i = 7
    pose = {k: r.traj.columns[f'robot.{k}'][i] for k in frames.POSE6}
    nose = frames._rot(np.float64(pose['yaw']), np.float64(pose['pitch']), np.float64(pose['roll']))[:, 0]
    assert math.isclose(math.atan2(nose[1], nose[0]), flat.columns['robot.yaw'][i], abs_tol=1e-9)


def test_two_clocks_resample_onto_one(tmp_path):
    a = record.write_run(tmp_path / 'a', a_meta('a', clock={'source': 'wall', 't0': 1000.0, 'time_multiplier': 1.0}),
                         traj=a_traj(hz=20.0, n=81)[0])                 # 0..4 s, wall 1000..1004
    b = record.write_run(tmp_path / 'b', a_meta('b', clock={'source': 'wall', 't0': 1001.0, 'time_multiplier': 1.0}),
                         traj=a_traj(hz=10.0, n=41)[0])                 # 0..4 s, wall 1001..1005
    a2, b2 = frames.align(a, b)
    assert np.allclose(a2.t, b2.t) and math.isclose(a2.t[0], 1.0) and math.isclose(a2.t[-1], 4.0)
    assert len(a2.t) == 31                                               # the lower rate, 10 Hz, over the overlap
    # b's own t = 0 is a's t = 1: b's y = 0.5 t_b, a's y = 0.5 t_a
    assert np.allclose(b2.columns['robot.y'], 0.5 * (a2.t - 1.0), atol=0.01)
    assert np.allclose(a2.columns['robot.y'], 0.5 * a2.t, atol=0.01)
    assert np.all(np.abs(b2.columns['robot.yaw']) <= math.pi)           # angles wrap after the interpolation
    sim = record.write_run(tmp_path / 's', a_meta('s'), traj=a_traj()[0])
    with pytest.raises(frames.FrameError, match='clocks differ'):
        frames.align(a, sim)
    a3, s3 = frames.align(a, sim, offset_s=2.0)
    assert math.isclose(s3.t[0], 2.0) and s3.clock == a.clock


def test_fastsim_writes_the_record_and_from_fastsim_keeps_the_identity(tmp_path):
    import toy
    from fastsim import config, run
    from fastsim.scenario import JsonScenarioFormat
    here = ROOT / 'fastsim' / 'tests'
    folder = tmp_path / 'scenarios'
    cfg, base = config.load(here / 'toy.yaml', {'scenarios': {'dir': str(folder)}})
    adapter = toy.ToyAdapter(base=here, reach_m=1.5, time_limit_s=240)
    scenario = JsonScenarioFormat().load(toy.write_scenario(folder, 3))
    summary = run.simulate(scenario, adapter, cfg, 3, tmp_path / 'run', base)
    d = tmp_path / 'run'
    native = (d / 'run.json').read_bytes()
    assert json.loads(native)['schema'] == run.SCHEMA
    r = record.read_run(d)
    assert r.meta['run_id'] == summary['identity']['run_sha256'] == r.meta['identity']['run_sha256']
    assert (r.meta['tier'], r.meta['kind'], r.frame, r.clock['source']) == (
        'T0', 'kinematic', {'world': 'enu', 'origin': 'scenario', 'body': 'flu'}, 'sim')
    assert r.traj.header['pose_dims'] == 'planar' and r.events and r.events[0]['kind']
    # an older run dir (no crane.run.json) gets the same meta; fastsim's own run.json is untouched
    sidecar = (d / run.CRANE_RUN).read_bytes()
    (d / run.CRANE_RUN).unlink()
    w = record.from_fastsim(d)
    assert (d / run.CRANE_RUN).read_bytes() == sidecar and (d / 'run.json').read_bytes() == native
    assert w.meta['identity']['run_sha256'] == summary['identity']['run_sha256']
    ned = frames.to_common(w, {'world': 'ned', 'origin': 'scenario', 'body': 'frd'})
    assert np.allclose(ned.columns['robot.x'], w.traj.columns['robot.y'])
