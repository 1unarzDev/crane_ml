"""crane.judge.run/2: header blocks, standard channels with ticks, declared absence, /1 compatibility,
deterministic save/digest, and the guard against evaluator truth posing as an observation."""

from dataclasses import replace
import json
import sys
from pathlib import Path

import numpy as np
import pytest

sys.path.insert(0, str(Path(__file__).resolve().parent))
from judge import record as R  # noqa: E402
from test_validity import make  # noqa: E402

V1_DOC = {
    'schema': 'crane.judge.run/1',
    'header': {'run_id': 'old', 'tier': 'sim', 'source': 'fastsim', 'domain': 'planar',
               'robot': {'id': 'v', 'model': '', 'morphology': None, 'plant_card': None}, 'task_id': '',
               'scenario': {'pair_key': None, 'truth_sha256': None, 'seed': 3, 'family': None, 'condition': {}},
               'provenance': {}, 'clock': {'base': 'sim', 'anchor': None, 'offset_s': 0.0, 'residual_s': None},
               'window': [0.0, 1.0, 'track_end'], 'source_version': '1'},
    'channels': {'pose.se2': {'name': 'pose.se2', 'fields': ['x', 'y', 'yaw'], 'units': ['m', 'm', 'rad'],
                              'frame': 'world', 'observability': 'exact', 'rate_hz': None, 'angles': ['yaw'],
                              'gaps': [], 'n': 2, 'has_sigma': False}},
    'events': [{'t': 0.0, 'kind': 'start', 'actor': '', 'payload': {}}], 'caps': {},
    'validity': {'valid': True, 'flags': {}, 'excluded_from': []}, 'truth': {},
}


def test_v1_fixture_written_by_the_old_writer_still_loads(tmp_path):
    (tmp_path / 'record.json').write_text(json.dumps(V1_DOC))
    (tmp_path / 'channels.npz').write_bytes(R._npz_bytes({'pose.se2/t': np.array([0.0, 1.0]),
                                                          'pose.se2/v': np.zeros((2, 3))}))
    rec = R.load(tmp_path)
    assert rec.header.schema == R.SCHEMA_V1
    assert all(getattr(rec.header, b) is None for b in R.V2_BLOCKS)
    assert rec.channel('pose.se2').tick is None
    # re-saved, it is still /1 with the same header keys as the old writer
    out = R.save(rec, tmp_path / 'again')
    doc = json.loads((out / 'record.json').read_text())
    assert doc['schema'] == R.SCHEMA_V1 and set(doc['header']) == set(V1_DOC['header'])
    assert set(doc['channels']['pose.se2']) == set(V1_DOC['channels']['pose.se2'])


def test_v2_roundtrip_and_deterministic_digest(tmp_path):
    a, b = make(), make()
    pa, pb = R.save(a, tmp_path / 'a'), R.save(b, tmp_path / 'b')
    assert R.digest(pa) == R.digest(pb)
    assert json.loads((pa / 'record.json').read_text())['schema'] == R.SCHEMA_V2
    back = R.load(pa)
    assert back.header == a.header and back.header.schema == R.SCHEMA_V2
    for n, ch in a.channels.items():
        bc = back.channels[n]
        assert bc.spec == ch.spec and np.array_equal(bc.t, ch.t) and np.array_equal(bc.v, ch.v)
        assert (bc.tick is None) == (ch.tick is None)
        if ch.tick is not None:
            assert bc.tick.dtype == np.int64 and np.array_equal(bc.tick, ch.tick)
    assert R.declared_absent(back) == {'env.wind': 'the fast layer models no wind'}
    assert not back.has('env.wind')
    with pytest.raises(R.Unobservable):
        back.channel('env.wind')
    assert R.digest(R.save(back, tmp_path / 'c')) == R.digest(pa)
    assert R.digest(R.save(make(seed=1), tmp_path / 'd')) != R.digest(pa)


def test_v2_save_requires_every_block(tmp_path):
    rec = make()
    partial = replace(rec, header=replace(rec.header, reset=None))
    with pytest.raises(ValueError, match='reset'):
        R.save(partial, tmp_path / 'p')
    v1 = replace(rec, header=replace(rec.header, **{b: None for b in R.V2_BLOCKS}))
    with pytest.raises(ValueError, match='/2 requires'):
        R.save(v1, tmp_path / 'q', schema=R.SCHEMA_V2)
    R.save(v1, tmp_path / 'r')          # no blocks at all is still a legal /1 record


def test_v2_load_refuses_a_file_missing_a_block(tmp_path):
    p = R.save(make(), tmp_path / 'a')
    doc = json.loads((p / 'record.json').read_text())
    doc['header'].pop('evidence')
    (p / 'record.json').write_text(json.dumps(doc))
    with pytest.raises(ValueError, match='evidence'):
        R.load(p)


def test_nullable_fields_inside_blocks_are_explicit_nulls(tmp_path):
    rec = make()
    h = replace(rec.header, calibration=R.Calibration(), reset=R.Reset(),
                clock_mode=R.ClockMode('recorded'), controller=R.Controller('c'))
    p = R.save(replace(rec, header=h), tmp_path / 'n')
    hd = json.loads((p / 'record.json').read_text())['header']
    assert hd['calibration'] == {'twin': None, 'arm': None, 'evidence_sha': None, 'lineage': []}
    assert hd['reset'] == {'method': None, 'verified': None, 'pose_error': None}
    assert R.load(p).header == h


@pytest.mark.parametrize('bad', [
    lambda: R.Backend('gpu', 'x'),
    lambda: R.Backend('fast', ''),
    lambda: R.ClockMode('wallclock'),
    lambda: R.ClockMode('fixed_step', dt_s=0),
    lambda: R.ClockMode('fixed_step', episode=1.5),
    lambda: R.Code('r', 'c', 'no'),
    lambda: R.Evidence('anecdote'),
    lambda: R.Reset(verified='yes'),
    lambda: R.Calibration(lineage=[1]),
    lambda: R.Controller(''),
])
def test_blocks_validate(bad):
    with pytest.raises(ValueError):
        bad()


def test_header_refuses_a_wrong_block_type():
    with pytest.raises(ValueError):
        R.Header('r', 'fast', 's', 'd', R.Robot('v'), backend={'layer': 'fast'})


def test_standard_constructors_and_ticks():
    ch = R.truth_pose([0, 0.02], [0, 1], [0, 0], [0, 0], tick=[0, 1])
    assert ch.spec.fields == ('x', 'y', 'yaw') and ch.spec.frame == 'scenario' and ch.tick.tolist() == [0, 1]
    assert R.cmd_applied([0], [[0.1, 0.2, 0.3, 0.4]], [0.01]).spec.fields == ('q0', 'q1', 'q2', 'q3', 't_apply')
    assert R.cmd_sent([0], [[1500] * 4], [0.0]).spec.units[-1] == 's'
    assert set(R.STANDARD) == {'truth.pose', 'truth.twist', 'obs.pose', 'obs.twist', 'obs.stamp', 'cmd.requested',
                               'cmd.sent', 'cmd.applied', 'act.thrust', 'env.current', 'env.wind'}
    with pytest.raises(ValueError, match='integers'):
        R.truth_pose([0, 1], [0, 0], [0, 0], [0, 0], tick=[0, 0.5])
    with pytest.raises(ValueError, match='ticks'):
        R.truth_pose([0, 1], [0, 0], [0, 0], [0, 0], tick=[0])
    with pytest.raises(ValueError, match='observability'):
        R.truth_pose([0], [0], [0], [0], observability='commanded')
    with pytest.raises(ValueError, match='not a standard'):
        R.standard('obs.lidar', [0], [[0]])
    w = ch.window(0.01, 1)
    assert w.tick.tolist() == [1]


def test_vocabulary_constants():
    assert R.EVENTS == ('episode_start', 'gate_cross', 'contact', 'dock_enter', 'dock_hold_start', 'docked',
                        'timeout', 'link_lost', 'reset', 'abort')
    assert R.TERMINAL == ('docked', 'timeout', 'abort', 'link_lost', 'infra_error', 'track_end')


def test_declare_absent_needs_a_reason():
    with pytest.raises(ValueError):
        R.declare_absent('cmd.applied', ' ')
    c = R.declare_absent('sensor.sonar.range', 'no sonar', fields=('r',), units=('m',))
    assert c.spec.observability == 'absent' and c.v.shape == (0, 1)


@pytest.mark.parametrize('how', ['object', 'view', 'copy'])
def test_truth_as_observation_is_refused(tmp_path, how):
    rec = make()
    tp = rec.channels['truth.pose']
    v = {'object': tp.v, 'view': tp.v[:], 'copy': tp.v.copy()}[how]
    obs = R.Channel(R.standard_spec('obs.pose'), tp.t, v, None, (), tp.tick)
    with pytest.raises(R.ObsIsTruth, match='obs.pose is truth.pose'):
        R.save(R.RunRecord(rec.header, {**rec.channels, 'obs.pose': obs}, rec.events), tmp_path / how)


def test_declared_obs_is_truth_saves_and_roundtrips(tmp_path):
    rec = make()
    tp = rec.channels['truth.pose']
    obs = R.obs_pose(tp.t, *tp.v.T, tick=tp.tick, observability='exact',
                     obs_is_truth='fast layer feeds the controller ground truth (perception: truth)')
    rec = R.RunRecord(rec.header, {**rec.channels, 'obs.pose': obs}, rec.events)
    back = R.load(R.save(rec, tmp_path / 'ok'))
    assert back.channels['obs.pose'].spec.obs_is_truth.startswith('fast layer')
    assert R.obs_truth_aliases(back) == []
    with pytest.raises(ValueError):
        R.standard_spec('truth.pose', obs_is_truth='nonsense')


def test_absent_channel_with_samples_is_refused(tmp_path):
    rec = make()
    tp = rec.channels['truth.pose']
    bad = R.Channel(R.standard_spec('env.wind', 'absent', note='x'), tp.t[:2], np.zeros((2, 2)))
    with pytest.raises(ValueError, match='declared absent'):
        R.save(R.RunRecord(rec.header, {**rec.channels, 'env.wind': bad}, rec.events), tmp_path / 'w')
