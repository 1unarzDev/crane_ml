"""crane-traj/1: round trip, windows, amend, corruption, size, the golden file and fastsim's writer."""

import hashlib
import json
import math
import random

import pytest

from theater import meta, traj
from theater.tests import golden_data


def boat_run(minutes=10.0, hz=20.0, seed=1):
    """A plausible boat: 1-2 m/s, wandering heading, small heave and roll, a few commands."""
    rng = random.Random(seed)
    n = int(minutes * 60 * hz)
    x = y = yaw = 0.0
    cols = {k: [] for k in ('boat.x', 'boat.y', 'boat.z', 'boat.yaw', 'boat.pitch', 'boat.roll',
                            'boat.speed', 'boat.cmd.v', 'boat.cmd.w')}
    for i in range(n):
        t = i / hz
        w = 0.3 * math.sin(t / 7.0) + rng.gauss(0, 0.02)
        v = 1.5 + 0.4 * math.sin(t / 11.0)
        yaw = math.atan2(math.sin(yaw + w / hz), math.cos(yaw + w / hz))
        x += v * math.cos(yaw) / hz
        y += v * math.sin(yaw) / hz
        for k, val in (('x', x), ('y', y), ('z', 0.04 * math.sin(2.1 * t) + rng.gauss(0, 0.003)),
                       ('yaw', yaw), ('pitch', 0.01 * math.sin(1.3 * t)), ('roll', 0.02 * math.sin(0.9 * t)),
                       ('speed', v), ('cmd.v', round(v, 2)), ('cmd.w', round(w, 2))):
            cols[f'boat.{k}'].append(val)
    return cols


def test_round_trip_within_the_quantum(tmp_path):
    cols = boat_run(minutes=1.0)
    events = [{'t': 1.0, 'kind': 'gate', 'index': 0}, {'t': 2.5, 'kind': 'contact', 'id': 'b1', 'x': float('nan')}]
    sha = traj.write(tmp_path / 'r.ctraj', {'seed': 3, 'robot': {'agents': ['boat']}}, cols, events, hz=20)
    assert sha == hashlib.sha256((tmp_path / 'r.ctraj').read_bytes()).hexdigest()
    t = traj.read(tmp_path / 'r.ctraj')
    assert t.header['schema'] == traj.SCHEMA and t.ticks == 1200 and t.header['duration_s'] == 60.0
    for name, values in cols.items():
        q = traj.scale_of(name)
        assert max(abs(a - b) for a, b in zip(values, t.columns[name])) <= q / 2 + 1e-9, name
    assert t.events[1]['x'] is None and t.events[0] == {'t': 1.0, 'kind': 'gate', 'index': 0}
    assert t.header['box']['min'][0] <= min(cols['boat.x']) + 0.01
    assert t.agents() == ['boat'] and set(t.pose('boat', 5)) == set(traj.POSE6)
    assert t.times()[20] == 1.0


def test_an_empty_events_section_reads_as_no_events():
    # the Unity recorder (.NET DeflateStream) writes 0 bytes for a run without events
    assert traj._inflate(b'') == b''


def test_equal_input_gives_equal_bytes():
    cols = boat_run(minutes=0.5)
    assert traj.encode({'a': 1}, cols, hz=20) == traj.encode({'a': 1}, cols, hz=20)


def test_ten_minutes_at_20_hz_stays_under_200_kb():
    data = traj.encode({'seed': 1}, boat_run(minutes=10.0), hz=20)
    assert len(data) < 200_000, len(data)


def test_a_window_decodes_only_the_blocks_it_overlaps(tmp_path):
    cols = boat_run(minutes=2.0)  # 4 blocks of 30 s
    traj.write(tmp_path / 'r.ctraj', {}, cols, hz=20)
    w = traj.read(tmp_path / 'r.ctraj', t_from=65.0, t_to=70.0)
    assert w.header['window_first_tick'] == 1200 and len(w.columns['boat.x']) == 600
    assert abs(w.columns['boat.x'][0] - cols['boat.x'][1200]) <= 0.005


def test_amend_changes_the_header_and_keeps_the_samples(tmp_path):
    p = tmp_path / 'r.ctraj'
    traj.write(p, {'outcome': 'unknown', 'drop': 1}, boat_run(minutes=0.2), hz=20)
    before = traj.read(p).columns
    sha = traj.amend(p, outcome='finished', score=0.75, drop=None)
    t = traj.read(p)
    assert t.header['outcome'] == 'finished' and t.header['score'] == 0.75 and 'drop' not in t.header
    assert t.columns == before and sha == hashlib.sha256(p.read_bytes()).hexdigest()
    assert traj.read_header(p)['score'] == 0.75


def test_corruption_and_bad_input_are_refused(tmp_path):
    p = tmp_path / 'r.ctraj'
    traj.write(p, {}, {'a.x': [0.0, 1.0]}, hz=10)
    data = bytearray(p.read_bytes())
    data[-10] ^= 0xFF
    with pytest.raises(traj.TrajError, match='crc'):
        traj.loads(bytes(data))
    with pytest.raises(traj.TrajError, match='not a crane-traj'):
        traj.loads(b'nope' * 10)
    with pytest.raises(traj.TrajError, match='non-finite'):
        traj.encode({}, {'a.x': [0.0, float('nan')]}, hz=10)
    with pytest.raises(traj.TrajError, match='length'):
        traj.encode({}, {'a.x': [0.0], 'a.y': [0.0, 1.0]}, hz=10)
    with pytest.raises(traj.TrajError, match='kind'):
        traj.encode({}, {'a.x': [0.0]}, [{'t': 0}], hz=10)


def test_problems_finds_teleports_and_bounds(tmp_path):
    cols = {'r.x': [0.0, 0.1, 50.0, 50.1], 'r.y': [0.0] * 4}
    t = traj.loads(traj.encode({'outcome': 'exploded'}, cols, [{'t': 99.0, 'kind': 'x'}], hz=10))
    found = traj.problems(t, bounds={'min': [-1, -1, -1], 'max': [10, 10, 10]})
    assert any('jump' in p for p in found) and any('bounds' in p for p in found)
    assert any('outside the run' in p for p in found) and any('outcome' in p for p in found)
    assert traj.problems(traj.loads(traj.encode({}, {'r.x': [0.0, 0.1], 'r.y': [0.0, 0.0]}, hz=10))) == []


def test_the_golden_file_is_current():
    """golden/small.ctraj (read by the Unity test) is what this writer makes from golden_data."""
    header, cols, events, hz = golden_data.data()
    want = json.loads((golden_data.HERE / 'small.json').read_text())
    data = traj.encode(header, cols, events, hz=hz, specs=golden_data.specs(cols))
    assert (golden_data.HERE / 'small.ctraj').read_bytes() == data
    assert hashlib.sha256(data).hexdigest() == want['sha256']
    t = traj.loads(data)
    assert t.ticks == want['ticks'] and t.events == want['events']
    for i, sample in want['samples'].items():
        for k, v in sample.items():
            assert abs(t.columns[k][int(i)] - v) <= traj.scale_of(k, traj.SCALES_V1) / 2 + 1e-9


def test_job_fields_and_run_meta(tmp_path):
    job = {'job_id': 'j1', 'hypothesis_id': 'h1', 'stage_index': 2, 'arm': 'v1',
           'spec': {'metric': 'score.total', 'theater': {'study': 's1', 'insight': 'raise lookahead'},
                    'params': {'FollowPath': {'desired_linear_vel': 1.2}}}}
    f = meta.job_fields(job, metrics={'score.total': 0.5})
    assert f['study'] == 's1' and f['iteration'] == 2 and f['score'] == 0.5 and f['arm'] == 'v1'
    assert f['params'] == {'FollowPath.desired_linear_vel': 1.2} and len(f['params_sha256']) == 64
    p = tmp_path / 'r.ctraj'
    traj.write(p, {'course': {'truth_sha256': 'abc'}, 'build': {'recorder': 'unity'}}, {'r.x': [0.0]}, hz=1)
    header, sha = meta.stamp(p, outcome='finished', **f)
    m = meta.run_meta(header, traj_sha256=sha)
    assert m['course_sha256'] == 'abc' and m['recorder'] == 'unity' and m['traj_sha256'] == sha
    assert m['iteration'] == 2 and m['insight'] == 'raise lookahead' and 'job_id' not in m
    assert meta.job_fields({'job_id': 'j'})['study'] == 'adhoc'


def test_writer_scales_keep_a_ten_minute_boat_small():
    """Recommended scales (yaw 0.02, pitch/roll 0.05 deg, speed dropped) against the first writer's."""
    cols = {k: v for k, v in boat_run(minutes=10.0).items() if not k.endswith(('speed', '.cmd.v', '.cmd.w'))}
    new = traj.encode({'seed': 1}, cols, hz=20)
    old = traj.encode({'seed': 1}, cols, hz=20, specs=traj.specs_for(cols, traj.SCALES_V1))
    assert len(new) < 0.85 * len(old) and len(new) < 30_000, (len(new), len(old))
    t = traj.loads(new)
    assert t.specs[3].scale == traj.SCALES['yaw']


def test_lod_is_small_public_and_follows_the_run(tmp_path):
    cols = boat_run(minutes=10.0)
    header = {'seed': 1, 'pose_dims': '6dof', 'robot': {'agents': ['boat']}, 'params': {'secret.path': '/home/x'},
              'course': {'name': 'c', 'truth_sha256': 'abc'}, 'study': 's', 'outcome': 'finished', 'valid': True}
    events = [{'t': 12.0, 'kind': 'gate', 'index': 0}]
    full = traj.encode(header, cols, events, hz=20)
    small = traj.lod(full)
    assert len(small) < 7_000, len(small)
    t = traj.loads(small)
    assert t.hz == 2.0 and t.ticks == 1200 and t.events == events
    assert set(t.columns) == {f'boat.{k}' for k in traj.POSE6}
    assert 'params' not in t.header and 'study' not in t.header and 'seed' not in t.header
    assert t.header['course'] == {'truth_sha256': 'abc'} and t.header['outcome'] == 'finished'
    assert t.header['parent_sha256'] == hashlib.sha256(full).hexdigest()
    assert t.header['lod'] == {'of_hz': 20.0, 'hz': 2.0, 'of_ticks': 12000}
    for i in (0, 7, 1199):
        assert abs(t.columns['boat.x'][i] - cols['boat.x'][i * 10]) <= 0.026
    p = tmp_path / 'run.ctraj'
    p.write_bytes(full)
    sha = traj.write_lod(p)
    assert hashlib.sha256((tmp_path / 'run.lod.ctraj').read_bytes()).hexdigest() == sha
    planar = traj.lod(traj.encode({}, {'r.x': [0.1 * i for i in range(100)], 'r.y': [0.0] * 100,
                                       'r.yaw': [0.0] * 100, 'r.speed': [1.0] * 100}, hz=10))
    assert set(traj.loads(planar).columns) == {'r.x', 'r.y', 'r.yaw'} and traj.loads(planar).ticks == 20


def test_fleet_pack_round_trips():
    a, b = traj.lod(traj.encode({}, {'r.x': [0.0, 1.0]}, hz=2)), b'xyz'
    assert traj.fleet_unpack(traj.fleet_pack([a, b])) == [a, b]
    with pytest.raises(traj.TrajError):
        traj.fleet_unpack(traj.fleet_pack([a])[:-1])
