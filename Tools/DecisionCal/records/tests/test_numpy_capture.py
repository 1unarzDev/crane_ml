"""Fast-layer writer: bit-identity with mission.run, event-derived outcome == mission metrics, stamp and
delay semantics, completeness, determinism, conversion."""

import hashlib
import math

import numpy as np
import pytest

from records import _common as C
from records import _compat
from records import numpy_capture as N

import mission
import unity_check as U

# (stratum, boat index, controller, noise seed, projection): includes failing docks (transient c00 hidden)
CASES = [('steady', 0, 8, 1, 'hidden'), ('steady', 0, 0, 1, 'hidden'), ('transient', 0, 0, 1, 'hidden'),
         ('transient', 0, 8, 1, 'hidden'), ('transient', 0, 0, 1, 'representable'), ('timing', 0, 8, 2, 'hidden'),
         ('mixed', 0, 5, 1, 'hidden')]


@pytest.mark.parametrize('case', CASES, ids=lambda c: f'{c[0]}{c[1]}-c{c[2]:02d}-s{c[3]}-{c[4]}')
def test_bit_identical_and_events_match(case):
    st, i, c, s, pr = case
    spec = N.boat(st, i, 4242, pr)
    ref, _ = mission.run(spec, [c], [s])
    rec, m = N.capture(st, i, 4242, c, s, projection=pr)
    for k in ref:                                             # every metric, bit for bit (J included)
        assert np.array_equal(np.asarray(ref[k]), np.asarray(m[k]), equal_nan=True), k
    out = rec['header']['provenance']['outcome_from_events']
    C.assert_outcome_matches(out, ref)                        # docked, t_dock, contacts, gate_miss, J exact
    kinds = [e['kind'] for e in rec['events']]
    assert kinds[0] == 'episode_start'
    assert kinds.count('contact') == int(ref['contacts'][0])
    assert ('docked' in kinds) == bool(ref['docked'][0])
    assert ('timeout' in kinds) == (not ref['docked'][0])
    assert rec['header']['window'][2] == ('docked' if ref['docked'][0] else 'timeout')


def test_failing_case_present():
    ref, _ = mission.run(N.boat('transient', 0, 4242), [0], [1])
    assert ref['docked'][0] == 0.0                         # the suite really covers a failed dock


def _quiet_timing_boat():
    spec = U.get_boat(4242, 'timing', 0)
    z = np.zeros(1)
    return spec.with_(pos_sd=z, vel_sd=z.copy(), yaw_sd=z.copy(), gyro_sd=z.copy())


def test_stamps_and_delay_lines_are_what_the_plant_did():
    spec = _quiet_timing_boat()
    rec, _ = N.capture('timing', 0, 4242, 8, 1, spec=spec, t_max=20.0)
    ch = rec['channels']
    L = rec['header']['provenance']['latency_ticks']
    D = rec['header']['provenance']['dead_time_ticks']
    assert L >= 2 and D >= 2                                 # the timing boat really has both delays
    tp = ch['truth.pose']
    tw = ch['truth.twist']
    ok = ch['obs.pose']['tick']
    acq = np.maximum(ok - L, 0)
    # noiseless sensors: what the controller received at tick k is the truth at tick k - L
    assert np.array_equal(ch['obs.pose']['v'], tp['v'][acq])
    assert np.array_equal(ch['obs.twist']['v'], tw['v'][acq])
    assert np.array_equal(ch['obs.stamp']['v'][:, 0], tp['t'][acq])
    assert np.array_equal(ch['obs.stamp']['v'][:, 1], tp['t'][ok])
    # cmd.applied at tick k = the request held at tick k - D (zero before D ticks)
    req = ch['cmd.requested']
    held = np.zeros((len(tp['t']) - 1, 4))
    for j, k in enumerate(req['tick']):
        held[k:] = np.clip(req['v'][j], -1, 1)
    app = ch['cmd.applied']
    want = np.vstack([np.zeros((D, 4)), held[:-D]])
    assert np.array_equal(app['v'][:, :4], want)
    assert np.array_equal(app['v'][:, 4], app['t'])


def test_channels_complete_and_obs_not_truth():
    rec, _ = N.capture('steady', 0, 4242, 8, 1, t_max=10.0)
    assert set(rec['channels']) | set(rec['absent']) >= set(C.STANDARD)
    assert set(rec['absent']) == {'cmd.sent'}
    for b in _compat.V2_HEADER_BLOCKS:
        assert b in rec['header'], b
    ch = rec['channels']
    assert not np.array_equal(ch['obs.pose']['v'], ch['truth.pose']['v'][ch['obs.pose']['tick']])
    h = rec['header']
    assert h['clock_mode'] == {'mode': 'fixed_step', 'dt_s': 0.02, 'time_scale': None, 'rtf': None, 'episode': 0}
    assert h['reset']['method'] == 'construct.rest' and h['reset']['verified'] is True
    assert h['calibration'] == {'twin': None, 'arm': None, 'evidence_sha': None, 'lineage': []}
    assert 'thrust.lag.up_down' in h['backend']['mechanisms'] or 'thrust.lag.first_order' in h['backend']['mechanisms']
    assert h['run_id'].startswith('fast-dock-steady0-h4242-c08-s1-')


def test_representable_drops_mechanisms():
    a, _ = N.capture('transient', 0, 4242, 8, 1, t_max=2.0)
    b, _ = N.capture('transient', 0, 4242, 8, 1, t_max=2.0, projection='representable')
    ma, mb = set(a['header']['backend']['mechanisms']), set(b['header']['backend']['mechanisms'])
    for gone in ('thrust.reversal_hold', 'thrust.slew', 'thrust.exponent', 'current.ou', 'wind.gust'):
        assert gone in ma and gone not in mb, gone
    assert a['header']['scenario']['id'] == b['header']['scenario']['id']      # same scenario, different backend
    assert a['header']['run_id'] != b['header']['run_id']


def _digest(files):
    h = hashlib.sha256()
    for f in sorted(files):
        h.update(f.read_bytes())
    return h.hexdigest()


def test_deterministic_save_and_v1_conversion(tmp_path):
    r1, _ = N.capture('steady', 0, 4242, 8, 1, t_max=8.0)
    r2, _ = N.capture('steady', 0, 4242, 8, 1, t_max=8.0)
    f1 = _compat.save_intermediate(r1, tmp_path / 'a')
    f2 = _compat.save_intermediate(r2, tmp_path / 'b')
    assert _digest(f1) == _digest(f2)
    back = _compat.load_intermediate(tmp_path / 'a')
    for n, c in r1['channels'].items():
        assert np.array_equal(back['channels'][n]['v'], c['v'])
        assert np.array_equal(back['channels'][n]['tick'], c['tick'])
    import judge.record as JR
    _compat.convert(tmp_path / 'a', tmp_path / 'v1', target='v1')
    jr = JR.load(tmp_path / 'v1')
    assert jr.has('truth.pose') and not jr.has('cmd.sent')
    assert jr.channels['cmd.sent'].spec.observability == 'absent'
    assert jr.header.provenance['v2']['backend']['layer'] == 'fast'
    assert np.array_equal(jr.channel('truth.pose').v, r1['channels']['truth.pose']['v'])
    _compat.convert(tmp_path / 'b', tmp_path / 'v1b', target='v1')
    assert JR.digest(tmp_path / 'v1') == JR.digest(tmp_path / 'v1b')
    assert math.isclose(jr.header.window[1], 8.0, abs_tol=1e-9)


def test_v2_save_load_validity_and_pack(tmp_path):
    """The schema owner's /2 constructors are present: real record files, judge.validity valid, the
    dock pack's own truth pass agrees with the stored events."""
    import judge.record as JR
    import judge.validity as V
    import judge.packs.dock_mission as D
    assert _compat.SCHEMA_PATH == 'v2'
    rec, m = N.capture('transient', 0, 4242, 0, 1)
    path, files = _compat.save(rec, tmp_path / 'r')
    assert path == 'v2' and {f.name for f in files} == {'record.json', 'channels.npz'}
    jr = JR.load(tmp_path / 'r')
    assert jr.header.schema == JR.SCHEMA_V2 and jr.header.backend.layer == 'fast'
    assert JR.declared_absent(jr) == {'cmd.sent': N.ABSENT['cmd.sent']}
    assert np.array_equal(jr.channel('obs.stamp').tick, rec['channels']['obs.stamp']['tick'])
    rep = V.check(jr)
    assert rep.valid, rep.as_dict()
    f = D.facts(jr)
    assert f['docked'] is False and len(f['contacts']) == int(m['contacts'][0])
    assert D.event_check(jr)['disagreements'] == []
    rec2, _ = N.capture('transient', 0, 4242, 0, 1)
    _compat.save(rec2, tmp_path / 'r2')
    assert JR.digest(tmp_path / 'r') == JR.digest(tmp_path / 'r2')


def test_cli_writes_run_dir_and_manifest(tmp_path):
    import json
    from records import capture
    done = capture.main(['numpy', '--stratum', 'steady', '--controller', '8', '--seed', '1', '--t-max', '4',
                         '--projection', 'both', '--out', str(tmp_path)])
    assert len(done) == 2
    for run_dir, man in done:
        assert run_dir.name == man['run_id'] and run_dir.name.startswith('fast-dock-steady0-h4242-c08-s1-')
        m = json.loads((run_dir / 'manifest.json').read_text())
        assert set(m['files']) == {'record.json', 'channels.npz'} and m['schema_path'] == 'v2'
        for f, sha in m['files'].items():
            assert hashlib.sha256((run_dir / f).read_bytes()).hexdigest() == sha
    assert done[0][0] != done[1][0]
    capture.main(['index', str(tmp_path)])
    assert len(json.loads((tmp_path / 'INDEX.json').read_text())) == 2


def test_intermediate_converts_to_the_same_v2_bytes(tmp_path):
    """A capture written without /2 (intermediate) converts to byte-identical /2 files."""
    import judge.record as JR
    rec, _ = N.capture('timing', 0, 4242, 8, 2, t_max=6.0)
    _compat.save(rec, tmp_path / 'direct')
    _compat.save_intermediate(rec, tmp_path / 'mid')
    _compat.convert(tmp_path / 'mid', target='v2')
    assert JR.digest(tmp_path / 'direct') == JR.digest(tmp_path / 'mid')
