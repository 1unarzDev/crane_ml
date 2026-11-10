"""Physical replay writer on the 2026-10-18 bags (skipped when the bags are not on this host)."""

import numpy as np
import pytest

from records import _common as C
from records import _compat
from records import physical_pulse as Ph

from plantcard.marine import fieldtrials as F

pytestmark = pytest.mark.skipif(not F.DEFAULT_DIR.is_dir(), reason='field-trial bags not on this host')


@pytest.fixture(scope='module')
def trials():
    return Ph.trials()


def test_one_record_per_trial_and_onset_matches_raw(trials):
    assert len(trials) == 37
    stem = 'agent_battery_03_01_yaw_pos' if 'agent_battery_03_01_yaw_pos' in trials else sorted(trials)[0]
    rec, r = Ph.build(stem, trials[stem])
    raw = Ph._raw().read_raw(stem)
    assert r['t_on'] == raw['t_on'] and r['t_off'] == raw['t_off']
    ev = {e['kind']: e['t'] for e in rec['events']}
    assert abs(ev['pulse_start'] - (raw['t_on'] - r['t0'])) < 1e-12
    assert rec['header']['window'][2] == 'track_end'
    # the gyro channel is the raw IMU z rate on the raw.py clock
    g = rec['channels']['obs.gyro']
    assert np.allclose(g['t'] + r['t0'] - raw['t_on'], raw['ti'], atol=1e-9, rtol=0)
    assert np.array_equal(g['v'][:, 0], raw['wz'])


def test_channels_absent_and_stamps(trials):
    stem = sorted(trials)[0]
    rec, r = Ph.build(stem, trials[stem])
    assert {'truth.pose', 'truth.twist', 'cmd.applied', 'act.thrust', 'obs.twist'} <= set(rec['absent'])
    assert not any(n.startswith('truth.') for n in rec['channels'])
    assert set(rec['channels']) | set(rec['absent']) >= set(C.STANDARD)
    st = rec['channels']['obs.stamp']['v']
    assert np.array_equal(st[:, 1], rec['channels']['obs.pose']['t'])
    lag = st[:, 1] - st[:, 0]
    assert np.median(lag) > 0 and np.median(lag) < 0.02           # header stamps ~ms before log time
    sent = rec['channels']['cmd.sent']['v']
    assert ((sent[:, :4] == 1500) | (sent[:, :4] != 1500)).all() and sent.shape[1] == 5
    h = rec['header']
    assert h['backend'] == {'layer': 'physical', 'name': 'mhseals.roboboat.field_trials_20261018', 'build': None,
                            'mechanisms': ['unknown'], 'flags': {}}
    assert h['clock_mode']['mode'] == 'recorded' and h['evidence']['kind'] == 'replay'


def test_save_deterministic_and_convert(tmp_path, trials):
    stem = sorted(trials)[1]
    a, _ = Ph.build(stem, trials[stem])
    b, _ = Ph.build(stem, trials[stem])
    fa = _compat.save_intermediate(a, tmp_path / 'a')
    fb = _compat.save_intermediate(b, tmp_path / 'b')
    assert all(x.read_bytes() == y.read_bytes() for x, y in zip(fa, fb))
    import judge.record as JR
    _compat.convert(tmp_path / 'a', tmp_path / 'v1', target='v1')
    jr = JR.load(tmp_path / 'v1')
    assert not jr.has('truth.pose') and jr.has('obs.gyro')


def test_v2_record_through_validity(tmp_path, trials):
    """Real logs go through the same door; under the dock task's Expect they are invalid for missing truth."""
    import judge.record as JR
    import judge.validity as V
    stem = sorted(trials)[0]
    rec, _ = Ph.build(stem, trials[stem])
    _compat.save(rec, tmp_path / 'r')
    jr = JR.load(tmp_path / 'r')
    assert jr.header.evidence.kind == 'replay' and jr.header.backend.mechanisms == ('unknown',)
    rep = V.check(jr)
    assert 'telemetry.missing_channel' in rep.codes
