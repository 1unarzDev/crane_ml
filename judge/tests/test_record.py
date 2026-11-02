"""RunRecord: deterministic save/load (G1 hashes), unobservable channels become unscorable."""

import numpy as np

from judge import record
from judge.metric import Status
from judge.packs import generic_motion, toy_quad as tq


def test_roundtrip_and_deterministic_digest(tmp_path):
    a = tq.simulate(tq.PARAMS_LOW, 4, 'low')
    b = tq.simulate(tq.PARAMS_LOW, 4, 'low')
    pa, pb = record.save(a, tmp_path / 'a'), record.save(b, tmp_path / 'b')
    assert record.digest(pa) == record.digest(pb)
    back = record.load(pa)
    assert back.header == a.header
    assert np.allclose(back.channel('pose.xz').v, a.channel('pose.xz').v)
    assert [e.kind for e in back.events] == [e.kind for e in a.events]
    c = tq.simulate(tq.PARAMS_LOW, 5, 'low')
    assert record.digest(record.save(c, tmp_path / 'c')) != record.digest(pa)


def test_missing_channel_is_unscorable_not_zero():
    r = tq.simulate(tq.PARAMS_LOW, 1, 'low')
    v = generic_motion.make('pose.se2').metrics['motion.path_length'](r, None)
    assert v.status == Status.UNSCORABLE and v.value is None
    v = generic_motion.make('pose.xz').metrics['motion.path_length'](r, None)
    assert v.status == Status.OK and v.value > 10


def test_channel_interp_wraps_angles():
    ch = record.se2_channel([0, 1], [0, 1], [0, 0], [3.1, -3.1])
    mid = ch.at([0.5])[0]
    assert abs(abs(mid[2]) - np.pi) < 0.05
