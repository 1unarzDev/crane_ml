"""Property metrics follow the NLC three-valued semantics (MATH.md 1.4)."""

import numpy as np

from judge.metric import INDETERMINATE, SATISFIED, VIOLATED, Margin, Property, Status, three_valued
from judge.record import Header, Robot, RunRecord, se2_channel


def test_three_valued_bounds():
    t = np.arange(5.0)
    assert three_valued(Margin(t, np.array([1, 1, 1, 1, 1.0])))[0] == SATISFIED
    assert three_valued(Margin(t, np.array([1, 1, -0.5, 1, 1.0])))[0] == VIOLATED
    # within the localisation bound: neither satisfied nor violated
    assert three_valued(Margin(t, np.array([1, 1, 0.1, 1, 1.0]), e=np.full(5, 0.2)))[0] == INDETERMINATE
    # inter-sample bound L also blocks a SATISFIED claim
    assert three_valued(Margin(t, np.full(5, 0.05), L=np.full(5, 0.1)))[0] == INDETERMINATE


class KeepRight(Property):
    name = 'keep_right'
    requires = frozenset({'pose.se2'})

    def __init__(self, sigma):
        self.sigma = sigma

    def margin(self, rec, task):
        ch = rec.channel('pose.se2')
        return Margin(ch.t, ch.col('y'), np.full(len(ch), 2 * self.sigma))


def _rec(y):
    t = np.arange(len(y)) * 0.1
    return RunRecord(Header('r', 'real', 'test', 'boat', Robot('b')),
                     {'pose.se2': se2_channel(t, t, y, 0 * t, 'surveyed')})


def test_property_value_spans_and_verdicts():
    v = KeepRight(0.0)(_rec(np.array([1, 0.5, -0.2, -0.3, 0.4])))
    assert v.extra['verdict'] == VIOLATED and v.value == -0.3
    assert v.extra['first_violation_t'] == 0.2 and len(v.spans) == 1
    # a surveyed track whose 2-sigma covers the margin is indeterminate, not violated or safe
    v = KeepRight(0.15)(_rec(np.array([1, 0.5, 0.2, 0.25, 0.4])))
    assert v.status == Status.INDETERMINATE and v.extra['verdict'] == INDETERMINATE
