"""Cross-checks on the tiniest instances + determinism.

Run: PYTHONPATH=Tools python Tools/MissionExact/test_mission_exact.py   (or pytest)
"""
import itertools

import numpy as np

try:
    import pytest
except ImportError:          # minimal shim so the file runs as a plain script
    class _Approx:
        def __init__(self, v, abs=1e-12):
            self.v, self.abs = v, abs

        def __eq__(self, o):
            return abs(o - self.v) <= self.abs

        __req__ = __eq__

    class _Mark:
        @staticmethod
        def parametrize(names, vals):
            def deco(f):
                f._params = (names, vals)
                return f
            return deco

    class pytest:  # noqa: N801
        approx = _Approx
        mark = _Mark()

from MissionExact.model import Instance, CONF, NULL, WEAK
from MissionExact.oracle import Oracle
from MissionExact.nocomm import NoComm
from MissionExact.team import Team


def tiny(T=4, d=0, link=(1, 0, 0), alpha=0.1, starts=(0, 0)):
    return Instance(tt=[[0, 1, 1], [1, 0, 1], [1, 1, 0]], link=list(link),
                    prior=[0, 0.6, 0.4], g=[[.5, .5], [.7, .3], [.4, .8]],
                    T=T, d=d, alpha=alpha, starts=starts)


def two_node(T=4, d=0, link=(1, 0), starts=(0, 0), alpha=0.0):
    return Instance(tt=[[0, 1], [1, 0]], link=list(link), prior=[0, 1], g=[[.5, .5], [.6, .3]],
                    T=T, d=d, alpha=alpha, starts=starts)


def test_likelihood_rows_sum_to_one():
    inst = tiny()
    assert np.allclose(inst.P.sum(1), 1.0)
    assert np.isclose(inst.p.sum(), 1.0)


@pytest.mark.parametrize("T,d", [(3, 0), (4, 0), (4, 1)])
def test_full_info_team_equals_oracle(T, d):
    inst = tiny(T=T, d=d)
    v_or = Oracle(inst).value()
    v_bnb = Team(inst, "full").solve()
    assert v_bnb == pytest.approx(v_or, abs=1e-12)


def test_full_info_bruteforce_equals_oracle():
    inst = two_node(T=3, d=0, link=(1, 1), starts=(1, 1), alpha=0.1)
    assert Team(inst, "full", bnb=False).solve() == pytest.approx(Oracle(inst).value(), abs=1e-12)


@pytest.mark.parametrize("T,d", [(3, 0), (4, 0), (4, 1), (5, 1)])
def test_nocomm_failure_vectors_equal_team_search(T, d):
    inst = tiny(T=T, d=d)
    assert NoComm(inst).solve() == pytest.approx(Team(inst, "none").solve(), abs=1e-12)


@pytest.mark.parametrize("mode,inst", [
    ("none", two_node(T=4)),
    ("delayed", two_node(T=3, d=1, link=(1, 1), starts=(1, 1), alpha=0.1)),
    ("none", two_node(T=3, d=0, link=(1, 1), starts=(1, 1), alpha=0.1))])
def test_bnb_equals_bruteforce(mode, inst):
    assert Team(inst, mode, bnb=True).solve() == pytest.approx(
        Team(inst, mode, bnb=False).solve(), abs=1e-12)


def test_two_node_closed_form():
    """Single search node, one drone useful window: T=2, d=0, base->n1 = 1.
    Both drones fly 1 step and can do one FAST look ending at t=2, then the
    report needs dlink=1 more step -> r=3 > 2: nothing possible."""
    assert Oracle(two_node(T=2)).value() == 0.0
    # T=3: both fly (t=1), both look fast (t=2), report at 3.  Best: views 0 and 1.
    inst = two_node(T=3)
    p_conf = lambda g: g * inst.pc[0]
    # two different views: independent latent states
    v_diff = 1 - (1 - p_conf(.6)) * (1 - p_conf(.3))
    v_same = 1 - (0.4 + 0.6 * (1 - inst.pc[0]) ** 2)      # view 0 twice
    assert Oracle(inst).value() == pytest.approx(max(v_diff, v_same), abs=1e-12)


def test_value_ordering():
    for d in (0, 1):
        inst = tiny(T=5, d=d, link=(1, 1, 0))
        vo = Oracle(inst).value()
        vd = Team(inst, "delayed").solve()
        vn = NoComm(inst).solve()
        assert vo + 1e-12 >= vd >= vn - 1e-12


def test_all_link_zero_delay_equals_oracle():
    inst = tiny(T=4, d=0, link=(1, 1, 1))
    assert Team(inst, "delayed").solve() == pytest.approx(Oracle(inst).value(), abs=1e-12)


def test_determinism():
    inst = tiny(T=4, d=1, link=(1, 1, 0))
    a = Team(inst, "delayed"); va = a.solve()
    b = Team(inst, "delayed"); vb = b.solve()
    assert va == vb
    assert a.policy_lines(0) == b.policy_lines(0) and a.policy_lines(1) == b.policy_lines(1)
    oa, ob = Oracle(inst), Oracle(inst)
    assert oa.value() == ob.value() and oa.policy_lines() == ob.policy_lines()


if __name__ == "__main__":
    import sys, time
    fails = 0
    for name, f in list(globals().items()):
        if not name.startswith("test_") or not callable(f):
            continue
        params = getattr(f, "_params", None)
        cases = [()] if params is None else [v if isinstance(v, tuple) else (v,) for v in params[1]]
        for c in cases:
            t0 = time.time()
            try:
                f(*c)
                print(f"PASS {name}{c} {time.time()-t0:.1f}s", flush=True)
            except AssertionError as e:
                fails += 1
                print(f"FAIL {name}{c}: {e!r}", flush=True)
    print("FAILURES:", fails)
    sys.exit(1 if fails else 0)
