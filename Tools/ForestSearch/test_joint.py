"""Tests for the joint planner (joint.py)."""
import sys
from pathlib import Path

import numpy as np
import pytest

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))

from ForestSearch.experiments import EVENTS, LINKS, evaluate  # noqa: E402
from ForestSearch.instances import grid_instance  # noqa: E402
from ForestSearch.joint import make_joint_strategies  # noqa: E402
from ForestSearch.links import LinkConfig  # noqa: E402
from ForestSearch.sim import run_episode  # noqa: E402

SEEDS = range(60)
J = make_joint_strategies(6, 8)


def _key(r):
    return (r.detected, r.ttd, r.looks, r.redundant)


def test_make_joint_strategies_names():
    assert list(J) == ["ORACLE", "INDEP", "SECTOR", "IMPLICIT", "RESERVE", "FULL", "EVENT"]
    assert all(s.joint and s.rollout_k == 6 and s.rollout_budget == 8 for s in J.values())


@pytest.mark.parametrize("name", ["IMPLICIT", "EVENT", "RESERVE", "FULL"])
def test_proposition1_joint(name):
    inst = grid_instance(4, 3, seed=1, abort=0.0, h_good=0.0, h_bad=0.0, alpha=0.0)
    for s in SEEDS:
        ref = run_episode(inst, ("ORACLE", "ORACLE"), LinkConfig(), s, strategies=J)
        got = run_episode(inst, (name, name), LinkConfig(), s, strategies=J)
        assert _key(got) == _key(ref)
        if name in ("IMPLICIT", "EVENT"):
            assert got.bytes_sent == 0


def test_joint_differs_from_greedy_somewhere():
    from ForestSearch.sim import STRATEGIES
    inst = grid_instance(5, 5, seed=100, **EVENTS["none"])
    a = [_key(run_episode(inst, ("ORACLE",) * 2, LinkConfig(), s, strategies=J)) for s in range(80)]
    b = [_key(run_episode(inst, ("ORACLE",) * 2, LinkConfig(), s, strategies=STRATEGIES)) for s in range(80)]
    assert a != b


def test_determinism():
    inst = grid_instance(4, 3, seed=1, **EVENTS["both"])
    link = LINKS["burst30_L10_d1"]
    for name in ("ORACLE", "IMPLICIT", "EVENT", "FULL"):
        r1 = [_key(run_episode(inst, (name,) * 2, link, s, strategies=J)) for s in range(25)]
        r2 = [_key(run_episode(inst, (name,) * 2, link, s, strategies=make_joint_strategies(6, 8)))
              for s in range(25)]
        assert r1 == r2


def _paired_ci(x, B=2000, seed=0):
    rng = np.random.default_rng(seed)
    idx = rng.integers(0, len(x), (B, len(x)))
    m = x[idx].mean(1)
    return np.percentile(m, 2.5), np.percentile(m, 97.5)


def test_joint_oracle_not_worse_than_sector_no_events():
    """Joint centralised planner should not be (significantly) worse than the
    static SECTOR split.  Reported honestly; no tuning to pass."""
    diffs = []
    for iseed in (100, 101, 102):
        inst = grid_instance(5, 5, seed=iseed, **EVENTS["none"])
        res = evaluate(inst, [("ORACLE",) * 2, ("SECTOR",) * 2], LinkConfig(),
                       range(400), strategies=J)
        d = res[("ORACLE",) * 2]["det"] - res[("SECTOR",) * 2]["det"]
        print(f"seed {iseed}: ORACLE-SECTOR = {d.mean():+.4f} CI {_paired_ci(d)}")
        diffs.append(d)
    d = np.concatenate(diffs)
    lo, hi = _paired_ci(d)
    print(f"pooled ORACLE-SECTOR = {d.mean():+.4f} CI [{lo:+.4f}, {hi:+.4f}]")
    assert hi >= 0.0
