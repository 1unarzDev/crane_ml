"""Tests for joint_silent.py (Proposition 1 and basic sanity)."""
import sys
from pathlib import Path

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))

from ForestSearch.experiments import EVENTS  # noqa: E402
from ForestSearch.instances import grid_instance  # noqa: E402
from ForestSearch.joint_silent import VARIANTS, make_silent  # noqa: E402
from ForestSearch.links import LinkConfig  # noqa: E402
from ForestSearch.sim import run_episode  # noqa: E402


def _key(r):
    return (r.detected, r.ttd, r.looks, r.redundant)


@pytest.mark.parametrize("vname", list(VARIANTS))
def test_proposition1_silent(vname):
    """No events + ideal link: silent variant == joint ORACLE, zero bytes."""
    inst = grid_instance(4, 3, seed=1, abort=0.0, h_good=0.0, h_bad=0.0, alpha=0.0)
    base, imp = make_silent(6, 8, **VARIANTS[vname])
    S = dict(base)
    S["X"] = imp
    for s in range(40):
        ref = run_episode(inst, ("ORACLE", "ORACLE"), LinkConfig(), s, strategies=S)
        got = run_episode(inst, ("X", "X"), LinkConfig(), s, strategies=S)
        assert _key(got) == _key(ref)
        assert got.bytes_sent == 0


@pytest.mark.parametrize("vname", ["A1_disc_time", "B1_mix_time", "C1_own_time"])
def test_variant_active_with_events(vname):
    """With events the variant must actually change behaviour somewhere
    (guards against a silently dead code path) and be deterministic."""
    inst = grid_instance(4, 3, seed=1, **EVENTS["both"])
    base, imp = make_silent(6, 8, **VARIANTS[vname])
    S = dict(base)
    S["X"] = imp
    a = [_key(run_episode(inst, ("X", "X"), LinkConfig(), s, strategies=S)) for s in range(60)]
    a2 = [_key(run_episode(inst, ("X", "X"), LinkConfig(), s, strategies=S)) for s in range(60)]
    b = [_key(run_episode(inst, ("IMPLICIT", "IMPLICIT"), LinkConfig(), s, strategies=S))
         for s in range(60)]
    assert a == a2
    assert a != b
