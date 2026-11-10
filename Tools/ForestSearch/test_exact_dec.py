"""Tests for exact_dec (exact value of communication on tiny instances)."""
import sys
from dataclasses import replace
from pathlib import Path

import numpy as np
import pytest

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))

from ForestSearch.exact import ExactTeam  # noqa: E402
from ForestSearch.exact_dec import TinyModel, brute_force_dec, pareto  # noqa: E402
from ForestSearch.links import LinkConfig  # noqa: E402
from ForestSearch.model import DETECT, Instance  # noqa: E402
from ForestSearch.sim import nominal_plan, run_episode  # noqa: E402


def _inst(hints=True, abort=0.0, n=2):
    pri = [0.6, 0.4] if n == 2 else [0.5, 0.3, 0.2]
    g = [[0.7, 0.4], [0.5, 0.6], [0.6, 0.6]][:n]
    kw = dict(h_good=0.3, h_bad=0.15, alpha=0.05) if hints else {}
    return Instance(prior=pri, g=g, q=np.full((n, 2), 0.8), abort=abort, **kw)


@pytest.mark.parametrize("hints", [False, True])
def test_vcent_matches_exact_team(hints):
    inst = _inst(hints)
    M = TinyModel(inst)
    for k in (1, 2, 3):
        assert M.v_cent(k) == pytest.approx(ExactTeam(inst).value(k), abs=1e-12)


@pytest.mark.parametrize("hints,abort", [(True, 0.0), (False, 0.2), (True, 0.2)])
def test_vdec_matches_brute_force(hints, abort):
    M = TinyModel(_inst(hints, abort))
    assert M.v_dec(2, prune=pareto) == pytest.approx(brute_force_dec(M, 2), abs=1e-12)


def test_reported_policies_attain_vdec():
    M = TinyModel(_inst(True))
    v, p0, p1, _ = M.v_dec(2, True, prune=pareto)
    assert M.pair_value(p0, p1, 2) == pytest.approx(v, abs=1e-12)


@pytest.mark.parametrize("hints,abort", [(True, 0.0), (False, 0.2), (True, 0.2)])
def test_ordering(hints, abort):
    M = TinyModel(_inst(hints, abort))
    for k in (1, 2):
        vc, vd, vi = M.v_cent(k), M.v_dec(k, prune=pareto), M.v_implicit(k)
        assert vi <= vd + 1e-12 <= vc + 2e-12
    # k = 2 counterexample: communication strictly helps with hints or aborts
    assert M.v_cent(2) - M.v_dec(2, prune=pareto) > 1e-5


def test_no_hints_no_aborts_voc_zero():
    for n in (2, 3):
        M = TinyModel(_inst(False, 0.0, n))
        for k in (1, 2, 3):
            assert M.v_dec(k, prune=pareto) == pytest.approx(M.v_cent(k), abs=1e-12)


@pytest.mark.parametrize("hints,abort", [(True, 0.0), (True, 0.2)])
def test_implicit_tree_matches_sim(hints, abort):
    """Every simulated IMPLICIT episode follows the exact policy trees."""
    k = 3
    inst = replace(_inst(hints, abort, n=3), horizon=k)
    M = TinyModel(inst)
    (p0, p1), _ = M.implicit_policies(k)
    nom = nominal_plan(inst)
    for seed in range(300):
        r = run_episode(inst, ("IMPLICIT", "IMPLICIT"), LinkConfig(), seed,
                        nominal=nom, keep_log=True)
        node = [p0, p1]
        for (t, j, c, v, out) in r.log:
            assert node[j].action == c * inst.V + v, (seed, t, j)
            if out == DETECT:
                break
            node[j] = node[j].children[out]
