"""Tests for theory_extra (VoC landscape, health-beacon Prop. 3, bounds, PBP)."""
import itertools
import sys
from pathlib import Path

import numpy as np
import pytest

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))

from ForestSearch.exact_dec import (Policy, TinyModel, base_instance,  # noqa: E402
                                    brute_force_dec, pareto)
from ForestSearch.theory_extra import (base2, beacon_dec, beacon_pair_value,  # noqa: E402
                                       best_response, centralised_to_beacon_pair,
                                       pbp_check, reviewer_instance, v_cent_fast,
                                       v_open_loop)


@pytest.mark.parametrize("hints,ab", [("none", 0.0), ("weak", 0.0), ("strong", 0.2), ("none", 0.2)])
def test_vcent_fast_matches_exact_dec(hints, ab):
    M = TinyModel(base_instance(2, hints, ab))
    for k in (1, 2, 3):
        assert v_cent_fast(M, k) == pytest.approx(M.v_cent(k), abs=1e-12)


def test_reviewer_maximum_reproduced():
    M = TinyModel(reviewer_instance())
    voc = v_cent_fast(M, 2) - brute_force_dec(M, 2)
    assert voc == pytest.approx(0.0820765, abs=1e-6)
    assert voc == pytest.approx(v_cent_fast(M, 2) - M.v_dec(2, prune=pareto), abs=1e-12)


@pytest.mark.parametrize("inst", [base_instance(2, "weak", 0.2), reviewer_instance(),
                                  base_instance(3, "strong", 0.0)])
def test_sandwich(inst):
    M = TinyModel(inst)
    for k in (1, 2):
        vol, vd, vc = v_open_loop(M, k), M.v_dec(k, prune=pareto), v_cent_fast(M, k)
        assert vol <= vd + 1e-12 <= vc + 2e-12


def test_uninformative_hints_give_zero_voc():
    # h_g = h_b = alpha: P(hint | non-detect) is the same under every truth
    M = TinyModel(base2(0.3, 0.3, 0.3, 0.0))
    assert v_cent_fast(M, 2) - M.v_dec(2, prune=pareto) == pytest.approx(0, abs=1e-12)


def test_best_response_is_exact():
    M = TinyModel(base_instance(2, "weak", 0.2))
    k = 2
    (p0, p1), _ = M.implicit_policies(k)
    w = M.p * M.policy_fail(p1, k)
    m = len(M.outs)
    best = np.inf
    for a0 in range(M.A):
        for acts in itertools.product(range(M.A), repeat=m):
            pol = Policy(a0, {o: Policy(a, {oo: None for oo in M.outs})
                              for o, a in zip(M.outs, acts)})
            best = min(best, float((w * M.policy_fail(pol, k)).sum()))
    assert best_response(M, w, k)[0] == pytest.approx(best, abs=1e-14)


def test_pbp_values_bounded_by_vdec():
    M = TinyModel(base_instance(2, "none", 0.2))
    (p0, p1), _ = M.implicit_policies(2)
    r = pbp_check(M, 2, p0, p1)
    assert r["v"] <= r["v_pbp"] + 1e-12 <= M.v_dec(2, prune=pareto) + 2e-12


def test_beacon_death_zero_reduces_to_exact_dec():
    M = TinyModel(base_instance(2, "weak", 0.0))
    assert v_cent_fast(M, 2, 0.0) == pytest.approx(M.v_cent(2), abs=1e-12)
    assert beacon_dec(M, 2, 0.0, 0.0, False)[0] == pytest.approx(M.v_dec(2, prune=pareto), abs=1e-12)


@pytest.mark.parametrize("n", [2, 3])
def test_prop3_ideal_frame_attains_vcent(n):
    M = TinyModel(base_instance(n, "none", 0.0))
    vc = v_cent_fast(M, 2, 0.2)
    assert beacon_dec(M, 2, 0.2, 0.0, True)[0] == pytest.approx(vc, abs=1e-12)
    assert vc - beacon_dec(M, 2, 0.2, 0.0, False)[0] > 1e-4      # no frame: VoC > 0
    assert vc - beacon_dec(M, 2, 0.2, 0.5, True)[0] > 1e-5       # lossy frame: VoC > 0


@pytest.mark.parametrize("k", [2, 3, 4])
def test_prop3_construction(k):
    M = TinyModel(base_instance(2, "none", 0.0))
    vc, pols = centralised_to_beacon_pair(M, k, 0.2)
    assert beacon_pair_value(M, k, 0.2, 0.0, pols[0], pols[1]) == pytest.approx(vc, abs=1e-12)


def test_prop3_fails_with_hints():
    M = TinyModel(base_instance(2, "strong", 0.0))
    assert v_cent_fast(M, 2, 0.2) - beacon_dec(M, 2, 0.2, 0.0, True)[0] > 1e-5
