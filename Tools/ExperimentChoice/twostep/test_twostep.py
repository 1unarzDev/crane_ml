#!/usr/bin/env python3
"""Checks for twostep.py.  Run: nice -n 19 python3 -I test_twostep.py"""
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

import numpy as np  # noqa: E402

from twostep import Instance, Solver, random_instance  # noqa: E402

TOL = 1e-12


def close(a, b, tol=1e-12):
    return abs(a - b) <= tol


def test_hand_single_course_full_tier():
    # One course, P(A better) = 0.7, no mechanism latent, full flip prob 0.1.
    # One full query: loss = min(.7*.9,.3*.1) + min(.7*.1,.3*.9) = .03 + .07 = 0.10
    s = Solver(Instance(1, 1.0, 1, e_lo=0.2, e_hi=0.1, q=0.0, V=[],
                        prior_b=[0.3, 0.7]))
    assert close(s.prior_loss, 0.3)
    assert close(s.v_opt(0), 0.10)
    # cheap alone would give min(.56,.06) + min(.14,.24) = 0.20
    kids = s.kids(0)
    cheap = [k for k in kids if k[0] == 0][0]
    assert close(s.term[cheap[2]] + s.term[cheap[3]], 0.20)
    # Two full queries: ++ .003, +- twice .027 each, -- .007  => 0.064
    s2 = Solver(Instance(1, 1.0, 2, e_lo=0.5, e_hi=0.1, q=0.0, V=[],
                         prior_b=[0.3, 0.7]))
    assert close(s2.v_opt(0), 0.064)
    assert close(s2.best_fixed()[0], 0.064)


def test_hand_misleading_cheap_has_zero_value_alone():
    # One course of type V, q = 0.5, noiseless tiers.  The cheap sign is
    # equally likely to be right or inverted, so alone it is worthless
    # (loss stays 0.3); one full query resolves everything.
    inst = Instance(1, 0.001, 0.001, e_lo=0.0, e_hi=0.0, q=0.5, V=[0],
                    prior_b=[0.3, 0.7])
    s = Solver(inst)
    assert close(s.v_opt(0), 0.3)
    s = Solver(Instance(1, 0.001, 1.0, 0.0, 0.0, 0.5, V=[0], prior_b=[0.3, 0.7]))
    assert close(s.v_opt(0), 0.0)


def test_hand_two_course_boundary():
    # Two courses, uniform boundary b in {0,1,2}, noiseless full tier, one
    # full query, p uniform, mag 1.  Prior: P(+ at 0)=2/3, P(+ at 1)=1/3,
    # prior loss = .5*(1/3) + .5*(1/3) = 1/3.
    # full@0: '+' (2/3): b in {1,2}, course 1 is 50/50 -> loss .5*.5 = .25;
    #         '-' (1/3): b=0, all known -> 0.   Expected = 1/6.
    s = Solver(Instance(2, 1.0, 1, 0.0, 0.0, 0.0, V=[]))
    assert close(s.prior_loss, 1 / 3)
    assert close(s.v_opt(0), 1 / 6)
    # Two full queries identify b exactly.
    s = Solver(Instance(2, 1.0, 2, 0.0, 0.0, 0.0, V=[]))
    assert close(s.v_opt(0), 0.0)


def test_optimal_dominates_and_one_query_greedy():
    rng = np.random.default_rng(7)
    for i in range(60):
        meta, inst = random_instance(rng)
        if inst.n == 5:
            continue
        s = Solver(inst)
        r = s.all_values()
        for k, v in r.items():
            if k in ("prior", "opt"):
                continue
            assert r["opt"] <= v + TOL, (k, r, inst.label())
        assert r["opt"] <= r["prior"] + TOL
        # rollout improvement: lookahead with greedy rollout never loses to greedy
        assert r["look1"] <= r["greedy"] + TOL
        # adaptive optimum is at least as good as the best fixed design
        assert r["opt"] <= r["fixed"] + TOL
        # screen-then-optimal is a valid policy
        assert r["opt"] <= s.evaluate("so", s.act_screen_then(s.act_opt)) + TOL

        # budget of exactly one query (equal costs so "per unit cost" is moot)
        one = Instance(inst.n, 1.0, 1, inst.e_lo, inst.e_hi, inst.q, V=inst.V,
                       prior_b=inst.prior_b, p=inst.p, mag=inst.mag)
        s1 = Solver(one)
        g = s1.evaluate("greedy", s1.act_greedy)
        assert close(g, s1.v_opt(0)), (g, s1.v_opt(0), one.label())
        assert close(s1.best_fixed()[0], s1.v_opt(0))


def test_beliefs_are_probabilities():
    # Unnormalised weights over the outcome leaves of any fixed plan sum to 1.
    rng = np.random.default_rng(3)
    meta, inst = random_instance(rng)
    s = Solver(inst)
    stack, total = [0], 0.0
    while stack:
        st = stack.pop()
        k = s.kids(st)
        if not k:
            total += s.mass[st]
        else:
            stack += [k[0][2], k[0][3]]
    assert close(total, 1.0, 1e-9)


if __name__ == "__main__":
    n = 0
    for name, fn in sorted(globals().items()):
        if name.startswith("test_") and callable(fn):
            fn()
            n += 1
            print("ok", name)
    print("%d tests passed" % n)
