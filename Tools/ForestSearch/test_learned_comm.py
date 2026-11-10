"""Tests for learned_comm.py (D1, DEV only)."""
import sys
from pathlib import Path

import numpy as np
import pytest

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))

from ForestSearch import learned_comm as L  # noqa: E402
from ForestSearch.nbeacon import HZ, run_nb  # noqa: E402
from ForestSearch.nscale import LINKS, build, fault_spec, make_strategies, nominal_plan_ns, run_ns  # noqa: E402

S = make_strategies("greedy")
F = ("detected", "ttd", "bytes_sent", "collisions")


def key(r):
    return tuple(getattr(r, f) for f in F)


def setup(N, travel, fa, seed=2000):
    inst = build(N, seed, 1, travel, HZ[(N, travel)])
    return inst, nominal_plan_ns(inst, N, S), fault_spec(fa, inst.horizon, N)


CASES = [(N, tr, fa, lk) for N in (2, 3, 5) for tr in (0, 1) for fa in ("none", "perm1")
         for lk in ("ideal", "burst30_d1")]


@pytest.mark.parametrize("N,tr,fa,lk", CASES)
def test_silent_equals_implicit(N, tr, fa, lk):
    inst, nom, fs = setup(N, tr, fa)
    for ep in range(4):
        a = L.make_ep(inst, N, lk, ep, nom, fs).run()
        b = run_ns(inst, N, "IMPLICIT", LINKS[lk], ep, S, nom, fs)
        assert key(a) == key(b) and key(a)[2] == 0


@pytest.mark.parametrize("N,tr,fa,lk", CASES)
def test_beacon_once_equals_health_part(N, tr, fa, lk):
    inst, nom, fs = setup(N, tr, fa)
    for ep in range(4):
        a = L.make_ep(inst, N, lk, ep, nom, fs, policy=L.pol_rule("BEACON_ONCE")).run()
        b = run_nb(inst, N, "HEALTH_PART", LINKS[lk], ep, S, nom, fs)
        assert key(a) == key(b)


@pytest.mark.parametrize("N", (2, 3, 5))
def test_always_healthy_equals_hybrid_p(N):
    inst, nom, fs = setup(N, 1, "none")
    for lk in ("ideal", "burst30_d1"):
        for ep in range(4):
            a = L.make_ep(inst, N, lk, ep, nom, fs, policy=L.pol_rule("ALWAYS")).run()
            b = run_ns(inst, N, "HYBRID_P", LINKS[lk], ep, S, nom, fs)
            assert (a.detected, a.ttd) == (b.detected, b.ttd)


def test_force_silent_everywhere_equals_silent_when_behaviour_off():
    inst, nom, fs = setup(3, 1, "perm1")
    bu = np.ones(4096)          # behaviour never sends
    for ep in range(4):
        a = L.make_ep(inst, 3, "burst30_d1", ep, nom, fs, behav=bu, force=(10 ** 6, True)).run()
        b = L.make_ep(inst, 3, "burst30_d1", ep, nom, fs).run()
        assert key(a) == key(b)


def test_labels_deterministic_and_paired():
    cells = [dict(N=3, travel=1, lam=1, link="burst30_d1", fault="perm1")]
    import tempfile, os
    d = tempfile.mkdtemp()
    for i in range(2):
        L.gen_labels(cells, (2000,), range(3), dict(rec=1, claim=1, beacon=2), os.path.join(d, f"{i}.npz"))
    a, b = np.load(os.path.join(d, "0.npz")), np.load(os.path.join(d, "1.npz"))
    assert np.array_equal(a["y"], b["y"]) and np.array_equal(a["X"], b["X"])
    assert set(np.unique(a["y"])) <= {-1, 0, 1}


def test_beacon_opportunity_only_when_dead():
    inst, nom, fs = setup(5, 1, "perm1")
    e = L.make_ep(inst, 5, "ideal", 0, nom, fs, collect=True)
    e.run()
    for (n, kind, t, ag, f) in e.points:
        if kind == "beacon":
            assert e.t_dead[ag] is not None and e.t_dead[ag] <= t and f[1] == 1.0


def test_seed_guard():
    for s in (3000, 4015, 5000, 5031, 6007):
        with pytest.raises(AssertionError):
            L._check_seeds([s])
    L._check_seeds(L.LABEL_SEEDS + L.VAL_SEEDS + L.EVAL_SEEDS)


def test_logit_learns_planted_signal():
    rng = np.random.default_rng(0)
    X = rng.normal(size=(4000, L.NF)); X[:, 0] = 1
    y = (rng.random(4000) < 1 / (1 + np.exp(-(2 * X[:, 3] - 2)))).astype(float)
    m = L.Logit(1e-3).fit(X, y)
    p = m.predict(X)
    assert np.corrcoef(p, X[:, 3])[0, 1] > 0.8


def test_mlp_fits_xor_like():
    rng = np.random.default_rng(1)
    X = rng.normal(size=(3000, L.NF)); X[:, 0] = 1
    y = np.sign(X[:, 1] * X[:, 2])
    m = L.MLP(epochs=80).fit(X, y)
    assert np.mean(np.sign(m.predict(X)) == y) > 0.8


@pytest.mark.xfail(strict=True, reason="negative control: ALWAYS gate must differ from silent under loss")
def test_always_equals_silent_mutant():
    inst, nom, fs = setup(5, 1, "perm1")
    outs = [(key(L.make_ep(inst, 5, "burst30_d1", ep, nom, fs, policy=L.pol_rule("ALWAYS")).run()),
             key(L.make_ep(inst, 5, "burst30_d1", ep, nom, fs).run())) for ep in range(6)]
    assert all(a == b for a, b in outs)


def test_mv_policy_deterministic_and_only_beacon():
    inst, nom, fs = setup(3, 1, "perm1")
    cache = {}
    outs = []
    for _ in range(2):
        pol = L.MVPolicy(0.0, R=2, cache=cache)
        e = L.make_ep(inst, 3, "ideal", 1, nom, fs, policy=pol)
        r = e.run()
        outs.append(key(r))
        assert e.sent_by_kind["rec"] == 0 and e.sent_by_kind["claim"] == 0
        assert e.sent_by_kind["beacon"] <= 1
    assert outs[0] == outs[1]


def test_mv_infinite_price_equals_implicit():
    inst, nom, fs = setup(3, 1, "perm1")
    for ep in range(3):
        a = L.make_ep(inst, 3, "burst30_d1", ep, nom, fs, policy=L.MVPolicy(1.0, R=1)).run()
        b = run_ns(inst, 3, "IMPLICIT", LINKS["burst30_d1"], ep, S, nom, fs)
        assert (a.detected, a.ttd, a.bytes_sent) == (b.detected, b.ttd, 0)


def test_resample_changes_future_but_not_prefix():
    inst, nom, fs = setup(3, 1, "perm1")
    bu = np.ones(4096)
    base = L.make_ep(inst, 3, "ideal", 2, nom, fs, behav=bu, force=(10 ** 6, False), collect=True)
    base.run()
    n = next(p[0] for p in base.points if p[1] == "beacon")
    a = L.make_ep(inst, 3, "ideal", 2, nom, fs, behav=bu, force=(n, False), resample=(n, 5), collect=True)
    a.run()
    assert [p[:4] for p in a.points[:n]] == [p[:4] for p in base.points[:n]]


def test_critic_dq_shapes():
    rng = np.random.default_rng(0)
    X = rng.normal(size=(500, L.NF + 1)); X[:, 0] = 1
    y = (X[:, -1] > 0).astype(float)
    c = L.CriticDQ(L.MLP(epochs=15).fit(X, y))
    assert c.predict(X[:5, :-1]).shape == (5,)
