"""Exact finite-horizon DP for tiny travel-free instances.

Setting: both searchers look synchronously once per step, no travel, no
aborts, all information shared instantly (centralised team).  Value =
P(confirmed detection within k steps).  Used to (i) bound how far the
centralised greedy reference is from optimal and (ii) compute exact values of
fixed policies without Monte Carlo error.

P(outcome tuple o | state s) = Z(s + o) / Z(s), where
Z(s) = sum_i prior_i * L(data_s | target in i).
"""
from __future__ import annotations

import itertools
from functools import lru_cache

import numpy as np

from .model import HINT, NULL, BeliefModel, Instance, _cell_log_lr


def _logZ(inst: Instance, m0: np.ndarray, m1: np.ndarray) -> float:
    bm = BeliefModel()
    lr = _cell_log_lr(inst, m0, m1, bm)
    a = inst.alpha
    # Same epsilon as _cell_log_lr so the absent terms cancel exactly.
    absent = float((m0 * np.log(1 - a)).sum() + m1.sum() * np.log(max(a, 1e-300)))
    x = np.log(inst.prior) + lr
    mx = x.max()
    if not np.isfinite(mx):
        return -np.inf
    return float(mx + np.log(np.exp(x - mx).sum()) + absent)


class ExactTeam:
    def __init__(self, inst: Instance, agents: int = 2):
        self.inst = inst
        self.n, self.V = inst.n, inst.V
        self.K = agents
        self.types = [(i, v) for i in range(self.n) for v in range(self.V)]
        self.actions = list(itertools.combinations_with_replacement(range(len(self.types)), agents))
        self.nodes = 0

    def _arrays(self, key):
        m = np.array(key, int).reshape(2, self.n, self.V)
        return m[0], m[1]

    def _outcomes(self, key, action):
        """Yield (prob, next_key) for every all-non-detect outcome tuple."""
        m0, m1 = self._arrays(key)
        lz = _logZ(self.inst, m0, m1)
        res = []
        for outs in itertools.product((NULL, HINT), repeat=len(action)):
            n0, n1 = m0.copy(), m1.copy()
            for a, o in zip(action, outs):
                i, v = self.types[a]
                if o == NULL:
                    n0[i, v] += 1
                else:
                    n1[i, v] += 1
            lzn = _logZ(self.inst, n0, n1)
            p = float(np.exp(lzn - lz)) if np.isfinite(lzn) else 0.0
            if p > 0:
                res.append((p, tuple(np.concatenate([n0.ravel(), n1.ravel()]))))
        return res

    def value(self, k: int, key=None) -> float:
        """Optimal P(detect within k steps)."""
        if key is None:
            key = tuple([0] * (2 * self.n * self.V))
        return self._opt(k, key)

    @lru_cache(maxsize=None)
    def _opt(self, k, key):
        if k == 0:
            return 0.0
        self.nodes += 1
        best = 0.0
        for act in self.actions:
            best = max(best, self._q(k, key, act, self._opt))
        return best

    def _q(self, k, key, act, cont):
        outs = self._outcomes(key, act)
        p_nd = sum(p for p, _ in outs)
        return (1.0 - p_nd) + sum(p * cont(k - 1, nk) for p, nk in outs)

    # ---- exact evaluation of the sequential-greedy centralised reference
    def greedy_action(self, key, bm: BeliefModel = BeliefModel()):
        from .model import detect_prob
        m0, m1 = self._arrays(key)
        m0 = m0.copy()
        act = []
        for _ in range(self.K):
            pd = detect_prob(self.inst, m0, m1, bm)
            k = int(np.argmax(pd))
            i, v = divmod(k, self.V)
            act.append(self.types.index((i, v)))
            m0[i, v] += 1  # virtual NULL for the next agent
        return tuple(sorted(act))

    def greedy_value(self, k: int, bm: BeliefModel = BeliefModel()) -> float:
        cache = {}

        def ev(kk, key):
            if kk == 0:
                return 0.0
            if (kk, key) in cache:
                return cache[(kk, key)]
            v = self._q(kk, key, self.greedy_action(key, bm), ev)
            cache[(kk, key)] = v
            return v
        return ev(k, tuple([0] * (2 * self.n * self.V)))
