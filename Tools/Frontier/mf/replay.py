# SPDX-License-Identifier: GPL-3.0-or-later
"""Replay simulator and selectors for multi-fidelity experiment selection (D4).

A selector "queries" (config, fidelity) pairs. The environment pays the cost of the fidelity (seconds per query, from
Tools/Frontier/costs.jsonl through costs.cost_per_unit) and returns the next *cached* outcome of that (config, fidelity) pool,
without replacement: nothing is simulated at query time. An exhausted pool returns None and charges nothing.

Regret: simple regret at L2 of the selector's current recommendation = max_x f2(x) - f2(rec) where f2 is the truth of the table
(for synthetic tables, the generating mean; for empirical tables, the pool mean). Reported against cumulative compute-hours.

Selectors: Random, AllHigh, LowOnly (cheap tier at face value), SuccessiveHalving (across fidelities), ScreenTopK (full cheap
screen, then spend the expensive tier on the top k: the strong baseline from the experiment-choice study when the cheap tier is
nearly free), AR1 (cost-aware two-fidelity Bayesian linear model, f2 = rho f1 + delta, knowledge-gradient per second).
"""
import math

import numpy as np

HI, LO = "L2", "L1"


class Env:
    def __init__(self, table, costs, rng):
        self.t, self.cost, self.rng = table, costs, rng
        self.spent = 0.0
        self.used = {k: 0 for k in table.pools}
        self.order = {k: rng.permutation(len(v)) for k, v in table.pools.items()}

    def remaining(self, i, fid):
        return len(self.t.pools[(i, fid)]) - self.used[(i, fid)]

    def query(self, i, fid):
        k = (i, fid)
        if self.remaining(i, fid) <= 0:
            return None
        y = float(self.t.pools[k][self.order[k][self.used[k]]])
        self.used[k] += 1
        self.spent += self.cost[fid]
        return y

    def regret(self, rec):
        return float(self.t.f2.max() - self.t.f2[rec])


class Stats:
    """Running sums per (config, fidelity)."""

    def __init__(self, N):
        self.n = {f: np.zeros(N) for f in (LO, HI)}
        self.s = {f: np.zeros(N) for f in (LO, HI)}
        self.ss = {f: np.zeros(N) for f in (LO, HI)}

    def add(self, i, fid, y):
        self.n[fid][i] += 1
        self.s[fid][i] += y
        self.ss[fid][i] += y * y

    def mean(self, fid):
        with np.errstate(invalid="ignore", divide="ignore"):
            return np.where(self.n[fid] > 0, self.s[fid] / np.maximum(self.n[fid], 1), -np.inf)

    def pooled_var(self, fid, default=1.0):
        dof = np.maximum(self.n[fid] - 1, 0).sum()
        if dof < 5:
            return default
        return float(max((self.ss[fid] - self.n[fid] * np.where(self.n[fid] > 0, self.s[fid] / np.maximum(self.n[fid], 1), 0) ** 2).sum() / dof, 1e-3))


class Selector:
    name = "base"

    def __init__(self, N, rng, features=None):
        self.N, self.rng, self.X = N, rng, features
        self.st = Stats(N)
        self.rec = 0
        self.done = False

    def obs(self, env, i, fid):
        y = env.query(i, fid)
        if y is not None:
            self.st.add(i, fid, y)
        return y

    def step(self, env):
        raise NotImplementedError


def run(selector, env, budget_s, checkpoints_h):
    """Drive a selector until the budget is spent, the pools are exhausted, or it declares done. Returns regret at checkpoints."""
    cps = sorted(checkpoints_h)
    out, ci = {}, 0
    last_rec_regret = env.regret(selector.rec)
    stall = 0
    while not selector.done and env.spent < budget_s:
        before = env.spent
        selector.step(env)
        if env.spent == before:
            stall += 1
            if stall > 3:
                break
        else:
            stall = 0
        last_rec_regret = env.regret(selector.rec)
        while ci < len(cps) and env.spent >= cps[ci] * 3600:
            out[cps[ci]] = last_rec_regret
            ci += 1
    while ci < len(cps):
        out[cps[ci]] = last_rec_regret
        ci += 1
    return out, env.spent / 3600


# ------------------------------------------------------------------ baselines

class Random(Selector):
    name = "random"

    def step(self, env):
        for _ in range(50):
            i = int(self.rng.integers(self.N))
            if env.remaining(i, HI) > 0:
                self.obs(env, i, HI)
                m = self.st.mean(HI)
                self.rec = int(np.argmax(m))
                return
        self.done = True


class AllHigh(Selector):
    name = "all-high"

    def __init__(self, *a, **k):
        super().__init__(*a, **k)
        self.perm = self.rng.permutation(self.N)
        self.k = 0

    def step(self, env):
        if self.k >= 40 * self.N:
            self.done = True
            return
        i = int(self.perm[self.k % self.N])
        self.k += 1
        self.obs(env, i, HI)
        self.rec = int(np.argmax(self.st.mean(HI)))


class LowOnly(Selector):
    name = "low-only"

    def __init__(self, *a, **k):
        super().__init__(*a, **k)
        self.perm = self.rng.permutation(self.N)
        self.k = 0

    def step(self, env):
        if self.k >= 400 * self.N:
            self.done = True
            return
        i = int(self.perm[self.k % self.N])
        self.k += 1
        if env.remaining(i, LO) == 0:
            self.done = all(env.remaining(j, LO) == 0 for j in range(self.N))
            return
        self.obs(env, i, LO)
        self.rec = int(np.argmax(self.st.mean(LO)))


class SuccessiveHalving(Selector):
    """Rungs (fid, cumulative samples per survivor); keep ceil(|S| / eta). Recommendation updates when a rung completes."""
    name = "succ-halving"

    def __init__(self, N, rng, features=None, eta=3, rungs=((LO, 4), (LO, 8), (HI, 2), (HI, 4), (HI, 8), (HI, 16))):
        super().__init__(N, rng, features)
        self.eta, self.rungs = eta, rungs
        self.S = list(rng.permutation(N))
        self.r, self.cur = 0, 0

    def step(self, env):
        if self.r >= len(self.rungs) or len(self.S) == 1:
            if len(self.S) == 1:                       # nothing left to rank: keep paying for the winner at the top tier
                self.obs(env, self.S[0], HI)
                self.rec = self.S[0]
                return
            self.done = True
            return
        fid, n = self.rungs[self.r]
        if self.cur >= len(self.S):
            m = self.st.mean(fid)
            keep = max(1, math.ceil(len(self.S) / self.eta))
            self.S = sorted(self.S, key=lambda i: -m[i])[:keep]
            self.rec = self.S[0]
            self.r, self.cur = self.r + 1, 0
            return
        i = self.S[self.cur]
        if self.st.n[fid][i] >= n or env.remaining(i, fid) == 0:
            self.cur += 1
            return
        self.obs(env, i, fid)


class ScreenTopK(Selector):
    """Cheap screen of every config (m samples), then the expensive tier round-robin on the top k by cheap mean."""
    name = "screen+topk"

    def __init__(self, N, rng, features=None, m=8, k=10):
        super().__init__(N, rng, features)
        self.m, self.k = m, k
        self.todo = [(i, j) for j in range(m) for i in rng.permutation(N)]
        self.top = None
        self.c = 0

    def step(self, env):
        if self.todo:
            i, _ = self.todo.pop(0)
            self.obs(env, int(i), LO)
            self.rec = int(np.argmax(self.st.mean(LO)))
            return
        if self.top is None:
            self.top = list(np.argsort(-self.st.mean(LO))[: self.k])
        i = int(self.top[self.c % len(self.top)])
        self.c += 1
        if self.c > 40 * len(self.top):
            self.done = True
            return
        self.obs(env, i, HI)
        m = self.st.mean(HI)
        self.rec = int(np.argmax(np.where(self.st.n[HI] > 0, m, -np.inf)))


# ------------------------------------------------------------------ AR1 Bayesian linear model

GH_X, GH_W = np.polynomial.hermite_e.hermegauss(15)
GH_W = GH_W / GH_W.sum()


def _blr(Z, y, noise_var, prior_mean, prior_prec):
    """Posterior of w for y = Z w + eps, eps_i ~ N(0, noise_var_i) (scalar or per-row); returns mean, covariance."""
    nv = np.broadcast_to(np.asarray(noise_var, float), (len(y),)) if len(y) else np.zeros(0)
    W = (1.0 / nv)[:, None] if len(y) else np.zeros((0, 1))
    P = prior_prec + (Z * W).T @ Z
    S = np.linalg.inv(P)
    return S @ (prior_prec @ prior_mean + (Z * W).T @ y), S


class AR1(Selector):
    """Cost-aware AR1: f1(x) = phi(x)' w1; f2(x) = rho * f1(x) + phi(x)' wd.

    Fit: BLR for w1 on all L1 samples; then BLR on L2 samples with design [mu1(x), phi(x)], prior rho ~ N(1, 1), wd ~ N(0, tau2 I).
    Acquisition per round: for each config x and tier, the knowledge-gradient (Gaussian, exact under the linear model, Gauss-Hermite)
    of observing m samples of that tier at x on max_x' f2(x'), divided by the cost of those samples. The L1 route acts on f2 only
    through rho * f1. Top `q` actions are executed (one sample each per m) before refitting.
    """
    name = "AR1-BLR"

    def __init__(self, N, rng, features, costs=None, m_lo=8, m_hi=1, q=6, init_lo=2, init_hi=10, tau2=0.5, alpha=1.0):
        super().__init__(N, rng, features)
        self.costs, self.m_lo, self.m_hi, self.q = costs, m_lo, m_hi, q
        self.tau2, self.alpha = tau2, alpha
        self.queue = [(int(i), LO) for _ in range(init_lo) for i in rng.permutation(N)]
        self.queue += [(int(i), HI) for i in rng.choice(N, size=min(init_hi, N), replace=False)]
        self.mu2 = np.zeros(N)
        self.rho = 1.0

    def _fit(self):
        X, d, N = self.X, self.X.shape[1], self.N
        # level 1: one row per config with data (the config mean, noise variance s1 / n_i)
        idx = np.where(self.st.n[LO] > 0)[0]
        s1 = self.st.pooled_var(LO, 4.0)
        y1 = self.st.s[LO][idx] / self.st.n[LO][idx] if len(idx) else np.zeros(0)
        self.ybar = float(y1.mean()) if len(y1) else 0.0
        w1, S1 = _blr(X[idx], y1 - self.ybar, s1 / np.maximum(self.st.n[LO][idx], 1), np.zeros(d), self.alpha * np.eye(d))
        mu1 = X @ w1 + self.ybar
        self.S1, self.s1var, self.mu1 = S1, s1, mu1
        # level 2: design [mu1(x) - ybar, phi(x)] -> [rho, wd]
        idx2 = np.where(self.st.n[HI] > 0)[0]
        s2 = self.st.pooled_var(HI, 9.0)
        D = np.hstack([mu1[:, None] - self.ybar, X])
        prior_mean = np.concatenate([[1.0], np.zeros(d)])
        prior_prec = np.diag(np.concatenate([[1.0], np.full(d, 1.0 / self.tau2)]))
        y2 = (self.st.s[HI][idx2] / self.st.n[HI][idx2] - self.ybar) if len(idx2) else np.zeros(0)
        w2, S2 = _blr(D[idx2], y2, s2 / np.maximum(self.st.n[HI][idx2], 1), prior_mean, prior_prec)
        self.rho = float(w2[0])
        self.mu2 = D @ w2 + self.ybar
        self.Sig2 = D @ S2 @ D.T + self.rho ** 2 * (X @ S1 @ X.T)   # level-1 uncertainty propagated through rho
        self.s2var = s2
        self.Sig1 = X @ S1 @ X.T
        self.rec = int(np.argmax(self.mu2))

    def _kg(self, Sig, noise, m, scale):
        """KG of m samples at each x on f2 with joint predictive std of change sig_t[x', x] = scale*Sig[x', x]/sqrt(Sig[x,x] + noise/m)."""
        N = self.N
        sd = scale * Sig / np.sqrt(np.maximum(np.diag(Sig), 1e-12) + noise / m)[None, :]       # [x', x]
        mu = self.mu2
        out = np.empty(N)
        for x in range(N):
            vals = mu[:, None] + sd[:, x][:, None] * GH_X[None, :]
            out[x] = float((vals.max(axis=0) * GH_W).sum() - mu.max())
        return out

    def step(self, env):
        if self.queue:
            i, fid = self.queue.pop(0)
            if env.remaining(i, fid) > 0:
                self.obs(env, i, fid)
            if not self.queue:
                self._fit()
            return
        self._fit()
        kg_hi = self._kg(self.Sig2, self.s2var, self.m_hi, 1.0) / (self.m_hi * env.cost[HI])
        kg_lo = self._kg(self.Sig1 * 1.0, self.s1var, self.m_lo, abs(self.rho)) / (self.m_lo * env.cost[LO])
        acts = [(kg_hi[i], i, HI) for i in range(self.N) if env.remaining(i, HI) >= self.m_hi]
        acts += [(kg_lo[i], i, LO) for i in range(self.N) if env.remaining(i, LO) >= self.m_lo]
        if not acts:
            self.done = True
            return
        acts.sort(reverse=True)
        for _, i, fid in acts[: self.q]:
            m = self.m_hi if fid == HI else self.m_lo
            for _ in range(m):
                self.obs(env, i, fid)
        self._fit()


SELECTORS = {"random": Random, "all-high": AllHigh, "low-only": LowOnly, "succ-halving": SuccessiveHalving,
             "screen+topk": ScreenTopK, "AR1-BLR": AR1}


SH_SEEDS_L2 = ((HI, 1), (HI, 2), (HI, 4), (HI, 8), (HI, 16))   # successive halving with seeds as the budget, L2 only


def make(name, N, rng, table, **kw):
    """`sh-seeds-L2` = Hyperband-style halving over seeds at the top tier only; `succ-halving` accepts eta/rungs overrides."""
    if name == "sh-seeds-L2":
        return SuccessiveHalving(N, rng, table.features, eta=kw.get("eta", 2), rungs=SH_SEEDS_L2)
    cls = SELECTORS[name]
    return cls(N, rng, table.features, **kw) if name != "AR1-BLR" else cls(N, rng, table.features, **kw)
