"""Tree-structured Parzen Estimator (Bergstra 2011 / Optuna defaults, Watanabe 2023).

gamma(n) = min(ceil(0.1 n), 25); 10 random startup trials; 24 EI candidates; per-dimension
truncated-normal Parzen windows on [0,1] with neighbour-distance bandwidths, "magic clip"
and a standard-normal-ish prior component. Good/bad split is among FEASIBLE trials with an
objective; infeasible trials are always bad.
"""
import math

import numpy as np

from .base import Optimizer

_erf = np.vectorize(math.erf, otypes=[float])


def _cdf(z):
    return 0.5 * (1 + _erf(np.asarray(z) / math.sqrt(2)))


class Parzen:
    def __init__(self, pts, dim):
        m = len(pts)
        mus = np.vstack([pts, np.full((1, dim), 0.5)]) if m else np.full((1, dim), 0.5)
        self.mu = mus
        sig = np.empty_like(mus)
        for d in range(dim):
            order = np.argsort(mus[:, d])
            s = np.concatenate([[0.0], mus[order, d], [1.0]])
            left, right = s[1:-1] - s[:-2], s[2:] - s[1:-1]
            sg = np.maximum(left, right)
            sg = np.clip(sg, 1.0 / min(100, 1 + len(mus)), 1.0)
            sig[order, d] = sg
        sig[-1, :] = 1.0     # prior component spans the range
        self.sig = sig

    def logpdf(self, x):
        mu, sg = self.mu[None], self.sig[None]
        z = (x[:, None, :] - mu) / sg
        norm = _cdf((1 - mu) / sg) - _cdf((0 - mu) / sg)
        lp = -0.5 * z ** 2 - np.log(sg * math.sqrt(2 * math.pi)) - np.log(np.maximum(norm, 1e-12))
        lp = lp.sum(2)                                  # (k, comps): product over dims
        m = lp.max(1)
        return m + np.log(np.exp(lp - m[:, None]).sum(1)) - math.log(lp.shape[1])

    def sample(self, rng, k):
        comp = rng.integers(0, len(self.mu), k)
        mu, sg = self.mu[comp], self.sig[comp]
        x = mu + sg * rng.standard_normal(mu.shape)
        for _ in range(20):
            bad = (x < 0) | (x > 1)
            if not bad.any():
                break
            x = np.where(bad, mu + sg * rng.standard_normal(mu.shape), x)
        return np.clip(x, 0, 1)


class TPEOpt(Optimizer):
    name = "tpe"

    def __init__(self, *a, n_startup=10, n_cand=24, **kw):
        super().__init__(*a, **kw)
        self.n_startup, self.n_cand = n_startup, n_cand

    def ask(self, ctx):
        idx = self._high()
        good_pool = [i for i in idx if self.feas[i] and self.Y[i] is not None]
        if len(idx) < self.n_startup or len(good_pool) < 2:
            return self.rng.random(self.dim), self.default_fidelity
        n = len(good_pool)
        ng = min(int(math.ceil(0.1 * len(idx))), 25)
        ng = max(1, min(ng, n - 1)) if n > 1 else 1
        order = sorted(good_pool, key=lambda i: -self.Y[i])
        good = np.array([self.X[i] for i in order[:ng]])
        rest = set(idx) - set(order[:ng])          # remaining feasible AND all infeasible are "bad"
        bad = np.array([self.X[i] for i in sorted(rest)])
        pl, pg = Parzen(good, self.dim), Parzen(bad, self.dim)
        cand = pl.sample(self.rng, self.n_cand)
        return cand[int(np.argmax(pl.logpdf(cand) - pg.logpdf(cand)))], self.default_fidelity
