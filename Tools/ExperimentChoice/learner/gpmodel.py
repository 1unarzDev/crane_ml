"""Small numpy Gaussian process and Sobol design for the learner pilot (EXPLORATORY).

Attribution: the Matern-5/2 kernel, the standardise-then-Cholesky fit with hyperparameters picked
from a grid by marginal likelihood, and the Joe-Kuo direction numbers are copied or adapted from
crane_ml/Tools/Experiment/crane_experiment/optimizers/{gp.py, sobol.py} (same owner, read-only
source). Differences: one lengthscale per input found by coordinate search instead of a full
grid, full posterior covariance, and a random digital shift instead of Owen scrambling.
"""
from __future__ import annotations

import math

import numpy as np

SQ5 = math.sqrt(5.0)


def norm_pdf(z):
    return np.exp(-0.5 * np.asarray(z, float) ** 2) / math.sqrt(2 * math.pi)


def norm_cdf(z):
    """Standard normal CDF, vectorised (Abramowitz-Stegun 7.1.26 for erf; absolute error < 1.5e-7)."""
    x = np.asarray(z, float) / math.sqrt(2.0)
    a = np.abs(x)
    t = 1.0 / (1.0 + 0.3275911 * a)
    poly = t * (0.254829592 + t * (-0.284496736 + t * (1.421413741 + t * (-1.453152027 + t * 1.061405429))))
    erf = np.sign(x) * (1.0 - poly * np.exp(-a * a))
    return 0.5 * (1.0 + erf)


def matern52_from_d2(d2):
    r = np.sqrt(np.maximum(d2, 0.0))
    return (1 + SQ5 * r + 5.0 / 3.0 * d2) * np.exp(-SQ5 * r)


class GP:
    """Zero-mean GP on standardised targets, Matern-5/2 with one lengthscale per input, unit signal
    variance, homoscedastic noise. Replicates are passed as repeated rows."""

    def __init__(self, ls_grid=(0.1, 0.2, 0.4, 0.8, 1.6, 3.2), noise_grid=(1e-3, 1e-2, 5e-2, 0.2, 0.5),
                 sweeps=2):
        self.ls_grid, self.noise_grid, self.sweeps = ls_grid, noise_grid, sweeps

    @staticmethod
    def _chol(K, noise):
        n = K.shape[0]
        return np.linalg.cholesky(K + (noise + 1e-8) * np.eye(n))

    def _lml(self, D2, ls, noise, yn):
        d2 = sum(D2[k] / (ls[k] * ls[k]) for k in range(len(ls)))
        try:
            L = self._chol(matern52_from_d2(d2), noise)
        except np.linalg.LinAlgError:
            return -np.inf, None, None
        a = np.linalg.solve(L.T, np.linalg.solve(L, yn))
        return float(-0.5 * yn @ a - np.log(np.diag(L)).sum()), L, a

    def fit(self, X, y, init=None):
        X = np.asarray(X, float)
        y = np.asarray(y, float)
        self.X = X
        self.ym, self.ys = float(y.mean()), float(max(y.std(), 1e-9))
        yn = (y - self.ym) / self.ys
        D = X.shape[1]
        D2 = [(X[:, k][:, None] - X[:, k][None, :]) ** 2 for k in range(D)]
        ls = list(init[0]) if init else [0.8] * D
        noise = init[1] if init else 5e-2
        best, L, a = self._lml(D2, ls, noise, yn)
        for _ in range(self.sweeps):
            for k in range(D):
                for cand in self.ls_grid:
                    if cand == ls[k]:
                        continue
                    trial = ls[:k] + [cand] + ls[k + 1:]
                    v, L_, a_ = self._lml(D2, trial, noise, yn)
                    if v > best:
                        best, ls, L, a = v, trial, L_, a_
            for cand in self.noise_grid:
                if cand == noise:
                    continue
                v, L_, a_ = self._lml(D2, ls, cand, yn)
                if v > best:
                    best, noise, L, a = v, cand, L_, a_
        if L is None:
            raise np.linalg.LinAlgError("GP fit failed")
        self.ls, self.noise, self.L, self.alpha, self.lml = np.array(ls), noise, L, a, best
        return self

    def _k(self, A, B):
        As, Bs = A / self.ls, B / self.ls
        d2 = ((As[:, None, :] - Bs[None, :, :]) ** 2).sum(2)
        return matern52_from_d2(d2)

    def posterior(self, Xs, full_cov=False):
        """Mean and (co)variance of the latent function in the units of y."""
        Xs = np.asarray(Xs, float)
        Ks = self._k(Xs, self.X)
        mu = Ks @ self.alpha
        v = np.linalg.solve(self.L, Ks.T)
        if full_cov:
            cov = self._k(Xs, Xs) - v.T @ v
            return mu * self.ys + self.ym, cov * self.ys ** 2
        var = np.maximum(1.0 - (v ** 2).sum(0), 1e-12)
        return mu * self.ys + self.ym, var * self.ys ** 2

    def augmented(self, extra_train, extra_ls):
        """The same fitted model with extra input columns that are CONSTANT over the training rows.

        Constant columns leave the training kernel, the likelihood and every prediction at the
        training value of those columns unchanged, so no refit is needed; their lengthscales are
        not identified by the data and must be supplied. Away from the training value the
        prediction shrinks toward the training mean at a rate set by ``extra_ls``.
        """
        extra_train = np.asarray(extra_train, float).reshape(1, -1)
        g = GP(self.ls_grid, self.noise_grid, self.sweeps)
        g.X = np.hstack([self.X, np.repeat(extra_train, self.X.shape[0], axis=0)])
        g.ls = np.concatenate([self.ls, np.asarray(extra_ls, float)])
        g.ym, g.ys, g.noise, g.L, g.alpha, g.lml = self.ym, self.ys, self.noise, self.L, self.alpha, self.lml
        return g

    @property
    def noise_var(self):
        """Observation noise variance in the units of y."""
        return self.noise * self.ys ** 2


# --------------------------------------------------------------------------- Sobol (dims <= 5)

_JOE_KUO = {2: (1, 0, [1]), 3: (2, 1, [1, 3]), 4: (3, 1, [1, 3, 1]), 5: (3, 2, [1, 1, 1])}
_BITS = 32


def _directions(dim):
    V = [[1 << (_BITS - 1 - j) for j in range(_BITS)]]
    for d in range(2, dim + 1):
        s, a, m = _JOE_KUO[d]
        v = [0] * _BITS
        for j in range(_BITS):
            if j < s:
                v[j] = m[j] << (_BITS - 1 - j)
            else:
                x = v[j - s] ^ (v[j - s] >> s)
                for k in range(1, s):
                    if (a >> (s - 1 - k)) & 1:
                        x ^= v[j - k]
                v[j] = x
        V.append(v)
    return V


def sobol(n, dim, seed=None):
    """First n Sobol points in [0, 1)^dim (gray-code order). ``seed`` adds a random digital shift."""
    V = _directions(dim)
    shift = [0] * dim
    if seed is not None:
        rng = np.random.Generator(np.random.PCG64([int(seed), 9173]))
        shift = [int(v) for v in rng.integers(0, 1 << _BITS, dim)]
    out = np.empty((n, dim))
    x = [0] * dim
    for i in range(n):
        if i:
            c = (i & -i).bit_length() - 1                 # lowest set bit of i (gray-code step)
            for d in range(dim):
                x[d] ^= V[d][c]
        for d in range(dim):
            out[i, d] = (x[d] ^ shift[d]) / float(1 << _BITS)
    return out
