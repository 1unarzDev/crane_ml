"""Gaussian-process Bayesian optimisation in pure numpy.

Matern-5/2 kernel, hyperparameters chosen by marginal likelihood over a small grid,
EI computed with math.erf. Variants: plain, contextual (context appended to inputs),
and two-fidelity (separable fidelity kernel, cost-aware fidelity choice).
"""
import itertools
import math

import numpy as np

from .base import Optimizer
from .sobol import SobolEngine

SQ5 = math.sqrt(5.0)
_erf = np.vectorize(math.erf, otypes=[float])


def norm_cdf(z):
    return 0.5 * (1.0 + _erf(np.asarray(z, float) / math.sqrt(2.0)))


def norm_pdf(z):
    return np.exp(-0.5 * np.asarray(z, float) ** 2) / math.sqrt(2 * math.pi)


def expected_improvement(mu, var, best):
    s = np.sqrt(np.maximum(var, 1e-12))
    z = (mu - best) / s
    return (mu - best) * norm_cdf(z) + s * norm_pdf(z)


def matern52(A, B):
    d2 = np.maximum(((A[:, None, :] - B[None, :, :]) ** 2).sum(2), 0.0)
    r = np.sqrt(d2)
    return (1 + SQ5 * r + 5.0 / 3.0 * d2) * np.exp(-SQ5 * r)


class GP:
    """groups: list of column-index lists; each group has its own lengthscale.

    mf=True switches to a two-fidelity autoregressive model (Kennedy & O'Hagan 2000):
    f_low ~ GP(k_l), f_high = rho * f_low + delta, delta ~ GP(sd2 * k_d), z in {0 (low), 1 (high)}.
    This captures a rescaled and additively biased low fidelity, not only a correlated one.
    """

    def __init__(self, groups, ls_grid=(0.05, 0.1, 0.2, 0.4, 0.8, 1.6, 3.2),
                 noise_grid=(1e-6, 1e-4, 1e-3, 1e-2, 5e-2, 2e-1), mf=False,
                 rho_grid=(-1.0, -0.5, 0.5, 1.0), sd2_grid=(0.3, 1.0, 3.0), low_noise_mult=4.0,
                 mf_ls_grid=(0.1, 0.2, 0.4, 0.8, 1.6), mf_noise_grid=(1e-6, 1e-3, 1e-2), n_mix=12):
        self.n_mix = n_mix
        self.groups, self.ls_grid, self.noise_grid = groups, ls_grid, noise_grid
        self.mf, self.rho_grid, self.sd2_grid, self.low_noise_mult = mf, rho_grid, sd2_grid, low_noise_mult
        self.mf_ls_grid, self.mf_noise_grid = mf_ls_grid, mf_noise_grid

    def _scale(self, X, ls):
        Xs = np.array(X, float)
        for cols, l in zip(self.groups, ls):
            Xs[:, cols] = Xs[:, cols] / l
        return Xs

    def _K(self, A, za, B, zb, p):
        if not self.mf:
            return matern52(self._scale(A, p[0]), self._scale(B, p[0]))
        ls_l, ls_d, rho, sd2 = p
        ra = np.where(za == 1, rho, 1.0)
        rb = np.where(zb == 1, rho, 1.0)
        K = np.outer(ra, rb) * matern52(self._scale(A, ls_l), self._scale(B, ls_l))
        both_hi = (za[:, None] == 1) & (zb[None, :] == 1)
        return K + both_hi * sd2 * matern52(self._scale(A, ls_d), self._scale(B, ls_d))

    def _param_grid(self):
        ng = len(self.groups)
        if not self.mf:
            for ls in itertools.product(*[self.ls_grid] * ng):
                yield (ls,)
            return
        for ll in self.mf_ls_grid:
            for ld in self.mf_ls_grid:
                for rho in self.rho_grid:
                    for sd2 in self.sd2_grid:
                        yield ((ll,) * ng, (ld,) * ng, rho, sd2)

    def fit(self, X, y, z=None):
        X = np.asarray(X, float)
        y = np.asarray(y, float)
        z = np.ones(len(y)) if z is None else np.asarray(z, float)
        self.X, self.z = X, z
        if self.mf and (z == 0).sum() >= 2 and (z == 1).sum() >= 2:
            # standardise each fidelity separately
            self.lev = {lv: (y[z == lv].mean(), max(y[z == lv].std(), 1e-9)) for lv in (0.0, 1.0)}
        else:
            self.lev = {lv: (y.mean(), max(y.std(), 1e-9)) for lv in (0.0, 1.0)}
        self.ym = np.where(z == 0, self.lev[0.0][0], self.lev[1.0][0])
        self.ys = np.where(z == 0, self.lev[0.0][1], self.lev[1.0][1])
        yn = (y - self.ym) / self.ys
        n = len(y)
        best = None
        cands = []
        noises = self.mf_noise_grid if self.mf else self.noise_grid
        for p in self._param_grid():
            K = self._K(X, z, X, z, p)
            for nz in noises:
                nvec = nz * np.where(z == 0, self.low_noise_mult, 1.0) if self.mf else nz * np.ones(n)
                try:
                    L = np.linalg.cholesky(K + np.diag(nvec) + 1e-9 * np.eye(n))
                except np.linalg.LinAlgError:
                    continue
                a = np.linalg.solve(L.T, np.linalg.solve(L, yn))
                lml = -0.5 * yn @ a - np.log(np.diag(L)).sum()
                cands.append((lml, p, nz, L, a))
                if best is None or lml > best[0]:
                    best = (lml, p, nz, L, a)
        if best is None:
            raise np.linalg.LinAlgError("GP fit failed for all hyperparameters")
        _, self.p, self.nz, self.L, self.alpha = best
        # hyperparameter marginalisation (multi-fidelity only): LML-weighted mixture of the top models,
        # which stops a few points from making the cross-fidelity relation look certain
        cands.sort(key=lambda c: -c[0])
        top = cands[: self.n_mix] if self.mf else [best]
        w = np.exp(np.array([c[0] for c in top]) - top[0][0])
        self.mix = [(wi / w.sum(), c[1], c[3], c[4]) for wi, c in zip(w, top)]
        return self

    def predict(self, Xs, zs=None):
        Xs = np.asarray(Xs, float)
        zs = np.ones(len(Xs)) if zs is None else np.asarray(zs, float)
        m = np.array([self.lev[1.0 if t == 1 else 0.0][0] for t in zs])
        sd = np.array([self.lev[1.0 if t == 1 else 0.0][1] for t in zs])
        mus, vs = [], []
        for w, p, L, alpha in self.mix:
            Ks = self._K(Xs, zs, self.X, self.z, p)
            mu = Ks @ alpha
            v = np.linalg.solve(L, Ks.T)
            prior = np.diag(self._K(Xs, zs, Xs, zs, p)) if self.mf else 1.0
            mus.append(mu)
            vs.append(np.maximum(prior - (v ** 2).sum(0), 1e-12))
        w = np.array([t[0] for t in self.mix])[:, None]
        mus, vs = np.array(mus), np.array(vs)
        mu = (w * mus).sum(0)
        var = (w * (vs + mus ** 2)).sum(0) - mu ** 2          # mixture variance
        return mu * sd + m, np.maximum(var, 1e-12) * sd ** 2

    def cov(self, A, za, B, zb):
        """Posterior covariance (standardised units) between two sets of points."""
        A, B = np.asarray(A, float), np.asarray(B, float)
        za, zb = np.asarray(za, float), np.asarray(zb, float)
        Kab = self._K(A, za, B, zb, self.p)
        Ka = self._K(A, za, self.X, self.z, self.p)
        Kb = self._K(B, zb, self.X, self.z, self.p)
        return Kab - Ka @ np.linalg.solve(self.L.T, np.linalg.solve(self.L, Kb.T))


def _candidates(rng, dim, X_top, n_rand=400, n_local=200, sd=0.07):
    parts = [rng.random((n_rand, dim))]
    if len(X_top):
        pick = rng.integers(0, len(X_top), n_local)
        parts.append(np.clip(X_top[pick] + sd * rng.standard_normal((n_local, dim)), 0, 1))
        parts.append(np.asarray(X_top))
    return np.vstack(parts)


def feasibility_model(Z, feas, groups):
    """P(feasible) from a GP on +-1 labels (probit-style approximation). Constant 1 if nothing failed."""
    feas = np.asarray(feas, bool)
    if feas.all():
        return lambda Q: np.ones(len(Q))
    if not feas.any():
        return lambda Q: np.full(len(Q), 0.05)
    gp = GP(groups, noise_grid=(1e-2, 5e-2, 2e-1)).fit(Z, np.where(feas, 1.0, -1.0))

    def pf(Q):
        mu, var = gp.predict(Q)
        return norm_cdf(mu / np.sqrt(0.1 + var))
    return pf


class GPBO(Optimizer):
    """EI x P(feasible). Objective GP uses only trials that returned an objective; failed trials
    reach the model only through the feasibility GP."""
    name = "gp_bo"

    def __init__(self, *a, n_init=6, **kw):
        super().__init__(*a, **kw)
        self.n_init = n_init
        self.init = SobolEngine(self.dim, True, self.seed)

    def _groups(self):
        return [list(range(self.dim))]

    def _row(self, i):
        return self.X[i]

    def _q(self, X, ctx):
        return X

    def _models(self):
        idx = self._high()
        iy = [i for i in idx if self.Y[i] is not None]
        Z = np.array([self._row(i) for i in iy])
        y = np.array([self.Y[i] for i in iy])
        gp = GP(self._groups()).fit(Z, y)
        pf = feasibility_model(np.array([self._row(i) for i in idx]), [self.feas[i] for i in idx], self._groups())
        return idx, iy, gp, pf

    def _top_x(self, iy):
        ok = [i for i in iy if self.feas[i]] or iy
        ok = sorted(ok, key=lambda i: -self.Y[i])[:5]
        return np.array([self.X[i] for i in ok])

    def ask(self, ctx):
        idx = self._high()
        iy = [i for i in idx if self.Y[i] is not None]
        if len(idx) < self.n_init or len(iy) < 3:
            return self.init.random(1)[0], self.default_fidelity
        idx, iy, gp, pf = self._models()
        cand = _candidates(self.rng, self.dim, self._top_x(iy), 1000, 300)
        p = pf(self._q(cand, ctx))
        feas_iy = [i for i in iy if self.feas[i]] or iy
        mu_obs, _ = gp.predict(self._q(np.array([self.X[i] for i in feas_iy]), ctx))
        mu, var = gp.predict(self._q(cand, ctx))
        acq = expected_improvement(mu, var, mu_obs.max()) * p
        return cand[int(np.argmax(acq))], self.default_fidelity

    def recommend(self, ctx):
        idx = self._high()
        iy = [i for i in idx if self.Y[i] is not None]
        if len(iy) < max(3, self.n_init // 2):
            return self.best_observed()
        idx, iy, gp, pf = self._models()
        cand = _candidates(self._fresh_rng(len(idx)), self.dim, self._top_x(iy), 1000, 400, 0.05)
        mu, _ = gp.predict(self._q(cand, ctx))
        p = pf(self._q(cand, ctx))
        mu = np.where(p >= 0.5, mu, -np.inf)
        if not np.isfinite(mu).any():
            return self.best_observed()
        return cand[int(np.argmax(mu))]


class ContextualGPBO(GPBO):
    name = "contextual_gp"
    contextual = True

    def _groups(self):
        return [list(range(self.dim)), list(range(self.dim, self.dim + self.ctx_dim))]

    def _row(self, i):
        return np.concatenate([self.X[i], self.C[i]])

    def _q(self, X, ctx):
        c = np.zeros(self.ctx_dim) if ctx is None else np.asarray(ctx, float)
        return np.hstack([X, np.tile(c, (len(X), 1))])


class MFBO(Optimizer):
    """Two-fidelity BO: EI x P(feasible) on the high fidelity; fidelity chosen by variance reduction per cost."""
    name = "mfbo"
    multifidelity = True

    def __init__(self, *a, n_init_high=3, n_init_low=10, max_low_streak=3, **kw):
        super().__init__(*a, **kw)
        names = sorted(self.fidelities, key=lambda k: self.fidelities[k])
        self.low, self.high = names[0], self.default_fidelity
        self.n_init_high, self.n_init_low, self.max_low_streak = n_init_high, n_init_low, max_low_streak
        self.init = SobolEngine(self.dim, True, self.seed)
        self.n_asked = 0
        self.low_streak = 0
        self.verify_radius = 0.07

    def _fit(self):
        iy = [i for i in range(len(self.Y)) if self.Y[i] is not None]
        X = np.array([self.X[i] for i in iy])
        z = np.array([1.0 if self.F[i] == self.high else 0.0 for i in iy])
        y = np.array([self.Y[i] for i in iy])
        gp = GP([list(range(self.dim))], mf=True).fit(X, y, z)
        pf = feasibility_model(np.array(self.X), self.feas, [list(range(self.dim))])
        return X, z, y, gp, pf, iy

    def ask(self, ctx):
        # initial design: each high-fidelity point is paired with a low-fidelity run at the SAME x
        # (identifies the cross-fidelity discrepancy directly), then extra low-fidelity points
        n_init = 2 * self.n_init_high + self.n_init_low
        if self.n_asked < n_init or sum(1 for y in self.Y if y is not None) < 4:
            k = self.n_asked
            if k < 2 * self.n_init_high:
                if k % 2 == 0:
                    self._pair_x = self.init.random(1)[0]
                x, fid = self._pair_x, (self.high if k % 2 == 0 else self.low)
            else:
                x, fid = self.init.random(1)[0], self.low
            self.n_asked += 1
            return x, fid
        X, z, y, gp, pf, iy = self._fit()
        hi = [j for j in range(len(iy)) if z[j] == 1 and self.feas[iy[j]]] or [j for j in range(len(iy)) if z[j] == 1]
        if not hi:
            self.n_asked += 1
            return self.init.random(1)[0], self.high
        Xh = X[hi]
        cand = _candidates(self.rng, self.dim, Xh[np.argsort(-y[hi])[:5]], 1000, 300)
        mu_obs, _ = gp.predict(Xh, np.ones(len(Xh)))
        mu, var = gp.predict(cand, np.ones(len(cand)))
        acq = expected_improvement(mu, var, mu_obs.max()) * pf(cand)
        xs = cand[int(np.argmax(acq))][None, :]
        scores = {}
        for name, zz in ((self.high, 1.0), (self.low, 0.0)):
            c = float(gp.cov(xs, [1.0], xs, [zz])[0, 0])
            v = float(gp.cov(xs, [zz], xs, [zz])[0, 0])
            noise = gp.nz * (gp.low_noise_mult if zz == 0 else 1.0)
            scores[name] = c * c / (v + noise) / self.fidelities[name]
        fid = max(scores, key=scores.get)
        # verify-before-exploit: never spend only low-fidelity evidence on a region the target
        # (high) fidelity has not visited, because the low fidelity can be biased
        if np.abs(Xh - xs).max(axis=1).min() > self.verify_radius:
            fid = self.high
        if self.low_streak >= self.max_low_streak:
            fid = self.high
        self.low_streak = self.low_streak + 1 if fid == self.low else 0
        self.n_asked += 1
        return xs[0], fid

    def recommend(self, ctx):
        n_high = sum(1 for f, y in zip(self.F, self.Y) if f == self.high and y is not None)
        if n_high < 2:
            return self.best_observed()
        X, z, y, gp, pf, iy = self._fit()
        top = X[z == 1][np.argsort(-y[z == 1])[:5]]
        cand = _candidates(self._fresh_rng(len(y)), self.dim, top, 1000, 400, 0.05)
        mu, _ = gp.predict(cand, np.ones(len(cand)))
        hi = np.array([self.X[i] for i in range(len(self.Y)) if self.F[i] == self.high and self.Y[i] is not None])
        near = np.abs(cand[:, None, :] - hi[None]).max(2).min(1) <= 0.1   # trust region around verified points
        mu = np.where((pf(cand) >= 0.5) & near, mu, -np.inf)
        if not np.isfinite(mu).any():
            return self.best_observed()
        return cand[int(np.argmax(mu))]
