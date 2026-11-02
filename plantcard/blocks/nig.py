"""Normal-Inverse-Gamma block: sequential Bayesian linear regression with exponential forgetting.

    y = X theta + e,  e ~ N(0, s2),  theta | s2 ~ N(mu, s2 Lambda^-1),  s2 ~ IG(a, b)

The update is the NLC study's ``nlc/calib/blr.py`` term for term (``tests/test_nig.py`` pins the
two together on fixtures), plus what a plant card needs on top: the closed-form log evidence
(delay/structure grids and model choice), joint draws (particles), the expected information gain
of a candidate design, and a JSON form.
"""

from dataclasses import dataclass
import math

import numpy as np


@dataclass
class NIG:
    mu: np.ndarray
    Lam: np.ndarray
    a: float
    b: float

    @classmethod
    def prior(cls, mean, rel_std, noise_std, strength=2.0):
        """Prior centred on ``mean`` with marginal std ~ rel_std |mean| (given s2 ~ noise_std^2)."""
        mean = np.asarray(mean, float)
        sd = np.maximum(np.abs(mean) * rel_std, 1e-3)
        s2 = noise_std ** 2
        return cls(mean.copy(), np.diag(s2 / sd ** 2), strength, strength * s2)

    def update(self, X, y, lam=1.0):
        X, y = np.asarray(X, float), np.asarray(y, float)
        L0 = lam * self.Lam
        Ln = L0 + X.T @ X
        mun = np.linalg.solve(Ln, L0 @ self.mu + X.T @ y)
        an = lam * self.a + len(y) / 2
        bn = lam * self.b + 0.5 * (y @ y + self.mu @ L0 @ self.mu - mun @ Ln @ mun)
        return NIG(mun, Ln, an, max(bn, 1e-12))

    @property
    def noise_var(self):
        return self.b / max(self.a - 1, 1e-9)

    def cov(self):
        """Marginal covariance of theta (multivariate t): b/(a-1) Lambda^-1."""
        return self.noise_var * np.linalg.inv(self.Lam)

    def std(self):
        return np.sqrt(np.diag(self.cov()))

    def predictive(self, X):
        """Posterior predictive mean and std at rows X."""
        X = np.atleast_2d(X)
        m = X @ self.mu
        v = self.noise_var * (1 + np.einsum('ij,jk,ik->i', X, np.linalg.inv(self.Lam), X))
        return m, np.sqrt(v)

    def record(self):
        return {'mean': self.mu.tolist(), 'std': self.std().tolist(), 'noise_std': float(np.sqrt(self.noise_var)),
                'a': float(self.a)}

    # ---- plant-card additions -------------------------------------------------------------

    def log_evidence(self, X, y, lam=1.0):
        """log p(y | X) under this prior (forgetting applied to the prior first): the marginal
        likelihood that ranks delays, structures and thrust shapes in closed form."""
        X, y = np.asarray(X, float), np.asarray(y, float)
        L0, a0, b0 = lam * self.Lam, lam * self.a, lam * self.b
        post = NIG(self.mu, self.Lam, self.a, self.b).update(X, y, lam)
        _, ld0 = np.linalg.slogdet(L0)
        _, ldn = np.linalg.slogdet(post.Lam)
        n = len(y)
        return float(-0.5 * n * math.log(2 * math.pi) + 0.5 * (ld0 - ldn) + a0 * math.log(b0)
                     - post.a * math.log(post.b) + math.lgamma(post.a) - math.lgamma(a0))

    def eig(self, X):
        """Expected information gain about theta of a design X (rows of regressors):
        0.5 log det(I + Lambda^-1 X^T X), independent of the unseen y."""
        X = np.atleast_2d(np.asarray(X, float))
        A = np.linalg.solve(self.Lam, X.T @ X)
        return float(0.5 * np.linalg.slogdet(np.eye(len(self.mu)) + A)[1])

    def sample(self, n, rng):
        """n joint draws (s2 ~ IG(a, b), theta | s2 ~ N(mu, s2 Lambda^-1)): [n, d] thetas, [n] s2."""
        s2 = self.b / rng.gamma(self.a, 1.0, n)
        Lc = np.linalg.cholesky(np.linalg.inv(self.Lam))
        z = rng.standard_normal((n, len(self.mu)))
        return self.mu + np.sqrt(s2)[:, None] * (z @ Lc.T), s2

    def to_dict(self):
        return {'mu': [float(v) for v in self.mu], 'Lam': [[float(v) for v in r] for r in self.Lam],
                'a': float(self.a), 'b': float(self.b)}

    @classmethod
    def from_dict(cls, d):
        return cls(np.asarray(d['mu'], float), np.asarray(d['Lam'], float), float(d['a']), float(d['b']))
