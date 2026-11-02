"""Grid posterior over a few nonlinear parameters (delay, thrust shape, structure choice).

p(g | data) is proportional to prior(g) times a closed-form evidence (the NIG blocks' log evidence
at that grid point), so a delay or a reverse-thrust ratio gets a posterior instead of a pattern
search.
"""

from dataclasses import dataclass
import itertools

import numpy as np


@dataclass
class Grid:
    axes: dict            # name -> 1-D list of values
    logp: np.ndarray      # shape = tuple(len(v) for v in axes.values()), normalised

    @classmethod
    def uniform(cls, axes):
        shape = tuple(len(v) for v in axes.values())
        return cls({k: [float(x) for x in v] for k, v in axes.items()}, np.full(shape, -np.log(np.prod(shape))))

    @property
    def names(self):
        return list(self.axes)

    def points(self):
        """(index tuple, {name: value}) for every grid point."""
        for idx in itertools.product(*(range(len(v)) for v in self.axes.values())):
            yield idx, {k: self.axes[k][i] for k, i in zip(self.axes, idx)}

    def posterior(self, loglik):
        """New grid with logp + loglik (an array of the grid's shape), normalised."""
        lp = self.logp + np.asarray(loglik, float)
        lp = lp - lp.max()
        lp = lp - np.log(np.exp(lp).sum())
        return Grid(self.axes, lp)

    def prob(self):
        p = np.exp(self.logp - self.logp.max())
        return p / p.sum()

    def map(self):
        idx = np.unravel_index(int(np.argmax(self.logp)), self.logp.shape)
        return {k: self.axes[k][i] for k, i in zip(self.axes, idx)}

    def marginal(self, name):
        j = self.names.index(name)
        p = self.prob().sum(axis=tuple(i for i in range(self.logp.ndim) if i != j))
        return np.asarray(self.axes[name]), p

    def mean_sd(self, name):
        v, p = self.marginal(name)
        m = float((v * p).sum())
        return m, float(np.sqrt(max(((v - m) ** 2 * p).sum(), 0.0)))

    def sample_idx(self, n, rng):
        flat = rng.choice(self.logp.size, size=n, p=self.prob().ravel())
        return [np.unravel_index(int(f), self.logp.shape) for f in flat]

    def to_dict(self):
        return {'axes': self.axes, 'logp': np.round(self.logp, 6).tolist()}

    @classmethod
    def from_dict(cls, d):
        return cls({k: list(v) for k, v in d['axes'].items()}, np.asarray(d['logp'], float))
