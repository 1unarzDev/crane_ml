"""Two-searcher discrete search model with correlated views and private hints.

A single stationary person is hidden in one of ``n`` cells.  Each cell can be
observed from ``V`` viewpoint classes (e.g. 0 = nadir/along-track, 1 = oblique).
Each (cell, view) pair has a latent visibility state drawn once per episode:
"good" with probability ``g[i, v]``.  Repeated looks from the *same* view share
that latent state, so a poor view stays poor; this is the correlated-view
effect.  A different view re-samples visibility.

Per-look outcomes (target in cell i, view v):
    view good : DETECT w.p. q, HINT w.p. (1-q)*h_good, NULL otherwise
    view bad  : HINT w.p. h_bad, NULL otherwise
Target not in cell i:
    HINT w.p. alpha (false hint), NULL otherwise

DETECT is a confirmed detection and ends the mission.  HINTs are weak,
private evidence that a planner may act on (e.g. verify from another view).

The belief over target location is a function only of per-(cell, view) counts
of NULL looks (m0) and HINT looks (m1); see :func:`posterior`.
"""
from __future__ import annotations

from dataclasses import dataclass, field

import numpy as np

NULL, HINT, DETECT, ABORT = 0, 1, 2, 3


@dataclass
class Instance:
    prior: np.ndarray              # (n,) sums to 1
    g: np.ndarray                  # (n, V) P(view good)
    q: np.ndarray                  # (n, V) P(detect | target, good view)
    h_good: float = 0.0            # P(hint | target, good view, no detect)
    h_bad: float = 0.0             # P(hint | target, bad view)
    alpha: float = 0.0             # P(false hint | no target)
    abort: float = 0.0             # P(look aborted, private to the looker)
    dist: np.ndarray | None = None  # (n, n) travel steps between cells
    starts: tuple[int, int] = (0, 0)
    horizon: int = 30
    name: str = "instance"
    coords: np.ndarray | None = field(default=None, repr=False)

    def __post_init__(self):
        self.prior = np.asarray(self.prior, float)
        self.prior = self.prior / self.prior.sum()
        self.g = np.asarray(self.g, float)
        self.q = np.asarray(self.q, float)
        if self.dist is None:
            self.dist = np.zeros((self.n, self.n), int)
        self.dist = np.asarray(self.dist, int)

    @property
    def n(self) -> int:
        return self.prior.shape[0]

    @property
    def V(self) -> int:
        return self.g.shape[1]


@dataclass(frozen=True)
class BeliefModel:
    """How a planner interprets evidence.

    ``correlated=True`` is the true generative model.  ``correlated=False`` is
    the common misspecification that treats every look as independent with the
    correct per-look marginals but no shared visibility state (ablation)."""
    correlated: bool = True
    use_hints: bool = True


def _cell_log_lr(inst: Instance, m0: np.ndarray, m1: np.ndarray,
                 bm: BeliefModel) -> np.ndarray:
    """log [ L(data | target in i) / L(data | target not in i) ] per cell."""
    g, q = inst.g, inst.q
    h1, hb, a = inst.h_good, inst.h_bad, inst.alpha
    if not bm.use_hints:
        # Hints are ignored: treat them as NULL looks.
        m0, m1 = m0 + m1, np.zeros_like(m1)
        h1 = hb = a = 0.0
    with np.errstate(divide="ignore"):
        if bm.correlated:
            good = g * ((1 - q) * (1 - h1)) ** m0 * ((1 - q) * h1) ** m1
            bad = (1 - g) * (1 - hb) ** m0 * hb ** m1
            present = np.log(good + bad)
        else:
            # Independent-look misspecification: right per-look marginals,
            # no shared latent visibility.
            p_null = g * (1 - q) * (1 - h1) + (1 - g) * (1 - hb)
            p_hint = g * (1 - q) * h1 + (1 - g) * hb
            present = m0 * np.log(p_null) + m1 * np.log(np.maximum(p_hint, 1e-300))
        absent = m0 * np.log(1 - a) + m1 * np.log(max(a, 1e-300))
    lr = present - absent
    return lr.sum(axis=1)


def posterior(inst: Instance, m0: np.ndarray, m1: np.ndarray,
              bm: BeliefModel = BeliefModel()) -> np.ndarray:
    """P(target in cell i | counts)."""
    lp = np.log(inst.prior) + _cell_log_lr(inst, m0, m1, bm)
    lp -= lp.max()
    w = np.exp(lp)
    return w / w.sum()


def p_good_given_target(inst: Instance, m0, m1, bm: BeliefModel) -> np.ndarray:
    """P(view good | target in i, counts) per (i, v)."""
    g, q, h1, hb = inst.g, inst.q, inst.h_good, inst.h_bad
    if not bm.use_hints:
        m0, m1 = m0 + m1, np.zeros_like(m1)
        h1 = hb = 0.0
    if not bm.correlated:
        return g  # misspecified planner never learns that a view is bad
    with np.errstate(divide="ignore", invalid="ignore"):
        good = g * ((1 - q) * (1 - h1)) ** m0 * ((1 - q) * h1) ** m1
        bad = (1 - g) * (1 - hb) ** m0 * hb ** m1
        return np.where(good + bad > 0, good / (good + bad), 0.0)


def detect_prob(inst: Instance, m0, m1, bm: BeliefModel = BeliefModel()) -> np.ndarray:
    """P(next look at (i, v) yields DETECT | counts), shape (n, V)."""
    post = posterior(inst, m0, m1, bm)
    return post[:, None] * p_good_given_target(inst, m0, m1, bm) * inst.q


@dataclass
class Truth:
    target: int
    good: np.ndarray  # (n, V) bool


def sample_truth(inst: Instance, rng: np.random.Generator) -> Truth:
    target = int(rng.choice(inst.n, p=inst.prior))
    good = rng.random(inst.g.shape) < inst.g
    return Truth(target, good)


def sample_outcome(inst: Instance, truth: Truth, cell: int, view: int,
                   u: np.ndarray) -> int:
    """Outcome of one look using pre-drawn uniforms ``u`` (len 3) so that
    paired policy comparisons share random numbers."""
    if u[0] < inst.abort:
        return ABORT
    if cell == truth.target:
        if truth.good[cell, view]:
            if u[1] < inst.q[cell, view]:
                return DETECT
            return HINT if u[2] < inst.h_good else NULL
        return HINT if u[2] < inst.h_bad else NULL
    return HINT if u[2] < inst.alpha else NULL
