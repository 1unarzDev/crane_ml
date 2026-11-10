# SPDX-License-Identifier: GPL-3.0-or-later
"""Revisit map: per-cell survivor posterior with correlated views, and per-(cell, view) "informativeness of past
passes" (the posterior over the latent view/decoy state), shared by gossip of observation logs.

Each node (base 0, drones 1..N) holds a `Belief`. It is a deterministic function of the set of observations
(origin, index) the node knows; knowledge sets are prefixes of each origin's log, tracked by a watermark vector,
so merging is idempotent and order-free (store-and-forward / epidemic semantics).

L[c, v, k] = sum of log P(o | state k, speed) over the looks at (c, v): sufficient statistic.
log-odds(c) = logit p0(c) + sum_v [ logsumexp_{k in H1}(logw + L) - logsumexp_{k in H0}(logw + L) ].
The cell-independence approximation (M unknown, survivors drawn with replacement) is the only misspecification.
"""
import numpy as np

from . import sensing as S


def _lse(a, axis=-1):
    m = a.max(axis, keepdims=True)
    return (m + np.log(np.exp(a - m).sum(axis, keepdims=True))).squeeze(axis)


def logit(p):
    p = np.clip(p, 1e-12, 1 - 1e-12)
    return np.log(p) - np.log1p(-p)


def sigmoid(x):
    return 1.0 / (1.0 + np.exp(-x))


class Belief:
    def __init__(self, pub, n_origins):
        self.pub = pub
        T = pub.tables
        self.T = T
        C = T.C
        self.L = np.zeros((C, 4, S.K))
        self.looks = np.zeros((C, 4), int)          # number of known looks per (cell, view)
        self.looks_s = np.zeros((C, 4, 2), int)
        self.alerts = np.zeros(C, int)              # known alerts (o >= 1) per cell
        self.wm = np.zeros(n_origins, int)          # watermark per origin log
        self.logit0 = logit(pub.p0)
        self._p = None
        self._pred = None

    def add(self, c, v, s, o):
        self.L[c, v] += self.T.logP[c, :, s, o]
        self.looks[c, v] += 1
        self.looks_s[c, v, s] += 1
        if o >= 1:
            self.alerts[c] += 1
        self._p = None
        self._pred = None

    def sync(self, logs, upto):
        """Absorb origin logs up to watermark vector `upto` (only entries this node now knows)."""
        changed = False
        for j in range(len(upto)):
            a, b = self.wm[j], upto[j]
            if b > a:
                for (_, c, v, s, o) in logs[j][a:b]:
                    self.add(c, v, s, o)
                self.wm[j] = b
                changed = True
        return changed

    def _view_terms(self):
        lw = self.T.logw[:, None, :] + self.L                     # (C, V, K)
        l1 = _lse(lw[..., S.H1])
        l0 = _lse(lw[..., S.H0])
        return lw, l1, l0

    def posterior(self):
        if self._p is None:
            _, l1, l0 = self._view_terms()
            self._p = sigmoid(self.logit0 + (l1 - l0).sum(1))
        return self._p

    def predictive(self):
        """p (C,), P1o (C, V, S, O) = P(outcome | present, history at (c, v)), P0o likewise | absent."""
        if self._pred is None:
            lw, l1, l0 = self._view_terms()
            w1 = np.exp(lw[..., S.H1] - l1[..., None])            # (C, V, 4)
            w0 = np.exp(lw[..., S.H0] - l0[..., None])            # (C, V, 2)
            P = self.T.P
            P1o = np.einsum("cvk,ckso->cvso", w1, P[:, S.H1])
            P0o = np.einsum("cvk,ckso->cvso", w0, P[:, S.H0])
            self._pred = (self.posterior(), P1o, P0o)
        return self._pred
