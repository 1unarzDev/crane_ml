# SPDX-License-Identifier: GPL-3.0-or-later
"""Gaussian-copula (latent-factor) repeat-look model shared by exact.py and the scale simulator.

Look k at one (cell, view class) with marginal alert probability p fires iff  sqrt(r) z + sqrt(1-r) e_k < Phi^-1(p),
z ~ N(0,1) shared by all looks of that (cell, view class), e_k iid N(0,1).  Looks from different view classes use
different z (independent).  r is the LATENT correlation; the observable quantity (and what Tools/Vision measures,
"ICC within a run") is the correlation between two binary looks, ICC(r; p).  `latent_for_icc` inverts it, so every
public parameter in this package is an ICC rho.  rho = 0 is the independence model.
"""
import math
from statistics import NormalDist

import numpy as np

_ND = NormalDist()
_erf = np.vectorize(math.erf)
K_NODES = 24


def probit(p):
    return _ND.inv_cdf(min(max(float(p), 1e-12), 1 - 1e-12))


def Phi(x):
    return 0.5 * (1.0 + _erf(np.asarray(x, float) / math.sqrt(2.0)))


def nodes(k=K_NODES):
    x, w = np.polynomial.hermite_e.hermegauss(k)
    return x, w / w.sum()


_X, _W = nodes()


def q_of_z(t, r, x=None):
    """P(fire | z) at nodes x for threshold t = probit(p) and latent correlation r."""
    x = _X if x is None else x
    if r <= 1e-12:
        return np.full_like(x, Phi(t))
    return Phi((t - math.sqrt(r) * x) / math.sqrt(1.0 - r))


def icc_of_latent(p, r, k=64):
    x, w = nodes(k)
    q = q_of_z(probit(p), r, x)
    return float((np.sum(w * q * q) - p * p) / (p * (1 - p)))


_CAL = {}


def latent_for_icc(p, icc):
    """Latent correlation r with binary ICC(r; p) = icc (bisection; monotone)."""
    if icc <= 1e-9:
        return 0.0
    key = (round(float(p), 6), round(float(icc), 6))
    if key not in _CAL:
        lo, hi = 0.0, 0.999
        for _ in range(40):
            mid = 0.5 * (lo + hi)
            if icc_of_latent(p, mid) < icc:
                lo = mid
            else:
                hi = mid
        _CAL[key] = 0.5 * (lo + hi)
    return _CAL[key]


def count_lik(p, r, h, m, x=None, w=None):
    """Likelihood of one specific sequence with h alerts and m non-alerts from one (cell, class) latent."""
    x = _X if x is None else x
    w = _W if w is None else w
    q = q_of_z(probit(p), r, x)
    return float(np.sum(w * q ** h * (1 - q) ** m))
