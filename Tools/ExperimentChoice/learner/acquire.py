"""Acquisition rules over a finite candidate pool, and the scoring functions (EXPLORATORY).

All rules work on the GP posterior of the paired gap Delta (or a sign-preserving transform of
it) over the pool: mean ``mu`` (P,), covariance ``cov`` (P, P), observation noise ``tau2``.

  straddle   Bryan et al. 2005:  1.96 sigma(x) - |mu(x)|                         (level 0)
  sur        stepwise reduction of the expected wrong-recommendation loss, in the spirit of
             Gap-SUR (Hu & Ludkovski 2017): the local loss of recommending by sign(mu) is
             EL(mu, sigma) = sigma phi(mu/sigma) - |mu| Phi(-|mu|/sigma) = E[|Delta| 1[sign wrong]];
             a candidate is scored by the drop of sum_x EL when its variance update is applied
             with the mean held fixed (the usual plug-in simplification, not the exact SUR integral).

Batches are filled greedily: after each pick the covariance is conditioned on a noisy
observation at the pick (mean unchanged), so one batch does not pile onto one spot.
"""
from __future__ import annotations

import numpy as np

from gpmodel import norm_cdf, norm_pdf


def expected_sign_loss(mu, sigma):
    """E[|Delta| 1[sign(Delta) != sign(mu)]] for Delta ~ N(mu, sigma^2). Zero when sigma = 0."""
    mu = np.abs(np.asarray(mu, float))
    sigma = np.asarray(sigma, float)
    safe = np.maximum(sigma, 1e-12)
    z = mu / safe
    return np.where(sigma > 1e-12, safe * norm_pdf(z) - mu * norm_cdf(-z), 0.0)


def straddle(mu, var, kappa=1.96):
    return kappa * np.sqrt(np.maximum(var, 0.0)) - np.abs(mu)


def condition(cov, c, noise_var):
    """Posterior covariance after one noisy observation at index c (rank-one downdate)."""
    k = cov[:, c]
    return cov - np.outer(k, k) / (cov[c, c] + noise_var)


def sur_scores(mu, cov, noise_var, weights=None):
    """Drop of the integrated expected sign loss for a look-ahead observation at every candidate."""
    var = np.maximum(np.diag(cov), 0.0)
    now = expected_sign_loss(mu, np.sqrt(var))
    new_var = np.maximum(var[:, None] - cov ** 2 / (var[None, :] + noise_var), 0.0)     # [x', c]
    after = expected_sign_loss(mu[:, None], np.sqrt(new_var))
    w = np.ones_like(mu) if weights is None else np.asarray(weights, float)
    return (w[:, None] * (now[:, None] - after)).sum(axis=0)


def select_batch(rule, mu, cov, noise_var, available, q):
    """Greedy batch of q distinct candidates among ``available`` (bool mask). Returns a list of indices."""
    available = np.array(available, bool)
    cov = np.array(cov, float)
    picks = []
    for _ in range(q):
        if not available.any():
            break
        if rule == "straddle":
            s = straddle(mu, np.diag(cov))
        elif rule == "sur":
            s = sur_scores(mu, cov, noise_var)
        else:
            raise ValueError(rule)
        s = np.where(available, s, -np.inf)
        c = int(np.argmax(s))
        picks.append(c)
        available[c] = False
        cov = condition(cov, c, noise_var)
    return picks


# --------------------------------------------------------------------------- targets and scoring

def slog(x):
    """Sign-preserving squash: sign(x) log(1 + |x|)."""
    x = np.asarray(x, float)
    return np.sign(x) * np.log1p(np.abs(x))


def recommend(pred):
    """'B' where the predicted gap J_A - J_B is positive, else 'A' (ties go to A)."""
    return np.where(np.asarray(pred, float) > 0, "B", "A")


def wrong_cost(delta, labels, rec):
    """Primary metric R: mean over courses of |Delta| 1[recommended winner wrong].

    Only courses labelled 'A_wins' or 'B_wins' can be wrong; indifferent, both-fail and
    unresolved courses contribute 0 but stay in the denominator.
    """
    delta, labels, rec = np.asarray(delta, float), np.asarray(labels), np.asarray(rec)
    decided = (labels == "A_wins") | (labels == "B_wins")
    winner = np.where(labels == "B_wins", "B", "A")
    wrong = decided & (rec != winner)
    return float((np.abs(delta) * wrong).sum() / max(len(delta), 1))


def sign_accuracy(labels, rec, subset=None):
    """Share of decided courses (optionally within ``subset``) whose recommended winner is right."""
    labels, rec = np.asarray(labels), np.asarray(rec)
    decided = (labels == "A_wins") | (labels == "B_wins")
    if subset is not None:
        decided &= np.asarray(subset, bool)
    if not decided.any():
        return float("nan")
    winner = np.where(labels == "B_wins", "B", "A")
    return float((rec[decided] == winner[decided]).mean())


def near_boundary(X, labels, radius=0.12, X_ref=None, labels_ref=None):
    """Courses within ``radius`` (normalised raw coordinates) of a course with a different label
    among A_wins / B_wins / indifferent. Reference courses default to the set itself."""
    X, labels = np.asarray(X, float), np.asarray(labels)
    Xr = X if X_ref is None else np.concatenate([X, np.asarray(X_ref, float)])
    lr = labels if labels_ref is None else np.concatenate([labels, np.asarray(labels_ref)])
    ok = np.isin(lr, ("A_wins", "B_wins", "indifferent"))
    d = np.sqrt(((X[:, None, :] - Xr[None, :, :]) ** 2).sum(2))
    differ = (labels[:, None] != lr[None, :]) & ok[None, :]
    return np.where(differ, d, np.inf).min(axis=1) <= radius


def bootstrap_ci(values, n_boot=2000, seed=0, q=(2.5, 97.5)):
    """Mean and percentile bootstrap interval over repeats."""
    v = np.asarray(values, float)
    v = v[np.isfinite(v)]
    if v.size == 0:
        return float("nan"), float("nan"), float("nan")
    rng = np.random.Generator(np.random.PCG64(seed))
    means = v[rng.integers(0, v.size, (n_boot, v.size))].mean(axis=1)
    lo, hi = np.percentile(means, q)
    return float(v.mean()), float(lo), float(hi)
