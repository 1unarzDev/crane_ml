"""Linear-in-parameters regressors and identifiability diagnostics.

Used to show *which* parameters a given experiment can and cannot identify
before anyone fits a CRANE underwater profile to sim or real data.  The rule is
the standard one: theta is identifiable from y = Phi theta only if Phi has full
column rank and a usable condition number.
"""
from __future__ import annotations

from dataclasses import dataclass

import numpy as np


@dataclass
class Identifiability:
    rank: int
    n_params: int
    condition: float
    singular_values: np.ndarray
    null_space: np.ndarray  # columns span the unidentifiable directions

    @property
    def identifiable(self) -> bool:
        return self.rank == self.n_params and np.isfinite(self.condition) and self.condition < 1e8


def analyse(phi: np.ndarray, rel_tol: float = 1e-9) -> Identifiability:
    phi = np.asarray(phi, float)
    # Column scaling so the diagnosis is not an artefact of units.
    norms = np.linalg.norm(phi, axis=0)
    safe = np.where(norms > 0, norms, 1.0)
    u, s, vt = np.linalg.svd(phi / safe, full_matrices=True)
    tol = rel_tol * (s[0] if s.size and s[0] > 0 else 1.0)
    rank = int(np.sum(s > tol))
    cond = float(s[0] / s[-1]) if s.size and s[-1] > tol else float("inf")
    null = vt[rank:].T / safe[:, None]
    return Identifiability(rank, phi.shape[1], cond, s, null)


def surge_regressor(udot: np.ndarray, u: np.ndarray) -> np.ndarray:
    """tau_X = (m + M_A) udot + X_u u + X_uu u|u|  -> theta = [m+M_A, X_u, X_uu]."""
    udot, u = np.asarray(udot, float), np.asarray(u, float)
    return np.column_stack([udot, u, u * np.abs(u)])


def surge_regressor_split_mass(udot: np.ndarray, u: np.ndarray) -> np.ndarray:
    """Same experiment, but asking for m and M_A separately: always rank-deficient."""
    udot, u = np.asarray(udot, float), np.asarray(u, float)
    return np.column_stack([udot, udot, u, u * np.abs(u)])


def hover_regressor(n_samples: int) -> np.ndarray:
    """Static vertical balance tau_Z = W - B from level hover: theta = [W, B]."""
    ones = np.ones(n_samples)
    return np.column_stack([ones, -ones])


def roll_static_regressor(phi: np.ndarray) -> np.ndarray:
    """Static roll moment K = (z_g W - z_b B) sin(phi): theta = [z_g W, z_b B]."""
    s = np.sin(np.asarray(phi, float))
    return np.column_stack([s, -s])


def fit(phi: np.ndarray, y: np.ndarray) -> np.ndarray:
    theta, *_ = np.linalg.lstsq(np.asarray(phi, float), np.asarray(y, float), rcond=None)
    return theta
