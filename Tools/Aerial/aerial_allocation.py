"""Control allocation and exact wrench-feasibility for multirotors (numpy only).

Wrench w = [T, tau_x, tau_y, tau_z] (body FLU) is linear in per-rotor thrust u:
w = B u with u_i in [0, u_max_i].  The attainable set is the zonotope
Z = { B u : 0 <= u <= u_max }.  For a 4-row B, Z's facet normals are the vectors
orthogonal to every triple of generators, so membership is exact via support functions
over that finite set of normals (no LP solver needed).
"""

from itertools import combinations

import numpy as np


def effectiveness_matrix(vehicle):
    cols = []
    for rotor in vehicle.rotors:
        a = rotor.axis / np.linalg.norm(rotor.axis)
        torque = np.cross(rotor.position, a) - rotor.spin * rotor.km * a
        cols.append(np.concatenate([[a[2]], torque]))
    return np.array(cols).T


def max_thrusts(vehicle):
    return np.array([r.kf * r.omega_max ** 2 for r in vehicle.rotors])


def _facet_normals(gens):
    n = gens.shape[0]
    normals = []
    for idx in combinations(range(gens.shape[1]), n - 1):
        sub = gens[:, idx]
        _, s, vt = np.linalg.svd(sub.T)
        if s.size and s[-1] < 1e-12 * max(1.0, s[0]):
            continue  # degenerate triple, not a facet direction
        normal = vt[-1]
        if np.linalg.matrix_rank(sub) == n - 1:
            normals.append(normal)
            normals.append(-normal)
    return np.array(normals)


def wrench_feasible(b, u_max, w, tol=1e-9):
    """Exact test: does some 0 <= u <= u_max satisfy B u = w?"""
    b = np.asarray(b, dtype=float)
    u_max = np.asarray(u_max, dtype=float)
    w = np.asarray(w, dtype=float)
    if np.linalg.matrix_rank(b) < b.shape[0]:
        raise ValueError("effectiveness matrix is rank deficient; vehicle cannot hold attitude")
    gens = b * u_max
    if gens.shape[1] == gens.shape[0]:
        u = np.linalg.solve(b, w)
        return bool(np.all(u >= -tol) and np.all(u <= u_max + tol))
    for d in _facet_normals(gens):
        support = np.sum(np.maximum(0.0, d @ gens))
        if d @ w > support + tol * max(1.0, abs(support)):
            return False
    return True


def allocate(b, u_max, w):
    """Minimum-norm allocation, then clipped.  Returns (u, achieved_wrench, exact)."""
    b = np.asarray(b, dtype=float)
    u = np.linalg.pinv(b) @ np.asarray(w, dtype=float)
    # Use the null space to pull a hexarotor solution toward the middle of the box.
    null = np.linalg.svd(b)[2][b.shape[0]:].T
    if null.size:
        mid = 0.5 * np.asarray(u_max)
        u = u + null @ (null.T @ (mid - u))
    clipped = np.clip(u, 0.0, u_max)
    return clipped, b @ clipped, bool(np.allclose(clipped, u))
