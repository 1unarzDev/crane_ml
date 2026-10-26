"""Sobol sequence (Joe-Kuo direction numbers, dims <= 10) with nested-uniform (Owen) scrambling."""
import numpy as np

from .base import Optimizer

BITS = 32
# (s, a, m) rows for dims 2..10 from new-joe-kuo-6.21201; dim 1 is van der Corput.
JOE_KUO = {
    2: (1, 0, [1]), 3: (2, 1, [1, 3]), 4: (3, 1, [1, 3, 1]), 5: (3, 2, [1, 1, 1]),
    6: (4, 1, [1, 1, 3, 3]), 7: (4, 4, [1, 3, 5, 13]), 8: (5, 2, [1, 1, 5, 5, 17]),
    9: (5, 4, [1, 1, 5, 5, 5]), 10: (5, 7, [1, 1, 7, 11, 19]),
}
MAX_DIM = 10
_M = np.uint64(0xFFFFFFFFFFFFFFFF)


def direction_numbers(dim):
    if dim > MAX_DIM:
        raise ValueError(f"Sobol supports at most {MAX_DIM} dimensions here")
    V = np.zeros((dim, BITS), dtype=np.uint64)
    for j in range(BITS):
        V[0, j] = 1 << (BITS - 1 - j)
    for d in range(1, dim):
        s, a, m = JOE_KUO[d + 1]
        for j in range(BITS):
            if j < s:
                V[d, j] = m[j] << (BITS - 1 - j)
            else:
                v = int(V[d, j - s]) ^ (int(V[d, j - s]) >> s)
                for k in range(1, s):
                    if (a >> (s - 1 - k)) & 1:
                        v ^= int(V[d, j - k])
                V[d, j] = v
    return V


def _mix64(z):
    with np.errstate(over="ignore"):
        z = (z ^ (z >> np.uint64(30))) * np.uint64(0xBF58476D1CE4E5B9)
        z = (z ^ (z >> np.uint64(27))) * np.uint64(0x94D049BB133111EB)
        return z ^ (z >> np.uint64(31))


def owen_scramble(ints, seed, dim_index):
    """Nested uniform scramble of uint32-valued ints: bit b flips by a hash of (seed, dim, higher bits)."""
    x = ints.astype(np.uint64)
    out = np.zeros_like(x)
    with np.errstate(over="ignore"):
        base = _mix64(np.uint64(seed & 0xFFFFFFFFFFFFFFFF) + np.uint64(dim_index + 1) * np.uint64(0x9E3779B97F4A7C15))
        for b in range(BITS):
            prefix = x >> np.uint64(BITS - b) if b > 0 else np.zeros_like(x)
            h = _mix64(base ^ (prefix * np.uint64(0xD6E8FEB86659FD93) + np.uint64(b + 1) * np.uint64(0xA24BAED4963EE407)))
            flip = h & np.uint64(1)
            bit = (x >> np.uint64(BITS - 1 - b)) & np.uint64(1)
            out |= (bit ^ flip) << np.uint64(BITS - 1 - b)
    return out


class SobolEngine:
    def __init__(self, dim, scramble=True, seed=0):
        self.dim, self.scramble, self.seed = dim, scramble, int(seed)
        self.V = direction_numbers(dim)
        self.index = 0

    def random(self, n):
        idx = np.arange(self.index, self.index + n, dtype=np.uint64)
        self.index += n
        gray = idx ^ (idx >> np.uint64(1))
        pts = np.zeros((n, self.dim), dtype=np.uint64)
        for j in range(BITS):
            on = ((gray >> np.uint64(j)) & np.uint64(1)).astype(bool)
            if not on.any():
                continue
            pts[on] ^= self.V[:, j][None, :]
        if self.scramble:
            for d in range(self.dim):
                pts[:, d] = owen_scramble(pts[:, d], self.seed, d)
        return pts.astype(np.float64) / float(1 << BITS)


def l2_star_discrepancy(P):
    """Warnock's formula for the L2-star discrepancy."""
    P = np.asarray(P, float)
    n, d = P.shape
    t1 = 3.0 ** (-d)
    t2 = (2.0 / n) * np.prod((1 - P ** 2) / 2.0, axis=1).sum()
    mx = np.maximum(P[:, None, :], P[None, :, :])
    t3 = np.prod(1 - mx, axis=2).sum() / n ** 2
    return float(np.sqrt(max(t1 - t2 + t3, 0.0)))


class SobolOpt(Optimizer):
    name = "sobol"

    def __init__(self, *a, **kw):
        super().__init__(*a, **kw)
        self.engine = SobolEngine(self.dim, True, self.seed)

    def ask(self, ctx):
        return self.engine.random(1)[0], self.default_fidelity
