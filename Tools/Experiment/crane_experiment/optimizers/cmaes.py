"""CMA-ES (Hansen's tutorial formulation), pure numpy. The engine minimises."""
import math

import numpy as np

from .base import Optimizer


class CMA:
    def __init__(self, x0, sigma0, seed, popsize=None):
        n = len(x0)
        self.n = n
        self.rng = np.random.default_rng(seed)
        self.lam = popsize or 4 + int(3 * math.log(n))
        self.mu = self.lam // 2
        w = math.log(self.mu + 0.5) - np.log(np.arange(1, self.mu + 1))
        self.w = w / w.sum()
        self.mueff = 1.0 / (self.w ** 2).sum()
        m = self.mueff
        self.cc = (4 + m / n) / (n + 4 + 2 * m / n)
        self.cs = (m + 2) / (n + m + 5)
        self.c1 = 2 / ((n + 1.3) ** 2 + m)
        self.cmu = min(1 - self.c1, 2 * (m - 2 + 1 / m) / ((n + 2) ** 2 + m))
        self.damps = 1 + 2 * max(0, math.sqrt((m - 1) / (n + 1)) - 1) + self.cs
        self.chiN = math.sqrt(n) * (1 - 1 / (4 * n) + 1 / (21 * n * n))
        self.mean = np.asarray(x0, float).copy()
        self.sigma = float(sigma0)
        self.pc = np.zeros(n)
        self.ps = np.zeros(n)
        self.B = np.eye(n)
        self.D = np.ones(n)
        self.C = np.eye(n)
        self.gen = 0

    def sample(self):
        z = self.rng.standard_normal((self.lam, self.n))
        y = (z * self.D) @ self.B.T
        return self.mean + self.sigma * y

    def update(self, X, f):
        n = self.n
        idx = np.argsort(f)
        xs = X[idx[: self.mu]]
        old = self.mean
        self.mean = self.w @ xs
        y = (self.mean - old) / self.sigma
        inv_sqrt = (self.B / self.D) @ self.B.T
        self.gen += 1
        self.ps = (1 - self.cs) * self.ps + math.sqrt(self.cs * (2 - self.cs) * self.mueff) * (inv_sqrt @ y)
        hsig = (np.linalg.norm(self.ps) / math.sqrt(1 - (1 - self.cs) ** (2 * self.gen)) / self.chiN
                < 1.4 + 2 / (n + 1))
        self.pc = (1 - self.cc) * self.pc + hsig * math.sqrt(self.cc * (2 - self.cc) * self.mueff) * y
        art = (xs - old) / self.sigma
        rank_mu = (art.T * self.w) @ art
        self.C = ((1 - self.c1 - self.cmu) * self.C
                  + self.c1 * (np.outer(self.pc, self.pc) + (1 - hsig) * self.cc * (2 - self.cc) * self.C)
                  + self.cmu * rank_mu)
        self.sigma *= math.exp((self.cs / self.damps) * (np.linalg.norm(self.ps) / self.chiN - 1))
        self.sigma = min(self.sigma, 1e6)
        self.C = (self.C + self.C.T) / 2
        d2, self.B = np.linalg.eigh(self.C)
        self.D = np.sqrt(np.maximum(d2, 1e-20))


class CMAESOpt(Optimizer):
    """CMA-ES in the unit cube with IPOP restarts. Constraint handling: infeasible candidates are
    ranked below every feasible one, ordered by violation; boundary excursions are penalised."""
    name = "cmaes"

    def __init__(self, *a, sigma0=0.3, ipop=True, **kw):
        super().__init__(*a, **kw)
        self.sigma0, self.ipop = sigma0, ipop
        self.cma = CMA(self.x0, sigma0, self.seed)
        self.pop, self.pop_f, self.next_i = None, [], 0
        self.best_f, self.stall = float("inf"), 0
        self.restarts = 0

    def ask(self, ctx):
        if self.pop is None:
            self.pop, self.pop_f, self.next_i = self.cma.sample(), [], 0
        if self.next_i >= len(self.pop):   # more asks outstanding than the generation holds
            raw = self.cma.mean + self.cma.sigma * self.cma.rng.standard_normal(self.dim)
        else:
            raw = self.pop[self.next_i]
        self.next_i += 1
        return np.clip(raw, 0, 1), self.default_fidelity

    def tell(self, x, ctx, fidelity, y, feasible=True, margin=None):
        super().tell(x, ctx, fidelity, y, feasible, margin)
        if self.pop is None or len(self.pop_f) >= len(self.pop):
            return
        raw = self.pop[len(self.pop_f)]
        pen = float(((raw - np.clip(raw, 0, 1)) ** 2).sum())
        viol = 0.0 if feasible else (max(-margin, 0.0) if margin is not None and margin < 0 else 1.0)
        self.pop_f.append((None if y is None else -y, feasible, viol, pen))
        if len(self.pop_f) == len(self.pop):
            self._end_generation()

    def _end_generation(self):
        feas_f = [f for f, ok, v, p in self.pop_f if ok and f is not None]
        worst = max(feas_f) if feas_f else 0.0
        fv = []
        for f, ok, v, p in self.pop_f:
            if ok and f is not None:
                fv.append(f + p)
            elif f is not None:      # objective known but constraint violated: rank after feasible
                fv.append(worst + 1.0 + v + p)
            else:
                fv.append(worst + 1.0 + v + p)
        fv = np.array(fv)
        self.cma.update(self.pop, fv)
        if feas_f and min(feas_f) < self.best_f - 1e-12 * max(1.0, abs(self.best_f)):
            self.best_f, self.stall = min(feas_f), 0
        else:
            self.stall += 1
        self.pop = None
        c = self.cma
        if self.ipop and (c.sigma * c.D.max() < 1e-9 or fv.max() - fv.min() < 1e-13
                          or self.stall > 10 + int(30 * self.dim / c.lam)):
            self.restarts += 1
            self.cma = CMA(np.random.default_rng([self.seed, self.restarts]).random(self.dim),
                           self.sigma0, self.seed + self.restarts, popsize=c.lam * 2)
            self.stall = 0

    def recommend(self, ctx):
        return self.best_observed() if self.restarts else np.clip(self.cma.mean, 0, 1)
