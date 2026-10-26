"""Optimizer interface. All optimizers work in the unit cube and MAXIMISE y."""
import numpy as np


class Optimizer:
    name = "base"
    contextual = False
    multifidelity = False

    def __init__(self, dim, seed, x0=None, ctx_dim=0, fidelities=None,
                 default_fidelity="sim_high", **kw):
        self.dim = dim
        self.seed = int(seed)
        self.rng = np.random.default_rng(self.seed)
        self.x0 = np.full(dim, 0.5) if x0 is None else np.asarray(x0, float)
        self.ctx_dim = ctx_dim
        self.fidelities = dict(fidelities or {})
        self.default_fidelity = default_fidelity
        self.X, self.C, self.F, self.Y = [], [], [], []
        self.feas, self.margin = [], []

    def _fresh_rng(self, salt):
        """Deterministic rng for recommend(): must not consume the ask-rng."""
        return np.random.default_rng([self.seed, 7919, salt])

    def ask(self, ctx):
        raise NotImplementedError

    def tell(self, x, ctx, fidelity, y, feasible=True, margin=None):
        """y is None when the trial produced no usable objective (failure => feasibility signal only)."""
        if y is None and feasible:
            raise ValueError("a trial without an objective must be marked infeasible")
        self.X.append(np.asarray(x, float))
        self.C.append(np.zeros(self.ctx_dim) if ctx is None else np.asarray(ctx, float))
        self.F.append(fidelity)
        self.Y.append(None if y is None else float(y))
        self.feas.append(bool(feasible))
        self.margin.append(None if margin is None else float(margin))

    def recommend(self, ctx):
        return self.best_observed()

    def _high(self):
        return [i for i, f in enumerate(self.F) if f == self.default_fidelity]

    def best_observed(self):
        idx = [i for i in self._high() if self.Y[i] is not None]
        ok = [i for i in idx if self.feas[i]] or idx
        if not ok:
            return self.x0.copy()
        return self.X[max(ok, key=lambda i: self.Y[i])].copy()
