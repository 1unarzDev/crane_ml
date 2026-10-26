"""Optional adapters. Import lazily; tests skip when optuna/botorch are not installed.
UNTESTED in the offline build environment (neither package was available)."""
import numpy as np

from .base import Optimizer


class OptunaTPEOpt(Optimizer):
    name = "optuna_tpe"

    def __init__(self, *a, **kw):
        super().__init__(*a, **kw)
        import optuna
        optuna.logging.set_verbosity(optuna.logging.WARNING)
        self.study = optuna.create_study(direction="maximize",
                                         sampler=optuna.samplers.TPESampler(seed=self.seed % (2 ** 31)))
        self._pending = []

    def ask(self, ctx):
        t = self.study.ask()
        x = np.array([t.suggest_float(f"x{i}", 0.0, 1.0) for i in range(self.dim)])
        self._pending.append(t)
        return x, self.default_fidelity

    def tell(self, x, ctx, fidelity, y):
        super().tell(x, ctx, fidelity, y)
        if self._pending:
            self.study.tell(self._pending.pop(0), float(y))


class BoTorchGPOpt(Optimizer):
    name = "botorch_gp"

    def __init__(self, *a, n_init=6, **kw):
        super().__init__(*a, **kw)
        import botorch  # noqa: F401
        import torch  # noqa: F401
        self.n_init = n_init

    def ask(self, ctx):
        import torch
        from botorch.acquisition import LogExpectedImprovement
        from botorch.fit import fit_gpytorch_mll
        from botorch.models import SingleTaskGP
        from botorch.optim import optimize_acqf
        from gpytorch.mlls import ExactMarginalLogLikelihood
        if len(self.Y) < self.n_init:
            return self.rng.random(self.dim), self.default_fidelity
        X = torch.tensor(np.array(self.X), dtype=torch.double)
        Y = torch.tensor(np.array(self.Y), dtype=torch.double).unsqueeze(-1)
        gp = SingleTaskGP(X, (Y - Y.mean()) / Y.std().clamp_min(1e-9))
        fit_gpytorch_mll(ExactMarginalLogLikelihood(gp.likelihood, gp))
        acq = LogExpectedImprovement(gp, best_f=((Y - Y.mean()) / Y.std().clamp_min(1e-9)).max())
        bounds = torch.stack([torch.zeros(self.dim), torch.ones(self.dim)]).double()
        x, _ = optimize_acqf(acq, bounds=bounds, q=1, num_restarts=5, raw_samples=64)
        return x[0].numpy(), self.default_fidelity
