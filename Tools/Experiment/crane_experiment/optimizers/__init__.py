from .base import Optimizer
from .cmaes import CMAESOpt
from .gp import GPBO, MFBO, ContextualGPBO
from .simple import DefaultOpt, RandomOpt
from .sobol import SobolOpt
from .tpe import TPEOpt

REGISTRY = {
    "default": DefaultOpt, "random": RandomOpt, "sobol": SobolOpt, "cmaes": CMAESOpt,
    "tpe": TPEOpt, "gp_bo": GPBO, "contextual_gp": ContextualGPBO, "mfbo": MFBO,
}


def make_optimizer(name, **kw):
    if name == "optuna_tpe":
        from .adapters import OptunaTPEOpt
        return OptunaTPEOpt(**kw)
    if name == "botorch_gp":
        from .adapters import BoTorchGPOpt
        return BoTorchGPOpt(**kw)
    if name not in REGISTRY:
        raise KeyError(f"unknown optimizer {name!r}; known: {sorted(REGISTRY)}")
    return REGISTRY[name](**kw)
