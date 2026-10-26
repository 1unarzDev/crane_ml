import numpy as np

from .base import Optimizer


class DefaultOpt(Optimizer):
    name = "default"

    def ask(self, ctx):
        return self.x0.copy(), self.default_fidelity

    def recommend(self, ctx):
        return self.x0.copy()


class RandomOpt(Optimizer):
    name = "random"

    def ask(self, ctx):
        return self.rng.random(self.dim), self.default_fidelity
