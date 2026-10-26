"""Fake worker: quadratic reward with a known contextual optimum.

FIXTURE ONLY. Regions (unit coordinates u0,u1,u2):
  invalid  : u0 < 0.06            -> worker error, no reward
  stale    : u1 < 0.06 (u0 >= .06) -> worker answers with an old profile sha and old observation
  loophole : u0 > 0.78 and u1 > 0.78 -> reports +2.0 reward (true max is 0) while the
             independent invariants (reported vs odometry progress, speed) are violated
  violation: u1 < 0.15            -> constraint violated (collision flag)
Low fidelity has a shifted optimum, a bias and double noise.
"""
import math

import numpy as np

from .spec import Space

W = np.array([1.0, 1.0, 0.5])


class FakeWorker:
    label_ceiling = "FIXTURE_VALIDATED"

    def __init__(self, noise=0.05, loophole_reward=2.0):
        self.noise, self.loophole_reward = noise, loophole_reward

    @staticmethod
    def optimum(c):
        return np.array([0.35 + 0.3 * (c - 0.5), 0.55 + 0.2 * (c - 0.5), 0.5])

    def true_reward(self, u, c, fidelity="sim_high"):
        u = np.asarray(u, float)
        if fidelity == "sim_low":
            return float(-2.0 * (W * (u - self.optimum(c) - np.array([0.08, 0, 0])) ** 2).sum() - 0.1)
        return float(-2.0 * (W * (u - self.optimum(c)) ** 2).sum())

    def true_regret(self, u, c):
        return 0.0 - self.true_reward(u, c)

    def run(self, config, context, fidelity, seed, spec):
        space = Space(spec["space"])
        u = space.to_unit(config)
        c = float(context["features"][0])
        rng = np.random.default_rng(seed)
        eps = float(rng.standard_normal())
        sd = self.noise * (2.0 if fidelity == "sim_low" else 1.0)
        cost = float(spec["fidelities"][fidelity]["cost"]) * (0.8 + 0.4 * float(u[1]))
        out = {"cost": cost, "violation": bool(u[1] < 0.15), "margin": float(u[1] - 0.15), "profile_sha": context["profile"],
               "obs_age_s": 0.05, "error": None, "reward": None}
        if u[0] < 0.06:
            out.update(error="sim_diverged", violation=True, margin=None)
            return out
        r = self.true_reward(u, c, fidelity) + sd * eps
        progress = 10.0 + 3.0 * r
        inv = {"reported_progress": progress, "odom_progress": progress / 1.02,
               "sim_time_s": 20.0, "vmax": 2.0}
        if u[1] < 0.06:
            out.update(profile_sha=context["profile"].split("@")[0] + "@stale0000", obs_age_s=2.5)
        if u[0] > 0.78 and u[1] > 0.78:
            r = self.loophole_reward + sd * eps
            inv = {"reported_progress": 50.0, "odom_progress": 3.0, "sim_time_s": 1.0, "vmax": 2.0}
        out.update(reward=float(r), invariants=inv)
        return out


class AlwaysInvalidWorker(FakeWorker):
    def run(self, config, context, fidelity, seed, spec):
        return {"cost": 1.0, "reward": None, "violation": True, "error": "sim_diverged",
                "profile_sha": context["profile"], "obs_age_s": 0.0}


def make_worker(name):
    if name == "fake":
        return FakeWorker()
    if name == "always_invalid":
        return AlwaysInvalidWorker()
    if name == "crane_nav2":
        from .crane_nav2 import CraneNav2Worker
        return CraneNav2Worker()
    raise KeyError(f"unknown worker {name!r}")
