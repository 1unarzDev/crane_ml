"""Lossy, delayed links between the two aircraft.

Each hop is a two-state Gilbert-Elliott channel advanced once per time step.
A topology is a list of hops a message must traverse; ``base`` relaying is
two hops (aircraft->base, base->aircraft) with independent channels.
Loss draws come from pre-generated uniforms so paired policy comparisons see
the same channel realisation.
"""
from __future__ import annotations

from dataclasses import dataclass

import numpy as np


@dataclass(frozen=True)
class GE:
    p_gb: float = 0.0     # P(good -> bad) per step
    p_bg: float = 1.0     # P(bad -> good) per step
    loss_good: float = 0.0
    loss_bad: float = 1.0

    @property
    def stationary_bad(self) -> float:
        s = self.p_gb + self.p_bg
        return self.p_gb / s if s > 0 else 0.0

    @property
    def mean_loss(self) -> float:
        b = self.stationary_bad
        return (1 - b) * self.loss_good + b * self.loss_bad

    @property
    def mean_burst(self) -> float:
        return 1.0 / self.p_bg if self.p_bg > 0 else float("inf")

    @staticmethod
    def bernoulli(loss: float) -> "GE":
        return GE(0.0, 1.0, loss, loss)

    @staticmethod
    def bursty(mean_loss: float, mean_burst: float) -> "GE":
        """Pure on/off channel (bad drops everything) with a given average
        loss and mean outage length in steps."""
        if mean_loss <= 0:
            return GE.bernoulli(0.0)
        p_bg = 1.0 / mean_burst
        p_gb = mean_loss * p_bg / (1 - mean_loss)
        return GE(min(p_gb, 1.0), p_bg, 0.0, 1.0)


@dataclass(frozen=True)
class LinkConfig:
    channel: GE = GE.bernoulli(0.0)
    delay: int = 0          # steps per hop
    hops: int = 1           # 1 = direct air-to-air, 2 = via base radio
    name: str = "ideal"


class DirectedLink:
    """Channel state for one direction (src -> dst) across all hops."""

    def __init__(self, cfg: LinkConfig, horizon: int, rng: np.random.Generator):
        self.cfg = cfg
        H = horizon + cfg.hops * cfg.delay + 2
        bad = np.zeros((cfg.hops, H), bool)
        ch = cfg.channel
        for k in range(cfg.hops):
            state = rng.random() < ch.stationary_bad
            u = rng.random(H)
            for t in range(H):
                bad[k, t] = state
                state = (u[t] >= ch.p_bg) if state else (u[t] < ch.p_gb)
        self.bad = bad
        self.u_loss = rng.random((cfg.hops, H))
        self.queue: list[tuple[int, object]] = []

    def send(self, t: int, msg) -> bool:
        """Returns True if the message will be delivered (decided now, using
        the channel state at each hop's transmit time)."""
        cfg, ch = self.cfg, self.cfg.channel
        tt = t
        for k in range(cfg.hops):
            loss = ch.loss_bad if self.bad[k, tt] else ch.loss_good
            if self.u_loss[k, tt] < loss:
                return False
            tt += cfg.delay
        self.queue.append((t + cfg.hops * cfg.delay, msg))
        return True

    def receive(self, t: int) -> list:
        out = [m for (ta, m) in self.queue if ta <= t]
        self.queue = [(ta, m) for (ta, m) in self.queue if ta > t]
        return out
