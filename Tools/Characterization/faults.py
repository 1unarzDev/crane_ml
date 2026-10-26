"""Fault-injection layer over the identification fixture (plant.VesselModel).

Fields mirror the RoboBoatCalibrationFaults config (ranges validated the same way): strengths[4] in
[0.01, 4], angles[4] in [-45, 45] deg, drag[3] in [1, 20] (EXTRA drag multiplier on surge/sway/yaw hull
drag, on the velocity relative to the current for surge/sway, absolute for yaw), dragOffset[2] in
[-1, 1] (bow, port) m, extra drag force only. Fixture-only additions: signs, lag_s, dead_time_s,
obs_delay_s, current, current_ramp_s, body_bias.

The mount-yaw sign convention of the Unity JSON is UNVERIFIED (Unity AngleAxis about +Y is clockwise
from above; this fixture is CCW positive). Do not transfer angle signs to Unity.
"""
from __future__ import annotations

import json
import math
from collections import deque
from dataclasses import dataclass, replace

import numpy as np

import plant as P


def _check(values, length, lo, hi, name):
    if values is None or len(values) != length:
        raise ValueError(f"{name} requires {length} values")
    for x in values:
        if not math.isfinite(x) or x < lo or x > hi:
            raise ValueError(f"{name} outside [{lo}, {hi}]")


@dataclass(frozen=True)
class FaultConfig:
    strengths: tuple = (1.0, 1.0, 1.0, 1.0)
    angles: tuple = (0.0, 0.0, 0.0, 0.0)   # deg, FL FR RR RL
    drag: tuple = (1.0, 1.0, 1.0)
    drag_offset: tuple = (0.0, 0.0)        # bow, port (m)
    signs: tuple = (1, 1, 1, 1)
    lag_s: float | None = None             # thruster time constant override
    dead_time_s: float = 0.0               # command delay, whole ticks
    obs_delay_s: float = 0.0               # observation value is this stale relative to its stamp
    current: tuple = (0.0, 0.0)            # world m/s
    current_ramp_s: float = 0.0            # >0: current scales linearly 0 -> 1 over this time
    body_bias: tuple = (0.0, 0.0, 0.0)     # observation offset (u, v, r), body frame

    def validate(self):
        _check(self.strengths, 4, 0.01, 4, "strengths")
        _check(self.angles, 4, -45, 45, "angles")
        _check(self.drag, 3, 1, 20, "drag")
        _check(self.drag_offset, 2, -1, 1, "dragOffset")
        _check(self.signs, 4, -1, 1, "signs")
        if any(s not in (-1, 1) for s in self.signs):
            raise ValueError("signs must be +-1")
        if self.lag_s is not None:
            _check((self.lag_s,), 1, 1e-3, 5.0, "lag_s")
        _check((self.dead_time_s, self.obs_delay_s), 2, 0.0, 5.0, "delays")
        _check(self.current, 2, -5, 5, "current")
        _check((self.current_ramp_s,), 1, 0.0, 1e4, "current_ramp_s")
        _check(self.body_bias, 3, -1, 1, "body_bias")
        return self

    @property
    def is_neutral(self):
        return self == FaultConfig()

    @staticmethod
    def from_json(text: str) -> "FaultConfig":
        d = json.loads(text)
        known = {"strengths", "angles", "drag", "dragOffset"}
        extra = set(d) - known
        if extra:
            raise ValueError(f"unknown fault keys {sorted(extra)}")
        kw = {}
        if "strengths" in d:
            kw["strengths"] = tuple(float(x) for x in d["strengths"])
        if "angles" in d:
            kw["angles"] = tuple(float(x) for x in d["angles"])
        if "drag" in d:
            kw["drag"] = tuple(float(x) for x in d["drag"])
        if "dragOffset" in d:
            kw["drag_offset"] = tuple(float(x) for x in d["dragOffset"])
        return FaultConfig(**kw).validate()


class FaultedVessel(P.VesselModel):
    """Base plant plus the fault layer. With a neutral config every added term is exactly zero."""

    def __init__(self, cfg: FaultConfig, dt: float = P.DT, **kw):
        cfg.validate()
        self.cfg = cfg
        specs = tuple(
            replace(s, strength=s.strength * cfg.strengths[i], mount_yaw_deg=cfg.angles[i],
                    sign=cfg.signs[i], tau=cfg.lag_s if cfg.lag_s is not None else s.tau)
            for i, s in enumerate(P.NOMINAL_THRUSTERS))
        super().__init__(specs, dt=dt, **kw)
        q = cfg.dead_time_s / dt
        if abs(q - round(q)) > 1e-6:
            raise ValueError(f"dead_time_s={cfg.dead_time_s} is not a whole number of {dt} s ticks")
        self._ticks = int(round(cfg.dead_time_s / dt))
        self._buf = deque([(0.0,) * 4] * self._ticks, maxlen=self._ticks or None)

    def reset(self, *a, **kw):
        super().reset(*a, **kw)
        if hasattr(self, "_buf"):
            self._buf = deque([(0.0,) * 4] * self._ticks, maxlen=self._ticks or None)

    @property
    def dead_time_realised_s(self):
        return self._ticks * self.dt

    def _delay_command(self, cmd):
        if self._ticks == 0:
            return cmd
        out = self._buf[0]
        self._buf.append(tuple(cmd))
        return out

    def _current_world(self, t):
        k = min(1.0, t / self.cfg.current_ramp_s) if self.cfg.current_ramp_s > 0 else 1.0
        return (self.cfg.current[0] * k, self.cfg.current[1] * k)

    def _external(self, t, ur, vr, r):
        k = self.cfg.drag
        ex = (k[0] - 1) * P.drag(ur, P.DRAG_SURGE)
        ey = (k[1] - 1) * P.drag(vr, P.DRAG_SWAY)
        en = (k[2] - 1) * P.drag(r, P.DRAG_YAW)
        d0, d1 = self.cfg.drag_offset  # bow, port: torque_z = d_bow*Fport - d_port*Fbow
        return (ex, ey, en + (d0 * ey - d1 * ex))


def build_vessel(cfg: FaultConfig | None, enabled: bool = True, dt: float = P.DT, **kw):
    """Flag-off returns the base plant regardless of ``cfg``."""
    if cfg is not None:
        cfg.validate()
    if not enabled or cfg is None:
        return P.VesselModel(dt=dt, **kw)
    return FaultedVessel(cfg, dt=dt, **kw)


def make_observations(log: P.Log, cfg: FaultConfig, seed: int, sigma_v: float = 0.02,
                      sigma_r: float = 0.005, decimate: int = 1):
    """Stamped noisy body-velocity observations. Value at stamp t is the truth at t - obs_delay_s
    (linear interpolation), plus body bias, plus Gaussian noise from a seeded generator."""
    rng = np.random.default_rng(seed)
    t = np.asarray(log.t)[::decimate]
    td = t - cfg.obs_delay_s
    u = np.interp(td, log.t, log.u) + cfg.body_bias[0]
    v = np.interp(td, log.t, log.v) + cfg.body_bias[1]
    r = np.interp(td, log.t, log.r) + cfg.body_bias[2]
    return {"t": t.copy(),
            "u": u + rng.normal(0, sigma_v, t.shape),
            "v": v + rng.normal(0, sigma_v, t.shape),
            "r": r + rng.normal(0, sigma_r, t.shape),
            "sigma_v": sigma_v, "sigma_r": sigma_r}
