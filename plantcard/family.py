"""Robot-agnostic model families and evidence units.

A ``Family`` is a parametric plant: named parameters (``ParamSpec``), a batched ``rhs`` over N
particles, and the identification hooks a fit needs (``regressors`` per conjugate block, a grid of
nonlinear parameters, envelope features, a disturbance model). The core (card, update, validate)
names no robot quantity; a marine boat, a planar quadrotor or a modular robot is one subclass.

Evidence is a list of ``Segment``: one time series of state and input from one session on one
tier ('real', 'unity', 'fast', 'synthetic'). Sessions are the replication units for validation.
"""

from dataclasses import dataclass, field

import numpy as np


@dataclass(frozen=True)
class ParamSpec:
    name: str
    unit: str
    bounds: tuple = (None, None)   # applied to pooled particles (truncation)
    group: str = ''                # the card block that owns it


class ParamBatch(dict):
    """Structure of arrays: name -> float64 array [N] (one entry per particle)."""

    @property
    def n(self):
        return len(next(iter(self.values()))) if self else 0

    def take(self, idx):
        return ParamBatch({k: np.asarray(v)[idx] for k, v in self.items()})

    def mean(self):
        return ParamBatch({k: np.atleast_1d(np.mean(v)) for k, v in self.items()})

    def row(self, i):
        return {k: float(v[i]) for k, v in self.items()}

    @classmethod
    def from_rows(cls, rows):
        return cls({k: np.asarray([r[k] for r in rows], float) for k in rows[0]})


@dataclass
class Segment:
    """One evidence unit. ``u`` is zero-order hold: u[k] applies on [t[k], t[k+1])."""

    session: str                   # replication unit (held out as a whole)
    tier: str                      # 'real' | 'unity' | 'fast' | 'synthetic'
    t: np.ndarray                  # [T] s
    x: np.ndarray                  # [T, nx] measured state (family.state order)
    u: np.ndarray                  # [T, nu] input (family.inputs order)
    u_kind: str = 'native'         # what u is ('pwm_q', 'wrench', ...), family-defined
    quality: dict = field(default_factory=dict)
    labels: list = field(default_factory=list)
    meta: dict = field(default_factory=dict)   # family-specific (e.g. a session current estimate)
    name: str = ''
    source_sha: str = ''
    t_event: float = None          # start of the prediction window (e.g. the pulse start)


@dataclass
class Evidence:
    segments: list
    provenance: dict = field(default_factory=dict)

    def sessions(self):
        return sorted({s.session for s in self.segments})

    def split(self, held):
        held = set(held)
        return (Evidence([s for s in self.segments if s.session not in held], self.provenance),
                Evidence([s for s in self.segments if s.session in held], self.provenance))


def delayed_inputs(t, u, delay):
    """u(t - d) per particle, zero-order hold, held at u[0] before the series: [n, T, nu]."""
    t, u, delay = np.asarray(t, float), np.asarray(u, float), np.atleast_1d(np.asarray(delay, float))
    idx = np.searchsorted(t, t[None, :] - delay[:, None] + 1e-9, side='right') - 1
    return u[np.clip(idx, 0, len(t) - 1)]


class Family:
    """Base class. Subclasses set the class attributes and implement ``rhs`` (batched) and the
    identification hooks. ``config`` holds robot constants that are not identified (geometry,
    reference thrust, gauge-fixing masses) and is stored in the card."""

    name = 'family'
    version = '1'
    params: list = []              # [ParamSpec]
    state: list = []
    inputs: list = []
    position = (0, 1)              # state indices of the position used by the coverage events
    heading = None                 # state index of a heading angle, if any
    delay_param = None             # param name (s) that delays every input
    blocks: dict = {}              # NIG block name -> [param names] in regressor column order
    grid_params: list = []         # params identified on the grid
    views: dict = {}               # view name -> 'module:function'

    def __init__(self, config=None):
        self.config = dict(config or {})

    # ---- dynamics -------------------------------------------------------------------------
    def rhs(self, x, u, p, w):
        raise NotImplementedError

    def step(self, x, u, p, dt, w=None):
        k1 = self.rhs(x, u, p, w)
        k2 = self.rhs(x + 0.5 * dt * k1, u, p, w)
        k3 = self.rhs(x + 0.5 * dt * k2, u, p, w)
        k4 = self.rhs(x + dt * k3, u, p, w)
        return x + dt / 6.0 * (k1 + 2 * k2 + 2 * k3 + k4)

    def disturbance(self, n, rng, seg=None, dist=None):
        """Per-rollout disturbance parameters (dict of arrays [n]); None = no disturbance."""
        return None

    def rollout(self, x0, t, u, p, w=None, substeps=1):
        """Trajectories [n, T, nx] from x0 ([nx] or [n, nx]) over times t with inputs u [T, nu]
        (zero-order hold), one particle (and one disturbance draw) per rollout. The input delay
        is resolved at the substep (the input at substep time s is u(s - delay))."""
        n = p.n
        t, u = np.asarray(t, float), np.asarray(u, float)
        x = np.broadcast_to(np.asarray(x0, float), (n, len(self.state))).copy()
        d = np.asarray(p[self.delay_param], float) if self.delay_param else np.zeros(n)
        out = np.empty((n, len(t), len(self.state)))
        out[:, 0] = x
        for k in range(len(t) - 1):
            h = (t[k + 1] - t[k]) / substeps
            for j in range(substeps):
                s = t[k] + j * h
                idx = np.clip(np.searchsorted(t, s - d + 1e-9, side='right') - 1, 0, len(t) - 1)
                x = self.step(x, u[idx], p, h, w)
            out[:, k + 1] = x
        return out

    # ---- identification hooks -------------------------------------------------------------
    def regressors(self, seg, g, ref, select=True):
        """{block: (Phi [m, len(block params)], y [m])} for a segment at grid point ``g``
        ({grid param: value}); ``ref`` is a parameter dict for any terms the fit holds fixed.
        ``select=False``: every row, also of segments a block would not learn from (residuals)."""
        raise NotImplementedError

    def envelope_features(self, seg):
        """z rows [m, dz] (the regime the evidence covers) and their names."""
        raise NotImplementedError

    def wrap_state(self, x):
        return x
