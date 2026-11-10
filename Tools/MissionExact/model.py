"""Two-drone SEARCH / VERIFY / REPORT model (exact, tiny instances).

World
-----
* Graph of ``n`` nodes, integer travel times ``tt[i][j]`` (direct flights).
* ``link[i]``: node i has base connectivity; other nodes are dead zones.
* One stationary person, node ~ ``prior``.
* Each node has 2 viewpoints v in {0, 1}.  Each (node, view) has a latent
  visibility drawn once per episode: good w.p. ``g[i, v]``.  Repeat looks from
  the same viewpoint share it (correlated views); the other view re-samples.
* A look is (view, speed) at the drone's current node.  speed 0 = FAST
  (``dur[0]`` steps, motion blur), speed 1 = SLOW (``dur[1]`` steps).
  Outcome of a look at node c, view v, speed s, given truth tau=(i, b0, b1):
      c == i, view good : CONF w.p. pc[s], WEAK w.p. pw[s], NULL otherwise
      c == i, view bad  : WEAK w.p. wb[s], NULL otherwise
      c != i            : WEAK w.p. alpha (false candidate), NULL otherwise
  WEAK = candidate needing verification; CONF = confirmed location.
* Reporting: the base counts a confirmation only when a report reaches it.
  A drone holding a CONF flies (shortest time) to the nearest link node and
  uplinks; the report arrives ``d`` steps later.  Report time
  r = t_conf + dlink(node) + d.  Success iff r <= T.  (Uplink loss is not
  modelled in the exact solvers: loss = 0.)
* Drone-to-drone messages (delayed-comm mode): a drone at a link node at
  integer time u uploads its full history; the partner downloads it at the
  first integer time >= u + d at which the partner is at a link node.

Key identity used everywhere: given tau (target node + visibility of its two
views), every look outcome is independent of every other look, so beliefs
are products over looks and decentralised processes are independent given
tau.
"""
from __future__ import annotations

import itertools
from dataclasses import dataclass, field

import numpy as np

NULL, WEAK, CONF = 0, 1, 2
ONAME = {NULL: "null", WEAK: "WEAK", CONF: "CONF"}
FAST, SLOW = 0, 1
SNAME = {FAST: "fast", SLOW: "slow"}


@dataclass
class Instance:
    tt: list                      # (n, n) travel times, tt[i][i] = 0
    link: list                    # (n,) bool
    prior: list                   # (n,)
    g: list                       # (n, 2) P(view good)
    pc: tuple = (0.5, 0.9)        # P(CONF | target, good view) [fast, slow]
    pw: tuple = (0.4, 0.05)       # P(WEAK | target, good view)
    wb: tuple = (0.3, 0.3)        # P(WEAK | target, bad view)
    alpha: float = 0.0            # P(false WEAK | no target)
    dur: tuple = (1, 2)           # look durations [fast, slow]
    starts: tuple = (0, 0)
    T: int = 8
    d: int = 0
    name: str = "inst"
    notes: str = ""

    def __post_init__(self):
        self.tt = np.asarray(self.tt, int)
        self.link = np.asarray(self.link, bool)
        self.prior = np.asarray(self.prior, float)
        self.prior = self.prior / self.prior.sum()
        self.g = np.asarray(self.g, float)
        n = len(self.prior)
        assert self.tt.shape == (n, n) and self.link.shape == (n,)
        assert self.link.any(), "need at least one link node"
        self.dlink = np.array([min(self.tt[i][j] for j in range(n) if self.link[j])
                               for i in range(n)], int)
        # truths tau = (node, b0, b1) with positive prior
        self.truths = [(i, b0, b1) for i in range(n) if self.prior[i] > 0
                       for b0 in (0, 1) for b1 in (0, 1)]
        self.p = np.array([self.prior[i] * (self.g[i, 0] if b0 else 1 - self.g[i, 0])
                           * (self.g[i, 1] if b1 else 1 - self.g[i, 1])
                           for i, b0, b1 in self.truths])
        # look types a = (node, view, speed) -> index
        self.looks = [(c, v, s) for c in range(n) if self.prior[c] > 0
                      for v in (0, 1) for s in (FAST, SLOW)]
        self.look_idx = {lk: k for k, lk in enumerate(self.looks)}
        P = np.zeros((len(self.looks), 3, len(self.truths)))
        for k, (c, v, s) in enumerate(self.looks):
            for t, (i, b0, b1) in enumerate(self.truths):
                good = (b0, b1)[v]
                if c == i and good:
                    pr = (1 - self.pc[s] - self.pw[s], self.pw[s], self.pc[s])
                elif c == i:
                    pr = (1 - self.wb[s], self.wb[s], 0.0)
                else:
                    pr = (1 - self.alpha, self.alpha, 0.0)
                P[k, :, t] = pr
        assert (P >= -1e-12).all()
        self.P = P

    @property
    def n(self):
        return len(self.prior)

    def replace(self, **kw):
        d = dict(tt=self.tt.tolist(), link=self.link.tolist(), prior=self.prior.tolist(),
                 g=self.g.tolist(), pc=self.pc, pw=self.pw, wb=self.wb, alpha=self.alpha,
                 dur=self.dur, starts=self.starts, T=self.T, d=self.d, name=self.name,
                 notes=self.notes)
        d.update(kw)
        return Instance(**d)

    # ------------------------------------------------------------ actions
    def actions(self, node, comm=False):
        """Actions of a free drone at ``node``: ('L', v, s) look, ('M', j) move,
        ('W',) wait one step.  Moves to zero-prior nodes and waiting are only
        useful for messaging, so they are offered only when ``comm`` (link
        nodes) -- reporting a CONF is automatic.  (Flights are direct, so a
        zero-prior node is never needed as a waypoint.)"""
        acts = []
        if self.prior[node] > 0:
            acts += [("L", v, s) for v in (0, 1) for s in (FAST, SLOW)]
        acts += [("M", j) for j in range(self.n) if j != node and
                 (self.prior[j] > 0 or (comm and self.link[j]))]
        if comm:
            acts.append(("W",))
        return acts

    def duration(self, node, a):
        if a[0] == "L":
            return self.dur[a[2]]
        if a[0] == "M":
            return int(self.tt[node][a[1]])
        return 1

    def hopeless(self, t):
        """True if no look started at or after t can be reported by T."""
        best = min(self.dlink[c] for c in range(self.n) if self.prior[c] > 0)
        return t + self.dur[0] + int(best) + self.d > self.T

    def report_time(self, t_conf, node):
        return t_conf + int(self.dlink[node]) + self.d

    def describe(self) -> str:
        n = self.n
        L = [f"**{self.name}**  n={n}, T={self.T}, d={self.d}, starts={tuple(self.starts)}"]
        L.append("")
        L.append("| node | prior | link | g(view0) | g(view1) | travel times to 0..n-1 |")
        L.append("|---|---|---|---|---|---|")
        for i in range(n):
            L.append(f"| {i} | {self.prior[i]:.3g} | {'yes' if self.link[i] else 'DEAD'} | "
                     f"{self.g[i,0]:.3g} | {self.g[i,1]:.3g} | {list(map(int, self.tt[i]))} |")
        L.append("")
        L.append(f"Sensor: fast look {self.dur[0]} step(s): P(CONF|good)={self.pc[0]}, "
                 f"P(WEAK|good)={self.pw[0]}, P(WEAK|bad)={self.wb[0]}; "
                 f"slow look {self.dur[1]} step(s): P(CONF|good)={self.pc[1]}, "
                 f"P(WEAK|good)={self.pw[1]}, P(WEAK|bad)={self.wb[1]}; "
                 f"false-WEAK alpha={self.alpha}.")
        if self.notes:
            L.append("")
            L.append(self.notes)
        return "\n".join(L)


def u_reward(inst: Instance, r: int, obj: str) -> float:
    """Terminal reward for a report arriving at time r.
    obj 'p': 1[r <= T]   (P(report by T));
    obj 'e': (T+1-r)^+   (maximising it minimises E[min(R, T+1)])."""
    if r > inst.T:
        return 0.0
    return 1.0 if obj == "p" else float(inst.T + 1 - r)


def fmt_action(a, node=None) -> str:
    if a is None:
        return "-"
    if a[0] == "L":
        return f"look(n{node}, view{a[1]}, {SNAME[a[2]]})" if node is not None \
            else f"look(view{a[1]}, {SNAME[a[2]]})"
    if a[0] == "M":
        return f"fly->n{a[1]}"
    return "wait"
