"""D1 (DEV ONLY): learned decision-aware communication gates (new file; nothing else is edited).

A gated team: every agent plays the HYBRID_P reading (default plan + records/leases when they
arrive + role rule) layered on the HEALTH_PART reading of a death frame.  Three message types
exist and a per-message gate decides, at each decision opportunity, whether to send:

  rec     record message (window of the last 4 own records), sent after a look completes
  claim   lease (current task), sent when a task is chosen
  beacon  12 B death frame (stamp = own death time), opportunity only while the sender's sensor
          is dead (an alive agent's beacon is a no-op for every receiver, so it is not offered)

Labels (counterfactual coordination-regret reduction, one-step deviation from "silent
continuation"): behaviour policy = random sends with prob P_B; at a sampled decision n the
episode is replayed twice with identical random numbers (truth, outcomes, links, fault, behaviour
draws), once forcing "send" at n and once forcing "silent" at n, silent afterwards.  The label
is  dV = det(send) - det(silent)  in {-1, 0, +1}  (paired CRN).

Only DEV seeds are used (see SEEDS below).  CLI:
  cd Tools && fsrun python3 -m ForestSearch.learned_comm labels --out X.npz ...
"""
from __future__ import annotations

import argparse
import math
import os
import sys
import time
from dataclasses import replace

import numpy as np

from .links import LinkConfig
from .model import ABORT, DETECT, HINT, NULL
from .nagent import NResult
from .nbeacon import HZ, NBEpisode
from .nscale import (HealthMsg, LINKS, build, fault_spec, make_strategies, nominal_plan_ns)
from .sim import HEADER_B, LEASE_B, RECORD_B, Message

HERE = os.path.dirname(os.path.abspath(__file__))
RES = os.path.join(HERE, "results")

# ---- seed discipline: DEV only.  Held-out ranges that must never be touched here:
FORBIDDEN = [(3000, 3015), (4000, 4015), (5000, 5031), (6000, 6015)]
LABEL_SEEDS = tuple(range(2000, 2008))      # instance seeds used to generate labels / train
VAL_SEEDS = tuple(range(2008, 2012))        # instance seeds used for validation (model choice)
EVAL_SEEDS = tuple(range(2100, 2116))       # fresh DEV instances for the evaluation (16)


def _check_seeds(seeds):
    for s in seeds:
        for lo, hi in FORBIDDEN:
            assert not (lo <= s <= hi), f"seed {s} is in a held-out range"


for _s in (LABEL_SEEDS, VAL_SEEDS, EVAL_SEEDS):
    _check_seeds(_s)

KINDS = ("rec", "claim", "beacon")
P_B = {"rec": 0.15, "claim": 0.15, "beacon": 0.25}
WINDOW = 4
FEAT = ["bias", "self_dead", "since_death", "silence_age", "since_sent", "n_sent",
        "est_loss", "logN", "remaining", "div_own", "task_dev", "unsent", "known_dead",
        "hints", "aborts"]
NF = len(FEAT)


def msg_bytes(kind, N, nrec=WINDOW):
    k = N - 1
    if kind == "beacon":
        return HEADER_B * k
    if kind == "rec":
        return (HEADER_B + RECORD_B * max(1, min(WINDOW, nrec))) * k
    return (HEADER_B + LEASE_B) * k


class LCEpisode(NBEpisode):
    """Gated-communication episode.  `policy(kind, feat, a, t) -> bool` decides sends.
    With policy=None and force=None every gate is silent and the run equals IMPLICIT (tested)."""

    def __init__(self, inst, strats, link, rng, nominal=None, fault=None, policy=None,
                 force=None, behav=None, collect=False, est_loss=0.0, resample=None):
        super().__init__(inst, strats, link, rng, nominal, fault=fault, variant="part",
                         rule="near")
        N = self.N
        self.policy, self.force, self.behav, self.collect = policy, force, behav, collect
        self.est_loss = est_loss
        self.resample = resample               # (n, seed): at decision n, redraw the unknown future
        self.pdec = []
        self.spec = None
        self.ndec = 0
        self.points = []                       # (n, kind, t, agent, feat) when collect
        self.last_heard = [[-1] * N for _ in range(N)]
        self.last_sent = [{k: -1 for k in KINDS} for _ in range(N)]
        self.n_sent = [{k: 0 for k in KINDS} for _ in range(N)]
        self.rec_sent = [0] * N
        self.sent_by_kind = {k: 0 for k in KINDS}
        self.bytes_by_kind = {k: 0 for k in KINDS}

    # ------------------------------------------------------------ delivery
    def _deliver(self, t):
        N = self.N
        for j in range(N):
            for i in range(N):
                if i == j:
                    continue
                for m in self.links[(i, j)].receive(t):
                    self.last_heard[j][i] = max(self.last_heard[j][i], t)
                    if isinstance(m, HealthMsg):
                        if m.t_dead is not None and i not in self.p_dead[j]:
                            self.p_dead[j][i] = m.t_dead
                    else:
                        a = self.agents[j]
                        for r in m.records:
                            a.p_records[i][r.seq] = r
                            a.p_leases[i].pop(r.seq, None)
                        if m.lease is not None:
                            seq, c, v, exp = m.lease
                            if seq not in a.p_records[i]:
                                a.p_leases[i][seq] = (c, v, exp)

    # ------------------------------------------------------------ features
    def _feat(self, kind, a, t):
        inst, i, N = self.inst, a.idx, self.N
        H = inst.horizon
        td = self.t_dead[i]
        dead = td is not None and td <= t
        heard = max(self.last_heard[i][j] for j in range(N) if j != i)
        nom = self.nominal[i]
        div = 0
        for r in a.records:
            if r.seq >= len(nom) or r.outcome != NULL or \
                    (nom[r.seq].cell, nom[r.seq].view) != (r.cell, r.view):
                div += 1
        tdev = 0.0
        if kind == "claim" and a.task is not None:
            tdev = float(a.task.seq >= len(nom) or
                         (nom[a.task.seq].cell, nom[a.task.seq].view) != (a.task.cell, a.task.view))
        ls = self.last_sent[i][kind]
        hints = sum(1 for r in a.records if r.outcome == HINT)
        aborts = sum(1 for r in a.records if r.outcome == ABORT)
        return np.array([
            1.0, float(dead), (t - td) / H if dead else 0.0, (t - heard) / H,
            (t - ls) / H if ls >= 0 else (t + 1) / H, self.n_sent[i][kind] / H,
            self.est_loss, math.log2(N), (H - t) / H, div / max(1, H), tdev,
            (len(a.records) - self.rec_sent[i]) / max(1, H), len(self.p_dead[i]) / N,
            hints / max(1, H), aborts / max(1, H)])

    def _nbytes(self, kind, a):
        return msg_bytes(kind, self.N, len(a.records) - self.rec_sent[a.idx])

    def _gate(self, kind, a, t):
        n = self.ndec
        self.ndec += 1
        f = None
        if self.collect or (self.policy is not None and self.force is None):
            f = self._feat(kind, a, t)
        if self.collect:
            self.points.append((n, kind, t, a.idx, f))
        if self.resample is not None and n == self.resample[0]:
            self._resample_future(t, self.resample[1])
        if self.force is not None:
            tn, send = self.force
            if n == tn:
                dec = send
            elif n < tn:
                dec = bool(self.behav[n % len(self.behav)] < P_B[kind])
            else:
                dec = False
        elif self.policy is not None:
            dec = bool(self.policy(kind, f, self._nbytes(kind, a), self, a))
        elif self.behav is not None:
            dec = bool(self.behav[n % len(self.behav)] < P_B[kind])
        else:
            dec = False
        if self.collect:
            self.pdec.append(bool(dec))
        if dec:
            i = a.idx
            nb = self._nbytes(kind, a)
            self.last_sent[i][kind] = t
            self.n_sent[i][kind] += 1
            self.sent_by_kind[kind] += 1
            self.bytes_by_kind[kind] += nb
        return dec

    # ------------------------------------------------------------ unknown-future resampling
    def _loglik(self, tr):
        inst = self.inst
        ll = 0.0
        for ag in self.agents:
            for r in ag.records:
                if r.outcome == ABORT:
                    continue
                c, v = r.cell, r.view
                if c == tr.target:
                    if tr.good[c, v]:
                        q = inst.q[c, v]
                        p = {HINT: (1 - q) * inst.h_good, NULL: (1 - q) * (1 - inst.h_good)}.get(r.outcome, 0.0)
                    else:
                        p = inst.h_bad if r.outcome == HINT else 1 - inst.h_bad
                else:
                    p = inst.alpha if r.outcome == HINT else 1 - inst.alpha
                ll += math.log(max(p, 1e-12))
        return ll

    def _resample_future(self, t, seed, K=64):
        """Redraw everything the deciding agent cannot know: the truth (sampling-importance-
        resampling from the posterior given ALL records so far, i.e. more information than a real
        agent has), the outcome uniforms of looks not yet completed, and the future link states."""
        from .model import sample_truth
        rng = np.random.default_rng(seed)
        inst = self.inst
        cands = [sample_truth(inst, rng) for _ in range(K)]
        ll = np.array([self._loglik(c) for c in cands])
        w = np.exp(ll - ll.max())
        self.truth = cands[int(rng.choice(K, p=w / w.sum()))]
        un = rng.random(self.u.shape)
        for ag in self.agents:
            s0 = max(ag.seq - 1, 0) if ag.task is not None else ag.seq
            s0 = min(s0, self.u.shape[1] - 1)
            self.u[ag.idx, s0:] = un[ag.idx, s0:]
        for L in self.links.values():
            ch = L.cfg.channel
            for k in range(L.bad.shape[0]):
                state = rng.random() < ch.stationary_bad
                uu = rng.random(L.bad.shape[1])
                for tt in range(t, L.bad.shape[1]):
                    L.bad[k, tt] = state
                    state = (uu[tt] >= ch.p_bg) if state else (uu[tt] < ch.p_gb)
            L.u_loss[:, t:] = rng.random(L.u_loss[:, t:].shape)

    # ------------------------------------------------------------ run
    def run(self):
        inst, N = self.inst, self.N
        looks = redundant = aborts = hints = 0
        for t in range(inst.horizon + 1):
            self._cur_t = t
            self._deliver(t)
            for a in self.agents:                      # beacon opportunities (dead senders)
                td = self.t_dead[a.idx]
                if td is not None and td <= t:
                    if self._gate("beacon", a, t):
                        self._hsend(a.idx, t, HealthMsg(td))
            self._deliver(t)
            for a in self.agents:
                if a.task is None or a.task.finish != t:
                    continue
                tk = a.task
                from .model import sample_outcome
                out = NULL if self.force_null else sample_outcome(
                    inst, self.truth, tk.cell, tk.view, self.u[a.idx, min(tk.seq, inst.horizon)])
                from .sim import Record
                rec = Record(tk.seq, tk.cell, tk.view, out, t)
                a.records.append(rec)
                a.task = None
                a.pos = tk.cell
                if out == ABORT:
                    aborts += 1
                else:
                    looks += 1
                    if any((tk.cell, tk.view) in self.looked_by[j]
                           for j in range(N) if j != a.idx):
                        redundant += 1
                    self.looked_by[a.idx].add((tk.cell, tk.view))
                if out == NULL:
                    a.m0[tk.cell, tk.view] += 1
                elif out == HINT:
                    a.m1[tk.cell, tk.view] += 1
                    hints += 1
                if self.log is not None:
                    self.log.append((t, a.idx, tk.cell, tk.view, out))
                if out == DETECT:
                    return NResult(True, t, looks, redundant, aborts, hints, self.bytes,
                                   self.msgs, self.delivered, self.collisions)
                if self._gate("rec", a, t):
                    recent = a.records[-WINDOW:]
                    self.rec_sent[a.idx] = len(a.records)
                    self._send(a, t, recent, None)
            self._deliver(t)
            if t == inst.horizon:
                break
            chosen = []
            for a in self.agents:
                if a.task is not None:
                    continue
                c, v = self._choose(a, t)
                from .sim import Task
                travel = int(inst.dist[a.pos, c])
                a.task = Task(a.seq, c, v, t, t + travel + 1)
                a.seq += 1
                chosen.append((c, v))
                if self._gate("claim", a, t):
                    self._send(a, t, [], self._lease(a))
                    self._deliver(t)
            for x in range(len(chosen)):
                for y in range(x + 1, len(chosen)):
                    if chosen[x] == chosen[y]:
                        self.collisions += 1
        return NResult(False, inst.horizon, looks, redundant, aborts, hints, self.bytes,
                       self.msgs, self.delivered, self.collisions)


# ---------------------------------------------------------------- strategies / runners
_S = {}


def strat():
    if "g" not in _S:
        S = make_strategies("greedy")
        _S["S"] = S
        from .nscale import NSStrategy
        b = S["HYBRID_P"]
        f = {k: getattr(b, k) for k in b.__dataclass_fields__}
        f.update(name="GATED", send="none", health="beacon")
        _S["g"] = NSStrategy(**f)
    return _S["S"], _S["g"]


def make_ep(inst, N, link, ep, nominal, fault, **kw):
    S, g = strat()
    f = None
    if fault:
        f = dict(fault)
        f["seed"] = 10_000 + ep
    e = LCEpisode(inst, (g,) * N, LINKS[link], np.random.default_rng(ep), nominal, fault=f,
                  est_loss=LINKS[link].channel.mean_loss, **kw)
    e.spec = dict(inst=inst, N=N, link=link, ep=ep, nominal=nominal, fault=fault)
    return e


def behav_draws(ep, salt=77):
    return np.random.default_rng((ep, salt)).random(4096)


# ---------------------------------------------------------------- labels
def gen_labels(cells, seeds, eps, k_per_kind, out, time_limit=None, tag=""):
    """cells: list of dicts(N, travel, lam, link, fault).  For each (cell, instance seed, episode)
    run a behaviour episode, sample up to k_per_kind decision points per kind and replay."""
    rng = np.random.default_rng(12345)
    X, Y, YT, K, META = [], [], [], [], []
    t0 = time.time()
    n_roll = 0
    for c in cells:
        N, travel, lam = c["N"], c["travel"], c["lam"]
        S, _ = strat()
        for seed in seeds:
            inst = build(N, seed, lam, travel, HZ[(N, travel)])
            nom = nominal_plan_ns(inst, N, S)
            fs = fault_spec(c["fault"], inst.horizon, N)
            for ep in eps:
                bu = behav_draws(ep)
                e0 = make_ep(inst, N, c["link"], ep, nom, fs, behav=bu, collect=True)
                e0.run()
                n_roll += 1
                pts = e0.points
                chosen = []
                for kind in KINDS:
                    pk = [p for p in pts if p[1] == kind]
                    if pk:
                        idx = rng.choice(len(pk), size=min(k_per_kind[kind], len(pk)), replace=False)
                        chosen += [pk[j] for j in idx]
                for (n, kind, t, ag, f) in chosen:
                    res = []
                    for send in (True, False):
                        e = make_ep(inst, N, c["link"], ep, nom, fs, behav=bu, force=(n, send))
                        r = e.run()
                        n_roll += 1
                        res.append(r)
                    X.append(f)
                    Y.append(int(res[0].detected) - int(res[1].detected))
                    YT.append((res[1].ttd - res[0].ttd) / inst.horizon)
                    K.append(KINDS.index(kind))
                    META.append((N, travel, lam, ["ideal", "burst30_d1"].index(c["link"]),
                                 c["fault"] == "perm1", seed, ep, t))
            if time_limit and time.time() - t0 > time_limit:
                break
        print(f"[{time.time()-t0:7.0f}s] {tag} N={N} travel={travel} {c['link']} {c['fault']} "
              f"labels={len(Y)} rollouts={n_roll}", file=sys.stderr, flush=True)
    np.savez_compressed(out, X=np.array(X), y=np.array(Y), yt=np.array(YT), kind=np.array(K),
                        meta=np.array(META), cpu=time.process_time(), n_roll=n_roll)
    return len(Y), time.time() - t0, n_roll


def default_cells(Ns=(2, 3, 5, 10), travels=(1, 0), links=("ideal", "burst30_d1"),
                  faults=("perm1", "none"), lam=1):
    return [dict(N=N, travel=tr, lam=lam, link=lk, fault=fa)
            for N in Ns for tr in travels for lk in links for fa in faults]


# ---------------------------------------------------------------- models
def standardize_fit(X):
    mu = X.mean(0)
    sd = X.std(0)
    sd[sd < 1e-9] = 1.0
    mu[0], sd[0] = 0.0, 1.0
    return mu, sd


class Logit:
    """dV_hat = P(dV=+1) - P(dV=-1) from two L2-regularised logistic regressions."""

    def __init__(self, l2=1e-3):
        self.l2 = l2

    @staticmethod
    def _fit1(Z, y, l2, iters=60):
        w = np.zeros(Z.shape[1])
        for _ in range(iters):
            p = 1 / (1 + np.exp(-np.clip(Z @ w, -30, 30)))
            g = Z.T @ (p - y) / len(y) + l2 * w
            Hs = (Z * (p * (1 - p))[:, None]).T @ Z / len(y) + l2 * np.eye(Z.shape[1])
            step = np.linalg.solve(Hs + 1e-9 * np.eye(Z.shape[1]), g)
            w -= step
            if np.abs(step).max() < 1e-7:
                break
        return w

    def fit(self, X, y):
        self.mu, self.sd = standardize_fit(X)
        Z = (X - self.mu) / self.sd
        self.wp = self._fit1(Z, (y > 0).astype(float), self.l2)
        self.wn = self._fit1(Z, (y < 0).astype(float), self.l2)
        return self

    def predict(self, X):
        Z = (np.atleast_2d(X) - self.mu) / self.sd
        sig = lambda a: 1 / (1 + np.exp(-np.clip(a, -30, 30)))
        return sig(Z @ self.wp) - sig(Z @ self.wn)


class MLP:
    """2 hidden layers (32, 32), tanh, MSE on dV, Adam, early stopping on a validation split."""

    def __init__(self, h=32, l2=1e-4, epochs=60, lr=3e-3, seed=0):
        self.h, self.l2, self.epochs, self.lr, self.seed = h, l2, epochs, lr, seed

    def _fwd(self, Z):
        a1 = np.tanh(Z @ self.W[0] + self.b[0])
        a2 = np.tanh(a1 @ self.W[1] + self.b[1])
        return a1, a2, (a2 @ self.W[2] + self.b[2])[:, 0]

    def fit(self, X, y, Xv=None, yv=None):
        rng = np.random.default_rng(self.seed)
        self.mu, self.sd = standardize_fit(X)
        Z = (X - self.mu) / self.sd
        h = self.h
        self.W = [rng.normal(0, 1 / math.sqrt(Z.shape[1]), (Z.shape[1], h)),
                  rng.normal(0, 1 / math.sqrt(h), (h, h)), rng.normal(0, 0.1, (h, 1))]
        self.b = [np.zeros(h), np.zeros(h), np.zeros(1)]
        params = self.W + self.b
        m = [np.zeros_like(p) for p in params]
        v = [np.zeros_like(p) for p in params]
        step = 0
        best, best_p = 1e18, None
        n = len(y)
        for ep in range(self.epochs):
            perm = rng.permutation(n)
            for s in range(0, n, 256):
                ix = perm[s:s + 256]
                zb, yb = Z[ix], y[ix]
                a1, a2, o = self._fwd(zb)
                d3 = ((o - yb) / len(ix))[:, None]
                gW2 = a2.T @ d3
                d2 = (d3 @ self.W[2].T) * (1 - a2 ** 2)
                gW1 = a1.T @ d2
                d1 = (d2 @ self.W[1].T) * (1 - a1 ** 2)
                gW0 = zb.T @ d1
                grads = [gW0 + self.l2 * self.W[0], gW1 + self.l2 * self.W[1],
                         gW2 + self.l2 * self.W[2], d1.sum(0), d2.sum(0), d3.sum(0)]
                step += 1
                for k, (p, g) in enumerate(zip(params, grads)):
                    m[k] = 0.9 * m[k] + 0.1 * g
                    v[k] = 0.999 * v[k] + 0.001 * g * g
                    p -= self.lr * (m[k] / (1 - 0.9 ** step)) / (np.sqrt(v[k] / (1 - 0.999 ** step)) + 1e-8)
            if Xv is not None:
                lv = float(np.mean((self.predict(Xv) - yv) ** 2))
                if lv < best:
                    best, best_p = lv, [p.copy() for p in params]
        if best_p is not None:
            for p, q in zip(params, best_p):
                p[...] = q
        return self

    def predict(self, X):
        Z = (np.atleast_2d(X) - self.mu) / self.sd
        return self._fwd(Z)[2]


class ModelPolicy:
    """Per-kind models; send iff dV_hat > lam * bytes."""

    def __init__(self, models, lam):
        self.models, self.lam = models, lam

    def __call__(self, kind, f, nbytes, ep, a):
        return float(self.models[kind].predict(f)[0]) > self.lam * nbytes


class RLPolicy:
    """IC3Net-style gate: per-kind Bernoulli(sigmoid(w.z)) on standardised features."""

    def __init__(self, W, mu, sd, rng=None, greedy=False, log=None):
        self.W, self.mu, self.sd, self.rng, self.greedy, self.log = W, mu, sd, rng, greedy, log

    def __call__(self, kind, f, nbytes, ep, a):
        z = (f - self.mu[kind]) / self.sd[kind]
        p = 1 / (1 + np.exp(-np.clip(z @ self.W[kind], -30, 30)))
        if self.greedy:
            return p > 0.5
        s = self.rng.random() < p
        if self.log is not None:
            self.log.append((kind, z, float(s), p, nbytes))
        return s


class MVPolicy:
    """Execution-time myopic value-of-information gate (no learning; DEC-COMM / ConTaCT style).
    At each beacon opportunity of a dead sender (first WINDOW+1 steps after death, one send max)
    it runs R paired rollouts of the SAME simulator, with the unknown future redrawn
    (_resample_future), and sends iff mean(det(send) - det(silent)) > lam * bytes.  rec / claim
    are never sent (their rollout value is ~0 or negative, see labels).  It is given the joint
    record history (more information than a real agent), so it is an optimistic baseline."""

    def __init__(self, lam, R=4, window=3, cache=None):
        self.lam, self.R, self.window = lam, R, window
        self.cache = cache if cache is not None else {}
        self.calls = 0

    def voi(self, ep, n):
        sp = ep.spec
        key = (id(sp["inst"]), sp["link"], sp["ep"], n)
        if key not in self.cache:
            d = 0.0
            for r in range(self.R):
                res = []
                for send in (True, False):
                    e = make_ep(sp["inst"], sp["N"], sp["link"], sp["ep"], sp["nominal"], sp["fault"],
                                behav=np.ones(4096), force=(n, send), resample=(n, hash((sp["ep"], n, r)) % 2 ** 31))
                    res.append(e.run())
                    self.calls += 1
                d += int(res[0].detected) - int(res[1].detected)
            self.cache[key] = d / self.R
        return self.cache[key]

    def __call__(self, kind, f, nbytes, ep, a):
        if kind != "beacon" or ep.n_sent[a.idx]["beacon"] > 0:
            return False
        if ep._cur_t - ep.t_dead[a.idx] > self.window:
            return False
        return self.voi(ep, ep.ndec - 1) > self.lam * nbytes


class CriticDQ:
    """ATOC-style arm: dQ_hat(f) = Q(f, send) - Q(f, silent) from a critic regressed on episode
    OUTCOMES of the behaviour policy (no paired rollouts)."""

    def __init__(self, mlp):
        self.mlp = mlp

    def predict(self, X):
        X = np.atleast_2d(X)
        one = np.hstack([X, np.ones((len(X), 1))])
        zero = np.hstack([X, np.zeros((len(X), 1))])
        return self.mlp.predict(one) - self.mlp.predict(zero)


def pol_rule(name):
    """Hand-made gates inside the same episode class, used for tests / ablations."""
    if name == "ALWAYS":
        return lambda kind, f, nb, ep, a: True
    if name == "BEACON_ONCE":
        return lambda kind, f, nb, ep, a: kind == "beacon" and ep.n_sent[a.idx]["beacon"] == 0
    if name == "BEACON_REP3":     # repeat every 3 steps (periodic beacon from the dead agent)
        return lambda kind, f, nb, ep, a: (kind == "beacon" and
                                           (ep.n_sent[a.idx]["beacon"] == 0 or
                                            ep.last_sent[a.idx]["beacon"] <= ep._cur_t - 3))
    if name == "SILENT":
        return None
    raise ValueError(name)


# ---------------------------------------------------------------- training
LAMS = [0.0, 1e-6, 3e-6, 1e-5, 3e-5, 1e-4, 3e-4, 1e-3]


def load_labels(paths):
    Xs, ys, ks, ms = [], [], [], []
    for p in paths:
        d = np.load(p)
        Xs.append(d["X"]); ys.append(d["y"]); ks.append(d["kind"]); ms.append(d["meta"])
    return np.concatenate(Xs), np.concatenate(ys).astype(float), np.concatenate(ks), np.concatenate(ms)


def fit_models(train, val, kind_kw=None):
    """Per-kind Logit and MLP.  Logit l2 and MLP hidden/l2 chosen on the validation split."""
    X, y, k, _ = train
    Xv, yv, kv, _ = val
    out = {"logit": {}, "mlp": {}}
    info = {}
    for ki, kind in enumerate(KINDS):
        m, mv = k == ki, kv == ki
        best = (1e18, None)
        for l2 in (1e-4, 1e-3, 1e-2, 1e-1):
            mdl = Logit(l2).fit(X[m], y[m])
            lv = float(np.mean((mdl.predict(Xv[mv]) - yv[mv]) ** 2))
            if lv < best[0]:
                best = (lv, mdl, l2)
        out["logit"][kind] = best[1]
        best2 = (1e18, None)
        for l2 in (1e-4, 1e-2):
            mdl = MLP(l2=l2, epochs=40).fit(X[m], y[m], Xv[mv], yv[mv])
            lv = float(np.mean((mdl.predict(Xv[mv]) - yv[mv]) ** 2))
            if lv < best2[0]:
                best2 = (lv, mdl, l2)
        out["mlp"][kind] = best2[1]
        base = float(np.mean((yv[mv] - y[m].mean()) ** 2))
        info[kind] = dict(n=int(m.sum()), n_nonzero=int((y[m] != 0).sum()), mean=float(y[m].mean()),
                          val_mse_const=base, val_mse_logit=best[0], val_mse_mlp=best2[0],
                          l2_logit=best[2], l2_mlp=best2[2])
    return out, info


def train_rl(lam, train_X, train_k, iters=150, batch=24, seed=0, cells=None, log=None):
    """REINFORCE (IC3Net-style Bernoulli gate on the same features; paired silent baseline)."""
    rng = np.random.default_rng(seed)
    S, _ = strat()
    mu, sd, W = {}, {}, {}
    for ki, kind in enumerate(KINDS):
        m = train_k == ki
        mu[kind], sd[kind] = standardize_fit(train_X[m])
        W[kind] = np.zeros(NF)
        W[kind][0] = -2.0
    mom = {k: np.zeros(NF) for k in KINDS}
    var = {k: np.zeros(NF) for k in KINDS}
    cells = cells or default_cells(travels=(1,))
    cache = {}
    rets = []
    for it in range(iters):
        grads = {k: np.zeros(NF) for k in KINDS}
        tot = 0.0
        for _ in range(batch):
            c = cells[rng.integers(len(cells))]
            seed_i = LABEL_SEEDS[rng.integers(len(LABEL_SEEDS))]
            ep = 50_000 + int(rng.integers(100_000))
            N = c["N"]
            key = (N, c["travel"], seed_i)
            if key not in cache:
                inst = build(N, seed_i, c["lam"], c["travel"], HZ[(N, c["travel"])])
                cache[key] = (inst, nominal_plan_ns(inst, N, S))
            inst, nom = cache[key]
            fs = fault_spec(c["fault"], inst.horizon, N)
            base = make_ep(inst, N, c["link"], ep, nom, fs).run()
            lg = []
            pol = RLPolicy(W, mu, sd, rng=rng, log=lg)
            r = make_ep(inst, N, c["link"], ep, nom, fs, policy=pol).run()
            A = (float(r.detected) - lam * r.bytes_sent) - float(base.detected)
            tot += A
            for (kind, z, s_, p_, nb) in lg:
                grads[kind] += A * (s_ - p_) * z
        for kind in KINDS:
            g = -grads[kind] / batch                    # ascent via descent on -J
            mom[kind] = 0.9 * mom[kind] + 0.1 * g
            var[kind] = 0.999 * var[kind] + 0.001 * g * g
            W[kind] -= 0.1 * (mom[kind] / (1 - 0.9 ** (it + 1))) / (np.sqrt(var[kind] / (1 - 0.999 ** (it + 1))) + 1e-8)
        rets.append(tot / batch)
        if log and it % 25 == 0:
            print(f"  rl lam={lam:g} it={it} adv={np.mean(rets[-25:]):+.4f}", file=sys.stderr, flush=True)
    return W, mu, sd, rets


def critic_data(out, E, per_kind=30, travels=(1,)):
    rng = np.random.default_rng(777)
    S, _ = strat()
    X, Y, K, M = [], [], [], []
    for c in default_cells(travels=travels):
        N = c["N"]
        for seed in LABEL_SEEDS:
            inst = build(N, seed, c["lam"], c["travel"], HZ[(N, c["travel"])])
            nom = nominal_plan_ns(inst, N, S)
            fs = fault_spec(c["fault"], inst.horizon, N)
            for ep in range(E):
                e = make_ep(inst, N, c["link"], ep + 30_000, nom, fs, behav=behav_draws(ep + 30_000, 99), collect=True)
                r = e.run()
                for ki, kind in enumerate(KINDS):
                    idx = [j for j, p in enumerate(e.points) if p[1] == kind]
                    if len(idx) > per_kind:
                        idx = list(rng.choice(idx, per_kind, replace=False))
                    for j in idx:
                        X.append(np.append(e.points[j][4], float(e.pdec[j])))
                        Y.append(float(r.detected)); K.append(ki); M.append((N, seed, ep))
    np.savez_compressed(out, X=np.array(X), y=np.array(Y), kind=np.array(K), meta=np.array(M))
    return len(Y)


def cmd_critic(a):
    t0 = time.process_time()
    n = critic_data(a.out, a.E)
    print(f"critic rows={n} cpu={time.process_time()-t0:.0f}s")


def cmd_rl(a):
    import pickle
    pk = pickle.load(open(a.models, "rb"))
    tr = load_labels(a.train)
    t0 = time.process_time()
    lam = float(a.lam)
    W, mu, sd, rets = train_rl(lam, tr[0], tr[2], iters=a.iters, batch=24, seed=a.seed, log=True)
    pk["rl"][lam] = dict(W=W, mu=mu, sd=sd, rets=rets)
    pk["cpu_rl"] = pk.get("cpu_rl", 0) + time.process_time() - t0
    pickle.dump(pk, open(a.models, "wb"))
    print("rl done cpu", time.process_time() - t0, "last adv", np.mean(rets[-25:]))


def cmd_diagnose(a):
    """Failure analysis on the validation labels (instances 2008-2011): calibration, and which
    states the gates get wrong.  Appends markdown to a.out."""
    import pickle
    pk = pickle.load(open(a.models, "rb"))
    X, y, k, meta = load_labels(a.val)
    L_ = []
    P = L_.append
    P("## 7. Failure analysis on validation labels (instances 2008-2011, never used for fitting)\n")
    P("Calibration of the beacon gates: bin by predicted dV (quintiles); mean realised label per bin "
      "(label noise is large: each label is a single paired replay, values in {-1,0,+1}).\n")
    P("| model | bin | n | mean predicted | mean label | send at lam=1e-4 (bytes~12(N-1)) |")
    P("|---|---|---|---|---|---|")
    b = k == 2
    for fam in ("logit", "mlp", "critic"):
        if fam not in pk["models"]:
            continue
        pred = pk["models"][fam]["beacon"].predict(X[b])
        qs = np.quantile(pred, [0, .2, .4, .6, .8, 1.0])
        for i in range(5):
            m = (pred >= qs[i]) & (pred <= qs[i + 1]) if i == 4 else (pred >= qs[i]) & (pred < qs[i + 1])
            if m.sum():
                P(f"| {fam} | {i+1} | {int(m.sum())} | {pred[m].mean():+.4f} | {y[b][m].mean():+.4f} | - |")
        P(f"| {fam} | corr(pred,label) | {int(b.sum())} | {np.corrcoef(pred, y[b])[0,1]:+.3f} | | |")
    P("")
    P("Where is the beacon label positive? Mean realised beacon label by N, link and steps since death "
      "(validation labels):\n")
    P("| N | link | steps since death | n | mean label | share +1 | share -1 |")
    P("|---|---|---|---|---|---|---|")
    sd_idx = FEAT.index("since_death")
    for N in (2, 3, 5, 10):
        for lk in (0, 1):
            for lo, hi, nm in ((0, 0.01, "0-1"), (0.01, 0.12, "early"), (0.12, 9, "late")):
                m = b & (meta[:, 0] == N) & (meta[:, 3] == lk) & (X[:, sd_idx] * 1 >= lo) & (X[:, sd_idx] < hi)
                if m.sum() >= 20:
                    P(f"| {N} | {['ideal','burst'][lk]} | {nm} | {int(m.sum())} | {y[m].mean():+.4f} | "
                      f"{(y[m]>0).mean():.3f} | {(y[m]<0).mean():.3f} |")
    P("")
    P("Sign errors of the logistic beacon gate at lam = 0 (send iff dV_hat > 0): fraction of "
      "validation beacon states where it sends yet the realised label is -1 (harm) and where it "
      "stays silent though the label is +1 (missed gain):\n")
    P("| N | n | sends | send & label -1 | silent & label +1 | mean label when sends | mean label when silent |")
    P("|---|---|---|---|---|---|---|")
    pred = pk["models"]["logit"]["beacon"].predict(X)
    for N in (2, 3, 5, 10):
        m = b & (meta[:, 0] == N)
        s_ = pred[m] > 0
        yy = y[m]
        P(f"| {N} | {int(m.sum())} | {s_.mean():.3f} | {((yy<0)&s_).mean():.4f} | {((yy>0)&~s_).mean():.4f} | "
          f"{yy[s_].mean() if s_.any() else float('nan'):+.4f} | {yy[~s_].mean() if (~s_).any() else float('nan'):+.4f} |")
    P("")
    P("rec / claim labels: mean realised label rec %+.4f, claim %+.4f (validation); logistic/MLP never reach "
      "a positive predicted value above 1e-3 at any bytes price where it matters, so those gates are silent.\n"
      % (y[k == 0].mean(), y[k == 1].mean()))
    open(a.out, "a").write("\n".join(L_))
    print("\n".join(L_))


def cmd_train(a):
    import pickle
    tr = load_labels(a.train)
    va = load_labels(a.val)
    t0 = time.process_time()
    models, info = fit_models(tr, va)
    cpu_fit = time.process_time() - t0
    t0 = time.process_time()
    rl = {}
    for lam in [float(x) for x in a.rl_lams.split(",")]:
        W, mu, sd, rets = train_rl(lam, tr[0], tr[2], iters=a.rl_iters, log=True)
        rl[lam] = dict(W=W, mu=mu, sd=sd, rets=rets)
    cpu_rl = time.process_time() - t0
    if a.critic:
        t0 = time.process_time()
        d = np.load(a.critic)
        Xc, yc, kc = d["X"], d["y"], d["kind"]
        models["critic"] = {}
        for ki, kind in enumerate(KINDS):
            m = np.where(kc == ki)[0]
            rng = np.random.default_rng(3)
            rng.shuffle(m)
            nv = max(1, len(m) // 7)
            mv, mt = m[:nv], m[nv:]
            mlp = MLP(epochs=30, l2=1e-3).fit(Xc[mt], yc[mt], Xc[mv], yc[mv])
            models["critic"][kind] = CriticDQ(mlp)
            info[kind]["critic_rows"] = int(len(m))
        cpu_rl += time.process_time() - t0
    with open(a.out, "wb") as fh:
        pickle.dump(dict(models=models, info=info, rl=rl, cpu_fit=cpu_fit, cpu_rl=cpu_rl), fh)
    for k, v in info.items():
        print(k, v)
    print(f"cpu fit {cpu_fit:.0f}s rl {cpu_rl:.0f}s")


# ---------------------------------------------------------------- evaluation
EV_FIELDS = ["N", "travel", "link", "fault", "proto", "seed", "ep", "det", "ttd", "H", "bytes",
             "coll", "hfirst", "hn", "hcoll", "dead_before_det"]
BASE_PROTOS = ["IMPLICIT", "HEALTH", "HEALTH_PART", "HEALTH_ONE", "HB_TO", "FULL", "HYBRID_P",
               "ORACLE"]


def herd_stats(e, nominal):
    """(survivors redirecting first post-death into an orphan cell, survivors with a post-death
    decision, same-step same-target survivor pairs after the death)."""
    deads = [(i, td) for i, td in enumerate(e.t_dead) if td is not None]
    if not deads:
        return 0, 0, 0
    dead, td = deads[0]
    orph = {tk.cell for tk in nominal[dead] if tk.start >= td}
    first, seen = 0, set()
    by_t = {}
    for (t, i, c, v) in e.decisions:
        if i == dead or t < td:
            continue
        by_t.setdefault(t, []).append((c, v))
        if i in seen:
            continue
        seen.add(i)
        first += int(c in orph)
    coll = 0
    for lst in by_t.values():
        for x in range(len(lst)):
            for y in range(x + 1, len(lst)):
                coll += int(lst[x] == lst[y])
    return first, len(seen), coll


def run_proto(proto, inst, N, link, ep, nom, fs, gates):
    S, _ = strat()
    if proto.startswith("G:"):
        pol = gates[proto]
        e = make_ep(inst, N, link, ep, nom, fs, policy=pol)
        r = e.run()
        return r, e
    if proto in ("BEACON_ONCE", "BEACON_REP3"):
        e = make_ep(inst, N, link, ep, nom, fs, policy=pol_rule(proto))
        return e.run(), e
    from .nbeacon import run_nb
    return run_nb(inst, N, proto, LINKS[link], ep, S, nom, fs, keep=True)


def build_gates(pk, lams=LAMS, rl_lams=None):
    g = {}
    for fam in ("logit", "mlp", "critic"):
        if fam not in pk["models"]:
            continue
        for lam in lams:
            g[f"G:{fam}:{lam:g}"] = ModelPolicy(pk["models"][fam], lam)
    cache = {}
    for lam in lams:
        g[f"G:mv:{lam:g}"] = MVPolicy(lam, cache=cache)
    for lam, d in pk["rl"].items():
        g[f"G:rl:{lam:g}"] = RLPolicy(d["W"], d["mu"], d["sd"], greedy=True)
    return g


def cmd_eval(a):
    import csv
    import gzip
    import pickle
    pk = pickle.load(open(a.models, "rb"))
    gates = build_gates(pk)
    protos = BASE_PROTOS + ["BEACON_ONCE", "BEACON_REP3"] + list(gates)
    if a.protos:
        protos = a.protos.split(",")
    S, _ = strat()
    Ns = [int(x) for x in a.Ns.split(",")]
    seeds = EVAL_SEEDS[a.seed0:a.seed0 + a.nseeds]
    _check_seeds(seeds)
    rows = []
    t0 = time.time()
    for N in Ns:
        for travel in [int(x) for x in a.travels.split(",")]:
            for seed in seeds:
                inst = build(N, seed, 1, travel, HZ[(N, travel)])
                nom = nominal_plan_ns(inst, N, S)
                for fa in ("none", "perm1"):
                    fs = fault_spec(fa, inst.horizon, N)
                    for lk in ("ideal", "burst30_d1"):
                        for ep in range(a.E):
                            for p in protos:
                                r, e = run_proto(p, inst, N, lk, ep, nom, fs, gates)
                                hf, hn, hc = herd_stats(e, nom) if fa == "perm1" else (0, 0, 0)
                                dead_before = int(fa == "perm1" and e.t_dead[[i for i, x in enumerate(e.t_dead) if x is not None][0]] <= r.ttd)
                                rows.append((N, travel, lk, fa, p, seed, ep, int(r.detected), r.ttd,
                                             inst.horizon, r.bytes_sent, r.collisions, hf, hn, hc,
                                             dead_before))
        print(f"[{time.time()-t0:7.0f}s] eval N={N} rows={len(rows)} cpu={time.process_time():.0f}s",
              file=sys.stderr, flush=True)
        with gzip.open(a.out, "wt", newline="") as fh:
            w = csv.writer(fh)
            w.writerow(EV_FIELDS)
            w.writerows(rows)
    print(f"wrote {a.out} rows={len(rows)} wall={time.time()-t0:.0f}s cpu={time.process_time():.0f}s")


# ---------------------------------------------------------------- analysis
def load_eval(paths):
    import csv
    import gzip
    raw = {}
    for p in paths:
        with gzip.open(p, "rt") as fh:
            for r in csv.DictReader(fh):
                k = (int(r["N"]), int(r["travel"]), r["link"], r["fault"], r["proto"])
                raw.setdefault(k, {}).setdefault(int(r["seed"]), {})[int(r["ep"])] = (
                    int(r["det"]), float(r["bytes"]), int(r["hfirst"]), int(r["hn"]),
                    int(r["hcoll"]), int(r["dead_before_det"]), int(r["ttd"]) / int(r["H"]))
    D = {}
    for k, bys in raw.items():
        seeds = sorted(bys)
        eps = sorted(bys[seeds[0]])
        A = np.array([[bys[s][e] for e in eps] for s in seeds], float)
        D[k] = dict(det=A[..., 0], bytes=A[..., 1], hf=A[..., 2], hn=A[..., 3], hc=A[..., 4],
                    db=A[..., 5], ttd=A[..., 6])
    return D


def pooled(D, N, travel, fault, proto, metric, links=("ideal", "burst30_d1")):
    return np.concatenate([D[(N, travel, lk, fault, proto)][metric] for lk in links], axis=1)


def jboot(arrs, fn, B=1000, seed=0):
    """Joint cluster bootstrap (instances, then episodes within instance) of fn(dict of
    pooled means), the SAME draws for every array so paired structure is kept."""
    rng = np.random.default_rng(seed)
    k = next(iter(arrs.values()))
    S_, E = k.shape
    ss = rng.integers(S_, size=(B, S_))
    ee = rng.integers(E, size=(B, S_, E))
    means = {n: X[ss[:, :, None], ee].mean(axis=(1, 2)) for n, X in arrs.items()}
    vals = fn(means)
    return np.percentile(vals, [2.5, 97.5])


def fam_points(D, N, travel, fault, fam):
    pts = []
    for k in D:
        if k[:4] == (N, travel, "ideal", fault) and k[4].startswith(f"G:{fam}:"):
            pts.append((float(k[4].split(":")[2]), k[4]))
    pts.sort()
    out = []
    for lam, name in pts:
        b = pooled(D, N, travel, fault, name, "bytes").mean()
        out.append((lam, name, b, pooled(D, N, travel, fault, name, "det").mean()))
    return out


def matched(D, N, travel, fault, fam, target_bytes, ref):
    """Family curve interpolated at target_bytes -> (per-episode array, flag)."""
    pts = fam_points(D, N, travel, fault, fam)
    pts = sorted(pts, key=lambda p: (p[2], -p[3]))
    # upper-left Pareto hull in (bytes, P)
    hull, bestp = [], -1
    for p in pts:
        if p[3] > bestp:
            hull.append(p); bestp = p[3]
    bs = np.array([p[2] for p in hull])
    flag = ""
    if target_bytes <= bs[0]:
        flag = "below-range"; lo = hi = hull[0]; w = 0.0
    elif target_bytes >= bs[-1]:
        flag = "above-range"; lo = hi = hull[-1]; w = 0.0
    else:
        j = int(np.searchsorted(bs, target_bytes))
        lo, hi = hull[j - 1], hull[j]
        w = (target_bytes - lo[2]) / (hi[2] - lo[2])
    Xlo = pooled(D, N, travel, fault, lo[1], "det")
    Xhi = pooled(D, N, travel, fault, hi[1], "det")
    return (1 - w) * Xlo + w * Xhi, flag, (lo[0], hi[0], w)


def fmtci(m, lo, hi):
    star = "*" if lo > 0 or hi < 0 else ""
    return f"{m:+.3f} [{lo:+.3f},{hi:+.3f}]{star}"


def cmd_analyze(a):
    import pickle
    D = load_eval(a.eval)
    pk = pickle.load(open(a.models, "rb"))
    Ns = sorted({k[0] for k in D})
    travel = 1
    out = []
    P = out.append
    P("# D1 learned decision-aware communication: DEV results\n")
    P("**DEV ONLY.** Label seeds 2000-2007 (train) and 2008-2011 (validation); evaluation on fresh "
      "instances 2100-2115. No held-out seed (3000-3015, 4000-4015, 5000-5031, 6000-6015) is used. "
      "Nothing here is preregistered; the confirmatory plan is `PREREG_LEARNEDCOMM.md` (DRAFT).\n")
    for k, v in pk["info"].items():
        pass
    P("## 1. Label models (validation split, instances 2008-2011)\n")
    P("| message | n labels | nonzero | mean dV | val MSE const | val MSE logistic | val MSE MLP |")
    P("|---|---|---|---|---|---|---|")
    for k, v in pk["info"].items():
        P(f"| {k} | {v['n']} | {v['n_nonzero']} | {v['mean']:+.4f} | {v['val_mse_const']:.5f} | "
          f"{v['val_mse_logit']:.5f} | {v['val_mse_mlp']:.5f} |")
    P("")
    for N in Ns:
        P(f"## 2.N{N}. N = {N}: protocols, travel 1, pooled over ideal and burst30_d1 (fresh DEV instances)\n")
        for fault in ("perm1", "none"):
            ref = pooled(D, N, travel, fault, "IMPLICIT", "det")
            ora = pooled(D, N, travel, fault, "ORACLE", "det")
            P(f"### fault = {fault}\n")
            P("| protocol | P(detect) | bytes/ep | vs IMPLICIT (paired 95% CI) | frac of ORACLE gain |")
            P("|---|---|---|---|---|")
            protos = sorted({k[4] for k in D if k[:2] == (N, travel) and k[3] == fault},
                            key=lambda p: (p.startswith("G:"), p))
            for p in protos:
                X = pooled(D, N, travel, fault, p, "det")
                B = pooled(D, N, travel, fault, p, "bytes")
                d = X - ref
                lo, hi = jboot({"d": d}, lambda m: m["d"], B=500)
                gain = (ora - ref).mean()
                fr = f"{d.mean()/gain:.2f}" if (abs(gain) > 0.005 and fault == "perm1") else "n/a"
                P(f"| {p} | {X.mean():.3f} | {B.mean():.1f} | {fmtci(d.mean(), lo, hi)} | {fr} |")
            P("")
    P("## 3. Matched-bytes comparison against HEALTH_PART / HEALTH (perm1, travel 1; the kill check)\n")
    P("Gate curve (lambda sweep) interpolated on its upper-left Pareto hull at HEALTH_PART's mean "
      "bytes. Difference = P(gate at matched bytes) - P(reference), paired cluster bootstrap "
      "(instances then episodes), 95% CI. `kill` = 95% CI inside +/-0.02.\n")
    P("| N | family | ref | ref bytes | diff vs ref | range flag | within +/-2pp (CI) | point within | frac ORACLE gain (gate / ref) |")
    P("|---|---|---|---|---|---|---|---|---|")
    summary = {}
    for N in Ns:
        hp_b = pooled(D, N, travel, "perm1", "HEALTH_PART", "bytes").mean()
        ref = pooled(D, N, travel, "perm1", "IMPLICIT", "det")
        ora = pooled(D, N, travel, "perm1", "ORACLE", "det")
        for fam in ("logit", "mlp", "rl", "critic", "mv"):
            if not fam_points(D, N, travel, "perm1", fam):
                continue
            X, flag, w = matched(D, N, travel, "perm1", fam, hp_b, "HEALTH_PART")
            for rname in ("HEALTH_PART", "HEALTH"):
                R = pooled(D, N, travel, "perm1", rname, "det")
                d = X - R
                lo, hi = jboot({"d": d}, lambda m: m["d"], B=500)
                gain = (ora - ref).mean()
                gfrac = f"{(X-ref).mean()/gain:.2f} / {(R-ref).mean()/gain:.2f}" if abs(gain) > 0.005 else "n/a"
                inside = (lo > -0.02) and (hi < 0.02)
                pin = abs(d.mean()) <= 0.02
                P(f"| {N} | {fam} | {rname} | {hp_b:.1f} | {fmtci(d.mean(), lo, hi)} | {flag} | "
                  f"{'yes' if inside else 'no'} | {'yes' if pin else 'no'} | {gfrac} |")
                summary[(N, fam, rname)] = (d.mean(), lo, hi, inside, pin)
    P("")
    P("## 3b. Kill check against the execution-time myopic-VoI gate (G:mv) and G3 label source (perm1, travel 1)\n")
    P("G:mv has no tunable price in the useful range (its rollout value is quantised in steps of 1/R = 0.25, so every lambda in the grid gives the same decision); it is a single point. Difference = P(family at G:mv bytes, interpolated) - P(G:mv); critic vs rollout: P(logit at critic-matched bytes) - P(critic).\n")
    P("| N | family | ref | ref bytes | diff | range flag | CI inside +/-2pp |")
    P("|---|---|---|---|---|---|---|")
    for N in Ns:
        if (N, travel, "ideal", "perm1", "G:mv:0") not in D:
            continue
        mvb = pooled(D, N, travel, "perm1", "G:mv:0", "bytes").mean()
        Rm = pooled(D, N, travel, "perm1", "G:mv:0", "det")
        for fam in ("logit", "mlp", "rl", "critic"):
            if not fam_points(D, N, travel, "perm1", fam):
                continue
            X, flag, w = matched(D, N, travel, "perm1", fam, mvb, "G:mv")
            d = X - Rm
            lo, hi = jboot({"d": d}, lambda m: m["d"], B=500)
            P(f"| {N} | {fam} | G:mv | {mvb:.1f} | {fmtci(d.mean(), lo, hi)} | {flag} | {'yes' if (lo > -0.02 and hi < 0.02) else 'no'} |")
    for N in Ns:
        for tb in (0.05,):
            pass
    P("")
    P("## 4. Pareto fronts (perm1, travel 1; per-N, pooled over links)\n")
    for N in Ns:
        P(f"### N = {N}\n")
        P("| family | lambda | bytes/ep | P(detect) | vs IMPLICIT |")
        P("|---|---|---|---|---|")
        ref = pooled(D, N, travel, "perm1", "IMPLICIT", "det").mean()
        for fam in ("logit", "mlp", "rl", "critic", "mv"):
            for (lam, name, b, p) in fam_points(D, N, travel, "perm1", fam):
                P(f"| {fam} | {lam:g} | {b:.1f} | {p:.3f} | {p-ref:+.3f} |")
        for p in ("IMPLICIT", "HEALTH", "HEALTH_PART", "HEALTH_ONE", "HB_TO", "BEACON_REP3", "FULL", "HYBRID_P", "ORACLE"):
            if (N, travel, "ideal", "perm1", p) in D:
                P(f"| {p} | - | {pooled(D, N, travel, 'perm1', p, 'bytes').mean():.1f} | "
                  f"{pooled(D, N, travel, 'perm1', p, 'det').mean():.3f} | "
                  f"{pooled(D, N, travel, 'perm1', p, 'det').mean()-ref:+.3f} |")
        P("")
    P("## 5. Herding index (perm1, episodes where the death precedes detection)\n")
    P("HI = survivors whose FIRST post-death decision targets one of the dead agent's orphan cells "
      "/ survivors with a post-death decision (ratio of sums, instance-cluster bootstrap CI). "
      "coll = same-step identical (cell, view) survivor pairs after the death, per episode.\n")
    P("| N | protocol | HI | 95% CI | coll/ep |")
    P("|---|---|---|---|---|")
    for N in Ns:
        for p in ("IMPLICIT", "HEALTH", "HEALTH_PART", "BEACON_REP3", "FULL", "ORACLE") + tuple(
                x for x in a.herd_gates.split(",") if x):
            if (N, travel, "ideal", "perm1", p) not in D:
                continue
            hf = pooled(D, N, travel, "perm1", p, "hf")
            hn = pooled(D, N, travel, "perm1", p, "hn")
            hc = pooled(D, N, travel, "perm1", p, "hc")
            db = pooled(D, N, travel, "perm1", p, "db")
            hfm, hnm = hf * db, hn * db
            lo, hi = jboot({"a": hfm, "b": hnm}, lambda m: m["a"] / np.maximum(m["b"], 1e-9), B=500)
            P(f"| {N} | {p} | {hfm.sum()/max(hnm.sum(),1):.3f} | [{lo:.3f},{hi:.3f}] | {(hc*db).mean():.3f} |")
    P("")
    P("## 6. Healthy teams (fault none): bytes and P(detect) of the gated protocols\n")
    P("| N | protocol | bytes/ep | vs IMPLICIT |")
    P("|---|---|---|---|")
    for N in Ns:
        ref = pooled(D, N, travel, "none", "IMPLICIT", "det")
        for p in ("HYBRID_P", "FULL", "ORACLE") + tuple(sorted(k[4] for k in D if k[:4] == (N, travel, "ideal", "none") and k[4].startswith("G:") and k[4].split(":")[2] in ("0", "1e-05", "0.0001"))):
            if (N, travel, "ideal", "none", p) in D:
                X = pooled(D, N, travel, "none", p, "det")
                d = X - ref
                lo, hi = jboot({"d": d}, lambda m: m["d"], B=300)
                P(f"| {N} | {p} | {pooled(D, N, travel, 'none', p, 'bytes').mean():.1f} | {fmtci(d.mean(), lo, hi)} |")
    P("")
    open(a.out, "w").write("\n".join(out))
    import json
    json.dump({f"{k[0]}|{k[1]}|{k[2]}": [float(x) if not isinstance(x, (bool, np.bool_)) else bool(x) for x in v]
               for k, v in summary.items()}, open(a.out + ".summary.json", "w"), indent=1)
    print("\n".join(out[-60:]))


# ---------------------------------------------------------------- CLI: measure + labels
def cmd_labels(a):
    cells = default_cells(Ns=tuple(int(x) for x in a.Ns.split(',')))
    kpk = dict(rec=a.krec, claim=a.kclaim, beacon=a.kbeacon)
    n, secs, nr = gen_labels(cells, LABEL_SEEDS if a.split == "train" else VAL_SEEDS,
                             range(a.e0, a.e0 + a.E), kpk, a.out, time_limit=a.time_limit, tag=a.split)
    print(f"labels={n} rollouts={nr} wall={secs:.0f}s cpu={time.process_time():.0f}s")


def main():
    ap = argparse.ArgumentParser()
    sp = ap.add_subparsers(dest="cmd")
    p = sp.add_parser("labels")
    p.add_argument("--out", required=True)
    p.add_argument("--split", default="train")
    p.add_argument("--E", type=int, default=2)
    p.add_argument("--e0", type=int, default=0)
    p.add_argument("--krec", type=int, default=2)
    p.add_argument("--kclaim", type=int, default=2)
    p.add_argument("--kbeacon", type=int, default=3)
    p.add_argument("--Ns", default="2,3,5,10")
    p.add_argument("--time-limit", type=float, default=None)
    p = sp.add_parser("train")
    p.add_argument("--train", nargs="+", required=True)
    p.add_argument("--val", nargs="+", required=True)
    p.add_argument("--out", required=True)
    p.add_argument("--rl-lams", default="1e-5,1e-4,1e-3")
    p.add_argument("--rl-iters", type=int, default=150)
    p.add_argument("--critic", default="")
    p = sp.add_parser("diagnose")
    p.add_argument("--models", required=True)
    p.add_argument("--val", nargs="+", required=True)
    p.add_argument("--out", required=True)
    p = sp.add_parser("rl")
    p.add_argument("--models", required=True)
    p.add_argument("--train", nargs="+", required=True)
    p.add_argument("--lam", required=True)
    p.add_argument("--iters", type=int, default=200)
    p.add_argument("--seed", type=int, default=1)
    p = sp.add_parser("critic")
    p.add_argument("--out", required=True)
    p.add_argument("--E", type=int, default=12)
    p = sp.add_parser("eval")
    p.add_argument("--models", required=True)
    p.add_argument("--out", required=True)
    p.add_argument("--Ns", default="2,3,5,10")
    p.add_argument("--travels", default="1")
    p.add_argument("--E", type=int, default=20)
    p.add_argument("--nseeds", type=int, default=16)
    p.add_argument("--seed0", type=int, default=0)
    p.add_argument("--protos", default="")
    p = sp.add_parser("analyze")
    p.add_argument("--eval", nargs="+", required=True)
    p.add_argument("--models", required=True)
    p.add_argument("--out", required=True)
    p.add_argument("--herd-gates", default="")
    a = ap.parse_args()
    {"diagnose": cmd_diagnose, "rl": cmd_rl, "critic": cmd_critic, "analyze": cmd_analyze, "labels": cmd_labels, "train": cmd_train, "eval": cmd_eval}[a.cmd](a)


if __name__ == "__main__":
    main()
