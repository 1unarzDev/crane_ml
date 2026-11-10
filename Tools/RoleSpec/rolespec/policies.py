# SPDX-License-Identifier: GPL-3.0-or-later
"""Policies. Every non-oracle policy uses the SAME scorer (`score_actions`) and differs only in its allowed action
set (speed x cell-class mask) and role logic, so effects are attributable to role structure.

Action = (cell c, view v, speed s). Score (rate):
    [ p_c * P(high alert | present, history at (c, v), s)  +  alpha * P(posterior crosses tau after this pass) ]
    * (1 - beta * claimed_by_other(c))  /  (travel time + scan time)
Cells already confirmed in the drone's own belief score 0. `claimed_by_other` uses teammates' last KNOWN target
(fresh within CLAIM_TTL), i.e. intent sharing through the same lossy comms.

Policies:
  HOMO_FAST            all drones, FAST only.
  HOMO_THOROUGH        all drones, SLOW only.
  HOMO_SHARED_REVISIT  all drones, both speeds, every cell: new coverage vs revisits by expected gain per unit time.
  SPEC_FIXED(r)        round(r N) recon drones (FAST, non-candidate cells; with verify_by_self they may use the
                       full action set), the rest verifiers (SLOW, candidate cells p >= p_cand or low-confidence
                       cells: looked at and p >= lowconf * p0); idle verifiers fall back to `fallback` coverage.
  SPEC_ADAPTIVE        role chosen at every decision from the drone's own belief and comm state:
                       rule='threshold' (response threshold): verifier iff backlog per verifier >
                         theta * uncovered prior mass per recon (hysteresis h);
                       rule='marginal': role whose best action scores highest (current role x (1 + h)).
                       comm_self_verify: own candidates never shared with anyone (no contact since the alert) are
                       verifiable by the recon that found them.
  ORACLE (labelled)    knows survivor cells (not their view latents); greedy assignment; SLOW best-view passes.
"""
import numpy as np

from . import sensing as S
from .belief import logit, sigmoid

CLAIM_TTL = 120.0
ROLE_ALL, ROLE_RECON, ROLE_VERIFY = "all", "recon", "verify"


def score_actions(dv, allowed, alpha, beta, home=None, sw=0.0):
    """Return (best (c, v, s) or None, best score, score array). home/sw: soft pre-launch sector preference
    (gain outside the drone's own sector is scaled by 1 - sw; sw = 0 disables it)."""
    p, P1o, P0o = dv.belief.predictive()
    tau = dv.pub.sc.tau_conf
    det = p[:, None, None] * P1o[..., 2]                                   # (C, V, S)
    if alpha > 0:
        lp = logit(p)[:, None, None, None]
        post = sigmoid(lp + np.log(P1o) - np.log(P0o))
        po = p[:, None, None, None] * P1o + (1 - p)[:, None, None, None] * P0o
        cross = (po * (post >= tau)).sum(-1)
        gain = det + alpha * cross
    else:
        gain = det
    gain = gain * (p < tau)[:, None, None]
    if beta > 0 and dv.claimed is not None:
        gain = gain * (1.0 - beta * dv.claimed)[:, None, None]
    if home is not None and sw > 0:
        gain = gain * np.where(home, 1.0, 1.0 - sw)[:, None, None]
    t = dv.travel_t[:, None, None] + dv.pub.tables.scan_t[:, None, :]
    sc = np.where(allowed & dv.feasible[:, None, :], gain / t, -1.0)
    i = int(np.argmax(sc))
    if sc.flat[i] <= 0:
        return None, 0.0, sc
    c, v, s = np.unravel_index(i, sc.shape)
    return (int(c), int(v), int(s)), float(sc.flat[i]), sc


def _mask(C, speeds, cells=None):
    m = np.zeros((C, 4, 2), bool)
    for s in speeds:
        m[:, :, s] = True
    if cells is not None:
        m &= cells[:, None, None]
    return m


class Policy:
    name = "base"
    oracle = False

    def __init__(self, **params):
        self.params = dict(self.DEFAULTS)
        unknown = set(params) - set(self.DEFAULTS)
        if unknown:
            raise ValueError("unknown params %s for %s" % (unknown, self.name))
        self.params.update(params)

    def reset(self, pub):
        self.pub = pub
        self._sec = {}

    def home(self, dv, k=None, idx=None):
        """Own band of a k-way pre-launch partition (default: all N drones, band = drone id)."""
        k = dv.pub.n if k is None else k
        if k not in self._sec:
            self._sec[k] = sectors(dv.pub, k)
        return self._sec[k] == (dv.me if idx is None else idx)

    def role(self, dv):
        return ROLE_ALL

    def decide(self, dv):
        raise NotImplementedError

    def label(self):
        return self.name


class Homo(Policy):
    DEFAULTS = {"alpha": 0.5, "beta": 0.8, "sector_w": 0.0}
    speeds = (S.FAST, S.SLOW)

    def decide(self, dv):
        a, _, _ = score_actions(dv, _mask(dv.pub.tables.C, self.speeds), self.params["alpha"], self.params["beta"],
                                self.home(dv), self.params["sector_w"])
        return a


class HomoFast(Homo):
    name = "HOMO_FAST"
    speeds = (S.FAST,)


class HomoThorough(Homo):
    name = "HOMO_THOROUGH"
    speeds = (S.SLOW,)


class HomoSharedRevisit(Homo):
    name = "HOMO_SHARED_REVISIT"
    speeds = (S.FAST, S.SLOW)


def _classes(dv, p_cand, lowconf):
    p = dv.belief.posterior()
    tau = dv.pub.sc.tau_conf
    cand = (p >= p_cand) & (p < tau)
    looked = dv.belief.looks.sum(1) > 0
    low = looked & (p >= lowconf * dv.pub.p0) & (p < tau)
    return cand, low


class _RoleBased(Policy):
    def _recon_mask(self, dv, vbs):
        C = dv.pub.tables.C
        if vbs:
            return _mask(C, (S.FAST, S.SLOW))
        cand, _ = _classes(dv, self.params["p_cand"], self.params["lowconf"])
        m = _mask(C, (S.FAST,), ~cand)
        return m

    def _verify_mask(self, dv):
        C = dv.pub.tables.C
        cand, low = _classes(dv, self.params["p_cand"], self.params["lowconf"])
        return _mask(C, (S.SLOW,), cand | low)

    def _fallback(self, dv):
        C = dv.pub.tables.C
        fb = self.params["fallback"]
        if fb == "slow_cover":
            return _mask(C, (S.SLOW,))
        if fb == "fast_cover":
            return self._recon_mask(dv, False)
        return _mask(C, (S.FAST, S.SLOW))  # "any"

    def recon_home(self, dv):
        return self.home(dv)

    def _act(self, dv, role):
        a, b = self.params["alpha"], self.params["beta"]
        sw = self.params.get("sector_w", 0.0)
        if role == ROLE_RECON:
            hm = self.recon_home(dv)
            act, _, _ = score_actions(dv, self._recon_mask(dv, self.params.get("verify_by_self", False)), a, b, hm, sw)
            if act is None:
                act, _, _ = score_actions(dv, _mask(dv.pub.tables.C, (S.FAST, S.SLOW)), a, b, hm, sw)
            return act
        act, _, _ = score_actions(dv, self._verify_mask(dv), a, b)
        if act is None:
            act, _, _ = score_actions(dv, self._fallback(dv), a, b)
        return act


class SpecFixed(_RoleBased):
    name = "SPEC_FIXED"
    DEFAULTS = {"r": 0.5, "p_cand": 0.15, "lowconf": 0.6, "fallback": "slow_cover", "verify_by_self": False,
                "alpha": 0.5, "beta": 0.8, "sector_w": 0.0}

    def recon_home(self, dv):
        """Recon drones 1..k partition the map among themselves; verifiers roam."""
        return self.home(dv, k=self.n_recon(dv.pub.n), idx=dv.me)

    def n_recon(self, n):
        r = self.params["r"]
        return int(min(n, max(1, round(r * n)))) if r > 0 else 0

    def role(self, dv):
        return ROLE_RECON if dv.me <= self.n_recon(dv.pub.n) else ROLE_VERIFY

    def decide(self, dv):
        return self._act(dv, self.role(dv))

    def label(self):
        return "SPEC_FIXED(r=%.2f)" % self.params["r"]


class SpecAdaptive(_RoleBased):
    name = "SPEC_ADAPTIVE"
    DEFAULTS = {"rule": "threshold", "theta": 1.0, "h": 0.2, "p_cand": 0.15, "lowconf": 0.6, "stim_range_m": 0.0,
                "fallback": "slow_cover", "comm_self_verify": True, "broad_roles": False,
                "alpha": 0.5, "beta": 0.8, "sector_w": 0.0}

    def reset(self, pub):
        super().reset(pub)
        self.cur = {}

    def _broad(self, dv, role):
        C = dv.pub.tables.C
        return _mask(C, (S.FAST,) if role == ROLE_RECON else (S.SLOW,))

    def _recon_mask(self, dv, vbs):
        if self.params["broad_roles"]:
            return self._broad(dv, ROLE_RECON)
        m = super()._recon_mask(dv, False)
        if self.params["comm_self_verify"] and dv.own_unshared is not None and dv.own_unshared.any():
            m = m | _mask(dv.pub.tables.C, (S.SLOW,), dv.own_unshared)
        return m

    def _verify_mask(self, dv):
        if self.params["broad_roles"]:
            return self._broad(dv, ROLE_VERIFY)
        return super()._verify_mask(dv)

    def role(self, dv):
        return self.cur.get(dv.me, ROLE_RECON)

    def decide(self, dv):
        P = self.params
        cur = self.cur.get(dv.me, ROLE_RECON)
        if P["rule"] == "marginal":
            a, b = P["alpha"], P["beta"]
            ar, sr, _ = score_actions(dv, self._recon_mask(dv, False), a, b, self.home(dv), P["sector_w"])
            av, sv, _ = score_actions(dv, self._verify_mask(dv), a, b)
            if cur == ROLE_RECON:
                sr *= 1.0 + P["h"]
            else:
                sv *= 1.0 + P["h"]
            role = ROLE_RECON if sr >= sv else ROLE_VERIFY
            if ar is None and av is None:
                self.cur[dv.me] = role
                return self._act(dv, role)
            self.cur[dv.me] = role
            act = ar if role == ROLE_RECON else av
            if act is None:
                act = self._act(dv, role)
            return act
        # response threshold
        p = dv.belief.posterior()
        cand, low = _classes(dv, P["p_cand"], P["lowconf"])
        bmask = cand | low
        if P["stim_range_m"] > 0:
            bmask = bmask & (dv.travel_t * dv.pub.sc.v_transit <= P["stim_range_m"])
        backlog = float((p * bmask).sum())
        looked = dv.belief.looks.sum(1) > 0
        uncovered = float((p * ~looked).sum())
        roles = [r for (_, _, r) in dv.known_roles()]
        n_v = sum(1 for r in roles if r == ROLE_VERIFY)
        n_r = sum(1 for r in roles if r == ROLE_RECON)
        stim_v = backlog / (n_v + 1.0)
        stim_r = uncovered / (n_r + 1.0)
        if cur == ROLE_RECON:
            role = ROLE_VERIFY if stim_v > P["theta"] * stim_r * (1 + P["h"]) else ROLE_RECON
        else:
            role = ROLE_RECON if P["theta"] * stim_r > stim_v * (1 + P["h"]) else ROLE_VERIFY
        self.cur[dv.me] = role
        return self._act(dv, role)


class Oracle(Policy):
    """LABELLED ORACLE: knows the survivor cells (truth), not their view latents. Upper-bound reference only."""
    name = "ORACLE"
    oracle = True
    DEFAULTS = {}

    def reset(self, pub, truth_cells=None):
        super().reset(pub)
        self.cells = list(truth_cells or [])
        self.assign = {}

    def decide(self, dv):
        p, P1o, _ = dv.belief.predictive()
        tau = dv.pub.sc.tau_conf
        open_cells = [c for c in self.cells if p[c] < tau]
        if not open_cells:
            return None
        taken = {c for d, c in self.assign.items() if d != dv.me and c in open_cells}
        pool = [c for c in open_cells if c not in taken] or open_cells
        c = min(pool, key=lambda x: (dv.travel_t[x], x))
        if not dv.feasible[c, S.SLOW]:
            return None
        self.assign[dv.me] = c
        v = int(np.argmax(P1o[c, :, S.SLOW, 2]))
        return (c, v, S.SLOW)


def _open_candidates(dv, p_cand):
    p = dv.belief.posterior()
    tau = dv.pub.sc.tau_conf
    return np.nonzero((p >= p_cand) & (p < tau))[0]


def _teammate_xy(dv, j, fresh_s):
    st = dv.status.get(j)
    if st is None or dv.t - st[0] > fresh_s or st[1] is None or st[1] < 0:
        return None
    return dv.pub.centers[st[1]]


class JinConfirm(_RoleBased):
    """B3 (Jin et al. 2003/2006, adapted): every drone searches; each open candidate spawns a Confirm task taken by
    the nearest FREE drone (decentralised: a drone takes it if no fresher-known teammate is nearer and none has
    claimed it). Confirm = SLOW pass(es) with next-best-view until the cell is confirmed or drops below p_cand."""
    name = "B3_JIN_CONFIRM"
    DEFAULTS = {"p_cand": 0.15, "lowconf": 0.6, "search_speeds": "fast", "alpha": 0.5, "beta": 0.8,
                "fallback": "any", "sector_w": 0.0}

    def reset(self, pub):
        super().reset(pub)
        self.task = {}

    def role(self, dv):
        return ROLE_VERIFY if self.task.get(dv.me) is not None else ROLE_RECON

    def decide(self, dv):
        P = self.params
        p = dv.belief.posterior()
        cur = self.task.get(dv.me)
        if cur is not None and not (P["p_cand"] <= p[cur] < dv.pub.sc.tau_conf):
            cur = None
        if cur is None:
            taken = {st[1] for j, st in dv.status.items() if j != dv.me and st is not None
                     and dv.t - st[0] <= CLAIM_TTL and st[2] == ROLE_VERIFY}
            best = None
            for c in _open_candidates(dv, P["p_cand"]):
                if c in taken:
                    continue
                d_me = dv.travel_t[c] * dv.pub.sc.v_transit
                nearer = False
                for j, st in dv.status.items():
                    if j == dv.me or st is None or dv.t - st[0] > CLAIM_TTL or st[2] == ROLE_VERIFY:
                        continue
                    xy = _teammate_xy(dv, j, CLAIM_TTL)
                    if xy is not None and np.hypot(*(xy - dv.pub.centers[c])) < d_me - 1e-9:
                        nearer = True
                        break
                if not nearer and (best is None or d_me < best[0]):
                    best = (d_me, int(c))
            cur = None if best is None else best[1]
        self.task[dv.me] = cur
        C = dv.pub.tables.C
        if cur is not None:
            cells = np.zeros(C, bool)
            cells[cur] = True
            act, _, _ = score_actions(dv, _mask(C, (S.SLOW,), cells), P["alpha"], 0.0)
            if act is not None:
                return act
            self.task[dv.me] = None
        sp = (S.FAST,) if P["search_speeds"] == "fast" else (S.FAST, S.SLOW)
        cand, _ = _classes(dv, P["p_cand"], P["lowconf"])
        act, _, _ = score_actions(dv, _mask(C, sp, ~cand), P["alpha"], P["beta"], self.home(dv), P["sector_w"])
        if act is None:
            act, _, _ = score_actions(dv, _mask(C, (S.FAST, S.SLOW)), P["alpha"], P["beta"], self.home(dv),
                                      P["sector_w"])
        return act


class Auction(_RoleBased):
    """B6 (market / CBBA-style, single-round greedy auction, adapted): at each decision the drone runs the auction
    over open candidates among ITSELF and the teammates it is CONNECTED to now (heard within one slot); bids =
    travel distance from the bidder's last known target; each bidder wins at most `cap` candidates; unconnected
    teammates neither bid nor are excluded (their claims are invisible unless fresh). Winner verifies (SLOW + NBV);
    others recon. Needs connectivity by construction."""
    name = "B6_AUCTION"
    DEFAULTS = {"p_cand": 0.15, "lowconf": 0.6, "alpha": 0.5, "beta": 0.8, "fallback": "fast_cover",
                "min_p_bid": 0.0, "sector_w": 0.0}

    def reset(self, pub):
        super().reset(pub)
        self.won = {}

    def role(self, dv):
        return ROLE_VERIFY if self.won.get(dv.me) is not None else ROLE_RECON

    def decide(self, dv):
        P = self.params
        sc = dv.pub.sc
        p = dv.belief.posterior()
        bidders = {dv.me: dv.pos}
        for j in dv.status:
            if j != dv.me and dv.contact_t[j] > 0 and dv.t - dv.contact_t[j] <= sc.slot_s + 1e-9:
                xy = _teammate_xy(dv, j, 2 * CLAIM_TTL)
                if xy is not None:
                    bidders[j] = xy
        cands = sorted(_open_candidates(dv, max(P["p_cand"], P["min_p_bid"])), key=lambda c: (-p[c], c))
        assigned = {}
        free = set(bidders)
        for c in cands:
            if not free:
                break
            j = min(free, key=lambda b: (np.hypot(*(bidders[b] - dv.pub.centers[c])), b))
            assigned[j] = int(c)
            free.discard(j)
        mine = assigned.get(dv.me)
        self.won[dv.me] = mine
        C = dv.pub.tables.C
        if mine is not None:
            cells = np.zeros(C, bool)
            cells[mine] = True
            act, _, _ = score_actions(dv, _mask(C, (S.SLOW,), cells), P["alpha"], 0.0)
            if act is not None:
                return act
        taken = np.zeros(C, bool)
        for j, c in assigned.items():
            taken[c] = True
        cand, _ = _classes(dv, P["p_cand"], P["lowconf"])
        act, _, _ = score_actions(dv, _mask(C, (S.FAST,), ~(cand | taken)), P["alpha"], P["beta"], self.home(dv),
                                  P["sector_w"])
        if act is None:
            act, _, _ = score_actions(dv, _mask(C, (S.FAST, S.SLOW)), P["alpha"], P["beta"], self.home(dv),
                                      P["sector_w"])
        return act


def sectors(pub, k=None):
    """Static partition known before launch: k (default N) bands across y with equal prior mass (no comms)."""
    n, N = pub.side, (pub.n if k is None else max(int(k), 1))
    jj = np.arange(pub.side * pub.side) % n
    mass = np.array([pub.prior[jj == j].sum() for j in range(n)])
    cum = np.cumsum(mass) - 0.5 * mass
    band = np.minimum((cum * N).astype(int), N - 1)
    return band[jj] + 1


class Solo(Policy):
    """B7 (Senthilnath et al. 2024, adapted): per-drone detect-then-confirm, NO drone-to-drone comms (drone -> base
    uplink only, for reporting). Each drone runs the full-action-set scorer on its OWN observations; optional
    static sectors (pre-launch partition) prevent everyone searching the same high-prior cells."""
    name = "B7_SOLO"
    no_a2a = True
    DEFAULTS = {"alpha": 0.5, "sectors": True}

    def reset(self, pub):
        super().reset(pub)
        self.sec = sectors(pub)

    def decide(self, dv):
        C = dv.pub.tables.C
        m = _mask(C, (S.FAST, S.SLOW))
        if self.params["sectors"]:
            m &= (self.sec == dv.me)[:, None, None]
        act, _, _ = score_actions(dv, m, self.params["alpha"], 0.0)
        if act is None:
            act, _, _ = score_actions(dv, _mask(C, (S.FAST, S.SLOW)), self.params["alpha"], 0.0)
        return act


REGISTRY = {"B3_JIN_CONFIRM": JinConfirm, "B6_AUCTION": Auction, "B7_SOLO": Solo, "HOMO_FAST": HomoFast, "HOMO_THOROUGH": HomoThorough, "HOMO_SHARED_REVISIT": HomoSharedRevisit,
            "SPEC_FIXED": SpecFixed, "SPEC_ADAPTIVE": SpecAdaptive, "ORACLE": Oracle}


def make(name, **params):
    return REGISTRY[name](**params)
