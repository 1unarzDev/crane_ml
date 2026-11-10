# SPDX-License-Identifier: GPL-3.0-or-later
"""Mission loop (event-driven tasks, comms slots), shared rules and metrics.

Shared by every policy (budget matching): battery, speeds {v_fast, v_slow}, transit speed, sensing tables,
confirmation threshold, comms realisation, reporting rule:
  * energy: a task is feasible only if, after it, the drone can still fly home with a 5 % reserve; otherwise RTB
    and land (a landed drone hands its whole log to the base).
  * reporting / rendezvous: a drone holding a locally confirmed cell that the base did not have at the drone's
    last base contact, and with no base link now, flies toward the base in 10 s legs until it hears the base.
  * comms: per slot, the up-links (Gilbert-Elliott x range; ideal = all up) define connected components; within a
    component logs, teammate status (target cell, role) and contact times are flooded (store-and-forward relay).
Observations are appended to the origin's log when the pass completes; others learn them only through comms.
"""
import heapq
import math

import numpy as np

from . import sensing as S
from .belief import Belief
from .world import World

RESERVE = 0.05
LEG_S = 10.0
WAIT_S = 10.0


class DroneView:
    """The only planner input (no truth)."""
    __slots__ = ("t", "me", "pos", "belief", "pub", "travel_t", "feasible", "claimed", "status", "own_unshared",
                 "energy_left", "contact_t", "last_speed")

    def known_roles(self):
        return [(j, st[1], st[2]) for j, st in self.status.items()
                if j != self.me and st is not None and self.t - st[0] <= 120.0]


class Drone:
    def __init__(self, i, base_xy, battery_wh):
        self.i = i
        self.pos = base_xy.copy()
        self.e_left = battery_wh
        self.e_used = 0.0
        self.active = True
        self.task = None   # dict
        self.last_base = 0.0
        self.delivered_conf = set()
        self.alert_t = {}  # cell -> time of own first alert
        self.last_contact_any = 0.0
        self.last_speed = None
        self.n_switch = 0


def run_mission(seed, sc, policy, allow_heldout=False, record=False, world=None, rep=0):
    w = world or World(seed, sc, allow_heldout=allow_heldout, rep=rep)
    pub = w.pub
    T = pub.tables
    N = sc.n_drones
    nn = N + 1
    D = sc.deadline_s
    tau = sc.tau_conf
    if policy.oracle:
        policy.reset(pub, truth_cells=w.survivor_cells)
    else:
        policy.reset(pub)
    logs = [[] for _ in range(nn)]
    beliefs = [Belief(pub, nn) for _ in range(nn)]
    drones = [None] + [Drone(i, pub.base_xy, sc.battery_wh) for i in range(1, nn)]
    status = {i: {j: None for j in range(1, nn)} for i in range(nn)}   # what node i knows of drone j
    own_status = {j: None for j in range(1, nn)}
    contact = np.zeros((nn, nn))
    dist_home = np.hypot(*(pub.centers - pub.base_xy).T)
    e_home_per_m = S.power_w(sc.v_transit) / 3600.0 / sc.v_transit
    pw_tr = S.power_w(sc.v_transit)
    surv = sorted(set(w.survivor_cells))
    M = len(w.survivor_cells)
    t_first = {c: None for c in surv}
    t_conf = {c: None for c in surv}
    t_rep = {c: None for c in surv}
    false_rep = set()
    pass_time = 0.0
    dup_time = 0.0
    dup_unaware_time = 0.0
    passers = {}
    trace = []
    landed_low = False
    no_a2a = bool(getattr(policy, "no_a2a", False))

    def pos_at(d, t):
        tk = d.task
        if tk is None:
            return d.pos
        if t >= tk["t_arr"]:
            return tk["dst"]
        f = (t - tk["t0"]) / max(tk["t_arr"] - tk["t0"], 1e-9)
        return tk["src"] + f * (tk["dst"] - tk["src"])

    def make_view(d, t):
        dv = DroneView()
        dv.t, dv.me, dv.pos, dv.pub = t, d.i, d.pos, pub
        dv.belief = beliefs[d.i]
        dist = np.hypot(*(pub.centers - d.pos).T)
        dv.travel_t = dist / sc.v_transit
        e_need = dv.travel_t[:, None] * pw_tr / 3600.0 + T.scan_e + (dist_home * e_home_per_m)[:, None]
        sw = np.zeros((1, 2))
        if sc.switch_s > 0 and d.last_speed is not None:
            sw[0, 1 - d.last_speed] = sc.switch_s
        e_need = e_need + sw * S.power_w(0.0) / 3600.0
        dv.feasible = (e_need <= d.e_left - RESERVE * sc.battery_wh) & (t + dv.travel_t[:, None] + T.scan_t + sw <= D)
        cl = np.zeros(T.C)
        for j, st in status[d.i].items():
            if j != d.i and st is not None and t - st[0] <= 120.0 and st[1] is not None and st[1] >= 0:
                cl[st[1]] = 1.0
        dv.claimed = cl
        dv.status = status[d.i]
        dv.energy_left = d.e_left
        own = np.zeros(T.C, bool)
        for c, ta in d.alert_t.items():
            if d.last_contact_any < ta:
                own[c] = True
        dv.own_unshared = own
        dv.contact_t = contact[d.i].copy()
        dv.last_speed = d.last_speed
        return dv

    def start_task(d, t, kind, c=-1, v=-1, s=-1, dst=None, dur_scan=0.0, speed=None):
        src = d.pos.copy()
        dst = pub.centers[c].copy() if dst is None else dst
        dist = float(np.hypot(*(dst - src)))
        t_arr = t + dist / sc.v_transit
        t_end = t_arr + dur_scan
        e = dist / sc.v_transit * pw_tr / 3600.0
        if kind == "scan":
            e += T.scan_e[c, s]
            if sc.switch_s > 0 and d.last_speed is not None and d.last_speed != s:
                t_end += sc.switch_s
                e += sc.switch_s * S.power_w(0.0) / 3600.0
            if d.last_speed is not None and d.last_speed != s:
                d.n_switch += 1
            d.last_speed = s
        elif kind == "wait":
            e += dur_scan * S.power_w(0.0) / 3600.0
        d.e_left -= e
        d.e_used += e
        d.task = {"kind": kind, "c": c, "v": v, "s": s, "src": src, "dst": dst, "t0": t, "t_arr": t_arr,
                  "t_end": t_end}
        return t_end

    def decide(d, t):
        if not d.active:
            return None
        p_own = beliefs[d.i].posterior()
        # shared reporting / rendezvous rule
        undelivered = [c for c in np.nonzero(p_own >= tau)[0] if c not in d.delivered_conf]
        in_base = contact[d.i, 0] > 0 and t - contact[d.i, 0] <= sc.slot_s + 1e-9
        if undelivered and not in_base and sc.comms != "ideal":
            dh = float(np.hypot(*(d.pos - pub.base_xy)))
            if dh > 1.0:
                leg = min(LEG_S * sc.v_transit, dh)
                dst = d.pos + (pub.base_xy - d.pos) * (leg / dh)
                own_status[d.i] = (t, -1, "courier")
                status[d.i][d.i] = own_status[d.i]
                return start_task(d, t, "rv", dst=dst)
            own_status[d.i] = (t, -1, "courier")
            return start_task(d, t, "wait", dst=d.pos.copy(), dur_scan=WAIT_S)
        dv = make_view(d, t)
        act = policy.decide(dv)
        role = policy.role(dv)
        if act is None:
            # nothing feasible / valuable: RTB if energy-limited, else hover-wait
            if not dv.feasible.any():
                own_status[d.i] = (t, -1, role)
                status[d.i][d.i] = own_status[d.i]
                return start_task(d, t, "rtb", dst=pub.base_xy.copy())
            own_status[d.i] = (t, -1, role)
            status[d.i][d.i] = own_status[d.i]
            return start_task(d, t, "wait", dst=d.pos.copy(), dur_scan=WAIT_S)
        c, v, s = act
        own_status[d.i] = (t, c, role)
        status[d.i][d.i] = own_status[d.i]
        if record:
            trace.append((round(t, 3), d.i, c, v, s, role))
        return start_task(d, t, "scan", c, v, s, dur_scan=float(T.scan_t[c, s]))

    def complete(d, t):
        nonlocal pass_time, dup_time, dup_unaware_time
        tk = d.task
        d.pos = tk["dst"].copy()
        if tk["kind"] == "scan":
            c, v, s = tk["c"], tk["v"], tk["s"]
            o = w.look(c, v, s)
            logs[d.i].append((t, c, v, s, o))
            beliefs[d.i].sync(logs, _own_wm(beliefs[d.i], d.i, len(logs[d.i])))
            dt = float(T.scan_t[c, s])
            pass_time += dt
            prev = passers.get((c, v), set())
            if prev - {d.i}:
                dup_time += dt
                if beliefs[d.i].looks[c, v] < w.looks[c, v]:
                    dup_unaware_time += dt
            passers.setdefault((c, v), set()).add(d.i)
            if o >= 1:
                d.alert_t.setdefault(c, t)
                if c in t_first and t_first[c] is None:
                    t_first[c] = t
            if beliefs[d.i].posterior()[c] >= tau and c in t_conf and t_conf[c] is None:
                t_conf[c] = t
        elif tk["kind"] == "rtb":
            d.active = False
        d.task = None

    def _own_wm(b, i, n):
        u = b.wm.copy()
        u[i] = n
        return u

    def comm_slot(t):
        nonlocal landed_low
        act_nodes = [0] + [i for i in range(1, nn) if drones[i].active]
        posn = {0: pub.base_xy}
        for i in act_nodes[1:]:
            posn[i] = pos_at(drones[i], t)
        # landed drones are wired to the base
        parent = list(range(nn))

        def find(x):
            while parent[x] != x:
                parent[x] = parent[parent[x]]
                x = parent[x]
            return x

        for i in range(1, nn):
            if not drones[i].active:
                parent[find(i)] = find(0)
        if no_a2a:
            # drone -> base uplink only: the base absorbs drone logs; drones never learn teammates' data
            for i in range(1, nn):
                if drones[i].active and w.link_up(0, i, posn[0], posn[i], t):
                    beliefs[0].sync(logs, np.maximum(beliefs[0].wm, beliefs[i].wm))
                    contact[i, 0] = contact[0, i] = t
                    drones[i].last_base = t
                    pb = beliefs[0].posterior()
                    drones[i].delivered_conf |= set(np.nonzero(beliefs[i].posterior() >= tau)[0]) & set(np.nonzero(pb >= tau)[0])
                elif not drones[i].active:
                    beliefs[0].sync(logs, np.maximum(beliefs[0].wm, beliefs[i].wm))
            parent = list(range(nn))
        for a_i, a in enumerate(act_nodes):
            if no_a2a:
                break
            for b in act_nodes[a_i + 1:]:
                if find(a) != find(b) and w.link_up(a, b, posn[a], posn[b], t):
                    parent[find(a)] = find(b)
        comps = {}
        for i in range(nn):
            comps.setdefault(find(i), []).append(i)
        for members in comps.values():
            if len(members) < 2:
                continue
            up = np.max([beliefs[i].wm for i in members], 0)
            for i in members:
                beliefs[i].sync(logs, up)
            for j in range(1, nn):
                best = None
                for i in members:
                    st = status[i][j]
                    if st is not None and (best is None or st[0] > best[0]):
                        best = st
                for i in members:
                    status[i][j] = best
            for i in members:
                for k in members:
                    contact[i, k] = t
                if i:
                    drones[i].last_contact_any = t
            if 0 in members:
                pb = beliefs[0].posterior()
                for i in members:
                    if i:
                        drones[i].last_base = t
                        drones[i].delivered_conf = set(np.nonzero(beliefs[i].posterior() >= tau)[0]) \
                            & set(np.nonzero(pb >= tau)[0]) | drones[i].delivered_conf
        # onboard confirmations from merged knowledge, base reports
        for i in range(1, nn):
            pi = beliefs[i].posterior()
            for c in surv:
                if t_conf[c] is None and pi[c] >= tau:
                    t_conf[c] = t
        pb = beliefs[0].posterior()
        for c in np.nonzero(pb >= tau)[0]:
            c = int(c)
            if c in t_rep:
                if t_rep[c] is None:
                    t_rep[c] = t
            else:
                false_rep.add(c)

    # ---------------------------------------------------------------- main loop
    heap = []
    for i in range(1, nn):
        te = decide(drones[i], 0.0)
        if te is not None:
            heapq.heappush(heap, (te, i))
    t_slot = 0.0
    while t_slot <= D + 1e-9:
        while heap and heap[0][0] <= t_slot:
            te, i = heapq.heappop(heap)
            d = drones[i]
            complete(d, te)
            if d.active:
                nt = decide(d, te)
                if nt is not None and nt <= D + LEG_S:
                    heapq.heappush(heap, (nt, i))
        comm_slot(t_slot)
        t_slot += sc.slot_s

    # ---------------------------------------------------------------- metrics
    def cens(x):
        return D if x is None else x

    per_cell_count = {c: w.survivor_cells.count(c) for c in surv}
    rec_rep = sum(per_cell_count[c] for c in surv if t_rep[c] is not None) / M
    rec_conf = sum(per_cell_count[c] for c in surv if t_conf[c] is not None) / M
    wt = np.array([per_cell_count[c] for c in surv], float)
    mean_t = lambda dct: float(np.sum([cens(dct[c]) * per_cell_count[c] for c in surv]) / M)
    first_any = min([x for x in t_first.values() if x is not None], default=None)
    first_rep = min([x for x in t_rep.values() if x is not None], default=None)
    first_conf = min([x for x in t_conf.values() if x is not None], default=None)
    energy = float(sum(drones[i].e_used for i in range(1, nn)))
    fail = "success"
    if rec_rep < 1.0:
        missing = [c for c in surv if t_rep[c] is None]
        if any(t_first[c] is None for c in missing):
            reached = any(w.looks[c].sum() > 0 for c in missing)
            fail = "missed_on_pass" if reached else "never_searched"
        elif any(t_conf[c] is None for c in missing):
            fail = "detected_unconfirmed"
        else:
            fail = "confirmed_unreported"
    if false_rep:
        fail = "false_confirm" if fail == "success" else fail + "+false_confirm"
    out = {
        "seed": seed, "rep": w.rep, "cell": sc.key(), "policy": policy.label(), "M": M,
        "recall_rep": rec_rep, "recall_conf": rec_conf,
        "t_first_det": first_any, "t_first_conf": first_conf, "t_first_rep": first_rep,
        "t_conf_mean_c": mean_t(t_conf), "t_rep_mean_c": mean_t(t_rep),
        "t_first_det_c": cens(first_any), "t_first_rep_c": cens(first_rep),
        "auc_rep": 1.0 - mean_t(t_rep) / D,
        "false_rep": len(false_rep), "energy_wh": energy,
        "dup_frac": dup_time / max(pass_time, 1e-9), "dup_unaware_frac": dup_unaware_time / max(pass_time, 1e-9),
        "n_passes": int(sum(len(l) for l in logs)), "fail": fail,
        "battery_wh_each": sc.battery_wh,
        "n_switch": int(sum(drones[i].n_switch for i in range(1, nn))),
    }
    out["J"] = out["auc_rep"] - 0.25 * out["false_rep"]
    if record:
        out["trace"] = trace
    return out
