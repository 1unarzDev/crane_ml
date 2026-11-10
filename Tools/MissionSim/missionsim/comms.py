# SPDX-License-Identifier: GPL-3.0-or-later
"""Radio network: per-direction Gilbert-Elliott burst loss, margin-dependent rate, delay with jitter (out-of-order),
hop-by-hop custody (store-and-forward with acks), base relay, partner data-mule relay, message priorities.

GE parametrisation follows ForestSearch `links.GE` (p_gb, p_bg, loss_good, loss_bad) but the transition
probabilities depend on the instantaneous link margin (dB) from the RF truth field, and a margin below zero forces
the bad state. Transition and loss uniforms are pre-drawn per directed link and tick so paired policies share the
channel realisation as far as their trajectories allow.

Nodes: 0 = base station, 1 and 2 = drones.
"""
import math

import numpy as np

from . import world as W

PRIO = {"ACK": 0, "DET": 0, "VERIFY": 0, "HB": 1, "LEASE": 1, "ASSIGN": 1, "COV": 2, "MAP": 3, "IMG": 4, "ACKDET": 0}
SIZE = {"ACK": 6, "DET": 32, "VERIFY": 12, "HB": 14, "LEASE": 16, "ASSIGN": 12, "COV": 48, "MAP": 2000, "IMG": 4000, "ACKDET": 10}
ACKED = {"DET", "VERIFY", "LEASE", "ASSIGN", "COV", "ACKDET"}
RELAYABLE = {"DET", "VERIFY", "LEASE", "ASSIGN", "COV", "HB", "ACKDET"}
RETRY_S = 3
FRESH_S = 5


def ge_params(m):
    """(p_gb, p_bg, loss_good) for margin m dB; m < 0 means forced outage."""
    return min(0.5, 0.03 * math.exp(-m / 5.0)), 0.35, 0.03 + 0.2 * math.exp(-m / 3.0)


def rate_bps(m):
    return 600.0 * min(max(m / 15.0, 0.08), 1.0)


class Msg:
    __slots__ = ("kind", "origin", "dst", "seq", "t_created", "payload", "size", "prio", "progress", "next_try",
                 "hop_from", "via")

    def __init__(self, kind, origin, dst, seq, t, payload):
        self.kind, self.origin, self.dst, self.seq, self.t_created = kind, origin, dst, seq, t
        self.payload = payload
        self.size = SIZE[kind]
        self.prio = PRIO[kind]
        self.progress = 0.0
        self.next_try = t
        self.hop_from = origin
        self.via = None

    def key(self):
        return (self.origin, self.kind, self.seq)


LINKS = [(0, 1), (0, 2), (1, 0), (2, 0), (1, 2), (2, 1)]


class Network:
    def __init__(self, world, layout_seed, rep, horizon, priority=True, forced=()):
        self.w = world
        self.priority = priority
        self.forced = list(forced)
        H = int(horizon) + 10
        r = W.rng(layout_seed, "comms", rep)
        self.u_tr = {l: r.random(H) for l in LINKS}
        self.u_loss = {l: r.random(H) for l in LINKS}
        self.u_jit = {l: r.random(H) for l in LINKS}
        self.bad = {l: False for l in LINKS}
        self.outbox = {0: [], 1: [], 2: []}
        self.inflight = []
        self.seen = {0: set(), 1: set(), 2: set()}
        self.heard = {n: {m: -1e9 for m in (0, 1, 2)} for n in (0, 1, 2)}
        self.seq = {}
        self.stats = {"sent_bytes": 0, "delivered": 0, "attempts": 0, "lost": 0, "link_up_ticks": {l: 0 for l in LINKS}}
        self.margins = {l: 0.0 for l in LINKS}

    # -------------------------------------------------------------- application API
    def send(self, t, origin, dst, kind, payload):
        k = (origin, kind)
        self.seq[k] = self.seq.get(k, 0) + 1
        m = Msg(kind, origin, dst, self.seq[k], t, payload)
        if kind in ("HB", "MAP", "COV"):  # newer supersedes older still queued
            self.outbox[origin] = [q for q in self.outbox[origin]
                                   if not (q.kind == kind and q.dst == dst and q.origin == origin and q.progress == 0)]
        self.outbox[origin].append(m)
        return m

    def pending(self, node, kind=None):
        return [m for m in self.outbox[node] if kind is None or m.kind == kind]

    def believed_up(self, n, m, t):
        return t - self.heard[n][m] <= FRESH_S

    # -------------------------------------------------------------- physics
    def _margin(self, l, pos):
        s, d = l
        if s == 0:
            m = self.w.margin_down[W.xy_cell(pos[d])]
        elif d == 0:
            m = self.w.margin_up[W.xy_cell(pos[s])]
        else:
            m = self.w.a2a_margin(pos[s], pos[d])
        return m

    def _forced(self, l, t):
        return any(l == (a, b) and t0 <= t < t1 for a, b, t0, t1 in self.forced)

    def _route(self, node, msg, t):
        """Next hop for msg held at node."""
        d = msg.dst
        if node == 0:
            return d
        partner = 3 - node
        if d == 0:
            if not self.believed_up(node, 0, t) and self.believed_up(node, partner, t) and msg.kind in RELAYABLE \
                    and msg.hop_from != partner:
                return partner
            return 0
        if d == partner:
            if self.believed_up(node, partner, t) or not self.believed_up(node, 0, t):
                return partner
            return 0
        return 0

    def tick(self, t, pos, active):
        """Advance one second. pos: {1: xy, 2: xy} TRUE positions; active: {1: bool, 2: bool} (landed = False)."""
        ti = int(t)
        for l in LINKS:
            s, d = l
            alive = all(active.get(n, True) for n in l if n != 0)
            m = self._margin(l, pos) if alive else -99.0
            self.margins[l] = m
            if m < 0 or self._forced(l, t):
                self.bad[l] = True
            else:
                p_gb, p_bg, _ = ge_params(m)
                u = self.u_tr[l][ti]
                self.bad[l] = (u >= p_bg) if self.bad[l] else (u < p_gb)
            if not self.bad[l]:
                self.stats["link_up_ticks"][l] += 1
        budget = {l: (0.0 if self.bad[l] else rate_bps(self.margins[l])) for l in LINKS}
        for node in (0, 1, 2):
            if node and not active.get(node, True):
                continue
            box = self.outbox[node]
            order = sorted(box, key=(lambda q: (q.prio, q.t_created)) if self.priority else (lambda q: q.t_created))
            keep = []
            for msg in order:
                if msg.next_try > t:
                    keep.append(msg)
                    continue
                hop = self._route(node, msg, t)
                l = (node, hop)
                if self.bad[l]:
                    msg.progress = 0.0  # fragment lost; restart
                    if msg.kind in ACKED or msg.kind == "ACK" or t - msg.t_created < 30:
                        keep.append(msg)
                    continue
                if budget[l] <= 0:
                    keep.append(msg)
                    if not self.priority:
                        budget[l] = 0.0
                    continue
                need = msg.size - msg.progress
                spend = min(budget[l], need)
                budget[l] -= spend
                msg.progress += spend
                self.stats["sent_bytes"] += spend
                if msg.progress < msg.size:
                    keep.append(msg)
                    continue
                msg.progress = 0.0
                self.stats["attempts"] += 1
                _, _, loss_good = ge_params(self.margins[l])
                ok = self.u_loss[l][ti] >= loss_good
                if ok:
                    u = self.u_jit[l][ti]
                    delay = 1 + (0 if u < 0.6 else 1 if u < 0.85 else 2 if u < 0.95 else 3)
                    self.inflight.append((t + delay, hop, node, msg))
                else:
                    self.stats["lost"] += 1
                if msg.kind in ACKED:
                    msg.next_try = t + RETRY_S
                    keep.append(msg)
            self.outbox[node] = keep
        # arrivals
        arrived, rest = [], []
        for item in self.inflight:
            (arrived if item[0] <= t else rest).append(item)
        self.inflight = rest
        out = []
        for ta, node, frm, msg in arrived:
            if node and not active.get(node, True):
                continue
            self.heard[node][frm] = t
            if msg.kind == "ACK":
                k = msg.payload["key"]
                self.outbox[node] = [q for q in self.outbox[node] if not (q.key() == k and q.kind in ACKED)]
                continue
            if msg.kind in ACKED:
                a = Msg("ACK", node, frm, 0, t, {"key": msg.key()})
                self.outbox[node].append(a)
            if msg.key() in self.seen[node]:
                continue
            self.seen[node].add(msg.key())
            if msg.dst == node:
                self.stats["delivered"] += 1
                out.append((node, frm, msg))
            else:  # custody: store and forward
                fwd = Msg(msg.kind, msg.origin, msg.dst, msg.seq, t, msg.payload)
                fwd.t_created = msg.t_created
                fwd.hop_from = frm
                self.outbox[node].append(fwd)
                if msg.kind == "DET" and node == 0:
                    out.append((node, frm, msg))  # base also reads detections it relays
        return out
