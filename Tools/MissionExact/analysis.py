"""Helpers: classify the action taken right after a first WEAK candidate."""
from __future__ import annotations

from .model import WEAK, NULL, CONF, fmt_action


def first_weak_keys(team, j):
    """Info-set keys of drone j whose own history ends with its first WEAK
    and which have not heard of any WEAK from the partner."""
    out = []
    for k, a in team.best_policy.items():
        if k[0] != j:
            continue
        _, t, node, hist, rcv = k
        if not hist or hist[-1][3] != WEAK:
            continue
        if sum(1 for e in hist if e[3] == WEAK) != 1:
            continue
        if _heard_weak(team, rcv):
            continue
        out.append((k, a))
    out.sort(key=lambda ka: (ka[0][1], repr(ka[0])))
    return out


def heard_weak_keys(team, j):
    """Keys of drone j that have heard of a partner WEAK and have seen no
    WEAK themselves."""
    out = []
    for k, a in team.best_policy.items():
        if k[0] != j:
            continue
        _, t, node, hist, rcv = k
        if any(e[3] == WEAK for e in hist):
            continue
        if _heard_weak(team, rcv):
            out.append((k, a))
    out.sort(key=lambda ka: (ka[0][1], repr(ka[0])))
    return out


def _heard_weak(team, rcv):
    if rcv is None:
        return False
    ph = rcv[1] if team.mode == "delayed" else rcv
    return any(e[3] == WEAK for e in ph)


def classify(inst, key, a):
    """'self-verify (same/other view, fast/slow)', 'fly to link', 'fly to n?'."""
    _, t, node, hist, rcv = key
    last = hist[-1]
    if a[0] == "L":
        same = "same view" if a[1] == last[1][1] else "other view"
        return f"self-verify ({same}, {'slow' if a[2] else 'fast'})"
    if a[0] == "M":
        if inst.link[a[1]] and inst.prior[a[1]] == 0:
            return "fly to link node"
        return f"fly to n{a[1]}" + (" (link)" if inst.link[a[1]] else "")
    return "wait"


def oracle_after_first_weak(oracle, max_depth=8):
    """Walk the ORACLE policy along all-NULL branches until the first
    branch with exactly one WEAK; return list of (t, joint action desc,
    weak node/view)."""
    res = []

    def rec(t, drones, counts, depth):
        if depth > max_depth:
            return
        oracle.V(t, drones, counts)
        _, ja = oracle._V[(t, oracle.canon(drones), counts)]
        if ja is None:
            return
        ja = oracle._match(drones, ja)
        for p, kind, pay in oracle.transitions(t, ja, counts):
            if kind != "cont":
                continue
            t2, d2, c2 = pay
            nweak = sum(c2[1::2])
            if nweak == 0:
                rec(t2, d2, c2, depth + 1)
            elif nweak == 1 and sum(counts[1::2]) == 0:
                oracle.V(t2, d2, c2)
                _, nja = oracle._V[(t2, oracle.canon(d2), c2)]
                if nja is None:
                    res.append((t2, p, "no useful action", d2))
                    continue
                nja = oracle._match(d2, nja)
                acts = []
                for k, ((node, e, pend), (n2, e2, p2)) in enumerate(zip(d2, nja)):
                    if pend == ():
                        acts.append(f"D{k}@n{node}: {fmt_action(p2, node)}")
                li = [i for i in range(len(c2) // 2) if c2[2 * i + 1] == 1][0]
                c, v, s = oracle.inst.looks[li]
                res.append((t2, p, f"WEAK at n{c} view{v} ({'slow' if s else 'fast'}) -> " + "; ".join(acts), d2))
    rec(*oracle.root(), 0)
    return res
