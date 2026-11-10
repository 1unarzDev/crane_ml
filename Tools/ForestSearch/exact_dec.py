"""Exact value of communication on tiny travel-free instances.

Setting (as exact.py): 2 searchers, synchronous, no travel, both look once per
step for k steps; value = P(confirmed detection within k steps).  Aborts are
allowed here (private to the looker in the decentralised case, shared in the
centralised case).

Key identity.  Let a *truth* be tau = (target cell i, visibility vector of
cell i over its V views); visibility of non-target cells never affects an
outcome, so |T| = n * 2^V.  Given tau, every look's outcome is independent of
every other look.  A decentralised policy of agent j maps its own
(non-detect) outcome history to a look, so given tau the two agents' outcome
processes are independent and

    P(no detection | pi_0, pi_1) = sum_tau p(tau) f_0(tau) f_1(tau),
    f_j(tau) = P(agent j never detects in k looks | tau, pi_j).

So V_dec = 1 - min_{f0, f1 in F_k} <p, f0 * f1>, where F_k is the set of
failure vectors achievable by deterministic single-agent policy trees.
F_k is built by the recursion

    F_k = U_a  { sum_o P(o | tau, a) * g_o  :  g_o in F_{k-1} },  F_0 = {1},

(o ranges over non-detect outcomes NULL/HINT/ABORT) and is reduced to its
componentwise-Pareto-minimal subset at every level.  That reduction is exact:
the objective is monotone non-decreasing in each f_j coordinate (p, f >= 0),
and Pareto(A (+) B) = Pareto(Pareto(A) (+) Pareto(B)).  Every policy pair is
thus covered without enumerating the A^(|history tree|) policies.

V_cent is an exact DP over the joint unnormalised belief (memoised on the
multiset of (look, outcome) pairs).  V_implicit is the exact value of the two
fixed policy trees induced by sim.py IMPLICIT in the synchronous travel-free
case.
"""
from __future__ import annotations

import itertools
from dataclasses import dataclass, replace
from functools import lru_cache

import numpy as np

from .model import ABORT, HINT, NULL, Instance, detect_prob

OUT_NAMES = {NULL: "N", HINT: "H", ABORT: "A"}


@dataclass
class Policy:
    """Deterministic policy tree: look `action` (cell*V+view), then the child
    for the observed non-detect outcome (keyed by outcome code)."""
    action: int
    children: dict

    def show(self, V: int, outs, indent: str = "") -> list[str]:
        c, v = divmod(self.action, V)
        lines = [f"{indent}look (cell {c}, view {v})"]
        for o in outs:
            ch = self.children.get(o)
            if ch is not None:
                sub = ch.show(V, outs, indent + "    ")
                sub[0] = f"{indent}  {OUT_NAMES[o]} -> " + sub[0].lstrip()
                lines += sub
        return lines


class TinyModel:
    def __init__(self, inst: Instance):
        self.inst = inst
        n, V = inst.n, inst.V
        self.n, self.V, self.A = n, V, n * V
        self.truths = [(i, bits) for i in range(n)
                       for bits in itertools.product((0, 1), repeat=V)]
        T = len(self.truths)
        self.p = np.array([inst.prior[i] * np.prod([inst.g[i, v] if b else 1 - inst.g[i, v]
                                                    for v, b in enumerate(bits)])
                           for i, bits in self.truths])
        self.outs = [NULL]
        if inst.h_good > 0 or inst.h_bad > 0 or inst.alpha > 0:
            self.outs.append(HINT)
        if inst.abort > 0:
            self.outs.append(ABORT)
        ab = inst.abort
        # P[a, oi, tau] for non-detect outcomes
        P = np.zeros((self.A, len(self.outs), T))
        for a in range(self.A):
            c, v = divmod(a, V)
            for t, (i, bits) in enumerate(self.truths):
                if c == i and bits[v]:
                    q = inst.q[c, v]
                    pn, ph = (1 - q) * (1 - inst.h_good), (1 - q) * inst.h_good
                elif c == i:
                    pn, ph = 1 - inst.h_bad, inst.h_bad
                else:
                    pn, ph = 1 - inst.alpha, inst.alpha
                d = {NULL: (1 - ab) * pn, HINT: (1 - ab) * ph, ABORT: ab}
                for oi, o in enumerate(self.outs):
                    P[a, oi, t] = d[o]
        self.P = P

    # ------------------------------------------------------------ centralised
    def v_cent(self, k: int) -> float:
        A, m = self.A, len(self.outs)
        acts = list(itertools.combinations_with_replacement(range(A), 2))
        P = self.P

        @lru_cache(maxsize=None)
        def fail(kk, key):
            cnt = np.array(key).reshape(A, m)
            w = self.p * np.prod(P ** cnt[:, :, None], axis=(0, 1))
            if kk == 0:
                return float(w.sum())
            best = np.inf
            for a0, a1 in acts:
                tot = 0.0
                for o0 in range(m):
                    for o1 in range(m):
                        c2 = cnt.copy()
                        c2[a0, o0] += 1
                        c2[a1, o1] += 1
                        tot += fail(kk - 1, tuple(c2.ravel()))
                best = min(best, tot)
            return best
        return 1.0 - fail(k, tuple([0] * (A * m)))

    # --------------------------------------------------------- decentralised
    def failure_sets(self, k: int, prune=None):
        """levels[j] = (vecs (N, T), policies list) for j looks, reduced by
        ``prune`` (default: convex/LP pruning if scipy is available, else
        componentwise Pareto).  Both reductions preserve V_dec exactly."""
        prune = prune or default_prune()
        T = len(self.truths)
        levels = [(np.ones((1, T)), [None])]
        for _ in range(k):
            Fv, Fp = levels[-1]
            cand_v, cand_p = [], []
            for a in range(self.A):
                # incremental Minkowski sum over outcomes with Pareto pruning
                S = self.P[a, 0][None, :] * Fv
                idx = [(j,) for j in range(len(Fv))]
                S, idx = prune(S, idx)
                for oi in range(1, len(self.outs)):
                    B = self.P[a, oi][None, :] * Fv
                    if not B.any():
                        idx = [t + (0,) for t in idx]
                        continue
                    Sum = (S[:, None, :] + B[None, :, :]).reshape(-1, T)
                    nidx = [t + (j,) for t in idx for j in range(len(Fv))]
                    S, idx = prune(Sum, nidx)
                cand_v.append(S)
                cand_p += [(a, t) for t in idx]
            V_, keep = prune(np.vstack(cand_v), list(range(len(cand_p))))
            pols = []
            for ci in keep:
                a, t = cand_p[ci]
                pols.append(Policy(a, {o: Fp[j] for o, j in zip(self.outs, t)}))
            levels.append((V_, pols))
        return levels

    def v_dec(self, k: int, return_policies: bool = False, prune=None):
        Fv, Fp = self.failure_sets(k, prune)[-1]
        W = Fv * self.p[None, :]
        best, arg = np.inf, (0, 0)
        for s in range(0, len(Fv), 512):
            M = W[s:s + 512] @ Fv.T
            i = int(np.argmin(M))
            r, c = divmod(i, M.shape[1])
            if M[r, c] < best:
                best, arg = float(M[r, c]), (s + r, c)
        val = 1.0 - best
        if return_policies:
            return val, Fp[arg[0]], Fp[arg[1]], len(Fv)
        return val

    def policy_fail(self, pol: Policy, k: int) -> np.ndarray:
        if k == 0 or pol is None:
            return np.ones(len(self.truths))
        out = np.zeros(len(self.truths))
        for oi, o in enumerate(self.outs):
            out += self.P[pol.action, oi] * self.policy_fail(pol.children[o], k - 1)
        return out

    def pair_value(self, pol0: Policy, pol1: Policy, k: int) -> float:
        return 1.0 - float((self.p * self.policy_fail(pol0, k) * self.policy_fail(pol1, k)).sum())

    # -------------------------------------------------------------- IMPLICIT
    def implicit_policies(self, k: int):
        """Policy trees of sim.py IMPLICIT (dist=0, starts=(0,0), horizon=k).

        At step t agent j's belief counts are: its own real outcomes (ABORT
        adds nothing), plus the partner's nominal looks of steps < t as NULLs;
        agent 1 additionally counts agent 0's nominal look of step t as a
        virtual NULL (index-order decision).  Action = argmax detect_prob."""
        from .sim import nominal_plan
        inst = replace(self.inst, dist=np.zeros((self.n, self.n), int),
                       starts=(0, 0), horizon=k)
        nom = nominal_plan(inst)
        pols = []
        for j in range(2):
            partner = [(tk.cell, tk.view) for tk in nom[1 - j]]

            def build(t, m0, m1, j=j, partner=partner):
                if t == k:
                    return None
                pm0 = m0.copy()
                for (c, v) in partner[:t + (1 if j == 1 else 0)]:
                    pm0[c, v] += 1
                pd = detect_prob(inst, pm0, m1)
                a = int(np.argmax(pd))
                c, v = divmod(a, self.V)
                ch = {}
                for o in self.outs:
                    n0, n1 = m0.copy(), m1.copy()
                    if o == NULL:
                        n0[c, v] += 1
                    elif o == HINT:
                        n1[c, v] += 1
                    ch[o] = build(t + 1, n0, n1)
                return Policy(a, ch)
            z = np.zeros((self.n, self.V), int)
            pols.append(build(0, z, z.copy()))
        return pols, nom

    def v_implicit(self, k: int) -> float:
        (p0, p1), _ = self.implicit_policies(k)
        return self.pair_value(p0, p1, k)


def pareto(X: np.ndarray, tags: list, tol: float = 1e-13):
    """Componentwise-minimal rows of X (duplicates collapsed)."""
    if len(X) <= 1:
        return X, list(tags)
    order = np.lexsort(X.T[::-1])
    order = order[np.argsort(X[order].sum(1), kind="stable")]
    kept = []
    K = np.empty((0, X.shape[1]))
    for i in order:
        x = X[i]
        if len(K) and np.any(np.all(K <= x + tol, axis=1)):
            continue
        kept.append(i)
        K = np.vstack([K, x])
    return X[kept], [tags[i] for i in kept]


def hull_prune(X: np.ndarray, tags: list, tol: float = 1e-12, seed: int = 0):
    """Drop rows that are weakly dominated by a convex combination of the
    remaining rows (Farkas: such a row is never the unique minimiser of
    w.f for any w >= 0).  Exact for V_dec because, for a fixed partner, the
    objective <p * f_partner, f> is linear in f with non-negative weights,
    and at every tree node the sub-policy objective is again linear with
    non-negative weights.  Rows are removed one at a time against the rows
    still present, so ties/duplicates keep one representative.  Each removal
    can raise the minimum of a normalised linear objective by at most ``tol``.
    """
    from scipy.optimize import linprog
    X, tags = pareto(X, tags)
    N, T = X.shape
    if N <= 1:
        return X, tags
    rng = np.random.default_rng(seed)
    W = np.vstack([np.eye(T), rng.dirichlet(np.full(T, 0.3), size=4000),
                   rng.dirichlet(np.ones(T), size=4000)])
    sure = np.zeros(N, bool)
    vals = W @ X.T
    best = vals.min(1, keepdims=True)
    # a row that is the unique strict minimiser at some weight is a vertex
    winners = vals <= best + 1e-15
    uniq = winners.sum(1) == 1
    sure[np.argmax(winners[uniq], axis=1)] = True
    alive = np.ones(N, bool)
    for i in np.where(~sure)[0]:
        others = np.where(alive)[0]
        others = others[others != i]
        G = X[others] - X[i]
        # variables (w_1..w_T, d): max d  s.t.  d - G w <= 0, sum w = 1, w >= 0
        A_ub = np.hstack([-G, np.ones((len(G), 1))])
        res = linprog(np.r_[np.zeros(T), -1.0], A_ub=A_ub, b_ub=np.zeros(len(G)),
                      A_eq=np.r_[np.ones(T), 0.0][None, :], b_eq=[1.0],
                      bounds=[(0, None)] * T + [(None, None)], method="highs")
        if res.status == 0 and -res.fun <= tol:
            alive[i] = False
    keep = np.where(alive)[0]
    return X[keep], [tags[j] for j in keep]


def default_prune():
    try:
        import scipy.optimize  # noqa: F401
        return hull_prune
    except ImportError:
        return pareto


def brute_force_dec(model: TinyModel, k: int) -> float:
    """Enumerate every deterministic policy tree pair (tiny cases only)."""
    m = len(model.outs)
    nodes = sum(m ** t for t in range(k))

    def tree(assign):
        layers, idx = [], 0
        for t in range(k):
            layers.append(assign[idx: idx + m ** t])
            idx += m ** t

        def build(t, pos):
            if t == k:
                return None
            return Policy(layers[t][pos],
                          {o: build(t + 1, pos * m + oi) for oi, o in enumerate(model.outs)})
        return build(0, 0)

    fails = [model.policy_fail(tree(asg), k)
             for asg in itertools.product(range(model.A), repeat=nodes)]
    F = np.array(fails)
    M = (F * model.p) @ F.T
    return 1.0 - float(M.min())


def mc_implicit(inst: Instance, k: int, episodes: int = 20000, seed0: int = 0):
    from .links import LinkConfig
    from .sim import nominal_plan, run_episode
    inst = replace(inst, dist=np.zeros((inst.n, inst.n), int), starts=(0, 0), horizon=k)
    nom = nominal_plan(inst)
    hits = sum(run_episode(inst, ("IMPLICIT", "IMPLICIT"), LinkConfig(), seed0 + s,
                           nominal=nom).detected for s in range(episodes))
    p = hits / episodes
    return p, float(np.sqrt(p * (1 - p) / episodes))


# ------------------------------------------------------------------ driver
HINTS = {"none": (0.0, 0.0, 0.0), "weak": (0.3, 0.15, 0.05),
         "strong": (0.6, 0.3, 0.05), "noisy": (0.3, 0.15, 0.2)}


def base_instance(n: int, hints: str, abort: float) -> Instance:
    pri = [0.6, 0.4] if n == 2 else [0.5, 0.3, 0.2]
    g = [[0.7, 0.4], [0.5, 0.6], [0.6, 0.6]][:n]
    hg, hb, a = HINTS[hints]
    return Instance(prior=pri, g=g, q=np.full((n, 2), 0.8), h_good=hg, h_bad=hb,
                    alpha=a, abort=abort, name=f"n{n}-{hints}-ab{abort}")


def random_instance(rng, n: int) -> Instance:
    return Instance(prior=rng.dirichlet(np.ones(n)), g=rng.uniform(0.05, 0.95, (n, 2)),
                    q=rng.uniform(0.3, 1.0, (n, 2)), h_good=rng.uniform(0, 1),
                    h_bad=rng.uniform(0, 1), alpha=rng.uniform(0, 0.5),
                    abort=rng.choice([0.0, rng.uniform(0, 0.5)]))


def main(argv=None):
    import argparse
    import time
    ap = argparse.ArgumentParser()
    ap.add_argument("--grid", default="2:2,2:3,3:2")  # n:k pairs
    ap.add_argument("--random", type=int, default=0)
    ap.add_argument("--random-nk", default="2:2")
    ap.add_argument("--mc", type=int, default=0)
    ap.add_argument("--prune", default="auto", choices=["auto", "pareto", "hull"])
    args = ap.parse_args(argv)
    prune = {"auto": None, "pareto": pareto, "hull": hull_prune}[args.prune]
    if args.grid:
        print("| n | k | hints | abort | V_cent | V_dec | V_implicit | VoC | V_dec-V_imp | |F| | s |")
        print("|---|---|---|---|---|---|---|---|---|---|---|")
        for nk in args.grid.split(","):
            n, k = map(int, nk.split(":"))
            for hints in HINTS:
                for ab in (0.0, 0.2):
                    inst = base_instance(n, hints, ab)
                    M = TinyModel(inst)
                    t0 = time.time()
                    vc = M.v_cent(k)
                    vd, _, _, nF = M.v_dec(k, True, prune)
                    vi = M.v_implicit(k)
                    xc = ExactTeamCheck(inst, k) if ab == 0 else float("nan")
                    assert np.isnan(xc) or abs(xc - vc) < 1e-10
                    print(f"| {n} | {k} | {hints} | {ab} | {vc:.6f} | {vd:.6f} | {vi:.6f} "
                          f"| {vc - vd:.2e} | {vd - vi:.2e} | {nF} | {time.time() - t0:.1f} |",
                          flush=True)
                    if args.mc:
                        p, se = mc_implicit(inst, k, args.mc)
                        print(f"|   MC IMPLICIT ({args.mc} eps) | {p:.4f} +- {se:.4f} "
                              f"(z = {(p - vi) / max(se, 1e-12):+.2f}) |", flush=True)
    if args.random:
        rng = np.random.default_rng(2026)
        n, k = map(int, args.random_nk.split(":"))
        rows = []
        for r in range(args.random):
            inst = random_instance(rng, n)
            M = TinyModel(inst)
            vc, vd, vi = M.v_cent(k), M.v_dec(k, prune=prune), M.v_implicit(k)
            rows.append((vc - vd, vd - vi, vc, vd, vi, inst))
        voc = np.array([x[0] for x in rows])
        gap = np.array([x[1] for x in rows])
        ab = np.array([x[5].abort > 0 for x in rows])
        nohint = np.array([x[5].h_good + x[5].h_bad + x[5].alpha == 0 for x in rows])
        print(f"random n={n} k={k}: {len(rows)} instances")
        for lab, msk in (("all", np.ones(len(rows), bool)), ("abort>0", ab), ("abort=0", ~ab)):
            if msk.any():
                print(f"  {lab}: VoC mean {voc[msk].mean():.2e} median {np.median(voc[msk]):.2e} "
                      f"max {voc[msk].max():.2e}; frac VoC>1e-9 {(voc[msk] > 1e-9).mean():.2f}; "
                      f"imp-gap mean {gap[msk].mean():.2e} max {gap[msk].max():.2e}")
        del nohint
        for x in sorted(rows, key=lambda x: -x[0])[:3]:
            i = x[5]
            print(f"  top VoC {x[0]:.4e} (rel {x[0] / x[2]:.2%}) Vc {x[2]:.5f} Vd {x[3]:.5f} Vi {x[4]:.5f} "
                  f"prior {np.round(i.prior, 3).tolist()} g {np.round(i.g, 2).tolist()} "
                  f"q {np.round(i.q, 2).tolist()} hg {i.h_good:.2f} hb {i.h_bad:.2f} "
                  f"alpha {i.alpha:.2f} abort {i.abort:.2f}")
        # correlation of VoC with parameters
        feats = {"h_good": [x[5].h_good for x in rows], "h_bad": [x[5].h_bad for x in rows],
                 "alpha": [x[5].alpha for x in rows], "abort": [x[5].abort for x in rows],
                 "mean_q": [x[5].q.mean() for x in rows], "mean_g": [x[5].g.mean() for x in rows],
                 "prior_max": [x[5].prior.max() for x in rows]}
        print("  Spearman(VoC, param): " + ", ".join(
            f"{k_}={_spearman(np.array(v), voc):+.2f}" for k_, v in feats.items()))


def _spearman(a, b):
    ra, rb = np.argsort(np.argsort(a)), np.argsort(np.argsort(b))
    return float(np.corrcoef(ra, rb)[0, 1])


def ExactTeamCheck(inst, k):
    from .exact import ExactTeam
    return ExactTeam(inst).value(k)


if __name__ == "__main__":
    main()
