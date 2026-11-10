"""Regenerate the exact-DP numbers quoted in the paper into results/.

  results/exact.md        exact optimal vs centralised greedy vs independent-view greedy
                          (n=3, V=2, travel-free, k=3,4; instances rng(3))
  results/alternation.md  independent-view greedy + view alternation vs correlated greedy
                          (n=3, k=3; instances rng(11))
Run from Tools/:  python -m ForestSearch.exact_provenance
"""
import numpy as np

from .exact import ExactTeam
from .model import BeliefModel, Instance, detect_prob

IND = BeliefModel(correlated=False)


def _inst(rng):
    return Instance(prior=rng.dirichlet(np.ones(3)), g=rng.uniform(.2, .9, (3, 2)),
                    q=np.full((3, 2), .8), h_good=.3, h_bad=.2, alpha=.05)


def alt_value(E, k):
    cache = {}

    def act(key):
        m0, m1 = E._arrays(key)
        m0 = m0.copy()
        a = []
        for _ in range(E.K):
            pd = detect_prob(E.inst, m0, m1, IND)
            cell = int(np.argmax(pd.max(1)))
            v = int(np.argmin((m0 + m1)[cell]))
            a.append(E.types.index((cell, v)))
            m0[cell, v] += 1
        return tuple(sorted(a))

    def ev(kk, key):
        if kk == 0:
            return 0.0
        if (kk, key) not in cache:
            cache[(kk, key)] = E._q(kk, key, act(key), ev)
        return cache[(kk, key)]
    return ev(k, tuple([0] * (2 * E.n * E.V)))


def main():
    L = ["# Exact DP: optimal vs centralised greedy (travel-free, 2 agents, n=3, V=2)", "",
         "| inst | k | optimal | greedy | gap | greedy, independent-view belief | loss from independence |",
         "|---|---|---|---|---|---|---|"]
    rng = np.random.default_rng(3)
    for t in range(4):
        E = ExactTeam(_inst(rng))
        for k in (3, 4):
            o, g, gi = E.value(k), E.greedy_value(k), E.greedy_value(k, IND)
            L.append(f"| {t} | {k} | {o:.4f} | {g:.4f} | {o-g:.4f} | {gi:.4f} | {g-gi:.4f} |")
            print(L[-1], flush=True)
    open("ForestSearch/results/exact.md", "w").write("\n".join(L) + "\n")
    A = ["# View-alternation heuristic (exact, n=3, k=3)", "",
         "| inst | optimal | correlated greedy | independent greedy | independent + alternation | gap recovered |",
         "|---|---|---|---|---|---|"]
    rng = np.random.default_rng(11)
    rec = []
    for t in range(6):
        E = ExactTeam(_inst(rng))
        o, g, gi, ga = E.value(3), E.greedy_value(3), E.greedy_value(3, IND), alt_value(E, 3)
        r = (ga - gi) / (g - gi) if g > gi else float("nan")
        rec.append(r)
        A.append(f"| {t} | {o:.4f} | {g:.4f} | {gi:.4f} | {ga:.4f} | {r:.2f} |")
        print(A[-1], flush=True)
    A.append(f"\nMedian gap recovered: {np.nanmedian(rec):.2f}; range {np.nanmin(rec):.2f} to {np.nanmax(rec):.2f}.")
    open("ForestSearch/results/alternation.md", "w").write("\n".join(A) + "\n")


if __name__ == "__main__":
    main()
