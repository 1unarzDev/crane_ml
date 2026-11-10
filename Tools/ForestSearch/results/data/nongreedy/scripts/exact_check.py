"""Exact-DP validation table: 12 random tiny instances (n=3, V=2, events 'both' hint rates, no abort)."""
import sys, time; sys.path.insert(0, "/home/lunarz/worktrees/crane-forest-sar/Tools")
import numpy as np
from ForestSearch.model import Instance
from ForestSearch.exact import ExactTeam
from ForestSearch.lookahead import exact_value_of_planner
rng = np.random.default_rng(2026)
print("| inst | k | optimal | greedy | lookahead | opt−greedy | opt−lookahead |\n|---|---|---|---|---|---|---|")
worst = 0
for i in range(12):
    p = rng.dirichlet(np.ones(3)); g = rng.uniform(0.1, 0.95, (3, 2)); q = rng.uniform(0.4, 0.95, (3, 2))
    E = ExactTeam(Instance(prior=p, g=g, q=q, h_good=0.3, h_bad=0.15, alpha=0.03))
    for k in (2, 3):
        o, gr = E.value(k), E.greedy_value(k)
        la = exact_value_of_planner(E, k, k=0, n_samples=512, pairs=6, z=1.0)
        worst = max(worst, o - la)
        print(f"| r{i} | {k} | {o:.4f} | {gr:.4f} | {la:.4f} | {o-gr:.4f} | {o-la:.4f} |", flush=True)
print("worst opt−lookahead", round(worst, 5))
