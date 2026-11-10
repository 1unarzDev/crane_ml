# Exact value of communication on tiny instances (exact_dec.py)

Setting: 2 searchers, synchronous, travel-free, V = 2, k steps, value =
P(confirmed detection within k).  Base instances (`exact_dec.base_instance`):
n=2 prior (0.6, 0.4), g = [[.7,.4],[.5,.6]]; n=3 prior (.5,.3,.2), extra
cell g = (.6,.6); q = 0.8 everywhere.  Hint settings (h_good, h_bad, alpha):
none (0,0,0), weak (.3,.15,.05), strong (.6,.3,.05), noisy (.3,.15,.2).
Abort 0 or 0.2.

## Method

* Truth = (target cell, visibility of the target cell's views), |T| = n*2^V.
  Given the truth, all looks are independent.  A decentralised policy depends
  only on the agent's own outcomes, so
  `P(fail) = sum_tau p(tau) f0(tau) f1(tau)` with `f_j` = agent j's
  per-truth failure vector.  `V_dec = 1 - min over f0, f1 in F_k`, where
  F_k (all achievable deterministic policy-tree failure vectors) is built by
  `F_k = U_a { sum_o P(o|tau,a) g_o : g_o in F_{k-1} }` and reduced at every
  level.  Two exact reductions: componentwise Pareto (always) and
  convex/LP dominance (scipy HiGHS, tol 1e-12; exact because every subtree
  objective is linear with non-negative weights).  Both agree with each other
  and with exhaustive enumeration of all policy pairs (n=2, k=2; hints, aborts).
* V_cent: exact DP over the joint unnormalised belief, aborts shared;
  matches `ExactTeam.value` to 1e-12 (asserted in every abort-free row).
* V_implicit: the two fixed policy trees induced by sim.py IMPLICIT
  (dist 0, starts (0,0), horizon k): own real outcomes + partner nominal looks
  of earlier steps as NULL, agent 1 also counts agent 0's current nominal look.
  `test_implicit_tree_matches_sim` replays 300 simulated episodes per setting
  and checks that every sim action equals the tree action.  Monte Carlo
  (`sim.run_episode`, 20000 eps, seeds 0..19999):

| instance (n=2,k=2) | exact V_implicit | MC | z |
|---|---|---|---|
| none, ab 0   | 0.695680 | 0.6899 +- 0.0033 | -1.75 |
| none, ab .2  | 0.614195 | 0.6087 +- 0.0035 | -1.59 |
| weak, ab 0   | 0.708470 | 0.7033 +- 0.0032 | -1.62 |
| weak, ab .2  | 0.629284 | 0.6269 +- 0.0034 | -0.71 |

  All within 2 sigma.  The z values are correlated because every row reuses
  the same seeds (common random numbers).  The full MC grid was stopped for
  time; the trajectory-replay test is the stronger check.

## Results (exact)

RESULTS_TABLE

## No hints and no aborts: VoC = 0 exactly (proof)

With outcomes in {NULL, DETECT} and no aborts, the only history that continues
past step t is the all-NULL one, for each agent and for the team.  So any
decentralised policy is an open-loop sequence per agent, and any centralised
policy is an open-loop sequence of joint actions.  A joint sequence
((a_1,b_1),...,(a_k,b_k)) is the same thing as the pair of sequences
(a_1..a_k), (b_1..b_k), so the two policy classes induce identical outcome
laws and V_dec = V_cent.  (This holds for any n, V, k, also with travel, as
long as decisions stay synchronous.)  It matches the `none, ab 0` rows.

## Counterexample: VoC > 0 at n=2, k=2

Take the instance `weak, ab 0`: prior (.6,.4), g=[[.7,.4],[.5,.6]], q=.8,
h_good=.3, h_bad=.15, alpha=.05.  V_cent = 0.70866176 and
V_dec = 0.70846976, so VoC = 1.92e-4.

Optimal decentralised pair (IMPLICIT finds the same pair):
* A: look (0,0); NULL -> (1,0), HINT -> (0,0)
* B: look (1,1); NULL -> (0,1), HINT -> (1,1)

Centralised optimum: the same first step.  Step 2 by joint outcome (A,B):
NN -> {(0,1),(1,0)}, NH -> {(1,0),(1,1)}, HN -> {(0,0),(0,1)}: all identical
to the decentralised pair.  HH -> {(0,0),(1,0)}, but decentralised it is
{(0,0),(1,1)}.  The gap comes entirely from the both-hint branch.  B cannot
tell whether A also got a hint.  The team prefers to move B's second look
from the repeated (1,1) view to the fresh (1,0) view.  B's rule
HINT -> (1,1) is optimal in the NH branch, which B cannot tell apart from HH,
so no decentralised rule matches the team in both.

With aborts and no hints (`none, ab .2`, VoC = 1.92e-3): the only branch
that differs is A-aborts/B-NULL.  The team then puts both looks on (0,0).
Decentralised, B (seeing NULL) must play (0,1), which is right when A
succeeded.

## Where VoC is largest

RANDOM_SUMMARY

## Limits

* Exact only for k <= 3 at n = 2 and k = 2 at n = 3.  At k = 4, the LP-pruned
  F_3 has ~270 vectors (n=2, weak hints).  The Minkowski step (~7e4
  candidates per action, each needing an LP) did not finish in the time
  budget, so k = 4 and n = 3, k = 3 are not reported.
* Deterministic policies only.  This is no restriction: the objective is
  bilinear, so a pure pair attains the minimum.
* Synchronous, travel-free, 2 agents, aborts shared in the centralised
  benchmark.  With travel, the decentralised class and the VoC can differ.
* The LP pruning tolerance is 1e-12 per removal.  Pareto-only (tolerance-free
  up to float rounding) and LP runs agreed to 1e-15 where both ran.
