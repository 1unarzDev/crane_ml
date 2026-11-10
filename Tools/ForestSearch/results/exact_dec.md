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

| n | k | hints | abort | V_cent | V_dec | V_implicit | VoC | V_dec-V_imp | |F| | s |
|---|---|---|---|---|---|---|---|---|---|---|
| 2 | 2 | none | 0.0 | 0.695680 | 0.695680 | 0.695680 | 0.00e+00 | 0.00e+00 | 10 | 0.3 |
| 2 | 2 | none | 0.2 | 0.620831 | 0.618914 | 0.614195 | 1.92e-03 | 4.72e-03 | 16 | 0.1 |
| 2 | 2 | weak | 0.0 | 0.708662 | 0.708470 | 0.708470 | 1.92e-04 | 0.00e+00 | 26 | 0.2 |
| 2 | 2 | weak | 0.2 | 0.631944 | 0.629284 | 0.629284 | 2.66e-03 | 1.11e-16 | 36 | 0.4 |
| 2 | 2 | strong | 0.0 | 0.730028 | 0.729644 | 0.729644 | 3.84e-04 | 0.00e+00 | 26 | 0.2 |
| 2 | 2 | strong | 0.2 | 0.649983 | 0.645256 | 0.645256 | 4.73e-03 | 0.00e+00 | 36 | 0.4 |
| 2 | 2 | noisy | 0.0 | 0.699136 | 0.699136 | 0.692749 | 0.00e+00 | 6.39e-03 | 38 | 0.2 |
| 2 | 2 | noisy | 0.2 | 0.622658 | 0.618914 | 0.618131 | 3.74e-03 | 7.83e-04 | 52 | 0.7 |
| 3 | 2 | none | 0.0 | 0.590400 | 0.590400 | 0.590400 | 0.00e+00 | 0.00e+00 | 21 | 0.4 |
| 3 | 2 | none | 0.2 | 0.517427 | 0.515830 | 0.515830 | 1.60e-03 | -1.11e-16 | 36 | 0.8 |
| 3 | 2 | weak | 0.0 | 0.605658 | 0.605293 | 0.605293 | 3.65e-04 | 0.00e+00 | 63 | 1.5 |
| 3 | 2 | weak | 0.2 | 0.528773 | 0.526423 | 0.526423 | 2.35e-03 | 0.00e+00 | 90 | 2.4 |
| 3 | 2 | strong | 0.0 | 0.638416 | 0.628506 | 0.628506 | 9.91e-03 | 0.00e+00 | 63 | 1.0 |
| 3 | 2 | strong | 0.2 | 0.555520 | 0.542991 | 0.542456 | 1.25e-02 | 5.35e-04 | 90 | 2.2 |
| 3 | 2 | noisy | 0.0 | 0.593280 | 0.593280 | 0.590400 | 0.00e+00 | 2.88e-03 | 99 | 0.8 |
| 3 | 2 | noisy | 0.2 | 0.518902 | 0.515830 | 0.515830 | 3.07e-03 | 1.11e-16 | 138 | 3.0 |
| 2 | 3 | none | 0.0 | 0.764416 | 0.764416 | 0.764416 | 0.00e+00 | 0.00e+00 | 20 | 1.0 |
| 2 | 3 | none | 0.2 | 0.717644 | 0.714442 | 0.709210 | 3.20e-03 | 5.23e-03 | 52 | 2.7 |
| 2 | 3 | weak | 0.0 | 0.771691 | 0.771316 | 0.770411 | 3.74e-04 | 9.05e-04 | 272 | 9.6 |

(`|F|` = size of the reduced decentralised policy-value set; time in s.  The n=2, k=3 rows with hints+aborts and with strong/noisy hints were not reached in the time budget.)


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

Base grid: VoC is 0 for `none, ab 0` (as proved), and 0 for `noisy, ab 0`
(alpha = 0.2 makes hints nearly useless for reallocation).  Otherwise it is
1e-4 to 1e-2: 0.02-0.06% relative for weak hints, up to 1.6% relative for
strong hints at n=3 (VoC 9.9e-3; with aborts 1.25e-2).  Aborts alone give
about 2-3e-3.

Random instances (n=2, k=2, 600 draws: prior ~ Dir(1), g ~ U(.05,.95),
q ~ U(.3,1), h_good, h_bad ~ U(0,1), alpha ~ U(0,.5), abort 0 or U(0,.5)),
seed 2026, Pareto pruning:

```
random n=2 k=2: 600 instances
  all: VoC mean 3.86e-03 median 2.18e-03 max 3.35e-02; frac VoC>1e-9 0.85; imp-gap mean 1.73e-02 max 1.44e-01
  abort>0: VoC mean 4.75e-03 median 3.37e-03 max 2.79e-02; frac VoC>1e-9 0.95; imp-gap mean 1.97e-02 max 1.43e-01
  abort=0: VoC mean 3.13e-03 median 1.47e-03 max 3.35e-02; frac VoC>1e-9 0.78; imp-gap mean 1.54e-02 max 1.44e-01
  top VoC 3.3485e-02 (rel 5.22%) Vc 0.64107 Vd 0.60759 Vi 0.57918 prior [0.45, 0.55] g [[0.47, 0.88], [0.44, 0.35]] q [[0.6, 0.45], [0.64, 0.76]] hg 0.11 hb 1.00 alpha 0.01 abort 0.00
  top VoC 2.7915e-02 (rel 3.89%) Vc 0.71758 Vd 0.68966 Vi 0.67738 prior [0.355, 0.645] g [[0.43, 0.48], [0.87, 0.8]] q [[0.66, 0.95], [0.5, 0.36]] hg 0.98 hb 0.86 alpha 0.29 abort 0.02
  top VoC 2.5298e-02 (rel 4.53%) Vc 0.55856 Vd 0.53326 Vi 0.51642 prior [0.28, 0.72] g [[0.06, 0.74], [0.68, 0.12]] q [[0.39, 0.34], [0.6, 0.7]] hg 0.08 hb 0.95 alpha 0.03 abort 0.08
  Spearman(VoC, param): h_good=+0.16, h_bad=+0.28, alpha=-0.22, abort=+0.24, mean_q=-0.07, mean_g=+0.04, prior_max=-0.09
```

VoC > 0 on 85% of random instances.  It is largest (up to 3.3e-2 absolute,
5% relative, at k=2) when hints are informative about the *target* view
being bad (high h_bad, low alpha).  Those are cases where a private HINT
should redirect the partner.  VoC also grows with abort probability.  The
IMPLICIT gap V_dec - V_implicit is usually larger than VoC: mean 1.7e-2,
max 0.14.  On the base grid, though, IMPLICIT is exactly decentralised-optimal
in most hint rows; it loses in `none, ab .2`, the `noisy` rows, and at k=3.


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
