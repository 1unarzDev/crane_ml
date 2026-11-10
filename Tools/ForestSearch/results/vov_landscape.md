# VoC landscape at n = 2 (theory_extra.py, response to review A, M1)

Exact VoC = V_cent - V_dec (DP for V_cent, Pareto-pruned policy-tree
enumeration for V_dec; both exact, no LP tolerance) as a function of the
bad-view hint rate h_b and the false-hint rate alpha, travel-free,
synchronous, two agents.  Regenerate from Tools/:
`python -m ForestSearch.theory_extra landscape` (T=2, 21x21 grid,
h_b in [0,1], alpha in [0,0.5]), `... landscape3` (T=3, 6x6 grid),
`... points`, `... plot` (paper/figs/figA_voc.pdf).  Raw grids:
results/theory_extra_landscape.json, results/theory_extra_landscape3.json.

Two instances:

* **base**: the exact_dec n=2 instance, prior (.6,.4), g=[[.7,.4],[.5,.6]],
  q=.8, with **h_good fixed at the paper base value 0.30**.
* **adversarial**: the reviewer's hill-climb instance, prior (.37,.63),
  g=[[.93,.05],[.95,.11]], q=[[.5,.87],[.37,.49]], with its own **h_good = 0**.

Abort rho in {0, 0.1} (0.1 = the paper's base rho).

## Headline numbers

| setting | T | V_cent | V_dec | V_implicit | VoC | rel. |
|---|---|---|---|---|---|---|
| base2 weak (exact_dec) | 2 | 0.708662 | 0.708470 | 0.708470 | 1.9200e-04 | 0.03% |
| base2 weak (exact_dec) | 3 | 0.771691 | 0.771316 | 0.770411 | 3.7432e-04 | 0.05% |
| base2, paper rates lambda=1 | 2 | 0.675348 | 0.674537 | 0.674537 | 8.1119e-04 | 0.12% |
| base2, paper rates lambda=1, no abort | 2 | 0.711717 | 0.711602 | 0.711602 | 1.1520e-04 | 0.02% |
| base2, paper rates lambda=1, no abort | 3 | 0.774079 | 0.773530 | 0.771712 | 5.4893e-04 | 0.07% |
| base2, paper rates lambda=2 | 2 | 0.648727 | 0.644226 | 0.644226 | 4.5013e-03 | 0.69% |
| base2, paper rates lambda=2, no abort | 2 | 0.728649 | 0.728188 | 0.728188 | 4.6080e-04 | 0.06% |
| base2, paper rates lambda=2, no abort | 3 | 0.784174 | 0.781807 | 0.781807 | 2.3665e-03 | 0.30% |
| base2, paper rates lambda=3 | 2 | 0.611011 | 0.602577 | 0.601966 | 8.4339e-03 | 1.38% |
| base2, paper rates lambda=3, no abort | 2 | 0.746474 | 0.745437 | 0.745437 | 1.0368e-03 | 0.14% |
| base2, paper rates lambda=3, no abort | 3 | 0.797053 | 0.795092 | 0.794422 | 1.9611e-03 | 0.25% |
| reviewer max (h_g=0,h_b=1,a=0) | 2 | 0.712677 | 0.630600 | 0.623721 | 8.2077e-02 | 11.52% |
| reviewer max (h_g=0,h_b=1,a=0) | 3 | 0.864892 | 0.831434 | 0.765118 | 3.3459e-02 | 3.87% |
| reviewer, h_b=.9, a=.02 | 2 | 0.683309 | 0.622617 | 0.622617 | 6.0692e-02 | 8.88% |
| reviewer, h_b=.9, a=.02 | 3 | 0.829166 | 0.811375 | 0.764056 | 1.7791e-02 | 2.15% |
| reviewer, paper rates lambda=1 | 2 | 0.627326 | 0.616457 | 0.572244 | 1.0869e-02 | 1.73% |

The paper's rates scale together with lambda: (rho, h_g, h_b, alpha) =
lambda * (.10, .30, .15, .03).  "no abort" rows set rho = 0.  T=3 rows with
aborts are not reported (3-outcome Pareto set too large for the budget).

* **Reviewer maximum reproduced:** VoC = 0.082077 (11.52% relative) at
  h_g=0, h_b=1, alpha=0, T=2 (also by brute force over all policy pairs,
  test_theory_extra.py).  With h_b=.9, alpha=.02: 0.0607 (reviewer: 0.061).
* **At the paper's base rates** VoC on the base instance is 1.2e-4 to 1.0e-3
  without aborts (lambda=1..3, T=2) and 8.1e-4 / 4.5e-3 / 8.4e-3 with the
  lambda-scaled aborts (0.12% / 0.69% / 1.38% relative).  At T=3 without
  aborts: 5.5e-4 / 2.4e-3 / 2.0e-3.
* On the adversarial instance, moving only the event rates to the paper's
  lambda=1 values cuts VoC from 0.082 to 0.011 (T=2).
* The T=2 maximum on the base instance is 0.017 (rho=0) / 0.019 (rho=0.1),
  both at h_b=1, alpha=0.

## When is VoC large? (precise statement for the slices computed)

On both slices VoC is non-monotone and piecewise smooth (policy switches),
but large values occur only in one corner:

* adversarial instance, T=2, rho=0: VoC >= half its maximum (>= 0.041)
  only where h_b >= 0.80 and alpha <= 0.125 (4% of the grid);
  VoC >= 0.01 on 15% of the grid: the corner h_b >= 0.65 with small alpha
  (up to 0.021 at h_b=0.65-0.7), plus the edge alpha = 0.5 (VoC 0.013 for
  every h_b <= 0.6; with h_g = h_b = 0 a HINT there is a pure false hint and
  certifies "target NOT here", the mirror-image near-certain event).
* base instance, T=2, rho=0: VoC >= 0.01 only at h_b >= 0.95 (2.3% of the
  grid); everywhere with h_b <= 0.9 VoC <= 0.0064.
* T=3 (6x6 grid, rho=0): maxima 0.0062 (base) and 0.033 (adversarial,
  again at h_b=1, alpha=0).  On the adversarial instance VoC *falls* from
  0.082 at T=2 to 0.033 at T=3: with more looks the decentralised team can
  recover the information a hint would have carried.

Mechanism.  With h_g = 0, h_b = 1, alpha = 0, a HINT at (c, v) occurs iff
the target is in c and view (c, v) is bad: it certifies "target here, view
bad", so the team's best response is to send the *partner* to the other view
of c at once.  A NULL is then also informative (it rules out "target here,
view bad").  The private event changes the partner's optimal action with
near certainty, which is exactly the failure of measurability in Remark
(VoC = 0 iff ...).  As h_b falls or alpha rises, the hint's likelihood ratio
h_b / alpha shrinks, the centralised optimum stops reacting to it, and VoC
collapses.  Exact zero on the uninformative line h_g = h_b = alpha
(P(HINT | non-detect) independent of the truth) at rho = 0 (tested).

## T = 2 grids (selected rows/cols; full grid in the json)

### base, abort = 0.0 (max 0.0170)

| h_b \ alpha | 0.000 | 0.025 | 0.050 | 0.100 | 0.150 | 0.250 | 0.500 |
|---|---|---|---|---|---|---|---|
| 0.00 | 3.57e-03 | 2.33e-03 | 0.00e+00 | 0.00e+00 | 0.00e+00 | 0.00e+00 | 3.65e-03 |
| 0.10 | 3.22e-03 | 1.60e-05 | 3.20e-05 | 0.00e+00 | 0.00e+00 | 0.00e+00 | 1.70e-03 |
| 0.15 | 4.51e-03 | 9.60e-05 | 1.92e-04 | 5.11e-04 | 0.00e+00 | 0.00e+00 | 2.98e-03 |
| 0.20 | 5.36e-03 | 3.37e-04 | 3.52e-04 | 5.43e-04 | 8.14e-04 | 0.00e+00 | 4.44e-03 |
| 0.30 | 5.60e-03 | 1.09e-03 | 1.83e-03 | 2.11e-03 | 1.69e-03 | 0.00e+00 | 3.51e-03 |
| 0.40 | 6.35e-03 | 1.19e-03 | 1.93e-03 | 1.36e-03 | 2.04e-03 | 3.46e-04 | 3.14e-04 |
| 0.50 | 5.56e-03 | 2.25e-04 | 1.20e-03 | 1.60e-03 | 2.40e-03 | 4.00e-03 | 0.00e+00 |
| 0.60 | 5.59e-03 | 1.29e-04 | 5.10e-04 | 8.62e-04 | 2.75e-03 | 4.41e-03 | 1.11e-16 |
| 0.70 | 4.48e-03 | 1.11e-16 | 0.00e+00 | 0.00e+00 | 1.11e-16 | 3.27e-03 | 1.11e-16 |
| 0.80 | 1.98e-03 | 0.00e+00 | 0.00e+00 | 0.00e+00 | 0.00e+00 | 1.46e-03 | 0.00e+00 |
| 0.90 | 1.28e-03 | 1.33e-03 | 1.38e-03 | 1.47e-03 | 1.57e-03 | 3.56e-03 | 0.00e+00 |
| 0.95 | 4.16e-03 | 4.10e-03 | 4.03e-03 | 3.90e-03 | 3.78e-03 | 3.56e-03 | 0.00e+00 |
| 1.00 | 1.70e-02 | 1.66e-02 | 1.62e-02 | 1.53e-02 | 1.45e-02 | 9.75e-03 | 0.00e+00 |

### base, abort = 0.1 (max 0.0187)

| h_b \ alpha | 0.000 | 0.025 | 0.050 | 0.100 | 0.150 | 0.250 | 0.500 |
|---|---|---|---|---|---|---|---|
| 0.00 | 4.36e-03 | 3.62e-03 | 2.95e-03 | 1.11e-03 | 5.75e-04 | 1.20e-03 | 2.15e-03 |
| 0.10 | 1.30e-03 | 3.24e-04 | 7.52e-05 | 4.12e-04 | 1.51e-03 | 1.18e-03 | 1.45e-04 |
| 0.15 | 1.79e-03 | 8.75e-04 | 5.56e-04 | 6.95e-04 | 1.24e-03 | 1.21e-03 | 4.71e-04 |
| 0.20 | 2.28e-03 | 1.43e-03 | 1.17e-03 | 1.08e-03 | 8.69e-04 | 4.20e-04 | 7.98e-04 |
| 0.30 | 3.41e-03 | 1.56e-03 | 1.26e-03 | 1.05e-03 | 2.44e-03 | 3.06e-04 | 5.08e-03 |
| 0.40 | 7.12e-03 | 1.89e-03 | 1.67e-03 | 1.68e-03 | 2.48e-03 | 1.29e-03 | 1.76e-03 |
| 0.50 | 1.09e-02 | 2.97e-03 | 8.88e-04 | 4.62e-04 | 4.36e-04 | 2.28e-03 | 1.76e-04 |
| 0.60 | 1.06e-02 | 2.40e-03 | 4.74e-04 | 8.40e-05 | 7.93e-05 | 1.88e-03 | 1.20e-03 |
| 0.70 | 8.16e-03 | 2.19e-03 | 1.10e-03 | 8.40e-05 | 7.93e-05 | 9.03e-04 | 1.21e-03 |
| 0.80 | 4.51e-03 | 3.05e-03 | 2.03e-03 | 1.16e-03 | 9.76e-04 | 8.61e-04 | 1.68e-03 |
| 0.90 | 7.33e-03 | 6.36e-03 | 5.39e-03 | 4.92e-03 | 5.11e-03 | 5.24e-03 | 1.68e-03 |
| 0.95 | 9.94e-03 | 8.87e-03 | 7.80e-03 | 7.01e-03 | 7.01e-03 | 7.02e-03 | 1.68e-03 |
| 1.00 | 1.87e-02 | 1.70e-02 | 1.54e-02 | 1.26e-02 | 1.09e-02 | 8.54e-03 | 1.68e-03 |

### reviewer, abort = 0.0 (max 0.0821)

| h_b \ alpha | 0.000 | 0.025 | 0.050 | 0.100 | 0.150 | 0.250 | 0.500 |
|---|---|---|---|---|---|---|---|
| 0.00 | 0.00e+00 | 2.10e-03 | 3.71e-03 | 5.12e-03 | 6.10e-03 | 6.78e-03 | 1.33e-02 |
| 0.10 | 1.56e-04 | 2.10e-03 | 3.71e-03 | 5.12e-03 | 6.10e-03 | 6.78e-03 | 1.33e-02 |
| 0.15 | 2.21e-04 | 2.10e-03 | 3.71e-03 | 5.12e-03 | 6.10e-03 | 6.78e-03 | 1.33e-02 |
| 0.20 | 2.77e-04 | 2.11e-03 | 3.71e-03 | 5.12e-03 | 6.10e-03 | 6.78e-03 | 1.33e-02 |
| 0.30 | 3.64e-04 | 2.20e-03 | 3.71e-03 | 5.12e-03 | 6.10e-03 | 6.78e-03 | 1.33e-02 |
| 0.40 | 4.16e-04 | 2.35e-03 | 3.77e-03 | 5.12e-03 | 6.10e-03 | 6.78e-03 | 1.33e-02 |
| 0.50 | 4.33e-04 | 2.58e-03 | 3.92e-03 | 5.12e-03 | 6.10e-03 | 6.78e-03 | 1.33e-02 |
| 0.60 | 1.12e-03 | 2.86e-03 | 4.10e-03 | 5.12e-03 | 6.10e-03 | 6.78e-03 | 1.33e-02 |
| 0.70 | 2.13e-02 | 1.39e-02 | 5.38e-03 | 5.12e-03 | 6.10e-03 | 6.78e-03 | 1.33e-02 |
| 0.80 | 4.13e-02 | 3.45e-02 | 2.71e-02 | 1.13e-02 | 6.10e-03 | 6.78e-03 | 1.33e-02 |
| 0.90 | 6.45e-02 | 5.91e-02 | 5.10e-02 | 3.39e-02 | 1.67e-02 | 6.78e-03 | 1.33e-02 |
| 0.95 | 7.67e-02 | 7.15e-02 | 6.32e-02 | 4.54e-02 | 2.78e-02 | 6.78e-03 | 1.33e-02 |
| 1.00 | 8.21e-02 | 7.87e-02 | 7.30e-02 | 5.76e-02 | 3.95e-02 | 6.78e-03 | 1.33e-02 |

### reviewer, abort = 0.1 (max 0.0647)

| h_b \ alpha | 0.000 | 0.025 | 0.050 | 0.100 | 0.150 | 0.250 | 0.500 |
|---|---|---|---|---|---|---|---|
| 0.00 | 0.00e+00 | 8.62e-04 | 1.72e-03 | 3.45e-03 | 5.17e-03 | 7.03e-03 | 1.44e-02 |
| 0.10 | 9.67e-05 | 8.67e-04 | 1.73e-03 | 3.47e-03 | 5.20e-03 | 7.03e-03 | 1.44e-02 |
| 0.15 | 1.45e-04 | 8.70e-04 | 1.74e-03 | 3.48e-03 | 5.22e-03 | 7.03e-03 | 1.44e-02 |
| 0.20 | 1.93e-04 | 8.73e-04 | 1.75e-03 | 3.49e-03 | 5.24e-03 | 7.03e-03 | 1.44e-02 |
| 0.30 | 2.90e-04 | 8.78e-04 | 1.76e-03 | 3.51e-03 | 5.27e-03 | 7.03e-03 | 1.44e-02 |
| 0.40 | 3.87e-04 | 8.83e-04 | 1.77e-03 | 3.53e-03 | 5.30e-03 | 7.03e-03 | 1.44e-02 |
| 0.50 | 4.83e-04 | 8.88e-04 | 1.78e-03 | 3.55e-03 | 5.33e-03 | 7.03e-03 | 1.44e-02 |
| 0.60 | 5.80e-04 | 8.93e-04 | 1.79e-03 | 3.57e-03 | 5.36e-03 | 7.03e-03 | 1.44e-02 |
| 0.70 | 6.11e-03 | 8.98e-04 | 1.80e-03 | 3.59e-03 | 5.39e-03 | 7.03e-03 | 1.44e-02 |
| 0.80 | 2.37e-02 | 1.88e-02 | 1.29e-02 | 3.61e-03 | 5.42e-03 | 7.03e-03 | 1.44e-02 |
| 0.90 | 4.42e-02 | 3.95e-02 | 3.29e-02 | 2.04e-02 | 7.97e-03 | 7.03e-03 | 1.44e-02 |
| 0.95 | 5.45e-02 | 4.98e-02 | 4.32e-02 | 3.01e-02 | 1.73e-02 | 7.03e-03 | 1.44e-02 |
| 1.00 | 6.47e-02 | 6.02e-02 | 5.36e-02 | 4.04e-02 | 2.72e-02 | 7.03e-03 | 1.44e-02 |

### T=3, base, abort = 0.0 (max 0.0062)

| h_b \ alpha | 0.00 | 0.10 | 0.20 | 0.30 | 0.40 | 0.50 |
|---|---|---|---|---|---|---|
| 0.0 | 1.18e-03 | 4.84e-05 | 5.82e-04 | 2.70e-03 | 4.10e-03 | 4.94e-03 |
| 0.2 | 4.48e-03 | 1.05e-03 | 2.11e-05 | 0.00e+00 | 9.25e-04 | 2.18e-03 |
| 0.4 | 4.82e-03 | 9.74e-04 | 1.18e-03 | 6.77e-05 | 0.00e+00 | 4.06e-04 |
| 0.6 | 3.70e-03 | 1.55e-03 | 2.01e-03 | 2.49e-03 | 4.67e-04 | 4.85e-04 |
| 0.8 | 2.33e-03 | 4.02e-04 | 5.55e-04 | 1.74e-03 | 3.97e-04 | 0.00e+00 |
| 1.0 | 3.30e-03 | 2.79e-03 | 4.24e-03 | 6.17e-03 | 4.05e-03 | 2.03e-03 |

### T=3, reviewer, abort = 0.0 (max 0.0335)

| h_b \ alpha | 0.00 | 0.10 | 0.20 | 0.30 | 0.40 | 0.50 |
|---|---|---|---|---|---|---|
| 0.0 | 1.11e-16 | 5.42e-03 | 8.95e-03 | 9.15e-03 | 9.93e-03 | 2.10e-02 |
| 0.2 | 3.47e-04 | 5.59e-03 | 9.25e-03 | 9.58e-03 | 1.03e-02 | 2.15e-02 |
| 0.4 | 4.54e-04 | 5.71e-03 | 9.46e-03 | 9.83e-03 | 1.06e-02 | 2.17e-02 |
| 0.6 | 1.54e-02 | 6.07e-03 | 9.60e-03 | 1.00e-02 | 1.07e-02 | 2.19e-02 |
| 0.8 | 1.99e-02 | 1.26e-02 | 9.70e-03 | 1.01e-02 | 1.09e-02 | 2.20e-02 |
| 1.0 | 3.35e-02 | 2.68e-02 | 1.83e-02 | 1.02e-02 | 1.10e-02 | 2.21e-02 |

