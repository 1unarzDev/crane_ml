# Decision-relevant divergence and coordination regret as predictors of VoC

Code: `predictor.py` (tasks `exact`, `sim`, `report`), tests: `test_predictor.py`.

## 1. Definitions and relations (LaTeX-ready)

```latex
\paragraph{Coordination regret.}
Let $h_t$ be the joint history (all agents' looks and outcomes before step $t$) and
$h^j_t$ agent $j$'s local history. Let $\mu^\ast$ be an optimal centralized policy with
value-to-go $V^\ast_t(h_t)$ and action value $Q^\ast_t(h_t,a)$ (team value of the joint
action $a$, optimal play afterwards). A decentralized team $\pi=(\pi^j)$ plays
$\pi(h_t)=(\pi^j(h^j_t))_j$.
\begin{definition}[DRD and CR]\label{def:cr}
The \emph{coordination regret} of $\pi$ at $h_t$ is
$\mathrm{CR}_t(h_t)=V^\ast_t(h_t)-Q^\ast_t(h_t,\pi(h_t))\ge 0$, the team value lost
because agents act on local information rather than on $h_t$.
The \emph{decision-relevant divergence} is
$\mathrm{DRD}_t=P_\pi\big(\pi(h_t)\notin\arg\max_a Q^\ast_t(h_t,a)\big)$.
For the \emph{prediction team} $\pi^\phi$, in which agent $j$ predicts the joint history
by a map $\phi^j$ (partners on the default plan, every partner outcome NULL) and plays
$[\mu^\ast(\phi^j(h^j_t))]_j$, the event inside $\mathrm{DRD}_t$ says that conditioning the
agent's decision rule on its partners' private information would change its action.
\end{definition}

\begin{proposition}[CR bounds VoC]\label{prop:cr}
For every decentralized team $\pi$ (finite horizon $T$),
\[
V_{\mathrm{cent}}-V(\pi)=\mathbb{E}_\pi\Big[\sum_{t<T}\mathrm{CR}_t(h_t)\Big],
\qquad\text{hence}\qquad
\mathrm{VoC}\le \mathbb{E}_\pi\Big[\sum_t\mathrm{CR}_t\Big]
\le \bar\Delta\sum_t \mathrm{DRD}_t ,
\]
with equality in the first inequality at an optimal decentralized team, and
$\bar\Delta=\max_{t,h,a}\big(V^\ast_t(h)-Q^\ast_t(h,a)\big)$. In particular, if
$\mathrm{DRD}_t=0$ for all $t$ along $\pi$'s play, then $\mathrm{VoC}=0$ and $\pi$ is optimal.
\end{proposition}
\begin{proof}[Sketch]
Performance-difference telescoping: $V^\ast_t(h_t)-V^\pi_t(h_t)
=\mathrm{CR}_t(h_t)+\mathbb{E}[V^\ast_{t+1}(h_{t+1})-V^\pi_{t+1}(h_{t+1})\mid h_t,\pi(h_t)]$,
since $Q^\ast_t(h,a)=\mathbb{E}[V^\ast_{t+1}\mid h,a]$ and $V^\pi_t(h)=\mathbb{E}[V^\pi_{t+1}\mid h,\pi(h)]$.
Unroll to $T$. $\mathrm{VoC}=V_{\mathrm{cent}}-\max_\pi V(\pi)\le V_{\mathrm{cent}}-V(\pi)$.
$\mathrm{CR}_t\le\bar\Delta\,\mathbf 1\{\pi(h_t)\notin\arg\max Q^\ast_t\}$ gives the last bound.
\end{proof}
Obs.~\ref{obs:voc} is the case $\mathrm{DRD}\equiv 0$: with no private events
$\phi^j(h^j_t)=h_t$ on every continuing path, so $\pi^\phi$ replays $\mu^\ast$.
Conversely, VoC $=0$ iff some $\pi$ has $\mathrm{DRD}\equiv0$ (take an optimal decentralized
team; Remark~\ref{rem:measurable}).

\begin{corollary}[CR lives after private events]\label{cor:support}
For the prediction team $\pi^\phi$, $\mathrm{CR}_t=0$ on every history in which no agent has
had a private event (HINT, ABORT, DEAD) before $t$. Hence
$\mathrm{VoC}\le\mathbb{E}[\sum_t \mathrm{CR}_t\,\mathbf 1\{N_t\ge1\}]
\le \bar\Delta\,\mathbb{E}[\#\{t: N_t\ge 1\}]$, where $N_t$ counts private events before $t$.
\end{corollary}
\begin{corollary}[Beacon: CR concentrates at the failure]\label{cor:beacon}
Under the assumptions of Prop.~\ref{prop:beacon}, let $D$ be the first death time.
Without a frame, $\mathrm{CR}_t(\pi^\phi)=0$ for $t\le D$, so all of $V_{\mathrm{cent}}-V(\pi^\phi)$
is regret incurred after $D$ by agents that do not know of the death. One frame sent at
$D$ makes $\phi^j(h^j_t)=h_t$ again, so $\mathrm{CR}_t=0$ for all $t$ and the beacon team is
optimal. The rule ``send iff the CR the message removes exceeds its cost'' fires exactly
once, at $D$.
\end{corollary}
\begin{proof}[Sketch]
Before the first private event the prediction is exact; afterwards, with
outcomes in $\{\text{NULL},\text{DETECT},\text{DEAD}\}$, the joint history is a
function of the death times (proof of Prop.~\ref{prop:beacon}), which the frame discloses.
\end{proof}
```

Comments on scope.  CR is defined against the *optimal* centralised value; in the
simulator we measure it against centralised *greedy* (ORACLE) with a short rollout, so the
empirical quantity is an estimate of the same object for the greedy team, not a bound.
The proposition is a performance-difference lemma (Kakade & Langford 2002) applied to a
team; the novelty is only the reading: the VoC bound is the expected regret on the
decision-relevant event, which is what an event trigger should measure.

## 2. Exact check on tiny instances (`predictor.py exact`)

Setting as in `exact_dec.py`: 2 agents (the `n` below is the number of CELLS), synchronous,
travel-free, V = 2, horizon k.  `V_cent` and `Q*` come from the exact centralised DP
(`theory_extra.v_cent_fast`, which also covers sensor death).  For each decentralised team
pi the code walks the full joint-history tree.  It accumulates `CR_t` (unnormalised
failure-mass form) and `DRD_t`, and checks the performance-difference identity.  Teams:
**dec\*** = the exact optimal decentralised pair (`TinyModel.v_dec`); **IMPLICIT** = the
policy trees the simulator's IMPLICIT induces (`implicit_policies`); **pi^phi** = replay of
mu\* on the predicted joint history (every partner outcome NULL).
Raw: `~/.cache/research/predictor/predictor_exact.json` (73 rows; the main run hit the 10-min quick-lane cap after the n2-weak-ab0 k=3 row, so n2 k=3 weak-ab.2 / strong / noisy are missing).  Tests: `test_predictor.py`, 11 pass.

Verified on all 22 no-death instances x 3 teams, plus 8 death rows:
* Identity `V_cent - V(pi) = E sum_t CR_t`: max abs error 4e-16.
* `VoC <= E sum CR`: holds in every row.  Equality (to 1e-16) for dec\*.
* `DRD == 0  =>  VoC = 0`: on `none, ab 0` (n = 2 and 3 cells, k = 2 and 3), pi^phi has DRD = 0 and CR = 0.
  On `noisy, ab 0`, pi^phi also has DRD = 0, while IMPLICIT has DRD = 0.09 and CR = 6.4e-3.
  So the zero-VoC certificate needs the *right* local team.  Greedy IMPLICIT is not
  person-by-person optimal there.
* Corollary (CR lives after private events): pi^phi has exactly 0 CR on histories with
  no prior HINT/ABORT/DEAD in all 30 rows.  (Teams other than pi^phi can have regret at
  t = 0: IMPLICIT at weak k = 3 and on the adversarial instance, because greedy is not
  centrally optimal.)

| instance (cells-hints-abort) | k | VoC | E sum CR: dec\* | IMPLICIT | pi^phi | sum DRD (IMPLICIT) | sum DRD (pi^phi) |
|---|---|---|---|---|---|---|---|
| n2-none-ab0 | 2 | 0 | 0 | 0 | 0 | 0 | 0 |
| n2-none-ab.2 | 2 | 1.92e-3 | 1.92e-3 | 6.64e-3 | 6.64e-3 | 0.236 | 0.236 |
| n2-weak-ab0 | 2 | 1.92e-4 | 1.92e-4 | 1.92e-4 | 1.34e-2 | 0.005 | 0.110 |
| n2-weak-ab.2 | 2 | 2.66e-3 | 2.66e-3 | 2.66e-3 | 1.80e-2 | 0.144 | 0.306 |
| n2-strong-ab0 | 2 | 3.84e-4 | 3.84e-4 | 3.84e-4 | 3.51e-2 | 0.009 | 0.196 |
| n2-strong-ab.2 | 2 | 4.73e-3 | 4.73e-3 | 4.73e-3 | 3.63e-2 | 0.155 | 0.361 |
| n2-noisy-ab0 | 2 | 0 | 0 | 6.39e-3 | 0 | 0.094 | 0 |
| n2-noisy-ab.2 | 2 | 3.74e-3 | 3.74e-3 | 4.53e-3 | 6.25e-3 | 0.189 | 0.214 |
| n3-none-ab.2 | 2 | 1.60e-3 | 1.60e-3 | 1.60e-3 | 1.16e-2 | 0.137 | 0.292 |
| n3-weak-ab0 | 2 | 3.65e-4 | 3.65e-4 | 3.65e-4 | 8.54e-3 | 0.046 | 0.107 |
| n3-weak-ab.2 | 2 | 2.35e-3 | 2.35e-3 | 2.35e-3 | 1.86e-2 | 0.167 | 0.361 |
| n3-strong-ab0 | 2 | 9.91e-3 | 9.91e-3 | 9.91e-3 | 2.27e-2 | 0.471 | 0.176 |
| n3-strong-ab.2 | 2 | 1.25e-2 | 1.25e-2 | 1.31e-2 | 3.14e-2 | 0.442 | 0.405 |
| n3-noisy-ab0 | 2 | 0 | 0 | 2.88e-3 | 0 | 0.115 | 0 |
| n3-noisy-ab.2 | 2 | 3.07e-3 | 3.07e-3 | 3.07e-3 | 1.12e-2 | 0.210 | 0.292 |
| n2-none-ab.2 | 3 | 3.20e-3 | 3.20e-3 | 8.43e-3 | 1.91e-2 | 0.238 | 0.326 |
| n2-weak-ab0 | 3 | 3.74e-4 | 3.74e-4 | 1.28e-3 | 7.12e-3 | 1.038 | 0.183 |
| paper rates lam=1 | 2 | 8.11e-4 | 8.11e-4 | 8.11e-4 | 1.71e-2 | 0.013 | 0.215 |
| adversarial (h_b=1) | 2 | 8.21e-2 | 8.21e-2 | 8.90e-2 | 8.26e-2 | 1.057 | 0.383 |
| adversarial | 3 | 3.35e-2 | 3.35e-2 | - | 6.20e-2 | - | 0.548 |

Predictiveness across the 21 tiny instances that have an IMPLICIT tree:

| predictor | Spearman with exact VoC | median VoC / predictor |
|---|---|---|
| E sum CR (IMPLICIT) | 0.78 | 1.00 |
| E sum CR (pi^phi) | 0.78 | 0.14 |
| sum DRD (IMPLICIT) | 0.80 | 0.02 |
| sum DRD (pi^phi) | 0.86 | 0.01 |

How to read this.  `E sum CR(pi)` is exactly `V_cent - V(pi)`: the most that *any*
protocol layered on pi can gain.  It equals VoC only when pi is decentralised-optimal,
which IMPLICIT is in 11 of the 21 rows.  The pi^phi bound is loose by about 7x (median),
because replaying the centralised optimum on a wrong prediction is worse than the best
silent pair.  DRD ranks instances about as well as CR, but its scale is meaningless
(0.01-0.02 VoC per unit DRD): most decision changes are cheap.

**Sensor death (Prop. 3 setting), death prob. 0.2 per look:**

| instance | k | V_cent | VoC without frame | E sum CR, pi^phi without frame | of which at dead-but-unknown nodes | of which at the first step after death | E sum CR with one frame |
|---|---|---|---|---|---|---|---|
| n2 none | 2 | 0.57584 | 0.01294 | 0.04612 | 0.04612 (100%) | 0.04612 | 0 (V = V_cent) |
| n2 none | 3 | 0.62940 | 0.00663 | 0.06449 | 0.06449 (100%) | 0.01746 (27%) | 0 (V = V_cent) |
| n3 none | 2 | 0.47867 | 0.01352 | 0.03297 | 0.03297 (100%) | 0.03297 | 0 (V = V_cent) |
| n2 weak hints | 2 | 0.58491 | 0.01528 | 0.05544 | 0.04752 (86%) | 0.04752 | 0.00792 (hint-caused residual) |

So "CR concentrates at the failure" holds in this precise sense: there is no regret
before the death, all regret sits at nodes where the death is unknown, and one frame at the
death removes all of it.  Without hints the frame team equals V_cent.  The regret itself is
*spread over the steps after the death* (only 27% in the first step at k = 3), not
spiked at it.  What concentrates is the **information event**: the CR-removing message has
exactly one useful send time.

## 3. Empirical predictiveness in the simulator (`predictor.py sim`, `report`)

Run in the 10-minute quick lane: dev seeds 1000-1011 (20 episodes per instance, used ONLY to
fit the calibration slope b), held-out seeds 4000-4015 (16 fresh instances, 50 episodes per
instance per condition, paired by episode seed across IMPLICIT / ORACLE / HEALTH / DEV).
25 conditions on 5x5 grids (H16 without travel, H30 with travel).  Raw json:
`~/.cache/research/predictor/predictor_sim_{dev,heldout}.json`; exact json:
`~/.cache/research/predictor/predictor_exact.json` (kept out of the repo: they are 2-8 MB).

Instrumentation.
* `PEpisode` (subclass of `beacon_robustness.BEpisode`, any fault kind).  At every IMPLICIT
  decision of a live agent, the greedy argmax is recomputed twice.
  * With the partner's TRUE state (records, plus the in-flight task unless it is a wasted
    dead-sensor look).  This is ORACLE's information.
  * With what a HEALTH frame would tell.  Default-plan tasks after the death stamp are
    dropped.  The frame is known only from `t_dead + delay`, never if the radio is dead or
    the frame is dropped, and it is weighted by `1 - mean loss`.
* DRD = the argmax changed.  CR = max(0, V_B(a_full) - V_B(a_imp)), where V_B is a 6-step
  greedy-rollout P(detect) under the true joint belief.  CR_health is the unclipped
  analogue for the beacon's information, so it can be negative.
* The probe consumes no randomness: trajectories equal plain IMPLICIT (tested), and
  DRD = CR = 0 at lambda = 0 (tested; Prop. 1).
* N = 3, 4 use a `nagent.NEpisode` subclass (healthy conditions only).
* The condition-level CIs come from a two-level bootstrap (resample conditions, then
  instances; B = 1000).  For G_oracle, the radio/link-only variants (`total`, `beacon_lost`,
  `perm_d1/d3/burst`) are excluded, because they duplicate `perm` for ORACLE and IMPLICIT.

### Condition-level table (held-out seeds 4000-4015)

| condition | N | lam | fault | link | IMP | G_oracle | G_health | DRD/ep | CR/ep | CR_health/ep | events/ep | lossrate | DEV frames/ep |
|---|---|---|---|---|---|---|---|---|---|---|---|---|---|
| healthy_l0 | 2 | 0 | healthy | d0_loss0 | 0.719 | +0.0000 | +0.0000 | 0.000 | 0.0000 | +0.0000 | 0.00 | 0.000 | 0.00 |
| healthy_l1 | 2 | 1 | healthy | d0_loss0 | 0.720 | +0.0138 | +0.0000 | 4.713 | 0.0005 | +0.0000 | 2.05 | 0.000 | 1.96 |
| healthy_l2 | 2 | 2 | healthy | d0_loss0 | 0.718 | +0.0012 | +0.0000 | 5.514 | 0.0017 | +0.0000 | 4.07 | 0.000 | 3.97 |
| healthy_l3 | 2 | 3 | healthy | d0_loss0 | 0.705 | +0.0225 | +0.0000 | 6.242 | 0.0037 | +0.0000 | 6.50 | 0.000 | 6.13 |
| hints_l3 | 2 | 3 | healthy | d0_loss0 | 0.794 | +0.0025 | +0.0000 | 3.857 | 0.0026 | +0.0000 | 1.61 | 0.000 | 1.56 |
| aborts_r3 | 2 | 1 | healthy | d0_loss0 | 0.625 | -0.0063 | +0.0000 | 6.484 | 0.0000 | +0.0000 | 5.64 | 0.000 | 5.44 |
| travel_l1 | 2 | 1 | healthy | d0_loss0 | 0.689 | -0.0038 | +0.0000 | 0.724 | 0.0121 | +0.0000 | 1.85 | 0.000 | 1.76 |
| travel_l3 | 2 | 3 | healthy | d0_loss0 | 0.709 | -0.0012 | +0.0000 | 1.690 | 0.0383 | +0.0000 | 6.98 | 0.000 | 6.60 |
| perm | 2 | 1 | perm | d0_loss0 | 0.525 | +0.0788 | +0.0650 | 7.296 | 0.0246 | +0.0239 | 8.27 | 0.380 | 7.80 |
| perm_early | 2 | 1 | perm_early | d0_loss0 | 0.470 | +0.1100 | +0.0900 | 8.450 | 0.0390 | +0.0382 | 10.09 | 0.431 | 9.34 |
| perm_late | 2 | 1 | perm_late | d0_loss0 | 0.662 | +0.0375 | +0.0150 | 4.865 | 0.0010 | +0.0005 | 3.81 | 0.196 | 3.60 |
| perm_travel | 2 | 1 | perm | d0_loss0 | 0.458 | +0.0850 | +0.0713 | 4.246 | 0.1375 | +0.1037 | 14.70 | 0.384 | 12.55 |
| perm_noev | 2 | 0 | perm | d0_loss0 | 0.541 | +0.0488 | +0.0488 | 6.339 | 0.0250 | +0.0250 | 6.57 | 0.380 | 6.34 |
| trans_m | 2 | 1 | trans_m | d0_loss0 | 0.680 | +0.0100 | -0.0363 | 6.623 | 0.0017 | -0.0377 | 4.40 | 0.118 | 4.29 |
| trans_l | 2 | 1 | trans_l | d0_loss0 | 0.629 | +0.0325 | -0.0050 | 7.243 | 0.0109 | -0.0075 | 6.38 | 0.235 | 6.11 |
| intermit | 2 | 1 | intermit | d0_loss0 | 0.662 | +0.0375 | -0.0375 | 6.705 | 0.0032 | -0.0638 | 4.70 | 0.134 | 4.53 |
| total | 2 | 1 | total | d0_loss0 | 0.525 | +0.0788 | +0.0000 | 7.296 | 0.0246 | +0.0000 | 8.27 | 0.380 | 1.51 |
| beacon_lost | 2 | 1 | beacon_lost | d0_loss0 | 0.525 | +0.0788 | +0.0000 | 7.296 | 0.0246 | +0.0000 | 8.27 | 0.380 | 7.91 |
| perm_d1 | 2 | 1 | perm | d1_loss0 | 0.525 | +0.0788 | +0.0638 | 7.296 | 0.0246 | +0.0239 | 8.27 | 0.380 | 15.11 |
| perm_d3 | 2 | 1 | perm | d3_loss0 | 0.525 | +0.0788 | +0.0738 | 7.296 | 0.0246 | +0.0239 | 8.27 | 0.380 | 22.27 |
| perm_burst | 2 | 1 | perm | d1_burst30 | 0.525 | +0.0788 | +0.0425 | 7.296 | 0.0246 | +0.0168 | 8.27 | 0.380 | 15.48 |
| n3_l1 | 3 | 1 | healthy | d0_loss0 | 0.792 | -0.0063 | +nan | 8.535 | 0.0016 | +0.0000 | 2.53 | 0.000 | nan |
| n3_l3 | 3 | 3 | healthy | d0_loss0 | 0.805 | +0.0175 | +nan | 10.864 | 0.0122 | +0.0000 | 7.69 | 0.000 | nan |
| n3_aborts | 3 | 1 | healthy | d0_loss0 | 0.721 | -0.0163 | +nan | 11.944 | 0.0011 | +0.0000 | 7.12 | 0.000 | nan |
| n4_l3 | 4 | 3 | healthy | d0_loss0 | 0.841 | -0.0000 | +nan | 14.356 | 0.0233 | +0.0000 | 8.85 | 0.000 | nan |

### Condition-level predictiveness (held-out; 95% two-level bootstrap CIs, B=1000)


### Target G_oracle (20 conditions)

| predictor | Spearman [CI] | Pearson [CI] | dev-fit slope b (G = b x) | held-out calib. slope [CI] | held-out MAE | MAE of mean-only |
|---|---|---|---|---|---|---|
| CR | +0.52 [-0.05,+0.82] | +0.59 [+0.03,+0.84] | 0.361 | 2.28 [1.12,6.90] | 0.0232 | 0.0257 |
| DRD | +0.03 [-0.42,+0.58] | +0.04 [-0.36,+0.48] | 0.000515 | 5.66 [1.56,12.01] | 0.0257 | 0.0257 |
| events | +0.40 [-0.10,+0.79] | +0.62 [+0.15,+0.83] | 0.00127 | 3.65 [1.45,5.66] | 0.0239 | 0.0257 |
| lam | -0.19 [-0.61,+0.24] | -0.27 [-0.55,+0.06] | 0.00355 | 2.30 [0.60,5.34] | 0.0264 | 0.0257 |
| lossrate | +0.85 [+0.50,+0.94] | +0.92 [+0.65,+0.97] | 0.0596 | 3.36 [2.11,4.39] | 0.0198 | 0.0257 |

### Target G_health (21 conditions)

| predictor | Spearman [CI] | Pearson [CI] | dev-fit slope b (G = b x) | held-out calib. slope [CI] | held-out MAE | MAE of mean-only |
|---|---|---|---|---|---|---|
| CR_health | +0.99 [+0.67,+1.00] | +0.82 [+0.61,+0.97] | 0.531 | 1.98 [1.23,4.83] | 0.0169 | 0.0267 |
| DRD_health | +0.40 [-0.10,+0.97] | +0.52 [+0.06,+0.93] | 0.0014 | 4.67 [0.75,8.46] | 0.0245 | 0.0267 |
| CR | +0.59 [+0.20,+0.81] | +0.57 [+0.33,+0.84] | 0.279 | 2.75 [1.55,7.22] | 0.0238 | 0.0267 |
| DRD | +0.32 [-0.18,+0.71] | +0.30 [-0.06,+0.57] | 0.000568 | 6.22 [1.09,11.65] | 0.0264 | 0.0267 |
| events | +0.62 [+0.24,+0.81] | +0.65 [+0.35,+0.80] | 0.000962 | 4.08 [1.51,6.14] | 0.0258 | 0.0267 |
| lam | -0.20 [-0.50,+0.04] | -0.23 [-0.45,+0.00] | 0.000634 | 11.74 [0.41,33.69] | 0.0265 | 0.0267 |
| lossrate | +0.65 [+0.31,+0.90] | +0.71 [+0.46,+0.87] | 0.0319 | 3.63 [1.62,5.38] | 0.0236 | 0.0267 |
| dev_frames | +0.62 [+0.27,+0.82] | +0.70 [+0.44,+0.86] | 0.000827 | 4.17 [1.96,6.05] | 0.0245 | 0.0267 |

### Episode-level: cumulative CR vs paired episode gain (held-out)

Pooled over conditions; Spearman within condition (ranks), then averaged, and pooled AUC for P(gain=+1) vs P(gain=-1) episodes.  CI: bootstrap over (condition, instance) clusters.

| target | predictor | mean within-cond Spearman [CI] | AUC(+1 vs -1) [CI] | P(gain>0 or <0) when predictor = 0 | P(gain != 0) when predictor > 0 |
|---|---|---|---|---|---|
| G_oracle | CR | +0.158 [+0.141,+0.174] | 0.745 [0.716,0.770] | 0.019 (n=5765) | 0.173 (n=10235) |
| G_oracle | DRD | +0.190 [+0.173,+0.205] | 0.821 [0.798,0.848] | 0.000 (n=4698) | 0.166 (n=11302) |
| G_oracle | events | +0.176 [+0.161,+0.190] | 0.793 [0.769,0.816] | 0.000 (n=3691) | 0.153 (n=12309) |
| G_health | CR_health | +0.199 [+0.176,+0.221] | 0.781 [0.748,0.810] | 0.002 (n=11380) | 0.264 (n=5420) |
| G_health | CR | +0.215 [+0.195,+0.233] | 0.825 [0.796,0.849] | 0.008 (n=5962) | 0.130 (n=10838) |
| G_health | events | +0.198 [+0.177,+0.216] | 0.846 [0.814,0.872] | 0.000 (n=4009) | 0.114 (n=12791) |

### A CR-threshold trigger, offline (held-out, all N=2 conditions)

Each DRD event is one decision where the full partner state would change the receiver's argmax (what DEV's info trigger approximates from common ground).  Keeping only events with CR > c:

| c | share of DRD events kept | share of total CR kept |
|---|---|---|
| 0 | 0.610 | 1.000 |
| 0.005 | 0.176 | 0.908 |
| 0.01 | 0.114 | 0.781 |
| 0.02 | 0.040 | 0.490 |
| 0.05 | 0.012 | 0.276 |
| 0.1 | 0.003 | 0.098 |

### Where CR sits in time under permanent loss (held-out)

| condition | CR before/at failure | CR within 2 steps after failure | CR later |
|---|---|---|---|
| perm | 0.0001 | 0.0001 | 0.0244 |
| perm_early | 0.0001 | 0.0001 | 0.0388 |
| perm_late | 0.0003 | 0.0001 | 0.0006 |
| perm_noev | 0.0000 | 0.0000 | 0.0250 |
| perm_travel | 0.0029 | 0.0017 | 0.1328 |

### Reading of the simulator results

* **Beacon gain (HEALTH - IMPLICIT): CR_health predicts it almost perfectly in rank**
  (Spearman +0.99 [+0.67, +1.00], Pearson +0.82, 21 conditions).  It gets the sign right
  everywhere.  It is negative for the transient and intermittent faults, where the
  one-shot beacon hurts (-0.036 / -0.038 measured, -0.038 / -0.064 predicted).  It is
  exactly 0 for total loss and the lost beacon.  It is reduced under the delay/burst
  links.  Its MAE is 0.017, against 0.027 for the mean-only predictor.  The naive
  predictors are clearly weaker: loss rate +0.65, DEV frames +0.62, private events +0.62,
  lambda -0.20.  Raw CR_health is nearly calibrated on held-out data (implied slope
  b x 1.98 = 1.05).  The dev-fitted slope does not transfer (held-out calibration slope
  1.98 [1.23, 4.83]), because dev gains were small and noisy at 20 episodes x 12 instances.
* **Full-information gain (ORACLE - IMPLICIT): CR is only moderately predictive and loses
  to the loss rate.**  CR: Spearman +0.52 [-0.05, +0.82].  Loss rate: +0.85 [+0.50,
  +0.94].  Events: +0.40.  lambda: -0.19.  DRD: +0.03.  CR fails on the travel and N >= 3
  cells, where it is large (0.012-0.038) but ORACLE does not beat IMPLICIT.  Greedy ORACLE
  is no optimality reference there, and the myopic single-agent rollout overstates what
  greedy can cash in.  So, on this simulator: CR measured with the right message content
  predicts the protocol that sends that content.  CR against a greedy "full information"
  reference is a weak predictor of the greedy reference's gain.
* **DRD (argmax changes) is a poor predictor at every level** (condition Spearman +0.03
  for ORACLE, +0.32 for HEALTH).  This matches the exact result that its scale carries
  no value information.
* **Episode level: weak.**  Within-condition Spearman of cumulative CR with the paired
  episode gain is +0.16 to +0.22.  AUC (gain +1 vs -1) is 0.75-0.83.  The private-event
  count does as well (AUC 0.79-0.85).  Episode gain is a {-1, 0, +1} coin flip dominated by
  luck, so this is not a usable per-episode predictor.  It is a necessary condition only:
  with CR = 0, the gain is nonzero in 0.2-2% of episodes.
* **CR timing under permanent loss:** about 99% of CR comes later than 2 steps after the
  failure (perm: 0.0244 of 0.0246).  The regret accumulates as the partner keeps
  "crediting" the dead agent's planned looks.  This agrees with the exact k = 3 death row
  (27% in the first step).  What happens at the failure is the information event, not the
  regret.
* **Offline CR trigger:** 61% of decisions with a DRD event carry positive CR.  Keeping
  only CR > 0.005 keeps 17.6% of DRD events and 91% of the CR mass.  CR > 0.01 keeps 11%
  of events and 78% of the mass.  An argmax-change trigger (DEV's info trigger) would send
  5-9x more often than a value-gated one for about 10-20% less recoverable regret.

## 4. Protocol design: "send iff CR > byte cost"

* **Principled trigger.**  Prop. `prop:cr` says the most any message stream can add to a
  base team pi is `E sum_t CR_t(pi)`.  A message that removes regret CR_t at cost c is worth
  sending iff CR_t > c (in P(detect) units, with lambda_bytes converting bytes).  It is the
  value-of-information rule applied to the partner's decision, not to the sender's belief.
* **DEVIATION (`deviation_comm.py`, info trigger) is the DRD half of this rule.**  It sends
  when the sender's records would change the partner's greedy argmax, as reconstructed from
  common ground.  That is a CR > 0 trigger with c = 0 that ignores how much the change is
  worth.  The exact table shows why this matters: DRD outruns VoC by 50-100x (median
  VoC/DRD = 0.01-0.02).  So most DRD events carry almost no regret.  A CR threshold keeps
  the few that matter.
* **The beacon is the CR rule's closed form under Prop. 3.**  Before any private event CR = 0
  (Corollary `cor:support`).  Under (i)-(iv) the only private event is the death, and a
  frame at D sets all later CR to 0 (Corollary `cor:beacon`).  So the rule fires once, at D,
  with a 12-byte frame: exactly HEALTH.  When the assumptions fail it says what to do
  instead.  Transient faults: the "drop forever" frame has negative CR after recovery, so
  the rule would add a recovery frame or not send.  Hints: residual CR (0.0079 of 0.0554
  in the weak-hint death row) stays after the frame, and only a CR-gated record message
  can remove it.
* **Caveat.**  The sender cannot compute the partner's CR exactly.  It needs the partner's
  decision rule, which it has (common ground / default plan), plus a value estimate (the
  rollout).  The simulator probe computes the receiver-side CR with full state, so it is
  an oracle version of the trigger.  A deployable version estimates it from common ground,
  as DEV already does for the argmax.

## 5. Figure (optional, appendix)

`paper/figs/figP_predictor.pdf` (7.0 x 2.0 in, Type 1 fonts).

```latex
\begin{figure*}[t]
\centering
\includegraphics[width=\textwidth]{figs/figP_predictor.pdf}
\caption{Coordination regret predicts the gain of the message that removes it. Each point
is one condition (held-out instances, seeds 4000--4015, 50 paired episodes each).
(a) Mean summed regret against full partner state versus $P_{\textsc{oracle}}-P_{\textsc{implicit}}$.
(b) Regret removed by the health frame, as the beacon would deliver it (link delay, lost
or dead radio, wrong ``dead forever'' reading after a transient fault), versus
$P_{\textsc{health}}-P_{\textsc{implicit}}$. Negative regret marks transient faults.
(c) Naive predictor: private events per episode. Titles give Spearman $\rho$ with 95\%
two-level bootstrap CIs. Dashed lines: $G=b\,x$ fitted on development seeds.}
\label{fig:predictor}
\end{figure*}
```

Main-text paragraph (about 4 sentences), for example after Prop. 3 or in Sec. "Lessons":

> *Coordination regret.* For any silent team, the most a protocol can add equals the expected
> sum of per-step coordination regret: the team value lost when an agent acts on its
> prediction of its partner rather than on the truth (Prop. X, App. Y).  Regret is zero
> until a private event occurs.  Under Prop. 3's assumptions one frame at the failure
> removes all of it, so ``send iff the regret a message removes exceeds its cost'' yields
> the beacon.  Measured in simulation, the regret a health frame would remove ranks the
> beacon's gain across 21 held-out conditions almost perfectly (Spearman 0.99, including
> its losses under transient faults), better than loss rate, event rate or message counts
> (0.62--0.65).  Argmax-change counts (DRD) do not (0.32).

## 6. Recommendation on inclusion

* **Include in the appendix (about 0.75 page):** Definition `def:cr`, Prop. `prop:cr` with
  its performance-difference proof sketch, Corollaries `cor:support` and `cor:beacon`, a
  condensed exact table (6-8 rows plus the death table), and Fig. `fig:predictor`.
  **Main text:** the paragraph above.  It ties Obs. 1, Remark `rem:measurable` and Prop. 3
  to one quantity.  It explains why the beacon sends once while an argmax-change trigger
  sends often.  And it adds a held-out, falsifiable prediction (the beacon's gain sign and
  size per condition) that came out right.
* **Claims to avoid:**
  * Do not say CR predicts the value of communication in general.  Against greedy
    full-information ORACLE it is only moderate (Spearman 0.52, CI includes 0), and the
    loss rate beats it (0.85).
  * Do not claim episode-level predictiveness.
  * Do not call the bound tight: it is tight only for decentralised-optimal teams.
  * Do not present Prop. `prop:cr` as new: it is the performance-difference lemma applied
    to a team.
  * The simulator CR is an oracle-side, myopic rollout estimate, not the exact Q*.
* **Caveats to state:** one grid family (5x5, H16/H30) and one rollout budget (6).  The
  dev-fitted calibration slope did not transfer (raw CR_health is the better-calibrated
  number: held-out G ~= 1.05 CR_health).  There are 20-21 conditions, chosen by us, and
  several share instances.  The 1000-1099 dev range was used only as 1000-1011.

## Reproduce

```
cd Tools
fsrun-quick python3 -u -m ForestSearch.predictor exact --out /tmp/pred_exact.json        # main rows (hit the 10-min cap after the k=3 weak row)
fsrun-quick python3 -u -m ForestSearch.predictor exact --part death --out /tmp/pred_exact_death.json
fsrun-quick python3 -u -m ForestSearch.predictor sim --split dev --episodes 20 --out /tmp/pred_dev.json      # ~100 s
fsrun-quick python3 -u -m ForestSearch.predictor sim --split heldout --episodes 50 --out /tmp/pred_ho.json   # ~330 s
fsrun-quick env PYTHONHASHSEED=0 ~/.cache/research/fs-venv/bin/python -m ForestSearch.predictor report \
    --dev /tmp/pred_dev.json --heldout /tmp/pred_ho.json --out /tmp/predictor_sim.md \
    --fig ForestSearch/paper/figs/figP_predictor.pdf
python3 -m pytest -q ForestSearch/test_predictor.py     # 11 pass
```
