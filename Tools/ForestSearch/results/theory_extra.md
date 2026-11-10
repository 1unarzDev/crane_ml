# Theory additions for review A (M1, M4, M5, PBP): numbers + paste-ready LaTeX

Code: `theory_extra.py` (tasks `landscape`, `landscape3`, `points`, `beacon`,
`bound`, `pbp`, `plot`), tests `test_theory_extra.py` (18 pass: fast V_cent =
exact_dec V_cent, reviewer max by brute force, sandwich OL <= dec <= cent,
exact best response, Prop. 3 at k=2 for n=2,3 and the construction at k=2..4).
Raw json: `results/theory_extra_{landscape,landscape3,points,beacon,pbp}.json`.
Landscape detail: `results/vov_landscape.md`.  Figure: `paper/figs/figA_voc.pdf`
(Type 1 fonts, 7.0 x 1.85 in).

## 1. Key numbers

* Reviewer maximum reproduced exactly: VoC = 0.08208 (11.5% rel.), n=2, T=2,
  h_g=0, h_b=1, alpha=0 (brute force agrees).  At T=3 the same instance gives
  0.0335; the upper bound falls to 0.012 / 0.0047 / 0.0021 at T=4/5/6.
* Base n=2 instance at the paper's rates (lambda=1..3, h_g fixed by lambda):
  VoC = 1.2e-4 / 4.6e-4 / 1.0e-3 (no aborts, T=2), 8.1e-4 / 4.5e-3 / 8.4e-3
  with lambda-scaled aborts.  Same rates on the adversarial instance: 0.011.
* T=2 landscape maxima (h_g = 0.3 base / 0 adversarial): 0.017 / 0.082 at
  rho=0, 0.019 / 0.065 at rho=0.1, all at h_b=1, alpha=0.

## 2. Prop. 3 (health beacon): numerical check, death hazard delta=0.2 per look

| instance | T | V_cent | V_dec, 1 frame ideal | V_dec, no frame | V_dec, frame lost w.p. .5 |
|---|---|---|---|---|---|
| n=2 base, no hints | 2 | 0.575836 | 0.575836 (VoC 0) | 0.562893 (VoC 0.0129) | 0.564941 (VoC 0.0109) |
| n=2 base, no hints | 3 | 0.629397 | 0.629397 (VoC 0) | 0.622764 (VoC 0.0066) | 0.623923 (VoC 0.0055) |
| n=3 base, no hints | 2 | 0.478669 | 0.478669 (VoC 0) | 0.465152 (VoC 0.0135) | 0.468224 (VoC 0.0104) |
| n=2 weak hints | 2 | 0.584910 | 0.584812 (VoC 9.8e-5) | 0.569628 (VoC 0.0153) | |
| n=2 strong hints | 2 | 0.601288 | 0.601091 (VoC 2.0e-4) | 0.583388 (VoC 0.0179) | |
| adversarial (h_b=1) | 2 | 0.518548 | 0.488694 (VoC 0.0299) | 0.488694 (VoC 0.0299) | |

V_dec is the exact optimum by brute force over all deterministic policy pairs
(64 / 4096 / 216 / 1024 policies per agent), where each agent's policy maps
its history of (own outcome, partner-frame bit) to a look.  Independently, the
Prop. 3 construction (each agent replays the centralised optimal policy on
the reconstructed joint history) evaluates to V_cent to 1e-16 at T=2..5
(n=2) and T=2..4 (n=3).  The reviewer instance without hints (h_b=0) is a
case where the frame is almost worthless (VoC without frame 1.7e-5 at T=2,
0 at T=3); Prop. 3 says the frame closes the gap, not that the gap is large.

## 3. Upper bound VoC <= V_cent - V_lower, k = 2..6

V_OL = best open-loop pair (exact, enumeration of look multisets);
V_PBP = person-by-person-optimal pair reached by alternating exact best
responses from IMPLICIT; both are values of feasible decentralised pairs,
so bound_OL = V_cent - V_OL and bound = V_cent - max(V_OL, V_PBP) are
upper bounds on VoC.  Exact VoC where V_dec is computable.

| instance | k | V_cent | V_OL | V_PBP | V_cent-V_OL | V_cent-max(V_OL,V_PBP) | exact VoC |
|---|---|---|---|---|---|---|---|
| n2 none ab0 | 2..6 | | = V_cent | = V_cent | 0 | 0 | 0 (Prop. 2) |
| n2 weak ab0 | 2 | 0.70866 | 0.69568 | 0.70847 | 1.30e-2 | 1.92e-4 | 1.92e-4 |
| n2 weak ab0 | 3 | 0.77169 | 0.76442 | 0.77041 | 7.27e-3 | 1.28e-3 | 3.74e-4 |
| n2 weak ab0 | 4 | 0.79589 | 0.79058 | 0.79520 | 5.31e-3 | 6.82e-4 | |
| n2 weak ab0 | 5 | 0.80545 | 0.80285 | 0.80497 | 2.60e-3 | 4.76e-4 | |
| n2 weak ab0 | 6 | 0.80932 | 0.80779 | 0.80908 | 1.53e-3 | 2.34e-4 | |
| n2 strong ab0 | 2 | 0.73003 | 0.69568 | 0.72964 | 3.43e-2 | 3.84e-4 | 3.84e-4 |
| n2 strong ab0 | 3 | 0.78512 | 0.76442 | 0.78236 | 2.07e-2 | 2.75e-3 | 2.57e-3 |
| n2 strong ab0 | 4 | 0.80348 | 0.79058 | 0.80200 | 1.29e-2 | 1.48e-3 | |
| n2 strong ab0 | 5 | 0.80920 | 0.80285 | 0.80841 | 6.35e-3 | 7.93e-4 | |
| n2 strong ab0 | 6 | 0.81109 | 0.80779 | 0.81075 | 3.30e-3 | 3.43e-4 | |
| n2 none ab.2 | 2 | 0.62083 | 0.59802 | 0.61420 | 2.28e-2 | 6.64e-3 | 1.92e-3 |
| n2 none ab.2 | 3 | 0.71764 | 0.69563 | 0.71347 | 2.20e-2 | 4.17e-3 | 3.20e-3 |
| n2 none ab.2 | 4 | 0.76305 | 0.74357 | 0.75998 | 1.95e-2 | 3.06e-3 | |
| n2 none ab.2 | 5 | 0.78644 | 0.77320 | 0.78521 | 1.32e-2 | 1.23e-3 | |
| n2 none ab.2 | 6 | 0.79844 | 0.78848 | 0.79739 | 9.96e-3 | 1.04e-3 | |
| n2 weak ab.2 | 2 | 0.63194 | 0.59802 | 0.62928 | 3.39e-2 | 2.66e-3 | 2.66e-3 |
| n2 weak ab.2 | 3 | 0.72658 | 0.69563 | 0.72312 | 3.09e-2 | 3.46e-3 | |
| n2 weak ab.2 | 4 | 0.77058 | 0.74357 | 0.76715 | 2.70e-2 | 3.43e-3 | |
| n2 weak ab.2 | 5 | 0.79155 | 0.77320 | 0.78941 | 1.84e-2 | 2.14e-3 | |
| n2 weak ab.2 | 6 | 0.80178 | 0.78848 | 0.80078 | 1.33e-2 | 1.00e-3 | |
| n2 strong ab.2 | 2 | 0.64998 | 0.59802 | 0.64526 | 5.20e-2 | 4.73e-3 | 4.73e-3 |
| n2 strong ab.2 | 3 | 0.74461 | 0.69563 | 0.73922 | 4.90e-2 | 5.39e-3 | |
| n2 strong ab.2 | 4 | 0.78362 | 0.74357 | 0.77889 | 4.01e-2 | 4.73e-3 | |
| n2 strong ab.2 | 5 | 0.79991 | 0.77320 | 0.79700 | 2.67e-2 | 2.91e-3 | |
| n2 strong ab.2 | 6 | 0.80683 | 0.78848 | 0.80551 | 1.84e-2 | 1.32e-3 | |
| n2 paper lambda=1 (rho=.1) | 2 | 0.67535 | 0.64270 | 0.67454 | 3.26e-2 | 8.11e-4 | 8.11e-4 |
| n2 paper lambda=1 | 3 | 0.75522 | 0.73395 | 0.75120 | 2.13e-2 | 4.01e-3 | |
| n2 paper lambda=1 | 4 | 0.78736 | 0.76915 | 0.78476 | 1.82e-2 | 2.60e-3 | |
| n2 paper lambda=1 | 5 | 0.80129 | 0.79142 | 0.80001 | 9.87e-3 | 1.28e-3 | |
| n2 paper lambda=1 | 6 | 0.80724 | 0.80036 | 0.80634 | 6.88e-3 | 8.97e-4 | |
| **adversarial (h_b=1)** | 2 | 0.71268 | 0.62090 | 0.63060 | 9.18e-2 | 8.21e-2 | 8.21e-2 |
| adversarial | 3 | 0.86489 | 0.76229 | 0.83143 | 1.03e-1 | 3.35e-2 | 3.35e-2 |
| adversarial | 4 | 0.91621 | 0.84019 | 0.90428 | 7.60e-2 | 1.19e-2 | |
| adversarial | 5 | 0.93560 | 0.88367 | 0.93086 | 5.19e-2 | 4.74e-3 | |
| adversarial | 6 | 0.94308 | 0.90827 | 0.94098 | 3.48e-2 | 2.10e-3 | |
| n3 none ab0 | 2..5 | | = V_cent | = V_cent | 0 | 0 | 0 |
| n3 none ab.2 | 2 / 3 / 4 / 5 | .51743 / .63202 / .70763 / .74889 | | | 2.08e-2 / 2.62e-2 / 2.63e-2 / 2.90e-2 | 1.60e-3 / 1.48e-3 / 2.14e-3 / 3.07e-3 | 1.60e-3 (k=2) |
| n3 weak ab0 | 2 / 3 / 4 | .60566 / .71874 / .76875 | | | 1.53e-2 / 1.60e-2 / 1.07e-2 | 3.65e-4 / 8.90e-4 / 2.13e-3 | 3.65e-4 (k=2) |
| n3 weak ab.2 | 2 / 3 / 4 | .52877 / .64791 / .72029 | | | 3.21e-2 / 4.21e-2 / 3.90e-2 | 2.35e-3 / 2.29e-3 / 4.22e-3 | 2.35e-3 (k=2) |

(The n3 strong rows and n3 k=5 hint rows were still running at the time
cap; see the `bound` task.)

How VoC grows with T.  It does not grow much.  The tight bound
V_cent - max(V_OL, V_PBP) stays at or below 5.4e-3 for every base-instance
row at every k <= 6, peaks at k = 3 or 4 and then falls, because the n=2
team saturates (V_cent -> 0.81).  The open-loop bound is loose by 1 to 2
orders of magnitude (e.g. weak ab0, k=2: 1.3e-2 vs 1.9e-4) and also shrinks
with k.  On the adversarial instance the exact VoC falls from 0.082 (T=2) to
0.033 (T=3) and the bound to 0.0021 at T=6: with more looks the team can
learn by itself what a near-certain hint would have told the partner.  At n=3
with aborts the bound grows slowly (1.6e-3 -> 3.1e-3 from k=2 to 5); larger
grids have not saturated by T=6, so the paper should not claim VoC shrinks
with T in general.  Caveat: on the adversarial instance the IMPLICIT tree
at k >= 4 hits a 0*log 0 in model.posterior (h_b=1, alpha=0); V_PBP is still
a valid lower bound because every pair value is evaluated exactly.

## 4. Person-by-person optimality of IMPLICIT

gain_j = (best response of agent j to the partner's fixed IMPLICIT tree) -
V_implicit, exact.  "PBP" = both gains <= 1e-9.

| instance | T=2 | T=3 | T=4 |
|---|---|---|---|
| n2 none ab0 | yes | yes | yes |
| n2 none ab.2 | yes | no (agent 1: 4.3e-3) | no (2.4e-3) |
| n2 weak ab0 | yes | yes | no (1.0e-4) |
| n2 weak ab.2 | yes | no (1.6e-3) | no (1.0e-3) |
| n2 strong ab0 | yes | yes | no (6.6e-5) |
| n2 strong ab.2 | yes | no (2.0e-3) | no (1.2e-3) |
| n2 noisy ab0 | no (agent 0: 2.9e-3) | yes | yes |
| n2 noisy ab.2 | no (7.8e-4) | no (2.4e-3) | no (1.5e-3) |
| n3 none ab0 | yes | yes | |
| n3 none ab.2 | yes | yes | |
| n3 weak ab0 | yes | no (1.2e-3) | |
| n3 weak ab.2 | yes | no (5.6e-5) | |
| n3 strong ab0 | yes | no (1.3e-3) | |
| n3 strong ab.2 | no (5.4e-4) | no (1.1e-4) | |
| n3 noisy ab0 | yes | no (1.4e-3) | |
| n3 noisy ab.2 | yes | yes | |
| adversarial | no (6.4e-3) | no (4.5e-2) | |

(Largest single-agent gain shown.)  On the base grid IMPLICIT is PBP-optimal
in 13 of 16 T=2 rows and fails mostly at T >= 3, with unilateral gains
<= 4.3e-3.  On 600 random n=2, T=2 instances (exact_dec generator, seed
2026) IMPLICIT is *not* PBP-optimal on 71%; the largest unilateral gain is
0.134 (mean 0.013).  Alternating best responses from IMPLICIT reach the
decentralised optimum on 78% of them (max remaining gap 0.039).  So
IMPLICIT is not, in general, even person-by-person optimal; its losses
against the best decentralised pair are greedy-planning losses, not
coordination failures that one agent could fix alone, on most base rows.

---

## 5. Paste-ready LaTeX

### 5a. Replacement for the "at most 0.034" sentence (Sec. 3, "With hints or aborts")

```latex
\paragraph{With hints or aborts.} We computed both values exactly on tiny travel-free instances (Appendix~\ref{app:exact}). With aborts or informative hints VoC is usually positive but small: over 600 random $n=2$, $T=2$ instances its mean is $3.9{\times}10^{-3}$ and the largest value \emph{among those draws} is $0.034$. That is a sample maximum, not a bound. A targeted search finds $\text{VoC}=0.082$ (11.5\% relative) at $n=2$, $T=2$, when a hint certifies ``target here, view bad'' ($h_g=0$, $h_b=1$, $\alpha=0$; Fig.~\ref{fig:voc}). At the paper's rates ($h_b=0.15$--$0.45$, $\alpha=0.03$--$0.09$ for $\lambda=1$--$3$) the base tiny instance has $\text{VoC}=1.2{\times}10^{-4}$--$8.4{\times}10^{-3}$, and the same rates cut the adversarial instance's VoC from $0.082$ to $0.011$. \imp{} itself can fall short of the best decentralized pair (gap up to $0.14$), and it is not always person-by-person optimal (Appendix~\ref{app:exact}).
```

Abstract fix: replace "at most 0.034" (if present) by "small at our base
rates (below 0.01 on tiny instances) but up to 0.08 when hints are
near-certain".

### 5b. Figure snippet (appendix)

```latex
\begin{figure*}[t]
\centering
\includegraphics[width=\textwidth]{figs/figA_voc.pdf}
\caption{VoC is large only when a private hint is near-certain. Exact $\text{VoC}=V_{\text{cent}}-V_{\text{dec}}$ at $n=2$, $T=2$ against the bad-view hint rate $h_b$ and the false-hint rate $\alpha$, with abort rate $\rho\in\{0,0.1\}$. Left two panels: the base tiny instance with $h_g=0.30$; circles mark the paper's rates $(\alpha,h_b)=\lambda\,(0.03,0.15)$ for $\lambda=1,2,3$ (there $h_g$ and $\rho$ also scale with $\lambda$). Right two panels: the adversarial instance found by search ($h_g=0$). Stars mark each panel's maximum (value printed). One color scale for all panels.}
\label{fig:voc}
\end{figure*}
```

(If space is tight, use only the $\rho=0$ panels; the figure is 7.0 x 1.85 in.)

### 5c. Proposition 3 (health beacon)

```latex
\paragraph{Agent loss.} Extend the model of Prop.~2 by \emph{sensor death}: each look of a live agent may instead return DEAD, after which that agent's sensor is permanently off (the look is wasted and later looks yield nothing). Death may depend on time and on the agent but not on the agent's partner's private information. A \emph{one-frame health beacon} sends a single frame when an agent's sensor dies.

\begin{proposition}[Health beacon]\label{prop:beacon}
Assume (i) and (ii) of Prop.~2 with agent loss allowed in (ii); (iii$'$) every look of a live agent returns NULL, DETECT or DEAD, DETECT is never false and ends the mission, and DEAD is observed by the dying agent at once; (iv) at its death step $t$ an agent sends one frame, which is delivered before every partner's decision at step $t{+}1$. Then for any number of agents $N$, any $n$, $V$ and horizon $T$, the best pre-coordinated decentralized team with the beacon attains $V_{\text{cent}}$, the optimal value of a centralized team that observes all outcomes.
\end{proposition}

\begin{proof}
Fix a step $t$ and a path on which no DETECT has occurred. Every look on it returned NULL or DEAD, and DEAD is absorbing, so the joint outcome history $h_t$ is a function of the vector of death times truncated at $t{-}1$, $\bigl(\min(D_j,t)\bigr)_{j\le N}$. Agent $j$ knows $\min(D_j,t)$ by (iii$'$). For a partner $k$, by (iv) agent $j$ has received $k$'s frame if and only if $D_k\le t-1$, and then knows $D_k$; otherwise it knows $D_k\ge t$. Hence there is a map $\phi^j_t$ with $h_t=\phi^j_t(h^j_t)$ for every continuing path, where $h^j_t$ is agent $j$'s local history (own outcomes and received frames). Let $\mu^*=(\mu^*_t)$ be an optimal centralized policy and define $\pi^j_t(h^j_t)=[\mu^*_t(\phi^j_t(h^j_t))]_j$. By induction on $t$, on every continuing path the team $(\pi^j)$ takes the same joint action as $\mu^*$, so the two induce the same law of the first detection time and $V(\pi)=V(\mu^*)=V_{\text{cent}}$. The reverse inequality $V_{\text{dec}}\le V_{\text{cent}}$ holds because every decentralized team is a centralized policy.
\end{proof}

\emph{Scope.} Prop.~\ref{prop:beacon} is about optimal policies: it says the beacon restores the full information structure, not that the greedy \health{} protocol is optimal. It needs all four assumptions. If the frame can be lost, ``no frame'' no longer certifies that the partner is alive; if hints or aborts exist, an agent's private history has other components no frame reports. On tiny instances (death probability $0.2$ per look, $n=2,3$, $T\le3$, exact brute force over all policy pairs) the one-frame team attains $V_{\text{cent}}$ exactly, whereas without the frame VoC is $0.0066$--$0.0135$, with a frame lost w.p.\ $0.5$ it is $0.0055$--$0.0109$, and with hints and an ideal frame it is $10^{-4}$--$0.030$. This turns the agent-loss result of Sec.~\ref{sec:loss} from a screening finding into a predicted exception: agent death is the one private event that a single, fixed-size message fully discloses.
```

### 5d. Remark: exact condition for VoC = 0

```latex
\begin{remark}[When is VoC zero?]\label{rem:measurable}
Call a centralized policy $\mu=(\mu_t)$ \emph{locally implementable} if for every agent $j$, step $t$ and pair of joint histories $h_t,h'_t$ that are reachable under $\mu$ and on which agent $j$'s own history agrees ($h^j_t=h'^j_t$), we have $[\mu_t(h_t)]_j=[\mu_t(h'_t)]_j$, i.e.\ agent $j$'s component of $\mu$ is measurable with respect to its own history on reachable paths. Then $\text{VoC}=0$ if and only if some centralized-optimal policy is locally implementable. \emph{Proof.} ($\Leftarrow$) Set $\pi^j_t(h^j_t)=[\mu_t(h_t)]_j$ for any reachable $h_t$ consistent with $h^j_t$ (well defined by assumption); by induction over $t$ the team plays $\mu$ on every reachable path, so $V_{\text{dec}}\ge V(\mu)=V_{\text{cent}}$. ($\Rightarrow$) An optimal decentralized pair $\pi^*$ (it exists: the policy sets are finite) is itself a centralized policy $\mu(h_t)=(\pi^{*j}_t(h^j_t))_j$, locally implementable by construction, and optimal because $V(\pi^*)=V_{\text{dec}}=V_{\text{cent}}$. \qedhere
\end{remark}

Prop.~2 and Prop.~\ref{prop:beacon} are the two cases where local implementability holds for \emph{every} policy, because each agent's history determines the joint history on continuing paths. The counterexample in Appendix~\ref{app:exact} is a failure of local implementability: in the both-hint branch the team's action for B depends on A's hint, which B does not see. When hints are uninformative ($h_g=h_b=\alpha$, no aborts) an optimal centralized policy ignores them and is locally implementable, so VoC $=0$; the \emph{noisy} rows of the base grid happen to admit a locally implementable optimum at $T=2$, which does not generalize.
```

### 5e. Bound (appendix, after the exact table)

```latex
\paragraph{Longer horizons.} Exact $V_{\text{dec}}$ is out of reach beyond $T=3$, but any feasible decentralized pair gives a lower bound. With $V_{\text{OL}}$ the best open-loop pair (each agent a fixed look sequence) and $V_{\text{PBP}}$ a person-by-person-optimal pair reached by alternating exact best responses from \imp{},
\[
V_{\text{OL}}\le V_{\text{dec}}\quad\text{and}\quad V_{\text{PBP}}\le V_{\text{dec}}\quad\Longrightarrow\quad \text{VoC}\le V_{\text{cent}}-\max(V_{\text{OL}},V_{\text{PBP}}).
\]
For $n=2$ and $T=2,\dots,6$ on the base instances, $V_{\text{cent}}-\max(V_{\text{OL}},V_{\text{PBP}})\le 5.4{\times}10^{-3}$ throughout (the cheaper $V_{\text{cent}}-V_{\text{OL}}$ is $1.5{\times}10^{-3}$--$5.2{\times}10^{-2}$). On the adversarial instance the exact VoC falls from $0.082$ ($T=2$) to $0.033$ ($T=3$), and the bound is $0.012$, $0.0047$ and $0.0021$ at $T=4,5,6$. In these tiny instances VoC does not grow with $T$, because the team saturates; for $n=3$ with aborts the bound grows slowly ($1.6{\times}10^{-3}$ at $T=2$ to $3.1{\times}10^{-3}$ at $T=5$), so we make no claim for larger grids.
```

### 5f. Person-by-person optimality (appendix sentence; also answers the "silence is enough" scope point)

```latex
\paragraph{Is \imp{} person-by-person optimal?} Not in general. Holding the partner's \imp{} policy tree fixed and computing one agent's exact best response, \imp{} is person-by-person optimal on 13 of 16 base-grid instances at $T=2$, but it fails on most hint- and abort-rows at $T=3,4$ (largest unilateral gain $4.3{\times}10^{-3}$), and on 71\% of 600 random $n=2$, $T=2$ instances (largest gain $0.134$). The ``silence is enough'' claim is therefore relative to greedy planning: silence loses nothing against centralized \emph{greedy}, but greedy silent agents are not decentralized-optimal.
```

### 5g. Note on Prop. 2's assumptions (M2), if Prop. 2 is relabelled "Observation"

```latex
The observation additionally assumes that DETECT is never false, that the objective depends only on the joint sequence of looks, and that policies are pre-coordinated (common knowledge); it holds for any number of agents $N$. Condition (iii) with no aborts already gives the no-abort part of (ii).
```
