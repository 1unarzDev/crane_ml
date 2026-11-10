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
