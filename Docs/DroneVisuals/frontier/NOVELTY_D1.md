# NOVELTY_D1: decision-aware communication gates supervised by counterfactual coordination-regret labels

Literature critic, 2026-11-10. **[V]** means checked on the primary abstract or full-text page. **[S]** means search snippet or secondary page only. **UNVERIFIED** means the claim was not confirmed. This brief does not repeat `mission/LITREVIEW.md` (ConTaCT, ACHORD, MOCHA rows).

**Verdict: the plan's claim is NOT novel as worded.** Training a send/no-send gate with supervision from counterfactual value-of-communication labels was published in 2018 (ATOC) and has been refined since (I2C 2020, CMVC 2025, MUTE 2026). The fallback in the plan ("reframe as a regime map") is the correct move.

## Closest prior work

| # | Work (year) | What it already does | Gap left for us |
|---|---|---|---|
| 1 | **ATOC**: Jiang & Lu, NeurIPS 2018, [arXiv 1805.07733](https://arxiv.org/abs/1805.07733) [V, full text] | The attention unit, which decides whether to initiate communication, "is trained as a binary classifier". Labels come from ΔQ = mean Q(coordinated) − Q(independent), min–max normalised, with log loss. **This is a counterfactual value-labelled gate.** | Labels come from a learned critic. No byte cost, no lossy or bursty link, no agent loss. |
| 2 | **I2C**: Ding, Huang & Lu, NeurIPS 2020, [arXiv 2006.06455](https://arxiv.org/abs/2006.06455) [V abstract; KL detail S] | A prior network trained on labels inferred causally from the joint Q. It decides whom to talk to from local observations. | Labels measure influence (policy KL), not regret. No budget, no loss. |
| 3 | **CMVC**: Gao, Qu, Hao & Ke, IEEE TSMC:Systems 2025, [XJTU record](https://scholar.xjtu.edu.cn/en/publications/communication-in-multiagent-reinforcement-learning-via-counterfac/) [S; the IEEE page did not load] | Counterfactual message value (CMV), a difference-reward style per-message value. Each agent predicts teammates' CMV from local observations and requests a message only when CMV > 0. | Same as above. The training details are **UNVERIFIED**. |
| 4 | **MUTE**: Zuo et al., arXiv 2026, [2607.03473](https://arxiv.org/abs/2607.03473) [V abstract] | CMV is computed by masking a message and measuring the change in Q_tot. Low-value messages are unlearned. It gives **an upper bound on return loss** and reports 80–90% bandwidth savings. | The bound has the same role as our PDL argument, so the PDL derivation is not a novelty lever. No packet loss is mentioned. |
| 5 | **IC3Net**: Singh, Jain & Sukhbaatar, ICLR 2019, [1812.09755](https://arxiv.org/abs/1812.09755) [V] | A binary gate trained by REINFORCE. Per the JaxMARL issue and Kaman 2026 [S], this is the high-variance baseline that we plan to beat. | none |
| 6 | **ETCNet**: Hu et al., arXiv 2020, [2010.04978](https://arxiv.org/abs/2010.04978) [V] | An event-triggered send decision, posed as a constrained MDP under a bandwidth limit. | This is the CMDP/budget formulation we propose; it is already taken. |
| 7 | **Gated-ACML**: Mao et al., AAAI 2020, [1912.05304](https://arxiv.org/abs/1912.05304) [V] | Message pruning under limited bandwidth, with a gate trained by an auxiliary task. | none |
| 8 | **VBC**: Zhang, Zhang & Lin, NeurIPS 2019, [1909.02682](https://arxiv.org/abs/1909.02682) [V] | Replies only if the message variance exceeds a threshold. A cheap hand-made trigger. | none |
| 9 | **SchedNet** (Kim et al., ICLR 2019, [1902.01554](https://arxiv.org/abs/1902.01554)) [V]; **TarMAC** (Das et al., ICML 2019, [1810.11187](https://arxiv.org/abs/1810.11187)) [V] | Limited bandwidth with medium contention (scheduling who broadcasts), and targeted attention. | Neither handles agent loss. |
| 10 | **Model-based value-of-information (VoI) communication**: Roth, Simmons & Veloso, AAMAS 2005 ([CMU](https://www.ri.cmu.edu/publications/reasoning-about-joint-beliefs-for-execution-time-communication-decisions/)) [V]; Becker et al., Comput. Intell. 2009 ([DOI](https://onlinelibrary.wiley.com/doi/10.1111/j.1467-8640.2008.01329.x)) [V]; Unhelkar & Shah (ConTaCT), AAAI 2016 ([MIT](https://interactive.mit.edu/biblio/contact-deciding-communicate-during-time-critical-collaborative-tasks-unknown-deterministic-domains)) [V] | Each computes at execution time whether sending raises expected joint value enough to justify its cost. Becker 2009 lists the error sources of *myopic* value-of-communication. | Our one-step ΔCR label is myopic in exactly this sense. |

Also relevant: Weil et al., arXiv 2023, [2311.14770](https://arxiv.org/abs/2311.14770) [V] learns *message size* over unreliable channels. Its loss model, i.i.d. or bursty, is **UNVERIFIED**.

## What's incremental (a reviewer will say "done")

- **A gate trained with supervised labels from counterfactual communication value.** ATOC (2018) did this, as did I2C, CMVC and MUTE.
- **"max E[V] − λ·bytes" as the objective, or a constrained MDP under a bandwidth limit.** ETCNet and Gated-ACML have this.
- **Using a performance-difference or regret decomposition to justify the label.** MUTE gives a return-loss bound, and difference rewards are older still.
- **Beating an IC3Net-style REINFORCE gate.** This is expected, not a contribution. Lower-variance supervised gates have been beating it since 2018.
- **Logistic or MLP models on hand-made features (DRD, silence age).** This is feature engineering.

## Remaining defensible delta

1. **The label source.** Our labels are ground-truth regret differences estimated by paired common-random-number rollouts in the true simulator. Prior work uses ΔQ from a jointly trained, biased critic. The testable claim is that rollout labels give better-calibrated gates at low byte budgets than critic labels. This needs an ATOC-style ΔQ-label arm on identical features.
2. **The channel and failure model.** No prior work we found combines bursty (Gilbert–Elliott) links, agent *loss* and typed messages with different byte costs (beacon, record, claim). The loss models in those papers are **UNVERIFIED**, so check the full texts of 2311.14770 and ETCNet before claiming this.
3. **The herding failure at N = 5–10.** Here the beacon *hurts*: survivors converge on a dead agent's sector. This is a coordination pathology, not a bandwidth one. We found no communication-learning paper that studies it. The market-reallocation literature (Karmani et al., SASO 2007 [S]) does not study it either.

**Rewritten claim.**
> In decentralized search with agent loss and bursty links, how far does a cheap, decision-aware gate close the gap to oracle communication? We report this as a regime map over N, loss rate and burstiness, against both a 12-byte hand-designed beacon and critic-labelled gates (ATOC/CMV style).
>
> The contribution is the map, plus a takeover rule that removes the herding failure. It is not "the first regret-supervised gate."

## Baselines a reviewer will demand

- An ATOC-style ΔQ-classifier gate.
- An I2C prior network.
- A CMV/MUTE-style masked-message value.
- IC3Net (REINFORCE).
- ETCNet (constrained-MDP trigger).
- A VBC variance threshold.
- **Execution-time myopic VoI** (DEC-COMM/ConTaCT): run the same rollout model online with no learning. This is the most dangerous baseline. If it matches the learned gate, learning only buys runtime.
- HEALTH and HEALTH_PART, periodic sending, FULL and ORACLE.
- For herding, a CBBA or auction re-allocation baseline.

## Contradicting evidence

- Our own [workshop] result ("silence is enough for greedy agents; a 12-byte beacon recovers the gain") predicts that little headroom is left at N = 2. LIT_ROLES notes that role-free homogeneous policies already reach high recall (Nathan 2025).
- Becker 2009 shows that myopic value-of-communication errs systematically. One-step ΔCR labels will undervalue messages whose payoff is delayed, such as a takeover claim.

Note: `~/worktrees/crane-forest-sar/paper/main.tex` was not found at that path. The [workshop] claims above are taken from the task brief.

## Falsifiable predictions

- **P1.** At N = 2, the learned gate is within ±2 pp of HEALTH at matched bytes. Any gain appears only at N ≥ 5.
- **P2.** At ≤ 5% of FULL's bytes, rollout-labelled gates beat ΔQ-critic-labelled gates by ≥ 5 pp of oracle gain. If the gap is < 2 pp, delta (1) is dead.
- **P3.** The execution-time myopic-VoI baseline achieves ≥ 90% of the learned gate's gain. If so, report "learning is a compression of myopic VoI".
- **P4.** Herding index under HEALTH at N = 10 is ≥ 2× that at N = 5. A claim-type message cuts it by ≥ 50% without raising bytes by > 2×.
- **P5.** Under bursty loss at a matched mean loss rate, every gate loses ≥ 2× more oracle gain than under i.i.d. loss. If not, the bursty-link framing is cosmetic.

## Kill condition

On DEV, kill the direction if either condition holds:
- the learned gate is within ±2 pp of HEALTH/HEALTH_PART at matched bytes for every N ≤ 10, **or**
- it is within ±2 pp of the execution-time myopic-VoI baseline.

In either case, publish "a hand-made beacon plus myopic VoI is near-optimal; herding needs a claim rule" as a short note.
