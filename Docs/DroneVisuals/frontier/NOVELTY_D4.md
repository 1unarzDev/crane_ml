# NOVELTY_D4: multi-fidelity experiment selection and ranking transfer across robotics simulators

Literature critic, 2026-11-10.

Tag legend:
- **[V]**: primary abstract or record checked.
- **[S]**: snippet only.
- **UNVERIFIED**: not confirmed.

**Verdict: weakest novelty of the four directions. It is an infrastructure win, not a research claim, unless it is reframed.** Prior work already covers most of the plan's pieces:
- Robotics-specific multi-fidelity selection between a simulator and an expensive source: Marco et al. 2017; Cutler et al. 2015.
- Measuring rank transfer between a cheap and an expensive evaluator: Kadian et al. 2020 (SRCC); SIMPLER 2024.
- Multi-fidelity failure falsification: Shahrooei et al. 2023; Koren et al. 2021.
- The modelling and acquisition machinery is standard: Kennedy–O'Hagan AR1, MF-GP-UCB, BOCA, MF-MES, BOHB.

## Closest prior work

| # | Work (year) | What it already does | Gap |
|---|---|---|---|
| 1 | **Marco, Berkenkamp, Hennig, Schoellig, Krause, Schaal & Trimpe**, ICRA 2017 ([arXiv 1703.01250](https://arxiv.org/abs/1703.01250)) [V] | Entropy search over simulator and real-robot sources, trading information against cost to tune controllers (cart-pole). | Two fidelities. Continuous gains. No ranking/speed-regime analysis. |
| 2 | **Cutler, Walsh & How**, T-RO 31(3), 2015 ([DOI 10.1109/TRO.2015.2419431](https://doi.org/10.1109/TRO.2015.2419431)) [V] | Rules for when to move between a *ladder* of simulators of increasing fidelity, applied to robots. | RL sample efficiency, not config selection. |
| 3 | **Kadian et al.**, RA-L 2020, "Sim2Real Predictivity" ([arXiv 1912.06321](https://arxiv.org/abs/1912.06321)) [V] | Defines the **Sim-vs-Real Correlation Coefficient (SRCC)** over 9 navigation models. SRCC was 0.18 before simulator tuning, because agents slid along walls, and 0.844 after. | **Rank transfer is a published metric and question.** The study is post hoc, with no selector and no regime variable. |
| 4 | **Truong et al.**, 2022, "Lower Fidelity Simulation Leads to Higher Sim2Real Transfer in Navigation" ([arXiv 2207.10821](https://arxiv.org/abs/2207.10821)) [V] | Extra fidelity did not help. Policies overfit simulator physics. | Contradicting evidence for any "higher fidelity is the truth" assumption. |
| 5 | **Li et al. (SIMPLER)**, 2024 ([arXiv 2405.05941](https://arxiv.org/abs/2405.05941)) [V abstract] | Reports strong correlation between simulated and real policy performance for manipulation. Names control and visual gaps as the main sources of mismatch. | The metric names (Mean Maximum Rank Violation (MMRV), Pearson) are **UNVERIFIED** from the abstract. |
| 6 | **Kennedy & O'Hagan**, Biometrika 87(1), 2000 ([DOI 10.1093/biomet/87.1.1](https://doi.org/10.1093/biomet/87.1.1)) [V] | AR1 multi-level GP: f_hi = ρ·f_lo + δ. | This is exactly our model f_L2 = ρ·f_L1 + δ. |
| 7 | **MF-GP-UCB**: Kandasamy et al., NeurIPS 2016 / JAIR 2019 ([arXiv 1603.06288](https://arxiv.org/abs/1603.06288)) [V]. **BOCA**: ICML 2017 ([PMLR v70](https://proceedings.mlr.press/v70/kandasamy17a/kandasamy17a.pdf)) [V] | Bandit multi-fidelity optimisation with regret guarantees. Finite (MF-GP-UCB) and continuous (BOCA) fidelity sets. | none |
| 8 | **MF-MES**: Takeno et al., ICML 2020 ([arXiv 1901.08275](https://arxiv.org/abs/1901.08275); [code](https://github.com/takeuchi-lab/MF-MES)) [V] | Information gain about max f per unit cost, with an asynchronous parallel variant. | **This is our planned acquisition function.** |
| 9 | **Hyperband**: Li et al., JMLR 18, 2018 ([arXiv 1603.06560](https://arxiv.org/abs/1603.06560)) [V]. **BOHB**: Falkner, Klein & Hutter, ICML 2018 ([arXiv 1807.01774](https://arxiv.org/abs/1807.01774)) [V] | Successive halving over budgets, with and without model-based proposals. | These are the baselines most likely to tie. |
| 10 | **Falsification across fidelities**: Shahrooei, Kochenderfer & Baheri, ECC 2023 ([arXiv 2212.14118](https://arxiv.org/abs/2212.14118); extended [2409.08097](https://arxiv.org/abs/2409.08097)) [V]. Koren, Nassar & Kochenderfer, IROS 2021, adaptive stress testing (AST) plus the backward algorithm, carrying low-fidelity failures to high fidelity ([arXiv 2107.12940](https://arxiv.org/abs/2107.12940)) [V]. Lee et al., JAIR 2020, AST ([arXiv 1811.02188](https://arxiv.org/abs/1811.02188)) [V]. Corso et al., JAIR 2021 survey ([arXiv 2005.02979](https://arxiv.org/abs/2005.02979)) [V]. Multi-fidelity RL safety validation ([arXiv 2203.03451](https://arxiv.org/pdf/2203.03451), 2022) [S, authors UNVERIFIED] | Our "failures found per compute-hour" objective is already published, including the multi-fidelity variant. | none |

## What's incremental

- **Kendall τ between NavPilot and .NET** is SRCC (Kadian 2020) with a different correlation statistic.
- **f_L2 = ρ·f_L1 + δ with cost-normalised information gain** is Kennedy–O'Hagan plus MF-MES.
- **A cost-aware selector that beats all-high-fidelity search** is the headline result of every MF-BO paper since 2016. Beating random or all-L2 is expected.
- **"Robotics-specific" multi-fidelity selection** was done by Marco 2017 and Cutler 2015.
- **Multi-fidelity failure discovery** was done by Shahrooei 2023 and Koren 2021.

## Remaining defensible delta

1. **Rank transfer conditioned on a physical regime variable, with a mechanistic predictor.** The question is *where* low fidelity stops preserving the ordering of navigation stacks: as a function of speed band, and of how timing or dynamics mismatch grows with speed. Kadian reports one SRCC per simulator configuration and does not model a regime variable. A testable statement:
   > rank correlation drops when v·Δτ_L1↔L2 exceeds a fraction of the obstacle spacing, where Δτ is the latency mismatch between fidelities.
2. **A prospective test on a four-level ladder (analytic → NavPilot → .NET → Unity) with measured costs**, run prospectively on the reduced recfeas sweep. Most MF-BO robotics papers have two fidelities and report retrospective results.
3. **An escalation rule as the deliverable**: run L2 only if the posterior probability of a rank reversal at L1 exceeds p. This is a decision rule a lab can adopt. As a research claim it is a regime-map plus decision-rule paper, not an algorithm paper.

**Rewritten claim.**
> For navigation-stack selection, cheap-simulator rankings are trustworthy in a predictable regime, set by speed × the latency/dynamics mismatch between simulators. A regime-gated escalation rule reaches the high-fidelity best for X% of all-high-fidelity compute. It matches tuned successive halving within the trustworthy regime and beats it where the cheap simulator inverts rankings.
>
> If the rule does not beat tuned successive halving there, the result is "use Hyperband over seeds", and that is the whole finding.

## Baselines a reviewer will demand

- Random search.
- All-L2.
- L1-only.
- Successive halving or Hyperband, with fidelity as the budget **and** with seeds as the budget.
- BOHB.
- An AR1 co-kriging selector.
- MF-GP-UCB.
- MF-MES (public code).
- Marco-style multi-source entropy search.
- For failures: random falsification at L2, AST at L2, and Koren's backward algorithm or MF-BO falsification (public code: [SAILRIT/MFBO_Falsification](https://github.com/SAILRIT/MFBO_Falsification)).

## Contradicting evidence and statistical threats

- Truong 2022: lower fidelity can transfer *better*. ".NET as ground truth" is itself a fidelity assumption, so state it.
- Kadian 2020: SRCC can be 0.18 before tuning, which shows rank transfer depends on simulator bugs, not on fidelity level alone.
- **Few configurations.** With about 10 configs, a 95% CI on Kendall τ spans about ±0.45 (rough estimate, UNVERIFIED for our data). H1's thresholds (≥ 0.6, then a drop) may not be statistically distinguishable. Bootstrap over configs and seeds.
- **Retrospective bias.** The cached configs were chosen by people who had already seen results. Only the prospective recfeas test counts as evidence.

## Falsifiable predictions

- **P1.** Kendall τ(L1, L2) ≥ 0.6 at ≤ 4 m/s and ≤ 0.3 at ≥ 6 m/s. The bootstrap CIs of the two bands do not overlap.
- **P2.** The speed at which τ drops is predicted within ±1 m/s by v·Δτ ≈ 0.25 × the mean obstacle spacing. Here Δτ is the measured latency mismatch from `fidelity.py`.
- **P3.** Within the ≤ 4 m/s band, tuned successive halving over seeds matches the MF selector's compute-to-best within 10%. The MF advantage appears only across the speed boundary.
- **P4.** L1-found failures replay as L2 failures at a rate ≥ 50% below 5 m/s and < 20% above 6 m/s.

## Kill condition

Two outcomes kill the selector claim:
- Rank correlation is < 0.3 in every band (the plan's condition). Then report it and keep L1 for failure discovery only.
- The MF selector does not beat tuned Hyperband/successive halving by ≥ 15% compute-to-best on the **prospective** recfeas test. In that case, ship successive halving as the queue's escalation rule and drop the selector.
