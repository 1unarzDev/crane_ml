# NOVELTY_D3: compute-aware high-speed navigation (joint speed and compute allocation under a measured latency tail)

Literature critic, 2026-11-10. Tags:
- **[V]**: primary abstract or record checked.
- **[S]**: snippet only.
- **UNVERIFIED**: not confirmed.

This brief does not repeat `Tools/NavPilot/NAV_LIT.md` (FASTER, RAPTOR, NanoMap, CBF/backup, Loquercio, Zhang 2025, CCNMPC).

**Verdict: narrowly novel. The analytic quantile governor is not novel. The joint (v, c) policy is novel only in its stochastic-tail and chance-constraint framing.**
- The compute → safe-velocity link is established: Falanga 2019; Krishnan F-1 2020/2022.
- Runtime adaptation of navigation compute to the environment exists: RoboRun 2021.
- Plugging τ_p99 into Falanga's envelope is a one-line extension.

## Closest prior work

| # | Work (year) | What it already does | What it leaves |
|---|---|---|---|
| 1 | **Falanga, Kim & Scaramuzza**, RA-L 4(2), 2019 ([DOI 10.1109/LRA.2019.2898117](https://doi.org/10.1109/LRA.2019.2898117); [pdf](https://rpg.ifi.uzh.ch/docs/RAL19_Falanga.pdf)) [V] | Max safe speed as a function of perception latency and sensing range. Linear second-order model, bounded input. | Latency is **deterministic**. Compute level is not a decision. |
| 2 | **Krishnan et al.**, IEEE CAL 19(1), 2020, "The Sky Is Not the Limit" (F-1 roofline) ([Google Research](https://research.google/pubs/pub49839)) [V]; **Krishnan et al.**, ISPASS 2022 ([arXiv 2204.10898](https://arxiv.org/abs/2204.10898)) [V] | A roofline that relates safe velocity to decision rate, sensor rate and body dynamics. Predicts the knee point with 5.1–9.5% error in flight tests. | A *design-time* model. Whether latency is stochastic is **UNVERIFIED**, probably mean/fixed. |
| 3 | **RoboRun**: Boroujerdian, Ghosal, Cruz, Plancher & Janapa Reddi, DAC 2021 ([arXiv 2108.13354](https://arxiv.org/abs/2108.13354)) [V abstract] | A runtime governor dynamically adjusts navigation-pipeline compute knobs to the spatial features of the environment. Reports 4.5× mission time, 4× energy, −36% CPU against a static worst-case design. | **The closest work.** Whether velocity is a co-adapted knob, and whether there is any collision-risk constraint, is **UNVERIFIED**. Read the full text first. No latency *tail* model is evident. |
| 4 | **MAVBench**: Boroujerdian et al., MICRO 2018 ([arXiv 1905.06388](https://arxiv.org/abs/1905.06388)) [V] | A closed-loop benchmark of compute versus mission time and energy. | Characterisation only. No policy. |
| 5 | **SUPER**: Ren et al., Sci. Robot. 10(98), 2025 ([DOI 10.1126/scirobotics.ado6187](https://doi.org/10.1126/scirobotics.ado6187)) [V record] | An exploratory trajectory plus a known-free backup trajectory every cycle, at more than 20 m/s. | A committed backup **makes compute stalls survivable by construction**. This is the main threat to H2. |
| 6 | **Sung, Kaelbling & Lozano-Pérez**, IROS 2021, "Learning When to Quit" ([arXiv 2103.04374](https://arxiv.org/abs/2103.04374)) [V] | Learned metareasoning for anytime planners (RRT*, PRM*): when to stop deliberating. | Static tasks. No speed or collision coupling. |
| 7 | **ARA\***: Likhachev, Gordon & Thrun, NeurIPS 2003 ([pdf](https://proceedings.neurips.cc/paper/2003/file/ee8fe9093fbbb687bef15a38facc44d2-Paper.pdf)) [V] | Anytime search with a bound that is tuned to the available time. | The standard anytime-planner baseline. |
| 8 | **Perception Simplex for landing**: Bansal et al., AIAA SciTech 2024 ([arXiv 2312.02937](https://arxiv.org/abs/2312.02937)) [V abstract] | A stopping-distance safety envelope that includes compute latency, with measured control limits. | Per the snippet, latency is a fixed L. The envelope with latency is [S]. |
| 9 | **Latency-to-speed heuristics**: edge-assisted flight, speed ∝ map update rate ([arXiv 2012.05517](https://arxiv.org/abs/2012.05517), 2020) [S]; adaptive speed scaling under VLM inference delay ([arXiv 2609.26084](https://arxiv.org/html/2609.26084), 2026) [S] | Speed is cut when processing is slow. | Heuristic. No chance constraint, no tail. |
| 10 | **TAPAS**: Vyas et al., CODES 2026 ([arXiv 2607.17317](https://arxiv.org/abs/2607.17317)) [V] | RL picks the perception FPS and model-to-core mapping by scene complexity on a Jetson Orin. | Adapts compute, not vehicle speed. |

Latency-aware MPC: Carlos et al., ICARCV 2020, delay-compensated real-time NMPC on a Crazyflie ([arXiv 2010.11264](https://arxiv.org/abs/2010.11264)) [V]. Whether its delay model is constant or stochastic is **UNVERIFIED**.

## What's incremental

- **The p99 governor.** Falanga's envelope evaluated at a latency quantile. A reviewer will call it a corollary. It is a baseline, not a contribution.
- **"Adapt compute to the environment."** RoboRun did this, and TAPAS did it for perception FPS.
- **"Compute limits safe velocity."** F-1/roofline (2020) and MAVBench (2018) established this.
- **A small learned policy for (v, c).** On its own it is RL engineering, unless it beats the analytic joint optimiser.

## Remaining defensible delta

1. **Coupled latency and quality with a stochastic tail.** Raising compute level c both *raises* latency τ ~ F_c and *improves* free-space quality q(c), which is detection range and thin-branch recall. Together these give an **interior optimum c\*(v)** under the chance constraint P(d_free(q(c)) < vτ + v²/2a + m) ≤ δ. We found no paper with a stochastic latency tail, compute-dependent detection quality and a risk constraint all at once (NAV_LIT gap; search above).
2. **Max safe speed at a fixed risk level, with CIs, under controlled degradation.** The NAV_LIT gap still stands: no paper reports speed at a fixed risk level with intervals.
3. **H1 as a counterfactual decomposition.** Perfect-latency versus perfect-perception replays attribute failures. This is a clean diagnostic contribution even if H2 fails.

**Rewritten claim.**
> Under a measured, heavy-tailed compute latency, the maximum safe speed at a fixed collision risk is set jointly by the latency quantile and a compute-dependent detection range, and has an interior optimal compute level. A per-cycle governor that optimises both beats the best speed-only governor by Δ m/s, *but only for planners without a committed backup trajectory*. With a SUPER/FASTER-style backup, the gain shrinks to Δ_b.
>
> Report both Δ and Δ_b. Claiming Δ alone invites the SUPER rebuttal.

## Baselines a reviewer will demand

- Fixed speed at 3, 5 and 7 m/s.
- Falanga envelope at the mean latency and at τ_p99 (the analytic governor).
- The existing lag/p99 safety layer.
- A RoboRun-style environment-driven compute governor with a speed-only cap.
- Recursive feasibility.
- A known-free backup trajectory (SUPER/FASTER structure), with and without the joint governor.
- Anytime replanning (ARA\*-style) with a deadline.
- Metareasoning stop (Sung 2021) if the planner is anytime.
- A fixed-c oracle sweep that is c-tuned per speed. This is the "is adaptivity needed at all?" control.

## Contradicting evidence

- **NAV_LIT P1** predicts that safe speed scales with the thin-branch 50%-detection range, with map trust shifting it by less than 10%. If so, H1 ("perfect latency removes most collisions") will **fail**: perception range, not latency, dominates. The plan's H1 and NAV_LIT P1 cannot both hold, so decide this first with the stage-1 counterfactual replays.
- Loquercio 2021: planners tie at ≤ 3 m/s.
- SUPER: the backup structure absorbs stalls.

## Falsifiable predictions

- **P1.** In the cf_fastperc replays at 7 m/s, perfect latency removes < 50% of collisions and perfect perception range removes > 50%. This would favour NAV_LIT P1 and refute plan H1.
- **P2.** With a committed backup trajectory, the gain of the joint governor over the τ_p99 governor is < 0.3 m/s. Without the backup it is ≥ 1 m/s.
- **P3.** The best fixed c\*(v) recovers ≥ 80% of the adaptive policy's gain. If so, per-cycle adaptivity is unnecessary and a lookup table suffices.
- **P4.** The τ_p99 governor's max safe speed is within 0.3 m/s of the governor that uses the full F_c. A larger gap would show that the tail shape, not just the quantile, matters.

## Kill condition

Kill H2 under either condition:
- the adaptive gain over the p99 governor is < 0.5 m/s (as in the plan), **or**
- the gain is < 0.5 m/s once a known-free backup trajectory is present.

Then publish H1/P1 (the failure decomposition plus speed at fixed risk with CIs) as the result.
