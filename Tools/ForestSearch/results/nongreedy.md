# Non-greedy centralised reference vs greedy silence (EXPLORATORY, post-preregistration)

This file answers senior re-review weakness 1 and requested change 2: does the null depend on greedy planning? Everything here is exploratory and was run after the preregistration.

## Planner: multiagent rollout with belief-sampled futures (`lookahead.py`)

- **Decision step.** When an agent is idle at time t, it scores its top-K greedy candidate looks (K = 6) by simulating both agents forward.
- **Joint actions.** If the partner is also idle at t (it decides next), the planner also scores joint first actions: K candidates for me × the partner's top 3 (`pairs=3`). My action is scored by its best joint continuation.
- **Continuation.** After the first action, both agents follow the paper's centralised greedy rule, P_detect/(travel+1), with in-flight looks as virtual NULLs. This is Bertsekas' one-agent-at-a-time multiagent rollout with greedy as the base policy. With exact Q-values it cannot do worse than greedy.
- **Futures.** Unlike `joint.py` (all-NULL futures), HINT, NULL and ABORT outcomes are sampled from the agent's Bayesian predictive distribution. N = 16 futures per candidate, with common random numbers.
- **Score.** DETECT is handled by survival weighting. The score is E[Σ_j S_j·pdet_j], an unbiased estimate of P(detect within the window), with window = 16 time units. Looks that finish at the same time are credited with the exact P(either detects) (Rao-Blackwellised).
- **Noise guard.** The greedy action is kept unless the best alternative's paired mean gain exceeds 1 standard error (z = 1).
- **Determinism.** The RNG is seeded from a hash of the decision inputs, so a decision is a deterministic function of the information state.
  - `nominal_plan` with this planner is reproducible offline.
  - Proposition 1 holds: with no private events, silent IMPLICIT-lookahead replays ORACLE-lookahead exactly (tested).
- **Information use.** ORACLE uses everything. IMPLICIT uses its own records plus the lookahead nominal plan (the partner's believed position and in-flight look come from the plan). INDEP runs a single-agent rollout.
- **Configuration.** `make_lookahead_strategies(k=6, n=16, window=16, pairs=3, z=1.0)`. It was sized only for compute; there was no tuning on held-out seeds. A 1-seed dev check on seed 100 was cut short by compute.

## Validation against the exact DP (travel-free, synchronous, abort-free; `exact.ExactTeam`)

The planner's policy is evaluated *exactly*: the outcome tree is enumerated with `lookahead.exact_value_of_planner`, using all n·V candidates, `pairs=6`, N = 512 and z = 1. It is compared with the optimum and with centralised greedy. Instances are 12 random tiny instances (n = 3, V = 2, hint rates of events `both`, prior ~ Dirichlet(1), g ~ U(0.1, 0.95), q ~ U(0.4, 0.95), RNG 2026), at k = 2 and k = 3 steps.

| inst | k | optimal | greedy | lookahead | opt−greedy | opt−lookahead |
|---|---|---|---|---|---|---|
| r0 | 2 | 0.7670 | 0.7622 | 0.7647 | 0.0048 | 0.0022 |
| r0 | 3 | 0.8265 | 0.8233 | 0.8262 | 0.0032 | 0.0003 |
| r1 | 2 | 0.4443 | 0.4442 | 0.4443 | 0.0000 | 0.0000 |
| r1 | 3 | 0.5263 | 0.5245 | 0.5254 | 0.0017 | 0.0009 |
| r2 | 2 | 0.5612 | 0.5560 | 0.5612 | 0.0052 | 0.0000 |
| r2 | 3 | 0.6906 | 0.6875 | 0.6890 | 0.0031 | 0.0016 |
| r3 | 2 | 0.5811 | 0.5811 | 0.5811 | 0.0000 | 0.0000 |
| r3 | 3 | 0.6804 | 0.6804 | 0.6804 | 0.0000 | 0.0000 |
| r4 | 2 | 0.7099 | 0.6960 | 0.7099 | 0.0139 | 0.0000 |
| r4 | 3 | 0.8104 | 0.8092 | 0.8095 | 0.0013 | 0.0009 |
| r5 | 2 | 0.3685 | 0.3682 | 0.3685 | 0.0003 | 0.0000 |
| r5 | 3 | 0.4289 | 0.4275 | 0.4289 | 0.0014 | 0.0001 |
| r6 | 2 | 0.7415 | 0.7400 | 0.7415 | 0.0015 | 0.0000 |
| r6 | 3 | 0.8216 | 0.8213 | 0.8216 | 0.0003 | 0.0000 |
| r7 | 2 | 0.5410 | 0.5410 | 0.5410 | 0.0000 | 0.0000 |
| r7 | 3 | 0.6179 | 0.6164 | 0.6168 | 0.0014 | 0.0010 |
| r8 | 2 | 0.5638 | 0.5607 | 0.5638 | 0.0031 | 0.0000 |
| r8 | 3 | 0.6287 | 0.6284 | 0.6287 | 0.0003 | 0.0000 |
| r9 | 2 | 0.3706 | 0.3586 | 0.3706 | 0.0119 | 0.0000 |
| r9 | 3 | 0.4442 | 0.4378 | 0.4397 | 0.0065 | 0.0046 |
| r10 | 2 | 0.7073 | 0.7034 | 0.7073 | 0.0039 | 0.0000 |
| r10 | 3 | 0.7734 | 0.7722 | 0.7725 | 0.0012 | 0.0009 |
| r11 | 2 | 0.7549 | 0.7549 | 0.7549 | 0.0000 | 0.0000 |
| r11 | 3 | 0.8305 | 0.8296 | 0.8304 | 0.0009 | 0.0001 |

**Summary.** Mean gap to the optimum is 0.0027 for greedy and 0.0005 for the lookahead planner.
- At k = 2 the planner is exactly optimal in 11 of 12 instances, closing greedy gaps of up to 0.014. The exception is r0, at 0.0022.
- At k = 3, 11 of 12 instances are within 0.0016 of the optimum. The exception is r9, at 0.0046 (greedy gap 0.0065 there).
- Overall, 22 of 24 cases are within 0.002 and the worst is 0.0046.
- The residual is the known limit of one-step rollout plus the z = 1 guard. Without the guard (z = 0), 23 of 24 are within 0.002, with worst 0.0039 (`/tmp/fs_nongreedy/exact_check.md`).
- With z = 1, the planner is never below greedy on these 24 cases.

Tests (`test_lookahead.py`, 14 tests, all pass, about 65 s):
- exact-DP agreement within 0.002 on three tiny instances × k ∈ {1, 2, 3}, including two "hard" instances where greedy is ≥ 0.008 suboptimal;
- exact optimum at k = 2 on the hard instances;
- determinism of `rollout_decide`, of `nominal_plan` and of whole episodes (ORACLE, IMPLICIT, INDEP);
- Proposition 1 replay: IMPLICIT-lookahead equals ORACLE-lookahead episode by episode, with 0 bytes, when there are no private events.

## Held-out comparison (seeds 200–207, travel 1, ideal link `d0_loss0`, horizon 2n)

Instances are `grid_instance(g, g, seed, travel_scale=1, horizon=2g², **event_curve.events(λ))`, the same construction as the confirmatory cells.
- Greedy arms run 300 episodes per seed. Lookahead arms run the first 100 episodes per seed (compute).
- Every lookahead contrast is paired on those 100 common episodes. The greedy ORACLE − IMPLICIT column uses all 300 episodes; it reproduces `hypotheses.md` H1.
- CIs are paired cluster-bootstrap 95% intervals (instances, then episodes; 2000 resamples).
- IMPLICIT-lookahead uses the nominal plan computed with the lookahead planner. It ran for λ = 1 only; λ = 3 was cut by the time cap.

| cell | seeds | eps/seed (lookahead) | greedy ORACLE | greedy IMPLICIT | lookahead ORACLE | lookahead IMPLICIT | greedy INDEP |
|---|---|---|---|---|---|---|---|
| 6x6 travel 1, λ=1 | 200-207 (8) | 100 | 0.789 | 0.796 | 0.791 | 0.770 | 0.729 |
| 6x6 travel 1, λ=3 | 200-207 (8) | 100 | 0.815 | 0.816 | 0.829 | – | 0.730 |
| 5x5 travel 1, λ=1 | 200-207 (8) | 100 | 0.770 | 0.756 | 0.807 | 0.766 | 0.720 |
| 5x5 travel 1, λ=3 | 200-207 (8) | 100 | 0.795 | 0.799 | 0.806 | – | 0.731 |

| cell | lookahead ORACLE − greedy IMPLICIT | lookahead ORACLE − greedy ORACLE | greedy ORACLE − greedy IMPLICIT (300 eps) | lookahead IMPLICIT − greedy IMPLICIT | lookahead ORACLE − lookahead IMPLICIT |
|---|---|---|---|---|---|
| 6x6 travel 1, λ=1 | -0.005 [-0.031,+0.021] | +0.003 [-0.028,+0.030] | -0.006 [-0.018,+0.006] | -0.026 [-0.056,+0.005] | +0.021 [-0.004,+0.046] |
| 6x6 travel 1, λ=3 | +0.013 [-0.010,+0.036] | +0.014 [-0.005,+0.033] | -0.003 [-0.015,+0.010] | – | – |
| 5x5 travel 1, λ=1 | +0.051 [+0.024,+0.079] | +0.037 [+0.006,+0.069] | +0.000 [-0.010,+0.011] | +0.010 [-0.024,+0.041] | +0.041 [+0.001,+0.083] |
| 5x5 travel 1, λ=3 | +0.007 [-0.016,+0.034] | +0.011 [-0.013,+0.036] | -0.005 [-0.018,+0.010] | – | – |

Pooled over the cells (exploratory; clusters = cell × λ × seed):
- lookahead ORACLE − greedy ORACLE: **+0.016 [+0.002,+0.030]** (32 clusters). A stronger centralised team exists, but its gain is small.
- lookahead ORACLE − greedy IMPLICIT: **+0.017 [+0.002,+0.033]** (32 clusters).
- lookahead IMPLICIT − greedy IMPLICIT: −0.008 [−0.032,+0.015] (λ = 1, 16 clusters).
- lookahead ORACLE − lookahead IMPLICIT: **+0.031 [+0.006,+0.057]** (λ = 1, 16 clusters).

Two-sided bootstrap p-values, with Holm correction over the 4 cells (`/tmp/fs_nongreedy/scripts/pvals.py`):
- lookahead ORACLE − greedy IMPLICIT: 5x5 λ=1 p = 0.0015 (Holm 0.006); the other three cells p ≥ 0.34.
- lookahead ORACLE − greedy ORACLE: 5x5 λ=1 p = 0.016 (Holm 0.062); 6x6 λ=3 p = 0.18; the others p ≥ 0.39.

## Compute per episode

Wall seconds per episode, single process, nice 19, on a shared host with load average about 45 on 20 cores. Expect a 2–3× speed-up on an idle machine.

| cell | greedy (any strategy) | lookahead ORACLE | lookahead IMPLICIT | lookahead nominal plan (once per instance) |
|---|---|---|---|---|
| 5x5 travel 1 | 0.004 s | 0.55 s (median 0.39) | 0.65 s (median 0.52) | 0.5–2 s |
| 6x6 travel 1 | 0.008 s | 1.24 s (median 0.84) | 1.06 s (median 0.79) | 1.5–4 s |

The lookahead costs about 150× greedy. By comparison, the joint planner (`make_joint_strategies(6,14)`) took about 0.3–0.6 s per episode under the same load. The held-out lookahead runs took about 1.2 h of single-process wall time.

## Answers

1. **Does a stronger centralised team beat greedy silence?** Slightly, and only in one cell.
   - Pooled, lookahead ORACLE beats greedy IMPLICIT by +0.017 [+0.002,+0.033].
   - The gain is concentrated in 5x5 travel 1, λ=1: +0.051 [+0.024,+0.079], Holm-significant over the 4 cells. That cell exceeds the paper's 0.03 margin on the point estimate, although the CI includes 0.03.
   - The other three cells are −0.005 to +0.013, with CIs inside about ±0.04.
   - So the answer to the reviewer's question ("with a lookahead Oracle, does Oracle − Implicit stay below 0.03?") is: in 3 of 4 cells yes; in 1 of 4 the point estimate is +0.05.
   - Most of the gap is planner quality, not communication. Lookahead ORACLE − greedy ORACLE is +0.016 pooled and +0.037 in that cell.
2. **Does silence with the stronger planner keep up?** No. The nominal-plan brittleness seen with the joint planner reappears.
   - Lookahead IMPLICIT is no better than greedy IMPLICIT: +0.010 in 5x5 and −0.026 in 6x6 (λ=1).
   - Lookahead IMPLICIT falls behind lookahead ORACLE by +0.041 [+0.001,+0.083] (5x5) and +0.021 [−0.004,+0.046] (6x6); pooled +0.031 [+0.006,+0.057].
   - The joint planner showed the same pattern in the same 5x5 travel 1 cell (ORACLE − IMPLICIT +0.039 [+0.017,+0.061], `joint_heldout_all8.md`).
   - So with a non-greedy planner, silence stops matching the centralised team that uses the same planner. Proposition 1 still holds without private events; the loss comes from private events breaking the replayed plan.
   - A non-greedy plan is more fragile to off-plan events than the greedy index rule. The greedy rule is close to a static priority order, so a partner who has deviated still looks nearly on-plan.
3. **Is the best silent team still as good as the best communicating team?** Not quite.
   - The best silent team found is still greedy IMPLICIT; lookahead IMPLICIT does not improve on it.
   - The best communicating team, lookahead ORACLE, is ahead by +0.017 pooled (CI excludes 0) and +0.051 in one cell.

## Recommendation on the title

"Greedy" is essential to the healthy-team claim as stated. The null ("communication buys nothing detectable for a healthy team") is a statement about greedy teams. With a stronger centralised planner:
- a small but non-zero value of communication appears (+1.7 pp pooled, up to +5 pp in one cell);
- silent agents cannot capture it by adopting the stronger planner, because the shared nominal plan becomes brittle.

Recommended framing:
- Keep "greedy" in the main title, not only the subtitle. Example: "[title withheld]".
- Add one sentence to the abstract and the discussion: with a rollout (non-myopic) centralised planner, communication buys about 2 pp (up to 5 pp), and silent replay of a non-greedy plan loses about 3 pp. The null is therefore a property of greedy/index planning, which happens to be both near-optimal here (exact DP gap ≤ 0.014 on tiny instances) and robust to replay.
- A positive reading is available: in this domain, greedy is the planner under which silence is free, and the price of a non-greedy team is that it must talk.

Caveats:
- Exploratory and post-preregistration.
- 100 lookahead episodes per seed. IMPLICIT-lookahead was run for λ = 1 only.
- One planner configuration, sized for compute rather than tuned. A larger rollout budget might widen the ORACLE gap.
- Travel-0 cells and lossy links were not run with the lookahead.

## Provenance

- Code:
  - `Tools/ForestSearch/lookahead.py` (new; patches `Episode._choose` to dispatch on `strategy.lookahead`; non-lookahead strategies run unchanged code);
  - `Tools/ForestSearch/test_lookahead.py`.
- Drivers (outside the repo, in `/tmp/fs_nongreedy/scripts/`):
  - `ng_worker.py` (arguments `OUT 100 6 16 16 3 1.0`);
  - `agg_ng.py` (bootstrap RNG 12345) and `pvals.py` (RNG 777);
  - `exact_check.py` (RNG 2026).
- Raw per-episode CSVs: `/tmp/fs_nongreedy/ng/out/*.csv` (48 files: part A = greedy ×3 + lookahead ORACLE; part B = lookahead IMPLICIT). These are not archived in `results/data/`; copy them there if this table goes into the paper.
