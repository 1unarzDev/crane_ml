# ForestSearch: Pilot 1 (two-searcher coordination under lossy links)

Grid/graph study only. **No forest, flight, speed or radio claims.**
Plan: `~/.claude/plans/steady-yawning-crayon.md`.

## Model (model.py)
- A single static person in one of n cells. Two view classes per cell.
- Latent per-(cell, view) visibility is drawn once per episode, so repeated same-view looks are correlated.
- Outcomes: DETECT (confirmed; ends the mission), HINT (weak private evidence, with false hints at rate alpha), NULL.
- ABORT is a private look failure.
- The belief is exact, from per-(cell, view) NULL/HINT counts.

## Protocols (sim.py)
| Name | What it does |
|---|---|
| ORACLE | Centralized, using the same base planner. |
| INDEP | Ignores the partner. |
| SECTOR | Static split of cells between the two agents. |
| IMPLICIT | Predicts the common-knowledge nominal plan; sends no messages. |
| RESERVE | Sends leases only. |
| FULL | Sends records plus leases, windowed store-and-forward. |
| EVENT | Stays silent until a private event, then switches to FULL. |

Links (links.py) are Gilbert–Elliott per hop. Base relaying is modeled as two hops.

## Verified (test_forest_search.py, 17 tests)
- **Proposition 1 (tested, not yet proved formally).** With no private events and common-knowledge deterministic planners, IMPLICIT and EVENT reproduce the centralized schedule exactly while sending 0 bytes, for *any* link. Messages only have value through private events. This also holds under the rollout planner (checked ad hoc).
- With an ideal link and an unbounded window, FULL equals the centralized policy even when private events occur.

## Early results (exact.md, game.md, sweep_summary.md)
1. **Travel-free tiny instances (n=3, V=2, k≤4).** Exact DP shows that centralized sequential greedy is within 0.001 of optimal. A planner that assumes independent views loses **3.5–12 points** of P(detect). This supports the correlated-view lead (A).
2. **The confound that blocks claims about communication.**
   - With travel, the myopic greedy base planner lets SECTOR beat the centralized policy.
   - Under greedy, IMPLICIT, which sends no messages, is the team optimum in the protocol game even when messages are free.
   - Under a rollout planner (k=6, budget 8), the ranking flips: IMPLICIT drops from 0.683 to 0.630 and FULL rises to 0.683.
   - So the protocol rankings depend on the planner. Any "value of communication" claim must be shown to hold across a family of base planners, including a non-myopic joint planner, before it goes into a paper.
3. **Protocol game.** Payoffs are common, so every pure Nash equilibrium is person-by-person optimal; this is a team problem, not a price-of-anarchy setting. With a message cost of 0.05 per kB, there is a second equilibrium (RESERVE, RESERVE) worth 0.654 against an optimum of 0.720. Mismatched protocol pairs are worse than either matched pair.

4. **Sweep** (greedy planner; 2 grids × 3 instances × 400 paired episodes; `sweep_summary.md`).
   - With any lossy link, explicit protocols (RESERVE, FULL) fall **below** IMPLICIT, which sends 0 bytes. On 5×5 with hints and abort events at 30% bursty loss, the ordering is FULL 0.751 < EVENT 0.757 < IMPLICIT 0.779 ≈ C-GREEDY 0.781.
   - Base relaying (two hops) makes this worse.
   - EVENT spends 30–90% of FULL's bytes and keeps most of IMPLICIT's robustness.
   - Hypothesis (not yet tested): messages should be *deltas against a shared default plan*. Protocols that replace the default plan with explicit state degrade under loss. This holds **only under the greedy planner** until result 2 is resolved.

## Next
- Add a joint (two-agent) rollout or MCTS reference with travel, and check that it dominates SECTOR. Then repeat the sweep across planners and report how stable the protocol rankings are.
- Run exact DP with private events (abort/hint) on travel-free tiny instances, to compute value-of-message bounds exactly.
- Add the counterexample search for stale leases (RESERVE under bursty loss with aborts).
