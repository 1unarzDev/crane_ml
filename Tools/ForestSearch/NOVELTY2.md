# Novelty check 2 (2026-11). Claims A and B

Method: search metadata plus abstract pages. Full texts mostly not read (Boutilier PDF unreadable via fetch; MIT ACBBA fetch got HTTP 429). Items marked [unverified] come from snippets or memory. Earlier notes (Nayyar, Roth, Wu, Spaan, etc.) not repeated.

## Claim A. Belief convergence plus stale default-plan prediction of the partner causes same-step action collisions

Closest works:
1. Boutilier (1999), "Sequential Optimality and Coordination in Multiagent Systems", IJCAI-99. https://www.cs.toronto.edu/~cebly/Papers/seqCoord.pdf. Even with an identical joint policy, independent deciders need a coordination mechanism; lexicographic (priority-order) conventions give immediate coordination. This is your fix, in general form. Source: search snippet only [unverified].
2. Johnson, Ponda, Choi, How (2011), "Asynchronous Decentralized Task Allocation for Dynamic Environments", AIAA Infotech@Aerospace. https://dspace.mit.edu/bitstream/handle/1721.1/81434/How_Asynchronous%20decentralized.pdf. ACBBA uses timestamped winning bids and conflict-resolution rules so stale information does not yield lasting duplicate assignment. Duplicate or conflicting assignment under asynchrony/delay is a known and handled problem. Details [unverified]. A companion paper on lossy networks exists (Ponda, Johnson, Kopeikin, How; title only) [unverified].
3. Choi, Brunet, How (2009), "Consensus-Based Decentralized Auctions for Robust Task Allocation", IEEE T-RO 25(4):912-926. CBBA is conflict-free despite inconsistent situational awareness (snippet) [unverified details].
4. 2026 arXiv "Randomness is sometimes necessary for coordination", arXiv:2605.06825. Under symmetric observations, a shared deterministic policy gives identical actions; a random rank ordering breaks the tie. Snippet only [unverified].

Related, not equivalent: Bramblett et al. 2025 (arXiv:2501.03907) on higher-order reasoning about teammates' beliefs for implicit coordination. It targets first-order prediction errors, but I found no collision analysis.

Verdict: PARTIALLY KNOWN. Symmetric agents choosing identical actions, and priority-order conventions as the cure, are KNOWN (Boutilier 1999). Duplicate assignment from stale information is KNOWN in task allocation (CBBA/ACBBA). I found no source describing the specific interaction: messages align beliefs, while a plan-based partner forecast stays stale after a deviation, so the aligned beliefs cause lockstep collisions. That mechanism appears OPEN, but absence in a 25-minute search is weak evidence.

Narrowest defensible statement: "communication that equalizes beliefs removes the natural asymmetry that kept agents apart, and an inconsistent partner model (message-updated state, plan-predicted action) then yields identical choices; sequential prediction under a priority order restores a consistent joint model."

Contradicting evidence: it may be a special case of Boutilier's "same policy, no convention" failure. Any equal-information symmetric system would collide, so messages are incidental. Control: run silent agents with identical beliefs and the stale forecast, and a randomized tie-break fix.

Falsifiable predictions: (a) collisions rise with delay up to about one deviation interval, then fall; (b) zero delay gives about 0 collisions; (c) random tie-breaking removes most collisions but costs more than priority-order prediction; (d) feeding the partner's true current action (oracle) matches the sequential fix.

## Claim B. Communication value depends on the planner

Closest works:
1. Bramblett, Reasoner, Bezzo (2025), arXiv:2501.03907, "Implicit Coordination using Active Epistemic Inference for Multi-Robot Systems". Verified abstract: coordination without communication through reasoning about teammates; beats greedy and first-order baselines. No communicating baseline in the abstract. Supports planner-dependence (reasoning depth matters).
2. Becker, Carlin, Lesser, Zilberstein (2009), Comput. Intell. 25(1) (from NOVELTY.md). Communication value is analysed by policy and domain.
3. Boldrer, Kamler, Ahmad, Saska (2026), arXiv:2603.08379, "Perception-Aware Communication-Free Multi-UAV Coordination in the Wild". Verified: communication-free coordination in forests. No comparison on the abstract page.
4. Survey/context: "Towards a communication free coordination for multi-robot exploration" (ResearchGate, ~2012) [unverified authors/venue]: path-distance frontier assignment needs no messages.

Contradicting evidence: arXiv:2602.12074 (snippet) says limited, infrequent communication improves coverage over disconnected teams. That favors communication regardless of planner. The common-information tradition (Nayyar 2013) predicts that value falls as shared knowledge rises.

Verdict: PARTIALLY KNOWN. That implicit coordination can match or approach communicating teams, and that value depends on the policy, is established qualitatively. I did not find a within-simulation demonstration that a stronger partner-forecasting planner increases the apparent value of communication (+0.04) while the best silent myopic system ties the best communicating one. Treat as an empirical finding, not a law.

Falsifiable predictions: (a) the +0.04 gap vanishes if the forecasting planner receives the correct partner model under silence; (b) the myopic-silent versus communicating tie holds across seeds, with the CI of the difference containing 0; (c) a stronger silent planner narrows the gap monotonically.

## BibTeX to add (fields from search metadata unless noted)

```bibtex
@inproceedings{boutilier1999sequential,
  author = {Boutilier, Craig},
  title = {Sequential Optimality and Coordination in Multiagent Systems},
  booktitle = {Proc. IJCAI},
  year = {1999}, % pages unverified
  url = {https://www.cs.toronto.edu/~cebly/Papers/seqCoord.pdf}
} % [unverified: PDF unreadable; from snippet]

@article{choi2009cbba,
  author = {Choi, Han-Lim and Brunet, Luc and How, Jonathan P.},
  title = {Consensus-Based Decentralized Auctions for Robust Task Allocation},
  journal = {IEEE Transactions on Robotics},
  volume = {25}, number = {4}, pages = {912--926}, year = {2009}
} % [unverified: from snippet; number inferred]

@misc{bramblett2025implicit,
  author = {Bramblett, Lauren and Reasoner, Jonathan and Bezzo, Nicola},
  title = {Implicit Coordination using Active Epistemic Inference for Multi-Robot Systems},
  year = {2025}, eprint = {2501.03907}, archivePrefix = {arXiv}
} % verified against arxiv.org/abs/2501.03907
```
