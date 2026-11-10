# Novelty assessment (2026-11)

Method note: citations were confirmed via search-result metadata and abstracts only. I did not open full texts. Anything marked [unverified] is from memory or inferred.

## C1. With common-knowledge deterministic plans, negative evidence needs no messages; messages only carry value via private events

Closest works:
- Nayyar, Mahajan, Teneketzis (2013), "Decentralized Stochastic Control with Partial History Sharing: A Common Information Approach", IEEE TAC 58(7):1644-1658. https://arxiv.org/pdf/1209.1695. Shows that beliefs conditioned on common information are a sufficient statistic, so anything common knowledge needs no transmission.
- Roth, Simmons, Veloso (2005), "Reasoning About Joint Beliefs for Execution-Time Communication Decisions", AAMAS, pp. 786-793. https://www.ri.cmu.edu/publications/reasoning-about-joint-beliefs-for-execution-time-communication-decisions/. Agents track the distribution over joint beliefs and communicate only when needed. See also "What to Communicate?" (DARS 2006), https://www.cs.cmu.edu/~reids/papers/dars06.pdf.
- Wu, Zilberstein, Chen (2009/2011), "Multi-Agent Online Planning with Communication", ICAPS 2009, https://ojs.aaai.org/index.php/ICAPS/article/download/13366/13214; AIJ 175(2):487-511 (2011). Communicates only on detected history inconsistency.
- Unhelkar & Shah (2016), "ConTaCT", AAAI, https://ojs.aaai.org/index.php/AAAI/article/view/10123. Deterministic domains, decentralized decision to communicate.
- Becker, Carlin, Lesser, Zilberstein (2009), "Analyzing Myopic Approaches for Multi-Agent Communication", Comput. Intell. 25(1), https://doi.org/10.1111/j.1467-8640.2008.01329.x. Value-of-communication analysis.

What they already show: the general principle (communicate only what is not inferable from common information; message value comes from information the receiver cannot predict).

Verdict: KNOWN as a principle. PARTIALLY KNOWN for the search instance.
Narrowest defensible statement: in a search task the default "nothing found" outcome is inferable from the plan, so a no-message protocol (IMPLICIT) is exactly as good as centralized when no private events occur. We quantify the loss from each private event type (hint, aborted look). Do not claim the principle itself.
Caveat (contradicting): the claim holds only if every searcher's look outcome is common knowledge. Aborted looks and private hints break that, and delay or loss breaks common knowledge of execution state. This is the lossy-link regime you study, so state the assumption explicitly.
Falsifiable prediction: with zero hints and aborts, IMPLICIT equals CENTRALIZED within Monte Carlo error for every loss rate. Any gap means a modelling leak.

## C2. Treating repeated looks as independent under correlated visibility loses a lot of detection probability

Closest works:
- Velez, Hemann, Huang, Posner, Roy (2011), "Planning to Perceive", ICAPS, pp. 266-273; extended in "Modelling Observation Correlations for Active Exploration and Robust Object Detection", JAIR 44:423-453 (2012), https://arxiv.org/pdf/1401.4612. Models observations as correlated through a latent environment variable. Repeat views from the same viewpoint add little information, which is exactly your same-view correlation.
- Koopman (1956), "The Theory of Search II: Target Detection", Operations Research 4(5), https://pubsonline.informs.org/doi/10.1287/opre.4.5.503. The independent-glimpse assumption is explicit, and Koopman warns it fails outside the visual case.
- Stone (1975/2007), Theory of Optimal Search; Washburn, Search and Detection. Standard detection functions are mostly exponential or independent. Washburn's treatment of dependent looks is [unverified].
- Zheng et al. (2021/2023), https://arxiv.org/pdf/2110.09991. Object-search POMDPs assume conditional independence of observations.

What they already show: independence is a known simplification; correlated observations reduce the value of same-view repeats; planners that model this diversify views.

Verdict: PARTIALLY KNOWN. The qualitative effect is known. The "lose a lot" claim is parameter-dependent and risks looking obvious.
Narrowest statement: a measured loss curve, as a function of correlation and view-class count, for multi-searcher graph search with a shared per-(cell, view) latent, and its interaction with communication protocol.
Contradicting evidence: with only 2 view classes, a heuristic that alternates views gets most of the benefit without modelling correlation, so the gap may be small for cheap fixes. Test this.
Falsifiable prediction: the loss grows monotonically with correlation and vanishes at zero correlation. A "view-alternation" heuristic recovers over 80% of the gap.

## C3. Under lossy links, deltas against a shared default plan beat explicit-state-replacement protocols (leases, full sharing)

Closest works:
- Gray & Cheriton (1989), "Leases", SOSP, pp. 202-210, https://dl.acm.org/doi/10.1145/74850.74870. Leases tolerate message loss by expiry. This undercuts your claim: a correct lease falls back safely.
- Almeida, Shoker, Baquero (2018), "Delta state replicated data types", JPDC 111:162-173, https://arxiv.org/abs/1603.01529. Deltas versus full state. Concerns bandwidth and convergence, not planning.
- Spaan, Oliehoek, Vlassis (2008), "Multiagent Planning under Uncertainty with Stochastic Communication Delays", ICAPS, pp. 338-345, https://aaai.org/papers/icaps-08-042-multiagent-planning-under-uncertainty-with-stochastic-communication-delays/.
- arXiv 2609.13711 (2026), "Decentralized Multi-Robot Task Allocation Under Degraded Communication" (search snippet only). Combines implicit coordination with consensus and uses Gilbert-Elliott loss. Closest on the loss model; the task is allocation, not search.
- arXiv 2603.20285 (2026), "AgentComm-Bench" (snippet only). Bursty Gilbert-Elliott loss weakens redundancy-based protection.
- Liu et al. (2013), CoBA, SAGE. Notes that duplicate assignment arises under packet loss.

What they already show: loss causes duplicated work; implicit coordination degrades gracefully; bursty loss defeats redundancy.

Verdict: APPEARS OPEN as a head-to-head in cooperative search. I found no paper comparing delta-on-default against replacement protocols. It is likely a design artifact.
Narrowest statement: in our protocol implementations, replacement semantics (a receiver discards the default plan when it receives or expects a message) cause duplication under bursty loss. Deltas against a retained default degrade gracefully.
Contradicting evidence: a lease that reverts to the default plan on expiry should match deltas. Your result may reflect implementation, not a general law. Include that variant as an ablation.
Falsifiable predictions: (a) RESERVE and FULL degrade faster than EVENT as the bad-state loss rate rises. (b) Adding fallback-to-default on expiry removes most of the gap. (c) Under i.i.d. loss at the same mean rate, the gap shrinks, because bursts are what defeat the replacement protocols.

## Recommendation
1. Frame as an empirical study, not a theory contribution: C1 is background (cite Nayyar, Roth, Wu), C2 is a quantified sanity check, and C3 is the headline finding.
2. Lead with C3 plus the fallback-to-default ablation. That turns "implementation artifact" into a design principle: retain the shared default, and send only deltas.
3. Keep C1's wording modest ("silence is informative given a common plan"), and state the common-knowledge assumption and the aborted-look and hint caveat in the paper.
