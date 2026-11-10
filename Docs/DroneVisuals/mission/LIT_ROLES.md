# LIT_ROLES: Recon vs verify role specialization in multi-UAV forest SAR

Compiled 2026-11-09. Scope: is adaptive recon/verify role specialization (conditions a-e, N = 2/3/5/10, occlusion, speed/blur/view-dependent detection, comms outages) a genuine contribution? Keys refer to `mission_refs.bib`. Legend: **V** = checked against a primary record (Crossref, arXiv abstract, publisher). **S** = from secondary snippets only; treat as UNVERIFIED.

## 1. Comparison table

| Work (year) | Sensing model | Comms | Team | Roles | Adaptive? | Scale | Evidence |
|---|---|---|---|---|---|---|---|
| Jin et al. CDC 2003; TSMC-B 2006 [jin2003cooperative, jin2006balancing] | Abstract detect/confirm events, battlefield | Shared | Heterogeneous classes | Search, Confirm, Attack, BDA ("assignable" tasks) | **Yes**, real-time task assignment; hybrid switching to balance search vs. response | Small (S) | Sim |
| Schumacher et al. ACC 2002 [schumacher2002task] | Search/classify/attack/BDA | Central | Homogeneous munitions | Classify = verify | Yes (network flow) | Small (S) | Sim |
| Senthilnath et al. TSMC 2024 [senthilnath2024metacognitive] | Two sensors: wide range with false positives; short range, perfect confirmation | **None** | Homogeneous | Detect/confirm as *modes within each UAV* | Yes, per agent ("when-to-switch-sensor") | n/a (S) | Monte Carlo sim |
| Lee et al. ICRA 2021 + workshop [lee2021upper, lee2021decentralised] | Scouts: long range, low res. Task robots: short range, detailed | Decentralised Dec-MCTS | **Heterogeneous** | Scout / task | No, fixed by hardware | Multi-drone (count not given) | Sim (hardware "ongoing") |
| Zhu et al. arXiv 2023 [zhu2023autonomous] | Local observation, grid | Global state for role policy | Homogeneous | Explore vs. coverage/target-localise | **Yes**, hierarchical MARL switches roles each step | Varied N (claims scalability) | Sim |
| Kwa et al. 2020; Artif. Life 2023 [kwa2020heterogeneous, kwa2023adapting] | Binary/CMOMMT target sensing | k-NN topology | **Fast + slow** agents | Implicit; tuning explore/exploit | Connectivity tuned, not role ratio | 50 agents | Sim |
| de Hoog et al. SSRR 2010 [dehoog2010dynamic] | Mapping | Limited range | Homogeneous | Explorer / relay | **Yes**, proximity-based swap | Small | Sim |
| Krieger et al. Nature 2000 [krieger2000ant] | Foraging | Local/recruitment | Homogeneous hardware, heterogeneous thresholds | Forage / rest | **Yes**, response thresholds | Up to 12 robots (companion RAS paper, S) | **Real robots** |
| Hayat et al. AuRo 2020 [hayat2020multi] | Coverage | Link-quality objective | Homogeneous | Search / inform / monitor tasks | Reassignment as information is shared | Small (S) | Sim (GA) |
| Spurný et al. JFR 2019 [spurny2019cooperative] | Colour objects | Shared | 3 homogeneous MAVs | Search then pick | Job manager, failure recovery | 3 | **Real (MBZIRC 2017 winner)** |
| Carpin et al. JFR 2013 [carpin2013variable] | Altitude-dependent FP/FN, probabilistic quadtree | n/a | Single quadrotor | Multi-resolution (coarse high, fine low) | Yes (information gain) | 1 | **Real flights** + dataset |
| Schedl et al. Sci. Robot. 2021 [schedl2021autonomous] | AOS integral imaging under canopy | n/a | Single drone | none | Adaptive path | 1 | **Field**: 38/42 persons found |
| Nathan et al. Commun. Eng. 2023, 2025 [nathan2023drone, nathan2025autonomous] | AOS anomaly detection, confidence-weighted | ~80 ms latency, high bandwidth, assumes good links | **Homogeneous** swarm | none (PSO particles) | Sampling pattern adapts to detection confidence | Field 6; framework up to 10 | Sim (2023); **field** (2025): precision 93.2%, recall 95.9% |
| Strand et al. arXiv 2025 [strand2025enhancing] | Occlusion-aware next-best view (NBV) | n/a | Single | Viewpoint planner (verify primitive) | Yes | 1 | Sim + field; >90% hidden objects found in sim |
| Kareem et al. EST-IJ 2026 (BSODS) [kareem2026enhancing] | High-altitude thermal coarse scan, low-altitude YOLO confirm | ? | Dual-altitude | Survey / confirm, "adaptive inter-UAV role assignment" | Claimed yes | ? | Sim, PX4/Gazebo in companion paper (S) |
| Liu et al. Drones 2024 [liu2024reinforcement] | 3 altitude levels, FOV/accuracy trade-off | ? | Homogeneous | High/low layers | MAPPO | 3-8 (S) | Sim |
| Giacomossi et al. MIPRO 2026 (IRDS) [giacomossi2026market] | n/s | n/s | Homogeneous | Sector bidding; consensus verification decoupled from exploration | Yes (reverse auction) | 8 | Sim |
| Lanča et al. Drones 2025 [lanca2025optimal] | YOLO detection vs. speed/height (motion blur) | n/a | n/a | n/a | n/a | n/a | Real imagery; optimum about 5.4-5.5 m/s |
| Kress et al. MMOR 2008 [kress2008optimal] | Discrete search with false positives | n/a | Single | Search/investigate | Optimal policy | 1 | Theory |
| Rosenfeld et al. AIJ 2008 [rosenfeld2008study] | Foraging/coverage | Varied | Homogeneous | none | Adaptive coordination | Group size swept; 2-25 in IJCAI'05 (S) | Sim |

## 2. KNOWN vs OPEN

**Known (with primary evidence)**
- *Search vs. confirm is a recognised trade-off.* Wide-area-search-munition work gives explicit Confirm/Classify tasks reassigned dynamically among UAVs and shows a search/response trade-off [jin2003cooperative, jin2006balancing, schumacher2002task].
- *Fixed heterogeneous scout/task teams* (long-range low-res plus short-range high-res) run exploration and exploitation at the same time without explicit balancing [lee2021upper].
- *Adaptive role switching* for explore vs. target coverage has been learned with MARL [zhu2023autonomous]. Explorer/relay swaps exist [dehoog2010dynamic]. Threshold-based division of labour works on real robots [krieger2000ant].
- *Coarse-to-fine, altitude-dependent detection.* Carpin et al. measured altitude-dependent FP/FN in real flights [carpin2013variable]. Dual-altitude survey-then-confirm for SAR victims is claimed [kareem2026enhancing, S]. Speed-dependent detection (motion blur) has an interior optimum [lanca2025optimal].
- *Forest occlusion.* Multi-view integration and confidence-driven swarm sampling handle forest occlusion in the field [schedl2021autonomous, nathan2025autonomous]. NBV raises under-canopy detection [strand2025enhancing].
- *Revisit-capable Bayesian search* with false alarms is standard [bourgault2003coordinated, waharte2010probabilistic, chung2012analysis]. Optimal search with false positives is solved for a single searcher [kress2008optimal].
- *Diminishing returns and interference* with group size [rosenfeld2008study].

**Contradicting evidence (against expecting large gains)**
- de Hoog et al. found dynamic roles gave "only a small improvement" in exploration speed. The main benefit was relaying [dehoog2010dynamic].
- Kwa et al. report that adding upgraded (faster) agents "may result in degraded mission performances" without strategy changes [kwa2020heterogeneous]. Giving fast and slow agents differentiated strategies was "challenging" [kwa2023adapting].
- Nathan et al. reach 95.9% recall with a *role-free* homogeneous swarm under dense canopy [nathan2025autonomous]. That is a strong argument that a confidence-driven homogeneous policy may leave little headroom.
- Senthilnath et al. get detect-then-confirm *within each agent* with no comms [senthilnath2024metacognitive]. Specialization across agents is not needed for confirmation.

**Open (no primary source found doing this)**
- A controlled comparison of homogeneous, fixed-ratio and adaptive recon/verify policies **against a strong role-free shared-revisit baseline (c)**, swept over N, under speed- and view-dependent detection *and* comms outages. Forest SAR with "confirmed and reported" latency as the metric has not been done.
- How the optimal verifier fraction scales with N and with outage rate.
- Whether role persistence (commitment plus hysteresis) beats per-task greedy reallocation when the candidate map is stale.

## 3. Three strongest threats to novelty

1. **Lee et al. 2021 scout-task architecture** [lee2021upper, lee2021decentralised], together with dual-altitude survey/confirm SAR [kareem2026enhancing] and MARL role selection [zhu2023autonomous]. Between them these cover (d) and a version of (e). Reviewers will say that heterogeneous recon/verify with adaptive role selection already exists.
2. **The Jin/Polycarpou search-confirm task allocation lineage (2003-2006)** [jin2003cooperative, jin2006balancing]. "Adaptive role reallocation" between search and confirm is dynamic task allocation of Confirm tasks, published 20 years ago. Under the Gerkey-Matarić taxonomy [gerkey2004formal], a "role" for homogeneous drones is just the currently assigned task type.
3. **Baseline (c) subsumes (e) by construction.** A homogeneous team that can choose speed per cell and revisits low-confidence cells from a shared Bayesian map [bourgault2003coordinated, chung2012analysis] *is* an adaptive role policy with zero switching cost. If (e) does not beat (c), there is no contribution. If it does, reviewers will ask whether (c) was tuned well. Nathan et al. 2025 shows role-free confidence-driven swarms already work in the field.

## 4. Narrowest genuinely open claim

> For physically identical drones in occluded forest with speed- and view-dependent detection, **explicit role commitment** (recon vs. verify, with switching hysteresis) improves time to *confirmed and reported* detection over a tuned role-free shared-revisit policy (c) **only in a characterisable regime**: large N relative to area, a steep detection-vs-speed curve, a non-trivial false-positive rate and intermittent comms. The paper maps that regime, including where it reverses.

This is an empirical regime-mapping claim, not a new algorithm. It survives threats 1-3 only if (c) is strong and tuned with the same budget as (e). Do not claim "first heterogeneous/adaptive recon-verify team".

**Falsifiable predictions** (each can kill the claim)
- P1. At N=2, (e) does not beat (c) (difference within the CI). A dedicated verifier either idles or starves recon.
- P2. The (e)-over-(c) gain grows with N and saturates or reverses by N=10 at fixed area, as coverage saturates and interference rises [rosenfeld2008study].
- P3. The optimal verifier fraction in (d) rises with N and with false-positive rate. If it is flat, the "adaptive" story loses its motivation.
- P4. If P_d(v) is flat (blur off), (a) dominates (b), and (d)/(e) collapse to (a).
- P5. Under outages, (c) degrades more than threshold-based (e), because (c) relies on a fresh shared map while (e) verifiers commit locally. If the reverse holds, the comms motivation fails.
- P6. Under unlimited time, recall of (b) is at least that of (e). The benefit of (e) shows only in time-to-confirmed and budgeted recall.

## 5. Reproducible baselines (with adaptation notes)

| ID | Baseline | Source | Adaptation |
|---|---|---|---|
| B1 | Boustrophedon at v_fast / v_slow → (a), (b) | standard | Use the same P_d(v, view, occlusion) as all arms |
| B2 | Bayesian probability map with FP/FN-aware updates, greedy/receding-horizon revisits → (c) | [bourgault2003coordinated, waharte2010probabilistic, chung2012analysis] | Let each drone choose speed per cell. Same budget for tuning hyperparameters as (e) |
| B3 | Dynamic Confirm-task assignment (nearest free UAV) | [jin2006balancing] | Confirm = slow pass + NBV. Strongest non-role reallocator |
| B4 | Fixed scout/task split → (d) | [lee2021upper] | No public code found; implement fixed ratios {1:1, 2:1, 4:1} |
| B5 | Response-threshold reallocation → (e) | [krieger2000ant] | Stimulus = unverified-candidate mass within range. Needs little comms |
| B6 | Auction/CBBA role reallocation → (e) alt. | [dias2006market, giacomossi2026market] | Needs connectivity, so a useful contrast with B5 under outages |
| B7 | Per-agent detect→confirm switching, no comms | [senthilnath2024metacognitive] | Key no-comms control: confirmation without roles |
| B8 | Confidence-driven homogeneous sampling | [nathan2025autonomous]; code/data via JKU-ICG/AOS | Approximate with confidence-weighted attraction if AOS imaging is out of scope |
| B9 | NBV verify primitive | [strand2025enhancing] | Shared by all arms that verify, so roles are not confounded with viewpoint quality |

## 6. Where specialization should help or fail (per the literature)

**Plausibly helps**
- Steep detection-vs-speed/altitude trade-off with an interior optimum [lanca2025optimal, carpin2013variable].
- High false-positive rate in clutter, e.g. thermal false alarms that need confirmation [kareem2026enhancing (S), senthilnath2024metacognitive].
- Large N relative to area, where spatial separation of roles reduces interference [rosenfeld2008study].
- Real hardware heterogeneity (sensor range or resolution) [lee2021upper].
- A metric that rewards time-to-confirmed rather than raw recall [jin2006balancing].

**Plausibly fails**
- Small N (2-3).
- A flat P_d(v).
- Negligible switching cost, where (c) is already adaptive.
- Low candidate density.
- Heavy outages that leave verifier queues stale or require rendezvous overhead [dehoog2010dynamic].
- Heterogeneity without matching strategy changes [kwa2020heterogeneous, kwa2023adapting].
- A homogeneous confidence-driven swarm that already reaches about 96% recall under canopy [nathan2025autonomous].

Not verified in full text: BSODS method and baselines (paywalled, 403); team sizes in Jin 2006 and Lee 2021; the companion experiments in Krieger 2000. Check these before citing specifics.
