# Literature review: two-drone GPS-denied forest SAR (find, verify, report) under radio outages and incomplete maps

Compiled 2026-11-09 under a 75-minute cap. Keys refer to `mission_refs.bib`.
**[V]** means checked on the primary source (paper, arXiv abstract, Crossref record, or official repo).
**[S]** means secondary source only (search snippet or third-party summary). Treat it as unverified.
"No license seen" means the repo showed no LICENSE file. For CRANE (GPL-3.0) such code can only be reimplemented from the paper, not copied.

## Adaptation labels (for our fast 2D/2.5D model)

- `[2D]`: collapse to a grid. 2.5D adds a height or canopy layer.
- `[POD]`: replace the detector with a view- and altitude-dependent probability of detection (POD) plus a false-alarm model.
- `[COMM]`: replace radios with a range/line-of-sight disk plus an outage schedule.
- `[DRIFT]`: replace SLAM/VIO with injected relative-frame drift plus discrete alignment events.
- `[KIN]`: point-mass kinematics with speed caps.
- `[REIMPL]`: reimplement from the paper.
- `[LIC]`: license blocks copying.

## Comparison table

| Work | Observes | Assumes (comms / localization / map) | Optimizes | Demonstrates | Code, license | Our baseline? |
|---|---|---|---|---|---|---|
| **RACER** `zhou2023racer` | Depth, onboard state estimation [V] | Async pairwise, limited range; decentralized; no prior map [V] | Hgrid partition, CVRP workload balance, min-time trajectories | Sim: 1–10 UAVs, comm range ∞/10/5 m. Real: 3 UAVs, 15×12×2 m³ forest, ≤1.5 m/s [V] | SYSU-STAR/RACER; no license seen | **Yes** `[2D][COMM][KIN][REIMPL][LIC]` |
| **Swarm in the Wild** `zhou2022swarm` | Depth, VIO [S] | Trajectory broadcast over WiFi [S]; no external infrastructure [V] | MINCO joint spatial-temporal optimization: safety, inter-robot avoidance, feasibility | 10 drones in a bamboo forest [S]; sim vs MADER and EGO-Swarm [S] | EGO-Planner-v2, GPL-3.0 [V] | Partial (deconfliction) `[KIN]` |
| **EGO-Swarm** `zhou2021egoswarm` | Depth, VIO | Unreliable trajectory sharing; drift corrected by detecting agents in depth images [V] | Gradient-based local trajectories | Sim plus real [V] | Code released; license not checked | Drift-reset-on-sighting `[DRIFT]` |
| **Learning High-Speed Flight** `loquercio2021learning` | Stereo depth plus inertial measurements [V] | Single robot; no map or comms; reference trajectory given [V] | Imitates a privileged sampling expert | 56 real runs at 3–10 m/s, 60% success at 10 m/s. Sim: "at low speeds (3 m/s) all methods perform similarly" [V] | agile_autonomy, GPL-3.0; Flightmare and TF-GPU [V] | Its **Reactive** baseline, yes `[KIN][2D]` |
| **FASTER** `tordesillas2022faster` | Depth to map | Single robot; known-free vs unknown space | Min-time MIQP through unknown space, with a safe backup in known-free space | UAV up to 7.8 m/s, ground robot 2 m/s, sim plus real [V] | mit-acl/faster, BSD-3; Gurobi [V] | **Yes** `[KIN][2D]` |
| **SUPER** `ren2025super` | LiDAR; FAST-LIO [V] | Single robot | Exploratory (known plus unknown) plus backup (known-free) trajectory each cycle [S] | Failure rate reduced 35.9× vs baselines [S]; >20 m/s (press release) [S] | hku-mars/SUPER; no license seen [V] | As FASTER `[KIN][LIC]` |
| **Bubble Planner** `ren2022bubble` | LiDAR [V] | No prior map, no external localization [V] | Receding-horizon sphere corridors | Forest flight >13.7 m/s [V] | Official code not found | Yes `[KIN][REIMPL]` |
| **Adaptive replanning** `jarinlipschitz2022adaptive` | Onboard sensing | Single robot | Tunes graph dispersion online to local clutter | Pine forest ≤2.5 m/s; beats a fixed planner in sim [V] | Not checked | Tuned-switch analogue `[KIN]` |
| **Intermittent Rendezvous** `dasilva2024intermittent` | 2D LiDAR, map merging (repo) [V] | Exchange only in range or at rendezvous; DEC-POMDP [V] | Job-shop scheduling (JSSP) rendezvous plan: waiting time, connectivity | Gazebo; beats relay and base-contact baselines [V] | Noetic sandbox, **GPL-3.0** [V] | **Yes** `[2D][COMM]` |
| **MACE** `rossano2026mace` | n/a | Scheduled windows; known comm points [V] | Orienteering: travel-to-connect vs exploration gain | Sim; up to 23% faster exploration [V] | None linked | Yes `[COMM][REIMPL]` |
| **MOCHA** `cladera2024mocha` | Air-ground team | Gossip-based opportunistic sync [V] | UAV route maximizing information shared [V] | Field plus sim sweep of team size and range [V] | BSD-3, ROS 2 [V] | **Yes** `[COMM]` |
| **ACHORD** `saboia2022achord` | SubT team | Intermittent; droppable radios; base station [V] | Data stratification, QoS/flow control | DARPA SubT Finals [V] | None linked | Priority classes `[COMM][REIMPL]` |
| **ConTaCT** `unhelkar2016contact` | Belief of teammate | Decentralized [V] | Communicate when value of information (VoI) is high [S] | Sim; fewer messages, similar performance [S] | None | Yes `[REIMPL]` |
| **Epistemic planning** `bramblett2023epistemic`, `bramblett2024robust` | n/a | Disconnections; belief reasoning; gossip [S]; MCTS (2024) [V] | Allocation robust to stale beliefs; rewards rendezvous | Sim, SAR-like case [S]; aerial experiments (2024) [V] | Not checked | Yes `[REIMPL]` |
| **Dec-MCTS** `best2019decmcts` | Active perception | Degraded comms tolerated [S] | Decentralized MCTS | Orienteering; object recognition on real data [S] | Not checked | Yes `[REIMPL]` |
| **SAR with QoS** `hayat2020multi`, `hayat2017multi` | Camera (reliable detection assumed [S]) | Known area; range-limited links to base station (BS) [S] | Search time plus report-path setup: finder hovers, then data mule or relay chain [S] | Sim; up to 65% shorter completion (2017) [S] | None | **Yes** `[2D][COMM][POD]` |
| **AOS drone** `schedl2021autonomous` | Synthetic-aperture imaging (thermal [S]) | Single drone; sends only detections over weak links [V] | Re-samples uncertain detections | 17 flights, 38/42 found, 86% AP; adaptive re-check raised confidence by 15% [V] | JKU-ICG/AOS; license not checked | **Yes** `[POD]` |
| **WiSARD** `broyles2022wisard`, plus `qingqing2020towards` | UAV RGB plus long-wave infrared [V]; detection vs altitude [S] | Dataset | n/a | 26,862 visual, 29,989 thermal, 15,453 synchronized pairs [V] | MIT-style license [V] | POD calibration `[POD]` |
| **Lin & Goodrich** `lin2009uav` | Koopman POD [S] | Known probability map | Probability collected per flight time [S] | Sim, 60×60 grid [S] | None | **Yes** `[2D]` |
| **Waharte et al.** `waharte2010probabilistic` | Altitude-dependent detection [S] | Grid, single target [S] | Variable-altitude search | Sim [S] | None | Yes `[POD]` |
| **Bourgault et al.** `bourgault2003coordinated` | POD likelihoods [S] | Decentralized filters, neighbour comms [S] | Coordinated search utility | Sim [S] | None | Yes `[2D]` |
| **OA-IPP** `meera2019obstacle` | Camera; GP target occupancy [V] | Obstacle map | Information gain, coverage, collision [V] | Sim, urban SaR [V] | IPP-SaR; no license seen [V] | Yes `[POD][LIC]` |
| **Search decisions** `chung2012analysis` | Imperfect sensor [S] | Grid | When to declare "found" [S] | n/a | None | Yes `[POD]` |
| **Viewpoint-dependent classification** `feldman2018bayesian` | Multi-view classifier scores [S] | Localization uncertainty [S] | Robust Bayesian classification | Experiments [S] | n/a | Verification belief `[POD]` |
| **DOOR-SLAM** `lajoie2020door` | VIO-class | Peer-to-peer; PCM outlier rejection [S] | Distributed pose-graph optimization | Sim, datasets, subterranean field tests [S] | MIT [V] | `[DRIFT]` |
| **Swarm-SLAM** `lajoie2024swarm` | LiDAR, stereo, RGB-D [S] | Sporadic rendezvous; loop-closure prioritization [S] | Collaborative SLAM | Datasets plus a 3-robot real test [S] | Not checked | `[DRIFT]` |
| **Kimera-Multi** `tian2022kimera` | VIO plus semantics | Peer-to-peer; demo "will most likely not work with intermittent communication" [V] | Robust distributed PGO | n/a | Index repo; no license seen [V] | `[DRIFT]` |
| **EM exploration** `huang2024multi` | Range sensors [V] | Localization plus map uncertainty [V] | EM-style action prediction for allocation [V] | Unverified | None linked | Partial `[DRIFT][REIMPL]` |
| **Comm-free coordination** `boldrer2026perception` | One 3D LiDAR for SLAM and neighbours [V] | **No comms**, no GNSS [V] | Perception-aware safe navigation | Sim plus forest (3 UAVs [S]) | None linked | Yes `[KIN][REIMPL]` |

Also checked: COVINS-G `patel2023covins` (centralized, GPLv3 per headers [S]) and Boldrer's Lloyd flocking `boldrer2025lloyd` (no explicit comms, ≤4 UAVs in forest [V]).

**Not found.** I found no Boldrer paper on "connectivity-aware vs connectivity-free *exploration*", and no paper using "trust in map" as a named method.

## Baseline shortlist

1. **B1, POD-greedy Bayesian search** (`lin2009uav`, `bourgault2003coordinated`, `waharte2010probabilistic`). A belief grid plus a receding-horizon route, with 2.5D POD calibrated from WiSARD. `[2D][POD]`. This is the floor.
2. **B2, RACER-lite.** Disjoint partitions exchanged only when in range, plus a greedy or CVRP tour. `[2D][COMM][KIN][REIMPL]`.
3. **B3, Intermittent rendezvous** (`dasilva2024intermittent`). A precomputed rendezvous schedule. The GPL-3.0 code is compatible; port the logic. `[2D][COMM]`.
4. **B4, Find-then-mule** (`hayat2020multi`). The finder holds or verifies while the partner ferries the report or forms a relay. `[2D][COMM][POD][REIMPL]`.
5. **B5, Threshold re-check** (`schedl2021autonomous`). Re-sample detections whose confidence lies in [τ_lo, τ_hi] before reporting. `[POD]`.
6. **B6, Connectivity-free gossip** (`cladera2024mocha`, `boldrer2026perception`). No comms planning: sync when in range, with an ACHORD-style priority queue (confirmed finds, then candidates, then map). `[COMM]`.
7. **B7, Local navigation.**
   - (a) Memoryless reactive primitives.
   - (b) An optimistic path with a known-free backup (FASTER/SUPER style).
   - (c) A fixed-threshold switch on clearance or map age, tuned per environment.

   `[KIN][2D]`
8. **B8, VoI-triggered messaging** (`unhelkar2016contact`). Send only when the teammate's decision would change. `[REIMPL]`.

## Three prior results most likely to undermine our claims

**T1. Partitioning already absorbs most of the outage cost** (`zhou2023racer`, Table III [V]). Cutting comm range from ∞ to 5 m raised RACER's exploration time only from 38.5 to 46.4 s (Pillar) and from 35.8 to 47.3 s (Office). The authors credit disjoint cells, which mean "a quadrotor does not visit regions assigned to others." With two drones in disjoint sectors, outage-aware routing has little coverage left to win.
*Prediction P1:* outage-aware routing improves coverage time by <10% over B2. Any meaningful gain appears only in time-to-confirmed-report.

**T2. "Detection and comms change routes" is already published** (`hayat2020multi` [S]; `schedl2021autonomous` [V]). Hayat et al. treat communication "not as a constraint but a mission goal" and plan search followed by a mule or relay report phase. Schedl et al. re-route to re-check uncertain detections and transmit only detections. Our novelty must therefore be the *joint* objective under view-dependent POD, false alarms and frame drift.
*Contradicting evidence:* Hayat appears to assume reliable detection and localization [S], and Schedl is single-drone with no comms planning.
*Prediction P2:* B4+B5 together capture ≥70% of our planner's time-to-confirmed-report gain when the false-alarm rate is below 5%.

**T3. At SAR speeds local planners tie, and backup planners already handle unknown space** (`loquercio2021learning` [V]; `tordesillas2022faster`; `ren2025super` [S]). In Loquercio's simulation, all methods were similar at 3 m/s and diverged only at ≥5 m/s. RACER's forest flights were at ≤1.5 m/s. FASTER and SUPER keep a known-free backup every cycle. A confidence-aware hybrid can win only where staleness or drift actually invalidates the map, and it must beat B7b and the tuned B7c.
*Prediction P3:* at ≤3 m/s with a static map and no drift, the hybrid's collision rate is statistically indistinguishable from tuned B7c. A gap requires injected staleness or relative drift of about one obstacle spacing or more.

## Open gaps (search-limited; absence of evidence is weak)

- **G1. Time-to-confirmed-report as the objective.** No paper I found optimizes the full find, verify, deliver chain under intermittent links. The pieces appear separately: map exchange (da Silva, MACE), finding plus reporting with reliable detection (Hayat), single-drone verification (Schedl).
- **G2. Verification-view budgeting in multi-UAV allocation with an explicit false-alarm cost.** View-dependent POD exists only for a single robot (Waharte, Meera) or for classification (Feldman).
- **G3. Report and task value conditioned on frame-alignment covariance.** Collaborative SLAM estimates alignment but does not feed it into allocation. Huang 2024 covers exploration only.
- **G4. Planner arbitration triggered by map staleness or confidence.** Existing switches trigger on clearance or detections. I found no comparison against tuned switching.
- **G5. VoI over compact SAR report content** (confidence, covariance, verification state). ACHORD stratifies data classes, and ConTaCT decides *whether* to send.

## Verification notes

- Full-text reads: RACER (experiments) and Loquercio (results).
- Crossref checks: da Silva, MOCHA, EGO-Swarm.
- Science Robotics returned 403, so Swarm-in-the-Wild and SUPER details are [S].
- Hayat, Chung, Feldman, Lin, Waharte and Bourgault contents are [S]. Read them before relying on them.
