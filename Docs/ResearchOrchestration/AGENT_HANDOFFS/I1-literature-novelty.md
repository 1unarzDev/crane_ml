# I1 — Literature & Novelty Critique (CRANE)

Date: 2026-10-20. Role: literature/novelty critic. No code edited.

Candidate RQ under review:
> Do calibrated transient actuator/delivery-timing models plus uncertainty improve held-out navigation-configuration selection, robust transfer, and abstention under limited data/trial budgets, versus strong black-box/contextual baselines with uncalibrated or steady-state models?

Verification legend: **[V]** = located primary source or official index (arXiv/PMLR/IEEE/DOI) during this session and checked title/venue/year. **[V-abs]** = verified bibliographic data and abstract-level claim only; the specific detail cited was not read in full text. **[U]** = could not verify; do not cite without checking.

---

## 1. Sources (verified bibliographic data)

### A. Navigation-parameter tuning
| Key | Citation | Status |
|---|---|---|
| APPLD | Xiao, Liu, Warnell, Fink, Stone. "APPLD: Adaptive Planner Parameter Learning from Demonstration." IEEE RA-L, 2020. arXiv:2004.00116 | [V-abs] |
| APPLR | Xu, Dhamankar, Nair, Xiao, Warnell, Liu, Wang, Stone. "APPLR: Adaptive Planner Parameter Learning from Reinforcement." ICRA 2021, pp. 6086–6092. arXiv:2011.00397 | [V] |
| APPL | Xiao et al. "APPL: Adaptive Planner Parameter Learning." Robotics and Autonomous Systems 154, 2022 | [V-abs] |
| APPLI / APPLE | APPLI (interventions) ICRA 2021; APPLE (evaluative feedback) RA-L 2021 — listed on APPL project page | [V-abs] (index only) |
| APPLV | Lu, Wang, Wu, Li, Lin, Mao, Xiao. "APPLV: Adaptive Planner Parameter Learning from Vision-Language-Action Model." arXiv:2603.08862, Mar 2026 (preprint) | [V-abs] |
| BARN'22 | Xiao et al. "Autonomous Ground Navigation in Highly Constrained Spaces: Lessons learned from the BARN Challenge at ICRA 2022." IEEE RAM 29(4):148–156, 2022. arXiv:2208.10473 | [V-abs] |
| BARN'23 | Xiao et al. 2nd BARN Challenge at ICRA 2023. IEEE RAM 30(4):91–97, 2023. arXiv:2308.03205 | [V-abs] |
| CURE | Hossen, Kharade, O'Kane, Schmerl, Garlan, Jamshidi. "CURE: Simulation-Augmented Auto-Tuning in Robotics." IEEE T-RO 41:2825–2842, 2025. DOI 10.1109/TRO.2025.3548546. arXiv:2402.05399. **CURE exists; venue is T-RO, not ICSE.** | [V] |
| EffiTune | Feng et al. "EffiTune: Diagnosing and Mitigating Training Inefficiency for Parameter Tuner in Robot Navigation System." IROS 2025. arXiv:2409.10832 | [V-abs] |
| Costmap-energy | Albonico, Wortmann, Malavolta. "Tuning ROS 2 for Energy-Efficient Navigation: Empirical Insights from Costmap 2D Configurations." arXiv:2609.12971, Sep 2026 (preprint) | [V-abs] |
| Zheng guide | Kaiyu Zheng. "ROS Navigation Tuning Guide." arXiv:1706.09068 (2016/2017; tech guide, later book chapter) | [V-abs] |
| Contextual GP-UCB | Krause, Ong. "Contextual Gaussian Process Bandit Optimization." NIPS 2011 | [V] |
| SafeOpt | Sui, Gotovos, Burdick, Krause. "Safe Exploration for Optimization with Gaussian Processes." ICML 2015 | [V] |
| SafeOpt-quad | Berkenkamp, Schoellig, Krause. "Safe Controller Optimization for Quadrotors with Gaussian Processes." ICRA 2016. DOI 10.1109/ICRA.2016.7487170 | [V] |
| SafeOpt-MC | Berkenkamp, Krause, Schoellig. "Bayesian Optimization with Safety Constraints: Safe and Automatic Parameter Tuning in Robotics." Machine Learning (2021/2023). arXiv:1602.04450 | [V-abs] (journal year not pinned) |
| MetaBO | Volpp et al. "Meta-Learning Acquisition Functions for Transfer Learning in Bayesian Optimization." ICLR 2020. arXiv:1904.02642 | [V] |
| MF-ES | Marco, Berkenkamp, Hennig, Schoellig, Krause, Schaal, Trimpe. "Virtual vs. Real: Trading Off Simulations and Physical Experiments in RL with Bayesian Optimization." ICRA 2017, pp. 1557–1563 | [V] |
| LLM-DWA | "LLM-DWA: a hybrid path planning framework combining LLMs with the dynamic window approach." Scientific Reports, 2026 (s41598-026-39524-1). LLM produces waypoints; DWA weights swept, **not LLM-tuned** | [V-abs] |
| LLM-Nav2 latency | "Latency-Aware Benchmarking of LLMs for Natural-Language Robot Navigation in ROS 2" (PMC12846292). LLM parses instructions into Nav2 goals; **not parameter tuning** | [V-abs] (authors/venue not pinned) |

**LLM-based Nav2/move_base parameter tuning:** I found **no** peer-reviewed or arXiv paper in which an LLM directly tunes Nav2/move_base controller/costmap parameters. The closest is APPLV (VLM backbone + regression head predicting planner parameters, 2026 preprint). Treat "LLM tuner" as an unverified category; absence of evidence from two searches, not proof of absence.

### B. Sim-to-real: SysID vs DR, actuator/delay modeling, adaptive DR
| Key | Citation | Status |
|---|---|---|
| Tan18 | Tan et al. "Sim-to-Real: Learning Agile Locomotion For Quadruped Robots." RSS 2018. arXiv:1804.10332. Actuator model + measured latency (reported ~3 ms PD, 15–19 ms TX2) injected into sim | [V-abs] (latency numbers via secondary snippet) |
| Peng18 | Peng, Andrychowicz, Zaremba, Abbeel. "Sim-to-Real Transfer of Robotic Control with Dynamics Randomization." ICRA 2018. arXiv:1710.06537 | [V]; **timestep/latency randomization detail and ablation [U] — not confirmed in full text** |
| Hwangbo19 | Hwangbo, Lee, Dosovitskiy, Bellicoso, Tsounis, Koltun, Hutter. "Learning agile and dynamic motor skills for legged robots." Science Robotics 4(26):eaau5872, 2019. arXiv:1901.08652 | [V]; actuator-net input (pos-error/vel history at t, t−10, t−20 ms) and "ideal actuator model fails" claim are **[V-abs/secondary]** |
| SimOpt | Chebotar et al. "Closing the Sim-to-Real Loop: Adapting Simulation Randomization with Real World Experience." ICRA 2019, pp. 8973–8979 | [V] |
| BayesSim | Ramos, Possas, Fox. "BayesSim: Adaptive Domain Randomization via Probabilistic Inference for Robotics Simulators." RSS 2019. DOI 10.15607/RSS.2019.XV.029 | [V] |
| DROPO | Tiboni, Arndt, Kyrki. "DROPO: Sim-to-real transfer with offline domain randomization." Robotics and Autonomous Systems 166:104432, 2023. arXiv:2201.08434 | [V] |
| ADR-bench | Tiboni et al. "Online vs. Offline Adaptive Domain Randomization Benchmark." HFR workshop (Springer) 2022. arXiv:2206.14661 — notes offline open-loop replay can fail | [V-abs] |
| AutoTuned | Du, Watkins, Darrell, Abbeel, Pathak. "Auto-Tuned Sim-to-Real Transfer." ICRA 2021, pp. 1290–1296 | [V] |
| AdaptSim | Ren, Dai, Burchfiel, Majumdar. "AdaptSim: Task-Driven Simulation Adaptation for Sim-to-Real Transfer." CoRL 2023, PMLR 229:3434–3452 | [V] |
| Truong22 | Truong, Rudolph, Yokoyama, Chernova, Batra, Rai. "Rethinking Sim2Real: Lower Fidelity Simulation Leads to Higher Sim2Real Transfer in Navigation." CoRL 2022 (PMLR 205:859–870, 2023) | [V] |
| SRCC | Kadian, Truong, et al. "Sim2Real Predictivity: Does Evaluation in Simulation Predict Real-World Performance?" IEEE RA-L 5:6670–6677, 2020 | [V] |
| EIDA | Qian, Wang, Hu, Zhang, Zhang. "EIDA: Execution-Interface Dynamics Adaptation for Real-to-Sim-to-Real Robot Navigation." arXiv:2610.01219, Oct 2026 (preprint, not peer-reviewed) | [V-abs] |
| WLeg-nav | "Learning Robust Autonomous Navigation and Locomotion for Wheeled-Legged Robots." arXiv:2405.01792 (neural actuator models incl. wheel velocity→current) | [V-abs] (venue not pinned) |
| PACE | "Towards bridging the gap: Systematic sim-to-real transfer for diverse legged robots." arXiv:2509.06342 (fits global delay, ~7.5 ms) | [V-abs] (secondary snippet) |

### C. USV / marine identification
| Key | Citation | Status |
|---|---|---|
| Fossen21 | T. I. Fossen. *Handbook of Marine Craft Hydrodynamics and Motion Control*, 2nd ed., Wiley, 2021. ISBN 978-1-119-57505-4 | [V] |
| Sonnenburg13 | Sonnenburg, Woolsey. "Modeling, Identification, and Control of an Unmanned Surface Vehicle." J. Field Robotics, 2013. DOI 10.1002/rob.21452 (bilinear thruster model; first-order Nomoto best compromise at speed; sideslip lag at low speed) | [V-abs] |
| Morel25 | Morel, Orihuela, Combastel, Bejarano. "Practical identification approach for the actuation dynamics of autonomous surface vehicles with minimal instrumentation." Ocean Engineering 319:120098, 2025. arXiv:2410.00631 (static polynomial vs dynamic propeller model; T200 asymmetric dead zone noted but neglected; R² > 0.94) | [V-abs] |
| Lanxin18 | "Modeling and Identification for Vector Propulsion of an USV: 3-DOF Model and Response Model." Sensors 18(6):1889, 2018 (includes thruster servo model) | [V-abs] (authors not pinned) |
| Harris23 | Harris, Mao, Paine, Whitcomb. "Stable nullspace adaptive parameter identification of 6-DOF plant and actuator models for underactuated vehicles." IJRR, 2023 | [V-abs] |
| MOSAIC-SV | Mentioned in a search snippet as "real-time adaptive dynamics identification ... from spec-sheet prior" | **[U] — no title/venue verified; do not cite** |

### D. Uncertainty, selective prediction, abstention
| Key | Citation | Status |
|---|---|---|
| SelCls | Geifman, El-Yaniv. "Selective Classification for Deep Neural Networks." NIPS 2017 (risk–coverage framing) | [V] |
| CP-shift | Tibshirani, Barber, Candès, Ramdas. "Conformal Prediction Under Covariate Shift." NeurIPS 2019 | [V-abs] (cited via secondary listing) |
| RobustCP-STL | Zhao, Hoxha, Fainekos, Deshmukh, Lindemann. "Robust Conformal Prediction for STL Runtime Verification under Distribution Shift." ICCPS 2024. arXiv:2311.09482 | [V-abs] |
| FallbackMPC | Sinha, Schmerling, Pavone. "Closing the Loop on Runtime Monitors with Fallback-Safe MPC." arXiv:2309.08603 (CDC 2023 per my recollection — **venue [U]**) | [V-abs] |
| FIPER | Römer et al. "Failure Prediction at Runtime for Generative Robot Policies." arXiv:2510.09459 | [V-abs] (venue [U]) |
| CPL | Huang et al. "Conformal policy learning for sensorimotor control under distribution shifts." ICRA 2024 | **[U] — only seen in a reference list** |

### E. Sensor emulation fidelity
| Key | Citation | Status |
|---|---|---|
| LiDARsim | Manivasagam et al. "LiDARsim: Realistic LiDAR Simulation by Leveraging the Real World." CVPR 2020 | [V] |
| Waabi-gap | "Towards zero domain gap: A comprehensive study of realistic LiDAR simulation for autonomy testing" (Waabi; ICCV 2023 per my recollection — **venue [U]**) | [V-abs] |
| TUM-shift | "Quantifying the LiDAR Sim-to-Real Domain Shift..." arXiv:2303.01899 (noise ↑ real AP 38.2→41.6; downsampling no effect) | [V-abs] |
| Kinect-noise | Nguyen, Izadi, Lovell. "Modeling Kinect Sensor Noise for Improved 3D Reconstruction and Tracking." 3DIMPVT 2012, pp. 524–530 (axial/lateral noise vs distance & angle) | [V] |
| Kalibr-time | Furgale, Rehder, Siegwart. "Unified Temporal and Spatial Calibration for Multi-Sensor Systems." IROS 2013, pp. 1280–1286 (camera–IMU time offset); Rehder et al. T-RO 32(2) 2016 | [V] |
| IMU Allan variance | Standard practice (IEEE Std 952; Kalibr IMU noise model) | **[U] in this session — not re-verified** |

### F. Simulation platforms
| Key | Citation | Status |
|---|---|---|
| VRX | Bingham et al. "Toward Maritime Robotic Simulation in Gazebo." OCEANS 2019 Seattle. DOI 10.23919/OCEANS40490.2019.8962724. Fossen-based hydrodynamics; WAM-V coefficients hand-tuned to at-sea maneuvers; static nonlinear thruster model | [V]; **whether VRX thrusters have first-order lag: [U]** |
| HoloOcean | Potokar, Ashford, Kaess, Mangelson. "HoloOcean: An Underwater Robotics Simulator." ICRA 2022, pp. 3040–3046; journal version IEEE JOE 49(4) 2024 | [V] |
| Isaac Lab | Mittal et al. "Isaac Lab: A GPU-Accelerated Simulation Framework for Multi-Modal Robot Learning." arXiv:2511.04831, 2025 (implicit PD, DC-motor, MLP actuator-net [Hwangbo], LSTM actuator models) | [V-abs] |
| Gazebo Sim | Hydrodynamics system derived from VRX plugin (gazebosim.org API docs) | [V-abs] |

---

## 2. Novelty matrix

| Prior work | What it already shows | Overlap with CRANE claim | What remains open | Contradicting / complicating evidence |
|---|---|---|---|---|
| APPLD/APPLR/APPL/APPLV (2020–2026) | Classical planners (DWA/TEB/etc.) benefit from context-dependent parameter selection learned in sim (BARN), transfers to a physical Jackal; APPLR keeps planner safety. APPLV uses VLM features. | Direct: "navigation-configuration selection" that generalizes to held-out environments. | None of these model actuator/delivery timing; contexts are geometric/perceptual, not dynamics. No abstention, no calibrated uncertainty, no cross-domain (water). | BARN reports: classical hand-tuned systems were competitive or better than learned in 2022; gains are environment-specific. |
| CURE (T-RO 2025) | Causal pruning of config space learned in Gazebo, optimization on physical Turtlebot 3, transfers to Husky; beats BO in convergence. Shows sim-optimal ≠ real-optimal config. | Very strong: sim-augmented, budget-limited, transfer of nav configurations sim→real and cross-platform. | CURE does not calibrate the simulator; it transfers structure (causal graph). Open: does *calibrating the sim's transient dynamics* beat transferring causal structure? No abstention. | Shows an uncalibrated sim can still yield transferable structure — weakens the "need calibration" premise. |
| EffiTune (IROS 2025) | Diagnoses training inefficiency of APPLR-style tuners; targeted resampling. | Budget efficiency of tuners. | No dynamics fidelity, no abstention. | — |
| Contextual GP-UCB (NIPS 2011), SafeOpt (ICML 2015)/SafeOpt-MC, MetaBO (ICLR 2020), MF-ES (ICRA 2017) | Context-aware, safe, meta-transferred, and multi-fidelity BO for controller tuning with few real trials; MetaBO and MF-ES specifically use simulators as cheap sources. | Direct baselines for "limited trial budget" and "selection under context." MF-ES already treats sim as an inaccurate information source with learned correlation. | Whether a *better-calibrated* low-fidelity source improves MF-BO/MetaBO regret by more than the BO already corrects for. | MF-BO can learn sim–real offset itself; calibration may be redundant if real trials are allowed. MetaBO reported sensitive to trial count (secondary). |
| Tan18, Hwangbo19, PACE, WLeg-nav | Actuator models (analytic+latency or learned actuator nets) are critical for zero-shot transfer of agile locomotion policies. | Direct support for "transient actuator/timing models matter." | Evidence is for high-bandwidth RL locomotion policies (ms-scale). Unshown for low-bandwidth classical planners (DWB/MPPI at 10–20 Hz) on wheeled/marine robots, and for *selection* among configs rather than policy training. | Truong22: lower-fidelity sim + simple data-fit motion models transferred better for navigation. |
| Peng18, SimOpt, BayesSim, DROPO, AutoTuned, AdaptSim | DR and adaptive DR (posteriors over sim params from real data) yield robust policies; AdaptSim: task-driven adaptation beats SysID-matching. | Direct: "calibration + uncertainty" ≈ BayesSim/DROPO posterior; CRANE's uncertainty ≈ these posteriors. | Applied to RL policies/manipulation; not to Nav2 config selection, not to delivery-timing (ROS message latency/jitter) as a parameter, not to abstention. | AdaptSim & Truong22 argue matching dynamics is neither necessary nor sufficient; DR robust to "significant calibration error" (Peng18 abstract). |
| EIDA (arXiv Oct 2026) | Fitting command→motion/feedback response from target-robot data (no actuator reconstruction) improves nav transfer (Go2 20/20 vs 4/20). | **Closest recent overlap** for "execution-interface timing/response model improves navigation transfer" on wheeled (Jackal) + legged. | Learned RL policy, not classical config selection; no uncertainty/abstention; no marine. Preprint. | Suggests black-box response fitting may suffice without physically structured transient models. |
| Sim2Real Predictivity (RA-L 2020) | Defines SRCC; Habitat SRCC for success only 0.18 — sim rankings did not hold on robot; agents exploited sim collision dynamics. | Directly defines the right metric for "held-out configuration selection" (rank correlation sim vs real). | SRCC for classical Nav2 configs under calibrated vs uncalibrated timing — not measured. | Strong warning: fidelity improvements may not raise rank predictivity. |
| Fossen21, Sonnenburg13, Morel25, Lanxin18, Harris23 | Grey-box Fossen-model ID with thruster models (static vs dynamic propeller; dead zone; servo lag) from minimal instrumentation; R² > 0.94. | CRANE's vessel actuator calibration. | No link from identified thruster transients to navigation-config selection/abstention; dead zone usually neglected. | Morel25: dead-zone constants small enough to ignore; Sonnenburg13: first-order Nomoto often enough at speed — transients may be second-order effects. |
| VRX, HoloOcean, Isaac Lab, Gazebo | Platforms provide Fossen hydrodynamics (VRX), actuator-net/DC-motor models (Isaac Lab), sensor models; VRX coefficients hand-tuned to sea trials. | CRANE is not first to offer calibratable actuator models or USV sim. | None I found ships an end-to-end *calibration-to-sim → config selection with abstention* evaluation protocol across land+water. | Isaac Lab already supports actuator nets — "transient actuator model in sim" is not novel per se. |
| LiDARsim, Waabi-gap, TUM-shift, Kinect-noise, Kalibr | Raydrop/noise and secondary returns measurably reduce detection domain gap; depth noise is distance/angle-dependent; sensor time offsets are calibratable. | CRANE sensor delivery-timing and noise emulation. | Effect of sensor *latency/jitter* fidelity on Nav2 config ranking is unmeasured in what I found. | TUM-shift: some realism (downsampling) had no downstream effect — fidelity ≠ utility. |
| Selective classification, conformal under shift, RobustCP-STL, FallbackMPC, FIPER | Calibrated reject options/monitors with bounded miss/false-alarm under (bounded) shift. | CRANE's "abstention." | Abstention over *configuration recommendations* (i.e., "don't deploy any config / request more trials") with coverage–risk curves on held-out real sites — not found. | CP guarantees break under unbounded shift; robust CP needs a shift budget that small-data CRANE may not be able to estimate. |

---

## 3. Novelty verdict

- **Not novel alone:** (a) context-dependent nav-parameter selection (APPL family, CURE); (b) actuator/latency models for sim-to-real (Tan18, Hwangbo19, Isaac Lab); (c) posterior/adaptive DR (BayesSim, DROPO); (d) multi-fidelity/safe BO (MF-ES, SafeOpt); (e) USV grey-box ID (Fossen, Morel25).
- **Plausibly open (narrow):** whether *physically structured, calibrated transient actuation + message-delivery timing* in the simulator measurably improves **rank predictivity (SRCC/regret) of classical Nav2 configurations on held-out real conditions**, and whether the resulting **posterior predictive uncertainty supports calibrated abstention**, **under a fixed small real-trial budget**, **across two embodiments (wheeled + USV)**, compared against (i) MF-BO/contextual BO that learn the sim–real offset directly, (ii) adaptive DR posteriors, (iii) black-box response fitting (EIDA-style), and (iv) CURE-style structural transfer.
- **Key threat:** Truong22, AdaptSim, SRCC, and CURE together give a credible prior that higher fidelity may *not* improve decisions. CRANE's contribution is only as strong as an honest test that could come out null.

---

## 4. Falsifiable predictions (CRANE must pre-register)

P1. **Rank predictivity.** For a fixed pool of ≥20 Nav2 configs on the land robot, Spearman SRCC(sim, real) on held-out real sites is higher with calibrated transient actuator+timing model than with the steady-state/ideal model, by ≥0.2 with a bootstrap 95% CI excluding 0. *Falsified* if CI overlaps 0.

P2. **Budgeted selection regret.** At real-trial budgets B ∈ {0, 5, 10, 20}, selection using calibrated-sim prior + real trials has lower simple regret on held-out sites than MF-BO (MF-ES/MF-GP-UCB) using the uncalibrated sim as low-fidelity source. Prediction: advantage is largest at B ≤ 5 and vanishes by B = 20. *Falsified* if no advantage at B ≤ 5, or if the advantage does not shrink with B (which would suggest leakage/overfitting).

P3. **Timing-specific effect (ablation).** Removing only the delivery-timing (latency/jitter) component while keeping actuator transients degrades SRCC more for aggressive configs (high max_vel/acc, short controller horizon) than for conservative ones. *Falsified* if the degradation is uniform or absent — then timing is not the operative mechanism.

P4. **Steady-state sufficiency counter-test.** A steady-state model with *DR over delay and gain* (Peng18-style) matches calibrated-transient SRCC within 0.05. If true, the CRANE "calibrated transient" claim fails and DR is sufficient.

P5. **Abstention calibration.** Using posterior-predictive (calibration-posterior) uncertainty to abstain, the risk–coverage curve on held-out real conditions dominates abstention based on (a) BO GP posterior variance alone and (b) ensemble disagreement over an uncalibrated sim; and empirical failure rate among accepted recommendations ≤ the nominal α (e.g., 10%) at ≥50% coverage. *Falsified* if observed failure > α + CI or curve does not dominate.

P6. **Cross-domain.** The same pipeline on the USV (thruster lag + dead zone, Fossen-ID) shows a positive SRCC gain of the same sign as land. Prediction weaker: gain smaller where thruster time constants ≪ controller period. *Falsified* if sign flips.

P7. **Shift detection.** Under a deliberately induced shift (e.g., surface change, payload, added 50–100 ms network latency), the calibrated model's uncertainty increases and abstention rate rises before failures; uncalibrated baselines do not. *Falsified* if abstention rate is insensitive to the induced shift.

P8. **Null-fidelity check.** Calibrated sensor noise/latency (LiDAR raydrop, depth noise, IMU noise) adds ≤ small SRCC gain beyond actuator+timing. (Predict near-null — if large, sensor fidelity, not actuation, is the driver.)

---

## 5. Strongest baselines CRANE must beat (or match honestly)

1. **Hand-tuned Nav2 defaults + expert tuning** (BARN shows this is strong).
2. **Contextual GP-UCB / contextual BO** over configs with real trials only (Krause & Ong 2011).
3. **Multi-fidelity BO** with the *uncalibrated* sim as low-fidelity source (MF-ES, Marco et al. 2017) — the most direct threat: it learns the sim–real gap itself.
4. **Safe BO (SafeOpt/SafeOpt-MC)** for constrained selection with collision constraints.
5. **MetaBO / transfer BO** meta-trained in sim.
6. **Adaptive DR posterior** (BayesSim or DROPO) over a *steady-state* model, with delay as a randomized parameter (Peng18-style).
7. **CURE** (causal pruning in sim → optimization on robot).
8. **APPLR-style learned parameter policy** trained in uncalibrated sim.
9. **Black-box execution-response fit** (EIDA-style learned cmd→motion model, no physical structure).
10. For abstention: **GP-variance threshold**, **deep-ensemble disagreement**, **split-conformal on sim residuals**.

Fairness rules: same real-trial budget, same held-out sites, same config pool, tuned baselines (report their hyperparameter search budget), seeds ≥5, pre-registered metrics.

---

## 6. Claims CRANE must NOT make

- "First simulator with calibrated actuator models" (Isaac Lab, Tan18, Hwangbo19 contradict).
- "First sim-to-real navigation parameter tuning" (APPLR, CURE contradict).
- "First USV simulator with Fossen dynamics / identification" (VRX, Fossen, Morel25).
- "Higher fidelity ⇒ better transfer" as a general law (Truong22, AdaptSim, SRCC contradict).
- "Calibrated uncertainty guarantees safety" — no formal guarantee under unbounded shift; conformal claims need exchangeability or a bounded shift budget.
- "Physical fidelity validated" from public datasets/bags alone, from in-sim replay, or from single-site data; "held-out" must mean held-out *site/condition*, not held-out time slice of the same run.
- "Sim-to-real" anything without physical robot (or a clearly labelled real public dataset) trials on the decision metric.
- Any claim about LLM-based Nav2 tuners being "outperformed" — no verified primary source exists to compare against.
- Any number from Peng18 timestep randomization, Hwangbo "ideal model failed," VRX thruster lag, MOSAIC-SV, CPL (Huang ICRA 2024) without first reading the full text.

---

## 7. Unverified / flagged items
- MOSAIC-SV — no primary source found. **Do not cite.**
- Huang et al., "Conformal policy learning... under distribution shifts," ICRA 2024 — seen only in a reference list.
- Peng18 exponential timestep randomization range and ablation result — from memory/secondary.
- Hwangbo19 claim that ideal-actuator policies failed on hardware — secondary only.
- VRX thruster first-order dynamics — not confirmed.
- Venues for FallbackMPC (Sinha et al.), FIPER, Waabi-gap, WLeg-nav — not pinned.
- LLM-based Nav2 parameter tuning — no source found; category unverified.
- arXiv items dated 2026 (APPLV, EIDA, Costmap-energy) are preprints; re-check status before citing.
