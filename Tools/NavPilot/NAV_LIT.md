# NAV_LIT: high-speed under-canopy navigation versus first-detection range

Compiled 2026-11-09. This complements `Docs/DroneVisuals/mission/LITREVIEW.md` (B7, T3) and does not repeat it.
Tags: [V] read on the primary page or repo; [S] snippet only; [unverified] number not confirmed.
Correction: Bubble Planner was published at IROS 2022, not RA-L.

## (a) Table

| Method (ID) | Assumes | Top speed & setting | Evidence | Code |
|---|---|---|---|---|
| SUPER, Ren 2025 SciRob (10.1126/scirobotics.ado6187) | 3D LiDAR, 70 m range; FAST-LIO; onboard; known-free backup plus exploratory plan | >20 m/s real; 2.5 mm wires avoided [V abstract-level] | Real plus sim; 35.9× fewer failures [S]; trial count [unverified] | hku-mars/SUPER: planner preview, no control module, no license [V] |
| Bubble Planner, Ren 2022 IROS (arXiv 2202.12177) | LiDAR; no prior map; sphere corridors | >13.7 m/s, woods [V] | Real | None found |
| FASTER, Tordesillas 2022 T-RO (arXiv 2001.04420) | Depth to map; known-free/unknown split; MIQP | 7.8 m/s UAV [V]; forest density [unverified] | Sim plus real | mit-acl/faster, BSD-3, needs Gurobi [V] |
| RAPTOR, Zhou 2021 T-RO (arXiv 2007.03465) | Fixed limited-FOV depth; VIO; yaw planning; "safe reaction distance" near unknown space | [unverified]; indoor plus forest | Real, count [unverified] | Perception-aware part **unreleased**; Fast-Planner GPL-3.0 [V] |
| EGO-Planner-v2, Zhou 2022 SciRob (10.1126/scirobotics.abm5954) | Depth, VIO, MINCO | [unverified]; bamboo forest swarm. Zhang 2025: "degrades sharply" above ~3 m/s in sim [V] | Real swarm plus sim | GPL-3.0 [V] |
| Loquercio 2021 SciRob (arXiv 2110.05113) | Stereo depth; VIO; reference path; no map | 10 m/s real, 60% success [V] | 56 real runs; sim ablations | agile_autonomy GPL-3.0 [V] |
| Zhang 2025 NMI (arXiv 2407.10648) | D435i depth pooled to **16×12**; attitude only, no VIO; A53 CPU | 20 m/s real forest; 7 m/s urban [V] | Real odometry ablation 3 speeds ×10; sim 10 runs/speed [V] | HenryHuYu/DiffPhysDrone, training code, license not checked |
| Seeing Through Pixel Motion, Hu 2025 RA-L (arXiv 2411.04413) | Monocular flow; **offboard RTX 3070** | 6 m/s real; sim 90% at 8 m/s (GT flow), 60% (estimated flow) [V] | Sim 20 runs/method; real count not reported | None linked |
| Uncertainty-mask follow-up, Deng 2026 (arXiv 2606.09088) | Translational flow plus forward/backward consistency mask; offboard RTX 5090 | 11.79 m/s real, sparse forest; 13.91 m/s sim [V] | 30 real, 93.3%; sim mask ablation [V] | None linked |
| Falanga 2019 RA-L (10.1109/LRA.2019.2898117) | Point mass; deterministic detection at range R; latency t | Analytical | Theory plus event-camera tests | n/a |
| Radar extension, Mandaokar & Rinner 2026 (arXiv 2608.01855) | mmWave; IMM plus CBF; Pi 4B; 75 ms system latency | 5 m/s forward; **thrown balls only** [V] | 20 tethered, 39 outdoor, 390 throws [V] | Claimed [S]; no link on abstract page |
| RAPPIDS, Bucki 2020 RA-L (arXiv 2003.01245) | Single depth image; no map | [unverified] | Sim plus real | nlbucki/RAPPIDS |
| Lopez & How 2017 ICRA (10.1109/ICRA.2017.7989677) | Depth point cloud in FOV; no map | ~3 m/s [S] | Real | None seen |
| NanoMap, Florence 2018 ICRA (arXiv 1802.09076) | Sliding window of depth frames plus **pose uncertainty** | 10 m/s real [V] | Real | nanomap_ros |
| Ames 2017 TAC (10.1109/TAC.2016.2638961); 2019 tutorial (arXiv 1903.11199) | Known safe set h(x) | n/a | Theory | n/a |
| Backup CBF: Gurriet 2018 CDC (10.1109/CDC.2018.8619139); Chen 2021 CDC (arXiv 2104.11332) | Backup policy plus forward integration | n/a | Theory and sim | n/a |
| gatekeeper, Agrawal 2024 T-RO | Backup controller (hover); online map | [unverified] | Real quadrotor [S] | [unverified] |
| Predictive safety filter, Wabersich 2021 Automatica (arXiv 1812.05506) | Model plus uncertainty bounds; MPC to a terminal safe set | n/a | Sim | n/a |
| Shielding, Alshiekh 2018 AAAI (arXiv 1708.08611) | Finite abstraction plus LTL spec | n/a | Grid-world, sim | n/a |
| PAMPC, Falanga 2018 IROS (arXiv 1804.04811) | Keep features or points of interest in view | [unverified] | Real | uzh-rpg/rpg_mpc |
| PANTHER, Tordesillas 2022 Access (arXiv 2103.06372) | Keeps dynamic obstacles in FOV | [unverified] | Sim plus real | mit-acl/panther |
| CCNMPC, Zhu & Alonso-Mora 2019 RA-L (10.1109/LRA.2019.2893494) | Gaussian obstacle and ego uncertainty; chance constraints | Indoor, moving humans [V] | 2 real quads, 16 in sim | tud-amr/mrca-mav |
| Richter 2015 ISRR; Kahn 2017 (arXiv 1702.01182) | Learned P(collision \| action, speed) | Ground vehicles [S] | Speed modulated by risk or uncertainty | None seen |
| Latency: async perception-control (arXiv 2509.13816); delay-compensated NMPC (arXiv 2010.11264) | Fast IMU-rate policy with slow LiDAR features; predictor in front of NMPC | [unverified]; Crazyflie | Sim plus real [S] | [unverified] |

## (b) What transfers to our failure modes

- **Thin branches.** The detection-range evidence sits outside the planners.
  - Nature Communications 2025 (10.1038/s41467-025-62801-y) detected 4–10 mm wire nets only within **3.53 m** and 16 mm ropes within 7.10 m, using LiDAR on a quadrotor. This independently supports our ~4–5 m first-detection range.
  - Contradicting evidence: SUPER reports 2.5 mm wires at >20 m/s with a 70 m LiDAR. Our bottleneck may therefore be specific to the sensor (a depth camera), not general.
  - Zhang 2025 flies 20 m/s on 16×12 pooled depth, where a 3 cm branch at 5 m covers well under one pooled cell. Thin-branch exposure in their forests is therefore probably low and unquantified.
- **Stale map / drift.**
  - NanoMap is the only planner here that carries pose uncertainty into collision queries.
  - Zhang 2025 reports VIO drift causing tree collisions at speed, and the odometry-free policy did better.
  - FASTER and SUPER backups trust "known-free" space unconditionally. Drift silently invalidates that guarantee.
- **Occlusion.** RAPTOR's visibility-aware refinement and yaw planning target exactly this case, but that code was not released.
- **Compute stalls.**
  - Falanga 2019 and the radar paper treat latency as a constant. No work models stochastic stalls.
  - Backup-trajectory methods (FASTER, SUPER, gatekeeper) give the right structure: an already committed safe trajectory survives a stall.
- **Skeptical note on our pilot.** Falanga assumes *lateral* evasion at constant forward speed, not stopping. Dodging one 3 cm branch by ~0.3 m at 10 m/s² takes ~0.25 s. With R = 4.5 m and t = 0.3 s, this gives v ≈ 8 m/s, which is above our stopping-based ~5.4 m/s. Dense stands may remove that option, so the pilot must model both modes.

## (c) Strongest reproducible baselines

1. **FASTER (B7b).** BSD-3 code, a real-flight top speed (7.8 m/s), and an explicit known-free/unknown split, which is exactly the "trust" variable we manipulate. The Gurobi dependency is the cost.
2. **SUPER.** The strongest real-flight evidence (>20 m/s, millimetre wires), and the repo ships `benchmark_high_speed` and `benchmark_dense` launch files. Caveats: no license (reimplement only), LiDAR-only, and the control module is unreleased.
3. **Loquercio 2021 / Zhang 2025 (learned reactive).**
   - Loquercio has GPL-3.0 code and 56 real runs.
   - Zhang has public training code and 20 m/s real flight.
   - Treat their success rates as point estimates. The Wilson 95% CI for 60% success at n = 10 is about 31–83%.

## (d) Genuinely open gap

- **Speed versus P(collision) with confidence intervals under controlled perception degradation.**
  - I found none. Papers report success at 3–5 speed levels with 10–30 trials and no intervals: Loquercio; Zhang; Deng 2026 (single speed); arXiv 2512.17553 [S].
  - Degradation appears only as categorical ablations (ground-truth vs estimated flow; Vicon vs VIO vs none). No paper sweeps detection range, drift or latency and reports speed at a fixed risk.
  - Falanga 2019 is deterministic.
- **Range-dependent thin-obstacle detection probability used to set speed or trust.**
  - I found no aerial planner that does this.
  - Closest analogues:
    - Kahn 2017 and Richter 2015: learned risk modulates speed, but is not decomposed by detection range.
    - A cable-detection patent (US 7839322; not peer-reviewed): P_miss = (1 − P_d)^(d/v).
    - RAPTOR's geometric safe reaction distance.
  - Not found: any planner that discounts "known-free" cells by the per-range thin-obstacle miss probability.
- **Falsifiable predictions.**
  - P1: for every planner family, the speed at P(coll) ≤ 5% scales roughly linearly with the 50%-detection range for thin branches. Map-trust variants shift it by <10% at fixed range. A larger shift would reopen map trust as the primary lever.
  - P2: allowing lateral evasion raises the 5%-risk speed by ≥1.3× in sparse stands (≤0.1 stems/m²) but by <1.1× in dense ones.
  - P3: a FASTER/SUPER backup with 5 m depth range stays ≤5.5 m/s at 5% risk. Doubling the range to 10 m raises it to ≥8 m/s.
