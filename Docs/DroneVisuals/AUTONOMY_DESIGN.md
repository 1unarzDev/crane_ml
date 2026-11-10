# Onboard autonomy design note: two-quad forest SAR

_A1, 2026-11-10. Design note only: nothing here is built or flown. **[U]** = unverified. "Gate" numbers are proposals for the user to accept or change._

**Hardware:** AOS 7 V5, F90, 6S, Cube Orange running PX4 (uXRCE-DDS), Orin Nano Super, OAK-D S2, Unitree L2, pMDDL900 (2 air nodes + ground), FrSky RC.

## 0. What carries over from astro_dock (read-only)

astro_dock (`/home/lunarz/astro_dock`) runs ROS 2 Jazzy with `mhseals_nav`, `mhseals_hardware`, `mhseals_testing` and `ros_tcp_endpoint`. Its FC is ArduRover, reached through MAVROS.

| Pattern | Carries over? |
|---|---|
| Device profiles (`config/devices/*.yaml`, `device.launch.py`) assign parts to computers | Yes: `air`, `ground`, `sim` |
| ROS-free, clock-injected arbiter `command_arbitration.py` (rc > manual > hold > auto) | Yes |
| Hold-only watchdogs that never release themselves (`hold_watchdog.py`) | Pattern yes. Action no: a quad cannot hold by cutting thrust |
| `fcu_watchdog`/`ekf_input_watchdog`: EKFs keep publishing fresh stamps after their inputs die, so check **input receipt age** | Yes, and it is critical for LIO |
| FCU failsafes disabled (`FS_GCS_ENABLE=0`, `FENCE_ENABLE=0`) | **No.** PX4 must own every failsafe that needs no companion |
| chrony LAN server (`setup.linux.sh`), 2 s stamp bound | Mechanism yes; bound tightens to 5 ms |
| MAVROS, dual EKF, Nav2/MPPI on 2D costmaps | No: use PX4 DDS and 3D planning |
| DLIO configured (`config/dlio.yaml`) | Candidate estimator |
| `log.launch.py`: `ros2 bag record -a` | No; record a selective list |
| CRANE ROS-TCP sensor contract (`CraneRoboBoatSensorContract.cs`, `fcu:=false`) | Yes for sensors |

CRANE constraints (this worktree's `README.md` and `Docs/SimulationPhysicsAndRuntimeModes.md`):
- ROS-TCP is a serialization boundary, and there is **no lockstep barrier**.
- The only FC bridges are an ArduPilot JSON fixture and an aerial direct-PWM fixture. Neither qualifies PX4.
- `PX4_LOCKSTEP_PLAN.md` and `PHYSICS.md` **did not exist yet**, so §4 assumes their content [U].
- `Tools/Aerial/profiles/aos7_v5_f90_6s.json` budgets 1.237 kg (ESTIMATED) with an 8 g F4 FC. It has **no Cube Orange, Microhard or receiver**, so stopping distances are untrustworthy until it is fixed.

## 1. Layers

| Layer | Owns | Never |
|---|---|---|
| **Cube/PX4** | Rate, attitude, position and velocity control; EKF2 fusing Jetson odometry (`vehicle_visual_odometry`); failsafes that must work with the Jetson dead; geofence; RC override | Mission logic |
| **Jetson** | Estimation, 3D map, planning, trajectory, setpoint arbiter, perception, supervisor and speed governor, comms agent | Stabilization |
| **Ground** | Pre-launch default plan, reassignment on beacon loss, detection fusion, operator UI | Real-time control |

Invariant: **any single failure leaves a stabilized aircraft the pilot can take on FrSky.** RC goes straight into the Cube, never through the Jetson or the Microhard.

Parameter names are from https://docs.px4.io/main/en/config/safety.html. Defaults and exceptions are [U] on the flashed firmware.

| # | Failsafe | Owner | Trigger | Action | Test |
|---|---|---|---|---|---|
| 1 | Pilot override | PX4 | Mode switch / `COM_RC_OVERRIDE` | Position or Stabilized | SITL; every field session |
| 2 | RC loss | PX4 | `COM_RC_LOSS_T` | Hold, else descend. `COM_RCL_EXCEPT` for Offboard is **D6** | Transmitter off: SITL, tether |
| 3 | Offboard loss (Jetson or arbiter dead) | PX4 | `COM_OF_LOSS_T` | `COM_OBL_RC_ACT`: Hold, then Land | Kill arbiter: SITL, HIL, field |
| 4 | Position invalid | PX4 EKF2 | EV timeout or innovation reject | Altitude, then descend (no GPS) | Cut EV: SITL, HIL |
| 5 | Geofence and ceiling | PX4 | `GF_*`; needs an EKF origin when GPS-denied [U] | Hold | SITL |
| 6 | Battery | PX4 floor; Jetson reserve | `BAT_*`; energy to home | Land; Jetson retraces home first | SITL, bench |
| 7 | Attitude divergence | PX4 | `FD_FAIL_R/P` | Termination [decide] | SITL only |
| 8 | Estimator degeneracy | Jetson | Hessian eigen floor, covariance growth, input age, LIO and PX4 odometry divergence | Zero velocity, then retrace free path or slow descent | CRANE corridors and canopy gaps; bag faults |
| 9 | Map stale | Jetson | Age > 2× period | Governor to 0 | Delay injection |
| 10 | Stopping margin < 0 | Jetson | §4 | Brake, replan | CRANE pop-up obstacles |
| 11 | Ground link loss | Jetson | Ground beacon missing > T₁ | Continue default plan; after T₂, go home on the onboard map | Radio off: CRANE, field |
| 12 | Partner lost | Ground (+ onboard) | Partner beacon missing > T₁ | Priority-order reassignment | CRANE, one agent killed |
| 13 | Compute or thermal | Jetson | Deadline misses, `tegrastats` | Shed perception, then speed | Hot bag replay |

## 2. Interfaces

**Jetson ↔ PX4** runs over uXRCE-DDS (https://docs.px4.io/main/en/middleware/uxrce_dds.html; https://github.com/PX4/px4_msgs) on serial or Ethernet [U which, on this carrier].
- In:
  - `/fmu/in/offboard_control_mode` and `trajectory_setpoint` at 50–100 Hz
  - `vehicle_visual_odometry` at ≥ 30 Hz
  - `vehicle_command`
- Out: `/fmu/out/vehicle_odometry`, `vehicle_status`, `failsafe_flags`, `battery_status`.
- Consider https://github.com/Auterion/px4-ros2-interface-lib so PX4 owns entry into and exit from a registered mode [U version fit].
- Frames: the planner works in the LIO map frame and setpoints go out in PX4 local NED. One node owns map→NED and publishes their divergence, which feeds failsafe 8.

**Onboard graph** (one domain, never bridged over the radio):
- L2 via `unilidar_sdk2` (https://github.com/unitreerobotics/unilidar_sdk2 [U ROS 2]).
- OAK-D via `depthai-ros` (https://github.com/luxonis/depthai-ros).
- Pipeline: estimator → map → planner → arbiter (RC > supervisor brake/hold > planner) → PX4.

| Stage | Budget | p99 gate |
|---|---|---|
| L2 acquisition → ROS cloud | 15 ms | 25 ms |
| LIO update | 20 ms | 35 ms |
| Map insert | 15 ms | 30 ms |
| Replan | 30 ms | 50 ms |
| Arbiter → PX4 receipt | 5 ms | 10 ms |
| **Sensor-to-setpoint age** | 85 ms | **≤ 120 ms** |

**Time sync.**
- PX4 uXRCE-DDS timesync handles the FC.
- The Jetson runs chrony for onboard Ethernet. Use PTP only if the Orin NIC and the L2 support hardware stamps [U].
- The OAK-D uses depthai host sync.
- Gate: cross-source skew < 5 ms, measured with an LED seen by the camera against IMU taps.
- Over the air, chrony is for log alignment only. Freshness is judged by **receipt age at the receiver**.

**Radio.** ForestSearch (`crane-forest-sar/Tools/ForestSearch/STATUS.md`) found:
- A shared default plan plus a ~10 B health beacon recovers nearly all of messaging's value.
- **Fresh messages layered on a stale in-flight partner model cost 3–6 points** unless a priority-order rule is used.

Evidence is limited to a grid abstraction, synthetic links and 2–4 agents. So:
- Before launch, the ground pushes a deterministic default plan (cells, order, ranks, seed) that each aircraft can recompute for its partner.
- In flight, partner models change only on health events.

| Message | Dir | Payload B | Rate |
|---|---|---|---|
| HEARTBEAT (id, seq, mode, battery, est-ok, cell) | air → all | 10 | 1 Hz, periodic because links are bursty |
| POSE (int16 dm, yaw, age) | air → ground | 12 | 1 Hz |
| COVERAGE delta (RLE cells) | air → ground | ≤ 48 | 0.2 Hz |
| DETECTION (class, conf, xyz, cov, t) | air → ground | 24 (+ thumbnail ≤ 4 kB on request) | Event |
| LEASE (cell, holder, expiry) | ground → air | 8 | On reassign |
| GROUND beacon | ground → air | 6 | 1 Hz |

- That is under 100 B/s per aircraft before framing.
- Default: **compact fixed-format UDP over the Microhard IP link**, with sequence numbers and expiring leases.
- Alternative: Zenoh (https://zenoh.io; https://github.com/ros2/rmw_zenoh).
- Settle with a pMDDL900 forest-range test of goodput, p99 latency, loss-burst length and per-message overhead. Zenoh wins only at under 2× overhead.
- **Never run DDS discovery over the radio.** Any QGC/MAVLink over the Microhard must be optional; set `NAV_DLL_ACT` deliberately.

## 3. Candidates and deciding tests (none chosen)

**Estimator.**
- Candidates:
  - FAST-LIO2 (https://github.com/hku-mars/FAST_LIO; https://arxiv.org/abs/2107.06829)
  - Point-LIO (https://github.com/hku-mars/Point-LIO)
  - DLIO (https://github.com/vectr-ucla/direct_lidar_inertial_odometry)
  - OpenVINS (https://github.com/rpng/open_vins) or VINS-Fusion (https://github.com/HKUST-Aerial-Robotics/VINS-Fusion) on the OAK-D
  - LIO with a VIO fallback for degeneracy
- Prior, not evidence: LIO is lighting-invariant, while VIO suffers at shadow transitions.
- Tests:
  - Drift per 100 m on CRANE canopy and on a public forest set (Wild-Places, https://csiro-robotics.github.io/Wild-Places/ [U fit to L2])
  - Sun-to-shadow cuts
  - Orin CPU and p99 latency with perception running
  - Degeneracy detection and recovery
- Confirm the L2 scan pattern against each LIO's scan model [U].

**Map.**
- Candidates: ikd-tree (https://github.com/hku-mars/ikd-Tree), ROG-Map (https://github.com/hku-mars/ROG-Map), voxblox (https://github.com/ethz-asl/voxblox), nvblox (https://github.com/nvidia-isaac/nvblox).
- Tests: insert p99; memory within 8 GB shared; recall of 2 cm branches at 5 m; whether the planner needs ESDF.
- nvblox contends with the detector for the GPU.

**Planner.**
- Candidates:
  - EGO-Planner-v2 (https://github.com/ZJU-FAST-Lab/EGO-Planner-v2; Sci. Robot. 2022)
  - SUPER (https://github.com/hku-mars/SUPER [U venue])
  - Bubble (https://arxiv.org/abs/2202.12177) or FASTER (https://github.com/mit-acl/faster)
  - MPC with an **age-aware governor**, v_max = f(free distance, p99 age, covariance), which sits after any planner
- Tests: margin violations, success and speed on held-out seeds, replan p99, and behaviour in unknown space (SUPER and FASTER keep a backup trajectory in known-free space).

**Controller.**
- Default: offboard `TrajectorySetpoint` with position, velocity and acceleration feedforward (https://docs.px4.io/main/en/flight_modes/offboard.html).
- Body-rate offboard is justified only if all three hold:
  - Tracking error at target speed exceeds the clearance budget.
  - The Jetson loop p99 is under 10 ms.
  - A fallback to PX4 hold is demonstrated.
- Unlikely at 2–4 m/s [U].

**Learned policy.**
- Examples: differentiable-physics flight (https://arxiv.org/abs/2407.10648 [U venue]) and privileged-expert imitation (https://doi.org/10.1126/scirobotics.abg5810).
- Justified only if the classical stack fails a gate that the policy passes on held-out seeds and on real-bag replay.
- Wrapper:
  - The policy outputs a velocity or trajectory into the same arbiter.
  - The governor and margin check stay classical.
  - An out-of-distribution monitor triggers hold.
  - PX4 failsafes are unchanged.
  - Never rate-level.

## 4. CRANE test ladder

| Stage | Proves | Pass gate |
|---|---|---|
| 1. Scripted flight into the `Tools/Aerial` parity plant | Plant, frames, stamps | Parity green; monotonic stamps; sensor timing within spec [U] |
| 2. Simple CRANE-side controller, ROS-TCP sensors | Estimator and map in a loop | **Drift ≤ 1.0 m and ≤ 2° per 100 m** under canopy, held-out seeds |
| 3. PX4 SITL lockstep (HIL_SENSOR/actuator interface; needs a barrier) | PX4 failsafes and offboard | Rows 1–7 fire correctly; tracking p95 < 0.3 m at 3 m/s |
| 4. Bag replay on the Jetson, full stack | Timing tails | **p99 sensor-to-setpoint ≤ 120 ms**, p99.9 ≤ 200 ms; no miss bursts > 3; thermally steady after 20 min |
| 5. HIL (Cube + Jetson + CRANE plant) | Wiring, timesync, arbiter | Skew < 5 ms; rows 3, 4, 8 and 11 pass |
| 6. Field: tethered → open → sparse → dense forest | Reality gap | Same gates; RC takeover drill; drift against surveyed markers |

Gates for every closed-loop stage:
- **Stopping-distance margin:** m = d_free − (v·t_age,p99 + v²/2a_brake + r_body + 0.5 m) ≥ 0 at every tick, across 100 held-out missions. a_brake comes from the corrected mass budget.
- **False-abort rate:** ≤ 1 per 20 fault-free missions, and **0 missed aborts** on the injected-fault suite.
- **Coordination:** after a partner is killed, coverage lands within 0.05 P(detect) of full messaging.

## 5. Next decisions, in order

| # | Decision | Settled by |
|---|---|---|
| D1 | Mass and inertia budget (Cube, Microhard, receiver, carrier) | Weigh the build; fix the profile |
| D2 | Radio protocol and topology | pMDDL900 range and burst test |
| D3 | Estimator | Stage 2 drift + stage 4 tails |
| D4 | Map | Insert p99, memory, branch recall |
| D5 | Planner and governor | Stages 2–3 margin and success |
| D6 | Offboard RC and data-link loss (`COM_RCL_EXCEPT`, `NAV_DLL_ACT`) | Safety review of failsafe 11 beyond RC range |
| D7 | Rate-level control? | Stage 3 tracking against clearance |
| D8 | Learned policy? | Only if a D5 gate fails |

## 6. What not to do yet

- No learned or rate-level control before the stage 3 and 4 gates exist.
- No DDS over the 900 MHz link, and no rich in-flight partner messaging.
- No `bag record -a` on the Orin.
- No disabling PX4 failsafes, astro_dock-style.
- No claim of PX4 lockstep in CRANE until the barrier exists.
- No estimator choice from papers alone.
- No forest autonomy before the open-area failsafe drills pass.
- No edits to astro_dock or any MHSeals repository; copy patterns into new drone packages.
