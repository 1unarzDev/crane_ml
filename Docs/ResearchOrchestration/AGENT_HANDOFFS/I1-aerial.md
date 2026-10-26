# I1: aerial-domain specialist (multirotor sim-to-real)

**Scope.** Repo `/home/lunarz/crane_ml` at `976ff1e` plus uncommitted user work. I modified no existing files. Everything new is under `Tools/Aerial/` and this file.

**Method.**
- Static reading of the C#, scene YAML and project settings.
- One headless run of the *existing* player `Builds/CRANE-Worker/CRANE.x86_64`. It was built 2026-09-19, so it is stale relative to the working tree. The run used `--crane-aerial-validation`, wrote only to `/tmp/crane-aerial-run/`, used no ROS and started no SITL bridge.
- Python counterexamples (`python -I`).
- Upstream files were fetched one at a time into `/tmp/crane-aerial-src/<name>/` (hashes are below). They were read as data and never executed.
- `MAVROSConnection` was never started. No flight controller or hardware was touched.

**Labels.** Evidence labels follow `CONTRACTS.md`. Defects are tagged **VERIFIED** (numerically demonstrated from source-mirrored formulas or primary sources) or **HYPOTHESIS**.

## 0. Top findings

| # | Finding | Where | Label | Counterexample / evidence |
|---|---|---|---|---|
| A1 | The ArduPilot-JSON SITL telemetry is wrong in sign and frame. Position and velocity "down" is +altitude. Roll and pitch angles and rates have reversed signs. Gyro is world-frame instead of body FRD. `accel_body` is the constant (0,0,-g). Timestamp is `float32`. Telemetry is sampled in `Update()`, not at the physics tick. | `Utils/MAVROS/MAVROSConnection.cs:169-199`, `:28` | VERIFIED | `test_ardupilot_json_signs_and_frames`: 5 m altitude is sent as D=+5 (spec: −5). Roll +10° is sent as +0.175 rad (spec: −0.175). With 90° heading a world yaw rate is sent as body roll rate 0.5 (spec: 0). Spec: ArduPilot `libraries/SITL/examples/JSON/readme.md` @`9c412ba` (sha256 `cfc34793…`). |
| A2 | Servo order and spin pattern do not match any autopilot. CRANE maps channels 0-3 to FL,FR,RR,RL. ArduCopter QUAD/X is FR,RL,FL,RR. CRANE spins FL/RR CCW, the mirror image of PX4 x500 and ArduCopter (FR/RL CCW). Connecting a real autopilot stack would flip the vehicle on arming. | `MAVROSConnection.cs:212-218`; `MultirotorDynamics.cs:33` | VERIFIED | `AP_MotorsMatrix.cpp` @`9c412ba` (sha256 `be933820…`) lines 593-600; x500 `turningDirection` per rotor. `test_ardupilot_servo_order_and_crane_spin_pattern_mismatch`. |
| A3 | The shared `Imu.cs` publishes gyro as a polar vector in the world frame, so all axes have the wrong sign. It has no linear acceleration at all. (Same as truth-inventory V4.) The aerial vehicle has **no sensors at all**. | `Sensors/Nav/Imu.cs:35`; the aerial scenes contain only `MultirotorDynamics` | VERIFIED | `test_imu_gyro_uses_polar_conversion_so_signs_flip`. The package's own `FLU.ConvertAngularVelocityFromRUF` negates; `Imu.cs` doesn't use it. |
| A4 | Physics runs at 50 Hz. Fixed dt is 2822399/141120000 = 0.0199999993 s. This is coarse for multirotors: x500 τ_up = 12.5 ms, PX4/ArduCopter rate loops run at 400 Hz or more. | `ProjectSettings/TimeManager.asset` | VERIFIED | dt > τ_motor(x500). An external 400 Hz autopilot would be under-sampled by 8×. |
| A5 | Quadratic drag is applied per body axis (`v_i·|v_i|`), which depends on basis/heading. With equal x/z coefficients, 5 m/s at 45° gets 20 % less drag than at 0°, and flow at 30° is deflected by more than 3°. | `MultirotorDynamics.cs:128-141` | VERIFIED | `test_per_axis_quadratic_drag_depends_on_airflow_heading` (ratio 0.798) |
| A6 | The mixer clamps each motor independently with no axis priority or desaturation. At collective 0.95 with full roll, it delivers 58 % of the requested roll torque and silently drops collective. | `MultirotorDynamics.cs:75-86` | VERIFIED | `test_mixer_clamps_without_priority_and_loses_authority` |
| A7 | Gust phase uses absolute `Time.fixedTime`, and `ResetEpisode` does not rewind it. In-place resets with gusts are therefore not episode-reproducible, which violates the reset-isolation contract. | `MultirotorDynamics.cs:124-126` | VERIFIED | `test_gust_phase_depends_on_absolute_time_not_episode_time` |
| A8 | The documented PX4-windy displacement (3.267, 4.732) m in Unity x/z is impossible under the serialized pure-translation drag model. That model gives (2.80, 0.77) m: z ≪ x. | `Docs/ReferenceEnvironments.md` §PX4 windy | VERIFIED inconsistency, cause HYPOTHESIS (attitude drift, a different build, or swapped fields) | `test_documented_windy_displacement_contradicts_drag_model`. No retained windy JSON exists under `PerformanceResults/`. |
| A9 | The validation gates are sign-only. Attitude passes if \|ω\| > 0.1 rad/s after an open-loop step. Saturation passes trivially because the command is `SetCommand(2,…)`, which clamps to 1. Hover passes trivially because the rotors are preset to analytic hover and there is no disturbance. | `CraneAerialValidationRunner.cs:169-186` | VERIFIED (code reading) | Stale-build live run: roll 4.71, pitch 4.68, yaw 0.23 rad/s; all `valid=true`. |
| A10 | A Python mirror reproduces the live roll rate to 1e-6 and pitch to 0.6 %, but yaw is off by 22 % (mirror 0.179 vs live 0.231 rad/s). | stale build vs working tree | HYPOTHESIS: the stale binary predates the current yaw path, or PhysX inertia differs | `test_mirror_reproduces_live_roll_and_pitch_but_not_yaw`. A rebuild and rerun decides it. |

## 1. Audit

### Rotor and airframe model (`MultirotorDynamics`, all four aerial scenes)

| Item | Status |
|---|---|
| Thrust ∝ ω² | **Partial.** `T = 8 N · s²`, where `s ∈ [0,1]` is a *normalized command*, not an RPM. There are no kf or ω units and no PWM→ω→T curve. |
| Motor time constant | First-order, symmetric, τ = 0.06 s. Exact ZOH discretization (`1-exp(-dt/τ)`). There are no separate up/down constants. |
| Drag (reaction) torque | Yes: `yawSign · T · 0.015 m`, applied about the body up axis. The spin pattern is mirrored (A2). |
| Blade flapping / induced (rotor) drag | **Absent.** Horizontal drag comes only from body linear and quadratic drag, which is not proportional to rotor speed. This is the dominant real-quad effect (Martin & Salaün 2010; gz `rotorDragCoefficient`). |
| Rolling moment | Absent. |
| Ground effect | Absent. |
| Battery / voltage sag | Absent. |
| Rotor inertia / gyroscopic terms | Absent. |
| Mass and inertia | 1.5 kg. Inertia is PhysX's implicit tensor of one 0.5×0.16×0.5 m BoxCollider (`m_ImplicitTensor: 1`): FLU diag (0.0345, 0.0345, 0.0625) kg m². This is confirmed by the roll replay matching the live run. |
| Frames | Unity RUF left-handed, rotors at ±0.23 m, FL/FR/RR/RL. PX4 reference environments map Gazebo ENU→Unity as (x,z,y), which is consistent. |
| Integration | PhysX semi-implicit Euler at 50 Hz (A4). No substepping, no interpolation, `CollisionDetection: 2`. |

### Autopilot coupling

- **PX4 SITL/HITL: none.** The "PX4" scenes are geometry-only reconstructions of `PX4-gazebo-models` worlds. Their own docs say this is no claim of PX4 equivalence.
- **ArduPilot JSON SITL.** `MAVROSConnection` is a UDP servo→plant bridge. It is only exercised by `Tools/Performance/run_aerial_sitl_fixture.sh` with a Python fake (constant PWM 1680). It has never run against ArduPilot and would fail per A1 and A2.
- **Lockstep is not implemented.** Telemetry is free-running on a 200 Hz thread. ArduPilot's readme lets the backend ignore the frame rate, but the 50 Hz physics means each state is sent about 4× unchanged.

### Sensors on the aerial vehicle

None: no IMU, baro, mag, GNSS, optical flow, or onboard camera. The scenes have one spectator camera. Shared components (`Imu.cs`, `GPS.cs`, depth cameras) exist, but none are attached, and `Imu.cs` is defective (A3).

### Enabled vs. dead code

| Component | Status |
|---|---|
| `MultirotorDynamics`, `CraneAerialValidationRunner` (flag `--crane-aerial-validation`), `CraneReferencePx4{Walls,Aruco,Windy}` | Live in 4 scenes. |
| `WindForce.cs` | **Not aerial.** It is a `[RequireComponent(Buoyancy, GeneralDynamics)]` per-triangle hull wind model for boats, unused by the multirotor, which has its own wind (`SetWind`). `wind.direction` feeds `Sin/Cos` directly, so radians are implied (unverified). |
| `MAVROSConnection` aerial path | Reachable only via `--crane-aerial-sitl-validation` (`CraneWorkerOverrides.cs:92-105`). |

### Retained evidence

| Item | Label |
|---|---|
| Aerial JSON result | None under `PerformanceResults/`. Only `train-cpu-aerial-*` throughput benchmarks exist. |
| Stale-build headless run (this session) | `valid=true`, hover drift 0, vertical-accel error 1.8 %. **LIVE_SIM_VALIDATED (stale build)** for the gate as written; the gate itself is weak (A9). |
| Docs: PX4 walls/aruco/windy | IMPLEMENTED/TESTED per the docs. The windy numbers are inconsistent (A8). |

## 2. Reference vehicles and datasets (licenses checked at the primary source)

| Source | Version | License (primary) | GPL-3.0 use | Coefficients (provenance) | Mesh |
|---|---|---|---|---|---|
| PX4-gazebo-models **x500** | `e5997f4` (main, 2026-10-04) | BSD-3-Clause (repo LICENSE sha `5fcb2a52…`; `models/x500/LICENSE` sha `cee4ef94…`, Rudis Labs) | Compatible | m = 2.0643 kg (5 links); I = diag(0.021667, 0.021667, 0.04); kf 8.54858e-6; km 0.016; τ_up/down 0.0125/0.025 s; ω_max 1000 rad/s; λ1 8.06428e-5; μ1 1e-6. **SIMULATOR_DEFAULT**: identical to RotorS and not a published sysid of the x500. | No per-mesh notice; BSD by directory. Not imported. |
| RotorS **firefly** (hex), **hummingbird** | `cd813b7` | Apache-2.0 (file headers; `package.xml` "ASL 2.0"; no top-level LICENSE) | Compatible (one-way) | firefly: m 1.5 (+6×0.005), I = diag(0.0347563, 0.0458929, 0.0977), arm 0.215, same kf/km/τ, ω_max 838. hummingbird: m 0.68, arm 0.17, I = diag(0.007, 0.007, 0.012). SIMULATOR_DEFAULT. | UNKNOWN; do not import. |
| bitcraze **crazyflie-simulation** | `7e93752` | MIT (sha `47b665f1…`) | Compatible | I = diag(1.657171e-5, 1.6655602e-5, 2.9261652e-5); km 0.005964552 (both attributed to Förster 2015); kf 1.28192e-8; ω_max 2618. | Bitcraze Blender meshes inside the MIT repo: best import candidate. |
| Bitcraze **PWM-to-thrust** page (2015 table) | retrieved 2026-10-20 | Web docs; facts cited | Facts-only | Fitted **kf = 2.218e-8 N/(rad/s)²** (2.9 % RMS); **R_int = 0.163 Ω**, V_oc 4.035 V; η ≈ 0.565. The gz kf is 0.58× this: unresolved conflict. | – |
| Crazyflie 2.1 datasheet Rev 3 | sha `dea768dc…` | Vendor datasheet | Facts-only | 29 g takeoff, 92 mm, 250 mAh, BMI088 IMU, BMP388 baro. | – |
| Förster 2015 thesis (doi 10.3929/ethz-b-000214143) | – | Thesis | Facts-only | **UNVERIFIED_AT_PRIMARY.** The PDF fetch was refused (HTTP 429, then 500). | – |
| Landry 2015 (MIT thesis) | – | – | – | **NOT_RUN** (not retrieved this session). | – |
| gz-sim `MulticopterMotorModel.cc` | `3984434` (gz-sim8) | Apache-2.0 | Compatible | Force law, read only. | – |
| Agilicious | `50bba45` | **Academic-use license agreement** (README → rpg.ifi.uzh.ch PDF) | **Not compatible**: do not port code. | – | – |
| Flightmare | `d4218ae` | Code MIT. Renderer/assets: separate terms (already AUDITED in `ReferenceEnvironments.md`). | Code only | – | Do not import. |
| **Blackbird** dataset | repo `8f02a20` | Repo MIT. **Data license not stated**; `blackbird-dataset.mit.edu` gave DNS ENOTFOUND today. | Local validation only, never vendor | 360 Hz mocap, 100 Hz IMU, ~190 Hz motor RPM (arXiv 1810.01987). Vehicle coefficients not published in the repo. | – |
| **UZH-FPV** | – | **CC BY-NC-SA 3.0** (fpv.ifi.uzh.ch) | Not GPL-compatible for redistribution; local research use only | Laser-tracker ground truth, no RPM. | – |
| idsia **nanodrone-sysid-benchmark** (CF 2.1 *Brushless*) | `2d921b5` | **No license file** (all rights reserved by default) | Ask the authors before use. Note: this is a different vehicle from CF 2.1 brushed. | – | – |
| PX4 **iris** | – | Not checked (it lives in `PX4-SITL_gazebo-classic`, not `PX4-gazebo-models`) | NOT_RUN | – | – |

**Takeaways.**
- No GPL-compatible source gives a *measured* sysid for a ~1.5–2 kg quad. PX4 and RotorS numbers are shared defaults.
- The only measured, license-clean numbers found are Bitcraze's Crazyflie table, plus Förster's values via Bitcraze's MIT model, which are unverified at the primary source.
- For a large quad, the honest path is to identify from Blackbird RPM+mocap logs, not to cite the x500.

## 3. Test design

### Analytic fixtures (implemented in Python, FIXTURE_VALIDATED)

| Fixture | Tests |
|---|---|
| Hover T = mg, torque-free and drift-free | all 4 profiles |
| Motor step reaches 63.2 % at τ_up / τ_down | 1 |
| Yaw from reaction-torque balance (sign and magnitude) | 1 |
| Yaw equivariance with wind and drag | 3 seeds |
| Full SO(3) equivariance with g = 0 | 1 |
| Free fall: energy and world angular momentum conserved (three-axis tumbling) | 1 |
| Drag-only dissipation | 1 |
| Quad allocation: exact yaw-authority boundary ±0.1 % | 1 |
| Hex zonotope feasibility: sound over 200 samples, tight on 50 support points | 1 |
| Hex PNPNPN one-rotor failure: hover margin = 0 (reproduces Du et al., JGCD 2015) | 1 |

### Wind/gust frame tests (implemented)

- Air-relative Galilean equivalence.
- Downwind push independent of heading at 0 / 0.7 / 90° / 2.5 rad.
- Rotor drag ∝ ω.
- Gust episode-time dependence (CRANE defect A7).

### Identifiability-failure case (implemented)

1. **Lag vs. latency.** With multisine excitation at 0.2–0.9 Hz, the Fisher correlation of (τ_motor, gyro delay) is below −0.99. At equal SNR, the τ CRB is more than 3× larger than with 0.5–30 Hz excitation. Adding a motor-RPM channel shrinks the τ CRB by more than 10×.
2. **Motor lag vs. IMU low-pass.** A first-order motor lag and a first-order IMU low-pass are *exactly* interchangeable from gyro alone: (0.03, 0.01) and (0.01, 0.03) give identical traces to 1e-3. The RPM channel separates them.

**Consequence.** Any CRANE motor-τ "calibration" from IMU alone is unfalsifiable. Require RPM (Blackbird) or a thrust stand.

### Environments (proposed; no Unity assets created)

| Environment | Design |
|---|---|
| (a) Indoor flight arena | 10×10×4 m. Truth pose at 360 Hz mocap rate, plus a separate noisy estimator topic. Blackbird-like geometry, enabling a RPM-driven open-loop replay. |
| (b) Walls / ArUco | Existing. Add an onboard camera and ArUco detection (currently NOT_RUN). |
| (c) Windy outdoor | Existing. Add yaw-equivariance (wind at 0/45/90° relative to heading must give a rotated response), Dryden turbulence with a seeded episode clock, and wind as a NED field so it can be passed to the ArduPilot JSON `velocity_wind`. |
| (d) Ground-effect landing pad | 1 m pad with hover OGE command at h/R ∈ {0.5, 1, 2, 4}. The measured thrust ratio must follow Cheeseman–Bennett within tolerance, or a user-supplied measured curve. This also covers descent, ground contact and bounce. |

### Public-dataset replay plan (Blackbird)

1. Pick 2–3 slow sequences and download them manually after confirming the data terms. Store them outside the repo.
2. A reviewed adapter converts the bag to arrays: t, mocap p/q, RPM×4, IMU.
3. `aerial_replay.thrust_replay_report` fits the kf scale and reports the thrust residual and the lateral specific force (the rotor-drag signature).
4. Identify km from yaw transients and inertia from rate response, using train flights.
5. Score on held-out flights: RPM-driven open-loop prediction of ω and v at multiple horizons (0.05 / 0.2 / 1 s), comparing the reference model, the CRANE plant, and the CRANE plant with fixes.
6. Preregister thresholds in `PREREGISTRATION.md` before running.

**Falsifier.** If the rotor-drag term does not reduce the lateral residual on held-out flights, drop it.

## 4. Prototype: `Tools/Aerial/`

`python -I -m pytest Tools/Aerial -q` → **43 passed** (Python 3.14.8, numpy 2.5.4).

| File | Content |
|---|---|
| `aerial_model.py` | Rigid body with RK4 and full inertia; asymmetric motor lag; kf ω² thrust; reaction torque; gz-form rotor drag and rolling moment; rotation-consistent body drag; Cheeseman–Bennett ground effect; Thevenin battery with ω_max sag |
| `aerial_allocation.py` | Effectiveness matrix; exact zonotope feasibility (no LP); null-space allocation |
| `aerial_frames.py` | RUF↔FLU/FRD/NED/ENU for polar vectors, pseudovectors and rotations; Unity Euler |
| `aerial_identifiability.py` | Multisine excitation; Fisher/CRB; commuting-lag fixture |
| `aerial_replay.py` | Format-agnostic mocap+RPM thrust replay metrics |
| `aerial_unity_mirror.py` | Line-cited mirrors of the current C# |
| `aerial_profiles.py` | Profile loader and validator |
| `profiles/{x500_px4_gz, crazyflie21, rotors_firefly_hex, crane_reference_quad}.json` | Every coefficient carries a source, locator and status (MEASURED_PUBLISHED / VENDOR_SPEC / DERIVED / SIMULATOR_DEFAULT / ASSUMED / UNKNOWN) |
| `test_aerial_{invariants, identifiability, replay, crane_defects}.py` | `crane_defects` tests pass *while the defect exists*. Flip each into a regression test when its fix lands. |

## 5. Proposed C# changes (not made; opt-in; existing defaults untouched per CONTRACTS)

1. `Assets/Scripts/Utils/MAVROS/ArduPilotJsonFrames.cs` (new, pure static):
   - body-FRD gyro `-(M·Rᵀω)`
   - NED position and velocity
   - `quaternion` field instead of Euler
   - real `accel_body` = Rᵀ(Δv/dt − g)
   - `double` timestamp = tick·dt
   - a configurable servo map with default ArduCopter QUAD/X `[FR,RL,FL,RR]`.

   `MAVROSConnection` aerial path: snapshot in `FixedUpdate` and send one reply per servo frame (lockstep). Leave the Omni-X aquatic mapping unchanged.
2. `Assets/Scripts/Physics/Aerial/MultirotorRotorModel.cs` (new, pure): per-rotor kf, km, τ_up/τ_down, ω_max, λ1, μ1, ground effect, battery. Ported 1:1 from `aerial_model.py`.
3. `Assets/Scripts/Physics/Aerial/MultirotorVehicleProfile.cs` (new): loads the `Tools/Aerial/profiles` schema and hashes it into the replay header. Also:
   - an opt-in `useRotorModel` and a `spinPattern` enum on `MultirotorDynamics`
   - isotropic `|v|v` drag behind a flag
   - a gust clock taken from episode time
   - mixer desaturation (thrust-preserving, yaw-last).
4. Aerial timestep: a scene-level `CraneAerialTimestep` component sets `fixedDeltaTime` to 0.0025 s for aerial scenes only and records it in result JSON. The global 50 Hz must not change.
5. `Assets/Scripts/Sensors/Nav/CraneAerialImu.cs` (new): body-frame pseudovector gyro, specific-force accel, bias random walk, white noise, rotor-harmonic vibration, first-order DLPF, latency, and the acquisition-tick stamp. Also `CraneBarometer.cs` and `CraneMagnetometer.cs`. GNSS should reuse `GPS.cs` after audit.
6. Tests: `Assets/Tests/Editor/AerialFrameContractTests.cs` and `MultirotorAnalyticTests.cs`, using the same fixtures and numbers as the Python suite. Runner schema v2 adds the equivariance, free-fall, motor-step, ground-effect and real-saturation gates.
7. PX4 SITL via the `simulator_mavlink` HIL_SENSOR/HIL_ACTUATOR_CONTROLS lockstep is design only. It is **HYPOTHESIS**: PX4 interface details were not verified at source this session. It must run only against an isolated localhost SITL.

## 6. Risks

| Risk | Impact |
|---|---|
| A1 and A2 make any ArduPilot/PX4 run unsafe to interpret | A crash in sim is the *best* case. |
| The stale player and source may differ (A10) | All live numbers here carry the "stale build" caveat. |
| Blackbird data host is unreachable and its license is unstated | Replay is plan-only. |
| The x500 "x500-class" label overstates the evidence | Its coefficients are RotorS defaults. |
| The two Crazyflie kf values differ by 1.73× | Unresolved. |
| Förster's values are unverified at the primary source | – |

## 7. Next actions

1. Rebuild the player and rerun `px4-windy` and the aerial validation, to resolve A8 and A10.
2. The user decides whether to fix A1/A2 now or mark the ArduPilot aerial path NOT_SUPPORTED.
3. Port item 2 above with EditMode tests mirroring `test_aerial_invariants.py`.
4. Get Förster/Landry PDFs and confirm the Blackbird data terms.
5. Preregister the replay metrics.
