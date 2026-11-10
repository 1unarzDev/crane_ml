# Drone physics (aerial-v2 port, sub-stepped plant, AOS 7 V5 profile)

Evidence label: FIXTURE_VALIDATED. That means EditMode parity against the Python oracle plus
analytic invariants. Nothing here is PHYSICALLY_CALIBRATED: there is no flight log and no
thrust-stand data from the owner's airframe.

## Files

| Path | Role |
|---|---|
| `Assets/Scripts/Physics/Aerial/CraneRotorModelV2.cs` | Force/torque model and RK4 step. It is a 1:1 port of `Tools/Aerial/aerial_model.py`. |
| `.../CraneAerialPlantCore.cs` | Unity-independent plant. It runs sub-steps at 1 kHz or faster and polls the controller at control ticks. Wind is the mean wind plus Dryden turbulence. It also feeds the IMU. |
| `.../CraneAerialVehicle.cs` | MonoBehaviour. It drives a kinematic Rigidbody or transform to the plant pose. This is the director API. |
| `.../CraneRotorMixerV2.cs` | Priority mixer. Yaw gives way first, then collective. The roll:pitch ratio is preserved. |
| `.../CraneMultirotorController.cs`, `CraneTrajectoryFollower.cs` | Cascaded controller and scripted waypoints |
| `.../CraneDrydenTurbulence.cs` | MIL-F-8785C Dryden turbulence. It is seeded and reset-safe. |
| `.../CraneAerialImu.cs`, `CraneAerialFrames.cs` | IMU stub, plus ENU/FLU, Unity RUF and NED/FRD conversions |
| `.../CraneAerialProfile.cs`, `CraneAerialOptions.cs` | Profile loader (records the SHA-256) and CLI gates (ported unchanged) |
| `Tools/Aerial/aerial_model.py` | Python oracle (numpy) |
| `Tools/Aerial/fit_aos7_profile.py` | Rebuilds `profiles/aos7_v5_f90_6s.json` from the cited numbers. `--check` verifies the file is current. |
| `Tools/Aerial/export_substep_parity.py`, `export_parity_trajectories.py` | Export the parity fixtures to `Tools/Aerial/parity/` |
| `Assets/Tests/Editor/Aerial/Aerial{RotorParity,SubstepParity,PlantFeature}Tests.cs` | EditMode tests (asmdef `Crane.Aerial.Physics.Tests`) |

Runtime code is in its own asmdef, `Crane.Aerial.Physics` (`Assets/Scripts/Physics/Aerial`). It
depends only on UnityEngine. Tests are in `Crane.Aerial.Physics.Tests`. Other assemblies that
want the plant add a reference to `Crane.Aerial.Physics`. No shared asmdef was edited.

## Model

Frames:
- World is ENU with z up. Body is FLU.
- The quaternion is Hamilton (w, x, y, z), rotating body to world.
- Spin +1 means the rotor turns CCW seen from above.
- AOS rotor order is QUAD-X: FR, RL, FL, RR. FR and RL spin CCW, the same as PX4 and ArduCopter.

Per-rotor terms:
- Thrust: `kf w^2 * GE(h)`.
- Reaction torque: `-spin km T` about the axis.
- Rotor drag: `-|w| lambda1 v_perp` (the gz-sim / Martin-Salaun form).
- Rolling moment: `-|w| mu1 v_perp`.
- Rotor gyroscopic torque: `-omega x sum(J spin w axis)`.

Body terms:
- Body drag: `-D v_air - c |v_air| v_air`. This form is rotation-consistent.
- Gravity.
- Full-inertia Euler equations.

Effects:
- Ground effect: Cheeseman-Bennett, `1/(1-(R/4h)^2)`, with h clamped to at least R/2.
- Battery: Thevenin source `V = Voc(soc) - I R`, where the current comes from shaft power
  `km kf w^3 / eta`. The rotor speed limit scales by `V/V_nom`, capped at 1. Soc is integrated.
- Motor: first-order lag. tau_up and tau_down are selected by the sign of the error, and the
  target is clamped to `[omega_min, limit(V)]`.
- Mixer: works in thrust-fraction space. When the request saturates, roll and pitch scale
  together, then yaw is reduced by bisection until it fits, then collective shifts into the
  remaining window.
- Turbulence: Dryden low-altitude scales. u uses an exact AR(1) filter. v and w use the
  `(1+sqrt3 T s)/(1+T s)^2` filter at the sub-step rate, normalized so the stationary sigma is
  exact. The advection speed is `max(|mean wind - v|, 1 m/s)`. `Reset(seed)` replays bit for bit.

## Decoupling from Unity's fixed step

`CraneAerialPlantCore.Advance(Time.fixedDeltaTime)` runs `round(dt / 1 ms)` RK4 sub-steps. The
plant owns the state in double precision. `CraneAerialVehicle` then moves a kinematic body to the
resulting pose. PhysX never integrates the airframe, so the trajectory does not depend on Unity's
step. `AdvanceIsIndependentOfUnityFixedStep` checks this: 50, 100 and 200 Hz Unity steps agree to
1e-12.

Integration scheme: each RK4 stage uses the exact lag solution at its stage time,
`w(h) = target + (w0 - target) e^{-h/tau}`, with the target, tau and battery limit frozen over the
1 ms sub-step. The original v2 scheme was different: it advanced the motors over dt, then ran RK4
with the end-of-step speed. That shifts thrust earlier by dt/2 and is first order in dt.
`aerial_model.step(motor_scheme=...)` keeps both schemes. The control rate is 500 Hz, and the
commands are held (ZOH) between ticks in both Python and C#.

The rotor lag tau = 34 ms is resolved by 34 sub-steps.

### Parity results (AOS 7 profile, battery on)

The 10 kHz reference is `aerial_model.py` with the exact-stage scheme. Values are RMS over 10 ms
samples. The C# numbers come from Unity EditMode (`AERIAL_SUBSTEP_PARITY` log lines). The
zoh-first and naive 50 Hz columns are Python only, for comparison.

| scenario | C# 1 kHz pos [m] | att [rad] | rate [rad/s] | rotor [rad/s] | identity vs oracle @1 kHz (max rel) | zoh-first 1 kHz pos / att | naive 50 Hz pos / att / rotor |
|---|---|---|---|---|---|---|---|
| step (with ground effect, wind; ground contact at t = 1.73 s) | 4.6e-5 | 2.2e-4 | 1.3e-3 | 3.6e-13 | ~2e-15 | 1.1e-3 / 2.7e-3 | 8.2e-4 / 3.7e-3 / 3e-13 |
| chirp 0.5-10 Hz, pitch | 6.4e-11 | 7.3e-9 | 1.9e-10 | 4.0e-13 | ~9e-15 | 6.0e-3 / 2.5e-3 | 1.2e-1 / 4.3e-2 / 4.8 |
| saturation (1.3x w_max, yaw pair, idle) | 1.1e-3 | 1.1e-3 | 3.7e-3 | 1.6e-1 | ~9e-15 | 9.0e-3 / 8.0e-3 | 2.6e-2 / 2.4e-2 / 4.4 |
| hover to pitch flip (body z reaches -0.994) | 9.8e-9 | 6.7e-9 | 2.4e-8 | 6.7e-13 | ~2e-15 | 4.3e-3 / 6.4e-3 | 1.6e-3 / 1.4e-3 / 6e-13 |

The tests assert these bounds:

| scenario | pos | att | rate | rotor |
|---|---|---|---|---|
| step | < 2e-4 | < 1e-3 | < 6e-3 | — |
| chirp, flip | < 1e-6 on every channel | | | |
| saturation | < 4e-3 | < 4e-3 | < 1.2e-2 | < 0.5 |
| identity, all scenarios | < 1e-7 | | | |

The error is concentrated at non-smooth events:
- the ground-contact clamp in step;
- the battery speed-limit clamp, which switches every sub-step during saturation.

Between such events the 1 kHz scheme is effectively exact (about 1e-9). The old fixture
(`rotor_parity_trajectories.json`) covers 6 scenarios across 5 profiles, including the AOS 7. It
also matches at the same step to 7.7e-15.

## AOS 7 V5 profile (`Tools/Aerial/profiles/aos7_v5_f90_6s.json`)

Airframe:
- AOS 7 V5 frame
- T-Motor F90 2806.5 **KV1300**, the 6S 7-inch variant (the KV1500 and KV1950 are for lighter or
  lower-voltage builds)
- 7-inch tri-blade props (HQ 7040, the prop in the vendor table)
- Holybro Tekko32 F4 Metal 65A 4-in-1 ESC
- 6S LiPo

Derived values:
- **Mass 1.383 kg**
- **T/W 6.38**. This is above the 2.5 flag threshold, so stopping distance and the speed envelope
  are thrust-limited only by tilt limits.
- Hover thrust is 346 g per rotor, at 7767 rpm.
- Hover ESC throttle is about 0.30, interpolated from the datasheet table (the table starts at
  30%).
- The model's hover thrust fraction is (w_h/w_max)^2 = 0.157.
- The CoM is placed on the motor centroid in x and y. The battery x position is solved for that,
  as builders do by sliding the pack. CoM z is 15 mm above the frame centre.

### Mass budget (orchestrator-corrected build)

| item | g | provenance | source / method |
|---|---|---|---|
| frame plates + standoffs | 95 | ESTIMATED | aos-rc.com has no fetchable spec. Assumed 170 g total frame, the 7-inch midpoint. |
| frame arms (4) | 75 | ESTIMATED | same |
| T-Motor F90 KV1300 (4) | 186.4 | MEASURED_DATASHEET | 46.6 g each incl. cable (T-Motor) |
| props HQ 7x4x3 (4) | 36 | ESTIMATED | 9 g each |
| Holybro Tekko32 F4 Metal 65A | 15.8 | MEASURED_DATASHEET | holybro.com |
| Cube Orange+ module | 75 | ESTIMATED | Vendor mass not found on docs.cubepilot.org. Cube class is 70-80 g. |
| Cube Mini carrier | 20 | ESTIMATED | no published mass |
| Jetson Orin Nano Super module + carrier + heatsink | 140 | ESTIMATED | Dev kit is ~176 g, minus its oversized carrier. The module datasheet requires a login. |
| L2 12 V regulator | 12 | ESTIMATED | buck-converter class |
| Microhard pMDDL900 OEM | 7 | MEASURED_DATASHEET | brochure Rev 1.2.1, "~7 grams" (sha256 ab0219...) |
| Microhard carrier | 15 | ESTIMATED | |
| 900 MHz antennas (2) | 20 | ESTIMATED | |
| FrSky receiver | 5 | ESTIMATED | Model not specified. R-XSR/Archer RS class. |
| Luxonis OAK-D S2 | 91 | MEASURED_DATASHEET | shop.luxonis.com |
| Unitree L2 | 230 | MEASURED_DATASHEET | unitree.com/L2 |
| 6S 1800 mAh LiPo | 290 | ESTIMATED | Pack not specified. 270-300 g class. |
| wiring, GNSS, mounts, BEC | 70 | ESTIMATED | |
| **total** | **1383** | ESTIMATED | |

Sensitivity: a 6S 3000 mAh pack (about 450 g) would give 1.54 kg and T/W about 5.7.

### Parameters

| parameter | value | provenance | method / source |
|---|---|---|---|
| kf | 5.128e-6 N/(rad/s)^2 | MEASURED_DATASHEET | Relative LSQ of T = kf w^2 over the 11-row F90 KV1300 + HQ7040 table at ~24 V. Max residual 8.3%, RMS 5.0%. C_T = 0.165. |
| omega_max | 2054 rad/s | MEASURED_DATASHEET | 19617 rpm at 100% throttle, 23.48 V |
| km | 0.0144 m | ESTIMATED | Q = eta P_elec / w, with datasheet power and eta = 0.80 assumed. kq is fit by LSQ, then km = kq/kf. |
| tau_up = tau_down | 33.8 ms | ESTIMATED | Linearised BLDC + prop: tau = J/(Kt Ke/R + 2 kq w_h), J = 2.8e-5 kg m^2 (props + bell estimated), Kt = 60/(2 pi Kv), R = 76 mOhm from the datasheet. PX4 gz x500 uses 12.5 / 25 ms (simulator default). Symmetric because AM32 complementary PWM brakes actively. |
| rotor_inertia | 2.8e-5 kg m^2 | ESTIMATED | as above |
| rotor_drag lambda1 | 1.49e-4 N s/(m rad) | ESTIMATED | Aggregate 0.35 1/s x mass, split over 4 rotors at hover. Order of magnitude from Faessler, Franchi and Scaramuzza, RA-L 2018 (doi 10.1109/LRA.2017.2776353). |
| rolling_moment | 0 | ESTIMATED | neglected |
| rotor_radius | 0.0889 m | MEASURED_DATASHEET | 7 in prop |
| drag_quadratic | 0.00858 N s^2/m^2 | ESTIMATED | 0.5 rho Cd A, with Cd = 1.0 and A = 0.014 m^2 |
| drag_linear | 0 | ESTIMATED | carried by rotor drag |
| inertia | diag(0.00598, 0.00831, 0.00949), Ixz = 2.1e-4 kg m^2 | ESTIMATED | parallel-axis sum over the component layout (`massBudget` positions and boxes) |
| battery V_full / V_empty | 25.2 / 21.0 V | LITERATURE / ESTIMATED | 4.20 V per cell; linear OCV to 3.5 V per cell |
| battery V_nom | 23.48 V | MEASURED_DATASHEET | voltage at which omega_max was measured |
| battery R | 30 mOhm | ESTIMATED | 6 x 4 mOhm + leads |
| capacity | 1.8 Ah | ESTIMATED | |
| eta | 0.80 | ESTIMATED | The same eta is used for km, so model electrical power tracks the datasheet power column. Max relative error 25%, at low throttle where efficiency is not constant. |
| ground effect | Cheeseman & Bennett 1955 | LITERATURE | ARC R&M 3021 |
| turbulence | Dryden, MIL-F-8785C | LITERATURE | |

T-Motor source: https://store.tmotor.com/product/f90-fpv-motor.html. The live page now redirects.
The values were read from the Wayback snapshot of 2025-05-24 (archived HTML sha256 9a66a6b9...).

## Controller and director API

`CraneMultirotorController` is a cascade: position P, then velocity PI (giving acceleration),
then thrust vector and attitude, then rate PI.
- The attitude error is the Lee et al. SO(3) error.
- All gains are kinematic: 1/s or 1/s^2.
- Conversion to force and torque is explicit, through the profile mass and inertia plus
  omega x J omega.
- It uses truth state, with no estimator.

Tests:
- Hover holds to 2 cm.
- A 4.9 m position step settles within 5 cm and yaw within 0.02 rad.
- Velocity tracking in Dryden gusts (W20 = 6 m/s) has a mean error below 0.6 m/s.
- A 3-leg waypoint mission ends within 10 cm, with cross-track error below 0.5 m on the first leg.

Director API on `CraneAerialVehicle` (Unity world frame):
- `SetTrajectory(Vector3[] waypoints, float[] speeds)`
- `SetVelocitySetpoint(Vector3 v, float yawRateDeg)`
- `HoldPosition(Vector3 p, float yawDeg)`
- `ResetEpisode(seed)`

`RotorSpeeds` is exposed for prop visuals. PX4 replacement: see `PX4_LOCKSTEP_PLAN.md`.

## IMU stub (v1 defects fixed)

`CraneAerialImu` runs at 500 Hz by default and refuses rates below 250 Hz. It integrates over
the sub-steps:
- Gyro: the body-frame FLU rate. The v1 code used the world rate, converted as a polar vector.
- Accelerometer: specific force `R^T(dv/dt - g)`. This reads +g on z at rest (FRD: -g) and about
  0 in free fall. The v1 code sent a constant (0, 0, -g).
- Time: double-precision plant time.
- Noise and bias random walk: optional and seeded.
- Queue: bounded; it drops the oldest sample and counts drops.

Tests cover the hover and free-fall signs, the body-vs-world gyro, and the FRD conversion.
NED/FRD helpers fix the v1 SITL D sign and roll/pitch sign errors (`NedFrdEuler`, `EnuToNed`).

## Known gaps

- Not physically calibrated. The tau, km split, rotor drag, frame mass and all ESTIMATED masses
  should be replaced with thrust-stand and flight-log identification.
- There is no rotor-acceleration reaction torque (J dw/dt about the shaft). This understates yaw
  transients during fast collective changes.
- No current limiting or ESC desync in the motor model. Large steps are pure first-order.
- The battery OCV is linear. There is no temperature model and no polarization RC branch.
- Contact is a flat ground plane only (clamp z, zero downward v). PhysX collisions with scene
  geometry are not fed back into the plant, because the body is kinematic.
- Dryden at hover uses mean-wind advection with a 1 m/s floor, which is an assumption. There is
  no wind shear and no rotor-wake or proximity effects other than ground effect.
- The force model allocates per call (managed arrays). That is fine for a few vehicles at 1 kHz;
  pooling is needed for swarms.
- The profile is loaded from `Tools/Aerial/profiles` next to the project. A standalone player
  build needs the JSON copied to StreamingAssets, which is not done.
- PX4 lockstep is planned, not implemented.
