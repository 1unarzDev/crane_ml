# I1-verify-aerial: independent verification of I1-aerial.md

Verifier did not write the work. Read-only on the repo. Scratch under `/tmp/crane-verify-aerial/` (fetched sources in `src/`, mutants in `m1..m5/`, script `hexcheck.py`). Nothing was executed from fetched material except `pdftotext` on two arXiv PDFs. No player, no autopilot, no push.

## 0. Source fetches

| Item | sha256 | Result |
|---|---|---|
| ArduPilot `libraries/SITL/examples/JSON/readme.md` @9c412ba | `cfc34793122fd2d2dd6146329f7e47e3003958f2d429e0a21dfd561abdb32ba7` | matches cited `cfc34793…` |
| ArduPilot `libraries/AP_Motors/AP_MotorsMatrix.cpp` @9c412ba | `be9338206b623d6b47ac2906ea6e69358177579b48e86f224e565c12bf6e664b` | matches cited `be933820…` |
| bitcraze `model.sdf` @7e93752 / `LICENSE` | `89877fbd…` / `47b665f1…` | match profile |
| PX4-gazebo-models @e5997f4 `LICENSE`, `models/x500/LICENSE`, `x500/model.sdf`, `x500_base/model.sdf` | `5fcb2a52…`, `cee4ef94…`, `cb76a4af…`, `6eccbff8…` | match profile |
| RotorS @cd813b7 `firefly.xacro`, `package.xml` | `90951ba8…`, `98e93ed1…` | match profile |
| Du et al. arXiv:1403.5986 PDF (author preprint of JGCD 38(5) 2015, DOI 10.2514/1.G000731) | `0e77ac17…` | read |

Process note: I1-aerial §0 cites the Du paper only as "JGCD 2015". The DOI I first guessed (1.G000795) is a different paper. The correct one is 1.G000731. Crossref confirms the authors Du, Quan, Yang, Cai.

## 1. Verdicts

### A1 signs and frames, `MAVROSConnection.cs:169-199`: CONFIRMED

The spec text is the readme: `gyro` body frame (roll, pitch, yaw), `position`/`velocity` earth NED, `attitude` roll/pitch/yaw radians, and `timestamp` physics time.

| Field | C# | Finding |
|---|---|---|
| position | `:180-185` sends (z, x, y) | N=z and E=x are right. D=+y is wrong; it must be −y. Sign error confirmed. |
| velocity | `:194-199` sends (z, x, y) | Same D sign error. |
| attitude | `:187-192` sends eulerAngles (z, x, y) in degrees×Deg2Rad | Roll and pitch sign reversed. Unity z-roll is counter-clockwise seen from behind, so left side down, while NED roll is right side down. Unity +x pitch is nose-down, while NED is nose-up. Yaw sign is correct (clockwise from above in both). Extra defect not in I1: `eulerAngles` is [0, 2π), not ±π. |
| gyro | `:171-176` sends (ω.z, ω.x, ω.y) | `telemetryBody` is a `RigidbodyAdapter` and its `angularVelocity` is the world-frame `rb.angularVelocity` (`RigidbodyAdapter.cs:16`). It is not rotated into the body frame. Roll and pitch rates are sign-reversed (pseudovector det −1). Yaw rate sign is right at identity attitude. |
| accel_body | `:178` constant (0, 0, −g) | The value is correct only for static level hover. It is not a measurement. The class default at `:20` is (0, −g, 0), and it is overwritten. |
| timestamp | `:26`, `:169` `(float)Clock.time` | float32 confirmed. Sampled in `Update()` (`:146`), not at the physics tick. Sent by a 200 Hz thread (`:51`, `:298`). |

The I1 text "Position and velocity 'down' is +altitude" and "roll/pitch reversed" are both right.
Minor imprecision: I1's "all axes wrong" wording belongs to Imu (A3), not A1. For A1, yaw is correct.

### A2 servo order and spin mismatch: CONFIRMED

- **Servo order.**
  - ArduCopter `MOTOR_FRAME_TYPE_X` (`AP_MotorsMatrix.cpp:593-600`): the array index is the servo output. Index 0 is angle 45 (FR) CCW; 1 is −135 (RL) CCW; 2 is −45 (FL) CW; 3 is 135 (RR) CW.
  - So servo order is FR, RL, FL, RR.
  - The fourth struct field is the test order, not the servo number. I1's reading is right.
  - CRANE `ApplyPwm` (`MAVROSConnection.cs:212-218`) sends pwm[0..3] to FL, FR, RR, RL (`SetMotorCommands`, `MultirotorDynamics.cs:88-94`). The orders differ.
- **Spin.**
  - CRANE `yawSigns = {+1, −1, +1, −1}` for FL, FR, RR, RL (`MultirotorDynamics.cs:34`).
  - A +Y torque in Unity (left-handed) is clockwise seen from above. So a reaction torque of +1 means the rotor spins CCW. FL and RR are CCW.
  - ArduCopter and the PX4 x500 have FR and RL CCW (`x500.sdf`: rotor_0 at (+0.174, −0.174) ccw, rotor_1 at (−0.174, +0.174) ccw).
  - CRANE's spin pattern is the mirror image. Confirmed.
- **Also true.** The x500 rotor order (rotor_0..3 = FR, RL, FL, RR) equals the ArduCopter servo order. The PX4/ArduPilot orders agree with each other.
- **Interaction.** The servo→rotor remap also changes which spin each channel sees. Channel 0 (FR in ArduCopter, CCW) lands on CRANE-FL, which is CCW (coincidentally right). Channel 1 (RL, CCW) lands on CRANE-FR, which is CW (wrong). The vehicle is inconsistent, not just mirrored.

### Mixer clamp, 58% roll at collective 0.95: CONFIRMED, with a metric caveat

- `SetCommand` (`:75-85`) gives for roll=1, axis mix 0.2: the unclamped motor commands are (0.75, 1.15, 1.15, 0.75). `Clamp01` makes them (0.75, 1, 1, 0.75).
- Thrust ∝ s² (`:117`). Roll torque ∝ (T_FR+T_RR) − (T_FL+T_RL).
  - Unclamped: 2·1.3225 − 2·0.5625 = 1.52.
  - Clamped: 2 − 1.125 = 0.875.
  - Ratio 0.576. I1 says 58%.
- Caveat: this is a steady-state thrust-torque ratio, with the motor lag ignored. In speed-linear terms (the command difference), the ratio is 0.5/0.8 = 62.5%.
- Collective also drops. Total thrust is 3.125 against 3.61 for a pure 0.95 hold. This is confirmed.

### Drag heading dependence, 20% lower at 45°: CONFIRMED (arithmetic); the "defect" framing is a modelling judgement

- Scenes serialize linear drag (0.18, 0.25, 0.18) and quadratic drag (0.08, 0.12, 0.08), with x and z equal (all four scenes).
- At 5 m/s:
  - Along an axis: 0.18·5 + 0.08·25 = 2.90 N.
  - At 45° (3.54 m/s on each axis): the linear part is 0.9 and the quadratic part is 0.08·12.5·√2 = 1.414. Total 2.314 N. The ratio is 0.798.
- The quadratic part alone drops 29%. The 20% figure is the total.
- The per-axis form `v_i|v_i|` (`:141-143`) is genuinely not isotropic. It is also body-frame, so a real quad's body-axis drag anisotropy could be legitimate. The finding is only that it is not rotation-consistent in the flow heading. Fine as stated.

### Imu.cs gyro sign and missing acceleration: CONFIRMED, consistent with V4

- `Imu.cs:35`: `body.angularVelocity.To<FLU>()`. `To<FLU>()` is the polar `ConvertFromRUF` (V4 cites `ROSVector3.cs:27-29` and `CoordinateSpaces.cs:253-256`). The package's own `ConvertAngularVelocityFromRUF` (`:41`) negates it, and Imu does not use it. The vector is also the world-frame ω.
- `Imu.cs:32-37` builds `ImuMsg` with only orientation, angular_velocity and header. There is no linear_acceleration and no covariance.
- Agrees with V4 in `I1-defect-verification.md` (CONFIRMED; orientation chain `(z, −x, y, −w)`; the campus gyro in `CraneCampusSensorSuite` has the same polar error). V4's detail that OmniBoat's `Imu` is disabled does not change this for aerial. I1 says aerial scenes attach no Imu at all.
- I1 A3 says "the aerial vehicle has no sensors". I did not rescan the scene YAML. I1 and V4 agree that no aerial scene attaches an Imu, but `MAVROSConnection.imu` (`:6x`, `[SerializeField] Imu imu`) could supply `telemetryBody` if a scene wired it. That would make the SITL gyro also world-frame (it already is).

### Gust timing not reset between episodes: CONFIRMED

- `MultirotorDynamics.cs:126`: `phase = Time.fixedTime * f * 2π`.
- `ResetEpisode` (`:46-52`) resets actuators, wind and gust amplitude. It never touches the phase.
- Nothing else in `Assets/Scripts` rewinds or offsets `Time.fixedTime`. A grep for `fixedTime`, `timeScale` and `SetFixedTime` found only readers. Unity has no API to rewind `fixedTime`.
- This holds for in-process resets. Each fresh process starts at 0.

## 2. Python oracle and mutants

Baseline copy of `Tools/Aerial` in `/tmp/crane-verify-aerial/base`: `python -I -m pytest` gives 43 passed.

| Property | Test | Independent check |
|---|---|---|
| hover T = mg | `test_hover_thrust_equals_weight_and_is_torque_free` (4 profiles) | Asserts force_z = mg to 1e-12, zero torque and no drift over 500 steps. Sound. |
| Yaw torque balance with spin directions | `test_yaw_acceleration_from_reaction_torque_balance` | Raises the CCW rotors and lowers the CW rotors at constant sum T, and checks the torque against the closed form and that CCW-up yaws the body clockwise. Sound for the x500 only. |
| Free-fall conservation | `test_free_fall_conserves_energy_and_angular_momentum` | Asymmetric tumble with RK4, rel 1e-9. Sound. |
| Hex one-rotor failure | `test_hex_single_rotor_failure_has_zero_hover_controllability_margin` | See below. |

**Du et al. claim, checked at the paper.**
- Table II (PNPNPN, equal efficiencies): for η_i = 0 on any one of the six rotors, the rank of C(A, B) stays 8 but the ACAI (the radius of the largest ball inside the attainable control set, centred on hover) is 0. The paper's wording is "uncontrollable", and "even though its controllability matrix is row full rank".
- ACAI = 0 means the hover wrench is not in the interior of the set. It does not say the hover is infeasible. The oracle's check is "margin = 0 to 1e-9 and `wrench_feasible` True". That is the faithful reading.
- Table III (PPNNPN): rotors 5 and 6 failing are uncontrollable, and the other four are controllable.
- My `hexcheck.py` on the firefly profile gives margin 0.0 for all six failures with PNPNPN (nominal margin 0.24).
- In my mutant with pattern (+, +, −, −, +, −), margins are 0.142 / 0.080 / 0.080 / 0.142 for rotors 0-3 (controllable) and −0.047 for rotors 4 and 5 (infeasible). This reproduces the paper's pattern, including its 5-th / 6-th exception.
- Du's numbers are not reproduced directly, because the profile uses RotorS parameters and the paper's ACAI is in its own units. Only the zero/nonzero outcome and the pattern are checked.

**Mutants (in /tmp copies, repo untouched).**

| Mutant | Change | Tests failing | Caught? |
|---|---|---|---|
| M1 | x500 rotor_0 spin flipped | 3 (`hover_thrust…[x500]` torque-free, yaw balance, quad allocation yaw authority) | Yes |
| M2 | thrust ∝ ω instead of ω² (`kf*speed*1.0`) | 7 (hover ×4, yaw balance, wind, near-pad hover) | Yes |
| M4 | firefly hex rotor m1 spin flipped (PNPNPN → PPPNPN) | 2 (hex torque-free hover, hex failure margin) | Yes |
| M5 | firefly spins changed to PPNNPN | 1 (hex failure test) | Yes, as the paper predicts |
| M3 | `effectiveness_matrix` yaw column sign flipped for all rotors (`- spin*km*a` to `+`) | 0 | **No: survived (43 passed)** |

M3 is a real gap. No test cross-checks `aerial_allocation.effectiveness_matrix` against `aerial_model.wrench`'s torque. All allocation tests (feasibility, yaw authority magnitude, zonotope, hex failure) are symmetric under the global yaw-sign flip. A sign convention error in the allocator would pass. Suggested test: for random u, `B @ u` must equal (Σ T, body torque from `wrench` with thrusts u).

Other limits of the oracle:
- The hex failure test uses the fixed, hardcoded index 0, and it checks only one index (the others have margin 0 by my check, so no test hole in outcome).
- The yaw balance test uses the x500 only.
- The CRANE-defect tests pass while the defects exist, as documented, so they cannot catch an error in the mirror itself. They are checked here against the C# by hand (section 1).

## 3. Crazyflie 2.x and x500 coefficients

**x500** (`x500/model.sdf`, `x500_base/model.sdf`): confirmed digit-for-digit: kf 8.54858e-06, km 0.016, τ_up/τ_down 0.0125/0.025, ω_max 1000, λ1 8.06428e-05, μ1 1e-06, mass 2.0 + 4×0.016077 = 2.0643, I = (0.021667, 0.021667, 0.04). Spin FR/RL ccw. All are `SIMULATOR_DEFAULT`; I found no x500 sysid, so the label in the profile stands. The rotor radius 0.1651 is inferred from the mesh name; it is not a stated parameter (the profile flags it ASSUMED).

**Crazyflie** (model.sdf @7e93752): inertia (16.5717e-6, 16.6556e-6, 29.2617e-6), km 0.005964552, kf 1.28192e-08, ω_max 2618 all match. One difference: the sdf base link mass is 0.025 kg, while the profile uses 29 g from the datasheet (it notes the gz total as 0.0282; with 4×0.8 g props the sum is 28.2 g, so that is right).

**The 1.73× disagreement.** Computed ratio 2.2183e-8 / 1.28192e-8 = 1.7305.

| Source | N/(rad/s)² per rotor | Basis |
|---|---|---|
| Bitcraze thrust-stand table (the Bitcraze PWM-to-thrust page), through origin | 2.218e-8 | Measured: 15 rows of whole-vehicle thrust (g) against mean RPM. Per-point relative RMS is 9% by my metric (I1 states 2.9%, likely a different weighting). |
| Bitcraze page polyfit, quadratic term 1.0942e-07 g/rpm² | 2.446e-8 | Same data |
| Luis & Le Ny 2016 (arXiv:1608.05786): C_T = 3.1582e-10 N/rpm² | 2.880e-8 | Not measured; derived from non-dimensional k_T = 0.2025, ρ and a rotor radius |
| gz `motorConstant` 1.28192e-08, labelled "Forster's" in CrazyS | 1.282e-8 | Unverified: Förster's thesis was HTTP 429 at every ETH URL again |

- **Units are not the cause.** The factors that could explain a units slip are 4 (per-motor vs total), 60/2π = 9.55, its square 91.2, 2π, and 1000/9.81. None is 1.73. The 1.73 is also not 4 and not 2. A per-rotor vs total slip would give exactly 4 or 1/4. An RPM vs rad/s slip would give 91.2. √3 = 1.7321 is numerically close, but I see no mechanism for it, so treat it as a coincidence.
- **Consistency checks decide which is physical.**
  - The gz kf with ω_max 2618 gives total max thrust 35.8 g, a thrust-to-weight of about 1.3 for a 28 g vehicle, and a hover RPM of about 22,100 (close to the 23,882 RPM the vehicle reaches at 94% PWM).
  - The Bitcraze data gives about 58 g at 23,882 RPM, a thrust-to-weight of about 2, and hover at about 16,800 RPM, which agrees with the table (29 g at 17,174 RPM).
  - The Bitcraze page also states about 58 g at 93.75% PWM, and the vehicle weighs about 27-29 g.
  - That makes the whole-vehicle reading right: if the column were per-motor, T/W would be about 8.
- **Which source is primary.**
  - For thrust vs RPM on a Crazyflie 2.x with a known test fixture, the Bitcraze table is the only primary source I could actually read. It is measured, and its units are explicit in the column headers (g, RPM).
  - The gz 1.28192e-8 is a secondary copy attributed to Förster. The primary was unreachable. Its origin (a different battery or prop wear, a thrust-vs-PWM fit rather than RPM, or a fit to a different quantity) cannot be decided here.
  - The profile's "unresolved" status is therefore correct. My conclusion goes one step further. For the Bitcraze-measured quantity the evidence favours 2.2e-8 to 2.4e-8. 1.28e-8 should not be used for thrust without the Förster PDF.
- The Bitcraze page does not state "whole vehicle". That is an inference from magnitudes (2.1 A, 58 g, a vehicle that weighs about 27 g). I1 flags this assumption in the profile notes.

## 4. Licences

| Item | Check | Result |
|---|---|---|
| PX4-gazebo-models @e5997f4 | `LICENSE` | BSD-3-Clause, "Copyright (c) 2022, PX4 Autopilot for Drones". Confirmed. |
| x500 model | `models/x500/LICENSE` | BSD-3-Clause, "Copyright (c) 2022, Rudis Laboratories". Confirmed. Meshes have no separate notice. I1's "not imported" caution stands. |
| RotorS @cd813b7 | `package.xml` `<license>ASL 2.0</license>`; `firefly.xacro` header "Licensed under the Apache License, Version 2.0", Copyright 2015 Furrer, Burri, Kamel, Nikolic, Achtelik | Confirmed. Raw `LICENSE` and `rotors_description/LICENSE` are both 404 at this commit, so there is no top-level licence file. Apache-2.0 is by header and package metadata only. Mesh status is unverified and I1 does not import it. |
| Bitcraze crazyflie-simulation @7e93752 | `LICENSE` | MIT, "Copyright (c) 2022 Bitcraze". Confirmed. Meshes: I did not re-check for a separate mesh notice (I1 cites `docs/functional_areas/meshes.md`). The Bitcraze PWM table is web documentation with no stated licence, cited as facts only. |

All four are compatible with GPL-3.0 use. Apache-2.0 is one-way compatible. This repo carries a new top-level GPL `LICENSE` (untracked), so attribution notices in `THIRD_PARTY_NOTICES.md` are needed before any mesh or code import.

## 5. Summary of corrections to I1

1. The Du et al. citation needs the correct DOI (10.2514/1.G000731). The claim is ACAI = 0 for PNPNPN under any single rotor failure ("uncontrollable"), not "infeasible hover". The oracle matches that.
2. The mixer 58% figure holds only as a steady-state thrust-torque ratio (62.5% in command space).
3. The A1 gyro defect is world-frame plus roll/pitch sign; yaw is correct. Euler range [0, 2π) is an extra defect.
4. Add a test tying `effectiveness_matrix` to `wrench` (mutant M3 survived).
5. The 1.73× is not a units error. Prefer the Bitcraze measured value over the unverifiable Förster-attributed one, until the Förster PDF is read.
