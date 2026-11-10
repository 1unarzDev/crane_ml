# PX4 SITL lockstep plan (not implemented)

Status: PLAN. Nothing here runs yet. Never connect this path to real hardware. Use simulated
topics and an isolated ROS domain or MAVLink port only.

## Goal

Replace `CraneMultirotorController` with PX4 running in SITL lockstep. The physics stays in
`CraneAerialPlantCore`. PX4 must see bit-reproducible sensor timing, and Unity's frame rate must
not change the closed-loop result.

## Seam already in place

- `IAerialRotorCommandSource.RotorCommands(StateV2 state, double time)` is polled every
  `ControlDt` at sub-step boundaries (`CraneAerialPlantCore.Substep`). A PX4 bridge implements this
  interface.
- `CraneAerialImu` already produces body-frame FLU gyro and specific force at 250 Hz or more, with
  double-precision plant time. FRD accessors (`GyroFrd`, `SpecificForceFrd`) and
  `CraneAerialFrames.EnuToNed` / `NedFrdEuler` give PX4/ArduPilot frame conventions.
- Rotor order of the AOS 7 profile is the PX4/ArduCopter QUAD-X order: FR, RL, FL, RR. FR and RL
  spin CCW. Actuator output i maps to rotor i directly.

## Transport

- PX4 `simulator_mavlink` (TCP 4560) with `HIL_SENSOR`, `HIL_GPS` and `HIL_STATE_QUATERNION`
  going in, and `HIL_ACTUATOR_CONTROLS` coming out. The alternative is the newer
  uXRCE-DDS/gz-bridge style; mavlink is chosen for its simplicity and lockstep support.
- Lockstep rule: the simulator sends `HIL_SENSOR` with `time_usec = round(plant.State.T * 1e6)`.
  PX4 advances its clock only on that message, then replies with one `HIL_ACTUATOR_CONTROLS`. The
  bridge blocks inside `RotorCommands` until the reply arrives, with a timeout. On timeout the
  episode is marked failed. It never free-runs.
- Rates:
  - IMU / HIL_SENSOR at 250 Hz (ControlDt = 4 ms; the plant stays at 1 kHz with 4 sub-steps per
    tick).
  - GPS at 10 Hz.
  - Mag and baro decimated from the same truth state.

## Mapping

| PX4 field | Source |
|---|---|
| xacc/yacc/zacc | `SpecificForceFrd` (m/s^2). At rest this reads (0, 0, -9.81). |
| xgyro/ygyro/zgyro | `GyroFrd` (rad/s) |
| abs_pressure / pressure_alt | ISA from `State.P[2]` plus seeded noise |
| xmag.. | world field (NED) rotated by R^T, in FRD |
| HIL_GPS lat/lon/alt, vel NED | `EnuToNed` about a fixed geodetic origin |
| actuator controls[0..3] in [0,1] | thrust fraction u_i; rotor command = OmegaMax * sqrt(u_i), the same law as the built-in controller |

## Work items

1. `CranePx4LockstepBridge : IAerialRotorCommandSource`. Socket I/O lives on a worker thread with
   bounded queues. The physics thread only blocks on a semaphore with a timeout.
2. Airframe file `4xxxx_crane_aos7`:
   - `CA_ROTOR*` positions and `KM` taken from `aos7_v5_f90_6s.json`.
   - `MPC_THR_HOVER` taken from `derived.hover_thrust_fraction_model`.
3. Reset: on `ResetEpisode`, restart PX4, or send `MAV_CMD_PREFLIGHT_REBOOT_SHUTDOWN` and wait
   for EKF alignment. Turbulence and IMU noise are seeded from the episode seed. They are already
   reset-safe.
4. Replay: log every HIL_SENSOR and HIL_ACTUATOR_CONTROLS pair with plant time. Replaying the
   actuator log through `CraneAerialPlantCore` must reproduce the state bit for bit. The same
   property is already tested for the built-in controller in
   `PlantResetReplaysTurbulentEpisodeExactly`.
5. Tests:
   - Frame sanity: at rest PX4 reports level attitude and a NED heading equal to `NedFrdEuler`.
   - Takeoff to 2 m and hold for 30 s.
   - Lockstep determinism: two runs with the same seed give identical actuator logs.
6. Retire the legacy refusal path in `CraneAerialOptions` only once 5 is green.

## Risks

- PX4 EKF2 needs consistent mag, baro and GPS. Missing sensors block arming.
- Blocking inside FixedUpdate can stall the editor. Run the lockstep in batchmode, or on a
  dedicated physics thread, for long runs.
- A 34 ms motor time constant with the default PX4 rate gains may need `MC_*RATE_K` tuning.
  Start from the PX4 defaults for a 7-inch frame.
