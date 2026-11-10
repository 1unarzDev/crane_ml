# crane-drone-log/1: synchronized flight logs

One directory per flight. Writer: `Assets/Scripts/Drones/Logging/` (`DroneRunWriter`, `DroneRunSession`).
Reader, validator, plot: `Tools/DroneData/drone_log.py`. Reproducibility checker: `Tools/DroneData/check_repro.py`.
Evidence label: IMPLEMENTED / FIXTURE_VALIDATED against the stub plant (`DroneReferenceFlight`); not yet run on the aerial physics module or the real sensors.

```
run_dir/
  manifest.json          provenance + clock (written at start, rewritten at close with "end")
  state.csv              ground truth, every state substep
  imu.csv                IMU at its own rate
  events.jsonl           collision | near_miss | detection | marker, time ordered
  obstacles.json         static obstacles in world ENU, for plotting / clearance audits
  camera/<name>.csv      one index per camera; camera/<name>/NNNNNN.png frames (RGB 8-bit, depth 16-bit gray in mm)
  lidar/<sensor>.csv     one index per lidar; lidar/<sensor>/NNNNNN.bin scans
```

## One clock
All streams are stamped `t_ns`: integer nanoseconds of **sim time**, never wall clock.
A tick is one Unity fixed step (the `CraneSimulationClock` convention); the plant sub-steps each tick:
`t_ns = tick * fixed_dt_ns + sub * fixed_dt_ns / substeps` (exact integers, `fixed_dt_ns % substeps == 0`).
Camera and LiDAR stamps are **acquisition** time (exposure / sweep start), as in `SensorAcquisitionTimingTests`;
`complete_ns` (readback done) is stored beside it and is always >= acquisition. Pair a frame with
ground truth by interpolating `state.csv` at `acq_ns` (`DroneLog.camera_poses`).
Writers refuse non-monotonic stamps (state, imu, camera, lidar strictly increasing per stream; events non-decreasing).
Event stamps are the time of emission; a near-miss is emitted when the drone leaves the threshold zone and carries
`t_min_ns` of the closest approach.

## manifest.json
`schema`, `run_id`, `crane_git_sha`, `scene`, `scene_seed`, `split` (from `Tools/DroneData/splits.json`),
`forest_manifest_sha256`, `physics_profile`, `physics_params_sha256`, `controller`, `trajectory_spec` (object),
`disturbance{wind,lighting}`, `unity_version`, `wall_clock_start_utc`,
`clock{unit,fixed_dt_ns,substeps,substep_dt_ns,substep_hz,state_every_substeps,state_hz}`,
`frames{world:ENU, body:FLU, quat:"wxyz world_from_body"}`, `rotor_count`, `imu{rate_hz,noise}`, `cameras[]`, `lidars[]`, `extra`,
and after a clean close `end{end_tick,end_t_ns,counts{stream:rows}}` (a missing `end` means the run did not close).
`crane_git_sha`, `forest_manifest_sha256` and `physics_params_sha256` are filled by the caller (the recorder does not guess).
Reproducibility compares manifests ignoring `run_id`, `wall_clock_start_utc`, `end`.

## state.csv
Columns: `tick,sub,t_ns, px,py,pz` (world ENU, m), `qw,qx,qy,qz` (world-from-body, canonical w>=0),
`vx,vy,vz` (world m/s), `wx,wy,wz` (body FLU rad/s), `ax,ay,az` (**coordinate** acceleration in body FLU, m/s^2,
gravity not removed; finite difference of velocity when the plant does not supply it), `rpm0..rpmN-1`, `cmd0..cmdN-1`
(normalised motor command 0..1), `batt_v, batt_a, batt_soc`. Floats are shortest round-trip; unknown = `nan`.
Rate is the substep rate divided by `state_every_substeps` and must be >= 200 Hz (validated).
The stub flight logs at 1 kHz (5 substeps x 5 ms fixed tick).

## imu.csv
`t_ns,gx,gy,gz,ax,ay,az,noise_flags` (body FLU; accel is **specific force**). Own rate (`imu.rate_hz`, integer divisor of the
substep rate). `noise_flags` bits: 1 white noise, 2 bias random walk, 4 saturation, 8 quantisation; 0 = ideal. Noise parameters are in `manifest.imu.noise`.

## camera
OAK-D-like set on the onboard profile (names `oakd_rgb`, `oakd_left`, `oakd_right`, `oakd_depth`), each at its own rate.
Index `camera/<name>.csv`: `seq,acq_ns,complete_ns,width,height,kind,file`. `kind` is Rgb | StereoLeft | StereoRight | Depth.
RGB/stereo are 8-bit RGB PNG; depth is 16-bit grayscale PNG in millimetres (0 = invalid). The PNG writer is dependency free
(`DronePng`); the reader needs only `zlib` (`drone_log.read_png`, `depth_mm`). MP4 is not used; videos are made from the PNGs.

## lidar
Index `lidar/<sensor>.csv`: `seq,acq_ns,points,t_first_ns,t_last_ns,file`. Scan file: little-endian records
`float32 x,y,z,intensity; int32 t_offset_ns` (20 B/point), sensor frame, `t_offset_ns` >= 0 non-decreasing relative to `acq_ns`.
Absolute point time = `acq_ns + t_offset_ns`.

## events.jsonl
One JSON object per line with `t_ns`, `kind`, and:
`collision`: `other, impact_speed_mps, normal_enu`; `near_miss`: `distance_m` (surface to surface), `obstacle`, `threshold_m`, `t_min_ns`;
`detection`: `target, camera, range_m, u, v` (ground-truth visibility: in frustum, in range, line of sight clear; emitted at each camera acquisition);
`marker`: `name`.

## Splits and the held-out guard
`Tools/DroneData/splits.json` (`crane-drone-splits/1`) assigns forest seed ranges and disturbance families to splits:

| split | seeds | wind | lighting |
|---|---|---|---|
| dev (iteration, demos, smoke; never reported) | 0 to 1999 | calm, light_breeze | GoldenHour |
| train | 2000 to 69999 | calm, light_breeze, gusty | GoldenHour, Overcast |
| val | 70000 to 79999 | moderate_gusty | Dusk |
| test | 80000 to 89999 | strong_steady | Noon |
| HELD-OUT | 90000 to 99999 | storm_gust, turbulent_shear | LowSunGlare, HeavyFog |

Seeds outside every range are refused. Held-out seeds or disturbance values are refused unless `--allow-heldout` is passed.
The guard fails closed (no `splits.json` or no `--drone-seed` = refuse). Entry points:
`DroneSplitGuard.EnforceFromCommandLine(args)` (C#, reads `--drone-seed --drone-wind --drone-lighting --allow-heldout --drone-splits`),
`DroneRunSession.Begin/BeginFromCommandLine` (called before anything is written) and `splits.enforce_argv` (Python).
**Capture owners (`TheaterCapture`, demo director) must call `DroneSplitGuard.EnforceFromCommandLine` before building the scene.**
Ranges must not change after the first dataset is generated. "Lighting" and "wind" names beyond the existing presets are reserved, not yet implemented.

## Seeded reproducibility
Contract: same `scene_seed`, scene, physics profile/params and trajectory spec reproduce the same logs. `t_ns`, event sequence and
sensor index stamps must match exactly; state values within the tolerance below (`check_repro.py` defaults, override with `--tol-*`).
All stub randomness comes from `DroneRng` (xorshift64*), never `System.Random` or `UnityEngine.Random`. PhysX runs in Script mode (`Physics.Simulate(1 ms)`).
**Measured (stub plant, seed 1337, 6 s, 1 kHz, 6001 state rows, one host, Unity 6000.5.10f1, `-nographics`):**
two runs in one Editor process and a run in a second, separate Unity process are **bitwise identical** in `state.csv`, `imu.csv`
and `events.jsonl` (max |d| = 0 for position, quaternion, velocity, rate, acceleration, rotors, battery; `cmp` clean), including
through a rigid-body collision with the box. A different seed differs (max |dpos| about 11 m). So the measured tolerance on this host
is 0. The `check_repro.py` defaults (pos 1e-6 m, quat 1e-6, vel 1e-6 m/s, rate 1e-5 rad/s, body accel 1e-2 m/s^2, rotors 1e-3, battery 1e-6)
are deliberately loose guard rails for other hosts or CPUs (PhysX and `Math.Exp/Sin` are not guaranteed bit-exact across CPU architectures);
tighten them once a second host has been measured. NOT measured: cross-host, GPU/render-mode runs, the real aerial plant with
sub-stepped articulations, multi-body contact with many trees. Rerun `check_repro.py` after each physics change.
Evidence label: FIXTURE_VALIDATED (stub plant).

## Commands
```
# Unity (one at a time):  unity-drones LOG -executeMethod Crane.Drones.Logging.DroneReproCli.Run \
#     --drone-seed 1337 --drone-out /tmp/flight --drone-duration 6 --drone-repeat 2
fsrun python3 Tools/DroneData/drone_log.py /tmp/flight/run_0 --plot /tmp/flight/traj.png
fsrun python3 Tools/DroneData/check_repro.py /tmp/flight/run_0 /tmp/flight/run_1
fsrun python3 -I -m unittest discover -s Tools/DroneData -p "test_drone_data.py"
```
Unity tests: `-runTests -testPlatform EditMode -testFilter DroneLogging`.

## Plugging in the real plant
Implement `IDroneStateSource` (and `IDroneImuSource`), build a `DroneRunManifest`, call `DroneRunSession.Begin`, then
`OnSubstep(tick, sub)` after every physics substep; push cameras/lidar with acquisition stamps; attach `DroneCollisionRelay`
to the body and `DroneDetectableTarget` to targets.
