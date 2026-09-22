# RoboBoat navigation baseline

This is the code-verified RoboBoat navigation baseline and the evidence for the September 22,
2026 terminal-control/speed correction. It describes the dedicated `roboboat-docking` worktree;
the environment scene and manual controller were not edited.

## Configuration identity

- Reconnaissance start: superproject `d44bd9de525bcf2f8dc429add3b566066e15c0a3`,
  `crane_ml` `60439faa5df320e54082ad0dfedad635442adf07`, and `astro_dock`
  `36202373ae186a8fd247a20b7b477312a744de99`.
- Diagnostic fixture commit: `caedee3` (`full-forward` command response only).
- Corrected active navigation commit: `2778bab`.
- Visible-bow/frame correction baseline: superproject `cb0c86523f0a7d2596497a5a5fbd88724b97da18`,
  `crane_ml` `e0495c623bcc3542be6195631d7f98e4e1286f83`, and `astro_dock`
  `36202373ae186a8fd247a20b7b477312a744de99`. The frame correction is documented by
  the file-level tests and runtime artifacts below because a commit cannot name its own hash.
- ROS image: `lunarzdev/astro:cuda`; installed Nav2 Debian packages are Jazzy `1.3.12`.
- Player SHA-256: `a7ad5b156bd9a1232544ff6fc12e5f863d8f1f2348c5b141f5c3d4230e5be292`.
- Locked scene SHA-256: `db1399a23ff791dcf49d389381cf6f745169b671b8fb87fdc8ddeebe2342ed84`.
- Active parameters are [nav2_controller_fixture.yaml](../Tools/Performance/nav2_controller_fixture.yaml),
  loaded explicitly by [run_nav2_controller_fixture.sh](../Tools/Performance/run_nav2_controller_fixture.sh).
  Its pre-fix blob SHA-256 was `047e0981...b401385c`; the corrected blob is
  `9737552e...179a92`; the visible-bow/BT revision is
  `0373293d0b18e900e34c9606af63718d0f66b8dba7660c72eadd144e20219039`.
- The active BT is
  [nav2_roboboat_distance_replanning.xml](../Tools/Performance/nav2_roboboat_distance_replanning.xml),
  SHA-256 `e47dadf5bb1228f031e1a7a15d60f0a784c5101ffff9fd11fc36ea7458ce3dce`.
- `nav2_land_fixture.yaml` and the land/TurtleBot launchers are alternatives for land fixtures and
  do not control RoboBoat runs. No velocity smoother executable is launched. No command-line BT
  override was used for the final runs; the active BT is selected by the parameter file above.

## Current architecture

```text
NavigateToPose (/navigate_to_pose)
  -> BT Navigator, nav2_roboboat_distance_replanning.xml
  -> NavfnPlanner /compute_path_to_pose (A*, rolling odom costmap)
  -> RegulatedPurePursuitController /follow_path at 10 Hz
  -> Twist on /nav2/cmd_vel
  -> nav2_follow_path_fixture.py stamps/forwards it as TwistStamped
     on /crane/cmd_vel_stamped
  -> ROS TCP endpoint -> Unity ROSSubscriber
  -> ROSOmniXCommand (desired body velocity -> normalized PI/feed-forward effort)
  -> OmniXController (four-thruster X mixer and common saturation)
  -> Thruster/MotorBase (shaft-velocity actuation; quadratic submerged thrust)
  -> ArticulationBody + buoyancy + GeneralDynamics + water current/waves
  -> CraneROSNavigationState at 50 Hz
  -> /crane/odom and /tf (odom -> base_link plus sensor children)
  -> Nav2 controller/costmaps/BT navigator
```

`run_nav2_controller_fixture.sh` launches four Nav2 servers directly, remaps controller and
behavior output to `/nav2/cmd_vel`, and launches the lifecycle manager. The fixture is the only
Twist-to-TwistStamped bridge in this path. ROS callbacks enqueue commands; Unity applies the latest
accepted command only on the 50 Hz physics boundary. Commands become stale after 25 ticks (0.5 s),
at which point effort is zeroed and integrators reset.

## Active Nav2 stack

The speed/terminal settings differ from the initial reconnaissance baseline in four controller
values: desired speed `0.15 -> 0.8 m/s`, fixed lookahead `0.8 -> 2.0 m`, terminal rotation
`false -> true`, and rotate speed `0.4 -> 0.2 rad/s`. The later visible-bow correction changes the
ROS boundary frame, and the active BT changes replanning cadence; neither changes boat physics.

- Planner: `nav2_navfn_planner::NavfnPlanner`, A* enabled, unknown allowed, 0.5 m tolerance,
  expected 5 Hz.
- Controller: `nav2_regulated_pure_pursuit_controller::RegulatedPurePursuitController`, 10 Hz,
  forward-only (`allow_reversing=false`), fixed 2.0 m lookahead, collision detection enabled,
  0.8 m/s desired surge, 0.02 m/s minimum approach speed, 2.0 m approach-scaling distance, and
  0.2 rad/s rotate-to-heading speed. It is differential-style pure pursuit: it emits surge and yaw,
  never `linear.y`, follows geometric path tangent, and now rotates in place for terminal heading.
- Goal checker: stateful `StoppedGoalChecker`; 0.20 m XY, 0.35 rad yaw, 0.05 m/s stopped
  translation, and 0.05 rad/s stopped rotation.
- Progress checker: `SimpleProgressChecker`; 0.05 m in 20 s.
- Recovery behaviors: Spin, BackUp, DriveOnHeading, Wait at 10 Hz; 0.5 rad/s maximum recovery
  rotation, 0.5 rad/s2 rotational acceleration. The active recovery-capable BT replans after each
  1 m of translation. This refreshes a wave-disturbed long approach but preserves the final
  sub-metre path; unconditional 1 Hz replanning was measured replacing it with a quantized
  two-pose Navfn path and causing terminal orbits.
- Local costmap: rolling `odom`, `base_link`, 20 x 20 m, 0.10 m cells, 10 Hz update/2 Hz publish.
- Global costmap: rolling `odom`, `base_link`, 60 x 60 m, 0.20 m cells, 5 Hz update/1 Hz publish.
- Both costmaps use a 0.80 m robot radius, 3-D VoxelLayer from `/points`, obstacle range 1-20 m,
  raytrace range 1-25 m, heights -1.5 to 3.0 m, plus InflationLayer radius 1.0 m and factor 3.0.
- All nodes use simulation time. Odometry is `/crane/odom`; the final plant command is
  `/crane/cmd_vel_stamped`. There is no map server, AMCL, static map, or velocity smoother in this
  launch path.
- Other installed controllers, not active: DWB, MPPI, Graceful Controller, and Rotation Shim.
  The correction does not rely on any uninstalled plugin and does not change controller family.

RPP assumes a nonholonomic plant that can promptly realize requested surge/yaw and can rotate in
place. The vessel is actually omnidirectional and inertial. The first mismatch is harmless while
forward-only motion is desired; the second is mitigated by the 2 m approach slowdown and verified
body-velocity loop. Wave drift remains external disturbance, not commanded sway.

## Boat command contract

At the Unity boundary, TwistStamped is a **desired body velocity**, not normalized thrust:

- `linear.x`: ROS FLU forward/surge, m/s; imported physics-body local `-X`, toward the LiDAR and
  catamaran noses and away from the chase camera/Blastoise head.
- `linear.y`: ROS FLU left/sway, m/s; imported physics-body local `-Z`.
- `angular.z`: ROS FLU counter-clockwise/yaw rate, rad/s; Unity local `-Y` axial rotation.

The logical ROS body frame is a -90 degree Unity-Y rotation from the imported physics body. The
same transform is applied to commands, odometry pose/twist, root TF, sensor-child TF, and docking
evaluation. `ROSOmniXCommand` clamps translation and yaw to +/-1.0 in their SI units. It measures
authoritative body-frame velocity each fixed step. Translation uses normalized feed-forward `0.21`, Kp `0.1`, Ki
`0.1`, and integral effort limit `0.2`; yaw uses square-root feed-forward `0.24` and Kp `0.05`.
Each resulting axis effort is clipped to [-1, 1]. The active RPP does not use sway. Manual input
continues to call `OmniXController.SetMotion` directly and was not edited.

The current yaw sweep confirms the contract: 0.025/0.05/0.1/0.2/0.4 rad/s requests produced
0.028/0.055/0.106/0.211/0.415 rad/s tail response. A directly-ahead 8 m goal produced exactly zero
lateral command; its small measured sway is consistent with the enabled waves.

## Thruster model

There are four fixed, diagonal thrusters around the COM. Their imported base-local positions in
metres are FL `(-0.26637,-0.28232,-0.01287)`, FR `(-0.26636,-0.28432,+0.01287)`,
RL `(+0.26625,-0.28232,-0.01277)`, and RR `(+0.26630,-0.28432,+0.01282)`; the scene rotates the
base +90 degrees about Unity Y. Their serialized rotations are the four complementary 45-degree
diagonals, and force is applied at each ArticulationBody position. The LiDAR/catamaran noses define
the bow; the Blastoise head is aft.

For requested normalized effort `(f, s, r)`, `OmniXController` computes:

```text
[FL]   [-1 -1 -1] [f]
[FR] = [ 1 -1  1] [s]
[RL]   [-1  1  1] [r]
[RR]   [ 1  1 -1]
```

All four values are divided by `max(1, max(abs(value)))`, preserving the requested vector while
saturating. The matrix has rank 3, symmetric pure-surge/pure-sway/pure-yaw columns, and no algebraic
coupling. `ROSOmniXCommand.ToControllerMotion` supplies the sign conversion needed for ROS FLU.
Axis-response tests verified surge, sway, and yaw signs independently. The manual input path still
calls `OmniXController.SetMotion` directly and was not modified.

`OmniThrusterConfig.asset` uses shaft velocity mode, +/-500 rad/s, 1 N m maximum motor torque,
response factor 0.98, quadratic coefficient -0.014, equal reverse factor 1.0, and 0.03 m full
submersion depth. `Thruster.FixedUpdate` applies
`sign(shaft_speed) * shaft_speed^2 * thrustK` along the transformed propulsor axis at the thruster
body position with `AddForceAtPosition`; partial submersion scales force linearly. The shared asset
does not contain a deadband.

## Physical model

The active scene instantiates `Blastoise.prefab`, not the similarly named simplified OmniBoat.
Scene overrides set `base_link` to 3 kg, linear damping 4, angular damping 15, COM Y -0.23 m, and
an explicit inertia tensor `(10,15,10)` with a serialized inertia rotation. Each catamaran hull link
is 8 kg with COM Y -0.33 m and implicit inertia; the other fixed child bodies also contribute to
the articulation's total mass. Gravity is enabled. Physics runs at 0.02 s.

Submersion clips a simplified hull mesh against the HDRP WaterSurface (3 m patch, resolution 20).
Buoyancy applies displaced-water force at the computed center of buoyancy. Enabled
`GeneralDynamics` applies triangle-based viscous resistance and pressure/suction drag (linear 100,
quadratic 30, reference speed 1 m/s). The Fossen component and its added-mass model are present but
disabled. `Current` is enabled, and the scene water surface supplies waves/current. These values are
scene/model inputs with no calibration provenance in code; they are therefore treated as estimates,
not measured real-platform constants, and were not changed.

The earlier controller-X full-effort test reached 1.246 m/s peak and 1.216 m/s tail, traveled
17.47 m, then coasted 0.655 m after release. That controller axis is imported-body `+Z`, not the
visible bow, so it is evidence for plant response but not a valid visible-forward top-speed test.
The corrected ROS surge loop accurately realizes requests through 1.0 m/s, which is also the
current adapter clamp. No physics change or claim about the reported 1.8 m/s bow speed is justified
until a corrected full-effort bow-axis test is compared with real mass, full-throttle speed,
propulsor thrust/RPM, and hull scale.

## Geometry and frames

- `odom` is the global/planning frame and `base_link` is the odometry child and rotation reference.
  Imported physics-body `(-X, -Z, Y)` maps to logical ROS `(X, Y, Z)`; angular vectors receive the
  required handedness sign.
- `CraneROSNavigationState` publishes odometry and TF at 50 Hz, including `lidar_link`,
  `front_camera_link`, `imu_link`, and `gps_link` when present.
- The 3-D LiDAR is on `lidar_link` at the bow, 10 Hz, 360-degree horizontal FOV, 16 vertical beams,
  1.3-100 m active range, publishing `points` (resolved as `/points`). This agrees with the physical
  bow; the character head is not the forward reference.
- Measured hull envelope used by the independent docking evaluator is 1.063 m long by 0.895 m beam.
  The costmap's 0.80 m circular radius exceeds the hull circumscribed radius (~0.695 m), so it is
  conservative rather than unrealistically small.
- The evaluated berth is 3.0 m deep by 2.0 m wide. An aligned physical hull has roughly 0.55 m
  lateral clearance per side, while the logical 1.6 m costmap diameter leaves 0.20 m per side.
  The successful far run proves the logical circle still permits entry; it is conservative in the
  rectangular dock and may reject tighter valid maneuvers.

## Existing evidence and capabilities

The repository already contains FollowPath/NavigateToPose fixtures, generated straight/gentle/S
paths, a supplied known dock path, command/yaw/thruster response fixtures, independent docking and
collision predicates, path metrics/plots, costmap-clearance audit, and retained per-run trajectories.
They measure action status, RMS/max cross-track and heading error, body velocity, final pose,
stopping/coast, contact, dock-region containment, recovery events, and transport validity.

Corrected results (all scene/physics unchanged, `linear.y` command always zero):

| Run | Result | Fixture time | Key evidence |
|---|---:|---:|---|
| ROS surge step, 1.0 m/s for 15 s | complete | 15 s | +15.363 m bow direction, +0.582 m wave drift; 0.9996 m/s tail surge, 0.0023 m/s tail sway |
| 8 m fixed FollowPath | success | 26.43 s | +7.837 m bow direction, -0.317 m wave drift, 0.025 rad final yaw |
| 8 m NavigateToPose, active distance BT | success | 16.95 s | +7.854 m bow direction, zero sway command, 0.032 rad final yaw |
| far dock `(0.864, -27.587, +pi/2)`, active distance BT | Nav2 success | 46.03 s action / 53.08 s observed | 23.37 m traveled; action result 0.267 m XY and 0.019 rad yaw; hull inside dock, zero contact |

The old direct-ahead baseline took 65.83 s at 0.15 m/s. A naive 0.30 m/s change with terminal
rotation disabled reached within 0.005 m, then looped nearly 180 degrees and timed out at 70 s.
Enabling terminal rotation made that exact repro succeed in 38.18 s. The later visible-bow audit
found a separate exact 90 degree boundary error: ROS surge had been bound to imported-body `+Z`
while the camera/LiDAR geometry proves the visible bow is imported-body `-X`. The scene-loaded
regression failed before correction with a bow/surge dot product of `9.31e-8` and passes after it.
RPP has no critics and emits no sway.

## Limitations, gaps, and next experiment

Resolved bottlenecks were the exact 90 degree ROS/imported-body frame error, unconditional 1 Hz
terminal replanning, controller terminal-heading configuration, and overly short speed/lookahead
settings. Goal/stopped checking and the conservative footprint were not the cause of the visible
sideways motion. Remaining gaps are:

1. **Plant speed/scale:** corrected bow-axis full-effort speed is not yet measured; the old 1.22 m/s
   result was for imported-body `+Z`. Confirm with a corrected open-loop bow-axis test plus measured
   real mass, hull dimensions, full-throttle speed, current draw/RPM, and bollard thrust.
2. **Disturbance robustness:** the active distance-triggered BT passed the direct-ahead and far-dock
   runs, but wave severity/current and multiple seeds have not been swept. Confirm with declared
   low/nominal/high water settings and repeated seeds; falsify concern if success, contact,
   clearance, and final pose remain bounded.
3. **Logical shape:** the 0.8 m circle is safe but conservative for a catamaran. A polygon could
   improve tight clearances, but only a failed clearance audit would justify changing it.
4. **High-speed turns:** 0.8 m/s is validated on a gentle turn and far planner path, not every
   obstacle geometry. The 1.0 m/s adapter ceiling should not be raised until stopping-distance and
   curved-path sweeps remain collision-free.
5. **Uncommanded station keeping:** after the far action succeeded, seven seconds with no command
   allowed waves to drift the boat 0.219 m. It stayed inside the physical dock with zero contact,
   but crossed the evaluator's 0.4 m pose tolerance before the required five-second settle latch.
   Confirm whether docking requires active hold after Nav2 success before adding any hold layer;
   do not hide this evidence by loosening tolerances or hydrodynamics.

The smallest next experiment is now a full-effort speed/RPM/thrust comparison against measured
real-platform mass, dimensions, propulsor RPM/current, bollard thrust, and calm-water top speed.
That directly separates a scale/thrust-model error from an incorrect 1.8 m/s expectation without
speculative hydrodynamic tuning. A declared low/nominal/high wave sweep is the next navigation-only
robustness check.
