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
- ROS image: `lunarzdev/astro:cuda`; installed Nav2 Debian packages are Jazzy `1.3.12`.
- Player SHA-256: `a7ad5b156bd9a1232544ff6fc12e5f863d8f1f2348c5b141f5c3d4230e5be292`.
- Locked scene SHA-256: `db1399a23ff791dcf49d389381cf6f745169b671b8fb87fdc8ddeebe2342ed84`.
- Active parameters are [nav2_controller_fixture.yaml](../Tools/Performance/nav2_controller_fixture.yaml),
  loaded explicitly by [run_nav2_controller_fixture.sh](../Tools/Performance/run_nav2_controller_fixture.sh).
  Its pre-fix blob SHA-256 was `047e0981...b401385c`; the corrected blob is
  `9737552e...179a92`.
- `nav2_land_fixture.yaml` and the land/TurtleBot launchers are alternatives for land fixtures and
  do not control RoboBoat runs. No velocity smoother executable is launched. No custom BT is passed.

## Current architecture

```text
NavigateToPose (/navigate_to_pose)
  -> BT Navigator, stock navigate_to_pose_w_replanning_and_recovery.xml
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

The corrected settings differ from the reconnaissance baseline only in the four marked controller
values: desired speed `0.15 -> 0.8 m/s`, fixed lookahead `0.8 -> 2.0 m`, terminal rotation
`false -> true`, and rotate speed `0.4 -> 0.2 rad/s`.

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
  rotation, 0.5 rad/s2 rotational acceleration. The stock NavigateToPose replanning/recovery BT is
  used because `default_nav_to_pose_bt_xml` is not overridden.
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

- `linear.x`: ROS FLU forward/surge, m/s; Unity local `+Z`.
- `linear.y`: ROS FLU left/sway, m/s; Unity local `-X`.
- `angular.z`: ROS FLU counter-clockwise/yaw rate, rad/s; Unity local `-Y` axial rotation.

`ROSOmniXCommand` clamps translation and yaw to +/-1.0 in their SI units. It measures authoritative
body-frame velocity each fixed step. Translation uses normalized feed-forward `0.21`, Kp `0.1`, Ki
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
Axis-response tests verified surge, sway, and yaw signs independently.

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

Full manual-equivalent forward effort (normalized effort 1.0 for 15 s) reached 1.246 m/s peak and
1.216 m/s tail, traveled 17.47 m, then coasted 0.655 m after release. Sway stayed within 0.009 m/s
and net yaw was 0.0012 rad. Thus the existing plant is straight and stable at full effort, but it
cannot produce the reported 1.8 m/s without changing the current thrust/drag/scale model. The
software adapter also clamps requests at 1.0 m/s. No physics change is justified until the real
mass, full-throttle speed, propulsor thrust curve, and hull scale are reconciled.

## Geometry and frames

- `odom` is the global/planning frame and `base_link` is the odometry child and rotation reference.
  Unity `(Z, -X, Y)` maps to ROS `(X, Y, Z)`; angular vectors receive the required handedness sign.
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
| 8 m directly ahead, 0.8 m/s | success | 19.93 s | 0.103 m final XY, 0.021 rad yaw, 0.870 m/s peak surge |
| 15 m gentle turn | success | 30.33 s | 0.090 m RMS / 0.292 m max cross-track, no contact |
| far dock, seeds 1000/2000/3000 | 3/3 success | 57.88-66.68 s | dock predicate true, no contact, stopped, 0.319-0.410 m final clearance |

The old direct-ahead baseline took 65.83 s at 0.15 m/s. A naive 0.30 m/s change with terminal
rotation disabled reached within 0.005 m, then looped nearly 180 degrees and timed out at 70 s.
Enabling terminal rotation made that exact repro succeed in 38.18 s. This falsifies a sway critic
problem: RPP has no critics and emits no sway. The failure was forward-only terminal geometry.

## Limitations, gaps, and next experiment

Resolved bottlenecks, in order, were controller terminal-heading configuration and overly short
speed/lookahead settings. Goal/stopped checking, the conservative footprint, planner geometry, and
waves were not the cause of the aggressive circle. Remaining gaps are:

1. **Plant speed/scale:** full effort is 1.22 m/s steady rather than the reported 1.8 m/s. Confirm
   with measured real mass, hull dimensions, full-throttle speed, current draw/RPM, and bollard
   thrust; falsify by showing the simulated quantities already match those measurements.
2. **Disturbance robustness:** three simulator seeds now pass the far dock, but wave severity/current
   itself has not been swept. Confirm with declared low/nominal/high water settings; falsify concern
   if success, contact, clearance, and final pose remain bounded.
3. **Logical shape:** the 0.8 m circle is safe but conservative for a catamaran. A polygon could
   improve tight clearances, but only a failed clearance audit would justify changing it.
4. **High-speed turns:** 0.8 m/s is validated on a gentle turn and far planner path, not every
   obstacle geometry. The 1.0 m/s adapter ceiling should not be raised until stopping-distance and
   curved-path sweeps remain collision-free.

The smallest next experiment is now a full-effort speed/RPM/thrust comparison against measured
real-platform mass, dimensions, propulsor RPM/current, bollard thrust, and calm-water top speed.
That directly separates a scale/thrust-model error from an incorrect 1.8 m/s expectation without
speculative hydrodynamic tuning. A declared low/nominal/high wave sweep is the next navigation-only
robustness check.
