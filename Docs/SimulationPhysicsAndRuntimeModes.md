# CRANE simulation physics and runtime modes

This guide is the practical reference for vehicle authors, navigation developers, training
operators, and replay users. It describes what the current checkout actually simulates, the
coordinate and lifecycle conventions that components rely on, and how to launch each supported
runtime profile. Architecture rationale and measured performance remain in
[ArchitectureAndDesign.md](ArchitectureAndDesign.md) and
[PerformanceEngineering.md](PerformanceEngineering.md).

## Physical and software conventions

### Units and coordinate frames

CRANE uses SI units unless a serialized field or API explicitly says otherwise:

| Quantity | Convention |
|---|---|
| Position, distance | metres |
| Linear velocity | metres/second |
| Linear acceleration | metres/second² |
| Mass | kilograms |
| Force | newtons |
| Torque | newton-metres |
| Unity inspector angles | normally degrees |
| Angular velocity and hydrodynamic rates | radians/second unless the API says degrees |
| Simulation time | scaled Unity seconds |

Unity world space is left-handed and Y-up. A normally oriented vehicle uses local `+Z` as
forward, `+X` as right, and `+Y` as up. ROS messages use the conversions supplied by the Unity
ROS-TCP Connector, principally ROS FLU body coordinates (forward, left, up). Code that converts
between Unity and ROS must use `ROSGeometry` conversion helpers or an explicitly tested mapping;
do not swap or negate axes ad hoc. `FossenDynamics`, for example, forms its six-state body vector
in ROS FLU and converts the resulting force/torque back into Unity coordinates before applying it.

Topic frame IDs are strings configured on sensor components, such as `base_link`, `imu_link`,
`lidar_link`, and `front_camera_link`. A frame ID does not itself create or publish a TF edge.
The scene/prefab transform hierarchy and the external ROS TF configuration must agree. The
opt-in production navigation adapter described below is the exception: it publishes the
authoritative `odom -> base_link` edge together with odometry from the same PhysX body.

### Time and stepping

The project setting is a fixed physical step of `0.02` simulated seconds (50 Hz). A process-wide
CRANE clock advances its authoritative tick exactly once per Unity `FixedUpdate`. Scene loads
start a new episode generation. `Time.timeScale` changes how rapidly Unity attempts to execute
fixed steps in wall time; it does not enlarge the physical timestep.

Simulation-critical code still uses Unity lifecycle callbacks:

- `Awake`: cache required bodies/configuration and install transport helpers.
- `Start`: resolve cross-component references and allocate persistent buffers.
- `FixedUpdate`: actuator dynamics, forces, wheel/rotor commands, sensor schedules, and normal ROS
  publication schedules.
- `Update`/`LateUpdate`: presentation and a small number of legacy components.
- render callbacks and `AsyncGPUReadback`: RGB/depth acquisition.
- ROS and ArduPilot JSON/SITL callbacks: enqueue commands; actuator mutation occurs at a fixed-step boundary.
- `WaitForFixedUpdate`: authoritative replay capture after physics.

`Physics.Simulate()` alone does not invoke these callbacks. CRANE therefore does not currently
support a safe manual-step mode; using it would skip systems or run them twice.

### Physics bodies and contacts

`IPhysicsBody` is the shared force/pose abstraction. `RigidbodyAdapter` and
`ArticulationBodyAdapter` supply the implementation for Unity PhysX bodies. Normal vehicle motion
must come from forces, torques, WheelCollider commands, articulation drives, gravity, and contact.
Direct transform or velocity assignment is reserved for reset and authoritative replay.

Mass, inertia tensor, centre of mass, collider geometry, solver settings, and collision layers
are part of the physical model. Rendering LOD must not silently replace collision or sensor-query
geometry. WheelCollider tire friction is its own slip/friction model and does not use ordinary
`PhysicsMaterial` friction in the same way as a normal collider.

## Surface-vehicle physics

The production `Roboboat Course` vehicle is a force-driven Rigidbody system coupled to HDRP
water. Its physical chain is:

```text
HDRP spectrum and displacement
  → CPU-accessible water search data
  → moving water patch around the hull
  → clipped submerged hull mesh
  → buoyancy/current/hydrodynamic forces
  → thruster forces and PhysX contacts
  → Rigidbody integration
```

`Submersion` constructs a grid patch around the hull. `Patch` batches HDRP
`WaterSimulationSearchJob` searches with an eight-iteration limit, 0.01 m error target, and water
deformation enabled. The hull mesh is clipped against that sampled surface to obtain submerged
triangles, face centres/normals, centroid, and displaced volume.

`Buoyancy` applies Archimedes' force at the calculated centre of buoyancy:

```text
F_b = ρ_water · g · V_displaced
```

where the checked-in constants use water density and gravitational acceleration in SI units.
`GeneralDynamics` can apply face-based viscous resistance and pressure/suction drag using the
local point velocity `v_body + ω × r`, submerged face area, and face normal. `Current` computes
quadratic relative-flow force per submerged face. Spatial HDRP current maps are queried per face;
when both large- and ripple-current maps are disabled, the uniform direction is queried once per
hull/tick and shared.

`Thruster` is a rotating-motor model with water-dependent thrust. It applies force at the
thruster location using:

```text
F = sign(ω) · ω² · K_thrust · direction · submersionFraction
```

Reverse thrust is multiplied by the configured `backK`. The submersion fraction ramps from zero
to one over the configured thruster height. Because the force is applied away from the centre of
mass, PhysX naturally produces the corresponding moment.

Optional `WindForce` integrates exposed mesh faces against a configured horizontal wind vector.
This is a simplified force model and is not a calibrated atmospheric solver.

Do not stack multiple buoyancy or drag implementations on the same hull unless the combined model
is intentional and validated. `Buoyancy`, voxelized buoyancy, simple drag, `GeneralDynamics`, and
`FossenDynamics` are alternative/complementary building blocks, not automatic fidelity tiers.

## Underwater-vehicle physics

Underwater vehicles use the same HDRP search, submersion, buoyancy, current, thruster, and PhysX
contact foundations as surface craft. The important differences are full or near-full
submersion, six-degree-of-freedom actuation, and stronger rotational/heave hydrodynamics.

`FossenDynamics` implements a configurable diagonal six-DOF approximation with:

- linear and quadratic damping in surge, sway, heave, roll, pitch, and yaw;
- diagonal added-mass derivatives;
- a limited Coriolis matrix based on the configured surge/sway terms;
- body-relative linear and angular velocity transformed through ROS FLU conventions.

The implementation estimates acceleration by a fixed-step finite difference and applies the sum
of enabled damping, Coriolis, and added-mass forces. Coefficients must be identified for the actual
vehicle; the defaults are not universal hydrodynamic truth.

`FullOmniController` maps translation plus roll/yaw requests to four horizontal corner thrusters
and two vertical thrusters. `OmniXController` maps planar translation/yaw to four X-layout
thrusters. These classes are command mixers, not dynamics models. Real behavior still comes from
the configured motor/thruster, hull, mass/inertia, and water components. `Ballast` is a simple
offset downward force for Rigidbody experiments, not a dynamic ballast-tank/fluid model.

Aquatic execution is not graphics-free. HDRP's GPU simulation and readback must remain alive for
valid hull queries. Linux aquatic workers currently require a graphics-capable Vulkan path and
`SDL_VIDEODRIVER=x11` on the validated machine. `-nographics` aquatic runs are rejected by the
`train-cpu` profile rather than silently changing the physical water field.

## Ground-vehicle physics

The current representative land fixture is `AckermannRoverDynamics`, a four-wheel Rigidbody
vehicle backed by PhysX `WheelCollider`s. It does not set transforms or body velocity during
ordinary motion.

Inputs are normalized throttle `[-1, 1]`, steering `[-1, 1]`, and brake `[0, 1]`. The current
fixture is four-wheel drive: every wheel receives the filtered motor torque. Motor response is a
first-order lag:

```text
response = 1 - exp(-fixedDeltaTime / motorTimeConstant)
appliedTorque = lerp(appliedTorque, targetTorque, response)
```

Torque is suppressed when the configured maximum forward/reverse speed is reached. With no
throttle or brake command, a rolling-resistance brake torque is estimated from vehicle weight,
wheel radius, coefficient, and wheel count. An explicit brake command scales the maximum brake
torque.

Front-wheel steering uses Ackermann geometry. For wheelbase `L`, track `W`, and requested centre
steering angle `δ`, the centre radius is `R = L / tan(|δ|)`. Inner and outer wheel angles use
`atan(L / (R ∓ W/2))` and are rate-limited per fixed step. Wheel suspension, tire slip curves,
spring/damper values, wheel mass/radius, chassis mass/inertia, and centre of mass remain serialized
WheelCollider/Rigidbody properties in the validation scene.

The current automated fixture validates flat-ground acceleration, coast-down, braking distance,
and turning-radius response. Slope hold, curb traversal, suspension transients, calibrated slip,
skid steering, and physical omni wheels remain unqualified. Existing Ackermann/omni controller
names elsewhere in the project must not be treated as proof of those physical models.

## Aerial-vehicle physics

`MultirotorDynamics` is the current quadrotor vertical slice. It is a six-DOF Rigidbody with rotor
forces applied at four serialized transforms ordered front-left, front-right, rear-right,
rear-left. Inputs are normalized collective `[0, 1]` and roll/pitch/yaw `[-1, 1]`.

The mixer adds bounded axis terms to collective and clamps each normalized motor target. Motor
speed follows a first-order lag. Per-rotor thrust and reaction torque are:

```text
T_i = T_max · motorSpeed_i²
τ_i = yawSign_i · T_i · reactionTorquePerThrust
```

`AddForceAtPosition` generates roll/pitch moments from rotor placement. Alternating reaction
torque signs generate yaw. Hover speed is computed analytically as
`sqrt(mass · |g| / (4 · T_max))`.

Air-relative local velocity includes steady wind plus a sinusoidal gust. Each body axis receives
linear-plus-quadratic drag, and local angular rates receive axis-wise angular damping. Unity
supplies gravity, inertia integration, collision, takeoff, landing, and ground contact. The
current validation checks hover equilibrium, vertical acceleration, roll/pitch/yaw response,
motor saturation, wind/gust displacement, and landing.

This is not yet a calibrated production airframe. Propeller RPM/aerodynamic lookup tables,
voltage/battery sag, rotor inflow, ground effect, sensor/flight-controller qualification, and a
real PX4/ArduPilot SITL closed loop remain future work.

## Sensors, ROS, and actions

RGB and GPU `32FC1` depth use bounded asynchronous readback queues. Depth is independent of RGB;
Train-GPU disables RGB but preserves GPU depth. Train-CPU replaces configured GPU depth components
with persistent, batched PhysX raycasts and Burst packing while leaving every `Camera` disabled.
Both backends publish top-to-bottom `32FC1` optical-axis Z in metres. Geometric misses outside the
near/far interval are quiet NaN. The current image contract records width, height,
encoding, row step, optical frame, acquisition tick/time, completion tick/time, episode, and byte
count in replay metadata. Camera info is published separately.

Geometric depth uses physical colliders and camera culling layers. It correctly models opaque
collider occlusion, thin colliders, and moving geometry, but it does not infer alpha-tested or
transparent appearance and cannot see an HDRP water surface without matching query geometry.
Those semantics are why it is currently a non-aquatic backend rather than a drop-in claim of GPU
image equivalence.

LiDAR uses persistent native arrays, Burst ray-command generation and PointCloud2 packing, and
batched PhysX raycasts. The production 72,000-point scanner uses batch size 64. Semantic 3D
detections use registered scene objects, camera frustum/range tests, centre-ray occlusion, and a
distance-based confidence model without RGB inference. They do not expose every known object, but
partial visibility, class confusion, correlated noise, and false-positive models remain limited.

ROS publishers run from simulated-time schedules and retain fractional period remainder. ROS or
SITL callbacks enqueue commands; `CraneActionGate` checks episode, sequence, source observation,
receive tick, and bounded-lag policy before fixed-step application. `--crane-disable-ros` suppresses
ROS-TCP and SITL while still exercising message construction for benchmarks. Use
`--crane-disable-ros-tcp` or `--crane-disable-sitl` when only one boundary should be disabled. This is not end-to-end
zero-copy: ROS-TCP remains a serialization boundary.

The optional production aquatic adapter subscribes to `geometry_msgs/TwistStamped` and maps ROS
FLU forward/lateral/yaw commands into the Omni-X mixer. It never mutates thrusters in the ROS
callback: a single-slot mailbox applies the newest valid command in `FixedUpdate`, and a
simulated-tick watchdog commands zero after a bounded hold interval. The header stamp is converted
to the source-observation tick. `OmniXController` combines translation and yaw before normalizing
all four thrusters, so a curved Nav2 command no longer discards its forward component.

Enable it with `--crane-ros-cmd-vel /crane/cmd_vel_stamped`. Velocity normalization defaults to
1 m/s and 1 rad/s and can be set with `--crane-cmd-vel-linear-scale` and
`--crane-cmd-vel-yaw-scale`. Use `--crane-command-timeout-ticks` for the hold watchdog and
`--crane-action-policy bounded --crane-max-action-lag-ticks N` for stale-action rejection.
`Tools/Performance/ros_observation_command_bridge.py --mode nav2` stamps standard Nav2 `/cmd_vel`
with the newest observation delivered to the bridge. This records delivery provenance; the
standard Nav2 message still cannot prove that a planner internally consumed that exact sample.

Enable authoritative navigation state with `--crane-ros-nav-state`. The adapter locates the same
Rigidbody or ArticulationBody that owns the production Omni-X controller and publishes
`nav_msgs/Odometry` on `/crane/odom` plus `odom -> base_link` on `/tf` at 50 Hz. It also publishes
`base_link` edges for the discovered `lidar_link`, `front_camera_link`, `imu_link`, and `gps_link`
so task sensors have a coherent TF tree. Pose conversion is
Unity `(x,y,z)` to ROS `(z,-x,y)`; both linear and angular velocities are first transformed into
the body frame and then converted to ROS FLU. Odometry and TF share the same episode-relative
timestamp. Override topics, frames, or rate with `--crane-ros-odom-topic`,
`--crane-ros-tf-topic`, `--crane-ros-odom-frame`, `--crane-ros-base-frame`, and
`--crane-ros-nav-state-hz`; override the comma-separated child set with
`--crane-ros-nav-child-frames` or pass `none`.

The included acceptance fixture runs the real Jazzy Nav2 lifecycle manager, BT navigator, NavFn
planner, behavior server, LiDAR-fed voxel/inflation global and local costmaps, Regulated Pure
Pursuit controller, and `NavigateToPose` action. It uses authoritative Unity `odom` as its global
frame and therefore does not validate localization/SLAM, static-map navigation, or lockstep. Its
bridge stamps each returned command with the newest odometry delivered to the fixture; this
bounds delivery age but does not reveal which sample Nav2 internally consumed. Set
`CRANE_NAV2_ACTION_MODE=follow-path` to run the narrower controller-only action.

`Clock.time` and published `/clock` are episode-relative, not process-uptime-relative. Scene load
or an explicit in-place reset starts the authoritative episode clock at zero. Runtime scene
selection disables the transient initial scene's sensors and transports before their `Start`
methods run, so only the requested scene owns the ROS graph.
On scene reload, the scene-owned `ROSConnection` cancels and joins its connection task before it is
destroyed. The paired endpoint must remove that socket's ROS nodes on disconnect; the validated
companion is `astro_dock` commit `3620237` (endpoint commit `3c3d405`). This prevents stale
callbacks and duplicate rosout node names when the replacement scene registers the same topics.

## Runtime profiles

Profiles are presets over subsystem gates and presentation ownership. They do not select a
different physics implementation unless stated explicitly.

| Profile | Rendering and sensors | Physics/control intent | Current validation |
|---|---|---|---|
| `train-gpu` | RGB and spectators off; depth, detections, LiDAR and navigation sensors retained | Accelerated graphics-backed training, including aquatic | Roboboat valid at about 2× on reference hardware |
| `train-cpu` | All Cameras off; RGB/GPU passes off; geometric `32FC1` depth, camera info, detections and non-camera sensors remain | Strict `-nographics` non-aquatic training | Dense depth plus land/multirotor fixtures pass at 2×; 1/2/4/8 worker sweep valid; aquatic rejected |
| `interactive-low` | Task sensors unchanged; spectator/window defaults to 960×540 | Human/Nav2 development on weaker hardware | Aquatic validity retained; modest GPU saving |
| `interactive-high` | Full presentation and configured sensors | Human parameter tuning and Nav2 development | Profile application validated; hardware cost is scene-dependent |
| `evaluation-high` | Full presentation and configured sensors | Live high-fidelity evaluation | Implemented; task-specific evaluation remains user-owned |
| `replay-high` | Full presentation and arbitrary spectator view | Authoritative state playback; live vehicle physics/controllers disabled | Aquatic v5 replay validated |
| `custom` | Only explicit `--crane-disable-*` flags | Experimental combinations | Must be validated by the operator |

Every applied profile logs `CRANE_RUNTIME_PROFILE_RESOLVED`. Use
`--crane-runtime-report PATH` for the same data as JSON. It includes the resolved graphics device,
sensor gates, camera ownership, water-surface count, and aquatic graphics-free status.

## Build and launch examples

Build the Linux player:

```bash
unity build /path/to/crane_sim \
  --editor-version 6000.5.10f1 \
  --target StandaloneLinux64 \
  --execute-method CranePerformanceBuild.BuildLinuxWorker \
  --allow-dirty-build --format json --no-tail
```

List profiles without entering a scene:

```bash
./Builds/CRANE-Worker/CRANE.x86_64 -batchmode -nographics \
  --crane-list-profiles
```

Run an interactive high-fidelity scene. `--crane-scene` works for ordinary profile launches:

```bash
SDL_VIDEODRIVER=x11 ./Builds/CRANE-Worker/CRANE.x86_64 \
  -screen-fullscreen 0 -screen-width 1280 -screen-height 720 \
  --crane-profile interactive-high \
  --crane-scene "Roboboat Course" \
  --crane-runtime-report ./runtime-profile.json
```

Run the weak-hardware interactive profile without changing robot sensor RenderTextures:

```bash
SDL_VIDEODRIVER=x11 ./Builds/CRANE-Worker/CRANE.x86_64 \
  -screen-fullscreen 0 \
  --crane-profile interactive-low \
  --crane-interactive-width 960 --crane-interactive-height 540 \
  --crane-scene "Roboboat Course"
```

Run a graphics-backed aquatic Train-GPU worker. Omit `--crane-disable-ros` when connecting to the
ROS-TCP endpoint:

```bash
SDL_VIDEODRIVER=x11 ./Builds/CRANE-Worker/CRANE.x86_64 \
  -screen-fullscreen 0 -screen-width 640 -screen-height 360 \
  --crane-worker --crane-profile train-gpu \
  --crane-scene "Roboboat Course" \
  --crane-seed 1000 --crane-disable-ros \
  --crane-record ./episodes/episode.crane \
  --crane-runtime-report ./episodes/runtime-profile.json
```

Run an interactive ROS/Nav2-development session with fixed-step stamped command application:

```bash
SDL_VIDEODRIVER=x11 ./Builds/CRANE-Worker/CRANE.x86_64 \
  -screen-fullscreen 0 -screen-width 1280 -screen-height 720 \
  --crane-profile interactive-high --crane-scene "Roboboat Course" \
  --crane-ros-ip 127.0.0.1 --crane-ros-port 10000 \
  --crane-ros-nav-state \
  --crane-ros-cmd-vel /crane/cmd_vel_stamped \
  --crane-action-policy bounded --crane-max-action-lag-ticks 10 \
  --crane-command-timeout-ticks 25
```

Benchmark results include an `actionTiming` block with known-source coverage, mean/maximum
observation-to-application and receive-to-application tick age, maximum application interval, and
watchdog stops. The Nav2 fixture additionally fails acceptance if any applied command lacks stamped
provenance or either measured lag maximum exceeds `--crane-max-action-lag-ticks`. In the accepted
single-worker 2× reference run, all 40 actions had provenance, mean/maximum source age was
4.175/7 ticks, queue age was 1/1 tick, and the sole watchdog stop occurred after goal completion.

On the ROS 2 side, pair Nav2's ordinary `Twist` output with the latest delivered detection stamp:

```bash
source /opt/ros/jazzy/setup.bash
python3 Tools/Performance/ros_observation_command_bridge.py \
  --mode nav2 --observation-topic /detections \
  --input-topic /cmd_vel --output-topic /crane/cmd_vel_stamped
```

Run the aquatic ArduPilot JSON/SITL UDP protocol fixture independently of ROS-TCP:

```bash
CRANE_RESULT_ROOT="$PWD/PerformanceResults/sitl-udp" \
CRANE_MAVROS_PORT=10302 Tools/Performance/run_sitl_udp_fixture.sh
```

The legacy filename/class `MAVROSConnection` implements ArduPilot's JSON backend rather than a
MAVROS node. The fixture waits for the measured episode boundary, injects one malformed datagram,
sends valid 40-byte servo packets, receives JSON IMU/pose/velocity telemetry, and verifies that its
timestamp follows accelerated simulated time monotonically. The accepted 2× reference run applied
198 of 200 packets (two intentional latest-value replacements), returned 653 peer-visible telemetry
packets at a 1.999× clock rate, and sustained 2.003× worker RTF. It is not a real autopilot or an
aerial-flight-controller acceptance test.

Launch the project-specific Nav2 graph separately with `use_sim_time:=true`. A single worker must
own one isolated ROS graph/domain and one `/clock`. If ROS nodes are split across Docker
containers while using Fast DDS, pass `--ipc host` to every participating container (or configure
an explicit non-shared-memory transport); graph discovery alone does not prove payload delivery.
If the full stack cannot keep up, lower the requested RTF or use bounded rejection; CRANE does not
currently provide a Nav2 lockstep barrier.

Run the reproducible Nav2 navigation vertical slice after building the player and the
`lunarzdev/astro:cuda` image/workspace are available:

```bash
Tools/Performance/run_nav2_controller_fixture.sh
```

The script owns the ROS-TCP endpoint, shared IPC configuration, lifecycle manager, planner,
global/local costmaps, BT navigator, behaviors, controller, action client/command bridge, Unity
worker, logs, and result directory. Its default scope is `nav2-navigate-to-pose`; report it as an
authoritative-odom navigation qualification, not localization/SLAM or training lockstep.

Exercise the accepted ROS-connected scene-reload boundary and require its machine-readable
transport/reset checks:

```bash
CRANE_RESULT_ROOT=PerformanceResults/nav2-scene-reload \
CRANE_DURATION=22 CRANE_FIXTURE_DELAY=12 \
CRANE_NAV2_UNITY_EXTRA_ARGS='--crane-reset-probe' \
Tools/Performance/run_nav2_controller_fixture.sh
```

The script writes `navigation-reset-summary.json` and fails unless the goal succeeds, benchmark
and water/sensor checks are valid, action rejection counters remain zero, endpoint errors and
duplicate registrations remain zero, no TCP connections overlap, the clock rewind is recovered,
and the scene-reload probe executes. The expected TF-buffer rewind warning is evidence of the
episode-time discontinuity; successful navigation afterward is the recovery criterion.

Run strict graphics-free land or aerial physics:

```bash
./Builds/CRANE-Worker/CRANE.x86_64 -batchmode -nographics \
  --crane-worker --crane-profile train-cpu \
  --crane-scene "Land Vehicle Validation" \
  --crane-seed 1000 --crane-disable-ros \
  --crane-record ./episodes/land.crane
```

`train-cpu` exits with status 3 if the selected scene contains HDRP water. Its dense depth backend
uses PhysX geometry, not rendering. Override the profile default with
`--crane-depth-backend gpu|geometric|off`; using GPU depth under `-nographics` is invalid. These are
explicit capability boundaries, not launch errors to bypass.

Replay an authoritative episode at high visual fidelity:

```bash
SDL_VIDEODRIVER=x11 ./Builds/CRANE-Worker/CRANE.x86_64 \
  -screen-fullscreen 0 -screen-width 1920 -screen-height 1080 \
  --crane-profile replay-high \
  --crane-replay ./episodes/episode.crane \
  --crane-runtime-report ./episodes/replay-profile.json
```

Replay time comes from recorded ticks. Display FPS, pause, step, seek, and playback rate do not
redefine the recorded simulation timeline. The player applies exact recorded body/joint states and
water simulation time; it does not claim deterministic PhysX re-simulation.

For a measured benchmark command and worker sweeps, use
[PerformanceEngineering.md](PerformanceEngineering.md). For ROS-connected runs, give every worker
its own ROS-TCP port, ROS domain/namespace, seed, logs, recorder path, and MAVROS/SITL port where
applicable.

Run isolated closed-loop Nav2 workers with one endpoint and Nav2 graph per Unity process:

```bash
CRANE_ROS_PORT_BASE=10100 CRANE_ROS_DOMAIN_BASE=80 \
CRANE_DURATION=24 CRANE_TIME_SCALE=0.75 \
Tools/Performance/sweep_nav2_workers.sh 6
```

The launcher writes one `navigation-reset-summary.json` per worker plus an aggregate
`sweep-summary.json`. It rejects duplicate ports/domains, invalid Unity sensor/physics results,
stale/rejected/cross-episode actions, endpoint errors, and missing endpoint/controller CPU/RAM
samples. On the reference machine six workers at 0.75× are the current accepted density point;
eight at 1× and eight at 0.75× produced stale depth observations, while four at 1× had one
stale-action outlier across two matched runs. For higher per-worker acceleration, one 2× worker
and two concurrent 2× workers pass; the latter delivers 4.003× aggregate measured RTF. Three
workers at 1.5× or 2× fail the observation-validity gate because at least one depth acquisition is
stale, even though every Nav2 goal succeeds. Choose worker count and time scale from the complete
validity record, not goal status or requested time scale alone.

Validate the in-place reset vertical slice without graphics:

```bash
./Builds/CRANE-Worker/CRANE.x86_64 -batchmode -nographics \
  --crane-profile train-cpu --crane-disable-ros \
  --crane-in-place-reset-validation \
  --crane-output ./PerformanceResults/in-place-reset/result.json
```

The fixture runs in `Aerial Vehicle Validation`, captures A, perturbs body/actuator/custom state,
and restores A while advancing the episode generation and zeroing `/clock`. It is an executable
test of the reset seam, not permission to replace scene reload in production aquatic campaigns.

## Authoring and validation rules

1. Treat scenes, prefabs, masses, inertias, colliders, sensor rates, and water settings as part of
   the experiment configuration.
2. Add forces/torques in `FixedUpdate`; do not move a live vehicle by assigning transforms.
3. Keep RGB, depth, camera-info, detections, and spectator rendering independently configurable.
4. Carry episode and acquisition identity through asynchronous work and reject prior-episode
   callbacks.
5. Use scene reload as the correctness reset baseline. The in-place coordinator is experimental
   until every task component and external ROS/controller state implements the reset contract.
6. Compare optimizations at equal simulated timestamps with the same seed, timestep, scene,
   sensors, water, and ROS configuration.
7. Do not describe aquatic `-nographics` as headless support and do not disable water to make it
   appear successful.
8. Keep oracle scene identity separate from policy observations. A detection sensor must apply
   visibility/range/occlusion rather than publishing all registered objects.
9. Record authoritative state for replay; do not promise arbitrary PhysX snapshot resume.
10. Label capabilities as implemented, validated, experimental, blocked, or absent.

The embedded ROS-TCP Connector is intentionally pinned under `Packages/`. Its CRANE patch removes
a pre-handshake publisher-registration duplicate and synchronizes topic creation with the
connection thread, gives concurrently queued system commands independent serializers, and performs
bounded connection-task teardown from `OnDestroy`. The paired
endpoint removes executor-owned ROS nodes and clears its registration tables on disconnect. Do not
replace either side with an unpinned upstream version without rerunning the live transport,
scene-reload, and reconnect fixtures.
