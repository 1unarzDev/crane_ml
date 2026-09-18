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
The scene/prefab transform hierarchy and the external ROS TF configuration must agree.

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
- ROS and MAVROS callbacks: enqueue commands; actuator mutation occurs at a fixed-step boundary.
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
MAVROS callbacks enqueue commands; `CraneActionGate` checks episode, sequence, source observation,
receive tick, and bounded-lag policy before fixed-step application. `--crane-disable-ros` suppresses
transport while still exercising message construction for benchmarks. This is not end-to-end
zero-copy: ROS-TCP remains a serialization boundary.

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

## Authoring and validation rules

1. Treat scenes, prefabs, masses, inertias, colliders, sensor rates, and water settings as part of
   the experiment configuration.
2. Add forces/torques in `FixedUpdate`; do not move a live vehicle by assigning transforms.
3. Keep RGB, depth, camera-info, detections, and spectator rendering independently configurable.
4. Carry episode and acquisition identity through asynchronous work and reject prior-episode
   callbacks.
5. Use scene reload as the correctness reset baseline until every component implements an
   explicit reset contract.
6. Compare optimizations at equal simulated timestamps with the same seed, timestep, scene,
   sensors, water, and ROS configuration.
7. Do not describe aquatic `-nographics` as headless support and do not disable water to make it
   appear successful.
8. Keep oracle scene identity separate from policy observations. A detection sensor must apply
   visibility/range/occlusion rather than publishing all registered objects.
9. Record authoritative state for replay; do not promise arbitrary PhysX snapshot resume.
10. Label capabilities as implemented, validated, experimental, blocked, or absent.
