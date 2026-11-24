# crane sim up --tier T1: the ROS-facing T1 world

```
crane sim up --tier T1                    # odom sensors (the real EKF runs), 1x real time, the T2 seed and course
crane sim up --tier T1 --odom truth       # /odom/local and TF straight from the twin, localization off
crane sim up --tier T1 --rate 5 --seed 3 --current 0 --card characterized
crane sim check --wait 120 ; crane sim status ; crane sim restart nav|ros ; crane sim down
```

The T1 world is one rclpy node (`crane/sim/t1node.py`) in the same ROS container as T2, beside
`ros2 launch mhseals_nav device.launch.py device:=sim`. It steps `ProfilePlant`
(`Tools/Characterization/plant.py`, the twin of the player's plant) at the card's `fixedDeltaTime` (0.02 s) on the
PWM it receives, and publishes what the player publishes, under the same names, types and frames. No player, no
Unity endpoint, no window. Everything specific (topics, rates, frames, lever arms, noise, overrides, ready nodes)
is in `config/domains/boat.yaml` `tiers.T1.sim`; what it does not name comes from `tiers.T2`.

## Topics

Rates are in sim time. Frames are REP 103: world ENU, body FLU at `base_link`.

| Topic | Type | Rate | Frame (child) | Mode | Unity source |
|---|---|---|---|---|---|
| `/clock` | rosgraph_msgs/Clock | every twin tick (50 Hz sim) | - | both | ROSClock |
| `/crane/odom` | nav_msgs/Odometry | 50 Hz | odom (base_link) | both | CraneROSNavigationState (truth, no TF beside the contract) |
| `/odom/mavros` | nav_msgs/Odometry | 20 Hz | odom (base_link) | sensors | CraneRoboBoatSensorContract |
| `/imu/raw` | sensor_msgs/Imu | 20 Hz | base_link | both | CraneRoboBoatSensorContract |
| `/gps/fix` | sensor_msgs/NavSatFix | 5 Hz | gps_link | both | CraneRoboBoatSensorContract |
| `/odom/local` | nav_msgs/Odometry | 50 Hz | odom (base_link) | truth | (ekf_local in sensors mode) |
| `/odom/global` | nav_msgs/Odometry | 50 Hz | map (base_link) | truth | (ekf_global in sensors mode) |
| `/tf` odom→base_link | tf2_msgs/TFMessage | 50 Hz | odom (base_link) | truth | (ekf_local in sensors mode) |
| `/tf_static` map→odom | tf2_msgs/TFMessage | once, latched (identity) | map (odom) | truth | (ekf_global in sensors mode) |
| `/crane/truth/course`, `/crane/truth/states` | std_msgs/String (JSON) | 0.5 Hz, 1 Hz | - | both, only with `publish_truth: true` | CourseTruthPublisher (`--crane-ros-course-truth`, off in T2 too) |
| `/crane/reset_to_start` (service) | std_srvs/Trigger | - | - | both | RoboBoatStartReset |
| `/thrusters/pwm` (in) | std_msgs/UInt16MultiArray | 20 Hz from thruster_pwm_node | - | both | ROSThrusterPwmCommand |

Message contents, as the player fills them:
- `/odom/mavros`: the truth pose (the EKFs use only vx, vy), body twist with `fcu_velocity_sigma` noise on vx, vy;
  twist covariance diagonal max(1e-4, sigma²).
- `/imu/raw`: labelled `base_link`; orientation (yaw), body rates, and the specific force of the `imu_link` point
  (+9.81 m/s² up at rest, plus the centripetal term in turns); covariance diagonals 1e-4.
- `/gps/fix`: the `gps_link` antenna position (with `gps_sigma` noise per axis) to latitude/longitude about the
  origin 27.3747, -82.4518 (a flat-earth tangent plane, R 6378137 m); status FIX, service GPS; covariance
  diag(h, h, 4h), h = max(0.01, sigma²), type DIAGONAL_KNOWN.
- Lever arms from blastoise.xacro (base_link): gps_link (0.0699, -0.1207, 0.0954), imu_link (-0.0572, -0.1016, 0.0954).

## Commands in

`/thrusters/pwm`: four pulses in µs, channel order FL, FR, RR, RL (outputs 1,2,3,4), from
`ros2 launch mhseals_hardware drive.launch.py output_backend:=topic use_sim_time:=true`. A message without exactly
four values is dropped. A pulse holds until the next one; after 25 ticks (0.5 s sim) without one the thrusters go
neutral, as the player does (`--crane-command-timeout-ticks`). Pulses map to thrust through the card's thruster
curves (1500 neutral, ±400 µs span, clamped to 1100–1900).

The player's other inputs are not used by the stack with PWM drive (`/cmd_vel` is refused beside PWM in the player).

## The ROS side

- **sensors** (default): `device:=sim` with overrides
  `{"parts": {"camera": {"enabled": false}, "lidar": {"enabled": false}, "objects": {"enabled": false},
  "sim_bridge": {"enabled": false}}}`. localization (robot_state_publisher, ekf_local, ekf_global,
  navsat_transform, `fcu:=false`), mapping, navigation and mission run unchanged. Ready: ekf_local, bt_navigator.
- **truth**: the same plus `"localization": {"enabled": false}`. Ready: bt_navigator. robot_state_publisher is
  part of localization, so the URDF sensor links are not on TF in this mode; Nav2 needs only base_link.
- Every node runs on `/clock`: the sim profile sets `args: {sim: 'true'}`, which each part's launch passes as
  `use_sim_time`.
- No objects part means no `/perception/costmap`: Nav2's costmaps see open water (no buoys).

## Clock and rate

The twin steps at the card's `fixedDeltaTime` (0.02 s). `--rate N` paces the steps at N× real time (default 1×);
when the host cannot keep up the twin runs as fast as it can and the heartbeat reports the achieved rate. Sim time
starts at 0, as the player's does.

## The course and the world

The course truth for (family, seed, `--set`) comes from coursegen (`domains/boat/coursegen.py`, the generator the
player uses), else the committed `domains/boat/courses/<family>-s<seed>.json`. It is written to the state dir as
`course_truth.json`. The boat starts at the truth's start pose; world = the course truth's frame = `/crane/odom`'s.

`--current 0 | default | SPEED,TOWARD_DEG` sets the water current in the twin (m/s, toward, counter-clockwise from
world +x). `default` is the T2 scene's, 0.15 m/s toward 270°.

## Noise

The player's sensor-noise flags and their defaults, which T1 takes from `tiers.T1.sim.noise`:

| Player flag | T1 key | Default |
|---|---|---|
| `--crane-gps-sigma` | `gps_sigma` | 0 m |
| `--crane-fcu-velocity-sigma` | `fcu_velocity_sigma` | 0 m/s |
| `--crane-contract-seed` | `seed` | 0 |

## State dir

`~/.cache/sim_up` (SIM_UP_RUN) holds `tier`, `odom`, `rate`, `t1.json` (the node's resolved config), `t1.log`,
`t1.alive` (a heartbeat the node rewrites each second), `ros.log`, `ros.docker` and `course_truth.json`.
`crane sim restart player` refuses in T1.
