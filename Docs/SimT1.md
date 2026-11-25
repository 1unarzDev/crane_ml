# crane sim up --tier T1: the ROS-facing T1 world

```
crane sim up --tier T1                    # odom sensors (the real EKF runs), 1x real time, the T2 seed and course
crane sim up --tier T1 --odom truth       # /odom/local and TF straight from the twin, localization off
crane sim up --tier T1 --rate 5 --seed 3 --current 0 --card characterized
crane sim up --tier T1 --world pool       # a 10 x 4.5 m pool whose walls Nav2's costmaps see; pool:9x5 sets the size
crane sim check --wait 120 ; crane sim status ; crane sim restart nav|ros ; crane sim down
```

The T1 world is one rclpy node (`crane/sim/t1node.py`, mounted read-only with the plant's folder) in the same ROS
container as T2, beside
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
| `/points` | sensor_msgs/PointCloud2 (x, y, z float32) | 10 Hz | lidar_link | pool | (the VLP16 in T2) |
| `/perception/costmap` | nav_msgs/OccupancyGrid, latched | 10 Hz | base_link | pool | (the objects part's perception costmap) |
| `/tf_static` base_link→lidar_link | tf2_msgs/TFMessage | once, latched | base_link (lidar_link) | pool, truth | (robot_state_publisher) |
| `/crane/reset_to_start` (service) | std_srvs/Trigger | - | - | both | RoboBoatStartReset |
| `/thrusters/pwm` (in) | std_msgs/UInt16MultiArray | 20 Hz from thruster_pwm_node | - | both | ROSThrusterPwmCommand |

Not published, as in T2: `/crane/truth/*` (the player sends them only with `--crane-ros-course-truth`, which T2
does not pass; the truth is the state dir's `course_truth.json`), the camera, lidar and detection topics, and
`/crane/sim/horn`.

Message contents, as the player fills them:
- `/odom/mavros`: the truth pose (the EKFs use only vx, vy), body twist with `fcu_velocity_sigma` noise on vx, vy;
  twist covariance diagonal max(1e-4, sigma²).
- `/imu/raw`: labelled `base_link`; orientation (yaw), body rates, and the specific force of the `imu_link` point
  (+9.81 m/s² up at rest, plus the centripetal term in turns); covariance diagonals 1e-4.
- `/gps/fix`: the `gps_link` antenna position (with `gps_sigma` noise per axis) to latitude/longitude about the
  origin 27.3747, -82.4518 (a flat-earth tangent plane, R 6378137 m); status FIX, service GPS; covariance
  diag(h, h, 4h), h = max(0.01, sigma²), type DIAGONAL_KNOWN.
- Lever arms come from the robot's URDF (`robot.robots.<robot>.urdf`, the fixed-joint chain from base_link), so
  they equal what robot_state_publisher publishes: gps_link (0.0699, -0.1207, 0.0954), imu_link
  (-0.0572, -0.1016, 0.0954) for blastoise. A test compares them with mhseals_nav's blastoise.xacro.

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
- No objects part means no `/perception/costmap` on a course: Nav2's costmaps see open water (no buoys). In the
  pool the node publishes it itself (below).

## The pool

`--world pool[:LxW]` (or `--env pool:LxW`; default `tiers.T1.sim.pool.default`, 10x4.5) is a rectangle with walls at
x 0..L, y -W/2..W/2; the boat starts at (1.5, 0) facing +x, 1.5 m from the end wall, centred. There is no course
truth, `--set` refuses and the water is still unless `--current` is given. T1 only: `--tier T2 --world pool` refuses.

The walls are a 2-D lidar ring at the lidar's mount height: 720 beams from the `lidar_link` lever arm (from the URDF,
as gps and imu), cut at 30 m, published on `/points` (PointCloud2, `lidar_link`, 10 Hz). Nav2's local and global
costmaps read no cloud: both are a StaticLayer on `/perception/costmap`, which the objects part's costmap node makes
from `/points` (STVL). That part is off in T1, so the node publishes `/perception/costmap` itself: the same returns as
lethal cells (100) in a 24 m square at 0.15 m about `base_link`. The global costmap has no static map layer; it is a
rolling 100 m window on the same `/perception/costmap`, so the walls reach it too and no `/map` is needed.
`crane sim status` shows `world: pool 10x4.5 m, T1 node running ...`.

## Nav arguments and parallel worlds

```
crane sim up --tier T1 --world pool --nav preset:=pool
crane sim restart nav preset:=pool        # new arguments replace the saved ones; bare `restart nav` reuses them
crane sim up --tier T1 --instance 2       # a second world beside the first
```

`NAME:=VALUE` goes under `parts.navigation.args` of the overrides the running world was launched with (kept in the
state dir as `ros.overrides`, so the T1 world keeps its disabled parts); `--nav-args "A:=x B:=y"` is several at once.
A host path value must lie under the workspace, `~/.config/astro` or `~/.config/mhseals` (mapped into the container),
else the command refuses.

`--instance N` (or `CRANE_SIM_INSTANCE`, else the cluster worker's `CRANE_ISOLATION_ID`) gives a world its own
`ROS_DOMAIN_ID` (the profile's + N), endpoint port (10000, or `CRANE_PORT_BASE`, + that), container `sim_up_ros_N` and
state dir `~/.cache/sim_up/N`. `down` stops only that instance; `status` without one lists the running ones. A T2
instance refuses while another T2 world runs.

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
