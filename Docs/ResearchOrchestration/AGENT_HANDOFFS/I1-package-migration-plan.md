# I1 — crane_* package migration plan (Plan agent, Opus)

The full plan was returned to the orchestrator. Its key points are condensed here.

## Source facts
- **Port only committed blobs**, via `git show <sha>:<path>`. mhseals_testing@f3f5d95 has uncommitted edits (rtk, field_test, ui, package.xml/setup.py) and mhseals_hardware@26c59f7 has uncommitted edits (manual_state.py, test_keyboard_chords.py). Do not copy from those working trees.
- **Missing Unity flags:** the characterization runner and crane_full_stack.yaml use Unity flags that do not exist on crane_ml main:
  - `--crane-ros-thruster-pwm` exists only on branch roboboat-thruster-pwm.
  - `--crane-roboboat-contract`, `--crane-water-current` and `--crane-calibration-faults` are not on main.
- **Domain collision:** the astro_dock devcontainer defaults to ROS_DOMAIN_ID 42, which is the boat's domain (the Jetson uses 142).

## Classification

### mhseals_nav
- **PORT:**
  - object_tracks/tracker, navigation_config, twist_converter, lidar_flattener
  - tasks/*, BTs, mission configs
  - nav2/ekf/navsat/twist_mux profiles, rviz
  - unit/contract tests
- **EXCLUDE:**
  - MAVROS thruster_allocation
  - ZED/TensorRT perception and models
  - DDS relays
  - blastoise CAD
- **REIMPLEMENT-GENERIC:**
  - omni_boat URDF, as primitives that keep the frames and thruster mounts
  - bringup, as `sim_nav.launch.py`

### mhseals_hardware
- **PORT** (committed @26c59f7 only):
  - thruster_mixer, velocity_law, configuration
  - TopicPWMOutputs, velocity_controller
  - thruster_pwm_node, restricted to the topic backend
  - identification, TUI helpers
- **EXCLUDE:** odroid_pwm, mosfet/thruster_test, fcu, remote_controller, udev/systemd/dts.
- **REIMPLEMENT:** sim_thrusters.yaml, which keeps the matrix/decoupling/order unchanged.

### mhseals_testing
- **PORT** (f3f5d95 committed, plus the a645657 characterization branch):
  - analysis/*
  - runner/procedures/ros/*
  - TUI, renamed `crane_test`
  - sim profiles, courses
  - tests
  - characterization runner
- **REIMPLEMENT:** arming and health, sim-only.
- **EXCLUDE:** boat.yaml, rtk*, untracked notes.
- **REFERENCE:** field_test, docs/*.tex.

## Layout
Each package contains:
- `package.xml` with `GPL-3.0-only`
- `LICENSE`
- `PROVENANCE.md`, mapping each target to `MHSeals/<repo>@<sha>:<path>`
- an SPDX header plus a `Derived from` line in every ported file

Executables are prefixed `crane_`, all topics are parameters, and the namespace is `/crane`.

Every `sim_*.launch.py` has a domain guard: it refuses unless ROS_DOMAIN_ID is in 200–228 (never 42/142), and it sets `ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST` and `use_sim_time`. A test asserts that no velodyne/mavros/zed/odroid nodes are present.

## Unity interface that must stay compatible
- **Unity publishes:**
  - /clock
  - imu/raw, gps/raw, odometry
  - points, scan, /scan
  - camera/image_raw, camera/camera_info, camera/depth/image_rect_raw
  - /detections, /crane/odom, /tf
  - /crane/docking_evaluator
- **Unity subscribes:**
  - /crane/cmd_vel_stamped
  - per-thruster Float32
  - /thrusters/pwm (UInt16MultiArray; branch only)
  - /plan, /local_costmap/costmap

## Constraints on values
- **Safe to change:** namespaces, remaps, domain, TCP port, rates, Nav2 limits via profiles.
- **Must not change silently:** thruster order FL,FR,RR,RL; thruster_matrix; decoupling; PWM range/neutral; kp/ki; 20 Hz republish; timeouts; frame IDs.

## Offline gates
- colcon build into /tmp, then pytest
- `ros2 launch ... --print`
- grep gate: `git grep -nIiE 'mhseals|seals|blastoise'`, excluding PROVENANCE/LICENSE/`Derived from` lines
- SPDX header test
