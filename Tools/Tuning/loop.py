"""Closed loop: plant + the boat's own velocity controller and mixer + a Nav2-like command layer.

The velocity law (mhseals_hardware.velocity_law) and the force-space mixer
(mhseals_hardware.thruster_mixer) are imported from the workspace source unchanged, so a
plant variant meets exactly the code the boat runs. Nav2 itself is not here: the command
layer is the velocity smoother (accel and velocity limits at 20 Hz) fed either a scripted
command or an omni pursuit to a goal (a stand-in for MPPI's GoalCritic / PathFollow with the
rotation shim's rotate_to_goal_heading). Everything the plant changes - lag, stop distance,
headroom, coupling - is felt through the same limits Nav2 would apply.
"""
from __future__ import annotations

import math
import os
import sys
from dataclasses import dataclass, field

import numpy as np

sys.path.insert(0, __import__('os').path.dirname(__import__('os').path.abspath(__file__)))  # python3 -I drops it
import yaml

HW = os.environ.get('MHSEALS_HARDWARE_SRC', '/home/lunarz/astro_dock/src/mhseals_hardware')
sys.path.insert(0, HW)
from mhseals_hardware.thruster_mixer import THRUSTER_MIXER, thruster_pulses  # noqa: E402
from mhseals_hardware.velocity_law import Plant, VelocityLaw  # noqa: E402

from plant import Boat, Variant  # noqa: E402

HERE = os.path.dirname(os.path.abspath(__file__))
SIM_HW = os.environ.get('MHSEALS_SIM_HW', '/home/lunarz/.cache/wt/sim-mhseals_testing/config/sim/hardware_crane.yaml')


def hardware(overrides=None):
    """The controller's view: the sim's characterized curves (hardware_crane.yaml) + overrides."""
    config = yaml.safe_load(open(SIM_HW))
    config.setdefault('thruster_matrix', [x for row in THRUSTER_MIXER for x in row])
    config.setdefault('decoupling', [1, 0, 0, 0, 1, 0, 0, 0, 1])
    config.setdefault('decoupling_negative', None)
    config.setdefault('controller', {'kp': [0.8, 0.8, 0.6], 'ki': [0.6, 0.6, 0.4],
                                     'integral_limit': [0.3, 0.3, 0.3], 'drift_time_constant': 5.0,
                                     'odom_timeout': 0.3, 'drift_command_limit': 0.1})
    for key, value in (overrides or {}).items():
        if key == 'controller':
            config['controller'] = {**config['controller'], **value}
        else:
            config[key] = value
    return config


@dataclass
class Nav:
    """The Nav2 limits under test (controller + smoother keys)."""
    vx_max: float = 0.4
    vx_min: float = -0.1
    vy_max: float = 0.15
    wz_max: float = 0.3
    ax_max: float = 0.3
    ax_min: float = -0.3
    ay_max: float = 0.3
    az_max: float = 0.5
    xy_tol: float = 0.4
    yaw_tol: float = 0.5
    k_pos: float = 0.6          # pursuit gain (1/s): MPPI's goal pull stand-in
    k_yaw: float = 1.0
    shim_threshold: float = 1.8
    rotate_to_goal: bool = True


class Smoother:
    """nav2_velocity_smoother, OPEN_LOOP, scale_velocities: per-axis accel limits at 20 Hz."""

    def __init__(self, nav: Nav):
        self.nav, self.out = nav, np.zeros(3)

    def __call__(self, cmd, dt):
        n = self.nav
        cmd = np.clip(cmd, [n.vx_min, -n.vy_max, -n.wz_max], [n.vx_max, n.vy_max, n.wz_max])
        up = np.array([n.ax_max, n.ay_max, n.az_max]) * dt
        down = np.array([-n.ax_min, n.ay_max, n.az_max]) * dt
        diff = cmd - self.out
        # Accelerating means moving away from zero; the decel limit applies toward zero.
        limit = np.where(np.sign(diff) == np.sign(self.out), up, down)
        limit = np.where(self.out == 0, up, limit)
        step = np.clip(diff, -limit, limit)
        # scale_velocities: keep the vector direction by scaling every axis to the slowest.
        with np.errstate(divide='ignore', invalid='ignore'):
            ratio = np.where(np.abs(diff) > 1e-9, np.abs(step) / np.abs(diff), 1.0)
        self.out = self.out + diff * ratio.min()
        return self.out.copy()


class Sensors:
    """EKF-like odometry: latency, white velocity noise, heading bias, AR(1) GPS position error."""

    def __init__(self, v: Variant, rng):
        self.v, self.rng = v, rng
        self.queue, self.pos_err = [], np.zeros(2)

    def sample(self, boat, t, dt):
        v = self.v
        a = math.exp(-dt / v.pos_tau) if v.pos_tau > 0 else 0.0
        self.pos_err = a * self.pos_err + v.pos_sd * math.sqrt(1 - a * a) * self.rng.standard_normal(2)
        b = v.heading_bias
        vel = boat.base_velocity()
        c, s = math.cos(-b), math.sin(-b)  # body velocity seen through a biased heading
        meas = np.array([c * vel[0] - s * vel[1], s * vel[0] + c * vel[1], vel[2]])
        meas += np.array([v.vel_sd, v.vel_sd, v.yaw_rate_sd]) * self.rng.standard_normal(3)
        pose = boat.base_pose()
        self.queue.append((t + v.odom_latency, meas, pose[2] + b, pose[:2] + self.pos_err))
        while len(self.queue) > 1 and self.queue[1][0] <= t + 1e-9:
            self.queue.pop(0)
        return self.queue[0] if self.queue[0][0] <= t + 1e-9 else None


def wrap(a):
    return (a + math.pi) % (2 * math.pi) - math.pi


@dataclass
class Trace:
    t: list = field(default_factory=list)
    pose: list = field(default_factory=list)      # truth base_link x, y, yaw
    vel: list = field(default_factory=list)       # truth base_link body u, v, r
    cmd: list = field(default_factory=list)       # smoothed command (what /cmd_vel carries)
    raw: list = field(default_factory=list)       # nav layer output (/cmd_vel_nav)
    effort: list = field(default_factory=list)    # axis effort from the law
    thruster: list = field(default_factory=list)  # per-thruster effort after the mixer
    goal: list = field(default_factory=list)      # active goal index

    def arrays(self):
        return {k: np.array(v) for k, v in self.__dict__.items()}


def run(variant: Variant, nav: Nav, script=None, goals=None, duration=30.0, hw=None, seed=0,
        start=(0.0, 0.0, 0.0), feedback=True):
    """Simulate. ``script(t) -> body velocity`` drives the smoother directly; ``goals`` (list of
    (x, y, yaw)) runs the pursuit layer instead. Returns (Trace arrays, info)."""
    config = hardware(hw)
    ctrl = config['controller']
    law = VelocityLaw(Plant(config), ctrl['kp'], ctrl['ki'], ctrl['integral_limit'],
                      ctrl['drift_time_constant'], ctrl['odom_timeout'], ctrl['drift_command_limit'])
    ratio = float(config.get('reverse_thrust_ratio', 1.0))
    mixer = tuple(tuple(config['thruster_matrix'][i:i + 3]) for i in range(0, 12, 3))
    dec = tuple(tuple(config['decoupling'][i:i + 3]) for i in range(0, 9, 3))
    boat = Boat(variant, seed=seed)
    boat.eta[:] = start
    rng = np.random.default_rng(seed + 1)
    sensors, smoother = Sensors(variant, rng), Smoother(nav)
    trace = Trace()
    thr = np.zeros(4)
    raw = np.zeros(3)
    goal_i, reached_at, shim = 0, [], None
    odom = None
    t, dt_plant = 0.0, boat.dt
    ticks = int(round(duration / dt_plant))
    for k in range(ticks):
        t = k * dt_plant
        if k % int(round(1 / 30 / dt_plant)) == 0:            # odometry at ~30 Hz
            s = sensors.sample(boat, t, 1 / 30)
            if s is not None:
                odom = s
                law.observe(s[1], s[2], t)
        if goals is not None and k % int(round(0.1 / dt_plant)) == 0 and odom is not None:  # 10 Hz
            if goal_i < len(goals):
                raw, done, shim = pursue(odom, goals[goal_i], nav, shim)
                if done:
                    reached_at.append(t)
                    goal_i, shim = goal_i + 1, None
            else:
                raw = np.zeros(3)
        if k % int(round(0.05 / dt_plant)) == 0:               # smoother + controller at 20 Hz
            if script is not None:
                raw = np.array(script(t), float)
            cmd = smoother(raw, 0.05)
            effort = law.step(cmd, t, feedback=feedback)
            pulses = thruster_pulses(effort, mixer, (1, 2, 3, 4), (False,) * 4, dec,
                                     None, ratio)
            thr = (np.array(pulses, float) - 1500.0) / 400.0
            trace.t.append(t)
            trace.pose.append(boat.base_pose())
            trace.vel.append(boat.base_velocity())
            trace.cmd.append(cmd)
            trace.raw.append(raw.copy())
            trace.effort.append(np.array(effort))
            trace.thruster.append(thr.copy())
            trace.goal.append(goal_i)
        boat.step(thr)
    return trace.arrays(), {'reached_at': reached_at, 'goals_done': goal_i,
                            'drift_estimate': law.drift.tolist()}


def shim_rate(err, nav: Nav):
    """RotationShim's computeRotateToHeadingCommand: full rate, braking at max_angular_accel so
    it would stop at the target (sqrt(2 a |err|)), in open loop on the remaining angle."""
    return math.copysign(min(nav.wz_max, math.sqrt(2 * nav.az_max * abs(err))), err)


def pursue(odom, goal, nav: Nav, shim):
    """Omni goal pursuit with a sqrt stopping profile and a rotate-to-heading shim.

    Returns (body command, goal reached, shim state). The speed toward the goal is
    min(v_max, k_pos d, sqrt(2 |ax_min| d)): the fastest profile the smoother can still stop.
    """
    _, _, yaw, pos = odom
    d_vec = np.array(goal[:2]) - pos
    d = float(np.hypot(*d_vec))
    bearing = math.atan2(d_vec[1], d_vec[0])
    yaw_err_goal = wrap(goal[2] - yaw)
    if d < nav.xy_tol and abs(yaw_err_goal) < nav.yaw_tol:
        return np.zeros(3), True, shim
    if d < nav.xy_tol:
        if not nav.rotate_to_goal:
            return np.zeros(3), False, shim
        return np.array([0, 0, shim_rate(yaw_err_goal, nav)]), False, shim
    heading_err = wrap(bearing - yaw)
    if shim is None:
        shim = abs(heading_err) > nav.shim_threshold
    if shim:
        if abs(heading_err) < 0.2:
            shim = False
        else:
            return np.array([0, 0, shim_rate(heading_err, nav)]), False, shim
    speed = min(nav.vx_max, nav.k_pos * d, math.sqrt(2 * abs(nav.ax_min) * d))
    c, s = math.cos(-yaw), math.sin(-yaw)
    world = d_vec / max(d, 1e-9) * speed
    body = np.array([c * world[0] - s * world[1], s * world[0] + c * world[1]])
    body[0] = np.clip(body[0], nav.vx_min, nav.vx_max)
    body[1] = np.clip(body[1], -nav.vy_max, nav.vy_max)
    face = bearing if d > 1.0 else goal[2]
    wz = np.clip(nav.k_yaw * wrap(face - yaw), -nav.wz_max, nav.wz_max)
    return np.array([body[0], body[1], wz]), False, shim


def characterize(variant: Variant, reverse_ratio=None):
    """What an ideal steady characterization of ``variant`` would store: per axis and sign, the
    least-squares e|e| = d1 v + d2 v|v| (d1 >= 0) over the efforts that hold 10-95% of top
    speed, and the top speeds as motion limits. Dynamics (mass, inertia, lag) do not enter."""
    from plant import static_thrust, thrust_columns
    B = thrust_columns(variant)
    rows = np.array(THRUSTER_MIXER)
    drags = (variant.drag_u, variant.drag_v, variant.drag_r)
    curves, limits = {}, {}
    for axis, name in enumerate(('surge', 'sway', 'yaw')):
        for sign, key in ((1, '+'), (-1, '-')):
            efforts = np.linspace(0.05, 1.0, 40)
            force = np.array([(B @ static_thrust(variant, rows[:, axis] * sign * e))[axis] * sign for e in efforts])
            c1, c2 = drags[axis]
            speed = (-c1 + np.sqrt(c1 * c1 + 4 * c2 * np.maximum(force, 0))) / (2 * c2)
            keep = (speed > 0.1 * speed[-1]) & (speed < 0.95 * speed[-1])
            A = np.stack([speed[keep], speed[keep] ** 2], 1)
            d, *_ = np.linalg.lstsq(A, efforts[keep] ** 2, rcond=None)
            if d[0] < 0:
                d = np.array([0.0, float(np.sum(A[:, 1] * efforts[keep] ** 2) / np.sum(A[:, 1] ** 2))])
            curves[name + key] = {'d1': float(d[0]), 'd2': float(d[1])}
            limits[name] = min(limits.get(name, 9.0), float(speed[-1]))
    out = {'characterization': {'speed_curves': curves, 'motion_limits': {'velocity': limits}}}
    if reverse_ratio is not None:
        out['reverse_thrust_ratio'] = reverse_ratio
    return out
