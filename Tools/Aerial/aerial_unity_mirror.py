"""Line-by-line python mirrors of current CRANE C# aerial code, for counterexamples.

Each function cites the C# it mirrors.  Unity numerics are reproduced on raw RUF triples
(see aerial_frames docstring).  Nothing here is a recommended model.
"""

import math

import numpy as np

from aerial_frames import M_RUF_TO_FLU, M_RUF_TO_FRD, unity_euler_matrix

# MultirotorDynamics.cs serialized defaults (all four aerial scenes)
T_MAX = 8.0
K_REACTION = 0.015
TAU = 0.06
AXIS_MIX = 0.2
LINEAR_DRAG = np.array([0.18, 0.25, 0.18])
QUADRATIC_DRAG = np.array([0.08, 0.12, 0.08])
YAW_SIGNS = np.array([1.0, -1.0, 1.0, -1.0])           # FL, FR, RR, RL
ROTORS_RUF = np.array([[-0.23, 0, 0.23], [0.23, 0, 0.23], [0.23, 0, -0.23], [-0.23, 0, -0.23]])
FIXED_DT = 2822399 / 141120000                        # ProjectSettings/TimeManager.asset


def set_command(collective, roll, pitch, yaw):
    """MultirotorDynamics.SetCommand (lines 75-86)."""
    c = min(max(collective, 0.0), 1.0)
    r = float(np.clip(roll, -1, 1)) * AXIS_MIX
    p = float(np.clip(pitch, -1, 1)) * AXIS_MIX
    y = float(np.clip(yaw, -1, 1)) * AXIS_MIX
    raw = np.array([c - r - p + y, c + r - p - y, c + r + p + y, c - r + p - y])
    return np.clip(raw, 0.0, 1.0)


def rotor_wrench_ruf(speeds):
    """ApplyRotorForces in body RUF: (force, torque) about the CoM at identity attitude."""
    thrust = T_MAX * np.asarray(speeds) ** 2
    up = np.array([0.0, 1.0, 0.0])
    force = thrust.sum() * up
    torque = sum(np.cross(r, up * t) for r, t in zip(ROTORS_RUF, thrust))
    torque = torque + up * float(np.sum(YAW_SIGNS * thrust * K_REACTION))
    return force, torque


def drag_local(v_local):
    """ApplyAerodynamics/DragAxis: per-axis linear + v_i|v_i| quadratic (lines 128-141)."""
    v = np.asarray(v_local, dtype=float)
    return -LINEAR_DRAG * v - QUADRATIC_DRAG * v * np.abs(v)


def gust(fixed_time, amplitude, frequency_hz=0.35):
    """Gust uses absolute Time.fixedTime, which ResetEpisode does not rewind (line 125)."""
    return np.asarray(amplitude) * math.sin(fixed_time * frequency_hz * 2 * math.pi)


def translate_in_wind(wind_ruf, seconds, mass=1.5, dt=FIXED_DT):
    """Pure-translation response of the hovering vehicle to steady wind (no torque path)."""
    v = np.zeros(3)
    p = np.zeros(3)
    for _ in range(math.ceil(seconds / dt)):
        f = drag_local(v - np.asarray(wind_ruf))
        v = v + f / mass * dt                 # PhysX: semi-implicit Euler
        p = p + v * dt
    return p


# --- MAVROSConnection.cs (ArduPilot JSON SITL) -----------------------------------

def mavros_telemetry(position_ruf, velocity_ruf, angular_velocity_world_ruf, euler_deg):
    """Update() lines 169-199: what CRANE sends to ArduPilot."""
    w = angular_velocity_world_ruf
    return {
        "gyro": np.array([w[2], w[0], w[1]]),
        "accel_body": np.array([0.0, 0.0, -9.81]),
        "position": np.array([position_ruf[2], position_ruf[0], position_ruf[1]]),
        "attitude": np.radians([euler_deg[2], euler_deg[0], euler_deg[1]]),
        "velocity": np.array([velocity_ruf[2], velocity_ruf[0], velocity_ruf[1]]),
    }


def ardupilot_expected(position_ruf, velocity_ruf, angular_velocity_world_ruf, euler_deg):
    """ArduPilot libraries/SITL/examples/JSON/readme.md: gyro body FRD, pos/vel NED."""
    r_unity = unity_euler_matrix(euler_deg)
    m = M_RUF_TO_FRD
    w_body_ruf = r_unity.T @ np.asarray(angular_velocity_world_ruf, dtype=float)
    r_ned_frd = m @ r_unity @ m.T
    from aerial_frames import zyx_euler
    return {
        "gyro": -(m @ w_body_ruf),
        "position": m @ np.asarray(position_ruf, dtype=float),
        "velocity": m @ np.asarray(velocity_ruf, dtype=float),
        "attitude": zyx_euler(r_ned_frd),
    }


# ArduCopter AP_MotorsMatrix.cpp MOTOR_FRAME_TYPE_X (QUAD): servo index -> position
ARDUCOPTER_QUAD_X = ["front_right", "rear_left", "front_left", "rear_right"]
CRANE_SERVO_TO_ROTOR = ["front_left", "front_right", "rear_right", "rear_left"]   # ApplyPwm


# --- Imu.cs ----------------------------------------------------------------------

def imu_angular_velocity(angular_velocity_world_ruf):
    """Imu.cs line 35: body.angularVelocity.To<FLU>() (polar conversion, world frame)."""
    return M_RUF_TO_FLU @ np.asarray(angular_velocity_world_ruf, dtype=float)


def float32_timestamp_resolution(seconds):
    """SITLCommsJsonOutputPacket.timestamp is float (single precision)."""
    s = np.float32(seconds)
    return float(np.spacing(s))
