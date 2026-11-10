"""Frame conventions for CRANE aerial work (pure numpy, no Unity dependency).

Unity is left-handed RUF (x right, y up, z forward).  Numerically Unity/PhysX use
the ordinary Hamilton quaternion/matrix formulas on raw triples, so a Unity
rotation matrix R_u satisfies dR_u/dt = [w_u]x R_u with w_u = Rigidbody.angularVelocity
(numerically).  Mapping to a right-handed frame is a reflection M (det M = -1):

    v'      = M v                   (polar vectors: position, velocity, force)
    R'      = M R M^T               (rotations)
    w'      = det(M) * M w = -M w   (pseudovectors: angular velocity, torque)

The last line is what ROS-TCP-Connector's FLU.ConvertAngularVelocityFromRUF encodes
(`-FLU.ConvertFromRUF(w)`).
"""

import numpy as np

# Unity RUF -> FLU (x fwd, y left, z up). Matches ROS-TCP-Connector FLU.ConvertFromRUF.
M_RUF_TO_FLU = np.array([[0.0, 0.0, 1.0], [-1.0, 0.0, 0.0], [0.0, 1.0, 0.0]])
# Unity RUF -> FRD body / NED world (N = Unity +z, E = Unity +x, D = -Unity y).
M_RUF_TO_FRD = np.array([[0.0, 0.0, 1.0], [1.0, 0.0, 0.0], [0.0, -1.0, 0.0]])
M_RUF_TO_NED = M_RUF_TO_FRD
# Unity RUF -> ENU (CRANE/PX4-gazebo mapping used by CraneReferencePx4*: Unity (x,z,y)).
M_RUF_TO_ENU = np.array([[1.0, 0.0, 0.0], [0.0, 0.0, 1.0], [0.0, 1.0, 0.0]])
# FLU <-> FRD (both right-handed): rotation by pi about x.
M_FLU_TO_FRD = np.diag([1.0, -1.0, -1.0])


def polar(m, v):
    return np.asarray(m) @ np.asarray(v, dtype=float)


def pseudo(m, w):
    m = np.asarray(m)
    return np.linalg.det(m) * (m @ np.asarray(w, dtype=float))


def rotation(m, r):
    m = np.asarray(m)
    return m @ np.asarray(r) @ m.T


def skew(v):
    x, y, z = v
    return np.array([[0.0, -z, y], [z, 0.0, -x], [-y, x, 0.0]])


def axis_angle_matrix(axis, angle):
    axis = np.asarray(axis, dtype=float)
    axis = axis / np.linalg.norm(axis)
    k = skew(axis)
    return np.eye(3) + np.sin(angle) * k + (1.0 - np.cos(angle)) * (k @ k)


def log_so3(r):
    """Rotation vector of a rotation matrix (angle < pi)."""
    c = np.clip((np.trace(r) - 1.0) * 0.5, -1.0, 1.0)
    angle = np.arccos(c)
    if angle < 1e-12:
        return np.zeros(3)
    w = np.array([r[2, 1] - r[1, 2], r[0, 2] - r[2, 0], r[1, 0] - r[0, 1]])
    return w * angle / (2.0 * np.sin(angle))


def unity_euler_matrix(euler_deg):
    """Unity Transform.eulerAngles convention: rotate z, then x, then y (extrinsic)."""
    x, y, z = np.radians(euler_deg)
    return (axis_angle_matrix([0, 1, 0], y) @ axis_angle_matrix([1, 0, 0], x)
            @ axis_angle_matrix([0, 0, 1], z))


def zyx_euler(r):
    """(roll, pitch, yaw) of a right-handed rotation R = Rz(yaw) Ry(pitch) Rx(roll)."""
    pitch = -np.arcsin(np.clip(r[2, 0], -1.0, 1.0))
    roll = np.arctan2(r[2, 1], r[2, 2])
    yaw = np.arctan2(r[1, 0], r[0, 0])
    return np.array([roll, pitch, yaw])
