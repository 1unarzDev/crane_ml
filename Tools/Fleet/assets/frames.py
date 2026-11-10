"""Frame conventions and the robot -> Unity conversion.

Robot/manifest frame: right-handed, x forward, y left, z up, metres, radians (URDF/ROS REP-103).
Unity frame: LEFT-handed, x right, y up, z forward, metres.

    (x_u, y_u, z_u) = (-y_r, z_r, x_r)          C = [[0,-1,0],[0,0,1],[1,0,0]],  det C = -1

Because det C = -1 the conversion also (a) reverses triangle winding, (b) negates rotation angles
about the mapped axis, (c) turns quaternion (x,y,z,w)_r into (y, -z, -x, w)_r.
"""
import math
import numpy as np

C = np.array([[0, -1, 0], [0, 0, 1], [1, 0, 0]], float)


def vec_to_unity(v):
    return C @ np.asarray(v, float)


def R_to_unity(R):
    """Conjugation C R C^T: a proper rotation in the left-handed frame (same physical motion)."""
    return C @ np.asarray(R, float) @ C.T


def quat_to_unity(q):
    x, y, z, w = q
    return np.array([y, -z, -x, w])


def R_to_quat(R):
    """(x,y,z,w) from a rotation matrix."""
    t = np.trace(R)
    if t > 0:
        s = math.sqrt(t + 1) * 2
        return np.array([(R[2, 1] - R[1, 2]) / s, (R[0, 2] - R[2, 0]) / s, (R[1, 0] - R[0, 1]) / s, s / 4])
    i = int(np.argmax(np.diag(R)))
    j, k = (i + 1) % 3, (i + 2) % 3
    s = math.sqrt(R[i, i] - R[j, j] - R[k, k] + 1) * 2
    q = np.zeros(4)
    q[i] = s / 4
    q[j] = (R[j, i] + R[i, j]) / s
    q[k] = (R[k, i] + R[i, k]) / s
    q[3] = (R[k, j] - R[j, k]) / s
    return q


def quat_to_R(q):
    x, y, z, w = q / np.linalg.norm(q)
    return np.array([[1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
                     [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
                     [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]])


def yaw_to_unity_euler_y_deg(yaw_rad):
    """A left turn (CCW seen from above, +z_r) is a NEGATIVE rotation about Unity's +y."""
    return -math.degrees(yaw_rad)


def tris_to_unity(P, idx=None):
    """Positions (...,3) -> Unity; if idx (T,3) triangle indices are given, also swap winding."""
    P = np.asarray(P, float) @ C.T
    if idx is None:
        return P
    idx = np.asarray(idx)
    return P, idx[:, [0, 2, 1]]


# camera optical frame (OpenCV/ROS optical): x right, y down, z forward, expressed in the link frame
# (x forward, y left, z up): columns are the optical axes in link coordinates.
R_LINK_TO_OPTICAL = np.array([[0, 0, 1], [-1, 0, 0], [0, -1, 0]], float)

# AprilTag3 detector convention for the tag frame: x right, y down (as seen by a viewer), z INTO the tag.


def worked_example():
    """Numbers quoted in ASSET_MANIFEST_SPEC.md (and asserted by tests)."""
    p_r = np.array([0.21, 0.06, 0.06])             # front-face hook pivot-ish point, robot frame
    return {"point_robot": p_r.tolist(), "point_unity": vec_to_unity(p_r).tolist(),
            "yaw_90deg_left_unity_euler_y_deg": yaw_to_unity_euler_y_deg(math.pi / 2),
            "forward_x_robot_in_unity": vec_to_unity([1, 0, 0]).tolist(),
            "left_y_robot_in_unity": vec_to_unity([0, 1, 0]).tolist(),
            "up_z_robot_in_unity": vec_to_unity([0, 0, 1]).tolist()}
