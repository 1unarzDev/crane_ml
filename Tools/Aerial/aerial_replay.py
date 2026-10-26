"""Format-agnostic real-flight replay metrics (Blackbird-style: mocap pose + motor RPM).

Inputs are plain arrays so a separate, reviewed adapter can convert a dataset's bag/CSV
without this module ever parsing untrusted files.  World frame z-up, body FLU,
quaternion [w,x,y,z] body->world.
"""

import math

import numpy as np

from aerial_model import quat_to_matrix


def smooth_derivatives(t, x, half_window):
    """Local quadratic least-squares fit -> (value, first, second derivative) per sample."""
    t = np.asarray(t, dtype=float)
    x = np.asarray(x, dtype=float)
    n = len(t)
    out1 = np.full_like(x, np.nan)
    out2 = np.full_like(x, np.nan)
    for k in range(half_window, n - half_window):
        sl = slice(k - half_window, k + half_window + 1)
        tt = t[sl] - t[k]
        a = np.vstack([np.ones_like(tt), tt, 0.5 * tt * tt]).T
        coef = np.linalg.lstsq(a, x[sl], rcond=None)[0]
        out1[k] = coef[1]
        out2[k] = coef[2]
    return out1, out2


def measured_body_specific_force(t, p_world, q, gravity=9.81, half_window=8):
    """Specific force (non-gravitational accel) in body frame from mocap pose."""
    acc = np.zeros_like(p_world, dtype=float)
    for axis in range(3):
        _, acc[:, axis] = smooth_derivatives(t, p_world[:, axis], half_window)
    out = np.full_like(acc, np.nan)
    for k in range(len(t)):
        if np.all(np.isfinite(acc[k])):
            r = quat_to_matrix(q[k])
            out[k] = r.T @ (acc[k] + np.array([0.0, 0.0, gravity]))
    return out


def predicted_thrust_accel(rotor_speed_rad_s, kf, mass):
    w = np.asarray(rotor_speed_rad_s, dtype=float)
    return (np.asarray(kf) * w * w).sum(axis=1) / mass


def rpm_to_rad_s(rpm):
    return np.asarray(rpm, dtype=float) * 2.0 * math.pi / 60.0


def thrust_replay_report(t, p_world, q, rotor_speed_rad_s, kf, mass, gravity=9.81,
                         half_window=8):
    meas = measured_body_specific_force(t, p_world, q, gravity, half_window)
    pred = predicted_thrust_accel(rotor_speed_rad_s, kf, mass)
    ok = np.isfinite(meas[:, 2])
    z = meas[ok, 2]
    pz = pred[ok]
    scale = float(pz @ z / (pz @ pz))
    resid = z - pz
    return {
        "samples": int(ok.sum()),
        "kf_scale_fit": scale,
        "thrust_rms_residual": float(np.sqrt(np.mean(resid ** 2))),
        "thrust_bias": float(np.mean(resid)),
        "lateral_specific_force_rms": float(np.sqrt(np.mean(np.sum(meas[ok, :2] ** 2, axis=1)))),
    }
