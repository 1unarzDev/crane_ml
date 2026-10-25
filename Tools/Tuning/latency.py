"""Command-to-response latency of the boat from open-loop yaw onsets. ROS-free core.

What the velocity loop and Nav2 feel is the round trip: a pulse leaves thruster_pwm_node, the
thrust builds, the hull turns, and the turn reaches the odometry. All three are taken from
receipt times on the one computer (bag record times), so no clock has to be aligned.

Each onset from rest (all four outputs neutral for >= ``rest`` s, then a step) is fitted with a
first-order-plus-dead-time response of the yaw rate,

    r(t) = r0 + K (1 - exp(-(t - t_on - L) / tau))   for t > t_on + L, else r0,

by a grid over (L, tau) with r0 and K solved by least squares at each node. L is the apparent
dead time (pure delay + about half the ESC/prop spool, Skogestad's half rule), tau the hull's
yaw time constant (I_z / linearized yaw damping). Yaw is used because its tau is the shortest
(0.1-0.3 s), so L is best separated from it, and the gyro is the least noisy channel.
"""
from __future__ import annotations

import numpy as np

L_GRID = np.arange(0.0, 0.801, 0.01)
TAU_GRID = np.arange(0.04, 1.501, 0.02)


def onsets(t_pwm, pulses, rest=1.0, band=25.0):
    """Times when the outputs leave neutral after at least ``rest`` s of neutral."""
    active = np.any(np.abs(np.asarray(pulses, float) - 1500.0) >= band, axis=1)
    found, quiet_since = [], None
    for t, a in zip(t_pwm, active):
        if not a:
            quiet_since = t if quiet_since is None else quiet_since
        else:
            if quiet_since is not None and t - quiet_since >= rest:
                found.append(float(t))
            quiet_since = None
    return found


def fit_fopdt(t, r, t_on, window=3.0, pre=0.5):
    """Fit one onset. Returns {L, tau, K, r0, rmse, n} or None when the data cannot say."""
    t, r = np.asarray(t, float), np.asarray(r, float)
    keep = (t >= t_on - pre) & (t <= t_on + window)
    tt, rr = t[keep] - t_on, r[keep]
    if len(tt) < 10 or np.ptp(rr) < 1e-3:
        return None
    best = None
    for L in L_GRID:
        after = tt > L
        for tau in TAU_GRID:
            shape = np.where(after, 1.0 - np.exp(-(tt - L) / tau), 0.0)
            A = np.stack([np.ones_like(tt), shape], 1)
            coef, res, *_ = np.linalg.lstsq(A, rr, rcond=None)
            sse = float(np.sum((A @ coef - rr) ** 2))
            if best is None or sse < best[0]:
                best = (sse, L, tau, coef)
    sse, L, tau, (r0, K) = best
    # A dead time on the grid's edge, or a response too small for the noise, says nothing.
    noise = float(np.std(rr[tt < 0])) if np.sum(tt < 0) > 3 else 0.0
    if abs(K) < 4 * noise or L >= L_GRID[-1] or tau >= TAU_GRID[-1]:
        return None
    return {'L': round(float(L), 3), 'tau': round(float(tau), 3), 'K': float(K), 'r0': float(r0),
            'rmse': float(np.sqrt(sse / len(tt))), 'n': int(len(tt))}


def estimate(t_pwm, pulses, t_rate, rate, **kw):
    """All onsets: per-onset fits, and the median L and tau with a bootstrap 90% interval."""
    fits = [f | {'t_on': t0} for t0 in onsets(t_pwm, pulses)
            if (f := fit_fopdt(t_rate, rate, t0, **kw)) is not None]
    if not fits:
        return {'fits': [], 'L': None, 'tau': None}
    L = np.array([f['L'] for f in fits])
    rng = np.random.default_rng(0)
    boot = [np.median(rng.choice(L, len(L))) for _ in range(2000)]
    return {'fits': fits, 'L': float(np.median(L)), 'L_ci90': [float(np.percentile(boot, 5)),
            float(np.percentile(boot, 95))], 'tau': float(np.median([f['tau'] for f in fits])),
            'n': len(fits)}
