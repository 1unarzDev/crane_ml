"""Loophole detection from invariants that are independent of the reward being optimised.

Set CRANE_EXP_DISABLE_LOOPHOLE_DETECTOR=1 to disable (used only by the meta-test that
proves the suite goes red without the detector).
"""
import math
import os

ENV_DISABLE = "CRANE_EXP_DISABLE_LOOPHOLE_DETECTOR"


def detector_enabled():
    return os.environ.get(ENV_DISABLE) != "1"


def _num(v):
    return isinstance(v, (int, float)) and math.isfinite(v)


def detect_loophole(result):
    """Return a list of violated-invariant names (empty = none found)."""
    if not detector_enabled():
        return []
    inv = result.get("invariants") or {}
    flags = []
    rp, op = inv.get("reported_progress"), inv.get("odom_progress")
    if _num(rp) and _num(op) and rp > 1.25 * op + 0.5:
        flags.append("PROGRESS_EXCEEDS_ODOMETRY")
    t, vmax = inv.get("sim_time_s"), inv.get("vmax")
    if _num(rp) and _num(t) and _num(vmax) and t > 0 and rp / t > 1.5 * vmax:
        flags.append("SPEED_EXCEEDS_VMAX")
    return flags
