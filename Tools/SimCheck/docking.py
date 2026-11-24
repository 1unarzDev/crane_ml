"""Terminal docking: no pathological rotate-in-place before the path, and a settled stop at the goal.

A trajectory is a list of samples ``{simSeconds, x, y, yaw, bodySurge, bodySway, bodyYawRate,
commandSurge}`` (world frame, body twist), as the player's Nav2 run result records it.
"""

import math

from . import report


def _wrap(a):
    return math.remainder(a, 2.0 * math.pi)


def rotate_to_path(trajectory, linear_command_threshold, maximum_yaw_travel_rad, maximum_drift_m):
    """Yaw travelled and xy drift before the first forward command stay bounded."""
    failures = [] if len(trajectory) >= 2 else ['trajectory has fewer than two samples']
    before, forward = [], None
    for sample in trajectory:
        if abs(sample.get('commandSurge', 0.0)) > linear_command_threshold:
            forward = sample
            break
        before.append(sample)
    travel = sum(abs(_wrap(b['yaw'] - a['yaw'])) for a, b in zip(before, before[1:]))
    drift = math.dist((before[0]['x'], before[0]['y']), (before[-1]['x'], before[-1]['y'])) if before else math.nan
    if forward is None:
        failures.append('controller never issued a forward command')
    if travel > maximum_yaw_travel_rad:
        failures.append(f'rotate-only yaw travel {travel:.3f} rad exceeds {maximum_yaw_travel_rad:.3f} rad')
    if math.isfinite(drift) and drift > maximum_drift_m:
        failures.append(f'rotate-only XY drift {drift:.3f} m exceeds {maximum_drift_m:.3f} m')
    return failures, {'rotateOnlyYawTravelRadians': travel, 'rotateOnlyDriftMeters': drift,
                      'rotateOnlySampleCount': len(before), 'translationCommandObserved': forward is not None}


def settled_s(trajectory, goal, xy_tolerance_m, yaw_tolerance_rad, speed_tolerance_m_s,
              yaw_rate_tolerance_rad_s, **_):
    """Longest continuous time inside the goal tolerances while stopped."""
    gx, gy, gyaw = goal
    best, since = 0.0, None
    for s in trajectory:
        inside = (math.dist((s['x'], s['y']), (gx, gy)) <= xy_tolerance_m
                  and abs(_wrap(s['yaw'] - gyaw)) <= yaw_tolerance_rad
                  and math.hypot(s.get('bodySurge', 0.0), s.get('bodySway', 0.0)) <= speed_tolerance_m_s
                  and abs(s.get('bodyYawRate', 0.0)) <= yaw_rate_tolerance_rad_s)
        if inside:
            since = s['simSeconds'] if since is None else since
            best = max(best, s['simSeconds'] - since)
        else:
            since = None
    return best


def check(trajectory, limits, goal=None):
    failures, facts = rotate_to_path(trajectory, **limits['rotate_to_path'])
    if goal is not None:
        held = settled_s(trajectory, goal, **limits['settle'])
        facts['settledSeconds'] = round(held, 3)
        if held < limits['settle']['settle_s']:
            failures.append(f"settled {held:.2f} s within tolerance, need {limits['settle']['settle_s']:.2f} s")
    return report('docking', failures, **facts)
