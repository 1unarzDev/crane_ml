"""The FLU axis contract: a positive surge, sway and yaw request moves the boat along that axis.

Input is the player's command-response result: ``{status, phases: [{name, sampleCount,
body_surge|body_sway|body_yaw_rate: {meanTail}, deltaPose: {yaw}}]}`` with the phases
``surge_positive``, ``sway_positive`` and ``yaw_positive``.
"""

import math

from . import report


def _tail(phase, field):
    value = (phase.get(field) or {}).get('meanTail')
    return float(value) if value is not None else math.nan


def _phase(result, name):
    return next((p for p in result.get('phases', []) if p.get('name') == name), None)


def check(result, dominance_ratio):
    failures = []
    if result.get('status') != 'completed':
        failures.append(f"fixture did not complete: status={result.get('status')!r}")

    for axis, cross in (('surge', 'sway'), ('sway', 'surge')):
        phase = _phase(result, f'{axis}_positive')
        own = _tail(phase, f'body_{axis}') if phase else math.nan
        other = _tail(phase, f'body_{cross}') if phase else math.nan
        if not phase or not phase.get('sampleCount') or not math.isfinite(own) or not math.isfinite(other):
            failures.append(f'{axis} request phase has no usable odometry samples')
        elif own <= 0.0 or abs(own) < dominance_ratio * abs(other):
            failures.append(f'{axis} request mapped incorrectly: {axis}={own:.6f}, {cross}={other:.6f}')

    phase = _phase(result, 'yaw_positive')
    rate = _tail(phase, 'body_yaw_rate') if phase else math.nan
    turned = float((phase or {}).get('deltaPose', {}).get('yaw', math.nan))
    if not phase or not phase.get('sampleCount') or not math.isfinite(rate) or not math.isfinite(turned):
        failures.append('yaw request phase has no usable odometry samples')
    elif rate <= 0.0 or turned <= 0.0:
        # Both positive also means pose and twist agree in sign.
        failures.append(f'positive yaw request mapped incorrectly: pose_delta={turned:.6f}, yaw_rate={rate:.6f}')
    return report('axis', failures)
