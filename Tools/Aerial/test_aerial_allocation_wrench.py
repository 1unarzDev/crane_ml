"""Ties the allocator (aerial_allocation) to the physical wrench (aerial_model.wrench).

Without this, flipping a sign in `effectiveness_matrix` (e.g. the yaw column) leaves every other
test passing: the allocator is only ever compared with itself.  Here the allocated thrusts are
turned into rotor speeds and pushed through the plant's own `wrench`; the body force/torque
must reproduce the requested wrench, one axis at a time and combined.
"""

import math

import numpy as np
import pytest

import aerial_allocation as alloc
import aerial_model as am
from aerial_profiles import load_vehicle

NAMES = ["x500_px4_gz", "crazyflie21", "rotors_firefly_hex", "crane_reference_quad"]


def _achieved(vehicle, wrench_request):
    b = alloc.effectiveness_matrix(vehicle)
    u, achieved, exact = alloc.allocate(b, alloc.max_thrusts(vehicle), wrench_request)
    assert exact, "request must be unsaturated for this test"
    speeds = np.sqrt(u / np.array([r.kf for r in vehicle.rotors]))
    state = am.initial_state(vehicle, p=(0, 0, 100.0), rotor_speed=speeds)   # v = 0, omega = 0, no ground effect
    force, torque = am.wrench(vehicle, state)
    return np.concatenate([[force[2]], torque]), force


@pytest.mark.parametrize("name", NAMES)
@pytest.mark.parametrize("axis", [0, 1, 2, 3])
def test_allocated_thrusts_produce_requested_wrench_on_each_axis(name, axis):
    v = load_vehicle(name, with_battery=False)
    weight = v.mass * v.gravity
    arm = max(np.linalg.norm(r.position[:2]) for r in v.rotors)
    scale = np.array([0.0, 0.05 * weight * arm, 0.05 * weight * arm,
                      0.05 * weight * max(r.km for r in v.rotors)])
    for sign in (1.0, -1.0):
        request = np.array([weight, 0.0, 0.0, 0.0])
        if axis > 0:
            request[axis] += sign * scale[axis]
        got, force = _achieved(v, request)
        assert got == pytest.approx(request, rel=1e-9, abs=1e-12), (name, axis, sign)
        assert abs(force[0]) < 1e-12 and abs(force[1]) < 1e-12     # purely along body z


@pytest.mark.parametrize("name", NAMES)
def test_combined_wrench_round_trip(name):
    v = load_vehicle(name, with_battery=False)
    weight = v.mass * v.gravity
    arm = max(np.linalg.norm(r.position[:2]) for r in v.rotors)
    km = max(r.km for r in v.rotors)
    request = np.array([1.1 * weight, 0.03 * weight * arm, -0.02 * weight * arm, 0.04 * weight * km])
    got, _ = _achieved(v, request)
    assert got == pytest.approx(request, rel=1e-9, abs=1e-12)


def test_positive_yaw_torque_needs_clockwise_rotors_faster_than_ccw_for_ccw_reaction():
    """Reaction torque is -spin*km*T: speeding up CCW (+1) rotors yields NEGATIVE body yaw torque."""
    v = load_vehicle("x500_px4_gz", with_battery=False)
    w0 = am.hover_rotor_speed(v)
    speeds = np.array([r.spin for r in v.rotors], dtype=float) * 50.0 + w0
    state = am.initial_state(v, p=(0, 0, 100.0), rotor_speed=speeds)
    _, torque = am.wrench(v, state)
    assert torque[2] < 0
