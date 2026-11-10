"""Counterexamples against the current CRANE C# aerial code (mirrored in python).

Each test PASSES when the defect is present, so it documents the defect. When a C# fix lands,
update the mirror and flip the assertion into a regression test.  Labels in docstrings:
VERIFIED = demonstrated numerically from source-mirrored formulas;
HYPOTHESIS = consistent with the evidence, cause not isolated.
"""

import math

import numpy as np
import pytest

import aerial_unity_mirror as um
from aerial_frames import M_RUF_TO_FLU, axis_angle_matrix, log_so3, rotation
from aerial_profiles import read_profile

# Headless run of Builds/CRANE-Worker (built 2026-09-19, i.e. older than the working tree):
# CRANE.x86_64 -batchmode -nographics --crane-profile train-cpu --crane-disable-ros-tcp
#   --crane-aerial-validation --crane-aerial-scene "Aerial Vehicle Validation"
LIVE_STALE_BUILD = {"roll": 4.710597515106201, "pitch": 4.683465957641602,
                    "yaw": 0.23068779706954957, "hover": 0.6781178116798401}


def test_imu_gyro_uses_polar_conversion_so_signs_flip():
    """VERIFIED (Imu.cs:35; also truth-inventory V4): To<FLU>() on a pseudovector."""
    w_unity = np.array([0.0, 0.3, 0.0])              # numeric Unity yaw rate
    dt = 1e-4
    r_unity = axis_angle_matrix(w_unity, np.linalg.norm(w_unity) * dt)
    true_flu = log_so3(rotation(M_RUF_TO_FLU, r_unity)) / dt
    published = um.imu_angular_velocity(w_unity)
    assert np.allclose(published, -true_flu, atol=1e-6)


def test_ardupilot_json_signs_and_frames():
    """VERIFIED (MAVROSConnection.cs:169-199 vs ArduPilot SITL JSON readme @9c412ba)."""
    pos = np.array([1.0, 5.0, 2.0])                  # Unity: 5 m above ground
    vel = np.array([0.0, 1.0, 0.0])                  # climbing
    sent = um.mavros_telemetry(pos, vel, np.zeros(3), [0, 0, 0])
    want = um.ardupilot_expected(pos, vel, np.zeros(3), [0, 0, 0])
    assert sent["position"][2] == pytest.approx(5.0) and want["position"][2] == pytest.approx(-5.0)
    assert sent["velocity"][2] == pytest.approx(-want["velocity"][2])
    # roll by +10 deg about Unity z: ArduPilot roll sign is reversed
    sent = um.mavros_telemetry(pos, vel, np.zeros(3), [0, 0, 10])
    want = um.ardupilot_expected(pos, vel, np.zeros(3), [0, 0, 10])
    assert sent["attitude"][0] == pytest.approx(-want["attitude"][0])
    # pitch by +10 deg about Unity x: reversed as well; yaw agrees
    sent = um.mavros_telemetry(pos, vel, np.zeros(3), [10, 0, 0])
    want = um.ardupilot_expected(pos, vel, np.zeros(3), [10, 0, 0])
    assert sent["attitude"][1] == pytest.approx(-want["attitude"][1])
    sent = um.mavros_telemetry(pos, vel, np.zeros(3), [0, 30, 0])
    want = um.ardupilot_expected(pos, vel, np.zeros(3), [0, 30, 0])
    assert sent["attitude"][2] == pytest.approx(want["attitude"][2])
    # gyro: world-frame rates sent as body rates; with 90 deg heading roll/pitch swap
    w_world = np.array([0.0, 0.0, 0.5])
    sent = um.mavros_telemetry(pos, vel, w_world, [0, 90, 0])
    want = um.ardupilot_expected(pos, vel, w_world, [0, 90, 0])
    assert not np.allclose(sent["gyro"], want["gyro"], atol=1e-3)
    assert abs(want["gyro"][0]) < 1e-9 and abs(sent["gyro"][0]) == pytest.approx(0.5)


def test_ardupilot_servo_order_and_crane_spin_pattern_mismatch():
    """VERIFIED: AP_MotorsMatrix QUAD/X order is FR,RL,FL,RR; CRANE maps ch0-3 to FL,FR,RR,RL.
    CRANE spins FL/RR CCW, PX4 x500 and ArduCopter spin FR/RL CCW (mirror image)."""
    assert um.ARDUCOPTER_QUAD_X != um.CRANE_SERVO_TO_ROTOR
    crane = {r["name"]: r["spin"] for r in read_profile("crane_reference_quad")["rotors"]}
    x500 = {"front_right": 1, "rear_left": 1, "front_left": -1, "rear_right": -1}
    assert all(crane[k] == -x500[k] for k in x500)


def test_per_axis_quadratic_drag_depends_on_airflow_heading():
    """VERIFIED (MultirotorDynamics.DragAxis): v_i|v_i| per body axis is not isotropic even
    with equal x/z coefficients: 5 m/s at 45 deg gets 20 % less drag and off-axis flow is
    pushed sideways."""
    def drag(heading_deg, speed=5.0):
        a = math.radians(heading_deg)
        v = np.array([speed * math.sin(a), 0.0, speed * math.cos(a)])
        return v, um.drag_local(v)
    _, f0 = drag(0)
    _, f45 = drag(45)
    assert np.linalg.norm(f45) / np.linalg.norm(f0) == pytest.approx(0.798, abs=0.002)
    v30, f30 = drag(30)
    angle = math.degrees(math.acos(-f30 @ v30 / np.linalg.norm(f30) / np.linalg.norm(v30)))
    assert angle > 3.0


def test_mixer_clamps_without_priority_and_loses_authority():
    """VERIFIED (SetCommand): independent Clamp01 per motor near full collective."""
    sat = um.set_command(0.95, 1.0, 0.0, 0.0)
    _, tau_sat = um.rotor_wrench_ruf(sat)
    raw = np.array([0.75, 1.15, 1.15, 0.75])
    _, tau_raw = um.rotor_wrench_ruf(raw)
    assert tau_sat[2] / tau_raw[2] == pytest.approx(0.576, abs=0.01)
    f_sat, _ = um.rotor_wrench_ruf(sat)
    f_hold, _ = um.rotor_wrench_ruf([0.95] * 4)
    assert f_sat[1] < f_hold[1]                     # collective silently changes too


def test_gust_phase_depends_on_absolute_time_not_episode_time():
    """VERIFIED (ApplyAerodynamics uses Time.fixedTime; ResetEpisode does not rewind it)."""
    amp = np.array([1.0, 0.0, 0.0])
    assert um.gust(0.0, amp)[0] == 0.0
    assert abs(um.gust(12.34, amp)[0]) > 0.5         # same episode step, different force


def test_float32_sitl_timestamp_resolution_exceeds_copter_loop_period():
    """VERIFIED: float timestamp ulp (3.9 ms) > 1/400 s from 2^15 s (9.1 h) of sim time;
    at 6 h it is already 1.95 ms (78 % of a 400 Hz tick)."""
    assert um.float32_timestamp_resolution(2 ** 15) > 1 / 400
    assert um.float32_timestamp_resolution(6 * 3600) > 0.75 / 400
    assert um.float32_timestamp_resolution(60) < 1e-5


def test_documented_windy_displacement_contradicts_drag_model():
    """VERIFIED inconsistency, HYPOTHESIS cause: Docs/ReferenceEnvironments.md reports
    (3.267, 4.732) m Unity x/z displacement for wind (5,0,2) m/s. A pure-translation response
    of the serialized drag model gives far less z than x; the doc's z > x needs another force
    path (attitude drift, a different build or swapped fields)."""
    p = um.translate_in_wind([5.0, 0.0, 2.0], 2.0)
    assert p[0] > 2.5 * p[2]
    assert p[2] < 1.5 < 4.732


def _replay_axis(axis):
    """Semi-implicit Euler mirror of CraneAerialValidationRunner.MeasureAxisResponse."""
    dt = um.FIXED_DT
    mass, g = 1.5, 9.81
    hover = math.sqrt(mass * g / (4 * um.T_MAX))
    i_roll = mass / 12 * (0.16 ** 2 + 0.5 ** 2)
    i_yaw = mass / 12 * (0.5 ** 2 + 0.5 ** 2)
    target = um.set_command(hover, *(0.25 if axis == k else 0.0 for k in ("roll", "pitch", "yaw")))
    speed = np.full(4, hover)
    w = 0.0
    alpha = 1 - math.exp(-dt / um.TAU)
    for _ in range(math.ceil(0.4 / dt)):
        speed = speed + (target - speed) * alpha
        _, tau = um.rotor_wrench_ruf(speed)
        component = {"roll": tau[2], "pitch": tau[0], "yaw": tau[1]}[axis]
        inertia = i_yaw if axis == "yaw" else i_roll
        w += (component - 0.025 * w) / inertia * dt
    return w, hover


def test_mirror_reproduces_live_roll_and_pitch_but_not_yaw():
    """LIVE_SIM (stale build): roll matches to 1e-6; pitch within 0.6 %; yaw differs by 22 %
    (HYPOTHESIS: the Sep-19 build predates the current yaw path or PhysX inertia differs)."""
    roll, hover = _replay_axis("roll")
    assert hover == pytest.approx(LIVE_STALE_BUILD["hover"], rel=1e-6)
    assert roll == pytest.approx(LIVE_STALE_BUILD["roll"], rel=1e-5)
    pitch, _ = _replay_axis("pitch")
    assert pitch == pytest.approx(LIVE_STALE_BUILD["pitch"], rel=0.01)
    yaw, _ = _replay_axis("yaw")
    assert abs(yaw / LIVE_STALE_BUILD["yaw"] - 1) > 0.2
