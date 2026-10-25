"""Tests for the ROS-free core of motion_quality.py on synthetic series (no bag, no ROS)."""
from pathlib import Path

import numpy as np
import pytest
import yaml

import motion_quality as mq

THRESHOLDS = yaml.safe_load(Path(mq.__file__).with_name('motion_quality.yaml').read_text())
P = THRESHOLDS['params']
DT = 0.02  # 50 Hz, the sim's odometry rate


def track(t, surge, r, sway=None):
    """Odometry from a body-frame velocity profile, integrated into world x, y, yaw."""
    sway = np.zeros_like(t) if sway is None else sway
    yaw = np.concatenate(([0.0], np.cumsum(r[:-1] * np.diff(t))))
    vxw = surge * np.cos(yaw) - sway * np.sin(yaw)
    vyw = surge * np.sin(yaw) + sway * np.cos(yaw)
    x = np.concatenate(([0.0], np.cumsum(vxw[:-1] * np.diff(t))))
    y = np.concatenate(([0.0], np.cumsum(vyw[:-1] * np.diff(t))))
    return {'t': t, 'x': x, 'y': y, 'yaw': np.angle(np.exp(1j * yaw)), 'vx': surge, 'vy': sway, 'r': r}


def score(odom, **extra):
    return mq.score({'odom': odom, **extra}, THRESHOLDS)


def times(duration):
    return np.arange(0.0, duration, DT)


# ---- 1, 2: yaw reversals and shaking

def test_sinusoidal_yaw_rate_reverses_twice_per_period():
    t = times(120.0)
    out = score(track(t, np.full_like(t, 0.3), 0.2 * np.sin(2 * np.pi * t / 10.0)))
    # 24 zero crossings in 120 s; the first lobe has nothing before it, so 23 reversals in 2 min.
    assert out['metrics']['yaw_reversals_per_min'] == pytest.approx(11.5, abs=0.1)
    assert out['metrics']['shake_index'] == 0.0  # half-period 5 s is a slow weave, not shaking
    assert out['pass']['yaw_reversals_per_min'] is False


def test_fast_sinusoid_is_shaking():
    t = times(60.0)
    out = score(track(t, np.full_like(t, 0.3), 0.2 * np.sin(2 * np.pi * t / 4.0)))
    # Period 4 s: half-period 2 s < 4 s and peak 0.2 > 0.1, so every reversal shakes.
    assert out['metrics']['shake_index'] == pytest.approx(29.0, abs=0.1)
    assert len(out['segments']['shakes']) == 29


def test_dither_inside_hysteresis_band_does_not_count():
    t = times(60.0)
    r = 0.04 * np.sign(np.sin(2 * np.pi * t / 1.0))  # square dither at ±0.04 < h = 0.05
    out = score(track(t, np.full_like(t, 0.3), r))
    assert out['metrics']['yaw_reversals_per_min'] == 0.0


def test_turn_stop_turn_back_is_not_a_reversal():
    t = times(15.0)
    moving = (t < 5) | (t >= 10)
    r = np.where(t < 5, 0.2, np.where(t >= 10, -0.2, 0.0))
    out = score(track(t, np.where(moving, 0.3, 0.0), r))
    assert out['metrics']['yaw_reversals_per_min'] == 0.0


# ---- 3, 4: command and thruster chatter

def test_cmd_chatter_per_axis():
    t = np.arange(0.0, 40.0, 0.1)
    cmd = {'t': t, 'vx': np.full_like(t, 0.5),
           'vy': 0.01 * np.sign(np.sin(2 * np.pi * t / 0.5)),  # inside the 0.02 band
           'wz': 0.1 * np.sign(np.sin(2 * np.pi * t / 4.0 + 0.1))}  # flips every 2 s
    out = score(track(times(40.0), np.full_like(times(40.0), 0.3), np.zeros(2000)), cmd_vel_nav=cmd)
    axes = out['metrics']['cmd_chatter_hz_axes']
    assert axes['vx'] == 0.0 and axes['vy'] == 0.0
    assert axes['wz'] == pytest.approx(19 / t[-1], abs=1e-6)
    assert out['metrics']['cmd_chatter_hz'] == axes['wz']
    assert out['pass']['cmd_chatter_hz'] is False


def test_thruster_chatter_ignores_esc_deadband():
    t = np.arange(0.0, 20.0, 0.025)
    us = np.full((len(t), 4), 1500.0)
    us[:, 0] += 100 * np.sign(np.sin(2 * np.pi * t / 2.0 + 0.1))  # flips once a second: 20 in 20 s
    us[:, 1] += 20 * np.sign(np.sin(2 * np.pi * t / 0.2))  # ±20 µs: inside the deadband
    odom = track(times(20.0), np.full(1000, 0.3), np.zeros(1000))
    out = score(odom, pwm={'t': t, 'us': us})
    rate = 20 / t[-1]
    assert out['metrics']['thruster_chatter_hz'] == pytest.approx(rate)
    assert out['metrics']['thruster_chatter_hz_mean'] == pytest.approx(rate / 4)
    assert out['metrics']['thruster_chatter_hz_each'][1] == 0.0


# ---- 5: jerk

def test_jerk_of_constant_velocity_is_zero():
    t = times(30.0)
    out = score(track(t, np.full_like(t, 0.5), np.zeros_like(t)))
    assert out['metrics']['jerk_rms'] == pytest.approx(0.0, abs=1e-9)


def test_jerk_of_sinusoidal_surge():
    t = times(126.0)  # 20 periods of omega = 1 rad/s
    out = score(track(t, 0.5 * np.sin(t), np.zeros_like(t)))
    # v = A sin(wt) has jerk -A w^2 sin(wt): RMS A w^2 / sqrt(2).
    assert out['metrics']['jerk_rms'] == pytest.approx(0.5 / np.sqrt(2), rel=0.03)
    assert out['metrics']['yaw_jerk_rms'] == pytest.approx(0.0, abs=1e-9)


# ---- 6: spin in place

def test_constructed_spin_is_found():
    t = times(26.0)
    spinning = (t >= 10) & (t < 16)
    out = score(track(t, np.where(spinning, 0.0, 0.5), np.where(spinning, 0.3, 0.0)))
    [spin] = out['segments']['spins']
    assert spin['start'] == pytest.approx(10.0, abs=DT)
    assert spin['duration_s'] == pytest.approx(6.0, abs=DT)
    assert spin['dyaw_rad'] == pytest.approx(1.8, abs=0.01)
    assert out['metrics']['spin_fraction'] == pytest.approx(6.0 / t[-1], abs=0.002)


def test_tight_circle_off_the_pivot_counts_as_spin_but_a_turn_does_not():
    t = times(20.0)
    # 0.3 rad/s with 0.07 m/s at base_link is a 0.23 m circle (spin); 0.3 m/s is a 1 m turn.
    surge = np.where(t < 10, 0.07, 0.3)
    out = score(track(t, surge, np.full_like(t, 0.3)))
    [spin] = out['segments']['spins']
    assert spin['duration_s'] == pytest.approx(10.0, abs=DT)


# ---- 7, 8, 10: goals

def goals_at(points, t0, t1):
    """/plan updates once a second for each (x, y, t_start) goal until the next starts (or t1)."""
    rows = [(t, x, y) for (x, y, start), stop in zip(points, [p[2] for p in points[1:]] + [t1])
            for t in np.arange(start, stop, 1.0)]
    return dict(zip(('t', 'x', 'y'), np.array(rows).T))


def test_overshoot_of_a_quarter_metre_is_reported():
    t = times(30.0)
    # Run along +x to 5.25 m (0.25 m past the goal at 5 m), come back to 5 m and hold.
    x = np.interp(t, [0, 10.5, 12, 30], [0, 5.25, 5.0, 5.0])
    odom = {'t': t, 'x': x, 'y': np.zeros_like(t), 'yaw': np.zeros_like(t),
            'vx': np.gradient(x, t), 'vy': np.zeros_like(t), 'r': np.zeros_like(t)}
    out = score(odom, goals=goals_at([(5.0, 0.0, 0.0)], 0.0, 15.0))
    [goal] = out['segments']['goals']
    assert goal['overshoot_m'] == pytest.approx(0.25, abs=1e-3)
    assert goal['reached_s'] == pytest.approx(4.6 / 5.25 * 10.5, abs=DT)
    assert goal['settle_s'] == pytest.approx(goal['reached_s'], abs=DT)  # never leaves 0.4 m
    assert goal['path_ratio'] == pytest.approx(5.5 / 5.0, abs=1e-3)
    assert out['metrics']['goal_overshoot_m'] == pytest.approx(0.25, abs=1e-3)
    assert out['pass']['goal_overshoot_m'] is True


def test_legs_split_on_goal_change_with_path_ratio_and_time():
    t = times(40.0)
    # Straight to (10, 0) by t = 20, then a dog-leg via (10, 5) to the second goal (15, 5).
    x = np.interp(t, [0, 20, 30, 40], [0, 10, 10, 15])
    y = np.interp(t, [0, 20, 30, 40], [0, 0, 5, 5])
    odom = {'t': t, 'x': x, 'y': y, 'yaw': np.zeros_like(t), 'vx': np.full_like(t, 0.5),
            'vy': np.zeros_like(t), 'r': np.zeros_like(t)}
    out = score(odom, goals=goals_at([(10.0, 0.0, 0.0), (15.0, 5.0, 20.0)], 0.0, 40.0))
    first, second = out['segments']['goals']
    assert first['time_s'] == pytest.approx(20.0) and second['time_s'] == pytest.approx(20.0)
    assert first['path_ratio'] == pytest.approx(1.0, abs=1e-3)
    assert second['path_ratio'] == pytest.approx(10.0 / np.hypot(5, 5), abs=3e-3)  # odom ends 20 ms early
    assert out['metrics']['path_ratio'] == second['path_ratio']
    assert out['pass']['path_ratio'] is False


# ---- 9: energy

def test_energy_full_forward_and_reverse():
    t = np.arange(0.0, 10.0 + 1e-9, 0.025)
    odom = track(times(10.0), np.full(500, 1.0), np.zeros(500))
    forward = score(odom, pwm={'t': t, 'us': np.full((len(t), 4), 1900.0)})['metrics']
    reverse = score(odom, pwm={'t': t, 'us': np.full((len(t), 4), 1100.0)})['metrics']
    assert forward['energy_proxy_Ns'] == pytest.approx(4 * 51.5 * 10.0)
    assert reverse['energy_proxy_Ns'] == pytest.approx(4 * 40.2 * 10.0)
    assert forward['energy_per_m_Ns'] == pytest.approx(4 * 51.5 * 10.0 / 9.98, rel=1e-3)


# ---- 11: stalls

def test_cmd_gap_is_a_stall_only_while_a_goal_is_active():
    t = times(40.0)
    odom = track(t, np.full_like(t, 0.1), np.zeros_like(t))
    cmd_t = np.concatenate((np.arange(0, 10, 0.1), np.arange(12, 20, 0.1), np.arange(30, 40, 0.1)))
    zeros = np.zeros_like(cmd_t)
    out = score(odom, cmd_vel_nav={'t': cmd_t, 'vx': zeros, 'vy': zeros, 'wz': zeros},
                goals=goals_at([(50.0, 0.0, 0.0)], 0.0, 18.0))  # updates 0..17 s: active to 18 s
    [stall] = out['segments']['stalls']
    assert stall['start'] == pytest.approx(9.9) and stall['duration_s'] == pytest.approx(2.1)
    assert out['metrics']['stall_count'] == 1 and out['pass']['stall_count'] is False


def test_missing_inputs_give_null_not_failure():
    t = times(10.0)
    out = score(track(t, np.full_like(t, 0.3), np.zeros_like(t)))
    assert out['metrics']['goal_overshoot_m'] is None and out['pass']['goal_overshoot_m'] is None
    assert 'cmd_chatter_hz' not in out['metrics'] and out['pass']['cmd_chatter_hz'] is None
