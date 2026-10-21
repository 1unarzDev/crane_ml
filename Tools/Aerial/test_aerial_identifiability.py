"""Identifiability-failure fixtures: motor time constant vs IMU latency / IMU filtering."""

import numpy as np

import aerial_identifiability as ai

DT = 0.002
TAU = 0.03
DELAY = 0.02


def _fisher(band):
    _, u = ai.multisine(10.0, DT, band, seed=1)
    jac = ai.gyro_sensitivities(u, DT, TAU, DELAY, skip=250)
    return u, jac


def test_lag_and_latency_estimates_become_collinear_at_low_bandwidth():
    u_low, low = _fisher([0.2, 0.5, 0.9])
    u_high, high = _fisher([0.5, 1, 2, 4, 8, 13, 20, 30])
    c_low = ai.partial_correlation(low)
    c_high = ai.partial_correlation(high)
    assert c_low < -0.99, "near-hover excitation: only tau + delay is identifiable"
    assert c_high > c_low
    # equal 5 % relative gyro noise (same SNR) in both experiments
    s_low = 0.05 * np.std(ai.simulate(u_low, DT, TAU, DELAY)[2])
    s_high = 0.05 * np.std(ai.simulate(u_high, DT, TAU, DELAY)[2])
    assert ai.crb_std(low, s_low)[0] > 3 * ai.crb_std(high, s_high)[0]


def test_motor_speed_channel_breaks_the_degeneracy():
    u, jac = _fisher([0.2, 0.5, 0.9])
    y = ai.simulate(u, DT, TAU, DELAY)[2]
    sigma_g = 0.05 * np.std(y)
    m = lambda tau: ai.simulate(u, DT, tau, 0.0)[0][250:]
    s_m = (m(TAU + 1e-4) - m(TAU - 1e-4)) / 2e-4
    jm = np.zeros((len(s_m), 3))
    jm[:, 0] = s_m
    sigma_m = 0.05 * np.std(m(TAU))
    gyro_only = np.sqrt(np.linalg.inv(jac.T @ jac / sigma_g ** 2)[0, 0])
    fused = np.sqrt(np.linalg.inv(jac.T @ jac / sigma_g ** 2 + jm.T @ jm / sigma_m ** 2)[0, 0])
    assert fused < gyro_only / 10


def test_motor_lag_and_gyro_lowpass_are_exactly_interchangeable():
    """Two first-order lags commute: (tau_m, tau_f) and (tau_f, tau_m) give the same gyro."""
    _, u = ai.multisine(5.0, 0.0005, [0.5, 2, 7, 15, 40], seed=4)
    a = ai.gyro_filtered(u, 0.0005, 0.03, 0.01)
    b = ai.gyro_filtered(u, 0.0005, 0.01, 0.03)
    assert np.max(np.abs(a - b)) < 1e-3 * np.max(np.abs(a))
    m_a = ai.simulate(u, 0.0005, 0.03, 0.0)[0]
    m_b = ai.simulate(u, 0.0005, 0.01, 0.0)[0]
    assert np.max(np.abs(m_a - m_b)) > 0.1 * np.max(np.abs(m_a)), "RPM separates them"
