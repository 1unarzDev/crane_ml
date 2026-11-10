"""Identifiability fixture: motor time constant vs IMU (gyro) latency.

Single-axis plant: m' = (u - m)/tau, omega' = b m, gyro y(t) = omega(t - d) + noise.
At frequencies f << 1/(2 pi tau), e^{-sd}/(1 + s tau) ~ e^{-s(tau+d)}: only tau + d is
identifiable from (u, gyro).  A motor-speed channel (e.g. Blackbird's ~190 Hz RPM
sensors) observes m directly and breaks the degeneracy.
"""

import math

import numpy as np


def multisine(duration, dt, freqs_hz, amplitude=1.0, seed=0):
    rng = np.random.default_rng(seed)
    t = np.arange(0.0, duration, dt)
    phases = rng.uniform(0, 2 * math.pi, len(freqs_hz))
    u = sum(np.sin(2 * math.pi * f * t + ph) for f, ph in zip(freqs_hz, phases))
    return t, amplitude * u / len(freqs_hz)


def simulate(u, dt, tau, delay, gain=1.0):
    """Return (motor, rate, delayed_rate) for input u."""
    alpha = 1.0 - math.exp(-dt / tau)
    m = np.zeros_like(u)
    w = np.zeros_like(u)
    for k in range(1, len(u)):
        m[k] = m[k - 1] + (u[k - 1] - m[k - 1]) * alpha
        w[k] = w[k - 1] + gain * m[k - 1] * dt
    t = np.arange(len(u)) * dt
    delayed = np.interp(t - delay, t, w, left=0.0)      # fractional (linear-interp) delay
    return m, w, delayed


def gyro_filtered(u, dt, tau_motor, tau_filter, delay=0.0):
    """Gyro passed through a first-order sensor low-pass (e.g. IMU DLPF) after the plant."""
    _, _, y = simulate(u, dt, tau_motor, delay)
    beta = 1.0 - math.exp(-dt / tau_filter)
    out = np.zeros_like(y)
    for k in range(1, len(y)):
        out[k] = out[k - 1] + (y[k - 1] - out[k - 1]) * beta
    return out


def _detrend(x):
    t = np.arange(len(x))
    a = np.vstack([t, np.ones_like(t)]).T
    return x - a @ np.linalg.lstsq(a, x, rcond=None)[0]


def gyro_cost(u, y, dt, tau, delay, skip=0):
    """Least-squares residual RMS of gyro y given hypothesis (tau, delay); gain fitted."""
    _, _, x = simulate(u, dt, tau, delay)
    x, yy = _detrend(x[skip:]), _detrend(y[skip:])
    gain = float(x @ yy / (x @ x))
    return float(np.sqrt(np.mean((yy - gain * x) ** 2)))


def motor_cost(u, r, dt, tau, skip=0):
    m, _, _ = simulate(u, dt, tau, 0.0)
    m, rr = m[skip:], r[skip:]
    gain = float(m @ rr / (m @ m))
    return float(np.sqrt(np.mean((rr - gain * m) ** 2)))


def gyro_sensitivities(u, dt, tau, delay, skip=0, h=1e-4):
    """Columns d(gyro)/d(tau), d(gyro)/d(delay), d(gyro)/d(gain) at gain 1."""
    def g(ta, de):
        return simulate(u, dt, ta, de)[2][skip:]
    base = g(tau, delay)
    s_tau = (g(tau + h, delay) - g(tau - h, delay)) / (2 * h)
    s_del = (g(tau, delay + h) - g(tau, max(0.0, delay - h))) / (h + min(h, delay))
    return np.vstack([s_tau, s_del, base]).T


def crb_std(jacobian, sigma):
    """Cramer-Rao lower bound standard deviations for white noise sigma."""
    fisher = jacobian.T @ jacobian / sigma ** 2
    return np.sqrt(np.diag(np.linalg.inv(fisher)))


def partial_correlation(jacobian):
    """Correlation of (tau, delay) estimates implied by the Fisher matrix."""
    cov = np.linalg.inv(jacobian.T @ jacobian)
    return float(cov[0, 1] / np.sqrt(cov[0, 0] * cov[1, 1]))


def best_tau(costs, taus):
    return float(taus[int(np.argmin(costs))])
