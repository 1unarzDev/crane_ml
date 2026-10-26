"""Excitation protocols on the fixture: per-thruster pulses, wrench steps/chirps, heading sweeps."""
from __future__ import annotations

import math

import numpy as np

REST_S, HOLD_S = 1.0, 3.0


def pulse_log(builder, i: int, cmd: float, rest=REST_S, hold=HOLD_S, tail=0.0):
    """Output ``i`` alone at ``cmd`` for ``hold`` s after ``rest`` s at zero, from rest."""
    m = builder()
    n = int(round((rest + hold + tail) / m.dt))
    k_on = int(round(rest / m.dt))
    k_off = int(round((rest + hold) / m.dt))

    def fn(k, t, mod):
        c = [0.0] * 4
        if k_on <= k < k_off:
            c[i] = cmd
        return c
    return m.run(n, fn)


def all_pulses(builder, levels=(0.3, 0.6, -0.3, -0.6), outputs=(0, 1, 2, 3)):
    out = []
    for i in outputs:
        for c in levels:
            cmd = [0.0] * 4
            cmd[i] = c
            out.append((tuple(cmd), pulse_log(builder, i, c)))
    return out


def mixed_pulse_log(builder, cmd, rest=REST_S, hold=HOLD_S):
    m = builder()
    n = int(round((rest + hold) / m.dt))
    k_on = int(round(rest / m.dt))
    return m.run(n, lambda k, t, mod: list(cmd) if k >= k_on else [0.0] * 4)


def wrench_program(builder, program, seconds):
    """Drive with allocate(program(t)) each tick. Returns (log, wrench_requested array)."""
    m = builder()
    n = int(round(seconds / m.dt))
    req = []

    def fn(k, t, mod):
        w = program(t)
        req.append(w)
        return mod.allocate(w).commands
    return m.run(n, fn), np.array(req)


def step_chirp_program(amp_x=40.0, amp_n=8.0, t_step=1.0, f0=0.2, f1=1.0, t_end=14.0):
    """Surge step at t_step, then a surge+yaw chirp. Rich enough to fix a delay to a few ms."""
    def prog(t):
        if t < t_step:
            return (0.0, 0.0, 0.0)
        tau = t - t_step
        if tau < 3.0:
            return (amp_x, 0.0, 0.0)
        s = tau - 3.0
        ph = 2 * math.pi * (f0 * s + 0.5 * (f1 - f0) / max(t_end - t_step - 3.0, 1e-9) * s * s)
        return (amp_x * 0.6 * math.sin(ph), 0.0, amp_n * math.sin(ph * 0.7))
    return prog


def hold_drift(builder, headings, hold_s=1.0, ramp_hold=False):
    """Zero-command holds at each heading. Returns per-heading mean (u, v, r) of the TRUTH.

    Start at the water-relative rest (body velocity = R^T current); with ramp_hold the state is
    carried across headings so a time-varying current shows up in the data."""
    m = builder()
    out = []
    n = int(round(hold_s / m.dt))
    for k, psi in enumerate(headings):
        if k == 0 or not ramp_hold:
            cx, cy = m._current_world(m.t)
            m.reset(state=(0.0, 0.0, psi, math.cos(psi) * cx + math.sin(psi) * cy,
                           -math.sin(psi) * cx + math.cos(psi) * cy, 0.0))
        else:
            m.set_heading(psi)
        log = m.run(n, lambda kk, t, mod: [0.0] * 4)
        out.append((float(np.mean(log.u)), float(np.mean(log.v)), float(np.mean(log.r))))
    return np.array(out)


def steady_state(builder, wrench, seconds=30.0, window=2.0):
    """Hold a requested wrench to steady state; returns mean (u,v,r) of the final window."""
    m = builder()
    cmd = m.allocate(wrench).commands
    n = int(round(seconds / m.dt))
    log = m.run(n, lambda k, t, mod: list(cmd))
    w = int(round(window / m.dt))
    return (float(np.mean(log.u[-w:])), float(np.mean(log.v[-w:])), float(np.mean(log.r[-w:])),
            m.allocate(wrench))
