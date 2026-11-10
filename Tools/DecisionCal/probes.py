"""Open-loop probe log: the calibration budget every twin ablation shares.

Mirrors the 2026-10-18 field pulse trials (``plantcard/marine/fieldtrials.py``): each segment
starts at rest, holds neutral for a 12 s baseline (the drift gives the session current), applies a
pulse, then coasts. Added to the field protocol: low-effort steps (deadband), reversals,
coast-downs (every post window) and turn pairs. The log is built the way the field reader builds
it: GPS at 5 Hz -> local quadratic velocity (``fieldtrials.sg_velocity``) rotated into the body
by the heading, heading at 10 Hz, gyro at 20 Hz box-smoothed, everything stamped on arrival
(so an odometry latency looks exactly like dead time: ID-07), commands ZOH on a 10 Hz grid.
"""

import hashlib

import numpy as np

import _paths  # noqa: F401
from plant import DT, N_THR, Plant
from plantcard.family import Evidence, Segment
from plantcard.marine.fieldtrials import sg_velocity
from vendor.thruster_mixer import mix_thrusters

BASELINE_S, SEG_S, GRID_DT = 12.0, 26.0, 0.1
GPS_DT, HEAD_DT, GYRO_DT = 0.2, 0.1, 0.05


def _axis(s, w, r):
    return np.asarray(mix_thrusters(s, w, r), float)


def _single(i, q):
    v = np.zeros(N_THR)
    v[i] = q
    return v


# (name, group, [(duration s, q[4]), ...]) -- phases after the baseline; the rest of the segment coasts
PROTOCOL = (
    [(f'surge{e:+.1f}', 'surge', [(3.0, _axis(e, 0, 0))]) for e in (0.3, -0.3, 0.6, -0.6, 0.9, -0.9)]
    + [(f'sway{e:+.1f}', 'sway', [(3.0, _axis(0, e, 0))]) for e in (0.8, -0.8)]
    + [(f'yaw{e:+.1f}', 'yaw', [(2.0, _axis(0, 0, e))]) for e in (0.3, -0.3)]
    + [(f'single{i}{q:+.1f}', 'single', [(1.5, _single(i, q))]) for i in range(N_THR) for q in (0.5, -0.5)]
    + [('rev_surge', 'reversal', [(2.0, _axis(0.8, 0, 0)), (2.0, _axis(-0.8, 0, 0))]),
       ('rev_yaw', 'reversal', [(1.5, _axis(0, 0, 0.4)), (1.5, _axis(0, 0, -0.4))]),
       ('rev_sway', 'reversal', [(2.0, _axis(0, 0.8, 0)), (2.0, _axis(0, -0.8, 0))]),
       ('coast_surge', 'coast', [(4.0, _axis(0.9, 0, 0))]),
       ('turn+', 'turn', [(3.0, _axis(0.6, 0, 0.3))]), ('turn-', 'turn', [(3.0, _axis(0.6, 0, -0.3))])]
)
assert len(PROTOCOL) == 24


def schedule(phases, t):
    """q [T, 4] at times t for a segment's phases (neutral before/after)."""
    q = np.zeros((len(t), N_THR))
    t0 = BASELINE_S
    for dur, v in phases:
        q[(t >= t0 - 1e-9) & (t < t0 + dur - 1e-9)] = v
        t0 += dur
    return q


def probe_seconds(protocol=PROTOCOL):
    return len(protocol) * SEG_S


def generate(spec_row, seed, protocol=PROTOCOL, session='probe', tier='synthetic'):
    """Evidence (24 Segments) of one plant (a 1-row PlantSpec) under the probe protocol, plus the
    raw tick log (truth kept out of the Segments)."""
    n = len(protocol)
    spec = spec_row.repeat(n)
    seeds = (np.uint64(seed) * np.uint64(1000003) + np.arange(n, dtype=np.uint64))
    plant = Plant(spec, seeds, t_max=SEG_S)
    ticks = int(round(SEG_S / DT))
    tt = np.arange(ticks) * DT
    Q = np.stack([schedule(ph, tt) for _, _, ph in protocol], 1)       # [ticks, n, 4]
    gps_every, head_every, gyro_every = (int(round(x / DT)) for x in (GPS_DT, HEAD_DT, GYRO_DT))
    tg, P, th, H, tr, R = [], [], [], [], [], []
    truth = np.empty((ticks, n, 6))
    for k in range(ticks):
        plant.step(Q[k])
        truth[k] = plant.st
        pose, vel, gyro = plant.measure()
        if plant.tick % gps_every == 0:
            tg.append(plant.t)
            P.append(pose[:, :2])
        if plant.tick % head_every == 0:
            th.append(plant.t)
            H.append(pose[:, 2])
        if plant.tick % gyro_every == 0:
            tr.append(plant.t)
            R.append(gyro)
    tg, P, th, H, tr, R = map(np.asarray, (tg, P, th, H, tr, R))
    grid = np.arange(1.0, SEG_S - 1.0, GRID_DT)
    segs = []
    for i, (name, group, phases) in enumerate(protocol):
        Pi = P[:, i]
        pos = np.stack([np.interp(grid, tg, Pi[:, 0]), np.interp(grid, tg, Pi[:, 1])], -1)
        psi = np.interp(grid, th, np.unwrap(H[:, i]))
        r = np.interp(grid, tr, np.convolve(R[:, i], np.ones(5) / 5, 'same'))
        vw = sg_velocity(tg, Pi, grid)
        c, s = np.cos(psi), np.sin(psi)
        X = np.stack([pos[:, 0], pos[:, 1], psi, c * vw[:, 0] + s * vw[:, 1], -s * vw[:, 0] + c * vw[:, 1], r], -1)
        U = schedule(phases, grid)
        bw = (grid >= 6.0) & (grid < BASELINE_S - 0.2)          # after the boat has settled into the current
        k0, k1 = np.flatnonzero(bw)[[0, -1]]
        cur = tuple(float(x) for x in (pos[k1] - pos[k0]) / (grid[k1] - grid[k0]))
        pulse_end = BASELINE_S + sum(d for d, _ in phases)
        t0 = grid[0]
        h = hashlib.sha256(np.ascontiguousarray(X).tobytes() + np.ascontiguousarray(U).tobytes()).hexdigest()
        segs.append(Segment(
            session=f'{session}:{group}', tier=tier, t=grid - t0, x=X, u=U, u_kind='pwm_q',
            quality={'synthetic': True}, labels=[group, name],
            meta={'current': cur, 'current_sd': float(np.sqrt(vw[bw].var(0).mean())),
                  'fit_window': (BASELINE_S - 1.0 - t0, min(grid[-1], pulse_end + 8.0) - t0),
                  'pulse_s': pulse_end - BASELINE_S, 'flags': ['current_from_drift', 'synthetic'],
                  'gps_t': tg - t0, 'gyro_t': tr - t0},
            name=name, source_sha=h, t_event=BASELINE_S - t0))
    ev = Evidence(segs, {'kind': 'decisioncal.probes', 'seed': int(seed), 'n_segments': n,
                         'probe_seconds': probe_seconds(protocol),
                         'sha256': hashlib.sha256(''.join(s.source_sha for s in segs).encode()).hexdigest()})
    return ev, {'truth': truth, 'Q': Q, 't': tt}
