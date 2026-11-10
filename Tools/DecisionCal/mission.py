"""Gate -> U-turn -> dock mission, batched over envs with one controller per env.

Route (world ENU, metres): start at (0, 0) heading east; a gate of two buoys at (10, +-1.0); a
right U-turn of radius 2.5 m starting at x = 15; back west along y = -5 into a 1.2 m slip (walls
y = -5 +- 0.6 for x in [2, 5], end wall at x = 2). The boat's footprint is 1.0 x 0.7 m, so the
slip leaves 0.25 m a side. Docked = centre within 0.25 m of the slip axis, 2.4 <= x <= 3.0, heading
within 15 deg of west and speed below 0.08 m/s, held for 2 s (the docking time is when the hold
began). Walls and buoys do not push the boat (no contact physics); every contact is counted and
penalised. Time limit 120 s.

Score (pre-registered 2026-11-09, lower is better):
    J = t_dock (or 120) + 60 (1 - docked) + 20 min(contacts, 3) + 30 gate_miss + 10 min(pose_err, 3)

(contact events are reported raw; the score caps them at 3 so a boat grinding along a wall does
not dominate every mean).
"""

import numpy as np

import _paths  # noqa: F401
from law import BatchedLaw, mix
from library import LIBRARY
from plant import DT, Plant

T_MAX = 120.0
CONTROL_EVERY = 2                      # ticks: 25 Hz control and odometry
DS = 0.05
TARGET = np.array([2.6, -5.0, np.pi])
GATE_X, GATE_Y, BUOY_R = 10.0, 1.0, 0.15
SLIP = {'x0': 2.0, 'x1': 5.0, 'y': -5.0, 'half': 0.6, 'wall': 0.1}
FOOT = np.array([[0.5, 0.35], [0.5, -0.35], [-0.5, -0.35], [-0.5, 0.35],
                 [0.5, 0.0], [0.0, -0.35], [-0.5, 0.0], [0.0, 0.35]])
J_WEIGHTS = {'fail': 60.0, 'contact': 20.0, 'gate_miss': 30.0, 'pose': 10.0}


def route():
    """Path points [P, 2], tangent heading [P], curvature [P], arclength [P]."""
    R, x_turn = 2.5, 15.0
    a = np.arange(0.0, x_turn, DS)
    pa = np.stack([a, np.zeros_like(a)], -1)
    ha, ka = np.zeros_like(a), np.zeros_like(a)
    phi = np.arange(0.0, np.pi, DS / R)
    pb = np.stack([x_turn + R * np.sin(phi), -R + R * np.cos(phi)], -1)
    hb, kb = -phi, np.full_like(phi, -1.0 / R)
    c = np.arange(x_turn, TARGET[0] - 1e-9, -DS)
    pc = np.stack([c, np.full_like(c, -2 * R)], -1)
    hc, kc = np.full_like(c, np.pi), np.zeros_like(c)
    P = np.concatenate([pa, pb, pc, TARGET[None, :2]])
    H = np.concatenate([ha, hb, hc, [np.pi]])
    K = np.concatenate([ka, kb, kc, [0.0]])
    s = np.concatenate([[0.0], np.cumsum(np.hypot(*np.diff(P, axis=0).T))])
    return P, H, K, s


PATH, HEAD, CURV, ARC = route()
S_END = ARC[-1]


def speed_profiles(lib=LIBRARY):
    """[C, P] speed limit along the path for every library member."""
    out = np.empty((len(lib), len(ARC)))
    d_end = S_END - ARC
    for i, c in enumerate(lib):
        v = np.full(len(ARC), c.cruise)
        with np.errstate(divide='ignore'):
            v = np.minimum(v, np.sqrt(c.turn_alat / np.maximum(np.abs(CURV), 1e-9)))
        last = d_end < 4.0
        if c.dock == 'const':
            v[last] = np.minimum(v[last], 0.3)
        elif c.dock == 'sqrt':
            v[last] = np.minimum(v[last], np.sqrt(2 * 0.15 * d_end[last]) + 0.03)
        elif c.dock.startswith('sqrt@'):                 # v2: sqrt profile assuming a = float after '@'
            v[last] = np.minimum(v[last], np.sqrt(2 * float(c.dock[5:]) * d_end[last]) + 0.03)
        elif c.dock == 'two_stage':
            v[last] = np.minimum(v[last], np.where(d_end[last] > 1.5, 0.4, 0.1))
        elif c.dock == 'creep':
            v[last] = np.minimum(v[last], 0.08)
        v = np.minimum(v, np.sqrt(2 * c.brake * d_end) + 0.02)
        for k in range(len(v) - 2, -1, -1):              # decelerate in time for every limit
            v[k] = min(v[k], np.sqrt(v[k + 1] ** 2 + 2 * c.brake * (ARC[k + 1] - ARC[k])))
        out[i] = v
    return out


PROFILES = speed_profiles()
_PROFILE_CACHE = {}


def profiles_for(lib):
    """Speed profiles of ``lib``; the default library reuses ``PROFILES`` (same object as before)."""
    if lib is LIBRARY:
        return PROFILES
    key = tuple(lib)
    if key not in _PROFILE_CACHE:
        _PROFILE_CACHE[key] = speed_profiles(lib)
    return _PROFILE_CACHE[key]


def _wrap(a):
    return (a + np.pi) % (2 * np.pi) - np.pi


def _contacts(pose):
    """Footprint points inside a wall rectangle or a buoy: bool [N]."""
    c, s = np.cos(pose[:, 2]), np.sin(pose[:, 2])
    px = pose[:, None, 0] + c[:, None] * FOOT[None, :, 0] - s[:, None] * FOOT[None, :, 1]
    py = pose[:, None, 1] + s[:, None] * FOOT[None, :, 0] + c[:, None] * FOOT[None, :, 1]
    w, y0, h = SLIP['wall'], SLIP['y'], SLIP['half']
    in_x = (px >= SLIP['x0']) & (px <= SLIP['x1'])
    hit = in_x & (((py >= y0 + h) & (py <= y0 + h + w)) | ((py <= y0 - h) & (py >= y0 - h - w)))
    hit |= (px <= SLIP['x0']) & (px >= SLIP['x0'] - w) & (py >= y0 - h - w) & (py <= y0 + h + w)
    for by in (GATE_Y, -GATE_Y):
        hit |= np.hypot(px - GATE_X, py - by) <= BUOY_R
    return hit.any(1)


def score(m):
    """J per env from a metrics dict (arrays)."""
    W = J_WEIGHTS
    t = np.where(m['docked'], m['t_dock'], T_MAX)
    return (t + W['fail'] * (1 - m['docked']) + W['contact'] * np.minimum(m['contacts'], 3.0) + W['gate_miss'] * m['gate_miss']
            + W['pose'] * np.minimum(m['pose_err'], 3.0))


def run(spec, ctrl_idx, seeds, t_max=T_MAX, lib=LIBRARY, log_every=0):
    """Fly every env's controller on its plant. Returns metrics (dict of [N] arrays) and, when
    ``log_every`` > 0, a stream dict sampled every that many control steps."""
    n = spec.n
    ctrl_idx = np.asarray(ctrl_idx, int)
    plant = Plant(spec, seeds, t_max=t_max)
    cs = [lib[i] for i in ctrl_idx]
    ff = np.array([c.feedforward for c in cs])
    lc = np.array([c.latcomp for c in cs])
    # v1 specs have no gain fields: kp x1.5 when feed-forward is off. v2 specs carry kp_scale / ki_scale.
    kp_s = np.array([getattr(c, 'kp_scale', 1.0 if c.feedforward else 1.5) for c in cs])
    ki_s = np.array([getattr(c, 'ki_scale', 1.0) for c in cs])
    kp = kp_s[:, None] * np.array([0.8, 0.8, 0.6])
    ki = ki_s[:, None] * np.array([0.6, 0.6, 0.4])
    law = BatchedLaw(n, kp=kp, ki=ki)
    prof = profiles_for(lib)[ctrl_idx]
    idx = np.zeros(n, int)
    ar = np.arange(n)
    win = np.arange(0, 80)
    P = len(ARC)
    docked = np.zeros(n, bool)
    t_dock = np.full(n, np.nan)
    hold_since = np.full(n, np.nan)
    contacts = np.zeros(n, int)
    in_contact = np.zeros(n, bool)
    gate_y = np.full(n, np.nan)
    recovery = np.zeros(n)
    energy = np.zeros(n)
    q = np.zeros((n, 4))
    streams = {k: [] for k in ('t', 'truth', 'obs', 'q_req', 'q_applied')} if log_every else None
    steps = int(round(t_max / DT))
    for k in range(steps):
        if k % CONTROL_EVERY == 0:
            now = plant.t
            pose, vel, _ = plant.measure()
            c, s = np.cos(pose[:, 2]), np.sin(pose[:, 2])
            x = pose[:, 0] + (c * vel[:, 0] - s * vel[:, 1]) * lc
            y = pose[:, 1] + (s * vel[:, 0] + c * vel[:, 1]) * lc
            psi = pose[:, 2] + vel[:, 2] * lc
            cand = np.minimum(idx[:, None] + win[None, :], P - 1)
            d2 = (PATH[cand, 0] - x[:, None]) ** 2 + (PATH[cand, 1] - y[:, None]) ** 2
            idx = cand[ar, np.argmin(d2, 1)]
            h = HEAD[idx]
            th = np.stack([np.cos(h), np.sin(h)], -1)
            nh = np.stack([-np.sin(h), np.cos(h)], -1)
            e_ct = (x - PATH[idx, 0]) * nh[:, 0] + (y - PATH[idx, 1]) * nh[:, 1]
            recovery += (np.abs(e_ct) > 1.0) * DT * CONTROL_EVERY * ~docked
            vs = prof[ar, np.minimum(idx + int(0.3 / DS), P - 1)]
            vw = vs[:, None] * th - np.clip(0.8 * e_ct, -0.4, 0.4)[:, None] * nh
            d_end = np.hypot(TARGET[0] - x, TARGET[1] - y)
            near = (d_end < 0.4) & (idx > P - 200)
            hold = np.clip(0.5 * (TARGET[:2] - np.stack([x, y], -1)), -0.15, 0.15)
            vw = np.where(near[:, None], hold, vw)
            r_des = np.clip(CURV[idx] * vs + 1.2 * _wrap(h - psi), -0.8, 0.8)
            r_des = np.where(near, np.clip(1.2 * _wrap(TARGET[2] - psi), -0.4, 0.4), r_des)
            c2, s2 = np.cos(psi), np.sin(psi)
            desired = np.stack([c2 * vw[:, 0] + s2 * vw[:, 1], -s2 * vw[:, 0] + c2 * vw[:, 1], r_des], -1)
            law.observe(vel, psi, now)
            effort = law.step(desired, now, feedforward=ff)
            q = mix(effort)
            q = np.where(docked[:, None], 0.0, q)
            if log_every and (k // CONTROL_EVERY) % log_every == 0:
                streams['t'].append(now)
                streams['truth'].append(plant.st.copy())
                streams['obs'].append(np.concatenate([pose, vel], 1))
                streams['q_req'].append(q.copy())
                streams['q_applied'].append(plant.q_applied.copy())
        x_prev = plant.st[:, 0].copy()
        plant.step(q)
        st = plant.st
        energy += np.abs(plant.thr.T).sum(1) * DT * ~docked
        cross = np.isnan(gate_y) & (x_prev < GATE_X) & (st[:, 0] >= GATE_X)
        gate_y = np.where(cross, st[:, 1], gate_y)
        hit = _contacts(st) & ~docked
        contacts += hit & ~in_contact
        in_contact = hit
        speed = np.hypot(st[:, 3], st[:, 4])
        ok = ((np.abs(st[:, 1] - TARGET[1]) <= 0.25) & (st[:, 0] >= 2.4) & (st[:, 0] <= 3.0)
              & (np.abs(_wrap(st[:, 2] - TARGET[2])) <= np.radians(15)) & (speed <= 0.08) & ~docked)
        hold_since = np.where(ok, np.where(np.isnan(hold_since), plant.t, hold_since), np.nan)
        newly = ok & (plant.t - hold_since >= 2.0 - 1e-9)
        t_dock = np.where(newly, hold_since, t_dock)
        docked |= newly
        if docked.all():
            break
    st = plant.st
    pose_err = np.hypot(st[:, 0] - TARGET[0], st[:, 1] - TARGET[1])
    m = {'docked': docked.astype(float), 't_dock': np.where(docked, t_dock, np.nan), 'contacts': contacts.astype(float),
         'gate_miss': (np.isnan(gate_y) | (np.abs(np.nan_to_num(gate_y, nan=9.9)) > GATE_Y)).astype(float),
         'pose_err': np.where(docked, 0.0, pose_err), 'recovery_s': recovery, 'energy_Ns': energy,
         'progress': ARC[idx] / S_END}
    m['J'] = score(m)
    if streams is not None:
        streams = {k: np.asarray(v) for k, v in streams.items()}
    return m, streams
