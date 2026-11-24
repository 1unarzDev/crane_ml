# SPDX-License-Identifier: GPL-3.0-or-later
"""The braking scenario: approach at v0, command a stop, measure how far, how long and how clean. Batched.

    from domains.drones import braking
    m = braking.stops('x500', v0=8.0, seeds=range(1000))            # {metric: [E]}, numba on every core (else numpy)
    m = braking.stops('x500', 8.0, range(64), variant={'mass': 2.4, 'tau_up': 0.02}, backend='numpy')
    python3 -m domains.drones.braking --card x500 --v0 4,8,12 --seeds 0:1000    # a table, and stops/s

Each env: level flight at ``altitude_m``, velocity v0 along a heading drawn from its seed, rotors at hover; the
cascaded controller (engine/kernels/aerial_control.py) holds v0 for ``approach_s``, then the stop command
(``stop: brake``: velocity setpoint 0; ``hold``: position hold at the command point), then ``after_s`` more.
The plant is engine/kernels/aerial.py's model.step on the card (with a card variant applied). Every number of
the scenario is config/domains/drones.yaml ``braking`` and ``control``; ``config=`` overrides them.

Metrics, from the stop command on (along-track s is the travel along the approach heading from the command
point): stop_distance_m (peak s), stop_time_s (first time the ground speed is below stopped_mps; NaN when it
never is), overshoot_m (peak s less the final s: the rebound), lateral_m (peak |cross-track|), alt_loss_m (peak
drop below the altitude), max_tilt_deg, saturated_s (time the collective was cut to keep the attitude),
v_cmd_mps (ground speed at the command), final_speed_mps; outcome STOPPED | NOT_STOPPED | CRASHED (below the
ground).
"""

import functools
import math
import sys
import time
from pathlib import Path

import numpy as np

from domains.drones import card as cards
from engine.kernels import aerial as A, aerial_control as C

ROOT = Path(__file__).resolve().parents[2]
CONFIG = ROOT / 'config' / 'domains' / 'drones.yaml'
METRICS = ['stop_distance_m', 'stop_time_s', 'overshoot_m', 'lateral_m', 'alt_loss_m', 'max_tilt_deg',
           'saturated_s', 'v_cmd_mps', 'final_speed_mps', 'crashed']
OUTCOMES = ('STOPPED', 'NOT_STOPPED', 'CRASHED')
SCENARIO = ['dt_s', 'altitude_m', 'approach_s', 'after_s', 'stop', 'stopped_mps', 'wind_sd_mps']


@functools.lru_cache(maxsize=2)
def _domain(mtime_ns):
    from crane import miniyaml   # PyYAML when installed, else crane's subset reader (the same result)
    return miniyaml.safe_load(CONFIG.read_text())


def config(over=None):
    """The domain's ``braking`` and ``control`` sections, with ``over`` ({braking: {...}, control: {...}})
    merged in."""
    doc = _domain(CONFIG.stat().st_mtime_ns)
    out = {'braking': dict(doc['braking']), 'control': dict(doc['control'])}
    for part in ('braking', 'control'):
        out[part].update((over or {}).get(part) or {})
    missing = [k for k in SCENARIO if k not in out['braking']]
    if missing:
        raise KeyError(f'{CONFIG} braking: missing {", ".join(missing)}')
    return out


def draws(seeds, wind_sd):
    """Per seed: heading (rad) and a steady horizontal wind (m/s), from the seed's own stream."""
    from fastsim.identity import derive_seed
    yaw, wind = np.empty(len(seeds)), np.zeros((len(seeds), 3))
    for k, s in enumerate(seeds):
        rng = np.random.default_rng(derive_seed(int(s), 'drones', 'braking'))
        yaw[k] = rng.uniform(-math.pi, math.pi)
        wind[k, :2] = rng.normal(0.0, 1.0, 2) * wind_sd
    return yaw, wind


def _scen(b):
    """The scenario row: dt, steps before the command, steps in all, altitude, stop mode, stopped speed."""
    dt = float(b['dt_s'])
    n_cmd, n_all = int(round(float(b['approach_s']) / dt)), int(round((float(b['approach_s']) + float(b['after_s'])) / dt))
    mode = {'brake': 0.0, 'hold': 1.0}[b['stop']]
    return np.array([dt, n_cmd, n_all, float(b['altitude_m']), mode, float(b['stopped_mps'])])


def _start(p, v0, yaw, alt):
    v0 = np.broadcast_to(np.asarray(v0, dtype=float), yaw.shape)
    vel = np.stack([v0 * np.cos(yaw), v0 * np.sin(yaw), 0.0 * yaw], axis=-1)
    return A.init(p, pos=(0.0, 0.0, alt), vel=vel, yaw=yaw, rotor_speed=A.hover_speed(p))


# ---- numpy (the float64 reference; lockstep) ------------------------------------------------------------------

def run_numpy(p, c, v0, yaw, wind, scen):
    dt, n_cmd, n_all, alt, mode, stopped = scen
    n_cmd, n_all = int(n_cmd), int(n_all)
    E = p.shape[0]
    x = _start(p, v0, yaw, alt)
    v0 = np.broadcast_to(np.asarray(v0, dtype=float), (E,))
    d = np.stack([np.cos(yaw), np.sin(yaw)], axis=-1)
    sp = np.zeros((E, C.SP))
    sp[:, 2], sp[:, 5] = alt, yaw
    m = {k: np.zeros(E) for k in METRICS}
    m['stop_time_s'][:] = np.nan
    peak = np.full(E, -np.inf)
    cmd = np.zeros((E, 2))
    s = np.zeros(E)
    for k in range(n_all):
        if k < n_cmd:
            sp[:, 3], sp[:, 4], sp[:, 6] = v0 * d[:, 0], v0 * d[:, 1], 0.0
        elif k == n_cmd:
            cmd = x[:, :2].copy()
            sp[:, 0], sp[:, 1], sp[:, 3], sp[:, 4], sp[:, 6] = cmd[:, 0], cmd[:, 1], 0.0, 0.0, mode
            m['v_cmd_mps'] = np.hypot(x[:, 3], x[:, 4])
        u, sat = C.control(x, sp, p, c)
        x = A.step(x, u, p, dt, wind)
        if k >= n_cmd:
            t = (k + 1 - n_cmd) * dt
            rel = x[:, :2] - cmd
            s = rel[:, 0] * d[:, 0] + rel[:, 1] * d[:, 1]
            peak = np.maximum(peak, s)
            m['lateral_m'] = np.maximum(m['lateral_m'], np.abs(-rel[:, 0] * d[:, 1] + rel[:, 1] * d[:, 0]))
            m['alt_loss_m'] = np.maximum(m['alt_loss_m'], alt - x[:, 2])
            q1, q2 = x[:, A.QUAT + 1], x[:, A.QUAT + 2]
            tilt = np.degrees(np.arccos(np.clip(1.0 - 2.0 * (q1 * q1 + q2 * q2), -1.0, 1.0)))
            m['max_tilt_deg'] = np.maximum(m['max_tilt_deg'], tilt)
            m['saturated_s'] = m['saturated_s'] + sat * dt
            speed = np.hypot(x[:, 3], x[:, 4])
            m['stop_time_s'] = np.where(np.isnan(m['stop_time_s']) & (speed < stopped), t, m['stop_time_s'])
        m['crashed'] = np.maximum(m['crashed'], (x[:, 2] < 0.0) * 1.0)
    m['stop_distance_m'] = peak
    m['overshoot_m'] = peak - s
    m['final_speed_mps'] = np.hypot(x[:, 3], x[:, 4])
    return m


# ---- numba (every core; one env per call) ---------------------------------------------------------------------

try:
    import numba
except ImportError:
    numba = None

if numba is not None:
    @numba.njit(cache=True)
    def nb_one(p, c, v0, yaw, wind, scen, x, out):
        """One env's whole stop, in place on its row x; the metrics (METRICS order) into out."""
        dt, n_cmd, n_all, alt, mode, stopped = scen[0], int(scen[1]), int(scen[2]), scen[3], scen[4], scen[5]
        R = x.shape[0] - A.BODY
        d0, d1 = math.cos(yaw), math.sin(yaw)
        sp = np.zeros(C.SP)
        sp[2], sp[5] = alt, yaw
        u = np.empty(R)
        peak, s, cx, cy = -np.inf, 0.0, 0.0, 0.0
        lat = loss = tilt = satd = crashed = vcmd = 0.0
        tstop = np.nan
        for k in range(n_all):
            if k < n_cmd:
                sp[3], sp[4], sp[6] = v0 * d0, v0 * d1, 0.0
            elif k == n_cmd:
                cx, cy = x[0], x[1]
                sp[0], sp[1], sp[3], sp[4], sp[6] = cx, cy, 0.0, 0.0, mode
                vcmd = math.hypot(x[3], x[4])
            sat = C.nb_control(x, sp, p, c, u)
            A.nb_step(x, u, p, dt, wind)
            if k >= n_cmd:
                t = (k + 1 - n_cmd) * dt
                r0, r1 = x[0] - cx, x[1] - cy
                s = r0 * d0 + r1 * d1
                peak = max(peak, s)
                lat = max(lat, abs(-r0 * d1 + r1 * d0))
                loss = max(loss, alt - x[2])
                q1, q2 = x[A.QUAT + 1], x[A.QUAT + 2]
                tilt = max(tilt, math.degrees(math.acos(min(max(1.0 - 2.0 * (q1 * q1 + q2 * q2), -1.0), 1.0))))
                satd = satd + sat * dt
                if np.isnan(tstop) and math.hypot(x[3], x[4]) < stopped:
                    tstop = t
            if x[2] < 0.0:
                crashed = 1.0
        out[0], out[1], out[2], out[3], out[4] = peak, tstop, peak - s, lat, loss
        out[5], out[6], out[7], out[8], out[9] = tilt, satd, vcmd, math.hypot(x[3], x[4]), crashed

    @numba.njit(cache=True, parallel=True)
    def nb_many(p, c, v0, yaw, wind, scen, x, out):
        for e in numba.prange(p.shape[0]):
            nb_one(p[e], c[e], v0[e], yaw[e], wind[e], scen, x[e], out[e])


def run_numba(p, c, v0, yaw, wind, scen):
    if numba is None:
        raise RuntimeError('the numba backend needs numba (pip install numba); fix: backend numpy')
    E = p.shape[0]
    x = _start(p, v0, yaw, scen[3])
    out = np.empty((E, len(METRICS)))
    nb_many(np.ascontiguousarray(p), np.ascontiguousarray(c),
            np.ascontiguousarray(np.broadcast_to(np.asarray(v0, dtype=float), (E,))),
            np.ascontiguousarray(yaw), np.ascontiguousarray(wind), scen, x, out)
    return {k: out[:, i] for i, k in enumerate(METRICS)}


BACKENDS = {'numpy': run_numpy, 'numba': run_numba}
DEFAULT = 'numba' if numba is not None else 'numpy'


def rows(card, variant, E, gains):
    """[E, P] and [E, C] for one card (ref or dict) with a card variant ({parameter: value})."""
    doc = cards.override(card, variant) if variant else cards.read(card)
    p = A.pack(cards.vehicle(doc))
    c = C.pack(p, gains)
    return np.repeat(p[None], E, axis=0), np.repeat(c[None], E, axis=0), cards.sha256(doc)


def outcome(m):
    return np.where(m['crashed'] > 0, 'CRASHED', np.where(np.isnan(m['stop_time_s']), 'NOT_STOPPED', 'STOPPED'))


def stops(card, v0, seeds, variant=None, config_over=None, backend=None):
    """{metric: [E]} (plus 'outcome', 'seed', 'card_sha') for the stops of ``seeds`` at speed v0 (m/s)."""
    cfg = config(config_over)
    seeds = [int(s) for s in seeds]
    p, c, sha = rows(card, variant, len(seeds), cfg['control'])
    yaw, wind = draws(seeds, float(cfg['braking']['wind_sd_mps']))
    m = BACKENDS[backend or DEFAULT](p, c, np.full(len(seeds), float(v0)), yaw, wind, _scen(cfg['braking']))
    return {**m, 'outcome': outcome(m), 'seed': np.array(seeds), 'card_sha': sha}


# ---- the study task: one stop per unit, computed a block of seeds at a time ---------------------------------------

_MEMO = {}


def stop(args, seed, cell=None):
    """crane.study task ``domains.drones.braking:stop``: args {card, v0, variant: {parameter: value}, backend,
    braking: {...}, control: {...}}. The first seed of an aligned block of ``braking.batch`` seeds runs the
    whole block in one batched call; the others are read from it (pair on the seed: crn: seed)."""
    import json
    from crane.ids import digest
    args = dict(args or {})
    over = {k: args.get(k) for k in ('braking', 'control') if args.get(k)}
    cfg = config(over)
    size = int(cfg['braking'].get('batch', 128))
    first = int(seed) - int(seed) % size
    key = digest([json.loads(json.dumps(args, sort_keys=True, default=str)), first])
    if key not in _MEMO:
        if len(_MEMO) > 64:
            _MEMO.clear()
        _MEMO[key] = stops(args.get('card'), float(args['v0']), range(first, first + size),
                           args.get('variant'), over, args.get('backend'))
    m = _MEMO[key]
    i = int(seed) - first
    got = {k: float(m[k][i]) for k in METRICS if math.isfinite(float(m[k][i]))}
    return {**got, 'stopped': float(m['outcome'][i] == 'STOPPED'), 'outcome': str(m['outcome'][i])}


# ---- python3 -m domains.drones.braking ------------------------------------------------------------------------

def main(argv=None):
    import argparse
    import json
    from crane import select
    ap = argparse.ArgumentParser(prog='python3 -m domains.drones.braking', description=__doc__.split('\n')[0])
    ap.add_argument('--card', default=None, help='an aerial card: a name in domains/drones/profiles or a path '
                    '(default: config/domains/drones.yaml card.default)')
    ap.add_argument('--v0', default='4,8,12', help='approach speeds, m/s (comma list)')
    ap.add_argument('--seeds', default='0:1000', help='N, A:B, a,b,c or @file')
    ap.add_argument('--set', action='append', default=[], metavar='K=V',
                    help='a card variant (mass=2.4) or braking.K=V / control.K=V')
    ap.add_argument('--backend', default=DEFAULT, choices=sorted(BACKENDS))
    ap.add_argument('--json', action='store_true')
    a = ap.parse_args(argv)
    try:
        seeds = select.parse_seeds(a.seeds)
        sets = select.parse_sets(a.set)
        v0s = [float(v) for v in a.v0.split(',')]
        a.card = a.card or cards.default()
        cards.path_of(a.card)
    except (select.SelectError, ValueError, FileNotFoundError) as error:
        print(f'braking: {error}; fix: python3 -m domains.drones.braking --help', file=sys.stderr)
        return 2
    over = {k: sets.pop(k) for k in ('braking', 'control') if k in sets}
    rows_out, total, wall = [], 0, 0.0
    if a.backend == 'numba':      # compile (or load the cache) outside the timing
        stops(a.card, v0s[0], seeds[:1], sets or None, over, 'numba')
    for v0 in v0s:
        t0 = time.perf_counter()
        m = stops(a.card, v0, seeds, sets or None, over, a.backend)
        dt = time.perf_counter() - t0
        wall, total = wall + dt, total + len(seeds)
        ok = m['outcome'] == 'STOPPED'
        rows_out.append({'v0_mps': v0, 'n': len(seeds), 'stopped': int(ok.sum()),
                         'crashed': int((m['outcome'] == 'CRASHED').sum()),
                         'stop_distance_m': float(np.mean(m['stop_distance_m'])),
                         'stop_distance_p95_m': float(np.percentile(m['stop_distance_m'], 95)),
                         'stop_time_s': float(np.nanmean(m['stop_time_s'])) if ok.any() else None,
                         'overshoot_m': float(np.mean(m['overshoot_m'])), 'alt_loss_m': float(np.mean(m['alt_loss_m'])),
                         'max_tilt_deg': float(np.mean(m['max_tilt_deg'])), 'wall_s': round(dt, 3)})
    rate = total / wall if wall > 0 else 0.0
    if a.json:
        print(json.dumps({'schema': 'crane.drones.braking/1', 'card': a.card, 'backend': a.backend, 'variant': sets,
                          'rows': rows_out, 'stops_per_s': round(rate, 1)}))
        return 0
    from crane.output import table
    head = ('v0', 'n', 'stopped', 'crashed', 'distance', 'p95', 'time', 'overshoot', 'alt loss', 'tilt')
    body = [(f"{r['v0_mps']:g} m/s", r['n'], r['stopped'], r['crashed'], f"{r['stop_distance_m']:.2f} m",
             f"{r['stop_distance_p95_m']:.2f} m", '-' if r['stop_time_s'] is None else f"{r['stop_time_s']:.2f} s",
             f"{r['overshoot_m']:.3f} m", f"{r['alt_loss_m']:.3f} m", f"{r['max_tilt_deg']:.1f} deg") for r in rows_out]
    for line in table([head, *body]):
        print(line)
    print(f'{total} stops on {a.backend} in {wall:.2f} s: {rate:,.0f} stops/s')
    print('next: crane study run examples/studies/braking.yaml')
    return 0


if __name__ == '__main__':
    sys.exit(main())
