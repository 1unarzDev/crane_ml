#!/usr/bin/env python3
"""Closed-loop parity: one mission (mission.run, unedited) with full per-tick logging, on the Unity player
over the lockstep link or on the NumPy plant, under environment / actuator interventions (PARITY.md, task D).

Per tick (both layers): tick, harness t, true state (course frame, body velocity), q sent/applied (the
command the plant was given this tick), and at control ticks the controller's observation (pose, body
velocity) and requested q (mission.run's own streams with log_every = 1). NumPy also logs thrust T and the
body wrench (Unity does not report them).

    # Unity (admission first; <= 60 s wall):
    python3 ~/astro_dock/src/mhseals_agents/tools/heavy probe unity && \
    nice -n 10 python3 Tools/DecisionCal/parity/closedloop.py unity --boat transient --ctrl 0 --seed 1 --env scenario --out L
    # NumPy representable boat (same env), snap on / off:
    nice -n 19 python3 -I Tools/DecisionCal/parity/closedloop.py numpy --boat transient --ctrl 0 --seed 1 --env scenario --snap off --out L
env: scenario (the boat's mean current + wind), nowind, nocurrent, none.  --deadband0: Unity faults deadband 0.
--projection hidden (NumPy only) flies the full hidden boat instead of the representable one.
"""

import argparse
import json
import os
import shlex
import signal
import subprocess
import sys
import threading
import time
from pathlib import Path
from types import SimpleNamespace

import numpy as np

DC = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(DC))
sys.path.insert(0, str(Path(__file__).resolve().parent))

import _paths  # noqa: E402,F401
import mission  # noqa: E402
import plant  # noqa: E402
import unity_check as U  # noqa: E402
from plant import DT  # noqa: E402
from _launch import RACE_EXIT, Player  # noqa: E402

OUT_ROOT = Path.home() / '.cache' / 'decisioncal' / 'parity' / 'closed'
BOATS = {'transient': ('transient', 0), 'steady': ('steady', 0), 'mixed': ('mixed', 0), 'timing': ('timing', 0)}


def env_scenario(spec, env):
    _, _, sc, _ = U.unity_config(spec, 0)
    if env in ('nowind', 'none'):
        sc.update(wind_speed=0.0, wind_toward_deg=0.0)
    if env in ('nocurrent', 'none'):
        sc.update(cur_speed=0.0, cur_toward_deg=0.0)
    return sc


def env_spec(spec, env):
    z = np.zeros((spec.n, 2))
    if env in ('nowind', 'none'):
        spec = spec.with_(wind=z.copy(), gust_sigma=np.zeros(spec.n))
    if env in ('nocurrent', 'none'):
        spec = spec.with_(current=z.copy(), ou_sigma=np.zeros(spec.n), spatial_amp=np.zeros(spec.n))
    return spec


class Log:
    def __init__(self):
        self.rows = []         # tick-level: [tick, t, x y psi u v r, q0..q3, T0..T3, X Y N]

    def add(self, tick, t, st, q, T=None, w=None):
        self.rows.append([tick, t, *st, *q, *(T if T is not None else [np.nan] * 4), *(w if w is not None else [np.nan] * 3)])


def numpy_run(spec, ctrl, seed, t_max, lib=None):
    log = Log()

    class RecPlant(plant.Plant):
        def __init__(self, *a, **kw):
            super().__init__(*a, **kw)
            log.add(0, self.t, self.st[0], np.zeros(4))

        def step(self, q):
            q = np.clip(np.nan_to_num(np.asarray(q, float)), -1.0, 1.0)
            log.rows[-1][8:12] = q[0]                      # the command given at this tick
            super().step(q)
            log.add(self.tick, self.t, self.st[0], np.full(4, np.nan), self.thr.T[0], self.wrench[0])

    orig = mission.Plant
    mission.Plant = RecPlant
    try:
        m, streams = mission.run(spec, [ctrl], [seed], t_max=t_max, log_every=1, **({'lib': lib} if lib else {}))
    finally:
        mission.Plant = orig
    return m, streams, log


def unity_run(link, spec, ctrl, seed, t_max, deadline, log):

    class RecUnity(U.UnityPlant):
        def __init__(self, *a, **kw):
            self._ticks = []
            super().__init__(*a, **kw)
            log.add(self._ticks[-1][0], self.t, self.st[0], np.zeros(4))

        def _receive(self, first=False):
            super()._receive(first)
            self._ticks.append((self.tick0 + self.tick,))

        def step(self, q):
            qq = np.clip(np.nan_to_num(np.asarray(q, float)), -1.0, 1.0)
            log.rows[-1][8:12] = qq[0]
            super().step(q)
            log.add(self._ticks[-1][0], self.t, self.st[0], np.full(4, np.nan))

    holder = {}

    def factory(sp, seeds, t_max=t_max, **_):
        holder['p'] = RecUnity(link, sp, seeds, t_max=t_max, deadline=deadline)
        return holder['p']

    orig = mission.Plant
    mission.Plant = factory
    try:
        m, streams = mission.run(spec, [ctrl], [seed], t_max=t_max, log_every=1)
    finally:
        mission.Plant = orig
        if 'p' in holder:
            holder['p'].close()
    return m, streams, log


def save(out, m, streams, log, meta):
    out.mkdir(parents=True, exist_ok=True)
    arr = {f's_{k}': np.asarray(v) for k, v in (streams or {}).items()}
    np.savez_compressed(out / 'closed.npz', ticks=np.array(log.rows, float), **arr)
    meta['metrics'] = U.fmt(m) if m is not None else None
    (out / 'meta.json').write_text(json.dumps(meta, indent=1, default=float))
    print(json.dumps({k: v for k, v in meta.items() if k not in ('lost', 'faults')}, default=float))


def main(argv=None):
    ap = argparse.ArgumentParser()
    ap.add_argument('layer', choices=('unity', 'numpy'))
    ap.add_argument('--boat', choices=tuple(BOATS), default='transient')
    ap.add_argument('--ctrl', type=int, default=0)
    ap.add_argument('--seed', type=int, default=1)
    ap.add_argument('--env', choices=('scenario', 'nowind', 'nocurrent', 'none'), default='scenario')
    ap.add_argument('--snap', choices=('on', 'off'), default='on')
    ap.add_argument('--projection', choices=('representable', 'hidden'), default='representable')
    ap.add_argument('--deadband0', action='store_true')
    ap.add_argument('--time-scale', type=float, default=8.0)
    ap.add_argument('--wall', type=float, default=60.0)
    ap.add_argument('--t-max', type=float, default=mission.T_MAX)
    ap.add_argument('--out', required=True)
    a = ap.parse_args(argv)
    st, idx = BOATS[a.boat]
    hidden_spec = U.get_boat(4242, st, idx)
    out = OUT_ROOT / a.out
    meta = {'layer': a.layer, 'boat': a.boat, 'ctrl': a.ctrl, 'seed': a.seed, 'env': a.env}
    if a.layer == 'numpy':
        plant.SNAP_FRAC = 1e-3 if a.snap == 'on' else 0.0
        spec = U.representable_numpy(hidden_spec) if a.projection == 'representable' else hidden_spec
        spec = env_spec(spec, a.env)
        meta.update(snap=a.snap, projection=a.projection, snap_frac=plant.SNAP_FRAC,
                    current=spec.current[0].tolist(), wind=spec.wind[0].tolist())
        m, streams, log = numpy_run(spec, a.ctrl, a.seed, a.t_max)
        return save(out, m, streams, log, meta)
    if a.wall > 60:
        raise SystemExit('budget: <= 60 s per Unity launch')
    out.mkdir(parents=True, exist_ok=True)
    _, lost = U.write_config(out, hidden_spec, 0, match=True, env=True)
    sc = env_scenario(hidden_spec, a.env)
    (out / 'scenario.json').write_text(json.dumps(sc, indent=2))
    faults = json.loads((out / 'faults.json').read_text())
    if a.deadband0:
        faults['deadband_us'] = [0.0] * 4
        (out / 'faults.json').write_text(json.dumps(faults, indent=2))
    meta.update(scenario=sc, faults=faults, lost=lost, deadband0=a.deadband0, time_scale=a.time_scale)
    link = U.Link()
    argv_ = U.player_argv(out, link.port, a.seed, SimpleNamespace(sim_profile='minimal', time_scale=a.time_scale,
                                                                   player_wall=a.wall - 3))
    (out / 'launch.txt').write_text(' '.join(shlex.quote(x) for x in argv_) + '\n')
    env = {k: v for k, v in os.environ.items() if k not in ('DISPLAY', 'WAYLAND_DISPLAY')}
    env.update({'HEAVY_WHO': 'decisioncal parity closedloop', 'HEAVY_MAX_S': str(int(a.wall)), 'SDL_VIDEODRIVER': 'dummy'})
    t0 = time.time()
    for f in ('player.log', 'result.json'):
        (out / f).unlink(missing_ok=True)
    pl = Player(argv_, env, out, wall=a.wall)
    m = streams = None
    log = Log()
    try:
        link.accept(pl.grant_timeout + 60, alive=pl.alive)
        meta['connect_after_start_s'] = time.time() - (pl.t_start or time.time())
        meta['hello'] = [link.hello.x0, link.hello.y0, link.hello.yaw0]
        link.scenario(sc['cur_speed'], sc['cur_toward_deg'], sc['wind_speed'], sc['wind_toward_deg'])
        tr = time.time()
        m, streams, log = unity_run(link, hidden_spec, a.ctrl, a.seed, a.t_max, pl.deadline(4.0), log)
        meta['complete'] = True
        meta['run_wall_s'] = time.time() - tr
    except U.WallLimit as e:
        meta['complete'] = False
        meta['wall_limit'] = str(e)
    except Exception as e:
        meta['complete'] = False
        meta['error'] = f'{type(e).__name__}: {e}'
    finally:
        link.close()
        pl.finish(meta)
        meta['launch_to_end_wall_s'] = time.time() - t0
    r = out / 'result.json'
    if r.is_file():
        try:
            meta['water_invalid_searches'] = int(json.loads(r.read_text()).get('invalidWaterSearches') or 0)
        except ValueError:
            pass
    save(out, m, streams, log, meta)
    if meta.get('race_abort'):
        sys.exit(RACE_EXIT)


if __name__ == '__main__':
    main()
