#!/usr/bin/env python3
"""Open-loop (and reset/clock) parity driver for the CRANE Unity player over the lockstep link.

One player launch runs a SUITE: for every sequence, ``reset`` -> the fresh state -> the sequence's per-tick
normalised commands q [n, 4] (FL FR RR RL) sent as pulses 1500 + 400 q. Everything that crosses the link is
logged per tick (tick, player t, world-ROS fields, course-frame state, pulses sent). The NumPy plant and the
Python twin of Unity's planar model are replayed on the SAME logged pulses offline (openloop_compare.py).

Speaks the protocol through ``unity_check.Link`` (no copy of it) and launches through ``heavy unity`` exactly
as ``unity_check.live`` does (player_argv, write_config), with a hard 60 s wall per launch.

    python3 ~/astro_dock/src/mhseals_agents/tools/heavy probe unity && \
    nice -n 10 python3 Tools/DecisionCal/parity/lockstep_openloop.py --boat transient --env none --out LABEL
    python3 Tools/DecisionCal/parity/lockstep_openloop.py --fake --boat transient --env none --out LABEL  (no Unity)
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
import unity_check as U  # noqa: E402
from plant import B as B_GEOM, DT  # noqa: E402
from _launch import RACE_EXIT, Player  # noqa: E402

OUT_ROOT = Path.home() / '.cache' / 'decisioncal' / 'parity' / 'unity'
BOATS = {'transient': ('transient', 0), 'steady': ('steady', 0), 'mixed': ('mixed', 0), 'timing': ('timing', 0)}
HIDDEN_SEED = 4242
SURGE, SWAY, YAW = (np.sign(B_GEOM[j]) for j in range(3))   # unit patterns: pure surge / sway / yaw thrust


def _const(q, sec):
    return np.tile(np.asarray(q, float), (int(round(sec / DT)), 1))


def suite(name):
    """[(sequence name, q [n, 4])]. The boat starts 3 m short of the real entry gate of the Unity course
    (buoys at course (3.06, +-1.16), r 0.23 m) and the marina lies 9.5 m to port, so every translation is driven
    ASTERN and to starboard (into the free water; the shore is 10 m astern): S = -surge pattern, W = -sway."""
    S, W = -SURGE, -SWAY
    seqs = [('zero30', _const(0 * S, 30)),
            ('surge0.5', _const(0.5 * S, 10)),
            ('yaw0.5', _const(0.5 * YAW, 10))]
    for q in (0.08, 0.12, 0.2, 0.3):
        seqs.append((f'hold_surge{q}', _const(q * S, 8)))
    for q in (0.08, 0.12, 0.2, 0.3):
        seqs.append((f'hold_yaw{q}', _const(q * YAW, 6)))
    comb = np.clip(_const(0.45 * S + 0.3 * W + 0.25 * YAW, 10), -1, 1)
    seqs += [('reversal0.8', np.concatenate([_const(0.8 * S, 5), _const(-0.8 * S, 5)])),
             ('coast1.0', np.concatenate([_const(1.0 * S, 6), _const(0 * S, 10)])),
             ('combined', comb),
             ('repeat_surge0.5', _const(0.5 * S, 10)),
             ('repeat_combined', comb.copy())]
    if name == 'C':
        return seqs
    pick = lambda *names: [x for x in seqs if x[0] in names]
    if name == 'A':                                   # ~55 s: drift, steps (one launch at RTF ~1.7)
        return pick('zero30', 'surge0.5', 'yaw0.5')
    if name == 'H':                                   # ~56 s: small-command holds
        return [x for x in seqs if x[0].startswith('hold')]
    if name == 'R':                                   # ~47 s: reversal, coast-down, combined
        return pick('reversal0.8', 'coast1.0', 'combined')
    if name == 'B':
        rep = np.concatenate([_const(0.5 * S, 6), _const(0.5 * YAW, 4), _const(0 * S, 4)])
        return [('zero2', _const(0 * S, 2))] + [(f'rep{i}', rep) for i in range(4)]
    if name == 'small':
        return [s for s in seqs if s[0].startswith('hold') or s[0] == 'zero30']
    raise ValueError(name)


def run_session(link, seqs, deadline, log):
    """Drive the suite on an accepted link. Returns {name: dict of arrays}; stops cleanly at the deadline."""
    fr = link.hello
    out = {}
    tick, t, w = link.read_state()           # the state after the connect-time ResetEpisode
    log['first_state'] = {'tick': tick, 't': t, 'world': w, 'course': fr.world_to_state(*w).tolist()}
    for name, q in seqs:
        if time.time() > deadline:
            log.setdefault('skipped', []).append(name)
            continue
        link.reset()
        tick, t, w = link.read_state()
        rows = [(tick, t, *w)]
        pulses = 1500.0 + 400.0 * np.clip(q, -1, 1)
        sent = 0
        for k in range(len(q)):
            if time.time() > deadline:
                break
            link.pulses(pulses[k])
            sent += 1
            tick, t, w = link.read_state()
            rows.append((tick, t, *w))
        R = np.array(rows, float)
        course = np.array([fr.world_to_state(*r[2:]) for r in R])
        out[name] = {'tick': R[:, 0], 't': R[:, 1], 'world': R[:, 2:], 'course': course, 'pulses': pulses[:sent],
                     'q': q[:sent], 'complete': sent == len(q)}
        print(f'  {name}: {sent}/{len(q)} ticks, final course state {np.round(course[-1], 3).tolist()}', flush=True)
    return out


def save(out_dir, seqs_out, meta):
    out_dir.mkdir(parents=True, exist_ok=True)
    flat = {f'{n}__{k}': v for n, d in seqs_out.items() for k, v in d.items() if isinstance(v, np.ndarray)}
    np.savez_compressed(out_dir / 'sequences.npz', **flat)
    meta['sequences'] = {n: {'ticks': int(len(d['q'])), 'complete': bool(d['complete'])} for n, d in seqs_out.items()}
    (out_dir / 'session.json').write_text(json.dumps(meta, indent=1, default=float))


def fake(a, spec, out_dir):
    """The NumPy representable plant behind the TCP protocol (unity_check.fake_player); no Unity."""
    link = U.Link()
    sp = U.representable_numpy(spec)
    if a.env == 'none':
        sp = sp.with_(wind=np.zeros((1, 2)), current=np.zeros((1, 2)))
    elif a.env == 'wind':
        sp = sp.with_(current=np.zeros((1, 2)))
    th = threading.Thread(target=U.fake_player, args=(link.host, link.port, sp, 1, (-2.275, -4.753, 0.0), 2000.0),
                          daemon=True)
    th.start()
    link.accept(30)
    link.scenario(0.0, 0.0, 0.0, 0.0)
    meta = {'mode': 'fake', 'boat': a.boat, 'env': a.env, 'hello': [link.hello.x0, link.hello.y0, link.hello.yaw0]}
    res = run_session(link, suite(a.suite), time.time() + 1e9, meta)
    link.end()
    link.close()
    save(out_dir, res, meta)


def live(a, spec, out_dir):
    scenario, lost = U.write_config(out_dir, spec, 0, match=True, env=a.env != 'none')
    if a.env == 'wind':
        scenario.update(cur_speed=0.0, cur_toward_deg=0.0)
    if a.env == 'current':
        scenario.update(wind_speed=0.0, wind_toward_deg=0.0)
    (out_dir / 'scenario.json').write_text(json.dumps(scenario, indent=2))
    faults = json.loads((out_dir / 'faults.json').read_text())
    if a.deadband0:
        faults['deadband_us'] = [0.0] * 4
        (out_dir / 'faults.json').write_text(json.dumps(faults, indent=2))
    link = U.Link()
    pa = SimpleNamespace(sim_profile='minimal', time_scale=a.time_scale, player_wall=a.wall - 3)
    argv = U.player_argv(out_dir, link.port, a.crane_seed, pa)
    (out_dir / 'launch.txt').write_text(' '.join(shlex.quote(x) for x in argv) + '\n')
    env = {k: v for k, v in os.environ.items() if k not in ('DISPLAY', 'WAYLAND_DISPLAY')}
    env.update({'HEAVY_WHO': 'decisioncal parity openloop', 'HEAVY_MAX_S': str(int(a.wall)), 'SDL_VIDEODRIVER': 'dummy'})
    meta = {'mode': 'live', 'boat': a.boat, 'env': a.env, 'suite': a.suite, 'scenario': scenario, 'not_representable': lost,
            'faults': faults, 'crane_seed': a.crane_seed, 'time_scale': a.time_scale, 'deadband0': a.deadband0}
    t_launch = time.time()
    for f in ('player.log', 'result.json'):
        (out_dir / f).unlink(missing_ok=True)
    pl = Player(argv, env, out_dir, wall=a.wall)
    res = {}
    try:
        link.accept(pl.grant_timeout + 60, alive=pl.alive)
        meta['connect_after_start_s'] = time.time() - (pl.t_start or time.time())
        meta['hello'] = [link.hello.x0, link.hello.y0, link.hello.yaw0]
        link.scenario(scenario['cur_speed'], scenario['cur_toward_deg'], scenario['wind_speed'], scenario['wind_toward_deg'])
        t_run = time.time()
        res = run_session(link, suite(a.suite), pl.deadline(), meta)
        meta['run_wall_s'] = time.time() - t_run
        meta['sim_s'] = float(sum(len(d['q']) for d in res.values()) * DT)
        link.end()
    except Exception as e:                     # keep whatever was logged
        meta['error'] = f'{type(e).__name__}: {e}'
    finally:
        link.close()
        pl.finish(meta)
        meta['launch_to_end_wall_s'] = time.time() - t_launch
    r = out_dir / 'result.json'
    if r.is_file():
        try:
            meta['water_invalid_searches'] = int(json.loads(r.read_text()).get('invalidWaterSearches') or 0)
        except ValueError:
            pass
    save(out_dir, res, meta)
    print(json.dumps({k: v for k, v in meta.items() if k not in ('not_representable', 'faults')}, default=float))
    if meta.get('race_abort'):
        sys.exit(RACE_EXIT)


def main(argv=None):
    ap = argparse.ArgumentParser()
    ap.add_argument('--boat', choices=tuple(BOATS), default='transient')
    ap.add_argument('--env', choices=('none', 'wind', 'current', 'scenario'), default='none')
    ap.add_argument('--suite', choices=('A', 'H', 'R', 'B', 'C', 'small'), default='C')
    ap.add_argument('--crane-seed', type=int, default=1)
    ap.add_argument('--time-scale', type=float, default=8.0)
    ap.add_argument('--wall', type=float, default=60.0, help='hard wall per launch, s (budget: <= 60)')
    ap.add_argument('--deadband0', action='store_true', help='Unity faults deadband_us = 0')
    ap.add_argument('--fake', action='store_true')
    ap.add_argument('--out', required=True)
    a = ap.parse_args(argv)
    if a.wall > 60:
        raise SystemExit('budget: <= 60 s per Unity launch')
    st, idx = BOATS[a.boat]
    spec = U.get_boat(HIDDEN_SEED, st, idx)
    out_dir = OUT_ROOT / a.out
    out_dir.mkdir(parents=True, exist_ok=True)
    (fake if a.fake else live)(a, spec, out_dir)


if __name__ == '__main__':
    main()
