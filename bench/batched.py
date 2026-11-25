"""The batched engine's throughput against table B (bench/BASELINE.md).

    python3 -m bench.fast --engine batched --domain boat --tier T1 --backend numba,jax --E 1,64,1024,4096,16384 --out DIR

Per backend and batch size E: host setup (scenario load, world arrays, plans), the first call (compile,
or a numba cache load), then the timed run. Episodes/s is seeds / wall of the timed run; the speedup is
against table B's single process (W=1) and its 8 workers (W=8). numba runs E envs at once on every core
(prange); jax runs E slots on the GPU, refilled from a queue of max(E, --min-seeds) seeds; numpy is the
float64 reference. The split times each part of a step on its own (bench/batched.py: split_*).
Run it through ``bin/heavy sim``.
"""

import math
import os
import time

import numpy as np

from crane.resources import cpus

B = {('boat', 'T1'): {'W1': 4.33, 'W8': 27.9}, ('boat', 'T0'): {'W1': 9.63, 'W8': 55.9}}


def backends(spec):
    out = []
    for name in spec.split(','):
        backend, _, dtype = name.partition(':')
        out.append((backend, dtype or ('float32' if backend == 'jax' else 'float64')))
    return out


def measure(cfg, base, adapter, backend, dtype, E, min_seeds, repeats=2, fastmath=False):
    from engine.episode import Setup, run
    n = E if backend != 'jax' else max(E, min(4 * E, min_seeds))   # through E slots, refilled
    seeds = list(range(n))
    t0 = time.perf_counter()
    setup = Setup(cfg, base, adapter, seeds, backend=backend)
    setup_s = time.perf_counter() - t0
    kw = {'slots': E} if backend == 'jax' else {}
    first = run(setup, backend, dtype, fastmath=fastmath, **kw)
    best = None
    for _ in range(repeats):
        out = run(setup, backend, dtype, fastmath=fastmath, **kw)
        best = out if best is None or out.wall_s < best.wall_s else best
    done = [best.outcome(e) for e in range(n)]
    sim = [round(int(best.S['end'][e]) * setup.dt, 3) for e in range(n)]
    return {'backend': backend, 'dtype': dtype, 'fastmath': fastmath, 'E': E, 'seeds': n,
            'setup_s': round(setup_s, 3), 'first_call_s': round(first.wall_s, 3), 'wall_s': round(best.wall_s, 4),
            'episodes_per_s': round(n / best.wall_s, 1), 'sim_s_mean': round(float(np.mean(sim)), 2),
            'succeeded': done.count('SUCCEEDED'), 'env_steps_per_s': round(sum(int(best.S['end'][e]) for e in range(n)) / best.wall_s)}


# ---- where a step's time goes ----------------------------------------------------------------------

def split_numba(cfg, base, adapter, n=200):
    """Single-thread ns per call of each part of the numba episode, times its calls per episode."""
    import numba
    from engine.episode import Setup, run
    setup = Setup(cfg, base, adapter, list(range(n)))
    run(setup, 'numba', threads=1)
    t0 = time.perf_counter()
    out = run(setup, 'numba', threads=1)
    wall = time.perf_counter() - t0
    steps = float(np.sum(out.S['end']))
    # perception: the same episodes with no target left to perceive (the episode then skips it)
    tm = setup.plans.A['tmatch']
    saved = tm.copy()
    tm[:] = False
    run(setup, 'numba', threads=1)
    t0 = time.perf_counter()
    run(setup, 'numba', threads=1)
    blind = time.perf_counter() - t0
    tm[:] = saved
    parts = _numba_parts(setup)
    per_ep = steps / n
    ms = {'plant (velocity loop, allocation, kernel)': parts['drive'] * per_ep,
          'tracker and progress check': parts['tracker'] * per_ep,
          'track row (rounding, scoring)': parts['row'] * per_ep / setup.track_every,
          'contacts (footprint clearance)': parts['clearance'] * per_ep / setup.track_every}
    total = 1e3 * wall / n
    ms = {k: round(v * 1e-6, 3) for k, v in ms.items()}
    ms['perception (cone, occlusion; full run less one with nothing to perceive)'] = round(1e3 * (wall - blind) / n, 3)
    ms['decision and other (by difference)'] = round(total - sum(ms.values()), 3)
    numba.set_num_threads(min(numba.config.NUMBA_NUM_THREADS, cpus()))
    return {'ms_per_episode_single_thread': round(total, 3), 'parts_ms': ms, 'steps_per_episode': round(per_ep, 1)}


def _numba_parts(setup, calls=200000):
    """ns per call of each part on env 0's data at a few poses (numba)."""
    import numba
    from engine import geometry as GEO, tracker as TR, nbexact
    from engine.vehicles import nb_drive
    V = setup.vehicle.init(1)
    x, c = V['x'][0], V['c'][0]
    p, q = setup.P[0], setup.Q[0]
    W, A = setup.W, setup.plans.A
    route = A['course'][0].copy()
    rlen = int(A['course_len'][0])

    @numba.njit
    def drive(k, kind, x, c, p, q, dt, tick, sub):
        t = np.zeros(3)
        pwm = np.zeros(4)
        for i in range(k):
            t[0], t[1], t[2] = 1.0, 0.1, 0.3 * math.sin(i * 1e-3)
            nb_drive(kind, x, c, t, p, q, dt, tick, sub, pwm, True)

    @numba.njit
    def tracker(k, route, rlen, lim):
        out = np.zeros(3)
        s = 0.0
        for i in range(k):
            h, d = TR.nb_command(route, 0, rlen, False, 1e-6 * i, 0.2, 0.1, lim, out)
            s += out[0] + TR.nb_far(0.1, 0.2, 0.3 + 1e-7 * i, 0.25, 0.3)
        return s

    @numba.njit
    def row(k):
        s = 0.0
        for i in range(k):
            s += nbexact.pyround(12.3456789 + 1e-7 * i, 1e4) + nbexact.pyround(-3.25 + 1e-7 * i, 1e4)
            s += nbexact.pyround(0.123 + 1e-7 * i, 1e5) + nbexact.chypot(3.0 + 1e-7 * i, 4.0)
            s += math.atan2(1.0, 2.0 + 1e-7 * i)
        return s

    @numba.njit
    def clearance(k, xy, radius, box, dims, n, foot, reach):
        s = 0.0
        for i in range(k):
            cl, j = GEO.nb_clearance(xy, radius, box, dims, n, 1e-6 * i, 0.0, 0.1, foot, reach)
            s += j
        return s

    @numba.njit
    def perception(k, xy, radius, visible, box, n, cam, targets, nt):
        d0, d1, dist = np.zeros(n), np.zeros(n), np.zeros(n)
        cone, blockers = np.zeros(n, np.bool_), np.zeros(n, np.bool_)
        s = 0
        for i in range(k):
            GEO.nb_cone(xy, radius, visible, box, n, 1e-6 * i, 0.0, 0.1, cam, d0, d1, dist, cone, blockers)
            for t in range(nt):
                s += GEO.nb_visible(xy, radius, visible, box, n, 1e-6 * i, 0.0, 0.1, cam, targets[t], d0, d1, dist,
                                    cone, blockers, True)
        return s
    n = int(W.count[0])
    jobs = {'drive': (drive, (setup.vehicle.kind, x, c, p, q, setup.dt, setup.vehicle.tick_s, setup.vehicle.substeps)),
            'tracker': (tracker, (route, rlen, setup.tlim)), 'row': (row, ()),
            'clearance': (clearance, (W.xy[0], W.radius[0], W.box[0], W.box_dims[0], n, setup.foot, setup.reach))}
    out = {}
    for name, (fn, args) in jobs.items():
        fn(10, *args)
        t0 = time.perf_counter()
        fn(calls, *args)
        out[name] = 1e9 * (time.perf_counter() - t0) / calls
    return out


def split_jax(cfg, base, adapter, E, dtype='float32', calls=200):
    """ms per jitted call of a physics-only step and of a track + tick step at batch size E (jax)."""
    import re
    from engine.episode import Setup, consts, init_state, step_body
    from engine.ops import jax_ops, tree_map
    setup = Setup(cfg, base, adapter, list(range(E)))
    ops = jax_ops(dtype)
    jax = ops.jax
    C = tree_map(ops.to, consts(setup))
    S = tree_map(ops.to, init_state(setup, 0))
    out = {}
    for name, (tr, tk) in {'physics step (tracker, loop, plant, progress)': (False, False),
                           'track + tick step (adds scoring, contacts, perception, decision)': (True, True)}.items():
        f = jax.jit(lambda C, S, tr=tr, tk=tk: step_body(ops, setup, C, S, 0, tr, tk))
        kernels = len(re.findall(r'^\s*\S+ = \S+ (?:fusion|custom-call|gather|scatter|sort|reduce)\(',
                                 f.lower(C, S).compile().as_text(), re.M))
        f(C, S)
        t0 = time.perf_counter()
        for _ in range(calls):
            S2 = f(C, S)
        jax.block_until_ready(S2)
        out[name] = {'ms_per_call': round(1e3 * (time.perf_counter() - t0) / calls, 4), 'kernels': kernels}
    return {'E': E, 'dtype': dtype, 'steps': out}


def main(a, cfg, base, adapter, domain, tier):
    """``bench.fast --engine batched``: write DIR/batched.json and DIR/batched.md."""
    os.environ.setdefault('XLA_PYTHON_CLIENT_PREALLOCATE', 'false')
    sizes = [int(e) for e in a.E.split(',')]
    base_b = B.get((domain, tier))
    rows = []
    for backend, dtype in backends(a.backend):
        for E in sizes:
            if backend == 'numpy' and E > a.numpy_max:
                continue
            r = measure(cfg, base, adapter, backend, dtype, E, a.min_seeds, fastmath=a.fastmath)
            if base_b:
                r['x_B_W1'] = round(r['episodes_per_s'] / base_b['W1'], 1)
                r['x_B_W8'] = round(r['episodes_per_s'] / base_b['W8'], 1)
            rows.append(r)
            print(f"{domain} {tier} {backend}:{dtype} E={E}: {r['episodes_per_s']} episodes/s "
                  f"({r.get('x_B_W1')}x B W=1, {r.get('x_B_W8')}x B W=8)", flush=True)
    report = {'schema': 'crane.bench.batched/1', 'domain': domain, 'tier': tier, 'baseline_B': base_b, 'rows': rows}
    if a.split:
        names = {b for b, _ in backends(a.backend)}
        if 'numba' in names:
            report['split_numba'] = split_numba(cfg, base, adapter)
        if 'jax' in names:
            report['split_jax'] = split_jax(cfg, base, adapter, max(sizes))
    return report


def markdown(r):
    lines = [f"# bench.fast --engine batched: {r['domain']} {r['tier']}", '',
             f"Table B: {r['baseline_B']} episodes/s (scalar fastsim, W=1 and W=8)."
             f"{' Course pool ' + str(r['course_pool']) + ' (scenario seed = seed % ' + str(r['course_pool']) + ').' if r.get('course_pool') else ''}", '',
             '| backend | dtype | E | seeds | setup s | first call s | wall s | episodes/s | x B (W=1) | x B (W=8) | succeeded |',
             '|---|---|---:|---:|---:|---:|---:|---:|---:|---:|---:|']
    for x in r['rows']:
        lines.append(f"| {x['backend']}{'-fastmath' if x['fastmath'] else ''} | {x['dtype']} | {x['E']} | {x['seeds']} | "
                     f"{x['setup_s']} | {x['first_call_s']} | {x['wall_s']} | {x['episodes_per_s']} | {x.get('x_B_W1')} | "
                     f"{x.get('x_B_W8')} | {x['succeeded']}/{x['seeds']} |")
    if 'split_numba' in r:
        s = r['split_numba']
        lines += ['', f"numba, one thread: {s['ms_per_episode_single_thread']} ms per episode "
                      f"({s['steps_per_episode']} steps)", '', '| part | ms per episode |', '|---|---:|']
        lines += [f'| {k} | {v} |' for k, v in s['parts_ms'].items()]
    if 'split_jax' in r:
        s = r['split_jax']
        lines += ['', f"jax {s['dtype']} at E={s['E']}, one jitted step per call:", '',
                  '| step | ms per call | kernels |', '|---|---:|---:|']
        lines += [f"| {k} | {v['ms_per_call']} | {v['kernels']} |" for k, v in s['steps'].items()]
    return '\n'.join(lines) + '\n'

