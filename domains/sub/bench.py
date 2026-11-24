"""Throughput of the 6-DOF kernel and the sub mission on the batched engine.

    python3 -m domains.sub.bench --E 1,64,1024,4096,16384 --out DIR       # writes DIR/sub.json and prints a table

kernel: env control periods per second of ``advance`` (dt 0.02 s, one RK4 step) on numpy, numba (envs in
parallel) and JAX (float64 and float32, ``lax.scan`` over 200 periods); the wrench is fixed per env.
mission: episodes per second of the pool-gates mission (numba, every core), seeds 0..E-1, the first call
(compile or cache load) listed apart.
"""

import argparse
import json
import os
from pathlib import Path
import time

import numba
import numpy as np

from engine.kernels import marine6 as K

ROOT = Path(__file__).resolve().parents[2]
PROFILE = ROOT / 'domains' / 'sub' / 'profiles' / 'generic_rov8.json'
DT = 0.02


@numba.njit(cache=True, parallel=True)
def _nb_periods(X, U, P, dt, n):
    for e in numba.prange(X.shape[0]):
        w = np.zeros((6, K.S))
        for _ in range(n):
            K.nb_advance(X[e], U[e], P[e], dt, w)


def batch(E):
    g = np.random.default_rng(0)
    P = K.with_current(np.repeat(K.pack(PROFILE, DT)[None], E, axis=0), g.uniform(-0.2, 0.2, (E, 3)))
    X = K.init(P, g.uniform(2.0, 10.0, (E, 3)), g.uniform(-0.3, 0.3, (E, 3)))
    U = np.zeros((E, K.U))
    U[:, 6:] = K.allocate(g.uniform(-20.0, 20.0, (E, 6)), P)
    return X, U, P


def kernel(backend, E, periods):
    X, U, P = batch(E)
    if backend == 'numpy':
        t0 = time.perf_counter()
        for _ in range(periods):
            X = K.advance(X, U, P, DT)
        return periods * E / (time.perf_counter() - t0), None
    if backend == 'numba':
        t0 = time.perf_counter()
        _nb_periods(X.copy(), U, P, DT, 1)
        first = time.perf_counter() - t0
        t0 = time.perf_counter()
        _nb_periods(X, U, P, DT, periods)
        return periods * E / (time.perf_counter() - t0), first
    from engine.jaxconf import jax, jnp
    dtype = {'jax-f64': jnp.float64, 'jax-f32': jnp.float32}[backend]
    x, u, p = (jnp.asarray(a, dtype) for a in (X, U, P))

    @jax.jit
    def run(x):
        return jax.lax.scan(lambda x, _: (K.advance(x, u, p, DT, jnp, nmax=1), None), x, None, length=periods)[0]
    t0 = time.perf_counter()
    run(x).block_until_ready()
    first = time.perf_counter() - t0
    t0 = time.perf_counter()
    run(x).block_until_ready()
    return periods * E / (time.perf_counter() - t0), first


def mission(E):
    from fastsim import config
    from fastsim.batch import adapter_of
    from domains.sub import mission as M
    cfg, base = config.load(ROOT / 'domains' / 'sub' / 'fastsim_t1.yaml')
    adapter = adapter_of(cfg, base)
    t0 = time.perf_counter()
    setup = M.Setup(adapter, cfg, base, list(range(E)))
    host = time.perf_counter() - t0
    M.run(M.Setup(adapter, cfg, base, [0], setup.scenarios[:1]))        # compile or cache load
    out = M.run(setup)
    steps = int(out.S['end'].sum())
    ok = sum(out.outcome(e) == 'SUCCEEDED' for e in range(E))
    return {'E': E, 'setup_s': round(host, 3), 'episodes_s': round(out.wall_s, 3),
            'episodes_per_s': round(E / out.wall_s, 1), 'periods_per_s': round(steps / out.wall_s),
            'succeeded': ok}


def main(argv=None):
    ap = argparse.ArgumentParser(prog='python3 -m domains.sub.bench', description=__doc__.split('\n')[0])
    ap.add_argument('--E', default='1,64,1024,4096,16384')
    ap.add_argument('--kernel', default='numpy,numba,jax-f64,jax-f32')
    ap.add_argument('--periods', type=int, default=200)
    ap.add_argument('--mission', default='1,64,1024,4096')
    ap.add_argument('--out', required=True)
    a = ap.parse_args(argv)
    rows = {'host': {'cpus': os.cpu_count(), 'numba_threads': numba.get_num_threads()}, 'kernel': [], 'mission': []}
    for E in (int(v) for v in a.E.split(',')):
        for b in a.kernel.split(','):
            if b == 'numpy' and E > 4096:
                continue
            rate, first = kernel(b, E, a.periods if b != 'numpy' else min(a.periods, 50))
            rows['kernel'].append({'backend': b, 'E': E, 'periods_per_s': round(rate),
                                   'first_s': first and round(first, 2)})
            print(f'kernel  {b:8s} E={E:6d}  {rate:14,.0f} env-periods/s' + (f'  first {first:.2f} s' if first else ''))
    for E in (int(v) for v in a.mission.split(',') if v):
        r = mission(E)
        rows['mission'].append(r)
        print(f"mission numba    E={E:6d}  {r['episodes_per_s']:10,.1f} episodes/s  {r['periods_per_s']:12,d} periods/s"
              f"  setup {r['setup_s']} s  ok {r['succeeded']}/{E}")
    out = Path(a.out).expanduser()
    out.mkdir(parents=True, exist_ok=True)
    (out / 'sub.json').write_text(json.dumps(rows, indent=1) + '\n')
    print(f'wrote {out / "sub.json"}')


if __name__ == '__main__':
    main()
