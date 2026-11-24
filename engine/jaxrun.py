"""The JAX backend: lockstep on the device over a pool of env slots, refilled as envs finish.

    S = run(setup, dtype='float32', record=0, slots=16384, chunk_steps=250)

One scan iteration is one tick period with its steps unrolled (track and tick are static per sub-step,
so the device never branches); the host looks at the slots between chunks. A slot whose episode has
ended takes the next queued seed (its constants and initial state rows are written in place), so a
long episode never holds a whole batch. Refills happen at chunk boundaries, where every slot's step
index is a multiple of the tick period: the static sub-step flags stay right for every env.
Returns the same full state dict (one row per seed) as engine/episode.py's numpy runner.
"""

import numpy as np

from engine import boat as BB
from engine.episode import consts, init_state, period, step_body
from engine.ops import jax_ops, tree_map

SHARED = ('tlim', 'foot', 'now')     # constants with no env axis


def _rows(tree, idx):
    return tree_map(lambda a: a[idx], tree)


def run(setup, dtype='float32', record=0, slots=None, chunk_steps=250):
    ops = jax_ops(dtype)
    jax, xp = ops.jax, ops.xp
    N = setup.E
    slots = max(1, min(N, slots or N))
    C_all = consts(setup)
    shared = {k: C_all.pop(k) for k in SHARED}
    S_all = init_state(setup, record)
    S_init = tree_map(lambda a: a.copy(), S_all)
    per = period(setup)
    n = max(1, chunk_steps // per)
    sh = tree_map(ops.to, shared)

    key = ('jax', dtype, record, n)
    cache = setup.__dict__.setdefault('_compiled', {})
    if key not in cache:
        @jax.jit
        def chunk(C, S):
            CC = {**C, **sh}

            def one_period(S, _):
                for k in range(per):
                    S = step_body(ops, setup, CC, S, record, k % setup.track_every == 0, k % setup.steps_per_tick == 0)
                return S, None
            S, _ = jax.lax.scan(one_period, S, None, length=n)
            return S

        @jax.jit
        def refill(C, S, idx, C_new, S_new):
            put = lambda a, b: a.at[idx].set(b)  # noqa: E731
            return tree_map2(put, C, C_new), tree_map2(put, S, S_new)

        @jax.jit
        def take_rows(S, idx):
            return tree_map(lambda a: a[idx], S)
        cache[key] = chunk, refill, take_rows
    chunk, refill, take_rows = cache[key]

    env = np.arange(slots)                     # the seed index each slot runs (-1: idle)
    queue = slots
    C = tree_map(ops.to, _rows(C_all, env))
    S = tree_map(ops.to, _rows(S_init, env))
    while True:
        S = chunk(C, S)
        out = np.asarray(S['M']['outcome'])
        done = np.flatnonzero((out != BB.OUT_NONE) & (env >= 0))
        if len(done):
            size = 1 << int(np.ceil(np.log2(len(done))))
            di = np.concatenate([done, np.full(size - len(done), done[0])])
            host = jax.device_get(take_rows(S, ops.to(di)))
            _put_rows(S_all, env[done], tree_map(lambda a: np.asarray(a)[:len(done)], host))
            env[done] = -1
            take = done[:max(0, N - queue)]
            if len(take):
                new = np.arange(queue, queue + len(take))
                queue += len(take)
                env[take] = new
                # pad to a power of two (repeating the first row: the same write) so refill compiles a few times
                size = 1 << int(np.ceil(np.log2(len(take))))
                ti = np.concatenate([take, np.full(size - len(take), take[0])])
                ni = np.concatenate([new, np.full(size - len(new), new[0])])
                C, S = refill(C, S, ops.to(ti), tree_map(ops.to, _rows(C_all, ni)), tree_map(ops.to, _rows(S_init, ni)))
        if (env < 0).all():
            break
    return S_all


def tree_map2(fn, a, b):
    if isinstance(a, dict):
        return {k: tree_map2(fn, a[k], b[k]) for k in a}
    return fn(a, b)


def _put_rows(dst, idx, src):
    if isinstance(dst, dict):
        for k in dst:
            _put_rows(dst[k], idx, src[k])
        return
    dst[idx] = src
