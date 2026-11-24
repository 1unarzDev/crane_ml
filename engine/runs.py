"""Seeds to crane.run/1 records with the batched engine (``crane run --engine batched``).

    rows, wall = run_seeds(cfg, base, seeds, out, backend='numba', dtype='float64', record=0)
    # OUT/<variant>/s<seed>: run.json, events.jsonl, metrics.json (+ traj.ctraj with record=K)
"""

from pathlib import Path
import time


def run_seeds(cfg, base, seeds, out, variant='base', backend='numba', dtype=None, record=0, provenance=None):
    """[(seed, outcome, sim_s, run_dir)] and the engine's wall seconds."""
    from engine.episode import Setup, run
    from fastsim.batch import adapter_of
    dtype = dtype or 'float64'
    setup = Setup(cfg, base, adapter_of(cfg, base), list(seeds))
    t0 = time.perf_counter()
    result = run(setup, backend, dtype, record=record)
    wall = time.perf_counter() - t0
    rows = []
    for e, seed in enumerate(setup.seeds):
        d = Path(out) / variant / f's{seed}'
        result.write(e, d, {'variant': variant, **(provenance or {})})
        s = result.summary(e)
        rows.append((seed, s['outcome'], s['sim_s'], d))
    return rows, wall
