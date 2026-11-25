"""The host's share of a batch: load every seed's scenario (the format's identity check included),
build its world arrays and plan, in worker processes for a large batch.

    infos, worlds, plans, limits = prepare(cfg, base, seeds)     # Worlds and BoatPlans stacked

Workers return compact per-env rows (no scenario bytes); ``ScenarioInfo`` keeps what a run record
needs of each scenario.
"""

from dataclasses import dataclass
from multiprocessing import get_context
import os
from pathlib import Path
import sys
from types import SimpleNamespace

CHUNK = 256


@dataclass(frozen=True)
class ScenarioInfo:
    content_sha256: str
    changing: bool
    name: str
    filename: str
    pair_key: tuple

    def changing_states(self):
        return self.changing

    @classmethod
    def of(cls, s):
        return cls(s.content_sha256, bool(s.changing_states()), s.name, s.filename, tuple(s.pair_key))


def _chunk(job):
    from engine.boat import BoatPlans
    from engine.world import Worlds
    from fastsim.batch import adapter_of
    from fastsim.plant import Limits
    from fastsim.scenario import ScenarioSource
    from fastsim.world import World
    cfg, base, seeds = job
    adapter = adapter_of(cfg, base)
    lim = adapter.limits() or Limits.from_dict(cfg['limits'])
    source = ScenarioSource(**cfg['scenarios'], base=base)
    scenarios = [source.load(s) for s in seeds]
    worlds = [World(s) for s in scenarios]
    limits = [float(adapter.time_limit(SimpleNamespace(seed=seed, scenario=s, world=w)) or cfg['time_limit_s'])
              for seed, s, w in zip(seeds, scenarios, worlds)]
    return ([ScenarioInfo.of(s) for s in scenarios], Worlds.pack(worlds, lim.footprint),
            BoatPlans.pack(adapter, scenarios, worlds), limits)


def prepare(cfg, base, seeds, workers=None, gpu=False):
    from engine.boat import BoatPlans
    from engine.world import Worlds
    from fastsim.scenario import ScenarioSource
    ScenarioSource(**cfg['scenarios'], base=base).generate(seeds)     # or refuse in one line, before any pool
    jobs = [(cfg, base, seeds[i:i + CHUNK]) for i in range(0, len(seeds), CHUNK)]
    from crane.resources import prep_processes, single_thread_env
    workers = min(len(jobs), workers) if workers else prep_processes(len(jobs), gpu=gpu)
    if workers <= 1:
        parts = [_chunk(j) for j in jobs]
    else:
        # Spawned workers inherit sys.path. A course truth already loaded here put Tools/RoboBoat first on
        # it (course_truth.py does), whose coverage.py would shadow the package numba imports at start-up.
        # Each child gets one numba/BLAS thread (set in the env it inherits): processes x threads <= cpus().
        saved, saved_env = list(sys.path), {k: os.environ.get(k) for k in single_thread_env()}
        sys.path[:] = [p for p in saved if not (Path(p) / 'course_truth.py').is_file()]
        try:
            os.environ.update(single_thread_env())
            with get_context('spawn').Pool(workers) as pool:
                parts = pool.map(_chunk, jobs, chunksize=1)
        finally:
            sys.path[:] = saved
            for k, v in saved_env.items():
                os.environ.pop(k, None) if v is None else os.environ.__setitem__(k, v)
    return ([i for p in parts for i in p[0]], Worlds.stack([p[1] for p in parts]),
            BoatPlans.stack([p[2] for p in parts]), [t for p in parts for t in p[3]])
