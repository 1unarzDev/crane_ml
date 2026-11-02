"""The ``fastsim`` backend: the fast-sim core (crane ``fastsim/``) under the cluster's interface.

    backend = FastsimBackend()
    backend.prepare(job)            # once per job, in the worker, after the refs are checked out
    run_dir = backend.run(seed)     # one seeded run -> its run directory
    numbers = backend.metrics(run_dir)

``job`` keys: ``config`` (the domain YAML, absolute or relative to ``workdir``), ``out`` (the
job's output folder), and optionally ``workdir``, ``variant`` (default 'base'),
``overrides`` (merged over the YAML, e.g. a hypothesis's parameter set), ``calibration`` (an
epoch file) and ``seeds`` (scenarios the source can generate are generated in ``prepare``).
Any CPU worker can run it; nothing here needs a GPU or a display.
"""

import json
from pathlib import Path

from fastsim import config as fsconfig
from fastsim.batch import adapter_of
from fastsim.pairs import flat_numbers
from fastsim.run import simulate
from fastsim.scenario import ScenarioSource

NAME = 'fastsim'
NEEDS = {'gpu': False, 'display': False}


class FastsimBackend:
    name = NAME
    needs = NEEDS

    def prepare(self, job):
        workdir = Path(job.get('workdir') or '.')
        overrides = dict(job.get('overrides') or {})
        if job.get('calibration'):
            overrides['calibration'] = str((workdir / job['calibration']).resolve())
        self.cfg, self.base = fsconfig.load(workdir / job['config'], overrides)
        self.adapter = adapter_of(self.cfg, self.base)
        self.source = ScenarioSource(**self.cfg['scenarios'], base=self.base)
        if job.get('seeds'):
            self.source.generate(job['seeds'])
        self.out = Path(job['out']) / job.get('variant', 'base')
        self.provenance = {'variant': job.get('variant', 'base'), 'job': job.get('id')}
        return self

    def run(self, seed):
        out = self.out / f's{int(seed)}'
        simulate(self.source.load(seed), self.adapter, self.cfg, seed, out, self.base, self.provenance)
        return out

    def metrics(self, run_dir):
        """{outcome, run_sha256, flags, **numbers}: the run's own numbers and the adapter's."""
        run = json.loads((Path(run_dir) / 'run.json').read_text())
        numbers = flat_numbers({'run': {'sim_s': run['sim_s'], 'contacts': len(run['contacts']),
                                        'min_clearance_m': run['min_clearance_m']}})
        numbers.update(flat_numbers(self.adapter.metrics(run_dir)))
        return {'outcome': run['outcome'], 'run_sha256': run['identity']['run_sha256'],
                'flags': run['flags'], **numbers}


def create():
    return FastsimBackend()


BACKEND = FastsimBackend
