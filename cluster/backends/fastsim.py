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


class ClusterBackend:
    """FastsimBackend for crane-worker: ``type: "cluster.backends.fastsim:ClusterBackend"``.

    The job spec carries the ``job`` keys above (``out`` is the worker's job folder). A spec
    ``repo`` resolves ``workdir`` inside that repo's checkout at the job's exact ref; otherwise
    ``workdir`` comes from the spec or the backend options. Metrics are numbers only:
    ``succeeded`` (1 or 0), ``flags`` (a count) and the run's and adapter's numbers.
    """

    def __init__(self, name=NAME, options=None):
        self.name, self.options, self.ctx = name, dict(options or {}), None

    def update(self):
        pass

    def prepare(self, ctx):
        self.ctx = ctx
        job = {**self.options, **ctx.spec, 'id': ctx.job.get('job_id'), 'out': str(ctx.work_dir / 'runs')}
        if job.get('repo'):
            job['workdir'] = str(Path(ctx.repos[job['repo']]) / job.get('workdir', '.'))
        self.fast = FastsimBackend().prepare(job)

    def run(self, seed):
        return self.fast.run(seed)

    def metrics(self, run_dir):
        m = self.fast.metrics(run_dir)
        numbers = {k: float(v) for k, v in m.items() if isinstance(v, (int, float)) and not isinstance(v, bool)}
        numbers.update(succeeded=float(m['outcome'] == 'SUCCEEDED'), flags=float(len(m['flags'])))
        return numbers

    def artifacts(self, run_dir):
        return [(p, p.name, 'summary' if p.name == 'run.json' else 'raw')
                for p in sorted(Path(run_dir).iterdir()) if p.is_file()]

    def cancel(self):
        pass

    def finish(self):
        pass


def create():
    return FastsimBackend()


BACKEND = FastsimBackend
