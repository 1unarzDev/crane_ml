"""The ``crane-command`` backend: one configured argv per seed, numbers from the run's metrics.json.

    backend = ClusterBackend('task', {'argv': ['python3', '-m', 'pkg.task', '--seed', '{seed}', '--out', '{out}']})
    backend.prepare(ctx)            # ctx: the cluster's JobContext; spec keys override the options
    run_dir = backend.run(seed)     # runs argv in workdir with {seed} and {out} (the run dir) filled in
    numbers = backend.metrics(run_dir)

Keys (backend options, overridden by the job spec): ``argv`` (list, required), ``workdir`` (resolved
like the fastsim backend: inside spec ``repo``'s checkout, else under option ``root``, else as is),
``env`` (mapping; values take the same fields), ``timeout_s`` (default 3600), ``metrics_file``
(default metrics.json). The child also gets CRANE_SEED and CRANE_RUN_DIR. A nonzero exit fails the
seed with the log tail. Metrics are the file's numbers (booleans and strings dropped).
"""

import json
import os
import signal
import subprocess
from pathlib import Path

NAME = 'crane-command'


class ClusterBackend:
    def __init__(self, name=NAME, options=None):
        self.name, self.options, self.ctx, self.proc = name, dict(options or {}), None, None

    def update(self):
        pass

    def prepare(self, ctx):
        self.ctx = ctx
        job = {**self.options, **(ctx.job.get('spec') or {})}
        if not isinstance(job.get('argv'), list) or not job['argv']:
            raise ValueError(f"backend {self.name}: 'argv' (a list) is required")
        workdir = Path(job.get('workdir') or '.')
        if job.get('repo'):
            workdir = Path(ctx.repos[job['repo']]) / workdir
        elif job.get('root'):
            workdir = Path(job['root']).expanduser() / workdir
        self.job, self.workdir = job, workdir.resolve()
        self.runs = Path(ctx.work_dir) / 'runs'

    def _fill(self, text, seed, out):
        return str(text).replace('{seed}', str(int(seed))).replace('{out}', str(out))

    def run(self, seed):
        out = self.runs / f's{int(seed)}'
        out.mkdir(parents=True, exist_ok=True)
        argv = [self._fill(a, seed, out) for a in self.job['argv']]
        env = {**os.environ, **(getattr(self.ctx, 'env', None) or {}),
               **{k: self._fill(v, seed, out) for k, v in (self.job.get('env') or {}).items()},
               'CRANE_SEED': str(int(seed)), 'CRANE_RUN_DIR': str(out)}
        with open(out / 'command.log', 'wb') as log:
            self.proc = subprocess.Popen(argv, cwd=self.workdir, env=env, stdout=log, stderr=subprocess.STDOUT,
                                         start_new_session=True)
            try:
                rc = self.proc.wait(timeout=float(self.job.get('timeout_s', 3600)))
            except subprocess.TimeoutExpired:
                self.cancel()
                raise RuntimeError(f'seed {seed} timed out')
            finally:
                self.proc = None
        if rc != 0:
            tail = (out / 'command.log').read_bytes()[-2000:].decode(errors='replace')
            raise RuntimeError(f'command exited {rc}: {tail}')
        return out

    def metrics(self, run_dir):
        data = json.loads((Path(run_dir) / self.job.get('metrics_file', 'metrics.json')).read_text())
        return {k: float(v) for k, v in data.items() if isinstance(v, (int, float)) and not isinstance(v, bool)}

    def artifacts(self, run_dir):
        return [(p, p.name, 'summary' if p.suffix == '.json' else 'raw')
                for p in sorted(Path(run_dir).iterdir()) if p.is_file()]

    def cancel(self):
        p = self.proc
        if p and p.poll() is None:
            try:
                os.killpg(p.pid, signal.SIGTERM)
            except ProcessLookupError:
                pass

    def finish(self):
        pass
