"""The ``crane-batch`` backend: one study cell x seed chunk per job, on every core of one slot.

    backend = ClusterBackend('crane_batch', {})     # worker.yaml: crane_batch: {type: crane-batch, slots: 1}
    backend.prepare(ctx)                              # ctx.job['spec'] is what crane.cluster.compile wrote
    run_dir = backend.run(0)                          # crane study run-cell SPEC --cell ID --seeds .. --out ..
    backend.metrics(run_dir)                          # at most 1000 finite numbers

    python3 -m crane_backends.batch --job JSON        # the same, as spec.run of a py_command/gpu_command job

What a run leaves in its run dir (``$CRANE_RUN_DIR``):
  progress.json   {done, total, failed, rate_per_s, eta_s, phase, updated}, by atomic rename at most 60 s apart
                  (run-cell's crane.progress.Progress writes it; this runner writes the first and the last state);
  cell.json       which study, stage, arm, block, role, cell, seeds, host and job this is (crane monitor reads it);
  metrics.json    rows, failed, wall_s, units_per_s, and per-column means (dev and other non-held-out blocks only:
                  the cluster feed publishes metric means, so a held-out block reports counts only);
  failure.json    on failure: reason (crashed / timed out without progress / cancelled), exit code, a traceback sample;
  artifacts/rows/ the columnar rows plus manifest.json (sha256 per file). With an output root (option ``out_root``,
                  CRANE_OUT_ROOT, or /workspace/out on a rented box) the rows go to OUT_ROOT/<study>/<job_id>/ and
                  only manifest.json stays under artifacts/rows/.
stdout gets one human line and one ``RC-PROGRESS {"done": n, "total": m}`` line per minute. The child gets every
core: nothing here limits threads. A child whose progress does not move for ``stall_s`` is stopped.
"""

import argparse
import json
import math
import os
from pathlib import Path
import signal
import socket
import subprocess
import sys
import time

NAME = 'crane-batch'
ARTIFACT_MAX = 95 * 1024 * 1024
METRICS_MAX = 1000
BOX_OUT = Path('/workspace/out')
HELD_OUT = 'held-out'
ROOT = Path(__file__).resolve().parent.parent
REASONS = ('crashed', 'timed out without progress', 'cancelled')


def _write_json(path, doc):
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    tmp = path.with_name(f'.{path.name}.tmp{os.getpid()}')
    tmp.write_text(json.dumps(doc, indent=1, allow_nan=False) + '\n')
    os.replace(tmp, path)


def _read_json(path):
    try:
        return json.loads(Path(path).read_text())
    except (OSError, ValueError):
        return None


def _now():
    return time.strftime('%Y-%m-%dT%H:%M:%SZ', time.gmtime())


def resolve_repo(name, repos=None, cwd=None):
    """A job repo's checkout: the backend's ctx.repos, else RC_WORK_DIR/src/NAME, ./src/NAME, ../NAME (a
    crane-command job runs inside the crane_ml checkout), else this crane root for crane_ml."""
    if repos and name in repos:
        return Path(repos[name])
    cwd = Path(cwd or os.getcwd())
    dirname = name.replace('/', '__')
    for base in filter(None, [os.environ.get('RC_WORK_DIR'), cwd]):
        if (Path(base) / 'src' / dirname).is_dir():
            return Path(base) / 'src' / dirname
    if (cwd.parent / dirname).is_dir() and cwd.parent.name == 'src':
        return cwd.parent / dirname
    if name == 'crane_ml':
        return ROOT
    raise FileNotFoundError(f'repo {name} is not checked out (looked in RC_WORK_DIR/src, ./src, ../)')


def out_dir_for(job, run_dir, job_id, out_root=None):
    """Where the rows go: OUT_ROOT/<study>/<job_id>/ when there is an output root, else run_dir/artifacts/rows."""
    root = out_root or os.environ.get('CRANE_OUT_ROOT')
    if not root and BOX_OUT.is_dir() and os.access(BOX_OUT, os.W_OK):
        root = BOX_OUT
    if root:
        return Path(root) / job['study'] / job_id, True
    return Path(run_dir) / 'artifacts' / 'rows', False


def _means(rows):
    """Per numeric column: the mean of its finite values (one streaming pass)."""
    sums, counts = {}, {}
    for r in rows:
        for k, v in r.items():
            if isinstance(v, bool) or not isinstance(v, (int, float)) or not math.isfinite(v):
                continue
            sums[k] = sums.get(k, 0.0) + float(v)
            counts[k] = counts.get(k, 0) + 1
    return {k: sums[k] / counts[k] for k in sums}


def metrics_of(out, role, progress, wall_s, units):
    """metrics.json: counts always; per-column means only off held-out blocks; at most METRICS_MAX finite numbers."""
    from crane.data.rows import read_rows
    man = _read_json(Path(out) / 'manifest.json') or {}
    m = {'rows': int(man.get('rows') or sum(int(f.get('rows') or 0) for f in man.get('files', []))),
         'failed': int((progress or {}).get('failed') or 0), 'units': units, 'wall_s': round(wall_s, 3),
         'units_per_s': round(units / wall_s, 6) if wall_s > 0 else 0.0}
    if role != HELD_OUT and man.get('files'):
        for k, v in sorted(_means(read_rows(out)).items()):
            if len(m) >= METRICS_MAX:
                break
            if k != 'seed':
                m[f'{k}.mean'] = round(v, 9)
    return {k: v for k, v in m.items() if isinstance(v, (int, float)) and math.isfinite(v)}


def _manifest(out, job, job_id):
    """out/manifest.json as run-cell wrote it, else one built over its rows* files; every listed file checked."""
    from crane.data.rows import _sha, write_manifest
    out = Path(out)
    man = _read_json(out / 'manifest.json')
    if not man:
        files = [{'path': p.name, 'rows': sum(1 for _ in _rows_in(p)), 'sha256': _sha(p)}
                 for p in sorted(out.glob('rows*')) if p.is_file()]
        man = write_manifest(out, {}, files, {})
    for f in man.get('files', []):
        p = out / f['path']
        if not p.is_file():
            raise RuntimeError(f'manifest names {f["path"]}, which run-cell did not write')
        if f.get('sha256') and _sha(p) != f['sha256']:
            raise RuntimeError(f'{f["path"]}: sha256 differs from manifest.json')
    extra = {'study': job['study'], 'stage': job.get('stage'), 'arm': job['arm'], 'block': job.get('block'),
             'role': job.get('role'), 'cell': job['cell'], 'seeds': job['seeds'], 'job_id': job_id}
    if any(man.get(k) != v for k, v in extra.items()):
        man.update(extra)
        _write_json(out / 'manifest.json', man)
    return man


def _rows_in(path):
    from crane.data.rows import read_rows
    return read_rows(path)


def _traceback(log, n=30):
    try:
        lines = Path(log).read_text(errors='replace').splitlines()
    except OSError:
        return ''
    starts = [i for i, ln in enumerate(lines) if ln.startswith('Traceback')]
    part = lines[starts[-1]:] if starts else lines[-n:]
    return '\n'.join(part[-n:])


class Runner:
    """One job: the child run-cell process, its progress, the stall guard, and what it leaves behind."""

    def __init__(self, job, run_dir, job_id=None, repos=None, out_root=None, command=None, stall_s=None,
                 poll_s=1.0, line_s=60.0, stream=None, python=None):
        self.job, self.run_dir = job, Path(run_dir)
        self.job_id = job_id or os.environ.get('CRANE_JOB_ID') or os.environ.get('RC_JOB_ID') or \
            f"{job['study']}.{job.get('dedup', 'job')[:12]}"
        self.job_id = self.job_id.split('/')[-1]
        self.repos, self.out_root, self.command = repos, out_root, command
        self.stall_s = float(stall_s or job.get('stall_s') or 900)
        self.poll_s, self.line_s, self.stream = float(poll_s), float(line_s), stream or sys.stdout
        self.python = python or sys.executable
        self.proc, self.cancelled = None, False

    def argv(self, spec, out):
        fields = {'spec': str(spec), 'cell': self.job['cell'], 'seeds': ','.join(map(str, self.job['seeds'])),
                  'out': str(out), 'python': self.python}
        if self.command:
            return [str(a).format(**fields) for a in self.command]
        return [self.python, '-m', 'crane', 'study', 'run-cell', fields['spec'], '--cell', fields['cell'],
                '--seeds', fields['seeds'], '--out', fields['out']]

    def _spec_path(self):
        src = self.job['spec_file']
        if 'text' in src:
            path = self.run_dir / src.get('name', 'study.yaml')
            path.write_text(src['text'])
            return path
        return resolve_repo(src['repo'], self.repos) / src['path']

    def say(self, state):
        done, total = int(state.get('done') or 0), int(state.get('total') or len(self.job['seeds']))
        from crane.progress import line
        print(line({'failed': 0, 'rate_per_s': 0.0, 'eta_s': None, **state, 'done': done, 'total': total}, 'units'),
              file=self.stream, flush=True)
        print('RC-PROGRESS ' + json.dumps({'done': done, 'total': total}), file=self.stream, flush=True)

    def run(self):
        """Runs the job; returns {status: ok|failed, reason, exit, wall_s, out, external}."""
        from crane.progress import FILE
        job, rd = self.job, self.run_dir
        rd.mkdir(parents=True, exist_ok=True)
        for stale in ('failure.json', 'metrics.json'):
            (rd / stale).unlink(missing_ok=True)
        out, external = out_dir_for(job, rd, self.job_id, self.out_root)
        out.mkdir(parents=True, exist_ok=True)
        meta = {'schema': 'crane.cluster.cell/1', 'job_id': self.job_id, 'host': socket.gethostname(),
                'started': _now(), 'out': str(out), **{k: job.get(k) for k in (
                    'study', 'stage', 'arm', 'block', 'role', 'cell', 'cell_dir', 'tier', 'seeds', 'dedup')}}
        _write_json(rd / 'cell.json', meta)
        total = len(job['seeds'])
        _write_json(rd / FILE, {'done': 0, 'total': total, 'failed': 0, 'rate_per_s': 0.0, 'eta_s': None,
                                'phase': 'start', 'updated': _now(), 'label': job['cell']})
        crane_root = resolve_repo('crane_ml', self.repos)
        env = {**os.environ, 'CRANE_RUN_DIR': str(rd), 'CRANE_STUDY_OUT': str(out),
               'PYTHONPATH': os.pathsep.join(filter(None, [str(crane_root), os.environ.get('PYTHONPATH')])),
               'CRANE_WORKERS': str(os.cpu_count() or 1)}
        t0 = time.monotonic()
        log_path = rd / 'run.log'
        reason, code = None, None
        try:
            argv = self.argv(self._spec_path(), out)
        except FileNotFoundError as error:
            return self._fail(rd, 'crashed', None, str(error), t0, out, external)
        with open(log_path, 'ab') as log:
            self.proc = subprocess.Popen(argv, cwd=crane_root, env=env, stdout=log, stderr=subprocess.STDOUT,
                                         start_new_session=True)
            last_done, moved, said = -1, t0, t0
            while True:
                try:
                    code = self.proc.wait(timeout=self.poll_s)
                    break
                except subprocess.TimeoutExpired:
                    pass
                now = time.monotonic()
                state = _read_json(rd / FILE) or {}
                done = int(state.get('done') or 0)
                if done != last_done or state.get('phase') not in (None, 'start', 'run'):
                    if done != last_done:
                        moved = now
                    last_done = done
                if now - said >= self.line_s:
                    said = now
                    self.say(state)
                if self.cancelled:
                    reason = 'cancelled'
                    self._stop()
                elif now - moved > self.stall_s:
                    reason = 'timed out without progress'
                    self._stop()
            self.proc = None
        wall = time.monotonic() - t0
        if self.cancelled and reason is None:
            reason = 'cancelled'
        if reason or code:
            return self._fail(rd, reason or 'crashed', code, _traceback(log_path), t0, out, external)
        try:
            man = _manifest(out, job, self.job_id)
            big = [f['path'] for f in man.get('files', []) if not external and
                   (out / f['path']).stat().st_size > ARTIFACT_MAX]
            if big:
                raise RuntimeError(f'{big[0]} is over 95 MB as an artifact; give the backend an out_root '
                                   '(CRANE_OUT_ROOT) or write smaller parts')
            if external:
                _write_json(rd / 'artifacts' / 'rows' / 'manifest.json', man)
        except (RuntimeError, OSError, ValueError) as error:
            return self._fail(rd, 'crashed', 0, f'{type(error).__name__}: {error}', t0, out, external)
        state = _read_json(rd / FILE) or {}
        metrics = metrics_of(out, job.get('role'), state, wall, total)
        _write_json(rd / 'metrics.json', metrics)
        self._final(rd, state, 'done', total)
        self.say(_read_json(rd / FILE) or {})
        return {'status': 'ok', 'reason': None, 'exit': 0, 'wall_s': wall, 'out': str(out), 'external': external}

    def _final(self, rd, state, phase, total):
        """The last progress.json: done = total when the job finished, as run-cell left it otherwise."""
        from crane.progress import FILE
        total = int(state.get('total') or total)
        done = total if phase == 'done' else int(state.get('done') or 0)
        _write_json(rd / FILE, {'failed': 0, 'rate_per_s': 0.0, **state, 'done': done, 'total': total, 'eta_s': 0.0
                                if phase == 'done' else None, 'phase': phase, 'updated': _now(),
                                'label': self.job['cell']})

    def _fail(self, rd, reason, code, tb, t0, out, external):
        from crane.progress import FILE
        wall = time.monotonic() - t0
        _write_json(rd / 'failure.json', {'reason': reason, 'exit': code, 'traceback': tb[-4000:], 'at': _now(),
                                          'host': socket.gethostname(), 'wall_s': round(wall, 3)})
        state = _read_json(rd / FILE) or {}
        self._final(rd, state, 'cancelled' if reason == 'cancelled' else 'failed', len(self.job['seeds']))
        _write_json(rd / 'metrics.json', {'rows': 0, 'failed': int(state.get('failed') or 0),
                                          'units': len(self.job['seeds']), 'wall_s': round(wall, 3)})
        return {'status': 'failed', 'reason': reason, 'exit': code, 'wall_s': wall, 'out': str(out),
                'external': external, 'traceback': tb}

    def _stop(self, grace_s=10.0):
        p = self.proc
        if p is None or p.poll() is not None:
            return
        try:
            os.killpg(p.pid, signal.SIGTERM)
            p.wait(timeout=grace_s)
        except ProcessLookupError:
            return
        except subprocess.TimeoutExpired:
            try:
                os.killpg(p.pid, signal.SIGKILL)
            except ProcessLookupError:
                pass

    def cancel(self):
        self.cancelled = True


class ClusterBackend:
    """The research-cluster backend (entry point ``crane-batch``). Options: out_root, command (an argv template with
    {spec} {cell} {seeds} {out} {python}), stall_s, poll_s, python."""

    def __init__(self, name=NAME, options=None):
        self.name, self.options, self.ctx, self.runner = name, dict(options or {}), None, None

    def update(self):
        pass

    def prepare(self, ctx):
        self.ctx = ctx
        spec = (ctx.job.get('spec') or {})
        for k in ('study', 'arm', 'cell', 'seeds', 'spec_file'):
            if k not in spec:
                raise ValueError(f"backend {self.name}: spec.{k} is missing; the job is not crane.cluster.job/1 "
                                 '(crane cluster submit writes them)')
        self.job = spec

    def run(self, seed):
        rd = Path(self.ctx.work_dir) / 'runs' / f's{int(seed)}'
        self.runner = Runner(self.job, rd, job_id=self.ctx.job.get('job_id'), repos=dict(self.ctx.repos or {}),
                             out_root=self.options.get('out_root'), command=self.options.get('command'),
                             stall_s=self.options.get('stall_s'), poll_s=self.options.get('poll_s', 1.0),
                             python=self.options.get('python'))
        try:
            result = self.runner.run()
        finally:
            self.runner = None
        if result['status'] != 'ok':
            last = (result.get('traceback') or '').strip().splitlines()[-1:]
            raise RuntimeError(f"{result['reason']} (exit {result['exit']})" + (f': {last[0]}' if last else ''))
        return rd

    def metrics(self, run_dir):
        data = _read_json(Path(run_dir) / 'metrics.json') or {}
        return {k: float(v) for k, v in data.items() if isinstance(v, (int, float)) and not isinstance(v, bool)
                and math.isfinite(v)}

    def artifacts(self, run_dir):
        rd = Path(run_dir)
        out = [(rd / n, n, 'summary') for n in ('metrics.json', 'progress.json', 'cell.json', 'failure.json')
               if (rd / n).is_file()]
        if (rd / 'run.log').is_file():
            out.append((rd / 'run.log', 'run.log', 'raw'))
        arts = rd / 'artifacts'
        if arts.is_dir():
            out += [(p, p.relative_to(rd).as_posix(), 'raw') for p in sorted(arts.rglob('*')) if p.is_file()]
        return out

    def cancel(self):
        if self.runner:
            self.runner.cancel()

    def finish(self):
        pass


def main(argv=None):
    ap = argparse.ArgumentParser(prog='python3 -m crane_backends.batch',
                                 description='Run one crane.cluster.job/1 (a cell x seed chunk) in $CRANE_RUN_DIR.')
    ap.add_argument('--job', required=True, help='The job spec as JSON (crane.cluster.compile writes it).')
    ap.add_argument('--run-dir', help='Default $CRANE_RUN_DIR, else ./run.')
    ap.add_argument('--out-root', help='Rows go to OUT_ROOT/<study>/<job_id>/ (default CRANE_OUT_ROOT, or '
                    '/workspace/out on a rented box).')
    a = ap.parse_args(argv)
    job = json.loads(a.job)
    runner = Runner(job, a.run_dir or os.environ.get('CRANE_RUN_DIR') or 'run', out_root=a.out_root)
    signal.signal(signal.SIGTERM, lambda *_: runner.cancel())
    result = runner.run()
    if result['status'] != 'ok':
        print(f"crane-batch: {job['cell']}: {result['reason']} (exit {result['exit']}); see run.log", file=sys.stderr)
        return 143 if result['reason'] == 'cancelled' else 1
    return 0


if __name__ == '__main__':
    sys.exit(main())
