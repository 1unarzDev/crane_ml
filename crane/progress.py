"""Batch progress: ``progress.json`` for monitors, one human line per minute for people.

    with Progress(out, total=1000, phase='run') as p:
        p.skip(n_reused)                      # done before this process started: counted, not rated
        for result in results:
            p.step(failed=result.outcome == 'ERROR')

``<run_dir>/progress.json`` is ``{done, total, failed, rate_per_s, eta_s, phase, updated}`` (plus ``label``
when one is given, e.g. the batch variant), written by temp file +
rename at most every ``every_s`` (15 s by default, never more than 60 s apart while work finishes). When
``$CRANE_RUN_DIR`` is set (the compute worker forwards it) the file goes there instead. The stdout line reads

    progress  412/1000  3.1 runs/s  eta 3m10s  failed 0

``read(run_dir)`` and ``line(state)`` are the reader side, for ``crane monitor``.
"""

from datetime import datetime, timezone
import json
import os
from pathlib import Path
import sys
import tempfile
import time

FILE = 'progress.json'
ENV = 'CRANE_RUN_DIR'
MAX_EVERY_S = 60.0


class Progress:
    def __init__(self, run_dir, total, phase='run', every_s=15.0, line_s=60.0, unit='runs', stream=None,
                 clock=time.monotonic, label=None):
        self.label = label
        where = os.environ.get(ENV) or run_dir
        self.path = Path(where) / FILE if where else None
        self.total, self.phase, self.unit = int(total), str(phase), unit
        self.every_s = min(max(float(every_s), 0.0), MAX_EVERY_S)
        self.line_s, self.stream, self.clock = float(line_s), stream, clock
        self.done = self.failed = self.skipped = 0
        self.started = self.clock()
        self._wrote = self._said = None   # clock of the last file write / human line
        self.write()

    # the writer side
    def skip(self, n=1):
        """Count ``n`` units already done (reused); they shorten the ETA but not the rate."""
        self.done += int(n)
        self.skipped += int(n)
        self.tick()

    def step(self, n=1, failed=False):
        self.done += int(n)
        self.failed += int(n) if failed else 0
        self.tick()

    def set_phase(self, phase):
        self.phase = str(phase)
        self.write()

    def tick(self):
        """Write the file and the line when they are due."""
        now = self.clock()
        if self._wrote is None or now - self._wrote >= self.every_s:
            self.write(now)
        if now - (self._said if self._said is not None else self.started) >= self.line_s:
            self.say(now)

    def state(self, now=None):
        now = self.clock() if now is None else now
        elapsed = max(now - self.started, 1e-9)
        worked = self.done - self.skipped
        rate = worked / elapsed if worked > 0 else 0.0
        left = max(self.total - self.done, 0)
        eta = round(left / rate, 1) if rate > 0 else (0.0 if left == 0 else None)
        state = {'done': self.done, 'total': self.total, 'failed': self.failed, 'rate_per_s': round(rate, 4),
                 'eta_s': eta, 'phase': self.phase,
                 'updated': datetime.now(timezone.utc).isoformat(timespec='seconds')}
        return {**state, 'label': self.label} if self.label is not None else state

    def write(self, now=None):
        now = self.clock() if now is None else now
        self._wrote = now
        if self.path is not None:
            write_atomic(self.path, json.dumps(self.state(now), indent=1) + '\n')

    def say(self, now=None):
        now = self.clock() if now is None else now
        self._said = now
        print(line(self.state(now), self.unit), file=self.stream or sys.stdout, flush=True)

    def close(self, phase='done'):
        """The final write (and a last line when any line was printed before)."""
        self.phase = phase
        self.write()
        if self._said is not None:
            self.say()

    def __enter__(self):
        return self

    def __exit__(self, kind, value, tb):
        self.close('done' if kind is None else 'failed')
        return False


def write_atomic(path, text):
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    fd, tmp = tempfile.mkstemp(dir=path.parent, prefix=f'.{path.name}.')
    try:
        with os.fdopen(fd, 'w') as f:
            f.write(text)
        os.replace(tmp, path)
    except BaseException:
        Path(tmp).unlink(missing_ok=True)
        raise


# the reader side
def read(run_dir):
    """The state in ``run_dir/progress.json`` (or the file itself) plus ``age_s`` since its update; None when
    there is none yet."""
    path = Path(run_dir)
    path = path / FILE if path.is_dir() else path
    try:
        state = json.loads(path.read_text())
    except (OSError, ValueError):
        return None
    try:
        updated = datetime.fromisoformat(state['updated'])
        state['age_s'] = round((datetime.now(timezone.utc) - updated).total_seconds(), 1)
    except (KeyError, TypeError, ValueError):
        state['age_s'] = None
    return state


def duration(seconds):
    """3m10s, 1h04m, 42s; '?' when unknown."""
    if seconds is None:
        return '?'
    s = int(round(seconds))
    if s >= 3600:
        return f'{s // 3600}h{s % 3600 // 60:02d}m'
    if s >= 60:
        return f'{s // 60}m{s % 60:02d}s'
    return f'{s}s'


def line(state, unit='runs'):
    """``progress  412/1000  3.1 runs/s  eta 3m10s  failed 0`` (a phase other than run/done is appended)."""
    text = (f"progress  {state['done']}/{state['total']}  {state['rate_per_s']:.1f} {unit}/s  "
            f"eta {duration(state['eta_s'])}  failed {state['failed']}")
    return text if state.get('phase') in ('run', 'done', None) else f"{text}  phase {state['phase']}"
