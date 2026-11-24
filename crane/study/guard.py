"""The held-out guard: the one held-out discipline kept.

    reason = guard.refuse(study, cells, confirm)     # None: may run; else the one-line refusal
    guard.targets_state(study)                        # {path, commit, committed_at} of the committed targets

A held-out cell (its seed block or its condition's split is held-out) runs only when both hold:
- the command says --confirm;
- the spec's ``targets:`` file is committed in git, has no uncommitted changes, and its last commit is older than
  now (so the predictions or targets were fixed before any held-out outcome existed).
Rows of held-out cells carry ``touched_held_out = true`` and the manifest records the targets commit; monitoring
shows progress only for them until their block is done.
"""

import subprocess
import time
from pathlib import Path


def _git(where, *argv):
    try:
        r = subprocess.run(['git', '-C', str(where), *argv], capture_output=True, text=True, check=False, timeout=30)
    except (OSError, subprocess.TimeoutExpired):
        return None
    return r.stdout.strip() if r.returncode == 0 else None


def targets_state(study, now=None):
    """({path, commit, committed_at}, None) when the targets file is committed, clean and older than now; else
    (None, the one-line reason with the fix)."""
    spec = study.path.name
    path = study.targets
    if path is None:
        return None, f'{spec} names no targets file; fix: add targets: PATH (committed before the run) to the spec'
    path = Path(path)
    if not path.is_file():
        return None, f'targets file {path} is missing; fix: write it, git add {path.name} && git commit'
    where = path.parent
    if _git(where, 'rev-parse', '--show-toplevel') is None:
        return None, f'targets file {path} is not in a git repository; fix: commit it in the study repo'
    log = _git(where, 'log', '-1', '--format=%H %ct', '--', path.name)
    if not log:
        return None, f'targets file {path.name} is not committed; fix: git add {path.name} && git commit'
    dirty = _git(where, 'status', '--porcelain', '--', path.name)
    if dirty:
        return None, f'targets file {path.name} has uncommitted changes; fix: git commit {path.name}, or restore it'
    commit, ct = log.split()
    now = time.time() if now is None else now
    if int(ct) >= now:
        return None, f'targets file {path.name} was committed at or after now ({int(ct)}); fix: wait, then rerun'
    return {'path': str(path), 'commit': commit, 'committed_at': int(ct)}, None


def refuse(study, cells, confirm, block=None, now=None):
    """(None, targets) when the cells may run, else (the one-line refusal, None). Cells with no held-out role pass."""
    held = sorted({c.block if c.split is None or study.splits.get(c.split) != 'held-out' else c.split
                   for c in cells if c.held_out})
    if not held:
        return None, None
    names = ', '.join(held)
    if not confirm:
        return (f'held-out {names} needs --confirm (and a committed targets file); fix: crane study run '
                f'{study.ref} --block {held[0]} --confirm'), None
    state, reason = targets_state(study, now)
    if reason:
        return f'held-out {names}: {reason}', None
    return None, state
