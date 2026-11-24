"""The held-out guard: the one held-out discipline kept.

    reason, state = guard.refuse(study, cells, confirm)                  # here: git says the targets are committed
    reason, state = guard.refuse(study, cells, True, pinned=SHA256)      # on a box: the sha256 the submit pinned
    state, reason = guard.targets_state(study)      # {path, commit, committed_at, sha256, entries}

A held-out cell (its seed block or its condition's split is held-out) runs only when both hold:
- the command says --confirm;
- the spec's ``targets:`` file parses to at least one target, is committed in git, has no uncommitted changes, and
  its last commit is older than the first held-out unit of this output (now, on a first run), so the targets were
  fixed before any held-out outcome existed.
A cluster job cannot ask git (a rented box has no checkout of the study repo): the submit side checks git and pins
the file's sha256 into the job's argv (``--targets-sha256``); the box then checks the file it got against that hash.
A target is ``{contrast: A - B, metric: M, direction: lower|higher|not lower|not higher}``; `crane study report`
gives each a verdict on the held-out rows once they are done.
"""

import json
import os
from pathlib import Path
import subprocess
import time

DIRECTIONS = ('lower', 'higher', 'not lower', 'not higher')


def _git(where, *argv):
    try:
        r = subprocess.run(['git', '-C', str(where), *argv], capture_output=True, text=True, check=False, timeout=30)
    except (OSError, subprocess.TimeoutExpired):
        return None
    return r.stdout.strip() if r.returncode == 0 else None


def _rel(study, path):
    """The targets path as the spec names it (relative to the spec's dir): what outputs record, never /home/..."""
    rel = os.path.relpath(path, study.base)
    return Path(rel).as_posix() if not rel.startswith('..') else Path(path).name


def read_targets(study, path):
    """(entries, None) when the file parses to targets naming the study's arms; else (None, the one-line reason)."""
    import yaml
    name = Path(path).name
    try:
        raw = yaml.safe_load(Path(path).read_text())
    except (OSError, yaml.YAMLError) as error:
        return None, f'targets file {name} does not parse ({type(error).__name__}); fix: a YAML list of targets'
    raw = raw.get('targets') if isinstance(raw, dict) else raw
    if not isinstance(raw, list) or not raw:
        return None, (f'targets file {name} holds no targets; fix: a list of {{contrast: A - B, metric: M, '
                      f'direction: {"|".join(DIRECTIONS)}}}')
    entries = []
    for i, t in enumerate(raw):
        where = f'targets file {name} entry {i}'
        if not isinstance(t, dict) or not {'contrast', 'metric', 'direction'} <= set(t):
            return None, f'{where} needs contrast, metric and direction; fix: {{contrast: A - B, metric: M, direction: lower}}'
        parts = [p.strip() for p in str(t['contrast']).split(' - ')]
        if len(parts) != 2 or not all(parts):
            return None, f'{where}: contrast {t["contrast"]!r}; fix: two arm names, A - B'
        unknown = [p for p in parts if p not in study.arms]
        if unknown:
            return None, f'{where}: no arm {unknown[0]!r}; fix: one of {", ".join(study.arms)}'
        if t['direction'] not in DIRECTIONS:
            return None, f'{where}: direction {t["direction"]!r}; fix: one of {", ".join(DIRECTIONS)}'
        entries.append({'contrast': f'{parts[0]} - {parts[1]}', 'a': parts[0], 'b': parts[1],
                        'metric': str(t['metric']), 'direction': t['direction']})
    return entries, None


def _file(study):
    """(path, None) or (None, reason) for the spec's targets file."""
    if study.targets is None:
        return None, f'{study.path.name} names no targets file; fix: add targets: PATH (committed before the run)'
    path = Path(study.targets)
    if not path.is_file():
        return None, f'targets file {_rel(study, path)} is missing; fix: write it, git add {path.name} && git commit'
    return path, None


def targets_state(study, before=None):
    """({path, commit, committed_at, sha256, entries}, None) when the targets file parses, is committed, clean and
    last committed before ``before`` (an epoch; default now); else (None, the one-line reason with the fix)."""
    from crane.ids import sha256
    path, reason = _file(study)
    if reason:
        return None, reason
    entries, reason = read_targets(study, path)
    if reason:
        return None, reason
    where = path.parent
    if _git(where, 'rev-parse', '--show-toplevel') is None:
        return None, (f'targets file {path.name} is not in a git repository; fix: commit it in the study repo '
                      f'(a cluster job gets --targets-sha256 from crane study compile instead)')
    log = _git(where, 'log', '-1', '--format=%H %ct', '--', path.name)
    if not log:
        return None, f'targets file {path.name} is not committed; fix: git add {path.name} && git commit'
    if _git(where, 'status', '--porcelain', '--', path.name):
        return None, f'targets file {path.name} has uncommitted changes; fix: git commit {path.name}, or restore it'
    commit, ct = log.split()
    if before is None and int(ct) >= time.time():
        return None, f'targets file {path.name} was committed at or after now ({int(ct)}); fix: wait, then rerun'
    if before is not None and int(ct) >= before:
        return None, (f'targets file {path.name} was committed ({int(ct)}) after this output\'s first held-out unit '
                      f'({int(before)}); fix: restore the earlier targets, or run into a new --out')
    return {'path': _rel(study, path), 'commit': commit, 'committed_at': int(ct),
            'sha256': sha256(path.read_bytes()), 'entries': entries}, None


def pinned_state(study, digest):
    """The box side: the targets file must hash to ``digest`` (what the submit side checked in git) and parse."""
    from crane.ids import sha256
    path, reason = _file(study)
    if reason:
        return None, reason
    got = sha256(path.read_bytes())
    if got != digest.removeprefix('sha256:'):
        return None, (f'targets file {path.name} has sha256 {got[:12]} but the job pinned {digest[:12]}; fix: ship '
                      f'the committed targets file with the spec, or recompile the jobs')
    entries, reason = read_targets(study, path)
    if reason:
        return None, reason
    return {'path': _rel(study, path), 'commit': None, 'committed_at': None, 'sha256': got, 'entries': entries,
            'pinned': True}, None


def first_touch(out):
    """(epoch, sha256) of this output's first held-out entry (OUT/held_out.jsonl), else (None, None)."""
    from datetime import datetime
    path = Path(out) / 'held_out.jsonl' if out else None
    if path is None or not path.is_file():
        return None, None
    for line in path.read_text().splitlines():
        try:
            entry = json.loads(line)
            return datetime.fromisoformat(entry['at']).timestamp(), (entry.get('targets') or {}).get('sha256')
        except (ValueError, KeyError, TypeError):
            continue
    return None, None


def refuse(study, cells, confirm, pinned=None, out=None, fix=None):
    """(None, targets state) when the cells may run, else (the one-line refusal, None). Cells with no held-out role
    pass with (None, None). ``fix`` is the command the refusal suggests (default the `crane study run` one)."""
    held = sorted({c.block if c.split is None or study.splits.get(c.split) != 'held-out' else c.split
                   for c in cells if c.held_out})
    if not held:
        return None, None
    names = ', '.join(held)
    if not confirm:
        fix = fix or f'crane study run {study.ref} --block {held[0]} --confirm'
        return f'held-out {names} needs --confirm (and a committed targets file); fix: {fix}', None
    before, first_sha = first_touch(out)
    state, reason = pinned_state(study, pinned) if pinned else targets_state(study, before)
    if reason:
        return f'held-out {names}: {reason}', None
    if first_sha and state['sha256'] != first_sha:
        return (f'held-out {names}: the targets file changed since this output\'s first held-out unit (sha256 '
                f'{first_sha[:12]}); fix: restore it, or run into a new --out'), None
    return None, state
