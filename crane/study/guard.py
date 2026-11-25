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
A target is one of (`crane study report` gives each a verdict on the held-out rows once they are done):
- ``{contrast: A - B, metric: M, direction: lower|higher|not lower|not higher}``;
- ``{contrast: A - B, metric: M, expect: {interval: [lo, hi]} | {threshold: X, direction: above|below}}``, with
  ``relative: true`` for (A - B) / mean(B);
- ``{gap_closed: {fixed: A, treated: B, reference: C}, metric: M, min: 0.7}``: (A - B) / (A - C) per seed;
- ``{share: {condition: TARGET, at_least: 0.8}}``: the share of cells where the contrast TARGET is met;
- ``{targets_fn: script.py:fn}``: ``fn(held-out rows) -> verdicts``. The script (relative to the targets file) is
  held like the targets file: committed, clean, older than the first held-out unit, and its sha256 is part of the
  pinned targets sha256 (and recorded on the entry).

What may be shown while work runs is decided here too, for every reader (crane study status, crane monitor, a
job's metrics.json, crane runs, crane data export):

    guard.is_held_out(role)       # held-out, heldout, hold-out, held_out ...
    guard.shows_means(role)       # interim per-arm means: dev blocks only; every other role shows progress only
    guard.shows_outcomes(role)    # a failure's message and traceback while work runs: dev and check blocks only
    guard.sealed(role, done)      # a held-out record that is not done: no outcome, no metrics
"""

import json
import os
from pathlib import Path
import subprocess
import time

DIRECTIONS = ('lower', 'higher', 'not lower', 'not higher')
SIDES = ('above', 'below')
VERDICTS = ('met', 'not met', 'inconclusive', 'no data')
HELD_OUT = 'held-out'
MEANS = frozenset({'dev', 'check'})     # interim per-arm means: dev and check blocks (the studies' rule); never held-out
OUTCOMES = frozenset({'dev', 'check'})  # per-unit outcome detail (failure messages) shows while running; train,
                                        # calib, held-out and a role-less block hide it by default


def norm_role(role):
    """A block role in its one spelling (held_out, heldout, hold-out ... are held-out); None stays None."""
    if role is None or str(role).strip() == '':
        return None
    r = str(role).strip().lower().replace('_', '-')
    return HELD_OUT if r in ('held-out', 'heldout', 'holdout', 'hold-out') else r


def is_held_out(role):
    return norm_role(role) == HELD_OUT


def shows_means(role):
    """Whether interim per-arm means (monitor, metrics.json, study status) of a block with this role may be shown."""
    return norm_role(role) in MEANS


def shows_outcomes(role):
    """Whether a running block's per-unit outcome detail (a failure's message and traceback) may be shown."""
    return norm_role(role) in OUTCOMES


def sealed(role, done):
    """Whether a record's outcome and metrics stay hidden: a held-out one until its work is done."""
    return is_held_out(role) and not done


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


def _number(v):
    import math
    return isinstance(v, (int, float)) and not isinstance(v, bool) and math.isfinite(v)


def _arm(study, where, name):
    return None if str(name) in study.arms else f'{where}: no arm {name!r}; fix: one of {", ".join(study.arms)}'


def _contrast(study, t, where):
    """(entry, None) of a contrast target, or (None, reason)."""
    if not isinstance(t, dict) or not {'contrast', 'metric'} <= set(t) or ('direction' in t) == ('expect' in t):
        return None, (f'{where} needs contrast, metric and direction or expect; fix: {{contrast: A - B, metric: M, '
                      f'direction: lower}} or {{..., expect: {{interval: [lo, hi]}}}}')
    bad = sorted(set(t) - {'contrast', 'metric', 'direction', 'expect', 'relative', 'name'})
    if bad:
        return None, f'{where}: unknown key {bad[0]!r}; fix: contrast, metric, direction | expect, relative, name'
    parts = [p.strip() for p in str(t['contrast']).split(' - ')]
    if len(parts) != 2 or not all(parts):
        return None, f'{where}: contrast {t["contrast"]!r}; fix: two arm names, A - B'
    for part in parts:
        reason = _arm(study, where, part)
        if reason:
            return None, reason
    entry = {'kind': 'contrast', 'contrast': f'{parts[0]} - {parts[1]}', 'a': parts[0], 'b': parts[1],
             'metric': str(t['metric']), 'direction': t.get('direction')}
    if 'direction' in t and t['direction'] not in DIRECTIONS:
        return None, f'{where}: direction {t["direction"]!r}; fix: one of {", ".join(DIRECTIONS)}'
    if 'expect' in t:
        e = t['expect']
        if isinstance(e, dict) and set(e) == {'interval'} and isinstance(e['interval'], list) \
                and len(e['interval']) == 2 and all(_number(x) for x in e['interval']) \
                and e['interval'][0] <= e['interval'][1]:
            entry['expect'] = {'interval': [float(x) for x in e['interval']]}
        elif isinstance(e, dict) and set(e) == {'threshold', 'direction'} and _number(e['threshold']) \
                and e['direction'] in SIDES:
            entry['expect'] = {'threshold': float(e['threshold']), 'direction': e['direction']}
        else:
            return None, (f'{where}: expect {e!r}; fix: {{interval: [lo, hi]}} or {{threshold: X, direction: '
                          f'above|below}}')
    if t.get('relative'):
        entry['relative'] = True
    if t.get('name'):
        entry['name'] = str(t['name'])
    return entry, None


def _entry(study, t, where, folder):
    """(entry, None) of one target of any kind, or (None, the one-line reason)."""
    from crane.ids import sha256
    if isinstance(t, dict) and 'targets_fn' in t:
        ref = t['targets_fn']
        target, sep, attr = str(ref).rpartition(':')
        if set(t) - {'targets_fn', 'name'} or not sep or not target.endswith('.py') or not attr:
            return None, f'{where}: targets_fn {ref!r}; fix: {{targets_fn: script.py:fn}} (a file next to the targets)'
        path = (Path(folder) / target).resolve()
        if not path.is_file():
            return None, f'{where}: targets_fn file {target} is missing; fix: write it, git add {target} && git commit'
        return {'kind': 'fn', 'contrast': str(t.get('name') or ref), 'metric': '-', 'fn': str(ref),
                'path': _rel(study, path), 'attr': attr, 'sha256': sha256(path.read_bytes()),
                'file': str(path)}, None
    if isinstance(t, dict) and 'gap_closed' in t:
        g = t['gap_closed']
        if not isinstance(g, dict) or set(g) != {'fixed', 'treated', 'reference'} or 'metric' not in t \
                or not _number(t.get('min')) or set(t) - {'gap_closed', 'metric', 'min', 'name'}:
            return None, (f'{where} needs gap_closed: {{fixed, treated, reference}}, metric and min; fix: '
                          f'{{gap_closed: {{fixed: A, treated: B, reference: C}}, metric: M, min: 0.7}}')
        for k in ('fixed', 'treated', 'reference'):
            reason = _arm(study, f'{where}.gap_closed.{k}', g[k])
            if reason:
                return None, reason
        arms = {k: str(g[k]) for k in ('fixed', 'treated', 'reference')}
        return {'kind': 'gap_closed', 'contrast': str(t.get('name') or
                                                      f"gap {arms['fixed']}>{arms['treated']} of >{arms['reference']}"),
                'metric': str(t['metric']), 'arms': arms, 'min': float(t['min'])}, None
    if isinstance(t, dict) and 'share' in t:
        sh = t['share']
        ok = isinstance(sh, dict) and set(sh) == {'condition', 'at_least'} and _number(sh['at_least']) \
            and 0 < sh['at_least'] <= 1 and not set(t) - {'share', 'name'}
        if not ok:
            return None, (f'{where} needs share: {{condition: TARGET, at_least: F}}; fix: {{share: {{condition: '
                          f'{{contrast: A - B, metric: M, direction: lower}}, at_least: 0.8}}}}')
        cond, reason = _contrast(study, sh['condition'], f'{where}.share.condition')
        if reason:
            return None, reason
        return {'kind': 'share', 'contrast': str(t.get('name') or cond['contrast']), 'metric': cond['metric'],
                'condition': cond, 'at_least': float(sh['at_least'])}, None
    return _contrast(study, t, where)


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
        entry, reason = _entry(study, t, f'targets file {name} entry {i}', Path(path).parent)
        if reason:
            return None, reason
        entries.append(entry)
    return entries, None


def pin_sha(path, entries):
    """The targets' sha256: the file's, or with targets_fn scripts, over the file and each script's sha256."""
    import hashlib
    data = Path(path).read_bytes()
    fns = sorted((e['path'], e['sha256']) for e in entries if e.get('kind') == 'fn')
    if not fns:
        return hashlib.sha256(data).hexdigest()
    h = hashlib.sha256(data)
    for rel, sha in fns:
        h.update(b'\0' + rel.encode() + b'\0' + sha.encode())
    return h.hexdigest()


def _public(entries):
    return [{k: v for k, v in e.items() if k != 'file'} for e in entries]


def _committed(path, before, what):
    """((commit, committed_at), None) when ``path`` is committed, clean and last committed before ``before`` (an
    epoch; default now), else (None, the one-line reason)."""
    where, name = path.parent, path.name
    if _git(where, 'rev-parse', '--show-toplevel') is None:
        return None, (f'{what} {name} is not in a git repository; fix: commit it in the study repo '
                      f'(a cluster job gets --targets-sha256 from crane study compile instead)')
    log = _git(where, 'log', '-1', '--format=%H %ct', '--', name)
    if not log:
        return None, f'{what} {name} is not committed; fix: git add {name} && git commit'
    if _git(where, 'status', '--porcelain', '--', name):
        return None, f'{what} {name} has uncommitted changes; fix: git commit {name}, or restore it'
    commit, ct = log.split()
    if before is None and int(ct) >= time.time():
        return None, f'{what} {name} was committed at or after now ({int(ct)}); fix: wait, then rerun'
    if before is not None and int(ct) >= before:
        return None, (f'{what} {name} was committed ({int(ct)}) after this output\'s first held-out unit '
                      f'({int(before)}); fix: restore the earlier targets, or run into a new --out')
    return (commit, int(ct)), None


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
    last committed before ``before`` (an epoch; default now), and so is each targets_fn script; else (None, the
    one-line reason with the fix). ``sha256`` is pin_sha: the file's own when it names no script."""
    path, reason = _file(study)
    if reason:
        return None, reason
    entries, reason = read_targets(study, path)
    if reason:
        return None, reason
    got, reason = _committed(path, before, 'targets file')
    if reason:
        return None, reason
    for e in entries:
        if e['kind'] == 'fn':
            fn_got, reason = _committed(Path(e['file']), before, 'targets_fn file')
            if reason:
                return None, reason
            e['commit'] = fn_got[0]
    commit, ct = got
    return {'path': _rel(study, path), 'commit': commit, 'committed_at': ct,
            'sha256': pin_sha(path, entries), 'entries': _public(entries)}, None


def pinned_state(study, digest):
    """The box side: the targets file (and its targets_fn scripts) must hash to ``digest`` (what the submit side
    checked in git) and parse."""
    path, reason = _file(study)
    if reason:
        return None, reason
    entries, reason = read_targets(study, path)
    got = pin_sha(path, entries or [])
    if got != digest.removeprefix('sha256:'):
        return None, (f'targets file {path.name} has sha256 {got[:12]} but the job pinned {digest[:12]}; fix: ship '
                      f'the committed targets file with the spec, or recompile the jobs')
    if reason:
        return None, reason
    return {'path': _rel(study, path), 'commit': None, 'committed_at': None, 'sha256': got,
            'entries': _public(entries), 'pinned': True}, None


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
