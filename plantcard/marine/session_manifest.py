"""The boat session manifest (``manifest.json`` beside a session's bag, schema ``mhseals.session_manifest/1``).

The boat TUI writes it from its session journal (mhseals_testing ``annotate/manifest.py`` is the reference):
operator marks and tags, bad-data spans, interventions, link holds, the operator's water conditions
(wind, waves, current: 0 unset, 1 calm .. 3 rough), steady-command calibration candidates and language
commands. Times are wall seconds, the bag's log-time axis. Stdlib only; another major version is refused.
"""

import json
from pathlib import Path

SCHEMA = 'mhseals.session_manifest/1'
FILE = 'manifest.json'


def load(path):
    """The manifest in ``path`` (a session folder or the file); ValueError on another schema."""
    path = Path(path)
    path = path / FILE if path.is_dir() else path
    doc = json.loads(path.read_text())
    if doc.get('schema') != SCHEMA:
        raise ValueError(f'{path}: schema {doc.get("schema")!r} is not {SCHEMA}')
    return doc


def find(path, up=4):
    """The manifest of the session holding ``path`` (searching ``up`` parents), or None."""
    for folder in [Path(path)] + list(Path(path).parents)[:up]:
        if (folder / FILE).is_file():
            try:
                return load(folder)
            except (OSError, ValueError):
                return None
    return None


def excluded(doc):
    """``(t0, t1, why)``: bad data, interventions, link holds and invalid-tag spans a fit must skip."""
    out = []
    for s in doc.get('segments', ()):
        if s.get('status') == 'rejected':
            continue
        if s['kind'] in ('bad', 'intervention', 'link'):
            out.append((s['t0'], max(s['t1'], s['t0'] + 1.0), s['kind']))
        elif s['kind'] == 'mark' and s.get('effect') == 'invalid':
            out.append((*s.get('span', (s['t0'], s['t1'])), s.get('label', 'mark')))
    return sorted(out)


def overlaps(doc, t0, t1):
    """The exclusion reasons overlapping [t0, t1]; empty when the window is clean."""
    return sorted({why for a, b, why in excluded(doc) if a < t1 and b > t0})


def conditions_at(doc, t):
    """The operator's scales in force at ``t`` ({} when none were set)."""
    rows = doc.get('conditions') or []
    before = [c for c in rows if c['t'] <= t] or rows[:1]
    return {k: v for k, v in before[-1].items() if k != 't'} if before else {}


def calibration(doc, allow=('ok', 'degraded')):
    """The steady and neutral windows the session offers for fitting, not rejected and of an allowed quality."""
    return [w for w in doc.get('calibration', ()) if w.get('status') != 'rejected' and w.get('quality') in allow]
