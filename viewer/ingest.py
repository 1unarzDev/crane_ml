"""Runs -> a board data root: ``python -m viewer ingest SRC... --out DATA [--study S] [--arm A]``.

SRC is a crane.run/1 record directory, a bare ``.ctraj``, or a directory searched for both. Each run lands as
``DATA/runs/<id>/run.ctraj`` (a record composed as the board serves it: meta, layers, ``events.jsonl`` merged
on lanes) with its overview LOD beside it and ``study``/``arm`` stamped in the header; a course truth it names
is copied to ``DATA/courses/<sha>.json``. The board can also serve the sources in place (``crane board SRC``);
ingest makes a self-contained copy (to keep, or to hand to the cluster's archive).
"""

from __future__ import annotations

import json
from pathlib import Path

from theater import traj as ctraj

from .store import Store


def _safe(run_id: str) -> str:
    return ''.join(c if c.isalnum() or c in '._-' else '_' for c in run_id)


def ingest(srcs, out, study: str | None = None, arm: str | None = None, log=print) -> int:
    """Every run under ``srcs`` -> ``out``. Returns the runs written."""
    out = Path(out).expanduser()
    store = Store([Path(s).expanduser() for s in srcs])
    n = 0
    for rid, run in sorted(store.runs.items()):
        dst = out / 'runs' / _safe(rid) / 'run.ctraj'
        dst.parent.mkdir(parents=True, exist_ok=True)
        dst.write_bytes(run.data())
        stamp = {k: v for k, v in (('study', study), ('arm', arm)) if v}
        if stamp:
            ctraj.amend(dst, **stamp)
        ctraj.write_lod(dst)
        sha = (run.header.get('course') or {}).get('truth_sha256')
        if sha and sha in store.courses:
            (out / 'courses').mkdir(parents=True, exist_ok=True)
            c = out / 'courses' / f'{sha}.json'
            if not c.exists():
                c.write_text(json.dumps(json.loads(store.courses[sha].read_text()), separators=(',', ':')))
        n += 1
        log(f'{rid}: {run.header.get("tier") or ""} {run.header.get("outcome") or ""}'.rstrip())
    return n
