"""``dc-run/1``: one JSONL row per run, shared by every DecisionCal study (and meant to carry over to
physical sessions). Evidence labels are never merged:

* SIM     -- simulated, same model class as the twin
* HSUR    -- held-out surrogate: hidden plants from a different model class or implementation
* PREPLAY -- physical log replay
* CLPHYS  -- new closed-loop physical trials

Heavy tick streams (requested vs applied commands, acquisition/arrival stamps, truth) go to an
``.npz`` next to the JSONL and are referenced by path + sha256. Evaluator truth is never an input
to a controller; a row says which channel each field came from.
"""

import hashlib
import json
from pathlib import Path

import numpy as np

SCHEMA = 'dc-run/1'
LABELS = ('SIM', 'HSUR', 'PREPLAY', 'CLPHYS')
TERMINATIONS = ('DOCKED', 'TIMEOUT', 'CONTACT_ABORT', 'GATE_MISS', 'ABSTAIN', 'ERROR')
REQUIRED = ('schema', 'run_id', 'study', 'evidence_label', 'plant', 'controller', 'seed', 'termination', 'metrics',
            'budget', 'code')


def validate(row):
    for k in REQUIRED:
        if k not in row:
            raise ValueError(f'dc-run/1: missing {k!r}')
    if row['schema'] != SCHEMA:
        raise ValueError(f'dc-run/1: schema {row["schema"]!r}')
    if row['evidence_label'] not in LABELS:
        raise ValueError(f'dc-run/1: evidence_label {row["evidence_label"]!r} not in {LABELS}')
    if row['termination'] not in TERMINATIONS:
        raise ValueError(f'dc-run/1: termination {row["termination"]!r}')
    for k in ('id', 'class'):
        if k not in row['plant']:
            raise ValueError(f'dc-run/1: plant.{k} missing')
    json.dumps(row, allow_nan=False)
    return row


def _clean(v):
    if isinstance(v, dict):
        return {k: _clean(x) for k, x in v.items()}
    if isinstance(v, (list, tuple)):
        return [_clean(x) for x in v]
    if isinstance(v, (np.floating, float)):
        return None if not np.isfinite(v) else float(v)
    if isinstance(v, (np.integer,)):
        return int(v)
    if isinstance(v, np.bool_):
        return bool(v)
    if isinstance(v, np.ndarray):
        return _clean(v.tolist())
    return v


def write(fp, row):
    row = validate(_clean(row))
    fp.write(json.dumps(row, sort_keys=True, allow_nan=False) + '\n')


def save_streams(path, streams):
    path = Path(path)
    np.savez_compressed(path, **{k: np.asarray(v) for k, v in streams.items()})
    return {'path': path.name, 'sha256': hashlib.sha256(path.read_bytes()).hexdigest()}
