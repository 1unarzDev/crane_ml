"""A run's theater metadata: what goes into a ``.ctraj`` header once the run is scored, and the
``run_meta`` row the coordinator indexes (``GET /v1/runs/query``) so ``crane-theater select``
can pick runs by study, iteration, score and outcome.

Where the fields come from (a job's spec, then the job, then the backend)::

    study       spec.theater.study | spec.study | the job's hypothesis_id | 'adhoc'
    iteration   spec.theater.iteration | spec.iteration | the job's stage_index | 0
    generation  spec.theater.generation | spec.generation | 0
    params      spec.theater.params | spec.params | the backend's parameter set (flattened, dotted)
    score       metrics[spec.theater.score_metric | spec.score_metric | spec.metric]
    insight     spec.theater.insight | spec.insight (one line, written by the tuner, never generated here)

Nothing here knows a robot or a simulator.
"""

from __future__ import annotations

import hashlib
import json
import math

from theater import traj as T

FASTSIM_OUTCOMES = {'SUCCEEDED': 'finished', 'TIMEOUT': 'timeout', 'ERROR': 'failed'}


def flatten(value, prefix: str = '') -> dict:
    """Nested mappings as {dotted.name: leaf}; lists and scalars are leaves."""
    if isinstance(value, dict):
        out = {}
        for k, v in value.items():
            out.update(flatten(v, f'{prefix}.{k}' if prefix else str(k)))
        return out
    return {prefix: value} if prefix else {}


def params_sha256(params: dict) -> str:
    return hashlib.sha256(T.dumps_json(params or {}).encode()).hexdigest()


def _first(*values):
    for v in values:
        if v is not None:
            return v
    return None


def job_fields(job: dict, spec: dict | None = None, *, params=None, metrics=None) -> dict:
    """Header fields from a cluster job (``job`` as the coordinator leases it)."""
    spec = spec if spec is not None else (job.get('spec') or {})
    th = spec.get('theater') if isinstance(spec.get('theater'), dict) else {}
    params = _first(th.get('params'), spec.get('params') if isinstance(spec.get('params'), dict) else None, params) or {}
    params = flatten(params)
    out = {'study': str(_first(th.get('study'), spec.get('study'), job.get('hypothesis_id'), 'adhoc')),
           'iteration': int(_first(th.get('iteration'), spec.get('iteration'), job.get('stage_index'), 0)),
           'generation': int(_first(th.get('generation'), spec.get('generation'), 0)),
           'hypothesis_id': job.get('hypothesis_id'), 'arm': job.get('arm'), 'job_id': job.get('job_id'),
           'params': params, 'params_sha256': params_sha256(params)}
    insight = _first(th.get('insight'), spec.get('insight'))
    if insight:
        out['insight'] = str(insight)[:300]
    metric = _first(th.get('score_metric'), spec.get('score_metric'), spec.get('metric'))
    if metric and metrics and isinstance(metrics.get(metric), (int, float)) and math.isfinite(metrics[metric]):
        out['score'], out['score_metric'] = float(metrics[metric]), metric
    return out


def run_meta(header: dict, *, traj_name: str = 'run.ctraj', traj_sha256: str | None = None) -> dict:
    """The coordinator's run_meta body (POST /v1/jobs/:id/runs ``meta``) from a stamped header."""
    course = header.get('course') or {}
    build = header.get('build') or {}
    keep = ('study', 'iteration', 'generation', 'params', 'params_sha256', 'score', 'outcome', 'failure_reason',
            'progress', 'valid', 'insight', 'seed', 'duration_s')
    out = {k: header[k] for k in keep if header.get(k) is not None}
    out.update(course_sha256=course.get('truth_sha256'), course_family=course.get('family'),
               build_hash=build.get('build_hash'), recorder=build.get('recorder'),
               water_mode=(header.get('water') or {}).get('mode'), traj_name=traj_name, traj_sha256=traj_sha256)
    return {k: v for k, v in out.items() if v is not None}


def stamp(path, **fields) -> tuple:
    """Merge ``fields`` into the file's header; return (header, sha256)."""
    sha = T.amend(path, **fields)
    return T.read_header(path), sha


def write_lod(path):
    """Write ``run.lod.ctraj`` next to a stamped ``run.ctraj`` (after stamping: the LOD copies the
    outcome and names the full file's sha256). A LOD failure never fails the run; returns its sha or None."""
    try:
        return T.write_lod(path)
    except (OSError, T.TrajError, ValueError, KeyError):
        return None


def outcome_of_fastsim(outcome: str) -> tuple:
    """(outcome, failure_reason) from a fastsim run outcome."""
    o = FASTSIM_OUTCOMES.get(outcome)
    return (o, None) if o else ('failed', str(outcome).lower())


def load_json(path):
    try:
        return json.loads(open(path, encoding='utf-8').read())
    except (OSError, ValueError):
        return None
