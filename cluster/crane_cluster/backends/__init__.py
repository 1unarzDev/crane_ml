"""Backend interface and registry.

A backend turns one job into runs: ``prepare(job)`` once, then ``run(seed)``
per seed returning a run directory, then ``metrics(run_dir)`` for the numbers
the coordinator stores. Backends are named in the worker's YAML::

    backends:
      dummy:  {type: dummy}
      fast:   {type: command, image: "ghcr.io/org/sim:latest", command: "sim --seed {seed} --out {run_dir}"}
      domain: {type: "my_pkg.sim_backend:MyBackend", some_option: 3}

``type`` is a registered name (``dummy``, ``command``) or ``module:Class`` for a
backend that lives in another package (a domain adapter), so the core never
imports domain code. ``path`` (a directory or a list) is put on ``sys.path``
before that import, for a package that is a checkout rather than installed.
"""
from __future__ import annotations

import importlib
import json
import os
import sys
from dataclasses import dataclass, field
from pathlib import Path

_TYPES: dict[str, type] = {}


def register(type_name: str):
    def deco(cls):
        _TYPES[type_name] = cls
        return cls
    return deco


@dataclass
class JobContext:
    """Everything a backend may use about the job it is running."""
    job: dict                      # the coordinator's job (job_id, arm, spec, seeds, needs, ...)
    work_dir: Path                 # private scratch directory for this job
    repos: dict[str, Path] = field(default_factory=dict)       # repo name -> checkout at the exact ref
    resolved: dict[str, str] = field(default_factory=dict)     # repo name -> commit sha checked out
    isolation: dict = field(default_factory=dict)              # slot, port_base, isolation_id
    env: dict[str, str] = field(default_factory=dict)          # extra environment for child processes
    client: object | None = None                               # the worker's coordinator client (job inputs)

    @property
    def spec(self) -> dict:
        return self.job.get("spec") or {}


class Backend:
    """Base class. Subclasses override prepare/run/metrics; the rest has defaults."""

    def __init__(self, name: str, options: dict):
        self.name = name
        self.options = options
        self.ctx: JobContext | None = None

    def update(self) -> None:
        """Between jobs only: refresh images or tools. Never called while a job runs."""

    def prepare(self, ctx: JobContext) -> None:
        self.ctx = ctx

    def run(self, seed: int) -> Path:
        raise NotImplementedError

    def metrics(self, run_dir: Path) -> dict[str, float]:
        """Default: the flat JSON object in run_dir/metrics.json (numbers only)."""
        p = Path(run_dir) / self.options.get("metrics_file", "metrics.json")
        data = json.loads(p.read_text())
        return {k: float(v) for k, v in data.items() if isinstance(v, (int, float)) and not isinstance(v, bool)}

    def artifacts(self, run_dir: Path) -> list[tuple[Path, str, str]]:
        """(path, name, retention) to upload: summary.json and metrics.json as 'summary', files under artifacts/ as 'raw'."""
        run_dir = Path(run_dir)
        out = [(run_dir / n, n, "summary") for n in ("summary.json", "metrics.json") if (run_dir / n).is_file()]
        raw = run_dir / "artifacts"
        if raw.is_dir():
            out += [(p, p.relative_to(run_dir).as_posix(), "raw") for p in sorted(raw.rglob("*")) if p.is_file()]
        return out

    def cancel(self) -> None:
        """Stop the current run as soon as possible (lease revoked, user active, shutdown)."""

    def finish(self) -> None:
        """After the job's last run, success or not."""

    def run_dir(self, seed: int) -> Path:
        d = self.ctx.work_dir / "runs" / f"seed-{seed}"
        d.mkdir(parents=True, exist_ok=True)
        return d


def backend_class(type_name: str) -> type:
    if ":" in type_name:
        mod, _, cls = type_name.partition(":")
        return getattr(importlib.import_module(mod), cls)
    if type_name not in _TYPES:
        importlib.import_module(f"{__name__}.{type_name}")  # built-ins register on import
    return _TYPES[type_name]


def build(config: dict) -> dict[str, Backend]:
    """Backends by name from the worker config's ``backends`` mapping."""
    out = {}
    for name, opts in (config or {}).items():
        opts = dict(opts or {})
        paths = opts.pop("path", [])
        for p in reversed([paths] if isinstance(paths, str) else paths):
            p = os.path.expanduser(p)
            if p not in sys.path:
                sys.path.insert(0, p)
        out[name] = backend_class(opts.pop("type", name))(name, opts)
    return out
