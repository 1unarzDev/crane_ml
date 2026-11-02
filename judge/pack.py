"""Packs: a domain's metrics, event vocabulary, segmenter, dependency model and failure taxonomy.

The core never imports a pack; packs are passed in or named by plugin reference
(``package.module:attr`` or ``path/file.py:attr``), the same form fastsim uses."""

from __future__ import annotations

from dataclasses import dataclass, field
import importlib
import importlib.util
import os
from pathlib import Path
import sys
from typing import Any, Callable, Mapping, Optional, Sequence

from judge.metric import Metric, MetricValue, unscorable
from judge.record import RunRecord


@dataclass(frozen=True)
class Node:
    """One stage of a dependency model. ``check(rec)`` returns anomalies [(t, what)]."""
    name: str
    upstream: tuple = ()
    check: Optional[Callable[[RunRecord], list]] = None
    channels: tuple = ()


@dataclass(frozen=True)
class DepGraph:
    id: str
    nodes: tuple                       # Node, any order; edges via upstream

    def node(self, name):
        return next(n for n in self.nodes if n.name == name)

    def ancestors(self, name):
        """Upstream nodes of ``name``, nearest first (breadth first)."""
        out, frontier = [], [name]
        while frontier:
            nxt = []
            for f in frontier:
                for u in self.node(f).upstream:
                    if u not in out:
                        out.append(u)
                        nxt.append(u)
            frontier = nxt
        return out


@dataclass(frozen=True)
class HintRule:
    """``when(insights_by_kind, context) -> text | None``: a pack's actionable next step."""
    id: str
    when: Callable[..., Optional[str]]


@dataclass(frozen=True)
class Pack:
    name: str
    domain: str
    metrics: Mapping[str, Metric] = field(default_factory=dict)
    event_vocab: Mapping[str, str] = field(default_factory=dict)   # kind -> description
    decision_events: tuple = ()        # kinds that are decisions (for the event-timeline diff)
    outcome: Optional[Callable[[RunRecord, Mapping[str, MetricValue]], str]] = None  # failure mode
    segmenter: Optional[Callable] = None   # (rec) -> [(t0, t1, label)]
    dependency: Optional[DepGraph] = None
    failure_node: Optional[Callable] = None  # (rec, values) -> node name where the failure shows
    taxonomy: Mapping[str, str] = field(default_factory=dict)
    hints: Sequence[HintRule] = ()
    replay: Optional[Any] = None       # decomposition replayer (see insight.divergence.Replayer)


def evaluate(rec: RunRecord, task=None, packs=()) -> dict:
    """{metric name: MetricValue} for every metric of every pack (name clashes: first wins)."""
    out = {}
    for pack in packs:
        for name, m in pack.metrics.items():
            if name in out:
                continue
            try:
                out[name] = m(rec, task)
            except Exception as e:  # a metric bug never sinks a report; it is reported as such
                out[name] = unscorable(f'metric error {type(e).__name__}: {e}')
    return out


def outcome_of(rec, values, packs):
    for p in packs:
        if p.outcome:
            return p.outcome(rec, values)
    return 'unknown'


def load(ref, base=None):
    """The object a plugin reference names (``module:attr`` or ``file.py:attr``)."""
    if not isinstance(ref, str):
        return ref
    target, sep, attr = ref.rpartition(':')
    if not sep or not target or not attr:
        raise ValueError(f'plugin reference {ref!r}: expected MODULE:ATTR or FILE.py:ATTR')
    if target.endswith('.py'):
        path = Path(os.path.expandvars(target)).expanduser()
        if not path.is_absolute() and base is not None:
            path = Path(base) / path
        name = f'_judge_plugin_{abs(hash(str(path.resolve())))}'
        module = sys.modules.get(name)
        if module is None:
            spec = importlib.util.spec_from_file_location(name, path)
            module = importlib.util.module_from_spec(spec)
            sys.modules[name] = module
            spec.loader.exec_module(module)
    else:
        module = importlib.import_module(target)
    obj = module
    for part in attr.split('.'):
        obj = getattr(obj, part)
    return obj
