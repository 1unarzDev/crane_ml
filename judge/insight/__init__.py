"""Insight generators: deterministic findings with evidence, scope, confidence and a basis.

  integrity    validity flags, missing channels, clock residuals, provenance mismatch (always first)
  divergence   where paired runs part (time and path residuals, CUSUM first divergence) and the
               counterfactual split: plant (open loop) / interaction (reset closed loop) /
               compounding (free run)
  events       event-timeline alignment: the first differing decision and the state residual there
               (small residual + different decision = decision divergence, not dynamics)
  clusters     failure signatures, Jaccard clustering, bootstrap stability (only stable reported)
  cause        first upstream anomaly on the pack's dependency graph (model-relative, labelled)
  hints        pack rules that turn insights into a next experiment
  summarise    the citation validator for any LLM summary (no sentence without insight ids)

Every generator ships with a seeded-fault test (tests/test_insights.py): plant a divergence, find
it, and stay quiet on a clean control.
"""

from __future__ import annotations

from dataclasses import asdict, dataclass, field
import hashlib
import json
from typing import Any, Mapping

SEVERITY = ('info', 'warn', 'fail')


@dataclass(frozen=True)
class Insight:
    kind: str
    severity: str
    claim: str                                 # one templated sentence, numbers included
    evidence: tuple = ()                       # Span / dict refs (metric, event, run)
    scope: Mapping[str, Any] = field(default_factory=dict)
    confidence: Mapping[str, float] = field(default_factory=dict)
    basis: str = 'descriptive'                 # statistical | model-relative:<graph> | descriptive
    next: tuple = ()                           # hints: what to run next
    data: Mapping[str, Any] = field(default_factory=dict)

    @property
    def id(self):
        blob = json.dumps([self.kind, dict(self.scope), self.claim], sort_keys=True, default=str)
        return f'{self.kind}:{hashlib.sha1(blob.encode()).hexdigest()[:8]}'

    def as_dict(self):
        d = asdict(self)
        d['id'] = self.id
        d['evidence'] = [e if isinstance(e, dict) else asdict(e) for e in self.evidence]
        return d


def rank(insights):
    """fail before warn before info; stable otherwise."""
    return sorted(insights, key=lambda i: -SEVERITY.index(i.severity))
