"""crane judge: robot- and tier-agnostic run evaluation that explains, not only scores.

A run from any tier (fast sim, Unity, a real robot) becomes one ``RunRecord`` (typed channels with
per-channel observability and sigma, typed events, validity). Domain packs add metrics; every metric
returns a value *with a status* (ok, provisional, unscorable, indeterminate) and evidence spans.
The comparator measures paired variants and cross-tier agreement (trust gates G3/G4), and the
insight generators say where and why runs diverge. The core imports no domain: a robot is a
Source plus a Pack (see ``packs/toy_quad`` for a complete second domain).

  record    RunRecord, Channel, Event, Header, Validity; deterministic save/load
  task      TaskSpec, Objective (safety first, never summed)
  metric    Metric, Property (three-valued robustness with bounds e and L), MetricValue, Status
  pack      Pack, evaluate, plugin references
  source    Source, FidelityProfile, Refused
  pairing   exact / block / loose pairing with the design label carried along
  stats     one statistics module (Wilson, Newcombe, McNemar, Wilcoxon, t, bootstrap, Holm, ...)
  compare   paired comparisons, tier agreement (G3 regret/MMRV/top-k, G4 gap bands)
  insight   integrity, divergence decomposition, event diff, failure clusters, first cause, hints
  render    report JSON (crane.judge.report/1), one-line verdicts, Markdown
"""

from judge.record import (Channel, ChannelSpec, Clock, Event, Header, Robot, RunRecord, Scenario,
                          Unobservable, Validity)
from judge.metric import MetricValue, Property, Span, Status, metric
from judge.task import Objective, TaskSpec
from judge.pack import Pack, evaluate
from judge.source import FidelityProfile, Refused, Source

SCHEMA_RUN = 'crane.judge.run/1'
SCHEMA_REPORT = 'crane.judge.report/1'

__all__ = ['Channel', 'ChannelSpec', 'Clock', 'Event', 'FidelityProfile', 'Header', 'MetricValue',
           'Objective', 'Pack', 'Property', 'Refused', 'Robot', 'RunRecord', 'Scenario', 'Source',
           'Span', 'Status', 'TaskSpec', 'Unobservable', 'Validity', 'evaluate', 'metric',
           'SCHEMA_RUN', 'SCHEMA_REPORT']
