"""crane judge: robot- and tier-agnostic run evaluation that explains, not only scores.

A run from any tier (fast sim, Unity, a real robot) becomes one ``RunRecord`` (typed channels with
per-channel observability and sigma, typed events, validity). Domain packs add metrics; every metric
returns a value *with a status* (ok, provisional, unscorable, indeterminate) and evidence spans.
The comparator measures paired variants and cross-tier agreement (trust gates G3/G4), and the
insight generators say where and why runs diverge. The core imports no domain: a robot is a
Source plus a Pack (see ``packs/toy_quad`` for a complete second domain).

  record    RunRecord, Channel, Event, Header, Validity; /2 header blocks and standard channels;
            deterministic save/load (/1 still loads)
  validity  check(record, expect) -> ValidityReport (clock, frame, telemetry, reset, command, infra);
            apply() writes the verdict into Validity
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

from judge.record import (EV, END, EVENTS, SCHEMA_V1, SCHEMA_V2, STANDARD, TERMINAL, Backend, Calibration,
                          Channel, ChannelSpec, Clock, ClockMode, Code, Controller, Event, Evidence, Header,
                          ObsIsTruth, Reset, Robot, RunRecord, Scenario, Unobservable, Validity, declare_absent)
from judge.metric import MetricValue, Property, Span, Status, metric
from judge.task import Objective, TaskSpec
from judge.pack import Pack, evaluate
from judge.source import FidelityProfile, Refused, Source

SCHEMA_RUN = SCHEMA_V2
SCHEMA_REPORT = 'crane.judge.report/1'

__all__ = ['Backend', 'Calibration', 'Channel', 'ChannelSpec', 'Clock', 'ClockMode', 'Code', 'Controller',
           'END', 'EV', 'EVENTS', 'Event', 'Evidence', 'FidelityProfile', 'Header', 'MetricValue', 'ObsIsTruth',
           'Objective', 'Pack', 'Property', 'Refused', 'Reset', 'Robot', 'RunRecord', 'Scenario', 'Source',
           'Span', 'STANDARD', 'Status', 'TERMINAL', 'TaskSpec', 'Unobservable', 'Validity', 'declare_absent',
           'evaluate', 'metric', 'SCHEMA_RUN', 'SCHEMA_V1', 'SCHEMA_V2', 'SCHEMA_REPORT']
