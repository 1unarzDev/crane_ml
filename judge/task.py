"""TaskSpec: what a run was trying to do and how its objectives are reported.

Objectives are a vector, never summed: safety objectives (Properties) are reported first and
lexicographically, then task completion, then efficiency, then diagnostics. A non-inferiority
claim needs a declared margin in the metric's units; without one the comparator reports the
interval and says the margin is undeclared.
"""

from __future__ import annotations

from dataclasses import dataclass, field
from typing import Any, Mapping, Optional

ROLES = ('safety', 'task', 'efficiency', 'diagnostic')


@dataclass(frozen=True)
class Objective:
    metric: str
    direction: str = 'higher'          # 'higher' | 'lower'
    role: str = 'task'
    ni_margin: Optional[float] = None  # in metric units; positive number = tolerated loss
    margin_cite: str = ''              # who set it / where (owner decision, rulebook)
    equivalence: Optional[tuple] = None
    band: Optional[float] = None       # G4: allowed |low - high| tier gap in metric units

    def __post_init__(self):
        if self.role not in ROLES:
            raise ValueError(f'{self.metric}: role {self.role!r}')
        if self.direction not in ('higher', 'lower'):
            raise ValueError(f'{self.metric}: direction {self.direction!r}')


@dataclass(frozen=True)
class Phase:
    name: str
    start: str                         # event kind that opens it
    end: str = ''                      # event kind that closes it ('' = run end)


@dataclass(frozen=True)
class TaskSpec:
    id: str
    version: str = '1'
    domain: str = ''
    objectives: tuple = ()
    phases: tuple = ()
    params: Mapping[str, Any] = field(default_factory=dict)

    def ordered(self):
        """Objectives in report order: safety, task, efficiency, diagnostic."""
        return sorted(self.objectives, key=lambda o: ROLES.index(o.role))

    def objective(self, metric):
        return next((o for o in self.objectives if o.metric == metric), None)
