"""Sources turn a tier's run artifacts into RunRecords. A new robot or simulator is one Source.

The fidelity profile is declared per source, never inferred from its name, so cross-tier reports
can say what each tier does not model."""

from __future__ import annotations

from dataclasses import asdict, dataclass
from pathlib import Path
from typing import Iterator

from judge.record import RunRecord


class Refused(Exception):
    """The artifact cannot become a record; the message says why (kept, counted, never dropped)."""


@dataclass(frozen=True)
class FidelityProfile:
    dynamics: str = 'kinematic'       # kinematic | first_order | physics | real
    perception: str = 'truth'         # truth | truth_cone | rendered | real
    timing: str = 'fixed_step'        # fixed_step | realtime_soft | real
    environment: str = 'none'         # none | parametric | rendered | real
    cost_s_per_run: float = 0.0
    note: str = ''

    def as_dict(self):
        return asdict(self)


class Source:
    """Subclass and set ``name``, ``tier``, ``profile``; implement ``discover`` and ``load``."""
    name = ''
    tier = ''
    version = '1'
    profile = FidelityProfile()

    def discover(self, root) -> Iterator:
        """Run references under ``root`` (paths by default)."""
        yield from sorted(Path(root).iterdir())

    def load(self, ref) -> RunRecord:
        raise NotImplementedError

    def load_all(self, root, refused=None):
        """Records under ``root``; refusals appended to ``refused`` as (ref, reason)."""
        out = []
        for ref in self.discover(root):
            try:
                out.append(self.load(ref))
            except Refused as e:
                if refused is not None:
                    refused.append((str(ref), str(e)))
        return out
