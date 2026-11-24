"""crane.study/1: one YAML spec of arms x axes x conditions x tiers x seed blocks, run locally or compiled to jobs.

    from crane import study
    s = study.load('examples/studies/toy_paired.yaml')
    for cell in s.cells():                      # Cell{study, stage, arm, axes, condition, tier, seeds, block, key}
        rows = cell.run('/tmp/toy-study')       # cached units are reused
    s.jobs()                                    # cell x seed chunks for the cluster compiler (rc-job/1)

Docs/Study.md describes the spec; crane/study/spec.py reads it, plan.py expands it, exec.py runs it, guard.py keeps
held-out blocks closed, report.py does status and the paired analysis.
"""

from crane.study.spec import ROLES, SCHEMA, Study, StudyError, load  # noqa: F401
from crane.study.plan import Cell, StageNotReady  # noqa: F401

__all__ = ['load', 'Study', 'Cell', 'StudyError', 'StageNotReady', 'SCHEMA', 'ROLES']
