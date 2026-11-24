"""crane on the research cluster: a study compiles to rc-job/1 jobs, the research CLI submits them, fetch brings the
outputs back into the same OUT/<study>/ layout a local run writes.

    jobs = compile.compile(study)                      # one job per (cell, seed chunk), 5-30 min each
    crane cluster submit study.yaml --dry-run          # the plan; without --dry-run, research submit
    crane cluster wait boat01 --timeout 3600           # one bounded wait, then the status table
    crane cluster fetch boat01                         # OUT/boat01/<stage>/<arm>/<cell>/ rows + manifest.json

The names every piece shares (the study's output root, a cell's id and dir, a block's role, the plan file) are here.
"""

import os
import re
from pathlib import Path

STUDIES_ROOT = Path.home() / '.cache' / 'crane' / 'studies'
PLAN = 'cluster/jobs.json'        # under OUT/<study>/: what submit sent, read by status, wait, fetch and monitor
HELD_OUT = 'held-out'


def slug(text, n=40):
    return re.sub(r'[^A-Za-z0-9._-]+', '-', str(text)).strip('-.')[:n] or 'x'


def studies_root(root=None):
    """OUT: --out, else CRANE_STUDIES, else ~/.cache/crane/studies (never inside the repo)."""
    return Path(root or os.environ.get('CRANE_STUDIES') or STUDIES_ROOT).expanduser()


def study_name(study):
    name = getattr(study, 'name', None)
    if not name:
        cells = list(study.cells())
        name = cells[0].study if cells else None
    if not name:
        raise ValueError('the study has no name (set name: in the spec)')
    return str(name)


def cell_id(cell):
    """The id `crane study run-cell --cell` takes: the cell's own id, else stage.arm.<key digest>."""
    own = getattr(cell, 'id', None) or getattr(cell, 'cell_id', None)
    if own:
        return str(own)
    from crane.ids import digest
    return f'{slug(cell.stage or "main", 20)}.{slug(cell.arm, 30)}.{digest(cell.key)[:10]}'


def role_of(study, cell):
    """A cell's block role (dev, check, train, calib, held-out), or None when the study does not say."""
    role = getattr(cell, 'role', None)
    if role is None:
        blocks = getattr(study, 'blocks', None) or {}
        spec = blocks.get(cell.block) if isinstance(blocks, dict) else None
        role = spec.get('role') if isinstance(spec, dict) else getattr(spec, 'role', None)
    if role is None and cell.block and norm_role(cell.block) == HELD_OUT:
        role = HELD_OUT
    return norm_role(role) if role else None


def norm_role(role):
    r = str(role).lower().replace('_', '-')
    return HELD_OUT if r in ('held-out', 'heldout', 'holdout', 'hold-out') else r


def cell_dir(root, study, cell, name=None):
    """OUT/<study>/<stage>/<arm>/<cell id>: where a cell's rows and manifest.json live, local or fetched. A study
    that defines cell_dir(cell, root) decides instead."""
    own = getattr(study, 'cell_dir', None)
    if callable(own):
        return Path(own(cell, root))
    name = name or study_name(study)
    return studies_root(root) / name / slug(cell.stage or 'main') / slug(cell.arm) / slug(cell_id(cell), 120)


def plan_path(root, name):
    return studies_root(root) / name / PLAN
