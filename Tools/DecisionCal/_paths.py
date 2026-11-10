"""Import paths and provenance pins for Tools/DecisionCal.

The harness is runtime-independent of the MHSeals autonomy stack (mhseals_nav / _testing /
_hardware are being refactored and are read-only): the velocity law and mixer are vendored
(``vendor/``), the rigid-body ODE and environment fields come from the NLC study's
``nlc/sim/vessel.py`` pinned by sha256, and the twin family is crane's ``plantcard``.
"""

import hashlib
import os
import subprocess
import sys
from pathlib import Path

HERE = Path(__file__).resolve().parent
CRANE_ROOT = HERE.parents[1]
NLC_ROOT = Path(os.environ.get('DC_NLC_ROOT', Path.home() / 'nl-constraints-bench'))
NLC_VESSEL_SHA256 = '7f4f3362aab62963acc89db463c70df8b56332b3a3e7ff6d17534fced30cd80c'   # nl-constraints-bench@fc4437a
NLC_COMMIT = 'fc4437a'


def _sha(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def setup():
    """Put the harness, crane root (plantcard) and the NLC repo on sys.path; refuse a drifted
    nlc vessel. Returns provenance hashes for the run record."""
    for p in (str(HERE), str(CRANE_ROOT), str(NLC_ROOT)):
        if p not in sys.path:
            sys.path.insert(0, p)
    vessel = NLC_ROOT / 'nlc' / 'sim' / 'vessel.py'
    got = _sha(vessel)
    if got != NLC_VESSEL_SHA256:
        raise RuntimeError(f'nlc/sim/vessel.py drifted (sha256 {got[:12]}, pinned {NLC_VESSEL_SHA256[:12]} '
                           f'@{NLC_COMMIT}); re-pin deliberately after review')
    return provenance()


def provenance():
    try:
        head = subprocess.run(['git', '-C', str(CRANE_ROOT), 'rev-parse', 'HEAD'], capture_output=True,
                              text=True, timeout=10).stdout.strip()
        dirty = bool(subprocess.run(['git', '-C', str(CRANE_ROOT), 'status', '--porcelain', '--', str(HERE)],
                                    capture_output=True, text=True, timeout=10).stdout.strip())
    except (OSError, subprocess.SubprocessError):
        head, dirty = 'unknown', True
    return {'crane_head': head, 'decisioncal_dirty': dirty, 'nlc_vessel_sha256': NLC_VESSEL_SHA256,
            'nlc_commit': NLC_COMMIT,
            'vendor_sha256': {p.name: _sha(p) for p in sorted((HERE / 'vendor').glob('*.py')) if p.name != '__init__.py'}}


setup()
