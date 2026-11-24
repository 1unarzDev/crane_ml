"""Identity helpers every crane tool shares: canonical JSON, digests, seed streams, seed specs, git state.

    from crane import ids
    ids.digest({'arm': 'a', 'seed': 3})          # sha256 hex over canonical JSON
    ids.derive_seed(7, 'perception')            # the same 64-bit stream seed fastsim uses
    ids.parse_seeds('block:dev', blocks={'dev': '1:21'})   # crane.select.parse_seeds

fastsim.identity is the source of truth for canonical JSON, digests and seed streams; this module re-exports it.
"""

import hashlib
from pathlib import Path
import subprocess

from fastsim.identity import canonical as canonical_json, derive_seed, digest, rng, sha256  # noqa: F401  re-export
from fastsim.identity import git_state as _git_state
from crane.select import parse_seeds  # noqa: F401  re-export: the one seed-spec parser

__all__ = ['canonical_json', 'digest', 'sha256', 'derive_seed', 'rng', 'parse_seeds', 'git_state']


def git_state(root):
    """{commit, dirty, diff_sha, source} of the git tree holding ``root``. ``diff_sha`` is the sha256 of
    ``git diff HEAD`` over tracked files when the tree is dirty (so two dirty runs of the same edit share it),
    else None. Outside git, fastsim.identity.git_state's ``COMMIT`` file rule applies and diff_sha is None."""
    state = dict(_git_state(root))
    state['diff_sha'] = None
    if state['dirty'] and state['commit']:
        where = Path(root).resolve()
        where = where if where.is_dir() else where.parent
        try:
            diff = subprocess.run(['git', '-C', str(where), 'diff', 'HEAD', '--binary', '--', '.'], capture_output=True,
                                  check=False).stdout
        except OSError:
            diff = b''
        state['diff_sha'] = hashlib.sha256(diff).hexdigest() if diff else None
    return state
