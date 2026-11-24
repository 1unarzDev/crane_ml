"""Identity helpers every crane tool shares: canonical JSON, digests, seed streams, seed specs, git state.

    from crane import ids
    ids.digest({'arm': 'a', 'seed': 3})          # sha256 hex over canonical JSON
    ids.derive_seed(7, 'perception')            # the same 64-bit stream seed fastsim uses
    ids.parse_seeds('block:dev', blocks={'dev': '1:21'})

fastsim.identity is the source of truth for canonical JSON, digests and seed streams; this module re-exports it.
"""

import hashlib
from pathlib import Path
import subprocess

from fastsim.identity import canonical as canonical_json, derive_seed, digest, rng, sha256  # noqa: F401  re-export
from fastsim.identity import git_state as _git_state

__all__ = ['canonical_json', 'digest', 'sha256', 'derive_seed', 'rng', 'parse_seeds', 'git_state']


def parse_seeds(spec, blocks=None, base=None):
    """Seeds from one spec, in order, duplicates dropped:

        N            the one seed N
        A:B          A..B-1
        a,b,c        a list; each item may itself be any form here (``1:4,9``)
        @FILE        seeds in a file (whitespace or comma separated, ``#`` comments); FILE relative to ``base``
        block:NAME   ``blocks[NAME]``: a spec string, a list of seeds, or ``{'seeds': ...}``

    A bad spec raises ValueError naming it."""
    out, seen = [], set()
    for seed in _expand(str(spec).strip(), blocks or {}, Path(base or '.'), depth=0):
        if seed not in seen:
            seen.add(seed)
            out.append(seed)
    return out


def _expand(spec, blocks, base, depth):
    if depth > 8:
        raise ValueError(f'seed spec {spec!r}: blocks or files nest too deep')
    if not spec:
        raise ValueError('seed spec is empty')
    if ',' in spec:
        for part in spec.split(','):
            if part.strip():
                yield from _expand(part.strip(), blocks, base, depth + 1)
        return
    if spec.startswith('@'):
        path = base / spec[1:]
        try:
            text = path.read_text()
        except OSError as error:
            raise ValueError(f'seed file {path}: {error.strerror or error}') from None
        tokens = [t for line in text.splitlines() for t in line.split('#')[0].replace(',', ' ').split()]
        if not tokens:
            raise ValueError(f'seed file {path} names no seeds')
        for token in tokens:
            yield from _expand(token, blocks, path.parent, depth + 1)
        return
    if spec.startswith('block:'):
        name = spec[len('block:'):]
        if name not in blocks:
            known = ', '.join(sorted(blocks)) or 'none defined'
            raise ValueError(f'seed block {name!r} is not defined (known: {known})')
        block = blocks[name]
        block = block.get('seeds') if isinstance(block, dict) else block
        if isinstance(block, (list, tuple, range)):
            for s in block:
                yield from _expand(str(s), blocks, base, depth + 1)
        else:
            yield from _expand(str(block), blocks, base, depth + 1)
        return
    lo, sep, hi = spec.partition(':')
    try:
        lo, hi = int(lo), int(hi) if sep else None
    except ValueError:
        raise ValueError(f'seed spec {spec!r}: use N, A:B, a,b,c, @FILE or block:NAME') from None
    if hi is None:
        yield lo
    elif hi <= lo:
        raise ValueError(f'seed range {spec!r} is empty (TO is exclusive)')
    else:
        yield from range(lo, hi)


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
