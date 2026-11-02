"""Identity hashes, deterministic seeding and provenance: what makes two runs the same run.

A run's identity is the sha256 of {scenario content, config, adapter identity, seed, core
code}; nothing here reads a wall clock, so equal identities mean byte-equal run directories.
"""

import functools
import hashlib
import json
from pathlib import Path
import subprocess

import numpy as np


def canonical(value):
    """Compact, key-sorted JSON: the one serialisation every hash here is taken over."""
    return json.dumps(value, sort_keys=True, separators=(',', ':'), default=str)


def digest(value):
    return hashlib.sha256(canonical(value).encode()).hexdigest()


def sha256(data):
    return hashlib.sha256(data).hexdigest()


def tree_sha(root, skip=('analysis',)):
    """sha256 over a directory's files (relative paths and bytes); top-level ``skip`` left out."""
    root, h = Path(root), hashlib.sha256()
    for path in sorted(p for p in root.rglob('*') if p.is_file()):
        rel = path.relative_to(root)
        if rel.parts[0] not in skip:
            h.update(str(rel).encode() + b'\0' + path.read_bytes() + b'\0')
    return h.hexdigest()


def derive_seed(seed, *stream):
    """A 64-bit seed for one named stream of one run (``derive_seed(7, 'perception')``):
    streams never share draws, and adding a stream changes no other stream's numbers."""
    return int.from_bytes(hashlib.sha256(canonical([int(seed), *stream]).encode()).digest()[:8], 'big')


def rng(seed, *stream):
    return np.random.default_rng(derive_seed(seed, *stream))


def git_state(path):
    """{commit, dirty, source} of the git tree holding ``path``. Outside git (or on a host without
    git) a ``COMMIT`` file in an enclosing folder names it: a copied tree's first word is its commit,
    and the word ``dirty`` anywhere in it marks local changes. Otherwise commit is None."""
    path = Path(path).resolve()
    where = path if path.is_dir() else path.parent

    def git(*args):
        try:
            return subprocess.run(['git', '-C', str(where), *args], capture_output=True, text=True,
                                  check=False).stdout.strip()
        except OSError:  # no git on this host
            return ''
    commit = git('rev-parse', 'HEAD') or None
    if commit is None:
        for folder in (where, *where.parents):
            if (folder / 'COMMIT').is_file():
                words = (folder / 'COMMIT').read_text().split()
                return {'commit': words[0] if words else None, 'dirty': 'dirty' in words, 'source': str(folder)}
    return {'commit': commit,
            'dirty': bool(git('status', '--porcelain', '--untracked-files=no', '--', str(where))),
            'source': git('rev-parse', '--show-toplevel') or str(where)}


@functools.lru_cache(maxsize=1)
def core_version():
    """The fast-sim core's own code version, part of every run identity (read once per process)."""
    state = git_state(Path(__file__).parent)
    return {'commit': state['commit'], 'dirty': state['dirty']}


def run_identity(scenario_sha256, config, adapter, seed, core):
    """The run identity block: each part's hash and the run hash over all of them."""
    parts = {'scenario_sha256': scenario_sha256, 'config_sha256': digest(config),
             'adapter_sha256': digest(adapter), 'seed': int(seed), 'core': core}
    return {**parts, 'run_sha256': digest(parts)}
