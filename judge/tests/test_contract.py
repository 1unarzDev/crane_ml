"""The core names no domain: a new robot is a Source plus a Pack, never a core change."""

import ast
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
# the CLI (__main__) wires packs in; everything else is core
CORE = [p for p in ROOT.rglob('*.py') if 'tests' not in p.parts and 'packs' not in p.parts
        and p.name != '__main__.py']
FORBIDDEN = ('mhseals', 'nlc', 'fastsim', 'crane_cluster', 'judge.packs')


def _imports(path):
    tree = ast.parse(path.read_text())
    for node in ast.walk(tree):
        if isinstance(node, ast.Import):
            yield from (a.name for a in node.names)
        elif isinstance(node, ast.ImportFrom) and node.module:
            yield node.module


def test_core_imports_no_pack_or_domain():
    bad = [(p.relative_to(ROOT), m) for p in CORE for m in _imports(p) if m.split('.')[0] in FORBIDDEN[:4]
           or m.startswith('judge.packs')]
    assert not bad, bad


def test_core_needs_only_numpy():
    allowed = {'numpy', 'judge', '__future__'}
    import sys
    std = set(sys.stdlib_module_names)
    extra = {m.split('.')[0] for p in CORE for m in _imports(p)} - allowed - std
    assert not extra, extra
