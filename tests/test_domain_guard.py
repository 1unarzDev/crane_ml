"""crane's Python packages name no team: a team plugs in from its own repo through module:function.

fastsim names no domain at all. judge, theater, plantcard and crane_backends may be marine- or
course-generic, but name no team repo, robot or host, and judge's core imports no pack or domain.
Tests and card data are exempt (they may use a team's card as the example).
"""

import ast
import re
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
TEAM_WORDS = ('mhseals', 'blastoise', 'squirtle', 'dingus', 'odroid')
DOMAIN_WORDS = ('roboboat', 'boat', 'buoy', 'nav2', 'mppi', 'ros', 'tranquility', 'referee', 'horn',
                'marina', 'dock')
PACKAGES = {'fastsim': DOMAIN_WORDS, 'judge': (), 'theater': (), 'plantcard': (), 'crane_backends': ()}
EXEMPT = {'tests', 'cards'}
JUDGE_IMPORTS = ('nlc', 'fastsim', 'crane_cluster', 'judge.packs') + TEAM_WORDS


def sources(root, pkg):
    for path in sorted((root / pkg).rglob('*')):
        if path.suffix in ('.py', '.yaml') and not EXEMPT & set(path.relative_to(root).parts):
            yield path


def leaks(root=ROOT):
    """(file, words) for every source whose path or text names a banned word."""
    out = []
    for pkg, domain in PACKAGES.items():
        for path in sources(root, pkg):
            rel = str(path.relative_to(root))
            text = (rel + '\n' + path.read_text()).lower()
            found = [w for w in TEAM_WORDS if w in text] + [w for w in domain if re.search(rf'\b{w}s?\b', text)]
            if found:
                out.append((rel, found))
    return out


def test_no_package_names_a_team_and_fastsim_no_domain():
    assert not leaks(), leaks()


def test_the_guard_sees_a_team_named_file(tmp_path):
    (tmp_path / 'plantcard').mkdir()
    (tmp_path / 'plantcard' / 'mhseals.py').write_text('x = 1\n')
    (tmp_path / 'fastsim').mkdir()
    (tmp_path / 'fastsim' / 'w.py').write_text('"""a boat"""\n')
    assert leaks(tmp_path) == [('fastsim/w.py', ['boat']), ('plantcard/mhseals.py', ['mhseals'])]


def _imports(path):
    for node in ast.walk(ast.parse(path.read_text())):
        if isinstance(node, ast.Import):
            yield from (a.name for a in node.names)
        elif isinstance(node, ast.ImportFrom) and node.module:
            yield node.module


JUDGE_CORE = [p for p in sources(ROOT, 'judge') if p.suffix == '.py' and 'packs' not in p.parts
              and p.name != '__main__.py']     # the CLI wires packs in; everything else is core


def test_judge_core_imports_no_pack_or_domain():
    bad = [(p.name, m) for p in JUDGE_CORE for m in _imports(p)
           if m.split('.')[0] in JUDGE_IMPORTS or m.startswith('judge.packs')]
    assert not bad, bad


def test_judge_core_needs_only_numpy():
    import sys
    extra = {m.split('.')[0] for p in JUDGE_CORE for m in _imports(p)} - {'numpy', 'judge', '__future__'}
    assert not extra - set(sys.stdlib_module_names), extra
