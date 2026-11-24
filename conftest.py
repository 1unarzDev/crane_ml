"""Root pytest hooks: `python3 -m pytest` here runs every suite (pyproject [tool.pytest.ini_options]).

Tests that need a host tool get a marker (dotnet, node, docker, cluster, display) and skip with one line naming the
fix when the tool is absent. `-m "not dotnet"` deselects them. A test can carry the marker itself
(`@pytest.mark.node`); NEEDS marks existing ones by path so the suites stay untouched. A NEEDS folder (ending in /)
whose modules import the tool at import time is skipped module by module before import (a package __init__'s
SkipTest does not stop importlib mode from importing the modules).
"""
import functools
import importlib.util
import shutil
import subprocess
import sys
from pathlib import Path

import pytest

ROOT = Path(__file__).resolve().parent

# test path (or path::name prefix) -> (marker, argument); dotnet's argument is the folder whose global.json pins the SDK
NEEDS = {
    'theater/tests/test_cs_codec.py': ('dotnet', '.'),
    'Tools/RoboBoat/tests/test_cli.py': ('dotnet', 'Tools/RoboBoat/cli'),
    'theater/tests/test_js_codec.py': ('node', None),
    'viewer/tests/test_viewer.py::ViewerTest::test_page_modules_parse': ('node', None),
    'judge/tests/test_stats.py': ('cluster', None),
    'crane_backends/tests/': ('cluster', None),
    'tests/test_view.py::test_screenshot_renders_offscreen': ('display', None),
}


def _run_ok(argv, cwd=None):
    try:
        return subprocess.run(argv, cwd=cwd, capture_output=True, timeout=30).returncode == 0
    except (OSError, subprocess.TimeoutExpired):
        return False


@functools.lru_cache(maxsize=None)
def missing(marker, arg=None):
    """None when the marker's tool is here, else the one-line reason with the fix."""
    if marker == 'dotnet':
        if not shutil.which('dotnet'):
            return 'dotnet: not installed (https://dot.net, the SDK its global.json pins)'
        if not _run_ok(['dotnet', '--version'], cwd=ROOT / (arg or '.')):
            return f'dotnet: no SDK matches {arg}/global.json (dotnet --list-sdks; install that SDK)'
    elif marker == 'node':
        if not shutil.which('node'):
            return 'node: not installed (install Node.js 18+)'
    elif marker == 'docker':
        if not _run_ok(['docker', 'info']):
            return 'docker: no reachable daemon (systemctl start docker)'
    elif marker == 'cluster':
        sys.path.insert(0, str(ROOT))
        from crane_backends import find_platform
        if not find_platform():
            return 'cluster: no research-cluster checkout (set RESEARCH_CLUSTER or clone it to ~/research-cluster)'
    elif marker == 'display':
        if any(importlib.util.find_spec(m) is None for m in ('PySide6', 'pyqtgraph')):
            return "display: the [view] extra is not installed (./install.sh --components view)"
    return None


class _Unavailable(pytest.Module):
    reason = ''

    def collect(self):
        pytest.skip(self.reason, allow_module_level=True)


def pytest_pycollect_makemodule(module_path, parent):
    path = Path(module_path).resolve()
    rel = path.relative_to(ROOT).as_posix() if path.is_relative_to(ROOT) else ''
    for key, (marker, arg) in NEEDS.items():
        if key.endswith('/') and rel.startswith(key) and not rel.endswith('__init__.py'):
            reason = missing(marker, arg)
            if reason:
                mod = _Unavailable.from_parent(parent, path=module_path)
                mod.reason = reason
                return mod
    return None


def _needs(nodeid):
    for key, need in NEEDS.items():
        if nodeid == key or nodeid.startswith(key if key.endswith('/') else key + '::'):
            return need
    return None


def pytest_collection_modifyitems(config, items):
    for item in items:
        need = _needs(item.nodeid)
        if need:
            item.add_marker(getattr(pytest.mark, need[0])(need[1]))
        for name in ('dotnet', 'node', 'docker', 'cluster', 'display'):
            mark = item.get_closest_marker(name)
            if mark:
                reason = missing(name, mark.args[0] if mark.args else None)
                if reason:
                    item.add_marker(pytest.mark.skip(reason=reason))
