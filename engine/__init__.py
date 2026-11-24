"""crane's batched fast-tier engine (Docs/Engine.md): kernels, batched episodes, backends numpy|numba|jax."""

import sys as _sys
from pathlib import Path as _Path

# Tools/RoboBoat/course_truth.py puts its folder first on sys.path, and its coverage.py would shadow the
# package numba imports (lazily, `import coverage`): import numba's coverage support without that folder.
_saved = list(_sys.path)
_sys.path[:] = [p for p in _saved if not (_Path(p or '.') / 'course_truth.py').is_file()]
try:
    import numba.misc.coverage_support  # noqa: F401
except ImportError:
    pass
finally:
    _sys.path[:] = _saved
