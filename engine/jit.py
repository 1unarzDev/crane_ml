"""numba when it is installed, else a stand-in, so the numpy backend imports and runs with numpy alone.

    from engine.jit import numba, nbexact, HAVE_NUMBA

The kernels' numba twins (``nb_*``) are decorated with ``numba.njit``; without numba the decorator returns
the plain function, which only the numba backend would call. ``nbexact`` is engine/nbexact.py, or a stand-in
whose functions say what is missing. ``engine.episode.run(setup, 'numba')`` then fails with one line naming
the fix.
"""

try:
    import numba
    HAVE_NUMBA = True
except ImportError:
    HAVE_NUMBA = False

FIX = "pip install 'crane[fast]' (numba), or run the numpy backend"


class _Missing:
    """numba's decorators as no-ops; anything else names the fix."""

    @staticmethod
    def njit(*args, **kwargs):
        if len(args) == 1 and callable(args[0]) and not kwargs:
            return args[0]
        return lambda fn: fn

    jit = njit
    prange = range

    def __getattr__(self, name):
        raise ImportError(f'engine: numba.{name} needs numba; fix: {FIX}')


class _NoExact:
    def __getattr__(self, name):
        def missing(*args, **kwargs):
            raise ImportError(f'engine: nbexact.{name} needs numba; fix: {FIX}')
        return missing


if HAVE_NUMBA:
    from engine import nbexact
else:
    numba = _Missing()
    nbexact = _NoExact()


def need_numba():
    if not HAVE_NUMBA:
        raise ImportError(f'engine: the numba backend needs numba; fix: {FIX}')
