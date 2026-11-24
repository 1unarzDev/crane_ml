"""crane's batched fast-tier engine (Docs/Engine.md): kernels, batched episodes, backends numpy|numba|jax."""

try:  # before any course truth is loaded: Tools/RoboBoat/course_truth.py puts its folder (and its
    import numba.misc.coverage_support  # noqa: F401  coverage.py) first on sys.path, which numba's
except ImportError:  # lazy `import coverage` would then pick up
    pass
