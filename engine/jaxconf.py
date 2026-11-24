"""JAX for the engine: float64 enabled (``jax_enable_x64``) before any array exists; float32 runs cast."""

import jax

jax.config.update('jax_enable_x64', True)
try:  # compiled programs persist across processes (the first call of a shape pays the compile once)
    import os
    jax.config.update('jax_compilation_cache_dir', os.path.expanduser(os.environ.get(
        'CRANE_JAX_CACHE', '~/.cache/crane/jax')))
    jax.config.update('jax_persistent_cache_min_compile_time_secs', 1.0)
except Exception:  # noqa: BLE001  an older jax without the persistent cache
    pass

import jax.numpy as jnp  # noqa: E402

__all__ = ['jax', 'jnp']
