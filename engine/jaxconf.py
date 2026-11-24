"""JAX for the engine: float64 enabled (``jax_enable_x64``) before any array exists; float32 runs cast."""

import jax

jax.config.update('jax_enable_x64', True)

import jax.numpy as jnp  # noqa: E402

__all__ = ['jax', 'jnp']
