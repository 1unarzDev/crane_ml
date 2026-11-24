"""Backend ops for the array programs: the same code runs eagerly on numpy and jitted on JAX.

    ops = numpy_ops()            # float64, the reference: exact helpers, BLAS-order products
    ops = jax_ops('float32')     # jit, lax control flow, plain sums
"""

from functools import partial

import numpy as np

from engine import exact


class Ops:
    name = 'numpy'
    blas = True
    dtype = np.float64

    def __init__(self):
        self.xp = np

    def while_loop(self, cond, body, x):
        while cond(x):
            x = body(x)
        return x

    def cond(self, pred, yes, no):
        return yes() if bool(pred) else no()

    def masked(self, mask, fn, args, default):
        """``fn(*args)`` for the envs in ``mask`` (rows of every array in ``args``), ``default``
        elsewhere: numpy computes only those rows."""
        idx = np.flatnonzero(mask)
        if not len(idx):
            return default
        got = fn(*tree_map(lambda a: a[idx], args))
        out = []
        for d, g in zip(default, got):
            d = d.copy()
            d[idx] = g
            out.append(d)
        return tuple(out)

    def chypot(self, x, y):
        return exact.chypot(x, y, self.xp)

    def pyround(self, x, nd):
        return exact.pyround(x, nd, self.xp)

    window = None   # the tracker pops passed poses in a while loop

    def put_row(self, buf, j, row, mask):
        """buf[e, j[e]] = row[e] where mask (in place on numpy)."""
        e = np.flatnonzero(mask)
        buf[e, j[e]] = row[e]
        return buf

    def to(self, a):
        return a


def numpy_ops():
    return Ops()


class JaxOps(Ops):
    name = 'jax'
    blas = False

    def __init__(self, dtype='float64'):
        from engine.jaxconf import jax, jnp
        self.jax, self.xp = jax, jnp
        self.dtype = jnp.float64 if dtype == 'float64' else jnp.float32
        self.while_loop = jax.lax.while_loop

    def cond(self, pred, yes, no):
        return self.jax.lax.cond(pred, yes, no)

    def masked(self, mask, fn, args, default):
        return tuple(self.xp.where(mask.reshape(mask.shape + (1,) * (g.ndim - 1)), g, d)
                     for g, d in zip(fn(*args), default))

    def chypot(self, x, y):
        if self.dtype == self.xp.float64:
            return exact.chypot(x, y, self.xp)
        return self.xp.hypot(x, y)

    def pyround(self, x, nd):
        return exact.pyround(x, nd, self.xp)

    window = 16     # the tracker pops passed poses from a window of the route (no device-side loop)

    def put_row(self, buf, j, row, mask):
        e = self.xp.arange(buf.shape[0])
        j = self.xp.clip(j, 0, buf.shape[1] - 1)
        return buf.at[e, j].set(self.xp.where(mask[:, None], row, buf[e, j]))

    def to(self, a):
        a = np.asarray(a)
        if a.dtype.kind == 'f':
            return self.xp.asarray(a, self.dtype)
        if a.dtype.kind in 'iu':
            return self.xp.asarray(a, self.xp.int32)
        return self.xp.asarray(a)


def jax_ops(dtype='float64'):
    return JaxOps(dtype)


def tree_map(fn, tree):
    if isinstance(tree, dict):
        return {k: tree_map(fn, v) for k, v in tree.items()}
    if isinstance(tree, (tuple, list)):
        return type(tree)(tree_map(fn, v) for v in tree)
    return fn(tree)


__all__ = ['numpy_ops', 'jax_ops', 'tree_map', 'partial']
