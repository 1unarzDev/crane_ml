"""engine/exact.py's helpers as numba scalars (njit), for the per-env numba kernels.

``fma`` is the LLVM intrinsic (the CPU's fused multiply-add, correctly rounded); ``pow_half`` and
``pow2`` call libm's ``pow`` under a private symbol name, because LLVM rewrites ``pow(x, 0.5)`` into
``sqrt`` and ``pow(x, 2)`` into ``x * x``, which are not Python's ``x ** 0.5`` and ``x ** 2``.
"""

import ctypes
import ctypes.util
import math

import llvmlite.binding as llvm
from llvmlite import ir
import numba
from numba import types
from numba.core import cgutils
from numba.extending import intrinsic

_libm = ctypes.CDLL(ctypes.util.find_library('m'))
llvm.add_symbol('crane_engine_libm_pow', ctypes.cast(_libm.pow, ctypes.c_void_p).value)

jit = numba.njit(cache=True, inline='always')


@intrinsic
def fma(typingctx, a, b, c):
    sig = types.float64(types.float64, types.float64, types.float64)

    def codegen(context, builder, signature, args):
        d = ir.DoubleType()
        fn = cgutils.get_or_insert_function(builder.module, ir.FunctionType(d, [d, d, d]), 'llvm.fma.f64')
        return builder.call(fn, args)
    return sig, codegen


@intrinsic
def _pow(typingctx, a, b):
    sig = types.float64(types.float64, types.float64)

    def codegen(context, builder, signature, args):
        d = ir.DoubleType()
        fn = cgutils.get_or_insert_function(builder.module, ir.FunctionType(d, [d, d]), 'crane_engine_libm_pow')
        return builder.call(fn, args)
    return sig, codegen


@jit
def two_prod(a, b):
    p = a * b
    t = 134217729.0 * a
    ah = t - (t - a)
    al = a - ah
    t = 134217729.0 * b
    bh = t - (t - b)
    bl = b - bh
    return p, ((ah * bh - p) + ah * bl + al * bh) + al * bl


@jit
def chypot(x, y):
    ax, ay = abs(x), abs(y)
    mx = max(ax, ay)
    if mx == 0.0:
        return mx
    _, e = math.frexp(mx)
    scale = math.ldexp(1.0, -e)
    csum, frac1, frac2 = 1.0, 0.0, 0.0
    v = ax * scale
    hi, lo = two_prod(v, v)
    s = csum + hi
    frac2 += (csum - s) + hi
    csum = s
    frac1 += lo
    v = ay * scale
    hi, lo = two_prod(v, v)
    s = csum + hi
    frac2 += (csum - s) + hi
    csum = s
    frac1 += lo
    h = math.sqrt(csum - 1.0 + (frac1 + frac2))
    hi, lo = two_prod(-h, h)
    s = csum + hi
    frac2 += (csum - s) + hi
    csum = s
    frac1 += lo
    v = csum - 1.0 + (frac1 + frac2)
    h += v / (2.0 * h)
    return h / scale


@jit
def pyround(x, m):
    """round(x, ndigits) with m = 10.0 ** ndigits."""
    p, e = two_prod(x, m)
    n0 = math.floor(p)
    d = ((p - n0) - 0.5) + e
    if d > 0:
        n = n0 + 1.0
    elif d < 0:
        n = n0
    elif n0 % 2.0 != 0:
        n = n0 + 1.0
    else:
        n = n0
    return math.copysign(n / m, x)


@jit
def wrap(a):
    return math.atan2(math.sin(a), math.cos(a))


@jit
def pow_half(v):
    return _pow(v, 0.5)


@jit
def pow2(v):
    return _pow(v, 2.0)


@jit
def dot2_blas(d0, d1, u0, u1, n):
    if n == 1:
        return d0 * u0 + d1 * u1
    return fma(d0, u0, d1 * u1)
