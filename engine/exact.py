"""Exact numerics: the scalar fastsim path's own roundings, reproduced bit for bit on arrays.

    from engine import exact
    exact.chypot(dx, dy)          # == math.hypot(dx, dy) == math.dist(...), elementwise
    exact.pyround(x, 4)           # == round(x, 4) (Python's correctly rounded decimal rounding)
    exact.fma(a, b, c)            # == fma(a, b, c), correctly rounded, without an fma instruction
    exact.matvec_blas(B, x)       # == B @ x for B [4, 3] as this host's OpenBLAS dgemv computes it
    exact.openblas()              # numpy's BLAS is OpenBLAS (the order matvec_blas reproduces)

The scalar path (fastsim, domains/boat) mixes three kinds of arithmetic: CPython's ``math`` (libm,
except ``hypot``/``dist``, which CPython computes itself, correctly rounded in almost all cases),
numpy ufuncs (libm) and OpenBLAS matrix-vector products (FMA, in the kernel's own order). numpy's
``hypot`` and ``x ** 0.5`` are not CPython's, and a plain ``a0*b0 + a1*b1`` is not dgemv's. The
float64 numpy reference uses these helpers wherever the scalar path does, so that its episodes
equal the scalar ones bit for bit. Each helper is a plain function of ``xp`` (numpy or jax.numpy)
arrays, plus a numba twin in ``engine/nbexact.py``.
"""

import numpy as np

VELTKAMP = {np.dtype(np.float64): 134217729.0, np.dtype(np.float32): 4097.0}  # 2**ceil(p/2) + 1


def _split(a, xp=np):
    t = VELTKAMP[np.dtype(a.dtype)] * a if hasattr(a, 'dtype') else 134217729.0 * a
    hi = t - (t - a)
    return hi, a - hi


def two_prod(a, b, xp=np):
    """(p, e) with p = fl(a*b) and p + e == a*b exactly (Dekker; no fma needed)."""
    p = a * b
    ah, al = _split(a, xp)
    bh, bl = _split(b, xp)
    return p, ((ah * bh - p) + ah * bl + al * bh) + al * bl


def two_sum(a, b):
    """(s, e) with s = fl(a+b) and s + e == a+b exactly (Knuth)."""
    s = a + b
    bb = s - a
    return s, (a - (s - bb)) + (b - bb)


def _odd_add(x, y, xp=np):
    """x + y rounded to odd (Boldo-Melquiond): the exact sum when representable, else the
    neighbour of fl(x + y) toward the exact sum whose last significand bit is 1."""
    s, e = two_sum(x, y)
    bits = s.view(np.int64) if xp is np else _bits(xp, s)
    even = (bits & 1) == 0
    toward = xp.nextafter(s, xp.where(e > 0, xp.inf, -xp.inf))
    return xp.where((e != 0) & even, toward, s)


def _bits(xp, s):
    import jax
    return jax.lax.bitcast_convert_type(s, xp.int64 if s.dtype == xp.float64 else xp.int32)


def fma(a, b, c, xp=np):
    """fma(a, b, c) = a*b + c rounded once (Boldo-Melquiond emulation with round-to-odd; exact
    for finite float64 operands away from overflow and underflow)."""
    a, b, c = (xp.asarray(v, dtype=xp.float64) for v in (a, b, c))
    uh, ul = two_prod(a, b, xp)
    th, tl = two_sum(c, uh)
    return th + _odd_add(tl, ul, xp)


def blas():
    """The BLAS numpy was built against (e.g. 'scipy-openblas', 'cblas'), or 'unknown'."""
    try:
        return str(np.show_config(mode='dicts')['Build Dependencies']['blas']['name'])
    except Exception:  # noqa: BLE001  an older numpy without mode='dicts'
        return 'unknown'


def openblas():
    """Whether numpy's BLAS is OpenBLAS, whose dgemv order ``matvec_blas`` reproduces. On another BLAS (a distro's
    reference CBLAS) the scalar path rounds differently and batched runs match it only up to that rounding."""
    return 'openblas' in blas().lower()


def matvec_blas(B, x, xp=np):
    """``B @ x`` for B [4, 3] and x [..., 3], as OpenBLAS's dgemv computes each row on this host:
    fma(b2, x2, fma(b0, x0, b1 * x1)). engine/tests checks the order against ``np.dot`` here."""
    return xp.stack([fma(B[i, 2], x[..., 2], fma(B[i, 0], x[..., 0], B[i, 1] * x[..., 1], xp), xp)
                     for i in range(B.shape[0])], axis=-1)


def dot2_blas(d0, d1, u0, u1, n, xp=np):
    """Row i of ``d @ u`` for d [n, 2], u [2] as numpy computes it: a plain sum for n == 1, else
    OpenBLAS's fma(d0, u0, d1 * u1). ``n`` broadcasts against the rows."""
    plain = d0 * u0 + d1 * u1
    return xp.where(n == 1, plain, fma(d0, u0, d1 * u1, xp))


def chypot(x, y, xp=np):
    """CPython's ``math.hypot(x, y)`` (and ``math.dist`` of a 2-D difference): its vector_norm,
    with lossless scaling, double-length squares and a differential correction."""
    ax, ay = xp.abs(x), xp.abs(y)
    mx = xp.maximum(ax, ay)
    _, e = xp.frexp(mx)
    scale = xp.ldexp(xp.ones_like(mx), -e)
    csum = xp.ones_like(mx)
    frac1 = xp.zeros_like(mx)
    frac2 = xp.zeros_like(mx)
    for v in (ax, ay):
        v = v * scale
        hi, lo = two_prod(v, v, xp)
        s = csum + hi
        sl = (csum - s) + hi
        csum = s
        frac1 = frac1 + lo
        frac2 = frac2 + sl
    h = xp.sqrt(csum - 1.0 + (frac1 + frac2))
    hi, lo = two_prod(-h, h, xp)
    s = csum + hi
    sl = (csum - s) + hi
    csum = s
    frac1 = frac1 + lo
    frac2 = frac2 + sl
    v = csum - 1.0 + (frac1 + frac2)
    h = h + v / (2.0 * h)
    safe = xp.where(mx == 0, xp.ones_like(mx), h / scale)
    return xp.where(mx == 0, mx, safe)


def pyround(x, ndigits, xp=np):
    """Python's ``round(x, ndigits)`` for floats: the exact value of x rounded half-even at
    ``ndigits`` decimals, then the nearest double (n / 10**ndigits is one correctly rounded
    division). The sign of zero is kept, as Python keeps it."""
    m = float(10 ** ndigits)
    p, e = two_prod(x, xp.asarray(m, dtype=x.dtype) if hasattr(x, 'dtype') else m, xp)
    n0 = xp.floor(p)
    d = ((p - n0) - 0.5) + e
    odd = xp.remainder(n0, 2.0) != 0
    n = xp.where(d > 0, n0 + 1.0, xp.where(d < 0, n0, xp.where(odd, n0 + 1.0, n0)))
    return xp.copysign(n / _opaque(xp, m, x), x)


def _opaque(xp, m, like):
    """``m`` as XLA may not fold it (x / c would become x * (1/c), which is not correctly rounded)."""
    if xp is np:
        return m
    import jax
    return jax.lax.optimization_barrier(xp.asarray(m, like.dtype))


def wrap(a, xp=np):
    """fastsim.world.wrap: atan2(sin a, cos a)."""
    return xp.arctan2(xp.sin(a), xp.cos(a))


def pow2(v, xp=np):
    """Python's ``v ** 2`` (libm pow, which is not always ``v * v``)."""
    if xp is np:
        return np.power(v, np.full_like(v, 2.0))
    return v * v


def pow_half(v, xp=np):
    """Python's ``v ** 0.5`` (libm pow; numpy's ``** 0.5`` is sqrt, which differs in the last bit)."""
    if xp is np:
        return np.power(v, np.full_like(v, 0.5))
    return xp.power(v, 0.5)
