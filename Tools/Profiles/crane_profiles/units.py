"""Unit registry with dimension algebra (pure python, no dependencies).

A unit string such as ``"N s/(m rad)"`` parses to a Unit(dim, scale): an integer
exponent vector over BASES and a multiplier to the coherent SI unit.  Parsing
rules: whitespace multiplies, ``/`` divides by the next factor (left
associative), ``^n`` raises an atom or parenthesised group to an integer power.
``"1"`` and ``"-"`` are dimensionless.
"""
from __future__ import annotations

import math
import re
from dataclasses import dataclass

BASES = ("kg", "m", "s", "A", "K", "rad", "norm")


class UnitError(ValueError):
    pass


@dataclass(frozen=True)
class Unit:
    dim: tuple
    scale: float = 1.0

    def __mul__(self, o: "Unit") -> "Unit":
        return Unit(tuple(a + b for a, b in zip(self.dim, o.dim)), self.scale * o.scale)

    def __truediv__(self, o: "Unit") -> "Unit":
        return Unit(tuple(a - b for a, b in zip(self.dim, o.dim)), self.scale / o.scale)

    def __pow__(self, n: int) -> "Unit":
        return Unit(tuple(a * n for a in self.dim), self.scale ** n)

    def same_dimension(self, o: "Unit") -> bool:
        return self.dim == o.dim

    def dimension_name(self) -> str:
        parts = [f"{b}^{e}" if e != 1 else b for b, e in zip(BASES, self.dim) if e]
        return " ".join(parts) or "1"


def _b(**kw: int) -> tuple:
    return tuple(kw.get(b, 0) for b in BASES)


ONE = Unit(_b())
_N = Unit(_b(kg=1, m=1, s=-2))
_V = Unit(_b(kg=1, m=2, s=-3, A=-1))
REGISTRY: dict[str, Unit] = {
    "1": ONE, "-": ONE,
    "kg": Unit(_b(kg=1)), "g": Unit(_b(kg=1), 1e-3),
    "m": Unit(_b(m=1)), "mm": Unit(_b(m=1), 1e-3), "cm": Unit(_b(m=1), 1e-2), "km": Unit(_b(m=1), 1e3),
    "s": Unit(_b(s=1)), "ms": Unit(_b(s=1), 1e-3), "A": Unit(_b(A=1)), "K": Unit(_b(K=1)),
    "rad": Unit(_b(rad=1)), "deg": Unit(_b(rad=1), math.pi / 180.0),
    "norm": Unit(_b(norm=1)),
    "N": _N, "Pa": _N / Unit(_b(m=2)), "J": _N * Unit(_b(m=1)), "W": _N * Unit(_b(m=1)) / Unit(_b(s=1)),
    "Hz": Unit(_b(s=-1)), "V": _V, "ohm": _V / Unit(_b(A=1)), "C": Unit(_b(A=1, s=1)),
    "Ah": Unit(_b(A=1, s=1), 3600.0),
}

_TOKEN = re.compile(r"\s*(?:(?P<id>[A-Za-z]+|-|1)|(?P<pow>\^-?\d+)|(?P<op>[/()]))")


def _tokens(text: str) -> list:
    out, pos = [], 0
    while pos < len(text):
        m = _TOKEN.match(text, pos)
        if not m or m.end() == pos:
            if text[pos:].strip() == "":
                break
            raise UnitError(f"cannot parse unit {text!r} at {pos}")
        pos = m.end()
        kind = m.lastgroup
        out.append((kind, m.group(kind).strip()))
    return out


def parse_unit(text: object) -> Unit:
    if not isinstance(text, str) or not text.strip():
        raise UnitError("unit must be a non-empty string")
    toks = _tokens(text)
    if not toks:
        raise UnitError(f"empty unit {text!r}")
    pos = 0

    def atom() -> Unit:
        nonlocal pos
        if pos >= len(toks):
            raise UnitError(f"unexpected end of unit {text!r}")
        kind, val = toks[pos]
        if kind == "id":
            if val not in REGISTRY:
                raise UnitError(f"unknown unit {val!r} in {text!r}")
            u = REGISTRY[val]
            pos += 1
        elif val == "(":
            pos += 1
            u = expr()
            if pos >= len(toks) or toks[pos][1] != ")":
                raise UnitError(f"unbalanced parenthesis in {text!r}")
            pos += 1
        else:
            raise UnitError(f"unexpected {val!r} in {text!r}")
        if pos < len(toks) and toks[pos][0] == "pow":
            u = u ** int(toks[pos][1][1:])
            pos += 1
        return u

    def expr() -> Unit:
        nonlocal pos
        u = atom()
        while pos < len(toks) and toks[pos][1] != ")":
            if toks[pos][1] == "/":
                pos += 1
                u = u / atom()
            else:
                u = u * atom()
        return u

    result = expr()
    if pos != len(toks):
        raise UnitError(f"trailing tokens in unit {text!r}")
    return result


def is_unit(text: object) -> bool:
    try:
        parse_unit(text)
        return True
    except UnitError:
        return False


# Key-suffix -> representative unit.  Longest suffix wins; case sensitive.
SUFFIX_UNITS = {
    "KgM2": "kg m^2", "Mps2": "m/s^2", "RadPerSec2": "rad/s^2", "RadPerSec": "rad/s", "Mps": "m/s",
    "Nm": "N m", "Kg": "kg", "Seconds": "s", "Meters": "m", "Ah": "Ah", "Ohm": "ohm", "Hz": "Hz",
    "Pa": "Pa", "Deg": "rad", "Rad": "rad", "N": "N", "M": "m", "S": "s", "V": "V",
}
_SUFFIXES = sorted(SUFFIX_UNITS, key=len, reverse=True)


def key_suffix(key: str) -> str | None:
    for suf in _SUFFIXES:
        if key.endswith(suf) and len(key) > len(suf) and not key[-len(suf) - 1].isupper():
            if key[:-len(suf)].endswith("Per"):  # rate/ratio keys (KgPerM, NsPerM) are not plain suffixes
                return None
            return suf
    return None


def expected_dimension(key: str) -> Unit | None:
    suf = key_suffix(key)
    return parse_unit(SUFFIX_UNITS[suf]) if suf else None


def convert(value: float, src: str, dst: str) -> float:
    a, b = parse_unit(src), parse_unit(dst)
    if not a.same_dimension(b):
        raise UnitError(f"cannot convert {src!r} ({a.dimension_name()}) to {dst!r} ({b.dimension_name()})")
    return value * a.scale / b.scale
