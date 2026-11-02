"""YAML loading without a hard dependency: PyYAML when installed, otherwise a
small parser for the subset the cluster's config files use (block mappings and
lists, flow lists/maps of scalars, quoted strings, numbers, booleans, null,
comments). Workers on bare Windows/macOS Pythons need no extra packages.
"""
from __future__ import annotations

import json
import re

try:  # pragma: no cover - exercised implicitly where PyYAML exists
    import yaml as _yaml
except ImportError:  # pragma: no cover
    _yaml = None


def load(text: str):
    if _yaml is not None:
        return _yaml.safe_load(text)
    return parse(text)


def load_file(path):
    with open(path, encoding="utf-8") as f:
        return load(f.read())


_NUM = re.compile(r"^[-+]?(\d+(\.\d*)?|\.\d+)([eE][-+]?\d+)?$")


def scalar(s: str):
    s = s.strip()
    if s == "" or s in ("~", "null", "Null", "NULL"):
        return None
    if s in ("true", "True", "TRUE", "yes", "on"):
        return True
    if s in ("false", "False", "FALSE", "no", "off"):
        return False
    if s[0] in "\"'":
        if s[0] == '"':
            return json.loads(s)
        return s[1:-1].replace("''", "'")
    if s[0] in "[{":
        return _flow(s)
    if _NUM.match(s):
        return int(s) if re.fullmatch(r"[-+]?\d+", s) else float(s)
    return s


def _split_flow(body: str) -> list[str]:
    parts, depth, cur, quote = [], 0, "", None
    for ch in body:
        if quote:
            cur += ch
            if ch == quote:
                quote = None
        elif ch in "\"'":
            quote, cur = ch, cur + ch
        elif ch in "[{":
            depth, cur = depth + 1, cur + ch
        elif ch in "]}":
            depth, cur = depth - 1, cur + ch
        elif ch == "," and depth == 0:
            parts.append(cur)
            cur = ""
        else:
            cur += ch
    if cur.strip():
        parts.append(cur)
    return parts


def _flow(s: str):
    body = s[1:-1].strip()
    items = _split_flow(body) if body else []
    if s[0] == "[":
        return [scalar(x) for x in items]
    out = {}
    for it in items:
        k, _, v = it.partition(":")
        out[str(scalar(k))] = scalar(v)
    return out


def _strip_comment(line: str) -> str:
    quote = None
    for i, ch in enumerate(line):
        if quote:
            if ch == quote:
                quote = None
        elif ch in "\"'":
            quote = ch
        elif ch == "#" and (i == 0 or line[i - 1] in " \t"):
            return line[:i].rstrip()
    return line.rstrip()


def parse(text: str):
    lines = []
    for raw in text.splitlines():
        line = _strip_comment(raw)
        if line.strip() and line.strip() != "---":
            lines.append((len(line) - len(line.lstrip(" ")), line.strip()))
    value, pos = _block(lines, 0, lines[0][0] if lines else 0)
    if pos != len(lines):
        raise ValueError(f"unexpected indentation near: {lines[pos][1]!r}")
    return value


def _block(lines, pos, indent):
    if pos >= len(lines):
        return None, pos
    if lines[pos][1].startswith("- ") or lines[pos][1] == "-":
        out = []
        while pos < len(lines) and lines[pos][0] == indent and (lines[pos][1].startswith("- ") or lines[pos][1] == "-"):
            rest = lines[pos][1][1:].strip()
            if not rest:
                v, pos = _block(lines, pos + 1, lines[pos + 1][0]) if pos + 1 < len(lines) and lines[pos + 1][0] > indent else (None, pos + 1)
                out.append(v)
            elif re.match(r"^[^\"'\[{][^:]*:(\s|$)", rest):
                # "- key: value" opens a mapping whose further keys sit at indent + 2
                sub = [(indent + 2, rest)]
                j = pos + 1
                while j < len(lines) and lines[j][0] > indent:
                    sub.append(lines[j])
                    j += 1
                v, _ = _block(sub, 0, indent + 2)
                out.append(v)
                pos = j
            else:
                out.append(scalar(rest))
                pos += 1
        return out, pos
    out = {}
    while pos < len(lines) and lines[pos][0] == indent:
        key, sep, rest = lines[pos][1].partition(":")
        if not sep:
            raise ValueError(f"expected 'key: value', got {lines[pos][1]!r}")
        key = str(scalar(key))
        if rest.strip():
            out[key] = scalar(rest)
            pos += 1
        elif pos + 1 < len(lines) and lines[pos + 1][0] > indent:
            out[key], pos = _block(lines, pos + 1, lines[pos + 1][0])
        elif pos + 1 < len(lines) and lines[pos + 1][0] == indent and lines[pos + 1][1].startswith("-"):
            out[key], pos = _block(lines, pos + 1, indent)  # list at the key's own indent
        else:
            out[key] = None
            pos += 1
    return out, pos
