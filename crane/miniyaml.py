"""YAML for the stdlib-only paths (crane sim, crane help): PyYAML when installed, else a small subset reader.

    from crane import miniyaml
    cfg = miniyaml.safe_load(path.read_text())

The subset is what crane's own config files use: block mappings and sequences by indentation, flow lists and maps
(``[a, b]``, ``{k: v}``, which may span lines), single- and double-quoted strings, plain scalars typed as PyYAML
types them (null, bool, int, float, str) and ``#`` comments. Anchors, tags, block scalars (``|``, ``>``) and
several documents raise ValueError naming PyYAML (``./install.sh --components core``). tests/test_libs.py keeps the
reader equal to PyYAML on every file under config/.
"""

import re

__all__ = ['safe_load', 'load']

_INT = re.compile(r'[-+]?(0|[1-9][0-9_]*)$')
_FLOAT = re.compile(r'[-+]?(\.[0-9]+|[0-9][0-9_]*(\.[0-9_]*)?)([eE][-+][0-9]+)?$')
_BOOL = {**dict.fromkeys(('true', 'True', 'TRUE', 'yes', 'Yes', 'YES', 'on', 'On', 'ON'), True),
         **dict.fromkeys(('false', 'False', 'FALSE', 'no', 'No', 'NO', 'off', 'Off', 'OFF'), False)}
_NULL = ('', '~', 'null', 'Null', 'NULL')
FIX = 'install PyYAML: ./install.sh --components core'


def safe_load(text):
    """PyYAML's safe_load when it is installed, else the subset reader."""
    try:
        import yaml
    except ImportError:
        return load(text)
    return yaml.safe_load(text)


def load(text):
    """The subset reader (see the module doc). ValueError on anything outside it."""
    lines = _logical_lines(text)
    if not lines:
        return None
    value, i = _block(lines, 0, lines[0][0])
    if i != len(lines):
        raise ValueError(f'line {lines[i][2]}: unexpected indentation; {FIX}')
    return value


def _strip_comment(line):
    quote = None
    for i, c in enumerate(line):
        if quote:
            if c == quote:
                quote = None
        elif c in '\'"' and (i == 0 or line[i - 1] in ' [{,:'):
            quote = c
        elif c == '#' and (i == 0 or line[i - 1] in ' \t'):
            return line[:i].rstrip()
    return line.rstrip()


def _depth(text):
    """Open flow brackets in text, outside quotes."""
    depth, quote = 0, None
    for i, c in enumerate(text):
        if quote:
            if c == quote:
                quote = None
        elif c in '\'"' and (i == 0 or text[i - 1] in ' [{,:'):
            quote = c
        elif c in '[{':
            depth += 1
        elif c in ']}':
            depth -= 1
    return depth


def _logical_lines(text):
    """[(indent, text, line number)], comments and blank lines dropped, open flow collections joined."""
    out, pending = [], None
    for n, raw in enumerate(text.splitlines(), 1):
        if raw.startswith(('---', '...')) and not out and pending is None and raw.strip() in ('---', '...'):
            continue
        if raw.strip() in ('---', '...'):
            raise ValueError(f'line {n}: several YAML documents; {FIX}')
        line = _strip_comment(raw)
        if not line.strip():
            continue
        if '\t' in raw[:len(raw) - len(raw.lstrip())]:
            raise ValueError(f'line {n}: a tab in the indentation')
        if pending is not None:
            pending[1] += ' ' + line.strip()
            if _depth(pending[1]) <= 0:
                out.append(tuple(pending))
                pending = None
            continue
        entry = [len(line) - len(line.lstrip()), line.strip(), n]
        if _depth(entry[1]) > 0:
            pending = entry
        else:
            out.append(tuple(entry))
    if pending is not None:
        raise ValueError(f'line {pending[2]}: an unclosed [ or {{')
    return out


def _split_key(text, n):
    """'key: value' -> (key, value or None); None when text is not a mapping entry."""
    if text[:1] in '\'"':
        end = _quoted_end(text, 0, n)
        key, rest = _scalar(text[:end], n), text[end:]
        if rest == ':' or rest.startswith(': '):
            return key, rest[1:].strip() or None
        return None
    m = re.search(r':( |$)', text)
    if not m or text[:1] in '[{':
        return None
    return _plain(text[:m.start()].strip()), text[m.end():].strip() or None


def _block(lines, i, indent):
    if lines[i][1] == '-' or lines[i][1].startswith('- '):
        return _sequence(lines, i, indent)
    return _mapping(lines, i, indent)


def _nested(lines, i, indent, n):
    """The value under a 'key:' or '-' with nothing after it: a deeper block, a same-indent sequence, or null."""
    if i < len(lines) and (lines[i][0] > indent or (lines[i][0] == indent and (lines[i][1] == '-'
                                                                                or lines[i][1].startswith('- ')))):
        return _block(lines, i, lines[i][0])
    return None, i


def _mapping(lines, i, indent):
    out = {}
    while i < len(lines) and lines[i][0] == indent:
        _, text, n = lines[i]
        if text == '-' or text.startswith('- '):
            break
        kv = _split_key(text, n)
        if kv is None:
            raise ValueError(f'line {n}: expected KEY: VALUE, got {text!r}')
        key, value = kv
        i += 1
        if value is None:
            value, i = _nested(lines, i, indent, n)
            if value is None and i < len(lines) and lines[i][0] == indent and lines[i][1].startswith('- '):
                value, i = _sequence(lines, i, indent)
        else:
            value = _value(value, n)
        out[key] = value
    if i < len(lines) and lines[i][0] > indent:
        raise ValueError(f'line {lines[i][2]}: unexpected indentation')
    return out, i


def _sequence(lines, i, indent):
    out = []
    while i < len(lines) and lines[i][0] == indent and (lines[i][1] == '-' or lines[i][1].startswith('- ')):
        _, text, n = lines[i]
        rest = text[1:].strip()
        i += 1
        if not rest:
            value, i = _nested(lines, i, indent, n)
        elif _split_key(rest, n) is not None and rest[:1] not in '[{':
            # '- key: value' opens a mapping whose further keys sit at the item's text column
            col = indent + len(text) - len(rest)
            value, used = _mapping([(col, rest, n)] + lines[i:], 0, col)
            i += used - 1
        else:
            value = _value(rest, n)
        out.append(value)
    return out, i


def _value(text, n):
    if text[:1] in '|>&*!':
        raise ValueError(f'line {n}: {text[:1]!r} (block scalars, anchors and tags) is outside the subset; {FIX}')
    if text[:1] in '[{':
        value, end = _flow(text, 0, n)
        if text[end:].strip():
            raise ValueError(f'line {n}: text after a flow collection: {text[end:]!r}')
        return value
    if text[:1] in '\'"':
        end = _quoted_end(text, 0, n)
        if text[end:].strip():
            raise ValueError(f'line {n}: text after a quoted string: {text[end:]!r}')
        return _scalar(text, n)
    return _plain(text)


def _quoted_end(text, i, n):
    q = text[i]
    j = i + 1
    while j < len(text):
        if text[j] == q:
            if q == "'" and text[j + 1:j + 2] == "'":
                j += 2
                continue
            return j + 1
        if q == '"' and text[j] == '\\':
            j += 2
            continue
        j += 1
    raise ValueError(f'line {n}: an unclosed {q}')


def _scalar(token, n):
    """A quoted token as its string."""
    if token[0] == "'":
        return token[1:-1].replace("''", "'")
    import json
    try:
        return json.loads(token)
    except ValueError:
        raise ValueError(f'line {n}: a double-quoted string outside the subset: {token}') from None


def _plain(text):
    if text in _NULL:
        return None
    if text in _BOOL:
        return _BOOL[text]
    if _INT.match(text):
        return int(text.replace('_', ''))
    if text in ('.inf', '.Inf', '+.inf', '-.inf', '-.Inf', '.nan', '.NaN'):
        return float(text.replace('.', '', 1).replace('Inf', 'inf').replace('NaN', 'nan'))
    if _FLOAT.match(text) and any(c.isdigit() for c in text) and ('.' in text):
        return float(text.replace('_', ''))
    return text


def _skip(text, i):
    while i < len(text) and text[i] == ' ':
        i += 1
    return i


def _flow(text, i, n):
    """A flow collection or scalar at text[i]: (value, index after it)."""
    i = _skip(text, i)
    c = text[i:i + 1]
    if c == '[':
        out, i = [], _skip(text, i + 1)
        while text[i:i + 1] != ']':
            item, i = _flow(text, i, n)
            out.append(item)
            i = _skip(text, i)
            if text[i:i + 1] == ',':
                i = _skip(text, i + 1)
            elif text[i:i + 1] != ']':
                raise ValueError(f'line {n}: expected , or ] in {text!r}')
        return out, i + 1
    if c == '{':
        out, i = {}, _skip(text, i + 1)
        while text[i:i + 1] != '}':
            key, i = _flow_scalar(text, i, n, key=True)
            i = _skip(text, i)
            if text[i:i + 1] != ':':
                raise ValueError(f'line {n}: expected : after {key!r} in {text!r}')
            i = _skip(text, i + 1)
            if text[i:i + 1] in (',', '}'):
                value = None
            else:
                value, i = _flow(text, i, n)
            out[key] = value
            i = _skip(text, i)
            if text[i:i + 1] == ',':
                i = _skip(text, i + 1)
            elif text[i:i + 1] != '}':
                raise ValueError(f'line {n}: expected , or }} in {text!r}')
        return out, i + 1
    return _flow_scalar(text, i, n)


def _flow_scalar(text, i, n, key=False):
    if text[i:i + 1] in ('"', "'"):
        end = _quoted_end(text, i, n)
        return _scalar(text[i:end], n), end
    j = i
    while j < len(text) and text[j] not in ',]}' and not (key and text[j] == ':' and text[j + 1:j + 2] in (' ', ',', '}', '')):
        if not key and text[j] == ':' and text[j + 1:j + 2] == ' ':
            break
        j += 1
    return _plain(text[i:j].strip()), j
