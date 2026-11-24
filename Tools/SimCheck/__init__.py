"""Acceptance checks of the T2 (unity) player contract, stdlib only.

    python3 -m Tools.SimCheck axis RESULT.json              # FLU command axes and yaw feedback
    python3 -m Tools.SimCheck sitl-peer --output PEER.json  # drive the player's SITL UDP port
    python3 -m Tools.SimCheck sitl RESULT_ROOT              # SITL UDP acceptance from peer + player result
    python3 -m Tools.SimCheck docking RESULT.json [--goal X Y YAW]   # rotate-to-path and settling
    python3 -m Tools.SimCheck smoke SESSION_DIR [--duration S]       # rates, spans, travel, thrust, truth

Each check prints one JSON report ``{check, passed, failures, ...}`` and exits 1 when it fails.
Numbers come from ``thresholds.yaml`` beside this file. ``watchdog.Watchdog`` is the in-run stop
rule (hang, stall, no motion) for a T2 run's supervisor.
"""

from pathlib import Path

THRESHOLDS = Path(__file__).with_name('thresholds.yaml')


def _scalar(text):
    text = text.strip()
    if text.startswith('[') and text.endswith(']'):
        inner = text[1:-1].strip()
        return [_scalar(part) for part in inner.split(',')] if inner else []
    if text in ('', '~', 'null'):
        return None
    if text in ('true', 'false'):
        return text == 'true'
    for cast in (int, float):
        try:
            return cast(text)
        except ValueError:
            pass
    return text.strip('\'"')


def load_yaml(text):
    """Nested block mappings of scalars and flow lists: the YAML subset thresholds.yaml uses."""
    root = {}
    stack = [(-1, root)]
    for raw in text.splitlines():
        line = '' if raw.lstrip().startswith('#') else raw.split(' #')[0].rstrip()
        if not line.strip():
            continue
        indent = len(line) - len(line.lstrip())
        key, _, value = line.strip().partition(':')
        while indent <= stack[-1][0]:
            stack.pop()
        if value.strip():
            stack[-1][1][key] = _scalar(value)
        else:
            stack[-1][1][key] = {}
            stack.append((indent, stack[-1][1][key]))
    return root


def thresholds(path=None):
    return load_yaml(Path(path or THRESHOLDS).read_text(encoding='utf-8'))


def report(check, failures, **facts):
    return {'check': check, 'passed': not failures, 'failures': list(failures), **facts}
