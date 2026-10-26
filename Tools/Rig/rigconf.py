"""Load the rig configuration: rig.yaml, then a host overlay, with ${VAR} expansion.

Host-specific values (player, courses, container image, mounts) live outside the repo:
  1. Tools/Rig/rig.yaml           checked-in defaults
  2. $CRANE_RIG_LOCAL, else rig.local.yaml next to the config (gitignored), merged on top
String values expand ${VAR}, ${VAR:-default} and a leading ~.

Usage from a shell: python3 -I rigconf.py CONFIG [KEY]  -> JSON of the merged config (or one key).
"""

import json
import os
import re
import sys
from pathlib import Path

import yaml

_VAR = re.compile(r'\$\{([A-Za-z_][A-Za-z0-9_]*)(?::-([^}]*))?\}')


def expand(value):
    """Expand ${VAR} / ${VAR:-default} and ~ in every string inside ``value``."""
    if isinstance(value, str):
        text = _VAR.sub(lambda m: os.environ.get(m.group(1)) or (m.group(2) or ''), value)
        return os.path.expanduser(text) if text.startswith('~') else text
    if isinstance(value, list):
        return [expand(v) for v in value]
    if isinstance(value, dict):
        return {k: expand(v) for k, v in value.items()}
    return value


def merge(base, over):
    """Deep-merge dict ``over`` onto ``base`` (lists and scalars replace)."""
    out = dict(base)
    for key, value in (over or {}).items():
        out[key] = merge(out[key], value) if isinstance(out.get(key), dict) and isinstance(value, dict) else value
    return out


def local_path(config):
    env = os.environ.get('CRANE_RIG_LOCAL')
    return Path(env) if env else Path(config).with_name('rig.local.yaml')


def load(config):
    config = Path(config)
    rig = yaml.safe_load(config.read_text()) or {}
    local = local_path(config)
    if local.is_file() and local.resolve() != config.resolve():
        rig = merge(rig, yaml.safe_load(local.read_text()) or {})
    return expand(rig)


if __name__ == '__main__':
    rig = load(sys.argv[1])
    print(json.dumps(rig[sys.argv[2]] if len(sys.argv) > 2 else rig))
