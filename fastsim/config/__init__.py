"""Configuration: the core defaults merged with a domain YAML, and a calibration epoch applied."""

import copy
from pathlib import Path

import yaml

from fastsim.identity import sha256

DEFAULT = Path(__file__).with_name('default.yaml')


def merge(base, over):
    out = copy.deepcopy(base)
    for key, value in (over or {}).items():
        out[key] = merge(out[key], value) if isinstance(out.get(key), dict) and isinstance(value, dict) \
            else copy.deepcopy(value)
    return out


def defaults():
    return yaml.safe_load(DEFAULT.read_text())


def load(path=None, overrides=None):
    """(config, base folder) of a domain YAML (or the defaults alone), calibration applied."""
    base = Path(path).resolve().parent if path else Path.cwd()
    domain = yaml.safe_load(Path(path).read_text()) if path else {}
    return resolve(merge(domain or {}, overrides), base), base


def resolve(domain, base):
    cfg = defaults()
    unknown = sorted(set(domain) - set(cfg))
    if unknown:
        raise ValueError(f'unknown config keys {unknown}; known: {sorted(cfg)}')
    cfg = merge(cfg, domain)
    cal = cfg.get('calibration')
    if isinstance(cal, (str, Path)):
        path = Path(base) / Path(cal).expanduser()
        epoch = yaml.safe_load(path.read_text())
        cfg['vehicle'] = merge(cfg['vehicle'], epoch['plant'])
        cfg['calibration'] = {'file': str(path), 'epoch': epoch.get('epoch'),
                              'basis': epoch.get('basis'), 'sha256': sha256(path.read_bytes())}
    cfg['workers'] = max(1, min(int(cfg['workers']), int(cfg['max_workers'])))
    return cfg
