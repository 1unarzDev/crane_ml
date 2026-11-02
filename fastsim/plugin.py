"""Plugin references from YAML: ``package.module:attr`` or ``path/to/file.py:attr``.

Every domain specific (scenario format checks, the controller adapter, twin describers) enters
the core through one of these, so core code names no domain.
"""

import importlib
import importlib.util
from pathlib import Path
import sys


def load(ref, base=None):
    """The object ``ref`` names; a file path is taken relative to ``base`` (the YAML's folder)."""
    if not isinstance(ref, str):
        return ref  # already an object (tests, programmatic use)
    target, sep, attr = ref.rpartition(':')
    if not sep or not target or not attr:
        raise ValueError(f'plugin reference {ref!r}: expected MODULE:ATTR or FILE.py:ATTR')
    if target.endswith('.py'):
        path = Path(target).expanduser()
        if not path.is_absolute() and base is not None:
            path = Path(base) / path
        name = f'_fastsim_plugin_{abs(hash(str(path.resolve())))}'
        module = sys.modules.get(name)
        if module is None:
            spec = importlib.util.spec_from_file_location(name, path)
            module = importlib.util.module_from_spec(spec)
            sys.modules[name] = module
            spec.loader.exec_module(module)
    else:
        module = importlib.import_module(target)
    obj = module
    for part in attr.split('.'):
        obj = getattr(obj, part)
    return obj


def build(spec, registry, where=None, **kwargs):
    """An instance from ``{name|ref: ..., **options}``: a registry name or a plugin reference
    (a file reference resolved in ``where``); ``kwargs`` go to the factory."""
    spec = dict(spec or {})
    name = spec.pop('name', None)
    ref = spec.pop('ref', None)
    if (name is None) == (ref is None):
        raise ValueError(f'give exactly one of name ({sorted(registry)}) or ref: {spec}')
    if name is not None and name not in registry:
        raise ValueError(f'unknown {name!r}; known: {sorted(registry)}')
    factory = registry[name] if name is not None else load(ref, where)
    return factory(**spec, **kwargs)
