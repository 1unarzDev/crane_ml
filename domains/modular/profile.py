"""Plant profiles of the modular domain: one card per plate state (variants), theta overrides and DR ranges.

    prof = load('example#plated')                  # domains/modular/profiles/example.json, variant 'plated'
    prof = load('/path/card.json', sets={'scale.stall': 1.1, 'floor.mu': 0.7})
    draws = randomize(prof, seeds)                 # one profile per seed, drawn in prof['dr'] ranges

A profile (``crane.modular.profile/1``) names every physical number the kernel reads:

    floor    mu, kappa_lat                                         the floor (conventional lateral = mu kappa_lat)
    wheel    roller_mu_f mu_rb roller_dev_deg s_x0 alpha0 c_rr k_v K tau_stall_Nm w_nl_rad_s b_visc tau_c tau_bd
             phi_max_rad_s cpr                                     per driven wheel (the truth model's names)
    caster   c_rr k_v
    module   mass_kg ({type: kg} or one number), inertia_f, com [dx, dy]
    scale    stall noload kp radius izz mass                       multipliers (theta-style overrides)
    latency_s                                                      command latency (whole plant steps)
    variants {NAME: {dotted.key: value}}                           e.g. a plate state; REF#NAME picks one
    dr       {dotted.key: [lo, hi]}                                uniform per-env draws (randomize)

Overrides (``sets``) are dotted keys over the resolved profile. Unknown keys are refused with the list of known ones.
"""

import copy
import json
from pathlib import Path

import numpy as np

HERE = Path(__file__).resolve().parent
PROFILES = HERE / 'profiles'
SCHEMA = 'crane.modular.profile/1'
KEYS = {
    'floor': ('mu', 'kappa_lat'),
    'wheel': ('roller_mu_f', 'mu_rb', 'roller_dev_deg', 's_x0', 'alpha0', 'c_rr', 'k_v', 'K', 'tau_stall_Nm',
              'w_nl_rad_s', 'b_visc', 'tau_c', 'tau_bd', 'phi_max_rad_s', 'cpr'),
    'caster': ('c_rr', 'k_v'),
    'module': ('mass_kg', 'inertia_f', 'com'),
    'scale': ('stall', 'noload', 'kp', 'radius', 'izz', 'mass'),
}
SCALE_DEFAULT = dict.fromkeys(KEYS['scale'], 1.0)


class ProfileError(ValueError):
    """A profile that cannot be used; one line naming the fix."""


def path_of(ref):
    path = Path(str(ref)).expanduser()
    if not path.suffix:
        path = PROFILES / f'{ref}.json'
    if not path.is_file():
        known = ', '.join(sorted(p.stem for p in PROFILES.glob('*.json')))
        raise ProfileError(f'no profile {ref!r} ({path} is missing); profiles: {known}')
    return path


def _set(doc, key, value, where):
    parts = key.split('.')
    if parts[0] == 'latency_s' and len(parts) == 1:
        doc['latency_s'] = float(value)
        return
    if parts[0] not in KEYS or len(parts) < 2 or parts[1] not in KEYS[parts[0]]:
        known = ', '.join(['latency_s'] + [f'{s}.{k}' for s, ks in KEYS.items() for k in ks])
        raise ProfileError(f'{where}: no profile key {key!r}; keys: {known}')
    node = doc.setdefault(parts[0], {})
    for p in parts[1:-1]:
        if not isinstance(node.get(p), dict):
            node[p] = {}
        node = node[p]
    node[parts[-1]] = value


def resolve(doc, variant=None, sets=None, where='profile'):
    """The profile with its variant and overrides applied, every section present; raises ProfileError."""
    if doc.get('schema') != SCHEMA:
        raise ProfileError(f'{where}: schema {doc.get("schema")!r}, want {SCHEMA}')
    out = copy.deepcopy({k: v for k, v in doc.items() if k not in ('variants', 'dr')})
    out['scale'] = {**SCALE_DEFAULT, **(out.get('scale') or {})}
    out.setdefault('latency_s', 0.0)
    out.setdefault('caster', {'c_rr': 0.04, 'k_v': 6.0e4})
    variants = doc.get('variants') or {}
    if variant:
        if variant not in variants:
            raise ProfileError(f'{where}: no variant {variant!r}; variants: {", ".join(variants) or "none"}')
        for k, v in variants[variant].items():
            _set(out, k, v, f'{where}#{variant}')
    for k, v in (sets or {}).items():
        _set(out, k, v, f'{where} override')
    for section, keys in KEYS.items():
        missing = [k for k in keys if k not in (out.get(section) or {}) and not (section == 'module' and k == 'com')]
        if missing:
            raise ProfileError(f'{where}: {section}.{missing[0]} is missing; add it to the profile')
    out['module'].setdefault('com', [0.0, 0.0])
    out['variant'] = variant
    out['dr'] = dict(doc.get('dr') or {})
    for k in out['dr']:
        _set(copy.deepcopy(out), k, 0.0, f'{where} dr')
    return out


def load(ref, sets=None):
    """A resolved profile from REF (a path or a name in domains/modular/profiles), REF#VARIANT for a variant."""
    if isinstance(ref, dict):
        return resolve(ref, None, sets)
    text, _, variant = str(ref).partition('#')
    path = path_of(text)
    prof = resolve(json.loads(path.read_text()), variant or None, sets, where=path.name)
    prof['source'] = {'file': path.name, 'variant': variant or None}
    return prof


def _get(doc, key):
    node = doc
    for p in key.split('.'):
        node = node[p]
    return node


def randomize(prof, seeds, stream='modular.dr'):
    """One profile per seed: every ``dr`` key drawn uniformly in its range from the seed's own stream."""
    from fastsim.identity import rng
    out = []
    for s in seeds:
        r = rng(int(s), stream)
        d = copy.deepcopy(prof)
        for k, (lo, hi) in sorted(prof['dr'].items()):
            _set(d, k, float(r.uniform(float(lo), float(hi))), 'dr')
        d['drawn'] = {k: float(_get(d, k)) for k in sorted(prof['dr'])}
        out.append(d)
    return out


def latency_steps(prof, dt):
    return int(round(float(prof['latency_s']) / dt))


def card(prof):
    """The plant card's numbers as flat {dotted key: value} (what a run records about its plant)."""
    out = {}
    for section in KEYS:
        for k, v in (prof.get(section) or {}).items():
            if isinstance(v, dict):
                out.update({f'{section}.{k}.{q}': w for q, w in v.items()})
            elif isinstance(v, (list, tuple)):
                out.update({f'{section}.{k}.{i}': w for i, w in enumerate(v)})
            else:
                out[f'{section}.{k}'] = v
    out['latency_s'] = prof['latency_s']
    return {k: float(v) for k, v in out.items() if isinstance(v, (int, float, np.floating))}
