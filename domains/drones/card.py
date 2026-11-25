# SPDX-License-Identifier: GPL-3.0-or-later
"""Aerial plant cards: cited vehicle-profile JSON (schema crane-aerial-vehicle-profile-v1) to a model.Vehicle.

    doc = card.read('x500')                             # a name in domains/drones/profiles, or a path
    doc = card.override(doc, {'mass': 2.4, 'tau_up': 0.02})   # a card variant (a study level): values replaced
    v = card.vehicle(doc)                               # model.Vehicle (battery when the card has one)

Provenance: platform/drones Tools/Aerial/aerial_profiles.py at 56dceb05 (validate_profile, load_vehicle), with
the profile folder of this domain and a card variant (``override``) added.
"""

import copy
import hashlib
import json
from pathlib import Path

import numpy as np

from domains.drones.model import Battery, Rotor, Vehicle

PROFILE_DIR = Path(__file__).resolve().parent / 'profiles'
ROOT = Path(__file__).resolve().parents[2]
SCHEMA = 'crane-aerial-vehicle-profile-v1'
REQUIRED = ('mass', 'inertia', 'kf', 'km', 'tau_up', 'tau_down', 'omega_max',
            'rotor_drag', 'rolling_moment', 'rotor_radius', 'drag_linear', 'drag_quadratic')
STATUSES = {'MEASURED_PUBLISHED', 'VENDOR_SPEC', 'VENDOR_SIM_MODEL_CITING_SYSID', 'SIMULATOR_DEFAULT', 'DERIVED',
            'ASSUMED', 'UNKNOWN', 'NOT_REPRESENTABLE', 'MEASURED_DATASHEET', 'LITERATURE', 'ESTIMATED', 'VARIANT'}


def default():
    """The domain's default card name (config/domains/drones.yaml card.default)."""
    from crane import miniyaml
    doc = miniyaml.safe_load((ROOT / 'config' / 'domains' / 'drones.yaml').read_text())
    return doc['card']['default']


def path_of(ref):
    """A card ref: a path (absolute, or from the cwd or the crane root), or a name in PROFILE_DIR."""
    path = Path(str(ref)).expanduser()
    if path.suffix:
        for p in (path, ROOT / path):
            if p.is_file():
                return p.resolve()
        raise FileNotFoundError(f'no aerial card {ref} (looked in . and {ROOT})')
    p = PROFILE_DIR / f'{ref}.json'
    if not p.is_file():
        known = ', '.join(sorted(q.stem for q in PROFILE_DIR.glob('*.json')))
        raise FileNotFoundError(f'no aerial card {ref!r} in {PROFILE_DIR}; cards: {known}')
    return p


def read(ref=None):
    """The card JSON; a dict passes through; None is the domain's default card."""
    if isinstance(ref, dict):
        return ref
    ref = default() if ref is None else ref
    return json.loads(path_of(ref).read_text(encoding='utf-8'))


def sha256(doc):
    return hashlib.sha256(json.dumps(doc, sort_keys=True, separators=(',', ':')).encode()).hexdigest()


def validate(doc):
    problems = []
    if doc.get('schema') != SCHEMA:
        problems.append('schema')
    sources = doc.get('sources', {})
    params = doc.get('parameters', {})
    for key in REQUIRED:
        if key not in params:
            problems.append(f'missing parameter {key}')
    for key, entry in params.items():
        if entry.get('status') not in STATUSES:
            problems.append(f"{key}: bad status {entry.get('status')}")
        if not entry.get('locator'):
            problems.append(f'{key}: no locator')
        src = entry.get('source')
        if src is not None and src not in sources:
            problems.append(f'{key}: unknown source {src}')
        if src is None and entry.get('status') not in {'ASSUMED', 'UNKNOWN', 'SIMULATOR_DEFAULT', 'ESTIMATED',
                                                       'VARIANT'}:
            problems.append(f"{key}: status {entry.get('status')} requires a source")
    for name, src in sources.items():
        if 'license' not in src:
            problems.append(f'source {name}: no license')
    for rotor in doc.get('rotors', []):
        if rotor.get('source') not in sources:
            problems.append(f"rotor {rotor.get('name')}: unknown source")
        if rotor.get('spin') not in (-1, 1):
            problems.append(f"rotor {rotor.get('name')}: spin")
    return problems


def value(doc, key, default=None):
    entry = doc['parameters'].get(key)
    return default if entry is None else entry['value']


def override(doc, values):
    """A card variant: the card with parameter values replaced ({name: value}; a name the card lacks is an
    error). Each replaced entry is marked VARIANT with the value it replaced, so the variant says what it is."""
    doc = copy.deepcopy(read(doc))
    for key, val in (values or {}).items():
        if key not in doc['parameters']:
            raise KeyError(f"card {doc.get('id')} has no parameter {key!r}; it has: {', '.join(doc['parameters'])}")
        entry = doc['parameters'][key]
        doc['parameters'][key] = {**entry, 'value': val, 'status': 'VARIANT', 'source': None,
                                  'locator': f"variant of {doc.get('id')} (was {entry['value']!r})"}
    if values:
        doc['id'] = f"{doc.get('id')}+variant"
    return doc


def vehicle(ref, with_battery=True, ground_effect=False):
    """A model.Vehicle from a card (ref or dict); raises ValueError naming what the card lacks."""
    doc = read(ref)
    problems = validate(doc)
    if problems:
        raise ValueError(f"invalid aerial card {doc.get('id')}: {problems}")
    rotors = [Rotor(position=np.array(r['position'], dtype=float), spin=int(r['spin']),
                    kf=value(doc, 'kf'), km=value(doc, 'km'),
                    omega_max=value(doc, 'omega_max'), tau_up=value(doc, 'tau_up'),
                    tau_down=value(doc, 'tau_down'), rotor_drag=value(doc, 'rotor_drag'),
                    rolling_moment=value(doc, 'rolling_moment'),
                    radius=value(doc, 'rotor_radius'),
                    inertia=value(doc, 'rotor_inertia', 0.0))
              for r in doc['rotors']]
    battery = None
    if with_battery and 'battery_resistance' in doc['parameters']:
        battery = Battery(v_full=value(doc, 'battery_v_full'), v_empty=value(doc, 'battery_v_empty'),
                          v_nominal=value(doc, 'battery_v_nominal'),
                          resistance=value(doc, 'battery_resistance'),
                          capacity_ah=value(doc, 'battery_capacity_ah'),
                          efficiency=value(doc, 'battery_efficiency'))
    return Vehicle(name=doc['id'], mass=value(doc, 'mass'),
                   inertia=np.array(value(doc, 'inertia'), dtype=float), rotors=rotors,
                   drag_linear=np.array(value(doc, 'drag_linear'), dtype=float),
                   drag_quadratic=value(doc, 'drag_quadratic'), ground_effect=ground_effect,
                   battery=battery)
