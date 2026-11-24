"""A domain's view spec: the ``view:`` section of ``config/domains/<domain>.yaml``.

    view:
      frame: {world: enu, origin: scenario, body: flu}    the frame its runs are drawn in
      outline: [[x, y], ...]                              the vehicle outline, body frame (m)
      layers: [elements, gates, footprints, boundary, constraints]   world layers drawn, in order
      lanes: {phase: mission, goal: goal, contact: judge} event kind -> timeline lane
      card: [vehicle.profile, adapter.options.profile]    config keys naming the plant card file
      current: vehicle.current                            config key taking the current [cx, cy] (map frame)
      params:                                             the live-mode sliders
        - {name, label, unit, min, max, step, default, bind, tiers: [T1]}

``bind`` says where a slider goes:
  ``config:<dotted.key>``   a key of the tier's fastsim config (an int last part indexes a list)
  ``card:<dotted.key>``     a number in the plant card file
  ``disturbance:<name>``    current_speed (m/s), current_toward (deg, ENU, the way the water flows),
                            delay_s (command delay), pose_noise_m, yaw_noise_deg (on what the controller sees)

A domain without a ``view:`` section gets ``DEFAULT``.
"""

from __future__ import annotations

from pathlib import Path

from crane import paths

DEFAULT = {'frame': {'world': 'enu', 'origin': 'world', 'body': 'flu'},
           'outline': [[0.5, 0.0], [0.25, 0.3], [-0.5, 0.3], [-0.5, -0.3], [0.25, -0.3]],
           'layers': ['elements', 'gates', 'footprints', 'boundary', 'constraints'],
           'lanes': {}, 'card': [], 'current': None, 'params': []}
DISTURBANCES = ('current_speed', 'current_toward', 'delay_s', 'pose_noise_m', 'yaw_noise_deg')
BASE_LANES = {'contact': 'judge', 'near_miss': 'judge', 'strike': 'judge', 'failure': 'judge', 'row': 'judge',
              'log': 'log'}


class SpecError(ValueError):
    pass


def domain_file(domain: str) -> Path:
    return paths.DOMAINS / f'{domain}.yaml'


def load_domain(domain: str) -> dict:
    import yaml
    p = domain_file(domain)
    if not p.is_file():
        known = ', '.join(sorted(q.stem for q in paths.DOMAINS.glob('*.yaml')))
        raise SpecError(f'no domain {domain!r} ({p}); domains: {known}')
    return yaml.safe_load(p.read_text()) or {}


def view_spec(domain: str | None) -> dict:
    """The domain's view spec with the defaults filled in (``DEFAULT`` for an unknown or empty domain)."""
    raw = {}
    if domain and domain_file(domain).is_file():
        raw = load_domain(domain).get('view') or {}
    spec = {**DEFAULT, **raw}
    spec['lanes'] = {**BASE_LANES, **(raw.get('lanes') or {})}
    for p in spec['params']:
        check_param(p)
    return spec


def check_param(p: dict) -> dict:
    for k in ('name', 'bind', 'min', 'max'):
        if k not in p:
            raise SpecError(f'view param {p.get("name", "?")}: {k} is required')
    kind, _, key = str(p['bind']).partition(':')
    if kind not in ('config', 'card', 'disturbance') or not key:
        raise SpecError(f"view param {p['name']}: bind {p['bind']!r} is config:KEY, card:KEY or disturbance:NAME")
    if kind == 'disturbance' and key not in DISTURBANCES:
        raise SpecError(f"view param {p['name']}: disturbance {key!r} is not one of {DISTURBANCES}")
    return p


def params_for(spec: dict, tier: str) -> list:
    """The sliders offered at ``tier``, each with its default."""
    return [{'step': (p['max'] - p['min']) / 100, 'default': p['min'], 'unit': '', 'label': p['name'], **p}
            for p in spec['params'] if tier in p.get('tiers', (tier,))]


def lane_of(event: dict, lanes: dict) -> str:
    """The timeline lane of an event: its own ``lane``, else the spec's mapping of its kind, else ``events``."""
    return event.get('lane') or lanes.get(event.get('kind')) or 'events'
