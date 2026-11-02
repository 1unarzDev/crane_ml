"""Scenarios and their truth: one neutral in-memory form, read through a format adapter.

A ``Scenario`` is the start pose, the elements (disks, or boxes with length/width/yaw), the
boundary, the world's state programs and the generator identity, plus the file's exact bytes
(a run copies them byte for byte and hashes them). Formats:

  crane_truth  crane's course truth (schema version 2, the format Unity and the course
               generator write); ``identity_check`` is an optional plugin reference returning
               a list of problems for a truth dict (a non-empty list refuses the scenario)
  json         the core's own minimal format: {name, start: [x, y, yaw], elements: [{id, x, y,
               radius, label, state, length, width, yaw}], boundary, states, identity}
  (a plugin)   any ScenarioFormat subclass by reference
"""

from abc import ABC, abstractmethod
from dataclasses import dataclass, field
import json
import os
from pathlib import Path
import subprocess

from fastsim import plugin
from fastsim.identity import sha256


@dataclass(frozen=True)
class Element:
    id: str
    x: float
    y: float
    radius: float = 0.0
    label: str = ''          # what a sensor reads; an empty label is never perceived
    state: str = None        # the id of the state program that changes what it shows
    box: tuple = None        # (length, width, yaw) for a rectangular element, world frame


@dataclass
class Scenario:
    name: str
    start: tuple                      # (x, y, yaw) of the start pose, world frame
    elements: list
    data: bytes                       # the file's bytes, copied into every run unchanged
    boundary: list = field(default_factory=list)
    states: list = field(default_factory=list)   # [{id, initial, program: {type, ...}}]
    identity: dict = None             # generator identity: {pair_key, truth_sha256, seed, ...}
    filename: str = 'scenario.json'   # the name a run directory keeps the bytes under
    raw: dict = None                  # the parsed file, for the domain adapter

    @property
    def content_sha256(self):
        return sha256(self.data)

    @property
    def pair_key(self):
        """(pair_key, truth_sha256) twins are matched on; the file hash without an identity."""
        i = self.identity or {}
        return i.get('pair_key') or self.content_sha256, i.get('truth_sha256') or self.content_sha256

    def state_log(self):
        """The world's state log, [{t_sim, id, value}]: every state at its initial value, and a
        program with a ``trigger_s`` switching to its ``pattern`` (or 'on') then."""
        log = [{'t_sim': 0.0, 'id': st['id'], 'value': st.get('initial')} for st in self.states]
        for st in self.states:
            program = st.get('program') or {}
            if isinstance(program.get('trigger_s'), (int, float)):
                log.append({'t_sim': float(program['trigger_s']), 'id': st['id'],
                            'value': program.get('pattern') or 'on'})
        return sorted(log, key=lambda r: (r['t_sim'], r['id']))

    def program(self, ident):
        return next(st.get('program') or {} for st in self.states if st['id'] == ident)

    def changing_states(self):
        """States of perceived elements whose program is not fixed (the camera shows initials)."""
        return sorted(e.state for e in self.elements if e.label and e.state
                      and self.program(e.state).get('type') != 'fixed')


class ScenarioFormat(ABC):
    filename = 'scenario.json'

    def __init__(self, base=None):
        self.base = base  # the folder relative references in the format's options resolve in

    @abstractmethod
    def parse(self, data: bytes) -> Scenario:
        """A scenario from a file's bytes."""

    def check(self, scenario) -> list:
        """Problems that refuse the scenario (an edited or unpairable file); none by default."""
        return []

    def load(self, path):
        scenario = self.parse(Path(path).read_bytes())
        problems = self.check(scenario)
        if problems:
            raise ValueError(f"{path}: scenario identity: {'; '.join(problems)}")
        return scenario


def _element(e):
    box = (float(e['length']), float(e.get('width', 0.0)), float(e.get('yaw', 0.0))) \
        if 'length' in e else None
    return Element(str(e['id']), float(e['x']), float(e['y']), float(e.get('radius') or 0.0),
                   e.get('label') or '', e.get('state'), box)


class JsonScenarioFormat(ScenarioFormat):
    def parse(self, data):
        t = json.loads(data)
        return Scenario(t.get('name', ''), tuple(float(v) for v in t.get('start', (0, 0, 0))),
                        [_element(e) for e in t['elements']], data, t.get('boundary') or [],
                        t.get('states') or [], t.get('identity'), self.filename, t)


class CraneTruthFormat(ScenarioFormat):
    filename = 'course_truth.json'

    def __init__(self, identity_check=None, base=None):
        super().__init__(base)
        self.checker = plugin.load(identity_check, base) if identity_check else None

    def parse(self, data):
        t = json.loads(data)
        start = tuple(float(t.get(k, 0.0)) for k in ('startX', 'startY', 'startYaw'))
        return Scenario(t.get('name', ''), start, [_element(e) for e in t['elements']], data,
                        t.get('boundary') or [], t.get('states') or [],
                        (t.get('generator') or {}).get('identity'), self.filename, t)

    def check(self, scenario):
        return list(self.checker(scenario.raw)) if self.checker else []


FORMATS = {'crane_truth': CraneTruthFormat, 'json': JsonScenarioFormat}


def scenario_format(spec, base=None):
    """A format from its YAML spec ({name|ref, **options}); every format takes ``base``."""
    return plugin.build(spec or {'name': 'crane_truth'}, FORMATS, base, base=base)


class ScenarioSource:
    """Seed -> scenario file: ``dir`` + ``pattern`` (``{seed}`` and any key of ``fields``),
    and optionally a ``generate`` command template run once for the missing seeds
    (``{seeds_from}``, ``{seeds_to}`` (exclusive), ``{dir}`` and the fields substituted)."""

    def __init__(self, dir, pattern='scenario-s{seed}.json', fields=None, generate=None,
                 format=None, base=None):
        self.dir = Path(base or '.') / Path(os.path.expandvars(dir)).expanduser()
        self.pattern, self.fields, self.generate_cmd = pattern, dict(fields or {}), generate
        self.format = scenario_format(format, base)

    def path(self, seed):
        return self.dir / self.pattern.format(seed=seed, **self.fields)

    def generate(self, seeds):
        missing = [s for s in seeds if not self.path(s).exists()]
        if missing and self.generate_cmd:
            self.dir.mkdir(parents=True, exist_ok=True)
            words = dict(self.fields, seeds_from=min(missing), seeds_to=max(missing) + 1, dir=self.dir)
            subprocess.run([str(w).format(**words) for w in self.generate_cmd], check=True,
                           capture_output=True)
        return missing

    def load(self, seed):
        return self.format.load(self.path(seed))
