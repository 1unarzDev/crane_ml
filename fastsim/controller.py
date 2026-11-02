"""Where a domain plugs in: the navigator it drives, the controller it ticks, the adapter around it.

A ``DomainAdapter`` (referenced from YAML as ``adapter: {ref: MODULE:CLASS, options}``) is built
once per worker process. Per run it returns a ``Controller`` for the run's ``RunContext``; the
core ticks it at ``tick_hz`` with an ``Observation`` until it returns an outcome. The controller
acts only through ``ctx.navigator`` (goals, holds, a speed cap) and ``ctx.emit`` (its events).
After the run the adapter may write its own files (``finalize``) and score them (``metrics``).
"""

from abc import ABC, abstractmethod
from dataclasses import dataclass, field

from fastsim.plant import Goal


class Navigator:
    """The goal interface a controller drives; the core's tracker and plant execute it."""

    def __init__(self, limits, clock):
        self.lim, self.clock = limits, clock
        self.goal, self.last, self.report = None, None, None
        self.cap = None
        self.driven = self.held = None  # last time a goal drove the plant / a hold went out

    def send(self, route, precise=False, quiet=False, why='', reverse=None):
        """A new goal (a route of (x, y, yaw) map-frame poses); it preempts the live one."""
        if route and not quiet:
            x, y, yaw = route[-1]
            self.last = {'x': round(x, 2), 'y': round(y, 2), 'yaw': round(yaw, 2), 'why': why}
            if self.report is not None:
                self.report('goal', **self.last)
        if self.goal is not None:
            self.goal.end('ABORTED')
        self.goal = Goal(route, precise, why, reverse)
        return self.goal

    def hold(self):
        if self.goal is not None:
            self.goal.cancel()

    def speed_limit(self, speed):
        self.cap = speed if speed and speed > 0 else None

    def keep_still(self, quiet_s=0.3):
        """An active hold once no goal has driven for ``quiet_s`` (a reverse leg drives itself);
        without one the run counts idle time (a vehicle with no command would drift)."""
        goal, now = self.goal, self.clock['now']
        if goal is not None and not goal.done and goal.reverse is not None:
            return
        if self.driven is None or now - self.driven > quiet_s:
            self.held = now

    def advance(self, pose, moved, now):
        """After a plant step: distance on a reverse leg, and the progress checker (a goal that
        moves less than ``progress_radius`` for ``progress_s`` is aborted)."""
        goal = self.goal
        if goal is None or goal.done:
            return
        if goal.travelled is not None:
            goal.travelled += (moved[0] ** 2 + moved[1] ** 2) ** 0.5
        if goal.reverse is None:
            if goal.anchor is None or ((goal.anchor[0][0] - pose[0]) ** 2 + (goal.anchor[0][1] - pose[1]) ** 2) \
                    ** 0.5 > self.lim.progress_radius:
                goal.anchor = (pose[:2], now)
            elif now - goal.anchor[1] > self.lim.progress_s:
                goal.end('ABORTED')

    def ready(self):
        return True

    def close(self):
        self.hold()


@dataclass
class Observation:
    t: float
    pose: tuple              # map frame (x, y, yaw)
    tracks: list             # perception.Track, sorted by id
    fov: tuple = None        # the sensor's (hfov rad, range m), or None


@dataclass
class RunContext:
    seed: int
    scenario: object         # scenario.Scenario (raw file in .raw)
    world: object            # world.World
    navigator: Navigator
    limits: object           # plant.Limits
    rng: object              # numpy Generator of the 'controller' stream
    clock: dict              # {'now': sim seconds}
    events: list = field(default_factory=list)

    def emit(self, event, **fields):
        """A controller event, stamped with sim time, kept in the run's sim_events.jsonl."""
        self.events.append({'t': self.clock['now'], 'event': event, **fields})


class Controller(ABC):
    @abstractmethod
    def tick(self, obs: Observation):
        """Decide; return None to go on, or the run's outcome (e.g. 'SUCCEEDED', 'FAILED')."""

    def close(self):
        pass


class DomainAdapter(ABC):
    """Built once per worker as ``Adapter(base=<the YAML's folder>, **adapter.options)``."""

    def __init__(self, base=None, **options):
        self.base, self.options = base, options

    def identity(self):
        """What makes this domain's runs differ (versions, parameter digests): hashed into
        every run identity and recorded with it."""
        return {}

    def limits(self):
        """The vehicle's Limits, or None to take ``limits`` from the config."""
        return None

    def overrides(self):
        """Config read from the domain's own files at run time (e.g. ``perception`` from a
        robot description, ``tracker`` from a tracker config), merged over the YAML for every
        run and so recorded and hashed with it."""
        return {}

    @abstractmethod
    def controller(self, ctx: RunContext) -> Controller:
        """The controller of one run."""

    def time_limit(self, ctx):
        """The run's time limit (s), or None for the config's ``time_limit_s``."""
        return None

    def condition(self, ctx):
        """What else must match for a twin run to pair with this one (e.g. task, profile);
        the run's time limit is always part of it."""
        return {}

    def finalize(self, run_dir, result):
        """Write domain files into the run directory (``result`` is the run summary)."""

    def metrics(self, run_dir):
        """Numeric results of a run directory (any nesting; numbers are flattened)."""
        return {}
