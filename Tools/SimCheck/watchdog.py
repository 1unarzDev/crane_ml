"""The in-run stop rule of a T2 run: when must a supervisor stop it, and why.

A progress probe in the run's ROS domain feeds a ``Tracker`` (clock, odometry and command
messages) and writes its ``snapshot`` about once a second. The supervisor calls
``Watchdog.check(now, snapshot, phase)`` every poll, with ``snapshot`` None when it could not be
read and ``phase`` 'starting' (before the course is ready), 'running' or 'post' (recording over),
and stops the run when it returns a reason:

  hard_timeout  wall time since launch over hard_s
  no_start      no probe data (no sim time, no topic data) within startup_s of launch
  stalled       neither sim time nor the message count changed for stall_s
  no_motion     motion commanded for motion_grace_s (or expect_motion and ready that long) and the
                boat moved less than min_travel_m: e.g. a drive held by a command mux
  post_timeout  post-processing over post_s

Limits default to thresholds.yaml ``watchdog``; a None limit disables that check.
"""

import math

from . import thresholds

CMD_EPS = 0.05     # |surge| or |sway| above this is a motion command (yaw alone is not)
CMD_GAP_S = 0.5    # consecutive motion commands closer than this add to cmd_active_s


class Watchdog:
    def __init__(self, started, **limits):
        defaults = thresholds()['watchdog']
        unknown = set(limits) - set(defaults)
        if unknown:
            raise ValueError(f'unknown watchdog limits {sorted(unknown)}')
        self.limits = {**defaults, **limits}
        self.started = started
        self.armed_at = self.ready_at = self.post_at = None
        self.last_key = None
        self.last_progress = started

    def check(self, now, snapshot, phase):
        lim = self.limits
        if lim['hard_s'] is not None and now - self.started > lim['hard_s']:
            return 'hard_timeout'
        if phase == 'post':
            self.post_at = self.post_at if self.post_at is not None else now
            if lim['post_s'] is not None and now - self.post_at > lim['post_s']:
                return 'post_timeout'
            return None
        if phase == 'running' and self.ready_at is None:
            self.ready_at = now
        if snapshot and (snapshot.get('msgs') or snapshot.get('sim_time') is not None):
            key = (snapshot.get('sim_time'), snapshot.get('msgs'))
            if key != self.last_key:
                self.last_key, self.last_progress = key, now
                if self.armed_at is None:
                    self.armed_at = now
        if self.armed_at is None:
            if lim['startup_s'] is not None and now - self.started > lim['startup_s']:
                return 'no_start'
            return None
        if lim['stall_s'] is not None and now - self.last_progress > lim['stall_s']:
            return 'stalled'
        return self._motion(now, snapshot or {})

    def _motion(self, now, snap):
        lim = self.limits
        if lim['motion_grace_s'] is None:
            return None
        if (snap.get('cmd_active_s') or 0) >= lim['motion_grace_s'] and \
                (snap.get('travel_since_cmd') or 0) < lim['min_travel_m']:
            return 'no_motion'
        if lim['expect_motion'] and self.ready_at is not None and \
                now - self.ready_at > lim['motion_grace_s'] and (snap.get('travel') or 0) < lim['min_travel_m']:
            return 'no_motion'
        return None


class Tracker:
    """The probe's pure state: sim time, message count, commanded-motion time and travel."""

    def __init__(self):
        self.sim_time = None
        self.msgs = 0
        self.start = self.odom = self.cmd_origin = None
        self.travel = 0.0
        self.travel_since_cmd = None
        self.cmd_active_s = 0.0
        self.last_cmd = None

    def on_clock(self, sim_time):
        self.msgs += 1
        self.sim_time = sim_time

    def on_odom(self, x, y):
        self.msgs += 1
        self.odom = (x, y)
        if self.start is None:
            self.start = self.odom
        self.travel = max(self.travel, math.dist(self.start, self.odom))
        if self.cmd_origin is not None:
            self.travel_since_cmd = max(self.travel_since_cmd or 0.0, math.dist(self.cmd_origin, self.odom))

    def on_command(self, now, surge, sway, yaw):
        # Only surge/sway count: a yaw-only command (spin, turn-in-place recovery) moves the
        # boat little, so it must not run the no-motion clock.
        if max(abs(surge), abs(sway)) <= CMD_EPS:
            self.last_cmd = None
            return
        if self.cmd_origin is None and self.odom is not None:
            self.cmd_origin = self.odom
            self.travel_since_cmd = 0.0
        if self.last_cmd is not None and now - self.last_cmd < CMD_GAP_S:
            self.cmd_active_s += now - self.last_cmd
        self.last_cmd = now

    def snapshot(self, wall):
        return {'wall': wall, 'sim_time': self.sim_time, 'msgs': self.msgs,
                'odom': list(self.odom) if self.odom else None, 'travel': round(self.travel, 3),
                'cmd_active_s': round(self.cmd_active_s, 2),
                'travel_since_cmd': None if self.travel_since_cmd is None else round(self.travel_since_cmd, 3)}
