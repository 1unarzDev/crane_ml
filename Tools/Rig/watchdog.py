"""Per-run watchdog: the pure decision of when a run must be stopped, and why.

The worker calls check(now, snapshot, phase) every poll with
  snapshot  the probe's progress.json (dict) or None when it could not be read
  phase     'starting' (before the course is ready), 'running', or 'post' (recording over)
and stops the run when it returns a reason:
  hard_timeout  wall time since launch over hard_s
  no_start      no probe data (no sim time, no topic data) within startup_s of launch
  stalled       neither sim time nor the message count changed for stall_s
  no_motion     commanded motion for motion_grace_s (or expect_motion and ready that long)
                and the boat moved less than min_travel_m: e.g. a drive held by command_mux
  post_timeout  post-processing over post_s
A None limit disables that check.
"""

DEFAULTS = {'hard_s': None, 'startup_s': 240, 'stall_s': 30, 'post_s': 300,
            'motion_grace_s': 10, 'min_travel_m': 0.5, 'expect_motion': False}


class Watchdog:
    def __init__(self, started, **limits):
        unknown = set(limits) - set(DEFAULTS)
        if unknown:
            raise ValueError(f'unknown watchdog limits {sorted(unknown)}')
        self.limits = {**DEFAULTS, **limits}
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
