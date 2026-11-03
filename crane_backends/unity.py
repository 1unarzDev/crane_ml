"""The ``unity`` backend: one crane player run per seed, scored from the player's own outputs.

No ROS, no SITL and no navigation stack: the player builds the seed's course itself
(``--crane-course-family``), runs its benchmark for ``duration`` simulated seconds and quits.
The run directory gets ``result.json`` (the benchmark, ``crane-benchmark-v1``),
``course_truth.json`` (the generated course and its identity) and ``player.log``. Metrics are
the numeric leaves of ``result.json`` (dotted names; lists of named entries by name) plus
``course_ready`` and ``player_s`` (wall seconds). Full-stack jobs (Nav2, MAVROS, the ROS TCP
endpoint) need a Linux host with the ROS workspace and are not run here; tag those ``os: linux``.

Worker YAML (``path`` puts this crane copy on sys.path)::

    unity:
      type: "crane_backends.unity:ClusterBackend"
      path: C:/Users/me/AppData/Local/crane-cluster/crane
      player: C:/Users/me/AppData/Local/crane-cluster/players/CRANE-Worker-Windows/CRANE.exe
      batchmode: true        # -batchmode: GPU rendering, no window. false only where no one can see a
                             # window (Windows session 0, a Linux host without DISPLAY); refused otherwise
      timeout_s: 900         # per seed
      args: []               # extra player arguments for this host

Water validity: a run whose benchmark sampled the water and got any invalid search
(``invalidWaterSearches`` > 0: HDRP's water never updated, so the boat's buoyancy and every
water-derived number are wrong) fails its seed with the counts, unless ``require_valid_water:
false``; then it is scored with ``water_valid`` 0 so analyses can censor it. ``water_valid`` is
in every run's metrics.

Job spec keys (override the options): ``family`` (default roboboat_2026), ``mode`` (iid,
corner=lo|hi|mixed), ``set`` (list of task.param=value), ``scene``, ``scenario``, ``sim_profile``
(default full), ``duration``, ``warmup``, ``time_scale``, ``args`` (list, appended), ``traj``
(default true: ``--crane-traj`` records ``run.ctraj``, crane-traj/1, at ``traj_hz``), ``params``,
``theater`` (study, iteration, insight, score_metric: see ``theater/meta.py``).

``run.ctraj`` is stamped with the job's study, iteration, params, score, the outcome (``finished``,
or ``invalid`` when result.json says ``valid: false``) and the course truth's hash, uploaded as a
``summary`` artifact (kept forever, a few hundred KB at most) and indexed through the run's meta.
"""

import json
import os
import signal
import subprocess
import sys
import time
from pathlib import Path

from .gpu import hidden_desktop, player_gpu_args, start_player

DEFAULTS = {'family': 'roboboat_2026', 'scene': 'Roboboat Course', 'scenario': 'cluster',
            'sim_profile': 'full', 'duration': 30, 'warmup': 5, 'time_scale': 1, 'batchmode': True,
            'timeout_s': 900, 'width': 640, 'height': 360, 'traj': True, 'traj_hz': 20,
            'require_valid_water': True}
TRAJ = 'run.ctraj'
LOD = 'run.lod.ctraj'   # the 2 Hz overview (theater.traj.lod), public-safe header


def water_check(result):
    """(valid, why) for a crane-benchmark-v1 result: every sampled water search must be valid."""
    invalid = int(result.get('invalidWaterSearches') or 0)
    samples = int(result.get('validationSamplesCaptured') or 0)
    if invalid:
        return False, (f'{invalid} invalid water searches in {samples} validation samples (HDRP water '
                       f'never updated: simulation time {water_time(result)}); rebuild the player with '
                       f'the batch water driver or run without -batchmode')
    return True, ''


def water_time(result):
    for sample in reversed(result.get('validation') or []):
        for w in sample.get('water') or []:
            return w.get('simulationTime')
    return None


def windows_session_id():
    """This process's Windows session (0: the services session, no desktop anyone sees)."""
    import ctypes
    sid = ctypes.c_ulong()
    ctypes.windll.kernel32.ProcessIdToSessionId(os.getpid(), ctypes.byref(sid))  # type: ignore[attr-defined]
    return sid.value


def window_visible(env):
    """Would a player window be visible to someone on this host? Not in session 0, nor on the gpu lane's
    hidden desktop (``CRANE_DESKTOP``)."""
    if sys.platform == 'win32':
        return windows_session_id() != 0 and not hidden_desktop(env)
    return bool(env.get('DISPLAY') or env.get('WAYLAND_DISPLAY'))


def flatten(value, prefix=''):
    """Numeric leaves as {dotted.name: float}; a list of {'name': ...} objects is keyed by name."""
    out = {}
    if isinstance(value, bool):
        return {prefix: float(value)} if prefix else {}
    if isinstance(value, (int, float)):
        return {prefix: float(value)} if prefix else {}
    if isinstance(value, dict):
        for k, v in value.items():
            out.update(flatten(v, f'{prefix}.{k}' if prefix else str(k)))
    elif isinstance(value, list) and value and all(isinstance(x, dict) and isinstance(x.get('name'), str)
                                                   for x in value):
        for x in value:
            out.update(flatten({k: v for k, v in x.items() if k != 'name'}, f'{prefix}.{x["name"]}'))
    return out


class ClusterBackend:
    def __init__(self, name='unity', options=None):
        self.name, self.options, self.ctx, self.proc = name, dict(options or {}), None, None

    def update(self):
        pass

    def prepare(self, ctx):
        self.ctx = ctx
        self.job = {**DEFAULTS, **self.options, **ctx.spec}
        if not self.job.get('player'):
            raise RuntimeError('unity backend: no player configured')
        self.player = Path(os.path.expandvars(str(self.job['player']))).expanduser()
        if not self.player.is_file():
            raise RuntimeError(f'unity backend: player {self.player} not found')
        self.env = {**os.environ, **getattr(ctx, 'env', {})}
        if not self.job['batchmode'] and window_visible(self.env):
            raise RuntimeError('unity backend: batchmode is off and a player window would be visible here')

    def argv(self, seed, run_dir):
        j = self.job
        a = [str(self.player)]
        if j['batchmode']:
            a.append('-batchmode')
        a += ['-screen-fullscreen', '0', '-screen-width', str(j['width']), '-screen-height', str(j['height']),
              '-logFile', str(run_dir / 'player.log'),
              '--crane-worker', '--crane-benchmark', '--crane-disable-ros', '--crane-disable-sitl',
              '--crane-worker-id', str(getattr(self.ctx, 'isolation', {}).get('slot', 0)),
              '--crane-seed', str(seed), '--crane-course-seed', str(seed),
              '--crane-course-family', str(j['family']), '--crane-course-truth', str(run_dir / 'course_truth.json'),
              '--crane-scene', str(j['scene']), '--crane-scenario', str(j['scenario']),
              '--crane-sim-profile', str(j['sim_profile']), '--crane-time-scale', str(j['time_scale']),
              '--crane-warmup', str(j['warmup']), '--crane-duration', str(j['duration']),
              '--crane-output', str(run_dir / 'result.json')]
        if j.get('mode'):
            a += ['--crane-course-mode', str(j['mode'])]
        if j.get('traj'):
            a += ['--crane-traj', str(run_dir / TRAJ), '--crane-traj-hz', str(j['traj_hz'])]
        for pin in j.get('set') or []:
            a += ['--crane-course-set', str(pin)]
        extra = [str(x) for x in self.options.get('args') or []] + [str(x) for x in self.ctx.spec.get('args') or []]
        return a + extra + player_gpu_args(extra)   # Windows with an iGPU and a dGPU: the dGPU (gpu.py)

    def run(self, seed):
        run_dir = Path(self.ctx.work_dir) / 'runs' / f's{int(seed)}'
        run_dir.mkdir(parents=True, exist_ok=True)
        t0 = time.time()
        self.proc = start_player(self.argv(seed, run_dir), str(self.player.parent), self.env)
        try:
            rc = self.proc.wait(timeout=float(self.job['timeout_s']))
        except subprocess.TimeoutExpired:
            self.cancel()
            raise RuntimeError(f'seed {seed}: player timed out after {self.job["timeout_s"]} s')
        finally:
            self.proc = None
        (run_dir / 'player_s.txt').write_text(f'{time.time() - t0:.3f}\n')
        if rc != 0 or not (run_dir / 'result.json').is_file():
            tail = ''
            if (run_dir / 'player.log').is_file():
                tail = (run_dir / 'player.log').read_bytes()[-1500:].decode(errors='replace')
            raise RuntimeError(f'seed {seed}: player exited {rc} without result.json: {tail}')
        ok, why = water_check(json.loads((run_dir / 'result.json').read_text()))
        if not ok and self.job.get('require_valid_water', True):
            raise RuntimeError(f'seed {seed}: invalid run: {why}')
        return run_dir

    def metrics(self, run_dir):
        run_dir = Path(run_dir)
        result = json.loads((run_dir / 'result.json').read_text())
        m = flatten(result)
        m['water_valid'] = float(water_check(result)[0])
        m['course_ready'] = float((run_dir / 'course_truth.json').is_file())
        m['player_s'] = float((run_dir / 'player_s.txt').read_text())
        return m

    def run_meta(self, run_dir, metrics):
        """Stamp run.ctraj (study, iteration, params, score, outcome, course hash); its run_meta, or None."""
        from theater import meta
        path = Path(run_dir) / TRAJ
        if not path.is_file():
            return None
        result = meta.load_json(Path(run_dir) / 'result.json') or {}
        truth = meta.load_json(Path(run_dir) / 'course_truth.json') or {}
        gen = truth.get('generator') or {}
        valid = result.get('valid')
        fields = meta.job_fields(self.ctx.job, self.ctx.spec, metrics=metrics)
        course = {'family': str(self.job['family']), 'name': truth.get('name'), 'truth_sha256': gen.get('truth_sha256'),
                  'pair_key': (gen.get('identity') or {}).get('pair_key'), 'set': list(self.job.get('set') or [])}
        fields.pop('job_id', None)
        header, sha = meta.stamp(path, **fields, valid=valid if isinstance(valid, bool) else None,
                                 outcome='invalid' if valid is False else 'finished',
                                 seed=int(Path(run_dir).name.lstrip('s') or 0),
                                 course={k: v for k, v in course.items() if v is not None})
        meta.write_lod(path)
        return meta.run_meta(header, traj_name=TRAJ, traj_sha256=sha)

    def artifacts(self, run_dir):
        run_dir = Path(run_dir)
        return [(run_dir / n, n, r) for n, r in (('result.json', 'summary'), ('course_truth.json', 'summary'),
                                                (TRAJ, 'summary'), (LOD, 'summary'),
                                                ('player.log', 'raw')) if (run_dir / n).is_file()]

    def cancel(self):
        p = self.proc
        if not p or p.poll() is not None:
            return
        try:
            if sys.platform == 'win32':  # the player and its crash handler
                subprocess.run(['taskkill', '/T', '/F', '/PID', str(p.pid)], capture_output=True, check=False,
                               creationflags=subprocess.CREATE_NO_WINDOW)
            else:
                os.killpg(p.pid, signal.SIGTERM)
        except (OSError, ProcessLookupError):
            pass

    def finish(self):
        self.cancel()
