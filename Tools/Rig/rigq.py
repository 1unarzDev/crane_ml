#!/usr/bin/env python3
"""rigq: a file-based job queue for headless sim runs, with an in-run watchdog.

  rigq submit [options] [-- run_sim.sh args]   queue a run (starts a worker if none runs)
  rigq list [--state S]                        jobs, newest last
  rigq show ID                                 one job
  rigq cancel ID                               dequeue, or stop it if running
  rigq results [-n N]                          finished jobs (results.jsonl)
  rigq worker [--slots N] [--until-empty]      run jobs: one worker per queue dir
  rigq worker install|uninstall [--slots N]    write/remove ~/.config/systemd/user/rigq-worker.service
                                               (a persistent worker; never enabled or started here)
Every command takes --json (one JSON document on stdout) and --config PATH.

Jobs are kind sim (run_sim.sh with the queue's domain and session dir), cmd (any argv; gets
ROS_DOMAIN_ID, RIGQ_DOMAIN and RIGQ_PROGRESS) or build (any argv, capped at queue.builders on
the host). The worker keeps queue.slots sim/cmd jobs and the host's rig slots, gives each job a
domain from queue.domains (never 201-230, never above 232) with no other running job on it,
clears that domain (stray processes, command_mux holds) before reuse, and stops a job whose
watchdog trips (watchdog.py): its process groups by exact PID, then whatever in the container
still carries its SIMRIG_DIR. Results land in results.jsonl, one line per finished job.

Queue dir layout: jobs/<id>.json (spec and state), logs/<id>.log, results.jsonl, worker.log.
Job states: queued, running, done, failed, tripped (watchdog), cancelled.
"""

import argparse
import fcntl
import json
import os
import re
import signal
import subprocess
import sys
import time
import uuid
from pathlib import Path

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))
import rigconf  # noqa: E402
from watchdog import DEFAULTS as WATCHDOG_DEFAULTS, Watchdog  # noqa: E402

FINAL = ('done', 'failed', 'tripped', 'cancelled')
RESERVED = range(201, 231)   # 210-230 are the tests', 201-209 other rig callers'


# --- configuration --------------------------------------------------------------------------

def parse_domains(spec):
    """'180-199,231' or a list -> sorted domains; refuses > 232 and 201-230."""
    items = spec if isinstance(spec, list) else str(spec).split(',')
    out = set()
    for item in items:
        item = str(item).strip()
        if not item:
            continue
        lo, _, hi = item.partition('-')
        out.update(range(int(lo), int(hi or lo) + 1))
    bad = sorted(d for d in out if d > 232 or d < 1 or d in RESERVED)
    if bad:
        raise ValueError(f'queue.domains must be 1-232 outside 201-230; got {bad}')
    if not out:
        raise ValueError('queue.domains is empty')
    return sorted(out)


def plant_applied(path):
    """{profile, sha256, status} of the player's CRANE_PLANT_APPLIED line, or None (as run_sim.sh's
    run log): a result is only attributable with the plant that produced it."""
    try:
        for text in path.read_text(errors='replace').splitlines():
            if text.startswith('CRANE_PLANT_APPLIED '):
                return dict(re.findall(r'(\w+)=(.*?)(?= \w+=|$)', text[len('CRANE_PLANT_APPLIED '):]))
    except OSError:
        pass
    return None


def load_config(path):
    rig = rigconf.load(path)
    q = dict(rig.get('queue') or {})
    q.setdefault('dir', str(Path.home() / '.local/state/crane-rig/queue'))
    q['domains'] = parse_domains(q.get('domains', '180-199'))
    q.setdefault('slots', 3)
    q.setdefault('builders', 2)
    q.setdefault('poll_s', 1)
    q.setdefault('grace_s', 20)
    rig['queue'] = q
    return rig


# --- queue storage --------------------------------------------------------------------------

class Queue:
    def __init__(self, directory):
        self.dir = Path(directory)
        (self.dir / 'jobs').mkdir(parents=True, exist_ok=True)
        (self.dir / 'logs').mkdir(exist_ok=True)

    def _lock(self):
        fd = os.open(self.dir / 'lock', os.O_CREAT | os.O_RDWR, 0o644)
        fcntl.flock(fd, fcntl.LOCK_EX)
        return fd

    def _unlock(self, fd):
        fcntl.flock(fd, fcntl.LOCK_UN)
        os.close(fd)

    def path(self, job_id):
        return self.dir / 'jobs' / f'{job_id}.json'

    def _write(self, job):
        tmp = self.path(job['id']).with_suffix('.tmp')
        tmp.write_text(json.dumps(job, indent=1, default=str))
        os.replace(tmp, self.path(job['id']))

    def get(self, job_id):
        try:
            return json.loads(self.path(job_id).read_text())
        except FileNotFoundError:
            matches = sorted(self.dir.glob(f'jobs/*{job_id}*.json'))
            if len(matches) == 1:
                return json.loads(matches[0].read_text())
            raise KeyError(job_id) from None

    def jobs(self, state=None):
        out = []
        for path in sorted((self.dir / 'jobs').glob('*.json')):
            try:
                job = json.loads(path.read_text())
            except (OSError, ValueError):
                continue
            if state is None or job['state'] == state:
                out.append(job)
        return sorted(out, key=lambda j: (j['submitted'], j['id']))

    def add(self, spec):
        job_id = time.strftime('%Y%m%d-%H%M%S-') + uuid.uuid4().hex[:6]
        job = dict(spec, id=job_id, state='queued', submitted=time.time(), attempts=0)
        job.setdefault('name', job_id)
        fd = self._lock()
        try:
            self._write(job)
        finally:
            self._unlock(fd)
        return job

    def update(self, job_id, _decide=None, **fields):
        """Read-modify-write one job under the queue lock; returns the new job.
        ``_decide(job)`` may return more fields, chosen from the job as read under the lock."""
        fd = self._lock()
        try:
            job = json.loads(self.path(job_id).read_text())
            job.update(fields)
            if _decide:
                job.update(_decide(job))
            self._write(job)
            return job
        finally:
            self._unlock(fd)

    def finish(self, job_id, state, **fields):
        return self._log_result(self.update(job_id, state=state, finished=time.time(), **fields))

    def _log_result(self, job):
        line = {k: job.get(k) for k in ('id', 'name', 'kind', 'state', 'reason', 'exit', 'domain',
                                        'session', 'submitted', 'started', 'finished', 'progress',
                                        'cleanup', 'log')}
        if job.get('started'):
            line['elapsed_s'] = round(job['finished'] - job['started'], 1)
        if job.get('kind') == 'sim' and job.get('session'):
            line['plant'] = plant_applied(Path(job['session']) / 'player.log')
        with open(self.dir / 'results.jsonl', 'a') as out:
            out.write(json.dumps(line, default=str) + '\n')
        return job

    def cancel(self, job_id):
        cancelled = []

        def decide(job):    # runs under the queue lock, so a worker's start cannot slip in between
            if job['state'] == 'queued':
                cancelled.append(1)
                return {'state': 'cancelled', 'reason': 'cancelled', 'finished': time.time()}
            return {'cancel_requested': True} if job['state'] == 'running' else {}
        job = self.update(self.get(job_id)['id'], decide)
        return self._log_result(job) if cancelled else job

    def results(self, n=None):
        try:
            lines = (self.dir / 'results.jsonl').read_text().splitlines()
        except FileNotFoundError:
            return []
        rows = [json.loads(line) for line in lines if line.strip()]
        return rows[-n:] if n else rows


# --- host resources -------------------------------------------------------------------------

def lock_free(path):
    """True when nobody holds flock on ``path`` (tests it without keeping it)."""
    try:
        fd = os.open(path, os.O_CREAT | os.O_RDWR, 0o644)
    except OSError:
        return False
    try:
        fcntl.flock(fd, fcntl.LOCK_EX | fcntl.LOCK_NB)
        fcntl.flock(fd, fcntl.LOCK_UN)
        return True
    except OSError:
        return False
    finally:
        os.close(fd)


def take_lock(path):
    """Hold flock on ``path``: the fd, or None when someone else has it."""
    fd = os.open(path, os.O_CREAT | os.O_RDWR, 0o644)
    try:
        fcntl.flock(fd, fcntl.LOCK_EX | fcntl.LOCK_NB)
        return fd
    except OSError:
        os.close(fd)
        return None


def pgid_of(pid):
    try:
        return os.getpgid(pid)
    except (ProcessLookupError, TypeError, ValueError):
        return None


def start_ticks(pid):
    """The process start time (clock ticks since boot, /proc/PID/stat field 22), or None."""
    try:
        return int(Path(f'/proc/{pid}/stat').read_text().rsplit(')', 1)[1].split()[19])
    except (OSError, IndexError, ValueError):
        return None


def cmdline(pid):
    try:
        return Path(f'/proc/{pid}/cmdline').read_bytes().replace(b'\0', b' ').decode(errors='replace')
    except OSError:
        return ''


def signal_group(pgid, sig):
    try:
        os.killpg(pgid, sig)
        return True
    except (ProcessLookupError, PermissionError, TypeError):
        return False


def group_alive(pgid):
    return signal_group(pgid, 0)


def docker_sh(container, script, timeout=30):
    try:
        out = subprocess.run(['docker', 'exec', '-e', 'BASH_ENV=', container, 'sh', '-c', script],
                             capture_output=True, text=True, timeout=timeout)
        return out.stdout.strip()
    except (subprocess.TimeoutExpired, OSError):
        return ''


# Stop every container process whose environ has NAME=VALUE: TERM, 1 s, KILL; print survivors.
SWEEP = r'''n=0; for sig in TERM KILL; do
  for e in /proc/[0-9]*/environ; do
    grep -qaz "^{name}={value}\$" "$e" 2>/dev/null || continue
    p=${{e#/proc/}}; p=${{p%/environ}}; [ "$p" = $$ ] && continue; kill -$sig "$p" 2>/dev/null
  done; [ $sig = TERM ] && sleep 1; done
for e in /proc/[0-9]*/environ; do grep -qaz "^{name}={value}\$" "$e" 2>/dev/null && n=$((n+1)); done
echo $n'''


def host_sweep(job_id, grace_s=1.0):
    """Stop every host process (of this user) whose environment carries RIGQ_JOB=<job_id>: what a
    job setsid'd out of its group, the player included. TERM, grace, KILL; returns the PIDs hit."""
    needle = f'RIGQ_JOB={job_id}'.encode()

    def tagged():
        found = []
        for env in Path('/proc').glob('[0-9]*/environ'):
            pid = int(env.parent.name)
            if pid == os.getpid():
                continue
            try:
                if needle in env.read_bytes().split(b'\0'):
                    found.append(pid)
            except OSError:
                continue
        return found

    hit = tagged()
    for sig in (signal.SIGTERM, signal.SIGKILL):
        for pid in tagged():
            try:
                os.kill(pid, sig)
                if sig == signal.SIGTERM:
                    os.kill(pid, signal.SIGCONT)
            except (ProcessLookupError, PermissionError):
                pass
        if sig == signal.SIGTERM and hit:
            time.sleep(grace_s)
    return hit


def container_is_ours(container):
    try:
        out = subprocess.run(['docker', 'inspect', '-f', '{{index .Config.Labels "crane.rig"}} {{.State.Running}}',
                              container], capture_output=True, text=True, timeout=15)
        return out.stdout.split() == ['1', 'true']
    except (subprocess.TimeoutExpired, OSError):
        return False


def clear_domain(container, domain):
    """Before a domain is reused: stray processes on it and command_mux's saved holds
    (hold_store: $XDG_RUNTIME_DIR or the temp dir, mhseals/command_mux_<domain>.json)."""
    script = SWEEP.format(name='ROS_DOMAIN_ID', value=domain) + (
        f'; rm -f /tmp/mhseals/command_mux_{domain}.json "${{XDG_RUNTIME_DIR:-/tmp}}/mhseals/command_mux_{domain}.json"')
    left = docker_sh(container, script, timeout=60)
    return {'domain': domain, 'stray_left': int(left) if left.isdigit() else None}


def sweep_job(job_id, session):
    """Host processes tagged with the job, then (sim) the container processes of its SIMRIG_DIR."""
    out = {}
    hit = host_sweep(job_id)
    if hit:
        out['host_stopped'] = hit
    env = read_env(Path(session) / 'run.env') if session else {}
    if env.get('cdir') and env.get('container') and container_is_ours(env['container']):
        left = docker_sh(env['container'], SWEEP.format(name='SIMRIG_DIR', value=env['cdir']), timeout=60)
        out['container_left'] = int(left) if left.isdigit() else None
    return out


# --- a running job --------------------------------------------------------------------------

def read_env(path):
    try:
        return dict(line.split('=', 1) for line in Path(path).read_text().splitlines() if '=' in line)
    except OSError:
        return {}


def duration_of(args, rig):
    for flag in ('-d', '--duration'):
        if flag in args[:-1]:
            return float(args[args.index(flag) + 1])
    return float(rig.get('duration', 30))


class Active:
    """One launched job: its process, watchdog, and the stop state machine."""

    def __init__(self, queue, rig, job, domain):
        self.queue, self.rig, self.job, self.domain = queue, rig, job, domain
        self.kind = job['kind']
        self.log = queue.dir / 'logs' / f"{job['id']}.log"
        self.progress_file = queue.dir / 'logs' / f"{job['id']}.progress.json"
        self.stopping = None        # (reason, state, since)
        self.killed = False
        self.builder_fd = None
        self.hang_done = False
        self.snapshot = None
        self.last_saved = 0
        self.session = None

    def start(self, builder_fd=None):
        self.builder_fd = builder_fd
        env = dict(os.environ)
        job = self.job
        if self.kind == 'sim':
            self.session = job.get('session') or str(Path(self.rig['sessions_dir']) / 'rigq' / job['id'])
            argv = [str(HERE / 'run_sim.sh'), '--config', job.get('config') or str(HERE / 'rig.yaml'),
                    '--domain', str(self.domain), '-o', self.session] + job.get('args', [])
        else:
            argv = list(job['argv'])
        if self.domain is not None:
            env.update(ROS_DOMAIN_ID=str(self.domain), RIGQ_DOMAIN=str(self.domain))
        env['RIGQ_PROGRESS'] = str(self.progress_file)
        env['RIGQ_JOB'] = job['id']
        with open(self.log, 'wb') as out:
            self.proc = subprocess.Popen(argv, stdout=out, stderr=subprocess.STDOUT, stdin=subprocess.DEVNULL,
                                         env=env, start_new_session=True, cwd=str(HERE))
        self.started = time.monotonic()
        limits = self._limits()
        self.watchdog = Watchdog(self.started, **limits)
        self.job = self.queue.update(job['id'], state='running', started=time.time(), pid=self.proc.pid,
                                     pid_start=start_ticks(self.proc.pid),
                                     domain=self.domain, session=self.session, log=str(self.log),
                                     limits=limits, attempts=job.get('attempts', 0) + 1)

    def _limits(self):
        job = self.job
        given = dict(job.get('watchdog') or {})
        if self.kind == 'sim':
            wd = self.rig.get('watchdog') or {}
            limits = {k: wd[k] for k in WATCHDOG_DEFAULTS if k in wd}
            limits['hard_s'] = duration_of(job.get('args', []), self.rig) + float(wd.get('hard_margin_s', 420))
        else:   # cmd and build: a hard timeout only, unless the job asks for more
            limits = {k: None for k in WATCHDOG_DEFAULTS if k not in ('min_travel_m', 'expect_motion')}
            limits['hard_s'] = 3600
        limits.update({k: v for k, v in given.items() if k in WATCHDOG_DEFAULTS})
        if job.get('timeout'):
            limits['hard_s'] = float(job['timeout'])
        return limits

    # progress and phase
    def run_env(self):
        return read_env(Path(self.session) / 'run.env') if self.session else {}

    def read_progress(self):
        if self.kind == 'sim':
            env = self.run_env()
            if not env.get('cdir'):
                return None
            text = docker_sh(env['container'], f"cat {env['cdir']}/progress.json 2>/dev/null", timeout=10)
        else:
            try:
                text = self.progress_file.read_text()
            except OSError:
                return None
        try:
            return json.loads(text) if text else None
        except ValueError:
            return None

    def phase(self, snapshot):
        if self.kind != 'sim':
            return (snapshot or {}).get('phase', 'running')
        try:
            text = self.log.read_text(errors='replace')
        except OSError:
            return 'starting'
        if re.search(r'\] recorded \d+s', text):
            return 'post'
        return 'running' if '] course ready' in text else 'starting'

    # the injected hang (watchdog validation): freeze the player N s after ready
    def inject(self, phase):
        after = (self.job.get('inject') or {}).get('hang_after_s')
        if after is None or self.hang_done or phase != 'running':
            return
        if self.watchdog.ready_at is not None and time.monotonic() - self.watchdog.ready_at >= after:
            pid = self._player_pid()
            if pid:
                signal_group(pid, signal.SIGSTOP)
                self.hang_done = True
                self.queue.update(self.job['id'], injected=f'player {pid} stopped at ready+{after}s')

    def _player_pid(self):
        """The player's PID from run.env, only if that PID is still the player and leads its group."""
        env = self.run_env()
        try:
            pid = int(env.get('player_pid') or 0)
        except ValueError:
            return None
        if pid and env.get('player') and env['player'] in cmdline(pid) and pgid_of(pid) == pid:
            return pid
        return None

    # stopping: TERM the groups, then KILL after grace, then sweep the container
    def request_stop(self, reason, state):
        if self.stopping:
            return
        self.stopping = (reason, state, time.monotonic())
        player = self._player_pid() if self.kind == 'sim' else None
        signal_group(self.proc.pid, signal.SIGTERM)
        if player:
            signal_group(player, signal.SIGTERM)
            signal_group(player, signal.SIGCONT)   # a stopped player only acts on TERM once continued
        self.queue.update(self.job['id'], stopping=reason)

    def poll(self, now):
        """None while running; the final (state, fields) once the job is over."""
        code = self.proc.poll()
        if self.stopping:
            reason, state, since = self.stopping
            if code is None and now - since > float(self.rig['queue']['grace_s']):
                self._kill_hard()
            if code is not None:
                return state, dict(reason=reason, exit=code, cleanup=self.cleanup())
            return None
        if code is not None:
            cleanup = self.cleanup()
            if code == 0:
                return 'done', dict(exit=0, reason=None, cleanup=cleanup)
            return 'failed', dict(exit=code, reason=self._fail_reason(code), cleanup=cleanup)
        snap = self.read_progress()
        if snap is not None:
            self.snapshot = snap
        phase = self.phase(snap)
        self.inject(phase)
        reason = self.watchdog.check(now, snap, phase)
        if now - self.last_saved > 5:
            self.last_saved = now
            self.queue.update(self.job['id'], progress=self.snapshot, phase=phase)
        if reason:
            self.request_stop(reason, 'tripped')
        return None

    def _fail_reason(self, code):
        try:
            tail = self.log.read_text(errors='replace').splitlines()[-30:]
        except OSError:
            tail = []
        if any('in use by another rig run' in line for line in tail):
            return 'domain_busy'
        hits = [line for line in tail if line.startswith('[simrig')]
        return (hits[-2] if len(hits) > 1 else (tail[-1] if tail else f'exit {code}'))[:200]

    def _kill_hard(self):
        if self.killed:
            return
        self.killed = True
        signal_group(self.proc.pid, signal.SIGKILL)
        player = self._player_pid() if self.kind == 'sim' else None
        if player:
            signal_group(player, signal.SIGKILL)

    def cleanup(self):
        """After the runner exits: nothing of the run may survive. Returns what was found."""
        out = {}
        if group_alive(self.proc.pid):          # children left in the runner's group
            signal_group(self.proc.pid, signal.SIGKILL)
            out['runner_group_killed'] = True
        out.update(sweep_job(self.job['id'], self.session))
        if self.builder_fd is not None:
            os.close(self.builder_fd)
            self.builder_fd = None
        return out


# --- the worker -----------------------------------------------------------------------------

class Worker:
    def __init__(self, rig, slots=None, until_empty=False, linger_s=20, out=sys.stderr):
        self.rig = rig
        self.q = Queue(rig['queue']['dir'])
        self.slots = int(slots or rig['queue']['slots'])
        self.until_empty, self.linger_s, self.out = until_empty, linger_s, out
        self.lock_dir = Path(rig.get('lock_dir', '/tmp'))
        self.active = {}
        self.last_used = {}     # domain -> monotonic time released

    def say(self, text):
        print(time.strftime('%H:%M:%S ') + text, file=self.out, flush=True)

    def host_slot_free(self):
        n = int(self.rig.get('max_instances', 3))
        return any(lock_free(self.lock_dir / f'simrig-slot-{i}.lock') for i in range(n))

    def pick_domain(self):
        used = {a.domain for a in self.active.values()}
        free = [d for d in self.rig['queue']['domains'] if d not in used
                and lock_free(self.lock_dir / f'simrig-domain-{d}.lock')
                and lock_free(self.lock_dir / f'simrig-port-{10000 + d}.lock')]
        return min(free, key=lambda d: self.last_used.get(d, 0.0)) if free else None

    def take_builder(self):
        for i in range(int(self.rig['queue']['builders'])):
            fd = take_lock(self.lock_dir / f'crane-builder-{i}.lock')
            if fd is not None:
                return fd
        return None

    def adopt_orphans(self):
        """Jobs left 'running' by a dead worker: stop their groups, mark them failed."""
        for job in self.q.jobs('running'):
            pid = job.get('pid')
            # the job's own process: a group leader that started when the job did (a reused PID
            # differs in start time) and still runs the job's command
            want = 'run_sim.sh' if job['kind'] == 'sim' else (job.get('argv') or [None])[0]
            if pid and pgid_of(pid) == pid and job.get('pid_start') is not None \
                    and start_ticks(pid) == job['pid_start'] and want and want in cmdline(pid):
                signal_group(pid, signal.SIGTERM)
                time.sleep(2)
                signal_group(pid, signal.SIGKILL)
            cleanup = sweep_job(job['id'], job.get('session'))
            self.q.finish(job['id'], 'failed', reason='worker_lost', cleanup=cleanup)
            self.say(f"{job['id']} {job['name']}: worker_lost")

    def launch(self, job):
        if self.q.get(job['id'])['state'] != 'queued':
            return False
        kind = job['kind']
        builder = None
        if kind == 'build':
            builder = self.take_builder()
            if builder is None:
                return False
            domain = None
        else:
            if sum(a.kind != 'build' for a in self.active.values()) >= self.slots:
                return False
            if kind == 'sim':
                # one sim at a time through start-up: run_sim takes its host slot before run.env
                if any(a.kind == 'sim' and not (Path(a.session) / 'run.env').exists() for a in self.active.values()):
                    return False
                if not self.host_slot_free():
                    return False
                if not container_is_ours(self.rig['container']):
                    self.q.finish(job['id'], 'failed', reason=f"container {self.rig['container']} is not "
                                  'running or not the rig\'s (Tools/Rig/container.sh up)')
                    return True
            domain = self.pick_domain()
            if domain is None:
                return False
            if kind == 'sim':
                cleared = clear_domain(self.rig['container'], domain)
                self.q.update(job['id'], domain_cleared=cleared)
                if cleared.get('stray_left') != 0:      # strays left, or docker did not answer (None)
                    self.q.finish(job['id'], 'failed', reason=f'domain {domain} not clean: '
                                  f"stray_left={cleared.get('stray_left')}")
                    return True
        active = Active(self.q, self.rig, job, domain)
        active.start(builder)
        self.active[job['id']] = active
        self.say(f"start {job['id']} {job['name']} ({kind}, domain {domain})")
        return True

    def step(self):
        now = time.monotonic()
        for job_id, active in list(self.active.items()):
            try:
                cancel = self.q.get(job_id).get('cancel_requested')
            except KeyError:
                cancel = True
            if cancel:
                active.request_stop('cancelled', 'cancelled')
            over = active.poll(now)
            if over is None:
                continue
            state, fields = over
            del self.active[job_id]
            if active.domain is not None:
                self.last_used[active.domain] = time.monotonic()
            if fields.get('reason') == 'domain_busy' and active.job.get('attempts', 1) < 3:
                self.q.update(job_id, state='queued', reason='domain_busy (requeued)')
                self.say(f'requeue {job_id}: domain_busy')
                continue
            self.q.finish(job_id, state, progress=active.snapshot, **fields)
            self.say(f"{state} {job_id} {active.job['name']}: {fields.get('reason') or 'exit 0'}")
        for job in self.q.jobs('queued'):
            if not self.launch(job):
                # FIFO per resource: a sim waiting for a slot does not block a build, nor the reverse
                continue

    def release_lock(self, fd):
        os.close(fd)

    def serve(self, stop):
        idle_since = None
        while True:
            if stop:
                for active in self.active.values():
                    active.request_stop('worker_stopped', 'cancelled')
                if not self.active:
                    return
            self.step()
            if self.until_empty and not self.active and not self.q.jobs('queued'):
                idle_since = idle_since or time.monotonic()
                if time.monotonic() - idle_since > self.linger_s:
                    return
            else:
                idle_since = None
            time.sleep(float(self.rig['queue']['poll_s']))

    def run(self):
        fd = take_lock(self.q.dir / 'worker.lock')
        if fd is None:
            self.say('another worker holds this queue')
            return 1
        stop = []
        signal.signal(signal.SIGTERM, lambda *_: stop.append(1))
        signal.signal(signal.SIGINT, lambda *_: stop.append(1))
        self.adopt_orphans()
        self.say(f"worker up: {self.q.dir}, slots {self.slots}, domains {self.rig['queue']['domains'][0]}-"
                 f"{self.rig['queue']['domains'][-1]}")
        while True:
            self.serve(stop)
            self.release_lock(fd)
            # A job submitted while we were exiting saw the lock held and started no worker, so
            # look again after releasing it; a submit after this look finds the lock free.
            if stop or not self.q.jobs('queued'):
                break
            fd = take_lock(self.q.dir / 'worker.lock')
            if fd is None:          # another worker took over
                break
        self.say('worker exits')
        return 0


def spawn_worker(config, qdir):
    """Start a detached `rigq worker --until-empty` unless one already holds the queue."""
    if not lock_free(Path(qdir) / 'worker.lock'):
        return False
    env = {k: v for k, v in os.environ.items() if k not in ('RIGQ_JOB', 'RIGQ_PROGRESS')}
    with open(Path(qdir) / 'worker.log', 'ab') as out:
        subprocess.Popen([sys.executable, '-I', str(HERE / 'rigq.py'), 'worker', '--until-empty',
                          '--config', str(config)], stdout=out, stderr=subprocess.STDOUT,
                         stdin=subprocess.DEVNULL, start_new_session=True, env=env)
    return True


# --- CLI ------------------------------------------------------------------------------------

def emit(args, data, human):
    if args.json:
        print(json.dumps(data, indent=1, default=str))
    else:
        human(data)


UNIT_NAME = 'rigq-worker.service'


def unit_path():
    base = os.environ.get('XDG_CONFIG_HOME') or str(Path.home() / '.config')
    return Path(base) / 'systemd' / 'user' / UNIT_NAME


def unit_text(config, slots=None):
    """The user unit from rigq-worker.service.in; systemd needs absolute, space-free paths."""
    values = {'@RIG_DIR@': str(HERE), '@RIGQ@': str(HERE / 'rigq'), '@CONFIG@': str(Path(config).resolve()),
              '@SLOTS@': f' --slots {int(slots)}' if slots else ''}
    for key, value in values.items():
        if key != '@SLOTS@' and (' ' in value or '%' in value):
            raise ValueError(f'unit path has a space or %: {value}')
    text = (HERE / 'rigq-worker.service.in').read_text()
    for key, value in values.items():
        text = text.replace(key, value)
    return text


def worker_unit(action, config, slots=None):
    """Write or remove the unit file. Never runs systemctl: enabling is the owner's call."""
    path = unit_path()
    if action == 'install':
        path.parent.mkdir(parents=True, exist_ok=True)
        path.write_text(unit_text(config, slots))
        return {'unit': str(path), 'written': True,
                'next': f'systemctl --user daemon-reload && systemctl --user enable --now {UNIT_NAME}'}
    existed = path.exists()
    if existed:
        path.unlink()
    return {'unit': str(path), 'removed': existed,
            'next': f'systemctl --user disable --now {UNIT_NAME}; systemctl --user daemon-reload'
                    if existed else 'nothing to remove'}


def row(job):
    p = job.get('progress') or {}
    extra = f"sim {p.get('sim_time') or 0:.0f}s travel {p.get('travel') or 0:.1f}m" if p else ''
    return (f"{job['id']}  {job['state']:9} {job['name'][:24]:24} {job['kind']:5} "
            f"dom {job.get('domain') or '-':>3}  {job.get('reason') or job.get('stopping') or ''} {extra}").rstrip()


def main(argv=None):
    ap = argparse.ArgumentParser(prog='rigq', description=__doc__.split('\n')[0])
    ap.add_argument('--config', default=str(HERE / 'rig.yaml'))
    ap.add_argument('--json', action='store_true', help='one JSON document on stdout')
    sub = ap.add_subparsers(dest='cmd', required=True)
    s = sub.add_parser('submit', help='queue a job')
    s.add_argument('--name')
    s.add_argument('--kind', choices=('sim', 'cmd', 'build'), default='sim')
    s.add_argument('--timeout', type=float, help='hard timeout, s [sim: duration + hard_margin_s]')
    s.add_argument('--stall', type=float, dest='stall_s', help='no-progress timeout, s')
    s.add_argument('--startup', type=float, dest='startup_s')
    s.add_argument('--motion-grace', type=float, dest='motion_grace_s')
    s.add_argument('--min-travel', type=float, dest='min_travel_m')
    s.add_argument('--expect-motion', action='store_true', default=None)
    s.add_argument('--drive', help='rig.yaml drives: preset (sim)')
    s.add_argument('--session', help='session dir (sim) [sessions_dir/rigq/<id>]')
    s.add_argument('--inject-hang', type=float, metavar='S',
                   help='watchdog test: freeze the player S s after the course is ready')
    s.add_argument('--no-start', action='store_true', help='do not start a worker')
    s.add_argument('rest', nargs=argparse.REMAINDER, help='-- run_sim.sh args (sim) or argv (cmd, build)')
    ls = sub.add_parser('list')
    ls.add_argument('--state', choices=('queued', 'running') + FINAL)
    sh = sub.add_parser('show')
    sh.add_argument('id')
    c = sub.add_parser('cancel')
    c.add_argument('id')
    r = sub.add_parser('results')
    r.add_argument('-n', type=int)
    w = sub.add_parser('worker')
    w.add_argument('action', nargs='?', choices=('install', 'uninstall'),
                   help='write or remove the systemd user unit (never enables it)')
    w.add_argument('--slots', type=int)
    w.add_argument('--until-empty', action='store_true', help='exit once nothing is queued or running')
    w.add_argument('--linger', type=float, default=20)
    args = ap.parse_args(argv)

    if args.cmd == 'worker' and args.action:
        try:
            emit(args, worker_unit(args.action, args.config, args.slots),
                 lambda d: [print(f'{k}: {v}') for k, v in d.items()])
        except (OSError, ValueError) as error:
            print(f'rigq: {error}', file=sys.stderr)
            return 2
        return 0
    rig = load_config(args.config)
    q = Queue(rig['queue']['dir'])
    try:
        if args.cmd == 'submit':
            rest = args.rest[1:] if args.rest[:1] == ['--'] else args.rest
            spec = {'name': args.name, 'kind': args.kind, 'config': str(Path(args.config).resolve())}
            if args.kind == 'sim':
                for flag in ('--domain', '--port', '--config'):
                    if flag in rest:
                        raise ValueError(f'{flag} belongs to the queue, not the job')
                if args.drive:
                    rest = ['--drive', args.drive] + rest
                spec['args'] = rest
            else:
                if not rest:
                    raise ValueError('cmd and build jobs need an argv after --')
                spec['argv'] = rest
            wd = {k: getattr(args, k) for k in ('stall_s', 'startup_s', 'motion_grace_s', 'min_travel_m',
                                                'expect_motion') if getattr(args, k) is not None}
            if wd:
                spec['watchdog'] = wd
            if args.timeout:
                spec['timeout'] = args.timeout
            if args.session:
                spec['session'] = str(Path(args.session).resolve())
            if args.inject_hang is not None:
                spec['inject'] = {'hang_after_s': args.inject_hang}
            spec = {k: v for k, v in spec.items() if v is not None}
            job = q.add(spec)
            started = False if args.no_start else spawn_worker(Path(args.config).resolve(), q.dir)
            emit(args, dict(job, worker_started=started),
                 lambda j: print(f"{j['id']} queued{' (worker started)' if started else ''}"))
        elif args.cmd == 'list':
            emit(args, q.jobs(args.state), lambda jobs: [print(row(j)) for j in jobs])
        elif args.cmd == 'show':
            emit(args, q.get(args.id), lambda j: print(json.dumps(j, indent=1, default=str)))
        elif args.cmd == 'cancel':
            emit(args, q.cancel(args.id), lambda j: print(row(j)))
        elif args.cmd == 'results':
            emit(args, q.results(args.n), lambda rows: [print(
                f"{x['id']}  {x['state']:9} {(x.get('name') or '')[:24]:24} {x.get('elapsed_s', '-'):>7}s  "
                f"{x.get('reason') or ''}") for x in rows])
        elif args.cmd == 'worker':
            return Worker(rig, args.slots, args.until_empty, args.linger).run()
    except (KeyError, ValueError) as error:
        if args.json:
            print(json.dumps({'error': str(error)}))
        else:
            print(f'rigq: {error}', file=sys.stderr)
        return 2
    return 0


if __name__ == '__main__':
    sys.exit(main())
