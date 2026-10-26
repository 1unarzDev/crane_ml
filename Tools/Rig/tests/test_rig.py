"""Host-side tests for the rig: python3 -I -m pytest Tools/Rig/tests -q

Pure logic (watchdog, probe tracker, domain pool, config) plus the queue worker driving fake
runs (fake_run.py) that hang, stall, never move, fail and succeed. No ROS, Docker or player.
"""

import json
import os
import sys
import time
from pathlib import Path

import pytest
import yaml

RIG = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(RIG))

import check  # noqa: E402
import drive  # noqa: E402
import probe  # noqa: E402
import rigconf  # noqa: E402
import rigq  # noqa: E402
from watchdog import Watchdog  # noqa: E402

FAKE = [sys.executable, '-I', str(RIG / 'tests' / 'fake_run.py')]


# --- watchdog --------------------------------------------------------------------------------

def snap(sim, msgs, **kw):
    return dict(sim_time=sim, msgs=msgs, **kw)


def test_hard_timeout_wins_in_any_phase():
    w = Watchdog(0, hard_s=10)
    assert w.check(5, snap(1, 1), 'running') is None
    assert w.check(11, snap(2, 2), 'post') == 'hard_timeout'


def test_no_start_without_probe_data():
    w = Watchdog(0, startup_s=20)
    assert w.check(10, None, 'starting') is None
    assert w.check(15, snap(None, 0), 'starting') is None   # probe up, nothing flowing
    assert w.check(21, None, 'starting') == 'no_start'


def test_stall_when_sim_time_and_counts_freeze():
    w = Watchdog(0, stall_s=30)
    assert w.check(1, snap(1.0, 10), 'running') is None
    assert w.check(20, snap(5.0, 50), 'running') is None
    assert w.check(49, snap(5.0, 50), 'running') is None
    assert w.check(51, snap(5.0, 50), 'running') == 'stalled'


def test_unreadable_probe_counts_as_no_progress():
    w = Watchdog(0, stall_s=5)
    w.check(1, snap(1, 1), 'running')
    assert w.check(7, None, 'running') == 'stalled'


def test_no_motion_when_commanded_but_static():
    w = Watchdog(0, motion_grace_s=10, min_travel_m=0.5)
    assert w.check(1, snap(1, 1, cmd_active_s=9, travel_since_cmd=0.0), 'running') is None
    assert w.check(2, snap(2, 2, cmd_active_s=11, travel_since_cmd=0.1), 'running') == 'no_motion'


def test_moving_boat_and_idle_boat_pass():
    w = Watchdog(0, motion_grace_s=10, min_travel_m=0.5)
    assert w.check(1, snap(1, 1, cmd_active_s=30, travel_since_cmd=4.0), 'running') is None
    assert w.check(2, snap(2, 2, cmd_active_s=0, travel=0.0), 'running') is None   # nothing commanded


def test_expect_motion_without_a_seen_command():
    w = Watchdog(0, motion_grace_s=10, min_travel_m=0.5, expect_motion=True)
    assert w.check(1, snap(1, 1, travel=0), 'running') is None
    assert w.check(12, snap(12, 12, travel=0.2), 'running') == 'no_motion'


def test_post_phase_has_its_own_limit_and_no_stall():
    w = Watchdog(0, stall_s=5, post_s=60)
    w.check(1, snap(1, 1), 'running')
    assert w.check(30, None, 'post') is None
    assert w.check(91, None, 'post') == 'post_timeout'


def test_unknown_limit_is_refused():
    with pytest.raises(ValueError):
        Watchdog(0, stal_s=3)


# --- probe tracker ---------------------------------------------------------------------------

def test_tracker_measures_travel_from_first_command():
    t = probe.Tracker()
    t.on_odom(0, 0)
    t.on_clock(1.0)
    t.on_odom(1, 0)          # drift before any command
    for i in range(11):
        t.on_command(i * 0.1, 0.6, 0, 0)
    t.on_odom(4, 0)
    s = t.snapshot(0)
    assert s['msgs'] == 4 and s['sim_time'] == 1.0
    assert s['cmd_active_s'] == pytest.approx(1.0)
    assert s['travel_since_cmd'] == 3 and s['travel'] == 4


def test_tracker_ignores_zero_commands_and_gaps():
    t = probe.Tracker()
    t.on_command(0, 0.01, 0, 0)
    t.on_command(0.1, 0.0, 0, 0)
    t.on_command(1.0, 0.5, 0, 0)
    t.on_command(3.0, 0.5, 0, 0)    # a 2 s gap does not count
    assert t.snapshot(0)['cmd_active_s'] == 0


# --- domains and config ----------------------------------------------------------------------

def test_domain_pool_refuses_test_range_and_above_232():
    assert rigq.parse_domains('180-182,231') == [180, 181, 182, 231]
    for bad in ('200-201', '210', '230', '233', '0'):
        with pytest.raises(ValueError):
            rigq.parse_domains(bad)


def test_shipped_config_pool_is_valid():
    cfg = yaml.safe_load((RIG / 'rig.yaml').read_text())
    rigq.parse_domains(cfg['queue']['domains'])
    assert not set(cfg['domains']) & set(range(201, 231))


def test_rigconf_expands_env_and_merges_local(tmp_path, monkeypatch):
    (tmp_path / 'rig.yaml').write_text('player: ${P_X:-none}\nwatchdog: {stall_s: 30, min_travel_m: 1}\n'
                                       'home: ~/x\n')
    (tmp_path / 'rig.local.yaml').write_text('watchdog: {stall_s: 5}\n')
    monkeypatch.delenv('CRANE_RIG_LOCAL', raising=False)
    monkeypatch.setenv('P_X', '/opt/p')
    rig = rigconf.load(tmp_path / 'rig.yaml')
    assert rig['player'] == '/opt/p' and rig['home'] == str(Path.home() / 'x')
    assert rig['watchdog'] == {'stall_s': 5, 'min_travel_m': 1}


def test_shipped_rig_yaml_has_no_host_paths():
    text = (RIG / 'rig.yaml').read_text()
    assert '/home/' not in text.replace('/home/roboboat/', '')


# --- inherited rig helpers -------------------------------------------------------------------

def test_segment_at_walks_the_schedule_then_ends():
    schedule = [[1, 0, 0, 0], [2, .5, 0, .1]]
    assert drive.segment_at(schedule, 0.5) == (0, 0, 0)
    assert drive.segment_at(schedule, 2.9) == (.5, 0, .1)
    assert drive.segment_at(schedule, 3.0) is None


def test_travel_and_active_pwm():
    assert check.travel([(0, 0), (3, 4), (1, 0)]) == 5
    assert check.active_pwm([[1500] * 4, [1500, 1500, 1530, 1500], [1510] * 4], 1500, 25) == 1


# --- the queue worker on fake runs -----------------------------------------------------------

@pytest.fixture
def rig(tmp_path, monkeypatch):
    monkeypatch.delenv('CRANE_RIG_LOCAL', raising=False)
    monkeypatch.setenv('FAKE_LOG', str(tmp_path / 'fake.jsonl'))
    cfg = {'container': 'none', 'lock_dir': str(tmp_path / 'locks'), 'max_instances': 3,
           'sessions_dir': str(tmp_path / 'sessions'),
           'queue': {'dir': str(tmp_path / 'q'), 'slots': 3, 'builders': 2, 'domains': '180-182',
                     'poll_s': 0.05, 'grace_s': 1}}
    (tmp_path / 'locks').mkdir()
    (tmp_path / 'rig.yaml').write_text(yaml.safe_dump(cfg))
    return rigq.load_config(tmp_path / 'rig.yaml')


FAST = {'startup_s': 1.5, 'stall_s': 1.0, 'motion_grace_s': 1.0, 'min_travel_m': 0.5}


def submit(q, mode, kind='cmd', **spec):
    return q.add(dict({'kind': kind, 'name': mode, 'argv': FAKE + [mode], 'watchdog': FAST}, **spec))


def drain(rig, slots=None, timeout=30):
    w = rigq.Worker(rig, slots=slots, out=open(os.devnull, 'w'))
    end = time.time() + timeout
    while time.time() < end:
        w.step()
        if not w.active and not w.q.jobs('queued'):
            return w
        time.sleep(0.05)
    raise AssertionError('queue did not drain: ' + json.dumps([j['state'] for j in w.q.jobs()]))


def fake_log(tmp_path):
    return [json.loads(line) for line in (tmp_path / 'fake.jsonl').read_text().splitlines()]


def alive(pid):
    try:
        os.kill(pid, 0)
    except ProcessLookupError:
        return False
    return Path(f'/proc/{pid}/stat').read_text().split()[2] != 'Z'


def test_each_failure_mode_trips_with_its_reason(rig):
    q = rigq.Queue(rig['queue']['dir'])
    jobs = {mode: submit(q, mode) for mode in ('succeed', 'stall', 'static', 'hang', 'fail')}
    drain(rig, slots=5)
    got = {mode: q.get(j['id']) for mode, j in jobs.items()}
    assert got['succeed']['state'] == 'done' and got['succeed']['exit'] == 0
    assert (got['stall']['state'], got['stall']['reason']) == ('tripped', 'stalled')
    assert (got['static']['state'], got['static']['reason']) == ('tripped', 'no_motion')
    assert (got['hang']['state'], got['hang']['reason']) == ('tripped', 'no_start')
    assert got['fail']['state'] == 'failed' and got['fail']['exit'] == 3
    rows = {r['name']: r for r in q.results()}
    assert set(rows) == set(jobs) and rows['stall']['reason'] == 'stalled'


def test_hung_run_is_killed_with_its_escaped_child_and_the_next_job_starts(rig, tmp_path):
    q = rigq.Queue(rig['queue']['dir'])
    hang = submit(q, 'hang', watchdog=dict(FAST, startup_s=None), timeout=1.0)   # hard timeout path
    nxt = submit(q, 'succeed')
    drain(rig, slots=1)
    hang, nxt = q.get(hang['id']), q.get(nxt['id'])
    assert (hang['state'], hang['reason']) == ('tripped', 'hard_timeout')
    assert hang['exit'] == -9                      # SIGTERM ignored: KILL after grace_s
    child = int(Path(rig['queue']['dir'], 'logs', f"{hang['id']}.progress.json.child").read_text())
    assert not alive(child), 'the setsid grandchild survived'
    assert child in hang['cleanup']['host_stopped']
    assert not alive(hang['pid'])
    assert nxt['state'] == 'done' and nxt['started'] >= hang['finished'] - 0.5


def test_concurrent_jobs_get_distinct_pool_domains_and_respect_slots(rig, tmp_path):
    q = rigq.Queue(rig['queue']['dir'])
    for _ in range(5):
        submit(q, 'succeed')
    w = rigq.Worker(rig, slots=2, out=open(os.devnull, 'w'))
    peak = 0
    end = time.time() + 30
    while (w.active or w.q.jobs('queued')) and time.time() < end:
        w.step()
        domains = [a.domain for a in w.active.values()]
        assert len(domains) == len(set(domains))
        peak = max(peak, len(domains))
        time.sleep(0.05)
    assert peak == 2
    assert {int(r['domain']) for r in fake_log(tmp_path)} <= {180, 181, 182}
    assert all(j['state'] == 'done' for j in q.jobs())


def test_domain_held_by_another_rig_run_is_skipped(rig, tmp_path):
    held = rigq.take_lock(tmp_path / 'locks' / 'simrig-domain-180.lock')
    try:
        q = rigq.Queue(rig['queue']['dir'])
        submit(q, 'succeed')
        drain(rig)
        assert fake_log(tmp_path)[0]['domain'] != '180'
    finally:
        os.close(held)


def test_builds_are_capped_by_host_builder_locks(rig, tmp_path):
    q = rigq.Queue(rig['queue']['dir'])
    other = rigq.take_lock(tmp_path / 'locks' / 'crane-builder-0.lock')   # another worker's build
    try:
        for _ in range(3):
            submit(q, 'succeed', kind='build')
        w = rigq.Worker(rig, out=open(os.devnull, 'w'))
        peak, end = 0, time.time() + 30
        while (w.active or w.q.jobs('queued')) and time.time() < end:
            w.step()
            peak = max(peak, len(w.active))
            time.sleep(0.05)
        assert peak == 1
        assert all(j['state'] == 'done' and j.get('domain') is None for j in q.jobs())
    finally:
        os.close(other)


def test_cancel_queued_and_running(rig):
    q = rigq.Queue(rig['queue']['dir'])
    running = submit(q, 'static', watchdog={})          # no limits: runs until cancelled
    queued = submit(q, 'succeed')
    w = rigq.Worker(rig, slots=1, out=open(os.devnull, 'w'))
    w.step()
    assert q.get(running['id'])['state'] == 'running'
    q.cancel(queued['id'])
    q.cancel(running['id'])
    end = time.time() + 10
    while w.active and time.time() < end:
        w.step()
        time.sleep(0.05)
    assert q.get(queued['id'])['state'] == 'cancelled'
    assert (q.get(running['id'])['state'], q.get(running['id'])['reason']) == ('cancelled', 'cancelled')


def test_jobs_left_running_by_a_dead_worker_are_failed(rig):
    q = rigq.Queue(rig['queue']['dir'])
    job = submit(q, 'succeed')
    q.update(job['id'], state='running', pid=999999999)
    rigq.Worker(rig, out=open(os.devnull, 'w')).adopt_orphans()
    assert (q.get(job['id'])['state'], q.get(job['id'])['reason']) == ('failed', 'worker_lost')


def test_cli_json_submit_list_show(rig, tmp_path, capsys):
    cfg = str(tmp_path / 'rig.yaml')
    assert rigq.main(['--config', cfg, '--json', 'submit', '--kind', 'cmd', '--no-start', '--name', 'x',
                      '--stall', '2', '--', 'true']) == 0
    job = json.loads(capsys.readouterr().out)
    assert job['state'] == 'queued' and job['argv'] == ['true'] and job['watchdog'] == {'stall_s': 2.0}
    rigq.main(['--config', cfg, '--json', 'list'])
    assert [j['id'] for j in json.loads(capsys.readouterr().out)] == [job['id']]
    assert rigq.main(['--config', cfg, '--json', 'submit', '--no-start', '--', '--domain', '7']) == 2
    assert 'belongs to the queue' in json.loads(capsys.readouterr().out)['error']


def test_worker_unit_install_uninstall(rig, tmp_path, monkeypatch, capsys):
    monkeypatch.setenv('XDG_CONFIG_HOME', str(tmp_path / 'xdg'))
    cfg = str(tmp_path / 'rig.yaml')
    assert rigq.main(['--config', cfg, '--json', 'worker', 'install', '--slots', '2']) == 0
    out = json.loads(capsys.readouterr().out)
    text = (tmp_path / 'xdg/systemd/user/rigq-worker.service').read_text()
    assert out['written'] and '@' not in text
    assert f'--config {cfg} worker --slots 2' in text and 'WantedBy=default.target' in text
    assert rigq.main(['--config', cfg, '--json', 'worker', 'uninstall']) == 0
    assert json.loads(capsys.readouterr().out)['removed'] and not (tmp_path / 'xdg/systemd/user/rigq-worker.service').exists()


# --- review fixes ----------------------------------------------------------------------------

def test_yaw_only_commands_never_start_the_no_motion_clock():
    t = probe.Tracker()
    t.on_odom(0, 0)
    for i in range(300):                       # 30 s of spinning in place at 10 Hz
        t.on_command(i * 0.1, 0, 0, 0.8)
    s = t.snapshot(0)
    assert s['cmd_active_s'] == 0 and s['travel_since_cmd'] is None
    wd = Watchdog(0, startup_s=None)
    assert wd.check(40, dict(s, sim_time=3.0, msgs=5), 'running') is None


def _sleeper(**kw):
    import subprocess
    return subprocess.Popen(['sleep', '60'], **kw)


def _reap(*procs):
    for p in procs:
        p.terminate()
        p.wait()


def _orphan_job(q, proc, **spec):
    job = submit(q, 'succeed', argv=['sleep', '60'])
    spec.setdefault('pid_start', rigq.start_ticks(proc.pid))
    q.update(job['id'], state='running', pid=proc.pid, **spec)
    return job


def test_adoption_spares_a_reused_pid(rig):
    q = rigq.Queue(rig['queue']['dir'])
    shared = _sleeper()                        # same process group as this test: not a job leader
    mismatch = _sleeper(start_new_session=True)
    try:
        a = _orphan_job(q, shared)             # argv[0] "sleep" matches the cmdline, pgid != pid
        b = _orphan_job(q, mismatch, pid_start=1)   # leader, but another process started at that PID
        rigq.Worker(rig, out=open(os.devnull, 'w')).adopt_orphans()
        assert shared.poll() is None and mismatch.poll() is None
        assert q.get(a['id'])['reason'] == q.get(b['id'])['reason'] == 'worker_lost'
    finally:
        _reap(shared, mismatch)


def test_adoption_stops_the_jobs_own_group(rig):
    q = rigq.Queue(rig['queue']['dir'])
    proc = _sleeper(start_new_session=True)
    try:
        _orphan_job(q, proc)
        rigq.Worker(rig, out=open(os.devnull, 'w')).adopt_orphans()
        assert proc.wait(timeout=5) != 0
    finally:
        _reap(proc)


@pytest.mark.parametrize('left', [None, 2])
def test_launch_fails_when_the_domain_is_not_clean(rig, monkeypatch, left):
    monkeypatch.setattr(rigq, 'container_is_ours', lambda name: True)
    monkeypatch.setattr(rigq, 'clear_domain', lambda c, d: {'domain': d, 'stray_left': left})
    q = rigq.Queue(rig['queue']['dir'])
    job = q.add({'kind': 'sim', 'name': 'x', 'args': []})
    w = rigq.Worker(rig, out=open(os.devnull, 'w'))
    w.step()
    got = q.get(job['id'])
    assert got['state'] == 'failed' and 'not clean' in got['reason'] and not w.active


def test_cancel_decides_under_the_queue_lock(rig, monkeypatch):
    q = rigq.Queue(rig['queue']['dir'])
    job = submit(q, 'succeed')
    real = q.update

    def worker_wins(job_id, _decide=None, **fields):     # the worker starts the job as cancel begins
        real(job_id, state='running')
        return real(job_id, _decide, **fields)
    monkeypatch.setattr(q, 'update', worker_wins)
    got = q.cancel(job['id'])
    assert got['state'] == 'running' and got['cancel_requested'] and q.results() == []


def test_a_job_submitted_during_the_exit_gets_served(rig):
    q = rigq.Queue(rig['queue']['dir'])
    late = []

    class Racy(rigq.Worker):
        def release_lock(self, fd):
            super().release_lock(fd)
            if not late:                       # the submit lands just after the lock is dropped
                late.append(submit(q, 'succeed'))
    w = Racy(rig, until_empty=True, linger_s=0, out=open(os.devnull, 'w'))
    assert w.run() == 0
    assert q.get(late[0]['id'])['state'] == 'done'
