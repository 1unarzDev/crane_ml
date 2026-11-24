"""Tools/SimCheck on synthetic inputs: python3 -I -m pytest Tools/SimCheck -q (no ROS, player or network)."""

import json
import socket
import struct
import subprocess
import sys
import threading
from pathlib import Path

import pytest

ROOT = Path(__file__).resolve().parents[3]
sys.path.insert(0, str(ROOT))

from Tools.SimCheck import axis, docking, sitl, smoke, thresholds  # noqa: E402
from Tools.SimCheck.watchdog import Tracker, Watchdog  # noqa: E402

LIMITS = thresholds()


# --- axis ------------------------------------------------------------------------------------

def response(surge=(0.5, 0.02), sway=(0.4, 0.01), yaw=(0.3, 0.8)):
    def phase(name, **fields):
        return {'name': name, 'sampleCount': 50, **{k: {'meanTail': v} for k, v in fields.items()}}
    return {'status': 'completed', 'phases': [
        phase('surge_positive', body_surge=surge[0], body_sway=surge[1]),
        phase('sway_positive', body_sway=sway[0], body_surge=sway[1]),
        dict(phase('yaw_positive', body_yaw_rate=yaw[0]), deltaPose={'yaw': yaw[1]})]}


def test_axis_contract_passes_flu_and_names_each_swapped_or_flipped_axis():
    ratio = LIMITS['axis']['dominance_ratio']
    assert axis.check(response(), ratio)['passed']
    swapped = axis.check(response(surge=(0.1, 0.5)), ratio)
    assert not swapped['passed'] and swapped['failures'][0].startswith('surge request mapped')
    flipped = axis.check(response(yaw=(-0.3, -0.8)), ratio)
    assert [f.split(':')[0] for f in flipped['failures']] == ['positive yaw request mapped incorrectly']


# --- sitl ------------------------------------------------------------------------------------

def test_peer_counts_telemetry_and_malformed_replies_over_loopback():
    """A stand-in player answers each servo packet with telemetry; one reply is garbage."""
    server = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    server.bind(('127.0.0.1', 0))
    server.settimeout(2.0)
    seen = {'invalid': 0, 'servo': 0}

    def serve():
        stamp = 0.0
        try:
            while True:
                data, peer_addr = server.recvfrom(65535)
                if len(data) != struct.calcsize('<HHI16H'):
                    seen['invalid'] += 1
                    continue
                seen['servo'] += 1
                stamp += 0.02
                reply = {'timestamp': stamp, 'imu': {'gyro': [0, 0, 0], 'accel_body': [0, 0, -9.8]},
                         'position': [0, 0, 0], 'attitude': [0, 0, 0], 'velocity': [0, 0, 0]}
                server.sendto(b'{bad' if seen['servo'] == 3 else json.dumps(reply).encode(), peer_addr)
        except OSError:
            pass

    thread = threading.Thread(target=serve, daemon=True)
    thread.start()
    result = sitl.peer(port=server.getsockname()[1], duration=0.4, frame_rate=50)
    server.close()
    assert seen['invalid'] == 1 and result['servoPacketsSent'] == seen['servo'] > 5
    assert result['malformedTelemetryPackets'] == 1 and not result['valid']
    assert result['telemetryPacketsReceived'] >= 3 and result['timestampMonotonic']


def test_acceptance_joins_peer_and_player_records():
    fixture = {'valid': True, 'lastFrame': 200}
    worker = {'valid': True, 'acceptedActions': 180, 'unknownSourceActions': 200, 'rejectedActions': 0,
              'staleActions': 0, 'crossEpisodeActions': 0,
              'actionTiming': {'acceptedActions': 180, 'knownSourceActions': 0, 'maximumReceiveToApplicationTicks': 2},
              'sitl': {'validServoPackets': 200, 'invalidServoPackets': 1, 'telemetryPackets': 190, 'lastFrame': 200}}
    lag = LIMITS['sitl']['maximum_lag_ticks']
    ok = sitl.acceptance(fixture, worker, lag)
    assert ok['passed'] and ok['actions']['replacedBeforeApplication'] == 20
    late = dict(worker, actionTiming=dict(worker['actionTiming'], maximumReceiveToApplicationTicks=lag + 1))
    assert sitl.acceptance(fixture, late, lag)['failures'] == ['action lag within bound']
    assert sitl.acceptance(fixture, worker, lag, 'aerial', climb=0.05)['failures'] == ['aerial climb']


# --- docking ---------------------------------------------------------------------------------

def sample(t, x, y, yaw, surge_cmd=0.0, v=0.0, r=0.0):
    return {'simSeconds': t, 'x': x, 'y': y, 'yaw': yaw, 'commandSurge': surge_cmd,
            'bodySurge': v, 'bodySway': 0.0, 'bodyYawRate': r}


def test_docking_flags_spinning_before_the_path_and_requires_a_settled_stop():
    spin = [sample(0.1 * k, 0, 0, 0.3 * k, r=3.0) for k in range(40)] + [sample(4.0, 0, 0, 0, surge_cmd=0.2)]
    assert 'rotate-only yaw travel' in docking.check(spin, LIMITS['docking'])['failures'][0]

    approach = [sample(0.1 * k, 0.1 * k, 0, 0, surge_cmd=0.15, v=0.15) for k in range(50)]
    stop = [sample(5.0 + 0.1 * k, 5.0, 0.05, 0.05) for k in range(70)]       # 6.9 s stopped at the goal
    assert docking.check(approach + stop, LIMITS['docking'], goal=(5.0, 0.0, 0.0))['passed']
    drifting = [dict(s, bodySurge=0.08) for s in stop]                        # creeping over 0.05 m/s
    held = docking.check(approach + drifting, LIMITS['docking'], goal=(5.0, 0.0, 0.0))
    assert not held['passed'] and held['settledSeconds'] == 0


# --- smoke -----------------------------------------------------------------------------------

def test_smoke_gates_on_rates_span_travel_thrust_and_truth():
    limits = LIMITS['smoke']
    assert smoke.travel([(0, 0), (3, 4), (1, 0)]) == 5
    assert smoke.active_pwm([[1500] * 4, [1500, 1500, 1530, 1500], [1510] * 4], 1500, 25) == 1
    ns = 10 ** 9
    stamps = {t: [int(k * ns / 50) for k in range(50 * 30)] for t in limits['min_hz']}
    odom = [(0.1 * k, 0.0) for k in range(30)]
    pwm = [[1600] * 4] * 20
    truth = {'schema': smoke.TRUTH_SCHEMA, 'elements': [{}]}
    rows = smoke.gates(smoke.stats_of(stamps, odom, pwm, limits), truth, limits, duration=30)
    assert all(ok for _, ok, _, _ in rows)
    cut = dict(stamps, **{'/clock': stamps['/clock'][:500]})                  # recorder stopped at 10 s
    rows = smoke.gates(smoke.stats_of(cut, odom, pwm, limits), {}, limits, duration=30)
    assert sorted(n for n, ok, _, _ in rows if not ok) == ['course_truth', 'span /clock']


# --- watchdog (stop rule) ----------------------------------------------------------------------

def snap(sim, msgs, **kw):
    return dict(sim_time=sim, msgs=msgs, **kw)


def test_watchdog_hard_timeout_wins_in_any_phase_and_post_has_its_own_limit():
    w = Watchdog(0, hard_s=10)
    assert w.check(5, snap(1, 1), 'running') is None
    assert w.check(11, snap(2, 2), 'post') == 'hard_timeout'
    w = Watchdog(0, stall_s=5, post_s=60)
    w.check(1, snap(1, 1), 'running')
    assert w.check(30, None, 'post') is None
    assert w.check(91, None, 'post') == 'post_timeout'


def test_watchdog_no_start_then_stall():
    w = Watchdog(0, startup_s=20)
    assert w.check(10, None, 'starting') is None
    assert w.check(15, snap(None, 0), 'starting') is None      # probe up, nothing flowing
    assert w.check(21, None, 'starting') == 'no_start'
    w = Watchdog(0, stall_s=30)
    assert w.check(1, snap(1.0, 10), 'running') is None
    assert w.check(49, snap(5.0, 50), 'running') is None
    assert w.check(80, snap(5.0, 50), 'running') == 'stalled'
    w = Watchdog(0, stall_s=5)
    w.check(1, snap(1, 1), 'running')
    assert w.check(7, None, 'running') == 'stalled'            # an unreadable probe is no progress


def test_watchdog_no_motion_when_commanded_but_static():
    w = Watchdog(0, motion_grace_s=10, min_travel_m=0.5)
    assert w.check(1, snap(1, 1, cmd_active_s=9, travel_since_cmd=0.0), 'running') is None
    assert w.check(2, snap(2, 2, cmd_active_s=11, travel_since_cmd=0.1), 'running') == 'no_motion'
    w = Watchdog(0, motion_grace_s=10, min_travel_m=0.5)
    assert w.check(1, snap(1, 1, cmd_active_s=30, travel_since_cmd=4.0), 'running') is None
    assert w.check(2, snap(2, 2, cmd_active_s=0, travel=0.0), 'running') is None   # nothing commanded
    w = Watchdog(0, motion_grace_s=10, min_travel_m=0.5, expect_motion=True)
    assert w.check(1, snap(1, 1, travel=0), 'running') is None
    assert w.check(12, snap(12, 12, travel=0.2), 'running') == 'no_motion'
    with pytest.raises(ValueError):
        Watchdog(0, stal_s=3)


def test_tracker_measures_travel_from_the_first_motion_command_only():
    t = Tracker()
    t.on_odom(0, 0)
    t.on_clock(1.0)
    t.on_odom(1, 0)                          # drift before any command
    for i in range(11):
        t.on_command(i * 0.1, 0.6, 0, 0)
    t.on_odom(4, 0)
    s = t.snapshot(0)
    assert s['msgs'] == 4 and s['sim_time'] == 1.0
    assert s['cmd_active_s'] == pytest.approx(1.0)
    assert s['travel_since_cmd'] == 3 and s['travel'] == 4
    t = Tracker()
    t.on_command(0, 0.01, 0, 0)
    t.on_command(1.0, 0.5, 0, 0)
    t.on_command(3.0, 0.5, 0, 0)            # a 2 s gap does not count
    assert t.snapshot(0)['cmd_active_s'] == 0


def test_yaw_only_commands_never_start_the_no_motion_clock():
    t = Tracker()
    t.on_odom(0, 0)
    for i in range(300):                     # 30 s of spinning in place at 10 Hz
        t.on_command(i * 0.1, 0, 0, 0.8)
    s = t.snapshot(0)
    assert s['cmd_active_s'] == 0 and s['travel_since_cmd'] is None
    assert Watchdog(0, startup_s=None).check(40, dict(s, sim_time=3.0, msgs=5), 'running') is None


# --- entry -----------------------------------------------------------------------------------

@pytest.mark.parametrize('entry', [['-m', 'Tools.SimCheck'], [str(ROOT / 'Tools' / 'SimCheck')]])
def test_one_entry_as_module_or_script_path(tmp_path, entry):
    (tmp_path / 'r.json').write_text(json.dumps(response(sway=(-0.4, 0.0))))
    out = subprocess.run([sys.executable, *entry, 'axis', str(tmp_path / 'r.json')],
                         cwd=ROOT, capture_output=True, text=True)
    assert out.returncode == 1, out.stderr
    assert json.loads(out.stdout)['failures'][0].startswith('sway request mapped')
