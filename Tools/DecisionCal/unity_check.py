#!/usr/bin/env python3
"""Independent-plant cross-check: fly one library controller on the gate -> U-turn -> dock mission
with the CRANE Unity player (C# boat plant) as the plant, over its lockstep link.

The mission, guidance, velocity law and score are ``mission.run`` itself, unedited: the only thing
swapped is the plant. ``mission.run`` builds its plant with ``Plant(spec, seeds, t_max=...)`` and
touches exactly (see UNITY_CHECK.md):  ``.t``  ``.tick`` (not read by run itself, kept for the
sensors)  ``.measure()``  ``.st`` ([n, 6] x y psi u v r, course frame, body velocities)
``.q_applied``  ``.thr.T``  ``.step(q)``.  ``UnityPlant`` provides those over the TCP protocol, and
``run_mission`` rebinds ``mission.Plant`` for the duration of one call (no file is edited).

Frames. The harness route starts at (0, 0) heading +x (ENU, yaw CCW from +x, body FLU). The player
reports world-ROS (/crane/odom: x forward = Unity +z, y = -Unity x, yaw CCW from +x, velocities
WORLD frame, yaw rate CCW = -Unity angular.y) and sends its course start pose in ``hello``.
Course -> world is crane's ``RoboBoatCourse.ToWorld``:  w = start + R(yaw0) c;  yaw_w = yaw_c + yaw0.
Scenario current/wind are given in the COURSE frame (the player adds yaw0 itself), so they are
passed through unrotated. World velocity is rotated to the course frame and then into the body.

Modes
    --self-test          numpy only, no Unity: guards the adapter (J identical to mission.run through
                         the TCP protocol for 3 controllers), the frame transform, and the config map
    --dry-run            one mission against the built-in fake player (numpy Plant behind the same TCP
                         protocol), then the same mission directly: prints both
    --plan               numpy only: rank the 16 controllers on the chosen boats (top-2 / bottom-2)
    --emit-config DIR    write profile.json / faults.json / scenario.json for the chosen boat; print the
                         launch command
    --live               launch the player through ``heavy unity`` (admission!), run one mission
Run with:  nice -n 19 python3 unity_check.py --self-test
"""

import argparse
import json
import math
import os
import shlex
import socket
import subprocess
import sys
import threading
import time
from pathlib import Path
from types import SimpleNamespace

import numpy as np

import _paths  # noqa: F401  (sys.path + provenance pins; also keeps this file runnable from any cwd)
import mission
from library import LIBRARY
from plant import B as B_GEOM, DT, N_THR, Plant, PlantSpec, Sensors, static_thrust

HOME = Path.home()
PLAYER = HOME / '.cache' / 'nlc-lockstep' / 'player-linux' / 'CRANE.x86_64'
HEAVY = HOME / 'astro_dock' / 'src' / 'mhseals_agents' / 'tools' / 'heavy'
OUT_ROOT = HOME / '.cache' / 'decisioncal' / 'unity-check'
K_UNITY = 51.5            # N at full command of every thruster in the Ideal profile (kForward = kReverse)
Q_MATCH = 0.6             # command at which a gamma != 2 hidden thrust curve is matched by Unity's k q^2
SCENE = 'Roboboat Course'
THRUSTERS = ('front_left', 'front_right', 'rear_right', 'rear_left')
MOUNT = ((0.27, 0.34, 135.0), (0.27, -0.34, -135.0), (-0.27, -0.34, -45.0), (-0.27, 0.34, 45.0))


def wrap(a):
    return (a + math.pi) % (2 * math.pi) - math.pi


# ---------------------------------------------------------------------------------------------
# frames
# ---------------------------------------------------------------------------------------------

class Frame:
    """Rigid transform between the harness course frame and the player's world-ROS frame."""

    def __init__(self, x0=0.0, y0=0.0, yaw0=0.0):
        self.x0, self.y0, self.yaw0 = float(x0), float(y0), float(yaw0)
        self.c, self.s = math.cos(self.yaw0), math.sin(self.yaw0)

    def point_to_world(self, x, y):
        return self.x0 + self.c * x - self.s * y, self.y0 + self.s * x + self.c * y

    def world_to_state(self, x, y, yaw, vx, vy, r):
        """Player (world-ROS, world-frame velocity) -> harness state (course frame, body velocity)."""
        dx, dy = x - self.x0, y - self.y0
        px, py = self.c * dx + self.s * dy, -self.s * dx + self.c * dy
        psi = wrap(yaw - self.yaw0)
        cv, sv = math.cos(yaw), math.sin(yaw)          # world velocity -> body with the WORLD yaw
        return np.array([px, py, psi, cv * vx + sv * vy, -sv * vx + cv * vy, r])

    def state_to_world(self, st):
        """Harness state -> player message fields (x y yaw vx vy r), the inverse of world_to_state."""
        x, y = self.point_to_world(st[0], st[1])
        yaw = wrap(st[2] + self.yaw0)
        cv, sv = math.cos(yaw), math.sin(yaw)
        return x, y, yaw, cv * st[3] - sv * st[4], sv * st[3] + cv * st[4], st[5]


# ---------------------------------------------------------------------------------------------
# link (controller side of the lockstep protocol)
# ---------------------------------------------------------------------------------------------

class LinkError(RuntimeError):
    pass


class Link:
    """TCP server for the player (the player connects). Line protocol, see CraneLockstepControl.cs."""

    def __init__(self, host='127.0.0.1', port=0, io_timeout=300.0):
        self.srv = socket.socket()
        self.srv.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.srv.bind((host, port))
        self.srv.listen(1)
        self.host, self.port = host, self.srv.getsockname()[1]
        self.io_timeout = io_timeout
        self.conn = self.rf = None
        self.hello = None

    def accept(self, timeout, alive=None):
        """Wait for the player; ``alive()`` False (the player process died) aborts early."""
        end = time.time() + timeout
        self.srv.settimeout(1.0)
        while True:
            try:
                self.conn, _ = self.srv.accept()
                break
            except socket.timeout:
                if alive is not None and not alive():
                    raise LinkError('the player exited before connecting')
                if time.time() > end:
                    raise LinkError(f'no player connected within {timeout:.0f} s')
        self.conn.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
        self.conn.settimeout(self.io_timeout)
        self.rf = self.conn.makefile('rb')
        parts = self._read()
        if parts[0] != 'hello' or len(parts) != 5:
            raise LinkError(f'expected hello, got {parts}')
        sx, sy, syaw, dt = (float(v) for v in parts[1:])
        if abs(dt - DT) > 1e-6:
            raise LinkError(f'player fixed_dt {dt} != harness DT {DT}')
        self.hello = Frame(sx, sy, syaw)
        return self.hello

    def _read(self):
        line = self.rf.readline()
        if not line:
            raise LinkError('the player closed the link')
        return line.decode('ascii').split()

    def _send(self, text):
        self.conn.sendall((text + '\n').encode('ascii'))

    def scenario(self, cur_speed, cur_toward_deg, wind_speed, wind_toward_deg):
        self._send(f'scenario {cur_speed!r} {cur_toward_deg!r} {wind_speed!r} {wind_toward_deg!r}')

    def read_state(self):
        p = self._read()
        if p[0] != 's' or len(p) != 9:
            raise LinkError(f'expected state, got {p}')
        return int(p[1]), float(p[2]), tuple(float(v) for v in p[3:])

    def pulses(self, p):
        self._send('p ' + ' '.join(repr(float(v)) for v in p))

    def reset(self):
        self._send('reset')

    def end(self):
        try:
            self._send('end')
        except OSError:
            pass

    def close(self):
        for o in (self.rf, self.conn, self.srv):
            try:
                o.close()
            except Exception:
                pass


# ---------------------------------------------------------------------------------------------
# the adapter: what mission.run needs from Plant
# ---------------------------------------------------------------------------------------------

class WallLimit(RuntimeError):
    pass


class UnityPlant:
    """Drop-in for ``plant.Plant`` (n = 1) whose truth comes from the Unity player.

    Sensors are NOT Unity's: the harness's own ``Sensors`` (odometry latency, white noise from the
    hidden boat's spec, counter-based draws keyed by seed/tick) are applied to the Unity truth, so the
    controller sees exactly the measurement model it sees on the numpy boats."""

    def __init__(self, link, spec, seeds, t_max=120.0, deadline=None, trace_every=25):
        if spec.n != 1:
            raise ValueError('UnityPlant drives one boat per player')
        self.link, self.spec, self.n, self.dt = link, spec, 1, DT
        self.frame = link.hello
        self.seeds = np.asarray(seeds, np.uint64)
        self.sensors = Sensors(spec, self.seeds, DT)
        self.deadline, self.trace_every = deadline, trace_every
        self.t, self.tick = 0.0, 0
        self.q_applied = np.zeros((1, N_THR))
        self.thr = SimpleNamespace(T=np.zeros((1, N_THR)))      # modelled thrust (energy metric only)
        self.trace = []
        self.tick0, self.wall0 = None, time.time()
        self._receive(first=True)
        self.delayed = self.sensors.push(self.st)

    def _receive(self, first=False):
        tick, _t, fields = self.link.read_state()
        if first:
            self.tick0 = tick
        elif tick != self.tick0 + self.tick:
            raise LinkError(f'tick mismatch: player {tick}, expected {self.tick0 + self.tick}')
        self.st = self.frame.world_to_state(*fields)[None, :]
        if self.tick % self.trace_every == 0:
            self.trace.append([self.t, *self.st[0].tolist()])

    def step(self, q):
        if self.deadline is not None and time.time() > self.deadline:
            raise WallLimit(f'wall limit at t = {self.t:.2f} s')
        q = np.clip(np.nan_to_num(np.asarray(q, float)), -1.0, 1.0)
        self.q_applied = q.copy()
        s = self.spec
        self.thr.T = static_thrust(q, s.k_fwd, s.k_rev, s.gamma, s.deadband)
        self.link.pulses(1500.0 + 400.0 * q[0])
        self.t += self.dt
        self.tick += 1
        self._receive()
        self.delayed = self.sensors.push(self.st)

    def measure(self, channel=0):
        return self.sensors.read(self.delayed, self.tick, channel)

    def close(self):
        self.link.end()


def run_mission(link, spec, ctrl_idx, seed, t_max=mission.T_MAX, deadline=None, log_every=0, lib=LIBRARY):
    """``mission.run`` with the plant replaced by the Unity (or fake) player on ``link``. Returns
    (metrics, streams, plant). The module attribute is restored on exit."""
    holder = {}

    def factory(sp, seeds, t_max=t_max, **_):
        holder['plant'] = UnityPlant(link, sp, seeds, t_max=t_max, deadline=deadline)
        return holder['plant']

    original = mission.Plant
    mission.Plant = factory
    try:
        m, streams = mission.run(spec, [ctrl_idx], [seed], t_max=t_max, lib=lib, log_every=log_every)
    except BaseException:
        if 'plant' in holder:
            holder['plant'].close()
        raise
    finally:
        mission.Plant = original
    holder['plant'].close()
    return m, streams, holder['plant']


# ---------------------------------------------------------------------------------------------
# hidden boat -> Unity configuration
# ---------------------------------------------------------------------------------------------

def match_k(spec):
    """Unity's static map is K q^2 with a hard deadband (no rescale). Return per-thruster (k_fwd, k_rev)
    that reproduce the hidden boat's thrust k e^gamma, e = (|q|-db)/(1-db), at command Q_MATCH."""
    e = np.maximum(Q_MATCH - spec.deadband, 0.0) / (1.0 - spec.deadband)
    shape = e ** spec.gamma[:, None] / Q_MATCH ** 2
    return spec.k_fwd * shape, spec.k_rev * shape


def unity_config(spec, i=0, match=True):
    """(profile dict, faults dict, scenario dict, notes) for boat ``i`` of ``spec``.

    profile.json (``--crane-boat-profile-json``) is a full MarineSurfaceParameters (it replaces the whole
    parameter block): hull mass/inertia, added mass, damping, Ideal mounts and k = 51.5 N.
    faults.json (``--crane-calibration-faults``) is the ActuatorErrorProfile layer on top: per-thruster
    strength / reverse ratio / extra deadband / dead time and the shared spin-up lag."""
    r = spec.take([i])
    kf, kr = match_k(r) if match else (r.k_fwd, r.k_rev)
    kf, kr = kf[0], kr[0]
    d1, d2 = r.d1[0], r.d2[0]
    sd = lambda j: {'d1Pos': float(d1[j]), 'd1Neg': float(d1[j]), 'd2Pos': float(d2[j]), 'd2Neg': float(d2[j])}
    profile = {
        'name': 'DecisionCalHidden', 'fixedDeltaTime': DT, 'mass': float(r.mass[0]), 'ixx': 2.2, 'iyy': 2.0,
        'izz': float(r.izz[0]), 'comBow': 0.0, 'comPort': 0.0,
        'addedMass': {'a11': float(r.a11[0]), 'a22': float(r.a22[0]), 'a26': float(r.a26[0]), 'a66': float(r.a66[0])},
        'surge': sd(0), 'sway': sd(1), 'yaw': sd(2),
        'heave': {'d1': 150.0, 'd2': 40.0}, 'roll': {'d1': 30.0, 'd2': 10.0}, 'pitch': {'d1': 30.0, 'd2': 10.0},
        'thrusters': [{'name': n, 'x': m[0], 'y': m[1], 'z': 0.0, 'alphaDeg': m[2], 'kForward': K_UNITY,
                       'kReverse': K_UNITY, 'vRef': 16.0, 'nV': 0.0, 'deadbandUs': 0.0, 'offsetUs': 0.0,
                       'tauT': 0.0, 'deadTime': 0.0, 'submersionDepth': 0.08, 'kappa': 0.0}
                      for n, m in zip(THRUSTERS, MOUNT)],
        'envelope': {'surge': 2.5, 'sway': 1.7, 'yaw': 1.9},
        'provenance': {'kind': 'ideal', 'summary': 'decisioncal hidden boat'}}
    # a thruster whose strength would be 0 is not representable as 'reverse'; ratios are k_rev / k_fwd
    faults = {
        'names': list(THRUSTERS),
        'strengths': [float(v) for v in kf / K_UNITY],
        'reverse': [float(v) for v in kr / kf],
        'deadband_us': [float(v) for v in r.deadband[0] * 400.0],
        'deadtime_s': [float(r.dead_time[0])] * N_THR,
        'spinup': float(0.5 * (r.tau_up[0] + r.tau_down[0]))}
    cur, wind = r.current[0], r.wind[0]
    scenario = {'cur_speed': float(np.hypot(*cur)), 'cur_toward_deg': float(np.degrees(np.arctan2(cur[1], cur[0]))),
                'wind_speed': float(np.hypot(*wind)), 'wind_toward_deg': float(np.degrees(np.arctan2(wind[1], wind[0])))}
    lost = {'rev_hold_s': float(r.rev_hold[0]), 'slew_N_per_s': float(r.slew[0]),
            'tau_up_s': float(r.tau_up[0]), 'tau_down_s': float(r.tau_down[0]), 'gamma': float(r.gamma[0]),
            'ou_sigma': float(r.ou_sigma[0]), 'spatial_amp': float(r.spatial_amp[0]),
            'gust_sigma': float(r.gust_sigma[0]), 'odom_latency_s': float(r.odom_latency[0])}
    return profile, faults, scenario, lost


def check_faults(f):
    """The ranges ActuatorErrorProfile.Parse enforces (so a bad boat fails here, not in the player)."""
    rng = {'strengths': (0, 4), 'reverse': (0.2, 1.5), 'deadband_us': (0, 200), 'deadtime_s': (0, 2)}
    for k, (lo, hi) in rng.items():
        for v in f[k]:
            if not lo <= v <= hi:
                raise ValueError(f'{k} {v:.4g} outside [{lo}, {hi}] (Unity would reject the fault JSON)')
    if not 0.001 <= f['spinup'] <= 2:
        if f['spinup'] != 0.0:
            raise ValueError(f'spinup {f["spinup"]} outside [0.001, 2]')
        f['spinup'] = 0.001                           # Unity's minimum; tau = 0 is not accepted
    return f


def representable_numpy(spec):
    """The numpy hidden boat reduced to what Unity can represent: mean current and wind only, no OU /
    spatial / gust, no reversal hold, no slew limit, one symmetric lag, gamma = 2 with Unity's matched k, no
    deadband. The difference between this and the Unity result is implementation, not representability."""
    kf, kr = match_k(spec)
    lag = 0.5 * (spec.tau_up + spec.tau_down)
    z = np.zeros(spec.n)
    return spec.with_(k_fwd=kf, k_rev=kr, gamma=np.full(spec.n, 2.0), deadband=np.zeros((spec.n, N_THR)),
                      tau_up=lag, tau_down=lag.copy(), rev_hold=z.copy(), slew=np.full(spec.n, np.inf),
                      ou_sigma=z.copy(), spatial_amp=z.copy(), gust_sigma=z.copy())


# ---------------------------------------------------------------------------------------------
# fake player (dry-run): the numpy Plant behind the same TCP protocol, in a rotated/translated world
# ---------------------------------------------------------------------------------------------

def fake_player(host, port, spec, seed, start, t_max=mission.T_MAX, log=None):
    frame = Frame(*start)
    f = socket.create_connection((host, port), timeout=60)
    f.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
    f.settimeout(300)
    rf = f.makefile('rb')
    send = lambda s: f.sendall((s + '\n').encode('ascii'))
    send(f'hello {start[0]!r} {start[1]!r} {start[2]!r} {DT!r}')
    sc = rf.readline().decode().split()
    assert sc[0] == 'scenario' and len(sc) == 5, sc
    if log is not None:
        log['scenario'] = [float(v) for v in sc[1:]]
    plant, tick, ep_tick0 = Plant(spec, [seed], t_max=t_max), 0, 0
    while True:
        w = frame.state_to_world(plant.st[0])
        send(f's {tick} {(tick - ep_tick0) * DT!r} ' + ' '.join(repr(float(v)) for v in w))
        line = rf.readline().decode().split()
        if not line or line[0] == 'end':
            break
        if line[0] == 'reset':
            plant, ep_tick0 = Plant(spec, [seed], t_max=t_max), tick
            continue
        assert line[0] == 'p' and len(line) == 5, line
        plant.step(np.array([[(float(v) - 1500.0) / 400.0 for v in line[1:]]]))
        tick += 1
    if log is not None:
        log['ticks'] = tick
    f.close()


def dry_run(spec, ctrl, seed, start, t_max=mission.T_MAX, log_every=0):
    """One mission through TCP + fake player. Returns (metrics, streams, plant, fake_log)."""
    link = Link()
    log = {}
    th = threading.Thread(target=fake_player, args=(link.host, link.port, spec, seed, start, t_max, log), daemon=True)
    th.start()
    link.accept(30)
    link.scenario(0.0, 0.0, 0.0, 0.0)
    try:
        m, streams, plant = run_mission(link, spec, ctrl, seed, t_max=t_max, log_every=log_every)
    finally:
        th.join(30)
        link.close()
    return m, streams, plant, log


# ---------------------------------------------------------------------------------------------
# selection
# ---------------------------------------------------------------------------------------------

def get_boat(hidden_seed, stratum, index):
    import hidden
    n = index + 1
    spec = hidden.sample_hidden(n, hidden_seed, stratum)
    return spec.take([index])


def rank_controllers(spec, seed=1, t_max=mission.T_MAX):
    """numpy J of all 16 controllers on one boat (one batched mission). Returns J [16], metrics."""
    ctrl = np.arange(len(LIBRARY))
    m, _ = mission.run(spec.repeat(len(ctrl)), ctrl, np.full(len(ctrl), seed), t_max=t_max)
    return m['J'], m


def fmt(m):
    return {k: float(np.asarray(v).ravel()[0]) for k, v in m.items() if k in
            ('J', 'docked', 't_dock', 'contacts', 'gate_miss', 'pose_err', 'progress')}


# ---------------------------------------------------------------------------------------------
# self-test
# ---------------------------------------------------------------------------------------------

def self_test():
    rng = np.random.default_rng(7)
    # 1. frame round trip and the documented sign convention (RoboBoatCourse.ToWorld)
    fr = Frame(3.0, -2.0, math.pi / 2)
    wx, wy = fr.point_to_world(1.0, 0.0)
    assert abs(wx - 3.0) < 1e-12 and abs(wy - -1.0) < 1e-12, (wx, wy)          # course +x -> world +y at yaw0 = 90 deg
    for _ in range(100):
        f = Frame(*rng.normal(size=2) * 20, rng.uniform(-math.pi, math.pi))
        st = np.array([*rng.normal(size=2) * 10, rng.uniform(-3, 3), *rng.normal(size=3)])
        back = f.world_to_state(*f.state_to_world(st))
        d = back - st
        d[2] = wrap(d[2])
        assert np.abs(d).max() < 1e-9, d
    # a boat moving along course +x at 1 m/s has body u = 1 regardless of the start yaw
    w = fr.state_to_world(np.array([0, 0, 0, 1.0, 0, 0.1]))
    assert abs(w[3]) < 1e-12 and abs(w[4] - 1.0) < 1e-12 and abs(w[5] - 0.1) < 1e-12, w
    print('frames: ok')

    # 2. config map: Unity's k q^2 reproduces the hidden thrust at Q_MATCH; ranges accepted by Unity
    import hidden
    for stratum in hidden.STRATA:
        spec = hidden.sample_hidden(20, 4242, stratum)
        kf, kr = match_k(spec)
        q = np.full((20, N_THR), Q_MATCH)
        want = static_thrust(q, spec.k_fwd, spec.k_rev, spec.gamma, spec.deadband)
        assert np.allclose(kf * Q_MATCH ** 2, want), stratum
        want_r = static_thrust(-q, spec.k_fwd, spec.k_rev, spec.gamma, spec.deadband)
        assert np.allclose(-kr * Q_MATCH ** 2, want_r), stratum
        for i in range(20):
            profile, faults, scenario, _ = unity_config(spec, i)
            check_faults(faults)
            json.dumps([profile, faults, scenario])
            assert len(faults['strengths']) == len(faults['reverse']) == len(faults['deadtime_s']) == 4
    print('config map: ok')

    # 3. adapter + TCP + fake player reproduce mission.run, three controllers (one with a rotated start)
    spec = hidden.sample_hidden(1, 4242, 'steady')
    worst = 0.0
    for ctrl, start in ((0, (0.0, 0.0, 0.0)), (8, (0.0, 0.0, 0.0)), (13, (12.5, -7.25, 2.2))):
        t0 = time.time()
        ref, _ = mission.run(spec, [ctrl], [11])
        got, _, plant, log = dry_run(spec, ctrl, 11, start)
        a, b = fmt(ref), fmt(got)
        dj = abs(a['J'] - b['J'])
        worst = max(worst, dj)
        same = all(a[k] == b[k] for k in ('docked', 'contacts', 'gate_miss'))
        assert same and dj < (1e-9 if start[2] == 0 and start[0] == 0 else 0.05), (ctrl, a, b)
        assert log['ticks'] == plant.tick, (log, plant.tick)
        print(f'  c{ctrl:02d} start={start}: J numpy {a["J"]:.4f} adapter {b["J"]:.4f} dJ={dj:.2e} '
              f'docked={a["docked"]:.0f} ({time.time() - t0:.0f} s)')
    print(f'adapter == mission.run for 3 controllers (max |dJ| {worst:.2e}): ok')
    print('SELF-TEST PASS')


# ---------------------------------------------------------------------------------------------
# launch
# ---------------------------------------------------------------------------------------------

def player_argv(out, port, seed, a):
    return [str(HEAVY), 'unity', str(PLAYER), '-batchmode', '-screen-fullscreen', '0', '-screen-width', '640',
            '-screen-height', '360', '-logFile', str(out / 'player.log'),
            '--crane-worker', '--crane-benchmark', '--crane-disable-ros', '--crane-disable-sitl',
            '--crane-worker-id', '0', '--crane-seed', str(seed), '--crane-course-seed', str(seed),
            '--crane-course-family', 'roboboat_2026', '--crane-course-truth', str(out / 'course_truth.json'),
            '--crane-scene', SCENE, '--crane-scenario', 'cluster', '--crane-sim-profile', a.sim_profile,
            '--crane-time-scale', str(a.time_scale), '--crane-warmup', '5', '--crane-duration', str(a.player_wall),
            '--crane-output', str(out / 'result.json'),
            '--crane-boat-profile', 'ideal', '--crane-boat-profile-json', str(out / 'profile.json'),
            '--crane-calibration-faults', str(out / 'faults.json'),
            '--crane-lockstep', f'127.0.0.1:{port}']


def write_config(out, spec, i, match=True, env=True):
    out.mkdir(parents=True, exist_ok=True)
    profile, faults, scenario, lost = unity_config(spec, i, match)
    faults = check_faults(faults)
    if not env:
        scenario = dict.fromkeys(scenario, 0.0)
    (out / 'profile.json').write_text(json.dumps(profile, indent=2))
    (out / 'faults.json').write_text(json.dumps(faults, indent=2))
    (out / 'scenario.json').write_text(json.dumps(scenario, indent=2))
    return scenario, lost


def progress_of(st):
    d2 = (mission.PATH[:, 0] - st[0]) ** 2 + (mission.PATH[:, 1] - st[1]) ** 2
    return float(mission.ARC[int(np.argmin(d2))] / mission.S_END)


def live(a, spec, label):
    out = OUT_ROOT / label
    scenario, lost = write_config(out, spec, 0, match=not a.no_match, env=not a.no_env)
    link = Link(port=a.port)
    argv = player_argv(out, link.port, a.seed, a)
    (out / 'launch.txt').write_text(' '.join(shlex.quote(x) for x in argv) + '\n')
    print('launch:', (out / 'launch.txt').read_text().strip())
    env = {k: v for k, v in os.environ.items() if k not in ('DISPLAY', 'WAYLAND_DISPLAY')}   # never the owner's display
    env.update({'HEAVY_WHO': 'decisioncal unity_check lockstep', 'HEAVY_MAX_S': str(int(a.player_wall) + 120),
                'SDL_VIDEODRIVER': 'dummy'})
    xvfb = None
    if a.xvfb:                                       # a private virtual X display (no window anyone can see)
        xvfb = subprocess.Popen(['Xvfb', a.xvfb, '-screen', '0', '640x360x24', '-nolisten', 'tcp'],
                                stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL, start_new_session=True)
        env['DISPLAY'] = a.xvfb
        time.sleep(1.5)
    proc = subprocess.Popen(argv, env=env, stdout=open(out / 'heavy.out', 'w'), stderr=subprocess.STDOUT,
                            start_new_session=True)
    t_launch = time.time()
    result = {'label': label, 'controller': a.controller, 'seed': a.seed, 'scenario': scenario, 'not_representable': lost}
    plant = None
    try:
        link.accept(a.accept_timeout, alive=lambda: proc.poll() is None)
        fr = link.hello
        result['hello'] = [fr.x0, fr.y0, fr.yaw0]
        link.scenario(scenario['cur_speed'], scenario['cur_toward_deg'], scenario['wind_speed'], scenario['wind_toward_deg'])
        deadline = time.time() + a.mission_wall
        t_run = time.time()
        try:
            m, streams, plant = run_mission(link, spec, a.controller, a.seed, deadline=deadline, log_every=25)
            result['metrics'] = fmt(m)
            result['complete'] = True
        except WallLimit as e:
            result['complete'] = False
            result['wall_limit'] = str(e)
        result['mission_wall_s'] = time.time() - t_run
    finally:
        link.close()
        try:
            proc.wait(60)
        except subprocess.TimeoutExpired:
            os.killpg(proc.pid, 15)
        result['player_rc'] = proc.poll()
        if xvfb is not None:
            xvfb.terminate()
        result['launch_to_end_wall_s'] = time.time() - t_launch
    res = out / 'result.json'
    if res.is_file():
        r = json.loads(res.read_text())
        result['water_invalid_searches'] = int(r.get('invalidWaterSearches') or 0)
    return result, plant


# ---------------------------------------------------------------------------------------------
# cli
# ---------------------------------------------------------------------------------------------

def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    mode = ap.add_mutually_exclusive_group(required=True)
    for k in ('self-test', 'dry-run', 'plan', 'live'):
        mode.add_argument('--' + k, action='store_true')
    mode.add_argument('--emit-config', metavar='DIR')
    ap.add_argument('--hidden-seed', type=int, default=4242, help='hidden.sample_hidden seed (development seed by default; never a confirmatory seed)')
    ap.add_argument('--stratum', default='steady')
    ap.add_argument('--boat-index', type=int, default=0)
    ap.add_argument('--boats', default='steady:0,steady:1,timing:0,timing:1,transient:0,mixed:0',
                    help='--plan: stratum:index list')
    ap.add_argument('--controller', type=int, default=8)
    ap.add_argument('--seed', type=int, default=1, help='noise seed (CRN key) and player --crane-seed')
    ap.add_argument('--t-max', type=float, default=mission.T_MAX)
    ap.add_argument('--start', default='5,-3,0.7', help='dry-run fake player start pose x,y,yaw (world)')
    ap.add_argument('--port', type=int, default=0)
    ap.add_argument('--sim-profile', default='minimal')
    ap.add_argument('--time-scale', type=float, default=4.0)
    ap.add_argument('--player-wall', type=float, default=170.0, help='player --crane-duration (WALL s, then it quits)')
    ap.add_argument('--mission-wall', type=float, default=150.0, help='controller-side wall limit, s, after connect')
    ap.add_argument('--accept-timeout', type=float, default=170.0, help='s to wait for the player (heavy admission + load)')
    ap.add_argument('--no-match', action='store_true', help='do not match gamma != 2 thrust curves at q = 0.6')
    ap.add_argument('--no-env', action='store_true', help='zero current and wind in the scenario')
    ap.add_argument('--xvfb', default=None, metavar=':N', help='start a private Xvfb on this display for the player '
                    '(the headless Vulkan device may need an X display; never the owner\'s :0/:1)')
    ap.add_argument('--out', default=None)
    a = ap.parse_args(argv)

    if a.self_test:
        return self_test()
    if a.plan:
        rows = []
        for tok in a.boats.split(','):
            st, idx = tok.split(':')
            J, m = rank_controllers(get_boat(a.hidden_seed, st, int(idx)), a.seed, a.t_max)
            order = np.argsort(J)
            rows.append({'boat': tok, 'J': J.round(2).tolist(), 'top2': order[:2].tolist(), 'bottom2': order[-2:].tolist(),
                         'docked': m['docked'].astype(int).tolist()})
            print(f'{tok:12s} top2 {order[:2].tolist()} J {J[order[:2]].round(1).tolist()}  bottom2 '
                  f'{order[-2:].tolist()} J {J[order[-2:]].round(1).tolist()}  docked {int(m["docked"].sum())}/16', flush=True)
        if a.out:
            Path(a.out).write_text(json.dumps({'hidden_seed': a.hidden_seed, 'seed': a.seed, 'rows': rows}, indent=1))
        return rows
    spec = get_boat(a.hidden_seed, a.stratum, a.boat_index)
    label = a.out or f'{a.stratum}{a.boat_index}-c{a.controller:02d}-s{a.seed}-{time.strftime("%H%M%S")}'
    if a.emit_config:
        scenario, lost = write_config(Path(a.emit_config), spec, 0, not a.no_match, not a.no_env)
        link = SimpleNamespace(port='<PORT>')
        print(' '.join(shlex.quote(x) for x in player_argv(Path(a.emit_config), '<PORT>', a.seed, a)))
        print('scenario', scenario, '\nnot representable in Unity:', lost)
        return
    if a.dry_run:
        start = tuple(float(v) for v in a.start.split(','))
        t0 = time.time()
        m, _, plant, log = dry_run(spec, a.controller, a.seed, start, a.t_max)
        ref, _ = mission.run(spec, [a.controller], [a.seed], t_max=a.t_max)
        print('adapter via TCP + fake player:', fmt(m))
        print('mission.run directly         :', fmt(ref))
        print(f'ticks {plant.tick}, final course-frame state {np.round(plant.st[0], 3).tolist()}, '
              f'dJ {abs(fmt(m)["J"] - fmt(ref)["J"]):.2e}, {time.time() - t0:.0f} s')
        return
    # live
    if not PLAYER.is_file():
        sys.exit(f'player not found: {PLAYER}')
    result, plant = live(a, spec, label)
    out = OUT_ROOT / label
    if plant is not None and plant.trace:
        tr = np.array(plant.trace)
        result['progress_final'] = progress_of(plant.st[0])
        result['t_final'] = float(plant.t)
        result['final_state'] = plant.st[0].tolist()
        np.savetxt(out / 'trace.csv', tr, delimiter=',', header='t,x,y,psi,u,v,r', comments='')
        result['trace_every_s'] = 25 * DT
        result['trace_samples'] = len(tr)
    ref, _ = mission.run(spec, [a.controller], [a.seed], t_max=a.t_max)
    result['numpy_hidden'] = fmt(ref)
    ref2, _ = mission.run(representable_numpy(spec), [a.controller], [a.seed], t_max=a.t_max)
    result['numpy_representable'] = fmt(ref2)
    (out / 'summary.json').write_text(json.dumps(result, indent=1))
    print(json.dumps(result, indent=1))
    return result


if __name__ == '__main__':
    main()
