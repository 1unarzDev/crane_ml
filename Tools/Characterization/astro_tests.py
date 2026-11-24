"""astro's test types (still, couple, speed, steps, nav2) on a simulated vessel: the same command programs
and the same result fields (names and units) as an astro result file, so a sim run and a field result compare
key by key.

    spec = Spec.load('state/tests.yaml')                 # astro's tests file: axes, spaces, tests, groups
    vessel = ProfileVessel(profile)                      # T1: the MarinePlanarModel twin; FirstOrderVessel: T0
    session = Session(spec, vessel)
    result = session.run('speed-surge', space='small')   # {test, type, layer, space, amp, summary, measured,
                                                         #  changes, holds, status: done}

The maths (response, speed_curve, speed_at, t95, step_metrics, nav2_limits) is astro's
(astro_dock tools/astro/engine/fit.py); the programs (baseline, settle, measure, coast per hold; levels x signs)
come from ``excitation.astro_holds``. A session applies each result's changes to its own state, as
``astro apply`` would, so ``nav2`` (and ``steps``' limits) read what the speed tests measured.

Units: speeds m/s (rad/s for the rotational axis), times s, efforts the normalised [-1, 1] of
``/cmd_vel_effort``. Efforts reach the thrusters through the linear mixer q = clip(M e) (the hardware's
for a single-axis command; it mixes in thrust for combined ones).
"""
from __future__ import annotations

import math
import statistics

import numpy as np

import excitation as X
import gate as G
import plant as P

TYPES = ('still', 'couple', 'speed', 'steps', 'nav2')
DEFAULT_LAYER = {'still': 'conditions', 'couple': 'decoupling', 'speed': 'speed curve',
                 'steps': 'velocity loop', 'nav2': 'nav2 limits'}
HOLD_HZ = 30.0
COAST_MAX_S, COAST_WINDOW_S, COAST_TOLERANCE = 8.0, 1.0, 0.03
RISE_LOW, RISE_HIGH = 0.1, 0.9
SETTLING_BAND, SETTLING_FLOOR, TAIL_FRACTION = 0.05, 0.02, 0.2


class Abort(Exception):
    pass


# ---- the tests file ----------------------------------------------------------------------------------

class Spec:
    """astro's tests file: ``axes``, ``rotational``, ``spaces``, ``tests``, ``groups``."""

    def __init__(self, data, path='tests.yaml'):
        self.path = path
        self.axes = list(data.get('axes') or G.AXES)
        self.rotational = data.get('rotational') or self.axes[-1]
        self.spaces = data.get('spaces') or {}
        self.tests = data.get('tests') or {}
        self.groups = data.get('groups') or {}
        for name, test in self.tests.items():
            if test.get('type') not in TYPES:
                raise ValueError(f'{path}: tests.{name}.type must be one of {", ".join(TYPES)}')

    @classmethod
    def load(cls, path):
        import yaml
        with open(path) as f:
            return cls(yaml.safe_load(f) or {}, str(path))

    def expand(self, name, seen=()):
        if name in self.tests:
            return [name]
        if name in self.groups:
            if name in seen:
                raise ValueError(f'group cycle at {name}')
            return [t for item in self.groups[name] for t in self.expand(item, (*seen, name))]
        raise KeyError(f'no test or group {name!r} in {self.path} (tests: {", ".join(self.tests)})')

    def layer(self, name):
        test = self.tests[name]
        return test.get('layer') or DEFAULT_LAYER[test['type']]

    def space(self, name):
        if name not in self.spaces:
            raise Abort(f'no space {name!r} in {self.path} (spaces: {", ".join(self.spaces)})')
        return self.spaces[name]


# ---- astro's maths (tools/astro/engine/fit.py) ----------------------------------------------------------

def response(baseline, hold):
    """Mean body velocity of ``hold`` minus the ``baseline`` drift, per body axis (drift removed in the world
    frame when rows carry yaw). Rows are (t, vx, vy, wz, yaw)."""
    if not baseline or not hold:
        raise ValueError(f'no velocity samples (baseline {len(baseline)}, hold {len(hold)})')
    wz = np.mean([r[3] for r in hold]) - np.mean([r[3] for r in baseline])
    if all(r[4] is not None for r in (*baseline, *hold)):
        def world(r):
            c, s = math.cos(r[4]), math.sin(r[4])
            return c * r[1] - s * r[2], s * r[1] + c * r[2]
        de, dn = np.mean([world(r) for r in baseline], axis=0)
        body = []
        for r in hold:
            e, n = world(r)
            c, s = math.cos(r[4]), math.sin(r[4])
            body.append((c * (e - de) + s * (n - dn), -s * (e - de) + c * (n - dn)))
        vx, vy = np.mean(body, axis=0)
    else:
        vx = np.mean([r[1] for r in hold]) - np.mean([r[1] for r in baseline])
        vy = np.mean([r[2] for r in hold]) - np.mean([r[2] for r in baseline])
    return [float(vx), float(vy), float(wz)]


def drift(rows):
    pts = [(math.cos(r[4]) * r[1] - math.sin(r[4]) * r[2], math.sin(r[4]) * r[1] + math.cos(r[4]) * r[2])
           for r in rows if r[4] is not None]
    if not pts:
        return None
    e, n = np.mean(pts, axis=0)
    return dict(east=float(e), north=float(n), speed=float(math.hypot(e, n)),
                heading_deg=float(math.degrees(math.atan2(n, e)) % 360))


def cross_response(delta, axis, sign):
    main = max(sign * delta[axis], 1e-6)
    return main, tuple(delta[k] / main for k in range(len(delta)) if k != axis)


def speed_curve(points):
    """Fit e^2 = d1 v + d2 v|v| to (speed, effort) points; both terms >= 0 (a negative one is dropped)."""
    points = [(v, e) for v, e in points if math.isfinite(v) and math.isfinite(e) and v > 0 and e > 0]
    if not points:
        return None
    v, e = (np.array(c, dtype=float) for c in zip(*points))
    e = e * e
    basis = (v, v * v)
    d1, d2 = np.linalg.lstsq(np.column_stack(basis), e, rcond=None)[0] \
        if len(points) > 1 and len(set(v)) > 1 else (0.0, -1.0)
    if d2 < 0 or d1 < 0:
        index = 1 if d1 < 0 and d2 >= 0 else 0
        coefficient = float(basis[index] @ e / (basis[index] @ basis[index]))
        d1, d2 = (coefficient, 0.0) if index == 0 else (0.0, coefficient)
    return dict(d1=float(d1), d2=float(d2), max_speed=float(v.max()),
                points=[[float(a), float(b)] for a, b in sorted(points)])


def speed_at(d1, d2, effort):
    magnitude = effort * effort
    if magnitude == 0:
        return 0.0
    root = math.sqrt(max(0.0, d1 * d1 + 4 * d2 * magnitude))
    return 2 * magnitude / (d1 + root) if d1 + root > 0 else 0.0


def t95(t, y, t_on, t_cut):
    """Seconds after ``t_on`` until ``y`` first reaches 95 % of its change from the mean before ``t_on`` to the
    mean over the hold's last third; None if never."""
    t, y = np.asarray(t, float), np.asarray(y, float)
    before = y[t < t_on]
    base = float(before.mean()) if before.size else 0.0
    hold = (t >= t_on) & (t <= t_cut)
    tail = hold & (t >= t_cut - (t_cut - t_on) / 3)
    if not tail.any():
        return None
    steady = float(y[tail].mean())
    if steady == base:
        return None
    reached = np.nonzero(hold & ((y - base) / (steady - base) >= 0.95))[0]
    return float(t[reached[0]] - t_on) if reached.size else None


def step_metrics(samples, target, start, end):
    window = [(t - start, v) for t, v in samples if start <= t <= end and v is not None]
    result = dict(target=target, samples=len(window), verdict='INCONCLUSIVE')
    if len(window) < 5 or target == 0:
        return result
    times = np.array([t for t, _ in window])
    sign, reference = np.sign(target), abs(target)
    values = sign * np.array([v for _, v in window])

    def crossing(level):
        above = np.nonzero(values >= level)[0]
        return float(times[above[0]]) if above.size else None
    low, high = crossing(RISE_LOW * reference), crossing(RISE_HIGH * reference)
    tolerance = max(SETTLING_BAND * reference, SETTLING_FLOOR)
    outside = np.nonzero(np.abs(values - reference) > tolerance)[0]
    if not outside.size:
        settling = 0.0
    elif outside[-1] + 1 < len(times):
        settling = float(times[outside[-1] + 1])
    else:
        settling = None
    tail = values[int(len(values) * (1 - TAIL_FRACTION)):]
    error = np.abs(reference - values)
    result.update(rise_time=high - low if low is not None and high is not None else None,
                  overshoot=max(0.0, float(values.max() - reference) / reference),
                  settling_time=settling, steady_state_error=float(reference - tail.mean()),
                  iae=float(np.sum((error[1:] + error[:-1]) / 2 * np.diff(times))),
                  verdict='SETTLED' if settling is not None else 'NOT SETTLED')
    return result


def nav2_limits(velocity, acceleration, t95s, rules, axes, rotational):
    k, delay = float(rules['ramp_lags']), float(rules['delay_s'])
    lag = {a: float(t95s[a]) / 3.0 + delay for a in axes}
    cap_v = {a: (float(rules['yaw_tolerance']) / (2 * lag[a]) if a == rotational
                 else float(rules['stop_distance_m']) / (lag[a] * (k / 2 + 1))) for a in axes}
    cap_a = {a: cap_v[a] / (k * lag[a]) for a in axes}
    return dict(velocity=[round(min(float(velocity[a]), cap_v[a]), 4) for a in axes],
                acceleration=[round(min(float(acceleration[a]), cap_a[a]), 4) for a in axes])


def coasted(rows):
    if len(rows) < 4:
        return True
    middle = rows[0][0] + (rows[-1][0] - rows[0][0]) / 2
    first, last = [r for r in rows if r[0] < middle], [r for r in rows if r[0] >= middle]
    if not first or not last:
        return True
    return all(abs(sum(r[k] for r in last) / len(last) - sum(r[k] for r in first) / len(first)) < COAST_TOLERANCE
               for k in (1, 2, 3))


def fmt(value):
    if isinstance(value, float):
        return f'{value + 0.0:.4g}'
    if isinstance(value, (list, tuple)):
        return '[' + ', '.join(fmt(v) for v in value) + ']'
    return str(value)


def get(state, key):
    node = state
    for part in key.split('.'):
        if not isinstance(node, dict) or part not in node:
            return None
        node = node[part]
    return node


def put(state, key, value):
    *parents, leaf = key.split('.')
    node = state
    for part in parents:
        node = node.setdefault(part, {})
    node[leaf] = value


# ---- simulated vessels ---------------------------------------------------------------------------------

def mixer_rows(profile, matrix=None):
    """Mixer rows in the profile's thruster order (canonical FL, FR, RR, RL rows matched by name)."""
    flat = [float(v) for row in G.mixer_matrix(matrix) for v in row]
    rows = {name: flat[3 * i:3 * i + 3] for i, name in enumerate(G.MIXER_ROWS)}
    return [rows[t['name']] for t in profile['thrusters']]


class Vessel:
    """The sim side of astro's runner: ``hold(command, seconds)`` and ``window(t0, t1)``, rows
    (t, vx, vy, wz, yaw, x, y) on the sim clock."""

    tier = ''
    kind = ''

    def __init__(self, profile, dt):
        self.profile, self.dt, self.rows, self.t = profile, float(dt), [], 0.0

    def now(self):
        return self.t

    def hold(self, command, seconds, mode='effort'):
        t0 = self.t
        for _ in range(int(round(seconds / self.dt))):
            self.tick(command, mode)
            self.t += self.dt
            self.rows.append((self.t, *self.body(), *self.pose()))
        return t0, self.t

    def window(self, t0, t1):
        return [r[:5] for r in self.rows if t0 <= r[0] <= t1]

    def envelope(self, axis):
        return float((self.profile.get('envelope') or {}).get(axis) or 1.0)


class ProfileVessel(Vessel):
    """T1: ``plant.ProfilePlant`` (the exact twin of Unity's MarinePlanarModel) from a profile. Efforts go
    through the linear mixer to PWM; a velocity command (steps) through a feedforward + P loop on the
    profile's own damping and effective mass with a pseudo-inverse allocation."""

    tier, kind = 'T1', 'dynamic'

    def __init__(self, profile, dt=None, matrix=None, current=(0.0, 0.0), kp=(2.0, 2.0, 2.0)):
        super().__init__(profile, dt or profile['fixedDeltaTime'])
        self.plant = P.ProfilePlant(profile, self.dt, current)
        self.mix = np.array(mixer_rows(profile, matrix))
        cols = []
        for t in profile['thrusters']:
            a = math.radians(t['alphaDeg'])
            cols.append((math.cos(a), math.sin(a), t['x'] * math.sin(a) - t['y'] * math.cos(a)))
        self.Bpinv = np.linalg.pinv(np.array(cols).T)
        am = profile['addedMass']
        self.meff = (profile['mass'] + am['a11'], profile['mass'] + am['a22'], profile['izz'] + am['a66'])
        self.kp = tuple(float(k) for k in kp)

    def body(self):
        return self.plant.body_velocity()

    def pose(self):
        x, y, psi = self.plant.state[:3]
        return (psi, x, y)

    def pwm_effort(self, effort):
        q = np.clip(self.mix @ np.clip(np.asarray(effort, float), -1.0, 1.0), -1.0, 1.0)
        return [1500.0 + 400.0 * float(v) for v in q]

    def pwm_velocity(self, target):
        nu = self.plant.body_velocity()
        tau = []
        for j, axis in enumerate(G.AXES):
            c, w = self.profile[axis], target[j]
            d1, d2 = (c['d1Pos'], c['d2Pos']) if w >= 0 else (c['d1Neg'], c['d2Neg'])
            tau.append((d1 + d2 * abs(w)) * w + self.kp[j] * self.meff[j] * (w - nu[j]))
        f = self.Bpinv @ np.array(tau)
        th = self.profile['thrusters']
        caps = [t['kForward'] if fi >= 0 else t['kReverse'] for t, fi in zip(th, f)]
        scale = max(1.0, max(abs(fi) / cap for fi, cap in zip(f, caps)))
        return [1500.0 + 400.0 * math.copysign(math.sqrt(abs(fi) / scale / cap), fi) for fi, cap in zip(f, caps)]

    def tick(self, command, mode):
        self.plant.step_pwm(self.pwm_effort(command) if mode == 'effort' else self.pwm_velocity(command))


class FirstOrderVessel(Vessel):
    """T0: no forces. Per body axis a first-order lag toward the steady speed the profile gives an effort
    (e^2 k = D1 v + D2 v|v|, k the axis gain of the mixer and thrusters), with the time constant of the
    damping linearised there (tau = M_eff / (D1 + 2 D2 |v|)); a velocity command is tracked with the same
    lag. Heading and position integrate the body velocity."""

    tier, kind = 'T0', 'kinematic'

    def __init__(self, profile, dt=0.02, matrix=None, current=(0.0, 0.0)):
        super().__init__(profile, dt)
        hw = {'thruster_matrix': [list(r) for r in (matrix or G.DEFAULT_MATRIX)]} if matrix else None
        self.gains = G.axis_gains(profile, hw)
        am = profile['addedMass']
        self.meff = (profile['mass'] + am['a11'], profile['mass'] + am['a22'], profile['izz'] + am['a66'])
        self.v = [0.0, 0.0, 0.0]
        self.psi = self.x = self.y = 0.0
        self.current = tuple(current)

    def _damping(self, j, sign):
        c = self.profile[G.AXES[j]]
        return (c['d1Pos'], c['d2Pos']) if sign >= 0 else (c['d1Neg'], c['d2Neg'])

    def steady(self, j, effort):
        if effort == 0:
            return 0.0
        sign = 1 if effort > 0 else -1
        d1, d2 = self._damping(j, sign)
        k = self.gains[G.AXES[j] + ('+' if sign > 0 else '-')]['k']
        return sign * speed_at(d1 / k, d2 / k, min(abs(effort), 1.0))

    def tick(self, command, mode):
        for j in range(3):
            want = self.steady(j, command[j]) if mode == 'effort' else float(command[j])
            ref = want if want else self.v[j]
            d1, d2 = self._damping(j, 1 if ref >= 0 else -1)
            tau = self.meff[j] / max(d1 + 2 * d2 * abs(ref), 1e-9)
            self.v[j] += (want - self.v[j]) * (1.0 - math.exp(-self.dt / tau))
        c, s = math.cos(self.psi), math.sin(self.psi)
        self.x += self.dt * (c * self.v[0] - s * self.v[1] + self.current[0])
        self.y += self.dt * (s * self.v[0] + c * self.v[1] + self.current[1])
        self.psi += self.dt * self.v[2]

    def body(self):
        c, s = math.cos(self.psi), math.sin(self.psi)
        cx, cy = self.current
        return (self.v[0] + c * cx + s * cy, self.v[1] - s * cx + c * cy, self.v[2])

    def pose(self):
        return (self.psi, self.x, self.y)


VESSELS = {'T0': FirstOrderVessel, 'T1': ProfileVessel}


# ---- the procedures ------------------------------------------------------------------------------------

class Session:
    """One vessel and its state, tests run in order (astro test + astro apply)."""

    def __init__(self, spec, vessel, state=None, log=None):
        self.spec, self.vessel = spec, vessel
        self.state = state if state is not None else {}
        self.events = []          # {t, kind: log, log_kind, text} on the sim clock
        self.holds = []
        self._log = log or (lambda line: None)

    def log(self, kind, message):
        self.events.append({'t': round(self.vessel.now(), 4), 'kind': 'log', 'log_kind': kind,
                            'text': f'{kind:<7} {message}'})
        self._log(f'{kind:<7} {message}')

    def record(self, main, command, settle, end, rows):
        kept, last = [], None
        for r in rows:
            if last is None or r[0] - last >= 1.0 / HOLD_HZ - 1e-6:
                kept.append(r)
                last = r[0]
        axis, sign, level = main or (None, None, None)
        self.holds.append(dict(
            command=dict(axis=axis, sign=sign, level=level, efforts=[round(float(v) + 0.0, 4) for v in command]),
            start=round(settle, 3), end=round(end, 3),
            samples={k: [round(float(r[i]), 4) for r in kept] for i, k in enumerate(('t', 'vx', 'vy', 'wz', 'yaw'))}))

    def _hold(self, n, command, baseline_s, settle_s, measure_s, main):
        v = self.vessel
        start = v.now()
        b0, b1 = v.hold([0.0] * n, baseline_s)
        while b1 - start < COAST_MAX_S and not coasted(v.window(b1 - 2 * COAST_WINDOW_S, b1)):
            _, b1 = v.hold([0.0] * n, 0.5)
        b0 = b1 - baseline_s
        t_on, _ = v.hold(command, settle_s)
        m0, t_cut = v.hold(command, measure_s)
        self.record(main, command, b0, t_cut, v.window(b0, t_cut))
        self.holds[-1]['start'] = round(t_on, 3)
        return v.window(b0, b1), v.window(m0, t_cut), v.window(b0, t_cut), t_on, t_cut

    def run(self, name, space='large', amp=None):
        spec, test = self.spec, self.spec.tests[name]
        kind = test['type']
        room = spec.space(space) if kind in ('couple', 'speed', 'steps') else {}
        amplitude = float(amp if amp is not None else room.get('amp', 0.5)) if room else None
        base = dict(test=name, type=kind, layer=spec.layer(name), space=space if room else None, amp=amplitude,
                    tier=self.vessel.tier)
        self.holds = []
        self.log('test', f'start {name}' + (f'  space={space} amp={amplitude:g}' if room else ''))
        outcome = getattr(self, kind)(test, room, amplitude)
        result = dict(base, status='done', **outcome)
        if self.holds:
            result['holds'] = self.holds
        for change in result.get('changes') or []:
            if 'column' not in change:
                put(self.state, change['key'], change['value'])
        self.log('test', f'done  {name}  {result["summary"]}')
        return result

    def _program(self, kind, j, n, levels, signs, test):
        holds = X.astro_holds(kind, j, n, levels, signs, test)
        for h in holds:
            h['main'] = (self.spec.axes[j], *h['main'][1:])
        return holds

    # still -------------------------------------------------------------------------------------
    def still(self, test, room, amplitude):
        seconds = float(test.get('seconds', 20))
        t0, t1 = self.vessel.hold([0.0] * len(self.spec.axes), seconds)
        rows = self.vessel.window(t0, t1)
        sigma = [round(statistics.pstdev(r[k] for r in rows), 4) for k in (1, 2, 3)]
        d = drift(rows)
        summary = f'noise sigma={fmt(sigma)}'
        if d:
            summary = f'drift={d["speed"]:.2f}m/s@{d["heading_deg"]:.0f}deg ' + summary
        return dict(summary=summary, measured=dict(drift=d, sigma=dict(zip(self.spec.axes, sigma)),
                                                   samples=len(rows)))

    # couple: the first hold of each sign (the current column), no search -------------------------
    def couple(self, test, room, amplitude):
        axes = self.spec.axes
        j, n = axes.index(test['axis']), len(axes)
        signs = (1, -1) if (test.get('apply') or {}).get('signed') else (1,)
        measured, parts = [], []
        for prog in self._program('couple', j, n, [amplitude], signs, test):
            before, during, _, _, _ = self._hold(n, prog['command'], prog['baseline_s'], prog['settle_s'],
                                                 prog['measure_s'], prog['main'])
            delta = response(before, during)
            sign = prog['main'][1]
            main, ratios = cross_response(delta, j, sign)
            measured.append(dict(sign=sign, column=[0.0] * (n - 1), response=[round(v, 4) for v in delta],
                                 cross=[round(r, 4) for r in ratios]))
            parts.append(f'{axes[j]}{"+" if sign > 0 else "-"} cross={fmt([round(r, 4) for r in ratios])} sim')
        return dict(summary=' | '.join(parts), measured=dict(holds=measured), changes=[])

    # speed -----------------------------------------------------------------------------------------
    def speed(self, test, room, amplitude):
        axes, target = self.spec.axes, test.get('apply') or {}
        j, n, axis = axes.index(test['axis']), len(axes), test['axis']
        levels = X.astro_levels(room, amplitude)
        points, rises = {1: [], -1: []}, []
        for prog in self._program('speed', j, n, levels, (1, -1), test):
            self.vessel.hold([0.0] * n, max(0.0, prog['coast_s'] - prog['baseline_s']))
            _, sign, level = prog['main']
            before, during, rows, t_on, t_cut = self._hold(n, prog['command'], prog['baseline_s'], prog['settle_s'],
                                                           prog['measure_s'], prog['main'])
            delta = response(before, during)
            points[sign].append((sign * delta[j], level))
            rise = t95([r[0] for r in rows], [sign * r[1 + j] for r in rows], t_on, t_cut)
            if rise is not None:
                rises.append(rise)
        self.vessel.hold([0.0] * n, float(test.get('coast_s', 4)))
        prefix = target.get('key', 'characterization')
        curves = {}
        for sign in (1, -1):
            name = f'{axis}{"+" if sign > 0 else "-"}'
            curve = speed_curve(points[sign])
            if not curve:
                raise Abort(f'{name}: no level moved the vessel forward along the axis ({points[sign]})')
            curves[name] = curve
        rise = round(statistics.median(rises), 3) if rises else None
        velocity = round(min(speed_at(c['d1'], c['d2'], 1.0) for c in curves.values()), 4)
        changes = [dict(key=f'{prefix}.speed_curves.{name}', value=dict(d1=round(c['d1'], 5), d2=round(c['d2'], 5)))
                   for name, c in curves.items()]
        changes.append(dict(key=f'{prefix}.motion_limits.velocity.{axis}', value=velocity))
        if rise:
            changes.append(dict(key=f'{prefix}.t95.{axis}', value=rise))
            changes.append(dict(key=f'{prefix}.motion_limits.acceleration.{axis}', value=round(velocity * 3 / rise, 4)))
        if target.get('version') is not None:
            changes.append(dict(key=f'{prefix}.version', value=target['version']))
        parts = [f'{name} d1={c["d1"]:.3g} d2={c["d2"]:.3g}' for name, c in curves.items()]
        summary = ' | '.join(parts) + (f' t95={rise:.2f}s' if rise else '') + f' vmax={velocity:.3g}'
        return dict(summary=summary, changes=changes,
                    measured=dict(points={k: [list(p) for p in v] for k, v in (('+', points[1]), ('-', points[-1]))},
                                  t95=rises))

    # steps -----------------------------------------------------------------------------------------
    def steps(self, test, room, amplitude):
        axes = self.spec.axes
        j, n, axis = axes.index(test['axis']), len(axes), test['axis']
        limit_key = test.get('limits')
        limit = get(self.state, f'{limit_key}.{axis}') if limit_key else None
        limit = limit or self.vessel.envelope(axis)
        target_v = float(test.get('fraction', 0.5)) * float(limit) * (amplitude / float(room.get('amp', amplitude)))
        metrics, parts = {}, []
        for prog in self._program('steps', j, n, [target_v], (1, -1), test):
            self.vessel.hold([0.0] * n, prog['baseline_s'])
            t_on, t_off = self.vessel.hold(prog['command'], prog['settle_s'], mode='velocity')
            self.vessel.hold([0.0] * n, prog['coast_s'], mode='velocity')
            self.record(prog['main'], prog['command'], t_on, t_off, self.vessel.window(t_on, t_off))
            sign = prog['main'][1]
            samples = [(r[0], r[1 + j]) for r in self.vessel.window(t_on, t_off)]
            m = step_metrics(samples, sign * target_v, t_on, t_off)
            name = f'{axis}{"+" if sign > 0 else "-"}'
            metrics[name] = {k: (round(v, 4) if isinstance(v, float) else v) for k, v in m.items()}
            parts.append(f'{name} rise={fmt(m.get("rise_time"))} over={fmt(m.get("overshoot"))} '
                         f'settle={fmt(m.get("settling_time"))} sse={fmt(m.get("steady_state_error"))}')
        return dict(summary=' | '.join(parts), measured=metrics)

    # nav2 -------------------------------------------------------------------------------------------
    def nav2(self, test, room, amplitude):
        axes, source, target = self.spec.axes, test.get('from') or {}, test.get('apply') or {}
        values = {}
        for part in ('velocity', 'acceleration', 't95'):
            key = source.get(part)
            values[part] = get(self.state, key) if key else None
            missing = [a for a in axes if not isinstance(values[part], dict) or values[part].get(a) is None]
            if missing:
                raise Abort(f'no {key}.{"/".join(missing)} in the state: run the speed tests first '
                            f'(--astro-test all runs the ladder)')
        limits = nav2_limits(values['velocity'], values['acceleration'], values['t95'], test['rules'], axes,
                             self.spec.rotational)
        summary = f'velocity={fmt(limits["velocity"])} acceleration={fmt(limits["acceleration"])}'
        return dict(summary=summary, changes=[dict(key=target.get('key'), value=limits)])
