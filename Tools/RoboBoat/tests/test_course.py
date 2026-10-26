"""Course truth, validator and agent-tool tests (design §7), stdlib unittest:
    python3 -m unittest discover -s Tools/RoboBoat/tests

Optional, set by the Codex campaigns:
    CRANE_COURSE_FUZZ_DIR        every truth in it must pass validate.py
    CRANE_COURSE_CSHARP_VERDICTS CourseCli.Validate output; Python must agree on every row of
                                 every truth in CRANE_COURSE_VERDICT_DIR
"""

import copy
import json
import math
import os
import sys
import unittest
from pathlib import Path

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE.parent))
sys.path.insert(0, str(HERE))
import check_truth_subscribers  # noqa: E402
import course_truth  # noqa: E402
import make_bad as mb  # noqa: E402
import validate  # noqa: E402

SPEC = json.loads((HERE.parent / 'constraints' / 'roboboat_2026.json').read_text())
BASE = json.loads((HERE / 'fixtures' / 'roboboat_2026-s1.json').read_text())
TESTING = Path('/home/lunarz/astro_dock/src/mhseals_testing')


def failed_rows(truth):
    return {r['id'] for r in validate.evaluate(SPEC, truth) if not r['pass']}


def row_value(truth, row):
    return next(r['value'] for r in validate.evaluate(SPEC, truth) if r['id'] == row)


BASE_VALUE = {r['id']: r['value'] for r in validate.evaluate(SPEC, BASE)}


# ---- one mutation per metric row -----------------------------------------------------------
# Each mutation moves one object (or one rigid group) of BASE so that the row's value becomes x;
# LIMITS lists x values just inside (True: every row passes) and just past (False: only this
# row fails) the row's limit and tolerance.

def _black_gap(t, x):  # two field blacks closer together: surface gap x
    a, b = mb.el(t, 'debris_clearance.black3'), mb.el(t, 'debris_clearance.black4')
    d = math.dist(mb.xy(a), mb.xy(b))
    k = (x + a['radius'] + b['radius']) / d
    b['x'], b['y'] = a['x'] + (b['x'] - a['x']) * k, a['y'] + (b['y'] - a['y']) * k


def _hull_off_start(t, x):  # the free hull x clear of the start
    h = mb.el(t, 'supply_drop.water1')
    s = mb.anchor(t, 'entry_gates.start')
    h.update(x=s['x'] - x - h['radius'], y=s['y'])


def _separation(t, x):  # entry gate 2 slides toward gate 1 along the line between them
    g1, g2 = mb.gate(t, 'entry_gates.gate1'), mb.gate(t, 'entry_gates.gate2')
    d = math.dist(mb.xy(g1), mb.xy(g2))
    k = x / d - 1
    mb.move_gate(t, g2['id'], (g2['x'] - g1['x']) * k, (g2['y'] - g1['y']) * k)


def _standoff(t, x):  # the start anchor x along gate 1's normal
    g = mb.gate(t, 'entry_gates.gate1')
    f = mb.unit(g['normal_yaw'])
    mb.anchor(t, 'entry_gates.start').update(x=g['x'] + f[0] * x, y=g['y'] + f[1] * x)


def _shift_along_gate2(t, x, row, move):
    f = mb.unit(mb.gate(t, 'entry_gates.gate2')['normal_yaw'])
    d = x - BASE_VALUE[row]
    move(t, f[0] * d, f[1] * d)


def _bend(t, x):  # channel gate 3 turns until the gate 2 -> 3 curvature is x
    (c2, y2), (c3, y3) = (validate.gate_pose(t, 'gate:debris_clearance.' + g) for g in ('gate2', 'gate3'))
    turn = validate.wrap(y3 - y2)
    want = 2 * math.asin(x * math.dist(c2, c3) / 2)
    mb.rotate_gate(t, 'debris_clearance.gate3', math.degrees(math.copysign(want - abs(turn), turn)))


def _field_beyond(t, x):  # the field object farthest off the channel axis comes to x past the last gate
    g = 'debris_clearance.gate3'
    e = max((e for e in t['elements'] if e['task'] == 'debris_clearance' and e['role'] in ('debris', 'beacon')),
            key=lambda e: abs(mb.gate_frame(t, g, mb.xy(e))[1]))
    mb.move_el(t, e['id'], *mb.place_in_gate_frame(t, g, e['id'], along=x))


def _mark(t, along=None, lateral=None):  # the turn mark with harbor zone 1 and the mark anchor on it
    dx, dy = mb.place_in_gate_frame(t, 'speed_challenge.gate', 'speed_challenge.mark', along, lateral)
    mb.transform(t, ids={'speed_challenge.mark'}, anchors=('speed_challenge.mark', 'harbor_alert.zone1'), dx=dx, dy=dy)


def _marker_ratio(t, x):  # the marker at x of the way from the gate to the mark, 3 m to starboard (clear of mark and posts)
    g = 'speed_challenge.gate'
    a, _ = mb.gate_frame(t, g, mb.xy(mb.el(t, 'speed_challenge.mark')))
    mb.move_el(t, 'speed_challenge.marker', *mb.place_in_gate_frame(t, g, 'speed_challenge.marker', x * a, -3.0))


def _sign_behind_gate(t, x):  # the light sign on the gate axis, x clear of the gate line
    r = mb.el(t, 'speed_challenge.sign')['radius']
    mb.move_el(t, 'speed_challenge.sign', *mb.place_in_gate_frame(t, 'speed_challenge.gate', 'speed_challenge.sign', -x - r, 0.0))


def _hull_by_gate1(t, x):  # the free hull in front of entry gate 1, x clear of its gate line
    g = mb.gate(t, 'entry_gates.gate1')
    h = mb.el(t, 'supply_drop.water1')
    f = mb.unit(g['normal_yaw'])
    h.update(x=g['x'] + f[0] * (x + h['radius']), y=g['y'] + f[1] * (x + h['radius']))


def _hull_away(t, x):  # the free hull x from its nearest object of another task, straight away from it
    h, near = mb.el(t, 'supply_drop.water1'), mb.el(t, 'debris_clearance.black8')
    d = math.dist(mb.xy(h), mb.xy(near))
    h.update(x=near['x'] + (h['x'] - near['x']) * x / d, y=near['y'] + (h['y'] - near['y']) * x / d)


def _hull_pair(t, x):  # two more free hulls x apart, one by harbor zone 2 and one by the marina
    mb.add_hull(t, 'supply_drop.ball2', 4.75, 23.5)
    mb.add_hull(t, 'supply_drop.water2', 4.75 + x * math.cos(-1.2), 23.5 + x * math.sin(-1.2), kind='roboboat/hull_water_orange')


def _tip_gap(t, x):  # the N pier slides toward the S pier
    a, b = mb.anchor(t, 'marina.tips_N'), mb.anchor(t, 'marina.tips_S')
    d = math.dist(mb.xy(a), mb.xy(b))
    mb.pier_group(t, 'N', dx=(b['x'] - a['x']) * (1 - x / d), dy=(b['y'] - a['y']) * (1 - x / d))


def _slip_vessel(t, x):  # a red slip at the fig 60 minimum width, vessel beam x
    mb.bay(t, 'marina.S3')['width'] = 1.38
    t['vessel']['beam'] = x


def _mouth(t, x):
    # The mouth anchor (and zone 2, which must stay 7-12 m off it) comes toward the entry lane.
    # Moving the marina itself that far also breaks G3/G4/P5 first, so only the anchor moves.
    d = x - BASE_VALUE['T5.mouth_clear']
    mb.anchor(t, 'marina.outer_end')['y'] += d
    b = mb.el(t, 'harbor_alert.zone2_buoy')
    mb.zone2_group(t, b['x'], b['y'] + d)


def _zone2_radial(t, x):  # zone 2 x off the marina mouth, on its current bearing
    b, m = mb.el(t, 'harbor_alert.zone2_buoy'), mb.anchor(t, 'marina.outer_end')
    d = math.dist(mb.xy(b), mb.xy(m))
    mb.zone2_group(t, m['x'] + (b['x'] - m['x']) * x / d, m['y'] + (b['y'] - m['y']) * x / d)


def _horn(t, key, x):
    h = mb.task(t, 'harbor_alert')['semantics']['horn']
    (h['program'] if key == 'gap_s' else h)[key] = x


IN, OUT = True, False
LIMITS = {
    # row: (mutate(truth, x), [(x, passes)])
    'G1.inside': (lambda t, x: mb.el(t, 'debris_clearance.black8').update(y=-65.0 + 0.102 + x), [(0.01, IN), (-0.01, OUT)]),
    'G2.clearance': (_black_gap, [(1.01, IN), (0.99, OUT)]),
    'G3.start_clear': (_hull_off_start, [(1.51, IN), (1.49, OUT)]),
    'T1.cc': (lambda t, x: mb.set_cc(t, 'entry_gates.gate1', x), [(3.05, IN), (3.06, OUT), (1.83, IN), (1.82, OUT)]),
    'T1.clear': (lambda t, x: mb.set_clear(t, 'entry_gates.gate1', x), [(1.20, IN), (1.19, OUT)]),
    'T1.separation': (_separation, [(7.625, IN), (7.61, OUT)]),
    'T1.standoff': (_standoff, [(-1.835, IN), (-1.825, OUT)]),
    'T1.gate2_yaw': (lambda t, x: mb.rotate_gate(t, 'entry_gates.gate1', BASE_VALUE['T1.gate2_yaw'] - x), [(14.99, IN), (15.03, OUT)]),
    'T2.cc': (lambda t, x: mb.set_cc(t, 'debris_clearance.gate2', x), [(3.05, IN), (3.06, OUT)]),
    'T2.after_entry': (lambda t, x: _shift_along_gate2(t, x, 'T2.after_entry', lambda t, dx, dy: mb.move_gate(t, 'debris_clearance.gate1', dx, dy)),
                       [(2.91, IN), (2.89, OUT)]),
    'T2.heading': (lambda t, x: mb.rotate_gate(t, 'entry_gates.gate2', x - BASE_VALUE['T2.heading']), [(14.99, IN), (15.03, OUT)]),
    'T2.bend': (_bend, [(0.0400, IN), (0.0410, OUT)]),
    'T2.field_beyond': (_field_beyond, [(1.01, IN), (0.99, OUT)]),
    'T3.cc': (lambda t, x: mb.set_cc(t, 'speed_challenge.gate', x), [(3.05, IN), (3.06, OUT)]),
    'T3.leg': (lambda t, x: _mark(t, along=x), [(12.195, IN), (12.18, OUT)]),
    'T3.lateral': (lambda t, x: _mark(t, lateral=math.copysign(x, mb.gate_frame(t, 'speed_challenge.gate', mb.xy(mb.el(t, 'speed_challenge.mark')))[1])),
                   [(2.995, IN), (3.01, OUT)]),
    'T3.marker_between': (_marker_ratio, [(0.99, IN), (1.01, OUT), (0.01, IN), (-0.01, OUT)]),
    'T3.beyond_gate2': (lambda t, x: _shift_along_gate2(t, x, 'T3.beyond_gate2', lambda t, dx, dy: mb.speed_group(t, dx=dx, dy=dy)),
                        [(3.01, IN), (2.99, OUT)]),
    'T3.approach_exit': (lambda t, x: mb.speed_group(t, deg=BASE_VALUE['T3.approach_exit'] - x), [(59.99, IN), (60.03, OUT)]),
    'T3.approach_gate2': (lambda t, x: mb.speed_group(t, deg=x - BASE_VALUE['T3.approach_gate2']), [(59.99, IN), (60.03, OUT)]),
    'T3.gate_free': (_sign_behind_gate, [(1.01, IN), (0.99, OUT)]),
    'T4.gates': (_hull_by_gate1, [(3.01, IN), (2.99, OUT)]),
    'T4.near_task': (_hull_away, [(24.99, IN), (25.01, OUT)]),
    'T4.spacing': (_hull_pair, [(8.01, IN), (7.99, OUT)]),
    'T5.antiparallel': (lambda t, x: mb.pier_group(t, 'N', deg=BASE_VALUE['T5.antiparallel'] - x), [(9.99, IN), (10.01, OUT)]),
    'T5.tip_gap': (_tip_gap, [(3.01, IN), (2.99, OUT)]),
    'T5.slip_clear': (lambda t, x: mb.bay(t, 'marina.S3').update(width=x), [(1.67, IN), (1.68, OUT), (1.37, IN), (1.36, OUT)]),
    'T5.slip_vessel': (_slip_vessel, [(1.075, IN), (1.085, OUT)]),
    # the shore-side limit (0) is also the course boundary (G1.inside), so only its inside case is here
    'T5.shore': (lambda t, x: mb.marina_group(t, dx=x - BASE_VALUE['T5.shore']), [(1.995, IN), (2.01, OUT), (0.005, IN)]),
    'T5.mouth_clear': (_mouth, [(11.01, IN), (10.99, OUT)]),
    'T6.zone1': (lambda t, x: mb.anchor(t, 'harbor_alert.zone1').update(x=mb.el(t, 'speed_challenge.mark')['x'] + x), [(1.99, IN), (2.01, OUT)]),
    'T6.zone2': (_zone2_radial, [(11.99, IN), (12.01, OUT), (7.01, IN), (6.99, OUT)]),
    'T6.zone2_lane': (lambda t, x: mb.zone2_group(t, 10.0, x), [(15.01, IN), (14.99, OUT)]),
    'T6.freq': (lambda t, x: _horn(t, 'freq_hz', x), [(1050.0, IN), (1050.1, OUT), (600.0, IN), (599.9, OUT)]),
    'T6.gap': (lambda t, x: _horn(t, 'gap_s', x), [(2.0, IN), (2.1, OUT), (1.9, OUT)]),
}

# Rows that cannot fail alone: (mutate, x inside, x past, the row that always fails with it).
# A gate (or slip) below beam + 0.3 clear leaves less than 2 x (beam/2 + margin/2) = beam + 0.3
# of free water, the same width the path check needs, so the path row trips at the same limit.
COUPLED = {
    'T2.clear': (lambda t, x: mb.set_clear(t, 'debris_clearance.gate2', x), 1.20, 1.19, 'P1.channel'),
    'T3.clear': (lambda t, x: mb.set_clear(t, 'speed_challenge.gate', x), 1.20, 1.19, 'P2.loop'),
}

# Metric rows no mutation can fail.
EXEMPT = {
    # axial angle_between folds to [0, 90] deg, so 'max 90' always holds
    'T5.yaw': 'axial angle never exceeds 90 deg',
    # the start is the first vertex of the entry lane polyline, so zone2_lane >= 15 implies this
    'T6.zone2_start': 'implied by T6.zone2_lane',
}

METRIC_PREDICATES = {'distance', 'gate_cc', 'gate_clear', 'along', 'lateral', 'angle_between', 'inside', 'clearance',
                     'facing', 'curvature', 'nearest', 'polyline_distance', 'value'}


def mismatches(got, want, where=''):
    """Where explain's output differs from expected.json: dict keys in ``want`` must match, lists
    item by item, and {'approx': v, 'tol': t} accepts a number within t of v."""
    if isinstance(want, dict) and want.keys() == {'approx', 'tol'}:
        ok = isinstance(got, (int, float)) and not isinstance(got, bool) and abs(got - want['approx']) <= want['tol']
        return [] if ok else [f'{where}: {got!r} not within {want["tol"]} of {want["approx"]}']
    if isinstance(want, dict):
        if not isinstance(got, dict):
            return [f'{where}: {got!r} is not an object']
        return [m for k, v in want.items() for m in mismatches(got.get(k), v, f'{where}.{k}')]
    if isinstance(want, list):
        if not isinstance(got, (list, tuple)) or len(got) != len(want):
            return [f'{where}: {got!r} != {want!r}']
        return [m for i, (g, w) in enumerate(zip(got, want)) for m in mismatches(g, w, f'{where}[{i}]')]
    return [] if got == want else [f'{where}: {got!r} != {want!r}']


class Validator(unittest.TestCase):
    def test_base_course_passes_every_row(self):
        self.assertEqual(failed_rows(BASE), set())
        self.assertEqual({r['id'] for r in BASE['constraints']}, {r['id'] for r in SPEC['rows']})

    def test_bad_layouts_fail_exactly_their_one_row(self):
        files = sorted((HERE / 'bad').glob('*.json'))
        self.assertGreaterEqual(len(files), 25)
        for f in files:
            t = json.loads(f.read_text())
            with self.subTest(f.name):
                self.assertEqual(t['expect'].keys(), {'fail', 'exact'})
                self.assertTrue(t['expect']['exact'])
                self.assertEqual(len(t['expect']['fail']), 1)
                self.assertEqual(failed_rows(t), set(t['expect']['fail']))

    def test_each_metric_row_fails_alone_just_past_its_limit(self):
        for row, (mutate, cases) in LIMITS.items():
            for x, ok in cases:
                t = copy.deepcopy(BASE)
                mutate(t, x)
                with self.subTest(row, x=x):
                    self.assertEqual(failed_rows(t), set() if ok else {row}, row_value(t, row))

    def test_coupled_metric_rows_fail_only_with_their_partner(self):
        for row, (mutate, inside, outside, partner) in COUPLED.items():
            for x, want in ((inside, set()), (outside, {row, partner})):
                t = copy.deepcopy(BASE)
                mutate(t, x)
                with self.subTest(row, x=x):
                    self.assertEqual(failed_rows(t), want, row_value(t, row))

    def test_every_metric_row_has_a_limit_case(self):
        metric = {r['id'] for r in SPEC['rows'] if r['predicate'] in METRIC_PREDICATES
                  and ({'min', 'max'} & set(r.get('limit') or {}))}
        covered = LIMITS.keys() | COUPLED.keys() | EXEMPT.keys()
        self.assertEqual(metric - covered, set(), 'metric rows without a limit case')
        self.assertEqual(covered - metric, set(), 'limit cases for rows that are not metric rows')
        self.assertFalse(LIMITS.keys() & (COUPLED.keys() | EXEMPT.keys()))

    @unittest.skipUnless(os.environ.get('CRANE_COURSE_FUZZ_DIR'), 'set CRANE_COURSE_FUZZ_DIR')
    def test_fuzz_truths_all_pass(self):
        files = [f for f in sorted(Path(os.environ['CRANE_COURSE_FUZZ_DIR']).glob('*.json')) if not f.name.startswith('perception_truth')]
        self.assertTrue(files)
        bad = {f.name: sorted(failed_rows(json.loads(f.read_text()))) for f in files}
        self.assertEqual({k: v for k, v in bad.items() if v}, {}, f'{len(files)} truths')

    @unittest.skipUnless(os.environ.get('CRANE_COURSE_CSHARP_VERDICTS'), 'set CRANE_COURSE_CSHARP_VERDICTS')
    def test_csharp_and_python_agree(self):
        verdicts = json.loads(Path(os.environ['CRANE_COURSE_CSHARP_VERDICTS']).read_text())
        folder = Path(os.environ['CRANE_COURSE_VERDICT_DIR'])
        for name, rows in verdicts.items():
            ours = {r['id']: r for r in validate.evaluate(SPEC, json.loads((folder / name).read_text()))}
            for r in rows:
                with self.subTest(name, row=r['id']):
                    self.assertEqual(ours[r['id']]['pass'], r['pass'])
                    v, w = ours[r['id']]['value'], r['value']
                    if isinstance(v, (int, float)) and isinstance(w, (int, float)):
                        self.assertAlmostEqual(v, w, delta=1e-3)


class Truth(unittest.TestCase):
    def test_v1_keys_stay_for_the_referee_and_sim_backend(self):
        self.assertEqual(BASE['schema'], 'crane-roboboat-course-truth-v1')
        self.assertEqual(BASE['frame'], 'world')
        for e in BASE['elements']:
            self.assertLessEqual({'task', 'label', 'template', 'x', 'y', 'radius'}, e.keys())
        for b in BASE['bays']:
            self.assertLessEqual({'dock', 'index', 'x', 'y', 'yaw', 'width', 'depth', 'banner', 'occupied'}, b.keys())

    def test_json_schema(self):
        schema = json.loads((HERE.parent / 'schema' / 'course_truth.2.json').read_text())
        try:
            import jsonschema
        except ImportError:
            for key in schema['required']:
                self.assertIn(key, BASE)
            return
        jsonschema.validate(BASE, schema)

    def test_perception_truth_is_stripped_and_camera_mode_leaks_nothing(self):
        oracle = json.loads((HERE / 'fixtures' / 'perception_truth-s1.oracle.json').read_text())
        camera = json.loads((HERE / 'fixtures' / 'perception_truth-s1.camera.json').read_text())
        for doc in (oracle, camera):
            for e in doc['elements']:
                self.assertEqual(set(e), {'x', 'y', 'z', 'label', 'id'})
        # Same detections in both modes; camera ids are salted per run, so only their shape is checked.
        pos = lambda doc: sorted((e['x'], e['y'], e['z']) for e in doc['elements'])
        self.assertEqual(pos(oracle), pos(camera))
        for doc in (oracle, camera):
            ids = [e['id'] for e in doc['elements']]
            self.assertEqual(len(ids), len(set(ids)))
            self.assertTrue(all(float(i).is_integer() and 0 <= i < 2 ** 31 for i in ids))
        for e in camera['elements']:
            if 'light' in e['label'] or e['label'].startswith('banner'):
                self.assertFalse(any(c in e['label'] for c in ('red', 'green')) or any(ch.isdigit() for ch in e['label']), e)
        # sim_backend reads elements with a label; ids are opaque ints
        self.assertTrue(all(isinstance(e['id'], (int, float)) for e in camera['elements'] if e.get('label')))

    def test_state_at_follows_programs(self):
        t = {'states': [{'id': 's', 'program': {'type': 'toggle', 'values': ['red', 'green'], 'period_s': 10, 'phase_s': 0}},
                        {'id': 'h', 'program': {'type': 'horn', 'freq_hz': 840.0, 'pattern': 'two', 'repeats': 2, 'gap_s': 2, 'trigger_s': 100}}]}
        self.assertEqual([course_truth.state_at(t, 's', x) for x in (0, 10.5, 21)], ['red', 'green', 'red'])
        self.assertEqual([course_truth.state_at(t, 'h', x) for x in (100.2, 100.7, 101.2, 103.6)], ['on:840', 'off', 'on:840', 'on:840'])

    def test_render_and_diff(self):
        self.assertIn('<svg', course_truth.render.svg(BASE))
        other = json.loads(json.dumps(BASE))
        other['elements'][0]['x'] += 0.5
        self.assertEqual(list(course_truth.diff(BASE, other)['moved']), [BASE['elements'][0]['id']])


class Referee(unittest.TestCase):
    def test_fixtures_per_scoring_tier(self):
        cases = sorted(p for p in (HERE / 'referee').iterdir() if p.is_dir())
        self.assertGreaterEqual(len(cases), 15)
        for case in cases:
            with self.subTest(case.name):
                got = json.loads(json.dumps(course_truth.explain(case)))  # tuples as lists
                self.assertEqual(mismatches(got, json.loads((case / 'expected.json').read_text())), [])

    def test_matcher_rejects_a_changed_verdict(self):
        want = {'tasks': {'marina': {'tier': 'green'}, 'x': {'sweep': {'approx': 380, 'tol': 1}}}, 'contacts': []}
        self.assertEqual(mismatches({'tasks': {'marina': {'tier': 'green'}, 'x': {'sweep': 380.4}}, 'contacts': []}, want), [])
        for got in ({'tasks': {'marina': {'tier': 'unoccupied'}, 'x': {'sweep': 380.4}}, 'contacts': []},
                    {'tasks': {'marina': {'tier': 'green'}, 'x': {'sweep': 250.0}}, 'contacts': []},
                    {'tasks': {'marina': {'tier': 'green'}, 'x': {'sweep': 380.4}}, 'contacts': ['a']}):
            self.assertTrue(mismatches(got, want), got)

    @unittest.skipUnless(TESTING.is_dir(), 'mhseals_testing not checked out')
    def test_mhseals_testing_referee_reads_generated_truth(self):
        sys.path.insert(0, str(TESTING))
        from mhseals_testing.analysis import referee
        case = HERE / 'referee' / 'entry_out_and_back'
        track = [(r['t'], r['x'], r['y'], r['yaw']) for r in course_truth.read_jsonl(case / 'track.jsonl')]
        for task in ('nav_channel', 'docking', 'speed_challenge', 'entry_gates'):
            verdict = referee.judge(task, BASE, track, banner='banner_1')
            self.assertIn(verdict['verdict'], ('PASS', 'FAIL', 'N/A'))


class Allowlist(unittest.TestCase):
    INFO = """Type: std_msgs/msg/String
Subscription count: 2

Node name: referee_post
Node namespace: /
Endpoint type: SUBSCRIPTION

Node name: mission_planner
Node namespace: /
Endpoint type: SUBSCRIPTION
"""

    def test_unlisted_subscriber_is_a_violation(self):
        allow, topics = check_truth_subscribers.read_allowlist(HERE.parent / 'truth_allowlist.yaml')
        self.assertIn('/crane/truth/course', topics)
        nodes = check_truth_subscribers.subscribers(self.INFO)
        self.assertEqual(nodes, ['referee_post', 'mission_planner'])
        self.assertEqual(check_truth_subscribers.violations(nodes, allow), ['mission_planner'])


class TruthSubscribersLive(unittest.TestCase):
    """check_truth_subscribers.py against a fake ``ros2`` on PATH."""
    NAMESPACED = """Type: std_msgs/msg/String

Node name: referee_post
Node namespace: /
Endpoint type: SUBSCRIPTION

Node name: referee
Node namespace: /intruder
Endpoint type: SUBSCRIPTION

Node name: rviz2
Node namespace: /
Endpoint type: PUBLISHER
"""

    def run_with_fake_ros2(self, body):
        import contextlib
        import io
        import stat
        import tempfile
        with tempfile.TemporaryDirectory() as d:
            fake = Path(d) / 'ros2'
            fake.write_text('#!/bin/sh\n' + body)
            fake.chmod(fake.stat().st_mode | stat.S_IXUSR)
            old = os.environ.get('PATH', '')
            os.environ['PATH'] = d + os.pathsep + old
            out = io.StringIO()
            try:
                with contextlib.redirect_stdout(out):
                    code = check_truth_subscribers.main([])
            finally:
                os.environ['PATH'] = old
        return code, out.getvalue()

    def test_ros2_failure_is_unavailable(self):
        code, out = self.run_with_fake_ros2("echo 'graph unavailable' >&2\nexit 1\n")
        self.assertEqual(code, 2)
        self.assertIn('TRUTH_SUBSCRIBERS_UNAVAILABLE', out)

    def test_namespace_is_part_of_the_node_name(self):
        code, out = self.run_with_fake_ros2("cat <<'EOF'\n" + self.NAMESPACED + "EOF\n")
        self.assertEqual(code, 1)
        self.assertIn('node=intruder/referee', out)
        self.assertNotIn('node=referee_post', out)
        self.assertEqual(check_truth_subscribers.subscribers(self.NAMESPACED), ['referee_post', 'intruder/referee'])

    def test_clean_graph_passes(self):
        code, out = self.run_with_fake_ros2("echo 'Type: std_msgs/msg/String'\necho 'Subscription count: 0'\n")
        self.assertEqual((code, out.strip()), (0, 'TRUTH_SUBSCRIBERS_OK'))


if __name__ == '__main__':
    unittest.main()
