"""Course truth, validator and agent-tool tests (design §7), stdlib unittest:
    python3 -m unittest discover -s Tools/RoboBoat/tests

Optional, set by the Codex campaigns:
    CRANE_COURSE_FUZZ_DIR        every truth in it must pass validate.py
    CRANE_COURSE_CSHARP_VERDICTS CourseCli.Validate output; Python must agree on every row of
                                 every truth in CRANE_COURSE_VERDICT_DIR
"""

import json
import math
import os
import sys
import unittest
from pathlib import Path

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE.parent))
import check_truth_subscribers  # noqa: E402
import course_truth  # noqa: E402
import validate  # noqa: E402

SPEC = json.loads((HERE.parent / 'constraints' / 'roboboat_2026.json').read_text())
BASE = json.loads((HERE / 'fixtures' / 'roboboat_2026-s1.json').read_text())
TESTING = Path('/home/lunarz/astro_dock/src/mhseals_testing')


def failed_rows(truth):
    return {r['id'] for r in validate.evaluate(SPEC, truth) if not r['pass']}


def matches(got, want):
    """want's leaves: equal, or '<N', '<=N', '>N', '>=N' on the absolute value."""
    for key, value in want.items():
        g = got.get(key) if isinstance(got, dict) else None
        if isinstance(value, dict):
            if not matches(g or {}, value):
                return False
        elif isinstance(value, str) and value[:1] in '<>':
            op = value[:2] if value[1] == '=' else value[:1]
            x, n = abs(g or 0), float(value[len(op):])
            if not {'>=': x >= n, '<=': x <= n, '<': x < n, '>': x > n}[op]:
                return False
        elif g != value:
            return False
    return True


class Validator(unittest.TestCase):
    def test_base_course_passes_every_row(self):
        self.assertEqual(failed_rows(BASE), set())
        self.assertEqual({r['id'] for r in BASE['constraints']}, {r['id'] for r in SPEC['rows']})

    def test_bad_layouts_are_rejected_by_their_rows(self):
        files = sorted((HERE / 'bad').glob('*.json'))
        self.assertGreaterEqual(len(files), 15)
        for f in files:
            t = json.loads(f.read_text())
            want, failed = set(t['expect']['fail']), failed_rows(t)
            with self.subTest(f.name):
                if t['expect']['any_of']:
                    self.assertTrue(want & failed, failed)
                else:
                    self.assertLessEqual(want, failed)
                    if t['expect']['exact']:
                        self.assertEqual(failed, want)

    def test_moving_one_object_past_a_limit_fails_exactly_that_row(self):
        g = next(x for x in BASE['gates'] if x['id'] == 'entry_gates.gate1')
        for cc, ok in ((3.04, True), (3.07, False), (1.84, True), (1.81, False)):
            t = json.loads(json.dumps(BASE))
            p = next(e for e in t['elements'] if e['id'] == g['port'])
            s = next(e for e in t['elements'] if e['id'] == g['starboard'])
            mx, my = (p['x'] + s['x']) / 2, (p['y'] + s['y']) / 2
            lx, ly = -math.sin(g['normal_yaw']), math.cos(g['normal_yaw'])
            p['x'], p['y'], s['x'], s['y'] = mx + lx * cc / 2, my + ly * cc / 2, mx - lx * cc / 2, my - ly * cc / 2
            with self.subTest(cc=cc):
                self.assertEqual(failed_rows(t), set() if ok else {'T1.cc'})

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
        self.assertEqual([e['id'] for e in oracle['elements']], [e['id'] for e in camera['elements']])
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
                got = course_truth.explain(case)
                self.assertTrue(matches(got, json.loads((case / 'expected.json').read_text())), json.dumps(got)[:600])

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


if __name__ == '__main__':
    unittest.main()
