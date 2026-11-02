"""viewer: ingest -> crane-traj/1, the local store in the feed's shapes, the server's routes, the page's modules."""

import json
import math
import shutil
import subprocess
import tempfile
import threading
import unittest
import urllib.error
import urllib.request
from pathlib import Path

from theater import traj as ctraj
from viewer import ingest
from viewer.server import WEB, make_handler
from viewer.store import Store, rank_agreement, reduce_course

COURSE = {'name': 'toy-s3', 'family': 'toy', 'startX': 1.0, 'startY': 2.0, 'startYaw': math.pi / 2,
          'generator': {'seed': 3, 'truth_sha256': 'ab' * 32}, 'vessel': {'length': 1.0, 'beam': 0.8},
          'elements': [{'id': 'g.gate1.port', 'x': 5.0, 'y': 3.0, 'radius': 0.2, 'label': 'red_pole_buoy', 'role': 'gate', 'task': 'nav_channel'},
                       {'id': 'g.gate1.starboard', 'x': 5.0, 'y': 1.0, 'radius': 0.2, 'label': 'green_pole_buoy', 'role': 'gate', 'task': 'nav_channel'}],
          'gates': [{'id': 'g.gate1', 'task': 'nav_channel', 'port': 'g.gate1.port', 'starboard': 'g.gate1.starboard', 'x': 5.0, 'y': 2.0, 'order': 1}]}


def run_dir(root: Path, name: str, t0: float, speed: float, points: int, contact=False) -> Path:
    d = root / name
    (d / 'analysis').mkdir(parents=True)
    # irregular samples on the run's own clock
    track = [{'t': t0 + i * 0.07, 'x': 1.0 + speed * i * 0.07, 'y': 2.0, 'yaw': 0.0} for i in range(300)]
    (d / 'track.jsonl').write_text(''.join(json.dumps(p) + '\n' for p in track))
    ev = [{'event': 'task_started', 'task': 'nav_channel', 'time': t0 + 1.0},
          {'event': 'decision', 'kind': 'execute', 'reason': 'through the middle', 'time': t0 + 1.0},
          {'event': 'goal', 'x': 2.0, 'y': 0.0, 'why': 'gate 1', 'time': t0 + 1.0},
          {'event': 'gate_cleared', 'gate': 1, 'time': t0 + 1.0 + 4.0 / speed},
          {'event': 'task_finished', 'outcome': 'SUCCESS', 'time': t0 + 20.0}]
    (d / 'task_events.jsonl').write_text(''.join(json.dumps(e) + '\n' for e in ev) + '{"torn')
    (d / 'task_status.jsonl').write_text(json.dumps({'elapsed': 0.0, 'phase': 'execute', 'behaviour': 'through the channel'}) + '\n')
    (d / 'course_truth.json').write_text(json.dumps(COURSE))
    contacts = [{'t': t0 + 5.0, 'object': 'g.gate1.port', 'min_clearance_m': 0.0}] if contact else []
    (d / 'analysis' / 'score.001.json').write_text(json.dumps({
        'summary': f'{points} pts', 'total': {'points': points},
        'tasks': {'entry_gates': {'rows': [{'id': 'T1.entry', 'desc': 'through both gates', 'possible': 100, 'points': points, 'status': 'scored', 't': t0 + 6.0}]}},
        'reliability': {'clean': not contact, 'failed': ['contact'] if contact else [], 'flags': {}},
        'diagnostics': {'contacts': {'contacts': contacts, 'near_misses': []}}}))
    (d / 'session.json').write_text(json.dumps({'id': name}))
    return d


class ViewerTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.tmp = Path(tempfile.mkdtemp())
        src = cls.tmp / 'src'
        cls.fast = run_dir(src, 'fastrun', 0.0, 1.0, 100)
        cls.unity = run_dir(src, 'unityrun', 5000.0, 0.8, -25, contact=True)
        cls.out = cls.tmp / 'data'
        ingest.write_run(cls.fast, cls.out / 'runs' / 'p1_fast', 'fast', 'p1/fast',
                         {'study': 'toy', 'arm': 'fast', 'pair': {'id': 'p1', 'high': 'unity:unityrun'}}, courses=cls.out / 'courses')
        ingest.write_run(cls.unity, cls.out / 'runs' / 'unity_unityrun', 'unity', 'unity:unityrun', {'study': 'toy', 'arm': 'unity'},
                         courses=cls.out / 'courses')
        rep = cls.tmp / 'judge' / 'pairs'
        rep.mkdir(parents=True)
        (rep / 'report.json').write_text(json.dumps({'schema': 'crane.judge.report/1', 'kind': 'tiers', 'n_pairs': 1,
            'pairs': [{'low': 'p1/fast', 'high': 'unity:unityrun', 'outcome': ['success', 'contact']}],
            'insights': [{'id': 'divergence.first:1', 'kind': 'divergence.first', 'claim': 'they part', 'scope': {'pair': ['p1/fast', 'unity:unityrun']}},
                         {'id': 'divergence.first:2', 'kind': 'divergence.first', 'claim': 'another pair', 'scope': {'pair': ['p9/fast', 'other']}}],
            'g4': {'metrics': {}}}))
        (rep / 'feed.json').write_text(json.dumps({'divergence.first:1': [['unity:unityrun', 5003.0, 5005.0, ['pose.se2'], 'diverge']],
                                                   'divergence.first:2': [['p9/fast', 1.0, 2.0, ['pose.se2'], 'diverge']]}))
        cls.store = Store([cls.out, cls.tmp / 'judge'])

    @classmethod
    def tearDownClass(cls):
        shutil.rmtree(cls.tmp, ignore_errors=True)

    def test_ingest_keeps_the_runs_clock_and_events(self):
        t = ctraj.read(self.out / 'runs' / 'unity_unityrun' / 'run.ctraj')
        self.assertEqual(t.header['run_id'], 'unity:unityrun')
        self.assertEqual(t.header['tier'], 'unity')
        self.assertAlmostEqual(t.header['t0_s'], 5000.0)
        self.assertEqual(t.hz, 10.0)
        kinds = [e['kind'] for e in t.events]
        for k in ('task_started', 'decision', 'goal', 'gate_cleared', 'task_finished', 'behaviour', 'row', 'contact'):
            self.assertIn(k, kinds)
        started = next(e for e in t.events if e['kind'] == 'task_started')
        self.assertAlmostEqual(started['t'], 1.0, places=3)  # seconds from t0, not absolute
        goal = next(e for e in t.events if e['kind'] == 'goal')
        self.assertAlmostEqual(goal['x'], 1.0, places=3)  # task frame (2, 0) turned by the start yaw of 90 degrees
        self.assertAlmostEqual(goal['y'], 4.0, places=3)
        contact = next(e for e in t.events if e['kind'] == 'contact')
        self.assertEqual((contact['x'], contact['y']), (5.0, 3.0))
        self.assertEqual(t.header['channels']['perception'], 'empty')
        self.assertEqual(t.header['outcome'], 'finished')
        self.assertTrue((self.out / 'runs' / 'unity_unityrun' / 'run.lod.ctraj').exists())

    def test_resample_is_fixed_rate_and_wraps_yaw(self):
        t0, (xs, _, yaws) = ingest.resample([{'t': 0, 'x': 0, 'y': 0, 'yaw': 3.1}, {'t': 1, 'x': 1, 'y': 0, 'yaw': -3.1}], 4.0)
        self.assertEqual(len(xs), 5)
        self.assertAlmostEqual(xs[2], 0.5)
        self.assertGreater(abs(yaws[2]), 3.1)  # through pi, not through 0

    def test_index_pairs_studies_and_rank(self):
        idx = self.store.index()
        self.assertEqual({r['run_id'] for r in idx['runs']}, {'p1/fast', 'unity:unityrun'})
        self.assertEqual(idx['pairs'], [{'id': 'p1', 'low': 'p1/fast', 'high': 'unity:unityrun', 'seed': 3, 'task': 'nav_channel',
                                         'low_score': 100, 'high_score': -25}])
        self.assertEqual(idx['studies'][0]['arms'], ['fast', 'unity'])
        self.assertEqual([j['id'] for j in idx['judge']], ['pairs'])

    def test_rank_agreement(self):
        ps = [{'low_score': a, 'high_score': b} for a, b in ((0, 0), (100, 0), (100, 100), (50, 200))]
        r = rank_agreement(ps)
        self.assertEqual(r['n'], 4)
        self.assertEqual(r['agree'], 2)
        self.assertEqual(r['mmrv'], 200)  # 100/0 ranked above 50/200
        self.assertIsNone(rank_agreement(ps[:2]))

    def test_judge_for_keeps_only_these_runs(self):
        j = self.store.judge_for(['p1/fast', 'unity:unityrun'])
        self.assertEqual(len(j), 1)
        self.assertEqual(list(j[0]['feed']), ['divergence.first:1'])
        self.assertEqual(len(j[0]['report']['pairs']), 1)
        self.assertEqual(self.store.judge_for(['nobody']), [])

    def test_reduce_course_keeps_ids_and_gates(self):
        c = reduce_course(COURSE)
        self.assertEqual(c['start'], [1.0, 2.0, round(math.pi / 2, 3)])
        self.assertEqual(c['elements'][0]['id'], 'g.gate1.port')
        self.assertEqual(c['gates'][0]['starboard'], 'g.gate1.starboard')

    def test_server_routes(self):
        from http.server import ThreadingHTTPServer
        httpd = ThreadingHTTPServer(('127.0.0.1', 0), make_handler(self.store))
        th = threading.Thread(target=httpd.serve_forever, daemon=True)
        th.start()
        base = f'http://127.0.0.1:{httpd.server_address[1]}'
        get = lambda p: urllib.request.urlopen(base + p, timeout=10)
        try:
            self.assertIn(b'crane viewer', get('/').read())
            self.assertIn(b'crane viewer', get('/viewer/').read())  # the dashboard's mount path
            self.assertIn(b'export async function decode', get('/ctraj.js').read())
            self.assertEqual(json.loads(get('/v1/viewer/index').read())['source'], 'local')
            body = get('/v1/feed/runs/p1%2Ffast/traj').read()
            self.assertEqual(ctraj.loads(body).header['run_id'], 'p1/fast')
            self.assertEqual(get('/v1/feed/runs/p1%2Ffast/traj?lod=1').headers['x-crane-lod'], '1')
            sha = ctraj.read_header(self.out / 'runs' / 'p1_fast' / 'run.ctraj')['course']['truth_sha256']
            self.assertEqual(json.loads(get(f'/v1/feed/courses/{sha}').read())['name'], 'toy-s3')
            g = json.loads(get('/v1/feed/ghosts?run=p1%2Ffast').read())
            self.assertEqual([r['run_id'] for r in g['runs']], ['unity:unityrun'])
            f = json.loads(get('/v1/feed/fleet?study=toy').read())
            files = ctraj.fleet_unpack(get(f'/v1/feed/fleet.pack?study=toy&h={f["pack_sha256"]}').read())
            self.assertEqual(len(files), len(f['runs']))
            self.assertEqual(json.loads(get('/v1/viewer/judge?runs=p1%2Ffast').read())[0]['id'], 'pairs')
            with self.assertRaises(urllib.error.HTTPError) as e:
                get('/v1/feed/runs/nope/traj')
            self.assertEqual(e.exception.code, 404)
            with self.assertRaises(urllib.error.HTTPError) as e:
                get('/../../etc/passwd')
            self.assertEqual(e.exception.code, 404)
        finally:
            httpd.shutdown()
            httpd.server_close()

    @unittest.skipUnless(shutil.which('node'), 'node not installed')
    def test_page_modules_parse(self):
        for f in sorted(WEB.glob('*.js')):
            r = subprocess.run(['node', '--input-type=module', '--check'], input=f.read_text(), capture_output=True, text=True)
            self.assertEqual(r.returncode, 0, f'{f.name}: {r.stderr}')


if __name__ == '__main__':
    unittest.main()
