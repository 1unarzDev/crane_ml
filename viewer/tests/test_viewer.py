"""viewer: the local store in the feed's shapes, ingest, the server's routes, the page's modules. The store over
crane.run/1 records is tested in tests/test_view.py (the board index over a mixed root)."""

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
from viewer.server import WEB, make_handler
from viewer.store import Store, rank_agreement, reduce_course

COURSE = {'name': 'toy-s3', 'family': 'toy', 'startX': 1.0, 'startY': 2.0, 'startYaw': math.pi / 2,
          'generator': {'seed': 3, 'truth_sha256': 'ab' * 32}, 'vessel': {'length': 1.0, 'beam': 0.8},
          'elements': [{'id': 'g.gate1.port', 'x': 5.0, 'y': 3.0, 'radius': 0.2, 'label': 'red_pole_buoy', 'role': 'gate', 'task': 'nav_channel'},
                       {'id': 'g.gate1.starboard', 'x': 5.0, 'y': 1.0, 'radius': 0.2, 'label': 'green_pole_buoy', 'role': 'gate', 'task': 'nav_channel'}],
          'gates': [{'id': 'g.gate1', 'task': 'nav_channel', 'port': 'g.gate1.port', 'starboard': 'g.gate1.starboard', 'x': 5.0, 'y': 2.0, 'order': 1}]}


def write_ctraj(path: Path, header: dict, t0: float, speed: float, events) -> Path:
    """A bare crane-traj/1 file as another writer (a cluster backend, Unity) leaves it: 10 Hz planar pose."""
    n = 300
    cols = {'boat.x': [1.0 + speed * i / 10 for i in range(n)], 'boat.y': [2.0] * n, 'boat.yaw': [0.0] * n}
    ctraj.write(path, {'pose_dims': 'planar', 'robot': {'model': 'boat', 'agents': ['boat']},
                       'frame': {'type': 'enu', 'origin': 'course'}, **header}, cols, events, hz=10.0, t0_s=t0)
    ctraj.write_lod(path)
    return path


class ViewerTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.tmp = Path(tempfile.mkdtemp())
        cls.out = cls.tmp / 'data'
        sha = COURSE['generator']['truth_sha256']
        (cls.out / 'courses').mkdir(parents=True)
        (cls.out / 'courses' / f'{sha}.json').write_text(json.dumps(COURSE))
        course = {'truth_sha256': sha, 'name': 'toy-s3'}
        ev = lambda t0, speed: [{'t': t0 + 1.0, 'kind': 'task_started', 'lane': 'task'},
                                {'t': t0 + 1.0 + 4.0 / speed, 'kind': 'gate_cleared', 'gate': 1, 'lane': 'task'}]
        (cls.out / 'runs' / 'p1_fast').mkdir(parents=True)
        write_ctraj(cls.out / 'runs' / 'p1_fast' / 'run.ctraj',
                    {'run_id': 'p1/fast', 'tier': 'fast', 'study': 'toy', 'arm': 'fast', 'seed': 3, 'task': 'nav_channel',
                     'score': 100, 'outcome': 'finished', 'course': course, 'pair': {'id': 'p1', 'high': 'unity:unityrun'}},
                    0.0, 1.0, ev(0.0, 1.0))
        (cls.out / 'runs' / 'unity_unityrun').mkdir(parents=True)
        write_ctraj(cls.out / 'runs' / 'unity_unityrun' / 'run.ctraj',
                    {'run_id': 'unity:unityrun', 'tier': 'unity', 'study': 'toy', 'arm': 'unity', 'seed': 3,
                     'task': 'nav_channel', 'score': -25, 'outcome': 'failed', 'course': course},
                    5000.0, 0.8, ev(0.0, 0.8) + [{'t': 5.0, 'kind': 'contact', 'id': 'g.gate1.port'}])
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

    def test_ingest_copies_runs_with_their_lod_and_stamps(self):
        from viewer.ingest import ingest
        dst = self.tmp / 'copy'
        n = ingest([self.out], dst, study='again', log=lambda *_: None)
        self.assertEqual(n, 2)
        t = ctraj.read(dst / 'runs' / 'unity_unityrun' / 'run.ctraj')
        self.assertEqual((t.header['run_id'], t.header['study']), ('unity:unityrun', 'again'))
        self.assertAlmostEqual(t.header['t0_s'], 5000.0)          # the run's own clock is kept
        self.assertIn('contact', [e['kind'] for e in t.events])
        self.assertTrue((dst / 'runs' / 'unity_unityrun' / 'run.lod.ctraj').exists())
        self.assertEqual(len(list((dst / 'courses').glob('*.json'))), 1)

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
