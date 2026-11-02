import json
import tempfile
import unittest
from pathlib import Path

from theater import pack, synth
from theater.tests import fixture
from theater.worlds import RoboBoatWorld


class PackTest(unittest.TestCase):
    def test_roundtrip_quantized(self):
        runs = [[{'x': 1.234, 'y': -5.67, 'z': 0.1, 'yaw': 179.99, 'pitch': 1.5, 'roll': -3.25, 'speed': 1.2345, 'progress': 0.5}],
                [{'x': 300.0, 'y': -300.0, 'yaw': -179.0}] * 3]
        with tempfile.TemporaryDirectory() as d:
            p = Path(d) / 't.pack'
            pack.write_pack(p, runs, 10)
            h, back = pack.read_pack(p)
            self.assertEqual(h['hz'], 10)
            self.assertEqual([len(r) for r in back], [1, 3])
            self.assertAlmostEqual(back[0][0]['x'], 1.23, places=6)
            self.assertAlmostEqual(back[0][0]['yaw'], 179.99, places=6)
            self.assertAlmostEqual(back[1][2]['y'], -300.0, places=6)
            self.assertEqual(back[1][0]['z'], 0.0)

    def test_crc_detects_corruption(self):
        with tempfile.TemporaryDirectory() as d:
            p = Path(d) / 't.pack'
            pack.write_pack(p, [[{'x': 1.0}]], 10)
            b = bytearray(p.read_bytes()); b[-6] ^= 1; p.write_bytes(bytes(b))
            with self.assertRaises(ValueError):
                pack.read_pack(p)

    def test_out_of_box_refused(self):
        with tempfile.TemporaryDirectory() as d, self.assertRaises(ValueError):
            pack.write_pack(Path(d) / 't.pack', [[{'x': 400.0}]], 10)

    def test_resample(self):
        s = pack.resample([(0.0, {'x': 0.0, 'yaw': 170.0}), (1.0, {'x': 10.0, 'yaw': -170.0})], 4)
        self.assertEqual(len(s), 5)
        self.assertAlmostEqual(s[2]['x'], 5.0)
        self.assertAlmostEqual(abs(s[2]['yaw']), 180.0)

    def test_golden_fixture_is_reproducible(self):
        """The committed golden (read by the C# tests) is exactly what this writer produces."""
        with tempfile.TemporaryDirectory() as d:
            fixture.write(Path(d))
            for name in ('golden.pack', 'golden_manifest.json', 'golden_expect.json'):
                self.assertEqual((Path(d) / name).read_bytes(), (fixture.DIR / name).read_bytes(), name)

    def test_progress_along_centreline(self):
        f = pack.centreline_progress([(0, 0), (10, 0), (10, 10)])
        self.assertAlmostEqual(f(5, 1), 0.25)
        self.assertAlmostEqual(f(11, 10), 1.0)


class SynthTest(unittest.TestCase):
    TRUTH = {'name': 't', 'startX': 0.0, 'startY': 0.0, 'startYaw': 0.0,
             'tasks': [{'id': 'entry_gates'}, {'id': 'debris_clearance'}, {'id': 'marina'}],
             'gates': [{'id': 'g1', 'task': 'entry_gates', 'order': 1, 'x': 5.0, 'y': 0.0, 'port': 'p1', 'starboard': 's1'},
                       {'id': 'g2', 'task': 'debris_clearance', 'order': 1, 'x': 15.0, 'y': 0.0, 'port': 'p2', 'starboard': 's2'}],
             'elements': [{'id': 'p1', 'task': 'entry_gates', 'x': 5.0, 'y': 1.0}, {'id': 's1', 'task': 'entry_gates', 'x': 5.0, 'y': -1.0},
                          {'id': 'p2', 'task': 'debris_clearance', 'x': 15.0, 'y': 1.0},
                          {'id': 's2', 'task': 'debris_clearance', 'x': 15.0, 'y': -1.0},
                          {'id': 'b1', 'task': 'debris_clearance', 'x': 25.0, 'y': 3.0},
                          {'id': 'm1', 'task': 'marina', 'x': 60.0, 'y': 3.0}],
             'bays': [], 'states': [], 'constraints': []}

    def test_only_tasks(self):
        t = RoboBoatWorld.only_tasks(self.TRUTH, ['entry_gates', 'debris_clearance'])
        self.assertEqual({e['task'] for e in t['elements']}, {'entry_gates', 'debris_clearance'})
        self.assertEqual([x['id'] for x in t['tasks']], ['entry_gates', 'debris_clearance'])

    def test_fleet_is_seeded_and_improves(self):
        w = RoboBoatWorld(RoboBoatWorld.only_tasks(self.TRUTH, ['entry_gates', 'debris_clearance']))
        a = synth.fleet(w, 40, 4, 3)
        b = synth.fleet(w, 40, 4, 3)
        self.assertEqual(json.dumps(a[1]), json.dumps(b[1]))
        first = [m for m in a[1] if m['iteration'] == 0]
        last = [m for m in a[1] if m['iteration'] == 3]
        self.assertGreater(sum(m['score'] for m in last) / len(last), sum(m['score'] for m in first) / len(first))
        self.assertEqual(len(a[2]), 4)


if __name__ == '__main__':
    unittest.main()
