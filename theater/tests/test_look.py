import json
import tempfile
import unittest
from pathlib import Path

from theater import render


def manifest():
    runs = [{'id': f'r{i:03d}', 'iteration': i // 2, 'score': float(i), 'outcome': 'finished', 'duration_s': 60.0,
             'params': {'vx_max': 0.5 + i * 0.01}} for i in range(6)]
    return {'schema': 'crane-theater/1', 'study': 's', 'runs': runs}


class LookTest(unittest.TestCase):
    def test_script_defaults_to_the_theater_look(self):
        with tempfile.TemporaryDirectory() as d:
            (Path(d) / 'manifest.json').write_text(json.dumps(manifest()))
            render.main(['script', d, '--iterations-planned', '16'])
            m = json.loads((Path(d) / 'manifest.json').read_text())
            self.assertEqual(m['look_preset'], 'theater')
            self.assertEqual(m['iterations_planned'], 16)
            heat = [a for a in m['script']['acts'] if a['type'] == 'heat'][0]
            self.assertEqual(heat['camera']['shot'], 'tactical')
            render.main(['script', d, '--look', 'plain'])
            m = json.loads((Path(d) / 'manifest.json').read_text())
            self.assertEqual(m['look_preset'], 'plain')
            heat = [a for a in m['script']['acts'] if a['type'] == 'heat'][0]
            self.assertEqual(heat['camera']['shot'], 'top')

    def test_metrics_come_from_the_tuner_log_only(self):
        with tempfile.TemporaryDirectory() as d:
            (Path(d) / 'manifest.json').write_text(json.dumps(manifest()))
            state = {'runs': [{'run_id': 'r001', 'time_s': 112.5, 'clearance_m': 0.2, 'contacts': 0, 'gates': 6, 'of': 6,
                               'score': 1.0, 'iteration': 0, 'params': {'vx_max': 1}, 'outcome': 'finished'}]}
            (Path(d) / 'state.json').write_text(json.dumps(state))
            render.main(['metrics', d, '--from', str(Path(d) / 'state.json')])
            runs = {r['id']: r for r in json.loads((Path(d) / 'manifest.json').read_text())['runs']}
            self.assertEqual(runs['r001']['metrics'], {'time_s': 112.5, 'clearance_m': 0.2, 'contacts': 0, 'gates': 6, 'of': 6})
            self.assertNotIn('metrics', runs['r000'])

    def test_look_name(self):
        self.assertEqual(render.look_name({}), 'theater')
        self.assertEqual(render.look_name({'look_preset': 'plain'}), 'plain')
        self.assertEqual(render.look_name({'look_preset': {'name': 'plain', 'time_of_day': 'dusk'}}), 'plain')


if __name__ == '__main__':
    unittest.main()
