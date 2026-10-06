"""Execute real launcher mode selection with conflicting inherited batch settings."""
import json
import os
from pathlib import Path
import subprocess
import tempfile
import unittest

ROOT = Path(__file__).resolve().parents[2]


class CampusDisplaySelection(unittest.TestCase):
    def test_requested_mode_overrides_inherited_batch_flag(self):
        launchers = [
            ('Tools/Performance/run_campus_nav2_fixture.sh', 'manifest=', []),
            ('Tools/Campus/run_calibration.sh', 'export CRANE_SCENE=', ['CAL-02', '.01']),
        ]
        for relative, boundary, preceding in launchers:
            for mode in ['headless', 'interactive']:
                with self.subTest(launcher=relative, mode=mode), tempfile.TemporaryDirectory() as directory:
                    target = Path(directory) / relative
                    target.parent.mkdir(parents=True)
                    prefix = (ROOT / relative).read_text().split(boundary, 1)[0]
                    target.write_text(prefix + "\npython3 - <<'END'\nimport json,os\nprint(json.dumps({key:os.environ.get(key) for key in ['CRANE_BATCHMODE','CRANE_NOGRAPHICS']}))\nEND\n")
                    expected = '1' if mode == 'headless' else '0'
                    environment = dict(os.environ, CRANE_BATCHMODE='0' if expected == '1' else '1',
                                       CRANE_RESULT_ROOT=str(Path(directory) / 'results'))
                    result = subprocess.run(['bash', str(target), *preceding, mode],
                                            env=environment, check=True, capture_output=True, text=True)
                    selected = json.loads(result.stdout)
                    self.assertEqual(expected, selected['CRANE_BATCHMODE'])
                    self.assertEqual(expected, selected['CRANE_NOGRAPHICS'])


if __name__ == '__main__':
    unittest.main()
