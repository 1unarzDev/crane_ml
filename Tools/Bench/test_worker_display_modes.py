"""Execute the launcher against a recording player to verify display isolation."""
import json
import os
from pathlib import Path
import subprocess
import tempfile
import unittest

LAUNCHER = Path(__file__).with_name('run_worker.sh')


class WorkerDisplayModes(unittest.TestCase):
    def launch(self, headless, batch=False):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            player = root / 'player'
            player.write_text('''#!/usr/bin/env python3
import json,os,sys
from pathlib import Path
Path(os.environ['RECORD']).write_text(json.dumps({'args':sys.argv[1:], 'display':os.environ.get('DISPLAY'), 'wayland':os.environ.get('WAYLAND_DISPLAY'), 'sdl':os.environ.get('SDL_VIDEODRIVER')}))
''')
            player.chmod(0o755)
            sampler = root / 'nvidia-smi'
            sampler.write_text('#!/bin/sh\nexit 0\n')
            sampler.chmod(0o755)
            environment = dict(os.environ, CRANE_PLAYER=str(player),
                               CRANE_RESULT_ROOT=str(root / 'results'),
                               CRANE_NOGRAPHICS=str(int(headless)),
                               CRANE_BATCHMODE=str(int(batch)),
                               PATH=str(root) + ':' + os.environ['PATH'],
                               DISPLAY=':test', WAYLAND_DISPLAY='test-wayland',
                               RECORD=str(root / 'record.json'))
            subprocess.run(['bash', str(LAUNCHER), '0'], env=environment,
                           check=True, capture_output=True, timeout=15)
            return json.loads((root / 'record.json').read_text())

    def test_headless_has_no_display_connection(self):
        result = self.launch(True)
        self.assertIn('-batchmode', result['args'])
        self.assertIn('-nographics', result['args'])
        self.assertIsNone(result['display'])
        self.assertIsNone(result['wayland'])
        self.assertEqual('dummy', result['sdl'])

    def test_interactive_preserves_display(self):
        result = self.launch(False)
        self.assertNotIn('-nographics', result['args'])
        self.assertNotIn('-batchmode', result['args'])
        self.assertEqual(':test', result['display'])
        self.assertEqual('test-wayland', result['wayland'])

    def test_gpu_batch_retains_device_connection_without_window(self):
        result = self.launch(False, batch=True)
        self.assertIn('-batchmode', result['args'])
        self.assertNotIn('-nographics', result['args'])
        self.assertEqual(':test', result['display'])
        self.assertEqual('test-wayland', result['wayland'])
        self.assertNotEqual('dummy', result['sdl'])


if __name__ == '__main__':
    unittest.main()
