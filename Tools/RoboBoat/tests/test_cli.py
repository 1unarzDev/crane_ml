"""coursegen (Tools/RoboBoat/cli) tests, stdlib unittest; skipped without dotnet:
    python3 -m unittest discover -s Tools/RoboBoat/tests -p 'test_cli.py'

- the tool compiles the generator files in Assets/ by reference, never a copy;
- its truths carry a consistent identity, keep the goldens and pass validate.py;
- a binary older than its sources refuses to run.

CRANE_COURSE_CLI_DIR: when set, the truths are also written there (seeds 1-50 per mode plus a
pinned case) for the Unity EditMode parity test CourseCliParity (same variable).
"""

import json
import os
import re
import shutil
import subprocess
import sys
import tempfile
import unittest
from pathlib import Path

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE.parent))
import course_truth  # noqa: E402
import validate  # noqa: E402

ROOT = HERE.parents[2]
CLI = HERE.parent / 'cli'
DLL = CLI / 'bin' / 'Release' / 'net10.0' / 'coursegen.dll'
CODE_DIRS = ('Assets/Scripts/Course/Core', 'Assets/Scripts/Course/RoboBoat/Tasks')
SPEC = json.loads((HERE.parent / 'constraints' / 'roboboat_2026.json').read_text())
GOLDEN = {('iid', 1): '85f4fa1b61bccdcb3bba1e1eda715a54a05ea79b91dc08ad29043eea0c3db38a',
          ('corner=hi', 7): 'ea13d0343448a5dddc97e6cc3bf551972ba42ca896ffbe93142acba0e9b3fb09'}
MODES = ('iid', 'corner=lo', 'corner=hi', 'corner=mixed')
PIN = ['--set', 'supply_drop.water_hulls=1', '--set', 'harbor_alert.zone2_offset=7.5']


def dotnet(*args, cwd=ROOT):
    return subprocess.run(['dotnet', *args], cwd=cwd, capture_output=True, text=True)


@unittest.skipUnless(shutil.which('dotnet'), 'dotnet SDK not installed')
class CourseGenCli(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        b = dotnet('build', '-c', 'Release', '-nologo', '-v', 'q', cwd=CLI)  # CLI/global.json pins the SDK
        assert b.returncode == 0, b.stdout + b.stderr
        out = os.environ.get('CRANE_COURSE_CLI_DIR')
        cls.tmp = tempfile.TemporaryDirectory()
        cls.out = Path(out) if out else Path(cls.tmp.name)
        for mode in MODES:
            r = dotnet(str(DLL), '--seeds', '1:51', '--mode', mode, '--out', str(cls.out / mode))
            assert r.returncode == 0, r.stderr
        r = dotnet(str(DLL), '--seeds', '1:11', *PIN, '--out', str(cls.out / 'pinned'))
        assert r.returncode == 0, r.stderr

    @classmethod
    def tearDownClass(cls):
        cls.tmp.cleanup()

    def truths(self):
        for d in sorted(p for p in self.out.iterdir() if p.is_dir()):
            for f in sorted(d.glob('*.json')):
                yield d.name, f, json.loads(f.read_text())

    def test_compiles_the_generator_by_reference_never_a_copy(self):
        items = json.loads(dotnet('msbuild', str(CLI / 'coursegen.csproj'), '-getItem:Compile').stdout)['Items']['Compile']
        compiled = {Path(i['FullPath']).resolve() for i in items}
        generator = {p.resolve() for d in CODE_DIRS for p in (ROOT / d).rglob('*.cs')}
        self.assertEqual(compiled - generator, {(CLI / 'Program.cs').resolve()},
                         'coursegen compiles a file that is not the generator in Assets/ (a copy?)')
        self.assertEqual(generator - compiled, set(), 'coursegen misses generator files')
        listed = dotnet(str(DLL), '--sources').stdout.split()[::2]
        self.assertEqual({(ROOT / p).resolve() for p in listed}, generator, 'built binary embeds other sources')
        # Program.cs is the adapter only: no generator type pasted into it, no other C# in Tools/.
        generator_types = {m for p in generator for m in re.findall(r'\b(?:class|struct|enum|interface)\s+(\w+)', p.read_text())}
        mine = set(re.findall(r'\b(?:class|struct|enum|interface)\s+(\w+)', (CLI / 'Program.cs').read_text()))
        self.assertEqual(mine & generator_types, set(), 'Program.cs redefines generator types')
        stray = [p for p in (ROOT / 'Tools').rglob('*.cs') if 'bin' not in p.parts and 'obj' not in p.parts
                 and p.resolve() != (CLI / 'Program.cs').resolve()]
        bodies = {p.read_bytes() for p in generator}
        self.assertEqual([p for p in stray if p.read_bytes() in bodies or 'namespace Sim.Course' in p.read_text()], [],
                         'a generator file copied under Tools/')

    def test_identity_goldens_and_validator(self):
        n = 0
        for mode, f, t in self.truths():
            i = course_truth.identity(t)
            self.assertEqual(course_truth.check_identity(t), [], f.name)
            self.assertEqual(course_truth.truth_sha256(t), t['generator']['truth_sha256'], f.name)
            self.assertEqual(i['options']['mode'], 'iid' if mode == 'pinned' else mode)
            if mode == 'pinned':
                self.assertEqual(i['options']['set'], {'harbor_alert.zone2_offset': '7.5', 'supply_drop.water_hulls': '1'})
            golden = GOLDEN.get((i['options']['mode'], i['seed']))
            if golden and not i['options']['set']:
                self.assertEqual(i['truth_sha256'], golden, f.name)
            failed = [r['id'] for r in validate.evaluate(SPEC, t) if not r['pass']]
            self.assertEqual(failed, [], f.name)
            n += 1
        self.assertEqual(n, 4 * 50 + 10)

    def test_pair_key_tracks_seed_options_and_version(self):
        keys = {course_truth.identity(t)['pair_key'] for _, _, t in self.truths()}
        self.assertEqual(len(keys), 4 * 50 + 10)
        i = course_truth.identity(json.loads((self.out / 'iid' / 'roboboat_2026-s1.json').read_text()))
        other = course_truth.pair_key(i['seed'], i['options'], '0' * 64)
        self.assertNotEqual(other, i['pair_key'])

    def test_same_seed_is_byte_identical(self):
        a = dotnet(str(DLL), '--seed', '3', '--mode', 'corner=mixed').stdout
        self.assertEqual(a, (self.out / 'corner=mixed' / 'roboboat_2026-s3.json').read_text())

    def test_a_stale_build_refuses_to_run(self):
        with tempfile.TemporaryDirectory() as tmp:
            for d in CODE_DIRS:
                shutil.copytree(ROOT / d, Path(tmp) / d)
            gen = Path(tmp) / CODE_DIRS[0] / 'CourseGenerator.cs'
            gen.write_text(gen.read_text() + '\n// edited\n')
            r = dotnet(str(DLL), '--seed', '1', '--root', tmp)
            self.assertEqual(r.returncode, 3)
            self.assertIn('stale build', r.stderr)

    def test_a_tampered_truth_fails_its_identity(self):
        t = json.loads((self.out / 'iid' / 'roboboat_2026-s2.json').read_text())
        t['elements'][0]['x'] += 0.5
        self.assertTrue(any('file edited' in p for p in course_truth.check_identity(t)))
        t = json.loads((self.out / 'iid' / 'roboboat_2026-s2.json').read_text())
        t['generator']['identity']['options']['mode'] = 'corner=hi'
        self.assertTrue(course_truth.check_identity(t))


if __name__ == '__main__':
    unittest.main()
