"""Write course truths for seeds the boat's scenario folder lacks, with crane's coursegen.

    python3 domains/boat/coursegen.py FROM:TO DIR     # seeds FROM..TO-1; existing files are kept

coursegen (Tools/RoboBoat/cli) targets .NET 10. It is built once per generator source into
``~/.cache/crane/coursegen/<sha>``; without a 10.x SDK the newest installed SDK builds it for its
own framework (the build runs outside the cli folder, so its global.json does not pin the SDK).
"""

import hashlib
import json
import os
from pathlib import Path
import shutil
import subprocess
import sys
import tempfile

ROOT = Path(__file__).resolve().parents[2]
CLI = ROOT / 'Tools' / 'RoboBoat' / 'cli'
SOURCES = ('Assets/Scripts/Course/Core', 'Assets/Scripts/Course/RoboBoat/Tasks', 'Tools/RoboBoat/cli/Program.cs')


def source_sha():
    h = hashlib.sha256()
    for top in SOURCES:
        p = ROOT / top
        for f in sorted([p] if p.is_file() else p.rglob('*.cs')):
            h.update(str(f.relative_to(ROOT)).encode() + b'\0' + f.read_bytes())
    return h.hexdigest()[:16]


def framework():
    sdks = subprocess.run(['dotnet', '--list-sdks'], capture_output=True, text=True, check=True).stdout.split()
    majors = sorted({int(v.split('.')[0]) for v in sdks if v[:1].isdigit()})
    if not majors:
        raise RuntimeError('coursegen: no .NET SDK installed')
    return 'net10.0' if 10 in majors else f'net{majors[-1]}.0'


def build():
    out = Path.home() / '.cache' / 'crane' / 'coursegen' / source_sha()
    dll = out / 'coursegen.dll'
    if not dll.exists():
        tfm = framework()
        proj = str(CLI / 'coursegen.csproj')
        for verb in (['restore', proj], ['build', '--no-restore', proj, '-c', 'Release', '-o', str(out)]):
            subprocess.run(['dotnet', *verb, f'-p:TargetFramework={tfm}'], cwd=tempfile.gettempdir(),
                           check=True, capture_output=True)
    return dll


def generate(seeds_from, seeds_to, folder):
    folder = Path(folder)
    folder.mkdir(parents=True, exist_ok=True)
    with tempfile.TemporaryDirectory() as tmp:
        subprocess.run(['dotnet', str(build()), '--seeds', f'{seeds_from}:{seeds_to}', '--out', tmp],
                       check=True, capture_output=True, env={**os.environ, 'DOTNET_ROLL_FORWARD': 'Major'})
        for f in sorted(Path(tmp).glob('*.json')):
            if not (folder / f.name).exists():
                json.loads(f.read_text())
                shutil.copyfile(f, folder / f.name)


if __name__ == '__main__':
    lo, _, hi = sys.argv[1].partition(':')
    generate(int(lo), int(hi), sys.argv[2])
