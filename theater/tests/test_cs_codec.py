"""The Unity codec (Assets/Scripts/Performance/CraneTraj.cs) against this one, through dotnet
when the host has it: C# decodes the golden file and re-encodes it; Python decodes that equal."""

import shutil
import subprocess
from pathlib import Path

import pytest

from theater import traj
from theater.tests import golden_data

CS = Path(__file__).parent / 'cs'


@pytest.mark.skipif(not shutil.which('dotnet'), reason='no dotnet on this host')
def test_csharp_reads_and_writes_what_python_does(tmp_path):
    build = subprocess.run(['dotnet', 'build', str(CS / 'CtrajCheck.csproj'), '-nologo', '-v', 'q',
                            '-o', str(tmp_path / 'bin'), f'-p:BaseIntermediateOutputPath={tmp_path}/obj/'],
                           capture_output=True, text=True, timeout=300)
    assert build.returncode == 0, build.stdout[-2000:] + build.stderr[-2000:]
    out = tmp_path / 'cs.ctraj'
    run = subprocess.run(['dotnet', str(tmp_path / 'bin' / 'CtrajCheck.dll'), str(golden_data.HERE / 'small.ctraj'), str(out)],
                         capture_output=True, text=True, timeout=120)
    assert run.returncode == 0, run.stderr
    assert run.stdout.split() == ['ticks=700', 'block_ticks=600', 'hz=20', 'columns=7', 'events=2']
    a, b = traj.read(golden_data.HERE / 'small.ctraj'), traj.read(out)
    assert a.header == b.header and a.columns == b.columns and a.events == b.events
