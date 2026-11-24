"""The crane CLI: every verb's help, tier names, a T0 run, the player build's stamp and tarball, install.sh, and
the help.yaml astro dispatches through."""

import hashlib
import json
import os
import shlex
import shutil
import subprocess
import sys
import tarfile
from pathlib import Path

import pytest
import yaml

from crane import build, cli, doctor, paths, run as crane_run, tiers
import toy

ROOT = paths.ROOT
TOY = ROOT / 'fastsim' / 'tests' / 'toy.yaml'
VERBS = sorted(cli.build_parser()._subparsers._group_actions[0].choices)
NESTED = [['bench', 'fast'], ['bench', 'player'], ['build', 'player'], ['course', 'gen']]


@pytest.mark.parametrize('words', [[v] for v in VERBS] + NESTED, ids=' '.join)
def test_every_verb_has_help_that_exits_zero(words, capsys):
    with pytest.raises(SystemExit) as done:
        cli.main([*words, '--help'])
    assert done.value.code == 0
    assert 'usage: crane ' + words[0] in capsys.readouterr().out


def test_bare_checkout_runs_crane_as_a_module_without_an_install():
    env = {k: v for k, v in os.environ.items() if k != 'PYTHONPATH'}
    out = subprocess.run([sys.executable, '-m', 'crane', '--help'], cwd=ROOT, env=env, capture_output=True,
                         text=True, timeout=30)
    assert out.returncode == 0 and 'crane doctor' in out.stdout


def test_tiers_normalise_every_spelling_and_refuse_control_layers():
    for spellings, label in ((['T0', 't0', '0', 'abstract', ' Abstract '], 'T0'),
                             (['T1', '1', 'dynamic'], 'T1'), (['T2', 't2', 'unity'], 'T2')):
        assert {tiers.normalise(s) for s in spellings} == {label}
    with pytest.raises(ValueError, match='control layers'):
        tiers.normalise('L1')
    with pytest.raises(ValueError, match='T0\\|abstract'):
        tiers.normalise('fast')


def test_run_t0_writes_a_run_dir_per_seed_from_a_config_or_a_domain(tmp_path, monkeypatch, capsys):
    for seed in (1, 2, 3):
        toy.write_scenario(tmp_path / 'sc', seed)
    common = ['--seeds', '1:4', '--set', f'scenarios.dir={tmp_path / "sc"}', '--set', 'workers=2']
    assert cli.main(['run', '--tier', 'abstract', '--config', str(TOY), '--out', str(tmp_path / 'a'), *common]) == 0
    runs = sorted((tmp_path / 'a' / 'base').iterdir())
    assert [r.name for r in runs] == ['s1', 's2', 's3']
    assert all(json.loads((r / 'run.json').read_text()) for r in runs)
    assert '3 runs at T0' in capsys.readouterr().out

    # a domain names its T0 config; a missing domain names the ones that exist
    domains = tmp_path / 'domains'
    domains.mkdir()
    (domains / 'toy.yaml').write_text(yaml.safe_dump({'tiers': {'T0': {'config': str(TOY)}}}))
    monkeypatch.setattr(paths, 'DOMAINS', domains)
    assert cli.main(['run', '--domain', 'toy', '--out', str(tmp_path / 'b'), *common]) == 0
    for a, b in zip(runs, sorted((tmp_path / 'b' / 'base').iterdir())):
        assert (a / 'track.jsonl').read_text() == (b / 'track.jsonl').read_text()
    capsys.readouterr()
    assert cli.main(['run', '--domain', 'boat', '--out', str(tmp_path / 'c'), *common]) == 2
    assert "no domain 'boat'" in (err := capsys.readouterr().err) and 'domain configs: toy' in err
    assert cli.main(['run', '--tier', 'T1', '--config', str(TOY), '--out', str(tmp_path / 'd'), *common]) == 2
    assert 'arrives in P2' in capsys.readouterr().err
    assert crane_run.overrides(['a.b=1', 'a.c=[1, 2]', 'd=x']) == {'a': {'b': 1, 'c': [1, 2]}, 'd': 'x'}


def _fake_build(builds):
    player = builds / 'CRANE-Worker'
    (player / 'CRANE_Data' / 'Managed').mkdir(parents=True)
    (player / 'CRANE.x86_64').write_bytes(b'\x7fELF player')
    (player / 'CRANE_s.debug').write_bytes(b'symbols')
    (player / 'ASV_BurstDebugInformation_DoNotShip').mkdir()
    (player / 'CRANE_Data' / 'Managed' / 'Sim.dll').write_bytes(b'..' + '--crane-roboboat-contract'.encode('utf-16-le'))
    (player / 'crane-build-manifest.json').write_text(json.dumps(
        {'schema': 'crane-build-manifest-v1', 'sourceCommit': '', 'sourceDirty': False, 'assets': [{'path': 'a'}]},
        indent=4))
    return player


def test_build_player_stamps_the_manifest_and_packs_a_verified_tarball(tmp_path, capsys):
    player = _fake_build(tmp_path)
    assert cli.main(['build', 'player', '--skip-unity', '--tarball', '--builds', str(tmp_path)]) == 0
    commit, dirty = build.source_state()
    manifest = json.loads((player / 'crane-build-manifest.json').read_text())
    assert manifest['sourceCommit'] == commit and manifest['sourceDirty'] is dirty
    assert manifest['assets'] == [{'path': 'a'}]                 # the rest of the manifest is kept
    [tarball] = [p for p in tmp_path.iterdir() if p.name.startswith(f'crane-player-{commit[:12]}-linux-x86_64.tar.')
                 and p.suffix != '.sha256']
    digest, name = (tmp_path / (tarball.name + '.sha256')).read_text().split()
    assert name == tarball.name and digest == hashlib.sha256(tarball.read_bytes()).hexdigest()
    raw = tarball.read_bytes()
    if tarball.suffix == '.zst':
        raw = subprocess.run(['zstd', '-dc'], input=raw, capture_output=True, check=True).stdout
        (tmp_path / 'plain.tar').write_bytes(raw)
        tarball = tmp_path / 'plain.tar'
    with tarfile.open(tarball) as tar:
        names = set(tar.getnames())
    assert {'CRANE-Worker/CRANE.x86_64', 'CRANE-Worker/crane-build-manifest.json',
            'CRANE-Worker/CRANE_Data/Managed/Sim.dll'} <= names
    assert not any('debug' in n.lower() or 'DoNotShip' in n for n in names)
    # the player the tarball carries passes sim_up's flag check; one without the flag does not
    assert doctor.has_flags(player / 'CRANE.x86_64')
    (player / 'CRANE_Data' / 'Managed' / 'Sim.dll').write_bytes(b'--crane-roboboat-contract')   # UTF-8 only
    assert not doctor.has_flags(player / 'CRANE.x86_64')

    # the Unity command: the project's editor, the build method, the source state handed to the manifest
    capsys.readouterr()
    assert cli.main(['build', 'player', '--dry-run', '--builds', str(tmp_path)]) == 0
    words = capsys.readouterr().out.split()
    version = (ROOT / 'ProjectSettings' / 'ProjectVersion.txt').read_text().split()[1]
    assert words[3:5] == [str(paths.HEAVY), 'unity'] and words[5].endswith(f'{version}/Editor/Unity')
    assert words[words.index('-executeMethod') + 1] == 'CranePerformanceBuild.BuildLinuxWorker'
    assert words[words.index('--crane-source-commit') + 1] == commit


def test_install_dry_run_plans_every_step_and_touches_nothing(tmp_path):
    home = tmp_path / 'home'
    home.mkdir()
    env = {**os.environ, 'HOME': str(home)}
    out = subprocess.run(['bash', ROOT / 'install.sh', '--dry-run', '--yes', '--components', 'core,fast,view',
                          '--domains', 'boat'], env=env, capture_output=True, text=True, timeout=60)
    assert out.returncode == 0, out.stderr
    assert f'repo: this checkout, {ROOT}' in out.stdout
    assert '--python 3.12' in out.stdout and f'-e {ROOT}[fast,view]' in out.stdout
    assert f'ln -sfn {ROOT}/.venv/bin/crane {home}/.local/bin/crane' in out.stdout
    assert not any(home.iterdir())                                 # a dry run writes nothing
    bad = subprocess.run(['bash', ROOT / 'install.sh', '--dry-run', '--components', 'core,warp'], env=env,
                         capture_output=True, text=True, timeout=60)
    assert bad.returncode == 2 and 'unknown component warp' in bad.stderr


def test_help_yaml_parses_and_every_run_line_is_a_crane_verb():
    data = yaml.safe_load((ROOT / 'help.yaml').read_text())
    assert data['package'] == 'crane'
    verbs = set(VERBS)
    for command in data['commands']:
        words = shlex.split(command['run'])
        assert words[0].startswith('PYTHONPATH=') and words[1:4] == ['python3', '-m', 'crane'], command['run']
        crane_words = words[4:]
        assert command['name'].split() == ['crane', *crane_words]
        assert crane_words[0] in verbs, command['name']
        if len(crane_words) > 1:
            assert crane_words[1] in cli.SUBVERBS[crane_words[0]], command['name']
        for param in command.get('params') or []:
            assert param['name'] and set(param) <= {'name', 'type', 'default', 'choices', 'description',
                                                    'required', 'form'}


@pytest.mark.skipif(not shutil.which('bash'), reason='bash')
def test_sim_up_check_reaches_crane_and_a_bad_current_is_refused_before_anything_starts(tmp_path):
    env = {**os.environ, 'SIM_UP_RUN': str(tmp_path / 'run'), 'HEAVY': 'none'}
    check = subprocess.run([paths.SIM_UP, 'check'], env=env, capture_output=True, text=True, timeout=30)
    assert check.returncode == 2 and 'arrives in P1-C' in check.stderr
    bad = subprocess.run([paths.SIM_UP, 'up', '--current', 'fast'], env=env, capture_output=True, text=True,
                         timeout=30)
    assert bad.returncode == 2 and '--current wants SPEED,TOWARD_DEG' in bad.stderr
    assert not (tmp_path / 'run').exists()
