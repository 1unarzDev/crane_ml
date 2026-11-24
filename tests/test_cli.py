"""The crane CLI: the registry drives argparse, help and help.yaml; the shared selectors; --json and exit codes;
the bin/crane launcher; tier names, a T0 run, the player build's stamp and tarball, install.sh."""

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

from crane import build, cli, commands as reg, doctor, paths, run as crane_run, select, simcheck, tiers
from fastsim import batch
import toy

ROOT = paths.ROOT
TOY = ROOT / 'fastsim' / 'tests' / 'toy.yaml'
LAUNCHER = ROOT / 'bin' / 'crane'
SAMPLE = {'int': ('3', 3), 'float': ('0.5', 0.5), 'tier': ('dynamic', 'T1'), 'env': ('course:a', 'course:a')}


def _strict(text):
    def refuse(constant):
        raise ValueError(f'{constant} is not JSON')
    return json.loads(text, parse_constant=refuse)


def _launch(*args, cwd, timeout=60):
    env = {'PATH': '/usr/bin:/bin', 'HOME': os.environ['HOME']}       # what astro's remote.sim gives: no shell
    return subprocess.run([str(LAUNCHER), *args], cwd=cwd, env=env, capture_output=True, text=True, timeout=timeout)


def test_the_registry_builds_the_parser_and_every_param_round_trips():
    parser = cli.build_parser()
    parsed = [c for c in reg.COMMANDS if not (c.passthrough or c.today)]
    assert {c.name for c in parsed} >= {'crane run', 'crane doctor', 'crane fit', 'crane compare', 'crane view',
                                         'crane field import', 'crane build player', 'crane bench player'}
    for command in parsed:
        argv, expect, groups = list(command.words), {}, set()
        params = reg.params_of(command)
        for p in sorted(params, key=lambda p: not p.positional):           # positionals first
            if p.group and p.group in groups:
                continue                                                    # one of a mutually exclusive group
            groups.add(p.group)
            dest = p.dest or p.name.lstrip('-').replace('-', '_')
            raw, value = SAMPLE.get(p.type, (p.choices[0] if p.choices else 'v',) * 2)
            if p.type == 'bool':
                argv.append(p.name)
                expect[dest] = True
                continue
            argv += [raw] if p.positional else [p.name, raw]
            expect[dest] = [value] if p.repeat or p.type == 'list' or p.nargs in ('*', '+') else value
        args = parser.parse_args(argv)
        assert args._command is command
        assert {k: getattr(args, k) for k in expect} == expect, command.name
        assert callable(reg.handler(command))
    for command in reg.COMMANDS:                    # passthroughs resolve too; a stub names what to use today
        assert reg.find(list(command.words)) is command
        assert command.today or callable(reg.handler(command))
        assert command.examples and all(e.startswith('crane ') for e in command.examples), command.name
    assert reg.find(['course', 'gen']) is reg.find(['env', 'gen'])         # course is env's alias


@pytest.mark.parametrize('command', reg.COMMANDS, ids=lambda c: ' '.join(c.words))
def test_every_command_has_help_with_an_example(command, capsys):
    with pytest.raises(SystemExit) as done:
        cli.main([*command.words, '--help'])
    out = capsys.readouterr().out
    assert done.value.code == 0 and out.startswith(f'usage: crane {" ".join(command.words)}')
    assert 'example' in out and command.examples[0] in out
    assert ('not yet available; today use' in out) == bool(command.today)


def test_stubs_refuse_with_one_line_and_globals_are_checked(capsys):
    assert cli.main(['study', 'run', 'spec.yaml', '--seeds', '0:9']) == 2
    err = capsys.readouterr().err.strip().splitlines()
    assert err == ['crane study run: not yet available; today use crane run --domain D --tier T --seeds A:B per arm']
    assert cli.main(['--json', 'view']) == 2                                   # view takes no --json
    assert capsys.readouterr().err.strip() == 'crane view: --json is not supported here; fix: crane view --help'
    with pytest.raises(SystemExit) as done:
        cli.main(['run', '--domain', 'toy', '--tier', 'L1'])
    assert done.value.code == 2 and 'control layers' in capsys.readouterr().err


def test_seed_specs_parse_one_way_everywhere(tmp_path, monkeypatch):
    for text in ('3', '1:4', '5,9', '0:3,7'):
        assert select.parse_seeds(text) == (batch.parse_seeds(text) if text != '0:3,7' else [0, 1, 2, 7])
    (tmp_path / 'seeds.txt').write_text('# held-out\n10:12, 20\n30  # last\n')
    assert select.parse_seeds(f'@{tmp_path / "seeds.txt"}') == [10, 11, 20, 30]
    domains = tmp_path / 'domains'
    domains.mkdir()
    (domains / 'd.yaml').write_text(yaml.safe_dump({'seeds': {'dev': '0:3', 'held': {'seeds': [7, 8], 'role':
                                                                                     'held-out'}}}))
    monkeypatch.setattr(paths, 'DOMAINS', domains)
    assert select.parse_seeds('block:dev', 'd') == [0, 1, 2] and select.parse_seeds('block:held', 'd') == [7, 8]
    assert select.block_role('d', 'held') == 'held-out'
    for bad, why in (('1:1', 'empty'), ('1,1', 'twice'), ('x', 'bad seed spec'), ('block:nope', 'blocks: dev, held'),
                     (f'@{tmp_path / "none"}', 'seed file')):
        with pytest.raises(select.SelectError, match=why):
            select.parse_seeds(bad, 'd')
    assert select.parse_sets(['a.b=1', 'a.c=[1, 2]', 'd=x', 'e=']) == {'a': {'b': 1, 'c': [1, 2]}, 'd': 'x', 'e': ''}
    out = select.default_out('boat', 'T1')
    assert out.parent == select.RUNS_ROOT / 'boat' / 'T1' and ROOT not in out.parents


def test_json_documents_for_doctor_a_dry_run_and_a_card(tmp_path, capsys):
    assert cli.main(['doctor', '--json']) in (0, 1)
    doc = _strict(capsys.readouterr().out)
    assert doc['schema'] == 'crane.doctor/1' and {c['name'] for c in doc['checks']} >= {'python', 'player', 'heavy'}
    sc = tmp_path / 'sc'
    assert cli.main(['--json', 'run', '--domain', 'toy', '--seeds', '0:4', '--set', f'scenarios.dir={sc}',
                     '--dry-run']) == 0
    plan = _strict(capsys.readouterr().out)
    assert plan['tier'] == 'T0' and plan['seeds'] == [0, 1, 2, 3] and plan['scenarios'] == str(sc)
    assert Path(plan['out']).parent == select.RUNS_ROOT / 'toy' / 'T0'
    assert not sc.exists() and not Path(plan['out']).exists()                 # a dry run writes nothing
    card = _launch('card', 'show', '--json', cwd=tmp_path)
    assert card.returncode == 0, card.stderr
    doc = _strict(card.stdout)
    assert doc['schema'] == 'crane.card/1' and doc['robot'] and doc['doc']['blocks']


def test_judge_exits_one_on_fail_and_zero_on_pass(tmp_path):
    report = {'schema': 'crane.judge.report/1', 'kind': 'runs', 'n': 2, 'outcomes': {'success': 2}, 'insights': []}
    (tmp_path / 'pass.json').write_text(json.dumps(report))
    report['insights'] = [{'kind': 'integrity.invalid', 'severity': 'fail', 'claim': 'run 1 is invalid'}]
    (tmp_path / 'fail.json').write_text(json.dumps(report))
    ok = _launch('judge', 'oneline', str(tmp_path / 'pass.json'), cwd=tmp_path)
    assert ok.returncode == 0 and ok.stdout.startswith('JUDGE 2 runs'), ok.stderr
    bad = _launch('judge', 'oneline', str(tmp_path / 'fail.json'), cwd=tmp_path)
    assert bad.returncode == 1 and 'judge: FAIL (insight integrity.invalid)' in bad.stderr
    doc = _launch('--json', 'judge', 'oneline', str(tmp_path / 'fail.json'), cwd=tmp_path)
    assert doc.returncode == 1 and _strict(doc.stdout)['insights'][0]['severity'] == 'fail'
    (tmp_path / 'empty').mkdir()                                           # nothing judged is never a pass
    assert _launch('judge', 'fastsim', str(tmp_path / 'empty'), cwd=tmp_path).returncode == 2
    assert _launch('judge', 'fastsim', str(tmp_path / 'nonexistent'), cwd=tmp_path).returncode == 2


def test_launcher_runs_from_any_cwd_without_a_shell_and_ignores_the_cwd(tmp_path):
    decoy = tmp_path / 'crane'
    decoy.mkdir()
    (decoy / '__init__.py').write_text('raise SystemExit("the cwd crane was imported")\n')
    (tmp_path / 'yaml.py').write_text('raise SystemExit("the cwd yaml was imported")\n')
    out = _launch('help', '--json', cwd=tmp_path)
    assert out.returncode == 0, out.stderr
    names = {c['name'] for c in _strict(out.stdout)['commands']}
    assert {'crane sim check', 'crane run', 'crane study run'} <= names
    link = tmp_path / 'bin' / 'crane'
    link.parent.mkdir()
    link.symlink_to(LAUNCHER)                                                 # ~/.local/bin/crane style
    assert subprocess.run([str(link), '--version'], cwd=tmp_path, env={'PATH': '/usr/bin:/bin'},
                          capture_output=True, text=True, timeout=30).stdout.startswith('crane ')


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
    assert cli.main(['run', '--domain', 'toy', '--out', str(tmp_path / 'b'), *common]) == 2   # written once
    assert 'already holds' in (err := capsys.readouterr().err) and 'fix: pass a new --out' in err
    assert cli.main(['run', '--domain', 'boat', '--out', str(tmp_path / 'c'), *common]) == 2
    assert "no domain 'boat'" in (err := capsys.readouterr().err) and 'domain configs: toy' in err
    assert cli.main(['run', '--tier', 'T2', '--config', str(TOY), '--out', str(tmp_path / 'd'), *common]) == 2
    assert 'not yet available' in (err := capsys.readouterr().err) and 'crane sim up' in err
    assert crane_run.overrides(['a.b=1', 'a.c=[1, 2]', 'd=x']) == {'a': {'b': 1, 'c': [1, 2]}, 'd': 'x'}


def test_run_domain_from_a_clean_checkout_applies_overrides_and_makes_missing_scenarios(tmp_path, capsys):
    # the real toy domain: its tier overrides name a generator, the scenarios do not exist yet
    sc = tmp_path / 'sc'
    args = ['run', '--domain', 'toy', '--seeds', '0:2', '--allow-dirty', '--set', f'scenarios.dir={sc}',
            '--out', str(tmp_path / 'out')]
    assert cli.main(args) == 0                                    # no --tier: the domain's first
    out = capsys.readouterr().out
    assert sorted(p.name for p in sc.iterdir()) == ['toy-s0.json', 'toy-s1.json']
    assert '2 runs at T0' in out and any(l.startswith('fidelity: ') for l in out.splitlines())
    # a source with no generator names its scenario dir instead of failing inside a worker
    bare = tmp_path / 'bare'
    assert cli.main(['run', '--config', str(TOY), '--seeds', '0:2', '--set', f'scenarios.dir={bare}',
                     '--out', str(tmp_path / 'o2')]) == 2
    assert str(bare) in capsys.readouterr().err


def test_sim_check_ignores_a_stale_log_and_names_what_failed(tmp_path, monkeypatch, capsys):
    (tmp_path / 'player.log').write_text('CRANE_ROBOBOAT_COURSE_READY\nCRANE_PLANT_APPLIED x\n')
    (tmp_path / 'player.path').write_text('/old/CRANE.x86_64')                # no player.pid: not running
    monkeypatch.setattr(simcheck, 'RUN', tmp_path)
    monkeypatch.setattr(simcheck, '_container_running', lambda: True)
    monkeypatch.setattr(simcheck, '_nodes', lambda: ['/ekf_local', '/bt_navigator'])
    rows = {name: (ok, detail) for name, ok, detail in simcheck.live(tmp_path)}
    assert not rows['player running'][0] and rows['player running'][1] == \
        'not running (last: /old/CRANE.x86_64); crane sim up --headless'
    assert not rows['course ready'][0] and not rows['plant applied'][0]       # the stale log is not read
    assert not rows['nav stack up'][0] and 'rtabmap' in rows['nav stack up'][1]
    assert simcheck.main(['--wait', '0']) == 1
    assert 'sim check: not ready, failed: player running, course ready' in capsys.readouterr().out
    monkeypatch.setattr(simcheck.time, 'sleep', lambda s: None)
    assert simcheck.main(['--wait', '0.05']) == 75                           # a bounded wait that ran out
    assert 'not ready after 0.05 s' in capsys.readouterr().out
    monkeypatch.setattr(simcheck, '_nodes', lambda: ['/ekf_local', '/rtabmap', '/bt_navigator'])
    assert dict((n, ok) for n, ok, _ in simcheck.live(tmp_path))['nav stack up']


def test_bare_sim_prints_help_and_doctor_names_unselected_extras_off(capsys, monkeypatch):
    with pytest.raises(SystemExit) as done:
        cli.main(['sim'])
    out = capsys.readouterr().out
    assert done.value.code == 0 and 'crane sim {up,down,status,check,logs} [ARGS...]' in out
    monkeypatch.setattr(doctor, '_found', lambda module: False)
    rows = {name: (state, detail) for name, state, detail in doctor.checks()}
    assert rows['[view]'][0] == 'off' and 'optional, add with' in rows['[view]'][1]
    assert doctor.build_label(paths.PLAYER_BUILD / 'CRANE.x86_64') in ('unknown build',) or \
        doctor.build_label(paths.PLAYER_BUILD / 'CRANE.x86_64').startswith('build ')


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
    if not (ROOT / '.venv').exists():                              # an existing venv is reused, not recreated
        assert '--python 3.12' in out.stdout
    assert f'-e {ROOT}[fast,view]' in out.stdout
    assert f'ln -sfn {ROOT}/.venv/bin/crane {home}/.local/bin/crane' in out.stdout
    assert not any(home.iterdir())                                 # a dry run writes nothing
    bad = subprocess.run(['bash', ROOT / 'install.sh', '--dry-run', '--components', 'core,warp'], env=env,
                         capture_output=True, text=True, timeout=60)
    assert bad.returncode == 2 and 'unknown component warp' in bad.stderr


def test_help_yaml_and_cli_md_are_generated_and_keep_astros_names(tmp_path, capsys):
    assert cli.main(['help', '--check']) == 0, capsys.readouterr().err
    data = yaml.safe_load((ROOT / 'help.yaml').read_text())
    names = [c['name'] for c in data['commands']]
    assert set(names) >= {'crane doctor', 'crane run', 'crane view', 'crane board', 'crane fit', 'crane compare',
                          'crane sim check', 'crane bench', 'crane build player', 'crane card', 'crane field import'}
    for command in data['commands']:
        words = shlex.split(command['run'].replace('${CRANE_DIR:-$HOME/crane_ml}', str(ROOT)))
        assert words[0] == str(LAUNCHER) and command['name'].split() == ['crane', *words[1:]]
        assert reg.find(words[1:]) or reg.noun(words[1]).export, command['name']
        for param in command.get('params') or []:
            assert param['name'] and set(param) <= {'name', 'type', 'default', 'choices', 'description',
                                                    'required', 'form'}
    # a drifted file is named with the fix
    (tmp_path / 'Docs').mkdir()
    shutil.copy(ROOT / 'Docs' / 'CLI.md', tmp_path / 'Docs' / 'CLI.md')
    (tmp_path / 'help.yaml').write_text((ROOT / 'help.yaml').read_text().replace('summary: Sim and', 'summary: Sim'))
    assert reg.check(root=tmp_path) == 1
    assert 'help.yaml differ from crane/commands.py; fix: crane help --write' in capsys.readouterr().err


@pytest.mark.skipif(not shutil.which('bash'), reason='bash')
def test_sim_up_check_reaches_crane_and_a_bad_current_is_refused_before_anything_starts(tmp_path):
    env = {**os.environ, 'SIM_UP_RUN': str(tmp_path / 'run'), 'HEAVY': 'none'}
    check = subprocess.run([paths.SIM_UP, 'check'], env=env, capture_output=True, text=True, timeout=30)
    assert check.returncode == 1 and 'sim check: not ready' in check.stdout  # nothing started under this SIM_UP_RUN
    bad = subprocess.run([paths.SIM_UP, 'up', '--current', 'fast'], env=env, capture_output=True, text=True,
                         timeout=30)
    assert bad.returncode == 2 and '--current wants SPEED,TOWARD_DEG' in bad.stderr
    assert not (tmp_path / 'run').exists()


def test_env_gen_checks_its_arguments_before_taking_the_unity_slot(monkeypatch, capsys):
    from crane import wrap
    monkeypatch.setattr(wrap, 'execute', lambda cmd, **kw: pytest.fail(f'ran {cmd}'))
    assert cli.main(['env', 'gen']) == 2 and '--out is required' in capsys.readouterr().err
    assert cli.main(['env', 'gen', '--seed', '3', '--seeds', '0:2', '--out', '/tmp/x']) == 2
    assert cli.main(['env', 'gen', '--seed', '3', '--out', '/tmp/c3.json', '--dry-run']) == 0
    out = capsys.readouterr().out
    assert 'unity' in out and 'generate.sh --seed 3 --out /tmp/c3.json' in out
