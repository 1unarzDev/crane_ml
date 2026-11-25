"""Builds, release, update, deploy and setup: a fake release through crane update into a temp home's player store
(current + prev, pruning, the sha256 and disk refusals); crane deploy over the local transport (no ssh) to a host
without a checkout and to one with a git checkout; the setup wizard's non-interactive plan; the image and release
workflows as data. No docker builds, no network, no Unity."""

import io
import json
import os
import shutil
import subprocess
import sys
import tarfile
from pathlib import Path

import pytest
import yaml

from crane import build, paths, update

ROOT = paths.ROOT
CRANE = str(ROOT / 'bin' / 'crane')


def _player(tmp, commit):
    """A fake CRANE-Worker build packed as crane-player-<commit12>-linux-x86_64.tar.* with its .sha256."""
    src = tmp / f'build-{commit[:6]}' / 'CRANE-Worker'
    (src / 'CRANE_Data').mkdir(parents=True)
    (src / 'CRANE.x86_64').write_text('#!/bin/sh\necho fake player\n')
    (src / 'CRANE.x86_64').chmod(0o755)
    (src / 'CRANE_Data' / 'level0').write_bytes(os.urandom(4096))
    (src / 'crane-build-manifest.json').write_text(json.dumps({'sourceCommit': commit, 'unityVersion': 'U1'}))
    return build.make_tarball(src, tmp, commit)


def _release(base, version, tarball, **player_changes):
    """A release laid out like GitHub's (BASE/latest/download/ and BASE/download/vV/) holding tarball."""
    doc = update.make_release(version, f'v{version}', 'c' * 40, 'U1', player=tarball)
    doc['player'].update(player_changes)
    for d in (base / 'latest' / 'download', base / 'download' / f'v{version}'):
        d.mkdir(parents=True, exist_ok=True)
        shutil.copy2(tarball, d / tarball.name)
        (d / 'release.json').write_text(json.dumps(doc))
    return doc


def _env(home, releases):
    path = f'{Path(sys.executable).parent}{os.pathsep}{os.environ.get("PATH", "")}'   # bin/crane's python3
    return {**os.environ, 'HOME': str(home), 'CRANE_RELEASES': str(releases), 'PATH': path,
            'CRANE_PLAYER': '', 'PYTHONDONTWRITEBYTECODE': '1'}


def _crane(env, *argv, cwd=None):
    return subprocess.run([CRANE, *argv], capture_output=True, text=True, env=env, cwd=cwd or ROOT, timeout=300)


@pytest.fixture
def home(tmp_path):
    h = tmp_path / 'home'
    h.mkdir()
    return h


@pytest.mark.skipif(not shutil.which('zstd'), reason='zstd packs the player tarball')
def test_update_installs_release_players_keeping_current_and_prev_and_refuses_bad_ones(tmp_path, home):
    store = home / '.local' / 'share' / 'crane' / 'player'
    rel = tmp_path / 'releases'
    env = _env(home, rel)
    names = []
    for i, commit in enumerate(('a' * 40, 'b' * 40, 'd' * 40)):
        tarball = _player(tmp_path, commit)
        _release(rel, f'0.9.{i}', tarball)
        plan = _crane(env, 'update', '--player-only', '--dry-run', '--json')
        assert plan.returncode == 0, plan.stderr
        assert json.loads(plan.stdout)['player_store']['current'] == (names[-1] if names else None)
        done = _crane(env, 'update', '--player-only', '--json')
        assert done.returncode == 0, done.stderr
        names.append(commit[:12])
        state = json.loads(done.stdout)['player_store']
        assert (state['current'], state['prev']) == (names[-1], names[-2] if len(names) > 1 else None)
        assert (store / 'current' / 'CRANE.x86_64').read_text().startswith('#!/bin/sh')
    assert sorted(update.store_state(store)['players']) == sorted(names[-2:])          # the oldest was pruned
    assert not list((home / '.cache' / 'crane' / 'downloads').rglob('*.tar.*'))       # downloads are removed

    # a release whose tarball does not match its sha256: refused (exit 1), the store untouched, nothing left over
    bad = _player(tmp_path, 'e' * 40)
    _release(rel, '0.9.9', bad, sha256='0' * 64)
    refused = _crane(env, 'update', '--player-only')
    assert refused.returncode == 1 and 'sha256' in refused.stderr and len(refused.stderr.strip().splitlines()) == 1
    assert update.store_state(store)['current'] == names[-1]
    assert not list((home / '.cache' / 'crane' / 'downloads').rglob('*.part'))

    # not enough disk for it: one line, exit 2, nothing fetched
    _release(rel, '0.9.9', bad, unpacked_bytes=10 ** 16)
    short = _crane(env, 'update', '--player-only')
    assert short.returncode == 2 and 'free' in short.stderr and 'fix:' in short.stderr
    assert update.store_state(store)['current'] == names[-1]


FAKE_UV = r'''#!/bin/sh
# uv for the deploy test: `venv ... DIR` links DIR/bin/python to the test's python; `pip install --python PY -e SPEC`
# writes DIR/bin/crane running the checkout's bin/crane. No network.
case $1 in
  venv) for a; do d=$a; done; mkdir -p "$d/bin"; ln -sfn "$FAKE_PY" "$d/bin/python" ;;
  pip) while [ "$1" != --python ]; do shift; done; v=$(dirname "$(dirname "$2")")
       printf '#!/bin/sh\nexec %s/bin/crane "$@"\n' "$(dirname "$v")" >"$v/bin/crane"; chmod +x "$v/bin/crane" ;;
esac
'''


@pytest.mark.skipif(not (shutil.which('zstd') and shutil.which('git')), reason='zstd and git')
def test_deploy_over_the_local_transport_to_a_bare_host_and_to_a_git_checkout(tmp_path, home):
    tools = tmp_path / 'tools'
    tools.mkdir()
    (tools / 'uv').write_text(FAKE_UV)
    (tools / 'uv').chmod(0o755)
    env = _env(home, tmp_path / 'no-releases')
    env.update(PATH=f'{tools}{os.pathsep}{env["PATH"]}', FAKE_PY=sys.executable)
    head = subprocess.run(['git', '-C', str(ROOT), 'rev-parse', 'HEAD'], capture_output=True, text=True).stdout.strip()
    tarball = _player(tmp_path, 'f' * 40)

    # a host with nothing on it: HEAD's files (no Unity project) by rsync, its .venv (install.sh), the player, doctor
    bare = tmp_path / 'bare'
    bare.mkdir()
    plan = _crane(env, 'deploy', f'local:{bare}', '--tarball', str(tarball), '--dry-run', '--json')
    assert plan.returncode == 0, plan.stderr
    assert [s['step'] for s in json.loads(plan.stdout)['steps']] == ['code', 'venv', 'player', 'doctor']
    assert not (bare / 'crane_ml').exists()
    done = _crane(env, 'deploy', f'local:{bare}', '--tarball', str(tarball), '--json')
    doc = json.loads(done.stdout)
    assert done.returncode == 0, done.stderr
    assert (bare / 'crane_ml' / 'bin' / 'crane').is_file() and not (bare / 'crane_ml' / 'Assets').exists()
    assert json.loads((bare / 'crane_ml' / '.crane-deploy.json').read_text())['commit'] == head
    assert [s['step'] for s in doc['steps']] == ['code', 'venv', 'player']
    assert (bare / 'crane_ml' / '.venv' / 'bin' / 'python').exists() and (bare / '.local' / 'bin' / 'crane').exists()
    assert doc['steps'][2]['store']['current'] == 'f' * 12
    assert any(c['name'] == 'player' for c in doc['doctor']['checks'])
    assert sorted(p.name for p in bare.iterdir()) == ['.local', 'crane_ml']        # nothing else on the host
    assert not list((bare / '.local' / 'share' / 'crane' / 'player' / '.incoming').iterdir())

    # a tarball whose .sha256 lies: the host refuses it, its player stays
    liar = _player(tmp_path, '1' * 40)
    liar.with_name(liar.name + '.sha256').write_text(f'{"0" * 64}  {liar.name}\n')
    refused = _crane(env, 'deploy', f'local:{bare}', '--tarball', str(liar))
    assert refused.returncode == 1 and 'sha256' in refused.stderr
    assert update.store_state(bare / '.local' / 'share' / 'crane' / 'player')['current'] == 'f' * 12

    # the player when there is no Builds tarball is the release's: too big for the host's disk, refused
    _release(tmp_path / 'releases', '0.9.0', _player(tmp_path, '2' * 40), unpacked_bytes=10 ** 16)
    short = _crane({**env, 'CRANE_RELEASES': str(tmp_path / 'releases')}, 'deploy', f'local:{bare}', '--player')
    if not list(paths.BUILDS.glob('crane-player-*.tar.*')):
        assert short.returncode == 2 and 'free' in short.stderr

    # a host with a git checkout one commit behind: fetch from here and fast-forward
    host = tmp_path / 'host'
    repo = host / 'crane_ml'
    subprocess.run(['git', 'clone', '--quiet', '--shared', '--no-checkout', str(ROOT), str(repo)], check=True)
    subprocess.run(['git', '-C', str(repo), 'sparse-checkout', 'set', 'crane', 'bin', 'config'], check=True)
    subprocess.run(['git', '-C', str(repo), 'checkout', '--quiet', '-B', 'deployed', f'{head}~1'], check=True)
    ff = _crane(env, 'deploy', f'local:{host}', '--no-venv', '--json')
    assert ff.returncode == 0, ff.stderr
    assert subprocess.run(['git', '-C', str(repo), 'rev-parse', 'HEAD'], capture_output=True,
                          text=True).stdout.strip() == head
    assert json.loads(ff.stdout)['steps'][0]['how'] == 'git'

    # a ~/crane_ml that is neither a crane checkout nor an earlier deploy: refused, untouched
    other = tmp_path / 'other'
    (other / 'crane_ml').mkdir(parents=True)
    (other / 'crane_ml' / 'notes.txt').write_text('mine\n')
    foreign = _crane(env, 'deploy', f'local:{other}')
    assert foreign.returncode == 2 and 'not a crane checkout' in foreign.stderr
    assert [p.name for p in (other / 'crane_ml').iterdir()] == ['notes.txt']


def test_deploy_refuses_robot_computers_from_astros_registry_and_by_name_without_it(tmp_path, home):
    from crane import deploy
    reg = tmp_path / 'profiles'
    (reg / 'vessel').mkdir(parents=True)
    (reg / 'vessel' / 'devices.yaml').write_text(
        'hosts:\n'
        '  hull-brain: {roles: [boat_computer], user: r}\n'
        '  shore: {roles: [operator, gateway], address: $SHORE_IP}   # the station: allowed\n'
        '  eyes: {roles: [camera_host], address: 10.0.0.7}\n')
    (reg / 'vessel' / 'network.env').write_text('SHORE_IP=10.0.0.2\n')
    (reg / 'sim').mkdir()
    (reg / 'sim' / 'devices.yaml').write_text('hosts:\n  localhost: {roles: [boat_computer, operator]}\n')
    env = {**_env(home, tmp_path / 'no-releases'), 'ASTRO_PROFILES': str(reg)}

    # refused before anything runs (no ssh): one line naming the fix
    for host in ('hull-brain', 'r@hull-brain', '10.0.0.7'):
        out = _crane(env, 'deploy', host, '--dry-run')
        name = host.rsplit('@', 1)[-1]
        assert (out.returncode, out.stderr) == (
            2, f'crane deploy: {name} is a robot computer; robot code goes through `astro hot`\n'), host
    # the station (by name or its $KEY address) and a host the registry does not list are allowed
    cfg = deploy.guard_config()
    for host in ('shore', 'me@10.0.0.2', 'localhost', 'innovation', 'spare-jetson', 'local:/tmp/h'):   # roles only
        assert deploy.robot_guard(host, cfg, {'ASTRO_PROFILES': str(reg)}) == (None, []), host
    # no registry: a warning, never a refusal (the decision is by registry role only)
    gone = {'ASTRO_PROFILES': str(tmp_path / 'missing')}
    refusal, warnings = deploy.robot_guard('hull-odroid', cfg, gone)
    assert refusal is None and len(warnings) == 1 and 'not checked for a robot role' in warnings[0]


@pytest.mark.skipif(not shutil.which('zstd'), reason='zstd packs the player tarball')
def test_crafted_tarballs_and_mismatched_release_players_are_refused(tmp_path):
    def crafted(name, *members):
        path = tmp_path / f'{name}.tar.gz'
        with tarfile.open(path, 'w:gz') as tar:
            for m, kind, link in (('CRANE-Worker/CRANE.x86_64', tarfile.REGTYPE, ''), *members):
                info = tarfile.TarInfo(m)
                info.type, info.linkname = kind, link
                tar.addfile(info, None if kind != tarfile.REGTYPE else io.BytesIO(b''))
        return path
    cases = [crafted('up', ('CRANE-Worker/../evil', tarfile.REGTYPE, '')),
             crafted('abs-sym', ('CRANE-Worker/s', tarfile.SYMTYPE, '/etc/passwd')),
             crafted('up-sym', ('CRANE-Worker/d/s', tarfile.SYMTYPE, '../../x')),
             crafted('hard-out', ('CRANE-Worker/h', tarfile.LNKTYPE, 'etc/passwd')),
             crafted('hard-later', ('CRANE-Worker/h', tarfile.LNKTYPE, 'CRANE-Worker/later'))]
    store = tmp_path / 'store'
    for path in cases:
        with pytest.raises(update.IntegrityError):
            update.install_player(path, update.sha256_file(path), store)
    assert update.store_state(store)['current'] is None and not (tmp_path / 'evil').exists()

    # release-json refuses a player built from another commit, or with another Unity
    tarball = _player(tmp_path, 'a' * 40)
    def release_json(commit, unity):
        return subprocess.run([sys.executable, '-m', 'crane.update', 'release-json', '--version', '0.9.0', '--tag',
                               'v0.9.0', '--commit', commit, '--unity', unity, '--player', str(tarball), '--out',
                               str(tmp_path / 'release.json')], capture_output=True, text=True, cwd=ROOT)
    assert release_json('b' * 40, 'U1').returncode == 1 and not (tmp_path / 'release.json').exists()
    assert release_json('a' * 40, 'U2').returncode == 1 and not (tmp_path / 'release.json').exists()
    ok = release_json('a' * 40, 'U1')
    assert ok.returncode == 0, ok.stderr
    assert json.loads((tmp_path / 'release.json').read_text())['player']['source_commit'] == 'a' * 40


def test_setup_plans_without_asking_or_writing(tmp_path, home):
    env = _env(home, tmp_path / 'no-releases')
    proc = _crane(env, 'setup', '--yes', '--dry-run', '--json', '--components', 'fast,player,astro',
                  '--domains', 'boat')
    assert proc.returncode == 0, proc.stderr
    doc = json.loads(proc.stdout)
    assert doc['components'] == ['core', 'fast', 'player', 'astro'] and doc['domains'] == ['boat']
    steps = {s['step']: s for s in doc['steps']}
    assert steps['install']['argv'][-1] == 'core,fast' and '--yes' in steps['install']['argv']
    assert steps['player']['argv'][-2:] == ['update', '--player-only']
    assert set(steps) >= {'astro', 'save', 'doctor'} and doc['total_mb'] > 0
    assert not (home / '.config').exists() and not (home / '.local').exists()
    bad = _crane(env, 'setup', '--yes', '--components', 'core,astro-images')
    assert bad.returncode == 2 and 'unknown component' in bad.stderr
    piped = subprocess.run([CRANE, 'setup'], capture_output=True, text=True, env=env, stdin=subprocess.DEVNULL)
    assert piped.returncode == 2 and '--yes' in piped.stderr
    planned = subprocess.run([CRANE, 'setup', '--dry-run'], capture_output=True, text=True, env=env,
                             stdin=subprocess.DEVNULL)
    assert 'no terminal' not in planned.stderr and 'components' in planned.stdout


def test_image_and_release_workflows_and_rotation():
    wf = ROOT / '.github' / 'workflows'
    images, release = (yaml.safe_load((wf / f'{n}.yml').read_text()) for n in ('images', 'release'))
    on = images[True]                                  # YAML 1.1 reads the `on` key as True
    assert {'push', 'workflow_dispatch', 'workflow_call'} <= set(on) and 'pull_request' not in on
    assert images['concurrency']['group'] == 'docker-hub-mutations'
    assert release['concurrency']['group'] != images['concurrency']['group']    # a called workflow would deadlock
    steps = images['jobs']['image']['steps']
    registry = [s for s in steps if 'docker push' in s.get('run', '') or 'imagetools' in s.get('run', '')
                or s.get('uses', '').startswith('docker/login')]
    assert registry and all("env.HAVE_HUB == 'true'" in s['if'] for s in registry)
    text = (wf / 'images.yml').read_text() + (wf / 'release.yml').read_text()
    assert ':latest' not in text.replace("never :latest", '')
    assert release['jobs']['sim-image']['uses'] == './.github/workflows/images.yml'
    rsteps = ' '.join(s.get('run', '') for s in release['jobs']['release']['steps'])
    assert 'pyproject' in rsteps and 'release-json' in rsteps and 'sha256sum -c' in rsteps
    for f in ('Dockerfile.core', 'Dockerfile.gpu', 'Dockerfile.sim'):
        text = (ROOT / f).read_text()
        assert 'ENTRYPOINT' not in text.replace('no blocking ENTRYPOINT', '').replace('ENTRYPOINT.', '')
        assert '/opt/crane_ml' in text or 'CORE_IMAGE' in text
    assert subprocess.run(['bash', '-n', str(ROOT / 'docker' / 'smoke.sh')]).returncode == 0

    sys.path.insert(0, str(ROOT / 'docker'))
    import registry as reg
    tags = [{'name': f'core-{i:040x}', 'last_updated': f'2026-11-{i + 1:02d}'} for i in range(5)]
    tags += [{'name': 'core-v0.2.0', 'last_updated': '2026-01-01'}, {'name': 'core-next', 'last_updated': '2026'},
             {'name': f'gpu-{0:040x}', 'last_updated': '2026-01-01'}]
    assert reg.pins_to_drop(tags, 'core', 2) == [f'core-{i:040x}' for i in (2, 1, 0)]
