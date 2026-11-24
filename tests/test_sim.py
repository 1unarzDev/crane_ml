"""crane sim and crane.player: the Python sim against the bash sim_up it replaces, run side by side on stubbed
docker and heavy (no Unity, no ROS); the domain-id rules; check and status on a fake state dir; the shim."""

import json
import os
import re
import shutil
import signal
import socket
import subprocess
import sys
import time

import pytest

from crane import paths, player, sim
from crane.sim import check, config, world

ROOT = paths.ROOT
REFERENCE = '20c29bbf'          # main, where Tools/RoboBoat/sim_up was the bash implementation
FLAG = '--crane-roboboat-contract'

DOCKER = r'''#!/bin/sh
case $1 in
  inspect) [ -f "$STUB/running" ] && echo true || { echo "Error: no such object" >&2; exit 1; } ;;
  run) shift; printf '%s\n' "$@" > "$STUB/docker_run.argv"; touch "$STUB/running"; echo cid ;;
  rm) rm -f "$STUB/running" ;;
  exec) printf '/ekf_local\n/rtabmap\n/bt_navigator\n' ;;
esac
'''
HEAVY = r'''#!/bin/sh
printf '%s\n' "$@" > "$STUB/heavy.argv"
echo "$HEAVY_WHO" > "$STUB/heavy.who"
prev=; for a; do [ "$prev" = -logFile ] && echo CRANE_ROBOBOAT_COURSE_READY >> "$a"; prev=$a; done
exec sleep 60
'''


def _exe(path, text):
    path.write_text(text)
    path.chmod(0o755)
    return path


def _free_domain():
    """A ROS_DOMAIN_ID whose endpoint port (10000 + id) is free, with a socket listening on it (the endpoint)."""
    for d in range(150, 230):
        s = socket.socket()
        try:
            s.bind(('127.0.0.1', 10000 + d))
        except OSError:
            s.close()
            continue
        s.listen()
        return d, s
    pytest.skip('no free port in 10150..10229')


@pytest.fixture
def world_env(tmp_path, monkeypatch):
    """Stubbed docker/heavy/xvfb-run, a fake player that knows the flag, a sim vessel profile, an endpoint."""
    stub = tmp_path / 'stub'
    bin_ = stub / 'bin'
    bin_.mkdir(parents=True)
    _exe(bin_ / 'docker', DOCKER)
    _exe(bin_ / 'xvfb-run', '#!/bin/sh\nexec "$@"\n')
    heavy = _exe(stub / 'heavy', HEAVY)
    play = tmp_path / 'player' / 'CRANE.x86_64'
    (play.parent / 'CRANE_Data' / 'Managed').mkdir(parents=True)
    _exe(play, '#!/bin/sh\nexit 0\n')
    (play.parent / 'CRANE_Data' / 'Managed' / 'Assembly-CSharp.dll').write_bytes(b'MZ\0' + FLAG.encode('utf-16-le'))
    d, sock = _free_domain()
    (tmp_path / 'profiles' / 'sim').mkdir(parents=True)
    (tmp_path / 'profiles' / 'sim' / 'network.env').write_text(f'LAN_SUBNET=127.0.0.0/8\nROS_DOMAIN_ID={d}\n')
    (tmp_path / 'home').mkdir()
    (tmp_path / 'dock').mkdir()
    card = tmp_path / 'card.json'
    card.write_text('{}')
    env = {k: v for k, v in os.environ.items()
           if k not in ('ROS_DOMAIN_ID', 'ASTRO_STATE', 'MHSEALS_HARDWARE_CONFIG', 'DISPLAY', 'WAYLAND_DISPLAY',
                        'ASTRO_PROFILES_CONFIG', 'SIM_UP_VESSEL', 'SIM_UP_READY_S', 'PYTHONPATH')}
    env.update(HOME=str(tmp_path / 'home'), SIM_UP_RUN=str(tmp_path / 'run'), ASTRO_DOCK=str(tmp_path / 'dock'),
               ASTRO_PROFILES=str(tmp_path / 'profiles'), ASTRO_IMAGE='dev:1', HEAVY=str(heavy),
               CRANE_PLAYER=str(play), STUB=str(stub), PATH=f'{bin_}{os.pathsep}{os.environ["PATH"]}')
    for k in list(os.environ):
        if k not in env:
            monkeypatch.delenv(k)
    for k, v in env.items():
        monkeypatch.setenv(k, v)
    yield {'env': env, 'stub': stub, 'run': tmp_path / 'run', 'domain': d, 'player': play, 'card': card,
           'tmp': tmp_path}
    sock.close()
    pid = (tmp_path / 'run' / 'player.pid')
    if pid.is_file():                       # a failed test leaves the stub's sleep behind otherwise
        try:
            os.killpg(int(pid.read_text()), signal.SIGKILL)
        except (OSError, ValueError):
            pass


def _sh(argv, env, timeout=60):
    out = subprocess.run(argv, env=env, stdin=subprocess.DEVNULL, capture_output=True, text=True, timeout=timeout)
    return out.returncode, out.stdout, out.stderr


def _bash_sim_up(tmp):
    """The reference bash sim_up in a copy of its repo layout (config/sim.yaml beside it)."""
    try:
        script = subprocess.run(['git', 'show', f'{REFERENCE}:Tools/RoboBoat/sim_up'], cwd=ROOT, capture_output=True,
                                check=True).stdout
        sim_yaml = subprocess.run(['git', 'show', f'{REFERENCE}:config/sim.yaml'], cwd=ROOT, capture_output=True,
                                  check=True).stdout
    except (OSError, subprocess.CalledProcessError):
        pytest.skip(f'no git history with {REFERENCE} (a shallow or exported checkout)')
    if not (shutil.which('strings') and shutil.which('ss') and shutil.which('setsid')):
        pytest.skip('the bash sim_up needs strings, ss and setsid')
    repo = tmp / 'ref'
    (repo / 'Tools' / 'RoboBoat').mkdir(parents=True)
    (repo / 'config').mkdir()
    (repo / 'config' / 'sim.yaml').write_bytes(sim_yaml)
    path = repo / 'Tools' / 'RoboBoat' / 'sim_up'
    path.write_bytes(script)
    path.chmod(0o755)
    return path


def _norm(text):
    return re.sub(r'pgid \d+', 'pgid N', text)


def _without_workdir(argv):
    i = argv.index('-w')
    return argv[:i] + argv[i + 2:], argv[i + 1]


@pytest.mark.parametrize('args', [[], ['--window'], ['--seed', '3', '--course', 'pool', '--current', '0',
                                                       '--profile-json', 'CARD', '--headless'],
                                  ['up', '--current', '0.3,45', '--seed', '7']],
                         ids=['bare-headless', 'window', 'seed-current0-card', 'up-current'])
def test_crane_sim_runs_the_world_exactly_as_the_bash_sim_up_did(world_env, args):
    """Same docker run, same player command line (guard, display, nice, player argv), same stdout, same status
    block, and each one's down stops the other's world."""
    env, stub, run = world_env['env'], world_env['stub'], world_env['run']
    args = [str(world_env['card']) if a == 'CARD' else a for a in args]
    bash = _bash_sim_up(world_env['tmp'])
    shim = ROOT / 'Tools' / 'RoboBoat' / 'sim_up'
    seen = {}
    for name, cmd in (('bash', [str(bash)]), ('python', [str(shim)])):
        rc, out, err = _sh([*cmd, *args], env)
        assert rc == 0, (name, out, err)
        docker = (stub / 'docker_run.argv').read_text().splitlines()
        heavy = (stub / 'heavy.argv').read_text().splitlines()
        who = (stub / 'heavy.who').read_text().strip()
        statuses = [_sh([str(bash), 'status'], env)[1], _sh([str(shim), 'status'], env)[1]]
        assert statuses[0] == statuses[1]                 # either status reads either up's state
        assert 'course:    ready' in statuses[0] and 'next:      astro on sim' in statuses[0]
        other = shim if name == 'bash' else bash
        rc, down, _ = _sh([str(other), 'down'], env)
        assert rc == 0 and down == 'sim_up: down\n' and not (stub / 'running').exists()
        assert not (run / 'player.pid').exists() and not (run / 'port').exists()
        seen[name] = (_norm(out), _without_workdir(docker), heavy, who, _norm(statuses[0]))
    assert seen['bash'] == seen['python']
    heavy = seen['python'][2]
    port = 10000 + world_env['domain']
    headless = '--window' not in args
    assert heavy[:2] == ['unity', 'env'] and ('xvfb-run' in heavy) == headless
    tail = heavy[heavy.index(str(world_env['player'])) + 1:]
    seed = args[args.index('--seed') + 1] if '--seed' in args else '1'
    current = args[args.index('--current') + 1] if '--current' in args else None
    card = str(world_env['card']) if '--profile-json' in args else None
    course = 'course:pool' if '--course' in args else None
    assert tail == player.resolve(player.load_domain('boat'), seed=int(seed), current=current, card=card, env=course,
                                  ros={'port': port}, out=run)


def test_resolve_gives_the_boat_player_argv_from_the_domain_config(tmp_path):
    cfg = player.load_domain('boat')
    card = tmp_path / 'c.json'
    card.write_text('{}')
    base = ['-screen-fullscreen', '0', '-screen-width', '1280', '-screen-height', '720', '-logFile', '/r/player.log',
            '--crane-profile', 'interactive-low', '--crane-scene', 'Roboboat Course',
            '--crane-ros-ip', '127.0.0.1', '--crane-ros-port', '10101',
            '--crane-disable-sitl', '--crane-ros-thruster-pwm', '--crane-roboboat-contract', '--crane-ros-nav-state',
            '--crane-lidar-returns-only', '--crane-lidar-water-level', '0.02',
            '--crane-course-family', 'roboboat_2026', '--crane-course-seed', '1',
            '--crane-roboboat-course-truth', '/r/course_truth.json']
    assert player.resolve(cfg, ros={'port': 10101}, out='/r') == base
    assert player.resolve(cfg, tier='unity', seed=1, current='0', card=str(card), ros={'port': 10101}, out='/r') == \
        base + ['--crane-boat-profile-json', str(card.resolve()), '--crane-water-current', '0,0']
    # a card by name from card.dir, a set, a runtime profile, extra args last
    named = player.resolve(cfg, card='ideal', sets=['gates.count=2'], profile='full', extra=['--x'], seed=2)
    assert named[named.index('--crane-boat-profile-json') + 1] == \
        str(ROOT / 'Tools' / 'Characterization' / 'profiles' / 'ideal.json')
    assert named[-3:] == ['--crane-course-set', 'gates.count=2', '--x'] and 'full' in named
    assert '-logFile' not in named and '--crane-ros-port' not in named
    for bad, needle in (({'current': '1,x'}, 'SPEED,TOWARD_DEG'), ({'env': 'world:x'}, 'KIND one of course'),
                        ({'robot': 'other'}, 'robots: blastoise'), ({'card': 'nope'}, 'no card nope'),
                        ({'tier': 'T1'}, 'player tier is T2')):
        with pytest.raises(player.ConfigError, match=needle):
            player.resolve(cfg, **bad)
    with pytest.raises(player.ConfigError, match='no tiers.T2'):
        player.resolve(player.load_domain('toy'))
    # crane doctor's flat required_flag is the boat world's
    assert config.default_domain() == 'boat'
    assert f'required_flag: {cfg["tiers"]["T2"]["required_flag"]}' in (paths.CONFIG / 'sim.yaml').read_text()


def test_the_ros_domain_id_comes_from_astro_or_the_sim_profile_never_a_bare_shell(tmp_path, monkeypatch):
    t2 = player.load_domain('boat')['tiers']['T2']
    for k in ('ROS_DOMAIN_ID', 'ASTRO_STATE', 'MHSEALS_HARDWARE_CONFIG', 'ASTRO_PROFILES', 'SIM_UP_VESSEL'):
        monkeypatch.delenv(k, raising=False)
    monkeypatch.setenv('HOME', str(tmp_path))
    monkeypatch.setenv('ASTRO_DOCK', str(tmp_path / 'dock'))
    monkeypatch.setenv('ASTRO_PROFILES_CONFIG', str(tmp_path / 'profiles.env'))
    for root, d in (('dock/profiles', 7), ('mine', 9)):
        (tmp_path / root / 'sim').mkdir(parents=True)
        (tmp_path / root / 'sim' / 'network.env').write_text(f'ROS_DOMAIN_ID=1\n# x\nROS_DOMAIN_ID={d}\n')
    assert config.domain_id(t2) == 7                                   # the dock's profiles, last line wins
    (tmp_path / 'profiles.env').write_text('ASTRO_PROFILES=/elsewhere\nASTRO_PROFILES=~/mine\n')
    assert config.domain_id(t2) == 9                                   # profiles.env, ~ expanded
    monkeypatch.setenv('ROS_DOMAIN_ID', '42')
    assert config.domain_id(t2) == 9                                   # a bare shell's id is not trusted
    monkeypatch.setenv('ASTRO_STATE', '~/.config/mhseals/hardware_sim.yaml')
    assert config.domain_id(t2) == 42                                  # astro's is
    monkeypatch.delenv('ASTRO_STATE')
    monkeypatch.setenv('ASTRO_PROFILES', str(tmp_path / 'none'))
    with pytest.raises(config.SimError, match='none/sim/network.env: the sim vessel profile names'):
        config.domain_id(t2)
    (tmp_path / 'none' / 'sim').mkdir(parents=True)
    (tmp_path / 'none' / 'sim' / 'network.env').write_text('ROS_DOMAIN_ID=\n')
    with pytest.raises(config.SimError, match='has no ROS_DOMAIN_ID'):
        config.domain_id(t2)


def test_check_status_and_refusals_on_a_fake_state_dir(world_env, monkeypatch, capsys):
    run = world_env['run']
    run.mkdir()
    # a player stand-in in its own process group, not our child (so down sees it go, as with a real up)
    spawn = ('import subprocess as s; d = s.DEVNULL; '
             'print(s.Popen(["sleep", "30"], start_new_session=True, stdin=d, stdout=d, stderr=d).pid)')
    pid = int(subprocess.run([sys.executable, '-c', spawn], capture_output=True, text=True, check=True).stdout)
    try:
        (run / 'player.pid').write_text(f'{pid}\n')
        (run / 'player.path').write_text(str(world_env['player']))
        (run / 'player.log').write_text('CRANE_ROBOBOAT_COURSE_READY\nCRANE_PLANT_APPLIED card=x\n'
                                        'CRANE_DOCK_FAILED\nNullReferenceException: at Foo\n')
        (run / 'current').write_text('0.3,45\n')
        (world_env['stub'] / 'running').touch()
        monkeypatch.setattr(check, 'RUN', run)
        assert check.main(['--json']) == 1
        out = json.loads(capsys.readouterr().out)
        rows = {r['name']: r for r in out['checks']}
        assert out['failed'] == ['no player errors'] and 'NullReferenceException' in rows['no player errors']['detail']
        assert rows['nav stack up']['ok'] and rows['plant applied']['detail'] == 'CRANE_PLANT_APPLIED card=x'
        assert out['notices'] == ['CRANE_DOCK_FAILED']
        (run / 'player.log').write_text('CRANE_ROBOBOAT_COURSE_READY\nCRANE_PLANT_APPLIED\n')
        assert check.main(['--wait', '0']) == 0 and capsys.readouterr().out.endswith('sim check: ready\n')
        assert check.main(['--wait']) == 2 and 'wants a value' in capsys.readouterr().err
        cfg, t2 = config.domain('boat')
        s = world.state(cfg, t2, run)
        assert s['player']['running'] and s['ros']['running'] and s['course']['ready']
        assert '  current:   0.3 m/s toward 45 deg (--current; the scene default is 0.15 m/s toward 270 deg)' \
            in world.status_lines(s)
        # up refuses while either part runs (status first), and names the fix
        assert sim.main(['up']) == 2
        out, err = capsys.readouterr()
        assert out.startswith('sim_up status') and err == 'sim_up: already up; sim_up down first\n'
        assert sim.main(['logs', 'player', '-n', '1']) == 0
        assert capsys.readouterr().out == f'==> {run}/player.log <==\nCRANE_PLANT_APPLIED\n'
        t0 = time.monotonic()
        assert sim.main(['down']) == 0 and capsys.readouterr().out == 'sim_up: down\n'
        assert time.monotonic() - t0 < 5 and not world.alive(run) and not (world_env['stub'] / 'running').exists()
    finally:
        try:
            os.kill(pid, signal.SIGKILL)
        except OSError:
            pass
    # headless without xvfb-run, and unknown verbs or options: one line, exit 2
    monkeypatch.setattr(world.shutil, 'which', lambda name: None if name == 'xvfb-run' else shutil.which(name))
    assert sim.main(['--headless']) == 2
    assert capsys.readouterr().err == 'sim_up: headless needs xvfb-run: install xvfb or use --window\n'
    assert sim.main(['up', '--current', 'fast']) == 2 and 'SPEED,TOWARD_DEG' in capsys.readouterr().err
    assert sim.main(['launch']) == 2 and sim.main(['up', '--bogus']) == 2


def test_the_sim_up_shim_means_up_when_bare_and_passes_verbs_through(tmp_path):
    repo = tmp_path / 'repo'
    (repo / 'Tools' / 'RoboBoat').mkdir(parents=True)
    shim = repo / 'Tools' / 'RoboBoat' / 'sim_up'
    shutil.copy2(ROOT / 'Tools' / 'RoboBoat' / 'sim_up', shim)
    (repo / 'crane').mkdir()
    (repo / 'crane' / '__init__.py').write_text('')
    (repo / 'crane' / 'cli.py').write_text('import sys\ndef main():\n    print("module", *sys.argv[1:])\n    return 0\n')
    # if the shim ever reached a real crane, this env makes up fail at once instead of starting a world
    env = {k: v for k, v in os.environ.items() if k != 'PYTHONPATH'}
    env.update(HOME=str(tmp_path), HEAVY=str(tmp_path / 'no-heavy'), SIM_UP_RUN=str(tmp_path / 'run'),
               CRANE_PLAYER=str(tmp_path / 'no-player'))
    cases = {(): 'sim up', ('--seed', '3'): 'sim up --seed 3', ('status',): 'sim status',
             ('check', '--wait', '5'): 'sim check --wait 5', ('--help',): 'sim help', ('down',): 'sim down'}
    for how in ('module', 'launcher'):
        if how == 'launcher':
            (repo / 'bin').mkdir()
            _exe(repo / 'bin' / 'crane', '#!/bin/sh\necho launcher "$@"\n')
        for args, want in cases.items():
            out = subprocess.run(['sh', str(shim), *args], env=env, cwd=ROOT, stdin=subprocess.DEVNULL,
                                 capture_output=True, text=True, timeout=60)   # cwd: a checkout with a real crane
            rc, out, err = out.returncode, out.stdout, out.stderr
            assert (rc, out, err) == (0, f'{how} {want}\n', ''), args      # nothing extra; no note off a TTY
