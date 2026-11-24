"""crane sim and crane.player: the Python sim against the bash sim_up it replaces, run side by side on stubbed
docker and heavy (no Unity, no ROS); the domain-id rules; check and status on a fake state dir; the shim."""

import json
import math
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
from crane.sim import check, config, restart as restart_mod, world

ROOT = paths.ROOT
REFERENCE = '20c29bbf'          # main, where Tools/RoboBoat/sim_up was the bash implementation
STATE_REF = 'b8384c43'          # the last bash sim_up: it passes astro's state path into the ROS container
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


def _bash_sim_up(tmp, ref=REFERENCE):
    """The reference bash sim_up in a copy of its repo layout (config/sim.yaml beside it)."""
    try:
        script = subprocess.run(['git', 'show', f'{ref}:Tools/RoboBoat/sim_up'], cwd=ROOT, capture_output=True,
                                check=True).stdout
        # mhseals_nav 6419229 renamed the sim device profile's endpoint part sensors -> sim_bridge (the same ros_port)
        script = script.replace(b'"parts": {"sensors"', b'"parts": {"sim_bridge"').replace(
            b'\\"sensors\\"', b'\\"sim_bridge\\"')
        sim_yaml = subprocess.run(['git', 'show', f'{ref}:config/sim.yaml'], cwd=ROOT, capture_output=True,
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


def test_the_ros_side_gets_astros_state_path_mapped_into_the_container_home(world_env):
    """ASTRO_STATE / MHSEALS_HARDWARE_CONFIG reach the ROS container with a leading $HOME as the container user's
    home (so its Nav2 reads the sim state's nav2_limits), as the last bash sim_up passed them."""
    home = world_env['env']['HOME']
    env = dict(world_env['env'], ASTRO_STATE=f'{home}/.config/mhseals/sim.yaml',
               MHSEALS_HARDWARE_CONFIG='/etc/mhseals/hw.yaml', ROS_DOMAIN_ID=str(world_env['domain']))
    seen = {}
    for name, cmd in (('bash', str(_bash_sim_up(world_env['tmp'], STATE_REF))),
                      ('python', str(ROOT / 'Tools' / 'RoboBoat' / 'sim_up'))):
        rc, out, err = _sh([cmd, 'up', '--headless'], env)
        assert rc == 0, (name, out, err)
        argv = (world_env['stub'] / 'docker_run.argv').read_text().splitlines()
        assert _sh([cmd, 'down'], env)[0] == 0
        seen[name] = argv
        envs = {argv[i + 1] for i, a in enumerate(argv) if a == '-e'}
        assert 'ASTRO_STATE=/home/roboboat/.config/mhseals/sim.yaml' in envs, name
        assert 'MHSEALS_HARDWARE_CONFIG=/etc/mhseals/hw.yaml' in envs, name      # not under $HOME: unchanged
    # the same docker run as the bash one, up to the order of its -e pairs and the workdir flag's place
    def key(argv):
        argv, _ = _without_workdir(argv)
        pairs = sorted(argv[i + 1] for i, a in enumerate(argv) if a == '-e')
        rest = [a for i, a in enumerate(argv) if a != '-e' and (i == 0 or argv[i - 1] != '-e')]
        return pairs, rest
    assert key(seen['bash']) == key(seen['python'])
    assert world.in_container(f'{home}x/y', '/home/roboboat') == f'{home}x/y'     # a path prefix, not a string one


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
    named = player.resolve(cfg, card='ideal', sets=['gates.count=2'], profile='interactive-high', extra=['--x'], seed=2)
    assert named[named.index('--crane-boat-profile-json') + 1] == \
        str(ROOT / 'Tools' / 'Characterization' / 'profiles' / 'ideal.json')
    assert named[-3:] == ['--crane-course-set', 'gates.count=2', '--x'] and 'interactive-high' in named
    assert '-logFile' not in named and '--crane-ros-port' not in named
    for bad, needle in (({'current': '1,x'}, 'SPEED,TOWARD_DEG'), ({'env': 'world:x'}, 'KIND one of course'),
                        ({'robot': 'other'}, 'robots: blastoise'), ({'card': 'nope'}, 'no card nope'),
                        ({'tier': 'T1'}, 'player tier is T2'), ({'profile': 'full'}, 'profiles: custom')):
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


def _group(script):
    """A process group running SCRIPT, not our child (so it is reaped when it goes, as a real player is)."""
    spawn = ('import subprocess as s, sys; d = s.DEVNULL; '
             'print(s.Popen(["sh", "-c", sys.argv[1]], start_new_session=True, stdin=d, stdout=d, stderr=d).pid)')
    return int(subprocess.run([sys.executable, '-c', spawn, script], capture_output=True, text=True,
                              check=True).stdout)


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
        assert out.startswith('sim_up status') and err == 'crane sim: already up; crane sim down first\n'
        assert sim.main(['logs', 'player', '-n', '1']) == 0
        assert capsys.readouterr().out == f'==> {run}/player.log <==\nCRANE_PLANT_APPLIED\n'
        t0 = time.monotonic()
        assert sim.main(['down']) == 0 and capsys.readouterr().out == 'crane sim: down\n'
        assert time.monotonic() - t0 < 5 and not world.alive(run) and not (world_env['stub'] / 'running').exists()
    finally:
        try:
            os.kill(pid, signal.SIGKILL)
        except OSError:
            pass
    # headless without xvfb-run, and unknown verbs or options: one line, exit 2
    monkeypatch.setattr(world.shutil, 'which', lambda name: None if name == 'xvfb-run' else shutil.which(name))
    assert sim.main(['--headless']) == 2                 # options without a verb: usage, never a world
    capsys.readouterr()
    assert sim.main(['up', '--headless']) == 2
    assert capsys.readouterr().err == 'crane sim: headless needs xvfb-run: install xvfb or use --window\n'
    assert sim.main(['up', '--current', 'fast']) == 2 and 'SPEED,TOWARD_DEG' in capsys.readouterr().err
    assert sim.main(['launch']) == 2 and sim.main(['up', '--bogus']) == 2
    # a domain without a T2 world names one that has it
    assert sim.main(['status', '--domain', 'toy']) == 2
    assert capsys.readouterr().err.rstrip().endswith('next: crane sim up --domain boat')


def test_an_agents_up_is_headless_even_with_a_display_and_json_keeps_stdout_clean(world_env):
    """No terminal on stdin (an agent, a script): headless under xvfb-run with Wayland hidden, whatever DISPLAY
    says. --json: progress on stderr, one JSON document on stdout, for up and down."""
    env = dict(world_env['env'], DISPLAY=':0', WAYLAND_DISPLAY='wayland-0')
    crane = [sys.executable, '-c', 'import sys; sys.path.insert(0, sys.argv.pop(1)); from crane.cli import main; '
             'sys.exit(main())', str(ROOT), 'sim']
    rc, out, err = _sh([*crane, 'up', '--json'], env)
    assert rc == 0, err
    heavy = (world_env['stub'] / 'heavy.argv').read_text().splitlines()
    assert heavy[:5] == ['unity', 'env', '-u', 'WAYLAND_DISPLAY', 'SDL_VIDEODRIVER=x11'] and 'xvfb-run' in heavy
    s = json.loads(out)
    assert s['mode'] == 'headless' and s['player']['running'] and s['course']['ready']
    assert 'crane sim: player queued on heavy unity (headless)' in err
    rc, out, err = _sh([*crane, 'down', '--json'], env)
    assert rc == 0 and json.loads(out)['down'] and json.loads(out)['player'] in ('term', 'kill')
    assert not (world_env['stub'] / 'running').exists()


def test_down_gives_the_player_its_grace_then_kills_and_the_endpoint_check_falls_back_to_ss(monkeypatch):
    stubborn, polite = _group('trap "" TERM; while :; do sleep 0.1; done'), _group('exec sleep 30')
    try:
        assert world.GRACE_S >= 10                    # the player gets time to flush its log and truth
        t0 = time.monotonic()
        assert world.stop_group(stubborn, grace_s=1.5) == 'kill' and time.monotonic() - t0 >= 1.5
        assert world.stop_group(polite) == 'term' and world.stop_group(polite) == 'gone'
    finally:
        for pg in (stubborn, polite):
            try:
                os.killpg(pg, signal.SIGKILL)
            except OSError:
                pass
    s = socket.socket()
    s.bind(('127.0.0.1', 0))
    s.listen()
    port = s.getsockname()[1]
    try:
        assert world.listening(port)
        if not shutil.which('ss'):
            pytest.skip('no ss for the fallback')
        real = world.Path.read_text
        def no_proc(self, *a, **k):
            if str(self).startswith('/proc/net/'):
                raise OSError('no /proc')
            return real(self, *a, **k)
        monkeypatch.setattr(world.Path, 'read_text', no_proc)
        assert world.listening(port)
    finally:
        s.close()
    assert not world.listening(port)


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


def test_the_sim_path_needs_no_pyyaml(tmp_path):
    """crane sim reads its domain config with crane.miniyaml when PyYAML is missing; the reader equals PyYAML on
    every config file."""
    import yaml
    from crane import miniyaml
    for path in sorted((ROOT / 'config').rglob('*.yaml')) + [ROOT / 'help.yaml']:
        assert miniyaml.load(path.read_text()) == yaml.safe_load(path.read_text()), path
    with pytest.raises(ValueError, match='install.sh --components core'):
        miniyaml.load('a: |\n  text\n')
    no_yaml = ('import sys; sys.modules["yaml"] = None; sys.path.insert(0, sys.argv.pop(1)); '
               'from crane.cli import main; sys.exit(main())')
    env = {k: v for k, v in os.environ.items() if k != 'PYTHONPATH'}
    env.update(SIM_UP_RUN=str(tmp_path / 'run'), HOME=str(tmp_path), HEAVY='none', CRANE_PLAYER=str(tmp_path / 'no'))
    rc, out, err = _sh([sys.executable, '-c', no_yaml, str(ROOT), 'sim', 'status'], env)
    assert rc == 0 and out.startswith('sim_up status\n') and 'next:' in out, err
    rc, out, err = _sh([sys.executable, '-c', no_yaml, str(ROOT), 'sim', 'up', '--dry-run', '--json',
                        '--headless', '--seed', '4'], env)
    plan = json.loads(out)
    assert rc == 0 and plan['schema'] == 'crane.sim.plan/1' and plan['unresolved'], err   # no player, no profile
    assert '--crane-roboboat-contract' in plan['player_argv'] and 'xvfb-run' in plan['player_argv']


RESTART_DOCKER = r'''#!/bin/sh
printf '%s\n' "$*" >> "$STUB/docker.log"
case $1 in
  inspect) [ -f "$STUB/running" ] && echo true || { echo "Error: no such object" >&2; exit 1; } ;;
  run) shift; printf '%s\n' "$@" > "$STUB/docker_run.argv"; touch "$STUB/running"; echo cid ;;
  rm) rm -f "$STUB/running" ;;
  exec) shift; case "$*" in
    *pgrep*) [ -f "$STUB/nav_alive" ] ;;
    *pkill*) [ -f "$STUB/stubborn" ] || rm -f "$STUB/nav_alive" ;;
    *"lifecycle get"*) cat "$STUB/lifecycle" 2>/dev/null || echo "active [3]" ;;
    *"node list"*) printf '/ekf_local\n/rtabmap\n/bt_navigator\n' ;;
  esac ;;
esac
'''


def _crane_sim(env, *args):
    return _sh([str(ROOT / 'bin' / 'crane'), 'sim', *args], env)


def test_restart_keeps_the_other_parts_and_updates_the_state(world_env, monkeypatch):
    """nav kills only the recipe's nav nodes in the container and relaunches only:=navigation with the same overrides
    and the current state path; ros re-runs the identical docker run, player the identical argv, each keeping the
    others; refusals and bounded waits exit 2, 1 and 75."""
    stub, run, env = world_env['stub'], world_env['run'], dict(world_env['env'])
    _exe(stub / 'bin' / 'docker', RESTART_DOCKER)
    env.update(ROS_DOMAIN_ID=str(world_env['domain']), ASTRO_STATE=f"{env['HOME']}/.config/mhseals/sim.yaml")
    assert _crane_sim(env, 'restart', 'nav')[0] == 2                      # nothing running
    assert _crane_sim(env, 'up', '--headless')[0] == 0
    log = stub / 'docker.log'
    pid, docker_run = (run / 'player.pid').read_text(), (stub / 'docker_run.argv').read_text()
    (stub / 'nav_alive').touch()

    rc, out, err = _crane_sim(env, 'restart', 'nav')
    assert rc == 0, err
    assert out.splitlines()[-1] == 'next: crane sim check' and 'nav restarted' in out
    execs = [l for l in log.read_text().splitlines() if l.split()[0] == 'exec']
    kinds = [next(k for k in ('pkill', 'pgrep', 'ros2 launch', 'lifecycle') if k in l) for l in execs]
    assert kinds[0] == 'pkill' and 'pgrep' in kinds and kinds.index('ros2 launch') > kinds.index('pgrep')
    assert '-TERM -f -- __node:=(controller_server|' in execs[0] and 'lifecycle_manager_navigation' in execs[0]
    start = next(l for l in execs if 'ros2 launch' in l)
    assert start.startswith('exec -d -e ASTRO_STATE=/home/roboboat/.config/mhseals/sim.yaml ')
    assert 'ros2 launch mhseals_nav device.launch.py device:=sim only:=navigation overrides:=' in start
    assert (run / 'player.pid').read_text() == pid and 'nav ' in (run / 'restarted').read_text()
    assert 'restarted: nav ' in _crane_sim(env, 'status')[1]

    # a stuck nav is never started a second time
    cfg, t2 = config.domain('boat')
    (stub / 'stubborn').touch()
    (stub / 'nav_alive').touch()
    before = log.read_text().count('ros2 launch')
    quick = {**t2, 'ros': {**t2['ros'], 'restart': {'nav': {**t2['ros']['restart']['nav'], 'stop_s': 0.2}}}}
    with pytest.raises(config.SimError, match='not starting a second set') as e:
        restart_mod.nav(cfg, quick, run)
    assert e.value.code == 1 and log.read_text().count('ros2 launch') == before
    (stub / 'stubborn').unlink()
    (stub / 'nav_alive').unlink()
    # a nav that never goes active times out with 75, naming the node and the timeout
    (stub / 'lifecycle').write_text('inactive [2]\n')
    slow = {**t2, 'ros': {**t2['ros'], 'restart': {'nav': {**t2['ros']['restart']['nav'], 'ready_s': 0.1}}}}
    with pytest.raises(config.SimError, match=r'not ready in 0.1 s: bt_navigator') as e:
        restart_mod.nav(cfg, slow, run)
    assert e.value.code == 75
    # no recipe: one line naming the fix
    bare = {**t2, 'ros': {k: v for k, v in t2['ros'].items() if k != 'restart'}}
    with pytest.raises(config.SimError, match='restart ros') as e:
        restart_mod.nav(cfg, bare, run)
    assert e.value.code == 2

    rc, out, err = _crane_sim(env, 'restart', 'ros', '--json')
    assert rc == 0, err
    doc = json.loads(out)
    assert doc['schema'] == 'crane.sim.restart/1' and doc['restarted'] == 'ros' and doc['status']['ros']['running']
    assert (stub / 'docker_run.argv').read_text() == docker_run and (run / 'player.pid').read_text() == pid

    heavy_argv = (stub / 'heavy.argv').read_text()
    (stub / 'heavy.argv').unlink()
    rc, out, err = _crane_sim(env, 'restart', 'player')
    assert rc == 0, err
    new = (run / 'player.pid').read_text()
    assert new != pid and (stub / 'heavy.argv').read_text() == heavy_argv
    rc, out, _ = _crane_sim(env, 'check')      # the stub player never logs the plant line, nothing else fails
    assert rc == 1 and 'failed: plant applied;' in out and 'FAIL  course' not in out and 'ok    nav stack up' in out
    assert _crane_sim(env, 'down')[0] == 0
    assert not (run / 'player.argv').exists() and _crane_sim(env, 'restart', 'player')[0] == 2


# ---- the T1 world (crane sim up --tier T1) ------------------------------------------------------------------------

def _t1_doc(odom='sensors', current=(0.0, 0.0)):
    from crane.sim import t1
    cfg = player.load_domain('boat')
    sec = config.t1_section(cfg, config.domain('boat', 'T2')[1], odom)
    profile = json.loads((ROOT / 'Tools/Characterization/profiles/characterized.json').read_text())
    urdf = ROOT / cfg['robot']['robots']['blastoise']['urdf']
    lever = t1.lever_arms(urdf, {'gps': 'gps_link', 'imu': 'imu_link'})
    return sec, {'odom': odom, 'rate': 1.0, 'dt': profile['fixedDeltaTime'], 'profile': profile,
                 'current': list(current), 'start': [2.0, -1.0, 0.3], 'lever': {k: list(v) for k, v in lever.items()},
                 'world': sec['world']}


def test_the_t1_node_steps_the_plant_exactly_as_profileboat_and_keeps_the_players_command_contract():
    """The node's twin, held on a PWM command, lands on ProfileBoat's state bit for bit (current included); it
    drops a message without four pulses, goes neutral 25 ticks after the last one, and its sensors read as the
    player's contract does (+g up at rest, the centripetal term of the IMU point in a turn, the antenna's fix)."""
    from crane.sim import t1, t1node
    from domains.boat.plant import ProfileBoat
    from fastsim.plant import Limits
    sec, doc = _t1_doc(current=(0.1, -0.05))
    twin = t1node.Twin(doc, t1.load_plant(sec))
    lim = Limits.from_dict({'vmax': [2, 1, 1], 'vmin': [-1, -1, -1], 'accel': [1, 1, 1], 'decel': [1, 1, 1],
                            'goal_tol': [0.5, 0.2], 'precise_tol': [0.2, 0.1], 'rotate_above': 1.0, 'rotate_wz': 0.5,
                            'progress_radius': 0.5, 'progress_s': 10, 'pass_radius': 1, 'footprint': [[0, 0]]})
    boat = ProfileBoat(lim, pose=tuple(doc['start']), current=(0.1, -0.05))
    assert twin.specific_force == (0.0, 0.0, t1node.GRAVITY)
    for pwm, ticks in (([1300, 1300, 1700, 1700], 100), ([1600, 1400, 1600, 1400], 50)):   # forward, then a turn
        assert not twin.command(pwm[:3])
        for k in range(ticks):
            if k % 2 == 0:                                   # thruster_pwm_node repeats at 20 Hz: every 2-3 ticks
                assert twin.command(pwm)
            twin.step()
        boat.apply_pwm(pwm, ticks * twin.dt)
        assert twin.plant.state == boat.twin.state
    u, v, r = twin.body_twist()
    assert u > 0.5 and abs(r) > 0.05
    fx, fy, fz = twin.specific_force
    assert fz == t1node.GRAVITY and abs(fy) > 0.01            # the turn's lateral acceleration at the IMU point
    lat, lon, alt, var = twin.fix()
    east, north, up = twin.point('gps')
    assert alt == up == doc['lever']['gps'][2] and var == 0.01
    assert abs((lat - 27.3747) * math.pi / 180 * t1node.EARTH_RADIUS - north) < 1e-6
    # silence: the last command holds for 25 ticks, then neutral (the player's --crane-command-timeout-ticks)
    twin.command([1550, 1550, 1550, 1550])
    twin.step()                                              # applied at this tick
    held = list(twin.pwm)
    for _ in range(25):
        twin.step()
    assert twin.pwm == held
    twin.step()
    assert twin.pwm == [1500.0] * 4
    twin.reset()
    assert twin.pose() == tuple(doc['start']) and twin.body_twist() == (0.0, 0.0, 0.0) and twin.k == 177


def test_the_t1_lever_arms_are_the_urdfs_the_stack_publishes():
    """gps_link and imu_link in base_link from crane's blastoise URDF equal mhseals_nav's xacro, which the sim's
    robot_state_publisher publishes (box_joint datum_z, then the sensor joints)."""
    _, doc = _t1_doc()
    assert doc['lever'] == {'gps': [0.0699, -0.1207, 0.0954], 'imu': [-0.0572, -0.1016, 0.0954]}
    xacro = config.home() / 'astro_dock/src/mhseals_nav/description/blastoise/blastoise.xacro'
    if not xacro.is_file():
        pytest.skip(f'no {xacro} to compare with')
    props = dict(re.findall(r'<xacro:property name="(\w+)" value="([-0-9.]+)"/>', xacro.read_text()))
    for link in ('gps', 'imu'):
        want = [float(props[f'{link}_x']), float(props[f'{link}_y']), float(props['datum_z']) + float(props[f'{link}_z'])]
        assert doc['lever'][link] == pytest.approx(want, abs=1e-9)


T1_DOCKER = r'''#!/bin/sh
printf '%s\n' "$*" >> "$STUB/docker.log"
case $1 in
  inspect) [ -f "$STUB/running" ] && echo true || { echo "Error: no such object" >&2; exit 1; } ;;
  run) shift; printf '%s\n' "$@" > "$STUB/docker_run.argv"; touch "$STUB/running"
       printf 'CRANE_T1_PLANT profile=Characterized\nCRANE_T1_READY course=x\n' > "$SIM_UP_RUN/t1.log"
       python3 -c 'import json, time; print(json.dumps({"t_sim": 4.2, "wall": time.time(), "rate": 1.0}))' \
         > "$SIM_UP_RUN/t1.alive"; echo cid ;;
  rm) rm -f "$STUB/running" ;;
  exec) printf '/crane_t1\n/ekf_local\n/rtabmap\n/bt_navigator\n' ;;
esac
'''


def test_t1_up_status_check_and_down_with_the_odom_modes(world_env, monkeypatch, capsys):
    """up --tier T1 runs the node beside device:=sim in the one container (no player), with the odom mode's
    overrides, the course from the generator or the committed truth; status shows the tier, check reads the node,
    restart player refuses and down clears the tier."""
    from domains.boat import coursegen
    stub, run = world_env['stub'], world_env['run']
    _exe(stub / 'bin' / 'docker', T1_DOCKER)
    monkeypatch.setattr(coursegen, 'truth', lambda *a: (_ for _ in ()).throw(RuntimeError('no dotnet')))
    for odom, ready in (('sensors', ['ekf_local', 'bt_navigator']), ('truth', ['bt_navigator'])):
        args = ['up', '--tier', 'T1', '--seed', '2', '--current', '0'] + (['--odom', odom] if odom == 'truth' else [])
        assert sim.main(args) == 0, capsys.readouterr()
        out, err = capsys.readouterr()
        assert 'from committed domains/boat/courses/roboboat_2026-s2.json' in out
        assert f'  tier:      T1 (odom {odom}, rate 1x)' in out and '  world:     T1 node running' in out
        assert not (stub / 'heavy.argv').exists() and not (run / 'player.pid').exists()
        docker = (stub / 'docker_run.argv').read_text().splitlines()
        command = docker[-1]
        assert command.startswith(f'python3 /opt/crane_t1/t1node.py {run}/t1.json >{run}/t1.log 2>&1 & exec ros2 '
                                  'launch mhseals_nav device.launch.py device:=sim overrides:=')
        overrides = json.loads(command.split("overrides:='", 1)[1].split("'", 1)[0])
        off = {'camera', 'lidar', 'objects', 'sim_bridge'} | ({'localization'} if odom == 'truth' else set())
        assert overrides == {'parts': {p: {'enabled': False} for p in off}}
        assert f'{ROOT}/crane/sim/t1node.py:/opt/crane_t1/t1node.py:ro' in docker
        assert f'{ROOT}/Tools/Characterization:/opt/crane_t1/plant:ro' in docker
        node = json.loads((run / 't1.json').read_text())
        assert node['odom'] == odom and node['current'] == [0.0, 0.0] and node['seed'] == 2
        assert node['plant'] == '/opt/crane_t1/plant/plant.py' and node['course'] == 'roboboat_2026-s2'
        assert json.loads((run / 'course_truth.json').read_text())['name'] == 'roboboat_2026-s2'
        cfg, sec = config.domain('boat')
        assert sec['tier'] == 'T1' and sec['ros']['ready_nodes'] == ready
        monkeypatch.setattr(check, 'RUN', run)
        monkeypatch.setattr(check, '_T2', None)
        assert check.main(['--json']) == 0, capsys.readouterr().out
        rows = {r['name']: r for r in json.loads(capsys.readouterr().out)['checks']}
        assert rows['T1 node running']['detail'] == f't 4.2 s sim, 1x real time, odom {odom}'
        assert rows['nav stack up']['detail'] == ', '.join(ready)
        assert sim.main(['up', '--tier', 'T1']) == 2 and 'already up' in capsys.readouterr().err
        assert sim.main(['restart', 'player']) == 2
        assert 'the T1 world runs no player' in capsys.readouterr().err
        assert sim.main(['down']) == 0 and not (run / 'tier').exists() and not (stub / 'running').exists()
        capsys.readouterr()
    # refusals: one line, exit 2
    for args, says in ((['up', '--tier', 'T1', '--profile', 'train-gpu'], '--profile is for the T2 player'),
                       (['up', '--odom', 'truth'], '--odom is for the T1 world'),
                       (['up', '--tier', 'T1', '--rate', '0'], '--rate wants a positive factor'),
                       (['up', '--tier', 'T0'], 'not T0'),
                       (['up', '--tier', 'T1', '--set', 'a.b=1'], '--set needs the generator')):
        assert sim.main(args) == 2, args
        assert says in capsys.readouterr().err, args
