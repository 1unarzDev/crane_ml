"""crane sim's recipe (named sims, the fidelity ladder, this machine's settings) and where up puts a sim."""

import json

import pytest

from crane import sim
from crane.sim import config, place, recipe, world


@pytest.fixture
def home(tmp_path, monkeypatch):
    for k in ('CRANE_SIM', 'CRANE_CONFIG', 'CRANE_SIM_OWNER', 'CRANE_SIM_INSTANCE', 'CRANE_ISOLATION_ID'):
        monkeypatch.delenv(k, raising=False)
    monkeypatch.setenv('HOME', str(tmp_path))
    return tmp_path


def test_a_sim_resolves_through_its_fidelity_this_machine_and_the_flags(home, monkeypatch, capsys):
    r = recipe.resolve('roboboat-high')
    assert (r.values['tier'], r.values['profile'], r.values['env']) == ('T2', 'interactive-high', 'course:roboboat_2026')
    assert r.sources['profile'] == 'fidelity high (sim roboboat-high)'
    # nothing named and no machine default: the domain's defaults, which are today's bare up
    bare = recipe.resolve(None)
    assert (bare.values['domain'], bare.values['tier'], bare.values['profile']) == ('boat', 'T2', 'interactive-low')
    assert bare.identity() == recipe.resolve('roboboat').identity()
    # this machine's default and display; a flag beats both; an explicit tier beats the rung, and T1 drops its profile
    assert sim.main(['use', 'roboboat-high', '--display', 'window']) == 0
    capsys.readouterr()
    r = recipe.resolve(None, {'seed': 3})
    assert r.name == 'roboboat-high' and r.values['display'] == 'window' and r.sources['seed'] == 'flag'
    assert recipe.resolve(None, {'display': 'headless'}).values['display'] == 'headless'
    t1 = recipe.resolve(None, {'tier': 'T1'})
    assert t1.values['tier'] == 'T1' and 'profile' not in t1.values
    assert recipe.resolve(None, {'domain': 'boat'}).name is None          # --domain alone drops the machine's sim
    assert recipe.resolve(None, {}).identity() == recipe.resolve(None, {'display': 'headless'}).identity()
    # the pool sim's nav arguments merge with the flags'; T1-only pins do not leak into a T2 override
    pool = recipe.resolve('roboboat-pool', {'nav': {'params_file': '/x.yaml'}})
    assert pool.values['nav'] == {'preset': 'pool', 'params_file': '/x.yaml'} and pool.values['world'] == 'pool'
    assert 'world' not in recipe.resolve('roboboat-pool', {'tier': 'T2'}).values
    with pytest.raises(recipe.RecipeError, match="no fidelity 'ultra'; fidelity: twin, low, high, eval"):
        recipe.resolve(None, {'fidelity': 'ultra'})
    with pytest.raises(recipe.RecipeError, match="no sim 'nope'"):
        recipe.resolve('nope')
    # a user's sim file joins the list and wins over the repo's of the same name
    (home / '.config' / 'crane' / 'sims').mkdir(parents=True)
    (home / '.config' / 'crane' / 'sims' / 'mine.yaml').write_text('domain: boat\nfidelity: twin\nseed: 4\n')
    assert sim.main(['list', '--json']) == 0
    listed = json.loads(capsys.readouterr().out)
    assert listed['default'] == 'roboboat-high' and {'mine', 'roboboat'} <= {s['sim'] for s in listed['sims']}
    assert sim.main(['use', 'none']) == 0 and recipe.machine().get('display') == 'window'
    assert recipe.default_name() is None


@pytest.mark.parametrize('running, who, fresh, auto, verdict', [
    (None, 'person', False, False, 'start'),
    ({'identity': 'a', 'owner': 'person'}, 'person', False, False, 'reuse'),
    ({'identity': 'a', 'owner': 'person'}, 'person', True, False, 'replace'),
    ({'identity': 'a', 'owner': 'person'}, 'script', False, False, 'next'),     # never a person's world
    ({'identity': 'a', 'owner': 'person'}, 'script', True, False, 'next'),
    ({'identity': 'a', 'owner': 'script'}, 'script', False, False, 'reuse'),
    ({'identity': 'b', 'owner': 'script'}, 'script', False, True, 'next'),
    ({'identity': 'a', 'owner': 'script'}, 'script', False, True, 'reuse'),
    ({'identity': 'b', 'owner': 'script'}, 'person', False, False, 'already up with another sim'),
    ({'identity': None, 'owner': None}, 'person', False, False, 'already up; crane sim down first'),
])
def test_up_reuses_the_same_sim_and_never_takes_over_a_persons(running, who, fresh, auto, verdict):
    try:
        got = place.decide(running, 'a', who, fresh, auto)
    except config.SimError as error:
        got = str(error)
    assert got.startswith(verdict)


def test_a_scripts_up_goes_beside_a_persons_sim_and_its_down_leaves_it(home, monkeypatch, capsys):
    """The person's T1 world runs in instance 0: a script's identical up goes to instance 1, its down refuses."""
    busy = {0: {'identity': recipe.resolve('roboboat-twin').identity(), 'owner': 'person', 'sim': 'roboboat-twin'}}
    monkeypatch.setattr(place, 'running_doc', lambda t2, run: busy.get(config.instance() or 0))
    started = {}
    monkeypatch.setattr(world, 'up',
                        lambda cfg, t2, run, o: world.remember(run, o) or started.update(instance=config.instance(), run=run) or 0)
    monkeypatch.setenv('CRANE_SIM_OWNER', 'script')
    assert sim.main(['up', 'roboboat-twin']) == 0
    assert started['instance'] == 1 and json.loads((started['run'] / 'recipe.json').read_text())['owner'] == 'script'
    assert sim.main(['down']) == 2 and 'started by a person (roboboat-twin)' in capsys.readouterr().err
    busy[0]['sim'] = 'roboboat-high'
    busy[0]['identity'] = recipe.resolve('roboboat-high').identity()
    assert sim.main(['up', 'roboboat-high']) == 2
    assert "a person's sim is up (roboboat-high): a script never takes it over" in capsys.readouterr().err


def test_stacks_lay_over_the_domain_and_astro_is_todays_world(home, monkeypatch):
    """The astro stack gives exactly the T2/T1 sections boat had before stacks; bridge is crane's own; auto picks."""
    from crane import player
    from crane.sim import stacks
    raw = player.load_domain('boat')
    astro = stacks.apply(raw, 'astro')['tiers']
    assert astro['T2']['ros']['container'] == 'sim_up_ros' and astro['T2']['host']['up'] == 'astro sim up'
    assert astro['T1']['sim']['ros']['ready_nodes'] == {'sensors': ['ekf_local', 'bt_navigator'], 'truth': ['bt_navigator']}
    bridge = stacks.apply(raw, 'bridge')['tiers']
    assert bridge['T2']['ros']['container'] == 'crane_sim_ros' and bridge['T2']['host'] == {}
    assert bridge['T2']['scene'] == 'Roboboat Course' and bridge['T1']['sim']['node'] == 'crane/sim/t1node.py'
    assert stacks.apply(raw, 'none')['tiers']['T2']['ros'] == {}
    monkeypatch.setenv('ASTRO_DOCK', str(home / 'dock'))
    assert stacks.choose(raw, 'auto')[0] == 'bridge'
    (home / 'dock').mkdir()
    assert stacks.choose(raw, None) == ('astro', 'auto: the astro checkout is here')
    with pytest.raises(stacks.StackError, match="no stack 'ros1'"):
        stacks.choose(raw, 'ros1')
    # the bridge's container: crane's image, the endpoint as the one command, the host user, the sim's own domain
    monkeypatch.setenv('ASTRO_DOCK', str(home / 'nowhere'))
    monkeypatch.delenv('ASTRO_PROFILES', raising=False)
    config.set_stack('bridge')
    try:
        cfg, t2 = config.domain('boat', 'T2')
        argv = world.docker_args(t2, home / 'run', config.domain_id(t2), 10101, config.image(t2))
    finally:
        config.set_stack(None)
    assert argv[argv.index('--user') + 1] == f'{__import__("os").getuid()}:{__import__("os").getgid()}'
    assert 'ROS_DOMAIN_ID=101' in argv and 'crane-bridge:jazzy' in argv
    assert argv[-1].startswith('exec ros2 run ros_tcp_endpoint default_server_endpoint --ros-args -p ROS_IP:=127.0.0.1 '
                               '-p ROS_TCP_PORT:=10101 >')


def test_watch_swaps_only_the_players_display_wrapper():
    from crane.sim import restart
    t2 = {'screen': [640, 480]}
    headless = world.launch_cmd(t2, ['/x/heavy', 'unity'], 'headless', '/p/CRANE.x86_64', ['--crane-profile', 'interactive-high'])
    window = restart.with_display(t2, headless, 'window')
    assert window == ['/x/heavy', 'unity', 'env', 'SDL_VIDEODRIVER=x11', 'nice', '-n', '10', '/p/CRANE.x86_64',
                      '--crane-profile', 'interactive-high']
    assert restart.with_display(t2, window, 'headless') == headless


def test_reset_calls_the_worlds_service_and_names_a_failure(monkeypatch, capsys):
    from crane.sim import restart
    t2 = {'ros': {'container': 'c'}, 'world': {'reset': '/crane/reset_to_start'}}
    calls = []
    monkeypatch.setattr(restart, '_need_container', lambda t2: 'crane sim')
    reply = {'out': "response:\nstd_srvs.srv.Trigger_Response(success=True, message='x=1.500 y=0.000 yaw=0.0000')\n", 'rc': 0}
    monkeypatch.setattr(restart, 'dexec', lambda t2, run, argv, timeout=60: calls.append(argv) or
                        __import__('types').SimpleNamespace(returncode=reply['rc'], stdout=reply['out'], stderr=''))
    assert restart.reset({}, t2, None) == 0
    assert calls[0][-1].endswith('ros2 service call /crane/reset_to_start std_srvs/srv/Trigger')
    assert capsys.readouterr().out == 'crane sim: back at the start (x=1.500 y=0.000 yaw=0.0000)\n'
    reply.update(out="std_srvs.srv.Trigger_Response(success=False, message='odom/local reported no pose')\n")
    with pytest.raises(config.SimError, match='reset failed: odom/local reported no pose'):
        restart.reset({}, t2, None)
    with pytest.raises(config.SimError, match='no ROS side'):
        restart.reset({}, {'ros': {}}, None)
