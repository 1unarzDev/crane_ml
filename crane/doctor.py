"""crane doctor: what is installed and what works on this machine, then the next commands. Never changes anything."""

import importlib.util
import json
import os
import platform
import shutil
import subprocess
import sys
from pathlib import Path

from crane import paths

EXTRAS = {'fast': ['numba'], 'gpu': ['jax'], 'view': ['PySide6', 'pyqtgraph'], 'field': ['mcap', 'mcap_ros2'],
          'setup': ['rich', 'questionary']}


def required_flag():
    """config/sim.yaml's required_flag (read without a YAML parser, as sim_up does)."""
    try:
        for line in (paths.CONFIG / 'sim.yaml').read_text().splitlines():
            if line.startswith('required_flag:'):
                return line.split(':', 1)[1].split('#')[0].strip()
    except OSError:
        pass
    return '--crane-roboboat-contract'


def has_flags(player, flag=None):
    """sim_up's has_flags: a managed assembly beside the player holds the flag (UTF-16LE, as .NET stores it)."""
    needle = (flag or required_flag()).encode('utf-16-le')
    for dll in Path(player).parent.glob('*_Data/Managed/*.dll'):
        try:
            if needle in dll.read_bytes():
                return True
        except OSError:
            continue
    return False


def _found(module):
    try:
        return importlib.util.find_spec(module) is not None
    except (ImportError, ValueError):
        return False


def build_label(player):
    """'build COMMIT12' from the crane-build-manifest.json beside the player, else 'unknown build'."""
    try:
        commit = json.loads((Path(player).parent / 'crane-build-manifest.json').read_text()).get('sourceCommit')
    except (OSError, ValueError):
        commit = None
    return f'build {str(commit)[:12]}' if commit else 'unknown build'


def checks():
    """[(name, state, detail)], state in ok | off | warn | missing | FAIL. Only FAIL breaks the core."""
    rows = []
    py = sys.version_info
    rows.append(('python', 'ok' if py >= (3, 10) else 'FAIL',
                 f'{platform.python_version()} at {sys.executable}' + ('' if py >= (3, 10) else ' (needs >= 3.10)')))
    for mod in ('numpy', 'yaml'):
        rows.append((mod, 'ok' if _found(mod) else 'FAIL', 'core' if _found(mod) else 'core: ./install.sh'))
    for extra, mods in EXTRAS.items():
        have = [m for m in mods if _found(m)]
        state = 'ok' if len(have) == len(mods) else 'off'
        rows.append((f'[{extra}]', state, ', '.join(mods) if state == 'ok' else
                     f'optional, add with ./install.sh --components core,{extra}'))
    players = [p for p in paths.player_candidates() if p.is_file() and os.access(p, os.X_OK)]
    flagged = [p for p in players if has_flags(p)]
    if flagged:
        rows.append(('player', 'ok', f'{flagged[0]} (has {required_flag()}; {build_label(flagged[0])})'))
    elif players:
        rows.append(('player', 'warn', f'{players[0]} lacks {required_flag()}: crane build player'))
    else:
        rows.append(('player', 'missing', 'no T2 player: crane build player, or set CRANE_PLAYER'))
    rows.append(('heavy', 'ok' if os.access(paths.HEAVY, os.X_OK) else 'FAIL', str(paths.HEAVY)))
    dock = paths.ASTRO_DOCK
    sim_profile = dock / 'profiles' / 'sim' / 'profile.yaml'
    rows.append(('astro_dock', 'ok' if sim_profile.is_file() else 'off',
                 f'{dock} (sim profile): the astro stack' if sim_profile.is_file() else
                 'optional: without it crane sim runs its own bridge stack'))
    rows.append(('docker', 'ok' if shutil.which('docker') else 'missing', 'the ROS side of crane sim'))
    rows.append(('bridge image', *_bridge_image()))
    rows.append(('xvfb-run', 'ok' if shutil.which('xvfb-run') else 'off',
                 'headless T2 sims' if shutil.which('xvfb-run') else
                 'no headless T2 here: install xvfb (e.g. sudo pacman -S xorg-server-xvfb, sudo apt install xvfb), '
                 'or crane sim use SIM --display window'))
    rows.append(('sim', *_machine_sim()))
    rows.append(('gpu', *_gpu()))
    research = paths.research_cli()
    rows.append(('cluster', 'ok' if research else 'missing', str(research) if research else 'no research CLI'))
    return rows


def _bridge_image():
    from crane import miniyaml
    try:
        spec = (miniyaml.safe_load((paths.CONFIG / 'stacks' / 'bridge.yaml').read_text())['T2']['ros']['image'])
    except (OSError, KeyError, TypeError, ValueError):
        return 'off', 'no config/stacks/bridge.yaml'
    img = os.environ.get(spec.get('env') or '') or spec.get('default')
    if not shutil.which('docker'):
        return 'off', f'{img}: needs docker'
    try:
        found = subprocess.run(['docker', 'image', 'inspect', img], capture_output=True, timeout=20).returncode == 0
    except (OSError, subprocess.TimeoutExpired):
        found = False
    return ('ok', f'{img} (crane sim --stack bridge)') if found else \
        ('off', f'{img} not built: docker build -t {img} {paths.ROOT / (spec.get("build") or "docker/bridge")}')


def _machine_sim():
    from crane.sim import recipe
    try:
        name = recipe.default_name()
        r = recipe.resolve(name)
    except (recipe.RecipeError, OSError) as error:
        return 'warn', f'{recipe.machine_path()}: {error}'
    v = r.values
    what = f"{v['domain']} {v.get('fidelity') or v['tier']}" + (f", display {v['display']}" if v.get('display') else '')
    return ('ok', f'{name} ({what}; crane sim up starts it)') if name else \
        ('off', f'no default sim ({what}); pick one: crane sim list, crane sim use SIM')


def _gpu():
    smi = shutil.which('nvidia-smi')
    if not smi:
        return 'missing', 'no nvidia-smi: T0/T1 run on CPU'
    try:
        out = subprocess.run([smi, '--query-gpu=name,driver_version,memory.total', '--format=csv,noheader'],
                             capture_output=True, text=True, timeout=10)
    except (OSError, subprocess.TimeoutExpired) as error:
        return 'warn', f'nvidia-smi failed: {error}'
    first = out.stdout.strip().splitlines()[:1]
    return ('ok', first[0]) if out.returncode == 0 and first else ('warn', 'nvidia-smi found no GPU')


def next_commands(rows):
    """The next commands for this machine's state."""
    state = dict((r[0], r[1]) for r in rows)
    domains = sorted(p.stem for p in paths.DOMAINS.glob('*.yaml'))
    source = f'--domain {"toy" if "toy" in domains else domains[0]}' if domains else '--config DOMAIN.yaml'
    out = [f'crane run {source} --seeds 0:3']
    if state.get('player') == 'ok' and state.get('docker') == 'ok':
        out.append('crane sim up' if state.get('sim') == 'ok' else 'crane sim list ; crane sim use SIM ; crane sim up')
    elif state.get('player') != 'ok':
        out.append(f'crane build player   (a Unity editor), or set CRANE_PLAYER to a player that has {required_flag()}')
    if 'boat' in domains:
        out.append('crane run --domain boat --tier T1 --seeds 0:2')
    out.append('crane board ~/.cache/crane/runs     the web replay board')
    return out


def main(args=None):
    from crane.output import Output, table
    out = Output.of(args) if args is not None else Output()
    rows = checks()
    ok = not any(r[1] == 'FAIL' for r in rows)
    nxt = next_commands(rows)
    if out.doc({'schema': 'crane.doctor/1', 'ok': ok,
                'checks': [{'name': n, 'state': s, 'detail': d} for n, s, d in rows], 'next': nxt}):
        return 0 if ok else 1
    out.line('crane doctor')
    for line in table([r for r in rows if not out.quiet or r[1] == 'FAIL']):
        out.say(line)
    out.next(*nxt)
    return 0 if ok else 1
