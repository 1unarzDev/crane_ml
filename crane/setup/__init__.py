"""crane setup: the setup wizard. Detect this machine, choose components and domains, show sizes and times, install
them with the tools install.sh and crane update use, then run crane doctor.

    crane setup                                              # asks (questionary when installed, else plain prompts)
    crane setup --yes --components core,fast,player --domains boat
    crane setup --yes --dry-run --json                       # the detected machine and the plan, nothing installed

The pip extras go through `install.sh --yes --no-clone --components ...` (the one installer); the player through
`crane update --player-only` (the release player, sha256 checked, into the player store). `astro` only checks the
astro integration: setup never builds astro images or workspaces, astro owns those. The choices are saved to
~/.config/crane/setup.json.
"""

import json
import subprocess
import sys
import time

from crane import output, paths
from crane.setup import detect

# name: (what it gives, install size in MB, minutes on a fast connection). Sizes are what pip or the release adds.
COMPONENTS = {
    'core': ('numpy, pyyaml: T0/T1 runs, studies, judge, the board', 80, 0.5),
    'fast': ('numba: the batched engine on CPU cores', 160, 0.5),
    'gpu': ('jax[cuda12]: the batched engine on an NVIDIA GPU', 6000, 4),
    'view': ('PySide6, pyqtgraph: crane view, the native window', 600, 1.5),
    'field': ('mcap: field data import', 20, 0.2),
    'data': ('pyarrow: parquet datasets', 150, 0.5),
    'player': ('the T2 Unity player from the release (crane sim)', 450, 1),
    'astro': ('check the astro integration (read-only; astro owns its images)', 0, 0),
}
EXTRAS = ('fast', 'gpu', 'view', 'field', 'data')


def defaults(m):
    """The components this machine wants: core; fast on 4+ cores; gpu with an NVIDIA driver; view and player with a
    display; player and astro with astro_dock."""
    want = ['core']
    if m['cores'] >= 4:
        want.append('fast')
    if m['gpu']:
        want.append('gpu')
    if m['display']:
        want.append('view')
    if m['display'] or m['astro_dock']:
        want.append('player')
    if m['astro_dock']:
        want.append('astro')
    return want


def domains():
    """{name: first comment line} of config/domains/*.yaml."""
    out = {}
    for path in sorted(paths.DOMAINS.glob('*.yaml')):
        first = next((ln.lstrip('# ').strip() for ln in path.read_text().splitlines() if ln.startswith('#')), '')
        out[path.stem] = first.split(':')[0] if first.lower().startswith('the ') else first
    return out


def _player_size():
    """The release player's size in MB (its tarball plus unpacked), else the table's estimate."""
    try:
        from crane import update
        release, _ = update.fetch_release(timeout=5)
        player = release.get('player') or {}
        return round((player['bytes'] + int(player.get('unpacked_bytes') or 4 * player['bytes'])) / 1e6)
    except Exception:          # no release yet, offline: the estimate
        return COMPONENTS['player'][1]


def extra_requirements(extras):
    """The requirements the installed crane lists for these extras (importlib.metadata)."""
    import importlib.metadata as md
    import re
    out = []
    for req in md.requires('crane') or []:
        name, _, marker = req.partition(';')
        m = re.search(r'extra\s*==\s*["\']([^"\']+)', marker)
        if m and m.group(1) in extras and not name.strip().startswith('crane'):
            out.append(name.strip())
    return out


def plan(components, chosen_domains, m):
    """[(step, detail, argv or None)] in run order."""
    steps = []
    extras = [c for c in components if c in EXTRAS]
    if paths.ROOT.joinpath('install.sh').is_file() and m['checkout']:
        steps.append(('install', f'crane{"[" + ",".join(extras) + "]" if extras else ""} into {paths.ROOT}/.venv',
                      ['bash', str(paths.ROOT / 'install.sh'), '--yes', '--no-clone',
                       '--components', ','.join(['core', *extras])]))
    elif extras:                                  # an installed wheel: the extras' requirements, as it lists them
        steps.append(('install', f'the [{",".join(extras)}] requirements into {sys.prefix}',
                      [sys.executable, '-m', 'pip', 'install', *extra_requirements(extras)]))
    if 'player' in components:
        steps.append(('player', 'the release player into ~/.local/share/crane/player (current + prev)',
                      [str(paths.ROOT / 'bin' / 'crane'), 'update', '--player-only']))
    if 'astro' in components:
        steps.append(('astro', 'check the astro integration (read-only)', None))
    steps.append(('save', f'{paths.USER_CONFIG / "setup.json"}: components and domains', None))
    steps.append(('doctor', 'crane doctor', None))
    return steps


def astro_checks(chosen_domains):
    """[(name, state, detail)] of each chosen domain's T2 host tooling: its checkout, its vessel profile, whether
    the profile's sim backend is crane, and whether it loads crane's help.yaml. Reads only."""
    from crane.player import ConfigError, load_domain
    from crane.sim import config
    rows = []
    for d in chosen_domains:
        try:
            t2 = (load_domain(d).get('tiers') or {}).get('T2') or {}
        except ConfigError:
            continue
        if not t2.get('host'):
            continue
        dock, root = config.dock(t2), config.profiles_root(t2)
        if not dock or not dock.is_dir():
            rows.append((f'{d}: host', 'missing', f'no {dock}: clone it (astro owns its setup)'))
            continue
        profile = (root / config.vessel(t2) / 'profile.yaml') if root else None
        if not profile or not profile.is_file():
            rows.append((f'{d}: profile', 'missing', f'no {profile}'))
            continue
        text = profile.read_text()
        sim = next((ln.split(':', 1)[1].split('#')[0].strip() for ln in text.splitlines()
                    if ln.strip().startswith('sim:')), '')
        rows.append((f'{d}: sim backend', 'ok' if 'bin/crane sim' in sim else 'warn',
                     f'{profile}: sim: {sim or "none"}' + ('' if 'bin/crane sim' in sim else
                                                          ' (want ${CRANE_DIR:-~/crane_ml}/bin/crane sim)')))
        helped = 'help.yaml' in text and 'crane' in text.split('help', 1)[-1][:200]
        rows.append((f'{d}: help', 'ok' if helped else 'off', 'astro help crane' if helped else
                     "astro does not load crane's help.yaml yet (an astro.help hook on astro's side)"))
    return rows or [('astro', 'off', 'no chosen domain names T2 host tooling')]


def _ask(m, comp_default, dom_all):
    """(components, domains) from the person: questionary when installed and on a terminal, else plain input."""
    try:
        import questionary
    except ImportError:
        questionary = None
    if questionary:
        comps = questionary.checkbox('Components', choices=[
            questionary.Choice(f'{name:<7} {info[0]}', value=name, checked=name in comp_default)
            for name, info in COMPONENTS.items()]).ask()
        doms = questionary.checkbox('Domains', choices=[
            questionary.Choice(f'{name:<8} {summary}', value=name, checked=True)
            for name, summary in dom_all.items()]).ask()
        if comps is None or doms is None:
            raise KeyboardInterrupt
        return (comps or ['core']), doms
    print('components (comma list; Enter keeps the default):')
    for line in output.table([(n, *info[:1]) for n, info in COMPONENTS.items()], indent=4):
        print(line)
    comps = input(f'  components [{",".join(comp_default)}]: ').strip() or ','.join(comp_default)
    print(f'domains: {", ".join(dom_all)}')
    doms = input(f'  domains [{",".join(dom_all)}]: ').strip() or ','.join(dom_all)
    return _split(comps), _split(doms)


def _split(text):
    return [x.strip() for x in str(text or '').split(',') if x.strip()]


def _show_machine(out, m):
    rows = [(k, m[k] or 'none') for k in ('os', 'python', 'cores', 'memory', 'disk', 'gpu', 'docker', 'astro_dock',
                                          'unity', 'display', 'checkout')]
    try:
        if out.json or out.quiet or not sys.stdout.isatty():
            raise ImportError
        from rich.console import Console
        from rich.table import Table
        table = Table(title='this machine', show_header=False, box=None, padding=(0, 2))
        for k, v in rows:
            table.add_row(k, str(v))
        Console().print(table)
    except ImportError:
        out.line('crane setup: this machine')
        out.table(rows)


def main(args):
    out = args.output
    interactive = not (args.yes or out.json or args.components or args.domains)
    if interactive and not sys.stdin.isatty():
        return out.usage('setup', 'no terminal to ask on', 'crane setup --yes [--components A,B] [--domains A,B]')
    m = detect.machine()
    dom_all = domains()
    _show_machine(out, m)
    comps = _split(args.components) or defaults(m)
    doms = _split(args.domains) or list(dom_all)
    if interactive:
        try:
            comps, doms = _ask(m, comps, dom_all)
        except (KeyboardInterrupt, EOFError):
            return out.usage('setup', 'stopped', 'crane setup')
    if 'core' not in comps:
        comps.insert(0, 'core')
    bad = [c for c in comps if c not in COMPONENTS]
    if bad:
        return out.usage('setup', f'unknown component {bad[0]} (components: {", ".join(COMPONENTS)})',
                         'crane setup --components core,fast')
    bad = [d for d in doms if d not in dom_all]
    if bad:
        return out.usage('setup', f'unknown domain {bad[0]} (domains: {", ".join(dom_all)})',
                         f'crane setup --domains {",".join(dom_all)}')
    sizes = {c: (_player_size() if c == 'player' else COMPONENTS[c][1]) for c in comps}
    total_mb, total_min = sum(sizes.values()), sum(COMPONENTS[c][2] for c in comps)
    steps = plan(comps, doms, m)
    doc = {'schema': 'crane.setup/1', 'dry_run': out.dry_run, 'machine': {k: v for k, v in m.items()
                                                                          if k != 'free_bytes'},
           'components': comps, 'domains': doms, 'sizes_mb': sizes, 'total_mb': total_mb, 'minutes': total_min,
           'steps': [{'step': s, 'detail': d, 'argv': a} for s, d, a in steps]}
    out.line('')
    out.line(f'components ({total_mb / 1000:.1f} GB, about {max(1, round(total_min))} min)')
    out.table([(c, f'{sizes[c]} MB' if sizes[c] else '-', COMPONENTS[c][0]) for c in comps])
    out.line(f'domains     {", ".join(doms)}')
    if total_mb * 1e6 * 1.2 > m['free_bytes']:
        out.doc({**doc, 'ok': False})
        return out.fail('setup', f'{m["free_bytes"] / 1e9:.1f} GB free, the components need about '
                        f'{total_mb * 1.2 / 1000:.1f} GB', 'free space, or crane setup --components core', output.USAGE)
    out.line('plan')
    out.table([(s, d) for s, d, _ in steps])
    if out.dry_run:
        if out.doc({**doc, 'ok': True}):
            return output.OK
        out.next('crane setup --yes --components ' + ','.join(comps) + ' --domains ' + ','.join(doms))
        return output.OK
    if interactive:
        answer = input('crane setup: go ahead? [Y/n] ').strip().lower()
        if answer not in ('', 'y', 'yes'):
            return out.usage('setup', 'stopped', 'crane setup')

    results = []
    for step, detail, argv in steps:
        if argv:
            out.line(f'+ {" ".join(argv)}')
            started = time.monotonic()
            proc = subprocess.run(argv, stdout=sys.stderr if out.json else None, text=True)
            results.append({'step': step, 'exit': proc.returncode, 'seconds': round(time.monotonic() - started, 1)})
            if proc.returncode:
                out.doc({**doc, 'ok': False, 'results': results})
                return out.fail('setup', f'{step} exited {proc.returncode}', ' '.join(argv))
        elif step == 'astro':
            rows = astro_checks(doms)
            results.append({'step': step, 'checks': [dict(zip(('name', 'state', 'detail'), r)) for r in rows]})
            out.table(rows)
        elif step == 'save':
            saved = paths.USER_CONFIG / 'setup.json'
            saved.parent.mkdir(parents=True, exist_ok=True)
            saved.write_text(json.dumps({'schema': 'crane.setup.choice/1', 'components': comps, 'domains': doms,
                                         'at': time.strftime('%Y-%m-%dT%H:%M:%S%z')}, indent=1) + '\n')
            results.append({'step': step, 'path': str(saved)})
            out.line(f'  saved {saved}')
    from crane import doctor
    if out.json:
        rows = doctor.checks()
        doc['doctor'] = [{'name': n, 'state': s, 'detail': d} for n, s, d in rows]
        out.doc({**doc, 'ok': not any(r[1] == 'FAIL' for r in rows), 'results': results})
        return output.OK if not any(r[1] == 'FAIL' for r in rows) else output.FAILED
    return doctor.main(args)

