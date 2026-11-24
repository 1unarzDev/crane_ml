"""Verbs that hand over to an existing tool: crane execs it with the arguments unchanged.

Stdlib only, so `crane sim` (astro's sim backend) needs nothing installed.
"""

import os
import sys

from crane import paths


def execute(cmd, python_path=False):
    """Replace this process with cmd. python_path puts the crane root first on PYTHONPATH (for `python -m X`)."""
    cmd = [str(c) for c in cmd]
    env = os.environ.copy()
    if python_path:
        env['PYTHONPATH'] = os.pathsep.join(filter(None, [str(paths.ROOT), env.get('PYTHONPATH')]))
    sys.stdout.flush()
    sys.stderr.flush()
    try:
        os.execvpe(cmd[0], cmd, env)
    except OSError as error:
        print(f'crane: cannot run {cmd[0]}: {error.strerror}', file=sys.stderr)
        return 127


def _python(module, rest):
    return execute([sys.executable, '-m', module, *rest], python_path=True)


def sim(rest):
    """crane sim VERB ...: check runs here, the other verbs go to Tools/RoboBoat/sim_up. With --json, check and
    status print one document (status wraps sim_up's text block)."""
    if '--json' in rest and rest[:1] in (['check'], ['status']):
        return _sim_json(rest[0], [a for a in rest[1:] if a != '--json'])
    if rest[:1] == ['check']:
        from crane import simcheck
        return simcheck.main(rest[1:])
    return execute([paths.SIM_UP, *rest])


def _sim_json(verb, rest):
    import contextlib
    import io
    import json
    import subprocess
    if verb == 'check':
        from crane import simcheck
        text = io.StringIO()
        with contextlib.redirect_stdout(text):
            code = simcheck.main(rest)
        if rest[:1] and not rest[0].startswith('--'):       # an offline Tools/SimCheck check: its text, as is
            doc = {'schema': 'crane.sim.check/1', 'check': rest[0], 'exit': code, 'text': text.getvalue()}
        else:
            # simcheck's rows: '  ok    NAME (18 wide) DETAIL', FAIL or warn in place of ok
            rows = [(line[2:6].strip(), line[8:26].strip(), line[27:]) for line in text.getvalue().splitlines()
                    if line[2:6].strip() in ('ok', 'FAIL', 'warn') and line[6:8] == '  ']
            doc = {'schema': 'crane.sim.check/1', 'ready': code == 0, 'exit': code,
                   'failed': [n for state, n, _ in rows if state == 'FAIL'],
                   'checks': [{'name': n, 'ok': state != 'FAIL', 'state': state, 'detail': d} for state, n, d in rows]}
    else:
        try:
            done = subprocess.run([str(paths.SIM_UP), 'status', *rest], capture_output=True, text=True, timeout=60)
        except subprocess.TimeoutExpired:
            print('crane sim status: sim_up status took over 60 s; fix: crane sim status (text) to see where it '
                  'stops', file=sys.stderr)
            return 75
        code = done.returncode
        fields = {}
        for line in done.stdout.splitlines():
            key, colon, value = line.strip().partition(':')
            if colon and key and ' ' not in key.strip() and value.strip():
                fields.setdefault(key.strip(), value.strip())
        doc = {'schema': 'crane.sim.status/1', 'exit': code, 'fields': fields, 'text': done.stdout}
        sys.stderr.write(done.stderr)
    print(json.dumps(doc, indent=1))
    return code


def heavy(rest):
    return execute([paths.HEAVY, *rest])


def judge(rest):
    return _python('judge', rest)


def card(rest):
    return _python('plantcard', rest)


def board(rest):
    return _python('viewer', ['serve', *rest])


def course(rest):
    what, args = rest[0], rest[1:]
    if what == 'gen':     # the Unity editor, headless, so through heavy unity: check the arguments first
        dry = '--dry-run' in args
        args = [a for a in args if a != '--dry-run']
        cause = _gen_args(args)
        if cause:
            print(f'crane env gen: {cause}; fix: crane env gen --seed 3 --out /tmp/c3.json (or --seeds A:B --out DIR, '
                  'or --validate DIR --out V.json)', file=sys.stderr)
            return 2
        cmd = [str(paths.HEAVY), 'unity', str(paths.COURSE / 'generate.sh'), *args]
        if dry:
            import shlex
            print('crane env gen (dry run): nothing runs')
            print(f'  command  {shlex.join(cmd)}')
            print(f'next: crane env gen {shlex.join(args)}')
            return 0
        return execute(cmd)
    if what in ('validate', 'render'):
        return execute([sys.executable, paths.COURSE / f'{what}.py', *args])
    print(f'crane env: unknown {what!r}; choose gen|validate|render', file=sys.stderr)
    return 2


GEN_OPTIONS = ('--family', '--seed', '--seeds', '--out', '--mode', '--set', '--validate')


def _gen_args(args):
    """Why generate.sh would refuse ARGS (None when they are complete): each option takes a value, --out is
    required, and exactly one of --seed, --seeds or --validate says what to do."""
    given = {}
    i = 0
    while i < len(args):
        if args[i] not in GEN_OPTIONS:
            return f'unknown argument {args[i]!r} (options: {" ".join(GEN_OPTIONS)})'
        if i + 1 >= len(args) or args[i + 1].startswith('--'):
            return f'{args[i]} needs a value'
        given[args[i]] = args[i + 1]
        i += 2
    if '--out' not in given:
        return '--out is required'
    jobs = [o for o in ('--seed', '--seeds', '--validate') if o in given]
    if len(jobs) != 1:
        return 'give one of --seed N, --seeds A:B or --validate DIR' + (f' (got {" ".join(jobs)})' if jobs else '')
    if '--seed' in given and not given['--seed'].lstrip('-').isdigit():
        return f'--seed wants an integer, got {given["--seed"]!r}'
    return None


def cluster(rest):
    cli = paths.research_cli()
    if cli is None:
        print('crane cluster: no research-cluster CLI; clone research-cluster to ~/research-cluster or set '
              'RESEARCH_CLUSTER to its checkout', file=sys.stderr)
        return 2
    return execute([cli, *rest])


def bench(args):
    """crane bench player."""
    try:
        sweep = None if args.sweep is None else [int(n) for n in args.sweep]
    except ValueError:
        print(f'crane bench player: --sweep wants worker counts, got {" ".join(args.sweep)}; fix: crane bench player '
              '--sweep 1 2 4', file=sys.stderr)
        return 2
    return bench_player(args.worker, sweep)


def bench_player(worker, sweep):
    """One worker, or a sweep over worker counts (default: config/bench.yaml), through heavy unity."""
    if sweep is not None:
        counts = sweep or _bench_sweep()
        return execute([paths.HEAVY, 'unity', paths.BENCH / 'sweep_workers.sh', *map(str, counts)])
    return execute([paths.HEAVY, 'unity', paths.BENCH / 'run_worker.sh', str(worker)])


def _bench_sweep():
    import yaml
    return (yaml.safe_load((paths.CONFIG / 'bench.yaml').read_text()) or {}).get('sweep') or [1, 2, 4, 8]
