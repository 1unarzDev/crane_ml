"""Verbs that hand over to an existing tool: crane execs it with the arguments unchanged.

Stdlib only, so `crane sim` (astro's sim backend) needs nothing installed.
"""

import os
import sys

from crane import paths
from crane.cli import SUBVERBS


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
    if rest[:1] == ['check']:
        print('crane: `crane sim check` (the player contract acceptance, Tools/SimCheck) arrives in P1-C.',
              file=sys.stderr)
        return 2
    return execute([paths.SIM_UP, *rest])


def heavy(rest):
    return execute([paths.HEAVY, *rest])


def judge(rest):
    return _python('judge', rest)


def card(rest):
    if rest[0] not in SUBVERBS['card']:
        print(f'crane card: unknown {rest[0]!r}; choose {"|".join(SUBVERBS["card"])}', file=sys.stderr)
        return 2
    return _python('plantcard', rest)


def board(rest):
    return _python('viewer', ['serve', *rest])


def course(rest):
    what, args = rest[0], rest[1:]
    if what == 'gen':     # the Unity editor, headless, so through heavy unity
        return execute([paths.HEAVY, 'unity', paths.COURSE / 'generate.sh', *args])
    if what in ('validate', 'render'):
        return execute([sys.executable, paths.COURSE / f'{what}.py', *args])
    print(f'crane course: unknown {what!r}; choose {"|".join(SUBVERBS["course"])}', file=sys.stderr)
    return 2


def cluster(rest):
    cli = paths.research_cli()
    if cli is None:
        print('crane cluster: no research-cluster CLI; clone research-cluster to ~/research-cluster or set '
              'RESEARCH_CLUSTER to its checkout', file=sys.stderr)
        return 2
    return execute([cli, *rest])


def bench_player(worker, sweep):
    """One worker, or a sweep over worker counts (default: config/bench.yaml), through heavy unity."""
    if sweep is not None:
        counts = sweep or _bench_sweep()
        return execute([paths.HEAVY, 'unity', paths.BENCH / 'sweep_workers.sh', *map(str, counts)])
    return execute([paths.HEAVY, 'unity', paths.BENCH / 'run_worker.sh', str(worker)])


def _bench_sweep():
    import yaml
    return (yaml.safe_load((paths.CONFIG / 'bench.yaml').read_text()) or {}).get('sweep') or [1, 2, 4, 8]
