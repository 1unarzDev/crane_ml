"""crane <verb>: the argument parser and the dispatch. Stdlib only; each verb's module is imported when it runs."""

import argparse
import sys

TOP = """crane: sim and research across domains (boat, sub, drones, modular) and tiers.

Tiers: T0 abstract (no forces), T1 dynamic (plant card dynamics), T2 unity (the Unity sim).
--tier takes T0|T1|T2 or abstract|dynamic|unity."""

TOP_EXAMPLES = """examples:
  crane doctor                                            what is installed, what works, what to run next
  crane run --domain toy --seeds 0:3 --out /tmp/runs                 T0 abstract, scenarios made on demand
  crane run --domain boat --tier T1 --seeds 0:2 --out /tmp/runs      T1 dynamic, the RoboBoat reference mission
  crane sim up --headless                                 the T2 world, used like the boat (astro sim up)
  crane VERB --help                                       each verb's options, an example and what you'll see"""


def _stub(what, today):
    def run(_args):
        print(f'crane: {what} is not yet available; use {today}.', file=sys.stderr)
        return 2
    return run


# Verbs whose arguments go to another tool untouched: crane only adds its help (`crane VERB --help`).
def _passthrough(verb, rest):
    from crane import wrap
    return getattr(wrap, verb)(rest)


PASSTHROUGH = ('sim', 'heavy', 'judge', 'card', 'board', 'course', 'cluster')
USAGE = {'sim': 'crane sim {up,down,status,check} [ARGS...]', 'heavy': 'crane heavy {unity,sim,preview,contain,status} [CMD...]',
         'judge': 'crane judge {demo,fastsim,oneline,markdown} [ARGS...]', 'card': 'crane card {show,export} [ARGS...]',
         'board': 'crane board [ROOTS...] [--host H] [--port P]', 'course': 'crane course {gen,validate,render} [ARGS...]',
         'cluster': 'crane cluster [ARGS...]'}
LOCAL_SUBHELP = {'course': ('gen',)}   # sub-verbs whose tool has no --help of its own
# The sub-verbs crane itself names (the others' are their tool's); help.yaml's `run:` lines resolve against these.
SUBVERBS = {'sim': ('up', 'down', 'status', 'check'), 'card': ('show', 'export'),
            'course': ('gen', 'validate', 'render'), 'bench': ('fast', 'player'), 'build': ('player',)}


def _add(sub, name, help, about, example):
    parser = sub.add_parser(name, help=help, description=f'{help}\n\nwhat you\'ll see: {about}',
                            epilog=f'example:\n  {example}', formatter_class=argparse.RawDescriptionHelpFormatter)
    if name in USAGE:
        parser.usage = USAGE[name]
    return parser


def build_parser():
    ap = argparse.ArgumentParser(prog='crane', description=TOP, epilog=TOP_EXAMPLES,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--version', action='version', version=_version())
    sub = ap.add_subparsers(dest='verb', metavar='VERB')

    _add(sub, 'sim', 'T2: the Unity world plus ROS, used like the real boat (Tools/RoboBoat/sim_up)',
         'sim_up\'s own output: the player queued on `heavy unity`, then its status block (mode, domain, endpoint, '
         'pids, logs, next commands). Verbs: up [--seed N] [--course FAMILY] [--profile-json F] [--window|--headless] '
         '[--player PATH], down, status, check [--wait SECONDS] (live readiness incl. the nav stack, polling with --wait; or CHECK INPUTS for the offline Tools/SimCheck checks). Everything after the '
         'verb goes to sim_up unchanged, with the same env (ROS_DOMAIN_ID, ASTRO_STATE, HEAVY, ASTRO_IMAGE, SIM_UP_*, '
         'CRANE_PLAYER). `crane sim up --help` prints sim_up\'s usage.',
         'crane sim up --headless --seed 3 && crane sim check --wait 120')
    _add(sub, 'heavy', 'the machine-wide concurrency guard for Unity, sims and test suites (bin/heavy)',
         'heavy\'s own output: it waits for a slot, runs CMD, releases it. `crane heavy status` lists the slots, the '
         'queue and ETAs. Arguments go to bin/heavy unchanged.',
         'crane heavy contain test python3 -m pytest fastsim/tests -q')

    r = _add(sub, 'run', 'run episodes at a tier and write one run dir per seed',
             'one line per seed (run dir, outcome, sim and wall time), then runs/s. Runs land in '
             'OUT/VARIANT/s<seed>. T0 and T1 run the fastsim batch runner; T2 is not yet available (use crane sim up). Missing scenarios are generated when the domain has a generator.',
             'crane run --domain toy --seeds 0:3 --out /tmp/runs --set workers=2')
    src = r.add_mutually_exclusive_group(required=True)
    src.add_argument('--domain', help='a domain: config/domains/DOMAIN.yaml (boat, toy); its tiers name the fastsim configs')
    src.add_argument('--config', help='a fastsim domain YAML, used as is')
    r.add_argument('--tier', default=None, help='T0|T1 or abstract|dynamic (default: the domain\'s first tier, T0 with --config)')
    r.add_argument('--seeds', required=True, help='N, FROM:TO (TO exclusive) or a,b,c')
    r.add_argument('--out', required=True, help='output root; runs go to OUT/VARIANT/s<seed>')
    r.add_argument('--workers', type=int, help='worker processes (default: the config\'s workers)')
    r.add_argument('--set', action='append', default=[], metavar='KEY=VALUE',
                   help='override a config key; dotted keys nest, the value is YAML (repeatable)')
    r.add_argument('--variant', default='base', help='name of this design or parameter set (default base)')
    r.add_argument('--generate', action='store_true', help="regenerate the seeds' scenarios with the config's generator (missing ones are always generated)")
    r.add_argument('--allow-dirty', action='store_true', help='run an adapter tree with uncommitted changes')

    _add(sub, 'judge', 'score runs against a judge pack (python -m judge)',
         'the judge\'s verdict lines; with --out, report.json and Markdown. Sub-verbs: demo, fastsim ROOT, '
         'oneline REPORT, markdown REPORT.',
         'crane judge fastsim /tmp/runs --out /tmp/runs-judge')
    _add(sub, 'card', 'show or export a plant card (python -m plantcard)',
         'show [REF]: a text table, one row per block (parameters with their sd and status) and the validation rows. '
         'export VIEW [REF]: the tier view (unity | nlc | control | fastsim) as JSON. REF defaults to '
         'roboboat/blastoise@HEAD.',
         'crane card show roboboat/blastoise@HEAD')
    _add(sub, 'board', 'the web replay board over local runs (python -m viewer serve)',
         'a local URL (default http://127.0.0.1:8765) listing every *.ctraj, course and judge report under ROOTS. '
         'The cluster dashboard deploys the same app.',
         'crane board /tmp/runs --port 8765')
    _add(sub, 'course', 'generate, validate or render course truth (Tools/RoboBoat)',
         'gen: the Unity course generator, headless through `heavy unity` (--family F --seed N --out T.json | '
         '--seeds A:B --out DIR | --validate DIR --out V.json; --mode, --set), printing CRANE_COURSE_* lines. '
         'validate TRUTH.json [--spec S] [--json]: the independent constraint check, exit 0 when every row passes. '
         'render TRUTH.json [--track T.jsonl]: an SVG on stdout.',
         'crane course gen --family roboboat_2026 --seed 3 --out /tmp/c3.json && crane course validate /tmp/c3.json')

    b = _add(sub, 'bench', 'throughput benchmarks: the Unity player (the fast tier: python3 -m bench.fast)',
             'player: per-worker results under PerformanceResults/ and, for a sweep, a summary.json per worker '
             'count. Player settings come from env (CRANE_PLAYER, CRANE_DURATION, CRANE_SCENE, ...).',
             'crane bench player --sweep 1 2 4')
    bs = b.add_subparsers(dest='what', metavar='WHAT', required=True)
    bs.add_parser('fast', help='fast-tier throughput table (not yet available here; use python3 -m bench.fast)')
    bp = bs.add_parser('player', help='Unity player real-time factor: one worker, or a sweep over worker counts')
    bp.add_argument('--worker', type=int, default=0, help='run one worker with this id (default 0)')
    bp.add_argument('--sweep', type=int, nargs='*', metavar='N',
                    help='run N workers at once, for each N (default: sweep in config/bench.yaml)')

    bd = _add(sub, 'build', 'build the Linux player (Unity, headless, through heavy unity)',
              'the Unity command, then the stamped Builds/CRANE-Worker/crane-build-manifest.json (sourceCommit, '
              'sourceDirty) and, with --tarball, Builds/crane-player-<commit12>-linux-x86_64.tar.zst and its .sha256.',
              'crane build player --tarball')
    bds = bd.add_subparsers(dest='what', metavar='WHAT', required=True)
    p = bds.add_parser('player', help='build Builds/CRANE-Worker, stamp its manifest, optionally pack a tarball')
    p.add_argument('--tarball', action='store_true', help='also write the player tarball and its .sha256')
    p.add_argument('--skip-unity', action='store_true', help='only stamp and pack an existing build')
    p.add_argument('--editor', help='the Unity editor (default UNITY_EDITOR, else the ProjectVersion.txt editor)')
    p.add_argument('--builds', help='the Builds dir (default <repo>/Builds)')
    p.add_argument('--dry-run', action='store_true', help='print the Unity command and stop')

    _add(sub, 'cluster', 'research-cluster jobs (the research CLI)',
         'the research CLI\'s output; arguments go to it unchanged. Found at $RESEARCH_CLUSTER/bin/research or '
         '~/research-cluster/bin/research.',
         'crane cluster status')
    _add(sub, 'doctor', 'check this machine: python, extras, player, heavy, astro_dock, GPU',
         'a table of checks (ok / off for unselected extras / missing / warn, with details), then the next commands. Exit 1 only when the '
         'core (python >= 3.10, numpy, pyyaml) is broken. Safe to run any time.',
         'crane doctor')
    _add(sub, 'setup', 'the setup wizard (not yet available; use install.sh)',
         'a pointer to install.sh.', './install.sh --yes --components core,fast')
    _add(sub, 'view', 'the native sandbox window (not yet available; use crane board)', 'a pointer to crane board.', 'crane view /tmp/runs')
    _add(sub, 'compare', 'tier gap, variant A/B and field-vs-sim comparison (not yet available; use crane judge)',
         'a pointer to crane judge.', 'crane compare /tmp/a /tmp/b --tiers')
    _add(sub, 'fit', 'field data to plant card (not yet available; use crane card)', 'a pointer to crane card.', 'crane fit BUNDLE')
    return ap


def _version():
    from crane import __version__
    return f'crane {__version__}'


def _run(args):
    from crane import run
    return run.main(args)


def _bench(args):
    if args.what == 'fast':
        return _stub('`crane bench fast`', '`python3 -m bench.fast --domain boat --tier T1 --seeds 0:50 --out DIR`')(args)
    from crane import wrap
    return wrap.bench_player(args.worker, args.sweep)


def _build(args):
    from crane import build
    return build.main(args)


def _doctor(_args):
    from crane import doctor
    return doctor.main()


HANDLERS = {
    'run': _run, 'bench': _bench, 'build': _build, 'doctor': _doctor,
    'setup': _stub('the `crane setup` wizard', '`./install.sh [--yes] [--components core,fast,gpu,view,field,setup]`'),
    'view': _stub('`crane view` (the native window)', '`crane board RUNS`'),
    'compare': _stub('`crane compare`', '`crane judge`'),
    'fit': _stub('`crane fit`', '`crane card show|export`'),
}


def main(argv=None):
    argv = sys.argv[1:] if argv is None else list(argv)
    if argv and argv[0] in PASSTHROUGH:
        verb, rest = argv[0], argv[1:]
        if rest[:1] in (['-h'], ['--help']) or not rest or \
                (rest[:1] and rest[0] in LOCAL_SUBHELP.get(verb, ()) and set(rest[1:2]) & {'-h', '--help'}):
            build_parser().parse_args([verb, '-h'])
        return _passthrough(verb, rest)
    parser = build_parser()
    args = parser.parse_args(argv)
    if not args.verb:
        parser.print_help()
        return 0
    return HANDLERS[args.verb](args)


if __name__ == '__main__':
    sys.exit(main())
