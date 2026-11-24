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
  crane view --live --domain boat --tier T1 --seed 3      the native sandbox: a live sim with sliders
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
SUBVERBS = {'sim': ('up', 'down', 'status', 'check'), 'card': ('show', 'export'), 'field': ('import',),
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
    r.add_argument('--seeds', help='N, FROM:TO (TO exclusive) or a,b,c (required, except with --astro-test: 0)')
    r.add_argument('--out', help='output root; runs go to OUT/VARIANT/s<seed> (required, except with --astro-test)')
    r.add_argument('--workers', type=int, help='worker processes (default: the config\'s workers)')
    r.add_argument('--set', action='append', default=[], metavar='KEY=VALUE',
                   help='override a config key; dotted keys nest, the value is YAML (repeatable)')
    r.add_argument('--variant', default='base', help='name of this design or parameter set (default base)')
    r.add_argument('--generate', action='store_true', help="regenerate the seeds' scenarios with the config's generator (missing ones are always generated)")
    r.add_argument('--allow-dirty', action='store_true', help='run an adapter tree with uncommitted changes')
    r.add_argument('--astro-test', metavar='TEST', help="astro's test or group (still, couple-*, speed-*, steps-*, "
                   'nav2, all) as a T0/T1 scenario: runs land in OUT/<card>/<test>/s<seed> with metrics named as '
                   "astro's result fields")
    r.add_argument('--card', action='append', metavar='CARD', help='with --astro-test: a profile JSON, a plant '
                   'card or a `crane fit --factorial` manifest (repeatable; default the domain\'s profile)')
    r.add_argument('--tests', help="with --astro-test: astro's tests file or a bundle holding one (default "
                   '$ASTRO_DOCK/src/mhseals_hardware/config/tests.yaml)')
    r.add_argument('--space', help="with --astro-test: the tests file's space (default large)")
    r.add_argument('--amp', type=float, help="with --astro-test: the effort amplitude (default the space's)")

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
         'a local URL (default http://127.0.0.1:8765) listing every crane.run/1 record (run.json or crane.run.json), '
         'bare *.ctraj, course and judge report under ROOTS, with their event lanes and constraint layers. The '
         'cluster dashboard deploys the same page.',
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
    v = _add(sub, 'view', 'the native sandbox window: replay runs, or run a sim live with sliders ([view] extra)',
             'a window: the top-down map (world layers, constraint volumes, trails, the vehicle at its heading), the '
             'event lanes under it, play/step/speed/scrub, and in live mode the parameter sliders, Rerun from t or 0 '
             'and Promote (writes a crane run spec and prints the command). With --screenshot, a PNG rendered '
             'offscreen and no window.',
             'crane view --live --domain boat --tier T1 --seed 3')
    v.add_argument('runs', nargs='*', metavar='RUN', help='crane.run/1 dirs, fastsim run dirs or bare .ctraj files '
                   '(overlaid in the first one\'s frame and clock)')
    v.add_argument('--live', action='store_true', help='run the sim in-process (T0 or T1)')
    v.add_argument('--domain', default='boat', help='live: config/domains/DOMAIN.yaml (default boat)')
    v.add_argument('--tier', default='T1', help='live: T0|T1 or abstract|dynamic (default T1)')
    v.add_argument('--seed', type=int, default=0, help='live: the scenario seed (default 0)')
    v.add_argument('--offset', action='append', metavar='S', help="run 2, 3, ...: the first run's time of its t = 0 "
                   '(needed when a sim and a wall clock meet)')
    v.add_argument('--transform', action='append', metavar='JSON', help='run 2, 3, ...: {from, to, xyz, yaw} '
                   'mapping its origin onto the first run\'s (needed when origins differ)')
    v.add_argument('--3d', dest='three_d', action='store_true', help='open in the 3-D view (6-DOF records)')
    v.add_argument('--screenshot', metavar='OUT.png', help='render offscreen to OUT.png and exit (no window)')
    v.add_argument('--at', type=float, help='screenshot: the playhead time (s; default the end; live: sim to it)')
    v.add_argument('--promote-dir', default='runs/promoted', help='where Promote writes specs (default runs/promoted)')
    c = _add(sub, 'compare', 'compare two run sets: variant A/B, tier gap (--tiers) or field vs sim (--field)',
             'a Markdown table on stdout and the same as JSON in OUT/compare.json. A B: per matched run the pose '
             'error on a common frame and clock (refused when they cannot be put on one) and the metric deltas. '
             '--tiers: per metric the ranking agreement (regret, MMRV, top-k) and the paired gap with a bootstrap CI. '
             "--field BUNDLE RUNS: per astro test the result's metrics, field vs sim, delta and a CI when samples "
             'allow.', 'crane compare --field bundle-173031.tgz runs/sim')
    c.add_argument('runs', nargs='+', metavar='RUNS', help='A B (run dirs or roots), or with --field: the sim runs')
    c.add_argument('--tiers', action='store_true', help='A is the lower tier, B the higher; variants are the '
                   'first directory level under each root')
    c.add_argument('--field', metavar='BUNDLE', help='an astro bundle (.tgz or its unpacked dir) to compare RUNS '
                   'against')
    c.add_argument('--metric', action='append', help='only these metrics (repeatable)')
    c.add_argument('--direction', default='higher', choices=('higher', 'lower'), help='--tiers: better is higher '
                   'or lower (default higher)')
    c.add_argument('--band', type=float, help='--tiers: the allowed |low - high| gap (metric units)')
    c.add_argument('--fidelity', metavar='CONFIG', help="--tiers: also replay B's goal windows on CONFIG's "
                   'vehicle (fastsim.fidelity.score)')
    c.add_argument('--out', help='write compare.json and compare.md here')
    f = _add(sub, 'fit', 'field data to a profile and a plant card: astro bundle -> gate -> evidence -> card',
             'the gate verdict per parameter (identified / provisional / prior, and why; on abstain the exact '
             'missing fields), the evidence (holds, bags, sessions), the card fitted from it with its blocks, and '
             'the held-out validation when there are two or more sessions. Writes OUT/profile.json, '
             'OUT/profile.gate.json, OUT/cards/, OUT/fit.json. --factorial SPEC writes card variants and a manifest.',
             'crane fit bundle-173031.tgz --bags ~/astro_dock/bags/sim/2026-11-24/bags --out /tmp/fit')
    f.add_argument('bundles', nargs='*', metavar='BUNDLE', help='astro bundles (.tgz); each is one session')
    f.add_argument('--out', help='output dir (default fit-<bundle name>)')
    f.add_argument('--bags', action='append', metavar='DIR', help='bag dirs to read besides the bundle\'s own '
                   '(read only; a bundle made without --bags has none)')
    f.add_argument('--base', help='the base profile the gate imports into (default Ideal)')
    f.add_argument('--robot', help='the card\'s robot name (default astro/<state file stem>)')
    f.add_argument('--n', type=int, default=64, help='rollouts per held-out segment (default 64)')
    f.add_argument('--no-card', action='store_true', help='stop after the gate (no evidence, no card)')
    f.add_argument('--factorial', metavar='SPEC', help='card variants from SPEC.yaml (measured vs prior levels per '
                   'property, plus hypotheses) and a manifest that `crane run --card` takes')
    fd = _add(sub, 'field', 'field data as crane.run/1 records (import)',
              'one line per bag: run dir, duration, odometry samples and rate, path length, events, frame and clock. '
              'Each run has traj.ctraj (odometry, resampled on header stamps), events.jsonl (/events and log.txt '
              'lines) and metrics.json.', 'crane field import bundle-173031.tgz --out /tmp/field')
    fds = fd.add_subparsers(dest='what', metavar='WHAT', required=True)
    fi = fds.add_parser('import', help='a bundle, a bag dir or an .mcap file to run records')
    fi.add_argument('source', help='an astro bundle (.tgz, made with --bags), a rosbag2 dir or an .mcap file')
    fi.add_argument('--out', required=True, help='the run dir (one bag) or the root (several: OUT/<bag>)')
    fi.add_argument('--odom', default='/odom/local', help='odometry topic (default /odom/local)')
    fi.add_argument('--events', default='/events', help='astro events topic (default /events)')
    fi.add_argument('--frame', action='append', metavar='ID=enu|ned', help='what an odometry frame id is, when '
                    'not a REP 103/105 name')
    fi.add_argument('--hz', type=float, default=20.0, help='trajectory rate (default 20)')
    fi.add_argument('--domain', default='boat', help='the run\'s domain (default boat)')
    return ap


def _version():
    from crane import __version__
    return f'crane {__version__}'


def _module(name, args):
    import importlib
    return importlib.import_module(f'crane.{name}').main(args)


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


def _view(args):
    from crane.view import app
    return app.main(args)


def _doctor(_args):
    from crane import doctor
    return doctor.main()


HANDLERS = {
    'run': _run, 'bench': _bench, 'build': _build, 'doctor': _doctor,
    'setup': _stub('the `crane setup` wizard', '`./install.sh [--yes] [--components core,fast,gpu,view,field,setup]`'),
    'view': _view,
    'compare': lambda a: _module('compare', a), 'fit': lambda a: _module('fit', a),
    'field': lambda a: _module('field', a),
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
