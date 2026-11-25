"""The one registry of crane's commands. argparse, `--help`, `crane help`, help.yaml and Docs/CLI.md all come from it.

A command is a noun and an optional verb, with its summary (one everyday sentence), what you'll see (`about`), notes
(success and the common mistake), details (the technical part, shown only when an entry is opened), params, examples,
`raw` (the low-level equivalent), a JSON schema hint and a handler (`module:function`, imported only when the command
runs).
A command with `today` set is not built yet: it prints one line naming what to use today and exits 2.
A `passthrough` command hands everything after its words to its tool unchanged (`handler(rest)`).

Stdlib only: `crane sim` (astro's sim backend) and `crane help` must work with nothing installed.
"""

import json
import re
from dataclasses import dataclass, field, replace

from crane import select

CRANE_RUN = '${CRANE_DIR:-$HOME/crane_ml}/bin/crane'     # how astro calls crane (no shell, no install)


@dataclass(frozen=True)
class Param:
    name: str                      # '--seeds' (an option) or 'runs' (a positional)
    type: str = 'str'              # str | int | float | bool | path | list | tier | env
    default: object = None
    choices: tuple = ()
    description: str = ''
    required: bool = False
    repeat: bool = False           # an option given several times (a list)
    nargs: str = None              # positionals: '?', '*', '+'
    metavar: str = None
    dest: str = None
    group: str = None              # a mutually exclusive group of the command
    free: str = ''                 # help.yaml: why a str with no choices is typed, not picked (astro's `free:`)

    @property
    def positional(self):
        return not self.name.startswith('-')


@dataclass(frozen=True)
class Command:
    words: tuple                   # ('run',) or ('sim', 'check')
    summary: str
    handler: str = ''              # 'crane.run:main'
    about: str = ''                # what you'll see
    params: tuple = ()
    examples: tuple = ()
    raw: str = ''                  # the low-level equivalent
    json: str = ''                 # the --json document's schema
    notes: str = ''                # at most two plain sentences: what success looks like, the common mistake
    details: str = ''              # the technical part: topics, paths, config keys, env, schemas, exit codes
    today: str = ''                # not built yet: what to use today
    passthrough: bool = False
    flags: tuple = ()              # the global flags it takes: json, quiet, dry_run
    groups: tuple = ()             # names of required mutually exclusive groups
    export: bool = False           # an entry in help.yaml (astro's help browser)

    @property
    def name(self):
        return 'crane ' + ' '.join(self.words)

    @property
    def noun(self):
        return self.words[0]

    @property
    def verb(self):
        return self.words[1] if len(self.words) > 1 else None


@dataclass(frozen=True)
class Noun:
    name: str
    summary: str
    aliases: tuple = ()
    owner: str = ''                # a tool that owns the noun's words: other verbs and options-only go to it
    verbs: list = field(default_factory=list, compare=False)


def sel(key, **kw):
    """A shared selector (crane/select.py) as a param, with this command's changes."""
    spec = {k: (tuple(v) if isinstance(v, list) else v) for k, v in select.SPECS[key].items()}
    return replace(Param(**spec), **kw)


P = Param
JSON, QUIET, DRY = 'json', 'quiet', 'dry_run'

NOUNS = [
    Noun('sim', 'The live simulator the robot software drives: the full 3-D world (T2) or its light twin (T1)', owner='crane.wrap:sim'),
    Noun('run', 'Simulate many seeds quickly in the light simulators (T0, T1) and save each one'),
    Noun('study', 'Plan, run and analyse an experiment: several settings over many seeds, compared fairly'),
    Noun('runs', 'Find and inspect runs you already have: list, search, show one, compare two'),
    Noun('repro', 'Rerun an old run and check it comes out the same'),
    Noun('data', 'Turn runs into a dataset file for analysis or training, and split it fairly'),
    Noun('monitor', 'See how far running work has got, here and on the cluster'),
    Noun('env', 'Make, check and draw the courses the robot drives', aliases=('course',)),
    Noun('card', "Look at and export a robot's plant card (its measured physics numbers)"),
    Noun('fit', "Turn a day's test bundle into updated simulator settings"),
    Noun('field', 'Turn real-robot recordings into runs crane can replay and compare'),
    Noun('compare', 'Put two sets of runs side by side, e.g. the boat vs the sim'),
    Noun('judge', 'Score runs pass or fail against a set of rules'),
    Noun('check', 'Acceptance checks on recorded results'),
    Noun('view', 'Watch runs replay, or try settings live, in a window'),
    Noun('board', 'Browse and replay runs in your web browser'),
    Noun('media', 'Make videos and screenshots of runs'),
    Noun('bench', 'Measure how fast the simulators run on this computer'),
    Noun('cluster', 'Send a study to the shared compute cluster and bring the results back',
         owner='crane.wrap:cluster'),
    Noun('build', 'Build the simulator program, the docker images or the Python package'),
    Noun('deploy', 'Copy crane (and the simulator) onto another computer you work from'),
    Noun('update', 'Get the latest crane and simulator on this computer'),
    Noun('heavy', 'Take turns on this computer for big jobs (Unity, sims, test suites)'),
    Noun('doctor', 'Check this computer can run the simulator; says what to run next'),
    Noun('setup', 'A guided install of crane on this computer'),
    Noun('help', 'List every command, and keep the help files up to date'),
]

_JSON = 'Print the result as one JSON document instead of text, for scripts (default off).'
_STUDIES = 'Where studies are kept (default CRANE_STUDIES, else ~/.cache/crane/studies).'


_INSTANCE = ('Which parallel world to use, 0 for the default world or 1 and up, e.g. 2: its own ROS_DOMAIN_ID (the usual one plus the '
             'number), port, container and state folder, so several T1 worlds can run side by side (default none: '
             'the one world; the environment variable CRANE_SIM_INSTANCE sets it too).')


def _instance():
    return P('--instance', 'int', metavar='N', default='none', description=_INSTANCE)


def _json(text=_JSON):
    return P('--json', 'bool', description=text)


COMMANDS = [
    # ---- sim -------------------------------------------------------------------------------------------------
    Command(('sim', 'up'), 'Start the simulator and wait until the course is ready to drive.',
            'crane.wrap:sim', passthrough=True, flags=(JSON, DRY), json='crane.sim.status/1', export=True,
            about='a line as each part starts, then a status block (what is running, where the logs are) ending '
                  'with next:.',
            notes='Works with no options: the default course, its normal water current, seed 1, and a window if you '
                  'have a screen; --tier T1 starts the light twin instead (no 3-D world, can run faster). It waits about 6 minutes at most, and refuses if a sim is already running '
                  '(run crane sim down first).',
            details='Starts the Unity player through heavy unity plus its ROS side (the sim_up_ros container), then '
                    'prints the status block (mode, ROS_DOMAIN_ID, endpoint, player pgid, ROS container, course, '
                    'current, logs, next:). Waits up to 60 s for the ROS endpoint and 300 s (SIM_UP_READY_S) for the '
                    'course. --tier T1 (tiers.T1.sim, Docs/SimT1.md): no Unity; a ROS node steps the plant-card twin and '
                    'publishes what the player does (/crane/odom, /imu/raw, /gps/fix, /clock; /odom/mavros with '
                    '--odom sensors; /odom/local, /odom/global and TF with --odom truth, which turns localization off) '
                    'and drives on /thrusters/pwm. --tier T0 refuses naming crane run. '
                    '--dry-run prints the resolved player argv and docker run (crane.sim.plan/1 with --json). Exit 0 '
                    'ready, 1 the world failed, 2 usage, config or a sim already running. Everything specific comes '
                    'from config/domains/DOMAIN.yaml tiers.T2. Env: ROS_DOMAIN_ID, ASTRO_STATE / '
                    'MHSEALS_HARDWARE_CONFIG (passed into the ROS side), the image env, SIM_UP_*, CRANE_PLAYER, HEAVY. '
                    'Same as Tools/RoboBoat/sim_up up.',
            params=(P('--seed', 'int', default=1, description='Which course layout to build; the same number always '
                      'gives the same buoys, e.g. 3 (default 1).'),
                    sel('env', default='course:roboboat_2026', description='Which world to load, written KIND:NAME, '
                        'e.g. course:roboboat_2026 (the default); --course NAME is a shortcut.'),
                    sel('card', type='path', default='none', description="The boat's physics settings: a fitted "
                        'profile file or a card name, e.g. characterized (default none: the built-in boat).'),
                    P('--profile', default='interactive-low', free='a preset name the player defines (config/domains/'
                      'DOMAIN.yaml tiers.T2.profiles)', description="The simulator's graphics and physics quality "
                      'preset, e.g. interactive-high (default interactive-low).'),
                    P('--current', metavar='default|0|SPEED,TOWARD_DEG', default='default',
                      description="The water current: default (the course's own, 0.15 metres per second toward 270 "
                      'degrees), 0 for still water, or SPEED,TOWARD_DEG, e.g. 0.3,90.'),
                    P('--display', choices=('auto', 'window', 'headless'), default='auto',
                      description='auto opens a window when you have a screen and runs without one otherwise; '
                      '--window and --headless are shortcuts.'),
                    P('--player', 'path', description='The simulator program to run, e.g. '
                      'Builds/CRANE-Worker/CRANE.x86_64 (default auto: the newest one found).'),
                    P('--tier', choices=('T2', 'T1', 'T0'), default='T2', description='How detailed the sim is: T2 '
                      'is the full 3-D simulator (default); T1 is the light twin with the same robot software '
                      'connections, no 3-D world, and can run faster; T0 runs only through crane run.'),
                    P('--odom', choices=('sensors', 'truth'), default='sensors', description='T1 only: sensors feeds '
                      "the robot's own position estimate like on the water (default); truth hands it perfect "
                      'positions instead.'),
                    P('--rate', 'float', metavar='N', default=1, description='T1 only: how many times faster than '
                      'real time to run, e.g. 2 (default 1).'),
                    P('--world', metavar='course|pool[:LxW]', default='course', free='pool:LENGTHxWIDTH in metres',
                      description='T1 only: course (the default) or a rectangular pool whose walls the lidar sees, '
                      'pool for 10 by 4.5 metres or pool:LxW for another size, e.g. pool:9x5; the boat starts '
                      '1.5 metres from one end, facing along it. --env pool:LxW does the same.'),
                    sel('robot', default='blastoise', free="a robot name from the domain's config",
                        description='Which robot to put in the world, e.g. blastoise (the default).'),
                    P('--set', metavar='K=V', repeat=True, default='none', free='a course setting name and value',
                      description='Pin one course setting as TASK_ID.PARAM=VALUE, e.g. a gate spacing in '
                      'metres; give it again for more (default none).'),
                    sel('domain', default='boat', free='a domain name: one file per domain in config/domains',
                        description='Which kind of robot world to use, e.g. boat (the default) or sub.'),
                    P('--nav', metavar='NAME:=VALUE', repeat=True, default='none', free='a navigation launch '
                      'argument as NAME:=VALUE', description='Hand the navigation part a launch argument, e.g. '
                      'preset:=pool; give it again for more. A restart of nav reuses them (default none).'),
                    P('--nav-args', metavar='"A:=x B:=y"', default='none', free='several NAME:=VALUE words in one '
                      'quoted text', description='Several --nav at once in one quoted text, e.g. '
                      '"preset:=pool params_file:=/path/variant.yaml"; a path must be under the workspace, '
                      '~/.config/astro or ~/.config/mhseals so the robot container sees it (default none).'),
                    _instance(),
                    P('--dry-run', 'bool', description='Show exactly what would start, without starting anything '
                      '(default off).'),
                    P('--json', 'bool', description='Print the final status as one JSON document for scripts; '
                      'progress goes to stderr.')),
            examples=('crane sim up', 'crane sim up --current 0', 'crane sim up --headless --seed 3',
                      'crane sim up --card characterized --headless', 'crane sim up --tier T1',
                      'crane sim up --tier T1 --odom truth --rate 2', 'crane sim up --tier T1 --world pool',
                      'crane sim up --tier T1 --world pool --nav preset:=pool', 'crane sim up --tier T1 --instance 2',
                      'crane sim up --dry-run'),
            raw='Tools/RoboBoat/sim_up up'),
    Command(('sim', 'down'), 'Stop the simulator that crane sim up started.', 'crane.wrap:sim', passthrough=True,
            flags=(JSON,), json='crane.sim.down/1', export=True, about='one line saying what was stopped.',
            notes='Only stops what crane sim up started; other programs are left alone. Safe to run when nothing '
                  'is running.',
            details="Stops the player's process group (TERM, then KILL after 10 s) and the sim_up_ros container. "
                    'Same as Tools/RoboBoat/sim_up down.',
            params=(_instance(),
                    P('--json', 'bool', description='Print what was stopped as one JSON document, for scripts.')),
            examples=('crane sim down', 'crane sim down --instance 2'), raw='Tools/RoboBoat/sim_up down'),
    Command(('sim', 'restart'), 'Restart one part of the running simulator (navigation, ROS or the 3-D world) and '
            'keep the rest, e.g. after changing navigation settings.',
            'crane.wrap:sim', passthrough=True, flags=(JSON,), json='crane.sim.restart/1', export=True,
            about='one line per step, then `crane sim: PART restarted, ready: ...` and next:.',
            notes='Use nav after editing nav settings: only the navigation restarts, so the course and boat stay '
                  'where they are. Any goal the boat was driving to is lost.',
            details='T1: ros restarts the twin node with the container (from the course start); player refuses. '
                    'nav stops the nav nodes named by the domain recipe (tiers.T2.ros.restart.nav) inside the ROS '
                    'container, starts them again from the same launch (only:=navigation), and waits up to the '
                    "recipe's ready_s (90 s) for bt_navigator to be active; a domain without the recipe refuses and "
                    'names `restart ros`. ros re-runs the saved docker run, waits up to 60 s for the endpoint and '
                    '120 s for the ready nodes. player re-runs the saved player argv and waits up to 300 s '
                    '(SIM_UP_READY_S) for the course. --json: {schema, restarted, ready, at, status}. Exit 0 ok, 1 '
                    'failed, 2 usage, config or nothing running, 75 timed out waiting.',
            params=(P('what', choices=('nav', 'ros', 'player'), description='Which part to restart: nav (navigation '
                      'only), ros (all the robot software) or player (the 3-D world).'),
                    P('words', 'list', nargs='*', metavar='NAME:=VALUE', default='none', free='navigation launch '
                      'arguments as NAME:=VALUE', description='Only with nav: launch arguments for the navigation '
                      'part, e.g. preset:=pool. They replace the ones from the last start; with none, those are '
                      'used again (default none).'),
                    P('--nav-args', metavar='"A:=x B:=y"', default='none', free='several NAME:=VALUE words in one '
                      'quoted text', description='The same words in one quoted text, e.g. "preset:=pool '
                      'params_file:=/path/variant.yaml"; a path must be under the workspace, ~/.config/astro or '
                      '~/.config/mhseals (default none).'),
                    _instance(),
                    P('--json', 'bool', description='Print the result as one JSON document instead of text, for '
                      'scripts.'),
                    sel('domain', free='a domain name: one file per domain in config/domains',
                        description='Which kind of robot world is running, e.g. boat (the default).')),
            examples=('crane sim restart nav', 'crane sim restart nav preset:=pool',
                      'crane sim restart player --json'),
            raw='docker exec CONTAINER pkill ...; docker exec -d CONTAINER ros2 launch ...'),
    Command(('sim', 'status'), 'Show whether the simulator is running and where its parts and logs are; changes '
            'nothing.', 'crane.wrap:sim', passthrough=True, flags=(JSON,), json='crane.sim.status/1', export=True,
            about='the status block (what is running, the course and current, the log files), ending with next:.',
            details='The block: mode, ROS_DOMAIN_ID, endpoint, player, ROS container, course, current, logs, next. '
                    '--json: {schema, domain, mode, player, ros, course, current, logs, next}. Same as '
                    'Tools/RoboBoat/sim_up status.',
            params=(_instance(),
                    P('--json', 'bool', description='Print the status as one JSON document instead of text, for '
                      'scripts.')),
            examples=('crane sim status', 'crane sim status --instance 2'), raw='Tools/RoboBoat/sim_up status'),
    Command(('sim', 'check'), 'Check the running simulator is ready to drive (world, course, boat, robot software, '
            'navigation); can wait until it is.', 'crane.wrap:sim', passthrough=True, flags=(JSON,),
            json='crane.sim.check/1', export=True,
            about='one row per check (ok or FAIL, with a reason), then `sim check: ready` or what failed.',
            notes='Ends with `sim check: ready` when everything is up. Right after crane sim up, give it time with '
                  '--wait 120 rather than checking once.',
            details='Exit 1 when not ready, 75 when --wait SECONDS pass first. --json: {schema, ready, exit, failed, '
                    'notices, checks[{name, ok, detail}]}. `crane sim check CHECK INPUTS` runs an offline '
                    'Tools/SimCheck check on recorded results instead. Same as Tools/RoboBoat/sim_up check.',
            params=(P('--wait', 'int', description='Keep checking every 2 seconds until ready, for at most this many '
                      'seconds, e.g. 120 (default: check once).'),
                    _instance(),
                    P('--json', 'bool', description='Print the checks as one JSON document instead of text, for '
                      'scripts.'),
                    sel('domain', free='a domain name: one file per domain in config/domains',
                        description='Which kind of robot world is running, e.g. boat (the default).')),
            examples=('crane sim check --wait 120',), raw='Tools/RoboBoat/sim_up check'),
    Command(('sim', 'logs'), "Show the last lines of the simulator's logs, to see why something went wrong.",
            'crane.wrap:sim', passthrough=True, export=True,
            about='the last lines of each log, under its file path.',
            details='The logs: player (the Unity player log), stdout (its console output) and ros (the ROS container). '
                    'Same as Tools/RoboBoat/sim_up logs.',
            params=(P('which', 'list', nargs='*', choices=('player', 'stdout', 'ros', 't1'),
                      description='Which logs: player, stdout, ros or t1 (default the running tier\'s).'),
                    P('-n', 'int', default=40, description='How many lines of each log to show, e.g. 100 (default 40).'),
                    _instance()),
            examples=('crane sim logs ros -n 100',), raw='Tools/RoboBoat/sim_up logs'),

    # ---- run -------------------------------------------------------------------------------------------------
    Command(('run',), 'Simulate many seeds quickly in a light simulator (T0 or T1) and save each run in its own '
            'folder; for the full 3-D simulator use crane sim up.',
            'crane.run:main', flags=(JSON, QUIET, DRY), groups=('source',), json='crane.run.summary/1', export=True,
            about='one line per seed (where it was saved, how it ended, sim and wall time), then runs per second and '
                  'next:.',
            notes='You need --domain (or --config) and --seeds, e.g. crane run --domain toy --seeds 0:3. Rerunning '
                  'the same command reuses finished runs instead of running them again.',
            details='Runs go to OUT/VARIANT/s<seed> (default ~/.cache/crane/runs/DOMAIN/TIER/KEY). Missing scenarios '
                    'are generated on demand. T2 is not available here (crane sim up). A run dir holding another run, '
                    'or an incomplete one, is refused unless --force. --dry-run prints the resolved plan; --json '
                    'prints one crane.run.summary/1 document. --engine batched runs every seed at once '
                    '(Docs/Engine.md); jax on a consumer GPU wants --dtype float32 (bench/RESULTS.md). modular: the '
                    'kernel is a port of the capplan truth stepper (FP0 module), not corpus-identified; corpus '
                    'validation pending.',
            params=(sel('domain', group='source', description='Which kind of robot world, e.g. boat (also sub, '
                        'drones, modular, toy); one file each in config/domains.'),
                    P('--config', 'path', group='source', description='Use this simulator config file as it is, '
                      'instead of a domain, e.g. my_domain.yaml.'),
                    sel('tier', choices=('T0', 'T1', 'T2', 'abstract', 'dynamic', 'unity'),
                        description="How detailed: T0 abstract (no physics) or T1 dynamic (physics) (default the "
                        "domain's first tier, T0 with --config)."),
                    sel('seeds', description='Which seeds to run, e.g. 0:10 (0 to 9), 5, or 1,4,9 (required, except '
                        'with --astro-test: 0).'),
                    sel('out', description='Where to save the runs (default ~/.cache/crane/runs/DOMAIN/TIER/...).'),
                    P('--workers', 'int', description="How many processes to run at once, e.g. 8 (default the "
                      "config's)."),
                    sel('set', default=[]),
                    P('--variant', default='base', description='A name for this set of settings, used in the folder '
                      'name, e.g. heavier-hull (default base).'),
                    P('--generate', 'bool', description='Make the scenarios again even if they exist (missing ones are '
                      'always made).'),
                    P('--allow-dirty', 'bool', description='Run even though the code has uncommitted changes.'),
                    P('--force', 'bool', description='Rerun a seed whose folder holds another or an unfinished run '
                      '(the old one is kept as <dir>.stale-N).'),
                    P('--engine', choices=('fastsim', 'batched'), default='fastsim', description='fastsim runs one '
                      'seed at a time; batched runs every seed at once, much faster for many seeds.'),
                    P('--backend', choices=('numpy', 'numba', 'jax'), default='numba', description='With --engine '
                      'batched: numpy (slow, exact reference), numba (every CPU core) or jax (GPU).'),
                    P('--dtype', choices=('float64', 'float32'), description='With --engine batched: number precision '
                      '(default float64; use float32 for jax on a gaming GPU).'),
                    P('--record', 'int', default=0, metavar='K', description='With --engine batched: also save the '
                      'track, keeping every K-th step (default 0: results only).'),
                    P('--replay', 'path', metavar='TIMELINES', description='Replay recorded command timelines from an '
                      'npz file (modular); every seed replays every timeline.'),
                    sel('robot', description="Which robot, e.g. mecanum-pair for modular (default the domain's)."),
                    sel('env', description='Which world, e.g. route:random for modular.'),
                    P('--astro-test', metavar='TEST', description="Run one of astro's characterization tests in the "
                      'sim, e.g. speed-surge or all, with results named as astro names them.'),
                    sel('card', repeat=True, description="The robot's physics numbers: a profile file or a card name "
                        "from the domain's card folder, e.g. /tmp/fit/profile.json (repeatable)."),
                    P('--tests', 'path', description="With --astro-test: astro's tests file, or a bundle holding one."),
                    P('--space', description="With --astro-test: which size of test space, e.g. large (the default)."),
                    P('--amp', 'float', description="With --astro-test: how hard to push, as an effort fraction, e.g. "
                      "0.5 (default the test's)."),
                    _json(),
                    P('--dry-run', 'bool', description='Show what would run; run nothing.')),
            examples=('crane run --domain toy --seeds 0:3',
                      'crane run --domain boat --tier T1 --seeds 0:2 --out /tmp/runs',
                      'crane run --domain sub --tier T1 --engine batched --seeds 0:64',
                      'crane run --domain boat --tier T1 --astro-test speed-surge --card /tmp/fit/profile.json',
                      'crane run --domain modular --replay timelines.npz --seeds 0:24 --card example#plated'),
            raw='python3 -m fastsim.batch --config CONFIG --seeds 0:3 --out OUT'),

    # ---- study ----------------------------------------------------------------------------------------------
    Command(('study', 'init'), 'Write a starter study file, with comments, for you to edit.',
            'crane.study.cli:init_main', flags=(JSON,), json='crane.study.init/1',
            about='the study file on screen, or `wrote FILE` and next:.',
            details='The file is a crane.study/1 spec: arms (the settings to compare), axes, seed blocks, stages.',
            params=(sel('domain', description='Which kind of robot world its runs use, e.g. boat (default toy).'),
                    P('--out', 'path', metavar='FILE', description='Save it to this file instead of printing it.'),
                    P('--force', 'bool', description='Overwrite FILE if it exists.')),
            examples=('crane study init --out study.yaml', 'crane study init --domain boat > boat_study.yaml')),
    Command(('study', 'plan'), 'Show how big a study is and roughly how long it will take, before running '
            'anything.', 'crane.study.cli:plan_main', flags=(JSON,), json='crane.study.plan/1',
            about='one row per stage (how many runs, how many already done, how many jobs, estimated time), the seed '
                  'groups, then next:.',
            notes='Nothing runs, so it is safe to try. A mistake in the study file is one line naming the setting '
                  'and the fix.',
            details='Rows: tiers, cells, units, done, held-out units, jobs, est, per job; plus seed blocks, splits and '
                    "pending adaptive stages. --json adds jobs[] (job_id, cell, seeds, est_seconds, kind, campaign, "
                    "purpose, expect, pin, argv): the cluster compiler's input. Estimates use pack.unit_s; jobs pack a "
                    "cell's seeds to about pack.target_s (default 900 s) each. A spec error exits 2.",
            params=(P('spec', 'path', description='The study file (YAML).'),
                    P('--stage', description='Only this stage, e.g. A.'),
                    P('--block', description='Only this group of seeds, e.g. dev.'),
                    P('--out', 'path', description="Where the study's results are, to count what is done (default "
                      '~/.cache/crane/studies/NAME).'),
                    _json('Print the plan, with every job, as one JSON document.')),
            examples=('crane study plan examples/studies/toy_paired.yaml',
                      'crane study plan examples/studies/boat_cards.yaml --json')),
    Command(('study', 'run'), 'Run a study on this computer, picking up where it stopped.',
            'crane.study.cli:run_main', flags=(JSON, QUIET, DRY), json='crane.study.run/1',
            about='a progress line each minute, then one row per stage (runs done, reused, failed), and where the '
                  'results are, then next:.',
            notes='Rerunning the same command only runs what is missing. Held-out seeds (kept back for the final '
                  'test) only run with --block NAME --confirm, once the targets file is committed.',
            details='Writes OUT/progress.json at least every 60 s (CRANE_RUN_DIR overrides where). Units are cached '
                    'in OUT/cache/records.jsonl on (study, stage, arm, cell hash, seed, card hash, code commit) within '
                    '1 s of finishing. A held-out block refuses (exit 2) without --confirm, or when the spec\'s '
                    'targets file is not committed, has local changes or is not older than now. --where cluster: use '
                    'crane study compile, then research submit (or crane cluster submit). Exit 0 ok, 1 units failed, '
                    '2 spec, usage or refusal.',
            params=(P('spec', 'path', description='The study file (YAML).'),
                    P('--stage', description='Only this stage, e.g. V (the stage it reads from must be done).'),
                    P('--block', description='Only this group of seeds, e.g. dev (a held-out one needs --confirm).'),
                    P('--where', default='local', choices=('local', 'cluster'),
                      description='local runs here; for the cluster use crane cluster submit.'),
                    P('--confirm', 'bool', description='Allow the held-out seeds to run (needs the committed targets '
                      'file).'),
                    P('--max-workers', 'int', default=1, description='How many processes to run at once, e.g. 4.'),
                    P('--out', 'path', description='Where to save results (default ~/.cache/crane/studies/NAME).'),
                    _json('Print the summary as one JSON document; progress lines go to stderr.'),
                    P('--dry-run', 'bool', description='Show the plan (like study plan); run nothing.')),
            examples=('crane study run examples/studies/toy_paired.yaml --max-workers 4',
                      'crane study run examples/studies/toy_paired.yaml --block test --confirm',
                      'crane study run examples/studies/adaptive.yaml --stage V')),
    Command(('study', 'status'), 'Show how far a study has got, any failures, and early results on the practice '
            'seeds.', 'crane.study.cli:status_main', flags=(JSON,), json='crane.study.status/1',
            about='one row per stage and seed group (runs done, percent), the latest failure with its error, early '
                  'averages per setting, then next:.',
            notes='Held-out seeds show progress only, never results, until they finish. Exit 1 means some runs '
                  'failed; the error is shown.',
            details='Rows: stage, block, role, units, done, %; the progress.json line; the latest failure and its '
                    'traceback tail; interim dev means per arm.',
            params=(P('target', 'path', metavar='SPEC|OUT', description='The study file, or its results folder.'),
                    P('--out', 'path', description='With a study file: its results folder (default '
                      '~/.cache/crane/studies/NAME).'),
                    _json()),
            examples=('crane study status examples/studies/toy_paired.yaml',
                      'crane study status ~/.cache/crane/studies/toy_paired')),
    Command(('study', 'report'), 'Analyse a finished study: how much each setting beats or trails the baseline, and '
            'whether the difference is real or chance.',
            'crane.study.cli:report_main', flags=(JSON,), json='crane.study.report/1',
            about='a table (setting minus baseline, the difference, its 95% range, the p-value) and where report.md '
                  'and report.json were saved.',
            notes='Each setting is compared with the baseline on the same seeds, so luck cancels out. Held-out seeds '
                  'count only once all of them have finished.',
            details='Paired on the seed per cell: bootstrap CI, sign-flip permutation, Holm across cells, via '
                    'judge.stats. Reads OUT/manifest.json and its rows (or every */manifest.json under OUT, as a '
                    'fetched cluster output).',
            params=(P('out', 'path', metavar='OUT', description="The study's results folder."),
                    P('--baseline', description="The setting the others are compared with, e.g. base (default the "
                      "study file's)."),
                    P('--metric', repeat=True, description='Only these results, e.g. succeeded (repeatable).'),
                    P('--bootstrap', 'int', default=2000, description='How many resamples for the ranges (more is '
                      'slower and steadier).'),
                    _json('Print the report as one JSON document (also saved to OUT/report.json).')),
            examples=('crane study report ~/.cache/crane/studies/toy_paired',
                      'crane study report /tmp/study --metric succeeded --json')),
    Command(('study', 'promote'), "Take the best settings from one stage of a study and write them into the next "
            'stage, ready to run.', 'crane.study.cli:promote_main', flags=(JSON,), json='crane.study.promote/1',
            about='the ranking (which settings were chosen) and the new study file, then next:.',
            details="Freezes an adaptive stage's selection (top-k of the stage it reads, its select: key) into a spec "
                    'whose stage names the chosen arms.',
            params=(P('spec', 'path', description='The study file (YAML).'),
                    P('--stage', required=True, description='The stage that chooses from an earlier one, e.g. V.'),
                    P('--out', 'path', description='The results folder holding the earlier stage (default '
                      '~/.cache/crane/studies/NAME).'),
                    P('--write', 'path', metavar='FILE', description='Where to save the new study file (default '
                      'SPEC.STAGE.yaml beside it).'),
                    _json()),
            examples=('crane study promote examples/studies/adaptive.yaml --stage V',)),
    Command(('study', 'run-cell'), 'Run one piece of a study into a folder; this is what each cluster job runs, '
            'you rarely call it yourself.', 'crane.study.cli:run_cell_main',
            flags=(JSON,), json='crane.study.run_cell/1',
            about='one line: ran, reused, failed, rows and the folder. Exit 1 when runs failed, 2 a refusal.',
            details='Writes rows, manifest.json with sha256, metrics.json and progress.json. JOB_ID is a '
                    'jobs[].job_id of crane study plan --json (CELL_ID, or CELL_ID.jK for one seed chunk). Held-out '
                    'cells need --confirm and the committed targets file, as study run, or on a box with no git '
                    'checkout --targets-sha256 from the compiled job. metrics.json holds at most 1000 finite numbers; '
                    'for held-out cells only unit counts.',
            params=(P('spec', 'path', description='The study file (YAML).'),
                    P('--cell', required=True, metavar='JOB_ID', description='Which piece, by its job id from study '
                      'plan --json.'),
                    P('--out', 'path', required=True, metavar='DIR', description='The folder to write into.'),
                    P('--seeds', metavar='A,B,...', description='Only these of its seeds, e.g. 0,1,2.'),
                    P('--confirm', 'bool', description='Allow a held-out piece to run.'),
                    P('--targets-sha256', metavar='SHA', description='The checked targets file fingerprint, for '
                      'machines with no git (crane study compile fills it in).'),
                    _json()),
            examples=('crane study run-cell study.yaml --cell JOB_ID --out /workspace/out/STUDY/JOB_ID',)),
    Command(('study', 'compile'), 'Turn a study into the list of cluster jobs, without sending them; crane cluster '
            'submit does this for you.', 'crane.study.cli:compile_main', flags=(JSON,), json='crane.study.compile/1',
            about='the job list on screen, or `N jobs in FILE` and next: research submit.',
            notes='The study needs a purpose: line and a time estimate per run. Jobs run the exact committed code, '
                  'so commit first.',
            details="rc-job/1 bodies for the research cluster: one crane_batch or gpu_command job per cell seed chunk "
                    "(5-30 min), each running crane study run-cell through crane-batch. Needs purpose (20-600 chars, "
                    "no local path) and pack.unit_s. Jobs pin crane_ml (and the spec's repo) at the exact commit, "
                    'carry dedup ids from the unit keys and the commit, needs from pin:, campaign and expect; units '
                    "in the study's unit cache are left out; field data is refused. Held-out jobs are left out "
                    'without --confirm; with it the targets file must be committed, clean and older than now, and '
                    'each held-out job pins its sha256, which crane-batch passes to run-cell as --targets-sha256. '
                    'Exit 2 on a refusal.',
            params=(P('spec', 'path', description='The study file (YAML).'),
                    P('--stage', description='Only this stage, e.g. A.'),
                    P('--block', description='Only this group of seeds, e.g. dev.'),
                    P('--confirm', 'bool', description='Include the held-out seeds (needs the committed targets '
                      'file).'),
                    P('--allow-dirty', 'bool', dest='allow_dirty', description='Go ahead with uncommitted changes '
                      '(the jobs still run the last commit).'),
                    P('--write', 'path', metavar='FILE', description='Save the job list to this file instead of '
                      'printing it.'),
                    _json('Print the job list as one JSON document.')),
            examples=('crane study compile examples/studies/toy_paired.yaml --write /tmp/jobs.json',
                      'crane study compile examples/studies/toy_paired.yaml --block test --confirm --json')),

    # ---- runs, repro, data, monitor ---------------------------------------------------------------------------
    Command(('runs', 'ls'), 'List the runs you have, newest first, with how each one ended.',
            'crane.runs.query:ls_main', flags=(JSON,), json='crane.runs.list/1',
            about='a table (run id, domain, tier, seed, outcome, setting, date, any results you asked for, folder), '
                  'then counts and next:.',
            notes='The first time, it finds runs under ~/.cache/crane/runs by itself; add other folders with crane '
                  'runs index. Held-out runs that are not finished show `sealed` instead of results.',
            details='The index is ~/.cache/crane/index.sqlite (CRANE_INDEX moves it). --cluster adds the '
                    "coordinator's runs of --study (GET /v1/runs/query, read-only; CRANE_COORDINATOR, and "
                    'CRANE_VIEWER_TOKEN or CRANE_TOKEN); offline it says so on stderr and lists the local runs. A '
                    'sealed run stores no outcome or metrics, and stderr says how many. --json: {schema, index, n, '
                    'matched, indexed, runs}.',
            params=(sel('domain', description='Only runs of this kind of world, e.g. boat.'),
                    sel('tier', description='Only runs at this level of detail, e.g. T1.'),
                    P('--study', description='Only runs of this study, e.g. boat01.'),
                    P('--where', metavar='EXPR', description="Only runs matching a condition, e.g. 'seed>10' (as "
                      'crane runs query).'),
                    P('--metric', repeat=True, metavar='NAME', description='Add a column for this result, e.g. time '
                      '(repeatable).'),
                    P('--limit', 'int', default=50, description='Show at most this many rows (0 for all).'),
                    P('--cluster', 'bool', description="Also list the cluster's runs of --study."),
                    _json()),
            examples=('crane runs ls --domain boat --tier T1', 'crane runs ls --study nlc08 --metric time --limit 20'),
            raw='sqlite3 ~/.cache/crane/index.sqlite "SELECT * FROM runs"'),
    Command(('runs', 'query'), "Find runs that match a condition, e.g. every T1 run that timed out.",
            'crane.runs.query:query_main', flags=(JSON,), json='crane.runs.list/1',
            about='the same table as crane runs ls, for the matching runs.',
            notes="Write conditions like \"outcome=TIMEOUT and metric.time>120\". A typo is one line naming the "
                  'column and showing an example.',
            details='FIELD OP VALUE joined by and / or / not and parentheses; FIELD is a column or metric.NAME. OP is = '
                    '!= < <= > >= or ~ (glob; a value without * or ? matches anywhere) or in (A, B). Columns: '
                    'run_id, path, source, domain, tier, kind, seed, card, backend, dtype, commit, dirty, outcome, '
                    'created, study, stage, arm, block, run_sha256, sealed. A sealed (held-out, not done) run has no '
                    'outcome or metrics, so metric terms never match it. `null` tests for a missing value; commit= '
                    'and run_id= take a hex prefix. Parsed to SQL with bound parameters, never evaluated. A bad '
                    'expression exits 2.',
            params=(P('--where', metavar='EXPR', required=True, description='The condition, e.g. '
                      '"tier=T1 and outcome=TIMEOUT".'),
                    P('--metric', repeat=True, metavar='NAME', description='Add a column for this result, e.g. '
                      'min_clearance_m (repeatable).'),
                    P('--limit', 'int', default=50, description='Show at most this many rows (0 for all).'),
                    P('--study', description='With --cluster: which study to ask the cluster about.'),
                    P('--cluster', 'bool', description="Also search the cluster's runs of --study."),
                    _json()),
            examples=('crane runs query --where "outcome=TIMEOUT and tier=T1 and metric.time>120"',
                      'crane runs query --where "arm in (a, b) and not dirty=true" --metric min_clearance_m')),
    Command(('runs', 'show'), 'Show everything about one run: its settings, results and files.',
            'crane.runs.query:show_main', flags=(JSON,), json='crane.runs.show/1',
            about='the settings (domain, tier, seed, card, code version, outcome), the results, the files with sizes, '
                  'then next: crane view RUN.',
            details='Meta also shows the study fields and identity hashes. --json: {schema, run, meta, files, next}.',
            params=(P('run', metavar='RUN', description='A run folder, or a run id from crane runs ls (the first few '
                      'characters will do).'),
                    _json()),
            examples=('crane runs show 3fa2c1d0', 'crane runs show ~/.cache/crane/runs/toy/T0/base-1a2b3c4d/base/s3')),
    Command(('runs', 'diff'), 'Compare two runs: which settings differ and how the results changed.',
            'crane.runs.query:diff_main', flags=(JSON,), json='crane.runs.diff/1',
            about='`identity: the same run`, or what differs (settings old -> new, scenario, seed, code version), then '
                  'each result that changed with A, B and B-A.',
            details='Identity parts: config, scenario, adapter, seed, core commit. --json: {schema, same_run, '
                    'identity, meta, metrics}.',
            params=(P('a', metavar='A', description='The first run: a folder or a run id.'),
                    P('b', metavar='B', description='The second run: a folder or a run id.'),
                    _json()),
            examples=('crane runs diff 3fa2c1d0 77b0e9aa',)),
    Command(('runs', 'index'), 'Teach crane about runs in other folders, so crane runs ls and query find them.',
            'crane.runs.query:index_main', flags=(JSON,), json='crane.runs.index/1',
            about='each folder, then how many runs were added, changed, unchanged or removed.',
            notes='Only new or changed files are read again, so it is quick to rerun. Runs whose files are gone '
                  'drop out of the list.',
            details='Incremental by mtime, then sha256. Default roots: the ones indexed before, else '
                    '~/.cache/crane/runs. Reads crane.run/1 dirs, fastsim run dirs (their crane.run/1 meta, nothing '
                    'written) and study outputs (manifest.json crane.data.manifest/1 with metrics.json). A remembered '
                    'root that is gone is dropped. Hidden and *.stale-N dirs are skipped. Held-out records stay sealed '
                    'until done (the record says complete, or every progress.json up to the root is phase done) and '
                    'are reread each time.',
            params=(P('roots', 'list', nargs='*', metavar='ROOT', description='Folders to look in, e.g. /tmp/study-out '
                      '(default the ones used before).'),
                    P('--rebuild', 'bool', description='Forget everything and read all folders again.'),
                    _json('Print the counts as one JSON document.')),
            examples=('crane runs index ~/.cache/crane/runs /tmp/study-out', 'crane runs index --rebuild')),
    Command(('repro',), 'Rerun an old run with the exact code it was made with, and check the result is the same.',
            'crane.runs.repro:main', flags=(JSON,), json='crane.repro/1',
            about='for the fingerprint, the files and the results: same or differs; then `repro: reproduced`, or '
                  'where the rerun was kept so you can crane runs diff it.',
            notes='Your own checkout is never touched. It refuses runs made from uncommitted code, since that code '
                  'cannot be recovered.',
            details="Reruns fastsim runs (T0/T1) in a temporary git worktree at the recorded commit: that commit's "
                    'fastsim on the scenario file in the run dir, with the recorded config and provenance; compares '
                    'run_sha256, tree_sha (or the first differing file) and metrics. Refuses with one line (exit 2) a '
                    'dirty-tree run (its diff sha when recorded), a commit no repository here has, or a non-fastsim '
                    'record. The worktree is always removed. With --check, exit 1 when it does not reproduce.',
            params=(P('run', metavar='RUN', description='A run folder, or a run id from crane runs ls.'),
                    P('--check', 'bool', description='Exit with 1 when the rerun differs (for scripts).'),
                    P('--timeout', 'int', default=1800, description='Give up after this many seconds.'),
                    _json()),
            examples=('crane repro 3fa2c1d0 --check', 'crane repro ~/.cache/crane/runs/toy/T0/base-1a2b3c4d/base/s3'),
            raw='git worktree add --detach TMP COMMIT, then fastsim.run.simulate on the recorded inputs'),
    Command(('data', 'export'), 'Turn runs into one dataset file (a table, one row per run) for analysis or '
            'training.', 'crane.data.export:main', flags=(JSON,), json='crane.data.export/1',
            about='one line (rows, columns, files, size), one row per file written, and next:.',
            notes='Pick rows with --where and columns with --select; the output folder must be new or empty. '
                  'Held-out runs are refused unless you add --confirm.',
            details='Writes npz, parquet, csv or jsonl plus manifest.json with a sha256 per file. A run dir gives one '
                    'metrics-first row (seed, outcome, condition.*, metrics, commit, run_sha256) and its recorded '
                    'arrays as obs / traj columns (record: in the domain YAML, Docs/Data.md). Array columns are '
                    'stacked to rows x T x channels (shorter rows NaN-padded, COL@len their lengths); with csv or '
                    'parquet they go to PART.arrays.npz beside each file. Files roll over at 90 MB. Held-out rows: '
                    'role, block_role or tag held-out, or held_out true. --json: {schema, out, manifest, rows, '
                    'columns, files, held_count, nonfinite}. Exit 0 written, 2 usage (no rows, an unknown column, a '
                    'non-empty --out, held-out rows).',
            params=(P('sources', 'list', nargs='+', metavar='SRC', description='Run folders, study results, datasets or '
                      'row files (.parquet .csv .npz .jsonl).'),
                    P('--format', choices=('npz', 'parquet', 'csv', 'jsonl'), default='npz',
                      description='The file type (parquet needs the [data] extra installed).'),
                    P('--select', metavar='COLS', repeat=True, description='Only these columns, comma-separated, e.g. '
                      'seed,outcome (default all).'),
                    P('--where', metavar='EXPR', repeat=True, description="Only rows where a condition holds, e.g. "
                      "'seed>=10'; OP is = != > >= < <= (repeatable, all must hold)."),
                    P('--split', metavar='FILE:NAME', description='Only the rows of one split from crane data splits, '
                      'e.g. splits.json:train.'),
                    P('--confirm', 'bool', description='Include held-out rows (refused without it).'),
                    sel('out', required=True, description='The dataset folder to write (new or empty).'),
                    _json()),
            examples=('crane data export ~/.cache/crane/runs/toy/T0 --format npz --out /tmp/ds',
                      "crane data export /tmp/runs --select seed,outcome,obs --where 'seed>=10' --out /tmp/ds2",
                      'crane data export /tmp/ds --split /tmp/ds/splits.json:train --out /tmp/train'),
            raw='python3 -c "from crane.data.export import export; export([SRC], OUT)"'),
    Command(('data', 'schema'), 'List the columns of a dataset (or of some runs) and how many rows there are.',
            'crane.data.export:schema_main', flags=(JSON,), json='crane.data.schema/1',
            about='one line (rows, columns, files), one row per column (name, type, and for recorded signals their '
                  'rate and channels), then next:.',
            details='--json: {schema, sources, columns, rows, files, arrays}.',
            params=(P('sources', 'list', nargs='+', metavar='SRC', description='A dataset folder, row files or run '
                      'folders.'),
                    _json()),
            examples=('crane data schema /tmp/ds', 'crane data schema ~/.cache/crane/runs/toy/T0 --json'),
            raw='python3 -c "import json; print(json.load(open(\'DS/manifest.json\'))[\'columns\'])"'),
    Command(('data', 'splits'), 'Divide a dataset into train, calibration and test parts, keeping held-out data '
            'out of training.', 'crane.data.splits:main', flags=(JSON, DRY), json='crane.data.splits/1',
            about='one line (units, how they were grouped, rows, file), one row per part (units, rows, fraction), '
                  'the held-out rule, then next:.',
            notes='Everything from one seed (or condition, or timeline) lands in the same part, so nothing leaks '
                  'between parts. A unit keeps its part as the data grows.',
            details='Units go to named splits by a salted hash. A row is held-out when its role / block_role / tag '
                    "column says held-out, held_out is true, its block or seed is a held-out block of --domain's "
                    'seeds, or it matches --held-out; a unit with any held-out row is held-out and never goes to train '
                    'or calib. Writing over an earlier split file refuses to move a unit into train or calib from '
                    'elsewhere. --json: the split file itself plus out and written. Exit 0 written, 2 usage.',
            params=(P('sources', 'list', nargs='+', metavar='SRC', description='A dataset folder, row files or run '
                      'folders.'),
                    P('--by', default='seed', description='What stays together: seed, condition, timeline, or a '
                      'column name, e.g. session.'),
                    P('--ratios', metavar='NAME=F,...', default='train=0.7,calib=0.15,test=0.15',
                      description='Part names and their shares, adding up to 1, e.g. train=0.8,calib=0.2.'),
                    P('--held-out', metavar='EXPR', repeat=True, description='Rows matching this are held-out too, '
                      'e.g. source=real (repeatable).'),
                    sel('domain', description="Also hold out the seeds this domain's config marks as held-out, e.g. "
                        'boat.'),
                    P('--salt', default='crane', description='Change this to get a different random division.'),
                    P('--out', 'path', description='The split file to write (default SRC/splits.json for one '
                      'dataset folder).'),
                    P('--dry-run', 'bool', description='Show the parts; write nothing.'),
                    _json()),
            examples=('crane data splits /tmp/ds --by seed',
                      'crane data splits /tmp/ds --by timeline --held-out source=real --ratios train=0.8,calib=0.2',
                      'crane data splits /tmp/ds --by condition --domain boat --out /tmp/splits.json')),
    Command(('monitor',), 'See how far running studies have got, on this computer and the cluster, in one table.',
            'crane.monitor:main', flags=(JSON,), json='crane.monitor/1',
            about='one row per stage, setting and seed group: done/total, speed, time left, failures, jobs and '
                  'hosts; then any failures with their reason, and next:.',
            notes='Add --follow to keep it updating until the work ends. Early averages appear for practice seeds '
                  'only; held-out seeds show progress only.',
            details='Local: every progress.json under a study (or a dir) with cell.json, failure.json and '
                    'metrics.json beside it. Cluster: when OUT/STUDY/cluster/jobs.json exists, the project job list '
                    '(research token) and the public feed (/v1/feed); unreachable means a note, not a failure. A '
                    'progress.json older than 180 s counts as stalled. Failure reasons: crashed, timed out without '
                    'progress, cancelled. --follow refreshes every --interval s (5..300) and stops when every job has '
                    'ended or after --timeout s (exit 75). --json: {schema, at, where, study, sources, notes, '
                    'finished, groups[{stage, arm, block, role, done, total, failed, rate_per_s, eta_s, hosts, jobs, '
                    'failures, metrics}]}; with --follow one document per refresh (JSONL).',
            params=(P('target', nargs='?', metavar='STUDY|DIR', description='A study name, e.g. boat01, or a folder '
                      '(default every study).'),
                    sel('out', description=_STUDIES),
                    P('--follow', 'bool', description='Keep updating until every job has ended or --timeout passes.'),
                    P('--interval', 'float', default=30.0, description='Seconds between updates (5 to 300).'),
                    P('--timeout', 'float', default=3600.0, description='With --follow: stop after this many '
                      'seconds.'),
                    P('--local', 'bool', description='Only this computer; do not ask the cluster.'),
                    _json('Print one JSON document (one per update with --follow).')),
            examples=('crane monitor boat01', 'crane monitor boat01 --follow --interval 60',
                      'crane monitor ~/.cache/crane/studies/boat01 --local --json'),
            raw='cat OUT/STUDY/**/progress.json; research status -p crane --campaign STUDY'),

    # ---- env (and its alias course) ---------------------------------------------------------------------------
    Command(('env', 'gen'), 'Make new random courses (buoy layouts) from seeds and save them as files.',
            'crane.wrap:course', passthrough=True, flags=(DRY,),
            about='a few CRANE_COURSE_* lines saying what was written.',
            notes='Give --seed N --out FILE.json for one course, or --seeds A:B --out DIR for many. It waits its '
                  'turn for Unity, so it can take a while on a busy computer.',
            details='Runs the Unity course generator headless through heavy unity. Options: [--family F] --seed N '
                    '--out T.json | --seeds A:B --out DIR | --validate DIR --out V.json; --mode, --set task.param=v. '
                    "The family defaults to the generator's. Arguments are checked before the heavy unity slot is "
                    'taken; --dry-run prints the command only.',
            examples=('crane env gen --seed 3 --out /tmp/c3.json', 'crane env gen --seeds 0:50 --out /tmp/courses'),
            raw='bin/heavy unity Tools/RoboBoat/generate.sh'),
    Command(('env', 'validate'), 'Check a course file follows the course rules; exit 0 when it does.',
            'crane.wrap:course', passthrough=True, about='one row per rule, pass or fail.',
            notes='Give it a course file, e.g. one crane env gen made.',
            details='The independent constraint check of a course truth. Options: TRUTH.json [--spec S] [--json].',
            examples=('crane env validate /tmp/c3.json',), raw='python3 Tools/RoboBoat/validate.py'),
    Command(('env', 'render'), 'Draw a course file as a picture (SVG).', 'crane.wrap:course', passthrough=True,
            about='the SVG on stdout; redirect it to a file.',
            notes='Give it a course file; open the SVG in a web browser to look at it.',
            details='Options: TRUTH.json [--track T.jsonl] (a track to draw over it).',
            examples=('crane env render /tmp/c3.json > c3.svg',), raw='python3 Tools/RoboBoat/render.py'),
    Command(('env', 'explain'), "Describe a course file in words: its tasks, objects and checks.",
            today='python3 Tools/RoboBoat/course_truth.py summary TRUTH.json', examples=('crane env explain /tmp/c3.json',)),
    Command(('env', 'diff'), 'Show two course files side by side.',
            today='python3 Tools/RoboBoat/course_truth.py diff A B', examples=('crane env diff a.json b.json',)),
    Command(('env', 'coverage'), 'Show how well a set of courses covers the range of task settings and corner cases.',
            today='python3 Tools/RoboBoat/coverage.py TRUTH.json...', examples=('crane env coverage /tmp/courses',)),

    # ---- card, fit, field, compare, judge, check --------------------------------------------------------------
    Command(('card', 'show'), "Show a robot's plant card: its physics numbers, how sure each one is, and how well "
            'it was checked.', 'crane.wrap:card', passthrough=True, flags=(JSON,), json='crane.card/1', export=True,
            about='a table, one row per group of numbers (value, spread, status), then the check results.',
            details='Rows per block: parameters with their sd and status; then the validation rows. --json: the card '
                    'document (robot, sha, status, blocks, validation).',
            params=(P('ref', nargs='?', description="Which card: FAMILY/ROBOT@HEAD, the first characters of its id, or "
                      "a card folder (default the standard card)."),
                    _json('Print the card as one JSON document.')),
            examples=('crane card show', 'crane card show FAMILY/ROBOT@HEAD --json'), raw='python3 -m plantcard show'),
    Command(('card', 'export'), "Write a robot's plant card in the format one simulator or tool reads.",
            'crane.wrap:card', passthrough=True, export=True, about='JSON on screen, or saved with --out FILE.',
            details='Tier views: unity (T2), fastsim (T0/T1), nlc and control.',
            params=(P('view', choices=('unity', 'nlc', 'control', 'fastsim'), description='Which format: unity (the 3-D '
                      'simulator), fastsim (the light simulators), or control and nlc (for those tools).'),
                    P('ref', nargs='?', description='Which card (default the standard card).'),
                    P('--out', 'path', description='Save it to this file.')),
            examples=('crane card export fastsim --out /tmp/card.json',),
            raw='python3 -m plantcard export'),
    Command(('card', 'fit'), 'Make a plant card from field data.', today='crane fit BUNDLE.tgz --bags DIR',
            examples=('crane card fit bundle.tgz --out /tmp/fit',)),
    Command(('card', 'factorial'), 'Make a set of card variants (e.g. measured vs default for each number) to test '
            'which ones matter.', today='crane fit BUNDLE.tgz --factorial SPEC.yaml',
            examples=('crane card factorial spec.yaml',)),
    Command(('card', 'validate'), 'Check how well a card predicts sessions it was not fitted on.',
            today='crane fit with two or more bundles', examples=('crane card validate FAMILY/ROBOT@HEAD',)),
    Command(('card', 'parity'), "Check a card behaves the same in the light and the full simulator (T1 vs T2).",
            today='crane compare --tiers LOW HIGH', examples=('crane card parity FAMILY/ROBOT@HEAD',)),
    Command(('fit',), "Turn a day's bundle into updated simulator settings, with why for each.", 'crane.fit:main',
            export=True,
            about='each setting with its verdict (measured, provisional, or left at the default) and why, the '
                  'evidence, the plant card, and how well it predicts held-out sessions when you gave two or more.',
            notes='Give it the bundle astro wrote after a day of testing; add --bags DIR for the recordings too, '
                  'which gives a full plant card. Use the result with crane sim up --card OUT/profile.json.',
            details='The bundle goes through the gate to a profile; the bags give evidence for a plant card. Verdicts: '
                    'identified / provisional / prior. Writes OUT/profile.json (sim_up --profile-json), '
                    'OUT/profile.gate.json, OUT/cards and OUT/fit.json. A bundle made without --bags gives the gate '
                    'only. --factorial SPEC.yaml writes card variants and a manifest. Held-out validation runs when '
                    'there are two or more sessions.',
            params=(P('bundles', 'list', nargs='*', metavar='BUNDLE', description="Astro's test bundles (.tgz), one "
                      'per session, e.g. bundle-173031.tgz.'),
                    P('--out', 'path', description='Where to save the results, e.g. /tmp/fit (default BUNDLE.fit/ '
                      'beside the first bundle, else ~/.cache/crane/fit/BUNDLE/).'),
                    P('--bags', 'path', repeat=True, metavar='DIR', description='A folder of recordings to read as '
                      'well, e.g. ~/astro_dock/bags/sim/2026-11-24/bags (only read; repeatable).'),
                    P('--base', free='a profile name the gate knows', description='The settings to start from and '
                      'update, e.g. Ideal (the default).'),
                    sel('robot', free='any name; it labels the card', description="The name to give the robot's "
                        'card, e.g. astro/blastoise (default astro/ plus the state file name).'),
                    P('--n', 'int', default=64, description='How many simulated tries per held-out check, e.g. 128 '
                      '(more is slower and steadier).'),
                    P('--no-card', 'bool', description='Stop after the settings; make no plant card (default off).'),
                    P('--factorial', 'path', metavar='SPEC', description='Also write card variants from a plan file '
                      '(measured vs default per number), e.g. spec.yaml.')),
            examples=('crane fit bundle-173031.tgz --bags ~/astro_dock/bags/sim/2026-11-24/bags --out /tmp/fit',),
            raw='python3 -m Tools.Characterization.from_bundle'),
    Command(('field', 'import'), 'Turn a real-robot recording into a run that crane can replay, view and compare '
            'with the sim.', 'crane.field:main', export=True,
            about='one line per recording: where it was saved, how long it is, how many position samples, distance '
                  'travelled, events.',
            details='Reads a bundle (made with --bags), a rosbag2 dir or an .mcap file into crane.run/1 records: the '
                    'odometry trajectory, /events and log.txt lines, frame and clock. --frame ID=enu|ned names an '
                    'odometry frame id that is not a REP 103/105 name.',
            params=(P('source', 'path', required=True, description='A bundle, a recording folder or an .mcap file.'),
                    P('--out', 'path', required=True, description='Where to save: a run folder (one recording) or a '
                      'parent folder (several).'),
                    P('--odom', default='/odom/local', description='The topic holding the robot position.'),
                    P('--events', default='/events', description="The topic holding astro's event messages."),
                    P('--frame', repeat=True, metavar='ID=enu|ned', description='Say which way a position frame faces, '
                      'e.g. odom=ned, when its name does not say.'),
                    P('--hz', 'float', default=20.0, description='How many position samples per second to keep.'),
                    sel('domain', default='boat', description='Which kind of robot it is, e.g. boat.')),
            examples=('crane field import ~/astro_dock/bags/sim/2026-11-24/bags/171208-smoke --out /tmp/field',)),
    Command(('compare',), 'Compare two sets of runs side by side (e.g. the boat vs the sim).', 'crane.compare:main',
            flags=(JSON,), json='crane.compare/1', export=True,
            about='a table on screen and, with --out, compare.json and compare.md. With --bounds, pass or FAIL per row '
                  'and a verdict line.',
            notes='For boat vs sim, give the bundle with --field and the sim runs after it. With --bounds file, exit 1 '
                  'means some result is further off than allowed.',
            details='Kinds: variant A/B on one frame and clock; tier gap (--tiers, A the lower tier); field vs sim per '
                    'astro test (--field BUNDLE RUNS). --bounds file reads config/fidelity/DOMAIN.yaml, --bounds FILE '
                    "a file of your own, plus the fit's gate (--fit DIR, default fit-<bundle>); 'card' is not yet "
                    'available. --fidelity CONFIG replays B\'s goal windows on CONFIG\'s vehicle. --json: {schema, '
                    'kind, result}; with --bounds, result holds rows, gate and verdict. Exit 0 pass, 1 FAIL, 2 usage.',
            params=(P('runs', 'list', nargs='+', metavar='RUNS', description='Two sets of runs, A then B, e.g. '
                      'runs/a runs/b; with --field, just the sim runs.'),
                    P('--tiers', 'bool', description='A is the lighter simulator and B the fuller one; show the gap '
                      '(default off).'),
                    P('--field', 'path', metavar='BUNDLE', description="Compare the runs with a real day's bundle, "
                      'e.g. bundle-173031.tgz.'),
                    P('--metric', repeat=True, free='any result name the runs record', description='Only compare '
                      'these results, e.g. time (repeatable; default all).'),
                    P('--direction', default='higher', choices=('higher', 'lower'), description='With --tiers: '
                      'whether a higher or a lower number is better.'),
                    P('--band', 'float', description="With --tiers: the biggest gap you'll accept, in the result's own "
                      'units, e.g. 0.5.'),
                    P('--fidelity', 'path', metavar='CONFIG', description="With --tiers: also replay B's goals on "
                      "another vehicle's config, e.g. config.yaml."),
                    P('--bounds', metavar='file|card|FILE', free='file, or a path to your own limits file',
                      description="With --field: pass or FAIL each row against allowed limits, e.g. file (the "
                      'standard limits) or your own FILE.'),
                    P('--fit', 'path', metavar='DIR', description='With --bounds: the crane fit output folder to '
                      'check too, e.g. /tmp/fit (default fit-<bundle>).'),
                    P('--domain', default='boat', free='a domain name: one file per domain in config/fidelity',
                      description='With --bounds: which limits file to use, e.g. boat (the default).'),
                    P('--out', 'path', description='Also save compare.json and compare.md here, e.g. /tmp/cmp.'),
                    _json('Print the comparison as one JSON document instead of the table, for scripts.')),
            examples=('crane compare --field bundle-173031.tgz runs/sim',
                      'crane compare --field bundle-173031.tgz runs/sim --bounds file --fit /tmp/fit')),
    Command(('judge', 'fastsim'), 'Score every run in a folder pass or fail with the standard motion rules.',
            'crane.wrap:judge', passthrough=True, flags=(JSON,), json='crane.judge.report/1',
            about='one verdict line; with --out, a full report. Exit 1 on FAIL.',
            details='Judges every fastsim run under ROOT with the generic motion pack; --out writes report.json and '
                    'Markdown.',
            params=(P('root', 'path', description='The folder of runs.'),
                    P('--out', 'path', description='Where to save the report.'),
                    _json('Print the report as one JSON document.')),
            examples=('crane judge fastsim /tmp/runs --out /tmp/runs-judge',), raw='python3 -m judge fastsim'),
    Command(('judge', 'oneline'), "Print a saved report's verdict in one line; exit 1 on FAIL.", 'crane.wrap:judge',
            passthrough=True, flags=(JSON,), params=(P('report', 'path', description='The report.json file.'),),
            examples=('crane judge oneline /tmp/runs-judge/report.json',), raw='python3 -m judge oneline'),
    Command(('judge', 'markdown'), 'Print a saved report as readable Markdown; exit 1 on FAIL.', 'crane.wrap:judge',
            passthrough=True, params=(P('report', 'path', description='The report.json file.'),),
            examples=('crane judge markdown /tmp/runs-judge/report.json',), raw='python3 -m judge markdown'),
    Command(('judge', 'demo'), 'See what judging does on a toy drone: one physics fault, one decision fault and a '
            'clean run.', 'crane.wrap:judge', passthrough=True,
            params=(P('--out', 'path', description='Where to save the reports.'),),
            examples=('crane judge demo',), raw='python3 -m judge demo'),
    Command(('judge', 'run'), 'Score runs against a named set of rules.', today='crane judge fastsim ROOT',
            examples=('crane judge run --pack generic_motion /tmp/runs',)),
    Command(('judge', 'report'), 'Print a saved report as text, Markdown or JSON.',
            today='crane judge markdown REPORT.json', examples=('crane judge report /tmp/runs-judge/report.json',)),
    *[Command(('check', v), s, today=t, examples=(e,)) for v, s, t, e in (
        ('axis', 'Check recorded results use the right directions and signs.', 'crane sim check axis INPUTS',
         'crane check axis results.json'),
        ('sitl', 'The software-in-the-loop acceptance check on recorded results.', 'crane sim check sitl INPUTS',
         'crane check sitl results.json'),
        ('docking', 'The docking acceptance check on recorded results.', 'crane sim check docking INPUTS',
         'crane check docking results.json'),
        ('smoke', 'A quick does-it-basically-work check on recorded results.', 'crane sim check smoke INPUTS',
         'crane check smoke results.json'),
        ('truth-subscribers', "Check no robot software secretly reads the simulator's ground truth.",
         'python3 Tools/RoboBoat/check_truth_subscribers.py', 'crane check truth-subscribers'))],

    # ---- view, board, media -----------------------------------------------------------------------------------
    Command(('view',), 'Watch runs replay, or try settings live.', 'crane.view.app:main', export=True,
            about='a window with a top-down map, a timeline of events and play/step/speed controls; live mode adds '
                  'sliders for the settings, Rerun, and Promote.',
            notes='Give run folders to replay them together, or --live to drive a light sim with sliders. Promote '
                  'saves the slider settings as a file crane run can use.',
            details='Replays crane.run/1 dirs, fastsim run dirs or .ctraj files overlaid in one frame and clock '
                    '(--offset and --transform line up runs 2, 3, ...). --live runs a T0/T1 sim in-process; Rerun from '
                    't or 0; Promote writes a crane run spec to --promote-dir. --screenshot renders offscreen to a PNG '
                    'and opens no window.',
            params=(P('runs', 'list', nargs='*', metavar='RUN', description='Run folders or track files to replay '
                      'together, e.g. /tmp/runs/base/s3.'),
                    P('--live', 'bool', description='Run a light sim right here with sliders, instead of replaying '
                      '(default off).'),
                    sel('domain', default='boat', free='a domain name: one file per domain in config/domains',
                        description='With --live: which kind of robot world, e.g. boat (the default).'),
                    sel('tier', default='T1', choices=('T0', 'T1', 'abstract', 'dynamic'), description='With --live: '
                        'T0 (no physics) or T1 (physics, the default).'),
                    sel('seed', default=0, description='With --live: which scenario to load, e.g. 3 (default 0).'),
                    P('--offset', repeat=True, metavar='S', free='a time in seconds', description='For the second and '
                      'later runs: when their start happened on the first run\'s clock, in seconds, e.g. 12.5.'),
                    P('--transform', repeat=True, metavar='JSON', free='a small JSON object',
                      description="For the second and later runs: how to move them onto the first run's map, e.g. "
                      '\'{"xyz": [1, 0, 0], "yaw": 90}\'.'),
                    P('--3d', 'bool', dest='three_d', description='Open in the 3-D view, for robots that move in all '
                      'directions.'),
                    P('--screenshot', 'path', metavar='OUT.png', description='Save a picture to this file instead of '
                      'opening a window, e.g. /tmp/view.png.'),
                    P('--at', 'float', description='With --screenshot: the moment to capture, in seconds, e.g. 60 '
                      '(default the end).'),
                    P('--promote-dir', 'path', default='runs/promoted', description='Where Promote saves settings '
                      'files, e.g. runs/promoted (the default).')),
            examples=('crane view --live --domain boat --tier T1 --seed 3',
                      'crane view /tmp/runs/base/s3 --screenshot /tmp/view.png --at 60')),
    Command(('board',), 'Browse and replay your runs, courses and reports in a web browser.', 'crane.wrap:board',
            passthrough=True, export=True,
            about='a local web address (default http://127.0.0.1:8765) to open; it lists every run under the folders, '
                  'with their events and course layers.',
            details='Indexes crane.run/1 records and .ctraj files, course truths and judge reports under ROOTS.',
            params=(P('roots', 'list', nargs='*', description='Folders to look in (default the current one).'),
                    P('--port', 'int', default=8765, description='The local port to serve on.'),
                    P('--host', default='127.0.0.1', description='The address to listen on (the default is only this '
                      'computer).')),
            examples=('crane board /tmp/runs',), raw='python3 -m viewer serve'),
    *[Command(('media', v), s, today=t, examples=(e,)) for v, s, t, e in (
        ('render', 'Make a video of runs.', 'python3 -m theater select ... then theater/pack',
         'crane media render RUNS --out /tmp/media'),
        ('shot', 'Take a screenshot of the board or a run.',
         'crane view RUN --screenshot OUT.png, or python3 -m viewer shot', 'crane media shot RUN --out /tmp/shot.png'),
        ('assemble', 'Join rendered clips into one video.', 'ffmpeg on the rendered clips',
         'crane media assemble /tmp/media'))],

    # ---- bench, cluster, build, deploy, update, heavy, doctor, setup ------------------------------------------
    Command(('bench', 'player'), 'Measure how much faster than real time the 3-D simulator runs here, with one '
            'or several copies at once.', 'crane.wrap:bench', export=True,
            about='results per copy under PerformanceResults/ and, when trying several counts, a summary for each.',
            details='Runs the Unity player through heavy unity: one worker, or a sweep over worker counts '
                    '(summary.json per count). Player settings come from env (CRANE_PLAYER, CRANE_DURATION, '
                    'CRANE_SCENE, ...).',
            params=(P('--worker', 'int', default=0, description='Run a single copy, numbered with this id.'),
                    P('--sweep', 'list', nargs='*', metavar='N', description='Try these numbers of copies at once, '
                      'e.g. 1 2 4 (default from config/bench.yaml).')),
            examples=('crane bench player --sweep 1 2 4',), raw='bin/heavy unity Tools/Bench/sweep_workers.sh 1 2 4'),
    Command(('bench', 'fast'), 'Measure how many runs per second the light simulators (T0, T1) manage here.',
            'bench.fast:crane_main', passthrough=True, export=True,
            about='tables of runs per second, saved as bench.json/bench.md (or batched.json/batched.md).',
            details='The scalar table B, the batched engine (--engine batched: episodes/s per backend and E, x table '
                    'B, the time split) or the gate-3 equivalence report (--equivalence). Arguments go to bench.fast '
                    'unchanged (python3 -m bench.fast --help).',
            params=(P('args', 'list', nargs='*', metavar='OPTION', description="bench.fast's own options, e.g. "
                      '--domain boat --tier T1 --seeds 0:50 (python3 -m bench.fast --help lists them).'),),
            examples=('crane bench fast --engine batched --domain boat --tier T1 --backend numba,jax --E 1024,4096 '
                      '--out /tmp/bench', 'crane bench fast --equivalence --tier T1 --seeds 0:500',
                      'crane bench fast --domain boat --tier T1 --seeds 0:50 --out /tmp/b'),
            raw='python3 -m bench.fast'),
    Command(('bench', 'cluster'), 'Measure speed and cost on the cluster.', today='python3 -m bench.fast with its '
            '--fresh-jobs, on a cluster host', examples=('crane bench cluster --domain boat',)),
    Command(('cluster', 'submit'), 'Send a study to the shared compute cluster as a batch of jobs.',
            'crane.cluster.submit:submit_main', flags=(JSON, DRY), json='crane.cluster.submit/1',
            about='the plan (jobs per stage and setting, time per job, total compute hours) and any warnings, then '
                  'how many jobs were created or already there, and next:.',
            notes='Try --dry-run first to see the plan and cost. Commit your changes first: jobs run the exact '
                  'committed code, and sending the same plan twice does not duplicate jobs.',
            details="Compiles a crane.study/1 spec to rc-job/1 jobs (one per cell x seed chunk, 5-30 min each, pinned "
                    'to the exact commit) and submits them with the research CLI; a jobs.json goes as is. Jobs: kind '
                    'crane_batch (one slot, every core) or gpu_command for a GPU cell; seeds [0] with the chunk in '
                    'spec.seeds; spec.run and spec.argv run python3 -m crane_backends.batch, which runs crane study '
                    'run-cell SPEC --cell JOB --out .. (--confirm --targets-sha256 on held-out jobs); job ids come '
                    'from the unit cache keys and the commit, so resubmitting is idempotent. Refused (exit 2): '
                    'uncommitted changes (--allow-dirty runs the commit), a spec outside crane/runtime.sparse, a cell '
                    'naming field data (~/field_corpus_*, a bag dir, .mcap/.db3/.bag), a held-out block whose targets '
                    'file is not committed before now, a cell with no time estimate. Without --confirm held-out cells '
                    'are left out (only held-out selected: exit 2). The plan is saved to OUT/STUDY/cluster/jobs.json '
                    'before research submit. An adaptive stage goes after its source: submit --stage A, wait, fetch, '
                    'crane study promote --stage B, submit the promoted spec. --cloud adds needs.tags cloud-ok and '
                    'cpu (torch for GPU cells) and needs an approved REQUESTS.md row.',
            params=(P('source', metavar='SPEC|JOBS.json', description='A study file, or a ready job list.'),
                    P('--stage', description='Only this stage, e.g. A.'),
                    P('--block', description='Only this group of seeds, e.g. dev.'),
                    P('--est-unit-s', 'float', dest='est_unit_s', description='Seconds one run takes, to size the '
                      "jobs, e.g. 12 (default the study's estimate)."),
                    P('--est-from', 'path', dest='est_from', description='Take the time per run from a crane bench '
                      'fast result (bench.json).'),
                    P('--measure', 'bool', description='Time one run here first to size the jobs (never for T2).'),
                    P('--project', default='crane', description='The cluster project to submit under.'),
                    P('--campaign', description='A label grouping the jobs (default the study name).'),
                    P('--purpose', description='One or two sentences on why, shown publicly on the cluster feed '
                      '(default the study file\'s purpose:).'),
                    P('--tag', repeat=True, description='Only run on machines with this tag, e.g. gpu (repeatable; '
                      'all required).'),
                    P('--cloud', 'bool', description='Allow rented cloud machines (only with an approved request).'),
                    P('--kind', choices=('crane_batch', 'gpu_command'), description='Force CPU (crane_batch) or GPU '
                      "(gpu_command) jobs (default the study file's choice)."),
                    P('--confirm', 'bool', description='Include the held-out seeds (needs the committed targets '
                      'file).'),
                    P('--allow-dirty', 'bool', dest='allow_dirty', description='Go ahead with uncommitted changes '
                      '(the jobs still run the last commit).'),
                    P('--no-skip-done', 'bool', dest='no_skip_done', description='Run seeds again even if their results '
                      'are already saved.'),
                    sel('out', description=_STUDIES),
                    P('--dry-run', 'bool', description='Show the plan; send nothing.'),
                    _json()),
            examples=('crane cluster submit study.yaml --dry-run', 'crane cluster submit study.yaml --est-unit-s 12',
                      'crane cluster submit study.yaml --stage A --block dev --cloud',
                      'crane cluster submit /tmp/jobs.json'),
            raw='research submit -p crane OUT/STUDY/cluster/jobs.json'),
    Command(('cluster', 'wait'), "Wait until a study's cluster jobs finish (up to a time limit), then show how they "
            'went.', 'crane.cluster.submit:wait_main', flags=(JSON,), json='crane.cluster.status/1',
            about='one line saying what it waits for and for how long, then the status table (as cluster status).',
            notes='Exit 0 means every job finished. If the time limit passes first the jobs keep running; run it '
                  'again or check with crane cluster status.',
            details='One research wait. Exit 0 every job done, 1 some failed or were cancelled, 75 the timeout passed '
                    'first (the jobs are untouched), 2 no plan or no research CLI.',
            params=(P('study', nargs='?', metavar='STUDY|DIR|JOBS.json', description='A study name, e.g. boat01, its '
                      'results folder, or a job list.'),
                    P('--timeout', 'float', default=3600.0, description='Stop waiting after this many seconds.'),
                    sel('out', description=_STUDIES),
                    _json('Print the final status as one JSON document.')),
            examples=('crane cluster wait boat01 --timeout 3600',),
            raw='research wait -p crane JOB... --timeout 3600'),
    Command(('cluster', 'status'), "Show a study's cluster jobs: how many are queued, running, done or failed, and "
            'why any failed.', 'crane.cluster.submit:status_main', flags=(JSON,), json='crane.cluster.status/1',
            about='a header with the counts, one row per stage, setting and seed group, then failures with their '
                  'last error line, and next:.',
            details='Read-only, digest style. --json: {schema, study, project, campaign, states, groups, failures, '
                    'finished}. Without STUDY: research status (every job).',
            params=(P('study', nargs='?', metavar='STUDY|DIR|JOBS.json', description='A study name, its results '
                      'folder, or a job list (default every job).'),
                    sel('out', description=_STUDIES),
                    _json()),
            examples=('crane cluster status boat01',), raw='research status -p crane --campaign STUDY'),
    Command(('cluster', 'fetch'), "Bring a study's finished cluster results back to this computer, laid out as if "
            'it ran here.', 'crane.cluster.fetch:main', flags=(JSON,), json='crane.cluster.fetch/1',
            about='one row per job brought back (fetched, missing or failed, rows, from where), the counts, and '
                  'next:.',
            notes='Safe to run again; it only fetches what is missing. Every file is checked, and exit 1 means one '
                  'arrived damaged.',
            details="Lands in OUT/STUDY/<stage>/<arm>/<cell>/parts/JOB/; the cell's manifest.json lists every part's "
                    'files, each checked by sha256. Box pulls (~/.cache/vast-out/BOX/STUDY/JOB/, CRANE_VAST_OUT) '
                    "first, else the coordinator's artifacts (a viewer token: RC_VIEWER_TOKEN or "
                    '~/.config/research-cluster/platform-viewer.token).',
            params=(P('study', metavar='STUDY|DIR|JOBS.json', description='A study name, e.g. boat01, its results '
                      'folder, or a job list.'),
                    sel('out', description=_STUDIES),
                    P('--boxes-only', 'bool', dest='boxes_only', description='Only take results already copied off '
                      'rented machines; do not ask the cluster.'),
                    _json()),
            examples=('crane cluster fetch boat01',), raw='research wait -p crane JOB... --out DIR; rsync from the box'),
    Command(('build', 'player'), 'Build the 3-D simulator program (no window) from this checkout, labelled with the '
            'code version.', 'crane.build:main', flags=(DRY,), export=True,
            about='the Unity command it runs, then the build label file and, with --tarball, the packed program.',
            notes="Needs this project's Unity editor installed, and waits its turn for Unity. --skip-unity only "
                  're-labels and packs a build you already have.',
            details='Builds the headless Linux player through heavy unity and stamps '
                    'Builds/CRANE-Worker/crane-build-manifest.json with the source commit; --tarball writes '
                    'Builds/crane-player-<commit12>-linux-x86_64.tar.zst and its .sha256.',
            params=(P('--tarball', 'bool', description='Also pack it into one file to copy to other computers.'),
                    P('--skip-unity', 'bool', description='Label and pack an existing build only.'),
                    P('--editor', 'path', description='The Unity editor to use (default UNITY_EDITOR, else the '
                      "project's version)."),
                    P('--builds', 'path', description='The builds folder (default Builds in this checkout).'),
                    P('--dry-run', 'bool', description='Show the Unity command; build nothing.')),
            examples=('crane build player --tarball',)),
    Command(('build', 'images'), 'Build the docker images used on cluster and rented machines (core, gpu, sim), '
            'labelled with the code version.', 'crane.build:images_main', flags=(DRY,),
            about='the docker command per image, then one row per image (name, size) and next:.',
            notes='It never uploads anything; publishing happens in CI. sim needs a packed simulator from crane '
                  'build player --tarball.',
            details='Tags crane:KIND-<commit12>. core < 300 MB compressed; gpu = python:3.12-slim + jax[cuda12] + '
                    'numba, < 4 GB; sim = core + the player + xvfb/Mesa Vulkan, about 1.2 GB. sim takes the tarball '
                    'from --player (default the newest Builds/crane-player-*.tar.* with its .sha256) and is built on '
                    'the core image of the same run. .github/workflows/images.yml pushes lunarzdev/crane:KIND-<sha> '
                    '(never :latest).',
            params=(P('kinds', 'list', nargs='*', choices=('core', 'gpu', 'sim'), metavar='KIND',
                      description='Which images: core, gpu, sim (default core).'),
                    P('--player', 'path', description='For sim: the packed simulator file (its .sha256 beside it).'),
                    P('--repo', default='crane', description='The image name.'),
                    P('--dry-run', 'bool', description='Show the docker commands; build nothing.')),
            examples=('crane build images core gpu', 'crane build images sim --player Builds/crane-player-X.tar.zst'),
            raw='docker build -f Dockerfile.KIND -t crane:KIND-COMMIT .'),
    Command(('build', 'wheel'), 'Build the crane Python package files, with their checksums.', 'crane.build:wheel_main',
            flags=(JSON, DRY), about='the build command, then one row per file (name, size, checksum) and next:.',
            details='The wheel and sdist, by uv build (else python3 -m build), and their sha256.',
            params=(sel('out', description='Where the files go (default ~/.cache/crane/dist).'),
                    P('--dry-run', 'bool', description='Show the build command only.'),
                    _json()),
            examples=('crane build wheel', 'crane build wheel --out dist'), raw='uv build --out-dir DIR'),
    Command(('deploy',), 'Copy this version of crane (and, if you ask, the 3-D simulator) onto another computer '
            'you work from, then check it there.', 'crane.deploy:main',
            flags=(JSON, DRY), json='crane.deploy/1', export=True,
            about='a line on what the other computer has, one row per step as it runs, then the crane doctor check '
                  'from there and next:.',
            notes='Try --dry-run first; it changes nothing there. Robot computers are refused: robot code goes '
                  'through `astro hot`.',
            details="Over ssh: checks the host's disk, syncs the code (git fetch from this machine + ff-only, or "
                    'rsync when it has no checkout), makes its .venv (install.sh), sends the T2 player with --player '
                    '(sha256 checked there, current + prev kept), then runs crane doctor there. Touches only '
                    '~/crane_ml, ~/.local/bin/crane and the player store (~/.local/share/crane/player) on HOST. '
                    "Refuses (exit 2) a robot computer (a host whose roles in astro's registry, "
                    'ASTRO_PROFILES/*/devices.yaml, include a robot role and no operator role; '
                    'config/deploy.yaml), a host with less free space than the player (tarball and unpacked) plus '
                    '1 GB, a checkout that cannot fast-forward, or a ~/crane_ml that is neither a crane checkout nor '
                    'an earlier deploy. The .venv step runs install.sh --yes --no-clone --components LIST there (uv, '
                    'Python 3.12) when HOST has no .venv or --components is given. The player tarball is --tarball, '
                    'else the newest Builds/crane-player-*.tar.* with its .sha256 (a warn row when its commit is not '
                    "the code's), else the release player; rsync --partial resumes a cut transfer. HOST fetches the "
                    'full commit from --from (default ssh://THISHOST/REPO, CRANE_DEPLOY_FROM). local:DIR deploys into '
                    'DIR as the home (no ssh; for tests). Exit 0 ok, 1 a step or the remote doctor failed, 2 usage or '
                    'a refusal.',
            params=(P('host', description='The computer to set up: its ssh name, e.g. lab-pc, or USER@HOST.'),
                    P('--player', 'bool', description='Also copy and install the 3-D simulator.'),
                    P('--tarball', 'path', description='The packed simulator file to send (implies --player).'),
                    P('--ref', default='HEAD', description='Which code version to send: a branch or commit (default '
                      'HEAD, what you have checked out).'),
                    P('--components', metavar='A,B', description='Which parts to install there, e.g. core,fast '
                      '(default core; only when it has none yet or you give this).'),
                    P('--no-venv', 'bool', description='Skip installing the Python environment there.'),
                    P('--from', dest='source', metavar='URL', description='Where the other computer fetches the code '
                      'from (default this computer over ssh).'),
                    P('--dry-run', 'bool', description='Look at the other computer and show the plan; change '
                      'nothing.'),
                    _json()),
            examples=('crane deploy HOST --dry-run', 'crane deploy HOST --player',
                      'crane deploy HOST --components core,fast --player'),
            raw="ssh HOST 'cd ~/crane_ml && git fetch ssh://THISHOST/REPO main && git merge --ff-only FETCH_HEAD'"),
    Command(('update',), 'Get the latest crane and 3-D simulator on this computer.',
            'crane.update:main', flags=(JSON, DRY), json='crane.update/1', export=True,
            about='which release it found, one row per step (code, simulator), then next:.',
            notes='The previous simulator is kept, so a bad update can be rolled back. It refuses when the disk is '
                  'too full, before downloading anything.',
            details='Updates this checkout (git fetch + ff-only) or this install (the release wheel), then the T2 '
                    'player from the release into the player store (sha256 checked, current + prev kept). The release '
                    'is release.json of the latest GitHub Release, or of v<VERSION> (CRANE_RELEASES overrides the base '
                    'URL or names a local dir laid out the same way). A checkout with no release yet still '
                    "fast-forwards. Refuses (exit 2) when the store's disk has less free space than the player "
                    '(tarball and unpacked) plus 1 GB. Exit 1 when a file fails its sha256 check. --tarball installs a '
                    'local player tarball (crane deploy runs it on the host).',
            params=(P('--version', metavar='X.Y.Z', description='Get this release, e.g. 0.3.0 (default the latest).'),
                    P('--player-only', 'bool', description='Only update the simulator.'),
                    P('--no-player', 'bool', description='Only update the code.'),
                    P('--tarball', 'path', description='Install this packed simulator file instead of downloading '
                      'one.'),
                    P('--sha256', metavar='HEX', description="--tarball's checksum (default read from its .sha256 "
                      'file).'),
                    P('--store', 'path', description='Where simulators are kept (default '
                      '~/.local/share/crane/player).'),
                    P('--dry-run', 'bool', description='Show the steps; change nothing.'),
                    _json()),
            examples=('crane update', 'crane update --dry-run', 'crane update --version 0.3.0 --player-only'),
            raw='git fetch && git merge --ff-only @{u}; curl -LO RELEASE/ASSET; sha256sum -c'),
    Command(('heavy',), 'Run a big job (Unity, a sim, a test suite) only when this computer has room, so jobs take '
            'turns instead of freezing it.', 'crane.wrap:heavy', passthrough=True,
            about="the job's own output, once it gets its turn.",
            notes='Put the kind of job and the command after it, e.g. crane heavy unity CMD, or crane heavy contain '
                  'test CMD for a test suite; crane heavy status shows what is running and waiting. A job that waits '
                  'too long gives up with exit 75 rather than hanging.',
            details="heavy's own CLI (bin/heavy): it waits for a slot (bounded, exit 75 on timeout), runs CMD and "
                    'releases it; slots come from config/heavy.yaml. `crane heavy status` lists the slots, the queue '
                    'and ETAs.',
            examples=('crane heavy contain test python3 -m pytest fastsim/tests -q',), raw='bin/heavy'),
    Command(('doctor',), 'Check this computer can run the simulator; prints what to run next.', 'crane.doctor:main',
            flags=(JSON, QUIET), json='crane.doctor/1', export=True,
            about='a table of checks (ok, off, missing, warn or FAIL, each with a reason), then next:.',
            notes='Safe to run any time; it changes nothing. Things marked missing are optional extras; only FAIL '
                  'needs fixing.',
            details='Checks python, the extras, the T2 player and its flags, heavy, astro_dock and the GPU. Exit 1 only '
                    'when the core (python >= 3.10, numpy, pyyaml) is broken. --json: {schema, ok, checks[{name, '
                    'state, detail}], next[]}.',
            params=(P('--json', 'bool', description='Print the checks as one JSON document instead of a table, for '
                      'scripts.'),),
            examples=('crane doctor',)),
    Command(('setup',), 'Install crane on this computer step by step: it looks at the machine, suggests what to '
            'install, then does it.', 'crane.setup:main', flags=(JSON, DRY), json='crane.setup/1', export=True,
            about='what it found (system, cores, GPU, docker, astro, Unity, screen), the choices with sizes and '
                  'times, one line per install step, then crane doctor.',
            notes='Answer the questions, or add --yes to take the suggestions. It never touches astro\'s own images '
                  'or workspaces.',
            details='Interactive with rich and questionary (the [setup] extra), else plain prompts; --yes takes the '
                    'defaults or --components/--domains. Components: core, fast, gpu, view, field, data (pip extras, '
                    'through install.sh), player (the release player, crane update --player-only), astro (a read-only '
                    'check of the astro integration). The choices are saved to ~/.config/crane/setup.json. --dry-run '
                    'stops after the plan.',
            params=(P('--yes', 'bool', description='Ask nothing: take the suggestions, or the lists given.'),
                    P('--components', metavar='A,B', description='Which parts to install, e.g. core,fast,player '
                      '(default suggested from this computer).'),
                    P('--domains', metavar='A,B', description='Which robot worlds to set up, e.g. boat,sub (default '
                      'all).'),
                    P('--dry-run', 'bool', description='Look and plan; install nothing.'),
                    _json('Print the result as one JSON document (implies --yes).')),
            examples=('crane setup', 'crane setup --yes --components core,fast,player --domains boat',
                      'crane setup --yes --dry-run'),
            raw='./install.sh --yes --components core,fast; crane update --player-only'),

    # ---- help -------------------------------------------------------------------------------------------------
    Command(('help',), 'List every crane command in one line each, or the commands of one noun.', 'crane.cli:help_main',
            flags=(JSON,), about='one line per command; with a noun, its commands and an example each.',
            notes='For everything about one command, run crane NOUN VERB --help.',
            details='--check regenerates help.yaml and Docs/CLI.md into a temp dir and exits 1 when they differ from '
                    'crane/commands.py; --write regenerates them in place.',
            params=(P('topic', 'list', nargs='*', metavar='NOUN', description='A noun (and verb), e.g. sim.'),
                    P('--check', 'bool', description='Exit 1 when the help files are out of date.'),
                    P('--write', 'bool', description='Rewrite the help files (help.yaml and Docs/CLI.md).'),
                    _json('Print every command as one JSON document.')),
            examples=('crane help sim', 'crane help --check')),
]


def _index():
    nouns = {n.name: n for n in NOUNS}
    for c in COMMANDS:
        if c.verb:
            nouns[c.noun].verbs.append(c)
    return nouns


NOUN = _index()
ALIASES = {a: n.name for n in NOUNS for a in n.aliases}
BY_WORDS = {c.words: c for c in COMMANDS}


def noun(name):
    """The Noun for a name or an alias, or None."""
    return NOUN.get(ALIASES.get(name, name))


def find(words):
    """The command for (noun[, verb]) with aliases resolved, or None."""
    n = noun(words[0]) if words else None
    if n is None:
        return None
    return BY_WORDS.get((n.name, *words[1:2])) if n.verbs else BY_WORDS.get((n.name,))


def handler(command):
    """The handler function of a command, or of a 'module:function' ref (imported now)."""
    import importlib
    module, _, fn = (command if isinstance(command, str) else command.handler).partition(':')
    return getattr(importlib.import_module(module), fn)


def unavailable(command):
    """The one-line refusal of a command that is not built yet (exit 2)."""
    return f'{command.name}: not yet available; today use {command.today}'


FLAG_PARAMS = {
    'json': Param('--json', 'bool', description='Print the result as one JSON document instead of text, for scripts.'),
    'quiet': Param('--quiet', 'bool', description='Print only the results and any failures, not the progress lines.'),
    'dry_run': Param('--dry-run', 'bool', description='Show what would happen, step by step, without doing any of it.'),
}
FLAG_OF = {'--json': 'json', '--quiet': 'quiet', '-q': 'quiet', '--dry-run': 'dry_run'}


def params_of(command):
    """The command's params plus its global flags (a param of the same name wins)."""
    names = {p.name for p in command.params}
    return [*command.params, *(FLAG_PARAMS[f] for f in command.flags if FLAG_PARAMS[f].name not in names)]


# ---- help.yaml (astro's help schema) and Docs/CLI.md, generated from the registry ---------------------------

HELP_YAML, CLI_MD = 'help.yaml', 'Docs/CLI.md'
_PLAIN = re.compile(r'^[A-Za-z0-9_$~./(][^#]*$')


def _scalar(value, flow=False):
    """A YAML scalar: plain when it reads back as the same string, else a JSON (double-quoted) string."""
    if isinstance(value, bool):
        return 'true' if value else 'false'
    if isinstance(value, (int, float)):
        return repr(value)
    text = str(value)
    unsafe = (': ' in text or text.endswith(':') or ' #' in text or text != text.strip() or not _PLAIN.match(text)
              or (flow and any(c in text for c in ',[]{}')) or text.lower() in ('true', 'false', 'yes', 'no', 'null',
                                                                              'on', 'off', '~'))
    if not unsafe:
        try:
            float(text)
            unsafe = True
        except ValueError:
            pass
    return json.dumps(text) if unsafe else text


def _yaml_param(param, choices_from=None, auto=False, form=False):
    kind = {'tier': 'str', 'env': 'str'}.get(param.type, param.type)
    d = {'name': param.name if not param.positional else (param.dest or param.name)}
    if kind != 'str':
        d['type'] = kind
    if auto:
        d['default'] = 'auto'
    elif param.default not in (None, False, [], ()) or (form and kind == 'bool'):
        d['default'] = param.default or False
    if param.choices:
        d['choices'] = list(param.choices)
    d['description'] = param.description + (' (repeatable)' if param.repeat and 'repeatable' not in
                                            param.description else '')
    if choices_from:
        d['choices_from'] = choices_from
    elif param.free and kind == 'str' and not param.choices:
        d['free'] = param.free
    if param.positional:
        d['required'] = param.nargs not in ('?', '*')
    elif param.required:
        d['required'] = True
    parts = []
    for k, v in d.items():
        shown = '[' + ', '.join(_scalar(c, True) for c in v) + ']' if isinstance(v, list) else _scalar(v, True)
        parts.append(f'{k}: {shown}')
    return '{' + ', '.join(parts) + '}'


def _entries():
    """help.yaml's entries: the exported commands, one per NOUN VERB (astro's rule: never a verb/command param)."""
    out = []
    for n in NOUNS:
        out += [dict(name=c.name, words=c.words, summary=c.summary, ros=c.raw, notes=c.notes, details=c.details,
                     params=params_of(c), examples=list(c.examples)) for c in COMMANDS if c.export and c.noun == n.name]
    return out


# astro's help browser: `astro help` shows `level: core` entries, `--all` adds `setup`. The everyday commands are core.
CORE = frozenset({'crane sim', 'crane sim up', 'crane sim down', 'crane sim restart', 'crane sim status',
                  'crane sim check', 'crane view', 'crane fit', 'crane compare', 'crane doctor'})
# astro's picker (`choices_from`): a command printing one choice per line, run when the param is missing on a TTY.
_RUNS = 'ls -1td ${CRANE_RUNS:-$HOME/.cache/crane/runs}/*/*/*/*/s* 2>/dev/null | head -n 30'   # DOMAIN/TIER/KEY/VARIANT/sN
PICKERS = {
    ('crane fit', 'bundles'): ('ls -1t ${ASTRO_DATA:-$HOME/astro_dock/bags}/*/bundle-*.tgz '
                               '${ASTRO_DATA:-$HOME/astro_dock/bags}/*/*/bundle-*.tgz 2>/dev/null | head -n 30'),
    ('crane view', 'runs'): _RUNS,
    ('crane compare', 'runs'): _RUNS,
    **{('crane sim up', f'--{w}'): f'{CRANE_RUN} sim up --list {w}' for w in ('seed', 'env', 'current', 'tier')},
}
# Commands whose form runs on Enter-Enter: every param shows a default, switches as off.
FORMS = frozenset({'crane sim up'})
# Params crane detects when unset: `default: auto` in help.yaml (astro shows "(auto: FOUND)"), with the detection
# command as choices_from when there is one (its first line is what auto finds).
_PLAYERS = ('for p in "$CRANE_PLAYER" "${CRANE_DIR:-$HOME/crane_ml}/Builds/CRANE-Worker/CRANE.x86_64" '
            '"$HOME/.cache/crane-phase1/player/CRANE.x86_64" "$HOME/.local/share/crane/player/current/CRANE.x86_64"; '
            'do [ -x "$p" ] && echo "$p"; done; true')
AUTO = {
    ('crane sim up', '--player'): _PLAYERS,
    ('crane build player', '--editor'): None,
    ('crane setup', '--components'): None,
}


def help_yaml():
    lines = ["# crane's commands for astro's help browser and verbs (`astro help crane`). Generated from",
             '# crane/commands.py by `crane help --write`; `crane help --check` fails when it drifts. Schema:',
             "# astro_dock/.devcontainer/astro-help.py. Each run line is crane's launcher, which needs no install.",
             'package: crane',
             'summary: ' + _scalar('Robot simulators and experiments: start the 3-D sim, fit it to a test day, '
                                   'compare boat and sim; crane help lists every command.'),
             'commands:']
    for e in _entries():
        lines += [f'  - name: {_scalar(e["name"])}',
                  f'    run: {_scalar(CRANE_RUN + " " + " ".join(e["words"]))}',
                  f'    summary: {_scalar(e["summary"])}',
                  f'    level: {"core" if e["name"] in CORE else "setup"}']
        for key in ('ros', 'notes', 'details'):
            if e.get(key):
                lines.append(f'    {key}: {_scalar(e[key])}')
        if e['params']:
            lines.append('    params:')
            for p in e['params']:
                key = (e['name'], p.dest or p.name)
                shown = _yaml_param(p, PICKERS.get(key) or AUTO.get(key), key in AUTO, e['name'] in FORMS)
                lines.append(f'      - {shown}')
        if e['examples']:
            lines.append('    examples:')
            lines += [f'      - {_scalar(x)}' for x in e['examples']]
        lines.append('')
    return '\n'.join(lines).rstrip('\n') + '\n'


def _md_cell(text):
    return str(text).replace('|', '\\|')


def cli_md():
    from crane.cli import TOP_EXAMPLES
    lines = ['# crane CLI', '', '```sh', *[ln.strip() for ln in TOP_EXAMPLES.splitlines()[1:] if ln.startswith('  ')],
             '```', '',
             'Generated from `crane/commands.py` by `crane help --write`; `crane help --check` fails when it drifts. '
             '`crane NOUN VERB --help` prints the same entry.', '',
             '## Conventions', '',
             '`crane NOUN [VERB] [selectors] [flags]`. Output is plain aligned text; a failure is one line, `crane NOUN '
             'VERB: cause; fix: CMD`; output ends with `next: CMD` where there is a next step.', '',
             '| Selector | Meaning |', '|---|---|',
             *[f'| `{s["name"]}{" " + s["metavar"] if s.get("metavar") else ""}` | {_md_cell(s["description"])} |'
               for s in select.SPECS.values()], '',
             '| Global flag | Meaning |', '|---|---|',
             *[f'| `{p.name}` | {_md_cell(p.description)} |' for p in FLAG_PARAMS.values()], '',
             'Exit codes: 0 ok; 1 a check or run failed (judge FAIL included); 2 usage or config error; 75 a bounded '
             'wait (heavy, cluster) timed out.', '']
    for n in NOUNS:
        cmds = n.verbs or [BY_WORDS[(n.name,)]]
        alias = f' (alias: {", ".join(n.aliases)})' if n.aliases else ''
        lines += [f'## {n.name}{alias}', '', f'{n.summary[0].upper()}{n.summary[1:]}.', '']
        for c in cmds:
            lines += [f'### {c.name}', '', '```sh', *c.examples, '```', '', c.summary, '']
            if c.today:
                lines += [f'Not yet available; today use `{c.today}`.', '']
            if c.about:
                lines += [f"What you'll see: {c.about}", '']
            if c.notes:
                lines += [c.notes, '']
            shown = [] if c.today else params_of(c)
            if shown:
                lines += ['| Param | Type | Description |', '|---|---|---|']
                for p in shown:
                    default = f' (default `{p.default}`)' if p.default not in (None, False, [], ()) else ''
                    choices = f' One of {", ".join(f"`{x}`" for x in p.choices)}.' if p.choices else ''
                    lines.append(f'| `{p.name}` | {p.type} | {_md_cell(p.description)}{choices}{default} |')
                lines.append('')
            extra = [f'raw: `{c.raw}`' if c.raw else '', f'--json: `{c.json}`' if c.json else '']
            if any(extra):
                lines += ['; '.join(x for x in extra if x), '']
            if c.details:
                lines += [f'Details: {c.details}', '']
    return '\n'.join(lines).rstrip('\n') + '\n'


def as_dict(command):
    return {'name': command.name, 'words': list(command.words), 'summary': command.summary,
            'available': not command.today, 'today': command.today or None, 'raw': command.raw or None,
            'json': command.json or None, 'examples': list(command.examples),
            'params': [{'name': p.name, 'type': p.type, 'default': p.default, 'choices': list(p.choices) or None,
                        'required': p.required, 'description': p.description} for p in params_of(command)]}


def generated():
    return {HELP_YAML: help_yaml(), CLI_MD: cli_md()}


def check(write=False, out=None, root=None):
    """Regenerate help.yaml and Docs/CLI.md into a temp dir and compare with the committed files (exit 1 on a
    difference); write=True writes them in place."""
    import tempfile
    from pathlib import Path
    from crane import output, paths
    out = out or output.Output()
    root = Path(root or paths.ROOT)
    bad = []
    with tempfile.TemporaryDirectory(prefix='crane-help-') as tmp:
        for rel, text in generated().items():
            fresh = Path(tmp) / rel
            fresh.parent.mkdir(parents=True, exist_ok=True)
            fresh.write_text(text)
            target = root / rel
            if write:
                target.parent.mkdir(parents=True, exist_ok=True)
                target.write_text(text)
                out.line(f'  {rel:<12}  written')
                continue
            same = target.is_file() and target.read_text() == fresh.read_text()
            out.line(f'  {rel:<12}  {"ok" if same else "differs"}')
            if not same:
                bad.append(rel)
    if out.doc({'schema': 'crane.help.check/1', 'ok': not bad, 'differs': bad, 'written': write}):
        return output.FAILED if bad else output.OK
    if bad:
        return out.fail('help --check', f'{", ".join(bad)} differ from crane/commands.py', 'crane help --write')
    out.say('help --check: ok' if not write else 'help: regenerated')
    return output.OK
