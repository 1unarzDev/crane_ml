"""The one registry of crane's commands. argparse, `--help`, `crane help`, help.yaml and Docs/CLI.md all come from it.

A command is a noun and an optional verb, with its summary, what you'll see (`about`), params, examples, `raw` (the
low-level equivalent), a JSON schema hint and a handler (`module:function`, imported only when the command runs).
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
    notes: str = ''
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
    export: str = ''               # export the noun itself to help.yaml: the description of its verb param
    verbs: list = field(default_factory=list, compare=False)


def sel(key, **kw):
    """A shared selector (crane/select.py) as a param, with this command's changes."""
    spec = {k: (tuple(v) if isinstance(v, list) else v) for k, v in select.SPECS[key].items()}
    return replace(Param(**spec), **kw)


P = Param
JSON, QUIET, DRY = 'json', 'quiet', 'dry_run'

NOUNS = [
    Noun('sim', 'T2: the Unity world plus ROS, used like the real boat'),
    Noun('run', 'episodes at a tier, one run dir per seed'),
    Noun('study', 'a study spec: arms x axes x seed blocks, run locally or on the cluster'),
    Noun('runs', 'the local run index: list, show, query, diff'),
    Noun('repro', 'rerun one run and check it reproduces'),
    Noun('data', 'datasets from runs: export, schema, splits'),
    Noun('monitor', 'progress of local and cluster work in one view'),
    Noun('env', 'worlds: courses, scenes, worlds (kind from --env)', aliases=('course',)),
    Noun('card', 'plant cards: show, export, fit, validate', export='show or export.'),
    Noun('fit', 'field data to a profile and a plant card'),
    Noun('field', 'field data as crane.run/1 records'),
    Noun('compare', 'compare two run sets'),
    Noun('judge', 'score runs against a judge pack'),
    Noun('check', 'acceptance checks on recorded results'),
    Noun('view', 'the native sandbox window'),
    Noun('board', 'the web replay board'),
    Noun('media', 'renders, shots and assembled videos'),
    Noun('bench', 'throughput benchmarks', export='Which benchmark.'),
    Noun('cluster', 'research-cluster jobs'),
    Noun('build', 'build the player, images and the wheel'),
    Noun('deploy', 'install crane and the player on a host'),
    Noun('update', 'update this checkout from a release'),
    Noun('heavy', 'the machine-wide concurrency guard'),
    Noun('doctor', 'check this machine'),
    Noun('setup', 'the setup wizard'),
    Noun('help', 'the commands, and the help.yaml / Docs/CLI.md check'),
]

COMMANDS = [
    # ---- sim -------------------------------------------------------------------------------------------------
    Command(('sim', 'up'), 'Start the T2 world (the Unity player plus the ROS side) and return once the course is ready.',
            'crane.wrap:sim', passthrough=True,
            about="the player queued on `heavy unity`, then the status block (mode, domain, endpoint, pids, logs, "
                  "next commands). Options: [--seed N] [--course FAMILY] [--profile-json F] [--window|--headless] "
                  "[--player PATH] [--current SPEED,TOWARD_DEG]. Env: ROS_DOMAIN_ID, ASTRO_DATA, ASTRO_STATE, HEAVY, "
                  "ASTRO_IMAGE, SIM_UP_*, CRANE_PLAYER.",
            params=(P('--seed', 'int', description='the course seed'),
                    P('--course', description='the course family'),
                    P('--profile-json', 'path', description='the plant profile the player applies'),
                    P('--headless', 'bool', description='run under xvfb-run (the default without a terminal)'),
                    P('--window', 'bool', description='show the player on your display'),
                    P('--current', metavar='SPEED,TOWARD_DEG', description='the water current (m/s, deg)')),
            examples=('crane sim up --headless --seed 3',), raw='Tools/RoboBoat/sim_up up'),
    Command(('sim', 'down'), 'Stop what sim up started (its player and its ROS container only).', 'crane.wrap:sim',
            passthrough=True, about='one line per stopped process.', examples=('crane sim down',),
            raw='Tools/RoboBoat/sim_up down'),
    Command(('sim', 'status'), 'The status block: mode, domain, port, pids, disturbance, logs, the next commands.',
            'crane.wrap:sim', passthrough=True, flags=(JSON,), json='crane.sim.status/1',
            about='the status block, ending with next:. --json: {schema, exit, fields, text}.',
            params=(P('--json', 'bool', description='one JSON document'),),
            examples=('crane sim status',), raw='Tools/RoboBoat/sim_up status'),
    Command(('sim', 'check'), 'Is the running sim ready (player, course, plant, ROS side, nav stack)? '
            '--wait SECONDS polls until ready; CHECK INPUTS runs an offline Tools/SimCheck check.',
            'crane.wrap:sim', passthrough=True, flags=(JSON,), json='crane.sim.check/1', export=True,
            about='one row per check (ok/FAIL, name, detail), then `sim check: ready` or the failed names. Exit 1 '
                  'when not ready, 75 when --wait SECONDS pass first. --json: {schema, ready, failed, checks[{name, ok, detail}]}.',
            params=(P('--wait', 'int', description='Poll until ready or this many seconds pass.'),
                    P('--json', 'bool', description='One JSON document.')),
            examples=('crane sim check --wait 120',), raw='Tools/RoboBoat/sim_up check'),
    Command(('sim', 'logs'), "The player's and the ROS side's logs.", today='tail -f ~/.cache/sim_up/player.log',
            examples=('crane sim logs',)),

    # ---- run -------------------------------------------------------------------------------------------------
    Command(('run',), 'Run episodes at a tier, one run dir per seed (OUT/VARIANT/s<seed>). T0 abstract and T1 dynamic; '
            'T2 is not yet available (use crane sim up). Missing scenarios are generated on demand.',
            'crane.run:main', flags=(JSON, QUIET, DRY), groups=('source',), json='crane.run.summary/1', export=True,
            about='one line per seed (run dir, outcome, sim and wall time), then runs/s and next:. --dry-run prints '
                  'the resolved plan and runs nothing; --json prints one summary document.',
            params=(sel('domain', group='source', description='A domain: config/domains/DOMAIN.yaml (boat, toy).'),
                    P('--config', 'path', group='source', description='A fastsim domain YAML, used as is.'),
                    sel('tier', choices=('T0', 'T1', 'T2', 'abstract', 'dynamic', 'unity'),
                        description="The tier (default: the domain's first tier, T0 with --config)."),
                    sel('seeds', description='N, A:B (B exclusive), a,b,c, @FILE or block:NAME (required, except '
                        'with --astro-test: 0).'),
                    sel('out', description='Output root; runs go to OUT/VARIANT/s<seed> (default '
                        '~/.cache/crane/runs/DOMAIN/TIER/STAMP).'),
                    P('--workers', 'int', description="Worker processes (default the config's workers)."),
                    sel('set', default=[]),
                    P('--variant', default='base', description='The name of this design or parameter set.'),
                    P('--generate', 'bool', description="Regenerate the seeds' scenarios (missing ones are always "
                      'made).'),
                    P('--allow-dirty', 'bool', description='Run an adapter tree with uncommitted changes.'),
                    P('--force', 'bool', description='Move a run dir holding another run (or an incomplete one) to '
                      '<dir>.stale-N and rerun it; without it such a dir is refused and matching ones are reused.'),
                    P('--astro-test', metavar='TEST', description="An astro test or group (still, couple-*, speed-*, "
                      "steps-*, nav2, all) at T0/T1, with metrics named as astro's result fields."),
                    sel('card', repeat=True, description='With --astro-test: a profile JSON, a plant card or a '
                        'factorial manifest (repeatable).'),
                    P('--tests', 'path', description="With --astro-test: astro's tests file or a bundle holding one."),
                    P('--space', description="With --astro-test: the tests file's space (default large)."),
                    P('--amp', 'float', description="With --astro-test: the effort amplitude (default the space's)."),
                    P('--json', 'bool', description='One summary document.'),
                    P('--dry-run', 'bool', description='Print the resolved plan; run nothing.')),
            examples=('crane run --domain toy --seeds 0:3',
                      'crane run --domain boat --tier T1 --seeds 0:2 --out /tmp/runs',
                      'crane run --domain boat --tier T1 --astro-test speed-surge --card /tmp/fit/profile.json'),
            raw='python3 -m fastsim.batch --config CONFIG --seeds 0:3 --out OUT'),

    # ---- study, runs, repro, data, monitor (not built yet) ----------------------------------------------------
    *[Command(('study', v), s, today=t, examples=(e,)) for v, s, t, e in (
        ('init', 'Write a study spec (crane.study/1) to start from.', 'a hypothesis.yaml beside your runs',
         'crane study init --domain boat > study.yaml'),
        ('plan', 'Print the jobs a study spec compiles to (local batch or rc-job/1).',
         'crane run --domain D --seeds A:B --dry-run', 'crane study plan study.yaml'),
        ('run', 'Run a study: resume, progress, held-out guard.', 'crane run --domain D --tier T --seeds A:B per arm',
         'crane study run study.yaml'),
        ('status', "A study's progress per cell.", 'crane heavy status, or crane cluster status',
         'crane study status study.yaml'),
        ('report', 'Paired analysis per cell (bootstrap, sign-flip, Holm) through judge.stats.',
         'crane compare A B', 'crane study report study.yaml'),
        ('promote', "Rerun a stage's top-k at a higher tier.", 'crane run --tier T1 on the chosen seeds',
         'crane study promote study.yaml --top 5'))],
    *[Command(('runs', v), s, today=t, examples=(e,)) for v, s, t, e in (
        ('ls', 'List local runs, newest first.', 'crane board ~/.cache/crane/runs', 'crane runs ls --domain boat'),
        ('show', "One run's header, metrics and files.", 'cat RUN/run.json', 'crane runs show RUN_ID'),
        ('query', 'Runs matching --where.', 'crane board ~/.cache/crane/runs', 'crane runs query --where tier=T1'),
        ('diff', "Two runs' configs and metrics side by side.", 'crane compare A B', 'crane runs diff RUN_A RUN_B'),
        ('index', 'Rebuild the local run index.', 'crane board ~/.cache/crane/runs', 'crane runs index'))],
    Command(('repro',), 'Rerun one run from its record and check it reproduces.', today='crane run with the run.json '
            'seed, config and overrides, then crane compare', examples=('crane repro RUN_ID --check',),
            params=(P('run_id', description='The run to reproduce.'),)),
    *[Command(('data', v), s, today=t, examples=(e,)) for v, s, t, e in (
        ('export', 'Runs to columnar files (npz or parquet) plus a manifest with sha256.',
         "each run's metrics.json and track.jsonl", 'crane data export RUNS --format npz --out /tmp/ds'),
        ('schema', "A dataset's columns and units.", 'Docs/RunRecord.md', 'crane data schema /tmp/ds'),
        ('splits', "A dataset's train/calib/held-out splits.", 'seed lists per split with --seeds @FILE',
         'crane data splits /tmp/ds'))],
    Command(('monitor',), 'Progress of local and cluster work in one view (progress.json and the cluster feed).',
            today='crane heavy status, and crane cluster status', examples=('crane monitor --follow',)),

    # ---- env (and its alias course) ---------------------------------------------------------------------------
    Command(('env', 'gen'), 'Generate course truth with the Unity course generator (headless, through heavy unity).',
            'crane.wrap:course', passthrough=True, flags=(DRY,),
            about='CRANE_COURSE_* lines. Options: [--family F] --seed N --out T.json | --seeds A:B --out DIR | '
                  "--validate DIR --out V.json; --mode, --set. The family defaults to the generator's. Arguments are "
                  'checked before the heavy unity slot is taken; --dry-run prints the command only.',
            examples=('crane env gen --seed 3 --out /tmp/c3.json', 'crane env gen --seeds 0:50 --out /tmp/courses'),
            raw='bin/heavy unity Tools/RoboBoat/generate.sh'),
    Command(('env', 'validate'), 'The independent constraint check of a course truth; exit 0 when every row passes.',
            'crane.wrap:course', passthrough=True, about='one row per constraint. Options: TRUTH.json [--spec S] '
            '[--json].', examples=('crane env validate /tmp/c3.json',), raw='python3 Tools/RoboBoat/validate.py'),
    Command(('env', 'render'), 'A course truth as an SVG on stdout.', 'crane.wrap:course', passthrough=True,
            about='SVG. Options: TRUTH.json [--track T.jsonl].', examples=('crane env render /tmp/c3.json > c3.svg',),
            raw='python3 Tools/RoboBoat/render.py'),
    Command(('env', 'explain'), "A course truth's tasks, objects and verdicts.",
            today='python3 Tools/RoboBoat/course_truth.py summary TRUTH.json', examples=('crane env explain /tmp/c3.json',)),
    Command(('env', 'diff'), 'Two course truths side by side.', today='python3 Tools/RoboBoat/course_truth.py diff A B',
            examples=('crane env diff a.json b.json',)),
    Command(('env', 'coverage'), 'Task parameters and corner flags binned across course truths.',
            today='python3 Tools/RoboBoat/coverage.py TRUTH.json...', examples=('crane env coverage /tmp/courses',)),

    # ---- card, fit, field, compare, judge, check --------------------------------------------------------------
    Command(('card', 'show'), 'A plant card: one row per block (parameters with their sd and status) and the '
            'validation rows.', 'crane.wrap:card', passthrough=True, flags=(JSON,), json='crane.card/1',
            about='a text table; --json: the card document (robot, sha, status, blocks, validation).',
            params=(P('ref', nargs='?', description="FAMILY/ROBOT@HEAD, a sha prefix or a card dir (default: "
                      "plantcard's default card)."), P('--json', 'bool', description='The card as one JSON document.')),
            examples=('crane card show', 'crane card show FAMILY/ROBOT@HEAD --json'), raw='python3 -m plantcard show'),
    Command(('card', 'export'), "One of a card's tier views (unity | nlc | control | fastsim) as JSON.",
            'crane.wrap:card', passthrough=True, about='JSON on stdout, or --out FILE.',
            params=(P('view', choices=('unity', 'nlc', 'control', 'fastsim'), description='The tier view.'),
                    P('ref', nargs='?', description="The card (default: plantcard's default card)."),
                    P('--out', 'path', description='Write the JSON here.')),
            examples=('crane card export fastsim --out /tmp/card.json',),
            raw='python3 -m plantcard export'),
    Command(('card', 'fit'), 'Fit a card from field data.', today='crane fit BUNDLE.tgz --bags DIR',
            examples=('crane card fit bundle.tgz --out /tmp/fit',)),
    Command(('card', 'factorial'), 'Card variants from a factorial spec, plus a manifest.',
            today='crane fit BUNDLE.tgz --factorial SPEC.yaml', examples=('crane card factorial spec.yaml',)),
    Command(('card', 'validate'), "A card's held-out prediction coverage.", today='crane fit with two or more bundles',
            examples=('crane card validate FAMILY/ROBOT@HEAD',)),
    Command(('card', 'parity'), "A card's tier views agree (T1 vs T2).", today='crane compare --tiers LOW HIGH',
            examples=('crane card parity FAMILY/ROBOT@HEAD',)),
    Command(('fit',), 'An astro bundle to a profile (through the gate) and a plant card (through its bags as '
            "evidence); prints each parameter's status and why.", 'crane.fit:main', export=True,
            about='the gate verdict per parameter (identified / provisional / prior, and why), the evidence, the card '
                  'and the held-out validation when there are two or more sessions.',
            notes='Writes OUT/profile.json (sim_up --profile-json), OUT/profile.gate.json, OUT/cards and OUT/fit.json. '
                  'A bundle made without --bags gives the gate only; --bags DIR reads bags beside it. --factorial '
                  'SPEC.yaml writes card variants and a manifest.',
            params=(P('bundles', 'list', nargs='*', metavar='BUNDLE', description='Astro bundles (.tgz); each is one '
                      'session.'),
                    P('--out', 'path', description='Output dir (default fit-<bundle name>).'),
                    P('--bags', 'path', repeat=True, metavar='DIR', description='A bag dir to read besides the bundle '
                      '(read only; repeatable).'),
                    P('--base', description='The base profile the gate imports into (default Ideal).'),
                    sel('robot', description="The card's robot name (default astro/<state file stem>)."),
                    P('--n', 'int', default=64, description='Rollouts per held-out segment.'),
                    P('--no-card', 'bool', description='Stop after the gate (no evidence, no card).'),
                    P('--factorial', 'path', metavar='SPEC', description='Card variants from a spec (measured vs '
                      'prior levels per property).')),
            examples=('crane fit bundle-173031.tgz --bags ~/astro_dock/bags/sim/2026-11-24/bags --out /tmp/fit',),
            raw='python3 -m Tools.Characterization.from_bundle'),
    Command(('field', 'import'), 'A bundle (made with --bags), a bag dir or an .mcap file to crane.run/1 records: '
            'odometry trajectory, /events and log.txt lines, frame and clock.', 'crane.field:main', export=True,
            about='one line per bag: run dir, duration, odometry samples and rate, path length, events, frame, clock.',
            params=(P('source', 'path', required=True, description='A bundle, a rosbag2 dir or an .mcap file.'),
                    P('--out', 'path', required=True, description='The run dir (one bag) or the root (several).'),
                    P('--odom', default='/odom/local', description='The odometry topic.'),
                    P('--events', default='/events', description='The astro events topic.'),
                    P('--frame', repeat=True, metavar='ID=enu|ned', description='ID=enu|ned for an odometry frame id '
                      'that is not a REP 103/105 name.'),
                    P('--hz', 'float', default=20.0, description='The trajectory rate (Hz).'),
                    sel('domain', default='boat', description="The run's domain.")),
            examples=('crane field import ~/astro_dock/bags/sim/2026-11-24/bags/171208-smoke --out /tmp/field',)),
    Command(('compare',), 'Two run sets as a Markdown table and JSON: variant A/B on one frame and clock, tier gap '
            '(--tiers) or field vs sim per astro test (--field BUNDLE RUNS).', 'crane.compare:main', flags=(JSON,),
            json='crane.compare/1', export=True,
            about='a Markdown table on stdout (--json: {kind, result} instead) and, with --out, compare.json and '
                  'compare.md. With --bounds: pass/FAIL per row, the gate and a verdict line; exit 0 pass, 1 FAIL, '
                  '2 usage.',
            params=(P('runs', 'list', nargs='+', metavar='RUNS', description='Two run sets, A then B; with --field, '
                      'the sim runs.'),
                    P('--tiers', 'bool', description='A is the lower tier, B the higher.'),
                    P('--field', 'path', metavar='BUNDLE', description='An astro bundle to compare the runs against.'),
                    P('--metric', repeat=True, description='Only these metrics (repeatable).'),
                    P('--direction', default='higher', choices=('higher', 'lower'), description='--tiers: better is '
                      'higher or lower.'),
                    P('--band', 'float', description='--tiers: the allowed |low - high| gap (metric units).'),
                    P('--fidelity', 'path', metavar='CONFIG', description="--tiers: also replay B's goal windows on "
                      "CONFIG's vehicle."),
                    P('--bounds', metavar='file|card|FILE', description="--field: pass/FAIL per row against "
                      "config/fidelity/DOMAIN.yaml ('file') or FILE, and the fit's gate; a verdict line, exit 1 on "
                      "FAIL ('card': not yet available)."),
                    P('--fit', 'path', metavar='DIR', description='--bounds: the `crane fit --out` dir whose gate is '
                      'required (default fit-<bundle>).'),
                    P('--domain', default='boat', description='--bounds: config/fidelity/DOMAIN.yaml.'),
                    P('--out', 'path', description='Write compare.json and compare.md here.'),
                    P('--json', 'bool', description='One JSON document: {schema, kind, result}; with --bounds, '
                      'result holds rows, gate and verdict.')),
            examples=('crane compare --field bundle-173031.tgz runs/sim',
                      'crane compare --field bundle-173031.tgz runs/sim --bounds file --fit /tmp/fit')),
    Command(('judge', 'fastsim'), 'Judge every fastsim run under ROOT with the generic motion pack.',
            'crane.wrap:judge', passthrough=True, flags=(JSON,), json='crane.judge.report/1',
            about='the one-line verdict; with --out, report.json and Markdown. Exit 1 on FAIL.',
            params=(P('root', 'path', description='The runs root.'), P('--out', 'path', description='The report dir.'),
                    P('--json', 'bool', description='The report as one JSON document.')),
            examples=('crane judge fastsim /tmp/runs --out /tmp/runs-judge',), raw='python3 -m judge fastsim'),
    Command(('judge', 'oneline'), "A report's one-line verdict; exit 1 on FAIL.", 'crane.wrap:judge', passthrough=True,
            flags=(JSON,), params=(P('report', 'path', description='report.json'),),
            examples=('crane judge oneline /tmp/runs-judge/report.json',), raw='python3 -m judge oneline'),
    Command(('judge', 'markdown'), 'A report as Markdown; exit 1 on FAIL.', 'crane.wrap:judge', passthrough=True,
            params=(P('report', 'path', description='report.json'),),
            examples=('crane judge markdown /tmp/runs-judge/report.json',), raw='python3 -m judge markdown'),
    Command(('judge', 'demo'), 'The toy quadrotor on two tiers: a plant fault, a decision fault and a clean control.',
            'crane.wrap:judge', passthrough=True, params=(P('--out', 'path', description='Write the reports here.'),),
            examples=('crane judge demo',), raw='python3 -m judge demo'),
    Command(('judge', 'run'), 'Judge runs against a named pack.', today='crane judge fastsim ROOT',
            examples=('crane judge run --pack generic_motion /tmp/runs',)),
    Command(('judge', 'report'), 'A report as text, Markdown or JSON.', today='crane judge markdown REPORT.json',
            examples=('crane judge report /tmp/runs-judge/report.json',)),
    *[Command(('check', v), s, today=t, examples=(e,)) for v, s, t, e in (
        ('axis', 'Axis and sign conventions of recorded results.', 'crane sim check axis INPUTS',
         'crane check axis results.json'),
        ('sitl', 'The SITL acceptance check.', 'crane sim check sitl INPUTS', 'crane check sitl results.json'),
        ('docking', 'The docking acceptance check.', 'crane sim check docking INPUTS',
         'crane check docking results.json'),
        ('smoke', 'The smoke check.', 'crane sim check smoke INPUTS', 'crane check smoke results.json'),
        ('truth-subscribers', 'No node outside the allowlist subscribes to /crane/truth/*.',
         'python3 Tools/RoboBoat/check_truth_subscribers.py', 'crane check truth-subscribers'))],

    # ---- view, board, media -----------------------------------------------------------------------------------
    Command(('view',), 'The native sandbox window. Replay runs overlaid in one frame and clock, or run a T0/T1 sim '
            'live with sliders, rerun from t, and Promote to a crane run spec. --screenshot renders offscreen (no '
            'window).', 'crane.view.app:main', export=True,
            about='a window: the top-down map, the event lanes, play/step/speed/scrub; live mode adds the parameter '
                  'sliders, Rerun from t or 0 and Promote. With --screenshot, a PNG and no window.',
            params=(P('runs', 'list', nargs='*', metavar='RUN', description='crane.run/1 dirs, fastsim run dirs or '
                      '.ctraj files.'),
                    P('--live', 'bool', description='Run the sim in-process (with --domain, --tier T0|T1, --seed).'),
                    sel('domain', default='boat', description='live: the domain.'),
                    sel('tier', default='T1', choices=('T0', 'T1', 'abstract', 'dynamic'), description='live: the tier.'),
                    sel('seed', default=0, description='live: the scenario seed.'),
                    P('--offset', repeat=True, metavar='S', description="Run 2, 3, ...: the first run's time of its "
                      't = 0.'),
                    P('--transform', repeat=True, metavar='JSON', description='Run 2, 3, ...: {from, to, xyz, yaw} '
                      "mapping its origin onto the first run's."),
                    P('--3d', 'bool', dest='three_d', description='Open in the 3-D view (6-DOF records).'),
                    P('--screenshot', 'path', metavar='OUT.png', description='Render offscreen to this PNG and exit.'),
                    P('--at', 'float', description='Screenshot: the playhead time (s; default the end).'),
                    P('--promote-dir', 'path', default='runs/promoted', description='Where Promote writes specs.')),
            examples=('crane view --live --domain boat --tier T1 --seed 3',
                      'crane view /tmp/runs/base/s3 --screenshot /tmp/view.png --at 60')),
    Command(('board',), 'The web replay board over local runs (crane.run/1 records and .ctraj), courses and judge '
            'reports; prints its URL.', 'crane.wrap:board', passthrough=True, export=True,
            about='a local URL (default http://127.0.0.1:8765) listing every record under ROOTS, with their event '
                  'lanes and constraint layers.',
            params=(P('roots', 'list', nargs='*', description='Dirs to index (default .).'),
                    P('--port', 'int', default=8765, description='Local port.'),
                    P('--host', default='127.0.0.1', description='The address to bind.')),
            examples=('crane board /tmp/runs',), raw='python3 -m viewer serve'),
    *[Command(('media', v), s, today=t, examples=(e,)) for v, s, t, e in (
        ('render', 'Render runs to video.', 'python3 -m theater select ... then theater/pack',
         'crane media render RUNS --out /tmp/media'),
        ('shot', 'A screenshot of the board or a run.', 'crane view RUN --screenshot OUT.png, or python3 -m viewer shot',
         'crane media shot RUN --out /tmp/shot.png'),
        ('assemble', 'Assemble rendered clips.', 'ffmpeg on the rendered clips', 'crane media assemble /tmp/media'))],

    # ---- bench, cluster, build, deploy, update, heavy, doctor, setup ------------------------------------------
    Command(('bench', 'player'), 'The Unity player real-time factor: one worker, or a sweep over worker counts, '
            'through heavy unity.', 'crane.wrap:bench',
            about='per-worker results under PerformanceResults/ and, for a sweep, a summary.json per worker count. '
                  'Player settings come from env (CRANE_PLAYER, CRANE_DURATION, CRANE_SCENE, ...).',
            params=(P('--worker', 'int', default=0, description='Run one worker with this id.'),
                    P('--sweep', 'list', nargs='*', metavar='N', description='Worker counts to sweep (default '
                      'config/bench.yaml).')),
            examples=('crane bench player --sweep 1 2 4',), raw='bin/heavy unity Tools/Bench/sweep_workers.sh 1 2 4'),
    Command(('bench', 'fast'), 'The T0/T1 throughput table.', today='python3 -m bench.fast --domain boat --tier T1 '
            '--seeds 0:50 --out DIR', examples=('crane bench fast --domain boat --tier T1 --seeds 0:50',)),
    Command(('bench', 'cluster'), 'Throughput and cost on the cluster.', today='python3 -m bench.fast with its '
            '--fresh-jobs, on a cluster host', examples=('crane bench cluster --domain boat',)),
    Command(('cluster',), 'Research-cluster jobs: submit, wait, status, fetch (the research CLI).', 'crane.wrap:cluster',
            passthrough=True, about="the research CLI's output; arguments go to it unchanged. Found at "
            '$RESEARCH_CLUSTER/bin/research or ~/research-cluster/bin/research.',
            examples=('crane cluster status',), raw='research'),
    Command(('build', 'player'), 'Build the headless Linux player through heavy unity and stamp its manifest with the '
            'source commit.', 'crane.build:main', flags=(DRY,), export=True,
            about='the Unity command, then the stamped Builds/CRANE-Worker/crane-build-manifest.json and, with '
                  '--tarball, Builds/crane-player-<commit12>-linux-x86_64.tar.zst and its .sha256.',
            notes="Needs the project's Unity editor. --skip-unity only stamps and packs an existing Builds/CRANE-Worker.",
            params=(P('--tarball', 'bool', description='Also write the player tarball and its .sha256.'),
                    P('--skip-unity', 'bool', description='Stamp and pack an existing build only.'),
                    P('--editor', 'path', description='The Unity editor (default UNITY_EDITOR, else the '
                      'ProjectVersion.txt editor).'),
                    P('--builds', 'path', description='The Builds dir (default <repo>/Builds).'),
                    P('--dry-run', 'bool', description='Print the Unity command and stop.')),
            examples=('crane build player --tarball',)),
    Command(('build', 'images'), 'The crane docker images (core, gpu, sim), pinned by commit.',
            today='docker build on the Dockerfiles', examples=('crane build images core',)),
    Command(('build', 'wheel'), 'The crane wheel and sdist.', today='python3 -m build', examples=('crane build wheel',)),
    Command(('deploy',), 'Install crane (a venv) and the player on a host, then run crane doctor there.',
            today='rsync the checkout, then ./install.sh --yes on the host', examples=('crane deploy HOST',),
            params=(P('host', description='The host.'),)),
    Command(('update',), 'Update this checkout from a release.', today='git pull, then ./install.sh --yes',
            examples=('crane update',)),
    Command(('heavy',), 'The machine-wide concurrency guard for Unity, sims and test suites.', 'crane.wrap:heavy',
            passthrough=True, about="heavy's own output: it waits for a slot (bounded, exit 75 on timeout), runs CMD "
            'and releases it. `crane heavy status` lists the slots, the queue and ETAs.',
            examples=('crane heavy contain test python3 -m pytest fastsim/tests -q',), raw='bin/heavy'),
    Command(('doctor',), 'Check this machine (python, extras, the T2 player and its flags, heavy, astro_dock, GPU) and '
            'print the next commands.', 'crane.doctor:main', flags=(JSON, QUIET), json='crane.doctor/1', export=True,
            about='a table of checks (ok / off / missing / warn / FAIL with details), then next:. --json: {schema, ok, '
                  'checks[{name, state, detail}], next[]}.',
            notes='Read-only; safe any time. Exit 1 only when the core (python >= 3.10, numpy, pyyaml) is broken.',
            params=(P('--json', 'bool', description='One JSON document.'),),
            examples=('crane doctor',)),
    Command(('setup',), 'The setup wizard.', today='./install.sh [--yes] [--components core,fast,gpu,view,field,setup]',
            examples=('crane setup',)),

    # ---- help -------------------------------------------------------------------------------------------------
    Command(('help',), 'The nouns and verbs; help NOUN for one noun; --check fails when help.yaml or Docs/CLI.md '
            'drift from the registry, --write regenerates them.', 'crane.cli:help_main', flags=(JSON,),
            about='one line per command; with NOUN, its verbs with examples.',
            params=(P('topic', 'list', nargs='*', metavar='NOUN', description='A noun (and verb).'),
                    P('--check', 'bool', description='Regenerate into a temp dir; exit 1 when the committed files '
                      'differ.'),
                    P('--write', 'bool', description='Regenerate help.yaml and Docs/CLI.md in place.'),
                    P('--json', 'bool', description='The registry as one JSON document.')),
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
    """The command's handler function (imported now)."""
    import importlib
    module, _, fn = command.handler.partition(':')
    return getattr(importlib.import_module(module), fn)


def unavailable(command):
    """The one-line refusal of a command that is not built yet (exit 2)."""
    return f'{command.name}: not yet available; today use {command.today}'


FLAG_PARAMS = {
    'json': Param('--json', 'bool', description='One JSON document on stdout (JSONL for streams).'),
    'quiet': Param('--quiet', 'bool', description='Only results and failures.'),
    'dry_run': Param('--dry-run', 'bool', description='Print the resolved plan; run nothing.'),
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


def _yaml_param(param):
    kind = {'tier': 'str', 'env': 'str'}.get(param.type, param.type)
    d = {'name': param.name if not param.positional else (param.dest or param.name)}
    if kind != 'str':
        d['type'] = kind
    if param.default not in (None, False, [], ()):
        d['default'] = param.default
    if param.choices:
        d['choices'] = list(param.choices)
    d['description'] = param.description + (' (repeatable)' if param.repeat and 'repeatable' not in
                                            param.description else '')
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
    """help.yaml's entries: the exported commands, and the exported nouns as NOUN VERB [ARGS]."""
    out = []
    for n in NOUNS:
        if n.export:
            ready = [c for c in n.verbs if not c.today]
            out.append(dict(name=f'crane {n.name}', words=(n.name,),
                            summary=f'{n.summary[0].upper()}{n.summary[1:]}: ' + '; '.join(
                                f'{c.verb} ({c.summary.split(". ")[0].rstrip(".").lower()})' for c in ready) + '.',
                            params=[Param('verb', choices=tuple(c.verb for c in ready), description=n.export),
                                    Param('args', 'list', nargs='*', description=f'The verb\'s options: crane '
                                          f'{n.name} VERB --help.')],
                            examples=[c.examples[0] for c in ready]))
        out += [dict(name=c.name, words=c.words, summary=c.summary, ros=c.raw, notes=c.notes, params=params_of(c),
                     examples=list(c.examples)) for c in COMMANDS if c.export and c.noun == n.name]
    return out


def help_yaml():
    lines = ["# crane's commands for astro's help browser and verbs (`astro help crane`). Generated from",
             '# crane/commands.py by `crane help --write`; `crane help --check` fails when it drifts. Schema:',
             "# astro_dock/.devcontainer/astro-help.py. Each run line is crane's launcher, which needs no install.",
             'package: crane',
             'summary: ' + _scalar('Sim and research across domains and tiers (T0 abstract, T1 dynamic, T2 unity); '
                                   'crane help for every command.'),
             'commands:']
    for e in _entries():
        lines += [f'  - name: {_scalar(e["name"])}',
                  f'    run: {_scalar(CRANE_RUN + " " + " ".join(e["words"]))}',
                  f'    summary: {_scalar(e["summary"])}']
        for key in ('ros', 'notes'):
            if e.get(key):
                lines.append(f'    {key}: {_scalar(e[key])}')
        if e['params']:
            lines.append('    params:')
            lines += [f'      - {_yaml_param(p)}' for p in e['params']]
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
