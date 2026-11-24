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
    owner: str = ''                # a tool that owns the noun's words: other verbs and options-only go to it
    verbs: list = field(default_factory=list, compare=False)


def sel(key, **kw):
    """A shared selector (crane/select.py) as a param, with this command's changes."""
    spec = {k: (tuple(v) if isinstance(v, list) else v) for k, v in select.SPECS[key].items()}
    return replace(Param(**spec), **kw)


P = Param
JSON, QUIET, DRY = 'json', 'quiet', 'dry_run'

NOUNS = [
    Noun('sim', "T2: a domain's Unity world plus its ROS side, used like the robot", owner='crane.wrap:sim'),
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
    Noun('cluster', 'research-cluster jobs: submit, wait, status, fetch (other verbs go to the research CLI)',
         owner='crane.wrap:cluster'),
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
    Command(('sim', 'up'), "Start a domain's T2 world (the Unity player through heavy unity, plus its ROS side) and "
            'return once the course is ready; prints the status block.',
            'crane.wrap:sim', passthrough=True, flags=(JSON, DRY), json='crane.sim.status/1', export=True,
            about='one line as the ROS side starts and one as the player is queued on `heavy unity`, then the status '
                  'block (mode, ROS_DOMAIN_ID, endpoint, player pgid, ROS container, course, current, logs, next:). '
                  '--dry-run: the resolved player argv and docker run, nothing starts (crane.sim.plan/1 with --json).',
            notes='Waits up to 60 s for the ROS endpoint and 300 s (SIM_UP_READY_S) for the course. Without '
                  '--display, a window only on a terminal with a display, else headless under xvfb-run. Refuses '
                  '(exit 2) while a sim runs. Exit 0 ready, 1 the world failed, 2 usage or config. Everything '
                  'specific comes from config/domains/DOMAIN.yaml tiers.T2. Env: ROS_DOMAIN_ID, ASTRO_STATE / '
                  'MHSEALS_HARDWARE_CONFIG (passed into the ROS side), the image env, SIM_UP_*, CRANE_PLAYER, HEAVY. '
                  'Same as Tools/RoboBoat/sim_up.',
            params=(P('--seed', 'int', description='The world seed (default 1).'),
                    sel('env', description='KIND:NAME, the world (default course:roboboat_2026); --course FAMILY is '
                        'the same as --env course:FAMILY.'),
                    sel('card', type='path', description='A plant profile JSON or a card name (--profile-json is the '
                        'same).'),
                    P('--profile', description="The player's runtime profile (default interactive-low)."),
                    P('--current', metavar='SPEED,TOWARD_DEG', description='SPEED,TOWARD_DEG (m/s, deg), or 0 for '
                      'still water (default the scene current).'),
                    P('--display', choices=('window', 'headless'), description='Window or xvfb (--window, --headless '
                      'are the same).'),
                    P('--player', 'path', description='The player (else CRANE_PLAYER, then the build).'),
                    sel('robot'),
                    P('--set', metavar='K=V', repeat=True, description='Pin one world parameter (repeatable).'),
                    sel('domain', description='config/domains/DOMAIN.yaml (default boat).'),
                    P('--dry-run', 'bool', description='Print the resolved player argv and docker run; start nothing.'),
                    P('--json', 'bool', description='Progress on stderr, the status as one JSON document.')),
            examples=('crane sim up --headless --seed 3 --current 0', 'crane sim up --card characterized --headless',
                      'crane sim up --dry-run'),
            raw='Tools/RoboBoat/sim_up up'),
    Command(('sim', 'down'), "Stop what crane sim up started: its player's process group (TERM, KILL after 10 s) and "
            'its ROS container, nothing else.', 'crane.wrap:sim', passthrough=True, flags=(JSON,),
            json='crane.sim.down/1', export=True, about='the down line.',
            params=(P('--json', 'bool', description='One JSON document instead of the down line.'),),
            examples=('crane sim down',), raw='Tools/RoboBoat/sim_up down'),
    Command(('sim', 'restart'), 'Restart one part of the running world and keep the rest: nav (only the Nav2 nodes, '
            'so new nav2_limits in the state take effect), ros (the ROS container; the player stays) or player.',
            'crane.wrap:sim', passthrough=True, flags=(JSON,), json='crane.sim.restart/1', export=True,
            about='one line per step, then `crane sim: PART restarted, ready: ...` and next:. --json: {schema, restarted, '
                  'ready, at, status}.',
            notes='nav stops the nav nodes named by the domain recipe (tiers.T2.ros.restart.nav) inside the ROS '
                  'container, starts them again from the same launch (only:=navigation), and waits up to the '
                  "recipe's ready_s (90 s) for bt_navigator to be active; a domain without the recipe refuses and "
                  'names `restart ros`. ros re-runs the saved docker run, waits up to 60 s for the endpoint and '
                  '120 s for the ready nodes. player re-runs the saved player argv, waits up to 300 s '
                  '(SIM_UP_READY_S) for the course. Nav goals in flight are lost. Exit 0 ok, 1 failed, 2 usage, '
                  'config or nothing running, 75 timed out waiting.',
            params=(P('what', choices=('nav', 'ros', 'player'), description='The part to restart.'),
                    P('--json', 'bool', description='One JSON document.'),
                    sel('domain', description='config/domains/DOMAIN.yaml (default boat).')),
            examples=('crane sim restart nav', 'crane sim restart player --json'),
            raw='docker exec CONTAINER pkill ...; docker exec -d CONTAINER ros2 launch ...'),
    Command(('sim', 'status'), 'The status block (mode, ROS_DOMAIN_ID, endpoint, player, ROS container, course, '
            'current, logs, next commands). Read-only.',
            'crane.wrap:sim', passthrough=True, flags=(JSON,), json='crane.sim.status/1', export=True,
            about='the status block, ending with next:. --json: {schema, domain, mode, player, ros, course, current, '
                  'logs, next}.',
            params=(P('--json', 'bool', description='One JSON document.'),),
            examples=('crane sim status',), raw='Tools/RoboBoat/sim_up status'),
    Command(('sim', 'check'), 'Is the running sim ready (player, course, plant, ROS side, nav stack)? '
            '--wait SECONDS polls until ready; CHECK INPUTS runs an offline Tools/SimCheck check.',
            'crane.wrap:sim', passthrough=True, flags=(JSON,), json='crane.sim.check/1', export=True,
            about='one row per check (ok/FAIL, name, detail), then `sim check: ready` or the failed names. Exit 1 '
                  'when not ready, 75 when --wait SECONDS pass first. --json: {schema, ready, exit, failed, notices, '
                  'checks[{name, ok, detail}]}.',
            params=(P('--wait', 'int', description='Poll every 2 s until ready or this many seconds pass.'),
                    P('--json', 'bool', description='One JSON document.'),
                    sel('domain', description='config/domains/DOMAIN.yaml (default boat).')),
            examples=('crane sim check --wait 120',), raw='Tools/RoboBoat/sim_up check'),
    Command(('sim', 'logs'), 'The last lines of the player, player stdout and ROS logs, each under its path.',
            'crane.wrap:sim', passthrough=True, export=True,
            params=(P('which', 'list', nargs='*', choices=('player', 'stdout', 'ros'),
                      description='Which logs (default all three).'),
                    P('-n', 'int', default=40, description='Lines per log.')),
            examples=('crane sim logs ros -n 100',), raw='Tools/RoboBoat/sim_up logs'),

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
                    P('--engine', choices=('fastsim', 'batched'), default='fastsim', description='fastsim: one '
                      'scalar episode per seed; batched: every seed at once on the batched engine (Docs/Engine.md).'),
                    P('--backend', choices=('numpy', 'numba', 'jax'), default='numba', description='With --engine '
                      'batched: numpy (float64 reference), numba (every core) or jax (GPU).'),
                    P('--dtype', choices=('float64', 'float32'), description='With --engine batched: default '
                      'float64; jax on a consumer GPU wants float32 (bench/RESULTS.md).'),
                    P('--record', 'int', default=0, metavar='K', description='With --engine batched: also write '
                      'traj.ctraj with every K-th track row (default: run.json, events.jsonl, metrics.json only).'),
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

    # ---- study ----------------------------------------------------------------------------------------------
    Command(('study', 'init'), 'Write a commented crane.study/1 spec to start from (stdout, or --out FILE).',
            'crane.study.cli:init_main', flags=(JSON,), json='crane.study.init/1',
            about='the spec on stdout, or `wrote FILE` and next:.',
            params=(sel('domain', description='The domain its sim arms run (default toy).'),
                    P('--out', 'path', metavar='FILE', description='Write the spec here instead of stdout.'),
                    P('--force', 'bool', description='Overwrite FILE.')),
            examples=('crane study init --out study.yaml', 'crane study init --domain boat > boat_study.yaml')),
    Command(('study', 'plan'), 'Count what a study spec compiles to: cells, units, jobs and estimated time per stage, '
            'with what is already cached. Nothing runs.', 'crane.study.cli:plan_main', flags=(JSON,),
            json='crane.study.plan/1',
            about='one row per stage (tiers, cells, units, done, held-out units, jobs, est, per job), the seed blocks '
                  'and splits, pending adaptive stages, then next:. --json adds jobs[] (job_id, cell, seeds, '
                  'est_seconds, kind, campaign, purpose, expect, pin, argv): the cluster compiler\'s input.',
            notes='A spec error is one line naming the key and the fix (exit 2). Estimates use pack.unit_s; jobs pack '
                  'a cell\'s seeds to about pack.target_s (default 900 s) each.',
            params=(P('spec', 'path', description='The study spec (YAML).'),
                    P('--stage', description='Only this stage.'),
                    P('--block', description='Only this seed block or split.'),
                    P('--out', 'path', description='The study output to count done units in (default '
                      '~/.cache/crane/studies/NAME).'),
                    P('--json', 'bool', description='One JSON document with jobs[].')),
            examples=('crane study plan examples/studies/toy_paired.yaml',
                      'crane study plan examples/studies/boat_cards.yaml --json')),
    Command(('study', 'run'), 'Run a study locally: every stage in order (or --stage), resuming from the unit '
            'cache; held-out blocks wait for --block B --confirm and a committed targets file.',
            'crane.study.cli:run_main', flags=(JSON, QUIET, DRY), json='crane.study.run/1',
            about='a progress line per minute, then one row per stage (cells, units, ran, reused, failed), the '
                  'held-out blocks skipped, where the rows and manifest are, and next:.',
            notes='OUT/progress.json is rewritten at least every 60 s (CRANE_RUN_DIR overrides where). Units are '
                  'cached in OUT/cache/records.jsonl on (study, stage, arm, cell hash, seed, card hash, code commit) '
                  'within 1 s of finishing; rerunning the same command runs only the missing ones. A held-out block refuses '
                  '(exit 2) without --confirm, or when the spec\'s targets file is not committed, has local changes '
                  'or is not older than now. Exit 0 ok, 1 units failed, 2 spec, usage or refusal.',
            params=(P('spec', 'path', description='The study spec (YAML).'),
                    P('--stage', description='Only this stage (its select stage must be done).'),
                    P('--block', description='Only this seed block or split (a held-out one needs --confirm).'),
                    P('--where', default='local', choices=('local', 'cluster'),
                      description='local runs here; cluster: use crane study compile, then research submit.'),
                    P('--confirm', 'bool', description='Run held-out blocks (needs the committed targets file).'),
                    P('--max-workers', 'int', default=1, description='Worker processes.'),
                    P('--out', 'path', description='The study output (default ~/.cache/crane/studies/NAME).'),
                    P('--json', 'bool', description='One summary document; progress lines go to stderr.'),
                    P('--dry-run', 'bool', description='Print the plan (as study plan); run nothing.')),
            examples=('crane study run examples/studies/toy_paired.yaml --max-workers 4',
                      'crane study run examples/studies/toy_paired.yaml --block test --confirm',
                      'crane study run examples/studies/adaptive.yaml --stage V')),
    Command(('study', 'status'), "A study's progress per stage and block, failures with a sample traceback, and "
            'interim per-arm means on dev blocks only (held-out blocks: progress only).',
            'crane.study.cli:status_main', flags=(JSON,), json='crane.study.status/1',
            about='one row per stage and block (role, units, done, %), the progress.json line, the latest failure '
                  'and its traceback tail, interim dev means per arm, then next:. Exit 1 when units failed.',
            params=(P('target', 'path', metavar='SPEC|OUT', description='A spec, or a study output dir.'),
                    P('--out', 'path', description='With a spec: its output (default ~/.cache/crane/studies/NAME).'),
                    P('--json', 'bool', description='One JSON document.')),
            examples=('crane study status examples/studies/toy_paired.yaml',
                      'crane study status ~/.cache/crane/studies/toy_paired')),
    Command(('study', 'report'), 'Paired analysis of a study output: each arm minus the baseline per cell, paired '
            'on the seed (bootstrap CI, sign-flip permutation, Holm across cells), via judge.stats.',
            'crane.study.cli:report_main', flags=(JSON,), json='crane.study.report/1',
            about='the pooled table (stage, block, arm-baseline, metric, seeds, cells, diff, 95% CI, p, cells with '
                  'Holm p < 0.05), held-out blocks still running, and where report.md / report.json went.',
            notes='Reads OUT/manifest.json and its rows (or every */manifest.json under OUT, as a fetched cluster '
                  'output). A held-out block enters only when all its units are done.',
            params=(P('out', 'path', metavar='OUT', description='A study output dir.'),
                    P('--baseline', description="The arm the others are compared with (default the spec's)."),
                    P('--metric', repeat=True, description='Only these metrics (repeatable).'),
                    P('--bootstrap', 'int', default=2000, description='Bootstrap resamples.'),
                    P('--json', 'bool', description='One JSON document (also written to OUT/report.json).')),
            examples=('crane study report ~/.cache/crane/studies/toy_paired',
                      'crane study report /tmp/study --metric succeeded --json')),
    Command(('study', 'promote'), "Freeze an adaptive stage's selection (top-k of the stage it reads) into a spec "
            'whose stage names the chosen arms, to run here or send to the cluster.',
            'crane.study.cli:promote_main', flags=(JSON,), json='crane.study.promote/1',
            about='the ranking (rank, group, mean, n, chosen) and the written spec, then next:.',
            params=(P('spec', 'path', description='The study spec (YAML).'),
                    P('--stage', required=True, description='The stage with select:.'),
                    P('--out', 'path', description='The study output holding the source stage (default '
                      '~/.cache/crane/studies/NAME).'),
                    P('--write', 'path', metavar='FILE', description='Where the promoted spec goes (default '
                      'SPEC.STAGE.yaml beside the spec).'),
                    P('--json', 'bool', description='One JSON document.')),
            examples=('crane study promote examples/studies/adaptive.yaml --stage V',)),
    Command(('study', 'run-cell'), 'Run one cell or one packed job of a study into DIR: what a cluster job runs '
            '(rows, manifest.json with sha256, metrics.json, progress.json).', 'crane.study.cli:run_cell_main',
            flags=(JSON,), json='crane.study.run_cell/1',
            about='one line: ran, reused, failed, rows and DIR. Exit 1 when units failed, 2 a refusal.',
            notes='JOB_ID is a jobs[].job_id of crane study plan --json (CELL_ID, or CELL_ID.jK for one seed chunk). '
                  'Held-out cells need --confirm and the committed targets file, as study run, or on a box with no '
                  'git checkout --targets-sha256 from the compiled job. metrics.json holds at '
                  'most 1000 finite numbers; for held-out cells only unit counts.',
            params=(P('spec', 'path', description='The study spec (YAML).'),
                    P('--cell', required=True, metavar='JOB_ID', description='The cell or job id.'),
                    P('--out', 'path', required=True, metavar='DIR', description='The job output dir.'),
                    P('--seeds', metavar='A,B,...', description='Only these of its seeds.'),
                    P('--confirm', 'bool', description='Run a held-out cell.'),
                    P('--targets-sha256', metavar='SHA', description='The targets file hash the submit side checked '
                      'in git (crane study compile pins it); stands in for git on a box.'),
                    P('--json', 'bool', description='One JSON document.')),
            examples=('crane study run-cell study.yaml --cell JOB_ID --out /workspace/out/STUDY/JOB_ID',)),
    Command(('study', 'compile'), 'Compile a study\'s jobs to rc-job/1 bodies for the research cluster: one '
            'crane_batch or gpu_command job per cell seed chunk (5-30 min), each running crane study run-cell '
            'through crane-batch. The compiler crane cluster submit uses.',
            'crane.study.cli:compile_main', flags=(JSON,), json='crane.study.compile/1',
            about='{jobs: [rc-job/1 ...]} on stdout, or `N jobs in FILE` and next: research submit.',
            notes='Needs purpose (20-600 chars, no local path) and pack.unit_s. Jobs pin crane_ml (and the spec\'s '
                  'repo) at the exact commit, carry dedup ids from the unit keys and the commit, needs from pin:, '
                  'campaign and expect; units in the study\'s unit cache are left out; field data is refused. '
                  'Held-out jobs are left out without --confirm; with it the targets file must be committed, clean '
                  'and older than now (checked here in git), and each held-out job pins its sha256, which '
                  'crane-batch passes to run-cell as --targets-sha256 for the box to check. Exit 2 on a refusal.',
            params=(P('spec', 'path', description='The study spec (YAML).'),
                    P('--stage', description='Only this stage.'),
                    P('--block', description='Only this seed block or split.'),
                    P('--confirm', 'bool', description='Include held-out jobs (needs the committed targets file).'),
                    P('--allow-dirty', 'bool', dest='allow_dirty', description='Compile with uncommitted changes '
                      '(jobs run the commit).'),
                    P('--write', 'path', metavar='FILE', description='Write {jobs: [...]} here instead of stdout.'),
                    P('--json', 'bool', description='One JSON document with jobs[].')),
            examples=('crane study compile examples/studies/toy_paired.yaml --write /tmp/jobs.json',
                      'crane study compile examples/studies/toy_paired.yaml --block test --confirm --json')),

    # ---- runs, repro, data, monitor ---------------------------------------------------------------------------
    Command(('runs', 'ls'), 'List indexed runs, newest first: crane.run/1 dirs, fastsim run dirs and study '
            'outputs (manifest.json).', 'crane.runs.query:ls_main', flags=(JSON,), json='crane.runs.list/1',
            about='an aligned table (run, domain, tier, seed, outcome, arm, created, any --metric columns, path), '
                  'then shown/matched/indexed counts and next:. A new, empty index first indexes '
                  '~/.cache/crane/runs.',
            notes='The index is ~/.cache/crane/index.sqlite (CRANE_INDEX moves it); crane runs index ROOT adds '
                  "roots. --cluster adds the coordinator's runs of --study (GET /v1/runs/query, read-only; "
                  'CRANE_COORDINATOR, and CRANE_VIEWER_TOKEN or CRANE_TOKEN); offline it says so on stderr and '
                  'lists the local runs. A held-out run that is not done is sealed: outcome and metrics read '
                  '`sealed`, none are stored, and stderr says how many.',
            params=(sel('domain', description='Only this domain.'), sel('tier', description='Only this tier.'),
                    P('--study', description='Only this study.'),
                    P('--where', metavar='EXPR', description='A condition, as crane runs query --where.'),
                    P('--metric', repeat=True, metavar='NAME', description='Show this metric as a column '
                      '(repeatable).'),
                    P('--limit', 'int', default=50, description='At most this many rows (0: all).'),
                    P('--cluster', 'bool', description="Also the coordinator's runs of --study."),
                    P('--json', 'bool', description='One JSON document: {schema, index, n, matched, indexed, runs}.')),
            examples=('crane runs ls --domain boat --tier T1', 'crane runs ls --study nlc08 --metric time --limit 20'),
            raw='sqlite3 ~/.cache/crane/index.sqlite "SELECT * FROM runs"'),
    Command(('runs', 'query'), 'Runs matching --where: FIELD OP VALUE joined by and / or / not and parentheses; '
            'FIELD is a column or metric.NAME.', 'crane.runs.query:query_main', flags=(JSON,),
            json='crane.runs.list/1',
            about='the crane runs ls table for the matching runs. A bad expression is one line naming the column '
                  'and an example (exit 2).',
            notes='OP is = != < <= > >= or ~ (glob; a value without * or ? matches anywhere) or in (A, B). Columns: '
                  'run_id, path, source, domain, tier, kind, seed, card, backend, dtype, commit, dirty, outcome, '
                  'created, study, stage, arm, block, run_sha256, sealed. A sealed (held-out, not done) run has '
                  'no outcome or metrics, so metric terms never match it. `null` tests for a missing value; commit= and '
                  'run_id= take a hex prefix. Parsed to SQL with bound parameters, never evaluated.',
            params=(P('--where', metavar='EXPR', required=True, description='The condition.'),
                    P('--metric', repeat=True, metavar='NAME', description='Show this metric as a column '
                      '(repeatable).'),
                    P('--limit', 'int', default=50, description='At most this many rows (0: all).'),
                    P('--study', description='With --cluster: the study the coordinator selects.'),
                    P('--cluster', 'bool', description="Also the coordinator's runs of --study."),
                    P('--json', 'bool', description='One JSON document, as crane runs ls.')),
            examples=('crane runs query --where "outcome=TIMEOUT and tier=T1 and metric.time>120"',
                      'crane runs query --where "arm in (a, b) and not dirty=true" --metric min_clearance_m')),
    Command(('runs', 'show'), 'One run: its meta, metrics and files.', 'crane.runs.query:show_main', flags=(JSON,),
            json='crane.runs.show/1',
            about='meta (run, path, source, domain, tier, seed, card, commit, outcome, study fields, identity '
                  'hashes), the metrics, the files with sizes, then next: crane view RUN.',
            params=(P('run', metavar='RUN', description='A run dir, or a run_id (a prefix will do) from the index.'),
                    P('--json', 'bool', description='One JSON document: {schema, run, meta, files, next}.')),
            examples=('crane runs show 3fa2c1d0', 'crane runs show ~/.cache/crane/runs/toy/T0/base-1a2b3c4d/base/s3')),
    Command(('runs', 'diff'), "Two runs' meta and metric deltas, and why their identities differ (config, "
            'scenario, adapter, seed, core).', 'crane.runs.query:diff_main', flags=(JSON,), json='crane.runs.diff/1',
            about='`identity: the same run` or which identity parts differ with the changed keys (config and '
                  'adapter keys old -> new, the scenario names, the seeds, the core commits), the meta fields that '
                  'differ, then each differing metric with A, B and B-A.',
            params=(P('a', metavar='A', description='A run dir or run_id.'),
                    P('b', metavar='B', description='A run dir or run_id.'),
                    P('--json', 'bool', description='One JSON document: {schema, same_run, identity, meta, metrics}.')),
            examples=('crane runs diff 3fa2c1d0 77b0e9aa',)),
    Command(('runs', 'index'), 'Index the runs under ROOTs (default: the roots indexed before, else '
            '~/.cache/crane/runs); incremental by mtime, then sha256.', 'crane.runs.query:index_main', flags=(JSON,),
            json='crane.runs.index/1',
            about='each root, then `N records under R root(s): added, changed, unchanged, removed; total indexed`.',
            notes='Reads crane.run/1 dirs, fastsim run dirs (their crane.run/1 meta, nothing written) and study '
                  'outputs (manifest.json crane.data.manifest/1 with metrics.json). Records gone from a root are '
                  'dropped, and so is a remembered root that is gone. Hidden and *.stale-N dirs are skipped. '
                  'Held-out records stay sealed until done (the record says complete, or every progress.json up '
                  'to the root is phase done) and are reread each time.',
            params=(P('roots', 'list', nargs='*', metavar='ROOT', description='Directories to index.'),
                    P('--rebuild', 'bool', description='Drop every row first and read everything again.'),
                    P('--json', 'bool', description='One JSON document with the counts.')),
            examples=('crane runs index ~/.cache/crane/runs /tmp/study-out', 'crane runs index --rebuild')),
    Command(('repro',), 'Rerun one run at its recorded commit (a temporary git worktree) and compare run_sha256, '
            'the files and the metrics.', 'crane.runs.repro:main', flags=(JSON,), json='crane.repro/1',
            about='run_sha256, tree_sha (or the first differing file) and metrics, each same or differs, then '
                  '`repro: reproduced` or the rerun kept for crane runs diff. With --check, exit 1 when it does not '
                  'reproduce.',
            notes="Reruns fastsim runs (T0/T1): that commit's fastsim on the scenario file in the run dir, with the "
                  'recorded config and provenance. Refuses with one line (exit 2) a run made from a dirty tree (its '
                  'diff sha when recorded), a commit no repository here has, or a non-fastsim record. The worktree '
                  'is always removed; your checkout is never touched.',
            params=(P('run', metavar='RUN', description='A run dir, or a run_id from the index.'),
                    P('--check', 'bool', description='Exit 1 when the rerun differs.'),
                    P('--timeout', 'int', default=1800, description='Seconds the rerun may take.'),
                    P('--json', 'bool', description='One JSON document (crane.repro/1).')),
            examples=('crane repro 3fa2c1d0 --check', 'crane repro ~/.cache/crane/runs/toy/T0/base-1a2b3c4d/base/s3'),
            raw='git worktree add --detach TMP COMMIT, then fastsim.run.simulate on the recorded inputs'),
    Command(('data', 'export'), 'Run dirs, batch outputs and datasets to one columnar dataset (npz, parquet, csv or jsonl) '
            'plus manifest.json with a sha256 per file; --select, --where and --split pick the rows.',
            'crane.data.export:main', flags=(JSON,), json='crane.data.export/1',
            about='one line (rows, columns, files, MB, format), one row per file (path, rows, sha256), the manifest '
                  'path, a note for confirmed held-out rows or non-finite floats, next:. --json: {schema, out, manifest, rows, '
                  'columns, files, held_count, nonfinite}.',
            notes='A run dir gives one metrics-first row (seed, outcome, condition.*, metrics, commit, run_sha256) and '
                  'its recorded arrays as obs / traj columns (record: in the domain YAML, Docs/Data.md). Array columns '
                  'are stacked to rows x T x channels (shorter rows NaN-padded, COL@len their lengths); with csv or '
                  'parquet they go to PART.arrays.npz beside each file. Files roll over at 90 MB. Held-out rows '
                  '(role, block_role or tag held-out, held_out true) are refused without --confirm. Exit 0 written, '
                  '2 usage (no rows, an unknown column, a non-empty --out, held-out rows).',
            params=(P('sources', 'list', nargs='+', metavar='SRC', description='Run dirs, batch outputs, datasets '
                      '(dirs with manifest.json) or row files (.parquet .csv .npz .jsonl).'),
                    P('--format', choices=('npz', 'parquet', 'csv', 'jsonl'), default='npz',
                      description='The file format (parquet needs pyarrow: the [data] extra).'),
                    P('--select', metavar='COLS', repeat=True, description='Columns to keep, comma-separated '
                      '(default all).'),
                    P('--where', metavar='EXPR', repeat=True, description="Keep rows where COLUMN OP VALUE holds; OP "
                      "is = != > >= < <=; = and != take a,b (repeatable, all must hold)."),
                    P('--split', metavar='FILE:NAME', description='Keep the rows of one split of a crane data splits '
                      'file.'),
                    P('--confirm', 'bool', description='Export held-out rows too (refused without it).'),
                    sel('out', required=True, description='The dataset dir to write (new or empty).'),
                    P('--json', 'bool', description='One JSON document.')),
            examples=('crane data export ~/.cache/crane/runs/toy/T0 --format npz --out /tmp/ds',
                      "crane data export /tmp/runs --select seed,outcome,obs --where 'seed>=10' --out /tmp/ds2",
                      'crane data export /tmp/ds --split /tmp/ds/splits.json:train --out /tmp/train'),
            raw='python3 -c "from crane.data.export import export; export([SRC], OUT)"'),
    Command(('data', 'schema'), "The columns of a dataset or of run dirs: name, type and, for arrays, rate and "
            'channels; plus the row count.', 'crane.data.export:schema_main', flags=(JSON,),
            json='crane.data.schema/1',
            about='one line (rows, columns, files), one row per column (name, type, Hz and channels for arrays), '
                  'next:. --json: {schema, sources, columns, rows, files, arrays}.',
            params=(P('sources', 'list', nargs='+', metavar='SRC', description='A dataset dir, row files or run dirs.'),
                    P('--json', 'bool', description='One JSON document.')),
            examples=('crane data schema /tmp/ds', 'crane data schema ~/.cache/crane/runs/toy/T0 --json'),
            raw='python3 -c "import json; print(json.load(open(\'DS/manifest.json\'))[\'columns\'])"'),
    Command(('data', 'splits'), "A dataset's units (seeds, conditions or timelines) to named splits by a salted "
            'hash; held-out units go to held-out and never to train or calib.', 'crane.data.splits:main',
            flags=(JSON, DRY), json='crane.data.splits/1',
            about='one line (units, by, rows, out), one row per split (units, rows, fraction), the held-out rule, '
                  'next:. --json: the split file itself plus out and written.',
            notes='A row is held-out when its role / block_role / tag column says held-out, held_out is true, its '
                  "block or seed is a held-out block of --domain's seeds, or it matches --held-out. A unit with any "
                  'held-out row is held-out. A unit keeps its split as data grows (the hash, not the count, decides). '
                  'Writing over an earlier split file refuses to move a unit into train or calib from elsewhere. '
                  'Exit 0 written, 2 usage.',
            params=(P('sources', 'list', nargs='+', metavar='SRC', description='A dataset dir, row files or run dirs.'),
                    P('--by', default='seed', description='The unit: seed, condition (the condition.* columns), '
                      'timeline, or COLUMN[,COLUMN].'),
                    P('--ratios', metavar='NAME=F,...', default='train=0.7,calib=0.15,test=0.15',
                      description='Split names and fractions (adding up to 1).'),
                    P('--held-out', metavar='EXPR', repeat=True, description='Rows matching COLUMN OP VALUE are '
                      'held-out too (repeatable; e.g. source=real).'),
                    sel('domain', description="Its seeds: blocks with role held-out mark their seeds held-out."),
                    P('--salt', default='crane', description='The hash salt (another salt, another assignment).'),
                    P('--out', 'path', description='The split file (default SRC/splits.json for one dataset dir).'),
                    P('--dry-run', 'bool', description='Print the splits; write nothing.'),
                    P('--json', 'bool', description='One JSON document.')),
            examples=('crane data splits /tmp/ds --by seed',
                      'crane data splits /tmp/ds --by timeline --held-out source=real --ratios train=0.8,calib=0.2',
                      'crane data splits /tmp/ds --by condition --domain boat --out /tmp/splits.json')),
    Command(('monitor',), 'Progress of local and cluster work in one view: every progress.json under a study (or a '
            'dir) merged with its cluster jobs, per stage, arm and block.', 'crane.monitor:main', flags=(JSON,),
            json='crane.monitor/1',
            about='one row per (stage, arm, block): done/total, rate, ETA, failed, jobs running/queued/done/failed, '
                  'hosts and interim metric means (dev blocks only; held-out and other blocks show progress only), '
                  'then failures with their reason (crashed, timed out without progress, cancelled), host and a '
                  'traceback line, and next:. --json: {schema, at, where, study, sources, notes, finished, '
                  'groups[{stage, arm, block, role, done, total, failed, rate_per_s, eta_s, hosts, jobs, failures, '
                  'metrics}]}; with --follow one document per refresh (JSONL).',
            notes='Local: progress.json with cell.json, failure.json and metrics.json beside it. Cluster: when '
                  'OUT/STUDY/cluster/jobs.json exists, the project job list (research token) and the public feed '
                  '(/v1/feed); unreachable means a note, not a failure. A progress.json older than 180 s counts as '
                  'stalled. --follow refreshes every --interval s (5..300) and stops when every job has ended or '
                  'after --timeout s (exit 75).',
            params=(P('target', nargs='?', metavar='STUDY|DIR', description='A study name under OUT, or a dir '
                      '(default every study under OUT).'),
                    sel('out', description='The studies root (default CRANE_STUDIES, else ~/.cache/crane/studies).'),
                    P('--follow', 'bool', description='Refresh until every job has ended or --timeout passes.'),
                    P('--interval', 'float', default=30.0, description='Seconds between refreshes (5..300).'),
                    P('--timeout', 'float', default=3600.0, description='With --follow: stop after this many seconds.'),
                    P('--local', 'bool', description='Local progress files only; do not ask the cluster.'),
                    P('--json', 'bool', description='One JSON document (one per refresh with --follow).')),
            examples=('crane monitor boat01', 'crane monitor boat01 --follow --interval 60',
                      'crane monitor ~/.cache/crane/studies/boat01 --local --json'),
            raw='cat OUT/STUDY/**/progress.json; research status -p crane --campaign STUDY'),

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
                    P('--out', 'path', description='Output dir (default BUNDLE.fit/ beside the first bundle, else ~/.cache/crane/fit/BUNDLE/ when '
                      'that is not writable).'),
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
    Command(('bench', 'fast'), 'Fast-tier throughput: the scalar table B, the batched engine (--engine batched) or '
            'the gate-3 equivalence report (--equivalence).', 'bench.fast:crane_main', passthrough=True,
            about="bench.fast's tables: bench.json/bench.md (scalar) or batched.json/batched.md (episodes/s per "
                  'backend and E, x table B, the time split); arguments go to bench.fast unchanged.',
            examples=('crane bench fast --engine batched --domain boat --tier T1 --backend numba,jax --E 1024,4096 '
                      '--out /tmp/bench', 'crane bench fast --equivalence --tier T1 --seeds 0:500',
                      'crane bench fast --domain boat --tier T1 --seeds 0:50 --out /tmp/b'),
            raw='python3 -m bench.fast'),
    Command(('bench', 'cluster'), 'Throughput and cost on the cluster.', today='python3 -m bench.fast with its '
            '--fresh-jobs, on a cluster host', examples=('crane bench cluster --domain boat',)),
    Command(('cluster', 'submit'), 'Compile a study spec to rc-job/1 jobs (one per cell x seed chunk, 5-30 min '
            'each, pinned to the exact commit) and submit them with the research CLI; a jobs.json goes as is.',
            'crane.cluster.submit:submit_main', flags=(JSON, DRY), json='crane.cluster.submit/1',
            about='the plan table (stage, arm, block, role, kind, jobs, units, est per job, compute hours) and any '
                  'warnings, then the created/exists/duplicate counts and next:. --dry-run sends nothing (with '
                  '--json: the plan and every job body).',
            notes='Jobs: kind crane_batch (the crane-batch backend: one slot, every core) or gpu_command for a GPU '
                  'cell; seeds [0] with the chunk in spec.seeds; spec.run and spec.argv run python3 -m '
                  'crane_backends.batch, which runs crane study run-cell SPEC --cell JOB --out .. (the study\'s own '
                  'job ids and seed chunks; --confirm --targets-sha256 on held-out jobs); job ids come from the unit cache keys and '
                  'the commit, so resubmitting the same plan is idempotent. Refused (exit 2): uncommitted changes '
                  '(--allow-dirty runs the commit), a spec outside crane/runtime.sparse, a cell naming field data '
                  '(~/field_corpus_*, a bag dir, .mcap/.db3/.bag), a held-out block whose targets file is not '
                  'committed before now, a cell with no time estimate. Without --confirm held-out cells are left '
                  'out (only held-out selected: exit 2). The same compiler as crane study compile. The plan is saved to '
                  'OUT/STUDY/cluster/jobs.json before research submit. An adaptive stage goes after its source: '
                  'submit --stage A, wait, fetch, crane study promote --stage B, submit the promoted spec.',
            params=(P('source', metavar='SPEC|JOBS.json', description='A crane.study/1 spec, or an rc-job/1 list.'),
                    P('--stage', description='Only this stage.'),
                    P('--block', description='Only this seed block.'),
                    P('--est-unit-s', 'float', dest='est_unit_s', description='Seconds per (cell, seed) on one job '
                      "(default the study's estimate)."),
                    P('--est-from', 'path', dest='est_from', description='A bench.fast bench.json: seconds per '
                      'episode at one worker.'),
                    P('--measure', 'bool', description='Time one seed of the first cell locally (never for T2).'),
                    P('--project', default='crane', description='The research-cluster project.'),
                    P('--campaign', description='The campaign label (default the study name).'),
                    P('--purpose', description='1-2 sentences, public on the feed (default the spec purpose:).'),
                    P('--tag', repeat=True, description='needs.tags (repeatable; all required).'),
                    P('--cloud', 'bool', description='Allow rented boxes: needs.tags += cloud-ok and cpu (torch for '
                      'GPU cells); only with an approved REQUESTS.md row.'),
                    P('--kind', choices=('crane_batch', 'gpu_command'), description='Override the backend kind '
                      '(default the spec\'s kind:, gpu_command for a GPU cell).'),
                    P('--confirm', 'bool', description='Include held-out blocks (crane.study.guard checks the targets '
                      'file in git; each job pins its sha256 for the box).'),
                    P('--allow-dirty', 'bool', dest='allow_dirty', description='Submit with uncommitted changes '
                      '(jobs run the commit).'),
                    P('--no-skip-done', 'bool', dest='no_skip_done', description="Keep seeds already in the study's "
                      'unit cache.'),
                    sel('out', description='The studies root (default CRANE_STUDIES, else ~/.cache/crane/studies).'),
                    P('--dry-run', 'bool', description='Print the plan; send nothing.'),
                    P('--json', 'bool', description='One JSON document.')),
            examples=('crane cluster submit study.yaml --dry-run', 'crane cluster submit study.yaml --est-unit-s 12',
                      'crane cluster submit study.yaml --stage A --block dev --cloud',
                      'crane cluster submit /tmp/jobs.json'),
            raw='research submit -p crane OUT/STUDY/cluster/jobs.json'),
    Command(('cluster', 'wait'), "Block until a study's jobs end, up to --timeout seconds (one research wait), then "
            'print the status table.', 'crane.cluster.submit:wait_main', flags=(JSON,), json='crane.cluster.status/1',
            about='one line on stderr naming the jobs and the timeout, then the status table (as cluster status). '
                  'Exit 0 every job done, 1 some failed or were cancelled, 75 the timeout passed first (the jobs '
                  'are untouched), 2 no plan or no research CLI.',
            params=(P('study', nargs='?', metavar='STUDY|DIR|JOBS.json', description='A study name, its OUT dir, or '
                      'a plan.'),
                    P('--timeout', 'float', default=3600.0, description='Seconds to wait at most.'),
                    sel('out', description='The studies root (default CRANE_STUDIES, else ~/.cache/crane/studies).'),
                    P('--json', 'bool', description='The final status as one JSON document.')),
            examples=('crane cluster wait boat01 --timeout 3600',),
            raw='research wait -p crane JOB... --timeout 3600'),
    Command(('cluster', 'status'), "A study's cluster jobs per stage, arm and block, counted by state, with the "
            'failures (digest style; read-only).', 'crane.cluster.submit:status_main', flags=(JSON,),
            json='crane.cluster.status/1',
            about='a header with the state counts, one row per (stage, arm, block) with jobs, units and a column per '
                  'state, then failures (state, job, last error line) and next:. --json: {schema, study, project, '
                  'campaign, states, groups, failures, finished}. Without STUDY: research status (every job).',
            params=(P('study', nargs='?', metavar='STUDY|DIR|JOBS.json', description='A study name, its OUT dir, or '
                      'a plan (default: research status, every job).'),
                    sel('out', description='The studies root (default CRANE_STUDIES, else ~/.cache/crane/studies).'),
                    P('--json', 'bool', description='One JSON document.')),
            examples=('crane cluster status boat01',), raw='research status -p crane --campaign STUDY'),
    Command(('cluster', 'fetch'), "Bring a study's finished cluster outputs into OUT/STUDY/<stage>/<arm>/<cell>/, "
            'the layout a local run writes, each file checked by sha256.', 'crane.cluster.fetch:main', flags=(JSON,),
            json='crane.cluster.fetch/1',
            about='one row per job not yet in place (fetched, missing, failed; rows; from a box pull or the '
                  'cluster), the counts, and next:. Exit 1 when a file fails its sha256 check.',
            notes="Box pulls (~/.cache/vast-out/BOX/STUDY/JOB/, CRANE_VAST_OUT) first, else the coordinator's "
                  'artifacts (a viewer token: RC_VIEWER_TOKEN or ~/.config/research-cluster/platform-viewer.token). '
                  "Each job lands in <cell>/parts/JOB/; the cell's manifest.json lists every part's files. Safe to "
                  'rerun.',
            params=(P('study', metavar='STUDY|DIR|JOBS.json', description='A study name, its OUT dir, or a plan.'),
                    sel('out', description='The studies root (default CRANE_STUDIES, else ~/.cache/crane/studies).'),
                    P('--boxes-only', 'bool', dest='boxes_only', description='Only box pulls; do not ask the '
                      'coordinator.'),
                    P('--json', 'bool', description='One JSON document.')),
            examples=('crane cluster fetch boat01',), raw='research wait -p crane JOB... --out DIR; rsync from the box'),
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
    """The handler function of a command, or of a 'module:function' ref (imported now)."""
    import importlib
    module, _, fn = (command if isinstance(command, str) else command.handler).partition(':')
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
