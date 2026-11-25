# Studies (`crane.study/1`)

```sh
crane study init --out study.yaml                       # a commented spec to start from
crane study plan examples/studies/toy_paired.yaml       # cells, units, jobs, estimated time; nothing runs
crane study run examples/studies/toy_paired.yaml --max-workers 4
crane study status ~/.cache/crane/studies/toy_paired    # progress, failures, interim dev means
crane study report ~/.cache/crane/studies/toy_paired    # paired analysis: report.md + report.json
crane study run examples/studies/toy_paired.yaml --block test --confirm    # the held-out block
crane study compile examples/studies/toy_paired.yaml --write jobs.json     # rc-job/1 bodies for the cluster
```

One YAML spec describes arms x axes x conditions x tiers x seed blocks. `crane study run` runs it here, and
`crane study compile` turns the same work into rc-job/1 cluster jobs, each of which runs `crane study run-cell`.

```python
from crane import study
s = study.load('examples/studies/toy_paired.yaml')
for cell in s.cells():          # Cell{study, stage, arm, axes, condition, tier, seeds, block, key, ...}
    rows = cell.run('/tmp/toy') # runs the seeds that are not cached; returns the rows of the done ones
s.jobs()                        # [{job_id, cell, seeds, est_seconds, kind, campaign, purpose, expect, pin, argv}]
```

## The spec

```yaml
schema: crane.study/1
name: toy_paired                 # the output dir name and part of every cache key
domain: toy                      # config/domains/toy.yaml: sim arms run its tiers
tier: T0                         # or tiers: [T0, T1]; a cell per tier
engine: batched                  # scalar (fastsim, the default) | batched (one engine call per cell x seed chunk);
backend: numba                   #   or per tier {T1: batched}. backend numpy | numba | jax, dtype float64 | float32
plugins: [plugins]               # dirs whose modules resolve module:attr refs, for this study's runs only
arms:                            # what is compared
  base: {}
  far: {set: {adapter.options.reach_m: 2.5}}    # config overrides, dotted keys
  measured: {card: characterized}                # a plant card (domain card.dir, or a path)
  llm: {task: tasks.py:judge, args: {model: m}}  # a callable (args, seed, cell) -> {name: number}
  ext: {argv: ['{python}', run.py, '{args}', '{out}'], timeout_s: 600}   # a command; JSON args
  rand: {sample: {n: 8, seed: 1, set: {vehicle.gain: {uniform: [0.6, 1.4]}}}}   # rand-0 .. rand-7
  mine: {mission: 'mymod:MyAdapter', engine: scalar}   # a plugin DomainAdapter (Python mission code); engine per arm
axes:                            # crossed with every arm
  fov: {param: perception.hfov_deg, levels: [60, 90]}
  cruise: {param: limits.vmax, levels: {slow: [1.0, 0.3, 0.6], fast: [1.5, 0.3, 0.6]}}
  gain: {levels: {pre: {vehicle.gain: 1.0}, measured: {vehicle.gain: 0.85}}}   # named sources: value sets
  card: {card: true, levels: {pre: ideal, measured: characterized}}           # named sources: cards
card: ideal                      # the base card the card axes edit (an arm's card overrides it)
card_axes:                       # axes whose levels edit card fields; crossed with everything else
  thrust: {fields: [thrusters.*.kForward, thrusters.*.kReverse], levels: {pre: base, measured: m.json}}
  surge: {fields: [addedMass.a11, surge.d1Pos], transform: scale, levels: [0.5, 1, 2]}
conditions: {csv: conditions.csv, id: id, set: {range_m: perception.range_m}}  # one condition per row
splits: {column: split, roles: {calib: calib, test: held-out}}                # conditions by a column
# splits: {column: session_id, groups: {test: [s07, s09], calib: '*'}, roles: {calib: calib, test: held-out}}
seeds:                           # the seed is the pairing unit: every arm runs the same seeds
  dev: {role: dev, seeds: '0:8'}             # N, A:B, a,b,c, @FILE, block:NAME (the domain's seeds:)
  test: {role: held-out, seeds: '@held.txt'}
targets: toy_paired.targets.yaml # committed before a held-out block may run
scorer: scorers.py:toy           # (result, cell) -> {part: number}; rows get score.PART
scorers: [{fn: 'pop:fit', over: card x block}]   # population scorers: (rows of one group, group) -> {part: number}
report: {population: {part: fit.loss, order: min, cluster: cond.session_id, B: 200}}
metrics: [succeeded, run.sim_s]  # the fixed row columns (default: every number the runs give, sorted)
record: metrics                  # metrics | events | traj_decimated | obs, or {level, hz, until_s}
crn: plain                       # plain: the run seed is the seed; derive (task arms): derive_seed(seed, study, condition)
baseline: base                   # the arm the report compares the others with (default the first)
stages:                          # default: one stage, main, of everything
  R: {tier: T0, blocks: [dev]}
  V: {after: R, select: {top_k: 2, metric: score.total, order: max, by: arm}, tier: T1, set: {dt: 0.01}}
pack: {unit_s: 0.05, target_s: 900}   # seconds per unit ({scalar: S, batched: S, task: S} by engine); the
                                      # cluster plan packs a cell's seeds to ~target_s
campaign: toy-paired             # campaign, purpose, expect, pin and kind go to the cluster compiler as given
purpose: Paired toy study of the visiting reach across cruise speed and field of view.
expect: far finishes sooner than base
pin: {hosts: [innovation], tags: [cloud-ok]}
kind: crane_batch
```

A spec error is one line naming the key and the fix: `study.yaml: seeds.dev.role: 'devv'; fix: one of dev, check,
train, calib, held-out`.

Each cell's config is the domain tier's, with the arm's `set`, the axis levels, the condition's `set` columns and the
stage's `set` merged over it, in that order. A card binds to the domain's `card.keys` (else `view.card`). A task arm
gets the same merge as its `args`; `argv` fills in `{args}` (JSON), `{seed}`, `{out}` (a unit dir), `{cell}` (JSON),
`{python}` and `{spec_dir}`, and its numbers are `{out}/metrics.json` or the last JSON line it prints. Relative paths
in a spec (CSV, targets, `file.py:function`, `@FILE`) are relative to the spec.

## The batched engine

`engine: batched` runs a sim cell's seed chunk (256 by default, `pack.local_chunk`) in one engine call of E envs
([Engine.md](Engine.md)): `backend: numpy` needs numpy alone (the float64 reference), `numba` every core, `jax` the
GPU (float32 by default; a jax cell asks the cluster for a GPU). The rows are the scalar path's: the same columns and,
in float64, the same values (`run.*`, the domain adapter's own metric names, `succeeded`, `score.*`), and the
`events`/`traj_decimated` records; `engine` names the path on each row. The engine is in the unit key, not in
`pair`, so arms on either path pair on the seed. `record: obs` (full rate) is refused on a batched cell with one
line. An arm the engine has no batched controller for, such as a plugin `mission`, runs on the scalar path, and
`crane study plan` says so and estimates it at `pack.unit_s.scalar`.

## Plugins

`plugins: [DIR]` (relative to the spec) puts those dirs on `sys.path` for this study's run, job or report only (its
modules are dropped afterwards, so two studies in one process never share one), so `mission: module:Class` (a
fastsim DomainAdapter replacing the domain's), `task:`, `scorer:` and `scorers[].fn` resolve there.
Nothing is copied: the manifest records each dir's git commit, dirty flag and diff sha, and the sha256 of its files
goes into every cell hash (an edited plugin reruns). On the cluster the dir ships with the spec's repo.

## Card axes

Each card axis names card fields (dotted paths into the card JSON; `*` is every list item or key) and its levels:
`base` (the card as is), a card ref (a named source: those fields' values from that file, which needs only those
fields), a number (every field set to it), `{values: {PATH: V}}`, or `{scale: X}` (a multiplicative transform;
`transform: scale` makes plain-number levels multipliers, named `x0.5`). Axes apply in spec order to the base card
(`card:`, or the arm's). The factorial is crossed lazily, cell by cell; each derived card is made once, written to
`OUT/cards/SHA.json` when its cells run and bound to the domain's card keys (a task arm gets its path as
`args.card`). Rows carry the level names and `card_sha`; the manifest's `cards` maps each sha to its levels.
`crane study plan` counts the cards.

## Population scorers

`scorers: [{fn, over, args, name}]` group the rows by `over` (keys joined by ` x `: card, block, split, role, arm,
tier, stage, an axis, or a `cond.` column) and call `fn(rows, group, **args)` with each group's rows (condition
columns and sim columns) to get named parts. `crane study report` lists every group's parts, and with
`report.population` ranks the cards (the `over` keys other than block/split/role) by `part` on the `calib` role's
rows (default `calib`), then gives the Spearman correlation of the calib and held-out (`test`, default `held-out`)
scores across cards, with a cluster bootstrap CI that resamples `cluster` values (default `condition`; e.g.
`cond.session_id`) within each side (`judge.stats.cluster_bootstrap_ci`). Held-out rows enter once their block is
done.

## Pairing and roles

- A cell is one (stage, arm, axis levels, condition, tier, seed block); its units are its seeds. Every row records
  `seed` (the pairing unit) and `run_seed` (what the episode ran with). Neither depends on the arm.
- Rows of cells that differ only in the arm share `pair`; the report pairs them on `seed`.
- A cell's `role` is its seed block's, or its split's when it has one; held-out wins over either. Blocks and splits
  are reported as `block` or `block/split`.
- A split is a condition column's value, or with `splits.groups` a named group of values (`[s07, s09]`, `@FILE`, or
  `'*'` for every other value), so whole sessions can be held out; a value sits in one split only.
- A seed in a held-out block may not also sit in another block.

## Held-out blocks

- `crane study run` skips held-out cells and says so. `--block B --confirm` runs them (and `--confirm` alone runs
  everything), but only when the spec's `targets` file parses, is committed, has no local changes and was last
  committed before the output's first held-out unit (before now, on a first run). Otherwise it refuses with one line
  and exits 2.
- A targets file is a list of `{contrast: A - B, metric: M, direction: lower|higher|not lower|not higher}` naming
  the spec's arms. `crane study report` gives each a verdict on the done held-out blocks (met, not met,
  inconclusive, by the 95% CI of the pooled paired difference).
- A rented box has no git checkout of the study repo. `crane study compile --confirm` checks git where it runs and
  pins the targets file's sha256 into each held-out job; crane-batch passes it to `run-cell` as `--targets-sha256`,
  and the box checks the file it got against that hash.
- Held-out rows carry `touched_held_out = true`. The manifest records the targets commit and sha256, and
  OUT/held_out.jsonl logs every confirmed run.
- `crane study status` shows progress only for held-out blocks. Interim means are shown for dev blocks only
  (`crane.study.guard.shows_means`, which crane monitor and crane-batch use too); a failure's message shows for dev
  and check blocks only. metrics.json gives every other block unit counts only. `crane study report` leaves a held-out block out until all its
  units are done.

## Adaptive stages

A stage with `select` waits until every non-held-out unit of the stage it reads (`from`, else its first `after`) is
done. It then keeps the `top_k` groups (`by: arm`, or arm plus axes) by the mean of `metric`; held-out rows never
count. `crane study run` runs the stages in order. `crane study promote SPEC --stage V` writes
`SPEC.V.yaml`, where V names its arms (or `only:` combinations) and has no `select`. A cluster job can run that spec,
and its cells hash the same, so the cache is shared.

## Outputs

`~/.cache/crane/studies/NAME/` by default (`--out DIR`); a cluster job writes `/workspace/out/STUDY/JOB_ID/`.

| Path | What |
|---|---|
| `cache/records.jsonl` | one line per finished unit, keyed on (study, stage, arm, cell hash, seed, card sha, code commit); `crane.cache.UnitCache` |
| `rows/STAGE.parquet` or `.csv` | the rows: key columns, axes, `cond.*`, outcome, error, censored, time_cap_s, metrics, `score.*`, wall_s, commit, card_sha, engine, touched_held_out |
| `cards/SHA.json` | the derived cards (card_axes) the rows name by `card_sha` |
| `manifest.json` | `crane.data.manifest/1`: columns, files with rows and sha256, units done/total per stage and block |
| `metrics.json` | at most 1000 finite numbers: unit counts, per-arm means of blocks that are not held-out |
| `progress.json` | `{done, total, failed, rate_per_s, eta_s, phase, updated, label}`, rewritten at least every 60 s |
| `failures.jsonl` | units that raised, with the traceback; they are not cached, so a rerun retries them |
| `traj.npz`, `events.jsonl` | with `record: traj_decimated` (default 2 Hz; `hz`, `until_s`), `events` or `obs` (full rate) |
| `report.md`, `report.json` | `crane study report` |
| `study.json` | the spec this output belongs to (`crane study status OUT` reads it) |

**Censoring.** A unit that ends at its time cap (outcome TIMEOUT, or a task returning `censored: true`) has
`censored = true` and `time_cap_s`, the cap; its time metrics are the cap or absent, never NaN. A task that returns
`censored` without `time_cap_s` fails with one line. On a sim cell the cap is where the sim stops the episode: the
run's `time_limit_s` plus the tier config's `grace_s`. Every sim column (`run.sim_s`, `boat.gates_passed`, ...)
covers the episode up to that cap, and an episode that succeeds before it is observed, not censored.

Sim run dirs are read and removed. With `keep_runs: true` they stay in `runs/CELL/`. A rerun of the same command
runs only the units not in the cache, and so does a run killed mid-way. A changed arm, level, condition, scorer or
commit changes the key, and those units run again; `crane study status` counts units cached at other commits.

## Report

`crane study report OUT` compares each arm with the baseline on every metric (`--metric` to choose):
- per cell: the mean of the seed-paired differences, a bootstrap CI over seeds, a sign-flip permutation p
  (`judge.stats.bootstrap_ci`, `permutation_paired`), and Holm across the cells of one (stage, arm, metric);
- pooled per (stage, block): each seed's differences averaged over the cells, then the same tests, plus how many
  cells pass Holm.

OUT may also be a dir of fetched job outputs, one `manifest.json` each.

## Cluster jobs

```sh
crane study compile SPEC --write jobs.json            # dev/check/calib jobs; held-out ones need --confirm
research submit -p crane jobs.json
```

`crane study compile` writes `{jobs: [rc-job/1 ...]}` through the cluster compiler (`crane.cluster.compile`, the
one `crane cluster submit` uses; [Cluster.md](Cluster.md) has every field): one job per cell and seed chunk of about
`pack.target_s` (default 15 min), `kind` crane_batch or gpu_command, repos pinned at the commit, dedup ids, `needs`
from `pin:`, and campaign, purpose and expect. Each job runs `crane study run-cell SPEC --cell JOB_ID --out DIR`
through crane-batch; a held-out one adds `--confirm --targets-sha256 SHA`. The spec must be committed under a
directory in `crane/runtime.sparse` (or in its own repo), and field data is refused. `crane study plan SPEC --json`
lists the same chunks with their `targets` pin, or `blocked` when the targets file is not ready. An adaptive stage
appears once its source stage is done; promote it to ship it. `crane study run --where cluster` names compile.

While units run, progress.json is rewritten at least every 20 s and a progress line goes to stdout every minute.
The manifest's `oversize` lists files over the cluster's 95 MB artifact cap.
