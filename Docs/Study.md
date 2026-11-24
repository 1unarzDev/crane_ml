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
arms:                            # what is compared
  base: {}
  far: {set: {adapter.options.reach_m: 2.5}}    # config overrides, dotted keys
  measured: {card: characterized}                # a plant card (domain card.dir, or a path)
  llm: {task: tasks.py:judge, args: {model: m}}  # a callable (args, seed, cell) -> {name: number}
  ext: {argv: ['{python}', run.py, '{args}', '{out}'], timeout_s: 600}   # a command; JSON args
  rand: {sample: {n: 8, seed: 1, set: {vehicle.gain: {uniform: [0.6, 1.4]}}}}   # rand-0 .. rand-7
axes:                            # crossed with every arm
  fov: {param: perception.hfov_deg, levels: [60, 90]}
  cruise: {param: limits.vmax, levels: {slow: [1.0, 0.3, 0.6], fast: [1.5, 0.3, 0.6]}}
  gain: {levels: {pre: {vehicle.gain: 1.0}, measured: {vehicle.gain: 0.85}}}   # named sources: value sets
  card: {card: true, levels: {pre: ideal, measured: characterized}}           # named sources: cards
conditions: {csv: conditions.csv, id: id, set: {range_m: perception.range_m}}  # one condition per row
splits: {column: split, roles: {calib: calib, test: held-out}}                # conditions by a column
seeds:                           # the seed is the pairing unit: every arm runs the same seeds
  dev: {role: dev, seeds: '0:8'}             # N, A:B, a,b,c, @FILE, block:NAME (the domain's seeds:)
  test: {role: held-out, seeds: '@held.txt'}
targets: toy_paired.targets.yaml # committed before a held-out block may run
scorer: scorers.py:toy           # (result, cell) -> {part: number}; rows get score.PART
metrics: [succeeded, run.sim_s]  # the fixed row columns (default: every number the runs give, sorted)
record: metrics                  # metrics | events | traj_decimated | obs, or {level, hz, until_s}
crn: plain                       # plain: the run seed is the seed; derive (task arms): derive_seed(seed, study, condition)
baseline: base                   # the arm the report compares the others with (default the first)
stages:                          # default: one stage, main, of everything
  R: {tier: T0, blocks: [dev]}
  V: {after: R, select: {top_k: 2, metric: score.total, order: max, by: arm}, tier: T1, set: {dt: 0.01}}
pack: {unit_s: 0.05, target_s: 900}   # seconds per unit; the cluster plan packs a cell's seeds to ~target_s
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

## Pairing and roles

- A cell is one (stage, arm, axis levels, condition, tier, seed block); its units are its seeds. Every row records
  `seed` (the pairing unit) and `run_seed` (what the episode ran with). Neither depends on the arm.
- Rows of cells that differ only in the arm share `pair`; the report pairs them on `seed`.
- A cell's `role` is its seed block's, or its split's when it has one; held-out wins over either. Blocks and splits
  are reported as `block` or `block/split`.
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
  pins the targets file's sha256 into each held-out job's argv (`--targets-sha256`); `run-cell` on the box checks the
  file it got against that hash.
- Held-out rows carry `touched_held_out = true`. The manifest records the targets commit and sha256, and
  OUT/held_out.jsonl logs every confirmed run.
- `crane study status` shows progress only for held-out blocks. Interim means are shown for dev blocks only.
  metrics.json gives held-out blocks unit counts only. `crane study report` leaves a held-out block out until all its
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
| `rows/STAGE.parquet` or `.csv` | the rows: key columns, axes, `cond.*`, outcome, metrics, `score.*`, wall_s, commit, card_sha, touched_held_out |
| `manifest.json` | `crane.data.manifest/1`: columns, files with rows and sha256, units done/total per stage and block |
| `metrics.json` | at most 1000 finite numbers: unit counts, per-arm means of blocks that are not held-out |
| `progress.json` | `{done, total, failed, rate_per_s, eta_s, phase, updated, label}`, rewritten at least every 60 s |
| `failures.jsonl` | units that raised, with the traceback; they are not cached, so a rerun retries them |
| `traj.npz`, `events.jsonl` | with `record: traj_decimated` (default 2 Hz; `hz`, `until_s`), `events` or `obs` (full rate) |
| `report.md`, `report.json` | `crane study report` |
| `study.json` | the spec this output belongs to (`crane study status OUT` reads it) |

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

`crane study compile` writes `{jobs: [rc-job/1 ...]}`, one job per cell and seed chunk (`kind` crane_batch or
gpu_command, `est_seconds` for the whole job, `needs.tags` from `pin.tags`). The backend runs `spec.argv`
(`crane study run-cell SPEC --cell JOB_ID --out /workspace/out/STUDY/JOB_ID [--confirm --targets-sha256 SHA]`)
from the crane checkout and ships `spec.out`. `crane study plan SPEC --json` lists the same jobs with their
`targets` pin, or `blocked` when the targets file is not ready. An adaptive stage appears once its source stage is
done; promote it to ship it. `crane study run --where cluster` names compile.

While units run, progress.json is rewritten at least every 20 s and a progress line goes to stdout every minute.
The manifest's `oversize` lists files over the cluster's 95 MB artifact cap.
