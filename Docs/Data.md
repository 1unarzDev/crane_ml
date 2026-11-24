# Data: rows, recorded arrays, datasets and splits

```sh
# 50 runs that also record full-rate actions and plant state, float16
crane run --domain toy --seeds 0:50 --out /tmp/runs \
  --set 'record={obs: {channels: [cmd, vel], hz: 20, dtype: float16}, labels: {fault: none}}'
crane data export /tmp/runs --format npz --out /tmp/ds          # one row per run, obs as runs x T x channels
crane data schema /tmp/ds
crane data splits /tmp/ds --by seed                              # /tmp/ds/splits.json
crane data export /tmp/ds --split /tmp/ds/splits.json:train --out /tmp/train
```

```python
import numpy as np
z = np.load('/tmp/train/part-0000.npz')     # numpy only, no pickle
z['obs'].shape, z['obs'].dtype              # (runs, T, 6) float16; shorter runs NaN-padded
z['obs@len']                                # each run's T (only when they differ)
z['seed'], z['outcome']
```

## Rows (`crane.data.rows`)
One row per episode, metrics first, with its condition keys and seed. Studies and exports write rows the same way:

```python
from crane.data.rows import RowWriter, write_manifest, read_rows
w = RowWriter(out, {'seed': 'int', 'arm': 'str', 'score': 'float', 'obs': 'array:float16'})
w.write({'seed': 3, 'arm': 'a', 'score': 0.5, 'obs': obs})      # a column the schema lacks is refused
write_manifest(out, w.schema, w.close(), {'study': 'capxfer'})
rows = list(read_rows(out))
```

- Types: `int`, `float`, `str`, `bool`, `json` (any JSON value) and `array:DTYPE` (one numpy array per row). A missing
  value is null.
- `fmt`: `auto` (parquet when pyarrow imports, else csv), `parquet`, `csv`, `npz` or `jsonl`. csv, jsonl and npz read
  back with the stdlib and numpy alone; parquet needs the `[data]` extra (`pip install 'crane[data]'`).
- `read_rows` also reads any `.jsonl` of one JSON object per line, such as a unit cache's `records.jsonl`.
- Null and NaN: parquet, jsonl and npz keep a null apart from a float NaN. csv has one empty cell for null, so an
  empty string reads back as null; a NaN reads back as NaN.
- Array columns are stored in npz. With `fmt='npz'` they sit in the part itself; with csv, jsonl or parquet they go in
  `PART.arrays.npz` beside it, row for row. Rows of unequal length are padded (NaN for floats, 0 for ints), and
  `COL@len` holds each length.
- A part rolls over to `PART-001`, `PART-002`... at one million rows or about 90 MB of arrays, so every file stays
  under the cluster's 95 MB artifact cap.
- npz files have fixed zip timestamps, so equal rows give equal bytes.

`manifest.json`:

```json
{"schema": "crane.data.manifest/1", "columns": {"seed": "int", "obs": "array:float16"}, "rows": 50,
 "files": [{"path": "part-0000.npz", "rows": 50, "sha256": "…"}], "source": ["…"], "arrays": {"obs": {…}}}
```

`rows.verify(dir)` lists every file that is missing or whose sha256 differs.

## Recording (`record:`)
The `record:` key goes in a fastsim domain YAML, or on the command line through `crane run --set record=...`.
Without the key nothing is recorded, and the run directory and run identity are byte for byte what they would be
otherwise. The key is hashed, so a recorded run never reuses a plain run's directory.

```yaml
record: metrics                                # the summary only (the same as no key)
record: events                                 # the events also become rows (Store)
record: {traj: {hz: 20, window: 12}}           # world pose + body velocity, 20 Hz, first 12 s
record: {obs: {channels: [cmd, vel, tracks.n], hz: 50, dtype: float16, window: [0, 40]}, labels: {fault: wheel2}}
record: [events, {traj: {hz: 20, window: 12}}, obs]
```

| Channel | Columns | What |
|---|---|---|
| `pose` | `pose.x pose.y pose.yaw` | the map-frame pose the controller sees |
| `vel` | `vel.vx vel.vy vel.wz` | the plant's body velocity |
| `cmd` | `cmd.vx cmd.vy cmd.wz` | the body velocity commanded this step (the action) |
| `tracks.n` | `tracks.n` | tracks in the last observation |
| (adapter) | its names | `DomainAdapter.observe(sample) -> {name: number or numbers}`; sample has t, pose, vel, cmd, obs, ctx |

- `hz` is rounded to a whole number of plant steps, and the rate actually kept is recorded. With no `hz`, every plant
  step is recorded.
- Labels are the config's `labels` merged with the adapter's optional `labels(ctx)`.
- A run dir gets `record/obs.npz` (`obs` T x C, `channels`, `t0`, `hz`, `labels` as JSON) and `record/traj.npz`
  (`traj` T x 6: x y yaw vx vy wz).
- `run.json` gets a `record` block, and `crane.run.json` gets `files.obs`. `recorder.load_run(dir)` reads the arrays
  back.

A study or a batched job can skip run dirs entirely and add each run to a store, one dataset per kind:

```python
from crane.data.recorder import Recorder, Store
store = Store(out / 'arrays', part=job_id)                    # one part per job; later jobs append
rec = Recorder({'obs': {'channels': ['cmd', 'vel'], 'hz': 50, 'dtype': 'float16'}})
simulate(scenario, adapter, cfg, seed, run_dir, base, recorder=rec)   # an explicit recorder: same identity
store.add({'cell': cell_id, 'seed': seed}, rec)
store.close()                                                  # out/arrays/obs/{part.npz, manifest.json}
```

## Export and schema
- `crane data export SRC... --out DIR` takes run dirs, batch outputs (walked; `.stale-N` and partial dirs are
  skipped), datasets (a dir with `manifest.json`) or row files.
- A run dir gives one row: `run variant seed outcome sim_s`, then `condition.*`, the flattened `metrics.json`,
  `commit`, `run_sha256`, and `obs` / `traj` with `label.*` when the run recorded them.
- `--select seed,outcome,obs` keeps only those columns. `--where 'seed>=10'` keeps matching rows; the operators are
  `= != > >= < <=`, and `=` and `!=` take a list `a,b`. `--split FILE:NAME` keeps one split.
- `--out` must be new or empty, because a dataset is written once.
- The export notes, without refusing, rows tagged held-out (they carry their outcomes; withholding them is the study
  layer's job) and non-finite floats per column. `--json` has them as `held_count` and `nonfinite`.
- `crane data schema SRC` prints every column's name and type (for arrays also the rate and channels) and the row
  count. It reads a dataset's manifest, or scans run dirs.

## Splits
`crane data splits SRC --by seed|condition|timeline|COLUMN --ratios train=0.7,calib=0.15,test=0.15` writes
`crane.data.splits/1`: `{by, columns, ratios, salt, held_out_rule, splits: {NAME: {units, rows, n_units, fraction}}}`.

- A unit's split comes from a salted sha256 of its value. A unit therefore keeps its split as data grows. Fractions
  are close to the ratios, not exact. With few units a split can come out empty; the output then warns
  (`warnings` in the file).
- A unit is `held-out`, and in no other split, when any of its rows is held-out. A row is held-out when:
  - its `role`, `block_role` or `tag` column is held-out;
  - its `held_out` column is true;
  - its `block` or seed is a held-out block of `--domain`'s `seeds:`;
  - or it matches a `--held-out` expression (e.g. `source=real`, so real sessions never enter train or calib).
- `held-out` cannot be a ratio name.
- Writing over an existing split file refuses to move any unit from another split, or from held-out, into `train` or
  `calib`.
