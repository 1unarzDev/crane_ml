# Runs: the local index, queries and reproduction

```sh
crane runs index ~/.cache/crane/runs /tmp/study-out          # incremental; the same roots again: crane runs index
crane runs ls --domain boat --tier T1 --metric time
crane runs query --where "outcome=TIMEOUT and tier=T1 and metric.time>120"
crane runs show 3fa2c1d0                                     # meta, metrics, files; next: crane view RUN
crane runs diff 3fa2c1d0 77b0e9aa                            # which identity parts differ, and the metric deltas
crane repro 3fa2c1d0 --check                                 # rerun at the recorded commit; exit 1 if it differs
```

Every verb takes `--json`. Exit codes: 0 ok, 1 `repro --check` did not reproduce, 2 usage, a bad `--where`, an
unknown run or a refused repro.

## The index

One SQLite file, `~/.cache/crane/index.sqlite` (`CRANE_INDEX` moves it). It is derived data: delete it or pass
`--rebuild` at any time. A row per record found under a root:

| Record | Found by | Metrics |
|---|---|---|
| crane.run/1 dir | `crane.run.json`, or `run.json` with schema `crane.run/1` | `metrics.json` |
| fastsim run dir | `run.json` with schema `crane.fastsim.run/1`; its crane.run/1 meta is derived, nothing is written | `time` (= `sim_s`), `sim_s`, `time_limit_s`, `min_clearance_m`, `contacts`, `idle_s`, and `metrics.json` |
| study or dataset output | `manifest.json` with schema `crane.data.manifest/1` | `metrics.json` beside it |

Columns: `run_id path source domain tier kind seed card backend dtype commit dirty outcome created study stage arm
block run_sha256`. Metrics are flattened to dotted keys (`score.a`), at most 1000 per record. A fastsim batch's
variant is its `arm`. `created` is the record's own time when it has one, else the meta file's mtime.

Indexing is incremental: a record whose meta files keep their mtime is skipped, one whose bytes keep their sha256
is not reread, and records gone from a root are dropped. Run dirs are not descended into; hidden dirs
(`.s3.partial-*`) and `*.stale-N` dirs are skipped. `crane runs ls` on a new, empty index first indexes
`~/.cache/crane/runs`.

## --where

```
outcome=TIMEOUT and tier=T1 and metric.time>120
(arm=a or arm~'base*') and not dirty=true and seed in (1, 2, 3)
commit=7597894c and metric.min_clearance_m<0.5 and created>2026-11-20
card=null
```

- `FIELD OP VALUE` or `FIELD in (V, ...)`, joined by `and`, `or`, `not` and parentheses.
- OP: `= != < <= > >=`, and `~`, a glob (`*`, `?`); a value without either matches anywhere.
- FIELD: a column above or `metric.NAME`. `tier` takes `T0|T1|T2` or `abstract|dynamic|unity`; `commit=` and
  `run_id=` take a hex prefix; a bare `null` with `=` or `!=` tests for a missing value (or metric).
- VALUE: a bare word, a number, or a quoted string.

The expression is parsed, never evaluated. Fields come from a fixed list, and every value is a bound SQL parameter.
A bad expression is one line naming the column.

## Cluster runs

`--cluster --study NAME` on `ls` and `query` adds the coordinator's runs of that study (`GET /v1/runs/query`,
read-only), filtered by the same `--where`. It reads `CRANE_COORDINATOR`, and `CRANE_VIEWER_TOKEN` or
`CRANE_TOKEN` (else `~/.config/research-cluster/crane-viewer.token`). When the coordinator is unset or
unreachable, a line on stderr says so and the local runs are listed.

## diff

`crane runs diff A B` says `identity: the same run` when the run_sha256 values match. Otherwise it names each
identity part that differs and why:
- config: the changed keys, old -> new;
- scenario: the scenario names;
- adapter: the changed identity keys;
- seed;
- core: the commits, and whether each tree was dirty.

Then it lists the meta fields that differ, and each differing metric with A, B and B-A.

## repro

`crane repro RUN` reruns a fastsim run (T0/T1):
1. It reads the run's identity: run_sha256 over scenario, config, adapter, seed and core commit
   (`fastsim/identity.py`).
2. It checks the recorded commit out into a temporary worktree (`git worktree add --detach`). The checkout you work
   in is never touched.
3. That commit's fastsim reruns the one seed on the scenario file the run dir holds, with the recorded config and
   provenance.
4. It compares run_sha256, the directory's tree_sha (naming the first differing file) and the metrics.

The worktree is always removed. A rerun that differs is kept, and `next:` gives the `crane runs diff` for it.

It refuses with one line (exit 2):
- a run made from a dirty tree, showing its diff sha when one was recorded;
- a commit no repository here has (the run's recorded source, then this checkout); fix it with `git fetch`;
- a record that is not a fastsim run.
