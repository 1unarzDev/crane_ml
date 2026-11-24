# crane

```bash
crane doctor
crane run --domain toy --seeds 0:3 --set workers=2          # runs land in ~/.cache/crane/runs/toy/T0/<stamp>
crane run --domain boat --tier T1 --seeds 0:2 --dry-run      # the resolved plan; nothing runs
crane sim up --headless && crane sim check --wait 120
crane help                                                    # every noun and verb
```

`crane NOUN [VERB] [selectors] [flags]`. `crane NOUN VERB --help` gives the options, an example, what you'll see and
the low-level equivalent. Without an install, `bin/crane` runs this checkout (it prefers `.venv/bin/python`, else
`python3`); astro calls `${CRANE_DIR:-~/crane_ml}/bin/crane` without a shell. The core is stdlib plus numpy and
pyyaml; each command imports what it needs only when it runs, and `crane sim` and `crane help` are stdlib only.

Every command, its params and examples: [Docs/CLI.md](../Docs/CLI.md), generated from the registry.

## Pieces

- `crane/commands.py`: the one registry of nouns and verbs (summary, params, examples, `raw`, JSON schema, handler
  `module:function`). argparse, `--help`, `crane help`, `help.yaml` and `Docs/CLI.md` come from it;
  `crane help --check` fails when the generated files drift, `crane help --write` regenerates them. A command not
  built yet names what to use today and exits 2.
- `crane/cli.py`: the parser built from the registry, and the dispatch. Pass-through commands (`sim`, `heavy`,
  `judge`, `card`, `board`, `env`/`course`, `cluster`) hand every argument after their words to their tool.
- `crane/select.py`: the shared selectors (`--domain --tier --env --robot --card --seed --seeds --set --out`).
  `parse_seeds` takes `N`, `A:B`, `a,b,c`, `@FILE` and `block:NAME` (`seeds:` in `config/domains/D.yaml`, each a
  spec or `{seeds, role}`). `--out` defaults to `~/.cache/crane/runs/DOMAIN/TIER/STAMP` (`CRANE_RUNS` moves the root).
- `crane/output.py`: `--json` (one document; JSONL for streams), `--quiet`, `--dry-run`; aligned tables, one-line
  failures `crane NOUN VERB: cause; fix: CMD`, `next:` lines. Exit codes: 0 ok, 1 failed (judge FAIL included),
  2 usage or config, 75 a bounded wait timed out.
- `crane/paths.py`: every path a command execs, named once.
- `crane/tiers.py`: `normalise('dynamic') == 'T1'`; T0 abstract, T1 dynamic, T2 unity.
- `crane/run.py`: `--domain D [--tier T]` reads `config/domains/D.yaml` through `bench.fast.domain_tier`: the tier
  (default the first) names a fastsim config and optional `overrides` merged over it. Missing scenarios are made by
  the config's generator, or the dir is named when there is none; `--generate` regenerates.
- `crane/data/`: `rows.py` (RowWriter, read_rows and manifest.json with a sha256 per file; parquet, csv or npz),
  `recorder.py` (a run's `record:` level: events, decimated trajectories, full-rate obs and actions; `Store` for
  study-level arrays), `export.py` (`crane data export`, `crane data schema`) and `splits.py` (`crane data splits`,
  held-out-safe). Examples: [Docs/Data.md](../Docs/Data.md).
- `crane/build.py`: after the build, `sourceCommit` and `sourceDirty` go into
  `Builds/CRANE-Worker/crane-build-manifest.json`; `--tarball` writes
  `Builds/crane-player-<commit12>-linux-x86_64.tar.zst` (`.tar.gz` without zstd), without debug symbols, and a
  `.sha256` in `sha256sum` format.
- `config/sim.yaml` (the scene's default current, the player's required flag) and `config/bench.yaml` (sweep counts).
- `help.yaml` (repo root, generated): the user-facing commands in astro's help schema, so `astro crane VERB` runs them.

## Install

`./install.sh [--yes] [--dir D] [--components core,fast,gpu,view,field,data,setup] [--domains …] [--no-clone]
[--dry-run]` installs uv (to `~/.local/bin`), a Python 3.12 venv at `<repo>/.venv`, `pip install -e .[extras]`
and the link `~/.local/bin/crane`. It never uses sudo and prints each step. The extras are `fast` (numba), `gpu`
(jax CUDA 12), `view` (PySide6, pyqtgraph, PyOpenGL), `field` (mcap), `data` (pyarrow, for parquet), `setup` (rich, questionary) and `all`. The editable
install also registers the `research_cluster.backends` entry points (fastsim, unity, theater-render).
