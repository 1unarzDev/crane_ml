# crane

```bash
crane doctor
crane run --domain toy --seeds 0:3 --out /tmp/runs --set workers=2
crane run --domain boat --tier T1 --seeds 0:2 --out /tmp/runs
crane sim up --headless && crane sim status
```

`crane VERB --help` gives each verb's options, an example and what you'll see. From a checkout with no install,
`python3 -m crane VERB` does the same (the repo root on `sys.path`). The core is stdlib plus numpy and pyyaml;
each verb imports what it needs only when it runs, and the `sim` path is stdlib only.

## Verbs

| Verb | Runs | Module or tool |
|---|---|---|
| `sim up\|down\|status` | the T2 world plus ROS, like the boat | execs `Tools/RoboBoat/sim_up` with the arguments unchanged |
| `sim check` | is the running sim ready | live readiness of the running sim; `crane sim check CHECK …` runs Tools/SimCheck (`sim_up check` calls it) |
| `heavy …` | the host concurrency guard | execs `bin/heavy` |
| `run --domain D [--tier T]` | episodes at T0 or T1, one run dir per seed | `fastsim.batch.run_batch` (`crane/run.py`) |
| `judge …` | scoring | `python -m judge` |
| `card show\|export` | plant cards and their tier views | `python -m plantcard` |
| `board [ROOTS]` | web replay board | `python -m viewer serve` |
| `course gen\|validate\|render` | course truth | `heavy unity Tools/RoboBoat/generate.sh`, `validate.py`, `render.py` |
| `bench player` | Unity player real-time factor | `heavy unity` + `run_worker.sh` / `sweep_workers.sh` (dir: `paths.BENCH`) |
| `build player` | the headless Linux player | `heavy unity <editor> … -executeMethod CranePerformanceBuild.BuildLinuxWorker` (`crane/build.py`) |
| `cluster …` | research-cluster jobs | execs `~/research-cluster/bin/research` (or `$RESEARCH_CLUSTER/bin/research`) |
| `doctor` | machine checks and next commands | `crane/doctor.py` |
| `fit BUNDLE…` | astro bundle to profile (gate) and plant card (evidence); `--factorial SPEC` card variants | `crane/fit.py` over `Tools/Characterization/bundle.py`, `plantcard` |
| `field import SRC` | bundle, bag dir or .mcap to crane.run/1 (kind field) | `crane/field.py` |
| `run --astro-test TEST` | astro's tests at T0/T1, metrics named as astro's results | `crane/astrotest.py` over `Tools/Characterization/astro_tests.py` |
| `compare A B [--tiers]`, `compare --field BUNDLE RUNS` | variant A/B, tier gap, field vs sim | `crane/compare.py` |
| `setup`, `view`, `bench fast`, `run --tier T2` | not yet available | say what to use today and exit 2 |

## Pieces

- `crane/cli.py`: the parser and dispatch. Pass-through verbs (`sim`, `heavy`, `judge`, `card`, `board`,
  `course`, `cluster`) hand every argument after the verb to their tool; only `crane VERB --help` is crane's own.
- `crane/paths.py`: every path a verb execs, named once.
- `crane/tiers.py`: `normalise('dynamic') == 'T1'`; T0 abstract, T1 dynamic, T2 unity.
- `crane/run.py`: `--domain D [--tier T]` reads `config/domains/D.yaml` through `bench.fast.domain_tier`: the tier
  (default the first) names a fastsim config (from the repo root) and optional `overrides` merged over it. Missing
  scenarios are made by the config's generator, or the dir is named when there is none; `--generate` regenerates.
  `--config` takes a fastsim config directly. `--set a.b=v` overrides a key (the value is YAML).
- `crane/build.py`: after the build, `sourceCommit` and `sourceDirty` go into
  `Builds/CRANE-Worker/crane-build-manifest.json`; `--tarball` writes
  `Builds/crane-player-<commit12>-linux-x86_64.tar.zst` (`.tar.gz` without zstd), without debug symbols, and a
  `.sha256` in `sha256sum` format. `--skip-unity` stamps and packs an existing build.
- `config/sim.yaml` (the scene's default current, the player's required flag) and `config/bench.yaml` (sweep counts).
- `help.yaml` (repo root): the user-facing verbs in astro's help schema, so `astro crane VERB` runs them.

## Install

`./install.sh [--yes] [--dir D] [--components core,fast,gpu,view,field,setup] [--domains …] [--no-clone]
[--dry-run]` installs uv (to `~/.local/bin`), a Python 3.12 venv at `<repo>/.venv`, `pip install -e .[extras]`
and the link `~/.local/bin/crane`. It never uses sudo and prints each step. The extras are `fast` (numba), `gpu`
(jax CUDA 12), `view` (PySide6, pyqtgraph), `field` (mcap), `setup` (rich, questionary) and `all`. The editable
install also registers the `research_cluster.backends` entry points (fastsim, unity, theater-render).
