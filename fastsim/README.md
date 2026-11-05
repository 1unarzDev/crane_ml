# fastsim: crane's domain-agnostic fast-sim core

This core runs a domain's real decision logic on seeded scenarios, many runs per second on any
CPU. It uses a kinematic vehicle, truth-in-cone perception and no planner. `fidelity` in
`config/default.yaml` says what it does not model, and every run records that note.

| module | role |
|---|---|
| `scenario.py` | scenarios through a format adapter (`crane_truth`, `json`, plugins); seed to file |
| `world.py` | the start (map) frame, element arrays, footprint contacts and their causes |
| `plant.py` | limits, goals, the stand-in goal tracker; plants `kinematic`, `first_order` (plant card) |
| `perception.py` | `cone` truth-label camera with occlusion; hold-over tracker |
| `controller.py` | the domain interface: `Navigator`, `Observation`, `RunContext`, `Controller`, `DomainAdapter` |
| `run.py` | one run on a fixed sim clock, then its run directory (`run.json` with identity hashes) |
| `batch.py` | seeds across spawned workers: `python3 -m fastsim.batch --config DOMAIN.yaml --seeds 1:21 --out OUT` |
| `pairs.py` | twin-pair archive (a fast run and another simulator's run of one scenario) |
| `fidelity.py` | goal-segment replay against a twin, prequential scoring, refit into new epochs |
| `identity.py` | canonical hashes, per-stream seeds, git state |

A domain plugs in through one YAML (`adapter`, `scenarios`, `limits` or the adapter's
`limits()`, `vehicle`, `perception`, `tracker`). See `tests/toy.yaml` and `tests/toy.py` for
the smallest complete domain; a domain's adapter lives in that domain's own repo. The cluster runs
this core as backend `fastsim` (`crane_backends/fastsim.py`).

Tests (from the crane root): `python3 -m pytest fastsim/tests`.
