# Fleet scenes v1: one scene/task spec for fastsim, CRANE and the mf levels

A scene is two JSON documents: `crane.fleet_scenario/v1` (the world and the fleet's start) and `crane.fleet_task/v1`
(alternative plans as task graphs, plus outcome rules). Nine procedural families generate them from a seed. The same
documents compile to a fastsim domain (seeded kinematic runs), to the files CRANE reads today (plus an `x_v1` block
for what it cannot read yet), and to an mf `TaskSpec` (L0 grid, L1 capplan truth). The handoff is
`Docs/Fleet/SCENES_v1.md`.

Run every command from `Tools/Fleet`, niced, one process at a time.

| file | what it does |
|---|---|
| `schema/scenario_v1.schema.json`, `schema/task_v1.schema.json` | JSON Schema (draft 2020-12) |
| `validate.py` | schema validation (a built-in subset validator) plus semantic checks |
| `common.py` | hashing, floor and payload tables, **`lower()`**: walls, doors, lintels, thresholds and obstacles as solid boxes (the one lowering both simulators use) |
| `catalog.py`, `catalog/units_v1.json` | the units scenes may use: capplan `train_v1` items and train-pool compositions, with their capability descriptor summary (frozen) |
| `families.py` | the nine generators, their train/held-out RULES and `in_split` |
| `splits.py`, `frozen/splits_v1.json` | the frozen split manifest (hashes) and the leakage checks L1 to L6 |
| `plan.py` | C-space lattice planner for rigid units (forward, reverse, strafe, rotate) |
| `fleetsim.py` | the fleet engine behind the fastsim domain (units, plants, executive, perception, traffic, outcome rules) |
| `fastsim_domain/` | `fleet_domain.py` (format adapter + DomainAdapter + plant plugin), `gen_scenarios.py`, `configs/*.yaml` (one per family/split/plan; `make_configs.py`) |
| `fastsim_run.py` | the same runs in-process (development and tests) |
| `crane_compile.py`, `crane_out/` | v1 to CRANE scenario + mission + jobs files, with a gap list per scene and plan |
| `consistency.py` | parity: fastsim and CRANE geometry, doors, zones, starts and payloads agree to 1 mm |
| `mf_adapter.py` | v1 to mf `TaskSpec`/`ConfigRef`; evaluates L0, L1, fastsim and an L2 (CRANE) request |
| `render.py`, `previews/` | top-down PNG previews (300 KB or less each) |
| `tests/test_scenes.py` | pytest |

## Commands

```bash
cd Tools/Fleet
# validate, generate, preview one scene
nice -n 15 python3 -m scenes.validate scenes/fastsim_domain/scenarios/*.json
nice -n 15 python3 -m scenes.render --family narrow_door --split heldout --seed 3 --out /tmp/nd.png

# fastsim: the official batch command (one YAML per family/split/plan; --generate writes the seed bundles)
cd scenes/fastsim_domain/configs
nice -n 15 python3 -m fastsim.batch --config narrow_door-train-single.yaml --seeds 1:6 --out /tmp/fs \
       --variant narrow_door-train-single --generate          # run from the crane root, or put it on PYTHONPATH
# fastsim: in-process (every plan of a family)
cd Tools/Fleet && nice -n 15 python3 -m scenes.fastsim_run --family wheel_failure --split heldout --seeds 1:4 --out /tmp/fs

# CRANE: compile one train + one held-out scene per family (scenarios, missions, jobs_scenes_v1.json, transfer_summary.json)
nice -n 15 python3 -m scenes.crane_compile --library
# then (CRANE integration agent, the Unity slot): Tools/Fleet/fleet.sh jobs Tools/Fleet/scenes/crane_out/jobs_scenes_v1.json

# parity and mf
nice -n 15 python3 -m scenes.consistency
nice -n 15 python3 -m scenes.mf_adapter --family long_payload_door --levels L0,L1,fastsim,L2

# splits: freeze once, check always
nice -n 15 python3 -m scenes.splits check
# tests
nice -n 15 python3 -m pytest -q scenes/tests -p no:cacheprovider
```

After editing a generator, regenerate the bundles (`rm -rf scenes/fastsim_domain/scenarios`). The fastsim format refuses a
stale bundle. A generator edit also breaks the frozen split (check L5): freeze a v2 manifest, and do not edit v1.

`fastsim.batch` refuses an uncommitted adapter tree unless `--allow-dirty` is given.
