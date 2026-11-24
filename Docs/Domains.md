# Domains

A domain is a vehicle and its task in the fast tiers: a fastsim adapter, a controller, a plant per tier and a
config per tier. The boat is the worked example.

```bash
python3 -m fastsim.batch --config domains/boat/fastsim_t1.yaml --seeds 0:5 --out runs/boat-t1
python3 -m bench.fast --domain boat --tier T1 --seeds 0:50 --workers 1,8 --out runs/bench-boat-t1
```

Each run directory holds `run.json`, `track.jsonl`, `run.ctraj`, `sim_events.jsonl` and the domain's own
files (the boat writes `boat.json`).

## The pieces

| Piece | What it is | Boat |
|---|---|---|
| Domain file | `config/domains/<name>.yaml`: `tiers: {T0: {config, overrides?}, T1: ...}`, one fastsim YAML per tier, paths from the repo root | `config/domains/boat.yaml` |
| fastsim YAML | the core's config: adapter, scenarios, limits, perception, `dt`, `vehicle`, `fidelity` (keys in `fastsim/config/default.yaml`) | `domains/boat/fastsim_t0.yaml`, `fastsim_t1.yaml` |
| Adapter | a `fastsim.controller.DomainAdapter`, built once per worker from `adapter: {ref, options}` | `domains/boat/adapter.py:BoatAdapter` |
| Controller | a `fastsim.controller.Controller`; `tick(obs)` at `tick_hz` returns `None` or an outcome | `Mission` in the same file |
| Plant | a `fastsim.plant.PlantModel` named by `vehicle` (`model:` a built-in, or `ref:`) | T0 `first_order`; T1 `domains/boat/plant.py:ProfileBoat` |
| Scenarios | seed to file through `scenarios: {dir, pattern, format, generate}` | `crane_truth` files in `domains/boat/courses` |

A tier's optional `overrides` are merged over its fastsim YAML (the toy uses them to write its scenarios under
`~/.cache/crane`).

## Adapter and controller

The core runs the loop. Every `dt` it steps the tracker and the plant. Every tick it hands the controller an
`Observation`: the pose in the start frame, the perceived tracks, the sensor's reach. The controller acts only
through `ctx.navigator`:
- `send(route, precise=…)` sends a goal;
- `hold()` stops;
- `speed_limit(v)` caps the speed.

It reports events through `ctx.emit`. The adapter supplies:
- `identity()`: hashed into every run identity (the boat hashes its tier and plant profile);
- `controller(ctx)`: one per run;
- `finalize(run_dir, result)`: domain files and scores;
- `metrics(run_dir)`: numbers for the cluster and `fastsim.pairs`.

The boat's `Mission` plans from the course truth (`domains/boat/mission.py`, world frame, mapped to the start
frame):

1. **course**: through each entry gate and nav-channel gate along its normal, through the speed gate, a full
   counter-clockwise turn round its mark, then back through the gate.
2. **fairway**: past the end of the chosen slip's pier, then to a standoff square to the slip's mouth.
3. **slip**: a precise goal at the slip's centre, facing in.

Legs detour round any element they would pass within `CLEARANCE_M`. An aborted goal is resent once with what
is left of it; a second abort fails the run.

`boat.json` scores a run:
- `success`: docked, every gate pass made, and within 0.5 m of the slip centre;
- `time_s`;
- `gates_passed` / `gates_total`, in order and in direction;
- `mark_sweep_deg`;
- `contacts`;
- `dock_error_m` and `dock_yaw_error_rad`;
- `gate_buoys_seen`.

## Plants per tier

| Tier | Boat plant | Parameters |
|---|---|---|
| T0 abstract | `first_order`: per-axis lag `tau_s`, then the `limits` rate limits, at `dt` 0.05 | `tau_s` = M_eff / J of `Tools/Characterization/profiles/characterized.json` at a cruise point (comment in `fastsim_t0.yaml`) |
| T1 dynamic | `ProfileBoat`: velocity smoother, feedforward + P velocity loop, pseudo-inverse allocation to PWM, then `ProfilePlant.step_pwm` ticks (the C# `MarinePlanarModel` twin), at `dt` 0.02 = one tick | the profile file named by `vehicle.profile` (read only) |

`domains/boat/tests` hold the T1 plant to `Tools/Characterization/parity/cases.json` (to 1e-12, through
its own PWM path). They also check that T0 and T1 episodes repeat byte for byte, and that the mission passes
every gate and docks at T1 on every committed course.

## Scenarios

The boat's scenarios are `roboboat_2026-s{seed}.json` course truths. Seeds 0–4 are committed. For other seeds,
`--generate` (or `bench.fast`) runs `domains/boat/coursegen.py`, which builds `Tools/RoboBoat/cli` once into
`~/.cache/crane/coursegen/` and writes only the missing seeds. It needs a .NET SDK: 10.x, or a newer one, which
builds it for its own framework. The format's `identity_check` refuses an edited truth.

## Adding a domain

1. Write `domains/<name>/adapter.py` with a `DomainAdapter` and a `Controller`. Add a plant module if no
   built-in plant fits the tier.
2. Write one fastsim YAML per tier next to it, and `config/domains/<name>.yaml` naming them.
3. Add a few deep tests in `domains/<name>/tests`: determinism, plant exactness against its reference, and
   one mission outcome.
4. Run `python3 -m bench.fast --domain <name> --tier T0 --seeds 0:50 --workers 1,8 --out DIR`.
