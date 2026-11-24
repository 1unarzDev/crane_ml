# Engine

The batched fast-tier engine runs many episodes of a fastsim domain at once, on numpy, numba (every CPU
core) or JAX (the GPU), with the same results as the scalar fastsim path.

```bash
crane run --domain boat --tier T1 --engine batched --backend numba --seeds 0:4096 --out runs/boat-t1
crane bench fast --engine batched --domain boat --tier T1 --backend numba,jax --E 1,64,1024,4096,16384 --out DIR
crane bench fast --equivalence --tier T1 --seeds 0:500
```

```python
from engine.episode import Setup, run
setup = Setup(cfg, base, adapter, seeds)          # host: scenarios, world arrays, plans (worker processes)
out = run(setup, 'numba')                         # or 'numpy', or 'jax' with dtype='float32'|'float64'
out.summary(e), out.events(e), out.numbers(e), out.metrics(e)
out.write(e, run_dir)                             # a crane.run/1 record, only when asked (record=k: traj too)
```

**Which backend.** numba float64 on the CPU (the default: bit for bit the scalar path, about 3,400 boat T1
episodes/s on this 20-thread host); JAX float32 on the GPU at E = 16,384 slots (about 4,800/s; float64 is
compute-bound on consumer GPUs). Numbers and gates: `bench/RESULTS.md`.

**On the cluster.** `crane_backends.fastsim` `FastsimBackend.run_batch(seeds)` and `ClusterBackend.run_batch(seeds)`
return `{seed: metrics}` with the keys `metrics(run(seed))` has, from one batched call, and write
`<out>/batch_<first>_<last>.jsonl`. The job's `engine: {backend, dtype}` picks the backend. It needs the
`engine` directory beside the sparse checkout's, and numba (or `jax[cuda12]` for the GPU).

## Design

| piece | module | what it is |
|---|---|---|
| kernel | `engine/kernels/profile.py`, `first_order.py` | one discrete map per plant: `step(x [E,S], u, p [E,P], dt)` as arrays, `nb_step` as a numba scalar |
| vehicle | `engine/vehicles.py` | a kernel plus what turns the tracker's body-velocity target into its input (the boat's FF + P loop and allocation) |
| tracker | `engine/tracker.py` | fastsim's GoalTracker and progress checker on route buffers `[E, R, 3]` |
| perception, contacts | `engine/geometry.py` | the cone camera with occlusion over envs × targets × blockers; footprint clearance |
| controller | `engine/boat.py` | the boat mission as a batched FSM (ticks at `tick_hz`) and its scoring accumulators |
| episode | `engine/episode.py`, `jaxrun.py`, `nbepisode.py` | fastsim/run.py's loop: lockstep arrays (numpy; JAX on a refilled slot pool) or one compiled episode per env (numba) |
| decoupled mode | `engine/decoupled.py` | any fastsim domain's scalar controller on the batched plant and tracker |
| exact numerics | `engine/exact.py`, `nbexact.py` | CPython's `hypot`, Python's `round`, libm `pow`, OpenBLAS's FMA order, on arrays |

Per step, as fastsim/run.py: at `track_hz` the track row, scoring and clearance; at `tick_hz` perception
and the decision; then idle accounting, tracker, vehicle and progress checker. Every env has its own
step index; numpy and JAX run envs in lockstep (a tick period's steps unrolled, so the device never
branches), numba runs each env's whole episode in one compiled call in parallel.

**Exactness.** The float64 numpy batch reproduces the scalar path bit for bit: the same expressions in the
same order, with the scalar path's own roundings. CPython's `math.hypot`/`math.dist` are not libm's;
`x ** 2` and `x ** 0.5` are libm `pow`, not `x * x` or `sqrt`; `np.linalg.pinv(...) @ tau` is OpenBLAS
dgemv with FMA in its own order; `round(x, 4)` is a decimal rounding. `engine/exact.py` has each of them
for arrays. The BLAS order is this host's (`engine/tests` checks it); a host whose BLAS differs makes the
scalar path itself differ, and the test says so.

**Batch invariance.** An env reads only its own rows. Per-env draws (`Setup(..., randomize={'current_sd':
0.05})`) come from the env's own stream, `fastsim.identity.derive_seed(seed, 'engine', name)`. Substeps are
fixed per card (dt / the profile's tick). The run identity hashes backend, dtype and the substep policy.

## Adding a plant

1. Write `engine/kernels/<plant>.py`: `pack(...) -> p [P]` (constants computed as the reference computes
   them), `init(p [E,P]) -> x [E,S]`, `step(x, u, p, dt, xp)` with every branch a `where`, and `nb_step(x, u,
   p, dt)` over one row (numba, `inline='always'`). Keep the reference's order of operations.
2. Add a `Vehicle` in `engine/vehicles.py`: `init`, `pose`, `drive` (target to kernel input) and its
   `nb_drive` branch; `vehicle()` maps the fastsim vehicle spec to it.
3. Add a kernel test in `engine/tests/test_kernels.py`: every backend against the plant's reference
   (parity cases or its simulator) to 1e-12 in float64, and the dt convergence order.
4. A domain without a batched controller runs in decoupled mode; a batched one adds a module like
   `engine/boat.py` (plans as arrays, an FSM `tick`, numba twin) and an entry in `episode.CONTROLLERS`.

## The aerial kernel (drones)

```bash
python3 -m domains.drones.braking --card x500 --v0 4,8,12,16 --seeds 0:10000      # stops, a table, stops/s
crane study run examples/studies/braking.yaml                                       # 10,240 stops as study rows
crane run --domain drones --tier T1 --seeds 0:4                                     # fly to a waypoint and stop
```

```python
from domains.drones import braking, card
from engine.kernels import aerial as A, aerial_control as C
p = A.pack(card.vehicle('x500'))                    # [P] from a plant card; pack(v, rotors=6) pads a quad
x = A.init(p[None], pos=(0, 0, 10), rotor_speed=A.hover_speed(p))
x = A.step(x, u, p[None], 0.004, wind)              # u [E, R] rotor targets (rad/s); numpy or jax.numpy (xp=)
A.nb_step(x[e], u[e], p, 0.004, wind[e])            # numba, one row in place
m = braking.stops('x500', 8.0, range(10000), variant={'mass': 2.7})   # {metric: [E]}
```

| piece | module | what it is |
|---|---|---|
| reference | `domains/drones/model.py` | the scalar rigid-body multirotor (RK4, exact motor lag per stage, rotor/body drag, battery, ground effect) |
| card | `domains/drones/card.py`, `profiles/*.json` | cited profiles (schema crane-aerial-vehicle-profile-v1): mass, inertia, kf, km, tau up/down, rotor positions; `override` makes a card variant |
| kernel | `engine/kernels/aerial.py` | `step(x [E, S], u [E, R], p [E, P], dt, wind)`; `nb_step` |
| controller | `engine/kernels/aerial_control.py` | position, velocity, SO(3) attitude, rate, pseudo-inverse allocation with a collective cut; gains from `config/domains/drones.yaml` `control` |
| braking | `domains/drones/braking.py` | v0 along a seeded heading, a stop command, metrics; numpy lockstep or numba per env in `prange`; a study task |
| T1 task | `domains/drones/{adapter,plant,waypoints}.py` | fastsim's tracker targets through the controller: fly to a waypoint, stop, hold |

**Gates.** Gate 1: numpy, numba and JAX float64 equal `model.step` to 1e-10 over 1,200 scripted steps in four
cases, so every kernel term is non-zero in at least one: x500 with ground effect and contact; crazyflie with its
battery; the zoh_first order; an x500 variant with rotor inertia and quadratic body drag, started at 10 m/s and
4 rad/s (gyroscopic torque, rotor angular momentum). Largest error 6.9e-13. A padded quad is the same map bit for
bit; with a held command the error falls about 16x per dt halving (RK4). Batch invariance: a stop alone equals the
same stop in a batch bit for bit, and numba equals numpy bit for bit (`engine/tests/test_aerial.py`). Gates 2–4
do not apply yet: the drones have no entry in `episode.CONTROLLERS`, so `crane run --domain drones` runs on
fastsim only, and braking is batch-only with no scalar twin to compare against.

**Bench** (2026-11-24, i7-12700K, 20 threads, numba 0.68.1; a stop is 8 s simulated at dt 4 ms, 2,000 steps):
numba on all cores 6,200 stops/s (10^4 stops in 1.6 s), one thread 820 stops/s; numpy (the reference) 26 stops/s
at E = 64. `examples/studies/braking.yaml` (10,240 stops, one unit per stop) runs in 3.9 s with one worker and 2.6 s
with four, cache and rows included.

**Limits.** The velocity loop is P only: a steady wind leaves a speed error, so about 0.2 % of stops at
wind_sd_mps 0.5 never fall below `stopped_mps` (NOT_STOPPED). The T1 task holds altitude and flies the planar
tracker's body velocity.

## Gates

| gate | check | where |
|---|---|---|
| 1 kernel exactness | every backend = `parity/cases.json` to 1e-12 (f64); dead time and lag; dt halving halves the error | `engine/tests/test_kernels.py` |
| 2 the scalar path | float64 batch = fastsim run for run on seeds 0–49: outcome, events, contacts, idle, boat.json, every track row; env i at E=1 = at E=4096; decoupled at E=1 = scalar | `engine/tests/test_episode.py` |
| 3 fast backends | ≥ 500 seeds: outcome agreement ≥ 0.98, TOST (90 %) time-to-finish ±2 %, dock error ±0.05 m | `engine/equivalence.py`, `crane bench fast --equivalence` |
| 4 fidelity | `fastsim/fidelity.py` scores with bootstrap CIs, batched T1 vs scalar T1 | `engine/fidelity.py` |

Results and the chosen backend and dtype: `bench/RESULTS.md`.
