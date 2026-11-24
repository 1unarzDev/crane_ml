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

## The 6-DOF marine kernel (sub)

```bash
crane run --domain sub --tier T1 --engine batched --seeds 0:4096 --record 5 --out runs/sub-t1
python3 -m domains.sub.bench --E 1,64,1024,4096,16384 --out DIR
```

```python
from engine.kernels import marine6 as K
p = K.pack('domains/sub/profiles/generic_rov8.json', dt=0.02, current_n=(0.1, 0.0, 0.0))   # [P]
x = K.init(P, pos, rpy)                    # [E, S]: pos (NED) | quat b->n | nu (FRD) | thrust per slot
x = K.advance(x, u, P, 0.02)               # u [E, 6 + NT]: external body wrench, thruster force commands
f = K.allocate(tau, P)                     # least-norm, one uniform scale into the limits
```

`engine/kernels/marine6.py` is Fossen 2011 eq. 10.1 as the reference `marine6_ref.Simulator.step` has it:
rigid-body and added-mass Coriolis (Thm 3.2), linear and quadratic damping, restoring forces, a NED current
constant per env, thrusters with first-order lag and saturation, classical RK4 with stage quaternions
normalised and the result put in the w >= 0 hemisphere. Parameters come only from a vehicle profile
(`crane.underwater.vehicle_profile/v1`, every coefficient cited; `marine6_profile.load` checks it); the
thruster deadband is the profile's `deadbandFraction` through its thrust curve. Up to `NT = 8` thrusters;
the rest are zero columns. Substeps are fixed per card: `ceil(dt / (2 min tau))` RK4 steps per control
period (the lag's stability bound). `step`/`advance` run on numpy or jax.numpy (`xp`); `nb_step`/
`nb_advance`/`nb_allocate` are the numba scalars over one env's row. Per-env draws are counter-based
(`normal(key, n)`, splitmix64 and Box-Muller), so an env's draws never depend on the batch.
`marine6_vec.VecDynamics` is the sub platform's batched-plant interface over the same kernel.

A domain with its own batched episode gives its adapter `run_batched(...)` (engine/runs.py calls it) and
`engines`/`backends` (crane run refuses others in one line). The sub's mission (`domains/sub/mission.py`)
runs one compiled episode per env on numba; its records are crane.run/1 with frame `{world: ned, body: frd}`
and a 6-DOF `traj.ctraj`.

## Gates

| gate | check | where |
|---|---|---|
| 1 kernel exactness | every backend = `parity/cases.json` to 1e-12 (f64); dead time and lag; dt halving halves the error | `engine/tests/test_kernels.py` |
| 2 the scalar path | float64 batch = fastsim run for run on seeds 0–49: outcome, events, contacts, idle, boat.json, every track row; env i at E=1 = at E=4096; decoupled at E=1 = scalar | `engine/tests/test_episode.py` |
| 3 fast backends | ≥ 500 seeds: outcome agreement ≥ 0.98, TOST (90 %) time-to-finish ±2 %, dock error ±0.05 m | `engine/equivalence.py`, `crane bench fast --equivalence` |
| 4 fidelity | `fastsim/fidelity.py` scores with bootstrap CIs, batched T1 vs scalar T1 | `engine/fidelity.py` |
| 1 (marine6) | numpy, numba and JAX float64 = `marine6_ref.Simulator.step` to 1e-10 over a scripted wrench program (current, lag, saturation, turns past 180 deg, 1 and 2 substeps); env i at E=1 = at E=1024 | `engine/tests/test_marine6.py`, `domains/sub/tests` |

Results and the chosen backend and dtype: `bench/RESULTS.md`.
