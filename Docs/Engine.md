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

## Gates

| gate | check | where |
|---|---|---|
| 1 kernel exactness | every backend = `parity/cases.json` to 1e-12 (f64); dead time and lag; dt halving halves the error | `engine/tests/test_kernels.py` |
| 2 the scalar path | float64 batch = fastsim run for run on seeds 0–49: outcome, events, contacts, idle, boat.json, every track row; env i at E=1 = at E=4096; decoupled at E=1 = scalar | `engine/tests/test_episode.py` |
| 3 fast backends | ≥ 500 seeds: outcome agreement ≥ 0.98, TOST (90 %) time-to-finish ±2 %, dock error ±0.05 m | `engine/equivalence.py`, `crane bench fast --equivalence` |
| 4 fidelity | `fastsim/fidelity.py` scores with bootstrap CIs, batched T1 vs scalar T1 | `engine/fidelity.py` |

Results and the chosen backend and dtype: `bench/RESULTS.md`.

## The modular plant

```bash
crane run --domain modular --replay timelines.npz --seeds 0:24 --card example#plated --set faults.p_healthy=0.25
crane run --domain modular --robot mecanum-pair --env route:random --seeds 0:256
python3 -m domains.modular.bench --E 1024,4096 --backend numba,jax:float32 --seconds 40
```

```python
from domains.modular import assembly, profile
from engine.kernels import mecanum as K, mecanum_run as R
b = assembly.build('mecanum-pair', profile.load('example#plated'))     # wheels, body, footprint, static loads
p = K.stack([K.pack(b['wheels'], b['body'], {'mode': [0, 1, 0, 0, 0, 0, 0, 3], 'gain': [1, .5] + [1] * 6})])
out = R.replay(p, timelines, which=None, delay=[2], backend='numba')   # timelines [M, T, 3] at 50 Hz
out['duty'], out['counts'], out['gz'], out['ax'], out['ay']            # [E, T, W] / [E, T]
plant = R.Ticks(p, ticks=T); x = plant.tick(u)                         # closed loop, one 50 Hz tick
```

**What it is.** One rigid assembly of modules per env (mecanum, omni and conventional wheels, ball casters; each
env its own, wheels padded to W) at a 5 ms plant step under a body-twist command held per 50 Hz tick. The physics
is the modular truth model's planar dynamics: the firmware's ideal rolling IK (clipped to phi_max), massless wheels
whose speed balances motor and ground torque (bisection, then Illinois per wheel), the tanh slip contact (friction
ellipse; mecanum rollers with bearing drag and a friction circle), rolling resistance, a finite-difference
Jacobian and one linearly-implicit Euler step, the exact SE(2) pose update. Not modelled: planar compliance between
modules, load transfer (normal loads are the rigid assembly's static ones), swivel casters, payload slip,
intermittent actuators, wheel and drivetrain inertia, a PI loop's integral term.

**Inputs.** Geometry from an assembly file (`domains/modular/assemblies`: module types, wheels, modules, IMU; the
footprint is each module's rectangle and their hull). Every physical number from a plant profile
(`domains/modular/profiles`, `crane.modular.profile/1`): one card per plate state as `variants` (REF#NAME), theta
overrides as `scale.*` multipliers or any key (`--set card.KEY=V`), DR ranges in `dr` (`--set dr=true` draws per
seed). `example.json` is a generic card, not a measured robot.

**Faults** per wheel from an onset time: weak(g) (stall torque and no-load speed times g: a duty scale), coast
(back-drive only), brake (plus the shorted winding's damping), locked. From the timelines file (`mode`, `gain`,
`onset` [M, W]) or a prior (`--set faults.p_healthy=0.25 faults.p_double=0.2 faults.modes.weak=2 ...`); the same
seed draws the same faults and DR on every timeline.

**Channels** at the tick rate, written per env through `crane.data.recorder.Store` (obs T x C, names, hz, labels):
`cmd` (the logged command), `duty` (the firmware's pre-fault duty), `counts` (encoder counts, the profile's cpr),
`speed`, `imu` (gz, ax, ay at the IMU point), `pose`, `vel`; the domain records `cmd duty counts imu` at 50 Hz
float32 (`--set record.obs.channels=[...]`). float16 loses encoder counts above 2048.

**Gate 1** (`engine/tests/test_mecanum.py`): numpy and numba equal the reference port `domains/modular/truth.py`
bit for bit, and JAX float64 within 1e-8, on a scripted program over three assemblies with every fault mode, an
onset mid-run and command latency; env i at E = 1 equals env i in a mixed batch (numpy, numba) and the closed loop
equals the replay; the plant step converges at first order. The port follows the truth model's expressions in
order; set `CRANE_MODULAR_TRUTH` to the directory holding its `capplan` package and the same test checks the port
against `simulate_assembly` itself: static loads within 5e-14 N, pose within 8e-12 m, twist within 3e-10, wheel
speed within 1.4e-9 rad/s (the root's tolerance), on ok, weak, dead-free and dead-locked wheels. JAX float32 is
not exact (the stiff slip curve and the 1e-5 Jacobian step): on the gate programs pose within 8e-5 m, twist within
1e-3, wheel speed within 6e-3 rad/s; use float64 where the channels are compared to logs.

**Throughput** (replay, 40 s timelines at 50 Hz, i7-12700K 20 threads, numba float64, host shared with other
jobs at load ~12, so a lower bound): mecanum-1 (4 wheels) 16 trials/s at E = 1, 179 at E = 256, 177 at 1,024,
196 at 4,096 (1.6 M plant steps/s); mecanum-pair (8 wheels) 93 at E = 1,024. A 900 timelines x 24 envs rung
(21,600 trials) is about 2 min on this host. `python3 -m domains.modular.bench` reproduces it; JAX on the GPU is
not yet measured.
