# The batched engine against table B

```bash
bin/heavy sim python3 -m bench.fast --engine batched --tier T1 --backend numba,numpy --E 1,64,1024,4096,16384 --split --out DIR
bin/heavy sim python3 -m bench.fast --engine batched --tier T1 --backend jax:float32 --E 4096,16384 --min-seeds 65536 --split --out DIR
bin/heavy sim python3 -m bench.fast --engine batched --tier T1 --backend numba --fastmath --E 4096 --out DIR
bin/heavy sim python3 -m engine.equivalence --tier T1 --seeds 0:500 --candidates numba,numba-fastmath,jax-f64,jax-f32
bin/heavy sim python3 -m engine.equivalence --tier T1 --seeds 0:50 --reference scalar --track --candidates numpy,numba,jax-f64
bin/heavy sim python3 -m engine.fidelity --seeds 0:50
```

Measured on 2026-11-24 on the table-B host: i7-12700K (8 P + 4 E cores, 20 threads), RTX 4070 Ti SUPER
(driver 615, `jax[cuda12]` 0.12.1), the project venv (Python 3.12, numpy 2.5.5, numba 0.68.1, OpenBLAS
0.3.34). Raw reports: `bench/results/*/batched.json`, `equivalence-*.json`, `fidelity-*.json`. Every
episode in every row succeeded. Episodes/s is seeds over the wall time of the timed call; host setup
(scenario load and its identity check, world arrays, plans) and the first call (compile or cache load)
are listed apart.

## Boat T1 (table B: 4.33 episodes/s at W=1, 27.9 at W=8)

| backend | dtype | E | seeds | setup s | first call s | episodes/s | x B (W=1) | x B (W=8) |
|---|---|---:|---:|---:|---:|---:|---:|---:|
| numba, all cores | float64 | 1 | 1 | 0.01 | 0.17 | 182 | 42 | 6.5 |
| numba, all cores | float64 | 64 | 64 | 0.29 | 0.02 | 2,934 | 678 | 105 |
| numba, all cores | float64 | 1,024 | 1,024 | 1.42 | 0.31 | 3,374 | 779 | 121 |
| numba, all cores | float64 | 4,096 | 4,096 | 2.37 | 1.19 | **3,385** | **782** | **121** |
| numba, all cores | float64 | 16,384 | 16,384 | 8.55 | 4.92 | 3,373 | 779 | 121 |
| numba fastmath | float64 | 4,096 | 4,096 | 2.42 | 1.32 | 3,884 | 897 | 139 |
| jax, GPU | float32 | 4,096 | 16,384 | 8.90 | 8.98 | 3,736 | 863 | 134 |
| jax, GPU | float32 | 16,384 | 65,536 | 33.6 | 16.8 | **4,849** | **1,120** | **174** |
| jax, GPU | float32 | 32,768 | 80,000 | 41.7 | 32.5 | 3,667 | 847 | 131 |
| jax, GPU | float64 | 4,096 | 16,384 | | | 189 | 44 | 6.8 |
| numpy (reference) | float64 | 1,024 | 1,024 | 1.41 | 120 | 8.7 | 2.0 | 0.3 |

## Boat T0 (table B: 9.63 episodes/s at W=1, 55.9 at W=8)

| backend | dtype | E | seeds | setup s | first call s | episodes/s | x B (W=1) | x B (W=8) |
|---|---|---:|---:|---:|---:|---:|---:|---:|
| numba, all cores | float64 | 64 | 64 | 0.29 | 0.01 | 5,485 | 570 | 98 |
| numba, all cores | float64 | 4,096 | 4,096 | 2.42 | 0.58 | **6,961** | **723** | **125** |
| numba, all cores | float64 | 16,384 | 16,384 | 8.59 | 2.38 | 7,011 | 728 | 125 |
| numba fastmath | float64 | 4,096 | 4,096 | 2.50 | 0.75 | 7,633 | 793 | 137 |
| jax, GPU | float32 | 4,096 | 16,384 | 9.45 | 8.14 | 4,665 | 484 | 83 |
| jax, GPU | float32 | 16,384 | 65,536 | 35.8 | 16.4 | **5,510** | **572** | **99** |
| jax, GPU | float64 | 4,096 | 16,384 | | | 212 | 22 | 3.8 |
| numpy (reference) | float64 | 1,024 | 1,024 | 1.40 | 98 | 10.4 | 1.1 | 0.2 |

JAX at E=1 and E=64 (3–200 episodes/s) and numba at E=1 are in the raw reports. Decoupled mode
(the boat's own scalar `Mission` on the batched numpy plant and tracker): see the last section.

## Against the targets

| target | measured | verdict |
|---|---|---|
| CPU, batched controllers, E ≥ 4096: ≥ 100× B | T1 782× W=1 / 121× W=8 (fastmath 897× / 139×); T0 723× / 125× | met against both baselines |
| GPU, E ≥ 4096: ≥ 1000× B | T1 float32 at E=16,384: 1,120× W=1 / 174× W=8; at E=4096: 863× / 134× | met against W=1 at E=16,384 only; not against W=8 |
| T0 ≥ 100× | 723× W=1 / 125× W=8 (CPU), 572× / 99× (GPU) | met (GPU just under against W=8) |

All of it at equal agreement: gate 2 (exact) for numba float64 and JAX float64, gate 3 for numba
fastmath and JAX float32.

## Chosen design and dtype

- **CPU: numba, float64, exact build** (`crane run --engine batched`'s default). Each env's whole
  episode is one compiled call, envs in `prange`. It is the scalar path bit for bit (gate 2) and has no
  lockstep waste. The fastmath build is 15 % faster and passes gate 3 (every record identical on 500
  seeds; raw state differs by up to 3e-11 m); use it with `--fastmath` when the exact identity does not
  matter.
- **GPU: JAX, float32, E = 16,384 slots refilled from a seed queue.** float64 is compute-bound on this
  consumer GPU (FP64 at 1/64 rate: 189 episodes/s). float32 passes gate 3: outcome agreement 1.000 on
  500 seeds, 461/500 records identical (T1), time-to-finish within 7e-6 relative and dock error within
  8e-4 m at 90 %.
- **numpy** stays the float64 reference; it is slow by design.
- On this host numba on the CPU and JAX on the GPU are within 1.5× of each other at T1; the 3090
  rental boxes (strong FP32) favour JAX, a many-core box numba.

## Where the time goes

numba, one thread, T1 (3.25 ms per episode, 7,749 steps; T0: 1.63 ms, 2,956 steps):

| part | T1 ms | T0 ms |
|---|---:|---:|
| plant (velocity loop, allocation, kernel) | 1.15 | 0.12 |
| tracker and progress check | 0.64 | 0.23 |
| perception (cone, occlusion) | 0.81 | 0.78 |
| contacts (footprint clearance) | 0.21 | 0.21 |
| track row (rounding, scoring) | 0.04 | 0.04 |
| decision and other | 0.39 | 0.25 |

Perception is measured as the full run less the same run with nothing left to perceive; the other parts
are per-call microbenchmarks times their calls. 20 threads give about 11× one thread (8 P-cores with SMT and
4 E-cores).

JAX float32 at E=16,384: a physics step is 9 kernels (0.17 ms per standalone call, dispatch included);
a track + tick step is 113 kernels, 1.3 ms. A tick period (one tick step and four physics steps) is about
2 ms inside the scan.

## Next bottlenecks

- **GPU: the tick step.** 113 kernels at 1.3 ms per tick at E=16,384 bound the GPU. Next: fuse the
  tick (perception, FSM, events) into fewer kernels (a Pallas kernel, or a per-env CUDA kernel from the
  numba code with numba-cuda), and refill more often (a chunk is 250 steps; finished slots idle until
  the chunk ends). E=32,768 needs more than 80,000 seeds to reach a steady state.
- **CPU: the plant and tracker.** About 55 % of a T1 episode is per-step trig, `expm1`, libm `pow` and
  CPython's `hypot` that exactness keeps; the fastmath build trims 15 %.
- **Host setup** is 2.4 s per 4,096 envs, mostly the course-truth identity check (canonical JSON per
  file), in worker processes. For numba at E=4096 that is twice the episodes' own time: a run that
  counts setup does about 1,150 episodes/s at T1. Caching verified truths by content hash would remove
  most of it.
- **The cluster** still pays its per-job cost (table B: ~0.55 runs/s observed). `run_batch(seeds)`
  turns one job into thousands of episodes; that is where these numbers reach the cluster.

## Gates

| gate | result |
|---|---|
| 1 kernel exactness | numpy, numba and JAX float64 within 1e-12 of `parity/cases.json` (numpy and numba 0.0, JAX 3.6e-15); dead time, lag, deadband and offset bit for bit (numpy, numba); dt halving ratios 2.02, 2.01, 2.005 (first order) |
| 2 the scalar path | seeds 0–49, T1 and T0: numpy, numba and JAX float64 equal fastsim run for run in outcome, sim time, flags, contacts, idle, clearance, every event, boat.json and every track row; env i at E=1 = at E=4096 bit for bit (numpy, numba; JAX: records equal, state within 3.6e-14 m, because XLA compiles each batch shape on its own); decoupled mode at E=1 writes the scalar run's files byte for byte |
| 3 fast backends (500 seeds) | T1 and T0: numba, numba fastmath, JAX float64: agreement 1.000, 500/500 records identical; JAX float32: agreement 1.000, 461 (T1) / 479 (T0) records identical, TOST passes with CIs 1000× inside the margins |
| 4 fidelity | `fidelity.score` of boat T0 on the T1 twin's goal segments, seeds 0–49: scalar T1 4.532 m [4.137, 4.964], batched T1 (numba) the same, paired difference 0.0 on 50/50 seeds; JAX float32 −6e-5 m. No T2 pairs exist on this host (`~/.cache/crane-fastsim-pairs`): they need regenerating with `crane run --tier T2` |

## Decoupled mode (scalar controllers)

The boat's own Python `Mission` per env on the batched plant and tracker (`engine/decoupled.py`): 0.15
episodes/s at E=1 and 2.5 at E=64 at T1, slower than scalar fastsim (4.33). It runs on the numpy
reference path (float64, exact: E=1 writes the scalar run's files byte for byte), so it is the
compatibility path for domains without a batched controller, not a fast one. Putting its plant and
tracker on numba kernels per step is the step that makes it pay.

## Sub T1: the 6-DOF marine kernel and the pool-gates mission

```bash
bin/heavy sim python3 -m domains.sub.bench --E 1,64,1024,4096,16384 --mission 1,64,1024,4096,16384 --out DIR
```

Measured on 2026-11-24 on the same host (i7-12700K, 20 threads; RTX 4070 Ti SUPER, `jax[cuda12]` 0.12.1;
Python 3.12, numpy 2.5.5, numba 0.68.1). Raw report: `bench/results/sub-T1/sub.json`. A control period is
dt = 0.02 s: one RK4 step of the 6-DOF plant (the generic eight-thruster profile, lag 0.02 s: one substep).
Baseline: the sub platform's numpy twin (`VecDynamics.step`), 0.62 M env-periods/s at E=1024 and 0.77 M at
E=4096 on this host.

| kernel | E | env-periods/s | x numpy twin |
|---|---:|---:|---:|
| numba, all cores | 1 | 1.18 M | |
| numba, all cores | 1,024 | 14.7 M | 24 |
| numba, all cores | 4,096 | **19.4 M** | **25** |
| numba, all cores | 16,384 | 17.4 M | |
| jax float64, GPU | 4,096 | 20.6 M | 27 |
| jax float64, GPU | 16,384 | 28.3 M | |
| jax float32, GPU | 4,096 | 75.0 M | 98 |
| jax float32, GPU | 16,384 | **99.4 M** | |
| numpy (reference) | 4,096 | 0.39 M | 0.5 |

The mission (numba, one compiled episode per env, every core; the first call, compile or cache load, apart):

| E | episodes/s | env-periods/s | host setup s | succeeded |
|---:|---:|---:|---:|---:|
| 1 | 527 | 1.29 M | 0.00 | 1/1 |
| 64 | 4,010 | 11.1 M | 0.00 | 64/64 |
| 1,024 | 4,462 | 12.4 M | 0.29 | 1,024/1,024 |
| 4,096 | 4,359 | 12.0 M | 0.83 | 4,091/4,096 |
| 16,384 | **4,746** | 13.1 M | 3.12 | 16,355/16,384 |

An episode is about 55 s simulated (2,800 periods: three gates and a 5 s hold). The autopilot, allocation,
deadband and mission bookkeeping cost less than the plant (13 M periods/s with them against 17-19 M for the
kernel alone). JAX float32 is 5x numba on this GPU for the kernel alone; the mission has no JAX twin yet.

| gate | result |
|---|---|
| 1 kernel exactness | numpy, numba and JAX float64 (GPU; the test pins the CPU device) against `marine6_ref.Simulator.step` over 600 periods (NED current, lag, saturation, turns past 180 deg): max state error 2.7e-15 / 1.8e-15 / 7.1e-15 at one substep, 3.6e-15 / 3.6e-15 / 7.1e-15 at two; allocation within 1e-12 of the reference |
| batch invariance | kernel: env i at E=1 = at E=1024 bit for bit (numpy), numba rows within 1e-12; mission: env i at E=1 = at E=1024 bit for bit (outcome, events, numbers, final state) under a per-env current and thrust noise |

## Rented RTX 3090 (2026-11-25)
Commit 01a78e11. Box vast-jax (RTX 3090, 26 cgroup cores, python:3.12-slim + jax[cuda12] + numba). Boat T1,
`--course-pool 5`. Job wall 338 s.

| backend | E | episodes/s | this host (4070 Ti SUPER, 20 cores) |
|---|---|---|---|
| jax float32 GPU | 16,384 | 4,596 | 4,849 |
| jax float32 GPU | 4,096 | 3,282 | – |
| numba CPU (26 cores) | 4,096 | 3,710 | 3,385 (20 cores) |

On the 3090 the GPU path barely beats numba on the same box, and it is no faster than the 4070 Ti SUPER, so the GPU
is underused. The time goes to host-side prep, transfers and many small per-tick kernels (113 per tick, see above),
not GPU compute. The next step toward the GPU target is a profile of one E=16,384 tick (jax.profiler or nsys),
followed by fusing the tick step and refilling more often. Until then, numba on CPU cores is as fast and cheaper.
