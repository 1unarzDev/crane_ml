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
out.summary(e), out.events(e), out.numbers(e), out.metrics(e)   # metrics: run.* plus the domain's own names
out.write(e, run_dir)                             # a crane.run/1 record, only when asked (record=k: traj too)
```

**Without numba.** The numpy backend imports and runs with numpy alone (`engine/jit.py` stands in for numba's
decorators); `run(setup, 'numba')` then fails with one line naming the fix, and the engine tests skip only their
numba and jax cases.

**Metrics.** `out.metrics(e)` is `run.*` plus what the domain adapter's `metrics(run_dir)` would give, under the
domain's own names: the adapter's `metrics_from(numbers)` when it has one, else its controller module's
`metrics(numbers)` (`engine/boat.py`). `crane study` cells on `engine: batched` ([Study.md](Study.md)) and
`crane.env` ([Env.md](Env.md)) build on the same pieces.

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
for arrays. The BLAS order is OpenBLAS's (`engine/tests` checks it on this host). On a numpy built against
another BLAS (a distro's reference CBLAS) the scalar path itself rounds differently: the BLAS-order and decoupled
E=1 tests skip with the fix (`pip install numpy`, whose wheel bundles OpenBLAS), and batched rows match scalar
ones only up to that rounding.

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
   `engine/boat.py` (plans as arrays, an FSM `tick`, numba twin, `metrics(numbers)` naming its metrics, and
   `goals(plans)` for crane.env) and an entry in `episode.CONTROLLERS`.

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

### The sub platform's old paths (shims)

The sub platform branch keeps its old import paths as shims onto these modules once it has merged main
(`engine/`). Each module shim makes the old module *be* the new one (`sys.modules`), so every name re-exports:
`G`, `RHO_FRESH`, `RHO_SEA`, `ALLOC_RCOND`, `Thruster`, `VehicleParams`, `State`, `Simulator`, `allocate`,
`allocation_rank`, `allocation_matrix`, `euler_to_quat`, `quat_to_rot`, `quat_to_euler`, ... from
`marine6_ref`, and `load`, `to_params`, `to_thrusters`, ... from `marine6_profile`. The crane root goes at the
front of `sys.path` (the shim's `parents[2]`), after `crane_sub/plant/_underwater.py` has put `Tools/Underwater`
there for `import fossen6dof` and `import vehicle_profile`.

`Tools/Underwater/fossen6dof.py`, the whole file:

```python
"""Shim: the 6-DOF Fossen reference lives on main as engine/kernels/marine6_ref.py (the same code). This module
is that module, so `import fossen6dof` (Tools/Underwater on sys.path) and every caller keep working."""
import sys
from pathlib import Path

_root = str(Path(__file__).resolve().parents[2])
if _root not in sys.path:
    sys.path.insert(0, _root)
from engine.kernels import marine6_ref as _ref  # noqa: E402

sys.modules[__name__] = _ref
```

`Tools/Underwater/vehicle_profile.py`, the whole file:

```python
"""Shim: the vehicle profile loader lives on main as engine/kernels/marine6_profile.py (the same code). This
module is that module, so `import vehicle_profile` (Tools/Underwater on sys.path) and every caller keep working."""
import sys
from pathlib import Path

_root = str(Path(__file__).resolve().parents[2])
if _root not in sys.path:
    sys.path.insert(0, _root)
from engine.kernels import marine6_profile as _profile  # noqa: E402

sys.modules[__name__] = _profile
```

`crane_sub/plant/dynamics.py`, the whole file (optional; `VecDynamics(params, thrusters, thruster_gain=1.0)`
has the platform's signature and methods, and agrees with the platform's numpy `VecDynamics` to 1e-13 over 300
steps, not bit for bit, so golden files that pin its rounding are regenerated):

```python
"""Shim: VecDynamics lives on main as engine/kernels/marine6_vec.py, over the batched marine6 kernel (the same
equations; parity with fossen6dof.Simulator within 1e-12, not bit for bit with the old numpy twin)."""
import sys

from ._underwater import REPO_ROOT  # noqa: F401  (puts Tools/Underwater on sys.path)

if str(REPO_ROOT) not in sys.path:
    sys.path.insert(0, str(REPO_ROOT))
from engine.kernels.marine6_vec import VecDynamics  # noqa: E402,F401
```

The other `crane_sub/plant/*` modules (contact, actuators, projectiles, hull, ...) stay on the platform branch
and reach the plant through these shims; no profile JSON changes.

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
| 1 (marine6) | numpy, numba and JAX float64 = `marine6_ref.Simulator.step` to 1e-10 over a scripted wrench program (current, lag, saturation, turns past 180 deg, 1 and 2 substeps); env i at E=1 = at E=1024 | `engine/tests/test_marine6.py`, `domains/sub/tests` |

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
