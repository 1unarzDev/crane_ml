# Table B: the scalar fastsim baseline

```bash
bin/heavy sim python3 -m bench.fast --domain boat --tier T1 --seeds 0:50 --workers 1,8 --out DIR
```

Measured on 2026-11-24 at commit `de613930`, a clean tree. Host: i7-12700K (20 cores) with an RTX 4070 Ti
SUPER (unused here); load average about 1. System `/usr/bin/python3`: Python 3.14.10, numpy 2.5.5, PyYAML
6.0.4. Each run went through `~/crane_ml/bin/heavy sim`. The raw reports are in `bench/baseline/*.json`.

Episodes are the reference mission on seeds 0–49 (boat), or the toy's visit-every-marker task. All 50
succeeded in every row. run_sha256 was the same at 1 and 8 workers.

| domain, tier | dt (s) | sim s per episode | plant alone (env-steps/s) | episodes/s, W=1 | episodes/s, W=8 | per core, W=8 | cluster: fresh process per job |
|---|---:|---:|---:|---:|---:|---:|---:|
| boat T0 | 0.05 | 144.9 | 512,863 | 9.63 | 55.9 | 6.98 | 205 ms/job (run 117) |
| boat T1 | 0.02 | 151.9 | 94,389 | 4.33 | 27.9 | 3.48 | 342 ms/job (run 252) |
| toy T0 | 0.05 | 38.9 | 935,527 | 57.1 | 210.0 | 26.3 | 113 ms/job (run 26) |

## Where an episode's time goes

The values are milliseconds per episode, timed over 10 episodes in one process. The timers are patched in by
`bench/fast.py`; the core is not modified.

| part | boat T0 | boat T1 | toy T0 |
|---|---:|---:|---:|
| plant (`plant.step`; T1 includes the velocity loop and allocation) | 8.3 (7.5%) | 113.6 (47.9%) | 1.2 (6.4%) |
| tracker/navigator (`GoalTracker.command`, `Navigator.advance`) | 5.2 (4.8%) | 18.2 (7.7%) | 1.2 (6.5%) |
| decision tick (`Controller.tick`) | 3.8 (3.5%) | 3.9 (1.7%) | 0.6 (3.3%) |
| perception (cone and occlusion, at tick_hz) | 61.0 (55.5%) | 61.5 (25.9%) | 7.4 (38.7%) |
| contacts (footprint clearance, at track_hz) | 14.1 (12.8%) | 15.3 (6.4%) | 5.1 (26.7%) |
| recording (summary, files, `.ctraj`, `finalize`) | 10.7 (9.8%) | 10.1 (4.3%) | 1.7 (8.9%) |
| other (loop, pose to world, track, idle) | 6.8 (6.2%) | 14.6 (6.2%) | 1.8 (9.5%) |
| total | 109.9 | 237.2 | 19.0 |
| scenario load (outside the total) | 4.4 | 3.7 | 0.04 |

**Top three costs:**
- boat T0: perception (56%), contacts (13%), recording (10%).
- boat T1: the plant (48%), perception (26%), tracker/navigator (8%).
- toy T0: perception (39%), contacts (27%), other (10%).

Per core step that is about 38 µs at T0 and 31 µs at T1. Perception costs about 42 µs per tick, mostly
the per-element occlusion loop over the boat course's 50-odd elements.

## Per-job overhead (the cluster path)

`crane_backends.fastsim.FastsimBackend`'s `run(seed)` + `metrics` loop in one process costs the same as
`run_batch` at W=1:

| | boat T0 | boat T1 | toy T0 |
|---|---:|---:|---:|
| loop (ms/episode) | 102 | 230 | 18 |
| `run_batch` W=1 (ms/episode) | 104 | 231 | 18 |

One fresh process per job adds about 90 ms. That is about 30 ms of interpreter start plus 58 ms of imports,
on top of a 3–6 ms `prepare`. For the toy, the 90 ms is 4× the episode. The cluster's observed ~0.55
runs/s is well below these numbers, so most of the cluster's per-job cost lies in the worker itself (lease,
transfer, upload), which this bench does not measure.

## Reading the W=8 rows

`run_batch` spawns a fresh pool on every call. Spawning 8 workers, each importing numpy and fastsim, costs
about 0.2 s. That cost is included, so the per-core rate at W=8 understates steady-state throughput, most of
all for the toy (50 episodes take 0.24 s in total).
