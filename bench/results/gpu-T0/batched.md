# bench.fast --engine batched: boat T0

Table B: {'W1': 9.63, 'W8': 55.9} episodes/s (scalar fastsim, W=1 and W=8).

| backend | dtype | E | seeds | setup s | first call s | wall s | episodes/s | x B (W=1) | x B (W=8) | succeeded |
|---|---|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| jax | float32 | 4096 | 16384 | 9.449 | 8.135 | 3.5124 | 4664.6 | 484.4 | 83.4 | 16384/16384 |
| jax | float32 | 16384 | 65536 | 35.799 | 16.366 | 11.8948 | 5509.6 | 572.1 | 98.6 | 65536/65536 |

jax float32 at E=16384, one jitted step per call:

| step | ms per call | kernels |
|---|---:|---:|
| physics step (tracker, loop, plant, progress) | 0.172 | 9 |
| track + tick step (adds scoring, contacts, perception, decision) | 1.2985 | 113 |
