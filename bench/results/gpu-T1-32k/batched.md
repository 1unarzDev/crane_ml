# bench.fast --engine batched: boat T1

Table B: {'W1': 4.33, 'W8': 27.9} episodes/s (scalar fastsim, W=1 and W=8).

| backend | dtype | E | seeds | setup s | first call s | wall s | episodes/s | x B (W=1) | x B (W=8) | succeeded |
|---|---|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| jax | float32 | 32768 | 80000 | 41.744 | 32.505 | 21.8147 | 3667.3 | 847.0 | 131.4 | 80000/80000 |

jax float32 at E=32768, one jitted step per call:

| step | ms per call | kernels |
|---|---:|---:|
| physics step (tracker, loop, plant, progress) | 0.3553 | 9 |
| track + tick step (adds scoring, contacts, perception, decision) | 3.2249 | 116 |
