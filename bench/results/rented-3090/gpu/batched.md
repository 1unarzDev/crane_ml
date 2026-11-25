# bench.fast --engine batched: boat T1

Table B: {'W1': 4.33, 'W8': 27.9} episodes/s (scalar fastsim, W=1 and W=8). Course pool 5 (scenario seed = seed % 5).

| backend | dtype | E | seeds | setup s | first call s | wall s | episodes/s | x B (W=1) | x B (W=8) | succeeded |
|---|---|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| jax | float32 | 4096 | 16384 | 26.721 | 24.455 | 4.9921 | 3282.0 | 758.0 | 117.6 | 16384/16384 |
| jax | float32 | 16384 | 65536 | 105.257 | 32.586 | 14.2587 | 4596.2 | 1061.5 | 164.7 | 65536/65536 |

jax float32 at E=16384, one jitted step per call:

| step | ms per call | kernels |
|---|---:|---:|
| physics step (tracker, loop, plant, progress) | 0.3201 | 11 |
| track + tick step (adds scoring, contacts, perception, decision) | 1.3005 | 126 |
