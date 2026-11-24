# bench.fast --engine batched: boat T1

Table B: {'W1': 4.33, 'W8': 27.9} episodes/s (scalar fastsim, W=1 and W=8).

| backend | dtype | E | seeds | setup s | first call s | wall s | episodes/s | x B (W=1) | x B (W=8) | succeeded |
|---|---|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| jax | float32 | 4096 | 16384 | 8.903 | 8.977 | 4.3852 | 3736.2 | 862.9 | 133.9 | 16384/16384 |
| jax | float32 | 16384 | 65536 | 33.561 | 16.788 | 13.5153 | 4849.0 | 1119.9 | 173.8 | 65536/65536 |

jax float32 at E=16384, one jitted step per call:

| step | ms per call | kernels |
|---|---:|---:|
| physics step (tracker, loop, plant, progress) | 0.1847 | 9 |
| track + tick step (adds scoring, contacts, perception, decision) | 1.3197 | 113 |
