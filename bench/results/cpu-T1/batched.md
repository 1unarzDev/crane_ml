# bench.fast --engine batched: boat T1

Table B: {'W1': 4.33, 'W8': 27.9} episodes/s (scalar fastsim, W=1 and W=8).

| backend | dtype | E | seeds | setup s | first call s | wall s | episodes/s | x B (W=1) | x B (W=8) | succeeded |
|---|---|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| numba | float64 | 1 | 1 | 0.01 | 0.169 | 0.0055 | 182.1 | 42.1 | 6.5 | 1/1 |
| numba | float64 | 64 | 64 | 0.293 | 0.021 | 0.0218 | 2933.5 | 677.5 | 105.1 | 64/64 |
| numba | float64 | 1024 | 1024 | 1.42 | 0.307 | 0.3035 | 3373.5 | 779.1 | 120.9 | 1024/1024 |
| numba | float64 | 4096 | 4096 | 2.37 | 1.191 | 1.21 | 3385.0 | 781.8 | 121.3 | 4096/4096 |
| numba | float64 | 16384 | 16384 | 8.553 | 4.916 | 4.8581 | 3372.5 | 778.9 | 120.9 | 16384/16384 |
| numpy | float64 | 1 | 1 | 0.009 | 6.924 | 6.8845 | 0.1 | 0.0 | 0.0 | 1/1 |
| numpy | float64 | 64 | 64 | 0.291 | 13.385 | 13.1857 | 4.9 | 1.1 | 0.2 | 64/64 |
| numpy | float64 | 1024 | 1024 | 1.414 | 119.651 | 117.0601 | 8.7 | 2.0 | 0.3 | 1024/1024 |

numba, one thread: 3.248 ms per episode (7748.6 steps)

| part | ms per episode |
|---|---:|
| plant (velocity loop, allocation, kernel) | 1.149 |
| tracker and progress check | 0.639 |
| track row (rounding, scoring) | 0.042 |
| contacts (footprint clearance) | 0.21 |
| perception (cone, occlusion; full run less one with nothing to perceive) | 0.814 |
| decision and other (by difference) | 0.394 |
