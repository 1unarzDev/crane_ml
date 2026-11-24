# bench.fast --engine batched: boat T0

Table B: {'W1': 9.63, 'W8': 55.9} episodes/s (scalar fastsim, W=1 and W=8).

| backend | dtype | E | seeds | setup s | first call s | wall s | episodes/s | x B (W=1) | x B (W=8) | succeeded |
|---|---|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| numba | float64 | 1 | 1 | 0.008 | 0.154 | 0.0031 | 324.0 | 33.6 | 5.8 | 1/1 |
| numba | float64 | 64 | 64 | 0.287 | 0.011 | 0.0117 | 5485.2 | 569.6 | 98.1 | 64/64 |
| numba | float64 | 1024 | 1024 | 1.384 | 0.177 | 0.1514 | 6763.0 | 702.3 | 121.0 | 1024/1024 |
| numba | float64 | 4096 | 4096 | 2.422 | 0.583 | 0.5884 | 6960.9 | 722.8 | 124.5 | 4096/4096 |
| numba | float64 | 16384 | 16384 | 8.589 | 2.377 | 2.337 | 7010.7 | 728.0 | 125.4 | 16384/16384 |
| numpy | float64 | 1 | 1 | 0.006 | 2.204 | 2.2146 | 0.5 | 0.1 | 0.0 | 1/1 |
| numpy | float64 | 64 | 64 | 0.291 | 7.39 | 7.4349 | 8.6 | 0.9 | 0.2 | 64/64 |
| numpy | float64 | 1024 | 1024 | 1.397 | 98.287 | 98.3093 | 10.4 | 1.1 | 0.2 | 1024/1024 |

numba, one thread: 1.628 ms per episode (2956.4 steps)

| part | ms per episode |
|---|---:|
| plant (velocity loop, allocation, kernel) | 0.122 |
| tracker and progress check | 0.228 |
| track row (rounding, scoring) | 0.04 |
| contacts (footprint clearance) | 0.207 |
| perception (cone, occlusion; full run less one with nothing to perceive) | 0.782 |
| decision and other (by difference) | 0.249 |
