# bench.fast --engine batched: boat T1

Table B: {'W1': 4.33, 'W8': 27.9} episodes/s (scalar fastsim, W=1 and W=8).

| backend | dtype | E | seeds | setup s | first call s | wall s | episodes/s | x B (W=1) | x B (W=8) | succeeded |
|---|---|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| numba-fastmath | float64 | 4096 | 4096 | 2.421 | 1.315 | 1.0545 | 3884.2 | 897.0 | 139.2 | 4096/4096 |
