# bench.fast --engine batched: boat T1

Table B: {'W1': 4.33, 'W8': 27.9} episodes/s (scalar fastsim, W=1 and W=8). Course pool 5 (scenario seed = seed % 5).

| backend | dtype | E | seeds | setup s | first call s | wall s | episodes/s | x B (W=1) | x B (W=8) | succeeded |
|---|---|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| numba | float64 | 4096 | 4096 | 3.825 | 83.306 | 1.1039 | 3710.5 | 856.9 | 133.0 | 4096/4096 |
