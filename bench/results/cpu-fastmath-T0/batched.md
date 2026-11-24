# bench.fast --engine batched: boat T0

Table B: {'W1': 9.63, 'W8': 55.9} episodes/s (scalar fastsim, W=1 and W=8).

| backend | dtype | E | seeds | setup s | first call s | wall s | episodes/s | x B (W=1) | x B (W=8) | succeeded |
|---|---|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| numba-fastmath | float64 | 4096 | 4096 | 2.496 | 0.752 | 0.5366 | 7632.6 | 792.6 | 136.5 | 4096/4096 |
