# Lease-timing fix check (EXPLORATORY; seeds 600-603, 5x5, travel 0, d1_loss0, 300 eps)

PH_* = HYBRID counting (nominal looks for unconfirmed post-deviation slots); NP_* = no phantom looks. Suffix = in-flight partner model.

| λ | strategy | P | Δ vs IMPLICIT [95% CI] | collisions/ep |
|---|---|---|---|---|
| 1 | IMPLICIT | 0.638 | +0.000 [+0.000,+0.000] | 0.00 |
| 1 | PH_nom | 0.599 | -0.039 [-0.064,-0.017] | 5.42 |
| 1 | PH_pred | 0.637 | -0.002 [-0.021,+0.018] | 0.90 |
| 1 | PH_orc | 0.646 | +0.007 [-0.014,+0.029] | 0.00 |
| 1 | FX_nom | 0.599 | -0.039 [-0.063,-0.015] | 5.42 |
| 1 | FX_pred | 0.637 | -0.002 [-0.019,+0.017] | 0.90 |
| 1 | FX_orc | 0.646 | +0.007 [-0.014,+0.029] | 0.00 |
| 3 | IMPLICIT | 0.662 | +0.000 [+0.000,+0.000] | 0.01 |
| 3 | PH_nom | 0.604 | -0.058 [-0.084,-0.032] | 6.00 |
| 3 | PH_pred | 0.653 | -0.009 [-0.030,+0.011] | 1.10 |
| 3 | PH_orc | 0.670 | +0.007 [-0.013,+0.028] | 0.00 |
| 3 | FX_nom | 0.604 | -0.058 [-0.085,-0.032] | 6.00 |
| 3 | FX_pred | 0.653 | -0.009 [-0.029,+0.010] | 1.10 |
| 3 | FX_orc | 0.670 | +0.007 [-0.013,+0.028] | 0.00 |
