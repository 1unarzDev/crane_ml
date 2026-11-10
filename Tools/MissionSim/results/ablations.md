# Isolation ablations: radio loss vs localization drift vs stale map, then correlated failure

Evidence label: IMPLEMENTED (synthetic mission-level simulator, placeholder observation model; not LIVE_SIM_VALIDATED or PHYSICALLY_CALIBRATED).

Split val, 8 layouts (70000..70007), 280 missions; paired by (layout, rep); CIs are 95% cluster bootstrap over layouts (2000 resamples). Failed missions are included in every rate.

baseline = medium canopy; radio_only = foliage attenuation x1.6 (planners keep the nominal forecast); drift_only = odometry drift x5; stale_only = stale prior map (more unmapped blockages, phantom blockages, landmark offsets ~6.8 m, canopy error); correlated_dense = dense canopy degrading radio, drift and map together.

| policy | baseline | radio_only | drift_only | stale_only | correlated_dense |
|---|---|---|---|---|---|
| pod | 0.12 [0.00, 0.38] | 0.12 [0.00, 0.38] | 0.00 [0.00, 0.00] | 0.25 [0.00, 0.62] | 0.00 [0.00, 0.00] |
| lease | 0.00 [0.00, 0.00] | 0.12 [0.00, 0.38] | 0.00 [0.00, 0.00] | 0.25 [0.00, 0.62] | 0.12 [0.00, 0.38] |
| pod_maintain | 0.00 [0.00, 0.00] | 0.00 [0.00, 0.00] | 0.00 [0.00, 0.00] | 0.12 [0.00, 0.38] | 0.12 [0.00, 0.38] |
| pod_deadzone | 0.12 [0.00, 0.38] | 0.00 [0.00, 0.00] | 0.12 [0.00, 0.38] | 0.12 [0.00, 0.38] | 0.00 [0.00, 0.00] |
| b45_mule_recheck | 0.00 [0.00, 0.00] | 0.00 [0.00, 0.00] | 0.00 [0.00, 0.00] | 0.12 [0.00, 0.38] | 0.12 [0.00, 0.38] |
| rendezvous | 0.12 [0.00, 0.38] | 0.12 [0.00, 0.38] | 0.00 [0.00, 0.00] | 0.12 [0.00, 0.38] | 0.00 [0.00, 0.00] |
| oracle | 1.00 [1.00, 1.00] | 0.88 [0.62, 1.00] | 0.50 [0.12, 0.88] | 0.75 [0.38, 1.00] | 0.75 [0.50, 1.00] |

Paired change vs baseline condition, same policy (success; cluster bootstrap over layouts):

| policy | radio_only | drift_only | stale_only | correlated_dense |
|---|---|---|---|---|
| pod | 0.00 [0.00, 0.00] | -0.12 [-0.38, 0.00] | 0.12 [0.00, 0.38] | -0.12 [-0.38, 0.00] |
| lease | 0.12 [0.00, 0.38] | 0.00 [0.00, 0.00] | 0.25 [0.00, 0.62] | 0.12 [0.00, 0.38] |
| pod_maintain | 0.00 [0.00, 0.00] | 0.00 [0.00, 0.00] | 0.12 [0.00, 0.38] | 0.12 [0.00, 0.38] |
| pod_deadzone | -0.12 [-0.38, 0.00] | 0.00 [-0.38, 0.38] | 0.00 [0.00, 0.00] | -0.12 [-0.38, 0.00] |
| b45_mule_recheck | 0.00 [0.00, 0.00] | 0.00 [0.00, 0.00] | 0.12 [0.00, 0.38] | 0.12 [0.00, 0.38] |
| rendezvous | 0.00 [-0.38, 0.38] | -0.12 [-0.38, 0.00] | 0.00 [0.00, 0.00] | -0.12 [-0.38, 0.00] |
| oracle | -0.12 [-0.38, 0.00] | -0.50 [-0.88, -0.12] | -0.25 [-0.62, 0.00] | -0.25 [-0.50, 0.00] |

pod_deadzone minus comparator, per condition (success):

| comparator | baseline | radio_only | drift_only | stale_only | correlated_dense |
|---|---|---|---|---|---|
| pod | 0.00 [0.00, 0.00] | -0.12 [-0.38, 0.00] | 0.12 [0.00, 0.38] | -0.12 [-0.38, 0.00] | 0.00 [0.00, 0.00] |
| lease | 0.12 [0.00, 0.38] | -0.12 [-0.38, 0.00] | 0.12 [0.00, 0.38] | -0.12 [-0.38, 0.00] | -0.12 [-0.38, 0.00] |
| pod_maintain | 0.12 [0.00, 0.38] | 0.00 [0.00, 0.00] | 0.12 [0.00, 0.38] | 0.00 [-0.38, 0.38] | -0.12 [-0.38, 0.00] |
| b45_mule_recheck | 0.12 [0.00, 0.38] | 0.00 [0.00, 0.00] | 0.12 [0.00, 0.38] | 0.00 [0.00, 0.00] | -0.12 [-0.38, 0.00] |
| rendezvous | 0.00 [0.00, 0.00] | -0.12 [-0.38, 0.00] | 0.12 [0.00, 0.38] | 0.00 [0.00, 0.00] | 0.00 [0.00, 0.00] |
| oracle | -0.88 [-1.00, -0.62] | -0.88 [-1.00, -0.62] | -0.38 [-0.75, 0.12] | -0.62 [-0.88, -0.25] | -0.75 [-1.00, -0.50] |

### baseline: failure causes

| policy | n | detected_not_confirmed | location_error | not_detected | success |
|---|---|---|---|---|---|
| pod | 8 | 0 | 0 | 7 | 1 |
| lease | 8 | 2 | 0 | 6 | 0 |
| pod_maintain | 8 | 0 | 1 | 7 | 0 |
| pod_deadzone | 8 | 1 | 0 | 6 | 1 |
| b45_mule_recheck | 8 | 2 | 0 | 6 | 0 |
| rendezvous | 8 | 1 | 0 | 6 | 1 |
| oracle | 8 | 0 | 0 | 0 | 8 |

### radio_only: failure causes

| policy | n | detected_not_confirmed | not_detected | success |
|---|---|---|---|---|
| pod | 8 | 0 | 7 | 1 |
| lease | 8 | 0 | 7 | 1 |
| pod_maintain | 8 | 1 | 7 | 0 |
| pod_deadzone | 8 | 3 | 5 | 0 |
| b45_mule_recheck | 8 | 2 | 6 | 0 |
| rendezvous | 8 | 0 | 7 | 1 |
| oracle | 8 | 1 | 0 | 7 |

### drift_only: failure causes

| policy | n | detected_not_confirmed | location_error | not_detected | success |
|---|---|---|---|---|---|
| pod | 8 | 1 | 1 | 6 | 0 |
| lease | 8 | 0 | 2 | 6 | 0 |
| pod_maintain | 8 | 0 | 0 | 8 | 0 |
| pod_deadzone | 8 | 2 | 0 | 5 | 1 |
| b45_mule_recheck | 8 | 0 | 1 | 7 | 0 |
| rendezvous | 8 | 1 | 0 | 7 | 0 |
| oracle | 8 | 2 | 1 | 1 | 4 |

### stale_only: failure causes

| policy | n | detected_not_confirmed | not_detected | success |
|---|---|---|---|---|
| pod | 8 | 1 | 5 | 2 |
| lease | 8 | 2 | 4 | 2 |
| pod_maintain | 8 | 3 | 4 | 1 |
| pod_deadzone | 8 | 1 | 6 | 1 |
| b45_mule_recheck | 8 | 0 | 7 | 1 |
| rendezvous | 8 | 0 | 7 | 1 |
| oracle | 8 | 2 | 0 | 6 |

### correlated_dense: failure causes

| policy | n | detected_not_confirmed | location_error | not_detected | success |
|---|---|---|---|---|---|
| pod | 8 | 3 | 0 | 5 | 0 |
| lease | 8 | 0 | 0 | 7 | 1 |
| pod_maintain | 8 | 1 | 1 | 5 | 1 |
| pod_deadzone | 8 | 1 | 0 | 7 | 0 |
| b45_mule_recheck | 8 | 1 | 0 | 6 | 1 |
| rendezvous | 8 | 2 | 0 | 6 | 0 |
| oracle | 8 | 1 | 0 | 1 | 6 |
