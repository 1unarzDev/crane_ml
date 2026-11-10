## Outcomes by speed (11 runs)

| speed | n | clusters | success (cluster-boot 95% CI) | collision (cluster-boot 95% CI) | Wilson success | labels |
|---|---|---|---|---|---|---|
| 5 | 2 | 2 | 0.00 [0.00, 0.00] | 1.00 [1.00, 1.00] | 0.00-0.66 | estimator_drift 1, tracking_lag 1 |
| 6 | 5 | 5 | 0.00 [0.00, 0.00] | 1.00 [1.00, 1.00] | 0.00-0.43 | estimator_drift 3, latency_stall 1, tracking_lag 1 |
| 8 | 4 | 4 | 0.00 [0.00, 0.00] | 1.00 [1.00, 1.00] | 0.00-0.49 | estimator_drift 1, geometry_graze 1, tracking_lag 2 |

## Failures and labels

| run | outcome | label | t [s] | struck | D [cm] | evidence |
|---|---|---|---|---|---|---|
| attrib_v5-f1337-corridor_rev-v5-r1-d0 | collision | tracking_lag | 14.34 | mid_472_young_conifer/seg555255 | 2.6 | belief knew the obstacle was inside the body radius (0.00 m), the vehicle did not stop/turn in time |
| attrib_v5-f1337-cross_2-v5-r1-d0 | collision | estimator_drift | 8.52 | tree_312/seg249783 | 25.4 | local error changed 0.16 m between first mapping and contact |
| attrib_v6-f1337-corridor_rev-v6-r1-d0 | collision | estimator_drift | 11.94 | tree_1418/seg513196 | 3.4 | local error changed 0.21 m between first mapping and contact |
| attrib_v6-f1337-cross_1-v6-r1-d0 | collision | latency_stall | 15.06 | tree_426/seg310308 | 20.9 | mapped in time, deadline miss / watchdog / perception stall in the last 1 s |
| attrib_v6-f1337-cross_3-v6-r1-d0 | collision | tracking_lag | 19.08 | tree_245/seg197660 | 2.2 | belief knew the obstacle was inside the body radius (0.20 m), the vehicle did not stop/turn in time |
| attrib_v6-f1337-cross_4-v6-r1-d0 | collision | estimator_drift | 8.04 | tree_748/seg435928 | 15.6 | local error changed 0.17 m between first mapping and contact |
| attrib_v6-f1337-cross_8-v6-r1-d0 | collision | estimator_drift | 2.88 | tree_344/seg267910 | 28.6 | local error changed 0.16 m between first mapping and contact |
| attrib_v8-f1337-cross_1-v8-r1-d0 | collision | tracking_lag | 4.52 | tree_782/seg445009 | 2.3 | belief knew the obstacle was inside the body radius (0.20 m), the vehicle did not stop/turn in time |
| attrib_v8-f1337-cross_2-v8-r1-d0 | collision | estimator_drift | 8.84 | tree_1003/seg494641 | 2.7 | local error changed 0.17 m between first mapping and contact |
| attrib_v8-f1337-cross_4-v8-r1-d0 | collision | tracking_lag | 5.60 | tree_590/seg394939 | 16.1 | belief knew the obstacle was inside the body radius (0.20 m), the vehicle did not stop/turn in time |
| attrib_v8-f1337-cross_8-v8-r1-d0 | collision | geometry_graze | 4.52 | tree_1553/seg519445 | 2.4 | mapped and kept, but the belief put the obstacle 0.28 m (voxel centres) from the estimate at contact (body 0.25, inflation 0.50): quantization/inflation under-r |
