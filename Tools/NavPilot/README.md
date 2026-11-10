# Tools/NavPilot: cheap navigation-pilot harness (GPS-denied fast flight under canopy, 7-inch quad)

Purpose: locate where a conventional map-plan-governor baseline fails before choosing between learned proposals,
speed policies and bounded residuals. Evidence label: **IMPLEMENTED** (2.5D, synthetic; nothing here is
LIVE_SIM_VALIDATED or PHYSICALLY_CALIBRATED). Pure Python + numpy, seeded, no Unity.

| Module | Content |
|---|---|
| `navpilot/forest.py` | Seeded stands (`crane-navpilot-stand/1`: stems with DBH/height/position, thin-branch capsules in the 1-4 m band, person targets). Families: stem density, branch density, clearings (slope recorded, not modelled in 2.5D). Split guard via `Tools/DroneData/splits.py`. Also adapts a TreeGen stand file if one appears. |
| `navpilot/vehicle.py` | Point mass at fixed altitude; tilt/thrust-limited horizontal accel from the AOS7 profile T/W; drag-dependent (direction-dependent) braking; P velocity loop (tau_track 0.3 s) + jerk limit; OU gusts. |
| `navpilot/sensors.py` | OAK-D-like stereo (80 deg, z^2 noise, per-object thin-structure dropout logistic in pixel width with motion blur), L2-like LiDAR (360 deg, Poisson hits ~ rho D L / r^2), compute-time tails (lognormal + Pareto stalls), person detection (pixel height, visibility, blur). |
| `navpilot/planner.py` | Log-odds occupancy (0.1 m), inflation, 48 headings x 5 speeds forward-simulated through the tracking loop, stopping rule `d_free >= v tau + v^2/2a + margin` in belief. |
| `navpilot/episode.py` | 200 Hz plant, sensor/plan timing with latency and stalls, metrics, failure-cause labels. |
| `navpilot/logio.py` | crane-drone-log/1 subset (manifest, state.csv, events.jsonl, obstacles.json). |
| `sweep.py` | `run` (sharded sweep), `report` (writes `results/nav_failure_map.md`), `episode` (one run, optional `--log`). |

Splits: DEV seeds 0-1999, VAL 70000-79999, HELD-OUT 90000-99999 (from `Tools/DroneData/splits.json`); held-out
layouts are refused unless `--allow-heldout`. The pilot sweeps DEV only.

```
F=/home/lunarz/.cache/research/fs-scripts/fsrun; PY=/home/lunarz/.cache/research/fs-venv/bin/python
$F $PY -m unittest discover -s Tools/NavPilot/tests -p 'test_*.py'
for i in 0 1 2; do $F $PY Tools/NavPilot/sweep.py run --shard $i --nshards 3 --layouts 8 & done; wait
$F $PY Tools/NavPilot/sweep.py report
$F $PY Tools/NavPilot/sweep.py episode --seed 3 --sensor lidar --v-limit 7 --branch high --log /tmp/np_run
$F $PY Tools/DroneData/drone_log.py /tmp/np_run
```
