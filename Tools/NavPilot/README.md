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

## E3: maximum reliably achievable speed (navigation-under-uncertainty track)
- Literature: `NAV_LIT.md`. Protocol: `PREREG_SPEED.md` (FROZEN 2026-11-10 00:59). Optional learned-policy arm:
  `BC_SHIELD_SPEC.md` (spec only).
- Code (new files only; the E2-frozen modules are untouched): `stacks/advanced.py` (AdvStack: opt_backup_t,
  reactive_plus, mpc_lc, ua_map, pas_opt, pas_full; RangeOccMap), `experiments/e3_episode.py`, `experiments/e3.py`,
  `experiments/worker.py`.
- DEV: `results/E3_dev_tuning.md`, `results/raw/e3_tune_*.jsonl`, `results/raw/e3_tuned.json`, parity
  `results/raw/e3_parity.json` (E3 reproduces E2 exactly).
- Held-out (layouts 90400-90407, 800 episodes): job files in `~/.cache/navpilot_e3/{pending,running,done}`, run by
  `experiments/worker.py` under fsrun, rows in `results/raw/e3_heldout_*.jsonl`. When all 40 job files are in
  `done/`, run:
  `fsrun python Tools/NavPilot/experiments/e3.py report --plan heldout` -> `results/E3_heldout.md` + `results/raw/e3_heldout_summary.json`.
  If the worker exited idle before finishing, restart it with
  `fsrun python Tools/NavPilot/experiments/worker.py --queue ~/.cache/navpilot_e3 --name w1`. Its job files resume
  from `pending/`. A job file interrupted mid-run must be moved from `running/` back to `pending/`. Duplicate rows
  are deduplicated by the report.
