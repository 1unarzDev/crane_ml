#!/bin/bash
# Reproduce the CRANE vision benchmark end to end (one heavy job at a time, as the host rules require).
# Data and weights stay outside the repo under ~/.cache/research/vision-data/ (untrusted downloads, never committed).
set -euo pipefail
V=~/.cache/research/vision-venv/bin/python
HEAVY=~/astro_dock/src/mhseals_agents/tools/heavy
FS=~/.cache/research/fs-scripts/fsrun
T=$(cd "$(dirname "$0")" && pwd)
D=~/.cache/research/vision-data
B=$D/bench_nomad_v1     # raw bench output (large, outside the repo)
R=$T/results

# 1. data (light, network): NOMAD annotations must already be in $D/nomad_annotations (gdown of the 3 json files)
$FS $V -I $T/fetch_nomad.py $D/nomad_annotations $D/nomad_subset --per-loc 3 --jobs 6
# 2. benchmark (GPU)
HEAVY_WHO=vision-bench $HEAVY sim $V -I $T/bench.py $D/nomad_subset $D/nomad_annotations $B --blur 0,3,6,12,24 --full-blur 0
# 3. analysis and fit (CPU)
$FS $V -I $T/analyze.py $B $D/nomad_annotations $R --tau 0.3
$FS $V -I $T/fit_obs.py $B $R/metrics.json $T/obs_params_tau03.json $R --tau 0.3
$FS $V -I $T/analyze.py $B $D/nomad_annotations $R/tau05 --tau 0.5
$FS $V -I $T/fit_obs.py $B $R/tau05/metrics.json $T/obs_params_tau05.json $R/tau05 --tau 0.5
$FS $V -I $T/make_figures.py
for b in 0 12; do $FS $V -I $T/compare_temporal.py $B $T/obs_params_tau03.json $R/temporal --blur $b; done
# 4. latency (GPU, with the navigation-load proxy)
HEAVY_WHO=vision-latency $HEAVY sim $V -I $T/latency.py $D/nomad_subset $R/latency_raw.json --n 600 --ncpu 4
# 5. tests
(cd $T && $FS $V -I -m pytest -q test_obs_model.py)
# 6. rendered rerun spec (renders themselves: see rerun_jobs.json, run one at a time via unity-drones)
python3 -I $T/make_rerun_jobs.py
