#!/bin/bash
# SPDX-License-Identifier: GPL-3.0-or-later
# Pending CRANE harness batch (written 2026-11-10 when the orchestrator capped CPU). ONE process at a time, sequential.
# Launch once, after the pilot, through the long lane only:
#     ~/.cache/research/fs-scripts/fsrun Tools/CraneHarness/run_pending.sh
# Resumable: every stage appends per-run rows and skips run ids already present, so a killed batch can be relaunched.
# Estimated single-core wall time at today's load: build 1 min, stage 1 (44 short replays) ~25 min, stage 2 (70 NavPilot
# runs) ~15 min, stage 3 (CRANE side verification, 10 runs) ~10 min, stage 4 (DR/fault/comms smoke, 4 two-drone runs)
# ~15 min, stage 5 (relabel the 11 attribution runs with brake_infeasible) ~8 min, plots/reports < 1 min.
set -u
cd "$(dirname "$0")/../.."
R=$(pwd); OUT=${CRANE_HARNESS_OUT:-/tmp/crane_harness}; PY=/home/lunarz/.cache/research/fs-venv/bin/python
BIN=/tmp/crane_exp_pending
mkdir -p "$OUT"
echo "[pending] build"; (cd Assets/Scripts/Drones/Autonomy/Harness~/Experiment && dotnet build -c Release -o $BIN -v q) || exit 1
X="dotnet $BIN/CraneExperiment.dll"

echo "[pending] 1/5 counterfactual replays of the 11 colliding sweep runs (one factor removed each)"
for f in nodrift nodrop infl25 fastperc; do for v in 5 6 8; do
  $X run --spec Tools/CraneHarness/specs/cf_${f}_v$v.json --out $OUT/cf_${f}_v$v || echo "[pending] cf_${f}_v$v failed"
done; done

echo "[pending] 2/5 NavPilot 2.5D twin on the exported stands (cross_0..9 x 7 speeds, seed 1)"
[ -f $OUT/stands/stand_f1337_cross_0.json ] || $X export-navpilot --spec Tools/CraneHarness/specs/fidelity_dev.json --out $OUT/stands
$PY Tools/CraneHarness/navpilot_bridge.py --stands $OUT/stands --speeds 2,2.5,3,4,5,6,8 --seeds 1 --out $OUT/navpilot_fidelity.jsonl

echo "[pending] 3/5 CRANE side: bit-identity of the lead's sweep rows through the harness (one mission per speed file)"
$X test-lead --spec Tools/CraneHarness/specs/fidelity_dev.json --lead /tmp/ah_swB/results.jsonl --only cross_0,cross_6 | tee $OUT/test_lead_swB.txt
$X test-lead --spec Tools/CraneHarness/specs/fidelity_dev.json --lead /tmp/ah_swA/results.jsonl --only cross_0,cross_6 | tee $OUT/test_lead_swA.txt
cat /tmp/ah_swA/results.jsonl /tmp/ah_swB/results.jsonl /tmp/ah_m5a/results.jsonl > $OUT/crane_lead_rows.jsonl
$PY Tools/CraneHarness/fidelity.py --crane $OUT/crane_lead_rows.jsonl --navpilot $OUT/navpilot_fidelity.jsonl --md $OUT/fidelity_report.md

echo "[pending] 4/5 DR + faults + comms + 2-drone smoke (dev_stress family, full crane-drone-log/1)"
$X run --spec Tools/CraneHarness/specs/dev_dr_demo.json --out $OUT/dev_dr_demo
for d in $OUT/dev_dr_demo/runs/*/; do $PY Tools/DroneData/drone_log.py "$d" > "$d/validate.txt" 2>&1 || true; done

echo "[pending] 5/5 relabel the attribution runs with the brake_infeasible rule (fresh output dirs)"
for v in 5 6 8; do $X run --spec Tools/CraneHarness/specs/attrib_v$v.json --out $OUT/attrib2_v$v; done
for b in $OUT/attrib2_v*/runs/*/replay_bundle.json; do $PY Tools/CraneHarness/plot_replay.py "$b" > /dev/null; done
$PY Tools/CraneHarness/aggregate.py $OUT/attrib2_v5 $OUT/attrib2_v6 $OUT/attrib2_v8 --md $OUT/attrib2_summary.md
for f in nodrift nodrop infl25 fastperc; do $PY Tools/CraneHarness/aggregate.py $OUT/cf_${f}_v5 $OUT/cf_${f}_v6 $OUT/cf_${f}_v8 --md $OUT/cf_${f}_summary.md > /dev/null; done
echo "[pending] done: $OUT/{fidelity_report.md,attrib2_summary.md,cf_*_summary.md,dev_dr_demo}"
