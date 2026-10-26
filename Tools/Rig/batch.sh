#!/usr/bin/env bash
# Run one rig configuration over several seeds, up to 3 at a time (run_sim.sh also caps the
# machine at rig.yaml max_instances and picks a free ROS domain and port per run).
#
# Usage: batch.sh NAME SEEDS [run_sim.sh options ...]
#   NAME   batch directory under rig.yaml sessions_dir (sessions/NAME/seed<N>)
#   SEEDS  seed list, e.g. "0 1 2" or "0-9"
#   -j N   as the first option: parallel runs (1-3, default 3)
# Example (the 2025 mission with the crane params, current 0.2 m/s toward 90 deg, 3 seeds):
#   batch.sh mission_base 0-2 --mode velocity --task mission -d 900 --current 0.2,90 \
#     --run 'sleep 15; ros2 launch mhseals_nav task.launch.py task:=mission sim:=true \
#            params_file:=$(ros2 pkg prefix mhseals_nav)/share/mhseals_nav/config/tasks/mission_crane.yaml'
# Each run appends one line to rig.yaml run_log; the batch prints a summary of its own runs.
set -uo pipefail
here="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
name=$1 seeds=$2; shift 2
jobs=3
if [[ "${1:-}" == -j ]]; then jobs=$2; shift 2; fi
((jobs >= 1 && jobs <= 3)) || { echo "-j must be 1-3" >&2; exit 2; }
[[ "${seeds}" =~ ^([0-9]+)-([0-9]+)$ ]] && seeds=$(seq "${BASH_REMATCH[1]}" "${BASH_REMATCH[2]}")
sessions=$(python3 -I "${here}/rigconf.py" "${here}/rig.yaml" sessions_dir | python3 -I -c 'import json,sys;print(json.load(sys.stdin))')
batch="${sessions}/${name}"
mkdir -p "${batch}"
printf '%s\n' "$*" >"${batch}/args.txt"
for seed in ${seeds}; do
    while (($(jobs -rp | wc -l) >= jobs)); do wait -n; done
    "${here}/run_sim.sh" --seed "${seed}" -o "${batch}/seed${seed}" "$@" >"${batch}/seed${seed}.log" 2>&1 &
    sleep 3   # stagger player start-up
done
wait
python3 -I - "${batch}" <<'PY'
import sys, yaml
from pathlib import Path
for run in sorted(Path(sys.argv[1]).glob('seed*/session.yaml')):
    meta = yaml.safe_load(run.read_text())
    out = meta.get('task_outcome') or {}
    print(f"{run.parent.name:8} seed {meta['seed']:>3}  run_status {meta.get('run_status')}  "
          f"outcome {out.get('outcome')}  score {out.get('score')}  referee {out.get('referee')}  "
          f"elapsed {out.get('elapsed_s')}  stages {out.get('stages', '')}")
PY
