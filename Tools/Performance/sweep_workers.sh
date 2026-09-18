#!/usr/bin/env bash
set -euo pipefail

root_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
counts=("${@:-1 2 4 8}")
run_stamp="$(date -u +%Y%m%dT%H%M%SZ)"

for count in ${counts[*]}; do
    sweep_dir="${CRANE_RESULT_ROOT:-${root_dir}/PerformanceResults/worker-sweeps}/${run_stamp}/${count}-workers"
    mkdir -p "${sweep_dir}"
    pids=()
    for ((worker=0; worker<count; worker++)); do
        CRANE_RESULT_ROOT="${sweep_dir}" "${root_dir}/Tools/Performance/run_worker.sh" "${worker}" &
        pids+=("$!")
    done
    status=0
    for pid in "${pids[@]}"; do wait "${pid}" || status=1; done
    python3 "${root_dir}/Tools/Performance/summarize_workers.py" "${sweep_dir}" \
        --output "${sweep_dir}/summary.json"
    if ((status != 0)); then exit "${status}"; fi
done
