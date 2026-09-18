#!/usr/bin/env bash
set -euo pipefail

root_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
workers="${1:-2}"
result_root="${CRANE_RESULT_ROOT:-${root_dir}/PerformanceResults/nav2-worker-sweep-$(date -u +%Y%m%dT%H%M%SZ)}"
base_port="${CRANE_ROS_PORT_BASE:-10000}"
base_domain="${CRANE_ROS_DOMAIN_BASE:-60}"
mkdir -p "${result_root}"
result_root="$(cd "${result_root}" && pwd)"

if (( workers < 1 )); then
    echo "worker count must be positive" >&2
    exit 2
fi
if (( base_domain < 0 || base_domain + workers - 1 > 232 )); then
    echo "ROS domain range must remain within 0..232" >&2
    exit 2
fi

start_ns="$(date +%s%N)"
pids=()
for ((worker_id = 0; worker_id < workers; worker_id++)); do
    worker_root="${result_root}/nav2-worker-${worker_id}"
    CRANE_RESULT_ROOT="${worker_root}" \
    CRANE_RUN_ID="sweep-$PPID-$$-${worker_id}" \
    CRANE_WORKER_ID="${worker_id}" \
    CRANE_ROS_PORT="$((base_port + worker_id))" \
    CRANE_ROS_DOMAIN_ID="$((base_domain + worker_id))" \
    CRANE_DURATION="${CRANE_DURATION:-22}" \
    CRANE_FIXTURE_DELAY="${CRANE_FIXTURE_DELAY:-8}" \
    "${root_dir}/Tools/Performance/run_nav2_controller_fixture.sh" \
        >"${result_root}/worker-${worker_id}.log" 2>&1 &
    pids+=("$!")
done

failed=0
for pid in "${pids[@]}"; do
    if ! wait "${pid}"; then failed=1; fi
done
if (( failed != 0 )); then
    echo "one or more Nav2 workers failed; inspect ${result_root}" >&2
    exit 1
fi

end_ns="$(date +%s%N)"
wall_seconds="$(awk -v start="${start_ns}" -v end="${end_ns}" \
    'BEGIN { printf "%.9f", (end - start) / 1000000000 }')"
python3 "${root_dir}/Tools/Performance/summarize_nav2_sweep.py" \
    "${result_root}" --workers "${workers}" --wall-seconds "${wall_seconds}"
