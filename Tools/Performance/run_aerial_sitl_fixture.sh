#!/usr/bin/env bash
set -euo pipefail

root_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
result_root="${CRANE_RESULT_ROOT:-${root_dir}/PerformanceResults/aerial-sitl-fixture}"
port="${CRANE_MAVROS_PORT:-10312}"
maximum_lag_ticks="${CRANE_MAX_ACTION_LAG_TICKS:-5}"
mkdir -p "${result_root}"
result_root="$(cd "${result_root}" && pwd)"

player_pid=""
cleanup() {
    if [[ -n "${player_pid}" ]]; then kill "${player_pid}" >/dev/null 2>&1 || true; fi
}
trap cleanup EXIT INT TERM

CRANE_RESULT_ROOT="${result_root}" CRANE_DISABLE_ROS=0 CRANE_NOGRAPHICS=1 \
CRANE_SCENE="Aerial Vehicle Validation" CRANE_SCENARIO=aerial-sitl-loopback \
CRANE_DURATION="${CRANE_DURATION:-8}" CRANE_WARMUP="${CRANE_WARMUP:-3}" \
CRANE_TIME_SCALE="${CRANE_TIME_SCALE:-2}" CRANE_MAVROS_PORT_BASE="${port}" \
CRANE_EXTRA_ARGS="--crane-profile train-cpu --crane-disable-ros-tcp --crane-aerial-sitl-validation --crane-action-policy bounded --crane-max-action-lag-ticks ${maximum_lag_ticks}" \
    "${root_dir}/Tools/Performance/run_worker.sh" 0 &
player_pid=$!

validation_stream="${result_root}/worker-0/result.validation.jsonl"
ready=0
for _ in $(seq 1 "${CRANE_READY_POLLS:-300}"); do
    if [[ -s "${validation_stream}" ]]; then ready=1; break; fi
    if ! kill -0 "${player_pid}" >/dev/null 2>&1; then break; fi
    sleep 0.1
done
if (( ready == 0 )); then
    echo "measured episode did not become ready: ${validation_stream}" >&2
    wait "${player_pid}" || true
    player_pid=""
    exit 1
fi
sleep "${CRANE_FIXTURE_DELAY:-0.5}"
fixture_failed=0
python3 "${root_dir}/Tools/Performance/sitl_udp_fixture.py" \
    --mode aerial --port "${port}" --pwm "${CRANE_AERIAL_PWM:-1680}" \
    --duration "${CRANE_FIXTURE_DURATION:-4}" \
    --output "${result_root}/fixture-summary.json" || fixture_failed=1
wait "${player_pid}"
player_pid=""

summary_failed=0
python3 "${root_dir}/Tools/Performance/summarize_sitl_fixture.py" \
    "${result_root}" --scope aerial \
    --maximum-lag-ticks "${maximum_lag_ticks}" || summary_failed=1
if (( fixture_failed != 0 || summary_failed != 0 )); then exit 1; fi
