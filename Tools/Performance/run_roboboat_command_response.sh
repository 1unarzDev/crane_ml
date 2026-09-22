#!/usr/bin/env bash
set -euo pipefail

root_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
astro_dir="${CRANE_ASTRO_DOCK:-${root_dir}/../astro_dock}"
astro_dir="$(cd "${astro_dir}" && pwd)"
image="${CRANE_ROS_IMAGE:-lunarzdev/astro:cuda}"
run_id="${CRANE_RUN_ID:-$$}"
ros_port="${CRANE_ROS_PORT:-10000}"
ros_domain_id="${CRANE_ROS_DOMAIN_ID:-42}"
worker_id="${CRANE_WORKER_ID:-0}"
result_root="${CRANE_RESULT_ROOT:-${root_dir}/PerformanceResults/roboboat-command-response}"
response_suite="${CRANE_RESPONSE_SUITE:-full}"
thruster_diagnostic_arg=""
if [[ "${CRANE_THRUSTER_DIAGNOSTICS:-0}" == "1" ]]; then
    thruster_diagnostic_arg="--crane-roboboat-thruster-diagnostics"
fi
endpoint_name="crane-endpoint-${run_id}"
fixture_name="crane-response-${run_id}"
mkdir -p "${result_root}"
result_root="$(cd "${result_root}" && pwd)"

cleanup() {
    docker rm -f "${fixture_name}" "${endpoint_name}" >/dev/null 2>&1 || true
}
trap cleanup EXIT INT TERM

docker run -d --rm --name "${endpoint_name}" --network host --ipc host \
    -e ROS_DOMAIN_ID="${ros_domain_id}" \
    -v "${astro_dir}:/workspace/astro_dock:ro" "${image}" bash -lc \
    'source /opt/ros/jazzy/setup.bash; source /workspace/astro_dock/install/setup.bash; exec /workspace/astro_dock/install/lib/ros_tcp_endpoint/default_server_endpoint --ros-args -p ROS_IP:=127.0.0.1 -p ROS_TCP_PORT:='"${ros_port}" \
    >"${result_root}/endpoint.container-id"
sleep 1

CRANE_RESULT_ROOT="${result_root}" \
CRANE_DURATION="${CRANE_DURATION:-65}" CRANE_WARMUP="${CRANE_WARMUP:-3}" \
CRANE_TIME_SCALE=1 CRANE_DISABLE_ROS=0 \
CRANE_ROS_PORT_BASE="$((ros_port - worker_id))" ROS_DOMAIN_ID="${ros_domain_id}" \
CRANE_SCENE="Roboboat Course" CRANE_SCENARIO=roboboat-command-response \
CRANE_EXTRA_ARGS="--crane-profile train-gpu --crane-ros-nav-state --crane-ros-cmd-vel /crane/cmd_vel_stamped --crane-action-policy bounded --crane-max-action-lag-ticks 10 --crane-command-timeout-ticks 25 ${thruster_diagnostic_arg}" \
    "${root_dir}/Tools/Performance/run_worker.sh" "${worker_id}" &
player_pid=$!

sleep "${CRANE_FIXTURE_DELAY:-7}"
docker run --rm --name "${fixture_name}" --network host --ipc host \
    -e ROS_DOMAIN_ID="${ros_domain_id}" \
    -v "${root_dir}:/workspace/crane_sim:ro" -v "${result_root}:/results" \
    "${image}" bash -lc \
    'source /opt/ros/jazzy/setup.bash; exec python3 /workspace/crane_sim/Tools/Performance/roboboat_command_response_fixture.py --suite '"${response_suite}"' --samples /results/body-response.csv --output /results/body-response.json' \
    | tee "${result_root}/fixture.log"

wait "${player_pid}"
docker logs "${endpoint_name}" >"${result_root}/endpoint.log" 2>&1 || true
