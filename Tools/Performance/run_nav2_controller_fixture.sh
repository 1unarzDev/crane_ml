#!/usr/bin/env bash
set -euo pipefail

root_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
astro_dir="${CRANE_ASTRO_DOCK:-/home/lunarz/astro_dock}"
image="${CRANE_ROS_IMAGE:-lunarzdev/astro:cuda}"
run_id="${CRANE_RUN_ID:-$$}"
endpoint_name="crane-endpoint-${run_id}"
controller_name="crane-controller-${run_id}"
fixture_name="crane-fixture-${run_id}"
result_root="${CRANE_RESULT_ROOT:-${root_dir}/PerformanceResults/nav2-controller-fixture}"
mkdir -p "${result_root}"
result_root="$(cd "${result_root}" && pwd)"

cleanup() {
    docker rm -f "${fixture_name}" "${controller_name}" "${endpoint_name}" \
        >/dev/null 2>&1 || true
}
trap cleanup EXIT INT TERM

# Fast DDS may select shared memory after discovery. All ROS containers therefore share the host
# IPC namespace; omitting this can expose topics in the graph while silently delivering no data.
docker run -d --rm --name "${endpoint_name}" --network host --ipc host \
    -v "${astro_dir}:/workspace/astro_dock:ro" "${image}" bash -lc \
    'source /opt/ros/jazzy/setup.bash; source /workspace/astro_dock/install/setup.bash; exec /workspace/astro_dock/install/lib/ros_tcp_endpoint/default_server_endpoint --ros-args -p ROS_IP:=127.0.0.1 -p ROS_TCP_PORT:=10000' \
    >"${result_root}/endpoint.container-id"
sleep 1

docker run -d --rm --name "${controller_name}" --network host --ipc host \
    -v "${root_dir}:/workspace/crane_sim:ro" "${image}" bash -lc \
    'source /opt/ros/jazzy/setup.bash; params=/workspace/crane_sim/Tools/Performance/nav2_controller_fixture.yaml; /opt/ros/jazzy/lib/nav2_controller/controller_server --ros-args --params-file "$params" -r cmd_vel:=/nav2/cmd_vel & p1=$!; /opt/ros/jazzy/lib/nav2_planner/planner_server --ros-args --params-file "$params" & p2=$!; /opt/ros/jazzy/lib/nav2_behaviors/behavior_server --ros-args --params-file "$params" -r cmd_vel:=/nav2/cmd_vel & p3=$!; /opt/ros/jazzy/lib/nav2_bt_navigator/bt_navigator --ros-args --params-file "$params" & p4=$!; sleep 1; /opt/ros/jazzy/lib/nav2_lifecycle_manager/lifecycle_manager --ros-args -r __node:=lifecycle_manager_controller --params-file "$params" & p5=$!; wait $p1 $p2 $p3 $p4 $p5' \
    >"${result_root}/controller.container-id"

CRANE_RESULT_ROOT="${result_root}" \
CRANE_DURATION="${CRANE_DURATION:-30}" CRANE_WARMUP="${CRANE_WARMUP:-3}" \
CRANE_TIME_SCALE="${CRANE_TIME_SCALE:-1}" CRANE_DISABLE_ROS=0 \
CRANE_SCENARIO=nav2-controller-follow-path \
CRANE_EXTRA_ARGS='--crane-profile train-gpu --crane-ros-nav-state --crane-ros-cmd-vel /crane/cmd_vel_stamped --crane-action-policy bounded --crane-max-action-lag-ticks 10 --crane-command-timeout-ticks 25' \
    "${root_dir}/Tools/Performance/run_worker.sh" 0 &
player_pid=$!

# CraneBenchmarkRunner starts a fresh measured episode after warmup. Sending control before that
# boundary correctly produces a cross-episode rejection, so wait past warmup plus scene/ROS
# activation margin before presenting the acceptance goal.
fixture_delay="${CRANE_FIXTURE_DELAY:-$(awk -v warmup="${CRANE_WARMUP:-3}" 'BEGIN { print warmup + 4 }')}"
sleep "${fixture_delay}"
nav2_action_mode="${CRANE_NAV2_ACTION_MODE:-navigate-to-pose}"

docker run --rm --name "${fixture_name}" --network host --ipc host \
    -v "${root_dir}:/workspace/crane_sim:ro" -v "${result_root}:/results" \
    "${image}" bash -lc \
    'source /opt/ros/jazzy/setup.bash; exec python3 /workspace/crane_sim/Tools/Performance/nav2_follow_path_fixture.py --input-type twist --action-mode '"${nav2_action_mode}"' --distance 0.5 --duration 20 --output /results/fixture-summary.json' \
    | tee "${result_root}/fixture.log"

wait "${player_pid}"
docker logs "${endpoint_name}" >"${result_root}/endpoint.log" 2>&1 || true
docker logs "${controller_name}" >"${result_root}/controller.log" 2>&1 || true
