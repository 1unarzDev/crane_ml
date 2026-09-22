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
maximum_action_lag_ticks="${CRANE_MAX_ACTION_LAG_TICKS:-10}"
command_timeout_ticks="${CRANE_COMMAND_TIMEOUT_TICKS:-25}"
controller_extra_args="${CRANE_NAV2_CONTROLLER_EXTRA_ARGS:-}"
bt_xml="${CRANE_NAV2_BT_XML:-}"
runtime_profile="${CRANE_NAV2_PROFILE:-train-gpu}"
scene="${CRANE_SCENE:-Roboboat Course}"
command_flag="${CRANE_NAV2_COMMAND_FLAG:---crane-ros-cmd-vel}"
goal_distance="${CRANE_NAV2_GOAL_DISTANCE:-0.5}"
action_duration="${CRANE_NAV2_ACTION_DURATION:-20}"
costmap_topic="${CRANE_NAV2_COSTMAP_TOPIC:-/local_costmap/costmap}"
bt_max_transitions="${CRANE_BT_MAX_TRANSITIONS:-4096}"
bt_max_invocations="${CRANE_BT_MAX_INVOCATIONS:-1024}"
if [[ ! "${bt_max_transitions}" =~ ^[1-9][0-9]*$ || \
      ! "${bt_max_invocations}" =~ ^[1-9][0-9]*$ ]]; then
    echo "BehaviorTree capture bounds must be positive integers" >&2
    exit 2
fi
endpoint_name="crane-endpoint-${run_id}"
controller_name="crane-controller-${run_id}"
fixture_name="crane-fixture-${run_id}"
result_root="${CRANE_RESULT_ROOT:-${root_dir}/PerformanceResults/nav2-controller-fixture}"
params_file="${CRANE_NAV2_PARAMS:-${root_dir}/Tools/Performance/nav2_controller_fixture.yaml}"
params_file="$(realpath "${params_file}")"
if [[ "${params_file}" != "${root_dir}"/* ]]; then
    echo "Nav2 params must be inside the CRANE repository: ${params_file}" >&2
    exit 2
fi
params_container="/workspace/crane_sim/${params_file#"${root_dir}"/}"
bt_xml_container=""
if [[ -n "${bt_xml}" ]]; then
    bt_xml="$(realpath "${bt_xml}")"
    if [[ "${bt_xml}" != "${root_dir}"/* ]]; then
        echo "Behavior tree XML must be inside the CRANE repository: ${bt_xml}" >&2
        exit 2
    fi
    bt_xml_container="/workspace/crane_sim/${bt_xml#"${root_dir}"/}"
fi
mkdir -p "${result_root}"
result_root="$(cd "${result_root}" && pwd)"
resource_sampler_pid=""

cleanup() {
    if [[ -n "${resource_sampler_pid}" ]]; then
        kill "${resource_sampler_pid}" >/dev/null 2>&1 || true
    fi
    docker rm -f "${fixture_name}" "${controller_name}" "${endpoint_name}" \
        >/dev/null 2>&1 || true
}
trap cleanup EXIT INT TERM

# Fast DDS may select shared memory after discovery. All ROS containers therefore share the host
# IPC namespace; omitting this can expose topics in the graph while silently delivering no data.
docker run -d --rm --name "${endpoint_name}" --network host --ipc host \
    -e ROS_DOMAIN_ID="${ros_domain_id}" \
    -v "${astro_dir}:/workspace/astro_dock:ro" "${image}" bash -lc \
    'source /opt/ros/jazzy/setup.bash; source /workspace/astro_dock/install/setup.bash; exec /workspace/astro_dock/install/lib/ros_tcp_endpoint/default_server_endpoint --ros-args -p ROS_IP:=127.0.0.1 -p ROS_TCP_PORT:='"${ros_port}" \
    >"${result_root}/endpoint.container-id"
sleep 1

docker run -d --rm --name "${controller_name}" --network host --ipc host \
    -e ROS_DOMAIN_ID="${ros_domain_id}" \
    -e CRANE_CONTROLLER_EXTRA_ARGS="${controller_extra_args}" \
    -e CRANE_BT_XML_CONTAINER="${bt_xml_container}" \
    -v "${root_dir}:/workspace/crane_sim:ro" "${image}" bash -lc \
    'source /opt/ros/jazzy/setup.bash; params='"${params_container}"'; controller_extra=(); bt_extra=(); if [[ -n "${CRANE_CONTROLLER_EXTRA_ARGS:-}" ]]; then read -r -a controller_extra <<<"${CRANE_CONTROLLER_EXTRA_ARGS}"; fi; if [[ -n "${CRANE_BT_XML_CONTAINER:-}" ]]; then bt_extra=(-p "default_nav_to_pose_bt_xml:=${CRANE_BT_XML_CONTAINER}"); fi; /opt/ros/jazzy/lib/nav2_controller/controller_server --ros-args --params-file "$params" -r cmd_vel:=/nav2/cmd_vel "${controller_extra[@]}" & p1=$!; /opt/ros/jazzy/lib/nav2_planner/planner_server --ros-args --params-file "$params" & p2=$!; /opt/ros/jazzy/lib/nav2_behaviors/behavior_server --ros-args --params-file "$params" -r cmd_vel:=/nav2/cmd_vel & p3=$!; /opt/ros/jazzy/lib/nav2_bt_navigator/bt_navigator --ros-args --params-file "$params" "${bt_extra[@]}" & p4=$!; sleep 1; /opt/ros/jazzy/lib/nav2_lifecycle_manager/lifecycle_manager --ros-args -r __node:=lifecycle_manager_controller --params-file "$params" & p5=$!; wait $p1 $p2 $p3 $p4 $p5' \
    >"${result_root}/controller.container-id"

sample_external_resources() {
    printf 'timestampUnix,role,container,cpuPercent,memoryUsage\n'
    while docker inspect "${endpoint_name}" "${controller_name}" >/dev/null 2>&1; do
        timestamp="$(date +%s.%N)"
        for role_and_name in "endpoint:${endpoint_name}" "controller:${controller_name}"; do
            role="${role_and_name%%:*}"
            name="${role_and_name#*:}"
            docker stats --no-stream --format '{{.Name}},{{.CPUPerc}},{{.MemUsage}}' "${name}" \
                2>/dev/null | while IFS= read -r sample; do
                    printf '%s,%s,%s\n' "${timestamp}" "${role}" "${sample}"
                done
        done
        sleep 1
    done
} >"${result_root}/external-resources.csv"
sample_external_resources &
resource_sampler_pid=$!

CRANE_RESULT_ROOT="${result_root}" \
CRANE_DURATION="${CRANE_DURATION:-30}" CRANE_WARMUP="${CRANE_WARMUP:-3}" \
CRANE_TIME_SCALE="${CRANE_TIME_SCALE:-1}" CRANE_DISABLE_ROS=0 \
CRANE_ROS_PORT_BASE="$((ros_port - worker_id))" ROS_DOMAIN_ID="${ros_domain_id}" \
CRANE_SCENE="${scene}" \
CRANE_SCENARIO=nav2-controller-follow-path \
CRANE_EXTRA_ARGS="--crane-profile ${runtime_profile} --crane-ros-nav-state ${command_flag} /crane/cmd_vel_stamped --crane-action-policy bounded --crane-max-action-lag-ticks ${maximum_action_lag_ticks} --crane-command-timeout-ticks ${command_timeout_ticks} ${CRANE_NAV2_UNITY_EXTRA_ARGS:-}" \
    "${root_dir}/Tools/Performance/run_worker.sh" "${worker_id}" &
player_pid=$!

# CraneBenchmarkRunner starts a fresh measured episode after warmup. Sending control before that
# boundary correctly produces a cross-episode rejection, so wait past warmup plus scene/ROS
# activation margin before presenting the acceptance goal.
fixture_delay="${CRANE_FIXTURE_DELAY:-$(awk -v warmup="${CRANE_WARMUP:-3}" 'BEGIN { print warmup + 4 }')}"
sleep "${fixture_delay}"
nav2_action_mode="${CRANE_NAV2_ACTION_MODE:-navigate-to-pose}"

docker run --rm --name "${fixture_name}" --network host --ipc host \
    -e ROS_DOMAIN_ID="${ros_domain_id}" \
    -e CRANE_BT_XML_CONTAINER="${bt_xml_container}" \
    -v "${root_dir}:/workspace/crane_sim:ro" -v "${result_root}:/results" \
    "${image}" bash -lc \
    'source /opt/ros/jazzy/setup.bash; bt_fixture_extra=(); if [[ -n "${CRANE_BT_XML_CONTAINER:-}" ]]; then bt_fixture_extra=(--bt-xml "${CRANE_BT_XML_CONTAINER}"); fi; exec python3 /workspace/crane_sim/Tools/Performance/nav2_follow_path_fixture.py --input-type twist --action-mode '"${nav2_action_mode}"' --distance '"${goal_distance}"' --duration '"${action_duration}"' --costmap-topic '"${costmap_topic}"' --bt-max-transitions '"${bt_max_transitions}"' --bt-max-invocations '"${bt_max_invocations}"' "${bt_fixture_extra[@]}" --episode-id '"${run_id}-worker-${worker_id}"' --run-id '"${run_id}"' --output /results/fixture-summary.json' \
    | tee "${result_root}/fixture.log"

wait "${player_pid}"
kill "${resource_sampler_pid}" >/dev/null 2>&1 || true
wait "${resource_sampler_pid}" 2>/dev/null || true
resource_sampler_pid=""
docker logs "${endpoint_name}" >"${result_root}/endpoint.log" 2>&1 || true
docker logs "${controller_name}" >"${result_root}/controller.log" 2>&1 || true
summary_args=()
summary_args+=(--expected-navigation-status "${CRANE_EXPECTED_NAV_STATUS:-succeeded}")
if [[ " ${CRANE_NAV2_UNITY_EXTRA_ARGS:-} " == *" --crane-reset-probe "* ]]; then
    summary_args+=(--require-reset)
fi
if [[ "${CRANE_REQUIRE_OCCUPIED_COSTMAP:-0}" == "1" ]]; then
    summary_args+=(--require-occupied-costmap)
fi
python3 "${root_dir}/Tools/Performance/summarize_nav2_reset.py" \
    "${result_root}" --worker-id "${worker_id}" --ros-port "${ros_port}" \
    --ros-domain-id "${ros_domain_id}" \
    --max-action-lag-ticks "${maximum_action_lag_ticks}" \
    --command-timeout-ticks "${command_timeout_ticks}" "${summary_args[@]}"
