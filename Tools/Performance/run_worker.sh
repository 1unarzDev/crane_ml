#!/usr/bin/env bash
set -euo pipefail

root_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
worker_id="${1:-0}"
shift || true

player="${CRANE_PLAYER:-${root_dir}/Builds/CRANE-Worker/CRANE.x86_64}"
result_root="${CRANE_RESULT_ROOT:-${root_dir}/PerformanceResults/workers}"
scene="${CRANE_SCENE:-Roboboat Course}"
duration="${CRANE_DURATION:-30}"
warmup="${CRANE_WARMUP:-5}"
time_scale="${CRANE_TIME_SCALE:-1}"
seed_base="${CRANE_SEED_BASE:-1000}"
ros_port_base="${CRANE_ROS_PORT_BASE:-10000}"
mavros_port_base="${CRANE_MAVROS_PORT_BASE:-9002}"

worker_dir="${result_root}/worker-${worker_id}"
mkdir -p "${worker_dir}"
seed=$((seed_base + worker_id))
ros_port=$((ros_port_base + worker_id))
mavros_port=$((mavros_port_base + worker_id))

gpu_sampler_pid=""
cleanup() {
    if [[ -n "${gpu_sampler_pid}" ]]; then kill "${gpu_sampler_pid}" 2>/dev/null || true; fi
}
trap cleanup EXIT INT TERM

if command -v nvidia-smi >/dev/null 2>&1; then
    nvidia-smi --query-gpu=timestamp,index,utilization.gpu,utilization.memory,memory.used,power.draw \
        --format=csv -lms 500 -f "${worker_dir}/gpu.csv" &
    gpu_sampler_pid=$!
fi

ros_args=()
if [[ "${CRANE_DISABLE_ROS:-1}" == "1" ]]; then ros_args+=(--crane-disable-ros); fi
sensor_args=()
if [[ "${CRANE_DISABLE_RGB:-0}" == "1" ]]; then sensor_args+=(--crane-disable-rgb); fi
if [[ "${CRANE_DISABLE_DEPTH:-0}" == "1" ]]; then sensor_args+=(--crane-disable-depth); fi
if [[ "${CRANE_DISABLE_CAMERA_INFO:-0}" == "1" ]]; then sensor_args+=(--crane-disable-camera-info); fi
if [[ "${CRANE_DISABLE_DETECTIONS:-0}" == "1" ]]; then sensor_args+=(--crane-disable-detections); fi
extra_args=()
if [[ -n "${CRANE_EXTRA_ARGS:-}" ]]; then read -r -a extra_args <<<"${CRANE_EXTRA_ARGS}"; fi
player_args=()
if [[ "${CRANE_NOGRAPHICS:-0}" == "1" ]]; then player_args+=(-batchmode -nographics); fi

export SDL_VIDEODRIVER="${CRANE_SDL_VIDEODRIVER:-x11}"
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-$((worker_id + 1))}"

"${player}" \
    "${player_args[@]}" \
    -screen-fullscreen 0 -screen-width "${CRANE_SCREEN_WIDTH:-640}" -screen-height "${CRANE_SCREEN_HEIGHT:-360}" \
    -logFile "${worker_dir}/player.log" \
    --crane-worker --crane-benchmark \
    --crane-worker-id "${worker_id}" --crane-seed "${seed}" \
    --crane-ros-ip "${CRANE_ROS_IP:-127.0.0.1}" --crane-ros-port "${ros_port}" \
    --crane-mavros-port "${mavros_port}" \
    --crane-scene "${scene}" --crane-scenario "${CRANE_SCENARIO:-worker-sweep}" \
    --crane-time-scale "${time_scale}" --crane-warmup "${warmup}" --crane-duration "${duration}" \
    --crane-output "${worker_dir}/result.json" \
    "${ros_args[@]}" "${sensor_args[@]}" "${extra_args[@]}" "$@"
