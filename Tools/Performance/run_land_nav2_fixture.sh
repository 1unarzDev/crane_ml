#!/usr/bin/env bash
set -euo pipefail

root_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
result_root="${CRANE_RESULT_ROOT:-${root_dir}/PerformanceResults/land-nav2-fixture}"
worker_id="${CRANE_WORKER_ID:-0}"

export CRANE_SCENE="Land Vehicle Validation"
export CRANE_NOGRAPHICS=1
export CRANE_NAV2_PROFILE=train-cpu
export CRANE_NAV2_PARAMS="${root_dir}/Tools/Performance/nav2_land_fixture.yaml"
export CRANE_NAV2_COMMAND_FLAG=--crane-ros-ackermann-cmd-vel
export CRANE_NAV2_GOAL_DISTANCE="${CRANE_NAV2_GOAL_DISTANCE:-3.0}"
export CRANE_NAV2_ACTION_DURATION="${CRANE_NAV2_ACTION_DURATION:-20}"
export CRANE_NAV2_COSTMAP_TOPIC="${CRANE_NAV2_COSTMAP_TOPIC:-/local_costmap/costmap}"
export CRANE_REQUIRE_OCCUPIED_COSTMAP=1
export CRANE_RESULT_ROOT="${result_root}"
export CRANE_NAV2_UNITY_EXTRA_ARGS="--crane-land-nav2 --crane-ros-nav-child-frames lidar_link --crane-land-evaluator-output ${result_root}/worker-${worker_id}/land-evaluator-truth.json ${CRANE_NAV2_UNITY_EXTRA_ARGS:-}"

exec "${root_dir}/Tools/Performance/run_nav2_controller_fixture.sh"
