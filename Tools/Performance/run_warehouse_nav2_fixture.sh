#!/usr/bin/env bash
set -euo pipefail

root_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"

export CRANE_SCENE="TurtleBot3 Warehouse Validation"
export CRANE_NAV2_COMMAND_FLAG=--crane-ros-differential-cmd-vel
export CRANE_NAV2_LIDAR_FRAME=base_scan
export CRANE_NAV2_GOAL_DISTANCE="${CRANE_NAV2_GOAL_DISTANCE:-13.0}"
export CRANE_NAV2_ACTION_DURATION="${CRANE_NAV2_ACTION_DURATION:-60}"
export CRANE_DURATION="${CRANE_DURATION:-75}"
export CRANE_NAV2_UNITY_EXTRA_ARGS="--crane-preserve-reference-environment \
--crane-land-scenario-id warehouse-cross-aisle-detour-v1 ${CRANE_NAV2_UNITY_EXTRA_ARGS:-}"

exec "${root_dir}/Tools/Performance/run_land_nav2_fixture.sh"
