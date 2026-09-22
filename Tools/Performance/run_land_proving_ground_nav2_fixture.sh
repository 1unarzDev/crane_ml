#!/usr/bin/env bash
set -euo pipefail

root_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
layout="${CRANE_PROVING_GROUND_LAYOUT:-alternate-corridors-v1}"
catalog="${CRANE_PROVING_GROUND_CATALOG:-v1}"

export CRANE_SCENE="TurtleBot3 Warehouse Validation"
export CRANE_NAV2_COMMAND_FLAG=--crane-ros-differential-cmd-vel
export CRANE_NAV2_LIDAR_FRAME=base_scan
export CRANE_NAV2_PARAMS="${CRANE_NAV2_PARAMS:-${root_dir}/Tools/Performance/nav2_land_proving_ground_fixture.yaml}"
export CRANE_NAV2_GOAL_DISTANCE="${CRANE_NAV2_GOAL_DISTANCE:-18.0}"
export CRANE_NAV2_ACTION_DURATION="${CRANE_NAV2_ACTION_DURATION:-100}"
export CRANE_DURATION="${CRANE_DURATION:-110}"
export CRANE_BT_MAX_TRANSITIONS="${CRANE_BT_MAX_TRANSITIONS:-16384}"
export CRANE_LAND_SCENARIO_ID="${CRANE_LAND_SCENARIO_ID:-${layout}}"
export CRANE_NAV2_UNITY_EXTRA_ARGS="--crane-land-proving-ground-catalog ${catalog} --crane-land-proving-ground-layout ${layout} ${CRANE_NAV2_UNITY_EXTRA_ARGS:-}"

exec "${root_dir}/Tools/Performance/run_land_nav2_fixture.sh"
