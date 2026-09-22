#!/usr/bin/env bash
set -euo pipefail

root_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"

export CRANE_SCENE="TurtleBot3 Warehouse Validation"
export CRANE_NAV2_PARAMS="${root_dir}/Tools/Performance/nav2_warehouse_fixture.yaml"
export CRANE_NAV2_COMMAND_FLAG=--crane-ros-differential-cmd-vel
export CRANE_NAV2_LIDAR_FRAME=base_scan
export CRANE_NAV2_GOAL_DISTANCE="${CRANE_NAV2_GOAL_DISTANCE:-13.0}"
# The declared detour is approximately 16.5 m. At the Waffle-class 0.26 m/s physical limit its
# no-stop lower bound is 63.5 s, so a 60 s deadline cannot test successful completion.
export CRANE_NAV2_ACTION_DURATION="${CRANE_NAV2_ACTION_DURATION:-75}"
export CRANE_DURATION="${CRANE_DURATION:-90}"
# Ecological recovery runs can approach the shared 4,096-transition default. Keep the generic
# default stable for controlled/frozen fixtures while providing bounded headroom here.
export CRANE_BT_MAX_TRANSITIONS="${CRANE_BT_MAX_TRANSITIONS:-16384}"
warehouse_scenario_id="${CRANE_WAREHOUSE_SCENARIO_ID:-}"
land_scenario_id="${CRANE_LAND_SCENARIO_ID:-${warehouse_scenario_id:-warehouse-cross-aisle-detour-v1}}"
warehouse_scenario_args=""
if [[ -n "${warehouse_scenario_id}" ]]; then
    warehouse_scenario_args="--crane-warehouse-scenario-id ${warehouse_scenario_id}"
fi
export CRANE_NAV2_UNITY_EXTRA_ARGS="--crane-preserve-reference-environment \
--crane-land-scenario-id ${land_scenario_id} ${warehouse_scenario_args} \
${CRANE_NAV2_UNITY_EXTRA_ARGS:-}"

exec "${root_dir}/Tools/Performance/run_land_nav2_fixture.sh"
