#!/usr/bin/env bash
set -euo pipefail

root_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"

export CRANE_WAREHOUSE_SCENARIO_ID="${CRANE_WAREHOUSE_SCENARIO_ID:-warehouse-occupied-goal-no-path-v1}"
export CRANE_LAND_SCENARIO_ID="${CRANE_LAND_SCENARIO_ID:-${CRANE_WAREHOUSE_SCENARIO_ID}}"
export CRANE_NAV2_GOAL_DISTANCE="${CRANE_NAV2_GOAL_DISTANCE:-3.0}"
export CRANE_NAV2_ACTION_DURATION="${CRANE_NAV2_ACTION_DURATION:-45}"
export CRANE_DURATION="${CRANE_DURATION:-60}"
# The stock continuously-replanning BT remains active around this occupied goal until the client
# deadline. Preserve that behavior as calibration; timeout must not be relabeled as Nav2 failure.
export CRANE_EXPECTED_NAV_STATUS="${CRANE_EXPECTED_NAV_STATUS:-timeout}"

exec "${root_dir}/Tools/Performance/run_warehouse_nav2_fixture.sh"
