#!/usr/bin/env bash
set -euo pipefail

root_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"

export CRANE_WAREHOUSE_SCENARIO_ID="${CRANE_WAREHOUSE_SCENARIO_ID:-warehouse-cross-aisle-complete-blockage-v1}"
export CRANE_LAND_SCENARIO_ID="${CRANE_LAND_SCENARIO_ID:-${CRANE_WAREHOUSE_SCENARIO_ID}}"
# With the installed stock continuously-replanning BT, this is a retained negative-calibration
# contract: the goal remains active until the client deadline. A timeout here is not a Nav2 abort.
export CRANE_EXPECTED_NAV_STATUS="${CRANE_EXPECTED_NAV_STATUS:-timeout}"

exec "${root_dir}/Tools/Performance/run_warehouse_nav2_fixture.sh"
