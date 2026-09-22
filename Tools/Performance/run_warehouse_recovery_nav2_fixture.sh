#!/usr/bin/env bash
set -euo pipefail

root_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"

export CRANE_WAREHOUSE_SCENARIO_ID="${CRANE_WAREHOUSE_SCENARIO_ID:-warehouse-temporary-enclosure-recovery-v1}"
export CRANE_LAND_SCENARIO_ID="${CRANE_LAND_SCENARIO_ID:-${CRANE_WAREHOUSE_SCENARIO_ID}}"
export CRANE_NAV2_BT_XML="${root_dir}/Tools/Performance/nav2_warehouse_replanning_recovery.xml"
export CRANE_NAV2_ACTION_DURATION="${CRANE_NAV2_ACTION_DURATION:-100}"
export CRANE_DURATION="${CRANE_DURATION:-110}"
export CRANE_EXPECTED_NAV_STATUS="${CRANE_EXPECTED_NAV_STATUS:-succeeded}"

exec "${root_dir}/Tools/Performance/run_warehouse_nav2_fixture.sh"
