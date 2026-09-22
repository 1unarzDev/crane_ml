#!/usr/bin/env bash
set -euo pipefail

root_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"

export CRANE_WAREHOUSE_SCENARIO_ID="${CRANE_WAREHOUSE_SCENARIO_ID:-warehouse-cross-aisle-complete-blockage-v1}"
export CRANE_LAND_SCENARIO_ID="${CRANE_LAND_SCENARIO_ID:-${CRANE_WAREHOUSE_SCENARIO_ID}}"
export CRANE_EXPECTED_NAV_STATUS="${CRANE_EXPECTED_NAV_STATUS:-aborted}"

exec "${root_dir}/Tools/Performance/run_warehouse_deadline_nav2_fixture.sh"
