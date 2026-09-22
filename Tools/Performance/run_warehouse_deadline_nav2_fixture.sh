#!/usr/bin/env bash
set -euo pipefail

root_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"

# This policy keeps stock 1 Hz replanning/recoveries but terminates from an explicit BT task
# deadline at 70 steady-clock seconds. The client deadline is deliberately later and remains only a
# harness failsafe; if it fires, the run must still be called client cancellation.
export CRANE_NAV2_BT_XML="${root_dir}/Tools/Performance/nav2_warehouse_replanning_deadline.xml"
export CRANE_NAV2_ACTION_DURATION="${CRANE_NAV2_ACTION_DURATION:-80}"
export CRANE_DURATION="${CRANE_DURATION:-95}"
export CRANE_EXPECTED_NAV_STATUS="${CRANE_EXPECTED_NAV_STATUS:-succeeded}"

exec "${root_dir}/Tools/Performance/run_warehouse_nav2_fixture.sh"
