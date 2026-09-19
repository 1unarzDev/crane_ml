#!/usr/bin/env bash
set -euo pipefail

root_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"

export CRANE_SCENE="TurtleBot3 Warehouse Validation"
export CRANE_NAV2_COMMAND_FLAG=--crane-ros-differential-cmd-vel
export CRANE_NAV2_LIDAR_FRAME=base_scan

exec "${root_dir}/Tools/Performance/run_land_nav2_fixture.sh"
