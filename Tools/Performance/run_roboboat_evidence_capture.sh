#!/usr/bin/env bash
set -euo pipefail

# The same Nav2/plant harness in both modes; interactive-high explicitly enables
# the rendered spectator window and read-only overlay. No second player/endpoint.
root_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
mode="${1:-interactive}"
case "$mode" in
    headless) profile=train-gpu; visual_args="" ;;
    interactive) profile=interactive-high ;;
    *) echo "Usage: $0 [headless|interactive]" >&2; exit 2 ;;
esac
export CRANE_RUN_ID="${CRANE_RUN_ID:-roboboat-evidence-${mode}-$(date -u +%Y%m%dT%H%M%SZ)}"
export CRANE_RESULT_ROOT="${CRANE_RESULT_ROOT:-${root_dir}/PerformanceResults/${CRANE_RUN_ID}}"
mkdir -p "$CRANE_RESULT_ROOT"
CRANE_RESULT_ROOT="$(realpath "$CRANE_RESULT_ROOT")"
export CRANE_PLAYER="${CRANE_PLAYER:-${root_dir}/Builds/CRANE-Evidence-ShoreFade/CRANE.x86_64}"
export CRANE_NAV2_PROFILE="$profile"
export CRANE_NAV2_PARAMS="${CRANE_NAV2_PARAMS:-${root_dir}/Tools/Performance/nav2_roboboat_first_dock_fixture.yaml}"
export CRANE_SCENE="Roboboat Course"
export CRANE_ROS_PORT="${CRANE_ROS_PORT:-12082}"
export CRANE_ROS_DOMAIN_ID="${CRANE_ROS_DOMAIN_ID:-220}"
export CRANE_NAV2_ACTION_MODE=navigate-to-pose
export CRANE_NAV2_GOAL_X="${CRANE_NAV2_GOAL_X:-0.8641434}"
export CRANE_NAV2_GOAL_Y="${CRANE_NAV2_GOAL_Y:--17.337}"
export CRANE_NAV2_GOAL_YAW="${CRANE_NAV2_GOAL_YAW:-1.5707963267948966}"
export CRANE_DOCKING_EVALUATOR=1
export CRANE_DURATION="${CRANE_DURATION:-180}"
export CRANE_WARMUP=3
# Half-speed simulation gives the ROS round-trip wall-time headroom without relaxing lag limits.
export CRANE_TIME_SCALE="${CRANE_TIME_SCALE:-0.5}"
export CRANE_NAV2_ACTION_DURATION="${CRANE_NAV2_ACTION_DURATION:-155}"
export CRANE_NAV2_POST_RESULT_DURATION=15
export CRANE_REQUIRE_OCCUPIED_COSTMAP=1
export CRANE_NOGRAPHICS=0
export CRANE_SCREEN_WIDTH=1600
export CRANE_SCREEN_HEIGHT=900
if [[ "$mode" == interactive ]]; then
    visual_args="--crane-evidence-overlay --crane-evidence-first-dock --crane-disable-rgb --crane-disable-lidar-debug-rays --crane-evidence-capture ${CRANE_RESULT_ROOT}/first-dock.png --crane-evidence-capture-delay 25 --crane-evidence-capture-2 ${CRANE_RESULT_ROOT}/mid.png --crane-evidence-capture-delay-2 30 --crane-evidence-capture-3 ${CRANE_RESULT_ROOT}/final.png --crane-evidence-capture-delay-3 145 --crane-interactive-width 1600 --crane-interactive-height 900"
fi
# Nav2 uses LiDAR /points. Avoid unrelated asynchronous RGB/depth readback backlog;
# the full-sensor diagnostic failures remain retained under the reproduction artifacts.
export CRANE_NAV2_UNITY_EXTRA_ARGS="--crane-disable-rgb --crane-disable-depth ${visual_args} ${CRANE_NAV2_UNITY_EXTRA_ARGS:-}"
cp "${BASH_SOURCE[0]}" "$CRANE_RESULT_ROOT/capture-launcher.sh"
# This is a measured worker in presentation mode, so the strict harness artifact
# and real Unity pixels describe one process, episode, and ROS clock.
env | sort | sed -n '/^CRANE_/p' > "$CRANE_RESULT_ROOT/launch-environment.txt"
cp "$CRANE_NAV2_PARAMS" "$CRANE_RESULT_ROOT/nav2-params.yaml"
cp "$root_dir/Tools/Performance/nav2_roboboat_distance_replanning.xml" "$CRANE_RESULT_ROOT/behavior-tree.xml"
cp "$(dirname "$CRANE_PLAYER")/crane-build-manifest.json" "$CRANE_RESULT_ROOT/build-manifest.json"
"${root_dir}/Tools/Performance/run_nav2_controller_fixture.sh"
