#!/usr/bin/env bash
set -euo pipefail
root_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
mode="${1:-headless}"
scenario="${2:-warehouse_nominal}"
case "$mode" in
  headless) export CRANE_NOGRAPHICS=1 CRANE_NAV2_PROFILE=train-cpu ;;
  interactive) export CRANE_NOGRAPHICS=0 CRANE_NAV2_PROFILE=interactive-high ;;
  *) echo 'Usage: run_campus_nav2_fixture.sh [headless|interactive] [scenario]' >&2; exit 2 ;;
esac
manifest="${CRANE_CAMPUS_MANIFEST:-$root_dir/Assets/Resources/ReferenceEnvironments/industrial_logistics_campus_v1.json}"
read -r goal_x goal_y seed expected < <(python3 - "$manifest" "$scenario" <<'PY'
import json,sys
m=json.load(open(sys.argv[1]));s=next(x for x in m['scenarios'] if x['id']==sys.argv[2]);print(s['goal'][1],-s['goal'][0],s['seed'],s['expectedOutcome'])
PY
)
export CRANE_RUN_ID="${CRANE_RUN_ID:-campus-${scenario}-$(date -u +%Y%m%dT%H%M%SZ)}"
export CRANE_RESULT_ROOT="${CRANE_RESULT_ROOT:-$root_dir/PerformanceResults/$CRANE_RUN_ID}"
mkdir -p "$CRANE_RESULT_ROOT"
export CRANE_PLAYER="${CRANE_PLAYER:-$root_dir/Builds/CRANE-Campus/CRANE.x86_64}"
export CRANE_SCENE='TurtleBot3 Warehouse Validation'
export CRANE_SEED_BASE="${CRANE_SEED_BASE:-$seed}"
export CRANE_ROS_PORT="${CRANE_ROS_PORT:-12086}"
export CRANE_ROS_DOMAIN_ID="${CRANE_ROS_DOMAIN_ID:-222}"
export CRANE_NAV2_PARAMS="$root_dir/Tools/Performance/nav2_campus_fixture.yaml"
export CRANE_NAV2_COMMAND_FLAG=--crane-ros-differential-cmd-vel
export CRANE_NAV2_GOAL_X="$goal_x" CRANE_NAV2_GOAL_Y="$goal_y"
export CRANE_NAV2_ACTION_DURATION="${CRANE_NAV2_ACTION_DURATION:-180}"
export CRANE_DURATION="${CRANE_DURATION:-200}"
export CRANE_WARMUP=3 CRANE_TIME_SCALE="${CRANE_TIME_SCALE:-1}"
export CRANE_SCREEN_WIDTH=1600 CRANE_SCREEN_HEIGHT=900
export CRANE_EXPECTED_NAV_STATUS="$expected"
export CRANE_REQUIRE_OCCUPIED_COSTMAP=1
export CRANE_BT_MAX_TRANSITIONS=16384
model_args=''
if [[ "${CRANE_CAMPUS_MODEL:-1}" == 1 ]]; then
  model_args='--crane-campus-model'
  if [[ "${CRANE_CAMPUS_CONTACT_WHEELS:-1}" == 1 ]]; then model_args+=' --crane-campus-contact-wheels'; fi
fi
export CRANE_NAV2_UNITY_EXTRA_ARGS="--crane-land-nav2 --crane-campus --crane-campus-manifest $manifest --crane-campus-scenario $scenario --crane-campus-seed $CRANE_SEED_BASE --crane-campus-output $CRANE_RESULT_ROOT $model_args --crane-ros-nav-child-frames base_scan --crane-disable-rgb --crane-disable-depth --crane-inspection-view oblique --crane-interactive-width 1600 --crane-interactive-height 900 ${CRANE_NAV2_UNITY_EXTRA_ARGS:-}"
cp "$manifest" "$CRANE_RESULT_ROOT/source-manifest.json"
cp "$CRANE_NAV2_PARAMS" "$CRANE_RESULT_ROOT/nav2-params.yaml"
env | sort | sed -n '/^CRANE_/p' > "$CRANE_RESULT_ROOT/launch-environment.txt"
observer_name="crane-campus-evidence-$CRANE_RUN_ID"
cleanup() { docker stop -t 5 "$observer_name" >/dev/null 2>&1 || true; docker rm "$observer_name" >/dev/null 2>&1 || true; }
trap cleanup EXIT
# Passive evidence collector starts before the action; scan, TF and acquisition stamps are retained.
docker run -d --name "$observer_name" --network host --ipc host -e ROS_DOMAIN_ID="$CRANE_ROS_DOMAIN_ID" \
  -v "$root_dir:/workspace/crane_sim:ro" -v "$CRANE_RESULT_ROOT:/results" "${CRANE_ROS_IMAGE:-lunarzdev/astro:cuda}" bash -lc \
  'source /opt/ros/jazzy/setup.bash; exec python3 /workspace/crane_sim/Tools/Campus/capture_ros.py --output /results/ros-evidence.jsonl.gz --seconds '"$((CRANE_DURATION+10))" \
  > "$CRANE_RESULT_ROOT/evidence-container-id"
"$root_dir/Tools/Performance/run_nav2_controller_fixture.sh"
