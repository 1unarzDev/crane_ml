#!/usr/bin/env bash
set -euo pipefail
root="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
trial="${1:-CAL-02}";step="${2:-0.01}";mode="${3:-headless}"
scenario=warehouse_nominal;duration=20
case "$trial" in
 CAL-01) duration=25;; CAL-02) duration=15;; CAL-03) duration=25;; CAL-04) duration=25;;
 CAL-05) scenario=proving_surface_transition;duration=35;; CAL-06) scenario=dock_threshold;duration=35;;
 CAL-07) scenario=dock_ramp;duration=55;; CAL-08) duration=90;; *) exit 2;;
esac
scenario="${CRANE_CALIBRATION_SCENARIO:-$scenario}";duration="${CRANE_CALIBRATION_DURATION:-$duration}"
export CRANE_PLAYER="${CRANE_PLAYER:-$root/Builds/CRANE-Campus/CRANE.x86_64}"
export CRANE_RESULT_ROOT="${CRANE_RESULT_ROOT:-$root/PerformanceResults/campus-$trial-$step-$mode}"
mkdir -p "$CRANE_RESULT_ROOT"
profile=train-cpu;export CRANE_NOGRAPHICS=1
if [[ "$mode" == interactive ]]; then profile=interactive-high;export CRANE_NOGRAPHICS=0 CRANE_SCREEN_WIDTH=1600 CRANE_SCREEN_HEIGHT=900; fi
export CRANE_SCENE='TurtleBot3 Warehouse Validation' CRANE_DURATION="$duration" CRANE_WARMUP=3
contact_args=''
if [[ "${CRANE_CAMPUS_CONTACT_WHEELS:-1}" == 1 ]]; then contact_args=--crane-campus-contact-wheels; fi
export CRANE_EXTRA_ARGS="--crane-profile $profile --crane-land-nav2 --crane-campus --crane-campus-model $contact_args --crane-campus-scenario $scenario --crane-campus-fixed-step $step --crane-campus-calibration $trial --crane-campus-output $CRANE_RESULT_ROOT --crane-interactive-width 1600 --crane-interactive-height 900 ${CRANE_CALIBRATION_EXTRA_ARGS:-}"
"$root/Tools/Performance/run_worker.sh" 0 > "$CRANE_RESULT_ROOT/launch.log" 2>&1
python3 "$root/Tools/Campus/analyze_calibration.py" "$CRANE_RESULT_ROOT" --trial "$trial"
