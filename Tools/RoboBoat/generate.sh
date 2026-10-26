#!/usr/bin/env bash
# Generate course truth headless (no water, no scene, no sim slot), for preview, validate and diff.
#   generate.sh --family roboboat_2026 --seed N --out t.json [--mode corner=hi] [--set task.param=v]...
#   generate.sh --family roboboat_2026 --seeds 0:500 --out DIR       one file per seed
#   generate.sh --validate DIR --out verdicts.json                   C# verdicts on every truth in DIR
# UNITY_EDITOR overrides the editor binary; the project is this checkout.
# Without Unity: Tools/RoboBoat/cli (coursegen), same files and output (cli/README.md).
set -euo pipefail
here=$(cd "$(dirname "$0")" && pwd)
project=$(cd "$here/../.." && pwd)
editor=${UNITY_EDITOR:-$HOME/Unity/Hub/Editor/6000.5.10f1/Editor/Unity}
method=Sim.Course.Editor.CourseCli.Generate
pass=()
while (($#)); do
    case "$1" in
        --family) pass+=(--crane-course-family "$2"); shift 2 ;;
        --seed) pass+=(--crane-course-seed "$2"); shift 2 ;;
        --mode) pass+=(--crane-course-mode "$2"); shift 2 ;;
        --set) pass+=(--crane-course-set "$2"); shift 2 ;;
        --seeds|--out) pass+=("$1" "$(realpath -m "$2")"); [[ $1 == --seeds ]] && pass[-1]=$2; shift 2 ;;
        --validate) method=Sim.Course.Editor.CourseCli.Validate; pass+=(--in "$(realpath "$2")"); shift 2 ;;
        *) echo "unknown argument $1" >&2; exit 2 ;;
    esac
done
log=$(mktemp -t crane-course-XXXX.log)
status=0
"$editor" -batchmode -nographics -quit -projectPath "$project" -logFile "$log" -executeMethod "$method" "${pass[@]}" || status=$?
grep -E "CRANE_COURSE_(READY|BATCH|FAILED)" "$log" || { echo "no result; see $log" >&2; }
exit $status
