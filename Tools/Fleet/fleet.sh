#!/usr/bin/env bash
# Fleet phase-1 commands. Every Unity process goes through heavy (admission) and our own flock.
#   Tools/Fleet/fleet.sh test                         EditMode tests (Crane.Fleet.Tests)
#   Tools/Fleet/fleet.sh run SEED [OUT_DIR]           seeded run in the editor (batchmode, -nographics)
#   Tools/Fleet/fleet.sh check [REPORT.json]          CAD-swap check in the editor
#   Tools/Fleet/fleet.sh build                        fleet player -> Builds/CRANE-Fleet/CRANE.x86_64
#   Tools/Fleet/fleet.sh player-run SEED [OUT_DIR]    seeded run in the player (-batchmode -nographics)
#   Tools/Fleet/fleet.sh player-check [REPORT.json]   CAD-swap check in the player
#   Tools/Fleet/fleet.sh still SEED [DIR]             stills (player, batchmode with GPU under xvfb-run)
# Outputs default under ~/.cache/crane-fleet/. Logs in ~/.cache/crane-fleet/logs/.
set -euo pipefail
root="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
cache="${CRANE_FLEET_CACHE:-$HOME/.cache/crane-fleet}"
heavy=/home/lunarz/astro_dock/src/mhseals_agents/tools/heavy
editor=/home/lunarz/Unity/Hub/Editor/6000.5.10f1/Editor/Unity
player="$root/Builds/CRANE-Fleet/CRANE.x86_64"
mkdir -p "$cache/logs" "$cache/runs" "$cache/stills" "$cache/check"
stamp=$(date +%Y%m%d-%H%M%S)
commit=$(git -C "$root" rev-parse HEAD)
dirty=$([[ -n "$(git -C "$root" status --porcelain --untracked-files=no)" ]] && echo true || echo false)

ed() {  # ed LOG args...
    local log=$1; shift
    flock "$cache/unity.lock" "$heavy" unity "$editor" -batchmode -nographics -projectPath "$root" -logFile "$log" "$@"
}
pl() {  # pl LOG args...  (player, no graphics)
    local log=$1; shift
    ( unset DISPLAY WAYLAND_DISPLAY; export SDL_VIDEODRIVER=dummy
      flock "$cache/unity.lock" "$heavy" unity "$player" -batchmode -nographics -logFile "$log" "$@" )
}

cmd=${1:?command}; shift || true
case "$cmd" in
    test)
        ed "$cache/logs/test-$stamp.log" -runTests -testPlatform EditMode -assemblyNames Crane.Fleet.Tests \
            -testResults "$cache/logs/test-$stamp.xml"
        grep -o 'total="[0-9]*" passed="[0-9]*" failed="[0-9]*"' "$cache/logs/test-$stamp.xml" | head -1 ;;
    run)
        seed=${1:?seed}; out=${2:-}
        ed "$cache/logs/run-$seed-$stamp.log" -quit -executeMethod Crane.Fleet.EditorTools.FleetCli.Run --crane-seed "$seed" \
            --crane-source-commit "$commit" --crane-source-dirty "$dirty" ${out:+--crane-fleet-out "$out"}
        grep -h "CRANE_FLEET_RUN_END\|CRANE_FLEET_FAILED" "$cache/logs/run-$seed-$stamp.log" ;;
    check)
        rep=${1:-$cache/check/cad_swap_check.json}
        ed "$cache/logs/check-$stamp.log" -quit -executeMethod Crane.Fleet.EditorTools.FleetCli.Check --crane-fleet-check "$rep" || true
        grep -h "CRANE_FLEET_CHECK" "$cache/logs/check-$stamp.log" ;;
    build)
        ed "$cache/logs/build-$stamp.log" -quit -executeMethod Crane.Fleet.EditorTools.FleetCli.BuildPlayer
        grep -h "CRANE_BUILD_COMPLETE\|CRANE_FLEET_FAILED" "$cache/logs/build-$stamp.log" ;;
    player-run)
        seed=${1:?seed}; out=${2:-}
        pl "$cache/logs/prun-$seed-$stamp.log" --crane-disable-ros --crane-disable-sitl --crane-fleet-run --crane-seed "$seed" \
            --crane-source-commit "$commit" --crane-source-dirty "$dirty" ${out:+--crane-fleet-out "$out"} || true
        grep -h "CRANE_FLEET_RUN_END\|CRANE_FLEET_FAILED\|CRANE_FLEET_EXIT" "$cache/logs/prun-$seed-$stamp.log" ;;
    player-check)
        rep=${1:-$cache/check/cad_swap_check_player.json}
        pl "$cache/logs/pcheck-$stamp.log" --crane-disable-ros --crane-disable-sitl --crane-fleet-check "$rep" || true
        grep -h "CRANE_FLEET_CHECK\|CRANE_FLEET_EXIT" "$cache/logs/pcheck-$stamp.log" ;;
    still)
        seed=${1:?seed}; dir=${2:-$cache/stills/seed$seed-$stamp}
        mkdir -p "$dir"
        ( unset WAYLAND_DISPLAY
          flock "$cache/unity.lock" xvfb-run -a -s "-screen 0 1280x720x24" "$heavy" unity "$player" -batchmode \
            -screen-fullscreen 0 -screen-width 320 -screen-height 180 -logFile "$dir/player.log" \
            --crane-disable-ros --crane-disable-sitl --crane-fleet-still "$dir" --crane-seed "$seed" \
            --crane-source-commit "$commit" --crane-source-dirty "$dirty" ) || true
        grep -h "CRANE_FLEET_STILL\|CRANE_FLEET_EXIT\|CRANE_FLEET_FAILED\|CRANE_FLEET_SPEC" "$dir/player.log" ;;
    *) echo "unknown command $cmd" >&2; exit 2 ;;
esac
