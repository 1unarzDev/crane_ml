#!/usr/bin/env bash
# Fleet phase-1 commands. Every Unity process goes through heavy (admission) and our own flock.
#   Tools/Fleet/fleet.sh test                         EditMode tests (Crane.Fleet.Tests)
#   Tools/Fleet/fleet.sh run SEED [OUT_DIR]           seeded run in the editor (batchmode, -nographics)
#   Tools/Fleet/fleet.sh check [REPORT.json]          CAD-swap check in the editor
#   Tools/Fleet/fleet.sh batch "check=R.json;run=1:DIR;run=2:DIR"   several jobs in one editor launch
#   Tools/Fleet/fleet.sh build                        fleet player -> Builds/CRANE-Fleet/CRANE.x86_64
#   Tools/Fleet/fleet.sh player-run SEED [OUT_DIR]    seeded run in the player (-batchmode -nographics)
#   Tools/Fleet/fleet.sh player-check [REPORT.json]   CAD-swap check in the player
#   Tools/Fleet/fleet.sh still SEED [DIR]             stills (player, batchmode with GPU under xvfb-run)
#   Tools/Fleet/fleet.sh demo SEED [DIR] | demo-player SEED [DIR] | demo-still SEED [DIR] [player args]
#   Tools/Fleet/fleet.sh jobs JOBS.json | jobs-player JOBS.json      paired-run harness (FleetHarness)
#   Tools/Fleet/fleet.sh request-run --scenario F --config profile --mission M.json --seed N --variant V [--failures X] [--out DIR] [--labels J]
# Outputs default under ~/.cache/crane-fleet/. Logs in ~/.cache/crane-fleet/logs/.
set -euo pipefail
root="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
cache="${CRANE_FLEET_CACHE:-$HOME/.cache/crane-fleet}"
heavy=/home/lunarz/astro_dock/src/mhseals_agents/tools/heavy
editor=/home/lunarz/Unity/Hub/Editor/6000.5.10f1/Editor/Unity
player="$root/Builds/CRANE-Fleet/CRANE.x86_64"
# players read the worktree spec (so data edits need no rebuild); override with CRANE_FLEET_SPEC
spec="${CRANE_FLEET_SPEC:-$root/Assets/StreamingAssets/Fleet}"
mkdir -p "$cache/logs" "$cache/runs" "$cache/stills" "$cache/check"
stamp=$(date +%Y%m%d-%H%M%S)
commit=$(git -C "$root" rev-parse HEAD)
dirty=$([[ -n "$(git -C "$root" status --porcelain --untracked-files=no)" ]] && echo true || echo false)

# Memory guard (coordinator, 2026-11-10): never launch Unity with less than CRANE_FLEET_MIN_MEM_GB available.
memwait() {
    local need=${CRANE_FLEET_MIN_MEM_GB:-12} avail held=0
    while true; do
        avail=$(awk '/MemAvailable/ {printf "%d", $2/1048576}' /proc/meminfo)
        # operator rule: at most one Unity process on the machine across all sessions
        if "$heavy" status 2>/dev/null | grep -E '^unity\.[0-9]+ ' | grep -vq ' free'; then
            (( ++held % 12 == 1 )) && echo "fleet.sh: waiting: a unity slot is held ($(date +%T))" >&2
            sleep 5; continue
        fi
        (( avail >= need )) && return 0
        (( ++held % 12 == 1 )) && echo "fleet.sh: waiting for memory (${avail} GB available < ${need} GB)" >&2
        sleep 5
    done
}

ed() {  # ed LOG args...
    local log=$1; shift
    # lock first, then check memory and every heavy slot INSIDE the lock (a long flock wait used to let another session in);
    # fd 9 is closed for the children so a stray Xvfb cannot inherit the lock.
    # wall-clock guard: an editor that hangs at startup (seen once, 2026-11-10 01:40) must not hold the machine's only Unity slot
    ( flock 9; memwait; "$heavy" unity timeout -k 30 "${CRANE_FLEET_EDITOR_TIMEOUT:-3000}" "$editor" -batchmode -nographics -projectPath "$root" -logFile "$log" "$@" 9>&- ) 9>"$cache/unity.lock"
}
pl() {  # pl LOG args...  (player, no graphics)
    local log=$1; shift
    ( unset DISPLAY WAYLAND_DISPLAY; export SDL_VIDEODRIVER=dummy
      flock 9; memwait; "$heavy" unity "$player" -batchmode -nographics -logFile "$log" --crane-fleet-spec "$spec" "$@" 9>&- ) 9>"$cache/unity.lock"
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
    batch)   # batch "check=REPORT;run=SEED:DIR;..."
        ed "$cache/logs/batch-$stamp.log" -quit -executeMethod Crane.Fleet.EditorTools.FleetCli.Batch --crane-fleet-batch "${1:?jobs}" \
            --crane-source-commit "$commit" --crane-source-dirty "$dirty" || true
        grep -h "CRANE_FLEET_RUN_END\|CRANE_FLEET_CHECK\|CRANE_FLEET_FAILED\|CRANE_FLEET_EXIT" "$cache/logs/batch-$stamp.log" ;;
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
          flock 9; memwait; xvfb-run -a -s "-screen 0 1280x720x24" "$heavy" unity "$player" -batchmode \
            -screen-fullscreen 0 -screen-width 320 -screen-height 180 -logFile "$dir/player.log" \
            --crane-disable-ros --crane-disable-sitl --crane-fleet-spec "$spec" --crane-fleet-still "$dir" --crane-seed "$seed" \
            --crane-source-commit "$commit" --crane-source-dirty "$dirty" 9>&- ) 9>"$cache/unity.lock" || true
        grep -h "CRANE_FLEET_STILL\|CRANE_FLEET_EXIT\|CRANE_FLEET_FAILED\|CRANE_FLEET_SPEC" "$dir/player.log" ;;
    demo)    # demo SEED [OUT_DIR]: demo mission in the editor (batchmode, -nographics), own physics scene
        seed=${1:?seed}; out=${2:-$cache/runs/fleet_warehouse_a_v0_demo_$seed}
        ed "$cache/logs/demo-$seed-$stamp.log" -quit -executeMethod Crane.Fleet.EditorTools.FleetCli.Batch --crane-fleet-mission demo \
            --crane-fleet-batch "run=$seed:$out" --crane-source-commit "$commit" --crane-source-dirty "$dirty" || true
        grep -h "CRANE_FLEET_RUN_END\|CRANE_FLEET_FAILED\|CRANE_FLEET_EXIT" "$cache/logs/demo-$seed-$stamp.log" ;;
    demo-player)   # demo-player SEED [OUT_DIR]: demo mission in the player, -nographics
        seed=${1:?seed}; out=${2:-$cache/runs-player/fleet_warehouse_a_v0_demo_$seed}
        pl "$cache/logs/pdemo-$seed-$stamp.log" --crane-disable-ros --crane-disable-sitl --crane-fleet-run --crane-fleet-mission demo --crane-seed "$seed" \
            --crane-source-commit "$commit" --crane-source-dirty "$dirty" --crane-fleet-out "$out" || true
        grep -h "CRANE_FLEET_RUN_END\|CRANE_FLEET_FAILED\|CRANE_FLEET_EXIT" "$cache/logs/pdemo-$seed-$stamp.log" ;;
    demo-still)    # demo-still SEED [DIR] [extra player args, e.g. --crane-fleet-clip --crane-fleet-still-scale 0.6]
        seed=${1:?seed}; dir=${2:-$cache/stills/demo_seed$seed}; shift 2 || shift $#
        mkdir -p "$dir"
        ( unset WAYLAND_DISPLAY
          flock 9; memwait; xvfb-run -a -s "-screen 0 1280x720x24" "$heavy" unity "$player" -batchmode \
            -screen-fullscreen 0 -screen-width 320 -screen-height 180 -logFile "$dir/player.log" \
            --crane-disable-ros --crane-disable-sitl --crane-fleet-spec "$spec" --crane-fleet-still "$dir" --crane-fleet-mission demo \
            --crane-seed "$seed" --crane-source-commit "$commit" --crane-source-dirty "$dirty" "$@" 9>&- ) 9>"$cache/unity.lock" || true
        grep -h "CRANE_FLEET_STILL \|CRANE_FLEET_EXIT\|CRANE_FLEET_FAILED\|CRANE_FLEET_CLIP\|CRANE_FLEET_RUN_END" "$dir/player.log" ;;
    autonomy-still)   # autonomy-still SEED [DIR] [MISSION.json] [extra player args]: closed-loop mission stills (GPU player)
        seed=${1:?seed}; dir=${2:-$cache/stills/autonomy_seed$seed}; mf=$(realpath "${3:-$root/Tools/Fleet/missions/autonomy_demo_v0.json}"); shift 3 || shift $#
        mkdir -p "$dir"
        ( unset WAYLAND_DISPLAY
          flock 9; memwait; xvfb-run -a -s "-screen 0 1280x720x24" "$heavy" unity "$player" -batchmode \
            -screen-fullscreen 0 -screen-width 320 -screen-height 180 -logFile "$dir/player.log" \
            --crane-disable-ros --crane-disable-sitl --crane-fleet-spec "$spec" --crane-fleet-still "$dir" --crane-fleet-mission autonomy \
            --crane-fleet-mission-file "$mf" --crane-seed "$seed" --crane-source-commit "$commit" --crane-source-dirty "$dirty" "$@" 9>&- ) 9>"$cache/unity.lock" || true
        grep -h "CRANE_FLEET_STILL \|CRANE_FLEET_EXIT\|CRANE_FLEET_FAILED\|CRANE_FLEET_CLIP\|CRANE_FLEET_RUN_END" "$dir/player.log" ;;
    request-run)   # request-run --mission M.json (--seed N | --seeds A-B) --variant V [--scenario F] [--config profile|GRAPH.json] [--failures X] [--out DIR] [--labels J.json] [--player]
        scen=""; cfg="profile"; mis=""; seeds=""; var=""; fail=""; out=""; lab=""; useplayer=0
        while [[ $# -gt 0 ]]; do case "$1" in
            --scenario) scen=$(realpath "$2"); shift 2 ;; --config) cfg="$2"; [[ "$cfg" != profile ]] && cfg=$(realpath "$2"); shift 2 ;;
            --mission) mis=$(realpath "$2"); shift 2 ;; --seed) seeds="$2"; shift 2 ;; --seeds) seeds="$2"; shift 2 ;; --variant) var="$2"; shift 2 ;;
            --failures) fail="$2"; shift 2 ;; --out) out=$(realpath -m "$2"); shift 2 ;; --labels) lab=$(realpath "$2"); shift 2 ;;
            --player) useplayer=1; shift ;;
            *) echo "request-run: unknown option $1" >&2; exit 2 ;; esac; done
        [[ -n "$mis" && -n "$seeds" && -n "$var" ]] || { echo "request-run needs --mission --seed(s) --variant" >&2; exit 2; }
        out=${out:-$cache/requests/${var}_${seeds}_$stamp}; mkdir -p "$out"
        python3 - "$out/job.json" "$out" "$scen" "$cfg" "$mis" "$seeds" "$var" "$fail" "$lab" <<'PY'
import json, sys
p, out, scen, cfg, mis, seeds, var, fail, lab = sys.argv[1:]
a, _, b = seeds.partition("-")
jobs = []
for s in range(int(a), int(b or a) + 1):
    job = {"id": f"{var}_{s}", "mission_file": mis, "seed": s, "variant": var, "failures": fail, "config": cfg}
    if scen: job["scenario"] = scen
    if lab: job["labels"] = json.load(open(lab))
    jobs.append(job)
json.dump({"out": out, "jobs": jobs}, open(p, "w"), indent=2)
PY
        if (( useplayer )); then "$0" jobs-player "$out/job.json"
        else
            ed "$cache/logs/request-$stamp.log" -quit -executeMethod Crane.Fleet.EditorTools.FleetCli.Jobs --crane-fleet-jobs "$out/job.json" || true
            grep -h "CRANE_FLEET_JOB\|CRANE_FLEET_FAILED\|CRANE_FLEET_EXIT" "$cache/logs/request-$stamp.log"
        fi
        echo "run directories: $out/runs/  summary: $out/summary.jsonl  replays: $out/replays/" ;;
    jobs)    # jobs JOBS.json: paired-run harness in the editor (headless)
        ed "$cache/logs/jobs-$stamp.log" -quit -executeMethod Crane.Fleet.EditorTools.FleetCli.Jobs --crane-fleet-jobs "$(realpath "${1:?jobs}")" || true
        grep -h "CRANE_FLEET_JOB\|CRANE_FLEET_FAILED\|CRANE_FLEET_EXIT" "$cache/logs/jobs-$stamp.log" ;;
    jobs-player)   # jobs-player JOBS.json: same in the player with a GPU (adds rendered replay frames for failed jobs)
        jf=$(realpath "${1:?jobs}")
        ( unset WAYLAND_DISPLAY
          flock 9; memwait; xvfb-run -a -s "-screen 0 1280x720x24" "$heavy" unity "$player" -batchmode \
            -screen-fullscreen 0 -screen-width 320 -screen-height 180 -logFile "$cache/logs/pjobs-$stamp.log" \
            --crane-disable-ros --crane-disable-sitl --crane-fleet-spec "$spec" --crane-fleet-jobs "$jf" \
            --crane-source-commit "$commit" --crane-source-dirty "$dirty" 9>&- ) 9>"$cache/unity.lock" || true
        grep -h "CRANE_FLEET_JOB\|CRANE_FLEET_STILL \|CRANE_FLEET_FAILED\|CRANE_FLEET_EXIT" "$cache/logs/pjobs-$stamp.log" ;;
    *) echo "unknown command $cmd" >&2; exit 2 ;;
esac
