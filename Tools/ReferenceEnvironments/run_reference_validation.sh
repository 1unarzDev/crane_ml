#!/usr/bin/env bash
set -euo pipefail

root_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
player="${CRANE_PLAYER:-${root_dir}/Builds/CRANE-Worker/CRANE.x86_64}"
result_root="${CRANE_REFERENCE_RESULT_ROOT:-${root_dir}/PerformanceResults/reference-environments}"
build_manifest="$(dirname "${player}")/crane-build-manifest.json"

usage() {
    cat <<'EOF'
Usage: run_reference_validation.sh TARGET [OUTPUT_JSON]

Targets:
  px4-walls
  px4-aruco
  px4-windy
  clearpath-pipeline
  f1tenth-spielberg
  turtlebot3-warehouse
  land-proving-ground

The selected scene must already be present in the player's crane-build-manifest.json.
Generated F1TENTH scenes require a worker built with --crane-extra-scene.
EOF
}

if [[ $# -lt 1 || $# -gt 2 ]]; then
    usage >&2
    exit 64
fi

target="$1"
case "${target}" in
    px4-walls)
        scene="PX4 Walls Validation"
        runner_args=(--crane-aerial-validation --crane-aerial-scene "${scene}")
        ;;
    px4-aruco)
        scene="PX4 ArUco Validation"
        runner_args=(--crane-aerial-validation --crane-aerial-scene "${scene}")
        ;;
    px4-windy)
        scene="PX4 Windy Validation"
        runner_args=(--crane-aerial-validation --crane-aerial-scene "${scene}")
        ;;
    clearpath-pipeline)
        scene="Clearpath Pipeline Validation"
        runner_args=(--crane-reference-validation --crane-reference-scene "${scene}")
        ;;
    f1tenth-spielberg)
        scene="F1TENTH Spielberg Validation"
        runner_args=(--crane-reference-validation --crane-reference-scene "${scene}")
        ;;
    turtlebot3-warehouse)
        scene="TurtleBot3 Warehouse Validation"
        warehouse_manifest="${root_dir}/Assets/Resources/ReferenceEnvironments/unity_turtlebot3_industrial_warehouse_v2.json"
        warehouse_hash="$(sha256sum "${warehouse_manifest}" | awk '{print $1}')"
        runner_args=(--crane-reference-validation --crane-reference-scene "${scene}"
            --crane-reference-environment-id crane-industrial-warehouse-v2
            --crane-reference-manifest-sha256 "${warehouse_hash}"
            --crane-reference-source-version 2.2.0
            --crane-reference-source-object-count 20)
        ;;
    land-proving-ground)
        scene="TurtleBot3 Warehouse Validation"
        layout="${CRANE_PROVING_GROUND_LAYOUT:-alternate-corridors-v1}"
        proving_manifest="${root_dir}/Assets/Resources/ReferenceEnvironments/crane_land_proving_ground_v1.json"
        proving_hash="$(sha256sum "${proving_manifest}" | awk '{print $1}')"
        proving_objects="$(python3 - "${proving_manifest}" "${layout}" <<'PY'
import json
import pathlib
import sys
manifest = json.loads(pathlib.Path(sys.argv[1]).read_text(encoding="utf-8"))
layout = next(value for value in manifest["layouts"] if value["id"] == sys.argv[2])
print(len(manifest["sharedBoxes"]) + len(layout["obstacles"]))
PY
)"
        runner_args=(--crane-reference-validation --crane-reference-scene "${scene}"
            --crane-reference-environment-id crane-land-proving-ground-v1
            --crane-reference-manifest-sha256 "${proving_hash}"
            --crane-reference-source-version 1.0.0
            --crane-reference-source-object-count "${proving_objects}"
            --crane-land-nav2 --crane-land-proving-ground-layout "${layout}")
        ;;
    -h|--help)
        usage
        exit 0
        ;;
    *)
        echo "Unknown reference-environment target: ${target}" >&2
        usage >&2
        exit 64
        ;;
esac

if [[ ! -x "${player}" ]]; then
    echo "CRANE player is missing or not executable: ${player}" >&2
    exit 66
fi
if [[ ! -f "${build_manifest}" ]]; then
    echo "Build manifest is missing: ${build_manifest}" >&2
    exit 66
fi

python3 - "${build_manifest}" "${scene}" <<'PY'
import json
import pathlib
import sys

manifest_path = pathlib.Path(sys.argv[1])
scene_name = sys.argv[2]
manifest = json.loads(manifest_path.read_text(encoding="utf-8"))
scene_names = {pathlib.PurePosixPath(path).stem for path in manifest.get("scenes", [])}
if scene_name not in scene_names:
    raise SystemExit(
        f"Scene {scene_name!r} is not in {manifest_path}. "
        "Rebuild the worker and pass generated scenes with --crane-extra-scene."
    )
PY

mkdir -p "${result_root}"
output="${2:-${result_root}/${target}.json}"
log="${output%.json}.log"
mkdir -p "$(dirname "${output}")"

exec "${player}" -batchmode -nographics -logFile "${log}" \
    --crane-profile train-cpu --crane-scene "${scene}" --crane-disable-ros \
    "${runner_args[@]}" --crane-output "${output}"
