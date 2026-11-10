#!/usr/bin/env bash
# Re-import the shared engine-free libraries from crane_ml (read only) into CRANE's Unity copy, report NEW / CHANGED / GONE per file,
# and regenerate each SOURCE_SHA256.txt. CRANE-own files in the target folders (DockingProfileBuilder.cs, asmdefs, metas) are kept.
# Usage: Tools/Fleet/import_core_libs.sh [--dry-run] [CORE_CS_DIR]   (default /home/lunarz/crane_ml/Tools/Fleet/core_cs)
set -euo pipefail
dry=0; [[ "${1:-}" == "--dry-run" ]] && { dry=1; shift; }
src="${1:-/home/lunarz/crane_ml/Tools/Fleet/core_cs}"
root="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"; W="$root/Assets/Scripts/Fleet"
# upstream file -> CRANE file ("" = not imported: CRANE builds the docking profile from FleetSpec with DockingProfileBuilder)
map() {
  local f="$1"
  case "$f" in
    Crane.Fleet.Autonomy/*.cs|Crane.Fleet.Autonomy/*.json) echo "$W/Autonomy/${f#Crane.Fleet.Autonomy/}" ;;
    Crane.Fleet.Capability/*.cs) echo "$W/Capability/${f#Crane.Fleet.Capability/}" ;;
    Crane.Fleet.Perception/Evaluator/*.cs) echo "$W/PerceptionEvaluator/${f#Crane.Fleet.Perception/Evaluator/}" ;;
    Crane.Fleet.Perception/*.cs) echo "$W/Perception/${f#Crane.Fleet.Perception/}" ;;
    Crane.Fleet.Docking/Profile/ProfileLoader.cs) echo "" ;;
    Crane.Fleet.Docking/*.cs) echo "$W/Core/Docking/$(basename "$f")" ;;
  esac
}
cd "$src"
files=$( { find Crane.Fleet.Autonomy \( -name '*.cs' -o -name '*.json' \); find Crane.Fleet.Capability Crane.Fleet.Perception Crane.Fleet.Docking -name '*.cs'; } | grep -v -e '/bin/' -e '/obj/' | LC_ALL=C sort)
declare -A seen
for f in $files; do
  dst=$(map "$f"); [[ -z "$dst" ]] && { echo "SKIP    $f (not imported)"; continue; }
  seen["$dst"]=1
  if [[ ! -f "$dst" ]]; then echo "NEW     $f"; elif ! cmp -s "$f" "$dst"; then echo "CHANGED $f ($(diff "$f" "$dst" | grep -c '^[<>]') lines)"; else continue; fi
  (( dry )) || { mkdir -p "$(dirname "$dst")"; cp "$f" "$dst"; }
done
for d in Autonomy Capability Perception PerceptionEvaluator Core/Docking; do
  while IFS= read -r g; do
    [[ "$(basename "$g")" == DockingProfileBuilder.cs ]] && continue
    [[ -n "${seen[$g]:-}" ]] || echo "GONE    $g (in CRANE, not upstream)"
  done < <(find "$W/$d" -name '*.cs' | LC_ALL=C sort)
done
(( dry )) && exit 0
cp Crane.Fleet.Autonomy/README.md "$W/Autonomy/README_upstream.md"
cp Crane.Fleet.Perception/README.md "$W/Perception/README_upstream.md" 2>/dev/null || true
for c in Perception Docking; do [[ -f Crane.Fleet.$c/CHANGELOG.md ]] && cp Crane.Fleet.$c/CHANGELOG.md "$W/$( [[ $c == Docking ]] && echo Core/Docking || echo Perception )/CHANGELOG_upstream.md"; done
{ find Crane.Fleet.Autonomy \( -name '*.cs' -o -name '*.json' \); find Crane.Fleet.Capability -name '*.cs'; } | grep -v -e '/bin/' -e '/obj/' | LC_ALL=C sort | xargs sha256sum > "$W/Autonomy/SOURCE_SHA256.txt"
( cd Crane.Fleet.Perception && find . -name '*.cs' ! -path './Evaluator/*' | grep -v -e '/bin/' -e '/obj/' | sed 's#^\./##' | LC_ALL=C sort | xargs sha256sum ) > "$W/Perception/SOURCE_SHA256.txt"
find "$src/Crane.Fleet.Docking" -name '*.cs' ! -name ProfileLoader.cs | grep -v -e '/bin/' -e '/obj/' | LC_ALL=C sort | xargs sha256sum > "$W/Core/Docking/SOURCE_SHA256.txt"
echo "imported from $src (git $(git -C "$src" rev-parse --short HEAD 2>/dev/null || echo '?'))"
