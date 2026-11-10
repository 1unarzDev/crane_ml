#!/usr/bin/env bash
# Copy the shared fleet contracts into this worktree: Tools/Fleet/spec (reviewable copy) and
# Assets/StreamingAssets/Fleet (what the editor and the player read at runtime).
# Usage: Tools/Fleet/sync_spec.sh [SOURCE_SPEC_DIR]   (default /home/lunarz/crane_ml/Tools/Fleet/spec)
# The CRANE-local overlay (crane_fleet_sim_v0.json) lives only here and is copied to StreamingAssets too.
set -euo pipefail
root="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
src="${1:-/home/lunarz/crane_ml/Tools/Fleet/spec}"
for dst in "$root/Tools/Fleet/spec" "$root/Assets/StreamingAssets/Fleet"; do
    mkdir -p "$dst/scenarios"
    if [[ "$dst" != "$root/Tools/Fleet/spec" ]]; then
        cp "$root/Tools/Fleet/spec/crane_fleet_sim_v0.json" "$dst/"
    fi
    cp "$src/fleet_profile_v0.json" "$dst/"
    cp "$src/scenarios/"*.json "$dst/scenarios/"
done
# v1 profile (CAD v1, selectable: profiles.v1) when the source has it
[[ -f "$src/fleet_profile_v1.json" ]] && for dst in "$root/Tools/Fleet/spec" "$root/Assets/StreamingAssets/Fleet"; do cp "$src/fleet_profile_v1.json" "$dst/"; done
# v1 assets (OBJ/MTL/manifests/textures only; not in git, see .gitignore and Tools/Fleet/assets_v1_SHA256SUMS)
v1src="${CRANE_FLEET_ASSETS_V1:-$HOME/.cache/crane-fleet/assets_v1}"
if [[ -d "$v1src" ]]; then
    v1dst="$root/Assets/StreamingAssets/Fleet/assets_v1"
    rm -rf "$v1dst"; mkdir -p "$v1dst"
    for role in scout lifter cargo payloads; do
        mkdir -p "$v1dst/$role"; cp "$v1src/$role/"*.json "$v1src/$role/"*.obj "$v1src/$role/"*.mtl "$v1dst/$role/" 2>/dev/null || true
    done
    cp -r "$v1src/textures" "$v1dst/"; cp "$v1src/BUILD_INFO.json" "$v1dst/" 2>/dev/null || true
    ( cd "$v1dst" && find . -type f ! -name '*.meta' | LC_ALL=C sort | xargs sha256sum ) > "$root/Tools/Fleet/assets_v1_SHA256SUMS"
fi
# provisional asset set (meshes, manifests, textures) for the runtime
rm -rf "$root/Assets/StreamingAssets/Fleet/assets_provisional_v0"
cp -r "$root/Tools/Fleet/assets_provisional_v0" "$root/Assets/StreamingAssets/Fleet/assets_provisional_v0"
( cd "$root/Assets/StreamingAssets/Fleet" && sha256sum fleet_profile_v0.json scenarios/*.json crane_fleet_sim_v0.json )
