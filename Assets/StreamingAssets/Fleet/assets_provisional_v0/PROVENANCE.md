# assets_provisional_v0: PROVISIONAL generated geometry (not CAD)

- Source: `~/.cache/crane-fleet/assets_v0/` written by `/home/lunarz/crane_ml/Tools/Fleet/assets/gen_provisional.py`
  (asset pipeline v0, handoff `Docs/ResearchOrchestration/AGENT_HANDOFFS/fleet/asset_pipeline.md`; that code is uncommitted
  in crane_ml at copy time). Copied 2026-11-10 by the CRANE fleet integration (phase 2). Previews not copied.
- Contract: `crane.fleet_asset_manifest/v0` (`Tools/Fleet/assets/ASSET_MANIFEST_SPEC.md` in crane_ml). base_link = centre of the body
  underside; ground at z = -ground_clearance; right-handed x forward, y left, z up; OBJ in the part frame.
- Every manifest section carries `provenance: provisional`. Validator on this set: all four manifests PASS (`validation_report.json`).
- AprilTag textures: tag36h11 codes from AprilRobotics/apriltag (`tag36h11.c`, BSD-2-Clause), rendered by the pipeline's
  `apriltag36h11.py`. All geometry and materials are CRANE-project generated (same licence as this repository).
- Integrity: `SHA256SUMS` (sha256 of every file as copied).
- Runtime copy: `Tools/Fleet/sync_spec.sh` mirrors this folder to `Assets/StreamingAssets/Fleet/assets_provisional_v0/`, which the
  overlay's `visual.mesh_manifest` points at. Replace the folder (or the overlay path) to swap in CAD; then run the CAD-swap check.
