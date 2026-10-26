#!/usr/bin/env python3
"""Fit a CRANE calibration profile from explicit fit-split observations.

The command never reads held-out samples.  With ``--update-manifest`` it records
the canonical artifact path/hash and the pre-profile dataset identity in a copy of
the manifest, which the held-out evaluator verifies before producing residuals.
"""

from __future__ import annotations

import argparse
import json
import os
from pathlib import Path

from calibration_profile import CalibrationProfileError, fit_profile, manifest_fit_sha256, write_profile


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("manifest", type=Path)
    parser.add_argument("--output", required=True, type=Path, help="canonical profile JSON path")
    parser.add_argument("--id", required=True, dest="profile_id")
    parser.add_argument("--version", required=True)
    parser.add_argument("--source-kind", choices=("physical", "synthetic-fixture"), default="physical")
    parser.add_argument("--update-manifest", type=Path, help="write a manifest with the generated profile reference")
    args = parser.parse_args()
    try:
        profile = fit_profile(args.manifest, args.profile_id, args.version, args.source_kind)
        digest = write_profile(profile, args.output)
        if args.update_manifest:
            manifest = json.loads(args.manifest.read_text())
            manifest["calibrationProfile"] = {
                "id": profile["id"],
                "version": profile["version"],
                "parameterArtifactSha256": digest,
                "artifactPath": os.path.relpath(args.output.resolve(), args.update_manifest.parent.resolve()),
                "fitManifestSha256": manifest_fit_sha256(manifest),
                "fitTrialIds": profile["fitProvenance"]["trialIds"],
            }
            args.update_manifest.parent.mkdir(parents=True, exist_ok=True)
            args.update_manifest.write_text(json.dumps(manifest, indent=2, sort_keys=True) + "\n")
        print(json.dumps({"schema": "crane-calibration-profile-result-v1", "profile": str(args.output.resolve()), "sha256": digest, "updatedManifest": str(args.update_manifest.resolve()) if args.update_manifest else None}, indent=2))
        return 0
    except (CalibrationProfileError, OSError, ValueError, KeyError) as exc:
        print(json.dumps({"schema": "crane-calibration-profile-result-v1", "state": "INVALID_PROFILE_INPUT", "error": str(exc)}, indent=2))
        return 2


if __name__ == "__main__":
    raise SystemExit(main())
