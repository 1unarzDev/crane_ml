#!/usr/bin/env python3
"""Validate a CRANE physical calibration manifest and held-out residuals."""

from __future__ import annotations

import argparse
import json
from pathlib import Path

from heldout_calibration import CalibrationValidationError, build_report


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("manifest", type=Path)
    parser.add_argument("--output", type=Path, help="Report path (default: <manifest>.heldout.json)")
    args = parser.parse_args()
    output = args.output or args.manifest.with_suffix(".heldout.json")
    try:
        report = build_report(args.manifest)
    except CalibrationValidationError as exc:
        report = {
            "schema": "crane-heldout-calibration-report-v1",
            "state": "INVALID_DATASET",
            "manifest": str(args.manifest.resolve()),
            "error": str(exc),
            "boundary": "No residual claim is made until the acquisition contract and split checks pass.",
        }
        output.write_text(json.dumps(report, indent=2) + "\n")
        print(json.dumps(report, indent=2))
        return 2
    output.write_text(json.dumps(report, indent=2) + "\n")
    print(json.dumps(report, indent=2))
    return 0 if report["state"] in ("HELDOUT_RESIDUAL_PASS", "RESIDUALS_COMPUTED", "SYNTHETIC_FIXTURE_RESIDUALS") else 1


if __name__ == "__main__":
    raise SystemExit(main())

