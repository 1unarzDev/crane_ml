#!/usr/bin/env python3
"""Generate fresh development-only layouts for P-contract v4."""

from __future__ import annotations

import argparse
import hashlib
import json
from pathlib import Path

from generate_diagnostic_land_catalog import generate, generate_stratum, validate


CATALOG_ID = "crane-land-proving-ground-v7"
GENERATOR_VERSION = "contract-limit-development-catalog-v1"
SPLIT = "contract-limit-v4-development"


def build_catalog() -> dict:
    base = generate()
    layouts = []
    layouts.extend(generate_stratum(SPLIT, "connected-detour", 8, 111000))
    layouts.extend(generate_stratum(SPLIT, "nominal-clear-route", 8, 112000))
    catalog = {
        "schema": "crane-land-proving-ground-catalog-v1",
        "environmentId": CATALOG_ID,
        "generatorVersion": GENERATOR_VERSION,
        "generator": {
            "algorithm": "fixed-stratum Python random.Random parameter sampling",
            "developmentOnly": True,
            "studyAdmission": "p-contract-v4-development-only-never-confirmatory-or-replication",
            "sourceCatalogGeneratorVersion": base["generatorVersion"],
            "effectiveRadiusMeters": base["generator"]["effectiveRadiusMeters"],
            "validationGridResolutionMeters": base["generator"]["validationGridResolutionMeters"],
            "developmentCount": 16,
            "connectedDetourCount": 8,
            "nominalClearRouteCount": 8,
        },
        "dimensionsMeters": base["dimensionsMeters"],
        "sharedBoxes": base["sharedBoxes"],
        "layouts": layouts,
    }
    validate(catalog)
    canonical = json.dumps(catalog, indent=2, sort_keys=False) + "\n"
    catalog["generator"]["catalogContentSha256WithoutSelfHash"] = hashlib.sha256(
        canonical.encode("utf-8")
    ).hexdigest()
    return catalog


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    if args.output.exists():
        raise FileExistsError(f"refusing existing catalog: {args.output}")
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(build_catalog(), indent=2) + "\n", encoding="utf-8")


if __name__ == "__main__":
    main()
