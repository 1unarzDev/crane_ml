#!/usr/bin/env python3
"""Generate a development-only land catalog for prospective candidate iteration.

This namespace is intentionally disjoint from the sealed v4 confirmatory inventory.  Its layouts
may be inspected, tuned against, or discarded, but may never be promoted into confirmation or
replication by renaming them.
"""

from __future__ import annotations

import argparse
import hashlib
import json
from pathlib import Path

from generate_diagnostic_land_catalog import generate, generate_stratum, validate


CATALOG_ID = "crane-land-proving-ground-v5"
GENERATOR_VERSION = "diagnostic-land-development-catalog-v2"


def build_catalog() -> dict:
    base = generate()
    layouts = []
    layouts.extend(
        generate_stratum("candidate-v2-development", "connected-detour", 12, 81000)
    )
    layouts.extend(
        generate_stratum("candidate-v2-development", "nominal-clear-route", 12, 82000)
    )
    catalog = {
        "schema": "crane-land-proving-ground-catalog-v1",
        "environmentId": CATALOG_ID,
        "generatorVersion": GENERATOR_VERSION,
        "generator": {
            "algorithm": "fixed-stratum Python random.Random parameter sampling",
            "developmentOnly": True,
            "studyAdmission": "development-only-never-confirmatory-or-replication",
            "sourceCatalogGeneratorVersion": base["generatorVersion"],
            "effectiveRadiusMeters": base["generator"]["effectiveRadiusMeters"],
            "validationGridResolutionMeters": base["generator"][
                "validationGridResolutionMeters"
            ],
            "developmentCount": len(layouts),
            "connectedDetourCount": 12,
            "nominalClearRouteCount": 12,
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
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(build_catalog(), indent=2) + "\n", encoding="utf-8")


if __name__ == "__main__":
    main()
