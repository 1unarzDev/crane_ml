#!/usr/bin/env python3
"""Generate fresh land layouts for the causal-restraint successor and replication.

The catalog adds physical configurations within the validated v6 proving-ground model. It does
not activate a semantic campaign; the parent study must separately bind method, endpoint,
schedule, alpha, and the exact discovery/replication subsets before capture.
"""

from __future__ import annotations

import argparse
import hashlib
import json
from pathlib import Path

from generate_diagnostic_land_catalog import generate, generate_stratum, validate


CATALOG_ID = "crane-land-proving-ground-v8"
GENERATOR_VERSION = "causal-restraint-reserved-catalog-v1"
DISCOVERY_SPLIT = "causal-restraint-discovery-reserve"
REPLICATION_SPLIT = "causal-restraint-replication-reserve"
COUNT_PER_GEOMETRY_AND_SPLIT = 30


def build_catalog() -> dict:
    base = generate()
    layouts = []
    layouts.extend(generate_stratum(DISCOVERY_SPLIT, "connected-detour", 30, 111000))
    layouts.extend(generate_stratum(DISCOVERY_SPLIT, "nominal-clear-route", 30, 112000))
    layouts.extend(generate_stratum(REPLICATION_SPLIT, "connected-detour", 30, 121000))
    layouts.extend(generate_stratum(REPLICATION_SPLIT, "nominal-clear-route", 30, 122000))
    catalog = {
        "schema": "crane-land-proving-ground-catalog-v1",
        "environmentId": CATALOG_ID,
        "generatorVersion": GENERATOR_VERSION,
        "generator": {
            "algorithm": "fixed-stratum Python random.Random parameter sampling",
            "developmentOnly": False,
            "studyAdmission": "causal-restraint-reserved-until-parent-campaign-activation",
            "semanticConfirmationActive": False,
            "sourceCatalogGeneratorVersion": base["generatorVersion"],
            "effectiveRadiusMeters": base["generator"]["effectiveRadiusMeters"],
            "validationGridResolutionMeters": base["generator"][
                "validationGridResolutionMeters"
            ],
            "discoveryReserveCount": 60,
            "replicationReserveCount": 60,
            "connectedDetourCountPerSplit": 30,
            "nominalClearRouteCountPerSplit": 30,
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
