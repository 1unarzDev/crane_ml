#!/usr/bin/env python3
"""Generate disjoint land layouts reserved for physical command-motion cohorts.

The catalog is materialized and hashable before collection.  Layout geometry supplies independent
scenario configurations; command-motion intervention schedules and evidence masks remain separate
campaign-level factors.  Nothing in this catalog declares a semantic-evaluation campaign active.
"""

from __future__ import annotations

import argparse
import hashlib
import json
from pathlib import Path

from generate_diagnostic_land_catalog import generate, generate_stratum, validate


CATALOG_ID = "crane-land-proving-ground-v6"
GENERATOR_VERSION = "command-motion-reserved-catalog-v1"
CONFIRMATION_SPLIT = "command-motion-confirmation-reserve"
REPLICATION_SPLIT = "command-motion-replication-reserve"
COUNT_PER_GEOMETRY_AND_SPLIT = 60


def build_catalog() -> dict:
    base = generate()
    layouts = []
    layouts.extend(generate_stratum(CONFIRMATION_SPLIT, "connected-detour", 60, 91000))
    layouts.extend(generate_stratum(CONFIRMATION_SPLIT, "nominal-clear-route", 60, 92000))
    layouts.extend(generate_stratum(REPLICATION_SPLIT, "connected-detour", 60, 101000))
    layouts.extend(generate_stratum(REPLICATION_SPLIT, "nominal-clear-route", 60, 102000))
    catalog = {
        "schema": "crane-land-proving-ground-catalog-v1",
        "environmentId": CATALOG_ID,
        "generatorVersion": GENERATOR_VERSION,
        "generator": {
            "algorithm": "fixed-stratum Python random.Random parameter sampling",
            "developmentOnly": False,
            "studyAdmission": "reserved-physical-evidence-only-until-separate-campaign-activation",
            "semanticConfirmationActive": False,
            "sourceCatalogGeneratorVersion": base["generatorVersion"],
            "effectiveRadiusMeters": base["generator"]["effectiveRadiusMeters"],
            "validationGridResolutionMeters": base["generator"]["validationGridResolutionMeters"],
            "confirmationReserveCount": 120,
            "replicationReserveCount": 120,
            "connectedDetourCountPerSplit": 60,
            "nominalClearRouteCountPerSplit": 60,
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
