"""Shared constants and document-walking helpers for crane.profile/v2."""
from __future__ import annotations

import re
from typing import Any, Iterator

PROFILE_SCHEMA = "crane.profile/v2"
MANIFEST_SCHEMA = "crane.dataset_manifest/v2"

# CONTRACTS.md labels in ascending order of claim strength.
LABEL_RANK = {"NOT_RUN": 0, "UNKNOWN": 0, "IMPLEMENTED": 1, "FIXTURE_VALIDATED": 2,
              "LIVE_SIM_VALIDATED": 3, "PHYSICALLY_CALIBRATED": 4}
LABELS = tuple(LABEL_RANK)
STATES = ("UNCALIBRATED_PRIOR", "SIM_TRUTH_RECOVERY", "PHYSICALLY_CALIBRATED", "FIXTURE_ONLY")
PROVENANCE = ("cited", "fitted", "prior", "derived", "fixtureOnly")
ORIGINS = ("physical", "sim_truth", "synthetic-fixture")
SPLITS = ("fit", "heldout", "none")
ID_STATUSES = ("IDENTIFIED", "RANK_DEFICIENT", "ILL_CONDITIONED", "CONFOUNDED_WITH", "COMBINATION_ONLY",
               "INSUFFICIENT_EXCITATION", "SINGLE_SITE", "NO_PHYSICAL_DATA", "ABSTAIN")
WITH_STATUSES = ("CONFOUNDED_WITH", "COMBINATION_ONLY")

# Top-level keys that are envelope metadata, never quantity sections.
RESERVED = frozenset({"schema", "kind", "domain", "version", "id", "frames", "calibrationState", "evidenceLabel",
                      "sources", "datasets", "components", "claimScope", "migratedFrom", "contentHash",
                      "mixLabel", "attributes", "structures", "notes"})
SHA256_RE = re.compile(r"^[0-9a-f]{64}$")
COMPONENT_RE = re.compile(r"^[A-Za-z0-9_.\-]+@[0-9a-f]{64}$")


def is_quantity(node: Any) -> bool:
    return isinstance(node, dict) and "value" in node


def iter_quantities(doc: dict) -> Iterator[tuple[str, dict]]:
    """Yield (dotted.path, quantity) for every quantity under non-reserved sections."""
    def walk(node: Any, path: str) -> Iterator[tuple[str, dict]]:
        if is_quantity(node):
            yield path, node
        elif isinstance(node, dict):
            for k in sorted(node):
                yield from walk(node[k], f"{path}.{k}" if path else k)
    for key in sorted(doc):
        if key not in RESERVED:
            yield from walk(doc[key], key)


def resolve_path(doc: dict, path: str) -> Any:
    node: Any = doc
    for part in path.split("."):
        if not isinstance(node, dict) or part not in node:
            return None
        node = node[part]
    return node


def flatten_numbers(value: Any) -> list:
    if isinstance(value, (list, tuple)):
        out: list = []
        for v in value:
            out.extend(flatten_numbers(v))
        return out
    return [value]


def is_number(x: Any) -> bool:
    import math
    return isinstance(x, (int, float)) and not isinstance(x, bool) and math.isfinite(x)


def label_of(text: str) -> str:
    """First whitespace token of a free-form legacy label."""
    return str(text).split()[0].strip("(:;,") if str(text).strip() else ""
