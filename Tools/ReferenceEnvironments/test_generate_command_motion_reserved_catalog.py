import json
from pathlib import Path

from generate_command_motion_reserved_catalog import (
    CONFIRMATION_SPLIT,
    REPLICATION_SPLIT,
    build_catalog,
)


ROOT = Path(__file__).resolve().parents[2]
V4 = ROOT / "Assets/Resources/ReferenceEnvironments/crane_land_proving_ground_v4.json"
V5 = ROOT / "Assets/Resources/ReferenceEnvironments/crane_land_proving_ground_v5.json"


def test_reserved_catalog_has_disjoint_confirmation_and_replication_capacity() -> None:
    catalog = build_catalog()
    prior = [json.loads(path.read_text(encoding="utf-8")) for path in (V4, V5)]
    layouts = catalog["layouts"]
    by_split = {
        split: [layout for layout in layouts if layout["studySplit"] == split]
        for split in (CONFIRMATION_SPLIT, REPLICATION_SPLIT)
    }
    assert catalog["environmentId"] == "crane-land-proving-ground-v6"
    assert catalog["generator"]["semanticConfirmationActive"] is False
    assert catalog["generator"]["studyAdmission"] == (
        "reserved-physical-evidence-only-until-separate-campaign-activation"
    )
    assert len(layouts) == 240
    assert {split: len(values) for split, values in by_split.items()} == {
        CONFIRMATION_SPLIT: 120,
        REPLICATION_SPLIT: 120,
    }
    for values in by_split.values():
        assert {layout["diagnosticMechanism"] for layout in values} == {
            "connected-detour", "nominal-clear-route"
        }
    assert {layout["id"] for layout in by_split[CONFIRMATION_SPLIT]}.isdisjoint(
        {layout["id"] for layout in by_split[REPLICATION_SPLIT]}
    )
    old_ids = {layout["id"] for item in prior for layout in item["layouts"]}
    old_seeds = {layout["generatorSeed"] for item in prior for layout in item["layouts"]}
    assert {layout["id"] for layout in layouts}.isdisjoint(old_ids)
    assert {layout["generatorSeed"] for layout in layouts}.isdisjoint(old_seeds)


def test_reserved_catalog_generation_is_deterministic() -> None:
    assert build_catalog() == build_catalog()
