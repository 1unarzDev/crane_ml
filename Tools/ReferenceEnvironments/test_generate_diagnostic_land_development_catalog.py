import json
from pathlib import Path

from generate_diagnostic_land_development_catalog import build_catalog


ROOT = Path(__file__).resolve().parents[2]
V4 = ROOT / "Assets/Resources/ReferenceEnvironments/crane_land_proving_ground_v4.json"


def test_development_catalog_is_disjoint_and_never_confirmatory() -> None:
    catalog = build_catalog()
    v4 = json.loads(V4.read_text(encoding="utf-8"))

    assert catalog["environmentId"] == "crane-land-proving-ground-v5"
    assert catalog["generator"]["developmentOnly"] is True
    assert catalog["generator"]["studyAdmission"] == (
        "development-only-never-confirmatory-or-replication"
    )
    assert len(catalog["layouts"]) == 24
    assert {layout["studySplit"] for layout in catalog["layouts"]} == {
        "candidate-v2-development"
    }
    assert {layout["diagnosticMechanism"] for layout in catalog["layouts"]} == {
        "connected-detour",
        "nominal-clear-route",
    }
    assert not (
        {layout["id"] for layout in catalog["layouts"]}
        & {layout["id"] for layout in v4["layouts"]}
    )
    assert not (
        {layout["generatorSeed"] for layout in catalog["layouts"]}
        & {layout["generatorSeed"] for layout in v4["layouts"]}
    )


def test_development_catalog_generation_is_deterministic() -> None:
    assert build_catalog() == build_catalog()
