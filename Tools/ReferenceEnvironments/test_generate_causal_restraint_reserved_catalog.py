from generate_causal_restraint_reserved_catalog import (
    DISCOVERY_SPLIT,
    REPLICATION_SPLIT,
    build_catalog,
)


def test_causal_restraint_catalog_has_disjoint_balanced_reserves() -> None:
    catalog = build_catalog()
    layouts = catalog["layouts"]
    assert catalog["environmentId"] == "crane-land-proving-ground-v8"
    assert catalog["generator"]["developmentOnly"] is False
    assert catalog["generator"]["semanticConfirmationActive"] is False
    assert len(layouts) == 120
    assert {item["studySplit"] for item in layouts} == {
        DISCOVERY_SPLIT,
        REPLICATION_SPLIT,
    }
    for split in (DISCOVERY_SPLIT, REPLICATION_SPLIT):
        selected = [item for item in layouts if item["studySplit"] == split]
        assert len(selected) == 60
        assert sum(item["diagnosticMechanism"] == "connected-detour" for item in selected) == 30
        assert sum(item["diagnosticMechanism"] == "nominal-clear-route" for item in selected) == 30
    assert len({item["id"] for item in layouts}) == 120
    assert len({item["seed"] for item in layouts}) == 120


def test_causal_restraint_catalog_generation_is_deterministic() -> None:
    assert build_catalog() == build_catalog()
