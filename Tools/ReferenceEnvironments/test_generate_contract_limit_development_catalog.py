from generate_contract_limit_development_catalog import SPLIT, build_catalog


def test_contract_limit_catalog_is_fresh_bounded_and_development_only() -> None:
    catalog = build_catalog()
    layouts = catalog["layouts"]
    assert catalog["environmentId"] == "crane-land-proving-ground-v7"
    assert catalog["generator"]["developmentOnly"] is True
    assert len(layouts) == 16
    assert {item["studySplit"] for item in layouts} == {SPLIT}
    assert {item["diagnosticMechanism"] for item in layouts} == {
        "connected-detour", "nominal-clear-route"
    }
    assert len({item["id"] for item in layouts}) == 16
    assert len({item["seed"] for item in layouts}) == 16


def test_contract_limit_catalog_generation_is_deterministic() -> None:
    assert build_catalog() == build_catalog()
