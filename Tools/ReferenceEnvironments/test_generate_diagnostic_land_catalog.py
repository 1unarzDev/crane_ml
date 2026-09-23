from collections import Counter

from generate_diagnostic_land_catalog import connected, direct_route_blocked, generate


def test_catalog_has_predeclared_distinct_split_counts():
    catalog = generate()
    assert catalog["environmentId"] == "crane-land-proving-ground-v4"
    counts = Counter(
        (value["studySplit"], value["diagnosticMechanism"])
        for value in catalog["layouts"]
    )

    assert counts[("development-calibration", "failed-induction-calibration")] == 8
    assert counts[("development", "connected-detour")] == 8
    assert counts[("development", "nominal-clear-route")] == 8
    assert counts[("confirmatory", "connected-detour")] == 48
    assert counts[("confirmatory", "nominal-clear-route")] == 24
    assert len({value["id"] for value in catalog["layouts"]}) == 96
    assert catalog["generator"]["developmentCalibrationOnlyCount"] == 8
    assert catalog["generator"]["confirmatoryPrimaryDiagnosableCount"] == 48


def test_generated_mechanism_geometry_is_independently_validated():
    for value in generate()["layouts"]:
        mechanism = value["diagnosticMechanism"]
        if mechanism == "failed-induction-calibration":
            assert not connected(value)
            assert direct_route_blocked(value)
            assert value["studySplit"] == "development-calibration"
            assert value["studyAdmission"] == "excluded-failed-runtime-induction"
            assert value["offlineGeometryClassification"] == "canonical-raster-disconnected"
        elif mechanism == "connected-detour":
            assert connected(value)
            assert direct_route_blocked(value)
        else:
            assert mechanism == "nominal-clear-route"
            assert connected(value)
            assert not direct_route_blocked(value)


def test_generation_is_byte_stable():
    assert generate() == generate()


def test_shared_boundaries_form_a_closed_canonical_arena():
    boxes = {value["id"]: value for value in generate()["sharedBoxes"]}

    assert set(boxes) == {
        "diagnostic-floor",
        "diagnostic-west-boundary",
        "diagnostic-east-boundary",
        "diagnostic-south-boundary",
        "diagnostic-north-boundary",
    }
    assert boxes["diagnostic-west-boundary"]["size"] == [0.25, 2.0, 24.0]
    assert boxes["diagnostic-east-boundary"]["size"] == [0.25, 2.0, 24.0]
    assert boxes["diagnostic-south-boundary"]["size"] == [5.5, 2.0, 0.25]
    assert boxes["diagnostic-north-boundary"]["size"] == [5.5, 2.0, 0.25]
    assert boxes["diagnostic-south-boundary"]["center"][2] == -2.125
    assert boxes["diagnostic-north-boundary"]["center"][2] == 22.125
