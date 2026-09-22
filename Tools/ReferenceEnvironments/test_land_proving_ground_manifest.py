#!/usr/bin/env python3
"""Contract checks for the deterministic land proving-ground catalog."""

import json
from pathlib import Path


ROOT = Path(__file__).resolve().parents[2]
MANIFEST = (
    ROOT
    / "Assets/Resources/ReferenceEnvironments/crane_land_proving_ground_v1.json"
)
MANIFEST_V2 = (
    ROOT
    / "Assets/Resources/ReferenceEnvironments/crane_land_proving_ground_v2.json"
)
MANIFEST_V3 = (
    ROOT
    / "Assets/Resources/ReferenceEnvironments/crane_land_proving_ground_v3.json"
)
NAVIGATION_GATES = (
    ROOT
    / "Tools/ReferenceEnvironments/land_proving_ground_navigation_gates_v1.json"
)


def load_manifest() -> dict:
    return json.loads(MANIFEST.read_text(encoding="utf-8"))


def load_manifest_v2() -> dict:
    return json.loads(MANIFEST_V2.read_text(encoding="utf-8"))


def load_manifest_v3() -> dict:
    return json.loads(MANIFEST_V3.read_text(encoding="utf-8"))


def by_id(values: list[dict]) -> dict[str, dict]:
    return {value["id"]: value for value in values}


def x_extent(box: dict) -> tuple[float, float]:
    half = box["size"][0] * 0.5
    return box["center"][0] - half, box["center"][0] + half


def test_catalog_identity_and_required_topologies() -> None:
    manifest = load_manifest()
    assert manifest["schema"] == "crane-land-proving-ground-catalog-v1"
    assert manifest["environmentId"] == "crane-land-proving-ground-v1"
    assert manifest["generatorVersion"] == "1.0.0"
    assert manifest["dimensionsMeters"] == [8.0, 22.0]
    assert set(by_id(manifest["layouts"])) == {
        "staggered-obstacles-v1",
        "slalom-s-turn-v1",
        "offset-gates-v1",
        "narrow-doorway-v1",
        "u-trap-v1",
        "alternate-corridors-v1",
        "complete-blockage-v1",
        "dynamic-gate-v1",
    }


def test_behavioral_navigation_gates_cover_every_layout_without_changing_scene_identity() -> None:
    layouts = set(by_id(load_manifest()["layouts"]))
    gates = json.loads(NAVIGATION_GATES.read_text(encoding="utf-8"))
    assert gates["schema"] == "crane-land-proving-ground-navigation-gates-v1"
    assert gates["environmentId"] == "crane-land-proving-ground-v1"
    assert gates["trajectorySampleDeadbandMeters"] == 0.15
    assert set(gates["layouts"]) == layouts
    assert gates["layouts"]["slalom-s-turn-v1"]["minimumLateralDirectionChanges"] >= 3
    assert gates["layouts"]["dynamic-gate-v1"]["minimumRecoveryCount"] >= 1


def test_v2_preserves_v1_and_replaces_only_the_failed_slalom_topology() -> None:
    original = load_manifest()
    corrected = load_manifest_v2()
    assert original["environmentId"] == "crane-land-proving-ground-v1"
    assert corrected["schema"] == original["schema"]
    assert corrected["environmentId"] == "crane-land-proving-ground-v2"
    assert corrected["generatorVersion"] == "2.0.0"
    assert corrected["sharedBoxes"] == original["sharedBoxes"]
    assert [layout["id"] for layout in corrected["layouts"]] == ["slalom-s-turn-v2"]


def test_v2_slalom_forces_four_alternating_openings() -> None:
    layout = load_manifest_v2()["layouts"][0]
    obstacles = layout["obstacles"]
    assert len(obstacles) == 4
    assert [box["center"][2] for box in obstacles] == [4.0, 7.5, 11.0, 14.5]
    extents = [x_extent(box) for box in obstacles]
    assert extents == [(-4.0, 0.5), (-0.5, 4.0), (-4.0, 0.5), (-0.5, 4.0)]
    assert [box["center"][0] < 0 for box in obstacles] == [True, False, True, False]
    gates = json.loads((
        ROOT / "Tools/ReferenceEnvironments/land_proving_ground_navigation_gates_v2.json"
    ).read_text(encoding="utf-8"))
    assert gates["environmentId"] == load_manifest_v2()["environmentId"]
    assert set(gates["layouts"]) == {layout["id"]}
    assert gates["layouts"][layout["id"]]["minimumLateralDirectionChanges"] >= 3


def test_v3_preserves_prior_catalogs_and_adds_only_bounded_recovery_layout() -> None:
    original = load_manifest()
    corrected = load_manifest_v2()
    recovery = load_manifest_v3()
    assert recovery["schema"] == original["schema"] == corrected["schema"]
    assert recovery["environmentId"] == "crane-land-proving-ground-v3"
    assert recovery["generatorVersion"] == "3.0.0"
    assert recovery["sharedBoxes"] == original["sharedBoxes"] == corrected["sharedBoxes"]
    assert [layout["id"] for layout in recovery["layouts"]] == [
        "temporary-enclosure-recovery-v3"
    ]


def test_v3_recovery_layout_is_a_bounded_four_wall_enclosure() -> None:
    layout = load_manifest_v3()["layouts"][0]
    obstacles = by_id(layout["obstacles"])
    assert set(obstacles) == {
        "enclosure-north", "enclosure-south", "enclosure-west", "enclosure-east"
    }
    assert all(value["role"] == "controlled-temporary-enclosure"
               for value in obstacles.values())
    assert all(value["activeInitially"] is False for value in obstacles.values())
    assert {value["activationAfterSeconds"] for value in obstacles.values()} == {18.0}
    assert {value["removalAfterSeconds"] for value in obstacles.values()} == {34.0}

    north = obstacles["enclosure-north"]
    south = obstacles["enclosure-south"]
    west = obstacles["enclosure-west"]
    east = obstacles["enclosure-east"]
    assert north["center"][2] > south["center"][2]
    assert west["center"][0] < 0 < east["center"][0]
    assert north["size"][0] == south["size"][0] == 3.0
    assert west["size"][2] == east["size"][2] == 2.15

    gates = json.loads((
        ROOT / "Tools/ReferenceEnvironments/land_proving_ground_navigation_gates_v3.json"
    ).read_text(encoding="utf-8"))
    assert gates["environmentId"] == load_manifest_v3()["environmentId"]
    assert gates["layouts"][layout["id"]]["minimumRecoveryCount"] >= 1


def test_each_layout_has_reproducible_route_and_unique_semantics() -> None:
    manifest = load_manifest()
    shared = manifest["sharedBoxes"]
    for layout in manifest["layouts"]:
        assert layout["seed"] >= 0
        assert layout["robot"] == "TurtleBot3 Waffle-class"
        assert len(layout["start"]) == len(layout["goal"]) == 3
        assert layout["start"] != layout["goal"]
        assert layout["approximateLengthMeters"] > 0
        assert layout["expectedChallenge"]
        assert layout["expectedBroadOutcome"]
        boxes = shared + layout["obstacles"]
        ids = [box["id"] for box in boxes]
        assert len(ids) == len(set(ids))
        obstacle_ids = {box["id"] for box in layout["obstacles"]}
        assert set(layout["relevantObstacles"]) <= obstacle_ids
        for box in boxes:
            assert len(box["center"]) == len(box["size"]) == len(box["color"]) == 3
            assert all(value > 0 for value in box["size"])


def test_offset_gates_and_narrow_doorway_preserve_declared_openings() -> None:
    layouts = by_id(load_manifest()["layouts"])
    offset = by_id(layouts["offset-gates-v1"]["obstacles"])
    first_gap = x_extent(offset["gate-1-east"])[0] - x_extent(offset["gate-1-west"])[1]
    second_gap = x_extent(offset["gate-2-east"])[0] - x_extent(offset["gate-2-west"])[1]
    assert abs(first_gap - 1.4) < 1e-9
    assert abs(second_gap - 1.4) < 1e-9

    doorway = by_id(layouts["narrow-doorway-v1"]["obstacles"])
    doorway_gap = (
        x_extent(doorway["doorway-east-jamb"])[0]
        - x_extent(doorway["doorway-west-jamb"])[1]
    )
    assert abs(doorway_gap - 1.0) < 1e-9


def test_complete_blockage_spans_the_full_canonical_width() -> None:
    layouts = by_id(load_manifest()["layouts"])
    wall = layouts["complete-blockage-v1"]["obstacles"][0]
    assert wall["role"] == "complete-blockage"
    assert x_extent(wall) == (-4.0, 4.0)
    assert layouts["complete-blockage-v1"]["alternatives"] == []


def test_dynamic_gate_has_one_bounded_simulation_time_intervention() -> None:
    layouts = by_id(load_manifest()["layouts"])
    obstacles = layouts["dynamic-gate-v1"]["obstacles"]
    dynamic = [value for value in obstacles if not value["activeInitially"]]
    assert len(dynamic) == 1
    assert dynamic[0]["id"] == "dynamic-gate-panel"
    assert dynamic[0]["activationAfterSeconds"] == 20.0
    assert dynamic[0]["removalAfterSeconds"] == 45.0


def test_u_trap_retains_two_exterior_routes() -> None:
    layouts = by_id(load_manifest()["layouts"])
    layout = layouts["u-trap-v1"]
    obstacles = by_id(layout["obstacles"])
    west_outer = x_extent(obstacles["u-trap-west"])[0]
    east_outer = x_extent(obstacles["u-trap-east"])[1]
    assert west_outer - (-4.0) >= 1.5
    assert 4.0 - east_outer >= 1.5
    assert layout["alternatives"] == ["west exterior", "east exterior"]
