#!/usr/bin/env python3
"""Contract checks for the deterministic land proving-ground catalog."""

import json
from pathlib import Path


ROOT = Path(__file__).resolve().parents[2]
MANIFEST = (
    ROOT
    / "Assets/Resources/ReferenceEnvironments/crane_land_proving_ground_v1.json"
)


def load_manifest() -> dict:
    return json.loads(MANIFEST.read_text(encoding="utf-8"))


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
