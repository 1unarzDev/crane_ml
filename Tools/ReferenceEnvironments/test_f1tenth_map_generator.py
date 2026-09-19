import json
from pathlib import Path

from PIL import Image

from f1tenth_map_generator import generate


def test_ros_origin_flip_hashes_and_merged_boundaries(tmp_path: Path) -> None:
    # Top PNG row is free. Two adjacent occupied cells in the bottom row become one rectangle
    # whose outer boundary has four merged segments.
    image = Image.new("L", (3, 2), 255)
    image.putpixel((0, 1), 0)
    image.putpixel((1, 1), 0)
    image.save(tmp_path / "track.png")
    (tmp_path / "track.yaml").write_text(
        "image: track.png\nresolution: 0.5\norigin: [1.0, 2.0, 0.0]\n"
        "negate: 0\noccupied_thresh: 0.65\nfree_thresh: 0.196\n",
        encoding="utf-8",
    )

    result = generate(tmp_path / "track.yaml", wall_height=0.8)

    assert result["map"]["occupiedCellCount"] == 2
    assert len(result["canonicalCollision"]["segments"]) == 4
    assert result["map"]["rosCorners"] == [
        (1.0, 2.0), (2.5, 2.0), (2.5, 3.0), (1.0, 3.0)
    ]
    assert len(result["source"]["image"]["sha256"]) == 64
    assert result["canonicalCollision"]["sourceBoundaryEdgeCount"] == 6
    assert result["canonicalCollision"]["contourCount"] == 1
    json.dumps(result)


def test_contour_simplification_and_centerline_provenance(tmp_path: Path) -> None:
    image = Image.new("L", (20, 20), 255)
    for x in range(2, 18):
        for y in range(2, 18):
            image.putpixel((x, y), 0)
    image.save(tmp_path / "track.png")
    (tmp_path / "track.yaml").write_text(
        "image: track.png\nresolution: 0.1\norigin: [1.0, 2.0, 0.2]\n"
        "negate: 0\noccupied_thresh: 0.65\nfree_thresh: 0.196\n",
        encoding="utf-8",
    )
    (tmp_path / "centerline.csv").write_text(
        "# x_m, y_m\n1.5, 2.5\n1.8, 2.7\n", encoding="utf-8")

    result = generate(
        tmp_path / "track.yaml", simplification_tolerance=0.1,
        environment_id="f1tenth-test-v1", upstream_project="test/maps",
        upstream_version="abc123", centerline_path=tmp_path / "centerline.csv")

    collision = result["canonicalCollision"]
    assert collision["sourceBoundaryEdgeCount"] == 64
    assert len(collision["segments"]) == 4
    assert collision["floor"]["semanticId"] == "track-floor"
    assert result["environmentId"] == "f1tenth-test-v1"
    assert result["source"]["upstreamVersion"] == "abc123"
    assert len(result["source"]["centerline"]["sha256"]) == 64
    assert result["robotSpawn"]["ros"]["position"] == [1.5, 2.5]
