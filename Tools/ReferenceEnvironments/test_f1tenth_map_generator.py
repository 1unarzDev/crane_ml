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
    json.dumps(result)
