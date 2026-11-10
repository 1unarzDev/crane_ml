"""tag36h11 pattern generator, built from the published AprilTag family definition.

Data: data/tag36h11.json (587 codes, 36-bit, bit layout) transcribed mechanically from
AprilRobotics/apriltag tag36h11.c (BSD-2-Clause). Image layout follows apriltag_to_image():
10x10 cells: 8x8 black border ring, 6x6 payload cells (a set code bit = white), and a
1-cell white quiet ring around the black border (as in the apriltag-imgs PNGs).

Verified in tests/test_apriltag.py against reference PNGs from AprilRobotics/apriltag-imgs
(ids 0, 7, 500, stored in tests/ref/) and by the family Hamming-distance property.

Usage: python apriltag36h11.py --ids 0 1 2 --out DIR [--scale 32]
Writes tag36h11_<id:05d>.png (nearest-neighbour upscaled, white quiet ring included).
"""
import argparse, json, os
import numpy as np

_DATA = json.load(open(os.path.join(os.path.dirname(os.path.abspath(__file__)), "data", "tag36h11.json")))
NCODES = len(_DATA["codes"])


def tag_cells(tag_id):
    """10x10 uint8 array, 255 = white, 0 = black. Row 0 is the top row of the image."""
    if not 0 <= tag_id < NCODES:
        raise ValueError(f"tag36h11 id must be in [0,{NCODES-1}]")
    code = _DATA["codes"][tag_id]
    n = _DATA["nbits"]
    im = np.full((10, 10), 255, np.uint8)
    im[1:9, 1:9] = 0
    for i in range(n):
        if code >> (n - i - 1) & 1:
            im[_DATA["bit_y"][i] + 1, _DATA["bit_x"][i] + 1] = 255
    return im


def tag_image(tag_id, scale=32):
    return np.kron(tag_cells(tag_id), np.ones((scale, scale), np.uint8))


def write_png(path, arr):
    from PIL import Image  # only needed for PNG output
    Image.fromarray(arr).save(path)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--ids", type=int, nargs="+", required=True)
    ap.add_argument("--out", required=True)
    ap.add_argument("--scale", type=int, default=32)
    a = ap.parse_args()
    os.makedirs(a.out, exist_ok=True)
    for t in a.ids:
        write_png(os.path.join(a.out, f"tag36h11_{t:05d}.png"), tag_image(t, a.scale))
        print("wrote", t)


if __name__ == "__main__":
    main()
