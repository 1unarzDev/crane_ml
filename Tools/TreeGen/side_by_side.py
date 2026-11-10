#!/usr/bin/env python3
# SPDX-License-Identifier: GPL-3.0-or-later
"""Compose labelled side-by-side PNGs of two Drone Forest captures (built-in trees vs TreeGen trees).

  fsrun <python> -I Tools/TreeGen/side_by_side.py --left DIR_builtin --right DIR_treegen --out DIR \
      --pairs 01_corridor_start_golden.png:corridor_start,03_canopy_up_golden.png:canopy_up,06_corridor_mid_dusk.png:corridor_mid

Writes OUT/unity_<name>.png (the TreeGen frame, max 1600 px wide) and OUT/sbs_<name>.png (built-in | TreeGen, max 1600 px).
"""
from __future__ import annotations

import argparse
import os

from PIL import Image, ImageDraw


def label(img, text):
    d = ImageDraw.Draw(img)
    w = 9 * len(text) + 16
    d.rectangle([8, 8, 8 + w, 34], fill=(0, 0, 0))
    d.text((16, 14), text, fill=(255, 255, 255))
    return img


def fit(img, max_w):
    if img.width <= max_w:
        return img
    return img.resize((max_w, round(img.height * max_w / img.width)), Image.LANCZOS)


def main(argv=None):
    ap = argparse.ArgumentParser()
    ap.add_argument("--left", required=True)
    ap.add_argument("--right", required=True)
    ap.add_argument("--out", required=True)
    ap.add_argument("--pairs", required=True)
    ap.add_argument("--max-w", type=int, default=1600)
    a = ap.parse_args(argv)
    os.makedirs(a.out, exist_ok=True)
    for item in a.pairs.split(","):
        fn, name = item.split(":")
        L = Image.open(os.path.join(a.left, fn)).convert("RGB")
        R = Image.open(os.path.join(a.right, fn)).convert("RGB")
        fit(R.copy(), a.max_w).save(os.path.join(a.out, f"unity_{name}.png"), optimize=True)
        h = min(L.height, R.height)
        canvas = Image.new("RGB", (L.width + R.width + 8, h), (255, 255, 255))
        canvas.paste(label(L.copy(), "built-in trees"), (0, 0))
        canvas.paste(label(R.copy(), "TreeGen trees"), (L.width + 8, 0))
        fit(canvas, a.max_w).save(os.path.join(a.out, f"sbs_{name}.png"), optimize=True)
        print("wrote", name)


if __name__ == "__main__":
    main()
