#!/usr/bin/env python3
"""Make a small repo copy of a full-rate crane-drone-log/1 run directory.

State, IMU, events, obstacles and manifest are copied unchanged. Camera frames are subsampled to --hz
(by acquisition stamp, keeping exact sim-clock stamps) and downscaled by an integer --scale factor
(RGB: box filter; depth: nearest valid sample so no invented depths at edges). Each camera keeps its
full-rate index as camera_fullrate/<name>.csv (the sidecar with every acquisition stamp) and the
subsampling is recorded in REPO_COPY.json. Pure standard library (zlib PNG, filter 0, like DronePng).
"""
import argparse
import csv
import json
import os
import shutil
import struct
import sys
import zlib

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from drone_log import read_png  # noqa: E402


def _chunk(tag, data):
    return struct.pack(">I", len(data)) + tag + data + struct.pack(">I", zlib.crc32(tag + data) & 0xFFFFFFFF)


def write_png(path, w, h, ch, depth, rows):
    color = {1: 0, 3: 2}[ch]
    raw = b"".join(b"\x00" + r for r in rows)
    data = b"\x89PNG\r\n\x1a\n" + _chunk(b"IHDR", struct.pack(">IIBBBBB", w, h, depth, color, 0, 0, 0))
    data += _chunk(b"IDAT", zlib.compress(raw, 9)) + _chunk(b"IEND", b"")
    with open(path, "wb") as f:
        f.write(data)


def downscale(w, h, ch, depth, data, k):
    nw, nh = w // k, h // k
    bpp = ch * depth // 8
    stride = w * bpp
    rows = []
    if depth == 8:
        for y in range(nh):
            out = bytearray(nw * ch)
            for x in range(nw):
                for c in range(ch):
                    s = 0
                    for dy in range(k):
                        base = (y * k + dy) * stride + x * k * ch + c
                        for dx in range(k):
                            s += data[base + dx * ch]
                    out[x * ch + c] = (s + k * k // 2) // (k * k)
            rows.append(bytes(out))
    else:  # 16-bit gray depth: nearest valid (non-zero) sample in the block, centre first
        vals = struct.unpack(">%dH" % (w * h), data)
        order = sorted(((dy, dx) for dy in range(k) for dx in range(k)), key=lambda p: (p[0] - k // 2) ** 2 + (p[1] - k // 2) ** 2)
        for y in range(nh):
            out = []
            for x in range(nw):
                v = 0
                for dy, dx in order:
                    v = vals[(y * k + dy) * w + x * k + dx]
                    if v:
                        break
                out.append(v)
            rows.append(struct.pack(">%dH" % nw, *out))
    return nw, nh, rows


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("src")
    ap.add_argument("dst")
    ap.add_argument("--hz", type=float, default=5.0)
    ap.add_argument("--scale", type=int, default=2)
    a = ap.parse_args(argv)
    if os.path.exists(os.path.join(a.dst, "manifest.json")):
        ap.error("destination already holds a run")
    os.makedirs(a.dst, exist_ok=True)
    for fn in ("manifest.json", "state.csv", "imu.csv", "events.jsonl", "obstacles.json"):
        if os.path.exists(os.path.join(a.src, fn)):
            shutil.copyfile(os.path.join(a.src, fn), os.path.join(a.dst, fn))
    period_ns = int(round(1e9 / a.hz))
    note = {"source_run": os.path.abspath(a.src), "camera_hz": a.hz, "scale": a.scale, "cameras": {},
            "note": "camera frames subsampled by acquisition stamp and downscaled; stamps unchanged; full-rate stamps in camera_fullrate/<name>.csv"}
    cdir = os.path.join(a.src, "camera")
    for fn in sorted(os.listdir(cdir)) if os.path.isdir(cdir) else []:
        if not fn.endswith(".csv"):
            continue
        name = fn[:-4]
        with open(os.path.join(cdir, fn), newline="") as f:
            rows = list(csv.DictReader(f))
        os.makedirs(os.path.join(a.dst, "camera", name), exist_ok=True)
        os.makedirs(os.path.join(a.dst, "camera_fullrate"), exist_ok=True)
        shutil.copyfile(os.path.join(cdir, fn), os.path.join(a.dst, "camera_fullrate", fn))
        out = []
        for r in rows:
            if int(r["acq_ns"]) % period_ns != 0:
                continue
            w, h, ch, depth, data = read_png(os.path.join(a.src, r["file"]))
            nw, nh, prow = downscale(w, h, ch, depth, data, a.scale)
            seq = len(out)
            rel = "camera/%s/%06d.png" % (name, seq)
            write_png(os.path.join(a.dst, rel), nw, nh, ch, depth, prow)
            out.append({"seq": seq, "acq_ns": r["acq_ns"], "complete_ns": r["complete_ns"], "width": nw, "height": nh,
                        "kind": r["kind"], "file": rel, "fullrate_seq": r["seq"]})
        with open(os.path.join(a.dst, "camera", fn), "w", newline="") as f:
            wr = csv.DictWriter(f, fieldnames=["seq", "acq_ns", "complete_ns", "width", "height", "kind", "file", "fullrate_seq"],
                                lineterminator="\n")
            wr.writeheader()
            wr.writerows(out)
        note["cameras"][name] = {"fullrate_frames": len(rows), "kept_frames": len(out)}
    with open(os.path.join(a.dst, "REPO_COPY.json"), "w") as f:
        json.dump(note, f, indent=2)
    print(json.dumps(note, indent=2))
    return 0


if __name__ == "__main__":
    sys.exit(main())
