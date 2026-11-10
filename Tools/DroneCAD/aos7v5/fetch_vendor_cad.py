#!/usr/bin/env python3
"""Downloads the vendor CAD / datasheets used for the AOS 7 V5 model into ~/.cache/research/vendor-cad/<part>/ (one empty dir per part).
Files are untrusted data: this script only downloads and hashes them, it never executes or imports them.  Downstream tools
(tools/inspect_step.py, tools/step_holes.py, tools/step_to_mesh.py) must be run as `python -I script path` with paths as arguments.
Only files whose licence allows it are redistributed (see VENDOR_CAD.md); `build.py` falls back to parametric parts when a file is absent.
usage: python3 fetch_vendor_cad.py [--check]      (--check: verify sha256 of files already present)"""
import hashlib, subprocess, sys, zipfile
from pathlib import Path

ROOT = Path.home() / ".cache" / "research" / "vendor-cad"
FILES = [
    # part, filename, url, sha256 (of the file as retrieved 2026-11-10), licence, redistributable in this repo
    ("oak_d_s2", "DM9098S2_enclosure.stp", "https://oak-files.fra1.cdn.digitaloceanspaces.com/OAK-D-S2/DM9098S2_enclosure.stp",
     "bedf072bc93267287445c80055966f3e615e0ebed4db5c15acef2dbdddd85cdf", "MIT (luxonis/depthai-hardware LICENSE; designs dual-licensed MIT or KiCad Libraries License)", True),
    ("oak_d_s2", "LICENSE.MIT", "https://raw.githubusercontent.com/luxonis/depthai-hardware/master/LICENSE", None, "MIT", True),
    ("cube_orange", "Cube.STEP.zip", "https://3175993181-files.gitbook.io/~/files/v0/b/gitbook-legacy-files/o/assets%2F-LUhw7cdLeWVORgnTA3i%2F-MM-egHyv1cmK1OIwmn3%2F-MM-lITizR4i-wJ3jx4t%2FCube.STEP.zip?alt=media",
     "a5a23d68f91ff285a90a0175c1759637de48fe2b5249e795f85dcad1bcb842b7", "CubePilot docs page, no licence stated (3D-printing / product 3D model page): NOT redistributed", False),
    ("mini_carrier", "MiniCarrier.STEP.zip", "https://3175993181-files.gitbook.io/~/files/v0/b/gitbook-legacy-files/o/assets%2F-LUhw7cdLeWVORgnTA3i%2F-MZHd3Ea8f-8Kobuogvh%2F-MZHeED3Eu-wn_aWmeEc%2FMini%20Carrier%20Board.STEP.zip?alt=media",
     "cae41103c19c95c9b054fa350ff2f2f9d0fa33a67352dbf8b71f6b4c96d20719", "CubePilot docs page, no licence stated: NOT redistributed", False),
    ("unitree_l2", "L2_manual.pdf", "https://oss-global-cdn.unitree.com/static/Unitree%204D%20LiDAR%20L2%20User%20Manual.pdf",
     "95a3e52ce5fa1cc9366095d646e6c6ff23de0fd23eddb1fff7bf5a698cbe76b1", "Unitree manual, (c) 2024 Unitree Robotics: NOT redistributed (dimensions used as facts)", False),
    ("holybro_esc", "Tekko32_F4_Metal_65A_spec.pdf", "https://cdn.shopifycdn.net/s/files/1/0604/5905/7341/files/Holybro_Tekko32_F4_Metal_4in1_65A_ESC_Spec_V1.8.pdf?v=1721116106",
     "a445f97652138146c3a045759daad1b06d4d40969d06d91fa520b2f355a43c40", "Holybro spec sheet: NOT redistributed (dimensions used as facts)", False),
]
MANUAL = {   # need a login or are unreachable; the user can drop the file into the named directory (the build never requires them)
    "jetson_orin_nano_devkit": ("NVIDIA Jetson Orin Nano Developer Kit carrier board STEP/3D model: https://developer.nvidia.com/embedded/downloads (NVIDIA developer login, filter 'Jetson Orin Nano', Mechanical / 3D model)", "NVIDIA licence: do not redistribute"),
    "avenger_motor": ("BrotherHobby Avenger motor datasheet/STEP for the exact model on the drone (no 2806.5 in the catalogue): https://www.brotherhobby.com/c/avenger-series_0089", "unknown"),
    "aos7_v5_frame": ("AOS 7 V5 plate/arm STEP or DXF: none published (https://www.aos-rc.com/designs/aos-7-v5 build resources list STL prints only); ask AOS RC", "unknown"),
}


def sha256(p):
    h = hashlib.sha256()
    with open(p, "rb") as f:
        for b in iter(lambda: f.read(1 << 20), b""):
            h.update(b)
    return h.hexdigest()


def main():
    check = "--check" in sys.argv
    bad = 0
    for part, name, url, digest, lic, _ in FILES:
        d = ROOT / part
        d.mkdir(parents=True, exist_ok=True)
        p = d / name
        if not p.exists() and not check:
            r = subprocess.run(["curl", "-sL", "-m", "180", "-A", "Mozilla/5.0", "-o", str(p), "-w", "%{http_code}", url], capture_output=True, text=True)
            print(f"download {part}/{name}: HTTP {r.stdout}")
        if not p.exists():
            print(f"MISSING {part}/{name}"); bad += 1; continue
        h = sha256(p)
        ok = digest is None or h == digest
        print(f"{'ok ' if ok else 'BAD'} {part}/{name} sha256={h[:16]} licence: {lic}")
        bad += 0 if ok else 1
    for part, (what, lic) in MANUAL.items():
        d = ROOT / part
        d.mkdir(parents=True, exist_ok=True)
        print(f"MANUAL {d}: {what} [{lic}] files present: {[q.name for q in d.iterdir()]}")
    sys.exit(1 if bad else 0)


if __name__ == "__main__":
    main()
