"""Top-down previews of scenes (PIL only), optionally with fastsim unit traces from a run's fleet.json.

    python3 -m scenes.render --all [--runs DIR]       # previews/<family>.jpg for every family (train seed 1) + index sheet
    python3 -m scenes.render --family F --split S --seed N [--fleet RUN/fleet.json] --out FILE.jpg

Every file is checked to be at most 300 KB.
"""
import argparse
import json
import math
from pathlib import Path

from PIL import Image, ImageDraw, ImageFont

from .common import HERE, door_frame, lower
from .families import FAMILIES, GENERATORS, generate
from . import catalog

PREVIEWS = HERE / "previews"
PX = 64          # pixels per metre
PAD = 28
HEAD = 64
FLOOR = {"hard_tile": (236, 236, 230), "carpet": (196, 170, 210), "rubber": (120, 120, 128), "polished": (188, 226, 240)}
ZONE = {"staging": (120, 170, 230), "loading": (240, 190, 90), "assembly": (170, 210, 140), "delivery": (110, 200, 120),
        "charging": (250, 220, 60), "slot": (230, 120, 120), "parking": (180, 180, 180), "aisle": (200, 200, 250)}
UNIT = {"carrier": (40, 110, 200), "carrier_v3": (60, 130, 210), "diff_carrier": (200, 90, 40), "lifter": (150, 60, 170),
        "scout": (30, 150, 120), "pair_chain": (40, 110, 200), "pair_side": (40, 110, 200), "l3_carrier": (20, 80, 170)}
CAM = {"working": (30, 160, 60), "occluded": (220, 140, 20), "disabled": (200, 40, 40)}


def font(size):
    try:
        return ImageFont.load_default(size=size)
    except TypeError:
        return ImageFont.load_default()


class Canvas:
    def __init__(self, X, Y):
        self.X, self.Y = X, Y
        self.w, self.h = int(X * PX + 2 * PAD), int(Y * PX + 2 * PAD + HEAD)
        self.img = Image.new("RGB", (self.w, self.h), (250, 250, 248))
        self.d = ImageDraw.Draw(self.img, "RGBA")

    def p(self, x, y):
        return (PAD + x * PX, HEAD + PAD + (self.Y - y) * PX)

    def poly(self, pts, fill=None, outline=None, width=1):
        self.d.polygon([self.p(*q) for q in pts], fill=fill, outline=outline, width=width)

    def rect(self, r, **kw):
        self.poly([(r[0], r[1]), (r[2], r[1]), (r[2], r[3]), (r[0], r[3])], **kw)

    def text(self, x, y, s, size=11, fill=(30, 30, 30), anchor="mm"):
        self.d.text(self.p(x, y), s, fill=fill, font=font(size), anchor=anchor)


def rect_pts(x, y, yaw, lx, ly):
    c, s = math.cos(yaw), math.sin(yaw)
    return [(x + c * u - s * v, y + s * u + c * v) for u, v in ((lx / 2, ly / 2), (lx / 2, -ly / 2), (-lx / 2, -ly / 2), (-lx / 2, ly / 2))]


def draw(scn, task, fleet=None, plan=None):
    X, Y = scn["arena"]["size_xy"]
    cv = Canvas(X, Y)
    cv.rect((0, 0, X, Y), fill=FLOOR[scn["arena"]["floor"]])
    for p in scn["floor_patches"]:
        cv.rect(p["rect"], fill=FLOOR[p["friction_class"]] + (255,))
        cv.text((p["rect"][0] + p["rect"][2]) / 2, p["rect"][3] - 0.2, p["friction_class"], 11, (70, 40, 90))
    for z in scn["zones"]:
        c = ZONE.get(z["kind"], (200, 200, 200))
        cv.rect(z["rect"], fill=c + (70,), outline=c + (255,), width=2)
        if z["kind"] != "aisle":
            cv.text((z["rect"][0] + z["rect"][2]) / 2, (z["rect"][1] + z["rect"][3]) / 2, z["id"], 10, (40, 40, 40))
    for s in lower(scn):
        pts = s.footprint()
        if s.kind == "outer_wall":
            cv.poly(pts, fill=(70, 70, 75))
        elif s.kind == "threshold":
            cv.poly(pts, fill=(250, 200, 0, 255))
        elif s.drive_block:
            col = (90, 90, 96) if s.kind in ("wall",) else (140, 120, 100) if s.kind in ("shelf", "rack") else \
                (160, 140, 110) if s.kind == "pallet" else (120, 110, 130)
            cv.poly(pts, fill=col + (255,), outline=(40, 40, 40))
        else:
            cv.poly(pts, fill=(80, 80, 200, 40), outline=(60, 60, 160, 200), width=1)   # elevated: lintel, banner
    for d in scn["doors"]:
        cx, cy, normal, w = door_frame(scn, d)
        a = (cx, cy - d["width"] / 2) if normal == "x" else (cx - d["width"] / 2, cy)
        b = (cx, cy + d["width"] / 2) if normal == "x" else (cx + d["width"] / 2, cy)
        cv.d.line([cv.p(*a), cv.p(*b)], fill=(0, 160, 200), width=3)
        tx, ty = (cx + 0.32, cy) if normal == "x" else (cx, cy + 0.3)
        cv.text(tx, ty, "%.2f" % d["width"], 10, (0, 110, 150))
    for t in scn["tags"]:
        x, y, _ = t["pose"]
        cv.rect((x - 0.05, y - 0.05, x + 0.05, y + 0.05), fill=(0, 0, 0))
    for c in scn["overhead_cameras"]:
        x, y, _ = c["position"]
        col = CAM[c.get("mode_declared", c["mode"])]
        cv.poly([(x, y + 0.16), (x - 0.14, y - 0.1), (x + 0.14, y - 0.1)], fill=col)
    for p in scn["payloads"]:
        if p["spawn"].get("on_unit"):
            continue
        x, y, yaw = p["spawn"]["pose"]
        cv.poly(rect_pts(x, y, yaw, p["size_lwh"][0], p["size_lwh"][1]), fill=(240, 120, 20), outline=(120, 50, 0))
        if not p["spawn"].get("pose_known", True):
            cv.text(x, y + 0.32, "hidden", 10, (160, 60, 0))
    plan = plan or task["default_plan"]
    for s in task["plans"][plan]["steps"]:
        for vp in s.get("viewpoints", []):
            cv.d.ellipse([cv.p(vp[0] - 0.08, vp[1] + 0.08), cv.p(vp[0] + 0.08, vp[1] - 0.08)], outline=(30, 150, 120), width=2)
    if fleet:
        for uid, u in fleet["units"].items():
            tr = u["trace"]
            if len(tr) > 1:
                cv.d.line([cv.p(t[1], t[2]) for t in tr], fill=UNIT.get(u["key"], (60, 60, 60)) + (190,), width=2)
    faults = {f["unit"] for f in scn["fleet"]["faults"]}
    for u in scn["fleet"]["units"]:
        cat = catalog.unit(u["catalog"])
        X0, Y0, T0 = u["start"]
        col = UNIT.get(u["catalog"], (60, 60, 60))
        for m in cat["modules"]:
            c, s_ = math.cos(T0), math.sin(T0)
            mx, my = X0 + c * m["x"] - s_ * m["y"], Y0 + s_ * m["x"] + c * m["y"]
            cv.poly(rect_pts(mx, my, T0 + m["yaw"], m["size"][0], m["size"][1]), fill=col + (230,), outline=(10, 10, 10))
        hx, hy = X0 + 0.25 * math.cos(T0), Y0 + 0.25 * math.sin(T0)
        cv.d.line([cv.p(X0, Y0), cv.p(hx, hy)], fill=(255, 255, 255), width=2)
        cv.text(X0, Y0 - 0.32, u["id"], 10, (10, 10, 60))
        if u["id"] in faults:
            cv.text(X0 + 0.25, Y0 + 0.25, "fault", 10, (200, 0, 0))
        for p in scn["payloads"]:
            if p["spawn"].get("on_unit") == u["id"]:
                x, y, yaw = p["spawn"]["pose"]
                cv.poly(rect_pts(x, y, yaw, p["size_lwh"][0], p["size_lwh"][1]), fill=(240, 120, 20, 200), outline=(120, 50, 0))
    f = scn["family"]
    st = FAMILIES[f["id"]]["stresses"]
    cv.d.text((PAD, 8), "%s  (%s, seed %d)   plan: %s" % (f["id"], f["split"], f["seed"], plan), fill=(20, 20, 20), font=font(15))
    cv.d.text((PAD, 28), st["decision"][:150], fill=(60, 60, 60), font=font(11))
    if fleet:
        cv.d.text((PAD, 44), "fastsim: %s (%.1f s) %s" % (fleet["outcome"], fleet["t_end"], (fleet["reason"] or "")[:110]),
                  fill=(0, 110, 0) if fleet["outcome"] == "SUCCEEDED" else (170, 0, 0), font=font(11))
    return cv.img


def save(img, path):
    """JPEG (the README curator takes JPEG only), quality 88, at most 300 KB; a .png path writes an indexed PNG."""
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    if path.suffix.lower() == ".png":
        img.convert("P", palette=Image.ADAPTIVE, colors=128).save(path, optimize=True)
    else:
        img.convert("RGB").save(path, "JPEG", quality=88, optimize=True, progressive=True)
    size = path.stat().st_size
    if size > 300 * 1024:
        raise ValueError("%s is %d bytes (> 300 KB)" % (path, size))
    return size


def render_all(runs=None, seed=1, split="train"):
    thumbs = []
    for f in GENERATORS:
        scn, task = generate(f, seed, split)
        fleet = None
        plan = task["default_plan"]
        if runs:
            fj = Path(runs) / ("%s-%s-%s" % (f, split, plan)) / ("s%d" % seed) / "fleet.json"
            if fj.exists():
                fleet = json.loads(fj.read_text())
        img = draw(scn, task, fleet, plan)
        n = save(img, PREVIEWS / ("%s.jpg" % f))
        print("%s.jpg %d bytes" % (f, n))
        thumbs.append(img)
    # index sheet: 3 x 3 thumbnails
    tw = 420
    th = int(thumbs[0].height * tw / thumbs[0].width)
    sheet = Image.new("RGB", (3 * tw, 3 * th), (255, 255, 255))
    for i, im in enumerate(thumbs):
        sheet.paste(im.resize((tw, int(im.height * tw / im.width))), ((i % 3) * tw, (i // 3) * th))
    n = save(sheet, PREVIEWS / "index.jpg")
    print("index.jpg %d bytes" % n)


def main(argv=None):
    ap = argparse.ArgumentParser(prog="scenes.render")
    ap.add_argument("--all", action="store_true")
    ap.add_argument("--runs", help="a fastsim_run output folder (traces of the default plan, seed 1)")
    ap.add_argument("--family")
    ap.add_argument("--split", default="train")
    ap.add_argument("--seed", type=int, default=1)
    ap.add_argument("--plan")
    ap.add_argument("--fleet")
    ap.add_argument("--out")
    a = ap.parse_args(argv)
    if a.all:
        return render_all(a.runs)
    scn, task = generate(a.family, a.seed, a.split)
    fleet = json.loads(Path(a.fleet).read_text()) if a.fleet else None
    print(save(draw(scn, task, fleet, a.plan), a.out or PREVIEWS / ("%s_%s_s%d.jpg" % (a.family, a.split, a.seed))))


if __name__ == "__main__":
    main()
