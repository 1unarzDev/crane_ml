"""Geometry parity: the same v1 scene compiled to fastsim and to CRANE describes the same world (to 1 mm).

    python3 -m scenes.consistency [--family F --split S --seed N]     # default: every family, seed 1, both splits

Each side is read back from its EMITTED file the way its simulator reads it:
  fastsim  the bundle through FleetBundleFormat.parse (fastsim Scenario elements: driving boxes, payload elements,
           start = ego start) plus the bundle's units and zones (what FleetEngine uses)
  CRANE    the compiled crane.fleet_scenario/v0 through a Python port of FleetWorld (outer walls from arena size and
           the overlay thickness, walls cut by their doorway, box obstacles), AutonomyMission.BuildMap's driving rule
           (bottom above 0.3 m does not block), zones.<id>.rect, and the mission's crane_start / crane_payload
Checked: driving solids (footprints), all solids (3-D boxes, against scenes.common.lower), door openings measured in
each geometry, zones, every unit start pose, payload spawn poses.
"""
import argparse
import json
import math
import sys
import tempfile
from pathlib import Path

from .common import BUNDLE_SCHEMA, FLEET, SPEC_DIR, dump, door_frame, lower
from .families import GENERATORS, generate
from . import crane_compile

TOL = 1e-3


def crane_world(path):
    """Boxes (id, cx, cy, cz, sx, sy, sz) FleetWorld builds from a v0 scenario file (port of FleetWorld's constructor)."""
    s = json.loads(Path(path).read_text())
    o = json.loads((SPEC_DIR / "crane_fleet_sim_v0.json").read_text())
    wt = o["scene"]["outer_wall_thickness"]["value"]
    X, Y = s["arena"]["size_xy"]
    H = s["arena"]["wall_height"]
    out = [("wall_south", X / 2, -wt / 2, H / 2, X + 2 * wt, wt, H), ("wall_north", X / 2, Y + wt / 2, H / 2, X + 2 * wt, wt, H),
           ("wall_west", -wt / 2, Y / 2, H / 2, wt, Y, H), ("wall_east", X + wt / 2, Y / 2, H / 2, wt, Y, H)]
    door = s.get("doorway")
    for w in s["walls"]:
        a, b = w["from"], w["to"]
        vertical = abs(a[0] - b[0]) < 1e-9
        lo, hi = (min(a[1], b[1]), max(a[1], b[1])) if vertical else (min(a[0], b[0]), max(a[0], b[0]))
        fixed = a[0] if vertical else a[1]
        segs = []
        if door and door["in_wall"] == w["id"]:
            c, dw, dh = door["centre_y" if vertical else "centre_x"], door["width"]["default"], door["height"]
            segs = [(lo, c - dw / 2, 0, H), (c + dw / 2, hi, 0, H), (c - dw / 2, c + dw / 2, dh, H)]
        else:
            segs = [(lo, hi, 0, H)]
        for k, (s0, s1, z0, z1) in enumerate(segs):
            if s1 - s0 <= 1e-6:
                continue
            m, ln = (s0 + s1) / 2, s1 - s0
            out.append(("%s_%d" % (w["id"], k), fixed, m, (z0 + z1) / 2, w["thickness"], ln, z1 - z0) if vertical else
                       ("%s_%d" % (w["id"], k), m, fixed, (z0 + z1) / 2, ln, w["thickness"], z1 - z0))
    for ob in s["obstacles"]:
        if ob["type"] != "box":
            raise ValueError("FleetWorld refuses obstacle type %s" % ob["type"])
        c, z = ob["centre"], ob["size"]
        out.append((ob["id"], c[0], c[1], c[2], z[0], z[1], z[2]))
    return out, s


def crane_driving(boxes):
    # AutonomyMission.BuildMap: skip obstacles whose bottom is above 0.3 m (outer walls are the map bounds)
    return [b for b in boxes if b[3] - b[6] / 2 <= 0.3]


def _close(a, b, tol=TOL):
    return all(abs(x - y) <= tol for x, y in zip(a, b))


def opening(boxes2d, cx, cy, normal, half_depth):
    """The clear width of a door measured in a set of 2-D boxes: the gap along the wall line through (cx, cy)."""
    lo, hi = -math.inf, math.inf
    for (_, bx, by, sx, sy) in boxes2d:
        if normal == "x":   # wall of constant x: the door spans y
            if abs(bx - cx) > sx / 2 + half_depth - 1e-9:
                continue
            y0, y1 = by - sy / 2, by + sy / 2
            if y1 <= cy + 1e-9:
                lo = max(lo, y1)
            elif y0 >= cy - 1e-9:
                hi = min(hi, y0)
            else:
                return 0.0
        else:
            if abs(by - cy) > sy / 2 + half_depth - 1e-9:
                continue
            x0, x1 = bx - sx / 2, bx + sx / 2
            if x1 <= cx + 1e-9:
                lo = max(lo, x1)
            elif x0 >= cx - 1e-9:
                hi = min(hi, x0)
            else:
                return 0.0
    return hi - lo


def check(scn, task, plan=None, work=None):
    """Problems ([] = parity) between the fastsim bundle and the CRANE files compiled from one scene."""
    sys.path.insert(0, str(FLEET / "scenes" / "fastsim_domain"))
    import fleet_domain  # noqa: E402  (the fastsim format adapter)
    plan = plan or task["default_plan"]
    work = Path(work or tempfile.mkdtemp(prefix="scenes_parity_"))
    bundle = {"schema": BUNDLE_SCHEMA, "scenario": scn, "task": task, "plan": plan}
    bp = dump(bundle, work / "bundle.json")
    fs = fleet_domain.FleetBundleFormat().load(bp)
    r = crane_compile.compile_scene(scn, task, work / "crane", [plan])
    cboxes, cs = crane_world(r["scenario"])
    probs = []
    # 1. driving solids: fastsim box elements vs CRANE's driving boxes (outer walls included)
    fbox = {e.id: (e.x, e.y, e.box[0], e.box[1], e.box[2]) for e in fs.elements if e.box}
    cdrive = {b[0]: (b[1], b[2], b[4], b[5], 0.0) for b in crane_driving(cboxes)}
    if set(fbox) != set(cdrive):
        probs.append("driving solid ids differ: fastsim-only %s, CRANE-only %s" % (sorted(set(fbox) - set(cdrive)),
                                                                                sorted(set(cdrive) - set(fbox))))
    for k in set(fbox) & set(cdrive):
        if not _close(fbox[k], cdrive[k]):
            probs.append("driving solid %s: fastsim %s vs CRANE %s" % (k, fbox[k], cdrive[k]))
    # 2. all solids in 3-D: CRANE's boxes vs the lowering fastsim's engine uses (thresholds are a declared CRANE gap)
    low = {s.id: (s.cx, s.cy, s.cz, s.sx, s.sy, s.sz) for s in lower(scn) if s.kind != "threshold"}
    call = {b[0]: b[1:] for b in cboxes}
    if set(low) != set(call):
        probs.append("3-D solid ids differ: %s" % sorted(set(low) ^ set(call)))
    for k in set(low) & set(call):
        if not _close(low[k], call[k]):
            probs.append("solid %s: v1 %s vs CRANE %s" % (k, low[k], call[k]))
    # 3. door openings measured in both geometries
    f2 = [(k, v[0], v[1], v[2], v[3]) for k, v in fbox.items()]
    c2 = [(k, v[0], v[1], v[2], v[3]) for k, v in cdrive.items()]
    for d in scn["doors"]:
        cx, cy, normal, w = door_frame(scn, d)
        wf, wc = opening(f2, cx, cy, normal, w["thickness"] / 2), opening(c2, cx, cy, normal, w["thickness"] / 2)
        if not (abs(wf - d["width"]) <= TOL and abs(wc - d["width"]) <= TOL):
            probs.append("door %s: width v1 %.4f, fastsim %.4f, CRANE %.4f" % (d["id"], d["width"], wf, wc))
    # 4. zones
    for z in scn["zones"]:
        cz = cs["zones"].get(z["id"])
        if cz is None or not _close(cz["rect"], z["rect"]):
            probs.append("zone %s differs" % z["id"])
    # 5. starts and payloads (CRANE side from the mission, when the fleet maps)
    ego = task["plans"][plan].get("ego", task["ego"])
    est = next(u["start"] for u in scn["fleet"]["units"] if u["id"] == ego)
    if not _close(fs.start, est):
        probs.append("fastsim start %s is not the ego's %s" % (fs.start, est))
    mp = r["plans"][plan]["mission"]
    approx = False
    if mp:
        m = json.loads(Path(mp).read_text())
        approx = m["mission_id"].endswith("/crane_approx")
    if mp and not approx:
        for uid, rid in m["crane_unit_map"].items():
            v1 = next(u["start"] for u in scn["fleet"]["units"] if u["id"] == uid)
            if not _close(m["crane_start"][rid], v1):
                probs.append("start of %s (CRANE %s) differs" % (uid, rid))
        if scn["payloads"]:
            p = scn["payloads"][0]
            fp = next(e for e in fs.elements if e.id == p["id"])
            if not (_close((fp.x, fp.y), m["crane_payload"]["pose"][:2]) and _close(m["crane_payload"]["pose"], p["spawn"]["pose"])):
                probs.append("payload %s pose differs" % p["id"])
    return probs, {"crane_mission": ("approx" if approx else True) if mp else False, "solids": len(call),
                   "driving": len(cdrive), "doors": len(scn["doors"])}


def main(argv=None):
    ap = argparse.ArgumentParser(prog="scenes.consistency")
    ap.add_argument("--family")
    ap.add_argument("--split")
    ap.add_argument("--seed", type=int, default=1)
    a = ap.parse_args(argv)
    fams = [a.family] if a.family else list(GENERATORS)
    splits = [a.split] if a.split else ["train", "heldout"]
    bad = 0
    for f in fams:
        for sp in splits:
            scn, task = generate(f, a.seed, sp)
            for plan in task["plans"]:
                probs, info = check(scn, task, plan)
                bad += bool(probs)
                print("%-20s %-7s s%d %-12s %s %s" % (f, sp, a.seed, plan, "PARITY" if not probs else "MISMATCH", info if not probs else probs[:3]))
    return 1 if bad else 0


if __name__ == "__main__":
    sys.exit(main())
