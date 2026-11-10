"""Shared definitions of scenes v1: paths, hashing, floor and payload tables, and the geometry lowering.

``lower(scenario)`` is the ONE place where walls, doors, thresholds and obstacles become solid boxes. The
fastsim format adapter and the CRANE compiler both call it, so the two simulators get the same geometry by
construction; ``consistency.py`` then checks the emitted files (after serialisation and after CRANE's own
outer-wall and wall-splitting rules) agree to 1 mm.
"""
import hashlib
import json
import math
import os
from dataclasses import dataclass, asdict
from pathlib import Path

HERE = Path(__file__).resolve().parent
FLEET = HERE.parent                                   # Tools/Fleet of this worktree
SPEC_DIR = FLEET / "spec"
CAPPLAN_ROOT = Path(os.environ.get("FLEET_CAPPLAN_ROOT", "/home/lunarz/crane_ml/Tools/Fleet"))  # read-only

SCENARIO_SCHEMA = "crane.fleet_scenario/v1"
TASK_SCHEMA = "crane.fleet_task/v1"
BUNDLE_SCHEMA = "crane.fleet_scene_bundle/v1"

# CRANE BuildMap skips obstacles whose bottom is above 0.3 m (AutonomyMission.BuildMap); fastsim uses the same rule.
DRIVE_BLOCK_ZMAX = 0.3
OUTER_WALL_THICKNESS = 0.15   # crane_fleet_sim_v0.json scene.outer_wall_thickness (FleetWorld uses the overlay value)


def canonical(obj):
    return json.dumps(obj, sort_keys=True, separators=(",", ":"), ensure_ascii=True)


def sha(obj, n=16):
    data = obj if isinstance(obj, bytes) else canonical(obj).encode()
    return hashlib.sha256(data).hexdigest()[:n]


def file_sha(path, n=16):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()[:n]


def dump(obj, path):
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(obj, indent=1, sort_keys=False) + "\n")
    return path


def r(v, nd=4):
    return round(float(v), nd) + 0.0


# ---------------------------------------------------------------- floors and payloads
def _profile():
    return json.loads((SPEC_DIR / "fleet_profile_v0.json").read_text())


def _v(d):
    return d["value"] if isinstance(d, dict) and "value" in d else d


PROFILE = _profile()

# friction class -> profile floor entry (None: not in the profile; local provisional numbers, a CRANE gap)
FRICTION_CLASSES = {
    "hard_tile": {"profile": "hard_tile", "speed_scale": 1.0},
    "carpet": {"profile": "low_pile_carpet", "speed_scale": 0.8},
    "rubber": {"profile": "rubber_mat", "speed_scale": 0.95},
    "polished": {"profile": None, "mu_long": 0.35, "lateral_efficiency": 0.60, "slip_sigma_frac": 0.08,
                 "speed_scale": 0.75,
                 "note": "low-friction sealed concrete / wet tile; provisional, NOT in fleet_profile_v0 floors"},
}
# speed_scale: the fastsim stand-in for rolling resistance (carpet pile) or traction-limited speed (polished). An
# ASSUMPTION of this tier (no source); CRANE gets friction from the profile's mu/lateral_efficiency, not this number.


def floor_props(cls):
    e = FRICTION_CLASSES[cls]
    if e["profile"]:
        f = PROFILE["floors"][e["profile"]]
        return {"mu_long": _v(f["mu_long"]), "lateral_efficiency": _v(f["lateral_efficiency"]),
                "slip_sigma_frac": _v(f["slip_sigma_frac"]), "profile": e["profile"], "speed_scale": e["speed_scale"]}
    return {k: e[k] for k in ("mu_long", "lateral_efficiency", "slip_sigma_frac", "speed_scale")} | {"profile": None}


def payload_type(name):
    p = PROFILE["payloads"][name]
    return {"size_lwh": list(_v(p["size_lwh"])), "mass": float(_v(p["mass"])), "needs": p.get("needs", "")}


# ---------------------------------------------------------------- lowering
@dataclass
class Solid:
    id: str
    kind: str          # outer_wall | wall | lintel | threshold | <obstacle kind>
    cx: float
    cy: float
    cz: float
    sx: float
    sy: float
    sz: float
    yaw: float = 0.0
    drive_block: bool = True    # blocks a rover's footprint (bottom below DRIVE_BLOCK_ZMAX, not traversable)
    view_block: bool = True     # occludes cameras
    source: str = ""            # the v1 element it came from

    @property
    def zmin(self):
        return self.cz - self.sz / 2

    @property
    def zmax(self):
        return self.cz + self.sz / 2

    def footprint(self):
        """Corners (x, y) of the box's ground projection."""
        c, s = math.cos(self.yaw), math.sin(self.yaw)
        hx, hy = self.sx / 2, self.sy / 2
        return [(self.cx + c * u - s * v, self.cy + s * u + c * v) for u, v in
                ((hx, hy), (hx, -hy), (-hx, -hy), (-hx, hy))]

    def to_dict(self):
        return {k: (r(v, 6) if isinstance(v, float) else v) for k, v in asdict(self).items()}


def wall_axis(w):
    (x0, y0), (x1, y1) = w["from"], w["to"]
    if abs(x0 - x1) < 1e-9:
        return "y", x0, min(y0, y1), max(y0, y1)      # vertical wall: runs along y at x = x0
    if abs(y0 - y1) < 1e-9:
        return "x", y0, min(x0, x1), max(x0, x1)
    raise ValueError("wall %s is not axis-aligned" % w["id"])


def door_frame(scn, door):
    """(centre_x, centre_y, normal_axis, wall) of a door: the axis a rover crosses it along."""
    w = next(w for w in scn["walls"] if w["id"] == door["wall"])
    along, fixed, _, _ = wall_axis(w)
    if along == "y":
        return fixed, door["centre"], "x", w
    return door["centre"], fixed, "y", w


def lower(scn):
    """Every solid of a v1 scenario, in a fixed order: outer walls, interior walls cut by their doors (segments,
    lintels), door thresholds, obstacles."""
    X, Y = scn["arena"]["size_xy"]
    H = float(scn["arena"]["wall_height"])
    t = float(scn["arena"].get("outer_wall_thickness", OUTER_WALL_THICKNESS))
    out = [Solid("wall_south", "outer_wall", X / 2, -t / 2, H / 2, X + 2 * t, t, H, source="arena"),
           Solid("wall_north", "outer_wall", X / 2, Y + t / 2, H / 2, X + 2 * t, t, H, source="arena"),
           Solid("wall_west", "outer_wall", -t / 2, Y / 2, H / 2, t, Y, H, source="arena"),
           Solid("wall_east", "outer_wall", X + t / 2, Y / 2, H / 2, t, Y, H, source="arena")]
    doors = scn.get("doors", [])
    for w in scn.get("walls", []):
        along, fixed, lo, hi = wall_axis(w)
        th = float(w["thickness"])
        wh = float(w.get("height", H))
        mine = sorted((d for d in doors if d["wall"] == w["id"]), key=lambda d: d["centre"])
        cuts, k = [], 0
        pos = lo
        for d in mine:
            a, b = d["centre"] - d["width"] / 2, d["centre"] + d["width"] / 2
            cuts.append((pos, a, 0.0, wh, "wall", w["id"]))
            if d["height"] < wh:
                cuts.append((a, b, float(d["height"]), wh, "lintel", d["id"]))
            pos = b
        cuts.append((pos, hi, 0.0, wh, "wall", w["id"]))
        for s0, s1, z0, z1, kind, src in cuts:
            if s1 - s0 <= 1e-6:
                continue
            mid, ln = (s0 + s1) / 2, s1 - s0
            sid = "%s_%d" % (w["id"], k) if kind == "wall" else "%s_lintel" % src
            k += kind == "wall"
            if along == "y":
                sol = Solid(sid, kind, fixed, mid, (z0 + z1) / 2, th, ln, z1 - z0, source=src)
            else:
                sol = Solid(sid, kind, mid, fixed, (z0 + z1) / 2, ln, th, z1 - z0, source=src)
            sol.drive_block = z0 <= DRIVE_BLOCK_ZMAX
            out.append(sol)
    for d in doors:
        h = float(d.get("threshold_height", 0.0) or 0.0)
        if h <= 0:
            continue
        cx, cy, normal, w = door_frame(scn, d)
        depth = float(d.get("threshold_depth", w["thickness"]))
        sx, sy = (depth, d["width"]) if normal == "x" else (d["width"], depth)
        out.append(Solid(d["id"] + "_threshold", "threshold", cx, cy, h / 2, sx, sy, h, drive_block=False,
                         view_block=False, source=d["id"]))
    for o in scn.get("obstacles", []):
        (cx, cy, cz), (sx, sy, sz) = o["centre"], o["size"]
        sol = Solid(o["id"], o.get("kind", "box"), cx, cy, cz, sx, sy, sz, float(o.get("yaw", 0.0)), source=o["id"])
        sol.drive_block = sol.zmin <= DRIVE_BLOCK_ZMAX
        sol.view_block = bool(o.get("blocks_view", True))
        out.append(sol)
    return out


def zone(scn, zid):
    return next(z for z in scn["zones"] if z["id"] == zid)


def rect_centre(rect):
    return ((rect[0] + rect[2]) / 2, (rect[1] + rect[3]) / 2)


def in_rect(p, rect, margin=0.0):
    return rect[0] - margin <= p[0] <= rect[2] + margin and rect[1] - margin <= p[1] <= rect[3] + margin


def unit_by_id(scn, uid):
    return next(u for u in scn["fleet"]["units"] if u["id"] == uid)


def geometry_signature(scn):
    """What a scene's geometry IS, without ids, notes, seeds or family labels (leakage hashing)."""
    sol = sorted([round(s.cx, 3), round(s.cy, 3), round(s.cz, 3), round(s.sx, 3), round(s.sy, 3), round(s.sz, 3),
                  round(s.yaw, 3), s.drive_block] for s in lower(scn))
    zones = sorted([z["kind"], [round(v, 3) for v in z["rect"]]] for z in scn["zones"])
    patches = sorted([p["friction_class"], [round(v, 3) for v in p["rect"]]] for p in scn.get("floor_patches", []))
    units = sorted([u["catalog"], [round(v, 3) for v in u["start"]]] for u in scn["fleet"]["units"])
    pls = sorted([p["type"], round(p["mass_kg"], 3), [round(v, 3) for v in p["spawn"]["pose"]]] for p in scn["payloads"])
    faults = sorted([f["unit"], f["target"], f["mode"], round(f["t_s"], 2)] for f in scn["fleet"].get("faults", []))
    cams = sorted([c["mode"], [round(v, 3) for v in c["position"]]] for c in scn.get("overhead_cameras", []))
    return {"solids": sol, "zones": zones, "patches": patches, "units": units, "payloads": pls, "faults": faults,
            "cameras": cams}
