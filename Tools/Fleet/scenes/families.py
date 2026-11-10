"""Procedural scene families v1: each family stresses one capability or decision (CONFIG_SPACE.md section 3 rule:
it changes the feasible actions, the required capabilities or the coalition needed, not the look).

    generate(family, seed, split) -> (scenario v1, task v1)

A family's parameters are drawn from the seed in its TRAIN or HELD-OUT regime; the regimes are disjoint RULES over the
parameters (``RULES``), checked by ``in_split`` after generation (splits.py freezes and leak-checks them). Every
family uses only catalogue units resolved from capplan train_v1 / train-pool types (the H7 control).

Geometry conventions: metres, x east, y north, origin at the south-west inside corner (as warehouse_a_v0).
"""
import math

import numpy as np

from .common import SCENARIO_SCHEMA, TASK_SCHEMA, payload_type, sha
from . import catalog

GENERATOR_VERSION = "families_v1.0"
WALL_T = 0.12
H = 2.8

FAMILIES = {
    "narrow_door": {
        "stresses": {"capability": "rigid-assembly length through a doorway dogleg (two offset doors, a narrow gap)",
                     "decision": "reconfiguration HARMFUL: coupling two carriers (e.g. for an unknown-mass payload) makes "
                                 "a unit that cannot turn in the gap; the single carrier passes"}},
    "long_payload_door": {
        "stresses": {"capability": "assembly minimum width vs door width; payload support (box_long needs two decks)",
                     "decision": "reconfiguration BENEFICIAL: the pre-assembled L of three carriers (min width 0.80 m) "
                                 "cannot pass the door; undocking one module leaves a chain that passes with the payload"}},
    "lateral_slot": {
        "stresses": {"capability": "lateral (strafe) motion: a shelf slot that admits the rover only sideways at yaw 0",
                     "decision": "which unit to assign: a mecanum carrier succeeds, a differential carrier cannot reach "
                                 "the slot pose"}},
    "friction_patches": {
        "stresses": {"capability": "capability identification: floor patches change speed, acceleration and lateral "
                                   "efficiency (carpet / rubber / polished)",
                     "decision": "route through the patch or around it, under a deadline set from hard-tile capability"}},
    "aisle_traffic": {
        "stresses": {"capability": "single-lane aisles shared by opposing robot flows",
                     "decision": "multi-robot coordination (WoMAPF stand-in): aisle reservations vs independent motion"}},
    "charging_contention": {
        "stresses": {"capability": "battery state and a scarce resource (fewer charging bays than robots)",
                     "decision": "bay assignment: nearest-bay greedy queueing vs balanced assignment"}},
    "blocked_faces": {
        "stresses": {"capability": "docking-face approach corridors (approach length) blocked by pallets or walls",
                     "decision": "which face pair to mate, or move the target first"}},
    "wheel_failure": {
        "stresses": {"capability": "mid-mission wheel fault (dead_locked: immobile, weak: slower, from the capplan "
                                   "health descriptor)",
                     "decision": "continue with the degraded unit or reassign the payload to a spare (handoff)"}},
    "clutter_observer": {
        "stresses": {"capability": "visibility: tall clutter hides the payload from low carrier cameras; overhead "
                                   "cameras may be working, occluded or disabled",
                     "decision": "send a mast-camera scout as observer, or let the carrier search itself"}},
}

# The split rules (H7 control): disjoint predicates over each family's parameters.
RULES = {
    "narrow_door": {"train": "min(door_w, gap) >= 0.55 and threshold_h <= 0.02",
                    "heldout": "min(door_w, gap) < 0.55 or threshold_h > 0.02"},
    "long_payload_door": {"train": "door_w >= 0.55 and n_doors == 1",
                          "heldout": "door_w < 0.55 or n_doors == 2"},
    "lateral_slot": {"train": "slot_wall == 'north' and slot_w >= 0.52",
                     "heldout": "slot_wall == 'east' or slot_w < 0.52"},
    "friction_patches": {"train": "patch_class in ('carpet', 'rubber')",
                         "heldout": "patch_class == 'polished'  (an unseen floor class)"},
    "aisle_traffic": {"train": "n_aisles == 2 and n_units <= 4",
                      "heldout": "n_aisles == 3 or n_units > 4"},
    "charging_contention": {"train": "n_units <= 3 and n_bays == 2",
                            "heldout": "n_units > 3 or n_bays != 2"},
    "blocked_faces": {"train": "n_blocked == 1",
                      "heldout": "n_blocked >= 2"},
    "wheel_failure": {"train": "n_faults == 1 and t_fail <= 35",
                      "heldout": "n_faults == 2 or t_fail > 35"},
    "clutter_observer": {"train": "overhead in ('working', 'occluded') and n_clutter <= 6",
                         "heldout": "overhead == 'disabled' or n_clutter > 6"},
}


def in_split(family, p):
    """Which split a family's parameter dict falls in, by RULES (the code mirrors the text)."""
    f = family
    if f == "narrow_door":
        tr = min(p["door_w"], p["gap"]) >= 0.55 and p["threshold_h"] <= 0.02
    elif f == "long_payload_door":
        tr = p["door_w"] >= 0.55 and p["n_doors"] == 1
    elif f == "lateral_slot":
        tr = p["slot_wall"] == "north" and p["slot_w"] >= 0.52
    elif f == "friction_patches":
        tr = p["patch_class"] in ("carpet", "rubber")
    elif f == "aisle_traffic":
        tr = p["n_aisles"] == 2 and p["n_units"] <= 4
    elif f == "charging_contention":
        tr = p["n_units"] <= 3 and p["n_bays"] == 2
    elif f == "blocked_faces":
        tr = p["n_blocked"] == 1
    elif f == "wheel_failure":
        tr = p["n_faults"] == 1 and p["t_fail"] <= 35
    elif f == "clutter_observer":
        tr = p["overhead"] in ("working", "occluded") and p["n_clutter"] <= 6
    else:
        raise KeyError(f)
    return "train" if tr else "heldout"


# ---------------------------------------------------------------- building blocks
def _r(v):
    return round(float(v), 3) + 0.0


class Scene:
    def __init__(self, family, seed, split, params, size=(10.0, 7.0), floor="hard_tile"):
        self.family, self.seed, self.split, self.params = family, seed, split, params
        self.d = {"schema": SCENARIO_SCHEMA, "scenario_id": "fleet/scenes_v1/%s/%s/s%d" % (family, split, seed),
                  "family": {"id": family, "seed": seed, "split": split, "params": params,
                             "generator": GENERATOR_VERSION, "rule": RULES[family][split]},
                  "note": FAMILIES[family]["stresses"]["capability"],
                  "arena": {"size_xy": [float(size[0]), float(size[1])], "wall_height": H, "floor": floor,
                            "outer_wall_thickness": 0.15},
                  "walls": [], "doors": [], "obstacles": [], "floor_patches": [], "zones": [], "tags": [],
                  "overhead_cameras": [], "payloads": [],
                  "fleet": {"units": [], "faults": [], "initial_couplings": []}}
        self._tag = 0

    def wall(self, wid, a, b):
        self.d["walls"].append({"id": wid, "from": [_r(a[0]), _r(a[1])], "to": [_r(b[0]), _r(b[1])],
                                "thickness": WALL_T, "height": H})

    def door(self, did, wall, centre, width, height=2.0, threshold=0.0):
        self.d["doors"].append({"id": did, "wall": wall, "centre": _r(centre), "width": _r(width), "height": height,
                                "threshold_height": _r(threshold), "threshold_depth": WALL_T})
        # door-jamb landmark tags on both faces of the wall (CRANE landmarks), like warehouse_a_v0's overlay tags
        w = next(x for x in self.d["walls"] if x["id"] == wall)
        vertical = abs(w["from"][0] - w["to"][0]) < 1e-9
        fixed = w["from"][0] if vertical else w["from"][1]
        for side, sgn in (("w" if vertical else "s", -1), ("e" if vertical else "n", 1)):
            for end, off in (("lo", -width / 2 - 0.2), ("hi", width / 2 + 0.2)):
                if vertical:
                    pose = [_r(fixed + sgn * (WALL_T / 2 + 0.002)), _r(centre + off), 0.0 if sgn > 0 else round(math.pi, 6)]
                else:
                    pose = [_r(centre + off), _r(fixed + sgn * (WALL_T / 2 + 0.002)),
                            round(math.pi / 2, 6) if sgn > 0 else round(-math.pi / 2, 6)]
                self.tag("lm_%s_%s_%s" % (did, side, end), pose, "door")

    def tag(self, name, pose, on, size=0.12, z=0.3):
        self.d["tags"].append({"id": 950 + self._tag, "name": name, "pose": pose, "z": z, "size": size, "on": on})
        self._tag += 1

    def box(self, oid, kind, x0, y0, x1, y1, z0=0.0, z1=1.8, yaw=0.0, blocks_view=True):
        self.d["obstacles"].append({"id": oid, "kind": kind, "centre": [_r((x0 + x1) / 2), _r((y0 + y1) / 2), _r((z0 + z1) / 2)],
                                    "size": [_r(x1 - x0), _r(y1 - y0), _r(z1 - z0)], "yaw": _r(yaw),
                                    "blocks_view": blocks_view})

    def zone(self, zid, kind, rect, **kw):
        self.d["zones"].append({"id": zid, "kind": kind, "rect": [_r(v) for v in rect], **kw})

    def patch(self, pid, rect, cls):
        self.d["floor_patches"].append({"id": pid, "rect": [_r(v) for v in rect], "friction_class": cls})

    def camera(self, cid, pos, look, mode, hfov=90.0, rng=8.0):
        self.d["overhead_cameras"].append({"id": cid, "position": [_r(v) for v in pos], "look_at": [_r(v) for v in look],
                                           "hfov_deg": hfov, "range_m": rng, "mode": mode})

    def payload(self, pid, ptype, pose, mass=None, unknown_mass=False, pose_known=True, zone_id=None, on_unit=None):
        spec = payload_type(ptype)
        self.d["payloads"].append({"id": pid, "type": ptype, "size_lwh": spec["size_lwh"],
                                   "mass_kg": _r(mass if mass is not None else spec["mass"]),
                                   "unknown_mass": bool(unknown_mass), "pallet_height": 0.03,
                                   "spawn": {"zone": zone_id, "pose": [_r(v) for v in pose], "pose_known": bool(pose_known),
                                             **({"on_unit": on_unit} if on_unit else {})}})

    def unit(self, uid, key, start, soc=1.0):
        u = catalog.unit(key)
        self.d["fleet"]["units"].append({"id": uid, "catalog": key, "module_graph": u["ref"],
                                         "config_hash": u["config_hash"], "start": [_r(v) for v in start],
                                         "soc": _r(soc), "crane_role": u.get("crane_role")})

    def fault(self, uid, target, mode, t):
        self.d["fleet"]["faults"].append({"unit": uid, "module": "m0", "target": target, "mode": mode, "t_s": _r(t)})


def _task(sc, ego, deadline, plans, default, success, stuck=60.0, fail_on=None, **kw):
    t = {"schema": TASK_SCHEMA, "task_id": sc.d["scenario_id"] + "/task", "scenario_id": sc.d["scenario_id"],
         "scenario_sha256": None, "stresses": FAMILIES[sc.family]["stresses"], "ego": ego,
         "deadline_s": _r(deadline), "stuck_s": stuck, "plans": plans, "default_plan": default,
         "outcomes": {"success": success, "fail_on": fail_on or ["collision", "deadline", "stuck", "step_failed"],
                      "collision_tol_m": 0.01}}
    t.update(kw)
    return t


def _u(rng, a, b):
    return float(rng.uniform(a, b))


# ---------------------------------------------------------------- families
def narrow_door(seed, split):
    rng = np.random.default_rng([1, seed, 0 if split == "train" else 1])
    for _ in range(100):
        if split == "train":
            p = {"door_w": _u(rng, 0.55, 0.75), "gap": _u(rng, 0.55, 0.75), "offset": _u(rng, 1.2, 2.4),
                 "threshold_h": float(rng.choice([0.0, 0.01, 0.02]))}
        else:
            p = {"door_w": _u(rng, 0.50, 0.60), "gap": _u(rng, 0.52, 0.60), "offset": _u(rng, 1.2, 2.4),
                 "threshold_h": float(rng.choice([0.0, 0.02, 0.03]))}
        p["mass_true"] = _u(rng, 1.5, 6.0)
        p = {k: round(v, 3) for k, v in p.items()}
        if in_split("narrow_door", p) == split:
            break
    sc = Scene("narrow_door", seed, split, p)
    x1 = 5.0
    x2 = x1 + WALL_T + p["gap"]
    y1 = _u(rng, 1.5, 3.0)
    y2 = min(5.6, y1 + p["offset"])
    sc.wall("wall_a", (x1, 0.0), (x1, 7.0))
    sc.wall("wall_b", (x2, 0.0), (x2, 7.0))
    sc.door("door_a", "wall_a", y1, p["door_w"], threshold=p["threshold_h"])
    sc.door("door_b", "wall_b", y2, p["door_w"], threshold=p["threshold_h"])
    sc.zone("staging", "staging", [0.5, 0.5, 2.5, 2.2])
    sc.zone("loading", "loading", [2.8, 4.5, 4.4, 6.3])
    sc.zone("assembly", "assembly", [2.6, 2.6, 4.4, 4.2])
    sc.zone("delivery", "delivery", [8.0, 2.5, 9.5, 4.5])
    sc.zone("charging", "charging", [0.5, 5.6, 1.3, 6.5], capacity=1)
    sc.box("shelf_w", "shelf", 0.4, 3.2, 0.9, 5.2, 0.0, 1.8)
    sc.camera("oh_west", (2.5, 3.5, 2.6), (3.5, 4.5, 0.0), "working")
    sc.camera("oh_east", (8.0, 3.5, 2.6), (8.5, 3.5, 0.0), "working")
    px, py = _u(rng, 3.2, 4.0), _u(rng, 4.9, 5.9)
    sc.payload("p1", "box_small", (px, py, 0.0), mass=p["mass_true"], unknown_mass=True, zone_id="loading")
    sc.unit("cargo_1", "carrier", (1.2, 1.0, 0.0))
    sc.unit("cargo_2", "carrier", (1.2, 1.7, 0.0))
    plans = {
        "single": {"expected": "success", "note": "one carrier takes the payload through both doors",
                   "steps": [{"id": "pick", "kind": "pickup", "unit": "cargo_1", "payload": "p1"},
                             {"id": "go", "kind": "deliver", "unit": "cargo_1", "payload": "p1", "zone": "delivery",
                              "after": ["pick"]}]},
        "couple_chain": {"expected": "fail", "note": "couple front-rear first (a hedge against the unknown mass); the "
                                                     "0.84 m chain cannot turn in the gap",
                         "steps": [{"id": "couple", "kind": "dock", "docker": "cargo_2", "docker_face": "front",
                                    "target": "cargo_1", "target_face": "rear", "as": "pair", "as_catalog": "pair_chain"},
                                   {"id": "pick", "kind": "pickup", "unit": "pair", "payload": "p1", "after": ["couple"]},
                                   {"id": "go", "kind": "deliver", "unit": "pair", "payload": "p1", "zone": "delivery",
                                    "after": ["pick"]}]}}
    return sc, _task(sc, "cargo_1", 240, plans, "single", [{"payload_in_zone": {"payload": "p1", "zone": "delivery"}}])


def long_payload_door(seed, split):
    rng = np.random.default_rng([2, seed, 0 if split == "train" else 1])
    for _ in range(100):
        if split == "train":
            p = {"door_w": _u(rng, 0.55, 0.75), "n_doors": 1}
        else:
            p = {"door_w": _u(rng, 0.48, 0.70), "n_doors": int(rng.choice([1, 2]))}
        p["detour"] = int(rng.random() < 0.5)
        p["door_y"] = _u(rng, 2.5, 4.5)
        p = {k: round(v, 3) if isinstance(v, float) else v for k, v in p.items()}
        if in_split("long_payload_door", p) == split:
            break
    sc = Scene("long_payload_door", seed, split, p)
    sc.wall("divider", (6.0, 0.0), (6.0, 7.0))
    sc.door("door_main", "divider", p["door_y"], p["door_w"])
    if p["detour"]:
        sc.door("door_wide", "divider", 6.35, 1.0)   # the far gate the L fits through (min width 0.80 m)
    if p["n_doors"] == 2:
        sc.wall("inner", (8.0, 0.0), (8.0, 5.6))
        sc.door("door_inner", "inner", _u(rng, 2.0, 4.5), p["door_w"])
    sc.zone("staging", "staging", [0.6, 0.6, 3.0, 2.8])
    sc.zone("delivery", "delivery", [8.6, 2.5, 9.6, 4.5] if p["n_doors"] == 2 else [7.8, 2.5, 9.5, 4.5])
    sc.zone("parking", "parking", [6.8, 5.2, 7.8, 6.2])
    sc.box("pallet_stack", "pallet", 3.6, 4.6, 4.6, 5.4, 0.0, 1.0)
    sc.camera("oh_west", (3.0, 3.5, 2.6), (3.5, 3.5, 0.0), "working")
    start = (1.8, 1.7, 0.0)
    sc.unit("asm", "l3_carrier", start)
    # box_long rides on the chain part (modules m0 + m1) of the L, along x
    l3 = catalog.unit("l3_carrier")
    m0 = next(m for m in l3["modules"] if m["id"] == "m0")
    m1 = next(m for m in l3["modules"] if m["id"] == "m1")
    cx, cy = start[0] + (m0["x"] + m1["x"]) / 2, start[1] + (m0["y"] + m1["y"]) / 2
    sc.payload("p1", "box_long", (cx, cy, 0.0), zone_id=None, on_unit="asm")
    plans = {
        "keep": {"expected": "fail" if not p["detour"] else "depends",
                 "note": "drive the L as it is (through the wide far gate if there is one)",
                 "steps": [{"id": "go", "kind": "deliver", "unit": "asm", "payload": "p1", "zone": "delivery"}]},
        "reconfigure": {"expected": "success", "note": "undock the side module, the chain carries box_long through",
                        "steps": [{"id": "split", "kind": "undock", "unit": "asm", "part": ["asm/m2"],
                                   "as": ["chain", "spare"], "catalogs": ["pair_chain", "carrier"]},
                                  {"id": "go", "kind": "deliver", "unit": "chain", "payload": "p1", "zone": "delivery",
                                   "after": ["split"]},
                                  {"id": "park", "kind": "goto", "unit": "spare", "zone": "parking", "after": ["go"]}]}}
    return sc, _task(sc, "asm", 150, plans, "reconfigure", [{"payload_in_zone": {"payload": "p1", "zone": "delivery"}}])


def lateral_slot(seed, split):
    rng = np.random.default_rng([3, seed, 0 if split == "train" else 1])
    for _ in range(100):
        if split == "train":
            p = {"slot_wall": "north", "slot_w": _u(rng, 0.52, 0.60)}
        else:
            p = {"slot_wall": str(rng.choice(["north", "east"])), "slot_w": _u(rng, 0.505, 0.56)}
        p["slot_d"] = _u(rng, 0.49, 0.53)
        p = {k: round(v, 3) if isinstance(v, float) else v for k, v in p.items()}
        if in_split("lateral_slot", p) == split:
            break
    X, Y = 10.0, 7.0
    sc = Scene("lateral_slot", seed, split, p)
    w, d = p["slot_w"], p["slot_d"]
    if p["slot_wall"] == "north":
        sx = _u(rng, 5.0, 8.0)
        sc.box("shelf_a", "shelf", sx - w / 2 - 1.2, Y - d, sx - w / 2, Y, 0.0, 1.8)
        sc.box("shelf_b", "shelf", sx + w / 2, Y - d, sx + w / 2 + 1.2, Y, 0.0, 1.8)
        slot = [sx - w / 2, Y - d, sx + w / 2, Y]
        req = 0.0
        sc.tag("lm_slot", [_r(sx), _r(Y - 0.002), round(-math.pi / 2, 6)], "shelf", z=0.25)
    else:
        sy = _u(rng, 2.5, 5.0)
        sc.box("shelf_a", "shelf", X - d, sy - w / 2 - 1.2, X, sy - w / 2, 0.0, 1.8)
        sc.box("shelf_b", "shelf", X - d, sy + w / 2, X, sy + w / 2 + 1.2, 0.0, 1.8)
        slot = [X - d, sy - w / 2, X, sy + w / 2]
        req = round(math.pi / 2, 6)
        sc.tag("lm_slot", [_r(X - 0.002), _r(sy), round(math.pi, 6)], "shelf", z=0.25)
    sc.zone("staging", "staging", [0.5, 0.5, 2.5, 2.5])
    sc.zone("loading", "loading", [0.5, 3.5, 2.0, 5.0])
    sc.zone("slot", "slot", slot, required_yaw=req)
    sc.camera("oh_centre", (5.0, 3.5, 2.6), (6.0, 5.5, 0.0), "working")
    sc.payload("p1", "box_small", (1.25, 4.25, 0.0), zone_id="loading")
    sc.unit("mec", "carrier", (1.0, 1.0, 0.0))
    sc.unit("diff", "diff_carrier", (1.0, 2.0, 0.0))
    plans = {
        "mecanum": {"expected": "success", "ego": "mec", "note": "the mecanum carrier strafes into the slot",
                    "steps": [{"id": "pick", "kind": "pickup", "unit": "mec", "payload": "p1"},
                              {"id": "place", "kind": "deliver", "unit": "mec", "payload": "p1", "zone": "slot",
                               "after": ["pick"]}]},
        "diffdrive": {"expected": "fail", "ego": "diff", "note": "the differential carrier cannot reach yaw-0 in the slot",
                      "steps": [{"id": "pick", "kind": "pickup", "unit": "diff", "payload": "p1"},
                                {"id": "place", "kind": "deliver", "unit": "diff", "payload": "p1", "zone": "slot",
                                 "after": ["pick"]}]}}
    return sc, _task(sc, "mec", 200, plans, "mecanum", [{"payload_in_zone": {"payload": "p1", "zone": "slot"}}])


def friction_patches(seed, split):
    rng = np.random.default_rng([4, seed, 0 if split == "train" else 1])
    cls = str(rng.choice(["carpet", "rubber"])) if split == "train" else "polished"
    p = {"patch_class": cls, "patch_x0": round(_u(rng, 3.0, 4.0), 3), "patch_w": round(_u(rng, 2.0, 3.0), 3),
         "bypass_y": round(_u(rng, 5.6, 6.2), 3), "deadline_factor": round(_u(rng, 1.10, 1.30), 3)}
    sc = Scene("friction_patches", seed, split, p)
    x0, x1 = p["patch_x0"], p["patch_x0"] + p["patch_w"]
    sc.patch("patch_1", [x0, 0.0, x1, p["bypass_y"] - 0.5], cls)
    sc.zone("staging", "staging", [0.4, 2.5, 1.8, 4.5])
    sc.zone("delivery", "delivery", [8.4, 2.5, 9.6, 4.5])
    sc.box("rack_s", "rack", x0 + 0.3, 0.0, x1 - 0.3, 1.0, 0.0, 1.5)
    sc.camera("oh_c", (5.0, 3.5, 2.6), (5.0, 3.5, 0.0), "working")
    sc.payload("p1", "box_small", (1.1, 3.0, 0.0), zone_id="staging")
    sc.unit("cargo_1", "carrier", (1.1, 4.0, -math.pi / 2))
    # deadline from hard-tile capability along the straight route (pickup + 7.3 m at ~0.4 m/s effective + handling)
    t_nominal = 6.0 + 4.0 + 8.0 / 0.40 + 8.0
    plans = {
        "through": {"expected": "depends", "note": "shortest route, straight across the patch",
                    "steps": [{"id": "pick", "kind": "pickup", "unit": "cargo_1", "payload": "p1"},
                              {"id": "go", "kind": "deliver", "unit": "cargo_1", "payload": "p1", "zone": "delivery",
                               "after": ["pick"]}]},
        "around": {"expected": "depends", "note": "via the hard-tile bypass north of the patch",
                   "steps": [{"id": "pick", "kind": "pickup", "unit": "cargo_1", "payload": "p1"},
                             {"id": "via_w", "kind": "goto", "unit": "cargo_1", "goal": [x0 - 0.5, p["bypass_y"], 0.0],
                              "after": ["pick"]},
                             {"id": "via_e", "kind": "goto", "unit": "cargo_1", "goal": [x1 + 0.5, p["bypass_y"], 0.0],
                              "after": ["via_w"]},
                             {"id": "go", "kind": "deliver", "unit": "cargo_1", "payload": "p1", "zone": "delivery",
                              "after": ["via_e"]}]}}
    return sc, _task(sc, "cargo_1", round(t_nominal * p["deadline_factor"], 1), plans, "through",
                     [{"payload_in_zone": {"payload": "p1", "zone": "delivery"}}],
                     believed_floor="hard_tile")


def aisle_traffic(seed, split):
    rng = np.random.default_rng([5, seed, 0 if split == "train" else 1])
    for _ in range(100):
        if split == "train":
            p = {"n_aisles": 2, "n_units": int(rng.choice([3, 4]))}
        else:
            p = {"n_aisles": int(rng.choice([2, 3])), "n_units": int(rng.choice([4, 5, 6]))}
        p["aisle_w"] = round(_u(rng, 0.55, 0.72), 3)
        if in_split("aisle_traffic", p) == split:
            break
    sc = Scene("aisle_traffic", seed, split, p)
    X, Y = 10.0, 7.0
    n, aw = p["n_aisles"], p["aisle_w"]
    # shelf rows run east-west between x = 3 and x = 7; aisles between them
    rows = n + 1
    span = Y - 1.2
    shelf_d = (span - n * aw) / rows
    y = 0.6
    aisles = []
    for k in range(rows):
        sc.box("row_%d" % k, "shelf", 3.0, y, 7.0, y + shelf_d, 0.0, 1.8)
        y += shelf_d
        if k < n:
            aisles.append((y, y + aw))
            sc.zone("aisle_%d" % k, "aisle", [2.6, y, 7.4, y + aw], capacity=1)
            y += aw
    sc.zone("west_hall", "staging", [0.3, 0.3, 2.3, Y - 0.3])
    sc.zone("east_hall", "delivery", [7.7, 0.3, 9.7, Y - 0.3])
    plans_steps = {"reserved": [], "none": []}
    ego = None
    ys = np.linspace(1.0, Y - 1.0, max(p["n_units"], 2))
    rng.shuffle(ys)
    nw = (p["n_units"] + 1) // 2
    goal_slots = {True: list(rng.permutation(np.linspace(1.0, Y - 1.0, max(nw, 2)))),
                  False: list(rng.permutation(np.linspace(1.0, Y - 1.0, max(p["n_units"] - nw, 2))))}
    for i in range(p["n_units"]):
        west = i % 2 == 0
        sy = float(ys[i])
        start = (1.0 if west else 9.0, sy, 0.0 if west else math.pi)
        uid = "r%d" % i
        sc.unit(uid, "carrier", start)
        ego = ego or uid
        ay = aisles[int(rng.integers(0, n))]
        goal_y = float(goal_slots[west].pop())
        gx = 9.0 if west else 1.0
        sc.zone("goal_%s" % uid, "delivery" if west else "staging", [gx - 0.35, goal_y - 0.35, gx + 0.35, goal_y + 0.35])
        for plan in plans_steps:
            ym = (ay[0] + ay[1]) / 2
            # one waypoint beside the aisle's entry mouth (a waiting robot does not stand in the opposite exit)
            via = [[2.05, _r(ym - 0.45)]] if west else [[7.95, _r(ym + 0.45)]]
            plans_steps[plan].append({"id": "go_%s" % uid, "kind": "goto", "unit": uid, "zone": "goal_%s" % uid,
                                      "via": via, "via_aisle": "aisle_%d" % aisles.index(ay)})
    plans = {"reserved": {"expected": "success", "traffic": "reserved",
                          "note": "aisle reservations (one robot per aisle) plus priority yielding",
                          "steps": plans_steps["reserved"]},
             "none": {"expected": "fail", "traffic": "none", "note": "independent motion, no yielding",
                      "steps": plans_steps["none"]}}
    success = [{"unit_in_zone": {"unit": "r%d" % i, "zone": "goal_r%d" % i}} for i in range(p["n_units"])]
    return sc, _task(sc, ego, 200, plans, "reserved", success)


def charging_contention(seed, split):
    rng = np.random.default_rng([6, seed, 0 if split == "train" else 1])
    for _ in range(100):
        if split == "train":
            p = {"n_units": int(rng.choice([2, 3])), "n_bays": 2}
        else:
            p = {"n_units": int(rng.choice([4, 5])), "n_bays": int(rng.choice([2, 3]))}
        if in_split("charging_contention", p) == split:
            break
    sc = Scene("charging_contention", seed, split, p)
    X, Y = 10.0, 7.0
    bays = []
    for k in range(p["n_bays"]):
        by = 1.2 + k * 2.3
        sc.zone("bay_%d" % k, "charging", [0.15, by - 0.3, 0.75, by + 0.3], capacity=1, required_yaw=0.0)
        sc.box("bay_post_%d_s" % k, "charger", 0.0, by - 0.42, 0.25, by - 0.32, 0.0, 0.6)
        sc.box("bay_post_%d_n" % k, "charger", 0.0, by + 0.32, 0.25, by + 0.42, 0.0, 0.6)
        sc.tag("lm_bay_%d" % k, [0.002, _r(by), 0.0], "bay", z=0.2)
        bays.append("bay_%d" % k)
    sc.box("rack_mid", "rack", 5.0, 2.8, 6.0, 4.2, 0.0, 1.5)
    socs = sorted(float(rng.uniform(0.12, 0.3)) for _ in range(p["n_units"]))
    steps_near, steps_bal = [], []
    for i in range(p["n_units"]):
        uid = "c%d" % i
        # the robots arrive clustered near the first bay (so nearest-bay greedy piles up there)
        sc.unit(uid, "carrier", (2.2 + 0.7 * (i % 3), 0.8 + 0.6 * (i // 3) + 0.25 * (i % 2), math.pi), soc=socs[i])
        h = (Y - 1.2) / p["n_units"]
        sc.zone("work_%s" % uid, "delivery", [8.4, 0.6 + i * h, 9.4, 0.6 + (i + 1) * h - 0.1])
        steps_near += [{"id": "charge_%s" % uid, "kind": "charge", "unit": uid, "bays": bays, "policy": "nearest",
                        "target_soc": 0.8, "then_zone": "work_%s" % uid}]
        steps_bal += [{"id": "charge_%s" % uid, "kind": "charge", "unit": uid, "bay": bays[i % len(bays)],
                       "policy": "assigned", "target_soc": 0.8, "then_zone": "work_%s" % uid}]
    plans = {"nearest": {"expected": "fail", "note": "each robot queues at its nearest bay", "steps": steps_near},
             "assigned": {"expected": "success", "note": "round-robin bay assignment", "steps": steps_bal}}
    success = [{"unit_in_zone": {"unit": "c%d" % i, "zone": "work_c%d" % i}} for i in range(p["n_units"])] + \
              [{"min_soc": 0.5}]
    per_slot = 40.0   # charge 0.12..0.3 -> 0.8 at 0.02/s, plus entering the bay
    deadline = 60.0 + per_slot * math.ceil(p["n_units"] / p["n_bays"]) + 15.0
    return sc, _task(sc, "c0", round(deadline, 1), plans, "assigned", success, stuck=90.0)


def blocked_faces(seed, split):
    rng = np.random.default_rng([7, seed, 0 if split == "train" else 1])
    p = {"n_blocked": 1 if split == "train" else int(rng.choice([2, 3])), "tx": round(_u(rng, 3.5, 5.0), 3),
         "ty": round(_u(rng, 2.5, 4.5), 3)}
    sc = Scene("blocked_faces", seed, split, p)
    tx, ty = p["tx"], p["ty"]
    # target cargo_1 at (tx, ty) yaw 0; pallets block faces in this order: rear, left, right (front stays free)
    blockers = [("rear", (tx - 0.21 - 0.18 - 0.5, ty - 0.35, tx - 0.21 - 0.18, ty + 0.35)),
                ("left", (tx - 0.35, ty + 0.2 + 0.15, tx + 0.35, ty + 0.2 + 0.55)),
                ("right", (tx - 0.35, ty - 0.2 - 0.55, tx + 0.35, ty - 0.2 - 0.15))]
    blocked = [b[0] for b in blockers[:p["n_blocked"]]]
    for name, (x0, y0, x1, y1) in blockers[:p["n_blocked"]]:
        sc.box("pallet_%s" % name, "pallet", x0, y0, x1, y1, 0.0, 0.9)
    p["blocked"] = blocked
    sc.family_note = blocked
    sc.zone("staging", "staging", [0.5, 0.5, 2.5, 2.0])
    sc.zone("loading", "loading", [6.5, 5.3, 8.0, 6.5])
    sc.zone("delivery", "delivery", [8.2, 1.0, 9.6, 3.0])
    sc.unit("cargo_1", "carrier", (tx, ty, 0.0))
    sc.unit("cargo_2", "carrier", (1.2, 1.2, 0.0))
    sc.payload("p1", "box_long", (7.25, 5.9, 0.0), zone_id="loading")
    tail = [{"id": "pick", "kind": "pickup", "unit": "pair", "payload": "p1", "after": ["couple"]},
            {"id": "go", "kind": "deliver", "unit": "pair", "payload": "p1", "zone": "delivery", "after": ["pick"]}]
    plans = {
        "fixed_face": {"expected": "fail", "note": "the task's default mating: cargo_2.front -> cargo_1.rear (blocked)",
                       "steps": [{"id": "couple", "kind": "dock", "docker": "cargo_2", "docker_face": "front",
                                  "target": "cargo_1", "target_face": "rear", "as": "pair", "as_catalog": "pair_chain"}] + tail},
        "free_face": {"expected": "success", "note": "mate the free front face: cargo_2.rear -> cargo_1.front",
                      "steps": [{"id": "couple", "kind": "dock", "docker": "cargo_2", "docker_face": "rear",
                                 "target": "cargo_1", "target_face": "front", "as": "pair", "as_catalog": "pair_chain"}] + tail},
        "move_target": {"expected": "success", "note": "cargo_1 drives out first, then the default mating",
                        "steps": [{"id": "clear", "kind": "goto", "unit": "cargo_1", "goal": [6.0, 3.5, 0.0]},
                                  {"id": "couple", "kind": "dock", "docker": "cargo_2", "docker_face": "front",
                                   "target": "cargo_1", "target_face": "rear", "as": "pair", "as_catalog": "pair_chain",
                                   "after": ["clear"]}] + tail}}
    return sc, _task(sc, "cargo_2", 260, plans, "free_face", [{"payload_in_zone": {"payload": "p1", "zone": "delivery"}}])


def wheel_failure(seed, split):
    rng = np.random.default_rng([8, seed, 0 if split == "train" else 1])
    for _ in range(100):
        if split == "train":
            p = {"n_faults": 1, "t_fail": round(_u(rng, 20.0, 35.0), 2)}
        else:
            p = {"n_faults": int(rng.choice([1, 2])), "t_fail": round(_u(rng, 30.0, 45.0), 2)}
        p["mode"] = str(rng.choice(["dead_locked", "weak"]))
        p["door_w"] = round(_u(rng, 1.2, 1.6), 3)   # wide: a robot that dies in it does not plug it
        if in_split("wheel_failure", p) == split:
            break
    sc = Scene("wheel_failure", seed, split, p)
    sc.wall("divider", (6.0, 0.0), (6.0, 7.0))
    sc.door("door_main", "divider", 3.5, p["door_w"])
    sc.zone("staging", "staging", [0.5, 0.5, 2.5, 2.0])
    sc.zone("loading", "loading", [0.6, 4.5, 2.0, 6.0])
    sc.zone("delivery", "delivery", [8.2, 2.5, 9.6, 4.5])
    sc.payload("p1", "box_small", (1.3, 5.2, 0.0), zone_id="loading")
    sc.unit("cargo_1", "carrier", (1.2, 3.2, math.pi / 2))
    sc.unit("cargo_2", "carrier", (1.2, 1.0, 0.0))
    sc.fault("cargo_1", "wheel_fl", p["mode"], p["t_fail"])
    if p["n_faults"] == 2:
        sc.fault("cargo_2", "wheel_fl", "weak", p["t_fail"] + 10.0)
    base = [{"id": "pick", "kind": "pickup", "unit": "cargo_1", "payload": "p1"}]
    plans = {
        "continue": {"expected": "fail" if p["mode"] == "dead_locked" else "depends",
                     "note": "the faulty carrier keeps the payload",
                     "steps": base + [{"id": "go", "kind": "deliver", "unit": "cargo_1", "payload": "p1",
                                       "zone": "delivery", "after": ["pick"]}]},
        "reassign": {"expected": "success", "note": "on the fault, the spare takes the payload over (handoff) and delivers",
                     "steps": base + [{"id": "go", "kind": "deliver", "unit": "cargo_1", "payload": "p1", "zone": "delivery",
                                       "after": ["pick"], "abort_on_fault": "cargo_1"},
                                      {"id": "take", "kind": "handoff", "from": "cargo_1", "to": "cargo_2", "payload": "p1",
                                       "when": {"fault": "cargo_1"}, "after": ["pick"]},
                                      {"id": "go2", "kind": "deliver", "unit": "cargo_2", "payload": "p1",
                                       "zone": "delivery", "when": {"fault": "cargo_1"}, "after": ["take"]}]}}
    return sc, _task(sc, "cargo_1", 150, plans, "reassign", [{"payload_in_zone": {"payload": "p1", "zone": "delivery"}}])


def clutter_observer(seed, split):
    rng = np.random.default_rng([9, seed, 0 if split == "train" else 1])
    LZ = [5.5, 2.0, 9.4, 6.4]
    for _ in range(200):
        if split == "train":
            p = {"overhead": str(rng.choice(["working", "occluded"])), "n_clutter": int(rng.integers(4, 7))}
        else:
            p = {"overhead": str(rng.choice(["working", "occluded", "disabled"])), "n_clutter": int(rng.integers(5, 10))}
        if in_split("clutter_observer", p) != split:
            continue
        # payload first, then clutter that keeps a rover-wide lane (0.5 m) around everything
        px, py = _u(rng, 6.6, 8.6), _u(rng, 3.0, 5.6)
        placed, boxes = [(px - 0.85, py - 0.85, px + 0.85, py + 0.85)], []
        for _ in range(600):
            if len(boxes) >= p["n_clutter"]:
                break
            w, d, h = _u(rng, 0.35, 0.7), _u(rng, 0.35, 0.7), _u(rng, 0.25, 0.8)
            x, y = _u(rng, LZ[0] + 0.15, LZ[2] - 0.15 - w), _u(rng, LZ[1] + 0.15, LZ[3] - 0.15 - d)
            r = (x, y, x + w, y + d)
            if any(not (r[2] + 0.5 < q[0] or r[0] - 0.5 > q[2] or r[3] + 0.5 < q[1] or r[1] - 0.5 > q[3]) for q in placed):
                continue
            placed.append(r)
            boxes.append((r, h))
        if len(boxes) == p["n_clutter"]:
            break
    else:
        raise RuntimeError("clutter_observer seed %d: no layout" % seed)
    sc = Scene("clutter_observer", seed, split, p)
    sc.zone("staging", "staging", [0.4, 0.4, 2.2, 2.2])
    sc.zone("loading", "loading", LZ)
    sc.zone("delivery", "delivery", [0.4, 4.8, 2.0, 6.5])
    for k, (r, h) in enumerate(boxes):
        sc.box("clutter_%d" % k, "clutter", *r, 0.0, h)
    # a ring of tall clutter on the side facing the carrier's route makes the low camera's view poor
    # a low screen on the side facing the carrier's approach: the mast camera (0.45 m) sees over it, a deck camera
    # (0.15 m) does not
    sc.box("screen_w", "clutter", LZ[0] - 0.05, py - 1.0, LZ[0] + 0.25, py + 1.0, 0.0, 0.4)
    sc.camera("oh_load", (7.5, 4.2, 2.6), (7.5, 4.2, 0.0), p["overhead"] if p["overhead"] != "occluded" else "working",
              hfov=80.0, rng=6.0)
    if p["overhead"] == "occluded":   # the occlusion is geometry: a hanging banner under the camera
        sc.box("banner", "banner", 6.9, 3.6, 8.1, 4.8, 2.1, 2.3)
        sc.d["overhead_cameras"][-1]["mode_declared"] = "occluded"
    sc.payload("p1", "box_small", (px, py, float(rng.choice([0.0, math.pi / 2]))), zone_id="loading", pose_known=False)
    sc.unit("scout_1", "scout", (1.2, 1.2, 0.0))
    sc.unit("cargo_1", "carrier", (1.4, 3.2, 0.0))
    vps = [[_r(LZ[0] - 0.4), _r(py), 0.0], [_r((LZ[0] + LZ[2]) / 2), _r(LZ[1] - 0.35), round(math.pi / 2, 6)],
           [_r((LZ[0] + LZ[2]) / 2), _r(LZ[3] + 0.3), round(-math.pi / 2, 6)], [_r(LZ[2] + 0.3), _r(py), round(math.pi, 6)]]
    vps = [v for v in vps if 0.3 < v[0] < 9.7 and 0.3 < v[1] < 6.7]
    tail = [{"id": "pick", "kind": "pickup", "unit": "cargo_1", "payload": "p1", "after": ["look"]},
            {"id": "go", "kind": "deliver", "unit": "cargo_1", "payload": "p1", "zone": "delivery", "after": ["pick"]}]
    plans = {
        "observer": {"expected": "depends", "note": "the scout (mast camera 0.45 m) surveys; the carrier waits, then picks",
                     "steps": [{"id": "look", "kind": "observe", "unit": "scout_1", "payload": "p1", "viewpoints": vps,
                                "dwell_s": 1.5}] + tail},
        "self_search": {"expected": "depends", "note": "the carrier (camera 0.15 m) searches the same viewpoints itself",
                        "steps": [{"id": "look", "kind": "observe", "unit": "cargo_1", "payload": "p1", "viewpoints": vps,
                                   "dwell_s": 1.5}] + tail}}
    return sc, _task(sc, "cargo_1", 220, plans, "observer", [{"payload_in_zone": {"payload": "p1", "zone": "delivery"}}])


GENERATORS = {"narrow_door": narrow_door, "long_payload_door": long_payload_door, "lateral_slot": lateral_slot,
              "friction_patches": friction_patches, "aisle_traffic": aisle_traffic,
              "charging_contention": charging_contention, "blocked_faces": blocked_faces,
              "wheel_failure": wheel_failure, "clutter_observer": clutter_observer}


def generate(family, seed, split="train"):
    sc, task = GENERATORS[family](int(seed), split)
    scn = sc.d
    actual = in_split(family, scn["family"]["params"])
    if actual != split:
        raise ValueError("%s seed %d: parameters fall in %s, not %s" % (family, seed, actual, split))
    task["scenario_sha256"] = sha(scn, 64)
    return scn, task
