"""Held-out split definitions, fixed before any learning (EXPLORATORY).

Every split is a pure function of a (course, robot configuration) pair that reads only the family
name, the integer course parameters and the integer robot configuration. It returns one of
``train``, ``test_interp``, ``test_extrap``, ``buffer``. The specification of each split is a JSON
object whose sha256 is recorded in ``RECORDED_SPEC_HASHES``; ``RECORDED_ASSIGNMENT_HASH`` pins the
behaviour of the functions on a fixed seeded reference sample. A test fails if either drifts.

Interpolation / extrapolation is declared with respect to the RAW variable the split is defined
on. Whether a raw extrapolation is still an interpolation in the dimensionless interaction
features is the hypothesis these splits exist to test; it is not assumed.
"""
from __future__ import annotations

import hashlib
import json

from . import envs as E
from . import robotspace as RS

TRAIN, INTERP, EXTRAP, BUFFER = "train", "test_interp", "test_extrap", "buffer"

SPLITS_V0 = {
    "iid": {
        "kind": "interpolation",
        "rule": "sha256(course_id + ':' + config_id) mod 5 == 0 -> test_interp",
        "modulus": 5,
    },
    "family": {
        "kind": "extrapolation",
        "rule": "family in train_families -> train, else test_extrap",
        "train_families": ["chicane", "lbend", "offset_doorway"],
        "test_families": ["cluttered", "dead_end", "occluded_doorway"],
    },
    "param_range": {
        "kind": "both",
        "rule": "per family one key parameter (cm); value inside 'extrap' (an end of the admissible box) -> "
                "test_extrap; inside 'interp' (an interior band) -> test_interp; else train. Bounds inclusive.",
        "keys": {
            "offset_doorway": {"key": "d_cm", "extrap": [84, 96], "interp": [118, 128]},
            "chicane": {"key": "s_cm", "extrap": [60, 99], "interp": [150, 180]},
            "lbend": {"key": "min(a_cm,b_cm)", "extrap": [80, 99], "interp": [125, 140]},
            "dead_end": {"key": "Bw_cm", "extrap": [80, 104], "interp": [140, 160]},
            "occluded_doorway": {"key": "ow_cm", "extrap": [120, 140], "interp": [75, 90]},
            "cluttered": {"key": "gmin_cm", "extrap": [70, 84], "interp": [105, 118]},
        },
    },
    "robot_size": {
        "kind": "extrapolation",
        "rule": "bounding box: L <= L_train and W <= W_train -> train; L >= L_test or W >= W_test -> test_extrap; else buffer",
        "L_train_mm": 1000, "W_train_mm": 720, "L_test_mm": 1100, "W_test_mm": 780,
    },
    "module_combination": {
        "kind": "both",
        "rule": "arrangement in train -> train; in interp (between trained line lengths) -> test_interp; "
                "in extrap (unseen shape class) -> test_extrap",
        "train": ["2x2", "L3", "line1", "line2", "line4", "rect"],
        "interp": ["line3"],
        "extrap": ["2x3", "L4", "row2"],
    },
    "sensor": {
        "kind": "both",
        "rule": "fov <= fov_poor or range <= range_poor -> test_extrap (poorer sensing than any trained); "
                "the held-out (fov, range) pair -> test_interp; else train",
        "fov_poor_deg": 120, "range_poor_mm": 4000, "held_out_pair": [270, 12000],
    },
    "drive": {
        "kind": "extrapolation",
        "rule": "drive in train_drives -> train, else test_extrap",
        "train_drives": ["differential"],
    },
}


def spec_hash(name) -> str:
    return hashlib.sha256(json.dumps(SPLITS_V0[name], sort_keys=True, separators=(",", ":")).encode()).hexdigest()


def _in(v, rng):
    return rng[0] <= v <= rng[1]


def assign(name, course, robot) -> str:
    """Deterministic assignment of one (course, robot) pair under split ``name``."""
    spec = SPLITS_V0[name]
    rc = RS.as_config(robot)
    if name == "iid":
        h = hashlib.sha256(("%s:%s" % (course.course_id, rc.config_id)).encode()).hexdigest()
        return INTERP if int(h[:12], 16) % spec["modulus"] == 0 else TRAIN
    if name == "family":
        return TRAIN if course.family in spec["train_families"] else EXTRAP
    if name == "param_range":
        k = spec["keys"][course.family]
        p = course.p
        v = min(p["a_cm"], p["b_cm"]) if k["key"] == "min(a_cm,b_cm)" else p[k["key"]]
        return EXTRAP if _in(v, k["extrap"]) else (INTERP if _in(v, k["interp"]) else TRAIN)
    if name == "robot_size":
        L, W = round(rc.L * 1000), round(rc.W * 1000)
        if L >= spec["L_test_mm"] or W >= spec["W_test_mm"]:
            return EXTRAP
        return TRAIN if (L <= spec["L_train_mm"] and W <= spec["W_train_mm"]) else BUFFER
    if name == "module_combination":
        a = rc.arrangement
        return TRAIN if a in spec["train"] else (INTERP if a in spec["interp"] else EXTRAP)
    if name == "sensor":
        if rc.lidar_fov_deg <= spec["fov_poor_deg"] or rc.lidar_range_mm <= spec["range_poor_mm"]:
            return EXTRAP
        return INTERP if [rc.lidar_fov_deg, rc.lidar_range_mm] == spec["held_out_pair"] else TRAIN
    if name == "drive":
        return TRAIN if rc.drive in spec["train_drives"] else EXTRAP
    raise KeyError(name)


def assign_all(course, robot) -> dict:
    return {name: assign(name, course, robot) for name in SPLITS_V0}


def leave_one_family_out(held_out, course) -> str:
    """Rotation of the family split: one family is the test set, the other five train."""
    if held_out not in E.FAMILIES:
        raise KeyError(held_out)
    return EXTRAP if course.family == held_out else TRAIN


REFERENCE = {"courses_per_family": 12, "course_seed": 20261110, "robots": 40, "robot_seed": 7}


def reference_pairs():
    """The fixed sample the assignment hash is computed on (432 pairs: each of the 72 courses with 6 of the 40 robots, cycled)."""
    courses = E.sample_all(REFERENCE["courses_per_family"], REFERENCE["course_seed"])
    robots = RS.sample_robots(REFERENCE["robots"], REFERENCE["robot_seed"])
    return [(c, robots[(7 * i + j) % len(robots)]) for i, c in enumerate(courses) for j in range(len(robots) // 6)]


def assignment_hash(pairs=None) -> str:
    pairs = pairs if pairs is not None else reference_pairs()
    lines = ["%s|%s|%s" % (c.course_id, r.config_id, ",".join(assign(n, c, r) for n in sorted(SPLITS_V0))) for c, r in pairs]
    return hashlib.sha256("\n".join(lines).encode()).hexdigest()


def counts(pairs=None) -> dict:
    pairs = pairs if pairs is not None else reference_pairs()
    out = {}
    for n in SPLITS_V0:
        out[n] = {}
        for c, r in pairs:
            a = assign(n, c, r)
            out[n][a] = out[n].get(a, 0) + 1
    return out


# Recorded when the splits were frozen (before any learner saw a label). Do not edit to make a test pass:
# a changed hash means the split changed, and results across the change are not comparable.
RECORDED_SPEC_HASHES = {
    "iid": "b6be0a9b554141d2791e91749d7219ff0c5449f17ed452ff2906aea04a0f0a78",
    "family": "725f29ec86a76fb0194f1df6946d60d82f509e8b96b3e4164adfc5ab83902e72",
    "param_range": "14dbe45bb5eea2d6db72026e357c4ee3ec82a3bb913b6cefb5439271885f1bca",
    "robot_size": "d3130dc41474f175993452c54e15fe1d2a1365f6ed9aaedb38198912940f5f9a",
    "module_combination": "2b5d52d1236491ba9d6c26af03d083f077298fbefda6464126a583b64a0e024c",
    "sensor": "d52570d7026f7a25ca5a127016a9c1e1f5c6030a94ef1af4f8090a03a14ed72d",
    "drive": "54e9c4e41cb72d38e0de4180ee6f07a8162c9cb46359f79e2a8d45fce3e239b6",
}
RECORDED_ASSIGNMENT_HASH = "cc214104d1c47a84bb0986008556cd2a42fe0db191b673294530ec1d9a1cad63"
