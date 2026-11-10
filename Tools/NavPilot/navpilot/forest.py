# SPDX-License-Identifier: GPL-3.0-or-later
"""Seeded forest stand layouts for the navigation pilot (2.5D flight slab).

Schema `crane-navpilot-stand/1` (JSON-serialisable dict):
  stems:    [{id, x, y, dbh_m, height_m}]                world ENU metres
  branches: [{id, stem, p0:[x,y,z], p1:[x,y,z], d0_m, d1_m}]  thin branches in the 1-4 m flight band
  targets:  [{id, x, y, size_m}]                         person targets on the ground
  corridor: {length_m, half_width_m, start:[x,y], goal:[x,y]}
  family, seed, split, generator
It is a subset of what TreeGen stands will carry (stem DBH/height/position + capsule branches), so
`load_stand` also accepts a TreeGen stand file if one appears (`trees[]` + `branches[]`/`colliders[]`).

Splits come from Tools/DroneData/splits.json (crane-drone-splits/1) through Tools/DroneData/splits.py:
dev 0-1999, train 2000-69999, val 70000-79999, test 80000-89999, HELD-OUT 90000-99999.
Held-out seeds are refused unless allow_heldout=True (CLI: --allow-heldout).

RNG: np.random.Generator(PCG64(SeedSequence([seed, stream, sub]))) per stream, the TreeGen convention, so
changing the branch density never moves a stem (branches are drawn at the maximum count and truncated).
"""
import hashlib
import json
import math
import os
import sys

import numpy as np

SCHEMA = "crane-navpilot-stand/1"
GENERATOR = "navpilot.forest/1"
_HERE = os.path.dirname(os.path.abspath(__file__))
_DRONEDATA = os.path.normpath(os.path.join(_HERE, "..", "..", "DroneData"))
if _DRONEDATA not in sys.path:
    sys.path.insert(0, _DRONEDATA)
import splits as _splits  # noqa: E402  (Tools/DroneData/splits.py, owned by the logging agent)

HeldoutRefused = _splits.HeldoutRefused

# Family axes. Stem densities span open mature stands to young dense regrowth [U: priors, not survey data].
STEM_DENSITY = {"sparse": 400.0, "medium": 800.0, "dense": 1400.0}      # stems / ha
BRANCH_PER_STEM = {"none": 0, "low": 3, "mid": 8, "high": 16}            # branches per stem in the 1-4 m band
CLEARINGS = {"none": 0, "some": 2}
MAX_BRANCH_PER_STEM = 30
HARD_D_MIN = 0.02
STREAM_STEMS, STREAM_BRANCHES, STREAM_CLEARINGS, STREAM_TARGETS = 1, 2, 3, 4


def _rng(seed, stream, sub=0):
    return np.random.Generator(np.random.PCG64(np.random.SeedSequence([int(seed) & 0xFFFFFFFF, stream, sub])))


def guard_seed(seed, wind=None, allow_heldout=False):
    """Split of a layout seed, refusing HELD-OUT/unassigned seeds or held-out wind unless allowed."""
    return _splits.enforce(_splits.load(), int(seed), wind=wind, allow_heldout=allow_heldout)


def family_spec(stem="medium", branch="mid", clearing="none", slope_deg=0.0):
    return {"stem": stem, "branch": branch, "clearing": clearing, "slope_deg": float(slope_deg),
            "stems_per_ha": STEM_DENSITY[stem], "branches_per_stem": BRANCH_PER_STEM[branch],
            "n_clearings": CLEARINGS[clearing]}


def generate_stand(seed, family=None, length_m=60.0, half_width_m=12.0, n_targets=6,
                   wind=None, allow_heldout=False):
    """Generate a seeded stand manifest. Raises HeldoutRefused for held-out seeds without the flag."""
    split = guard_seed(seed, wind=wind, allow_heldout=allow_heldout)
    fam = dict(family or family_spec())
    L, hw = float(length_m), float(half_width_m)
    x0, x1, y0, y1 = -6.0, L + 8.0, -hw - 4.0, hw + 4.0
    start, goal = (0.0, 0.0), (L, 0.0)

    rc = _rng(seed, STREAM_CLEARINGS)
    clear = []
    for _ in range(fam["n_clearings"]):
        clear.append((rc.uniform(10.0, L - 8.0), rc.uniform(-0.5 * hw, 0.5 * hw), rc.uniform(5.0, 9.0)))

    rs = _rng(seed, STREAM_STEMS, int(fam["stems_per_ha"]))
    area = (x1 - x0) * (y1 - y0)
    n_want = int(round(fam["stems_per_ha"] * area / 1e4))
    cand = np.column_stack([rs.uniform(x0, x1, 6 * n_want + 10), rs.uniform(y0, y1, 6 * n_want + 10)])
    dbh_c = np.clip(np.exp(rs.normal(math.log(0.24), 0.45, len(cand))), 0.08, 0.85)
    stems = []
    cell = 2.0
    grid = {}
    for (cx, cy), d in zip(cand, dbh_c):
        if len(stems) >= n_want:
            break
        if math.hypot(cx - start[0], cy - start[1]) < 3.0 or math.hypot(cx - goal[0], cy - goal[1]) < 3.0:
            continue
        if any(math.hypot(cx - a, cy - b) < r for a, b, r in clear):
            continue
        gi, gj = int((cx - x0) // cell), int((cy - y0) // cell)
        ok = True
        for di in (-1, 0, 1):
            for dj in (-1, 0, 1):
                for k in grid.get((gi + di, gj + dj), ()):
                    s = stems[k]
                    if math.hypot(cx - s["x"], cy - s["y"]) < 0.5 * (d + s["dbh_m"]) + 0.6:
                        ok = False
                        break
                if not ok:
                    break
            if not ok:
                break
        if not ok:
            continue
        h = 1.3 + 28.0 * (1.0 - math.exp(-3.5 * d))  # Chapman-Richards-like allometry [U]
        grid.setdefault((gi, gj), []).append(len(stems))
        stems.append({"id": len(stems), "x": float(cx), "y": float(cy), "dbh_m": float(d), "height_m": float(h)})

    nb = int(fam["branches_per_stem"])
    branches = []
    for s in stems:
        rb = _rng(seed, STREAM_BRANCHES, s["id"])
        z = rb.uniform(1.0, 4.0, MAX_BRANCH_PER_STEM)
        az = rb.uniform(0.0, 2 * math.pi, MAX_BRANCH_PER_STEM)
        ln = rb.uniform(0.6, 3.0, MAX_BRANCH_PER_STEM) * (1.0 + 0.8 * s["dbh_m"])
        pitch = np.radians(rb.uniform(-15.0, 25.0, MAX_BRANCH_PER_STEM))
        d0 = rb.uniform(0.006, 0.045, MAX_BRANCH_PER_STEM)
        for k in range(nb):
            r0 = 0.5 * s["dbh_m"]
            p0 = [s["x"] + r0 * math.cos(az[k]), s["y"] + r0 * math.sin(az[k]), float(z[k])]
            c = math.cos(pitch[k])
            p1 = [p0[0] + ln[k] * c * math.cos(az[k]), p0[1] + ln[k] * c * math.sin(az[k]),
                  p0[2] + ln[k] * math.sin(pitch[k])]
            if any(_seg_point_dist(p0, p1, q) < 2.0 for q in (start, goal)):
                continue  # keep the launch and goal pads clear (slot stays consumed, so draws never shift)
            branches.append({"id": len(branches), "stem": s["id"], "p0": [float(v) for v in p0],
                             "p1": [float(v) for v in p1], "d0_m": float(d0[k]), "d1_m": float(0.3 * d0[k])})

    rt = _rng(seed, STREAM_TARGETS)
    targets = []
    tries = 0
    while len(targets) < n_targets and tries < 1000:
        tries += 1
        tx = rt.uniform(8.0, L - 4.0)
        ty = rt.choice([-1.0, 1.0]) * rt.uniform(1.5, 0.85 * hw)
        if any(math.hypot(tx - s["x"], ty - s["y"]) < 0.5 * s["dbh_m"] + 0.6 for s in stems):
            continue
        targets.append({"id": len(targets), "x": float(tx), "y": float(ty), "size_m": 0.8})

    m = {"schema": SCHEMA, "generator": GENERATOR, "seed": int(seed), "split": split, "family": fam,
         "corridor": {"length_m": L, "half_width_m": hw, "start": list(start), "goal": list(goal),
                      "bounds": [x0, x1, y0, y1]},
         "clearings": [{"x": a, "y": b, "r": r} for a, b, r in clear],
         "stems": stems, "branches": branches, "targets": targets}
    return m


def _seg_point_dist(p0, p1, q):
    ax, ay = p1[0] - p0[0], p1[1] - p0[1]
    L2 = ax * ax + ay * ay
    s = 0.0 if L2 == 0 else max(0.0, min(1.0, ((q[0] - p0[0]) * ax + (q[1] - p0[1]) * ay) / L2))
    return math.hypot(q[0] - p0[0] - s * ax, q[1] - p0[1] - s * ay)


def manifest_sha256(m):
    return hashlib.sha256(json.dumps(m, sort_keys=True, separators=(",", ":")).encode()).hexdigest()


def load_stand(path, allow_heldout=False):
    """Load a navpilot stand, or best-effort adapt a TreeGen stand manifest (trees + capsule branches)."""
    with open(path) as f:
        m = json.load(f)
    if m.get("schema") == SCHEMA:
        guard_seed(m["seed"], allow_heldout=allow_heldout)
        return m
    trees = m.get("trees") or m.get("stems")
    if trees is None:
        raise ValueError("unrecognised stand manifest %s (schema %r)" % (path, m.get("schema")))
    seed = int(m.get("seed", m.get("forest_seed", -1)))
    split = guard_seed(seed, allow_heldout=allow_heldout)
    stems = []
    for i, t in enumerate(trees):
        pos = t.get("position") or t.get("pos") or [t["x"], t["y"]]
        stems.append({"id": i, "x": float(pos[0]), "y": float(pos[1]),
                      "dbh_m": float(t.get("dbh_m", t.get("dbh", 0.25))),
                      "height_m": float(t.get("height_m", t.get("height", 15.0)))})
    branches = []
    for i, b in enumerate(m.get("branches") or m.get("colliders") or []):
        d0 = float(b.get("d0_m", 2 * b.get("radius", 0.01)))
        branches.append({"id": i, "stem": int(b.get("stem", b.get("tree", -1))), "p0": list(b["p0"]),
                         "p1": list(b["p1"]), "d0_m": d0, "d1_m": float(b.get("d1_m", d0))})
    xs = [s["x"] for s in stems]
    L = float(m.get("corridor", {}).get("length_m", max(xs) - 8.0 if xs else 60.0))
    hw = float(m.get("corridor", {}).get("half_width_m", 12.0))
    return {"schema": SCHEMA, "generator": "adapted:" + str(m.get("schema")), "seed": seed, "split": split,
            "family": m.get("family", {}), "corridor": {"length_m": L, "half_width_m": hw, "start": [0.0, 0.0],
            "goal": [L, 0.0], "bounds": [-6.0, L + 8.0, -hw - 4.0, hw + 4.0]},
            "clearings": [], "stems": stems, "branches": branches, "targets": m.get("targets", [])}


class World:
    """2.5D collision/sensing geometry: every obstacle is a chain of circles in the flight slab.

    Stems are one circle (r = DBH/2). A branch contributes circles every `step` m along its 3D axis where
    |z - fly_h| < slab_half (the part of the branch that can touch the airframe at cruise altitude).
    Objects keep an id so the sensor models can draw one detection per object per frame.
    """

    def __init__(self, stand, fly_h=2.0, slab_half=0.35, step=0.1, cell=2.0):
        self.stand = stand
        self.fly_h, self.slab_half = fly_h, slab_half
        c = stand["corridor"]
        self.length, self.half_width = c["length_m"], c["half_width_m"]
        self.start, self.goal = np.array(c["start"], float), np.array(c["goal"], float)
        cx, cy, cr, co = [], [], [], []
        okind, odiam, olen = [], [], []
        for s in stand["stems"]:
            cx.append(s["x"]); cy.append(s["y"]); cr.append(0.5 * s["dbh_m"]); co.append(len(okind))
            okind.append(0); odiam.append(s["dbh_m"]); olen.append(2.0)
        for b in stand["branches"]:
            p0, p1 = np.array(b["p0"]), np.array(b["p1"])
            n = max(2, int(math.ceil(np.linalg.norm(p1 - p0) / step)) + 1)
            s = np.linspace(0.0, 1.0, n)
            pts = p0[None, :] + s[:, None] * (p1 - p0)[None, :]
            keep = np.abs(pts[:, 2] - fly_h) < slab_half
            if not keep.any():
                continue
            d = b["d0_m"] + s[keep] * (b["d1_m"] - b["d0_m"])
            oid = len(okind)
            cx.extend(pts[keep, 0]); cy.extend(pts[keep, 1]); cr.extend(0.5 * d); co.extend([oid] * int(keep.sum()))
            okind.append(1); odiam.append(float(d.mean()))
            olen.append(float(keep.sum()) * float(np.linalg.norm((p1 - p0)[:2])) / (n - 1))
        self.cx, self.cy = np.array(cx, float), np.array(cy, float)
        self.cr, self.cobj = np.array(cr, float), np.array(co, np.int64)
        self.okind, self.odiam, self.olen = np.array(okind, np.int8), np.array(odiam), np.array(olen)
        self.n_obj = len(okind)
        t = stand.get("targets", [])
        self.targets = np.array([[q["x"], q["y"], q["size_m"]] for q in t], float).reshape(-1, 3)
        # CSR spatial hash
        x0, x1, y0, y1 = c["bounds"]
        self.ox, self.oy, self.cell = x0 - 2.0, y0 - 2.0, cell
        self.nx = int(math.ceil((x1 - x0 + 4.0) / cell)) + 1
        self.ny = int(math.ceil((y1 - y0 + 4.0) / cell)) + 1
        gi = np.clip(((self.cx - self.ox) // cell).astype(np.int64), 0, self.nx - 1)
        gj = np.clip(((self.cy - self.oy) // cell).astype(np.int64), 0, self.ny - 1)
        cid = gi * self.ny + gj
        order = np.argsort(cid, kind="stable")
        self._perm = order
        counts = np.bincount(cid, minlength=self.nx * self.ny)
        self._end = np.cumsum(counts)
        self._start = self._end - counts
        # pad radius so a sensing ray cannot slip between consecutive branch circles
        # TREEGEN_DESIGN collider contract: diameter >= 2 cm in the 1-4 m band is a hard collider; thinner = soft
        self.hard = (self.okind[self.cobj] == 0) | (2.0 * self.cr >= HARD_D_MIN)
        self.cr_sense = np.where(self.okind[self.cobj] == 1, np.maximum(self.cr, 0.5 * step), self.cr)

    def query(self, x, y, radius):
        """Indices of circles whose centre lies in the axis-aligned box of half-size `radius`."""
        i0 = max(int((x - radius - self.ox) // self.cell), 0)
        i1 = min(int((x + radius - self.ox) // self.cell), self.nx - 1)
        j0 = max(int((y - radius - self.oy) // self.cell), 0)
        j1 = min(int((y + radius - self.oy) // self.cell), self.ny - 1)
        if i1 < i0 or j1 < j0:
            return np.zeros(0, np.int64)
        ids = (np.arange(i0, i1 + 1)[:, None] * self.ny + np.arange(j0, j1 + 1)[None, :]).ravel()
        s, e = self._start[ids], self._end[ids]
        n = e - s
        tot = int(n.sum())
        if tot == 0:
            return np.zeros(0, np.int64)
        off = np.repeat(s - (np.cumsum(n) - n), n) + np.arange(tot)
        return self._perm[off]

    def clearance(self, x, y, radius=1.5):
        """Surface-to-centre clearance (min over circles of dist - r) and the circle index."""
        idx = self.query(x, y, radius)
        if len(idx) == 0:
            return math.inf, -1
        d = np.hypot(self.cx[idx] - x, self.cy[idx] - y) - self.cr[idx]
        k = int(np.argmin(d))
        return float(d[k]), int(idx[k])

    def obstacles_json(self):
        """obstacles.json payload (crane-drone-log/1): stems as cylinders, slab branches as capsules."""
        obs = [{"id": "stem_%d" % s["id"], "type": "cylinder", "center": [s["x"], s["y"], 0.0],
                "radius": 0.5 * s["dbh_m"], "height": s["height_m"]} for s in self.stand["stems"]]
        for b in self.stand["branches"]:
            if min(b["p0"][2], b["p1"][2]) - self.slab_half < self.fly_h < max(b["p0"][2], b["p1"][2]) + self.slab_half:
                obs.append({"id": "branch_%d" % b["id"], "type": "capsule", "p0": b["p0"], "p1": b["p1"],
                            "radius": 0.5 * b["d0_m"]})
        return {"frame": "ENU", "obstacles": obs}
