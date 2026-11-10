# SPDX-License-Identifier: GPL-3.0-or-later
"""Experiment-2 scenario families on top of a seeded NavPilot stand (sparse stems, low branch density).

A *layout* is one stand seed (split-guarded: DEV / HELD-OUT by whole layout). Each family adds a structure across
the corridor at x = X_w (a deadfall/thicket wall rendered as touching 0.30 m columns, hard colliders) and defines
the PRIOR MAP the map-based stacks receive and the estimator processes:

  S1 stale shortcut   wall with a near-axis shortcut gap (2.0-2.6 m) and a detour gap (3 m) 5-8 m to the side.
                      Truth: 2 thin hard branches (2.5-4 cm) span the shortcut. The prior shows it open.
  S2 mirror           near-axis opening (2.6 m) leads into a closed pocket 8-12 m deep; the useful detour gap is
                      7-9 m to one side. The prior is correct (it shows the pocket closed).
  S3 alignment drift  wall with a narrow shortcut gap (1.8-2.2 m) and a 3 m alternative 6-8 m away; prior correct,
                      but the map alignment drifts laterally to 1.2-2.0 m before the wall.
  S4 neither          single narrow gap (1.8-2.0 m) partly crossed by a thin branch, inside an observation-degraded
                      zone (stereo invalid 60%, LiDAR points x0.25), local-estimator drift x10 in the zone, and
                      alignment drift 1.0-1.8 m. Safe behaviour is to slow or stop.

  S0 control          S3 geometry, prior = truth (stems AND branches, no phantoms), no alignment ramp
                      ("unchanged map, no drift"; the P8/T3 regime).

Common prior staleness (S1-S4): no branches at all (missing branches) and ~5% phantom stems that are no
longer there (removed obstacles). Prior log-odds: occupied +2.0, everything else in bounds observed-free -1.0.

Local estimate vs alignment are separate processes (episode loop):
  local:     e(t) += sigma_loc * sqrt(ds) * N(0, I)   (sigma_loc 0.03 m/sqrt(m); x10 inside an S4 degraded zone)
  alignment: delta(t) = small constant N(0, 0.1 m) + family ramp (S3/S4), independent of e(t).
The fresh map is registered at x + e; the prior appears in the planner frame displaced by delta.
"""
import math

import numpy as np

from navpilot.forest import family_spec, generate_stand

FAMILIES = ["S1", "S2", "S3", "S4", "S0"]   # S0 appended last so S1-S4 draws never shift
WALL_D = 0.30
WALL_STEP = 0.25
FLY_H = 2.0


def _rng(seed, fam):
    return np.random.Generator(np.random.PCG64(np.random.SeedSequence([int(seed) & 0xFFFFFFFF, 0xE2, 20 + FAMILIES.index(fam)])))


def _wall_x(x, y0, y1, gaps):
    """Column centres along x = const from y0 to y1 skipping gaps [(yc, width)] (clear width between surfaces)."""
    out = []
    for y in np.arange(y0, y1 + 1e-9, WALL_STEP):
        if any(abs(y - yc) < 0.5 * w + 0.5 * WALL_D for yc, w in gaps):
            continue
        out.append((float(x), float(y)))
    return out


def _wall_y(y, x0, x1):
    return [(float(x), float(y)) for x in np.arange(x0, x1 + 1e-9, WALL_STEP)]


def _seg_pt(p0, p1, q):
    ax, ay = p1[0] - p0[0], p1[1] - p0[1]
    L2 = ax * ax + ay * ay
    s = 0.0 if L2 == 0 else max(0.0, min(1.0, ((q[0] - p0[0]) * ax + (q[1] - p0[1]) * ay) / L2))
    return math.hypot(q[0] - p0[0] - s * ax, q[1] - p0[1] - s * ay)


def build(seed, fam, allow_heldout=False, length_m=60.0, half_width_m=12.0):
    base = generate_stand(seed, family_spec(stem="sparse", branch="low"), length_m=length_m,
                          half_width_m=half_width_m, allow_heldout=allow_heldout)
    r = _rng(seed, fam)
    x0b, x1b, y0b, y1b = base["corridor"]["bounds"]
    Xw = float(r.uniform(26.0, 34.0))
    side = float(r.choice([-1.0, 1.0]))
    clear_boxes = []        # (xa, xb, ya, yb): no base stems / branches
    gap_lanes = []          # (xc, yc, half-width) circles kept free of base clutter
    walls = []
    block_br = []
    info = {"family": fam, "X_w": Xw}
    align = {"kind": "const"}
    loc_zone = None
    obs_zone = None
    if fam == "S1":
        ys, ws = float(r.uniform(-1.5, 1.5)), float(r.uniform(2.0, 2.6))
        yd = ys + side * float(r.uniform(5.0, 8.0))
        walls += _wall_x(Xw, y0b, y1b, [(ys, ws), (yd, 3.0)])
        gap_lanes += [(Xw, ys, 0.5 * ws + 1.2), (Xw, yd, 2.7)]
        d0 = float(r.uniform(0.025, 0.04))
        e = 0.5 * ws + 0.05
        block_br.append({"p0": [Xw - 0.05, ys - e, FLY_H - 0.05], "p1": [Xw + 0.25, ys + e, FLY_H + 0.05], "d": d0})
        d1 = float(r.uniform(0.025, 0.04))
        block_br.append({"p0": [Xw - 0.45, ys + e, FLY_H + 0.05], "p1": [Xw - 0.15, ys - e, FLY_H - 0.05], "d": d1})
        info.update({"shortcut_y": ys, "shortcut_w": ws, "detour_y": yd})
    elif fam == "S2":
        yc, wc, D = float(r.uniform(-1.0, 1.0)), 2.6, float(r.uniform(8.0, 12.0))
        yd = yc + side * float(r.uniform(7.0, 9.0))
        walls += _wall_x(Xw, y0b, y1b, [(yc, wc), (yd, 3.0)])
        e = 0.5 * wc + 0.5 * WALL_D
        walls += _wall_y(yc - e, Xw + WALL_STEP, Xw + D) + _wall_y(yc + e, Xw + WALL_STEP, Xw + D)
        walls += [(Xw + D, float(y)) for y in np.arange(yc - e, yc + e + 1e-9, WALL_STEP)]
        clear_boxes.append((Xw - 4.0, Xw + D + 0.8, yc - e - 0.8, yc + e + 0.8))
        gap_lanes += [(Xw, yc, 0.5 * wc + 1.2), (Xw, yd, 2.7)]
        info.update({"pocket_y": yc, "pocket_depth": D, "detour_y": yd})
    elif fam in ("S3", "S0"):
        ys, ws = float(r.uniform(-1.5, 1.5)), float(r.uniform(1.8, 2.2))
        yd = ys + side * float(r.uniform(6.0, 8.0))
        walls += _wall_x(Xw, y0b, y1b, [(ys, ws), (yd, 3.0)])
        gap_lanes += [(Xw, ys, 0.5 * ws + 1.2), (Xw, yd, 2.7)]
        A = float(r.choice([-1.0, 1.0])) * float(r.uniform(1.2, 2.0))
        if fam == "S3":
            align = {"kind": "ramp", "A": (0.15 * float(r.normal()), A), "x_start": 3.0, "x_full": Xw - 8.0}
        info.update({"shortcut_y": ys, "shortcut_w": ws, "detour_y": yd, "align_A": A if fam == "S3" else 0.0})
    elif fam == "S4":
        ys, ws = float(r.uniform(-3.0, 3.0)), float(r.uniform(1.8, 2.0))
        walls += _wall_x(Xw, y0b, y1b, [(ys, ws)])
        gap_lanes += [(Xw, ys, 0.5 * ws + 1.2)]
        d0 = float(r.uniform(0.025, 0.035))
        sgn = float(r.choice([-1.0, 1.0]))
        block_br.append({"p0": [Xw + 0.1, ys + sgn * (0.5 * ws + 0.05), FLY_H],
                         "p1": [Xw + 0.4, ys + sgn * (0.5 * ws - 0.6), FLY_H], "d": d0})
        A = float(r.choice([-1.0, 1.0])) * float(r.uniform(1.0, 1.8))
        align = {"kind": "ramp", "A": (0.15 * float(r.normal()), A), "x_start": Xw - 15.0, "x_full": Xw - 4.0}
        loc_zone = (Xw - 12.0, Xw + 6.0, 0.30)
        obs_zone = (Xw - 12.0, Xw + 6.0, 0.60, 0.25)
        info.update({"gap_y": ys, "gap_w": ws, "align_A": A})
    else:
        raise ValueError(fam)
    align["sigma0"] = 0.1
    align["c0"] = (float(r.normal(0.0, 0.1)), float(r.normal(0.0, 0.1)))
    if fam == "S0":
        align["c0"] = (0.0, 0.0)

    wall_xy = np.array(walls) if walls else np.zeros((0, 2))

    def cluttered(x, y, pad):
        if len(wall_xy) and np.min(np.hypot(wall_xy[:, 0] - x, wall_xy[:, 1] - y)) < pad + 0.5 * WALL_D:
            return True
        if any(xa <= x <= xb and ya <= y <= yb for xa, xb, ya, yb in clear_boxes):
            return True
        return any(abs(x - gx) < 4.0 and abs(y - gy) < hw for gx, gy, hw in gap_lanes)

    keep_ids = set()
    stems = []
    for s in base["stems"]:
        if cluttered(s["x"], s["y"], 1.0 + 0.5 * s["dbh_m"]):
            continue
        keep_ids.add(s["id"])
        stems.append(dict(s))
    n_base = len(stems)
    nid = max([s["id"] for s in base["stems"]] + [0]) + 1
    wall_ids = []
    for (x, y) in walls:
        stems.append({"id": nid, "x": x, "y": y, "dbh_m": WALL_D, "height_m": 8.0, "wall": True})
        wall_ids.append(nid)
        nid += 1
    branches = []
    for b in base["branches"]:
        if b["stem"] not in keep_ids:
            continue
        p0, p1 = b["p0"], b["p1"]
        bad = False
        for t in np.linspace(0.0, 1.0, 6):
            q = (p0[0] + t * (p1[0] - p0[0]), p0[1] + t * (p1[1] - p0[1]))
            if cluttered(q[0], q[1], 0.3):
                bad = True
                break
        if not bad:
            branches.append(dict(b))
    n_base_br = len(branches)
    for k, bb in enumerate(block_br):
        branches.append({"id": 100000 + k, "stem": wall_ids[0] if wall_ids else -1, "p0": bb["p0"], "p1": bb["p1"],
                         "d0_m": bb["d"], "d1_m": 0.85 * bb["d"], "scenario_block": True})
    targets = [t for t in base["targets"] if not cluttered(t["x"], t["y"], 0.8)]
    truth = dict(base)
    truth.update({"stems": stems, "branches": branches, "targets": targets})
    truth["family"] = dict(base["family"], scenario=fam)

    # ---- prior map content (stale): truth stems + phantom stems, no branches
    phantoms = []
    n_ph = max(1, int(round(0.05 * n_base)))
    tries = 0
    while len(phantoms) < n_ph and tries < 500:
        tries += 1
        x, y = float(r.uniform(4.0, length_m + 4.0)), float(r.uniform(-half_width_m, half_width_m))
        if cluttered(x, y, 1.5) or math.hypot(x, y) < 4.0 or abs(x - length_m) < 4.0:
            continue
        phantoms.append({"x": x, "y": y, "dbh_m": float(np.clip(r.lognormal(math.log(0.24), 0.4), 0.1, 0.6))})
    prior_stems = [{"x": s["x"], "y": s["y"], "dbh_m": s["dbh_m"]} for s in stems] + phantoms
    if fam == "S0":   # unchanged map: no phantoms; branches rasterised as their slab circles
        prior_stems = [{"x": s["x"], "y": s["y"], "dbh_m": s["dbh_m"]} for s in stems]
        for b in branches:
            p0, p1 = np.array(b["p0"]), np.array(b["p1"])
            n = max(2, int(math.ceil(np.linalg.norm(p1 - p0) / 0.1)) + 1)
            for u in np.linspace(0.0, 1.0, n):
                q = p0 + u * (p1 - p0)
                if abs(q[2] - FLY_H) < 0.35:
                    prior_stems.append({"x": float(q[0]), "y": float(q[1]),
                                        "dbh_m": float(b["d0_m"] + u * (b["d1_m"] - b["d0_m"]))})

    # ---- detection-value hints: noisy target positions + 2 decoys (a prior on where people are likely)
    hints = [(t["x"] + float(r.normal(0, 2.0)), t["y"] + float(r.normal(0, 2.0))) for t in targets]
    hints += [(float(r.uniform(8, length_m - 4)), float(r.uniform(-half_width_m, half_width_m))) for _ in range(2)]

    info.update({"n_base_stems": n_base, "n_base_branches": n_base_br, "n_phantoms": len(phantoms),
                 "n_targets": len(targets)})
    return {"seed": int(seed), "family": fam, "truth": truth, "prior_stems": prior_stems, "align": align,
            "loc_zone": loc_zone, "obs_zone": obs_zone, "hints": hints, "info": info}


def prior_grid(prior_stems, m, l_occ=2.0, l_free=-1.0):
    """Prior log-odds grid on the same geometry as map m, its observed mask and a distance field (m, capped 0.6)."""
    L = np.full((m.nx, m.ny), l_free, np.float32)
    occ = np.zeros((m.nx, m.ny), bool)
    res = m.res
    for s in prior_stems:
        rr = 0.5 * s["dbh_m"]
        k = int(math.ceil(rr / res)) + 1
        ci, cj = int((s["x"] - m.x0) / res), int((s["y"] - m.y0) / res)
        for a in range(-k, k + 1):
            for b in range(-k, k + 1):
                i, j = ci + a, cj + b
                if 0 <= i < m.nx and 0 <= j < m.ny:
                    cx, cy = m.x0 + (i + 0.5) * res, m.y0 + (j + 0.5) * res
                    if math.hypot(cx - s["x"], cy - s["y"]) <= rr + 0.5 * res:
                        occ[i, j] = True
    L[occ] = l_occ
    obs = np.ones((m.nx, m.ny), bool)
    dist = np.full((m.nx, m.ny), 0.6, np.float32)
    dist[occ] = 0.0
    cur = occ.copy()
    for k in range(1, 6):
        P = np.pad(cur, 1)
        nxt = cur | P[2:, 1:-1] | P[:-2, 1:-1] | P[1:-1, 2:] | P[1:-1, :-2]
        dist[nxt & ~cur] = k * res
        cur = nxt
    return L, obs, dist


def align_offset(spec, x_prog):
    """Alignment error delta (m, map vs planner frame) as a function of truth progress along x."""
    c0 = spec["c0"]
    if spec["kind"] == "const":
        return c0
    f = min(max((x_prog - spec["x_start"]) / max(spec["x_full"] - spec["x_start"], 1e-6), 0.0), 1.0)
    return (c0[0] + f * spec["A"][0], c0[1] + f * spec["A"][1])


def hint_value(hints, x, y, sigma=3.0):
    if not hints:
        return 0.0
    h = np.asarray(hints)
    return float(np.max(np.exp(-((h[:, 0] - x) ** 2 + (h[:, 1] - y) ** 2) / (2 * sigma * sigma))))
