"""Family-agnostic robot-environment interaction features (EXPLORATORY).

``interaction_features(course, robot)`` reads only the boxes, the start pose, the goal point and
the robot configuration. It never looks at the family name or at a family parameter. All
features are dimensionless. Definitions, units and verification status: ``FEATURE_TABLE``.

Construction (one pass, about a second):

1. 2-D clearance field on a grid (exact distance at each cell centre).
2. PASSAGE: the widest route start -> goal (largest disc that can travel), its width, and the
   route point where that width is attained.
3. TRAVEL ROUTE: shortest 8-connected route of a disc of diameter W (the body width), pulled
   taut, resampled. If no such disc route exists the widest route is used instead.
4. HEADINGS: at every route point the exact set of admissible headings of the full footprint
   (composite shapes included). This is the coarse configuration-space grid: route x heading.
   The body heading at a point is the admissible heading nearest to the direction of travel.
5. CRAB SEGMENTS: maximal pieces of the route on which the body cannot be aligned with the
   direction of travel. For each: the net displacement perpendicular to the confined heading
   (the sideways shift that has to be produced) and the depth of the free slot along that heading
   (the strip depth). ``mechanics.min_stretches_exact`` turns the two into a stretch count.
6. TURNS: at every route vertex, the best clearance available at each intermediate heading.
7. SENSING: line of sight, range and field of view from the sensor, at the start and along the route.

``raster(course, robot)`` gives the raw input for learned representations.
"""
from __future__ import annotations

import math

import numpy as np

from . import _PARENT  # noqa: F401
from . import cspace as CS
from .robotspace import as_config
import geometry as G
import mechanics as M

CAP = 1000.0            # ratios are clipped here (a zero clearance must not produce inf)
STRETCH_CAP = 1.0e6
ALIGN_TOL = math.radians(3.0)
TURN_MIN = math.radians(20.0)

FEATURE_TABLE = [
    # name, definition, units, status
    ("aspect", "L / W of the bounding box", "-", "exact"),
    ("fill", "footprint area / bounding-box area (1 for a rectangle)", "-", "exact"),
    ("rc_offset_over_L", "distance of the rotation centre from the bounding-box centre / L", "-", "exact"),
    ("pivot_disc_over_2rho", "in-place pivot disc diameter 2 r_circ / bounding-box diagonal", "-", "exact"),
    ("holonomic", "1 if the drive can translate sideways", "0/1", "exact"),
    ("route_exists", "1 if start and goal are connected for a point", "0/1", "grid (2.5 cm)"),
    ("disc_route_exists", "1 if a disc of diameter W can travel start -> goal", "0/1", "grid; conservative by 0.75 cell"),
    ("passage_over_W", "width of the narrowest place on the widest route / W", "-",
     "grid, snapped to the exact gap between two boxes when exactly one lies within the grid error; tested on doorways"),
    ("passage_over_L", "same width / L", "-", "as passage_over_W"),
    ("route_len_over_rho", "length of the travel route / rho", "-", "grid; taut 8-connected route, not the true geodesic"),
    ("turn_total", "sum of |direction change| along the travel route, including the first turn from the start heading / pi", "-", "grid"),
    ("clear_min_over_W", "smallest footprint-to-wall distance over the route, body at its route heading / W", "-",
     "exact distance at sampled poses; route and heading choice are heuristic"),
    ("clear_p10_over_W", "10th percentile of the same", "-", "as above"),
    ("clear_med_over_W", "median of the same", "-", "as above"),
    ("blocked_fraction", "fraction of route points where no heading of the footprint is free", "-", "exact per point (1.5 deg)"),
    ("align_fraction", "fraction of route points where the body can point along the direction of travel", "-", "exact per point"),
    ("reverse_fraction", "fraction of route points where the body can follow the route only tail first (heading continuity kept)", "-",
     "exact per point; tested on the dead-end bay against Bw >= 2 rho"),
    ("reverse_len_over_L", "length of that part of the route / L", "-", "as above"),
    ("rot_range_start", "in-place rotation range at the start pose / 2 pi", "-", "exact (bisection), tested against alpha_max"),
    ("rot_range_min", "smallest in-place rotation range around the route heading along the route / 2 pi", "-", "exact per point (1.5 deg)"),
    ("rot_range_max", "largest of the same", "-", "exact per point (1.5 deg)"),
    ("crab_shift_over_W", "sum over crab segments of the net shift perpendicular to the confined heading / W", "-",
     "heuristic segmentation; equals (e - door slack) / W on the offset doorway (tested)"),
    ("slot_depth_over_L", "smallest free slot depth along the confined heading over crab segments / L (CAP if none)", "-",
     "exact ray casts on a heuristic segment; equals d / L on the offset doorway (tested)"),
    ("slot_depth_over_2rho", "same depth / 2 rho; below 1 means heading confinement", "-", "as above"),
    ("strip_blocked", "1 if a crab segment has slot depth < L (the strip model has no solution)", "0/1", "heuristic"),
    ("shimmy_stretches_log", "log(1 + stretches a pivot drive needs by the strip formula, summed over crab segments)", "-",
     "strip formula exact for a plain strip; first-order elsewhere; tested against mechanics on the offset doorway"),
    ("predicted_stretches_log", "same for this robot's drive (1 stretch if holonomic)", "-", "as above"),
    ("corner_clear_over_W", "min over route turns and intermediate headings of the best footprint clearance near the vertex / W", "-",
     "grid search (5 cm, 9 headings); sign tested against the exact L-bend statement"),
    ("stop_over_clear_min", "stopping distance (v latency + v^2 / 2a) / smallest route clearance", "-", "definition; clearance as above"),
    ("stop_over_clear_med", "stopping distance / median route clearance", "-", "definition"),
    ("latency_travel_over_clear_min", "v_max (command delay + observation delay) / smallest route clearance", "-", "definition"),
    ("latency_turn_over_rot_slack", "omega_max latency / half the smallest rotation range", "-", "definition"),
    ("stop_over_passage_slack", "stopping distance / ((passage - W) / 2)", "-", "definition"),
    ("route_visible_fraction", "fraction of route points in line of sight, range and field of view of the sensor at the start", "-",
     "exact rays to sampled points"),
    ("passage_visible_at_start", "1 if the passage point is seen from the start", "0/1", "exact ray; tested on the occluded doorway"),
    ("passage_first_obs_arc", "fraction of the route travelled before the passage point is first seen", "-", "exact rays at route points"),
    ("passage_first_obs_range_over_stop", "range to the passage point when first seen / stopping distance", "-", "as above"),
    ("passage_first_obs_bearing", "|bearing| of the passage point in the body frame when first seen / pi", "-", "as above"),
    ("beams_across_passage", "passage width / (range when first seen x angular resolution), log(1 + .)", "-", "first-order (small angle)"),
    ("noise_over_passage_slack", "range noise sigma / ((passage - W) / 2)", "-", "definition"),
    ("range_over_route", "lidar range / route length", "-", "definition"),
]
FEATURE_NAMES = [r[0] for r in FEATURE_TABLE]


def feature_table_markdown() -> str:
    rows = ["| Feature | Definition | Units | Verification status |", "|---|---|---|---|"]
    rows += ["| `%s` | %s | %s | %s |" % r for r in FEATURE_TABLE]
    return "\n".join(rows)


# --------------------------------------------------------------------------- pieces

def _ratio(a, b):
    if b <= 1e-9:
        return CAP if a > 0 else 0.0
    return float(min(a / b, CAP))


def rotation_range(x, y, th, parts, boxes, n=360):
    """In-place rotation range (rad, up to 2 pi) around heading th at (x, y): exact up to bisection."""
    if CS.hits(x, y, th, parts, boxes)[0]:
        return 0.0, 0.0
    out = []
    for sign in (1.0, -1.0):
        grid = th + sign * np.linspace(0.0, 2 * math.pi, n + 1)
        bad = np.flatnonzero(CS.hits(x, y, grid, parts, boxes))
        if bad.size == 0:
            return math.pi, math.pi
        lo, hi = grid[bad[0] - 1], grid[bad[0]]
        for _ in range(30):
            mid = 0.5 * (lo + hi)
            if CS.hits(x, y, mid, parts, boxes)[0]:
                hi = mid
            else:
                lo = mid
        out.append(abs(lo - th))
    return out[0], out[1]


def _visible(sensor_xy, heading, target, boxes, sensor):
    """Line of sight, range and field of view from a sensor pose to target points (n, 2)."""
    target = np.asarray(target, float).reshape(-1, 2)
    dx, dy = target[:, 0] - sensor_xy[0], target[:, 1] - sensor_xy[1]
    dist = np.hypot(dx, dy)
    ang = np.arctan2(dy, dx)
    rng = np.array([G.raycast(np.array([sensor_xy]), np.array([a]), boxes, d + 1.0)[0, 0] for a, d in zip(ang, dist)])
    bearing = G.wrap(ang - heading)
    ok = (rng >= dist - 1e-6) & (dist <= sensor["max_range"]) & (np.abs(bearing) <= sensor["fov"] / 2 + 1e-9)
    return ok, dist, bearing


def _sensor_xy(x, y, th, sensor):
    mx, my = sensor["mount"]
    return (x + math.cos(th) * mx - math.sin(th) * my, y + math.sin(th) * mx + math.cos(th) * my)


def _slot_depth(px, py, th, width, boxes, reach=30.0):
    """Free length through (px, py) along +-th of a slot of the given width (three rays each way)."""
    nx, ny = -math.sin(th), math.cos(th)
    org = np.array([[px + k * nx, py + k * ny] for k in (-0.45 * width, 0.0, 0.45 * width)])
    r = G.raycast(org, np.array([th, th + math.pi]), boxes, reach)
    return float(r[:, 0].min() + r[:, 1].min())


def _circ_dist(a, b):
    return np.abs(G.wrap(a - b))


def _run_around(adm_row, k):
    """Indices (lo, hi] offsets of the circular run of True around index k: (count_minus, count_plus)."""
    n = adm_row.size
    if adm_row.all():
        return n, n
    up = 0
    while adm_row[(k + up + 1) % n]:
        up += 1
    dn = 0
    while adm_row[(k - dn - 1) % n]:
        dn += 1
    return dn, up


def _labels(row):
    """Circular run labels of a bool array (equal labels = same run of True)."""
    lab = np.cumsum(~row)
    if row[0] and row[-1]:
        lab = np.where(lab == lab[-1], lab[0], lab)
    return lab


def _empty(rc, flags):
    f = {k: 0.0 for k in FEATURE_NAMES}
    f.update(flags)
    return f


def _robot_only(rc):
    L, W = rc.L, rc.W
    rx, ry = rc.rotation_centre
    return {"aspect": L / W, "fill": rc.area / (L * W), "rc_offset_over_L": math.hypot(rx, ry) / L,
            "pivot_disc_over_2rho": rc.r_circ / rc.rho, "holonomic": 1.0 if rc.drive == "mecanum" else 0.0}


# --------------------------------------------------------------------------- main

def interaction_features(course, robot, res=0.025, n_head=240, max_pts=120, return_meta=False):
    """Dimensionless interaction features of one (course, robot) pair. See the module docstring."""
    rc = as_config(robot)
    parts, L, W, rho = rc.parts(), rc.L, rc.W, rc.rho
    boxes = course.boxes()
    sx, sy, sth = course.start
    gx, gy = course.goal
    sensor = rc.sensor
    feats = dict(_robot_only(rc))
    meta = {"course_id": course.course_id, "config_id": rc.config_id, "res": res}

    # 1. clearance field on a grid through the start point
    b = (boxes[:, 0].min(), boxes[:, 1].max(), boxes[:, 2].min(), boxes[:, 3].max())
    xs = sx + res * np.arange(math.ceil((b[0] - sx) / res), math.floor((b[1] - sx) / res) + 1)
    ys = sy + res * np.arange(math.ceil((b[2] - sy) / res), math.floor((b[3] - sy) / res) + 1)
    clear = CS.clearance_field(boxes, xs, ys)

    def cell(x, y):
        return (int(round((y - ys[0]) / res)), int(round((x - xs[0]) / res)))

    s_idx, g_idx = cell(sx, sy), cell(gx, gy)

    def connected(thr):
        m = clear >= thr
        return bool(m[s_idx] and m[g_idx] and CS.flood(m, s_idx)[g_idx])

    if not connected(res / 4):
        out = _empty(rc, feats)
        return (out, meta) if return_meta else out

    # 2. passage: binary search for the largest disc radius that connects start and goal
    lo, hi = res / 4, float(min(clear[s_idx], clear[g_idx]))
    if connected(hi):
        lo = hi
    else:
        while hi - lo > res / 8:
            mid = 0.5 * (lo + hi)
            if connected(mid):
                lo = mid
            else:
                hi = mid
    passage = 2 * lo
    # exact value: the bottleneck is a gap between two boxes; take it if exactly one candidate is near
    d2 = np.hypot(np.maximum(np.maximum(boxes[:, None, 0] - boxes[None, :, 1], boxes[None, :, 0] - boxes[:, None, 1]), 0),
                  np.maximum(np.maximum(boxes[:, None, 2] - boxes[None, :, 3], boxes[None, :, 2] - boxes[:, None, 3]), 0))
    cand = np.unique(np.round(d2[(d2 > passage - 1e-6) & (d2 < passage + 1.6 * res)], 6))
    meta["passage_grid"] = passage
    meta["passage_exact"] = cand.size == 1
    if cand.size == 1:
        passage = float(cand[0])
    wide_mask = clear >= lo - 1e-9
    wide = CS.grid_route(wide_mask, s_idx, g_idx)
    cw = clear[wide[:, 0], wide[:, 1]]
    kq = int(np.flatnonzero(cw <= cw.min() + res / 4)[-1])
    q = (float(xs[wide[kq, 1]]), float(ys[wide[kq, 0]]))
    meta["passage_point"] = q

    # 3. travel route of a disc of diameter W
    r_route = W / 2 + 0.75 * res
    mask = clear >= r_route
    disc_ok = bool(mask[s_idx] and mask[g_idx] and CS.flood(mask, s_idx)[g_idx])
    if disc_ok:
        cells = CS.grid_route(mask, s_idx, g_idx)
    else:
        mask, cells = wide_mask, wide
    taut = CS.pull_string(mask, cells)
    vx, vy = xs[taut[:, 1]], ys[taut[:, 0]]
    seg = np.hypot(np.diff(vx), np.diff(vy))
    length = float(seg.sum())
    if length < 1e-9:                                  # start is at the goal
        out = _empty(rc, feats)
        out.update(route_exists=1.0, disc_route_exists=float(disc_ok), passage_over_W=_ratio(passage, W),
                   passage_over_L=_ratio(passage, L))
        return (out, meta) if return_meta else out
    n_pts = int(min(max_pts, max(8, math.ceil(length / 0.05)))) + 1
    arc = np.linspace(0.0, length, n_pts)
    cum = np.concatenate([[0.0], np.cumsum(seg)])
    px, py = np.interp(arc, cum, vx), np.interp(arc, cum, vy)
    k_seg = np.clip(np.searchsorted(cum, arc, side="right") - 1, 0, seg.size - 1)
    tau = np.arctan2(np.diff(vy), np.diff(vx))[k_seg]                    # direction of travel at each point
    ds = length / (n_pts - 1)

    # 4. admissible headings at every route point (the route x heading grid)
    ths = sth + 2 * math.pi * np.arange(n_head) / n_head
    dth = 2 * math.pi / n_head
    adm = ~CS.hits(np.repeat(px, n_head), np.repeat(py, n_head), np.tile(ths, n_pts), parts, boxes).reshape(n_pts, n_head)
    # The body heading must vary continuously along the route: ``R`` is the set of headings reachable
    # at this point from the previous one (runs of admissible headings that overlap the previous set).
    body = np.full(n_pts, sth)             # body heading along the route
    mis = np.full(n_pts, math.pi)          # its misalignment with the direction of travel (mod pi)
    rot = np.zeros(n_pts)                  # in-place rotation range around the body heading
    kbody = np.zeros(n_pts, int)
    cont = np.zeros(n_pts, bool)           # heading continuity holds at this point
    mis_fwd = np.full(n_pts, math.pi)      # misalignment if the body has to point along the travel direction
    R_prev = np.zeros(n_head, bool)
    R_prev[0] = True
    for i in range(n_pts):
        row = adm[i]
        seeds = row & (R_prev | np.roll(R_prev, 1) | np.roll(R_prev, -1))
        if not seeds.any():                            # the route cannot be followed here with a continuous heading
            body[i], kbody[i] = (body[i - 1], kbody[i - 1]) if i else (sth, 0)
            continue
        lab = _labels(row)
        R = row & np.isin(lab, np.unique(lab[seeds]))
        m = np.where(R, np.minimum(_circ_dist(ths, tau[i]), _circ_dist(ths, tau[i] + math.pi)), np.inf)
        best = np.flatnonzero(m <= m.min() + 1e-9)
        if best.size > 1:
            best = best[[int(np.argmin(_circ_dist(ths[best], body[i - 1] if i else sth)))]]
        k = int(best[0])
        kbody[i], body[i], mis[i], cont[i] = k, ths[k], m[k], True
        mis_fwd[i] = float(np.where(R, _circ_dist(ths, tau[i]), np.inf).min())
        dn, up = _run_around(row, k)
        rot[i] = min((dn + up) * dth, 2 * math.pi)
        R_prev = R
    blocked = ~adm.any(axis=1)
    aligned = cont & (mis <= ALIGN_TOL)
    reverse_only = aligned & (mis_fwd > ALIGN_TOL)
    r0 = rotation_range(sx, sy, sth, parts, boxes)
    rot_start = min(r0[0] + r0[1], 2 * math.pi)

    # clearance profile, body at its route heading; points where continuity failed are left out
    cl = CS.clearance(px[cont], py[cont], body[cont], parts, boxes) if cont.any() else np.zeros(1)

    # 5. crab segments
    crab_shift, slot_min, stretches, strip_blocked = 0.0, math.inf, 1.0, 0.0
    segments = []
    i = 0
    while i < n_pts:
        if aligned[i]:
            i += 1
            continue
        j = i
        while j + 1 < n_pts and not aligned[j + 1]:
            j += 1
        # confined heading: centre of the admissible run at the entry (or the last known body heading)
        if cont[i]:
            dn, up = _run_around(adm[i], kbody[i])
            th_c = body[i] if dn + up >= n_head else ths[kbody[i]] + 0.5 * (up - dn) * dth
        else:
            th_c = body[i]
        end = min(j + 1, n_pts - 1)
        shift = abs(-(px[end] - px[i]) * math.sin(th_c) + (py[end] - py[i]) * math.cos(th_c))
        depth = min(_slot_depth(px[k], py[k], th_c, W, boxes) for k in range(i, j + 1))
        n_st = 1.0
        if shift > 0.01:
            n_st = M.min_stretches_exact(shift, depth, L, W, "centre")
            if n_st == M.INF:
                n_st, strip_blocked = STRETCH_CAP, 1.0
            crab_shift += shift
            slot_min = min(slot_min, depth)
        stretches += max(n_st - 1.0, 0.0)
        segments.append({"from": i, "to": j, "heading": th_c, "shift": shift, "slot_depth": depth, "stretches": n_st})
        i = j + 1
    stretches = min(stretches, STRETCH_CAP)

    # 6. turns: best clearance at each intermediate heading near every route vertex
    turn_dirs = np.arctan2(np.diff(vy), np.diff(vx))
    first = G.wrap(turn_dirs[0] - sth)
    if abs(first) > math.pi / 2:                       # the body may equally well back into the route
        first = G.wrap(first - math.pi)
    turns = [(vx[0], vy[0], sth, first)]
    for k in range(1, turn_dirs.size):
        turns.append((vx[k], vy[k], turn_dirs[k - 1], G.wrap(turn_dirs[k] - turn_dirs[k - 1])))
    turn_total = sum(abs(t[3]) for t in turns)
    corner = math.inf
    half = rho                                         # placements within rho of the vertex and in its line of sight
    off = np.arange(-half, half + 1e-9, 0.05)
    OX, OY = np.meshgrid(off, off)
    OX, OY = OX.ravel(), OY.ravel()
    keep = np.hypot(OX, OY) <= half + 1e-9
    OX, OY = OX[keep], OY[keep]
    for tx, ty, t0, dturn in turns:
        if abs(dturn) < TURN_MIN:
            continue
        los = G.raycast(np.array([[tx, ty]]), np.arctan2(OY, OX), boxes, 2 * half)[0] >= np.hypot(OX, OY) - 1e-9
        worst = math.inf
        for th in t0 + np.linspace(0.0, dturn, 9):
            worst = min(worst, float(CS.clearance(tx + OX[los], ty + OY[los], th, parts, boxes).max()))
        corner = min(corner, worst)

    # 7. sensing
    s_xy = _sensor_xy(sx, sy, sth, sensor)
    route_pts = np.stack([px, py], 1)
    vis_route, _, _ = _visible(s_xy, sth, route_pts, boxes, sensor)
    first_arc, first_rng, first_bear, seen0 = 1.0, 0.0, 1.0, 0.0
    for i in range(n_pts):
        th_i = body[i]
        ok, dist, bearing = _visible(_sensor_xy(px[i], py[i], th_i, sensor), th_i, [q], boxes, sensor)
        if ok[0]:
            first_arc, first_rng, first_bear = arc[i] / length, float(dist[0]), abs(float(bearing[0])) / math.pi
            seen0 = 1.0 if i == 0 else 0.0
            break

    stop = rc.stopping_distance
    cmin, cmed = float(cl.min()), float(np.median(cl))
    slack = max(passage - W, 0.0) / 2
    feats.update({
        "route_exists": 1.0, "disc_route_exists": float(disc_ok),
        "passage_over_W": _ratio(passage, W), "passage_over_L": _ratio(passage, L),
        "route_len_over_rho": _ratio(length, rho), "turn_total": turn_total / math.pi,
        "clear_min_over_W": cmin / W, "clear_p10_over_W": float(np.percentile(cl, 10)) / W, "clear_med_over_W": cmed / W,
        "blocked_fraction": float(blocked.mean()), "align_fraction": float(aligned.mean()),
        "reverse_fraction": float(reverse_only.mean()), "reverse_len_over_L": float(reverse_only.sum()) * ds / L,
        "rot_range_start": rot_start / (2 * math.pi),
        "rot_range_min": float(rot.min()) / (2 * math.pi), "rot_range_max": float(rot.max()) / (2 * math.pi),
        "crab_shift_over_W": crab_shift / W,
        "slot_depth_over_L": _ratio(slot_min, L) if slot_min < math.inf else CAP,
        "slot_depth_over_2rho": _ratio(slot_min, 2 * rho) if slot_min < math.inf else CAP,
        "strip_blocked": strip_blocked,
        "shimmy_stretches_log": math.log1p(stretches),
        "predicted_stretches_log": math.log1p(1.0 if rc.drive == "mecanum" else stretches),
        "corner_clear_over_W": min(corner / W, CAP) if corner < math.inf else CAP,
        "stop_over_clear_min": _ratio(stop, cmin), "stop_over_clear_med": _ratio(stop, cmed),
        "latency_travel_over_clear_min": _ratio(rc.v_max * rc.latency, cmin),
        "latency_turn_over_rot_slack": _ratio(rc.omega_max * rc.latency, float(rot.min()) / 2),
        "stop_over_passage_slack": _ratio(stop, slack),
        "route_visible_fraction": float(vis_route.mean()),
        "passage_visible_at_start": seen0, "passage_first_obs_arc": first_arc,
        "passage_first_obs_range_over_stop": _ratio(first_rng, stop), "passage_first_obs_bearing": first_bear,
        "beams_across_passage": math.log1p(_ratio(passage, max(first_rng, 1e-6) * sensor["angular_resolution"])),
        "noise_over_passage_slack": _ratio(sensor["noise_sigma"], slack),
        "range_over_route": _ratio(sensor["max_range"], length),
    })
    feats = {k: float(feats[k]) for k in FEATURE_NAMES}
    if return_meta:
        meta.update(passage=passage, route=route_pts, arc=arc, body_heading=body, aligned=aligned, clearance=cl,
                    rotation_range=rot, segments=segments, stretches=stretches, turns=turns, disc_route=disc_ok,
                    vertices=np.stack([vx, vy], 1))
        return feats, meta
    return feats


def feature_vector(course, robot, **kw):
    f = interaction_features(course, robot, **kw)
    return np.array([f[k] for k in FEATURE_NAMES], float)


# --------------------------------------------------------------------------- raw input

def raster(course, robot, pose=None, n=48, cell=0.10):
    """Raw local input in the robot frame, fixed size.

    Returns uint8 arrays [n, n] indexed [forward, left] with the rotation centre at the middle:
    ``occupancy`` (cell centre inside a wall) and ``footprint`` (cell centre inside the footprint),
    plus the goal in the body frame (metres). ``cell`` is metric; pass cell = k * rho for a
    robot-scaled window.
    """
    rc = as_config(robot)
    x, y, th = pose if pose is not None else course.start
    c = (np.arange(n) - (n - 1) / 2.0) * cell
    U, V = np.meshgrid(c, c, indexing="ij")
    WX, WY = x + math.cos(th) * U - math.sin(th) * V, y + math.sin(th) * U + math.cos(th) * V
    occ = np.zeros((n, n), bool)
    for xmin, xmax, ymin, ymax in course.boxes():
        occ |= (WX >= xmin) & (WX <= xmax) & (WY >= ymin) & (WY <= ymax)
    fp = np.zeros((n, n), bool)
    for cx, cy, lx, ly in rc.parts():
        fp |= (np.abs(U - cx) <= lx / 2) & (np.abs(V - cy) <= ly / 2)
    gx, gy = course.goal[0] - x, course.goal[1] - y
    return {"occupancy": occ.astype(np.uint8), "footprint": fp.astype(np.uint8), "cell": cell,
            "goal_body": (math.cos(th) * gx + math.sin(th) * gy, -math.sin(th) * gx + math.cos(th) * gy)}
