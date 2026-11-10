"""Offset-doorway course generator, truth manifest and exporters (EXPLORATORY).

Layout (frame in geometry.py): approach bay 0 <= x <= d, |y| <= B/2; door wall d <= x <= d + t
with an opening of width w centred at y = e; exit room d + t <= x <= d + t + EXIT_DEPTH of the
same width. The robot starts in the bay heading +x (toward the door wall) and must reach a
goal beyond the door. All lengths are integer centimetres internally.

Exporters are pure functions: ``to_boxes`` (numpy surrogate), ``to_occupancy_grid`` /
``pgm_bytes`` / ``map_yaml`` (Nav2 map, 0.025 m cells) and ``to_sdf`` (Gazebo world string).
Nothing here runs Gazebo; the SDF and map have NOT been loaded in any simulator.
"""
from __future__ import annotations

import hashlib
import json
import os
from dataclasses import dataclass

import numpy as np

try:
    from . import mechanics as M, robots as R
except ImportError:
    import sys
    sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
    import mechanics as M
    import robots as R

SCHEMA = "expchoice.course/v0"
LABEL = "exploratory"
WALL = 0.20            # thickness of the outer walls (m)
EXIT_DEPTH = 2.50      # depth of the room beyond the door (m)
GOAL_BEYOND = 1.20     # default goal distance beyond the far face of the door wall (m)
GOAL_TOLERANCE = 0.25  # goal radius for the rotation centre (m)
WALL_HEIGHT = 1.0      # SDF only
GRID_RES = 0.025       # occupancy grid cell (m)

# Frozen admissible box (integer centimetres, inclusive). The sampler draws uniformly from it;
# an optimiser may choose where to sample but never changes this measure.
ADMISSIBLE_BOX_V0 = {
    "name": "offset_doorway_box_v0",
    "w_cm": [80, 120],
    "e_cm": [0, 150],
    "d_cm": [84, 160],
    "t_cm": [5, 40],
    "B_cm": [500, 500],
    "side_clearance_cm": 60,       # required: |e| + w/2 <= B/2 - side_clearance
}


def _cm(v) -> int:
    return int(round(float(v) * 100))


def box_hash(box=None) -> str:
    return hashlib.sha256(json.dumps(box or ADMISSIBLE_BOX_V0, sort_keys=True, separators=(",", ":")).encode()).hexdigest()


@dataclass(frozen=True)
class Course:
    w_cm: int
    e_cm: int
    d_cm: int
    B_cm: int
    t_cm: int
    start_cm: tuple          # (x_cm, y_cm, heading in millirad)
    goal_cm: tuple           # (x_cm, y_cm)

    # metres
    @property
    def w(self): return self.w_cm / 100.0
    @property
    def e(self): return self.e_cm / 100.0
    @property
    def d(self): return self.d_cm / 100.0
    @property
    def B(self): return self.B_cm / 100.0
    @property
    def t(self): return self.t_cm / 100.0
    @property
    def start(self): return (self.start_cm[0] / 100.0, self.start_cm[1] / 100.0, self.start_cm[2] / 1000.0)
    @property
    def goal(self): return (self.goal_cm[0] / 100.0, self.goal_cm[1] / 100.0)

    def params(self) -> dict:
        """Canonical integer parameters (the hashed content)."""
        return {"schema": SCHEMA, "family": "offset_doorway", "w_cm": self.w_cm, "e_cm": self.e_cm,
                "d_cm": self.d_cm, "B_cm": self.B_cm, "t_cm": self.t_cm,
                "start_cm_mrad": list(self.start_cm), "goal_cm": list(self.goal_cm),
                "wall_cm": _cm(WALL), "exit_depth_cm": _cm(EXIT_DEPTH)}

    @property
    def course_id(self) -> str:
        canon = json.dumps(self.params(), sort_keys=True, separators=(",", ":"))
        return hashlib.sha256(canon.encode()).hexdigest()[:16]

    def walls(self) -> list:
        """Named axis-aligned boxes (metres)."""
        d, t, B, w, e = self.d, self.t, self.B, self.w, self.e
        x_end = d + t + EXIT_DEPTH
        out = [
            ("back_wall", -WALL, 0.0, -B / 2 - WALL, B / 2 + WALL),
            ("side_wall_neg", -WALL, x_end + WALL, -B / 2 - WALL, -B / 2),
            ("side_wall_pos", -WALL, x_end + WALL, B / 2, B / 2 + WALL),
            ("end_wall", x_end, x_end + WALL, -B / 2 - WALL, B / 2 + WALL),
        ]
        if e - w / 2 > -B / 2:
            out.append(("door_wall_neg", d, d + t, -B / 2, e - w / 2))
        if e + w / 2 < B / 2:
            out.append(("door_wall_pos", d, d + t, e + w / 2, B / 2))
        return [{"name": n, "xmin": round(a, 4), "xmax": round(b, 4), "ymin": round(c, 4), "ymax": round(f, 4)}
                for n, a, b, c, f in out]

    def boxes(self) -> np.ndarray:
        return to_boxes(self)


def make_course(w, e, d, B=5.0, t=0.20, start=None, goal=None) -> Course:
    """Course from metres (rounded to centimetres). Default start: bay centre, heading 0.
    Default goal: on the door axis, GOAL_BEYOND past the far face of the door wall."""
    w_cm, e_cm, d_cm, B_cm, t_cm = _cm(w), _cm(e), _cm(d), _cm(B), _cm(t)
    if min(w_cm, d_cm, B_cm, t_cm) <= 0:
        raise ValueError("w, d, B, t must be positive")
    if abs(e_cm) + w_cm / 2 > B_cm / 2:
        raise ValueError("door opening must lie inside the bay width")
    if start is None:
        start_cm = (d_cm // 2, 0, 0)                 # odd d: half a centimetre behind the bay centre
    else:
        start_cm = (_cm(start[0]), _cm(start[1]), int(round(float(start[2]) * 1000)))
    goal_cm = (d_cm + t_cm + _cm(GOAL_BEYOND), e_cm) if goal is None else (_cm(goal[0]), _cm(goal[1]))
    return Course(w_cm, e_cm, d_cm, B_cm, t_cm, tuple(start_cm), tuple(goal_cm))


def course_from_ratios(robot, eps_over_W, e_over_W, w_over_W=1.3, t=0.20, B=5.0) -> Course:
    """Course for a robot from dimensionless ratios (then rounded to cm; read back the real ratios)."""
    return make_course(w=w_over_W * robot.W, e=e_over_W * robot.W, d=robot.L + eps_over_W * robot.W, B=B, t=t)


def sample_courses(n, seed, box=None) -> list:
    """n courses drawn uniformly (in integer cm) from the frozen box; deterministic in (seed, box)."""
    box = box or ADMISSIBLE_BOX_V0
    rng = np.random.Generator(np.random.PCG64(int(seed)))
    out = []
    while len(out) < n:
        v = {k: int(rng.integers(box[k][0], box[k][1] + 1)) for k in ("w_cm", "e_cm", "d_cm", "t_cm", "B_cm")}
        if v["e_cm"] + v["w_cm"] / 2 > v["B_cm"] / 2 - box["side_clearance_cm"]:
            continue
        out.append(make_course(v["w_cm"] / 100, v["e_cm"] / 100, v["d_cm"] / 100, v["B_cm"] / 100, v["t_cm"] / 100))
    return out


# --------------------------------------------------------------------------- exporters

def to_boxes(course) -> np.ndarray:
    """(M, 4) array of (xmin, xmax, ymin, ymax) for the numpy surrogate."""
    return np.array([[b["xmin"], b["xmax"], b["ymin"], b["ymax"]] for b in course.walls()], float)


def to_occupancy_grid(course, resolution=GRID_RES, pad=0.25):
    """Occupancy grid: (occ, origin). occ[iy, ix] True where the cell overlaps a wall (conservative);
    cell (ix, iy) covers origin + [ix, ix+1] x [iy, iy+1] * resolution. origin = (x0, y0) of cell (0, 0)."""
    bx = to_boxes(course)
    x0, x1 = bx[:, 0].min() - pad, bx[:, 1].max() + pad
    y0, y1 = bx[:, 2].min() - pad, bx[:, 3].max() + pad
    nx, ny = int(np.ceil((x1 - x0) / resolution - 1e-9)), int(np.ceil((y1 - y0) / resolution - 1e-9))
    occ = np.zeros((ny, nx), bool)
    for xmin, xmax, ymin, ymax in bx:
        ix0 = int(np.floor((xmin - x0) / resolution + 1e-9))
        ix1 = int(np.ceil((xmax - x0) / resolution - 1e-9))
        iy0 = int(np.floor((ymin - y0) / resolution + 1e-9))
        iy1 = int(np.ceil((ymax - y0) / resolution - 1e-9))
        occ[max(iy0, 0):iy1, max(ix0, 0):ix1] = True
    return occ, (round(float(x0), 6), round(float(y0), 6))


def pgm_bytes(occ) -> bytes:
    """Binary PGM (P5) in map_server convention: row 0 is the top (max y); 0 occupied, 254 free."""
    img = np.where(occ[::-1], 0, 254).astype(np.uint8)
    return b"P5\n%d %d\n255\n" % (img.shape[1], img.shape[0]) + img.tobytes()


def map_yaml(image_name, origin, resolution=GRID_RES) -> str:
    return ("image: %s\nmode: trinary\nresolution: %.3f\norigin: [%.3f, %.3f, 0.0]\n"
            "negate: 0\noccupied_thresh: 0.65\nfree_thresh: 0.25\n" % (image_name, resolution, origin[0], origin[1]))


def to_sdf(course, world_name=None) -> str:
    """Minimal SDF 1.8 world: ground plane, sun and one static box model per wall. Not run anywhere."""
    name = world_name or "offset_doorway_%s" % course.course_id
    parts = ['<?xml version="1.0"?>', '<sdf version="1.8">', '  <world name="%s">' % name,
             '    <!-- EXPLORATORY: generated by Tools/ExperimentChoice/course.py; course_id %s -->' % course.course_id,
             '    <light type="directional" name="sun"><pose>0 0 10 0 0 0</pose><direction>-0.5 0.1 -0.9</direction></light>',
             '    <model name="ground_plane"><static>true</static><link name="link">',
             '      <collision name="collision"><geometry><plane><normal>0 0 1</normal><size>100 100</size></plane></geometry></collision>',
             '      <visual name="visual"><geometry><plane><normal>0 0 1</normal><size>100 100</size></plane></geometry></visual>',
             '    </link></model>']
    for b in course.walls():
        sx, sy = b["xmax"] - b["xmin"], b["ymax"] - b["ymin"]
        cx, cy = (b["xmax"] + b["xmin"]) / 2, (b["ymax"] + b["ymin"]) / 2
        geom = "<geometry><box><size>%.4f %.4f %.4f</size></box></geometry>" % (sx, sy, WALL_HEIGHT)
        parts += ['    <model name="%s"><static>true</static><pose>%.4f %.4f %.4f 0 0 0</pose><link name="link">'
                  % (b["name"], cx, cy, WALL_HEIGHT / 2),
                  '      <collision name="collision">%s</collision>' % geom,
                  '      <visual name="visual">%s</visual>' % geom,
                  '    </link></model>']
    parts += ['  </world>', '</sdf>', '']
    return "\n".join(parts)


def manifest(course, robot) -> dict:
    """Truth manifest (JSON-serialisable): the single description every tier derives from."""
    return {
        "schema": SCHEMA,
        "label": LABEL,
        "course_id": course.course_id,
        "family": "offset_doorway",
        "parameters": course.params(),
        "parameters_m": {"w": course.w, "e": course.e, "d": course.d, "B": course.B, "t": course.t},
        "frame": "x along bay depth (back wall inner face x=0, door wall x in [d, d+t]); y lateral; heading from +x",
        "walls": course.walls(),
        "start": {"x": course.start[0], "y": course.start[1], "theta": course.start[2]},
        "goal": {"x": course.goal[0], "y": course.goal[1], "tolerance": GOAL_TOLERANCE},
        "robot": {k: v for k, v in robot.to_dict().items() if k != "provenance"},
        "robot_provenance": robot.provenance,
        "certificate": M.certificate(course, robot),
        "certificates_by_drive": M.certificates_all_drives(course, robot),
        "admissible_box": {"name": ADMISSIBLE_BOX_V0["name"], "sha256": box_hash()},
        "sources": R.source_hashes(),
    }


def manifest_json(course, robot) -> str:
    return json.dumps(manifest(course, robot), sort_keys=True, indent=1)


def write_course(out_dir, course, robot) -> dict:
    """Write manifest, map (PGM + YAML) and SDF for one course. Returns the paths."""
    os.makedirs(out_dir, exist_ok=True)
    stem = "course_%s" % course.course_id
    occ, origin = to_occupancy_grid(course)
    paths = {k: os.path.join(out_dir, stem + ext) for k, ext in
             (("manifest", ".json"), ("pgm", ".pgm"), ("yaml", ".yaml"), ("sdf", ".sdf"))}
    with open(paths["manifest"], "w", encoding="utf-8") as f:
        f.write(manifest_json(course, robot))
    with open(paths["pgm"], "wb") as f:
        f.write(pgm_bytes(occ))
    with open(paths["yaml"], "w", encoding="utf-8") as f:
        f.write(map_yaml(stem + ".pgm", origin))
    with open(paths["sdf"], "w", encoding="utf-8") as f:
        f.write(to_sdf(course))
    return paths
