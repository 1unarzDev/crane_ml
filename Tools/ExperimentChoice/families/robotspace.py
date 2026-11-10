"""Robot-configuration space (EXPLORATORY).

A ``RobotConfig`` is a tuple of integers (millimetres, milliseconds, ...) so that its id is a
stable hash. It covers: footprint (a rectangle or a composite of unit modules), rotation centre,
drive type, speed and acceleration limits, controller latency, and the 2-D lidar configuration.

MODULE COMBINATIONS. The modular robot (mrmn-main, ``morphologies.yaml``, read-only) rearranges one
set of modules into bodies with different footprints. Here that idea is reduced to a declared rule:

* a unit module is a rectangle ``unit_l x unit_w``; an arrangement is a set of grid cells
  (``ARRANGEMENTS``): modules in a line, side by side, 2x2, 2x3, or an L of 3 or 4;
* the footprint is the union of the cells (a rectangle, or two rectangles for an L);
* the rotation centre is the area centroid of the cells, shifted along the body axis by
  ``rc_shift_mm`` (positive = forward);
* limits follow power laws in the half diagonal rho of the bounding box, fitted through the two
  simulated morphologies of the yaml (compact_diff: 0.80 x 0.72, 0.9 m/s, 1.6 rad/s, 1.2 m/s^2;
  narrow_tandem: 1.76 x 0.66, 0.65, 0.75, 0.55), then multiplied by a sampled scale.
  The 2x2 arrangement of 0.40 x 0.36 units reproduces compact_diff exactly.

The limits rule is an interpolation device through two points, not a physical model, and the yaml's
hand-written footprints are NOT in general unions of its inventory modules.

Drive types: ``differential`` (pivot, no lateral motion), ``mecanum`` (holonomic), ``carlike``
(reserved: representable, never drawn by the v0 sampler, no surrogate support).
"""
from __future__ import annotations

import hashlib
import json
import math
from dataclasses import dataclass, asdict

import numpy as np

from . import _PARENT  # noqa: F401
import robots as R

SCHEMA = "expchoice.robot_config/v0"
DRIVES = ("differential", "mecanum", "carlike")

# cells (i along the body axis, j lateral)
ARRANGEMENTS = {
    "rect": [(0, 0)],
    "line1": [(0, 0)],
    "line2": [(0, 0), (1, 0)],
    "line3": [(0, 0), (1, 0), (2, 0)],
    "line4": [(0, 0), (1, 0), (2, 0), (3, 0)],
    "row2": [(0, 0), (0, 1)],
    "2x2": [(0, 0), (1, 0), (0, 1), (1, 1)],
    "2x3": [(0, 0), (1, 0), (2, 0), (0, 1), (1, 1), (2, 1)],
    "L3": [(0, 0), (1, 0), (0, 1)],
    "L4": [(0, 0), (1, 0), (2, 0), (0, 1)],
}

# the two simulated morphologies (transcribed in robots.py from the yaml)
_C = dict(rho=0.5 * math.hypot(0.80, 0.72), v=0.9, om=1.6, acc=1.2)
_T = dict(rho=0.5 * math.hypot(1.76, 0.66), v=0.65, om=0.75, acc=0.55)
LIMIT_EXPONENTS = {k: math.log(_T[k] / _C[k]) / math.log(_T["rho"] / _C["rho"]) for k in ("v", "om", "acc")}
LIMIT_CLIP = {"v": (0.3, 1.5), "om": (0.3, 2.5), "acc": (0.3, 2.0)}

# Frozen sampling box. Integer ranges are inclusive; lists are categorical.
ROBOT_BOX_V0 = {
    "name": "robot_box_v0",
    "drive": ["differential", "mecanum"],
    "drive_reserved": ["carlike"],
    "arrangement": ["line1", "line2", "line3", "line4", "row2", "2x2", "2x3", "L3", "L4"],
    "unit_l_mm": [300, 500],
    "unit_w_mm": [280, 400],
    "rc_shift_pct_of_L": [-20, 20],          # differential only; 0 for mecanum
    "limit_scale_pct": [70, 130],
    "cmd_delay_ms": [0, 300],
    "obs_delay_ms": [0, 300],
    "lidar_range_mm": [4000, 8000, 12000, 25000],
    "lidar_fov_deg": [120, 180, 270, 360],
    "lidar_res_mdeg": [250, 500, 1000],
    "lidar_mount_pct_of_L": [-30, 45],       # mount position along the body axis from the bbox centre
    "lidar_noise_um": [5000, 10000, 30000],
    "lidar_rate_mhz": [10000],
}


def box_hash(box=None) -> str:
    return hashlib.sha256(json.dumps(box or ROBOT_BOX_V0, sort_keys=True, separators=(",", ":")).encode()).hexdigest()


def rule_limits(rho, scale=1.0):
    """(v_max, omega_max, acc) from the declared power-law rule."""
    out = []
    for k in ("v", "om", "acc"):
        val = _C[k] * (rho / _C["rho"]) ** LIMIT_EXPONENTS[k] * scale
        out.append(min(max(val, LIMIT_CLIP[k][0]), LIMIT_CLIP[k][1]))
    return tuple(out)


@dataclass(frozen=True)
class RobotConfig:
    name: str
    drive: str
    arrangement: str
    unit_l_mm: int
    unit_w_mm: int
    rc_shift_mm: int            # rotation centre ahead (+) of the area centroid, along the body axis
    v_max_mmps: int
    vy_max_mmps: int            # 0 unless holonomic
    omega_max_mradps: int
    acc_mmps2: int
    cmd_delay_ms: int = 0       # command-to-motion delay
    obs_delay_ms: int = 0       # age of the observation the controller acts on
    lidar_range_mm: int = 12000
    lidar_fov_deg: int = 360
    lidar_n_beams: int = 720
    lidar_mount_x_mm: int = 0   # relative to the rotation centre, body frame
    lidar_mount_y_mm: int = 0
    lidar_noise_um: int = 10000
    lidar_rate_mhz: int = 10000
    min_turn_radius_mm: int = 0  # carlike only

    # ---- identity
    def canonical(self) -> dict:
        d = asdict(self)
        d.pop("name")
        d["schema"] = SCHEMA
        return d

    @property
    def config_id(self) -> str:
        return hashlib.sha256(json.dumps(self.canonical(), sort_keys=True, separators=(",", ":")).encode()).hexdigest()[:16]

    # ---- footprint
    def _cells(self):
        return ARRANGEMENTS[self.arrangement]

    def _rects_bbox_frame(self):
        """Rectangles (cx, cy, lx, ly) in metres, origin at the bounding-box centre."""
        ul, uw = self.unit_l_mm / 1000.0, self.unit_w_mm / 1000.0
        cells = self._cells()
        ni, nj = 1 + max(c[0] for c in cells), 1 + max(c[1] for c in cells)
        rows = {}
        for j in range(nj):
            run = sorted(i for i, jj in cells if jj == j)
            rows.setdefault((run[0], run[-1]), []).append(j)
        rects = []
        for (i0, i1), js in rows.items():
            lx, ly = (i1 - i0 + 1) * ul, len(js) * uw
            cx = (i0 + i1 + 1) / 2 * ul - ni * ul / 2
            cy = (min(js) + max(js) + 1) / 2 * uw - nj * uw / 2
            rects.append((cx, cy, lx, ly))
        return rects

    @property
    def L(self) -> float:
        return (1 + max(c[0] for c in self._cells())) * self.unit_l_mm / 1000.0

    @property
    def W(self) -> float:
        return (1 + max(c[1] for c in self._cells())) * self.unit_w_mm / 1000.0

    @property
    def area(self) -> float:
        return len(self._cells()) * self.unit_l_mm * self.unit_w_mm / 1e6

    def _centroid(self):
        rects = self._rects_bbox_frame()
        A = sum(r[2] * r[3] for r in rects)
        return (sum(r[0] * r[2] * r[3] for r in rects) / A, sum(r[1] * r[2] * r[3] for r in rects) / A)

    @property
    def rotation_centre(self):
        """Rotation centre in the bounding-box frame (metres)."""
        cx, cy = self._centroid()
        return (cx + self.rc_shift_mm / 1000.0, cy)

    @property
    def axle_offset(self) -> float:
        """geometry.py convention: rotation centre BEHIND the bounding-box centre by this much."""
        return -self.rotation_centre[0]

    @property
    def rc_lateral(self) -> float:
        return self.rotation_centre[1]

    def parts(self):
        """Rectangles (cx, cy, lx, ly) in the body frame whose origin is the rotation centre."""
        rx, ry = self.rotation_centre
        return [(cx - rx, cy - ry, lx, ly) for cx, cy, lx, ly in self._rects_bbox_frame()]

    @property
    def is_rect(self) -> bool:
        return len(self._rects_bbox_frame()) == 1

    def polygon(self):
        """Outline of the footprint in the body frame, counter-clockwise, collinear points removed."""
        ul, uw = self.unit_l_mm / 1000.0, self.unit_w_mm / 1000.0
        cells = set(self._cells())
        ni, nj = 1 + max(c[0] for c in cells), 1 + max(c[1] for c in cells)
        edges = {}
        for i, j in cells:
            for a, b, nb in (((i, j), (i + 1, j), (i, j - 1)), ((i + 1, j), (i + 1, j + 1), (i + 1, j)),
                             ((i + 1, j + 1), (i, j + 1), (i, j + 1)), ((i, j + 1), (i, j), (i - 1, j))):
                if nb not in cells:
                    edges[a] = b
        start = min(edges)
        loop, cur = [start], edges[start]
        while cur != start:
            loop.append(cur)
            cur = edges[cur]
        out = []
        for k, p in enumerate(loop):
            a, b = loop[k - 1], loop[(k + 1) % len(loop)]
            if (p[0] - a[0]) * (b[1] - p[1]) - (p[1] - a[1]) * (b[0] - p[0]) != 0:
                out.append(p)
        rx, ry = self.rotation_centre
        return [(i * ul - ni * ul / 2 - rx, j * uw - nj * uw / 2 - ry) for i, j in out]

    @property
    def rho(self) -> float:
        return 0.5 * math.hypot(self.L, self.W)

    @property
    def r_circ(self) -> float:
        """Radius of the disc swept by an in-place rotation (about the rotation centre)."""
        return max(math.hypot(abs(p[0]) + p[2] / 2, abs(p[1]) + p[3] / 2) for p in self.parts())

    # ---- limits, latency, sensor (SI)
    @property
    def v_max(self): return self.v_max_mmps / 1000.0
    @property
    def vy_max(self): return self.vy_max_mmps / 1000.0
    @property
    def omega_max(self): return self.omega_max_mradps / 1000.0
    @property
    def acc(self): return self.acc_mmps2 / 1000.0
    @property
    def ang_acc(self): return self.omega_max * self.acc / self.v_max        # same assumption as robots.py
    @property
    def cmd_delay(self): return self.cmd_delay_ms / 1000.0
    @property
    def obs_delay(self): return self.obs_delay_ms / 1000.0
    @property
    def latency(self): return self.cmd_delay + self.obs_delay
    @property
    def stopping_distance(self):
        """Distance covered from the instant an obstacle appears in the world until standstill."""
        return self.v_max * self.latency + self.v_max ** 2 / (2 * self.acc)

    @property
    def sensor(self) -> dict:
        fov = math.radians(self.lidar_fov_deg)
        return {"max_range": self.lidar_range_mm / 1000.0, "fov": fov, "n_beams": self.lidar_n_beams,
                "angular_resolution": fov / self.lidar_n_beams,
                "mount": (self.lidar_mount_x_mm / 1000.0, self.lidar_mount_y_mm / 1000.0),
                "noise_sigma": self.lidar_noise_um / 1e6, "rate_hz": self.lidar_rate_mhz / 1000.0}

    @property
    def lidar(self) -> dict:
        """The dict robots.Robot carries (full-circle beam count equivalent)."""
        s = self.sensor
        return {"max_range": s["max_range"], "n_beams": int(round(2 * math.pi / s["angular_resolution"])),
                "rate_hz": s["rate_hz"], "noise_sigma": s["noise_sigma"]}

    # ---- bridges
    def to_robot(self) -> "R.Robot":
        """robots.Robot for the existing surrogate (bounding box; see ``surrogate_gaps``)."""
        if self.drive not in ("differential", "mecanum"):
            raise ValueError("the surrogate has no %s drive" % self.drive)
        return R.Robot(name=self.name, drive=self.drive, L=self.L, W=self.W, v_max=self.v_max,
                       omega_max=self.omega_max, acc=self.acc, vy_max=self.vy_max,
                       axle_offset=self.axle_offset, lidar=self.lidar,
                       provenance={"config_id": self.config_id, "lossy": self.surrogate_gaps()})

    def surrogate_gaps(self) -> list:
        """What the existing surrogate cannot represent of this configuration."""
        g = []
        if not self.is_rect:
            g.append("composite footprint replaced by its bounding box")
        if abs(self.axle_offset) > 1e-9:
            g.append("rotation-centre offset (surrogate controllers refuse it)")
        if abs(self.rc_lateral) > 1e-9:
            g.append("lateral rotation-centre offset")
        if self.cmd_delay_ms or self.obs_delay_ms:
            g.append("command and observation delay")
        if self.lidar_fov_deg != 360:
            g.append("limited lidar field of view")
        if self.lidar_mount_x_mm or self.lidar_mount_y_mm:
            g.append("lidar mounting offset")
        if self.drive == "carlike":
            g.append("car-like drive")
        return g

    def to_dict(self) -> dict:
        d = asdict(self)
        d.update(config_id=self.config_id, L=self.L, W=self.W, rotation_centre_bbox_frame=list(self.rotation_centre),
                 parts=[list(p) for p in self.parts()], polygon=[list(p) for p in self.polygon()],
                 n_modules=len(self._cells()), limits_rule="power law in rho through compact_diff and narrow_tandem")
        return d


def make_config(arrangement, unit_l, unit_w, drive="differential", rc_shift=0.0, limit_scale=1.0, name=None,
                v_max=None, omega_max=None, acc=None, vy_max=None, **kw) -> RobotConfig:
    """Config from metres and seconds. Limits default to the declared rule."""
    if arrangement not in ARRANGEMENTS or drive not in DRIVES:
        raise KeyError("unknown arrangement or drive")
    ul, uw = int(round(unit_l * 1000)), int(round(unit_w * 1000))
    cells = ARRANGEMENTS[arrangement]
    L = (1 + max(c[0] for c in cells)) * ul / 1000.0
    W = (1 + max(c[1] for c in cells)) * uw / 1000.0
    v, om, a = rule_limits(0.5 * math.hypot(L, W), limit_scale)
    v, om, a = v_max or v, omega_max or om, acc or a
    if vy_max is None:
        vy_max = v if drive == "mecanum" else 0.0
    cfg = RobotConfig(name=name or "", drive=drive, arrangement=arrangement, unit_l_mm=ul, unit_w_mm=uw,
                      rc_shift_mm=int(round(rc_shift * 1000)), v_max_mmps=int(round(v * 1000)),
                      vy_max_mmps=int(round(vy_max * 1000)), omega_max_mradps=int(round(om * 1000)),
                      acc_mmps2=int(round(a * 1000)), **kw)
    if not cfg.name:
        cfg = RobotConfig(**{**asdict(cfg), "name": "%s_%s_%s" % (drive[:4], arrangement, cfg.config_id[:6])})
    return cfg


def from_profile(robot) -> RobotConfig:
    """RobotConfig for a robots.Robot profile (single rectangle, no latency, full-circle lidar)."""
    lid = robot.lidar
    return RobotConfig(name=robot.name, drive=robot.drive, arrangement="rect",
                       unit_l_mm=int(round(robot.L * 1000)), unit_w_mm=int(round(robot.W * 1000)),
                       rc_shift_mm=-int(round(robot.axle_offset * 1000)),
                       v_max_mmps=int(round(robot.v_max * 1000)), vy_max_mmps=int(round(robot.vy_max * 1000)),
                       omega_max_mradps=int(round(robot.omega_max * 1000)), acc_mmps2=int(round(robot.acc * 1000)),
                       lidar_range_mm=int(round(lid["max_range"] * 1000)), lidar_n_beams=int(lid["n_beams"]),
                       lidar_noise_um=int(round(lid["noise_sigma"] * 1e6)), lidar_rate_mhz=int(round(lid["rate_hz"] * 1000)))


def as_config(robot) -> RobotConfig:
    return robot if isinstance(robot, RobotConfig) else from_profile(robot)


def sample_robots(n, seed, box=None) -> list:
    """n configurations drawn from the frozen box; deterministic in (seed, box)."""
    box = box or ROBOT_BOX_V0
    rng = np.random.Generator(np.random.PCG64(int(seed)))

    def integer(key):
        return int(rng.integers(box[key][0], box[key][1] + 1))

    def choice(key):
        return box[key][int(rng.integers(0, len(box[key])))]

    out = []
    for _ in range(n):
        drive, arr = choice("drive"), choice("arrangement")
        ul, uw = integer("unit_l_mm"), integer("unit_w_mm")
        shift_pct, scale = integer("rc_shift_pct_of_L"), integer("limit_scale_pct") / 100.0
        cmd, obs = integer("cmd_delay_ms"), integer("obs_delay_ms")
        rng_mm, fov, res = choice("lidar_range_mm"), choice("lidar_fov_deg"), choice("lidar_res_mdeg")
        mount_pct, noise, rate = integer("lidar_mount_pct_of_L"), choice("lidar_noise_um"), choice("lidar_rate_mhz")
        base = make_config(arr, ul / 1000.0, uw / 1000.0, drive=drive, limit_scale=scale, name="tmp")
        L_mm = int(round(base.L * 1000))
        rc_shift = 0 if drive == "mecanum" else int(round(shift_pct / 100.0 * L_mm))
        # mount is sampled relative to the bounding-box centre, stored relative to the rotation centre
        cx, cy = base._centroid()
        mount_x = int(round(mount_pct / 100.0 * L_mm - cx * 1000)) - rc_shift
        mount_y = -int(round(cy * 1000))
        d = asdict(base)
        d.update(rc_shift_mm=rc_shift, cmd_delay_ms=cmd, obs_delay_ms=obs, lidar_range_mm=rng_mm, lidar_fov_deg=fov,
                 lidar_n_beams=int(round(fov * 1000 / res)), lidar_mount_x_mm=mount_x, lidar_mount_y_mm=mount_y,
                 lidar_noise_um=noise, lidar_rate_mhz=rate)
        cfg = RobotConfig(**d)
        out.append(RobotConfig(**{**d, "name": "%s_%s_%s" % (drive[:4], arr, cfg.config_id[:6])}))
    return out


def reference_configs() -> dict:
    """The three existing profiles as configurations (mecanum only if the fleet file is present)."""
    out = {}
    for name in ("compact_diff", "narrow_tandem", "mecanum"):
        try:
            out[name] = from_profile(R.load_robot(name))
        except (OSError, KeyError):
            pass
    return out
