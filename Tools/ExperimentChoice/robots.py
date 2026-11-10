"""Robot profiles for the experiment-choice track (EXPLORATORY).

* ``compact_diff`` and ``narrow_tandem`` are transcribed from the modular study's
  ``morphologies.yaml`` (read-only source; git sha and file sha256 recorded below).
  The yaml lists footprint half-extents and (linear, angular, acceleration) limits.
* ``mecanum`` is read AT RUNTIME from the fleet session's ``fleet_profile_v0.json``
  (read-only; every value there is labelled provisional). Its sha256 goes into every
  manifest. The file has no acceleration; the value used here is an explicit assumption.

No profile has an angular acceleration; we use omega_max / (v_max / acc), i.e. the same
ramp time as the linear axis (assumption, stated in every provenance block).
"""
from __future__ import annotations

import hashlib
import json
import os
from dataclasses import dataclass, field, asdict

MORPHOLOGIES_PATH = "/home/lunarz/mrmn-main/src/modular_robot_description/config/morphologies.yaml"
MORPHOLOGIES_REPO_HEAD = "00ac590e077ad92ddf2d2b0ada2967d18c95d7cd"      # mrmn-main HEAD when transcribed
MORPHOLOGIES_FILE_COMMIT = "02151691a0535e8fecc92e2c3b0cb263359f5ee8"    # last commit touching the file
MORPHOLOGIES_SHA256 = "c0d2931da13ba6124e901e2890aee302d46b6a8de0f8f33863f61347e14ff696"
FLEET_PROFILE_PATH = "/home/lunarz/crane_ml/Tools/Fleet/spec/fleet_profile_v0.json"
FLEET_PROFILE_ENV = "EXPCHOICE_FLEET_PROFILE"
MECANUM_ASSUMED_ACC = 1.0        # m/s^2, NOT in the fleet profile (assumption)

# Lidar defaults: range and points per scan as in the fleet profile's provisional lidar_2d
# (12 m, 720 points, 10 Hz); range noise sigma is an assumption.
LIDAR_DEFAULT = {"max_range": 12.0, "n_beams": 720, "rate_hz": 10.0, "noise_sigma": 0.01}


@dataclass(frozen=True)
class Robot:
    name: str
    drive: str                  # "differential" or "mecanum"
    L: float                    # footprint length along heading (m)
    W: float                    # footprint width (m)
    v_max: float                # m/s
    omega_max: float            # rad/s
    acc: float                  # m/s^2
    vy_max: float = 0.0         # lateral speed, mecanum only
    axle_offset: float = 0.0    # rotation centre behind geometric centre (m)
    lidar: dict = field(default_factory=lambda: dict(LIDAR_DEFAULT))
    provenance: dict = field(default_factory=dict)

    @property
    def rho(self) -> float:
        return 0.5 * (self.L ** 2 + self.W ** 2) ** 0.5

    @property
    def ang_acc(self) -> float:
        return self.omega_max * self.acc / self.v_max

    def to_dict(self) -> dict:
        return asdict(self)


def _sha256(path: str) -> str:
    with open(path, "rb") as f:
        return hashlib.sha256(f.read()).hexdigest()


def _modular(name, hx, hy, v, om, acc) -> Robot:
    prov = {
        "source": MORPHOLOGIES_PATH,
        "repo_head": MORPHOLOGIES_REPO_HEAD,
        "file_last_commit": MORPHOLOGIES_FILE_COMMIT,
        "file_sha256_when_transcribed": MORPHOLOGIES_SHA256,
        "fields": "morphologies.%s.footprint (half-extents %.2f x %.2f) and limits" % (name, hx, hy),
        "assumptions": "angular acceleration = omega_max*acc/v_max; lidar from fleet provisional lidar_2d, sigma assumed",
    }
    return Robot(name=name, drive="differential", L=2 * hx, W=2 * hy, v_max=v, omega_max=om, acc=acc,
                 provenance=prov)


def fleet_profile_path() -> str:
    return os.environ.get(FLEET_PROFILE_ENV, FLEET_PROFILE_PATH)


def load_mecanum(path: str | None = None) -> Robot:
    """Mecanum profile from the fleet contract file (read-only, provisional values)."""
    path = path or fleet_profile_path()
    with open(path, "r", encoding="utf-8") as f:
        prof = json.load(f)
    ch = prof["chassis"]
    L = float(ch["body_size_lwh"]["value"][0])
    W = float(max(ch["body_size_lwh"]["value"][1], ch["overall_width_with_wheels"]["value"]))
    v = float(ch["max_speed_xy"]["value"])
    om = float(ch["max_yaw_rate"]["value"])
    lid = prof["sensors"]["lidar_2d"]
    lidar = {"max_range": float(lid["max_range"]["value"]), "n_beams": int(lid["points_per_scan"]["value"]),
             "rate_hz": float(lid["rate_hz"]["value"]), "noise_sigma": LIDAR_DEFAULT["noise_sigma"]}
    prov = {
        "source": path,
        "file_sha256": _sha256(path),
        "profile_id": prof.get("profile_id"),
        "schema": prof.get("schema"),
        "value_status": "provisional (every value in the fleet profile is a pre-CAD placeholder)",
        "fields": "chassis.body_size_lwh[0] -> L; max(body width, overall_width_with_wheels) -> W; "
                  "max_speed_xy -> v_max and vy_max; max_yaw_rate -> omega_max; sensors.lidar_2d",
        "assumptions": "acc = %.2f m/s^2 (not in file); angular acceleration = omega_max*acc/v_max; "
                       "lidar sigma assumed" % MECANUM_ASSUMED_ACC,
    }
    return Robot(name="mecanum", drive="mecanum", L=L, W=W, v_max=v, vy_max=v, omega_max=om,
                 acc=MECANUM_ASSUMED_ACC, lidar=lidar, provenance=prov)


def load_robot(name: str) -> Robot:
    if name == "compact_diff":
        return _modular("compact_diff", 0.40, 0.36, 0.9, 1.6, 1.2)
    if name == "narrow_tandem":
        return _modular("narrow_tandem", 0.88, 0.33, 0.65, 0.75, 0.55)
    if name == "mecanum":
        return load_mecanum()
    raise KeyError("unknown robot profile %r (compact_diff, narrow_tandem, mecanum)" % name)


def source_hashes() -> dict:
    """Hashes recorded in every manifest. Missing files are reported, not hidden."""
    out = {"morphologies_repo_head": MORPHOLOGIES_REPO_HEAD,
           "morphologies_file_last_commit": MORPHOLOGIES_FILE_COMMIT,
           "morphologies_sha256_when_transcribed": MORPHOLOGIES_SHA256}
    p = fleet_profile_path()
    out["fleet_profile_path"] = p
    out["fleet_profile_sha256"] = _sha256(p) if os.path.exists(p) else "MISSING"
    out["fleet_profile_value_status"] = "provisional"
    return out
