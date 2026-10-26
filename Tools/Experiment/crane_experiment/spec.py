"""crane.study/v1 spec: validation, canonical hash, parameter space."""
import copy
import math

import numpy as np

from .canon import obj_sha
from .errors import SpecError
from .splits import validate_split

SCHEMA = "crane.study/v1"
CAL_ORDER = ["FIXTURE_ONLY", "UNCALIBRATED_PRIOR", "SIM_TRUTH_RECOVERY", "PHYSICALLY_CALIBRATED"]


def _check_profile(p):
    if not isinstance(p, str) or p.count("@") != 1 or not all(p.split("@")):
        raise SpecError(f"profile must be an opaque 'id@sha' string, got {p!r}")


def validate_spec(spec):
    if not isinstance(spec, dict) or spec.get("schema") != SCHEMA:
        raise SpecError(f"schema must be {SCHEMA}")
    for key in ("study_id", "seed", "space", "default", "contexts", "fidelities",
                "default_fidelity", "budget"):
        if key not in spec:
            raise SpecError(f"missing field {key}")
    space = spec["space"]
    if not space:
        raise SpecError("empty space")
    for n, b in space.items():
        if not (b["low"] < b["high"]):
            raise SpecError(f"space {n}: low must be < high")
        d = spec["default"].get(n)
        if d is None or not (b["low"] <= d <= b["high"]):
            raise SpecError(f"default for {n} missing or out of bounds")
    if spec["default_fidelity"] not in spec["fidelities"]:
        raise SpecError("default_fidelity not in fidelities")
    for n, f in spec["fidelities"].items():
        if not (f["cost"] > 0 and math.isfinite(f["cost"])):
            raise SpecError(f"fidelity {n}: cost must be > 0")
    for c in spec["contexts"]:
        for key in ("id", "site", "session", "split", "profile"):
            if key not in c:
                raise SpecError(f"context missing {key}")
        if c["split"] not in ("train", "heldout"):
            raise SpecError("context split must be train|heldout")
        _check_profile(c["profile"])
    train = [c for c in spec["contexts"] if c["split"] == "train"]
    held = [c for c in spec["contexts"] if c["split"] == "heldout"]
    validate_split(train, held)
    if any(k in spec.get("objective", {}) for k in ("invalid_reward", "violation_penalty")):
        raise SpecError("failures are feasibility signals, never objective values: remove invalid_reward/violation_penalty")
    b = spec["budget"]
    if not b["total_cost"] > 0 or b.get("reserve_cost", 0) < 0 or b.get("reserve_cost", 0) >= b["total_cost"]:
        raise SpecError("budget.total_cost must be > reserve_cost >= 0")
    if spec.get("baseline_replicates", 2) > spec.get("crn_replicates", 2):
        raise SpecError("baseline_replicates must be <= crn_replicates")
    for p, meta in spec.get("profiles", {}).items():
        _check_profile(p)
        if meta.get("calibrationState") not in CAL_ORDER:
            raise SpecError(f"profile {p}: unknown calibrationState")
    return spec


def spec_sha(spec):
    return obj_sha(spec)


def with_defaults(spec):
    s = copy.deepcopy(spec)
    s.setdefault("crn_replicates", 2)
    s.setdefault("baseline_replicates", 2)
    s.setdefault("max_obs_age_s", 1.0)
    s.setdefault("min_valid_fraction", 0.5)
    s.setdefault("alpha", 0.2)
    s.setdefault("profiles", {})
    s.setdefault("claim_scope", {"requiredCalibrationState": "FIXTURE_ONLY"})
    return s


class Space:
    def __init__(self, space):
        self.names = list(space)
        self.low = np.array([space[n]["low"] for n in self.names], float)
        self.high = np.array([space[n]["high"] for n in self.names], float)
        self.dim = len(self.names)

    def to_unit(self, config):
        x = np.array([config[n] for n in self.names], float)
        return (x - self.low) / (self.high - self.low)

    def from_unit(self, x):
        x = np.clip(np.asarray(x, float), 0.0, 1.0)
        v = self.low + x * (self.high - self.low)
        return {n: float(a) for n, a in zip(self.names, v)}
