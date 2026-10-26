"""Data-quality gate for importing an mhseals_testing characterization (design section 4).

Every check returns ``ok`` or ``abstain(reason, **evidence)`` (estimators.abstain); an abstain carries
a severity: ``reject`` (the parameter stays at its prior) or ``warn`` (capped at provisional).
Signatures (attribution, section 4) are reported and never change a parameter by themselves.

Inputs (paths or already-parsed dicts):
  yaml        hardware YAML holding ``characterization`` (version 2: speed_curves, t95, wave_floor,
              motion_limits); a bare characterization block is accepted too
  provenance  calibration/current.json, schema ``mhseals.calibration/1``:
              directions.<dir>.{fit:{id,time,session,run,config}, hardware:{hash,epoch,verify},
              gate:{muted}, battery_v, conditions, dead}. The mhseals_testing sha is read from
              ``software.mhseals_testing`` (top level) or ``fit.mhseals_testing`` (per direction);
              today's writer stores neither, so a real record is PROVENANCE_MISSING until it does.
  run         the characterization run record (procedure, directions, holds, conditions,
              holds_rejected, ...). The gate needs per-hold samples, which today's record does not
              keep (procedure handoff section 9). Extension read here:
                holds[i].samples   {t: [s], vx: [m/s], vy: [m/s], wz: [rad/s], yaw?: [rad],
                                    t_sensor?: [s]}  body-frame odometry from settle start to hold
                                    end; t_sensor (header stamps) enables the clock-skew check
                holds[i].override  true when an operator override touched the hold
                settings           {transient_s, coast_s, settle_s} (characterize.yaml defaults)
                neutral            [{start, end, samples}] neutral windows (gyro bias, drift)
                coastdowns         [{axis, sign, start, end, samples}] unforced decays from speed
                lag_streams        {t_raw, raw, t_odom, odom, sigma} raw IMU/GPS vs odom (one channel)
                scale_anchor       true when a known-distance run anchors the velocity scale
  snapshot    the config-history hardware version at fit time (``mhseals.config_version/1``,
              value.effective), or a plain hardware dict.

The report maps each profile parameter (the six signed D curves and M_eff per axis) to
identified / provisional / prior with its reasons, and gives D = d*k_j, M_eff = mu*k_j (section 3).
PyYAML is used here and in import_yaml.py only.
"""
from __future__ import annotations

import hashlib
import json
import math
import os

import numpy as np

import estimators as E
import plant as P

SCHEMA = "crane.characterization_gate/1"
DIRECTIONS = ("surge+", "surge-", "sway+", "sway-", "yaw+", "yaw-")
AXES = ("surge", "sway", "yaw")
CHANNELS = ("vx", "vy", "wz")
WAVE_FLOOR_ORDER = ("yaw", "surge", "sway")  # characterization wave_floor is [yaw, surge, sway]

IDENTIFIED, PROVISIONAL, PRIOR = "identified", "provisional", "prior"
REJECT, WARN = "reject", "warn"

# Thresholds. Sources: design section 4; mhseals_testing config/conditions.yaml (verdict.good),
# config/characterize.yaml (transient/coast/settle, min_response, tolerance), calibration.py.
CEILING_GOOD, DRIFT_TRUST_GOOD = 0.90, 0.80
BATTERY_TOLERANCE_V = 0.8
MAX_GAP_S, MIN_SAMPLES, MAX_SKEW_S, MIN_COVERAGE = 0.25, 30, 0.006, 0.5
MIN_RESPONSE = 0.03
MISFIT_Z = 2.33
LB_LAGS = 5
WHITE_R1 = 0.2
ASYM_RANGE = (0.5, 2.0)
T95_RANGE = (0.5, 3.0)
LAG_FRACTION = 0.2
CURVE_MATCH = 0.05          # YAML curve must reproduce the holds' recomputed curve within 5 %
ASYM_SIGNATURE = 0.05       # d+ vs d- beyond 5 % is reported as THRUST_OR_HULL_ASYM
COAST_MISMATCH = 0.20       # coast-down vs curve deceleration beyond 20 % is PROP_DRAG_OR_K
DRIFT_PRACTICAL = 0.02      # m/s, conditions.yaml drift.practical
GYRO_MIN_S = 20.0           # conditions.yaml gyro_bias.min_s
TOLERANCE = {"surge": (0.05, 0.015), "sway": (0.05, 0.015), "yaw": (0.10, 0.10)}
DEFAULT_SETTINGS = {"transient_s": 1.0, "coast_s": 2.0, "settle_s": 6.0}
# Hardware default mixer (mhseals_hardware config/default.yaml), rows FL, FR, RR, RL.
DEFAULT_MATRIX = [[-1, 1, 1], [-1, -1, -1], [1, -1, 1], [1, 1, -1]]
MIXER_ROWS = ("front_left", "front_right", "rear_right", "rear_left")
DEP_INVALID = ("thruster_matrix", "reversed", "hardware_epoch")
DEP_VERIFY = ("channel_map", "pwm_channels")


def abstain(reason, severity=REJECT, **evidence):
    return E.abstain(reason, severity=severity, evidence=evidence)


def ok(**evidence):
    return E.ok(evidence=evidence)


# ------------------------------------------------------------------ input loading
def load_doc(x):
    """A dict as-is, a JSON/YAML path parsed, None as None."""
    if x is None or isinstance(x, dict):
        return x
    with open(x) as f:
        text = f.read()
    if str(x).endswith(".json"):
        return json.loads(text)
    import yaml
    return yaml.safe_load(text)


def sha256_file(path):
    with open(path, "rb") as f:
        return hashlib.sha256(f.read()).hexdigest()


def characterization_block(doc):
    if not doc:
        return {}
    if "characterization" in doc:
        return doc["characterization"] or {}
    return doc if "speed_curves" in doc else {}


def snapshot_hardware(snap):
    if not snap:
        return None
    value = snap.get("value", snap) if "value" in snap or "subsystem" in snap else snap
    if isinstance(value, dict) and "effective" in value:
        return value["effective"]
    return value


def _canonical(value):
    """mhseals_testing history._canonical: sorted keys, floats to 9 significant digits."""
    def clean(node):
        if isinstance(node, float):
            return float(f"{node:.9g}")
        if isinstance(node, dict):
            return {str(k): clean(v) for k, v in node.items()}
        if isinstance(node, (list, tuple)):
            return [clean(v) for v in node]
        return node
    return json.dumps(clean(value), sort_keys=True, separators=(",", ":"))


def dependency_hash(hardware, keys):
    """mhseals_testing calibration.dependency_hash (absent hardware_epoch hashes as 0)."""
    values = {k: hardware.get(k) for k in sorted(keys)}
    if "hardware_epoch" in values and values["hardware_epoch"] is None:
        values["hardware_epoch"] = 0
    return "sha256:" + hashlib.sha256(_canonical(values).encode()).hexdigest()[:12]


def parse_direction(name):
    axis, sign = name[:-1], (1 if name[-1] == "+" else -1)
    return AXES.index(axis), sign


def direction_of(command):
    """(direction name, amplitude) of a hold command: the axis with the largest |effort|."""
    c = [float(x) for x in command]
    j = int(np.argmax(np.abs(c)))
    if abs(c[j]) == 0.0:
        return None, 0.0
    return AXES[j] + ("+" if c[j] > 0 else "-"), abs(c[j])


# ------------------------------------------------------------------ mapping (design section 3)
def yaw_arm(t):
    a = math.radians(t["alphaDeg"])
    return abs(t["x"] * math.sin(a) - t["y"] * math.cos(a))


def axis_gains(profile, hardware=None):
    """k_j^+- = sum_i K_i^{sgn q_i} M_ij^2 |d_i . e_j| [N per unit e|e|], with the mixer row signs and
    the ``reversed`` flags of the fit-time hardware config. Also returns forward/reverse counts."""
    hw = hardware or {}
    matrix = hw.get("thruster_matrix") or DEFAULT_MATRIX
    reversed_ = hw.get("reversed") or [False] * len(matrix)
    by_name = {t["name"]: t for t in profile["thrusters"]}
    out = {}
    for name in DIRECTIONS:
        j, s = parse_direction(name)
        k, n_fwd, n_rev = 0.0, 0, 0
        for i, row in enumerate(matrix):
            mij = float(row[j])
            if mij == 0.0:
                continue
            t = by_name[MIXER_ROWS[i]]
            a = math.radians(t["alphaDeg"])
            proj = (abs(math.cos(a)), abs(math.sin(a)), yaw_arm(t))[j]
            q = s * mij * (-1.0 if reversed_[i] else 1.0)
            if q > 0:
                k += t["kForward"] * mij * mij * proj
                n_fwd += 1
            else:
                k += t["kReverse"] * mij * mij * proj
                n_rev += 1
        out[name] = {"k": k, "forward": n_fwd, "reverse": n_rev}
    return out


def curve_value(c, v):
    return c["d1"] * v + c["d2"] * v * abs(v)


def t95_closed(mu, e2, d1, d2):
    """Time from rest to 95 % of the steady speed of mu v' = e2 - d1 v - d2 v^2 (no lags)."""
    if d2 <= 0.0:
        return mu / d1 * math.log(20.0)
    vinf = P.steady_speed(d1, d2, e2)
    v2 = vinf + d1 / d2
    return mu / (d2 * (vinf + v2)) * math.log((0.95 * vinf + v2) / (0.05 * v2))


def t95_model(mu, e2, d1, d2, tau_t=0.0, t_d=0.0, tau_est=0.0):
    """t95 of the step ODE with thrust lag tau_t, dead time t_d and a first-order estimator lag."""
    if tau_t == 0.0 and t_d == 0.0 and tau_est == 0.0:
        return t95_closed(mu, e2, d1, d2)
    vinf = P.steady_speed(d1, d2, e2)
    tc = mu / (d1 + 2.0 * d2 * vinf)
    # fixed resolution over the response span: the lags use exact exponential updates and the
    # velocity a linearized-implicit one, so any step is stable
    h = (t_d + 3.0 * (tau_t + tau_est + tc)) / 4000.0
    T = v = y = 0.0
    t = 0.0
    a_t = 1.0 if tau_t == 0.0 else 1.0 - math.exp(-h / tau_t)
    a_e = 1.0 if tau_est == 0.0 else 1.0 - math.exp(-h / tau_est)
    t_end = t_d + 50.0 * (tc + tau_t + tau_est)
    while t < t_end:
        T += a_t * ((e2 if t >= t_d else 0.0) - T)
        v_new = v + h * (T - d1 * v - d2 * v * abs(v)) / (mu + h * (d1 + 2 * d2 * abs(v)))
        y_new = y + a_e * (v_new - y)
        if y_new >= 0.95 * vinf:
            return t + h * (0.95 * vinf - y) / (y_new - y)
        v, y, t = v_new, y_new, t + h
    return math.inf


def fit_mu(t95, e2, d1, d2, lags, mu_prior):
    """Solve t95_model(mu) = t95 for mu = M_eff/k (bisection; t95 is increasing in mu)."""
    tau_t, t_d, tau_est = lags
    if lags == (0.0, 0.0, 0.0):
        return t95 / t95_closed(1.0, e2, d1, d2)
    lo, hi = 1e-6 * mu_prior, 1e3 * mu_prior
    if t95_model(lo, e2, d1, d2, *lags) > t95:
        return 0.0
    for _ in range(60):
        mid = math.sqrt(lo * hi)
        if t95_model(mid, e2, d1, d2, *lags) > t95:
            hi = mid
        else:
            lo = mid
        if hi / lo < 1 + 1e-7:
            break
    return math.sqrt(lo * hi)


# ------------------------------------------------------------------ hold processing
def _arr(samples, key):
    return np.asarray(samples[key], float) if samples and key in samples else None


def n_eff(x):
    x = np.asarray(x, float)
    r1 = min(max(E.lag1(x), 0.0), 0.99)
    return max(len(x) * (1.0 - r1) / (1.0 + r1), 1.0)


class Hold:
    """One powered hold with its preceding neutral, as the gate sees it."""

    def __init__(self, index, rec, settings):
        self.index = index
        self.rec = rec
        self.direction, self.amplitude = direction_of(rec.get("command", [0, 0, 0]))
        self.start, self.end = float(rec.get("start", 0.0)), float(rec.get("end", 0.0))
        self.settings = settings
        self.samples = rec.get("samples")
        self.result = None      # per-hold check result
        self.delta = None       # 3-vector, hold mean - baseline mean
        self.se = None          # 3-vector standard error of delta
        self.neff = None
        self.baseline = None    # (mean vx, vy, wz, mean yaw or None, se vx, se vy, n)
        self.resid = None       # main-channel within-hold residuals

    def windows(self):
        s = self.settings
        settle = float(self.rec.get("settle", s["settle_s"]))
        b0 = self.start - settle + min(s["coast_s"], 0.5 * settle)
        return (b0, self.start), (self.start + s["transient_s"], self.end)


def check_hold(h: Hold, est, rejected, median_rate):
    """Per-hold checks (design section 4); fills the hold's delta on success."""
    if h.direction is None:
        return abstain("HOLD_INVALID:neutral_command", index=h.index)
    if h.rec.get("override"):
        return abstain("HOLD_INVALID:override", index=h.index)
    if (h.start, h.end) in rejected:
        return abstain("HOLD_INVALID:rejected_by_audit", index=h.index)
    t = _arr(h.samples, "t")
    if t is None or any(_arr(h.samples, c) is None for c in CHANNELS):
        return abstain("HOLD_INVALID:no_samples", index=h.index)
    (b0, b1), (w0, w1) = h.windows()
    v = est.window_validity(t, w0, w1, MAX_GAP_S, MIN_SAMPLES)
    if v["status"] != E.OK:
        return abstain(f"HOLD_INVALID:{v['reason']}", index=h.index, window="hold", **_ev(v))
    vb = est.window_validity(t, b0, b1 - 1e-9, MAX_GAP_S, MIN_SAMPLES)
    if vb["status"] != E.OK:
        return abstain(f"HOLD_INVALID:baseline_{vb['reason']}", index=h.index, window="baseline", **_ev(vb))
    mh = (t >= w0) & (t <= w1)
    mb = (t >= b0) & (t < b1)
    data = {c: _arr(h.samples, c) for c in CHANNELS}
    if all(np.ptp(data[c][mh]) == 0.0 for c in CHANNELS):
        return abstain("HOLD_INVALID:flat", index=h.index)
    same = np.ones(len(t), bool)
    same[0] = False
    for c in CHANNELS:
        same[1:] &= data[c][1:] == data[c][:-1]
    key = np.where(same, -1.0, np.arange(len(t), dtype=float))
    key = np.maximum.accumulate(key)  # index of the last change: repeats while the sample is held
    frozen = est.detect_frozen(key[mh])
    if frozen is not None:
        return abstain("HOLD_INVALID:frozen", index=h.index, at=float(t[mh][frozen]))
    coverage = float(mh.sum()) / max((w1 - w0) * median_rate, 1e-9)
    if coverage < MIN_COVERAGE:
        return abstain("HOLD_INVALID:coverage", index=h.index, coverage=coverage)
    ts = _arr(h.samples, "t_sensor")
    skew_ev = {"checked": ts is not None}
    if ts is not None:
        sk = est.estimate_clock_skew(ts[mh], t[mh], MAX_SKEW_S)
        if sk["status"] != E.OK:
            return abstain(f"HOLD_INVALID:clock_{sk['reason']}", index=h.index, **_ev(sk))
        if abs(sk["offset_at_end_s"]) > MAX_SKEW_S:
            return abstain("HOLD_INVALID:clock_skew", index=h.index, offset_at_end_s=sk["offset_at_end_s"])
        skew_ev["offset_at_end_s"] = sk["offset_at_end_s"]
    # A baseline still decaying from the previous hold (a coast shorter than the hull's settling
    # time) biases the main-channel delta the curve uses; that is the procedure's fault, never the
    # curve's (no MISFIT). Cross channels only feed signatures, so they are not checked here.
    j = parse_direction(h.direction)[0]
    early = t[mb] < 0.5 * (b0 + b1)
    xa, xz = data[CHANNELS[j]][mb][early], data[CHANNELS[j]][mb][~early]
    if len(xa) > 2 and len(xz) > 2:
        step = float(xz.mean() - xa.mean())
        se_step = math.sqrt(float(np.var(xa, ddof=1)) / n_eff(xa) + float(np.var(xz, ddof=1)) / n_eff(xz))
        if abs(step) > MISFIT_Z * se_step:
            return abstain("HOLD_INVALID:baseline_unsettled", index=h.index, channel=CHANNELS[j], step=step,
                           se=se_step, baseline_s=float(b1 - b0), note="lengthen the coast (settle_s) before each hold")
    delta, se, ne = [], [], []
    for c in CHANNELS:
        xh, xb = data[c][mh], data[c][mb]
        nh, nb = n_eff(xh), n_eff(xb)
        delta.append(float(xh.mean() - xb.mean()))
        se.append(math.sqrt(float(np.var(xh, ddof=1)) / nh + float(np.var(xb, ddof=1)) / nb))
        ne.append(nh)
    h.delta, h.se, h.neff = np.array(delta), np.array(se), ne[j]
    yaw = _arr(h.samples, "yaw")
    h.baseline = (float(data["vx"][mb].mean()), float(data["vy"][mb].mean()), float(data["wz"][mb].mean()),
                  None if yaw is None else float(np.angle(np.mean(np.exp(1j * yaw[mb])))),
                  se_mean(data["vx"][mb]), se_mean(data["vy"][mb]), int(mb.sum()))
    h.resid = data[CHANNELS[j]][mh] - data[CHANNELS[j]][mh].mean()
    return ok(index=h.index, coverage=coverage, n=int(mh.sum()), clock=skew_ev)


def se_mean(x):
    x = np.asarray(x, float)
    return math.sqrt(float(np.var(x, ddof=1)) / n_eff(x)) if len(x) > 1 else math.inf


def _ev(res):
    return {k: v for k, v in res.items() if k not in ("status", "reason")}


# ------------------------------------------------------------------ the gate
class Report:
    def __init__(self):
        self.checks = []
        self.signatures = []
        self.notes = []

    def add(self, check, scope, res):
        entry = {"check": check, "scope": scope, "status": res["status"]}
        if res["status"] != E.OK:
            entry["reason"] = res["reason"]
            entry["severity"] = res.get("severity", REJECT)
        entry["evidence"] = _jsonable(res.get("evidence", {}))
        self.checks.append(entry)
        return res

    def failures(self, scopes):
        return [c for c in self.checks if c["scope"] in scopes and c["status"] != E.OK]


def _jsonable(x):
    if isinstance(x, dict):
        return {str(k): _jsonable(v) for k, v in x.items()}
    if isinstance(x, (list, tuple)):
        return [_jsonable(v) for v in x]
    if isinstance(x, (np.floating, float)):
        f = float(x)
        return f if math.isfinite(f) else str(f)
    if isinstance(x, (np.integer,)):
        return int(x)
    if isinstance(x, np.bool_):
        return bool(x)
    return x


def run_gate(yaml_doc, provenance=None, run=None, snapshot=None, *, yaml_path=None,
             base_profile=None, tau_est=None, est=None):
    """Run every check and return the gate report (a JSON-serialisable dict)."""
    est = est or E.Estimators()
    base = base_profile or P.IDEAL
    yaml_doc, provenance, run, snapshot = (load_doc(x) for x in (yaml_doc, provenance, run, snapshot))
    char = characterization_block(yaml_doc)
    curves = char.get("speed_curves") or {}
    run = run or {}
    settings = {**DEFAULT_SETTINGS, **(run.get("settings") or {})}
    hardware = snapshot_hardware(snapshot)
    rep = Report()
    gains = axis_gains(base, hardware)
    v_ref = base["thrusters"][0]["vRef"]

    # ---------------- provenance (global + per direction)
    path_norm = (yaml_path or "").replace(os.sep, "/")
    sim_path = "/config/sim/" in "/" + path_norm or path_norm.startswith("config/sim/")
    prov_dirs = (provenance or {}).get("directions") or {}
    software_sha = ((provenance or {}).get("software") or {}).get("mhseals_testing")
    for name in DIRECTIONS:
        scope = f"provenance:{name}"
        entry = prov_dirs.get(name)
        hw_rec = (entry or {}).get("hardware") or {}
        if sim_path or hw_rec.get("sim_derived") or not hw_rec.get("hash") or "epoch" not in hw_rec:
            rep.add("sim_derived", scope, abstain("SIM_DERIVED", path=yaml_path, sim_path=sim_path,
                                                  flagged=bool(hw_rec.get("sim_derived")),
                                                  hardware_hash=hw_rec.get("hash"), epoch=hw_rec.get("epoch")))
        fit = (entry or {}).get("fit") or {}
        missing = [k for k in ("id", "session", "run") if not fit.get(k)]
        sha = fit.get("mhseals_testing") or software_sha
        if char.get("version") != 2:
            missing.append("characterization.version==2")
        if not sha:
            missing.append("mhseals_testing sha")
        if ((provenance or {}).get("software") or {}).get("dirty") is True or fit.get("dirty") is True:
            missing.append("clean mhseals_testing tree (software.dirty)")
        if provenance is not None and provenance.get("schema", "mhseals.calibration/1") != "mhseals.calibration/1":
            missing.append("schema mhseals.calibration/1")
        if entry is None or missing:
            rep.add("provenance_complete", scope, abstain("PROVENANCE_MISSING", missing=missing,
                                                          direction_recorded=entry is not None))
        if entry is not None:
            if hardware is None:
                rep.add("calibration_valid", scope, abstain("PROVENANCE_MISSING", missing=["config-history snapshot"]))
            else:
                want = dependency_hash(hardware, DEP_INVALID)
                if hw_rec.get("hash") and hw_rec.get("hash") != want:
                    rep.add("calibration_valid", scope, abstain("CALIBRATION_INVALID", recorded=hw_rec.get("hash"),
                                                                snapshot=want, fit_config=fit.get("config")))
                verify = dependency_hash(hardware, DEP_VERIFY)
                muted = bool((entry.get("gate") or {}).get("muted"))
                if muted or (hw_rec.get("verify") and hw_rec.get("verify") != verify):
                    rep.add("needs_verify", scope, abstain("NEEDS_VERIFY", WARN, muted=muted,
                                                           recorded=hw_rec.get("verify"), snapshot=verify))
            bv = entry.get("battery_v")
            if bv is not None and abs(bv - v_ref) > BATTERY_TOLERANCE_V:
                rep.notes.append({"note": "BATTERY_OFF_VREF", "direction": name, "battery_v": bv, "v_ref": v_ref,
                                  "action": "recorded, not rejected"})
        rep.add("provenance", scope, ok(battery_v=(entry or {}).get("battery_v"), v_ref=v_ref))

    # ---------------- conditions
    cond = run.get("conditions")
    if cond is None:
        cond = next((e.get("conditions") for e in prov_dirs.values() if e.get("conditions")), None)
    if not cond or cond.get("verdict") in (None, "unknown"):
        rep.add("conditions", "conditions", abstain("CONDITIONS_UNKNOWN", WARN, summary=cond))
    else:
        poor = (cond.get("verdict") != "good" or (cond.get("ceiling") is not None and cond["ceiling"] < CEILING_GOOD)
                or (cond.get("drift_trust") is not None and cond["drift_trust"] < DRIFT_TRUST_GOOD))
        if poor:
            rep.add("conditions", "conditions", abstain("CONDITIONS_POOR", verdict=cond.get("verdict"),
                                                        ceiling=cond.get("ceiling"), drift_trust=cond.get("drift_trust")))
        elif cond.get("state") in ("trending", "changed"):
            rep.add("conditions", "conditions", abstain("DRIFT_UNSTEADY", state=cond.get("state")))
        else:
            rep.add("conditions", "conditions", ok(verdict=cond.get("verdict"), state=cond.get("state")))

    # ---------------- holds
    rejected = {(float(r.get("start", 0)), float(r.get("end", 0))) for r in run.get("holds_rejected") or []}
    holds = [Hold(i, rec, settings) for i, rec in enumerate(run.get("holds") or [])]
    diffs = [np.diff(np.asarray(h.samples["t"], float)) for h in holds if h.samples and "t" in h.samples]
    diffs = [d for d in diffs if len(d)]
    median_rate = 1.0 / float(np.median(np.concatenate(diffs))) if diffs else 0.0
    for h in holds:
        h.result = rep.add("hold", f"hold:{h.index}", check_hold(h, est, rejected, median_rate))

    wave_floor = char.get("wave_floor")
    dir_fit = {}
    for name in DIRECTIONS:
        dir_fit[name] = check_direction(name, holds, curves.get(name), wave_floor, run, rep, est)

    # ---------------- consistency
    for axis in AXES:
        consistency(axis, curves, dir_fit, gains, rep)
    tau_meas = estimator_lag(run, est, rep)
    lags = (max(t["tauT"] for t in base["thrusters"]), max(t["deadTime"] for t in base["thrusters"]),
            tau_est if tau_est is not None else (tau_meas or 0.0))
    meff = {}
    for j, axis in enumerate(AXES):
        meff[axis] = check_t95(axis, j, char, curves.get(axis + "+"), holds, gains[axis + "+"]["k"], base, lags, rep)

    signatures(run, curves, dir_fit, holds, wave_floor, meff, char, rep, est)
    return outcome(rep, curves, gains, meff, char, base, yaml_path)


def check_direction(name, holds, curve, wave_floor, run, rep, est):
    """Per-direction checks; returns the recomputed fit (or None)."""
    scope = name
    j, sign = parse_direction(name)
    axis = AXES[j]
    mine = [h for h in holds if h.direction == name and h.result and h.result["status"] == E.OK]
    amps = sorted({round(h.amplitude, 6) for h in mine})
    v = np.array([sign * h.delta[j] for h in mine]) if mine else np.zeros(0)
    e2 = np.array([h.amplitude ** 2 for h in mine]) if mine else np.zeros(0)
    se = np.array([h.se[j] for h in mine]) if mine else np.zeros(0)
    fit = {"holds": [h.index for h in mine], "amplitudes": amps, "speeds": v.tolist()}
    if curve is None:
        rep.add("curve_present", scope, abstain("UNDERDETERMINED", why="no speed curve in the YAML"))
    # LOW_SNR
    wf = None
    if wave_floor and len(wave_floor) == 3:
        wf = float(wave_floor[WAVE_FLOOR_ORDER.index(axis)])
    low = []
    for h, vi in zip(mine, v):
        floor = wf if wf is not None else float(np.std(h.resid, ddof=1))
        thr = max(MIN_RESPONSE, 3.0 * floor / math.sqrt(h.neff))
        if vi < thr:
            low.append({"hold": h.index, "dv": float(vi), "threshold": thr})
    if low:
        rep.add("snr", scope, abstain("LOW_SNR", holds=low))
    # UNDERDETERMINED
    dof = len(mine) - 2
    if len(amps) < 2:
        rep.add("determined", scope, abstain("UNDERDETERMINED", distinct_speeds=len(amps), dof=dof))
        return None
    if len(amps) == 2:
        rep.add("determined", scope, abstain("UNDERDETERMINED", WARN, distinct_speeds=2, dof=dof,
                                             why="exact fit, capped at provisional"))
    # recompute the curve from the holds (detects the silent term-drop refit)
    X = np.column_stack([v, v * np.abs(v)])
    theta = np.linalg.lstsq(X, e2, rcond=None)[0]
    fit.update(d1=float(theta[0]), d2=float(theta[1]))
    if theta[0] < 0 or theta[1] < 0:
        rep.add("coefficients", scope, abstain("NEG_COEFF", d1=float(theta[0]), d2=float(theta[1]),
                                               yaml=curve))
    elif curve is not None:
        vmax = float(np.max(np.abs(v)))
        g_yaml, g_fit = curve_value(curve, vmax), float(theta[0] * vmax + theta[1] * vmax * vmax)
        if abs(g_yaml / g_fit - 1.0) > CURVE_MATCH:
            rep.add("curve_reproduced", scope, abstain("MISFIT", why="yaml curve not reproduced from holds",
                                                       yaml=curve, recomputed=fit, at_speed=vmax))
    # ILL_CONDITIONED
    Emat = np.column_stack([se, 2.0 * np.abs(v) * se])
    good, smin, noise, cond = E.min_sv_vs_noise(X, Emat, k=3.0, hard_cond=1e6)
    if not good:
        rep.add("conditioning", scope, abstain("ILL_CONDITIONED", smin=smin, noise=noise, cond=cond))
    # DEGENERATE / MISFIT on the curve residuals (only with spare dof)
    if dof >= 1:
        resid = e2 - X @ theta
        slope = theta[0] + 2.0 * theta[1] * np.abs(v)  # d(e^2)/dv, propagates the speed error
        if E.zero_variance(resid, scale=float(np.max(np.abs(e2)))) or float(np.max(np.abs(resid))) <= 1e-12 * float(np.max(e2)):
            rep.add("curve_residuals", scope, abstain("DEGENERATE", why="zero residual variance", dof=dof))
        elif np.all(slope > 0) and np.all(se > 0):  # otherwise NEG_COEFF/ILL_CONDITIONED already reject
            chi2 = float(np.sum((resid / (slope * se)) ** 2))
            crit = E.chi2_crit(dof, MISFIT_Z)
            if chi2 > crit:
                rep.add("curve_chi2", scope, abstain("MISFIT", why="chi2", chi2=chi2, crit=crit, dof=dof))
    # Ljung-Box on within-hold residuals decimated to the wave correlation time
    for h in mine:
        x, stride = E.decimate_white(h.resid, WHITE_R1)
        lb = E.ljung_box(x, LB_LAGS)
        if lb["status"] != E.OK:
            rep.add("ljung_box", scope, abstain("DEGENERATE", hold=h.index, why=lb["why"], n=lb["n"], stride=stride))
        else:
            crit = E.chi2_crit(LB_LAGS, MISFIT_Z)
            if lb["q"] > crit:
                rep.add("ljung_box", scope, abstain("MISFIT", why="ljung_box", hold=h.index, q=lb["q"], crit=crit,
                                                    stride=stride))
    # NOT_CONVERGED (cross responses)
    d = (run.get("directions") or {}).get(name)
    if d is None or d.get("dead") or not d.get("converged") or d.get("low_ok") is False:
        rep.add("cross_response", scope, abstain("NOT_CONVERGED", record=d))
    rep.add("direction", scope, ok(**fit))
    return fit


def consistency(axis, curves, dir_fit, gains, rep):
    cp, cm = curves.get(axis + "+"), curves.get(axis + "-")
    if not cp or not cm:
        return
    speeds = [abs(s) for n in (axis + "+", axis + "-") if dir_fit.get(n) for s in dir_fit[n]["speeds"]]
    v_ref = 0.5 * max(speeds) if speeds else 1.0
    ratio = curve_value(cp, v_ref) / curve_value(cm, v_ref)
    gp, gm = gains[axis + "+"], gains[axis + "-"]
    scope = f"consistency:{axis}"
    if gp["forward"] == gp["reverse"]:
        if not ASYM_RANGE[0] <= ratio <= ASYM_RANGE[1]:
            rep.add("asymmetry", scope, abstain("ASYMMETRY_IMPLAUSIBLE", ratio=ratio, at_speed=v_ref))
    else:
        hull = ratio * gp["k"] / gm["k"]
        if not ASYM_RANGE[0] <= hull <= ASYM_RANGE[1]:
            rep.add("reverse", scope, abstain("REVERSE_INCONSISTENT", d_ratio=ratio, implied_D_ratio=hull,
                                              k_plus=gp["k"], k_minus=gm["k"], counts=[gp["forward"], gp["reverse"]]))
    rep.add("consistency", scope, ok(d_ratio=ratio, at_speed=v_ref,
                                     counts={"+": [gp["forward"], gp["reverse"]], "-": [gm["forward"], gm["reverse"]]}))


def check_t95(axis, j, char, curve, holds, k, base, lags, rep):
    """M_eff/k from t95 with the lags held at their priors; returns {mu, meff} or None."""
    scope = f"t95:{axis}"
    t95 = (char.get("t95") or {}).get(axis)
    if t95 is None or curve is None:
        rep.add("t95", scope, abstain("UNDERDETERMINED", why="no t95 or no + curve", t95=t95))
        return None
    lag_sum = sum(lags)
    if lag_sum >= LAG_FRACTION * t95:
        rep.add("t95_lag", scope, abstain("LAG_DOMINATED", lags=list(lags), t95=t95))
        return None
    amps = [h.amplitude for h in holds if h.direction == axis + "+"]
    hi = max(amps) if amps else ((char.get("motion_limits") or {}).get("measured_at_effort") or [None] * 3)[j]
    if not hi:
        rep.add("t95", scope, abstain("UNDERDETERMINED", why="no high amplitude for the t95 hold"))
        return None
    meff_prior = P.effective_mass(base)[j]
    mu_prior = meff_prior / k
    mu = fit_mu(float(t95), hi * hi, curve["d1"], curve["d2"], lags, mu_prior)
    meff = mu * k
    rigid = base["mass"] if j < 2 else base["izz"]
    if not T95_RANGE[0] <= mu / mu_prior <= T95_RANGE[1] or meff < rigid:
        rep.add("t95_plausible", scope, abstain("T95_IMPLAUSIBLE", mu=mu, mu_prior=mu_prior, ratio=mu / mu_prior,
                                                meff=meff, rigid=rigid))
        return {"mu": mu, "meff": meff, "rejected": True}
    rep.add("t95", scope, ok(mu=mu, mu_prior=mu_prior, meff=meff, lags=list(lags), amplitude=hi))
    return {"mu": mu, "meff": meff}


def estimator_lag(run, est, rep):
    ls = run.get("lag_streams")
    if not ls:
        return None
    r = est.fit_delay(np.asarray(ls["t_raw"], float), [np.asarray(ls["raw"], float)],
                      np.asarray(ls["t_odom"], float), [np.asarray(ls["odom"], float)], [float(ls["sigma"])])
    if r["status"] == E.OK and r["delay_s"] > 2.0 * r["std_s"]:
        rep.signatures.append({"signature": "ESTIMATOR_LAG", "delay_s": r["delay_s"], "std_s": r["std_s"],
                               "note": "odom lags raw IMU/GPS; held as the estimator-lag prior for LAG_DOMINATED"})
        return r["delay_s"]
    rep.notes.append({"note": "estimator lag not resolved", "result": _jsonable(r)})
    return None


def signatures(run, curves, dir_fit, holds, wave_floor, meff, char, rep, est):
    sig = rep.signatures
    # THRUST_OR_HULL_ASYM: cross response at a constant ratio across amplitudes, or d+ != d-
    for name, d in (run.get("directions") or {}).items():
        if name not in DIRECTIONS or not d or d.get("dead"):
            continue
        hi, lo = d.get("response"), d.get("low_response")
        if not hi or not lo:
            continue
        axis = name[:-1]
        cross = [a for a in AXES if a != axis]
        for kk, (a, b) in enumerate(zip(hi, lo)):
            tol = TOLERANCE[axis][kk]
            if abs(a) > tol and abs(b) > tol and a * b > 0 and 0.5 <= b / a <= 2.0:
                sig.append({"signature": "THRUST_OR_HULL_ASYM", "direction": name, "cross_axis": cross[kk],
                            "ratio_high": a, "ratio_low": b, "tolerance": tol,
                            "note": "cross response at a constant ratio across amplitudes; thrust imbalance vs hull "
                                    "cannot be split (single-thruster pulses and the bollard pull split it)"})
    for axis in AXES:
        cp, cm = curves.get(axis + "+"), curves.get(axis + "-")
        if cp and cm:
            r = curve_value(cp, 1.0) / curve_value(cm, 1.0)
            if abs(r - 1.0) > ASYM_SIGNATURE:
                sig.append({"signature": "THRUST_OR_HULL_ASYM", "axis": axis, "d_ratio_at_1": r,
                            "note": "d+ != d-: hull asymmetry or per-thruster imbalance, never a reverse ratio "
                                    "when the axis runs equal forward/reverse thruster counts"})
    # PROP_DRAG_OR_K: coast-down deceleration vs the curve-implied one
    for cd in run.get("coastdowns") or []:
        axis = cd["axis"]
        j = AXES.index(axis)
        name = axis + ("+" if cd.get("sign", 1) > 0 else "-")
        m = meff.get(axis)
        s = cd.get("samples") or {}
        t, x = _arr(s, "t"), _arr(s, CHANNELS[j])
        if m is None or m.get("rejected") or t is None or curves.get(name) is None:
            rep.notes.append({"note": "coast-down not compared", "axis": axis, "why": "no M_eff/k or samples"})
            continue
        sel = (t >= cd["start"]) & (t <= cd["end"])
        t, x = t[sel], np.abs(x[sel])
        acc = -np.gradient(x, t)
        A = np.column_stack([x, x * x])
        coef = np.linalg.lstsq(A, acc, rcond=None)[0]
        vm = float(np.median(x))
        dec_coast = float(coef[0] * vm + coef[1] * vm * vm)
        dec_curve = curve_value(curves[name], vm) / m["mu"]
        rel = dec_coast / dec_curve - 1.0
        if abs(rel) > COAST_MISMATCH:
            sig.append({"signature": "PROP_DRAG_OR_K", "axis": axis, "coast_decel": dec_coast,
                        "curve_decel": dec_curve, "at_speed": vm, "relative": rel,
                        "note": "idle-prop drag kappa or a k error; separable once the bollard k exists"})
    # Environment: neutral drift, subtracted (hold minus neutral) and never put in D
    neutral = []
    for h in holds:
        if h.baseline is not None and h.baseline[3] is not None:
            neutral.append((h.baseline[3], h.baseline[0], h.baseline[1], max(h.baseline[4], h.baseline[5])))
    for w in run.get("neutral") or []:
        s = w.get("samples") or {}
        t = _arr(s, "t")
        if t is None or _arr(s, "yaw") is None:
            continue
        msk = (t >= w["start"]) & (t <= w["end"])
        if msk.sum() < MIN_SAMPLES:
            continue
        neutral.append((float(np.angle(np.mean(np.exp(1j * _arr(s, "yaw")[msk])))), float(_arr(s, "vx")[msk].mean()),
                        float(_arr(s, "vy")[msk].mean()), max(se_mean(_arr(s, "vx")[msk]), se_mean(_arr(s, "vy")[msk]))))
    env = None
    if len(neutral) >= 3:
        psi = np.array([n[0] for n in neutral])
        dxy = np.array([[n[1], n[2]] for n in neutral])
        sigma = float(np.median([n[3] for n in neutral]))
        sep = est.separate_current_bias(psi, dxy, sigma, max_std=DRIFT_PRACTICAL / 2)
        world = np.array([[math.cos(p) * a - math.sin(p) * b, math.sin(p) * a + math.cos(p) * b]
                          for p, (a, b) in zip(psi, dxy)])
        mean_world = world.mean(axis=0)
        se_world = sigma / math.sqrt(len(neutral))
        if sep["status"] == E.OK and np.hypot(*sep["current"]) > max(DRIFT_PRACTICAL, 3 * max(sep["std"][:2])):
            env = {"signature": "ENVIRONMENT_DRIFT", "current_world": sep["current"], "body_bias": sep["bias_xy"],
                   "separated": True}
        elif np.hypot(*mean_world) > max(DRIFT_PRACTICAL, 3 * se_world):
            env = {"signature": "ENVIRONMENT_DRIFT", "drift_world_mean": mean_world.tolist(), "separated": False,
                   "split": sep.get("reason", "ok but below threshold")}
    cond = run.get("conditions") or {}
    if env is None and cond.get("drift") and np.hypot(*cond["drift"]) > DRIFT_PRACTICAL:
        env = {"signature": "ENVIRONMENT_DRIFT", "drift_conditions": cond["drift"], "separated": False}
    if env is not None:
        env["note"] = "environment: subtracted as hold minus neutral, never put in D"
        sig.append(env)
    # GYRO_BIAS: a constant r at neutral
    for w in run.get("neutral") or []:
        s = w.get("samples") or {}
        t, r = _arr(s, "t"), _arr(s, "wz")
        if t is None or r is None or w["end"] - w["start"] < GYRO_MIN_S:
            continue
        sel = (t >= w["start"]) & (t <= w["end"])
        t, r = t[sel], r[sel]
        if len(t) == 0:
            continue
        mask = t >= t[0] + 3.0  # 3 s of recorded zero command before the rest window
        sr = float(wave_floor[0]) if wave_floor and len(wave_floor) == 3 and wave_floor[0] > 0 else float(np.std(r, ddof=1))
        g = est.gyro_bias(t, r, mask, np.zeros(len(t)), sr, settle_s=3.0)
        if g["status"] == E.OK:
            se = sr / math.sqrt(n_eff(r[mask]))
            if abs(g["bias_r"]) > 3.0 * se:
                sig.append({"signature": "GYRO_BIAS", "bias_r": g["bias_r"], "se": se,
                            "note": "constant yaw rate at neutral; cancels in hold-minus-neutral responses"})
                break
        else:
            rep.notes.append({"note": "gyro bias window not usable", "reason": g["reason"]})
    if not run.get("scale_anchor"):
        sig.append({"signature": "SCALE_UNANCHORED",
                    "note": "a GNSS/odom velocity scale error scales every d alike; indistinguishable from k "
                            "without a known-distance run"})


def outcome(rep, curves, gains, meff, char, base, yaml_path):
    def combine(scopes):
        fails = rep.failures(scopes)
        rejects = [f for f in fails if f.get("severity", REJECT) == REJECT]
        warns = [f for f in fails if f.get("severity") == WARN]
        reasons = sorted({f["reason"] for f in rejects}) or sorted({f["reason"] for f in warns})
        status = PRIOR if rejects else PROVISIONAL if warns else IDENTIFIED
        return status, reasons

    params = {}
    for name in DIRECTIONS:
        axis = name[:-1]
        status, reasons = combine({f"provenance:{name}", "conditions", name, f"consistency:{axis}"})
        p = {"status": status, "reasons": reasons, "reason": ", ".join(reasons) or "all checks pass",
             "k": gains[name]["k"]}
        c = curves.get(name)
        if c is not None:
            p.update(d1=c["d1"], d2=c["d2"], D1=c["d1"] * gains[name]["k"], D2=c["d2"] * gains[name]["k"])
        params[name] = p
    for j, axis in enumerate(AXES):
        key = f"meff.{axis}"
        status, reasons = combine({f"provenance:{axis}+", "conditions", f"t95:{axis}"})
        dplus = params[axis + "+"]
        rank = {IDENTIFIED: 0, PROVISIONAL: 1, PRIOR: 2}
        if rank[dplus["status"]] > rank[status]:  # mu comes from the + curve: no better than it
            status = dplus["status"]
            reasons = sorted(set(reasons) | {f"{axis}+:{r}" for r in dplus["reasons"]})
        p = {"status": status, "reasons": reasons, "reason": ", ".join(reasons) or "all checks pass"}
        m = meff.get(axis)
        if m is not None:
            p.update(mu=m["mu"], meff=m["meff"])
        params[key] = p
    characterized = all(params[n]["status"] == IDENTIFIED for n in ("surge+", "surge-", "yaw+", "yaw-"))
    counted = [params[n]["status"] for n in params]
    if characterized:
        summary = "characterized"
    elif any(s != PRIOR for s in counted):
        summary = "partial"
    else:
        summary = "prior"
    top = sorted({r for p in params.values() for r in p["reasons"] if p["status"] != IDENTIFIED})
    return _jsonable({
        "schema": SCHEMA, "yaml": yaml_path, "summary": summary, "characterized": characterized,
        "reasons": top, "parameters": params, "signatures": rep.signatures, "notes": rep.notes,
        "envelope": ((char.get("motion_limits") or {}).get("velocity")),
        "checks": rep.checks})
