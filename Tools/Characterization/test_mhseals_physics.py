"""mhseals_physics.py: an mhseals.physics/1 survey record (characterization-v2) through the adapter,
the gate and import_yaml. The fixture writes what mhseals_testing physics_record.write and
procedures/physics.survey write (record, compact samples sidecar with its sha256, config-history),
with the survey run on the exact Ideal twin."""
import copy
import hashlib
import json
import math
import os

import numpy as np
import pytest
import yaml

import gate as G
import import_yaml as I
import mhseals_physics as MP
import plant as P
import synthetic as S

FIT = "f20261025T141200"
CFG = {"levels": [1.0, 0.75, 0.5, 0.25], "repeats": 2, "steady_share": 0.4}  # config/physics.yaml survey
SIGMA = (0.01, 0.01, 0.005)  # odometry vx, vy [m/s], wz [rad/s]


def _speed_fit(points):
    """procedures/physics speed_fit shape: e|e| = d1 v + d2 v|v| with covariance and residuals."""
    v = np.array([p[0] for p in points])
    e2 = np.array([p[1] * abs(p[1]) for p in points])
    X = np.column_stack([v, v * np.abs(v)])
    d, *_ = np.linalg.lstsq(X, e2, rcond=None)
    res = e2 - X @ d
    dof = len(v) - 2
    var = float(res @ res / dof)
    return {"covariance": (var * np.linalg.inv(X.T @ X)).tolist(), "residuals": res.tolist(), "dof": dof,
            "residual_var": var, "cond": float(np.linalg.cond(X)), "d1": float(d[0]), "d2": float(d[1]),
            "n": len(v), "levels": len({p[2] for p in points}), "pure_error_var": None,
            "points": [list(p) for p in points]}


def write_survey(root, *, sim_derived=False, dirty=False, seed=4, settle_s=40.0, hold_s=10.0):
    """<root>/characterization/<fit>-survey.{yaml,samples.json} and <root>/config-history."""
    hw = S.hardware()
    hw.update(decoupling=[[1, 0, 0], [0, 1, 0], [0, 0, 1]], decoupling_negative=None)
    value = {"overrides": {"reverse_thrust_ratio": 1.0}, "effective": hw}
    sha = hashlib.sha256(G._canonical(value).encode()).hexdigest()[:12]
    hist = os.path.join(root, "config-history")
    os.makedirs(os.path.join(hist, "versions", "hardware"))
    with open(os.path.join(hist, "changes.jsonl"), "w") as f:
        for cid, before, after in (("c0006", None, "0123456789ab"), ("c0007", "0123456789ab", sha)):
            f.write(json.dumps({"schema": "mhseals.config_change/1", "id": cid, "subsystem": "hardware",
                                "before": before, "after": after, "actor": "operator"}) + "\n")
    with open(os.path.join(hist, "versions", "hardware", f"{sha}.json"), "w") as f:
        json.dump({"schema": "mhseals.config_version/1", "subsystem": "hardware", "sha": sha, "value": value,
                   "raw": "", "meta": {}}, f, indent=2)

    rng = np.random.default_rng(seed)
    dt = 0.04
    pl = P.ProfilePlant(P.IDEAL, dt)
    t, odo, segments, holds = 0.0, [], [], []

    def run(label, command, seconds, **extra):
        nonlocal t
        seg = {"label": label, "start": round(t, 6), "end": None, "command": list(command),
               "pulses": P.pwm_for_effort(*command), "battery": [[t, 15.9]], "overridden": False, **extra}
        for _ in range(int(round(seconds / dt))):
            pl.step_pwm(seg["pulses"])
            t += dt
            u, v, r = pl.body_velocity()
            odo.append([round(t, 4), u + rng.normal(0, SIGMA[0]), v + rng.normal(0, SIGMA[1]),
                        r + rng.normal(0, SIGMA[2]), pl.state[2]])
        seg["end"] = round(t, 6)
        seg["battery"].append([t, 15.9])
        segments.append(seg)
        return seg

    run("neutral", (0.0, 0.0, 0.0), settle_s)
    for repeat in range(CFG["repeats"]):
        for level in (CFG["levels"] if repeat % 2 == 0 else CFG["levels"][::-1]):
            for axis in range(3):
                for sign in (1, -1):
                    cmd = [0.0] * 3
                    cmd[axis] = sign * level
                    name = G.AXES[axis] + ("+" if sign > 0 else "-")
                    seg = run(f"{name} {level:.2f}", cmd, hold_s, axis=axis, sign=sign, level=level,
                              repeat=repeat, applied=list(cmd))
                    begin = seg["end"] - CFG["steady_share"] * hold_s
                    rows = np.array([r for r in odo if begin <= r[0] <= seg["end"]])
                    stats = {"n": len(rows)}
                    for i, k in enumerate(("vx", "vy", "wz"), 1):
                        stats[k], stats[k + "_sd"] = float(rows[:, i].mean()), float(rows[:, i].std(ddof=1))
                    stats["yaw"] = float(rows[:, 4].mean())
                    seg["stats"] = stats
                    holds.append(dict(name=seg["label"], start=seg["start"], end=seg["end"], command=cmd,
                                      applied=list(cmd), repeat=repeat, overridden=False, **stats))
                    run("coast", (0.0, 0.0, 0.0), settle_s)
    curves = {}
    for axis, key in enumerate(("vx", "vy", "wz")):
        for sign in (1, -1):
            name = G.AXES[axis] + ("+" if sign > 0 else "-")
            curves[name] = _speed_fit([(sign * h[key], sign * h["applied"][axis], abs(h["applied"][axis]))
                                       for h in holds if h["name"].split()[0] == name])
    tails = np.array([r[1:4] for s in segments if s["label"] == "coast" for r in odo
                      if s["end"] - 8.0 <= r[0] <= s["end"]])
    folder = os.path.join(root, "characterization")
    os.makedirs(folder)
    samples = {"clock": "monitor receive time, s", "pulse_source": "commanded", "segments": segments,
               "odometry": odo, "gyro": [[r[0], r[3]] for r in odo]}
    text = json.dumps(samples, separators=(",", ":"), allow_nan=False) + "\n"
    with open(os.path.join(folder, f"{FIT}-survey.samples.json"), "w") as f:
        f.write(text)
    source = ("/opt/ros/ws/install/mhseals_testing/share/mhseals_testing/config/sim/hardware_crane.yaml"
              if sim_derived else "/home/roboboat/.config/mhseals/hardware.yaml")
    record = {
        "schema": MP.SCHEMA, "step": "survey",
        "fit": {"id": FIT, "time": "2026-10-25T14:12:00-04:00", "session": "s0012", "run": "r0003"},
        "software": {"mhseals_testing": "38b96a45db5693ef175b3235b12e811181960640", "dirty": dirty},
        "hardware": {"source": source, "sim_derived": sim_derived, "history": {"id": "c0007", "sha": sha},
                     "hash": G.dependency_hash(hw, G.DEP_INVALID),
                     "physics_hash": G.dependency_hash(hw, ("thruster_matrix", "reversed", "hardware_epoch")),
                     "keys": {k: hw.get(k) for k in ("thruster_matrix", "reversed", "reverse_thrust_ratio",
                                                     "decoupling", "decoupling_negative", "channel_map",
                                                     "hardware_epoch")},
                     "thrusters": ["FL", "FR", "RR", "RL"],
                     "driver": {"config_sha": sha, "effective_config_sha": sha, "pending_restart_keys": []}},
        "conditions": copy.deepcopy(S.GOOD_CONDITIONS), "battery_v": 15.9, "gate_muted": False,
        "amplitude": 1.0, "levels": CFG["levels"], "repeats": CFG["repeats"], "effort_source": "commanded",
        "speed_curves": curves, "coast_down": {n: None for n in G.DIRECTIONS}, "holds": holds,
        "wave_floor": dict(zip(("vx", "vy", "wz"), tails.std(axis=0, ddof=1).tolist())), "env_floor": None,
        "heading_spread": {"R": 0.9, "ok": False, "max": 0.9, "n": len(holds)},
        "units": {"effort": "normalized", "v": "m/s (surge, sway), rad/s (yaw)", "time": "s, monitor clock"},
        "counts": {"holds": len(holds), "samples": len(odo), "overridden": 0},
        "samples_file": {"path": f"{FIT}-survey.samples.json",
                         "sha256": "sha256:" + hashlib.sha256(text.encode()).hexdigest()},
    }
    path = os.path.join(folder, f"{FIT}-survey.yaml")
    with open(path, "w") as f:
        yaml.safe_dump(record, f, sort_keys=False)
    return path


@pytest.fixture(scope="module")
def survey(tmp_path_factory):
    return write_survey(str(tmp_path_factory.mktemp("data")))


def _import(path, tmp_path):
    out, rep = str(tmp_path / "profile.json"), str(tmp_path / "report.json")
    assert I.main(["--yaml", path, "--out-profile", out, "--out-report", rep]) == 0
    with open(out) as f, open(rep) as g:
        return json.load(f), json.load(g)


def test_adapter_maps_the_record_to_gate_inputs(survey):
    char, prov, run, snap = MP.adapt(survey)
    assert char["characterization"]["version"] == 2 and "t95" not in char["characterization"]
    assert len(run["holds"]) == 48 and len(run["coastdowns"]) == 48 and len(run["neutral"]) == 1
    h = run["holds"][0]
    assert h["command"] == [1.0, 0.0, 0.0] and h["override"] is False and h["settle"] == pytest.approx(40.0)
    assert h["samples"]["t"][0] == pytest.approx(h["start"] - 40.0, abs=0.05)
    assert run["settings"]["transient_s"] == pytest.approx(6.0)
    wf = char["characterization"]["wave_floor"]
    assert wf[0] < wf[1] and len(wf) == 3, "[wz, vx, vy]"
    assert snap["sha"] == yaml.safe_load(open(survey))["hardware"]["history"]["sha"]
    assert prov["directions"]["yaw-"]["hardware"]["sim_derived"] is False


def test_survey_imports_d_and_keeps_m_eff_prior(survey, tmp_path):
    prof, rep = _import(survey, tmp_path)
    for n in ("surge+", "surge-", "yaw+", "yaw-", "sway+", "sway-"):
        assert rep["parameters"][n]["status"] in (G.IDENTIFIED, G.PROVISIONAL), (n, rep["parameters"][n])
    for axis in G.AXES:
        for sfx in ("Pos", "Neg"):
            d_ideal = P.IDEAL[axis]["d1" + sfx] + P.IDEAL[axis]["d2" + sfx]  # D(1)
            d_got = prof[axis]["d1" + sfx] + prof[axis]["d2" + sfx]
            assert d_got == pytest.approx(d_ideal, rel=0.08), (axis, sfx, d_got, d_ideal)
        assert rep["parameters"][f"meff.{axis}"]["status"] == G.PRIOR
        assert "UNDERDETERMINED" in rep["parameters"][f"meff.{axis}"]["reasons"]
    assert prof["addedMass"] == P.IDEAL["addedMass"]
    assert any(i["path"].endswith(".samples.json") for i in prof["provenance"]["inputs"])


def test_sim_derived_record_is_rejected(tmp_path):
    path = write_survey(str(tmp_path / "data"), sim_derived=True)
    prof, rep = _import(path, tmp_path)
    assert all("SIM_DERIVED" in rep["parameters"][n]["reasons"] for n in G.DIRECTIONS)
    assert not rep["characterized"]
    for axis in G.AXES:
        assert prof[axis] == P.IDEAL[axis]
    assert "SIM_DERIVED" in prof["provenance"]["summary"]


def test_samples_sha_mismatch_refuses(survey, tmp_path):
    folder = os.path.dirname(survey)
    bad = tmp_path / "characterization"
    bad.mkdir()
    for name in os.listdir(folder):
        text = open(os.path.join(folder, name)).read()
        (bad / name).write_text(text.replace('"overridden":false', '"overridden":true', 1)
                                if name.endswith(".json") else text)
    with pytest.raises(ValueError, match="SAMPLES_SHA_MISMATCH"):
        MP.adapt(str(bad / os.path.basename(survey)), os.path.join(os.path.dirname(folder), "config-history"))


def test_snapshot_comes_from_history_never_the_record_keys(survey, tmp_path):
    rec = yaml.safe_load(open(survey))
    rec["hardware"]["keys"]["reversed"] = [True, True, True, True]  # the record's own keys are ignored
    hist = os.path.join(os.path.dirname(os.path.dirname(survey)), "config-history")
    p = tmp_path / os.path.basename(survey)
    p.write_text(yaml.safe_dump(rec, sort_keys=False))
    (tmp_path / rec["samples_file"]["path"]).write_bytes(
        open(os.path.join(os.path.dirname(survey), rec["samples_file"]["path"]), "rb").read())
    _, _, _, snap = MP.adapt(str(p), hist)
    assert snap["value"]["effective"]["reversed"] == [False] * 4
    rec["hardware"]["history"]["id"] = "c0006"  # names another version than the record saw
    p.write_text(yaml.safe_dump(rec, sort_keys=False))
    assert MP.adapt(str(p), hist)[3] is None
    rep = G.run_gate(*MP.adapt(str(p), hist), yaml_path=str(p))
    assert all("PROVENANCE_MISSING" in rep["parameters"][n]["reasons"] for n in G.DIRECTIONS)


def test_dirty_software_tree_is_provenance_missing(tmp_path):
    path = write_survey(str(tmp_path / "data"), dirty=True)
    rep = G.run_gate(*MP.adapt(path), yaml_path=path)
    assert all("PROVENANCE_MISSING" in rep["parameters"][n]["reasons"] for n in G.DIRECTIONS)


def test_short_coasts_are_blamed_on_the_procedure_not_the_curve(tmp_path):
    """The producer's settle_s 10 s: the Ideal hull's surge/sway coasts have not settled, so those
    holds are HOLD_INVALID:baseline_unsettled (lengthen settle_s) with the fix named. Holds biased
    below detection can still add a MISFIT (open item in the handoff)."""
    path = write_survey(str(tmp_path / "data"), settle_s=10.0)
    rep = G.run_gate(*MP.adapt(path), yaml_path=path)
    flagged = [c for c in rep["checks"] if c.get("reason") == "HOLD_INVALID:baseline_unsettled"]
    assert sum(c["evidence"]["channel"] in ("vx", "vy") for c in flagged) >= 10, flagged
    assert all("settle_s" in c["evidence"]["note"] for c in flagged)
    for n in ("surge+", "surge-"):
        assert rep["parameters"][n]["status"] == G.PRIOR
    assert rep["parameters"]["yaw+"]["status"] == G.IDENTIFIED


E2E_PROFILE = os.path.join(P.HERE, "profiles", "fixture-survey.json")


def test_e2e_fixture_profile_is_current(tmp_path):
    """Fixture YAML -> adapter -> gate -> profile JSON, committed for the EditMode end-to-end test
    (ProfileImportTests.SurveyFixtureImportsAndThePlantUsesIt). Rewrite with CRANE_WRITE_FIXTURES=1."""
    path = write_survey(str(tmp_path / "data"))
    prof, _ = _import(path, tmp_path)
    prov = prof["provenance"]
    prov["inputs"] = [{"path": os.path.basename(i["path"]), "sha256": i["sha256"]} for i in prov["inputs"]]
    prov["gateReport"] = "fixture-survey.gate.json (not kept)"
    prof["name"] = "FixtureSurvey"
    if os.environ.get("CRANE_WRITE_FIXTURES"):
        with open(E2E_PROFILE, "w") as f:
            json.dump(prof, f, indent=4)
            f.write("\n")
    with open(E2E_PROFILE) as f:
        kept = json.load(f)
    for axis in G.AXES:
        for key, value in prof[axis].items():
            assert kept[axis][key] == pytest.approx(value, rel=1e-9), (axis, key)
    assert kept["provenance"]["summary"] == prov["summary"] and kept[axis] != P.IDEAL[axis]
