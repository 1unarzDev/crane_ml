import copy
import hashlib
import json
import math
import os
import subprocess
import sys

import numpy as np
import pytest

import geom as g
import gen_provisional
import minischema
from rover_model import RoverModel

HERE = os.path.dirname(os.path.abspath(__file__))
ASSETS = os.path.dirname(HERE)
SCHEMA = json.load(open(os.path.join(ASSETS, "fleet_asset_manifest_v0.schema.json")))
ROLES = ["scout", "lifter", "cargo", "payloads"]


def man(d, role):
    return json.load(open(os.path.join(d, role, f"{role}.manifest.json")))


@pytest.mark.parametrize("role", ROLES)
def test_generated_manifest_conforms_to_schema(gen_dir, role):
    assert minischema.validate(man(gen_dir, role), SCHEMA) == []


@pytest.mark.parametrize("role", ["scout", "lifter", "cargo"])
def test_sections_have_separate_provenance_and_source(gen_dir, role):
    m = man(gen_dir, role)
    for sec in ("render", "collision", "mass_properties", "wheels", "sensors", "attachments", "actuated"):
        assert m[sec]["provenance"] == "provisional" and m[sec]["source_file"]


def test_schema_rejects_bad_manifests(gen_dir):
    m = man(gen_dir, "cargo")
    bad = copy.deepcopy(m)
    bad["collision"]["provenance"] = "guess"
    assert minischema.validate(bad, SCHEMA)
    bad = copy.deepcopy(m)
    del bad["mass_properties"]["inertia_tensor"]
    assert minischema.validate(bad, SCHEMA)
    bad = copy.deepcopy(m)
    bad["units"]["length"] = "mm"
    assert minischema.validate(bad, SCHEMA)
    bad = copy.deepcopy(m)
    del bad["wheels"]
    assert minischema.validate(bad, SCHEMA)            # role-conditional requirement
    bad = copy.deepcopy(m)
    bad["wheels"]["items"][0]["roller_angle_sign"] = 2
    assert minischema.validate(bad, SCHEMA)


def test_generation_is_deterministic(gen_dir, tmp_path):
    out2 = str(tmp_path / "again")
    gen_provisional.main(["--out", out2])
    def digest(d):
        h = {}
        for root, _, fs in os.walk(d):
            for f in fs:
                p = os.path.join(root, f)
                h[os.path.relpath(p, d)] = hashlib.sha256(open(p, "rb").read()).hexdigest()
        return {k: v for k, v in h.items() if not k.startswith("_")}
    a, b = digest(gen_dir), digest(out2)
    assert set(a) == set(b)
    assert [k for k in a if a[k] != b[k]] == []


def test_dimensions_come_from_the_profile(tmp_path):
    """No hard-coded sizes: edit the profile and the geometry follows."""
    prof = json.load(open(gen_provisional.DEFAULT_PROFILE))
    prof["chassis"]["body_size_lwh"]["value"] = [0.50, 0.34, 0.12]
    prof["chassis"]["wheel"]["roller_count"]["value"] = 12
    prof["chassis"]["wheel"]["half_track_ly"]["value"] = 0.2
    prof["chassis"]["wheel"]["half_wheelbase_lx"]["value"] = 0.17
    prof["docking"]["face_frames"]["front"]["value"] = [0.25, 0.0, 0.07, 0.0]
    prof["docking"]["face_frames"]["rear"]["value"] = [-0.25, 0.0, 0.07, 3.14159265]
    pp = str(tmp_path / "profile.json")
    json.dump(prof, open(pp, "w"))
    out = str(tmp_path / "out")
    gen_provisional.main(["--profile", pp, "--out", out, "--roles", "cargo"])
    mdl = RoverModel(os.path.join(out, "cargo", "cargo.manifest.json"))
    lo, hi = mdl.world_mesh("chassis_body", {}).bbox()
    assert hi[0] - lo[0] == pytest.approx(0.50, abs=2e-3) and hi[1] - lo[1] == pytest.approx(0.34, abs=2e-3)
    wm = mdl.mesh("wheel_FL")
    assert len([n for n in wm.groups if n.startswith("roller")]) == 12
    assert mdl.m["wheels"]["items"][0]["position"][:2] == [0.17, 0.2]


def test_roller_count_and_handedness_per_wheel(gen_dir):
    m = man(gen_dir, "scout")
    signs = {w["name"]: w["roller_angle_sign"] for w in m["wheels"]["items"]}
    assert signs == {"FL": -1, "FR": 1, "RL": 1, "RR": -1}
    mdl = RoverModel(os.path.join(gen_dir, "scout", "scout.manifest.json"))
    assert len([n for n in mdl.mesh("wheel_FL").groups if n.startswith("roller")]) == 10


def test_x_configuration_kinematics_matches_documented_signs():
    """The documented inverse kinematics must reproduce: all forward -> +x, FL/RR back + FR/RL fwd -> +y (left)."""
    lx, ly, r = 0.14, 0.175, 0.0485
    k = np.array([-1, 1, 1, -1.0])       # FL FR RL RR : roller-axis sign s_i
    kz = np.array([-1, 1, -1, 1.0])
    def wheel_speeds(vx, vy, wz):
        return (vx + k * vy + (lx + ly) * kz * wz) / r
    w = wheel_speeds(0.3, 0, 0)
    assert np.allclose(w, w[0])
    w = wheel_speeds(0, 0.2, 0)
    assert np.sign(w).tolist() == [-1, 1, 1, -1]
    # forward kinematics inverse
    vx = r / 4 * w.sum()
    vy = r / 4 * (-w[0] + w[1] + w[2] - w[3])
    assert (vx, vy) == pytest.approx((0, 0.2), abs=1e-12)


def test_hook_geometry_closes_on_opposing_pins(gen_dir):
    m = man(gen_dir, "cargo")
    hooks = [h for h in m["actuated"]["hooks"] if h["face"] == "front"]
    assert len(hooks) == 2 and hooks[0]["open_angle"] == -hooks[1]["open_angle"]
    assert hooks[0]["level_dz"] == -hooks[1]["level_dz"] != 0


def test_payload_manifest(gen_dir):
    m = man(gen_dir, "payloads")
    names = {i["name"]: i for i in m["payloads"]["items"]}
    assert set(names) == {"box_small", "box_long", "tray_wide"}
    assert names["box_long"]["size_lwh"] == [0.8, 0.25, 0.15]
    for it in names.values():
        I = np.array(it["inertia_tensor"])
        assert np.all(np.linalg.eigvalsh(I) > 0)


def test_tag_plate_has_uvs_and_texture(gen_dir):
    mdl = RoverModel(os.path.join(gen_dir, "cargo", "cargo.manifest.json"))
    q = mdl.mesh("tag_front")
    assert len(q) == 2 and q.UV.min() == 0 and q.UV.max() == 1
    t = [t for t in mdl.m["attachments"]["tags"] if t["name"] == "tag_front"][0]
    assert os.path.exists(os.path.normpath(os.path.join(mdl.dir, t["texture"])))
    assert t["plate_size"] == pytest.approx(t["size"] * 1.25)


def test_no_generator_default_missing_provenance():
    d = json.load(open(os.path.join(ASSETS, "generator_defaults.json")))["groups"]
    for grp, items in d.items():
        for k, v in items.items():
            if k.startswith("_"):
                continue
            assert v["provenance"] == "provisional", (grp, k)
