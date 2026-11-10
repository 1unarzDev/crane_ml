import copy
import json
import math
import os
import shutil

import numpy as np
import pytest

import geom as g
import validate_manifest as V


def path(d, role):
    return os.path.join(d, role, f"{role}.manifest.json")


def load(d, role):
    return json.load(open(path(d, role)))


def save(d, role, m):
    json.dump(m, open(path(d, role), "w"), indent=1)


def level(rep, cid):
    return [c["level"] for c in rep.checks if c["id"] == cid or c["id"].startswith(cid + ".")]


def run(d, role, mate=False, **kw):
    return V.validate(path(d, role), None, kw or None, None, mate)


# ------------------------------------------------------------ baseline
@pytest.mark.parametrize("role", ["scout", "lifter", "cargo", "payloads"])
def test_provisional_set_passes(gen_dir, role):
    rep = V.validate(path(gen_dir, role), None, None, None, True)
    fails = [c for c in rep.checks if c["level"] == "FAIL"]
    assert fails == [], fails
    j = rep.to_json()
    assert j["schema"] == "crane.fleet_asset_validation/v0" and j["overall"] == "PASS"


def test_report_cli_writes_json_and_summary(gen_dir, tmp_path, capsys):
    out = str(tmp_path / "r.json")
    rc = V.main([path(gen_dir, "cargo"), "--no-mate", "--report", out])
    assert rc == 0 and json.load(open(out))["overall"] == "PASS"
    assert "overall PASS" in capsys.readouterr().out


# ------------------------------------------------------------ deliberately broken manifests
def test_unit_error_mm_mesh_is_caught(work):
    """A CAD export in millimetres loaded as metres (mesh 1000x too big)."""
    p = os.path.join(work, "cargo", "chassis_body.obj")
    mesh, _ = g.read_obj(p)
    mesh.P *= 1000.0
    g.write_obj(p, [("chassis_body", mesh)], "cargo.mtl")
    rep = run(work, "cargo")
    assert "FAIL" in level(rep, "scale")
    assert rep.overall() == "FAIL"


def test_unit_error_mm_values_in_manifest_is_caught(work):
    m = load(work, "cargo")
    for w in m["wheels"]["items"]:
        w["radius"] *= 1000
        w["width"] *= 1000
        w["position"] = [v * 1000 for v in w["position"]]
    save(work, "cargo", m)
    rep = run(work, "cargo")
    assert "FAIL" in level(rep, "profile.wheels") and "FAIL" in level(rep, "scale.wheel_radius")


def test_declared_unit_mismatch_is_caught(work):
    m = load(work, "cargo")
    m["units"]["length"] = "mm"
    save(work, "cargo", m)
    assert "FAIL" in level(run(work, "cargo"), "schema")


def test_mirrored_wheel_sign_in_manifest_is_caught(work):
    m = load(work, "scout")
    m["wheels"]["items"][0]["roller_angle_sign"] *= -1
    save(work, "scout", m)
    rep = run(work, "scout")
    assert "FAIL" in level(rep, "roller_handedness")
    assert "FL" in [c for c in rep.checks if c["id"] == "roller_handedness"][0]["message"]


def test_mirrored_wheel_mesh_is_caught(work):
    p = os.path.join(work, "cargo", "wheel_FR.obj")
    mesh, _ = g.read_obj(p)
    mir = mesh.transformed(np.diag([1, -1, 1, 1.0]))    # mirrors the roller lean, winding kept outward
    g.write_obj(p, [("wheel_FR", mir)], "cargo.mtl")
    rep = run(work, "cargo")
    c = [c for c in rep.checks if c["id"] == "roller_handedness"][0]
    assert c["level"] == "FAIL" and "FR" in c["message"]


def test_shifted_face_frame_is_caught(work):
    m = load(work, "cargo")
    f = [f for f in m["attachments"]["docking_faces"] if f["name"] == "front"][0]
    f["origin"]["xyz"][0] += 0.02
    save(work, "cargo", m)
    rep = run(work, "cargo", mate=True)
    assert "FAIL" in level(rep, "profile.faces")
    assert "FAIL" in level(rep, "mating.front_rear.gap") and "FAIL" in level(rep, "mating.front_rear.hook_engagement")


def test_face_frame_raised_is_caught(work):
    m = load(work, "cargo")
    f = [f for f in m["attachments"]["docking_faces"] if f["name"] == "rear"][0]
    f["origin"]["xyz"][2] += 0.01
    save(work, "cargo", m)
    rep = run(work, "cargo", mate=True)
    assert "FAIL" in level(rep, "profile.faces")


def test_wrong_inertia_is_caught(work):
    m = load(work, "cargo")
    I = np.array(m["mass_properties"]["inertia_tensor"])
    I[2, 2] *= 60
    m["mass_properties"]["inertia_tensor"] = I.tolist()
    save(work, "cargo", m)
    assert "FAIL" in level(run(work, "cargo"), "mass.box_consistency")


def test_unphysical_inertia_is_caught(work):
    m = load(work, "cargo")
    m["mass_properties"]["inertia_tensor"] = [[1.0, 0, 0], [0, 0.01, 0], [0, 0, 0.01]]
    save(work, "cargo", m)
    assert "FAIL" in level(run(work, "cargo"), "mass.physical")
    m["mass_properties"]["inertia_tensor"] = [[0.1, 0, 0], [0, -0.1, 0], [0, 0, 0.1]]
    save(work, "cargo", m)
    assert "FAIL" in level(run(work, "cargo"), "mass.physical")


def test_wheels_floating_is_caught(work):
    m = load(work, "cargo")
    m["wheels"]["items"][1]["position"][2] += 0.006
    save(work, "cargo", m)
    rep = run(work, "cargo")
    assert "FAIL" in level(rep, "wheels_ground")


def test_missing_file_is_caught(work):
    os.remove(os.path.join(work, "cargo", "deck.obj"))
    assert "FAIL" in level(run(work, "cargo"), "files")


def test_missing_source_file_is_caught(work):
    m = load(work, "cargo")
    m["mass_properties"]["source_file"] = "cad/does_not_exist.step"
    save(work, "cargo", m)
    assert "FAIL" in level(run(work, "cargo"), "files.source")


def test_collision_bbox_mismatch_is_flagged(work):
    m = load(work, "cargo")
    for s in m["collision"]["shapes"]:
        if s["name"] == "chassis_col":
            s["size"] = [0.30, 0.30, 0.10]
    save(work, "cargo", m)
    assert "WARN" in level(run(work, "cargo"), "bbox")


def test_non_watertight_is_flagged(work):
    p = os.path.join(work, "cargo", "hook_front_a.obj")
    mesh, _ = g.read_obj(p)
    mesh.P, mesh.UV, mesh.M, mesh.G, mesh.S, mesh.N = (mesh.P[:-3], mesh.UV[:-3], mesh.M[:-3], mesh.G[:-3], mesh.S[:-3], mesh.N[:-3])
    g.write_obj(p, [("hook_front_a", mesh)], "cargo.mtl")
    assert "WARN" in level(run(work, "cargo"), "watertight")


def test_inside_out_mesh_is_caught(work):
    p = os.path.join(work, "cargo", "deck.obj")
    mesh, _ = g.read_obj(p)
    mesh.P = mesh.P[:, [0, 2, 1]]
    g.write_obj(p, [("deck", mesh)], "cargo.mtl")
    assert "FAIL" in level(run(work, "cargo"), "winding")


def test_hook_pin_mismatch_is_caught_by_mating(work):
    m = load(work, "cargo")
    for h in m["actuated"]["hooks"]:
        h["engage_point"][1] += 0.006
    save(work, "cargo", m)
    rep = run(work, "cargo", mate=True)
    assert "FAIL" in level(rep, "mating.front_rear.hook_engagement")


def test_body_overlap_detected(work):
    """Face frames pushed 5 mm outward from the body make the rovers interpenetrate when mated."""
    m = load(work, "cargo")
    for f in m["attachments"]["docking_faces"]:
        if f["name"] == "front":
            f["origin"]["xyz"][0] -= 0.01
    save(work, "cargo", m)
    rep = run(work, "cargo", mate=True)
    assert "FAIL" in level(rep, "mating.front_rear")


def test_asymmetric_wheels_are_caught(work):
    m = load(work, "cargo")
    m["wheels"]["items"][2]["position"][1] = 0.19
    save(work, "cargo", m)
    rep = run(work, "cargo")
    assert "FAIL" in level(rep, "symmetry")


def test_invalid_json_report_is_graceful(work):
    open(path(work, "cargo"), "w").write("{ not json")
    rep = run(work, "cargo")
    assert rep.overall() == "FAIL"


def test_mate_cli_cross_role(gen_dir):
    rep = V.validate(path(gen_dir, "scout"), None, None, [f"{path(gen_dir,'scout')}:front,{path(gen_dir,'cargo')}:rear"], True)
    assert not [c for c in rep.checks if c["level"] == "FAIL"]
    assert len(rep.mating) == 1 and rep.mating[0]["B"] == "cargo_provisional_v0"
