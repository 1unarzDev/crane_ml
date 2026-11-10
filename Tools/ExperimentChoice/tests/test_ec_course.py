"""Course generator, manifest and exporters (EXPLORATORY)."""
import json
import math
import os
import xml.etree.ElementTree as ET

import numpy as np
import pytest

import course as C
import geometry as G
import mechanics as M
import robots as R

COMPACT = R.load_robot("compact_diff")
TANDEM = R.load_robot("narrow_tandem")


def test_profiles_match_sources():
    assert (COMPACT.L, COMPACT.W, COMPACT.v_max, COMPACT.omega_max, COMPACT.acc) == (0.80, 0.72, 0.9, 1.6, 1.2)
    assert (TANDEM.L, TANDEM.W, TANDEM.v_max, TANDEM.omega_max, TANDEM.acc) == (1.76, 0.66, 0.65, 0.75, 0.55)
    assert COMPACT.provenance["repo_head"] == R.MORPHOLOGIES_REPO_HEAD and len(R.MORPHOLOGIES_REPO_HEAD) == 40


@pytest.mark.skipif(not os.path.exists(R.MORPHOLOGIES_PATH), reason="modular checkout not present")
def test_transcribed_profiles_still_agree_with_the_yaml_text():
    text = open(R.MORPHOLOGIES_PATH, encoding="utf-8").read()
    assert "footprint: [[-0.40, -0.36], [0.40, -0.36], [0.40, 0.36], [-0.40, 0.36]]" in text
    assert "limits: {linear: 0.9, angular: 1.6, acceleration: 1.2}" in text
    assert "footprint: [[-0.88, -0.33], [0.88, -0.33], [0.88, 0.33], [-0.88, 0.33]]" in text
    assert "limits: {linear: 0.65, angular: 0.75, acceleration: 0.55}" in text


@pytest.mark.skipif(not os.path.exists(R.FLEET_PROFILE_PATH), reason="fleet profile not present")
def test_mecanum_is_read_at_runtime_and_hashed():
    mec = R.load_robot("mecanum")
    raw = json.load(open(R.FLEET_PROFILE_PATH, encoding="utf-8"))
    assert mec.drive == "mecanum" and mec.L == raw["chassis"]["body_size_lwh"]["value"][0]
    assert mec.W == max(raw["chassis"]["body_size_lwh"]["value"][1], raw["chassis"]["overall_width_with_wheels"]["value"])
    assert mec.vy_max == mec.v_max == raw["chassis"]["max_speed_xy"]["value"]
    assert len(mec.provenance["file_sha256"]) == 64 and "provisional" in mec.provenance["value_status"]
    man = C.manifest(C.make_course(0.6, 0.3, 0.6), mec)
    assert man["sources"]["fleet_profile_sha256"] == mec.provenance["file_sha256"]


def test_mecanum_from_a_fixture_file(tmp_path):
    prof = {"profile_id": "x", "schema": "s", "chassis": {
        "body_size_lwh": {"value": [0.5, 0.3, 0.1]}, "overall_width_with_wheels": {"value": 0.44},
        "max_speed_xy": {"value": 0.4}, "max_yaw_rate": {"value": 1.0}},
        "sensors": {"lidar_2d": {"max_range": {"value": 8.0}, "points_per_scan": {"value": 360}, "rate_hz": {"value": 10}}}}
    p = tmp_path / "fleet.json"
    p.write_text(json.dumps(prof))
    mec = R.load_mecanum(str(p))
    assert (mec.L, mec.W, mec.v_max, mec.lidar["n_beams"]) == (0.5, 0.44, 0.4, 360)


def test_course_id_is_stable_and_centimetre_resolved():
    a = C.make_course(0.94, 0.72, 1.02)
    b = C.make_course(0.9400001, 0.7199, 1.0204)
    assert a == b and a.course_id == b.course_id and len(a.course_id) == 16
    assert C.make_course(0.95, 0.72, 1.02).course_id != a.course_id
    assert a.start == (0.51, 0.0, 0.0) and a.goal == (pytest.approx(1.02 + 0.2 + 1.2), 0.72)
    with pytest.raises(ValueError):
        C.make_course(1.0, 2.3, 1.0, B=5.0)


def test_course_id_pinned():
    # canonical JSON hash; changes only if the parameter schema changes
    import hashlib
    c = C.make_course(0.94, 0.72, 1.02)
    canon = json.dumps(c.params(), sort_keys=True, separators=(",", ":"))
    assert c.course_id == hashlib.sha256(canon.encode()).hexdigest()[:16]
    assert json.loads(canon)["d_cm"] == 102


def test_sampler_is_seeded_and_inside_the_frozen_box():
    a, b = C.sample_courses(40, seed=7), C.sample_courses(40, seed=7)
    assert [c.course_id for c in a] == [c.course_id for c in b]
    assert [c.course_id for c in a] != [c.course_id for c in C.sample_courses(40, seed=8)]
    box = C.ADMISSIBLE_BOX_V0
    for c in a:
        for k in ("w_cm", "e_cm", "d_cm", "t_cm", "B_cm"):
            assert box[k][0] <= getattr(c, k) <= box[k][1]
        assert c.e_cm + c.w_cm / 2 <= c.B_cm / 2 - box["side_clearance_cm"]
    assert len(C.box_hash()) == 64


def test_walls_enclose_start_and_leave_the_door_open():
    c = C.make_course(0.94, 0.72, 1.02)
    bx = c.boxes()
    assert bx.shape == (6, 4)
    sx, sy, sth = c.start
    assert not G.rect_hits_boxes(sx, sy, sth, COMPACT.L, COMPACT.W, bx).any()
    # a ray from the bay through the door centre reaches the end wall; one beside the door hits the door wall
    org = np.array([[c.d - 0.05, c.e]])
    assert G.raycast(org, np.array([0.0]), bx, 12.0)[0, 0] == pytest.approx(0.05 + c.t + C.EXIT_DEPTH)
    org2 = np.array([[c.d - 0.05, c.e + c.w / 2 + 0.05]])
    assert G.raycast(org2, np.array([0.0]), bx, 12.0)[0, 0] == pytest.approx(0.05)
    # closed room: every ray from the start hits something within range
    r = G.raycast(np.array([[sx, sy]]), np.linspace(0, 2 * math.pi, 720, endpoint=False), bx, 12.0)
    assert r.max() < 12.0
    # goal is free for the footprint
    assert not G.rect_hits_boxes(c.goal[0], c.goal[1], 0.0, COMPACT.L, COMPACT.W, bx).any()


def test_occupancy_grid_agrees_with_boxes():
    c = C.make_course(0.94, 0.72, 1.02, t=0.2)
    occ, origin = C.to_occupancy_grid(c)
    bx = c.boxes()
    rng = np.random.default_rng(1)
    pts = np.column_stack([rng.uniform(-0.4, c.d + c.t + 2.9, 4000), rng.uniform(-2.9, 2.9, 4000)])
    inside = ((pts[:, None, 0] > bx[None, :, 0]) & (pts[:, None, 0] < bx[None, :, 1])
              & (pts[:, None, 1] > bx[None, :, 2]) & (pts[:, None, 1] < bx[None, :, 3])).any(1)
    ix = np.floor((pts[:, 0] - origin[0]) / C.GRID_RES).astype(int)
    iy = np.floor((pts[:, 1] - origin[1]) / C.GRID_RES).astype(int)
    cell = occ[iy, ix]
    assert np.all(cell[inside])                              # conservative: every wall point is occupied
    near = np.array([np.min(np.maximum(np.maximum(bx[:, 0] - p[0], p[0] - bx[:, 1]),
                                       np.maximum(bx[:, 2] - p[1], p[1] - bx[:, 3]))) for p in pts])
    assert not np.any(cell[near > C.GRID_RES * 1.5])         # and nothing far from a wall is
    # door opening is free in the grid
    jx = int((c.d + c.t / 2 - origin[0]) / C.GRID_RES)
    jy = int((c.e - origin[1]) / C.GRID_RES)
    assert not occ[jy, jx]


def test_pgm_yaml_and_sdf_are_well_formed(tmp_path):
    c = C.make_course(0.94, 0.72, 1.02)
    occ, origin = C.to_occupancy_grid(c)
    pgm = C.pgm_bytes(occ)
    head = b"P5\n%d %d\n255\n" % (occ.shape[1], occ.shape[0])
    assert pgm.startswith(head) and len(pgm) == len(head) + occ.size
    img = np.frombuffer(pgm[len(head):], np.uint8).reshape(occ.shape)
    assert set(np.unique(img)) == {0, 254} and np.array_equal(img[::-1] == 0, occ)
    y = C.map_yaml("m.pgm", origin)
    assert "resolution: 0.025" in y and "origin: [%.3f, %.3f, 0.0]" % origin in y
    root = ET.fromstring(C.to_sdf(c))
    models = root.find("world").findall("model")
    assert len(models) == 1 + len(c.walls())
    sizes = {m.get("name"): m.find("link/collision/geometry/box/size") for m in models if m.get("name") != "ground_plane"}
    dw = next(b for b in c.walls() if b["name"] == "door_wall_pos")
    sx, sy, sz = map(float, sizes["door_wall_pos"].text.split())
    assert sx == pytest.approx(c.t) and sy == pytest.approx(dw["ymax"] - dw["ymin"]) and sz == C.WALL_HEIGHT
    paths = C.write_course(str(tmp_path), c, COMPACT)
    man = json.load(open(paths["manifest"]))
    assert man["label"] == "exploratory" and man["course_id"] == c.course_id
    assert all(os.path.getsize(p) > 0 for p in paths.values())


def test_manifest_content_and_certificates():
    L, W = COMPACT.L, COMPACT.W
    two_rho = 2 * M.rho(L, W)
    roomy = C.make_course(0.94, 0.72, round(two_rho + 0.05, 2))
    tight = C.make_course(0.94, 0.72, 1.00)
    blocked = C.make_course(0.94, 0.72, 0.79)
    narrow = C.make_course(0.70, 0.72, 1.20)
    man = C.manifest(tight, COMPACT)
    json.dumps(man)
    for key in ("course_id", "walls", "start", "goal", "parameters", "certificate", "sources", "label"):
        assert key in man
    assert man["sources"]["morphologies_repo_head"] == R.MORPHOLOGIES_REPO_HEAD
    assert "fleet_profile_sha256" in man["sources"]
    cert = {n: M.certificate(c, COMPACT) for n, c in dict(roomy=roomy, tight=tight, blocked=blocked, narrow=narrow).items()}
    assert cert["roomy"]["verdict"] == "fits" and cert["roomy"]["min_reversals"] == 0
    assert cert["tight"]["verdict"] == "fits" and cert["tight"]["min_reversals"] >= 1
    assert cert["tight"]["four_flow_cycles"] == M.four_flow_cycles(0.72, 1.00, L, W)
    assert cert["blocked"]["verdict"] == "does_not_fit" and "bay depth" in cert["blocked"]["reason"]
    assert cert["narrow"]["verdict"] == "does_not_fit" and "door width" in cert["narrow"]["reason"]
    by = man["certificates_by_drive"]
    assert by["differential"]["verdict"] == "fits" and by["mecanum"]["verdict"] == "fits"
    assert "sideways" in by["mecanum"]["reason"]
    assert M.certificate(C.make_course(0.94, 0.72, 1.00), COMPACT, margin=0.11)["verdict"] == "undetermined"
    assert M.certificate(C.make_course(0.94, 0.72, 1.00, start=(0.5, 0.1, 0.0)), COMPACT)["verdict"] == "undetermined"


def test_features_are_dimensionless_and_consistent():
    c = C.course_from_ratios(COMPACT, 0.25, 1.0)
    f = M.features(c, COMPACT)
    assert f["label"] == "exploratory" and f["regime"] == "tight"
    assert f["eps_over_W"] == pytest.approx((c.d - COMPACT.L) / COMPACT.W)
    assert f["w_over_W"] == pytest.approx(c.w / COMPACT.W) and f["e_over_W"] == pytest.approx(1.0)
    assert f["d_over_2rho"] < 1
    assert f["predicted_cycles"] == M.four_flow_cycles(c.e, c.d, COMPACT.L, COMPACT.W) > 1
    assert f["predicted_cycles_first_order"] > f["predicted_cycles"] - 1     # first-order form overstates
    assert f["heading_tolerance"] > f["heading_tolerance_small_angle"]
    assert f["braking_over_d"] == pytest.approx(0.9 ** 2 / 2.4 / c.d)
    assert f["certificate"]["verdict"] == "fits"
    wide = M.features(C.course_from_ratios(COMPACT, 0.6, 1.0), COMPACT)
    assert wide["regime"] == "roomy" and wide["predicted_cycles"] == 0 and wide["d_over_2rho"] > 1
    assert isinstance(f["door_far_side_visible"], bool) and f["visibility"] >= 0
    # the tandem footprint in the same bay does not fit at all
    assert M.features(c, TANDEM)["certificate"]["verdict"] == "does_not_fit"
