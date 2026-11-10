"""Robot-configuration space (EXPLORATORY)."""
import math
import os

import numpy as np
import pytest

import geometry as G
import robots as R
from families import cspace as CS, robotspace as RS


def _area(poly):
    p = np.array(poly)
    return 0.5 * abs(np.dot(p[:, 0], np.roll(p[:, 1], -1)) - np.dot(p[:, 1], np.roll(p[:, 0], -1)))


def test_limits_rule_reproduces_both_simulated_morphologies():
    c = RS.make_config("2x2", 0.40, 0.36)
    assert (c.L, c.W, c.v_max, c.omega_max, c.acc) == (0.80, 0.72, 0.9, 1.6, 1.2)
    t = RS.make_config("line4", 0.44, 0.66)
    assert (t.L, t.W) == (1.76, 0.66)
    assert (t.v_max, t.omega_max, t.acc) == pytest.approx((0.65, 0.75, 0.55), abs=1e-3)


@pytest.mark.skipif(not os.path.exists(R.MORPHOLOGIES_PATH), reason="modular checkout not present")
def test_rule_anchor_values_are_still_in_the_yaml():
    text = open(R.MORPHOLOGIES_PATH, encoding="utf-8").read()
    assert "limits: {linear: 0.9, angular: 1.6, acceleration: 1.2}" in text
    assert "limits: {linear: 0.65, angular: 0.75, acceleration: 0.55}" in text


@pytest.mark.parametrize("arr", [a for a in RS.ARRANGEMENTS])
def test_polygon_parts_and_rotation_centre_agree(arr):
    c = RS.make_config(arr, 0.45, 0.35)
    poly, parts = c.polygon(), c.parts()
    n_mod = len(RS.ARRANGEMENTS[arr])
    assert _area(poly) == pytest.approx(n_mod * 0.45 * 0.35) == pytest.approx(sum(p[2] * p[3] for p in parts))
    assert len(poly) == (6 if arr.startswith("L") else 4) and c.is_rect == (not arr.startswith("L"))
    # the rotation centre (origin of the body frame) is the area centroid
    cx = sum(p[0] * p[2] * p[3] for p in parts) / c.area
    cy = sum(p[1] * p[2] * p[3] for p in parts) / c.area
    assert abs(cx) < 1e-12 and abs(cy) < 1e-12
    # every polygon vertex is a corner of some part, and r_circ is the farthest vertex
    assert c.r_circ == pytest.approx(max(math.hypot(*v) for v in poly))
    lo_x, hi_x, lo_y, hi_y = (float(v[0]) for v in CS.extents(0.0, parts))
    assert (hi_x - lo_x, hi_y - lo_y) == pytest.approx((c.L, c.W))


def test_rotation_centre_shift_matches_the_axle_offset_convention():
    c = RS.make_config("line2", 0.45, 0.5, rc_shift=-0.15)        # rotation centre 0.15 behind the centroid
    assert c.axle_offset == pytest.approx(0.15)
    cor = G.rect_corners(0.3, -0.2, 0.7, c.L, c.W, c.axle_offset).reshape(4, 2)
    p = c.parts()[0]
    cor2 = G.rect_corners(*CS.part_centres(0.3, -0.2, 0.7, p), 0.7, p[2], p[3]).reshape(4, 2)
    assert np.allclose(cor, cor2)


def test_sampler_is_seeded_ids_are_stable_and_values_stay_in_the_box():
    a, b, c = RS.sample_robots(60, 3), RS.sample_robots(60, 3), RS.sample_robots(60, 4)
    assert [r.config_id for r in a] == [r.config_id for r in b] != [r.config_id for r in c]
    assert len({r.config_id for r in a}) == 60
    box = RS.ROBOT_BOX_V0
    assert {r.drive for r in a} == set(box["drive"]) and "carlike" not in {r.drive for r in a}
    assert len({r.arrangement for r in a}) >= 7
    for r in a:
        assert box["unit_l_mm"][0] <= r.unit_l_mm <= box["unit_l_mm"][1]
        assert 0 <= r.cmd_delay_ms <= 300 and 0 <= r.obs_delay_ms <= 300
        assert r.lidar_fov_deg in box["lidar_fov_deg"] and r.lidar_range_mm in box["lidar_range_mm"]
        assert r.sensor["angular_resolution"] == pytest.approx(math.radians(r.lidar_fov_deg) / r.lidar_n_beams)
        assert (r.vy_max > 0) == (r.drive == "mecanum")
        if r.drive == "mecanum":
            assert r.rc_shift_mm == 0
        # the lidar sits on the body axis inside the bounding box
        mx, my = r.sensor["mount"]
        lo_x, hi_x, lo_y, hi_y = (float(v[0]) for v in CS.extents(0.0, r.parts()))
        assert lo_x - 1e-9 <= mx <= hi_x + 1e-9 and lo_y - 1e-9 <= my <= hi_y + 1e-9
        assert r.stopping_distance == pytest.approx(r.v_max * (r.cmd_delay + r.obs_delay) + r.v_max ** 2 / (2 * r.acc))
    # the id ignores the name and nothing else
    d = dict(a[0].__dict__)
    assert RS.RobotConfig(**{**d, "name": "x"}).config_id == a[0].config_id
    assert RS.RobotConfig(**{**d, "cmd_delay_ms": d["cmd_delay_ms"] + 1}).config_id != a[0].config_id
    assert len(RS.box_hash()) == 64


def test_bridge_to_the_existing_robot_profile_and_declared_gaps():
    prof = R.load_robot("compact_diff")
    cfg = RS.from_profile(prof)
    back = cfg.to_robot()
    assert (back.L, back.W, back.v_max, back.omega_max, back.acc, back.drive) == (prof.L, prof.W, prof.v_max, prof.omega_max, prof.acc, prof.drive)
    assert back.lidar == prof.lidar and cfg.surrogate_gaps() == []
    odd = RS.make_config("L3", 0.4, 0.3, rc_shift=0.05, cmd_delay_ms=100, lidar_fov_deg=180, lidar_n_beams=360)
    gaps = " ".join(odd.surrogate_gaps())
    for word in ("composite", "rotation-centre", "delay", "field of view"):
        assert word in gaps
    car = RS.make_config("line2", 0.4, 0.3, drive="carlike", min_turn_radius_mm=520)
    with pytest.raises(ValueError):
        car.to_robot()
