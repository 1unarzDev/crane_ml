"""Interaction features: family-agnostic, dimensionless, and equal to the known mechanics where a
closed form exists (EXPLORATORY)."""
import math

import numpy as np
import pytest

import mechanics as M
import robots as R
from families import envs as E, features as F, robotspace as RS

COMPACT = RS.from_profile(R.load_robot("compact_diff"))
TANDEM = RS.from_profile(R.load_robot("narrow_tandem"))


class Bare:
    """A course with no family and no parameters: only what the features are allowed to read."""

    def __init__(self, co):
        self._b, self.start, self.goal, self.course_id = co.boxes(), co.start, co.goal, "bare"

    def boxes(self):
        return self._b


def test_table_and_output_agree_and_every_row_has_definition_units_status():
    assert len(F.FEATURE_NAMES) == len(set(F.FEATURE_NAMES)) >= 30
    assert all(len(r) == 4 and all(r) for r in F.FEATURE_TABLE)
    f = F.interaction_features(E.make_lbend(1.2, 1.2), COMPACT)
    assert list(f) == F.FEATURE_NAMES
    assert F.feature_table_markdown().count("\n") == len(F.FEATURE_NAMES) + 1


def test_features_do_not_read_the_family_or_its_parameters():
    for co in (E.make_chicane(1.0, 0.0, 1.0, 0.8, 0.95), E.make_dead_end(0.95, 3.0, 2.0, 1.0)):
        assert F.interaction_features(Bare(co), COMPACT) == F.interaction_features(co, COMPACT)


def test_all_families_and_sampled_robots_give_finite_bounded_features():
    robots = RS.sample_robots(2, 9)
    for name, fam in E.FAMILIES.items():
        co = fam.sample(1, 21)[0]
        for rob in robots:
            v = F.feature_vector(co, rob)
            assert v.shape == (len(F.FEATURE_NAMES),) and np.isfinite(v).all() and (v >= 0).all() and (v <= F.CAP).all()


@pytest.mark.parametrize("rob,w,e,d", [(COMPACT, 0.94, 0.72, 0.90), (COMPACT, 0.94, 0.72, 1.00), (COMPACT, 0.80, 1.00, 1.05),
                                       (TANDEM, 0.90, 0.70, 1.84), (TANDEM, 1.00, 1.20, 1.80)])
def test_offset_doorway_tight_bay_reproduces_the_strip_mechanics(rob, w, e, d):
    co = E.make_offset_doorway(w, e, d)
    f, meta = F.interaction_features(co, rob, return_meta=True)
    assert f["slot_depth_over_L"] == pytest.approx(d / rob.L, abs=1e-6)                 # strip depth found from the boxes
    assert f["slot_depth_over_2rho"] < 1
    # in-place range at the start pose: 2 alpha_max at the exact bay centre, smaller off centre (odd d in cm)
    full = 2 * M.alpha_max(d, rob.L, rob.W) / (2 * math.pi)
    assert f["rot_range_start"] <= full + 1e-4
    if co.p["d_cm"] % 2 == 0:
        assert f["rot_range_start"] == pytest.approx(full, abs=1e-4)
    e_low = max(e - (w - rob.W) / 2, 0.0)                                                # the door slack may be used
    assert e_low / rob.W - 0.05 <= f["crab_shift_over_W"] <= e / rob.W + 0.05
    n = math.expm1(f["shimmy_stretches_log"])
    lo, hi = (M.min_stretches_exact(x, d, rob.L, rob.W, "centre") for x in (e_low, e))
    assert lo - 1 <= n <= hi + 1 and n > 10
    assert f["passage_over_W"] == pytest.approx(min(w, d) / rob.W, abs=0.03)             # the bay itself can be the neck


def test_offset_doorway_roomy_bay_needs_no_shimmy_and_passage_is_the_door():
    co = E.make_offset_doorway(0.94, 0.72, 1.30)
    f, meta = F.interaction_features(co, COMPACT, return_meta=True)
    assert meta["passage_exact"] and f["passage_over_W"] == pytest.approx(0.94 / 0.72, abs=1e-6)
    assert math.expm1(f["shimmy_stretches_log"]) == pytest.approx(1.0) and f["rot_range_start"] == 1.0
    assert f["strip_blocked"] == 0 and f["slot_depth_over_L"] == F.CAP
    mec = RS.make_config("rect", 0.80, 0.72, drive="mecanum")
    tight = E.make_offset_doorway(0.94, 0.72, 0.90)
    fm = F.interaction_features(tight, mec)
    assert math.expm1(fm["predicted_stretches_log"]) == pytest.approx(1.0)
    assert math.expm1(fm["shimmy_stretches_log"]) > 50
    assert fm["holonomic"] == 1.0


def test_cluttered_passage_feature_recovers_the_minimum_gap():
    for lay in (0, 1, 2):
        f = F.interaction_features(E.make_cluttered(0.9, 3, 2.0, layout=lay), COMPACT)
        assert f["passage_over_W"] == pytest.approx(0.9 / COMPACT.W, abs=1e-6)


def test_visibility_features_separate_the_occluded_doorway_from_the_plain_one():
    occ = E.FAMILIES["occluded_doorway"].sample(1, 5)[0]
    plain = E.make_offset_doorway(occ.w, occ.e, occ.d, B=occ.B, t=occ.t, start=occ.start)
    f_o, f_p = F.interaction_features(occ, COMPACT), F.interaction_features(plain, COMPACT)
    assert f_p["passage_visible_at_start"] == 1.0 and f_p["passage_first_obs_arc"] == 0.0
    assert f_o["passage_visible_at_start"] == 0.0 and f_o["passage_first_obs_arc"] > 0.05
    assert f_o["route_visible_fraction"] < f_p["route_visible_fraction"]
    assert f_o["passage_over_W"] == pytest.approx(f_p["passage_over_W"], abs=1e-6)       # geometric fit unchanged
    # a short-range or narrow sensor sees less; the geometry features do not move
    blind = RS.RobotConfig(**{**COMPACT.__dict__, "lidar_range_mm": 1500, "lidar_fov_deg": 120, "lidar_n_beams": 240})
    f_b = F.interaction_features(plain, blind)
    assert f_b["passage_visible_at_start"] == 0.0 and f_b["route_visible_fraction"] < f_p["route_visible_fraction"]
    assert f_b["passage_over_W"] == f_p["passage_over_W"] and f_b["clear_min_over_W"] == f_p["clear_min_over_W"]


def test_dead_end_reverse_feature_follows_turnaround_room():
    narrow = F.interaction_features(E.make_dead_end(0.95, 3.0, 2.0, 1.0), COMPACT)     # Bw < 2 rho = 1.076
    wide = F.interaction_features(E.make_dead_end(1.30, 3.0, 2.0, 1.0), COMPACT)
    assert narrow["reverse_fraction"] > 0.5 and narrow["reverse_len_over_L"] > 2.0
    assert wide["reverse_fraction"] == 0.0
    assert narrow["rot_range_start"] == pytest.approx(2 * E.dead_end_alpha(0.95, 0.80, 0.72) / (2 * math.pi), abs=1e-4)


def test_corner_clearance_sign_follows_the_exact_bend_statement_away_from_threshold():
    no, yes = (F.interaction_features(E.make_lbend(a, a), TANDEM) for a in (1.00, 1.20))
    assert no["corner_clear_over_W"] == 0.0 and no["strip_blocked"] == 1.0
    assert yes["corner_clear_over_W"] > 0.03 and yes["strip_blocked"] == 0.0
    assert yes["turn_total"] == pytest.approx(0.5, abs=0.05)


def test_latency_features_scale_with_delay_only():
    co = E.make_lbend(1.2, 1.2)
    slow = RS.RobotConfig(**{**COMPACT.__dict__, "cmd_delay_ms": 200, "obs_delay_ms": 100})
    f0, f1 = F.interaction_features(co, COMPACT), F.interaction_features(co, slow)
    assert f0["latency_travel_over_clear_min"] == 0.0 < f1["latency_travel_over_clear_min"]
    assert f1["stop_over_clear_med"] / f0["stop_over_clear_med"] == pytest.approx(slow.stopping_distance / COMPACT.stopping_distance)
    assert f1["clear_med_over_W"] == f0["clear_med_over_W"]


def test_known_weak_spot_chamfered_bend_is_flagged_blocked_by_the_strip_heuristic():
    """Documented failure: the bend is passable (exact statement, swept path) yet the crab-segment
    strip model reports no solution. Kept as a test so the README claim stays true or gets fixed."""
    co = E.make_lbend(1.04, 1.04, 0.4)
    assert E.FAMILIES["lbend"].feasibility(co, TANDEM)["verdict"] == "fits"
    assert F.interaction_features(co, TANDEM)["strip_blocked"] == 1.0


def test_raster_is_fixed_size_in_the_robot_frame():
    co = E.make_offset_doorway(0.94, 0.72, 0.95)
    for rob in (COMPACT, RS.make_config("L3", 0.4, 0.3)):
        r = F.raster(co, rob, n=48, cell=0.05)
        assert r["occupancy"].shape == r["footprint"].shape == (48, 48) and r["occupancy"].dtype == np.uint8
        assert r["footprint"].sum() * 0.05 ** 2 == pytest.approx(rob.area, rel=0.08)
        assert not (r["occupancy"] & r["footprint"]).any()                              # start pose is free
        assert r["goal_body"] == pytest.approx((co.goal[0] - co.start[0], co.goal[1] - co.start[1]))
    # door wall ahead of the robot: occupied cells in the forward half only; back wall behind
    r = F.raster(co, COMPACT, n=48, cell=0.05)
    fwd = r["occupancy"][24:, :].sum(), r["occupancy"][:24, :].sum()
    assert fwd[0] > 0 and fwd[1] > 0
    turned = F.raster(co, COMPACT, pose=(co.start[0], co.start[1], math.pi / 2), n=48, cell=0.05)
    assert turned["goal_body"] == pytest.approx((co.goal[1] - co.start[1], -(co.goal[0] - co.start[0])))
