"""Environment families: interface, samplers, exporters, and each analytic feasibility statement
against a brute-force check (EXPLORATORY). Lattice runs are kept few and coarse (shared host)."""
import json
import math
import xml.etree.ElementTree as ET

import numpy as np
import pytest

import course as C
import geometry as G
import mechanics as M
import robots as R
from families import cspace as CS, envs as E, robotspace as RS

COMPACT = RS.from_profile(R.load_robot("compact_diff"))
TANDEM = RS.from_profile(R.load_robot("narrow_tandem"))
L3 = RS.make_config("L3", 0.5, 0.4)


# --------------------------------------------------------------------------- interface

def test_at_least_five_families_with_binding_statement_and_status():
    assert len(E.FAMILIES) >= 5
    for fam in E.FAMILIES.values():
        assert fam.binding and fam.statement and fam.status and len(fam.box_hash()) == 64


@pytest.mark.parametrize("name", list(E.FAMILIES))
def test_sampler_is_seeded_inside_the_box_and_ids_are_stable(name):
    fam = E.FAMILIES[name]
    a, b, c = fam.sample(6, 11), fam.sample(6, 11), fam.sample(6, 12)
    assert [x.course_id for x in a] == [x.course_id for x in b] != [x.course_id for x in c]
    for co in a:
        for k in fam.keys():
            assert fam.box[k][0] <= co.p[k] <= fam.box[k][1]
        assert co.family == name and len(co.course_id) == 16
        assert fam.build(co.p).course_id == co.course_id          # parameters -> same course
        gx, gy = co.goal                                           # goal point is in free space
        bx = co.boxes()
        assert not ((bx[:, 0] < gx) & (gx < bx[:, 1]) & (bx[:, 2] < gy) & (gy < bx[:, 3])).any()


@pytest.mark.parametrize("name", list(E.FAMILIES))
def test_exporters_of_course_py_work_unchanged(name, tmp_path):
    co = E.FAMILIES[name].sample(1, 3)[0]
    occ, origin = C.to_occupancy_grid(co)
    assert occ.any() and not occ.all()
    assert C.pgm_bytes(occ).startswith(b"P5\n")
    root = ET.fromstring(C.to_sdf(co, world_name="w"))
    assert len(root.findall("./world/model")) == 1 + len(co.walls())
    paths = E.write_course(str(tmp_path), co, COMPACT)
    man = json.load(open(paths["manifest"], encoding="utf-8"))
    assert man["course_id"] == co.course_id and man["family"] == name and man["label"] == "exploratory"
    assert man["feasibility"]["verdict"] in ("fits", "does_not_fit", "undetermined")
    assert man["admissible_box"]["sha256"] == E.FAMILIES[name].box_hash()


# --------------------------------------------------------------------------- brute-force tools

def test_clearance_is_exact_against_dense_sampling():
    boxes = np.array([[1.0, 1.2, -1.0, 0.3], [0.2, 0.6, 0.9, 1.4]])
    parts = L3.parts()
    rng = np.random.default_rng(0)
    for x, y, th in rng.uniform([-0.5, -0.5, -3], [0.5, 0.5, 3], (6, 3)):
        d = CS.clearance(x, y, th, parts, boxes)[0]
        if d == 0:
            assert CS.hits(x, y, th, parts, boxes)[0]
            continue
        # push the footprint by slightly less / more than d in the best direction: free / colliding
        ang = np.linspace(0, 2 * math.pi, 1441)
        assert not CS.hits(x + 0.98 * d * np.cos(ang), y + 0.98 * d * np.sin(ang), th, parts, boxes).any()
        assert CS.hits(x + (1.02 * d + 1e-3) * np.cos(ang), y + (1.02 * d + 1e-3) * np.sin(ang), th, parts, boxes).any()


def test_composite_hits_respect_the_notch_of_an_l_shape():
    parts = L3.parts()
    lo_x, hi_x, lo_y, hi_y = (float(v[0]) for v in CS.extents(0.0, parts))
    box_in_notch = np.array([[hi_x - 0.2, hi_x - 0.05, hi_y - 0.2, hi_y - 0.05]])
    assert not CS.hits(0, 0, 0, parts, box_in_notch)[0]            # inside the bounding box, outside the L
    assert CS.hits(0, 0, 0, [(0.5 * (lo_x + hi_x), 0.5 * (lo_y + hi_y), hi_x - lo_x, hi_y - lo_y)], box_in_notch)[0]


# --------------------------------------------------------------------------- (a) offset doorway

def test_offset_doorway_wraps_the_existing_course_and_certificate():
    legacy = C.make_course(0.94, 0.72, 0.95)
    co = E.make_offset_doorway(0.94, 0.72, 0.95)
    assert co.course_id == legacy.course_id and co.walls() == legacy.walls()
    assert co.start == legacy.start and co.goal == legacy.goal and co.d == legacy.d
    rob = R.load_robot("compact_diff")
    f = E.FAMILIES["offset_doorway"].feasibility(co, rob)
    cert = M.certificate(legacy, rob)
    assert f["verdict"] == cert["verdict"] == "fits" and f["min_reversals"] == cert["min_reversals"]
    assert [c.course_id for c in E.FAMILIES["offset_doorway"].sample(5, 4)] == [c.course_id for c in C.sample_courses(5, 4)]


def test_offset_doorway_roomy_verdict_against_the_lattice():
    co = E.make_offset_doorway(0.94, 0.72, 1.3)
    assert E.FAMILIES["offset_doorway"].feasibility(co, COMPACT)["verdict"] == "fits"
    assert CS.reachable(co, COMPACT.parts())


# --------------------------------------------------------------------------- (b) chicane

def test_chicane_band_is_exact_and_a_straight_drive_along_it_is_free():
    co = E.make_chicane(1.2, 0.0, 1.2, 0.25, 0.6, t=0.05)
    bw, phi, c0 = E.chicane_band(co)
    f = E.FAMILIES["chicane"].feasibility(co, COMPACT)
    assert f["verdict"] == "fits" and "band" in f["reason"] and bw > COMPACT.W
    xs = np.linspace(2.5 - COMPACT.r_circ, 2.5 + 0.7 + COMPACT.r_circ, 3000)
    ys = (c0 + xs * math.sin(phi)) / math.cos(phi)
    assert CS.swept_free(xs, ys, phi, COMPACT.parts(), co.boxes())
    # a body 2 % wider than the band cannot be driven along it at ANY lateral offset or nearby angle
    fat = [(0.0, 0.0, 3.0, 1.02 * bw)]
    for dphi in np.linspace(-0.3, 0.3, 61):
        for dc in np.linspace(-0.15, 0.15, 31):
            y2 = (c0 + dc + xs * math.sin(phi + dphi)) / math.cos(phi + dphi)
            assert CS.hits(xs[1500], y2[1500], phi + dphi, fat, co.boxes())[0]


def test_chicane_roomy_tight_and_blocked_against_the_lattice():
    fam = E.FAMILIES["chicane"]
    roomy = E.make_chicane(1.0, 0.0, 1.0, 0.8, 1.6)
    tight = E.make_chicane(1.0, 0.0, 1.0, 0.8, 0.95)
    blocked = E.make_chicane(0.70, 0.0, 1.0, 0.5, 1.5)
    f_r, f_t, f_b = (fam.feasibility(c, COMPACT) for c in (roomy, tight, blocked))
    assert f_r["verdict"] == "fits" and f_r["stretches_strip"] == 1
    assert f_t["verdict"] == "fits" and f_t["stretches_strip"] > 5 and "tight" in f_t["reason"]
    assert f_t["stretches_strip"] == M.min_stretches_exact(0.8 - 0.28, 0.95, COMPACT.L, COMPACT.W, "rear")
    assert f_b["verdict"] == "does_not_fit"
    assert CS.reachable(roomy, COMPACT.parts())
    assert CS.reachable(tight, COMPACT.parts())
    assert not CS.reachable(blocked, COMPACT.parts())


def test_chicane_short_chamber_without_band_is_left_undetermined():
    co = E.make_chicane(1.0, 0.0, 1.0, 0.8, 0.6)
    assert E.FAMILIES["chicane"].feasibility(co, COMPACT)["verdict"] == "undetermined"


# --------------------------------------------------------------------------- (c) L-bend

def test_lbend_closed_form_matches_the_corner_placement_test():
    for rob, a, b in ((COMPACT, 0.9, 0.95), (TANDEM, 1.07, 1.07), (TANDEM, 1.2, 1.0), (TANDEM, 0.8, 1.9)):
        co = E.make_lbend(a, b)
        th, adm, _, _ = E.lbend_admissible_headings(co, rob)
        q = th <= math.pi / 2
        cf = E.lbend_closed_form(th[q], a, b, rob.L, rob.W)
        # disagreement only within one heading step of a boundary of the closed form
        diff = np.flatnonzero(cf != adm[q])
        edge = np.flatnonzero(np.diff(cf.astype(int)) != 0)
        assert all(np.abs(edge - k).min() <= 1 for k in diff)
        assert (E.lbend_corner_margin(a, b, rob.L, rob.W) >= 0) == bool(cf.all())


def test_lbend_blocked_heading_has_no_free_placement_by_brute_force():
    co = E.make_lbend(1.07, 1.07)                       # tandem threshold is a = b = 1.0889
    assert E.lbend_corner_margin(1.0889, 1.0889, TANDEM.L, TANDEM.W) < 0 < E.lbend_corner_margin(1.0890, 1.0890, TANDEM.L, TANDEM.W)
    th, adm, _, _ = E.lbend_admissible_headings(co, TANDEM)
    X, Y = np.meshgrid(np.arange(1.0, 4.2, 0.01), np.arange(0.0, 3.2, 0.01))
    for deg, expect in ((45.0, False), (40.0, False), (30.0, True), (20.0, True)):
        assert adm[int(round(deg * 4))] == expect
        free = ~CS.hits(X.ravel(), Y.ravel(), math.radians(deg), TANDEM.parts(), co.boxes())
        assert free.any() == expect
    assert E.FAMILIES["lbend"].feasibility(co, TANDEM)["verdict"] == "does_not_fit"


@pytest.mark.parametrize("rob,a,c,fits", [
    (TANDEM, 1.09, 0.0, True), (TANDEM, 1.08, 0.0, False), (TANDEM, 1.04, 0.2, True),
    (L3, 0.95, 0.0, True), (L3, 0.80, 0.0, False), (COMPACT, 0.86, 0.1, True),
    (RS.make_config("L4", 0.4, 0.35, rc_shift=0.1), 1.0, 0.1, True),
])
def test_lbend_fits_verdict_has_a_swept_collision_free_corner_path(rob, a, c, fits):
    co = E.make_lbend(a, a, c)
    f = E.FAMILIES["lbend"].feasibility(co, rob)
    assert (f["verdict"] == "fits") == fits
    path = E.lbend_corner_path(co, rob, n=2500)
    if fits:
        assert CS.swept_free(*path, rob.parts(), co.boxes())
        assert abs(path[1][-1] - co.goal[1]) < 1e-9 and abs(path[0][-1] - co.goal[0]) <= co.b
    else:
        assert path is None


def test_lbend_verdicts_against_the_lattice_away_from_the_threshold():
    fam = E.FAMILIES["lbend"]
    for a, expect in ((1.00, False), (1.20, True)):
        co = E.make_lbend(a, a)
        assert (fam.feasibility(co, TANDEM)["verdict"] == "fits") == expect
        assert CS.reachable(co, TANDEM.parts()) == expect


def test_lbend_chamfer_only_removes_material():
    sharp, cham = E.make_lbend(1.0, 1.0, 0.0), E.make_lbend(1.0, 1.0, 0.3)
    X, Y = np.meshgrid(np.arange(-0.1, 4.3, 0.02), np.arange(-0.1, 4.3, 0.02))
    def occ(co):
        b = co.boxes()
        return ((b[:, 0, None, None] <= X) & (X <= b[:, 1, None, None]) & (b[:, 2, None, None] <= Y) & (Y <= b[:, 3, None, None])).any(0)
    o_s, o_c = occ(sharp), occ(cham)
    assert not (o_c & ~o_s).any() and (o_s & ~o_c).any()
    # the staircase is inscribed in the true chamfered solid (cut line (X1 - x) + (y - a) = c) to within one step
    tri = (X > 2.7 + 1e-9) & (X < 3.0 - 1e-9) & (Y > 1.0 + 1e-9) & (Y < 1.3 - 1e-9)
    depth = (3.0 - X) + (Y - 1.0)
    assert not (o_c & tri & (depth < 0.3 - 1e-9)).any()            # never sticks out past the cut
    assert (o_c | ~tri | (depth < 0.3 + 0.02 + 1e-9)).all()        # and fills it up to one 2 cm step


# --------------------------------------------------------------------------- (d) dead end

def test_dead_end_heading_confinement_is_exact_by_brute_force():
    Bw = 0.95
    alpha = E.dead_end_alpha(Bw, COMPACT.L, COMPACT.W)
    assert alpha == pytest.approx(math.asin(Bw / (2 * COMPACT.rho)) - math.atan2(COMPACT.W, COMPACT.L), abs=1e-12)
    co = E.make_dead_end(Bw, 3.0, 2.0, 1.0)
    ys = np.linspace(-0.5, 0.5, 4001)
    assert (~CS.hits(1.5, ys, alpha - 1e-3, COMPACT.parts(), co.boxes())).any()
    assert not (~CS.hits(1.5, ys, alpha + 1e-3, COMPACT.parts(), co.boxes())).any()


def test_dead_end_forward_only_needs_turnaround_room_but_reverse_out_always_works():
    fam = E.FAMILIES["dead_end"]
    for Bw, turn in ((0.95, False), (1.30, True)):
        co = E.make_dead_end(Bw, 3.0, 2.0, 1.0)
        f = fam.feasibility(co, COMPACT)
        assert f["verdict"] == "fits" and f["turnaround_in_bay"] == turn and f["reverse_out_free"]
        assert f["forward_only_exit"] == ("yes" if turn else "no")
        lat = CS.Lattice(co.boxes(), COMPACT.parts(), co.start)
        assert lat.near(lat.reach(), co.goal, 0.25)                                   # any motion: reachable
        fwd = lat.reach(forward_only=True)
        assert lat.near(fwd, co.goal, 0.25) == turn                                   # never reversing
        if not turn:
            assert np.abs(G.wrap(lat.headings_reached(fwd))).max() <= E.dead_end_alpha(Bw, COMPACT.L, COMPACT.W) + 1e-9
    # straight reverse-out, swept
    co = E.make_dead_end(0.95, 3.0, 2.0, 1.0)
    xs = np.linspace(co.start[0], -1.0, 2000)
    assert CS.swept_free(xs, 0.0, 0.0, COMPACT.parts(), co.boxes())


# --------------------------------------------------------------------------- (e) occluded doorway

def test_occluder_visibility_matches_ray_casting():
    for args in ((1.0, 0.8, 4.0, 0.6, 0.2, 1.5, 0.5), (1.0, 0.3, 4.0, 0.9, 0.2, 1.5, 0.2), (1.0, 1.2, 4.0, 0.5, 0.3, 1.3, -0.4)):
        co = E.make_occluded_doorway(*args)
        v = E.occluder_visibility(co)
        lo, hi = v["opening_interval"]
        ang = np.linspace(lo, hi, 8003)[1:-1]
        r = G.raycast(np.array([co.start[:2]]), ang, co.boxes(), 30.0)[0]
        frac = float(np.mean(co.start[0] + r * np.cos(ang) >= co.d - 1e-6))
        assert v["visible_fraction"] == pytest.approx(frac, abs=2e-3)


def test_occluded_sampler_hides_the_door_and_leaves_the_fit_unchanged():
    fam = E.FAMILIES["occluded_doorway"]
    cs = fam.sample(8, 5)
    assert all(E.occluder_visibility(c)["hidden"] for c in cs)
    co = cs[0]
    f = fam.feasibility(co, COMPACT)
    assert f["verdict"] == "fits" and f["fit_unchanged"] and f["binding"]["visible_fraction"] == 0.0
    plain = E.make_offset_doorway(co.w, co.e, co.d, B=co.B, t=co.t, start=co.start)
    assert E.FAMILIES["offset_doorway"].feasibility(plain, R.load_robot("compact_diff"))["verdict"] == "fits"
    assert [w for w in co.walls() if w["name"] != "occluder"] == plain.walls()        # same doorway
    assert CS.reachable(co, COMPACT.parts())


# --------------------------------------------------------------------------- (f) cluttered

def test_cluttered_minimum_gap_is_what_the_parameter_says():
    for lay in range(12):
        v = {"gmin_cm": 90, "K": 4, "pitch_cm": 200, "ps_cm": 30, "layout": lay, "Bc_cm": 300}
        st = E.clutter_stations(v)
        assert min(s["widest_cm"] for s in st) == 90
        co = E.FAMILIES["cluttered"].build(v)
        b = co.boxes()
        for k, s in enumerate(st):                                  # measure the gaps from the boxes
            pil = sorted([(w["ymin"], w["ymax"]) for w in co.walls() if w["name"].startswith("pillar_%d_" % k)])
            gaps = [pil[0][0] + 1.5, pil[1][0] - pil[0][1], 1.5 - pil[1][1]]
            assert np.allclose(np.array(gaps) * 100, s["gaps_cm"], atol=0.51) and s["pillar_cm"] > 0
            assert max(gaps) * 100 == pytest.approx(s["widest_cm"], abs=0.51)
        assert b.shape[0] == 4 + 2 * 4


def test_cluttered_outer_bounds_against_the_lattice():
    fam = E.FAMILIES["cluttered"]
    easy = E.make_cluttered(1.3, 4, 1.5, layout=11)
    assert fam.feasibility(easy, COMPACT)["verdict"] == "fits" and CS.reachable(easy, COMPACT.parts())
    big = RS.make_config("2x2", 0.5, 0.5)
    hard = E.make_cluttered(0.9, 3, 2.0, layout=1)
    assert fam.feasibility(hard, big)["verdict"] == "does_not_fit" and not CS.reachable(hard, big.parts())
    assert fam.feasibility(hard, COMPACT)["verdict"] == "undetermined"                # the heuristic middle range
