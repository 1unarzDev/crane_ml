"""Mechanics formulas against brute force and the numerical planners (EXPLORATORY)."""
import math

import numpy as np
import pytest

import geometry as G
import mechanics as M

COMPACT = (0.80, 0.72)
TANDEM = (1.76, 0.66)
SHAPES = [COMPACT, TANDEM, (1.0, 0.5), (0.42, 0.40)]


def test_h_is_the_corner_extent_and_peaks_at_two_rho():
    rng = np.random.default_rng(0)
    for L, W in SHAPES:
        th = rng.uniform(-math.pi, math.pi, 200)
        cx = G.rect_corners(0.0, 0.0, th, L, W)[..., 0]
        assert np.allclose(cx.max(-1) - cx.min(-1), M.h(th, L, W), atol=1e-12)
        assert M.h(math.atan2(W, L), L, W) == pytest.approx(2 * M.rho(L, W), abs=1e-12)
        assert np.all(M.h(th, L, W) <= 2 * M.rho(L, W) + 1e-12)


@pytest.mark.parametrize("L,W", SHAPES)
@pytest.mark.parametrize("frac", [0.02, 0.2, 0.5, 0.9, 0.999])
def test_alpha_max_matches_brute_force_rotation(L, W, frac):
    d = L + frac * (2 * M.rho(L, W) - L)
    am = M.alpha_max(d, L, W)
    assert M.brute_alpha_max(d, L, W) == pytest.approx(am, abs=2e-5)
    # feasibility edge: just inside rotates freely at the centre, just outside fits nowhere
    xs = np.linspace(0, d, 2001)
    assert M.swept_free_in_strip(np.full(500, d / 2), np.linspace(0, am * (1 - 1e-6), 500), d, L, W)
    assert not M.footprint_in_strip(xs, am + 1e-5, d, L, W).any()
    assert M.footprint_in_strip(xs, am - 1e-5, d, L, W).any()


def test_alpha_max_regimes_and_small_slack():
    L, W = COMPACT
    assert math.isnan(M.alpha_max(L - 0.01, L, W))
    assert M.alpha_max(2 * M.rho(L, W) + 1e-9, L, W) == math.pi / 2
    assert M.alpha_max(L, L, W) == pytest.approx(0.0, abs=1e-7)
    for eps_w, tol in [(0.001, 0.002), (0.01, 0.02), (0.1, 0.15)]:
        eps = eps_w * W
        ratio = M.alpha_max(L + eps, L, W) / M.alpha_max_small(eps, W)
        assert 1.0 < ratio < 1.0 + tol                      # first-order form underestimates


@pytest.mark.parametrize("L,W", SHAPES)
@pytest.mark.parametrize("a", [0.0, 0.08])
def test_four_flow_cycle_is_swept_free_and_shifts_delta(L, W, a):
    d = L + 0.6 * (2 * M.rho(L, W) - L)
    am = M.alpha_max(d, L, W)
    for alpha in [0.2 * am, 0.5 * am, 0.9 * am]:
        x, y, th = M.four_flow_poses(alpha, d, L, W, a=a)
        assert M.swept_free_in_strip(x, th, d, L, W, a=a)
        assert y[-1] == pytest.approx(float(M.four_flow_delta(alpha, d, L, W)), abs=1e-12)
        assert x[-1] == pytest.approx(x[0], abs=1e-12) and th[-1] == 0.0
        # feasibility edge: a 1 % longer stroke hits the door wall
        x2, _, th2 = M.four_flow_poses(alpha, d, L, W, a=a, stroke_scale=1.01)
        assert not M.swept_free_in_strip(x2, th2, d, L, W, a=a)
        # the swept check with full 2-D boxes agrees with the strip check
        boxes = np.array([[-1.0, 0.0, -50, 50], [d, d + 1.0, -50, 50]])
        assert not G.rect_hits_boxes(x, y, th, L, W, boxes, a=a).any()
        assert G.rect_hits_boxes(x2, y, th2, L, W, boxes, a=a).any()


@pytest.mark.parametrize("L,W", SHAPES)
def test_four_flow_maximiser(L, W):
    d = L + 0.4 * (2 * M.rho(L, W) - L)
    a_star, d_star = M.four_flow_best(d, L, W)
    grid = np.linspace(0, M.alpha_max(d, L, W), 200001)
    vals = M.four_flow_delta(grid, d, L, W)
    assert d_star == pytest.approx(vals.max(), rel=1e-8)
    assert a_star == pytest.approx(grid[vals.argmax()], abs=1e-4)
    assert M.four_flow_best(2 * M.rho(L, W) + 0.01, L, W) == (math.pi / 2, M.INF)
    assert M.four_flow_cycles(1.0, 2 * M.rho(L, W) + 0.01, L, W) == 0
    assert M.four_flow_cycles(0.5, d, L, W) == math.ceil(0.5 / d_star)


def test_small_slack_forms_are_first_order_and_underestimate():
    L, W = COMPACT
    prev = None
    for eps_w in [0.3, 0.1, 0.03, 0.01]:
        eps = eps_w * W
        a_star, d_star = M.four_flow_best(L + eps, L, W)
        ra = a_star / M.alpha_star_small(eps, W)
        rd = d_star / M.delta_star_small(eps, W)
        rs = M.stretch_gain_exact(L + eps, L, W) / (eps ** 2 / (2 * W))
        assert ra > 1 and rd > 1 and rs > 1                 # exact gains exceed the first-order forms
        if prev is not None:
            assert rd < prev[0] and rs < prev[1]            # and converge monotonically
        prev = (rd, rs)
    assert prev[0] == pytest.approx(1.0, abs=0.01) and prev[1] == pytest.approx(1.0, abs=0.01)
    # leading corrections: delta* ~ eps^2/4W (1 + (L/W)(eps/W)/4), Delta ~ eps^2/2W (1 + (L/W)(eps/W)/3)
    eps = 0.01 * W
    assert M.four_flow_best(L + eps, L, W)[1] / M.delta_star_small(eps, W) == pytest.approx(1 + (L / W) * 0.01 / 4, abs=2e-4)
    assert M.stretch_gain_exact(L + eps, L, W) / (eps ** 2 / (2 * W)) == pytest.approx(1 + (L / W) * 0.01 / 3, abs=2e-4)


@pytest.mark.parametrize("L,W", [COMPACT, TANDEM])
@pytest.mark.parametrize("frac", [0.1, 0.5, 0.95])
def test_stretch_gain_closed_form_matches_swept_path_and_planner(L, W, frac):
    d = L + frac * (2 * M.rho(L, W) - L)
    exact = M.stretch_gain_exact(d, L, W)
    # brute force: follow theta_max(x) (found by corner bisection, no formula), integrate tan
    xs = np.linspace(L / 2, d - L / 2, 4001)
    lo, hi = np.zeros_like(xs), np.full_like(xs, math.pi / 2)
    hi[:] = M.brute_alpha_max(d, L, W) + 1e-3
    for _ in range(50):
        mid = 0.5 * (lo + hi)
        ok = M.footprint_in_strip(xs, mid, d, L, W)
        lo, hi = np.where(ok, mid, lo), np.where(ok, hi, mid)
    assert M.swept_free_in_strip(xs, lo, d, L, W)
    t = np.tan(lo)
    brute = float(np.sum(0.5 * (t[1:] + t[:-1]) * np.diff(xs)))
    assert brute == pytest.approx(exact, rel=2e-4)
    assert M.numeric_stretch_gain(d, L, W, nx=801, ntheta=2401) == pytest.approx(exact, rel=1e-2)
    assert M.numeric_stretch_gain(d, L, W, nx=801, ntheta=2401) <= exact + 1e-12   # inner approximation


def test_stretch_gain_does_not_depend_on_axle_offset():
    L, W = COMPACT
    d = L + 0.2
    base = M.numeric_stretch_gain(d, L, W, a=0.0, nx=801, ntheta=2401)
    for a in [0.05, 0.15, -0.1]:
        assert M.alpha_max(d, L, W) <= math.atan(W / (L + 2 * abs(a)))          # nested regime
        assert M.numeric_stretch_gain(d, L, W, a=a, nx=801, ntheta=2401) == pytest.approx(base, rel=5e-3)


@pytest.mark.parametrize("eps_w", [0.1, 0.2, 0.35])
@pytest.mark.parametrize("start", ["rear", "centre"])
def test_planner_reversals_match_exact_count(eps_w, start):
    L, W = COMPACT
    d = L + eps_w * W
    D = M.stretch_gain_exact(d, L, W)
    for k in [0.3, 2.4, 7.4]:                                # offsets away from the integer thresholds
        e = k * D
        got = M.plan_min_reversals(e, d, L, W, x_start=start, nx=401, ntheta=1201)
        assert got["reversals"] == M.min_reversals_exact(e, d, L, W, start)


def test_planner_general_method_with_arcs_agrees_with_fast():
    L, W = COMPACT
    d = L + 0.25 * W
    D = M.stretch_gain_exact(d, L, W)
    for k in [0.4, 2.5]:
        fast = M.plan_min_reversals(k * D, d, L, W, nx=61, ntheta=121, method="fast")
        gen = M.plan_min_reversals(k * D, d, L, W, nx=61, ntheta=121, method="general", arcs=True)
        assert gen["method"] == "general" and fast["method"] == "fast"
        assert gen["reversals"] == fast["reversals"]
        assert gen["history"][0] == pytest.approx(fast["history"][0], rel=0.02)


def test_true_3d_lattice_search_agrees_with_the_dynamic_programme():
    L, W = COMPACT
    d = L + 0.3 * W
    for e in [0.02, 0.07, 0.15]:
        bfs = M.lattice_bfs_min_reversals(e, d, L, W, nx=31, kappa=0.04)
        dp = M.plan_min_reversals(e, d, L, W, xs=bfs["xs"], thetas=bfs["thetas"], method="general", arcs=False)
        assert bfs["reversals"] == dp["reversals"]
        assert bfs["reversals"] >= M.min_reversals_exact(e, d, L, W)       # coarse lattice can only cost more


def test_planner_roomy_bay_needs_no_reversal_and_blocked_is_infinite():
    L, W = COMPACT
    r = M.plan_min_reversals(3.0, 2 * M.rho(L, W) + 0.02, L, W, nx=41, ntheta=401, x_start="centre")
    assert r["reversals"] == 0
    assert M.plan_min_reversals(1.0, L - 0.05, L, W)["reversals"] == M.INF
    assert M.min_reversals_exact(1.0, 2 * M.rho(L, W), L, W) == 0
    assert M.min_reversals_exact(1.0, L, L, W) == M.INF


def test_first_order_bound_is_asymptotic_not_a_strict_bound():
    """2 W e / eps^2 overstates the minimum at finite slack: the planner beats it."""
    L, W = COMPACT
    eps = 0.3 * W
    d = L + eps
    e = 20.5 * eps ** 2 / (2 * W)                            # first-order claim: at least 21 stretches
    assert math.ceil(M.lower_bound_small(e, eps, W)) == 21
    got = M.plan_min_reversals(e, d, L, W, nx=401, ntheta=1201)
    assert got["stretches"] == M.min_stretches_exact(e, d, L, W) < 21
    # the exact bound holds: no lattice plan uses fewer stretches than ceil(e / Delta)
    for eps_w in [0.05, 0.15]:
        dd = L + eps_w * W
        for e2 in [0.05, 0.31]:
            assert M.plan_min_reversals(e2, dd, L, W)["stretches"] >= M.min_stretches_exact(e2, dd, L, W)
    # four-flow against the bound, in the same unit (stretches): ratio -> 4, not 2
    eps = 0.01 * W
    cyc = M.four_flow_cycles(1.0, L + eps, L, W)
    assert (2 * cyc - 1) / M.min_stretches_exact(1.0, L + eps, L, W) == pytest.approx(4.0, rel=0.01)


# ----------------------------------------------------------------------------- door

def _brute_door_ok(theta, w, t, L, W, n_x=121, n_c=1601):
    """For every depth of the footprint relative to the wall, is some lateral position free?"""
    boxes = np.array([[0.0, t, w / 2, 50.0], [0.0, t, -50.0, -w / 2]])
    reach = L / 2 + W / 2 + t
    for xc in np.linspace(-reach, t + reach, n_x):
        cs = np.linspace(-w, w, n_c)
        if G.rect_hits_boxes(np.full(n_c, xc), cs, np.full(n_c, theta), L, W, boxes).all():
            return False
    return True


@pytest.mark.parametrize("L,W,w,t", [(0.80, 0.72, 0.94, 0.20), (0.80, 0.72, 0.94, 1.20), (1.76, 0.66, 0.80, 0.10),
                                     (0.80, 0.72, 0.80, 0.05)])
def test_door_heading_tolerance_matches_brute_force(L, W, w, t):
    tol = M.door_heading_tolerance(w, t, L, W)
    assert 0 < tol < math.pi / 2
    assert _brute_door_ok(tol - 0.02, w, t, L, W)
    assert not _brute_door_ok(tol + 0.02, w, t, L, W)


def test_door_tolerance_closed_forms():
    L, W = COMPACT
    # tunnel (t >= L): footprint fully inside, L sin + W cos = w; small-angle (w - W)/L
    w = W + 0.02
    tol = M.door_heading_tolerance(w, 2.0, L, W)
    assert L * math.sin(tol) + W * math.cos(tol) == pytest.approx(w, abs=1e-9)
    assert tol == pytest.approx(M.door_heading_tolerance_small(w, L, W), rel=0.03)
    # thin wall: static tolerance equals the drive-through tolerance W + t sin = w cos
    for t in [0.05, 0.2]:
        tol = M.door_heading_tolerance(0.94, t, L, W)
        dt = M.door_drive_through_tolerance(0.94, t, W)
        assert tol == pytest.approx(dt, abs=1e-6)
        assert W + t * math.sin(dt) == pytest.approx(0.94 * math.cos(dt), abs=1e-12)
        assert tol > 1.5 * M.door_heading_tolerance_small(0.94, L, W)      # the plan's form is far too strict here
    assert M.door_heading_tolerance(0.70, 0.2, L, W) == 0.0
    assert M.door_heading_tolerance(1.2, 0.2, 0.42, 0.40) == math.pi / 2
    assert M.door_effective_width(0.0, 0.3, L, W) == pytest.approx(W)


def test_door_visibility_analytic_and_rays():
    d, t, w = 1.5, 0.2, 0.94
    head_on = M.door_visibility(0.75, 0.0, d, t, 0.0, w, n_beams=720)
    assert head_on["near_width_rad"] == pytest.approx(2 * math.atan(w / 2 / 0.75))
    assert head_on["through_width_rad"] == pytest.approx(2 * math.atan(w / 2 / 0.95))
    assert head_on["far_visible"]
    assert head_on["beams_through"] == pytest.approx(head_on["through_width_rad"] / head_on["angular_resolution_rad"], abs=2)
    # oblique + thick wall: the opening still subtends an angle but no ray clears both faces
    hid = M.door_visibility(1.3, 0.0, d, 0.4, 2.0, w, n_beams=720)
    assert hid["near_width_rad"] > 0 and hid["through_width_rad"] == 0.0
    assert hid["beams_through"] == 0 and not hid["far_visible"]
    # first-order form w r / (r^2 + y^2) at long range
    far = M.door_visibility(0.0, 0.0, 8.0, 0.0, 3.0, w, n_beams=7200)
    assert far["near_width_rad"] == pytest.approx(far["first_order_width_rad"], rel=0.01)
    with pytest.raises(ValueError):
        M.door_visibility(2.0, 0.0, d, t, 0.0, w)
