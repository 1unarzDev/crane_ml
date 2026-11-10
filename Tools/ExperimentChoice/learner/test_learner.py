"""Tests for the learner pilot: acquisition, scoring, GP, features, lattice pair, labels (EXPLORATORY)."""
import math

import numpy as np
import pytest

import course as C
import mechanics as M
import robots as R
import surrogate as S

import acquire as A
import features as F
import gpmodel as GPM
import latticenav as LN
import pilot as P
import truth as T

COMPACT = R.load_robot("compact_diff")
TANDEM = R.load_robot("narrow_tandem")


# ---------------------------------------------------------------- normal helpers, expected sign loss

def test_norm_cdf_matches_erf():
    z = np.linspace(-6, 6, 241)
    ref = np.array([0.5 * (1 + math.erf(v / math.sqrt(2))) for v in z])
    assert np.max(np.abs(GPM.norm_cdf(z) - ref)) < 2e-7


def test_expected_sign_loss_against_quadrature():
    x = np.linspace(-60, 60, 600001)
    for mu, sig in ((0.0, 1.0), (0.7, 1.3), (-2.0, 0.5), (3.0, 4.0)):
        pdf = np.exp(-0.5 * ((x - mu) / sig) ** 2) / (sig * math.sqrt(2 * math.pi))
        wrong = np.sign(x) != (1.0 if mu >= 0 else -1.0)
        ref = float(np.sum(np.abs(x) * wrong * pdf) * (x[1] - x[0]))
        assert float(A.expected_sign_loss(mu, sig)) == pytest.approx(ref, abs=2e-4)


def test_expected_sign_loss_properties():
    assert float(A.expected_sign_loss(1.0, 0.0)) == 0.0
    assert float(A.expected_sign_loss(0.0, 2.0)) == pytest.approx(2.0 / math.sqrt(2 * math.pi), rel=1e-6)
    assert float(A.expected_sign_loss(-1.5, 0.8)) == pytest.approx(float(A.expected_sign_loss(1.5, 0.8)))
    s = np.linspace(0.1, 3, 30)
    assert np.all(np.diff(A.expected_sign_loss(1.0, s)) > 0)            # more uncertainty, more loss
    m = np.linspace(0, 3, 30)
    assert np.all(np.diff(A.expected_sign_loss(m, 1.0)) < 0)            # further from zero, less loss


# ---------------------------------------------------------------- acquisition

def test_straddle_prefers_uncertain_points_near_zero():
    mu = np.array([0.0, 0.0, 5.0, 0.2])
    var = np.array([1.0, 0.01, 1.0, 1.0])
    s = A.straddle(mu, var)
    assert int(np.argmax(s)) == 0
    assert s[0] == pytest.approx(1.96) and s[2] == pytest.approx(1.96 - 5.0)
    assert s[3] > s[1]


def _kernel(n=7, ls=0.3, seed=0):
    rng = np.random.default_rng(seed)
    X = rng.random((n, 2))
    d2 = ((X[:, None] - X[None]) ** 2).sum(2)
    return X, np.exp(-0.5 * d2 / ls ** 2)


def test_condition_is_the_exact_gaussian_update():
    _, K = _kernel()
    c, tau2 = 3, 0.2
    new = A.condition(K, c, tau2)
    # brute force: joint Gaussian of (f, y_c), y_c = f_c + noise
    ref = K - K[:, [c]] @ K[[c], :] / (K[c, c] + tau2)
    assert np.allclose(new, ref)
    assert new[c, c] == pytest.approx(K[c, c] * tau2 / (K[c, c] + tau2))
    assert np.all(np.diag(new) <= np.diag(K) + 1e-12)
    assert np.all(np.linalg.eigvalsh(new) > -1e-9)


def test_sur_scores_match_brute_force_and_rank_sensibly():
    _, K = _kernel(n=6, seed=2)
    mu = np.array([0.1, -0.2, 3.0, -4.0, 0.05, 1.0])
    tau2 = 0.1
    s = A.sur_scores(mu, K, tau2)
    for c in range(6):
        after = A.condition(K, c, tau2)
        ref = (A.expected_sign_loss(mu, np.sqrt(np.diag(K))) - A.expected_sign_loss(mu, np.sqrt(np.maximum(np.diag(after), 0)))).sum()
        assert s[c] == pytest.approx(float(ref), abs=1e-9)
    assert np.all(s >= -1e-12)
    # independent points: observing one far from the boundary is worth less than one on it
    s2 = A.sur_scores(np.array([0.0, 6.0]), np.eye(2), 0.1)
    assert s2[0] > 100 * s2[1]


def test_sur_ignores_points_that_teach_nothing():
    cov = np.diag([1.0, 1e-14, 1.0])
    s = A.sur_scores(np.array([0.0, 0.0, 0.0]), cov, 0.1)
    assert s[1] < 1e-6 and s[0] == pytest.approx(s[2])


@pytest.mark.parametrize("rule", ["straddle", "sur"])
def test_select_batch_distinct_available_and_spread(rule):
    # candidates 0 and 1 are the same point; 2 is independent; 3 is far from the boundary
    cov = np.array([[1.0, 1.0, 0.0, 0.0], [1.0, 1.0, 0.0, 0.0], [0.0, 0.0, 0.9, 0.0], [0.0, 0.0, 0.0, 1.0]])
    mu = np.array([0.0, 0.0, 0.0, 9.0])
    picks = A.select_batch(rule, mu, cov, 1e-6, np.ones(4, bool), 2)
    assert len(set(picks)) == 2 and picks[0] in (0, 1) and picks[1] == 2      # not the duplicate
    avail = np.array([False, False, True, True])
    assert A.select_batch(rule, mu, cov, 1e-6, avail, 3) in ([2, 3], [3, 2])
    assert A.select_batch(rule, mu, cov, 1e-6, np.zeros(4, bool), 2) == []


def test_select_batch_rejects_unknown_rule():
    with pytest.raises(ValueError):
        A.select_batch("ei", np.zeros(2), np.eye(2), 0.1, np.ones(2, bool), 1)


# ---------------------------------------------------------------- scoring

def test_slog_keeps_sign_and_order():
    x = np.array([-150.0, -4.0, 0.0, 0.3, 150.0])
    y = A.slog(x)
    assert np.all(np.sign(y) == np.sign(x)) and np.all(np.diff(y) > 0)
    assert y[-1] == pytest.approx(math.log(151.0))


def test_recommend_and_wrong_cost_by_hand():
    delta = np.array([-4.0, 60.0, 0.2, 0.0, -30.0, 12.0])
    labels = np.array(["A_wins", "B_wins", "indifferent", "both_fail", "A_wins", "unresolved"])
    assert list(A.recommend([-1.0, 0.0, 2.0])) == ["A", "A", "B"]
    all_a, all_b = np.array(["A"] * 6), np.array(["B"] * 6)
    assert A.wrong_cost(delta, labels, all_a) == pytest.approx(60.0 / 6)            # only the B_wins course
    assert A.wrong_cost(delta, labels, all_b) == pytest.approx((4.0 + 30.0) / 6)
    perfect = np.array(["A", "B", "B", "A", "A", "A"])
    assert A.wrong_cost(delta, labels, perfect) == 0.0
    assert A.sign_accuracy(labels, all_a) == pytest.approx(2 / 3)
    assert A.sign_accuracy(labels, all_a, subset=np.array([0, 1, 1, 1, 0, 0], bool)) == 0.0
    assert math.isnan(A.sign_accuracy(np.array(["indifferent"]), np.array(["A"])))


def test_wrong_cost_is_never_negative_and_bounded_by_constant_rules():
    rng = np.random.default_rng(0)
    delta = rng.normal(0, 20, 200)
    labels = np.where(delta > 1, "B_wins", np.where(delta < -1, "A_wins", "indifferent"))
    rec = rng.choice(["A", "B"], 200)
    r = A.wrong_cost(delta, labels, rec)
    assert 0 <= r <= A.wrong_cost(delta, labels, np.array(["A"] * 200)) + A.wrong_cost(delta, labels, np.array(["B"] * 200))
    assert A.wrong_cost(delta, labels, A.recommend(delta)) == 0.0


def test_near_boundary():
    X = np.array([[0.0, 0.0], [0.1, 0.0], [0.5, 0.0], [0.58, 0.0], [0.9, 0.0]])
    labels = np.array(["A_wins", "A_wins", "A_wins", "B_wins", "both_fail"])
    near = A.near_boundary(X, labels, radius=0.1)
    assert list(near) == [False, False, True, True, False]       # both_fail neighbours do not make a boundary
    near2 = A.near_boundary(X[:2], labels[:2], radius=0.15, X_ref=np.array([[0.2, 0.0]]), labels_ref=np.array(["indifferent"]))
    assert list(near2) == [False, True]


def test_bootstrap_ci():
    m, lo, hi = A.bootstrap_ci([2.0] * 10)
    assert (m, lo, hi) == (2.0, 2.0, 2.0)
    v = np.random.default_rng(1).normal(5, 1, 40)
    m, lo, hi = A.bootstrap_ci(v)
    assert lo < m < hi and m == pytest.approx(v.mean()) and hi - lo < 1.0
    assert all(math.isnan(x) for x in A.bootstrap_ci([]))


# ---------------------------------------------------------------- GP and Sobol

def test_gp_learns_a_step_and_reports_calibrated_variance():
    rng = np.random.default_rng(0)
    X = rng.random((60, 2))
    y = np.where(X[:, 0] > 0.5, 3.0, -1.0) + 0.05 * rng.standard_normal(60)
    gp = GPM.GP().fit(X, y)
    Xt = rng.random((400, 2))
    mu, var = gp.posterior(Xt)
    far = np.abs(Xt[:, 0] - 0.5) > 0.1
    assert np.mean((mu[far] > 1.0) == (Xt[far, 0] > 0.5)) > 0.97
    assert gp.ls[0] < gp.ls[1]                                      # the irrelevant input gets the longer scale
    mu2, cov = gp.posterior(Xt[:50], full_cov=True)
    assert np.allclose(mu2, mu[:50]) and np.allclose(np.diag(cov), var[:50], atol=1e-8)
    mu_far, var_far = gp.posterior(np.array([[30.0, 30.0]]))
    assert var_far[0] == pytest.approx(gp.ys ** 2, rel=1e-6) and mu_far[0] == pytest.approx(gp.ym, abs=1e-6)


def test_gp_handles_replicates():
    X = np.repeat(np.linspace(0, 1, 8)[:, None], 3, axis=0)
    y = np.sin(4 * X[:, 0]) + np.tile([0.2, -0.2, 0.0], 8)
    gp = GPM.GP().fit(X, y)
    mu, _ = gp.posterior(np.linspace(0, 1, 8)[:, None])
    assert np.max(np.abs(mu - np.sin(4 * np.linspace(0, 1, 8)))) < 0.15


def test_sobol_is_a_low_discrepancy_sequence():
    p = GPM.sobol(8, 3)
    assert np.allclose(p[:4, 0], [0.0, 0.5, 0.75, 0.25])
    for d in range(3):
        assert sorted(np.floor(p[:, d] * 8).astype(int)) == list(range(8))       # one point per eighth
    q = GPM.sobol(64, 3, seed=5)
    assert q.min() >= 0 and q.max() < 1 and not np.allclose(q[:8], p)
    for d in range(3):
        assert sorted(np.floor(q[:, d] * 64).astype(int)) == list(range(64))     # the shift keeps the net
    assert np.array_equal(q, GPM.sobol(64, 3, seed=5))


# ---------------------------------------------------------------- features and rules

def test_mech_features_agree_with_mechanics_module():
    for w, e, d in ((0.94, 0.72, 1.30), (0.94, 0.36, 1.00), (1.10, 0.05, 0.90), (0.85, 1.2, 0.86)):
        c = C.make_course(w, e, d)
        f = M.features(c, COMPACT)
        got = F.mech(c, COMPACT)
        assert got[0] == pytest.approx(f["d_over_2rho"]) and got[1] == pytest.approx(f["e_over_W"])
        assert got[2] == pytest.approx(f["w_over_W"])
        rev = min(f["min_reversals_exact"], F.REV_CAP)
        assert got[3] == pytest.approx(math.log1p(rev) / math.log1p(F.REV_CAP))
        assert got[4] == (1.0 if f["regime"] == "roomy" else 0.0)


def test_slack_features_are_raw_shifted_by_the_footprint():
    c = C.make_course(0.94, 0.72, 1.30)
    assert np.allclose(np.array(F.raw(c)) - np.array(F.slack(c, COMPACT)), [COMPACT.W, 0.0, COMPACT.L])


def test_barn_metrics_on_a_known_course():
    c = C.make_course(0.90, 0.60, 1.40)
    cm, cmin, disp, cdm, cdmin = F.barn(c)
    assert cmin == pytest.approx(0.45, abs=1e-6)                      # half the door width, on the door axis
    assert cdmin == pytest.approx(0.90, abs=1e-6)                     # the door opening is the narrowest passage
    assert cm > cmin and cdm > cdmin and disp > 0
    wide = F.barn(C.make_course(1.20, 0.60, 1.40))
    assert wide[1] > cmin and wide[4] > cdmin
    pts = F.reference_path(c)
    assert np.allclose(pts[0], c.start[:2]) and np.allclose(pts[-1], c.goal)
    assert np.max(np.hypot(*np.diff(pts, axis=0).T)) <= 0.05 + 1e-9


def test_closest_wall_distance_exact():
    boxes = np.array([[1.0, 2.0, -1.0, 1.0]])
    pts = np.array([[0.0, 0.0], [3.0, 2.0], [1.5, 0.0]])
    assert np.allclose(F.closest_wall_distance(pts, boxes), [1.0, math.sqrt(2.0), 0.0])


def test_rules():
    two_rho = 2 * COMPACT.rho
    lo, hi = C.make_course(0.94, 0.72, two_rho - 0.02), C.make_course(0.94, 0.72, two_rho + 0.02)
    assert F.rule_2rho(lo, COMPACT) == "B" and F.rule_2rho(hi, COMPACT) == "A"
    assert F.rule_stretch(lo, COMPACT) == "B" and F.rule_stretch(hi, COMPACT) == "A"
    tiny = C.make_course(0.94, 0.01, two_rho - 0.02)                  # tight bay, offset one stretch can make
    assert F.rule_2rho(tiny, COMPACT) == "B" and F.rule_stretch(tiny, COMPACT) == "A"
    assert F.RULES["always_A"](lo, COMPACT) == "A" and F.RULES["always_B"](lo, COMPACT) == "B"


def test_scaler():
    ref = np.array([[1.0, 5.0, 2.0], [3.0, 5.0, 4.0]])
    sc = F.Scaler(ref)
    assert np.allclose(sc(ref), [[0, 0, 0], [1, 0, 1]])
    assert np.allclose(sc(np.array([[5.0, 6.0, 3.0]])), [[2.0, 1.0, 0.5]])       # extrapolates, never clips


# ---------------------------------------------------------------- independent pair

def test_lattice_headings_and_kernels():
    assert LN.NH == 32 and len(set(LN.DIRS)) == 32
    ang = np.unwrap(np.array(LN.ANGLES))
    assert np.all(np.diff(ang) > 0) and ang[-1] - ang[0] < 2 * math.pi
    k0 = LN.footprint_kernel(0.0, 0.8, 0.72, 0.025, 0.05)
    assert np.array_equal(k0, k0[::-1, ::-1])
    K = k0.shape[0] // 2
    assert k0[K + 8, K] and not k0[K + 9, K]                          # 0.40 + 0.025 pad: 8 cells ahead, not 9
    assert k0[K, K + 7] and not k0[K, K + 8]                          # 0.36 + 0.025: 7 cells to the side
    k90 = LN.footprint_kernel(math.pi / 2, 0.8, 0.72, 0.025, 0.05)
    assert np.array_equal(k90, k0.T)


def _corridor_planner(goal):
    cfg = dict(LN.LATTICE_CFG)
    grid = LN.LatticeGrid((-3.0, 3.0), (-2.0, 2.0), COMPACT, cfg)
    xs = np.arange(-3.0, 3.0, 0.02)
    half = COMPACT.W / 2 + 0.15                                      # corridor 1.02 m wide: too narrow to turn round
    grid.add_points(np.concatenate([xs, xs]), np.concatenate([np.full(xs.size, half + 0.03), np.full(xs.size, -half - 0.03)]))
    return grid, LN.LatticePlanner(grid, COMPACT, goal, cfg)


def test_only_the_reverse_setting_reaches_a_goal_behind_in_a_corridor():
    grid, pl = _corridor_planner((-1.5, 0.0))
    start = pl.snap(0.0, 0.0, 0.0)
    assert start is not None
    assert pl.plan(start, allow_reverse=False) is None
    path = pl.plan(start, allow_reverse=True)
    assert path is not None and {mv for _, mv in path[1:]} == {"R"}
    segs = pl.segments(path)
    assert len(segs) == 1 and segs[0]["kind"] == "lin" and segs[0]["dir"] == -1 and segs[0]["x"] < -1.3


def test_both_settings_agree_when_the_goal_is_ahead():
    grid, pl = _corridor_planner((1.5, 0.0))
    start = pl.snap(0.0, 0.0, 0.0)
    pf, pr = pl.plan(start, False), pl.plan(start, True)
    assert pf is not None and [s for s, _ in pf] == [s for s, _ in pr]
    assert {mv for _, mv in pf[1:]} == {"F"} and pl.path_valid(pf)


def test_planner_refuses_a_start_in_collision_and_grid_updates_incrementally():
    cfg = dict(LN.LATTICE_CFG)
    grid = LN.LatticeGrid((-2.0, 2.0), (-2.0, 2.0), COMPACT, cfg)
    pl = LN.LatticePlanner(grid, COMPACT, (1.0, 0.0), cfg)
    assert pl.snap(0.0, 0.0, 0.0) is not None
    v = grid.version
    assert grid.add_points(np.array([0.2, 0.2]), np.array([0.0, 0.0])) and grid.version == v + 1
    assert not grid.add_points(np.array([0.2]), np.array([0.0]))             # nothing new
    assert pl.snap(0.0, 0.0, 0.0) is None                                    # a hit inside the footprint
    assert grid.blocked(0, *grid.cell(0.0, 0.0)) and not grid.blocked(0, *grid.cell(-1.0, 1.0))


def test_independent_pair_is_registered_and_differs_in_one_setting():
    for arm in LN.INDEPENDENT_ARMS:
        assert S.EXTERNAL_CONTROLLERS[arm] is LN.LatticeFollower
    c = C.make_course(0.94, 0.72, 1.30)
    a = LN.LatticeFollower(LN.ARM_FORWARD, COMPACT, c.start, c.goal, dict(S.DEFAULT_CFG))
    b = LN.LatticeFollower(LN.ARM_REVERSE, COMPACT, c.start, c.goal, dict(S.DEFAULT_CFG))
    assert a.cfg == b.cfg and (a.allow_reverse, b.allow_reverse) == (False, True)
    assert a.robot.v_max == b.robot.v_max
    src = open(LN.__file__, encoding="utf-8").read().split('"""', 2)[2]      # code without the module docstring
    for word in ("import mechanics", "M.", "rho(", ".rho", "alpha_max", "stretch_gain", "four_flow"):
        assert word not in src


def test_independent_pair_episode_roomy_bay():
    c = C.make_course(0.94, 0.72, 1.30)
    g = S.paired_gap(c, COMPACT, [1000], arms=LN.INDEPENDENT_ARMS)
    assert g["arm_A"] == LN.ARM_FORWARD and g["arm_B"] == LN.ARM_REVERSE
    assert g["status_A"] == ["success"] and g["status_B"] == ["success"]
    assert abs(g["gap"][0]) < 2.0 and g["J_A"][0] < 30.0
    again = S.paired_gap(c, COMPACT, [1000], arms=LN.INDEPENDENT_ARMS)
    assert again["J_A"] == g["J_A"] and again["J_B"] == g["J_B"]                # deterministic


def test_constructed_pair_is_still_the_default():
    assert S.ARMS["differential"] == ("rotate_then_follow", "stroke_shimmy")
    assert T.PAIRS["constructed"] == S.ARMS["differential"] and S.SURROGATE_VERSION == 2


# ---------------------------------------------------------------- truth labels

def _agg(gap, sa="success", sb="success"):
    return {"gap": list(gap), "status_A": [sa] * len(gap), "status_B": [sb] * len(gap)}


def test_labels_and_sign_rule():
    assert T.label(_agg([0.0, 0.0, 0.0], "stuck", "stuck"), 1.0) == "both_fail"
    assert T.label(_agg([0.3, -0.2, 0.4]), 1.0) == "indifferent"
    assert T.label(_agg([-4.0, -4.1, -3.9]), 1.0) == "A_wins"
    assert T.label(_agg([140.0, 139.0, 141.0], "stuck", "success"), 1.0) == "B_wins"
    assert T.label(_agg([140.0, -4.0, -4.0, -4.0]), 1.0) == "unresolved"
    assert T.sign_settled([5.0, 5.0, 5.0]) and not T.sign_settled([5.0]) and not T.sign_settled([0.0, 0.0])
    m, med = T.noise_margin([_agg([0.0, 0.1, 0.2]), _agg([1.0, 3.0, 5.0]), _agg([9.0, 9.0], "stuck")])
    assert med == pytest.approx(np.median([0.1, 2.0])) and m == pytest.approx(2 * med)
    assert T.noise_margin([_agg([0.0, 0.01, 0.02])])[0] == 1.0


def test_sets_are_disjoint_in_seeds_and_match_the_manifest():
    import json
    man = json.load(open(T.MANIFEST, encoding="utf-8"))
    assert man["sets"]["heldout"]["course_ids_sha256"] == T.ids_hash(T.heldout_courses())
    assert man["sets"]["pool"]["course_ids_sha256"] == T.ids_hash(T.pool_courses())
    assert man["sets"]["transfer"]["course_ids_sha256"] == T.ids_hash(T.transfer_courses())
    seeds = [set(T.POOL_SEEDS), set(T.HELDOUT_SEEDS) | {s for g in T.HELDOUT_TOPUPS for s in g}, set(T.TRANSFER_SEEDS)]
    assert not (seeds[0] & seeds[1]) and not (seeds[0] & seeds[2]) and not (seeds[1] & seeds[2])
    box = T.BOX
    for c in T.heldout_courses():
        assert box["w_cm"][0] <= c.w_cm <= box["w_cm"][1] and box["d_cm"][0] <= c.d_cm <= box["d_cm"][1]
        assert c.t_cm == T.T_CM and c.e_cm + c.w_cm / 2 <= c.B_cm / 2 - box["side_clearance_cm"]
    assert man["frozen_box_on_narrow_tandem"] == {"does_not_fit": len(T.pool_courses())}


# ---------------------------------------------------------------- end to end on synthetic truth

def _synthetic_rows(noise=0.3):
    """A made-up truth with a sharp flip at d = 2 rho for e > 0.3 m: +60 s below, -4 s above."""
    rng = np.random.default_rng(3)
    rows = []
    for set_name, robot in (("pool", COMPACT), ("heldout", COMPACT), ("transfer", TANDEM),
                            ("mf_small", T.load_robot("synth_small")), ("mf_xl", T.load_robot("synth_xl"))):
        for c in T.courses_of(set_name):
            seeds = T.base_seeds(set_name)
            tight = c.d < 2 * robot.rho and c.e > 0.3
            gap = (60.0 if tight else -4.0) + noise * rng.standard_normal(len(seeds))
            ja = [150.0 if tight else 10.0] * len(seeds)
            rows.append({"set": set_name, "pair": "constructed", "robot": robot.name, "course_id": c.course_id,
                         "w_cm": c.w_cm, "e_cm": c.e_cm, "d_cm": c.d_cm, "t_cm": c.t_cm, "seeds": seeds,
                         "J_A": ja, "J_B": [a - g for a, g in zip(ja, gap)],
                         "status_A": ["stuck" if tight else "success"] * len(seeds),
                         "status_B": ["success"] * len(seeds), "strokes_B": [0] * len(seeds)})
    return rows


@pytest.fixture(scope="module")
def synth():
    return P.Data("constructed", _synthetic_rows())


def test_data_shapes_and_shared_randomness(synth):
    D = synth
    assert D.P == 504 and D.gaps.shape == (504, 3) and len(D.delta["heldout"]) == 150       # 56 pool courses
    assert len(D.delta["transfer"]) == 80                             # and 16 transfer courses break side clearance
    assert set(D.labels["heldout"]) == {"A_wins", "B_wins"}
    assert D.X[("raw", "pool")].min() == 0.0 and D.X[("raw", "pool")].max() == 1.0
    assert D.X[("raw", "transfer")][:, 2].min() > 1.0                 # the tandem bay is outside the trained range
    assert D.X[("slack", "transfer")][:, 2].max() <= 1.0 + 1e-9       # but its slack is inside
    so = D.sobol_order(4)
    assert len(set(so)) == len(so) == 256 and so == D.sobol_order(4) and so != D.sobol_order(5)
    assert np.array_equal(D.seed_perm(2), D.seed_perm(2))
    idx, y = D.observe([5, 9], D.seed_perm(0), 2)
    assert list(idx) == [5, 5, 9, 9] and y.shape == (4,)
    assert 0 < D.near.sum() < 150


def test_rules_on_synthetic_truth(synth):
    D = synth
    recs = D.rule_recs("heldout")
    r2 = A.wrong_cost(D.delta["heldout"], D.labels["heldout"], recs["rule_2rho"])
    ra = A.wrong_cost(D.delta["heldout"], D.labels["heldout"], recs["always_A"])
    assert 0 < r2 < 1.0 < ra                                           # only the e <= 0.3 sliver is wrong


@pytest.mark.parametrize("arm", ["sobol-raw", "random-mech", "straddle-mech", "sur-raw", "sobol-barn"])
def test_run_arm_end_to_end(synth, arm):
    D = synth
    res = P.run_arm(D, arm, rep=0, checkpoints=(20, 40))
    assert set(k for k in res if k != "chosen") == {20, 40}
    always_a = A.wrong_cost(D.delta["heldout"], D.labels["heldout"], np.array(["A"] * 150))
    for n in (20, 40):
        m = res[n]
        assert 0.0 <= m["R"] <= always_a + 4.0 and 0.0 <= m["acc"] <= 1.0
        assert any(k.startswith("Rt_") for k in m)
        if arm.endswith("-raw"):
            assert {"Rt_raw", "Rt_slack", "Rt_rawdim", "Rt_rawdim_ls3.2"} <= set(m)
    if arm.split("-")[0] in ("straddle", "sur"):
        ch = res["chosen"]
        assert len(ch) == len(set(ch)) == 40 and ch[:10] == D.sobol_order(0)[:10]     # same start as the Sobol arm
        assert res[40]["n_actual"] == 40
    if arm == "straddle-mech":
        assert res[40]["R"] < 0.5 * always_a                           # learns the made-up flip


def test_active_and_passive_arms_see_the_same_first_ten_runs(synth):
    D = synth
    perm = D.seed_perm(1)
    _, y1 = D.observe(D.sobol_order(1)[:10], perm, 1)
    res = P.run_arm(D, "straddle-raw", rep=1, checkpoints=(20,))
    _, y2 = D.observe(res["chosen"][:10], perm, 1)
    assert np.array_equal(y1, y2)


# ---------------------------------------------------------------- dimension vector, three footprints

def test_constant_dimension_columns_change_nothing_in_distribution():
    rng = np.random.default_rng(0)
    X = rng.random((40, 3))
    y = np.where(X[:, 2] > 0.4, -1.5, 3.0) + 0.1 * rng.standard_normal(40)
    dims = np.array(F.dims(COMPACT))
    plain = GPM.GP().fit(X, y)
    full = GPM.GP().fit(np.hstack([X, np.repeat(dims[None], 40, axis=0)]), y)       # refit with the constant columns
    assert full.lml == pytest.approx(plain.lml) and np.allclose(full.ls[:3], plain.ls)
    aug = plain.augmented(dims, [0.8] * 4)
    Xt = rng.random((30, 3))
    Xt_same = np.hstack([Xt, np.repeat(dims[None], 30, axis=0)])
    assert np.allclose(aug.posterior(Xt_same)[0], plain.posterior(Xt)[0])
    assert np.allclose(full.posterior(Xt_same)[0], plain.posterior(Xt)[0])
    # at another robot's dimension vector the prediction shrinks toward the training mean
    Xt_other = np.hstack([Xt, np.repeat(np.array(F.dims(TANDEM))[None], 30, axis=0)])
    dev_same = np.abs(plain.posterior(Xt)[0] - plain.ym)
    dev_other = np.abs(aug.posterior(Xt_other)[0] - plain.ym)
    assert dev_other.mean() < 0.5 * dev_same.mean()
    assert np.abs(plain.augmented(dims, [50.0] * 4).posterior(Xt_other)[0] - plain.posterior(Xt)[0]).max() < 0.05


def test_rawdim_features_and_synthetic_footprints():
    c = C.make_course(0.94, 0.72, 1.30)
    assert F.rawdim(c, COMPACT) == F.raw(c) + [0.8, 0.72, 0.9, 1.6]
    small, xl = T.load_robot("synth_small"), T.load_robot("synth_xl")
    for a, b, t in zip(F.dims(small), F.dims(xl), F.dims(TANDEM)):
        assert min(a, b) < t < max(a, b)                              # the held-out footprint is bracketed
    for name, n in (("mf_small", 88), ("mf_xl", 72)):
        cs, rb = T.courses_of(name), T.load_robot(T.SETS[name])
        assert len(cs) == n and all(T.admissible(c) for c in cs)
        assert all(M.certificate(c, rb)["verdict"] == "fits" for c in cs)
        sl = F.matrix("slack", cs, rb)
        assert sl[:, 0].min() == pytest.approx(0.08) and sl[:, 2].max() == pytest.approx(0.76)
    assert not T.admissible(C.make_course(1.2, 1.5, 1.3)) and T.admissible(C.make_course(0.8, 1.5, 1.3))


@pytest.fixture(scope="module")
def synth_mf():
    return P.MFData("constructed", _synthetic_rows())


def test_three_footprint_data_and_arms(synth_mf):
    D = synth_mf
    assert D.ready() and [len(t["courses"]) for t in D.train] == [504, 88, 72]
    assert D.X[("rawdim", "train")].shape == (664, 7) and D.X[("mech", "transfer")].shape == (80, 5)
    Xd = D.X[("rawdim", "transfer")][:, 3:]
    assert Xd.min() > 0 and Xd.max() < 1                               # interpolation in the dimension vector
    o = D.order(1, 0)
    assert len(o) == len(set(o)) == 88 and min(o) >= 504 and max(o) < 592
    always_a = A.wrong_cost(D.delta["transfer"], D.labels["transfer"], np.array(["A"] * 80))
    res = {arm: P.run_mf(D, arm, rep=0, checkpoints=(30, 60)) for arm in P.MF_ARMS}
    for arm, r in res.items():
        assert r[30]["n_actual"] == 30 and r[60]["n_actual"] == 60
        assert 0 <= r[60]["R"] and 0 <= r[60]["Rt"] <= always_a + 4.0
    assert res["mf3-sobol-mech"][60]["Rt"] < 0.5 * always_a            # the made-up flip is at d = 2 rho for every robot


def test_ratio_ci():
    m, lo, hi = P._ratio_ci([1.0] * 10, [2.0] * 10)
    assert (m, lo, hi) == (0.5, 0.5, 0.5)
    assert all(math.isnan(v) for v in P._ratio_ci([1.0], [0.0]))


# ---------------------------------------------------------------- sensitivity overrides, extra analysis

def test_lattice_cfg_override_is_explicit_and_default_neutral():
    c = C.make_course(0.94, 0.72, 1.30)
    base = LN.LatticeFollower(LN.ARM_REVERSE, COMPACT, c.start, c.goal, dict(S.DEFAULT_CFG))
    assert base.cfg == LN.LATTICE_CFG                                   # no lattice_ key: defaults untouched
    over = LN.LatticeFollower(LN.ARM_REVERSE, COMPACT, c.start, c.goal, dict(S.DEFAULT_CFG, lattice_pad=0.0))
    assert over.cfg["pad"] == 0.0 and LN.LATTICE_CFG["pad"] == 0.025    # module default not mutated
    assert {k: v for k, v in over.cfg.items() if k != "pad"} == {k: v for k, v in LN.LATTICE_CFG.items() if k != "pad"}
    with pytest.raises(KeyError):
        LN.LatticeFollower(LN.ARM_REVERSE, COMPACT, c.start, c.goal, dict(S.DEFAULT_CFG, lattice_no_such_key=1))


def test_sensitivity_table_counts():
    import sensitivity as SN
    def row(e, d, ja, jb, sa, sb):
        return {"variant": "default", "e_cm": e, "d_cm": d, "J_A": ja, "J_B": jb, "status_A": sa, "status_B": sb,
                "reversals_B": 0}
    done = {("default", 43, 119): row(43, 119, 150.0, 20.0, "stuck", "success"),
            ("default", 43, 125): row(43, 125, 10.0, 10.5, "success", "success"),
            ("default", 0, 84): row(0, 84, 3.0, 3.0, "success", "success"),
            ("default", 64, 84): row(64, 84, 150.0, 150.0, "stuck", "stuck")}
    line = SN.table(done).splitlines()[2]
    assert [s.strip() for s in line.strip("|").split("|")] == ["default", "4", "2", "3", "1", "0", "1", "1.25 / 1.19"]


def test_extras_ties_and_effects_on_synthetic_results():
    import extras as X
    rng = np.random.default_rng(0)
    res = {a: {n: {"R": list(5.0 + rng.normal(0, 0.1, 20))} for n in P.CHECKPOINTS} for a in P.ARMS}
    for n in P.CHECKPOINTS:
        res["sobol-raw"][n]["R"] = list(np.array(res["sobol-raw"][n]["R"]) - 3.0)     # clearly best
    out = []
    X.ties(res, out)
    X.effects(res, out)
    rows = [l for l in out if l.startswith("| 50 ")]
    assert rows and "| sobol-raw |" in rows[0] and rows[0].rstrip(" |").endswith("| 1 | yes")
    mech = [l for l in out if l.startswith("| mechanics - raw")][0]
    assert float(mech.split("|")[2].split("[")[0]) == pytest.approx(1.0, abs=0.15)   # (3 + 0 + 0) / 3
