"""Kinematic surrogate, lidar model, controller stand-ins and grid CLI (EXPLORATORY)."""
import inspect
import json
import math
import os

import numpy as np
import pytest

import course as C
import geometry as G
import mechanics as M
import robots as R
import surrogate as S

COMPACT = R.load_robot("compact_diff")


def test_score_constants_are_explicit():
    assert (S.DT, S.T_CAP, S.PEN_COLLISION, S.PEN_TIMEOUT) == (0.05, 120.0, 60.0, 30.0)
    assert S.score(S.SUCCESS, 12.5) == 12.5 and S.score(S.SUCCESS, 500.0) == S.T_CAP
    assert S.score(S.COLLISION, 3.0) == 180.0 and S.score(S.TIMEOUT, 120.0) == 150.0 == S.score(S.STUCK, 9.0)


def test_noise_stream_is_counter_based():
    a = S.stream_normals(7, 1, 42, 16)
    assert np.array_equal(a, S.stream_normals(7, 1, 42, 16))
    assert not np.array_equal(a, S.stream_normals(7, 1, 43, 16))
    assert not np.array_equal(a, S.stream_normals(8, 1, 42, 16))
    assert not np.array_equal(a, S.stream_normals(7, 2, 42, 16))
    big = S.stream_normals(3, 1, 0, 20000)
    assert abs(big.mean()) < 0.03 and abs(big.std() - 1) < 0.03


def test_lidar_ranges_noise_and_pairing():
    c = C.make_course(0.94, 0.72, 1.30)
    sim = S.BatchSim(c, COMPACT, [5, 5, 6])
    r = sim.scan()
    assert r.shape == (3, 720)
    assert np.array_equal(r[0], r[1]) and not np.array_equal(r[0], r[2])      # same seed -> same noise
    true = G.raycast(sim.pose[:, :2], sim.pose[:, 2:3] + sim.beam[None, :], sim.boxes, sim.max_range)
    assert true[0, 0] == pytest.approx(c.d - c.start[0])                       # beam 0 looks at the door wall
    assert true[0, 360] == pytest.approx(c.start[0])                           # beam 360 at the back wall
    err = r[0] - true[0]
    assert abs(err.std() - COMPACT.lidar["noise_sigma"]) < 0.002 and abs(err.mean()) < 0.002


def test_velocity_and_acceleration_limits():
    c = C.make_course(0.94, 0.0, 3.0)
    sim = S.BatchSim(c, COMPACT, [1])
    vs = []
    for _ in range(30):
        sim.step(np.array([[9.0, 5.0, 9.0]]))
        vs.append(sim.vel[0].copy())
    vs = np.array(vs)
    assert np.all(np.diff(vs[:, 0]) <= COMPACT.acc * S.DT + 1e-12)
    assert vs[0, 0] == pytest.approx(COMPACT.acc * S.DT)
    assert vs[:, 0].max() == pytest.approx(COMPACT.v_max) and vs[:, 2].max() == pytest.approx(COMPACT.omega_max)
    assert np.all(vs[:, 1] == 0.0)                                             # differential: no lateral speed


def test_collision_and_timeout_statuses():
    c = C.make_course(0.94, 1.0, 1.30)
    sim = S.BatchSim(c, COMPACT, [1, 2])
    while sim.running.any():
        sim.step(np.array([[0.5, 0, 0], [0, 0, 0]]))
    assert S.STATUS_NAMES[int(sim.status[0])] == "collision" and sim.t_end[0] < 3.0
    # stopped by the door wall: nose within one step of it
    assert c.d - 0.03 < sim.pose[0, 0] + COMPACT.L / 2 < c.d + 0.03
    assert S.STATUS_NAMES[int(sim.status[1])] == "timeout" and sim.t_end[1] == pytest.approx(S.T_CAP)
    # a start pose that does not fit is a collision at t = 0
    assert (S.BatchSim(C.make_course(0.94, 0.5, 0.70), COMPACT, [1]).status == S.COLLISION).all()


def test_odometry_drifts_but_stays_close():
    c = C.make_course(0.94, 0.0, 3.0)
    sim = S.BatchSim(c, COMPACT, [1])
    for _ in range(40):
        sim.step(np.array([[0.4, 0, 0.2]]))
    diff = np.abs(sim.odom[0] - sim.pose[0])
    assert 0 < diff.max() < 0.02


def test_free_straight_matches_brute_force():
    rng = np.random.default_rng(0)
    hl, hw, mf, ms = 0.4, 0.36, 0.02, 0.01
    for _ in range(50):
        u, n = rng.uniform(-2, 2, 40), rng.uniform(-2, 2, 40)
        keep = ~((np.abs(u) < hl + 0.05) & (np.abs(n) < hw + 0.05))
        u, n = u[keep], n[keep]
        got = float(S.free_straight(u, n, hl, hw, mf, ms))
        s = np.arange(0, 3.0, 0.002)
        inside = (np.abs(u[None, :] - s[:, None]) < hl + mf) & (np.abs(n[None, :]) < hw + ms)
        bad = np.flatnonzero(inside.any(1))
        brute = s[bad[0]] if bad.size else np.inf
        assert (np.isinf(got) and np.isinf(brute)) or abs(got - brute) < 0.004


def test_free_rotation_matches_brute_force():
    rng = np.random.default_rng(1)
    hl, hw, m = 0.4, 0.36, 0.02
    for _ in range(60):
        u, n = rng.uniform(-0.8, 0.8, 12), rng.uniform(-0.8, 0.8, 12)
        keep = ~((np.abs(u) < hl + m + 0.01) & (np.abs(n) < hw + m + 0.01))
        u, n = u[keep], n[keep]
        for sign in (1, -1):
            got = S.free_rotation(u, n, hl, hw, m, 0.005, sign)
            d = np.arange(0, math.pi, 0.001)
            cu = u[None, :] * np.cos(sign * d)[:, None] + n[None, :] * np.sin(sign * d)[:, None]
            cn = -u[None, :] * np.sin(sign * d)[:, None] + n[None, :] * np.cos(sign * d)[:, None]
            bad = np.flatnonzero(((np.abs(cu) < hl + m) & (np.abs(cn) < hw + m)).any(1))
            brute = d[bad[0]] if bad.size else math.pi
            assert abs(got - brute) < 0.003


def test_free_rotation_in_a_strip_reproduces_alpha_max():
    L, W = COMPACT.L, COMPACT.W
    d = L + 0.2
    y = np.linspace(-3, 3, 4001)
    u = np.concatenate([np.full(y.size, d / 2), np.full(y.size, -d / 2)])
    n = np.concatenate([y, y])
    assert S.free_rotation(u, n, L / 2, W / 2, 0.0, 0.0, +1) == pytest.approx(M.alpha_max(d, L, W), abs=2e-3)


def test_controllers_see_only_lidar_and_odometry():
    params = list(inspect.signature(S.Controller.__init__).parameters)
    assert params == ["self", "kind", "robot", "start", "goal", "cfg"]
    assert list(inspect.signature(S.Controller.on_scan).parameters) == ["self", "ranges", "odom", "t"]
    assert list(inspect.signature(S.Controller.command).parameters) == ["self", "odom", "twist", "t"]
    ctl = S.Controller("rotate_then_follow", COMPACT, (0.5, 0, 0), (2.4, 0.7), dict(S.DEFAULT_CFG))
    assert not any(isinstance(v, C.Course) for v in vars(ctl).values())


def test_planner_finds_the_door_from_observed_points_only():
    c = C.make_course(0.94, 0.72, 1.30)
    sim = S.BatchSim(c, COMPACT, [3])
    ctl = S.Controller("rotate_then_follow", COMPACT, c.start, c.goal, dict(S.DEFAULT_CFG))
    ctl.on_scan(sim.scan()[0], sim.odom[0], 0.0)
    assert ctl.path is not None
    p = ctl.path
    in_wall = (p[:, 0] > c.d) & (p[:, 0] < c.d + c.t)
    assert in_wall.any() and np.all(np.abs(p[in_wall, 1] - c.e) < c.w / 2 - COMPACT.W / 2 + 0.06)
    assert np.hypot(*(p[-1] - np.array(c.goal))) < 0.08


@pytest.fixture(scope="module")
def roomy_and_tight():
    roomy = S.paired_gap(C.course_from_ratios(COMPACT, 0.85, 1.0), COMPACT, [11, 12])
    tight = S.paired_gap(C.course_from_ratios(COMPACT, 0.38, 0.5), COMPACT, [11, 12])
    return roomy, tight


def test_roomy_bay_both_succeed_and_no_reverse_follower_is_faster(roomy_and_tight):
    roomy, _ = roomy_and_tight
    assert roomy["label"] == "exploratory-surrogate"
    assert roomy["arm_A"] == "rotate_then_follow" and roomy["arm_B"] == "stroke_shimmy"
    assert roomy["status_A"] == ["success"] * 2 and roomy["status_B"] == ["success"] * 2
    assert all(g < 0 for g in roomy["gap"])                    # A wins (by the declared speed constants)
    assert roomy["gap"] == [round(a - b, 3) for a, b in zip(roomy["J_A"], roomy["J_B"])]


def test_tight_bay_no_reverse_follower_fails_and_shimmy_succeeds(roomy_and_tight):
    _, tight = roomy_and_tight
    assert all(s in ("stuck", "timeout") for s in tight["status_A"])
    assert tight["J_A"] == [S.T_CAP + S.PEN_TIMEOUT] * 2
    assert tight["status_B"] == ["success"] * 2 and all(r >= 1 for r in tight["reversals_B"])
    assert all(g > 0 for g in tight["gap"])


def test_paired_gap_is_deterministic():
    c = C.course_from_ratios(COMPACT, 0.85, 0.5)
    a, b = S.paired_gap(c, COMPACT, [21]), S.paired_gap(c, COMPACT, [21])
    assert a == b
    one = S.run_batch(c, COMPACT, ["rotate_then_follow"], [21])[0]
    assert one["J"] == a["J_A"][0]                              # batching does not change an episode


@pytest.mark.skipif(not os.path.exists(R.FLEET_PROFILE_PATH), reason="fleet profile not present")
def test_mecanum_holonomic_passes_a_bay_too_tight_to_turn_in():
    mec = R.load_robot("mecanum")
    c = C.course_from_ratios(mec, 0.2, 1.0)
    assert c.d < 2 * mec.rho and M.certificate(c, mec)["verdict"] == "fits"
    g = S.paired_gap(c, mec, [31])
    assert (g["arm_A"], g["arm_B"]) == ("holonomic", "rotate_then_follow")
    assert g["status_A"] == ["success"] and g["status_B"] != ["success"] and g["gap"][0] < 0


def test_grid_cli_is_resumable_and_summarises(tmp_path):
    out = str(tmp_path / "g.jsonl")
    n1 = S.run_grid("compact_diff", [0.85], [0.25], [41], out, log=lambda *_: None)
    with open(out, "a") as f:
        f.write('{"key": "torn')                               # interrupted write
    n2 = S.run_grid("compact_diff", [0.85], [0.25], [41], out, log=lambda *_: None)
    assert (n1, n2) == (1, 0)
    rows = S.load_grid(out)
    assert len(rows) == 1 and rows[0]["label"] == "exploratory-surrogate"
    assert len(rows[0]["sources"]["fleet_profile_sha256"]) in (64, 7)
    assert rows[0]["features"]["certificate"]["verdict"] == "fits"
    text = S.summarize(rows)
    assert "EXPLORATORY" in text and "both stand-ins fail" in text
    assert S.classify(rows[0], margin=1.0) in ("A_wins", "B_wins", "indifferent", "both_fail")
