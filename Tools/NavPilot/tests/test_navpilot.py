# SPDX-License-Identifier: GPL-3.0-or-later
"""Run: fsrun python -I -m unittest discover -s Tools/NavPilot/tests -p 'test_*.py'"""
import math
import os
import sys
import tempfile
import unittest

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.normpath(os.path.join(HERE, "..")))
sys.path.insert(0, os.path.normpath(os.path.join(HERE, "..", "..", "DroneData")))

from navpilot import sensors as S  # noqa: E402
from navpilot.episode import default_config, run_episode  # noqa: E402
from navpilot.forest import HeldoutRefused, World, family_spec, generate_stand, manifest_sha256  # noqa: E402
from navpilot.planner import OccMap, plan, stop_distance  # noqa: E402
from navpilot.vehicle import braking_authority, load_params  # noqa: E402


class Determinism(unittest.TestCase):
    def test_stand_is_seeded(self):
        a = generate_stand(7, family_spec(branch="mid"))
        b = generate_stand(7, family_spec(branch="mid"))
        c = generate_stand(8, family_spec(branch="mid"))
        self.assertEqual(manifest_sha256(a), manifest_sha256(b))
        self.assertNotEqual(manifest_sha256(a), manifest_sha256(c))

    def test_branch_density_does_not_move_stems(self):
        a = generate_stand(7, family_spec(branch="low"))
        b = generate_stand(7, family_spec(branch="high"))
        self.assertEqual(a["stems"], b["stems"])

    def test_episode_repeats_exactly(self):
        w = World(generate_stand(5, family_spec(branch="mid")))
        cfg = default_config(sensor="fusion", v_limit=6.0, latency_tail="heavy")
        r1, r2 = run_episode(w, cfg, 11), run_episode(w, cfg, 11)
        self.assertEqual(r1, r2)


class StoppingRule(unittest.TestCase):
    def test_planner_never_exceeds_rule_in_belief(self):
        veh = load_params()
        rng = np.random.default_rng(0)
        checked = 0
        for seed in range(3):
            w = World(generate_stand(seed, family_spec(stem="dense", branch="high")))
            m = OccMap(w.stand["corridor"]["bounds"], w.half_width)
            for k in range(40):
                x, y = rng.uniform(2, 50), rng.uniform(-8, 8)
                yaw = rng.uniform(-math.pi, math.pi)
                fr = S.lidar_frame(w, 0.0, x, y, yaw, rng) if k % 2 else S.stereo_frame(w, 0.0, x, y, yaw, 0, 0, 0, rng)
                m.integrate(fr, 0.1 * k)
                vx, vy = rng.normal(0, 4, 2)
                for vl in (2.0, 6.0, 12.0):
                    o = plan(m, x, y, vx, vy, w.goal, vl, veh)
                    self.assertLessEqual(o["v_cmd"], vl + 1e-12)
                    if o["v_cmd"] > 0:
                        v_rule = max(o["v_cmd"], math.hypot(vx, vy))
                        need = stop_distance(v_rule, o["tau"], o["a_brake"], o["margin"])
                        self.assertLessEqual(need, o["d_free_cmd"] + 1e-9)
                        # every primitive flagged feasible satisfies the rule, every other one violates it
                        need_all = stop_distance(np.maximum(o["speeds"], math.hypot(vx, vy)), o["tau"], o["a_brake"], o["margin"])
                        self.assertTrue(np.all(need_all[o["feasible"]] <= o["d_free"][o["feasible"]] + 1e-9))
                        self.assertTrue(np.all(need_all[~o["feasible"]] > o["d_free"][~o["feasible"]]))
                    checked += 1
        self.assertGreater(checked, 300)

    def test_closed_loop_has_no_belief_violations(self):
        w = World(generate_stand(2, family_spec(branch="mid")))
        for sensor in ("stereo", "lidar"):
            r = run_episode(w, default_config(sensor=sensor, v_limit=9.0, latency_tail="severe"), 2)
            self.assertEqual(r["belief_violations"], 0)

    def test_braking_authority_direction_dependent(self):
        p = load_params()
        self.assertGreater(braking_authority(p, 8.0, "brake"), braking_authority(p, 8.0, "lateral"))
        self.assertGreater(braking_authority(p, 8.0, "lateral"), braking_authority(p, 8.0, "accelerate"))
        self.assertAlmostEqual(braking_authority(p, 0.0, "brake"), p["a_h_max"])


class SensorMonotonicity(unittest.TestCase):
    def test_stereo_miss_rises_as_diameter_shrinks_and_range_grows(self):
        D = np.array([0.005, 0.01, 0.02, 0.03, 0.05, 0.1, 0.3])
        R = np.array([0.5, 1, 2, 4, 6, 10, 15.0])
        for r in R:
            p = S.stereo_p_hit(D, r)
            self.assertTrue(np.all(np.diff(1 - p) <= 1e-15), "miss must fall with diameter at r=%g" % r)
        for d in D:
            p = S.stereo_p_hit(d, R)
            self.assertTrue(np.all(np.diff(1 - p) >= -1e-15), "miss must rise with range at D=%g" % d)

    def test_stereo_motion_blur_raises_miss(self):
        v = np.array([0, 2, 5, 10, 15.0])
        p = S.stereo_p_hit(0.02, 2.0, v_lat=v)
        self.assertTrue(np.all(np.diff(p) <= 1e-15))
        self.assertLess(S.stereo_p_hit(0.02, 2.0, yaw_rate=2.0), S.stereo_p_hit(0.02, 2.0))

    def test_lidar_miss_monotone(self):
        D = np.array([0.005, 0.01, 0.02, 0.05])
        R = np.array([1, 3, 6, 12, 20.0])
        for r in R:
            self.assertTrue(np.all(np.diff(1 - S.lidar_p_hit(D, 1.0, r)) <= 0))
        for d in D:
            self.assertTrue(np.all(np.diff(1 - S.lidar_p_hit(d, 1.0, R)) >= 0))

    def test_detection_monotone(self):
        rs = [3, 6, 10, 20, 30]
        p = [S.detection_p(r, 0.8, 1.0, 0.0, 0.0) for r in rs]
        self.assertTrue(all(a >= b for a, b in zip(p, p[1:])))
        self.assertGreater(S.detection_p(5, 0.8, 1.0, 0.0, 0.0), S.detection_p(5, 0.8, 1.0, 8.0, 0.0))
        self.assertGreater(S.detection_p(5, 0.8, 1.0, 0.0, 0.0), S.detection_p(5, 0.8, 0.4, 0.0, 0.0))

    def test_latency_tails_ordered(self):
        q = [S.latency_quantiles(t, n=5000)["p99"] for t in ("nominal", "heavy", "severe")]
        self.assertTrue(q[0] < q[1] < q[2])


class SplitGuard(unittest.TestCase):
    def test_heldout_refused_without_flag(self):
        with self.assertRaises(HeldoutRefused):
            generate_stand(90001)
        with self.assertRaises(HeldoutRefused):
            generate_stand(99999)

    def test_heldout_allowed_with_flag(self):
        m = generate_stand(90001, allow_heldout=True)
        self.assertEqual(m["split"], "heldout")

    def test_unassigned_and_heldout_wind_refused(self):
        with self.assertRaises(HeldoutRefused):
            generate_stand(123456)
        with self.assertRaises(HeldoutRefused):
            generate_stand(5, wind="storm_gust")

    def test_dev_and_val(self):
        self.assertEqual(generate_stand(5)["split"], "dev")
        self.assertEqual(generate_stand(70001)["split"], "val")

    def test_sweep_cli_refuses_heldout(self):
        import sweep
        with self.assertRaises(HeldoutRefused):
            sweep.layout_seeds("heldout", 2, allow_heldout=False)
        self.assertEqual(len(sweep.layout_seeds("heldout", 2, allow_heldout=True)), 2)


class LogFormat(unittest.TestCase):
    def test_log_validates(self):
        import drone_log
        w = World(generate_stand(4, family_spec(branch="low")))
        with tempfile.TemporaryDirectory() as d:
            run_episode(w, default_config(sensor="lidar", v_limit=5.0), 4, log_dir=d)
            lg = drone_log.DroneLog(d)
            self.assertEqual(lg.validate(), [])
            self.assertEqual(lg.manifest["split"], "dev")


if __name__ == "__main__":
    unittest.main()
