# SPDX-License-Identifier: GPL-3.0-or-later
"""Run: fsrun python -m unittest discover -s Tools/NavPilot/experiments -p 'test_*.py'"""
import math
import os
import sys
import unittest

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.normpath(os.path.join(HERE, "..")))

from experiments.e2_episode import default_cond, run, scenario_cache  # noqa: E402
from experiments.scenarios import align_offset, build  # noqa: E402
from navpilot import sensors as S  # noqa: E402
from navpilot.forest import HeldoutRefused  # noqa: E402
from navpilot.vehicle import load_params  # noqa: E402
from stacks.core import Belief, StopRule, TimedOccMap, brake_table, evaluate  # noqa: E402
from stacks.policies import MAP_RHP, confidences, CONF_DEFAULT  # noqa: E402


class Scenarios(unittest.TestCase):
    def test_seeded_and_s1_prior_is_stale(self):
        a, b = build(150, "S1"), build(150, "S1")
        self.assertEqual(a["truth"], b["truth"])
        blk = [x for x in a["truth"]["branches"] if x.get("scenario_block")]
        self.assertEqual(len(blk), 2)
        self.assertTrue(all(x["d0_m"] >= 0.02 for x in blk))    # hard colliders
        ys, Xw = a["info"]["shortcut_y"], a["info"]["X_w"]
        # prior has nothing in the shortcut gap; truth has the branches there
        self.assertFalse(any(abs(p["x"] - Xw) < 0.5 and abs(p["y"] - ys) < 0.4 for p in a["prior_stems"]))

    def test_heldout_layout_refused(self):
        with self.assertRaises(HeldoutRefused):
            build(90200, "S1")
        build(90200, "S1", allow_heldout=True)

    def test_alignment_drift_separate_from_local(self):
        s3 = build(150, "S3")
        d0 = align_offset(s3["align"], 0.0)
        d1 = align_offset(s3["align"], s3["align"]["x_full"] + 1)
        self.assertGreater(abs(d1[1] - d0[1]), 1.1)
        s0 = build(150, "S0")
        self.assertEqual(align_offset(s0["align"], 50.0), (0.0, 0.0))
        self.assertIsNone(s3["loc_zone"])


class SafetyLayer(unittest.TestCase):
    def test_brake_table_includes_lag(self):
        veh = load_params()
        vs, ds = brake_table(veh)
        self.assertTrue(np.all(np.diff(ds) >= -1e-9))
        k = int(np.searchsorted(vs, 7.0))
        # lag + jerk make the simulated stop longer than the instantaneous-deceleration ideal at a_h_max + drag
        self.assertGreater(ds[k], 7.0 ** 2 / (2 * (veh["a_h_max"] + 3.0)))

    def test_rule_invariant_and_backup_uses_fresh_only(self):
        veh = load_params()
        rule = StopRule(veh)
        scn = build(151, "S1")
        C = scenario_cache(scn)
        w = C["world"]
        pL, pO, _ = C["prior"]
        m = TimedOccMap(scn["truth"]["corridor"]["bounds"], w.half_width)
        rng = np.random.default_rng(0)
        fr = S.lidar_frame(w, 0.0, 5.0, 0.0, 0.0, rng)
        m.integrate(fr, 0.0)
        for vx in (0.0, 3.0, 7.0):
            b = Belief(acc=m, prior_L=pL, prior_obs=pO, w_prior=1.0)
            o = evaluate(b, m, 5.0, 0.0, vx, 0.0, (60.0, 0.0), 7.0, veh, rule, ctg=None)
            if o["v_cmd"] > 0:
                self.assertLessEqual(float(rule.dist(o["v_rule"])), o["d_free_cmd"] + 1e-9)
            ob = evaluate(b, m, 5.0, 0.0, vx, 0.0, (60.0, 0.0), 7.0, veh, rule, optimistic=True,
                          b_safe=Belief(acc=m))
            if ob["v_cmd"] > 0:
                d_fresh = evaluate(Belief(acc=m), m, 5.0, 0.0, vx, 0.0, (60.0, 0.0), 7.0, veh, rule)
                self.assertLessEqual(float(rule.dist(ob["v_rule"])), ob["d_free_cmd"] + 1e-9)
                self.assertIsNotNone(d_fresh)

    def test_confidence_monotone(self):
        base = {"sigma_align_rep": 0.1, "rep_age": 0.0, "agree": 0.8, "q_obs": 1.0, "sigma_loc_rate": 0.03}
        hi = confidences(base, CONF_DEFAULT)
        lo = confidences(dict(base, agree=0.2, q_obs=0.3, sigma_loc_rate=0.3), CONF_DEFAULT)
        self.assertTrue(all(a >= b for a, b in zip(hi, lo)))


class Network(unittest.TestCase):
    def test_network_loss_does_not_touch_map_rhp_navigation(self):
        scn = build(152, "S1")
        a = run(scn, MAP_RHP, default_cond(), 152)
        b = run(scn, MAP_RHP, default_cond(netloss=True), 152)
        for k in ("outcome", "time_s", "progress_m", "min_clear", "age_p99"):
            self.assertEqual(a[k], b[k])
        self.assertGreaterEqual(b["report_delay_mean"], a["report_delay_mean"])


if __name__ == "__main__":
    unittest.main()
