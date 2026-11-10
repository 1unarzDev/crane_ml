# SPDX-License-Identifier: GPL-3.0-or-later
"""Run: ~/.cache/research/fs-venv/bin/python -m unittest discover -s Tools/Frontier/mf -p 'test_*.py'"""
import os
import sys
import unittest

import numpy as np

sys.dont_write_bytecode = True
HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
import analyze  # noqa: E402
import data  # noqa: E402
import replay  # noqa: E402
import synth  # noqa: E402

COST = {"L1": 1.0, "L2": 30.0}


def tiny_table(seed=0, rho=1.0, N=12):
    cfgs = [("opt_backup_t", 4.0 + 0.2 * i, 0) for i in range(N)]
    f = np.linspace(0, 1, N)
    p = np.zeros(N)                         # no collisions: utility = progress rate, increasing with the index
    a = 2.0 + 2.0 * f                       # config N-1 is best at both tiers; the single-run noise is small
    sd = np.full(N, 0.2)
    return synth.Table(cfgs, p, a, sd, p, a, 30, 30, seed)


class ReplayTests(unittest.TestCase):
    def test_env_charges_and_exhausts(self):
        t = tiny_table()
        env = replay.Env(t, COST, np.random.default_rng(0))
        ys = [env.query(0, "L2") for _ in range(30)]
        self.assertAlmostEqual(env.spent, 30 * 30.0)
        self.assertEqual(sorted(ys), sorted(t.pools[(0, "L2")].tolist()))   # without replacement, all cached values returned
        self.assertIsNone(env.query(0, "L2"))
        self.assertAlmostEqual(env.spent, 900.0)                            # exhausted pool charges nothing
        env.query(1, "L1")
        self.assertAlmostEqual(env.spent, 901.0)

    def test_selectors_respect_budget_and_regret_nonneg(self):
        t = tiny_table(1)
        for name in replay.SELECTORS:
            env = replay.Env(t, COST, np.random.default_rng(3))
            sel = replay.make(name, t.N, np.random.default_rng(4), t)
            out, h = replay.run(sel, env, 600.0, [0.05, 0.1, 0.2])
            self.assertLessEqual(env.spent, 600.0 + 8 * 30.0 + 1e-6, name)   # at most one batch over
            self.assertTrue(all(v >= -1e-9 for v in out.values()), name)

    def test_selectors_find_best_on_easy_problem(self):
        t = tiny_table(2)
        best = {}
        for name in ("all-high", "succ-halving", "screen+topk", "AR1-BLR", "sh-seeds-L2"):
            env = replay.Env(t, COST, np.random.default_rng(5))
            sel = replay.make(name, t.N, np.random.default_rng(6), t)
            out, _ = replay.run(sel, env, 3 * 3600, [3.0])
            best[name] = out[3.0]
        for name, r in best.items():
            self.assertLess(r, 0.5, name)   # utility range is 2.0, per-run sd 0.2

    def test_low_only_plateaus_at_face_value(self):
        # tier 1 inverted relative to tier 2: trusting the cheap tier must keep a large regret forever
        N = 10
        cfgs = [("opt_backup_t", 4.0 + 0.2 * i, 0) for i in range(N)]
        f = np.linspace(0, 1, N)
        p1, p2 = 0.05 + 0.25 * f, 0.30 - 0.25 * f
        a = np.full(N, 3.0)
        t = synth.Table(cfgs, p1, a, np.full(N, 0.3), p2, a, 40, 30, 0)
        env = replay.Env(t, COST, np.random.default_rng(0))
        sel = replay.make("low-only", N, np.random.default_rng(0), t)
        out, h = replay.run(sel, env, 6 * 3600, [6.0])
        self.assertGreater(out[6.0], 1.0)
        self.assertLess(h, 1.0)             # and it is cheap: the pool is exhausted early

    def test_determinism(self):
        r = []
        for _ in range(2):
            t = tiny_table(7)
            env = replay.Env(t, COST, np.random.default_rng(1))
            sel = replay.make("AR1-BLR", t.N, np.random.default_rng(2), t)
            r.append(replay.run(sel, env, 1800.0, [0.5])[0][0.5])
        self.assertEqual(r[0], r[1])


class AnalysisTests(unittest.TestCase):
    def test_kendall(self):
        self.assertAlmostEqual(synth.kendall([1, 2, 3, 4], [10, 20, 30, 40]), 1.0)
        self.assertAlmostEqual(synth.kendall([1, 2, 3, 4], [4, 3, 2, 1]), -1.0)
        self.assertAlmostEqual(analyze.kendall([1, 2, 3, 4], [1, 3, 2, 4]), synth.kendall([1, 2, 3, 4], [1, 3, 2, 4]))

    def test_tau_ci_contains_point_and_width_shrinks_with_n(self):
        rng = np.random.default_rng(0)
        w = {}
        for n in (10, 60):
            z = rng.normal(size=(n, 2))
            z[:, 1] += z[:, 0]
            r = analyze.tau_ci(z[:, 0], z[:, 1], B=400)
            self.assertLessEqual(r["ci95"][0], r["tau"] + 0.2)
            w[n] = r["ci95"][1] - r["ci95"][0]
        self.assertGreater(w[10], w[60])

    @unittest.skipUnless(os.path.isdir(data.RAW), "NavPilot rows not present")
    def test_logged_row_counts(self):
        self.assertEqual(len(data.navpilot_e3("heldout")), 800)
        self.assertEqual({r["split"] for r in data.navpilot_e3("heldout")}, {"RETRO-HELDOUT"})   # always labelled
        self.assertEqual(len(data.navpilot_e3("tune")), 480)

    def test_synth_backbone_and_determinism(self):
        if not os.path.isdir(data.RAW):
            self.skipTest("no rows")
        bb = synth.e3_backbone(data.navpilot_e3("heldout"))
        cf, p1, a1, sd = synth.build_configs(bb)
        self.assertEqual(len(cf), 135)
        a = synth.build_truth(cf, p1, a1, "moderate", 3)
        b = synth.build_truth(cf, p1, a1, "moderate", 3)
        self.assertTrue((a[0] == b[0]).all())
        # faithful transfers ranks better than unfaithful
        f1 = synth.utility(p1, a1)
        tf = synth.kendall(f1, synth.utility(*synth.build_truth(cf, p1, a1, "faithful", 0)))
        tu = synth.kendall(f1, synth.utility(*synth.build_truth(cf, p1, a1, "unfaithful", 0)))
        self.assertGreater(tf, tu + 0.3)


if __name__ == "__main__":
    unittest.main()
