# SPDX-License-Identifier: GPL-3.0-or-later
"""Unit tests for Tools/CraneHarness (no simulation): bootstrap, pairing, DR registry vs splits, stand guard.
   python -I -m unittest discover -s Tools/CraneHarness -p 'test_*.py'"""
import json
import os
import sys
import tempfile
import unittest

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
import aggregate as A  # noqa: E402


def row(mission, speed, ok, seed=1):
    return {"run_id": "%s-%s-%d" % (mission, speed, seed), "pair_key": "f1|%s|r%d|d0" % (mission, seed), "cluster": "f1|%s" % mission,
            "speed": speed, "outcome": "success" if ok else "collision", "label": "" if ok else "brake_infeasible"}


class Bootstrap(unittest.TestCase):
    def test_cluster_bootstrap_bounds_and_determinism(self):
        rows = [row("m%d" % i, 4, i % 3 != 0) for i in range(12)]
        e1, c1 = A.cluster_bootstrap(rows, A.rate(A.is_success), B=500)
        e2, c2 = A.cluster_bootstrap(rows, A.rate(A.is_success), B=500)
        self.assertEqual((e1, c1), (e2, c2))
        self.assertAlmostEqual(e1, 8 / 12)
        self.assertTrue(0 <= c1[0] <= e1 <= c1[1] <= 1)

    def test_clusters_widen_the_interval(self):
        # 4 clusters x 5 identical runs: clustered CI must be wider than treating 20 runs as independent clusters
        rows = [dict(row("m%d" % (i // 5), 4, (i // 5) % 2 == 0), pair_key="k%d" % i) for i in range(20)]
        _, cc = A.cluster_bootstrap(rows, A.rate(A.is_success), B=1000)
        indep = [dict(r, cluster=r["pair_key"]) for r in rows]
        _, ci = A.cluster_bootstrap(indep, A.rate(A.is_success), B=1000)
        self.assertGreater(cc[1] - cc[0], ci[1] - ci[0])

    def test_paired_difference_uses_common_keys_only(self):
        a = [row("m%d" % i, 2, True) for i in range(6)]
        b = [row("m%d" % i, 6, i >= 3) for i in range(6)] + [row("extra", 6, False)]
        est, ci, n = A.paired_cluster_bootstrap(a, b, A.is_success, B=500)
        self.assertEqual(n, 6)
        self.assertAlmostEqual(est, -0.5)
        self.assertLessEqual(ci[1], 0.0 + 1e-12)


class Registry(unittest.TestCase):
    def setUp(self):
        self.f = json.load(open(os.path.join(HERE, "dr_families.json")))
        self.s = json.load(open(os.path.join(HERE, "..", "DroneData", "splits.json")))

    def test_families_belong_to_splits_and_avoid_reserved(self):
        splits = {r["split"] for r in self.s["seed_ranges"]}
        heldout_vals = {(d["family"], v) for d in self.s["disturbance"] if d["split"] == "heldout" for v in d["values"]}
        for name, fam in self.f["families"].items():
            self.assertIn(fam["split"], splits, name)
            if fam["split"] == "heldout":
                continue
            for w in fam.get("wind", []):
                self.assertNotIn(("wind", w), heldout_vals, name)
            for l in fam.get("lighting", []):
                self.assertNotIn(("lighting", l), heldout_vals, name)
            for dim, d in fam["dims"].items():
                if dim in self.f["heldout_reserved"] and "lo" in d:
                    lo, hi = self.f["heldout_reserved"][dim]
                    self.assertTrue(d["hi"] < lo or d["lo"] > hi, "%s.%s overlaps reserved" % (name, dim))

    def test_every_split_wind_has_a_preset(self):
        for d in self.s["disturbance"]:
            if d["family"] == "wind":
                for v in d["values"]:
                    self.assertIn(v, self.f["wind_presets"], v)


class StandGuard(unittest.TestCase):
    def test_bridge_refuses_heldout_stand(self):
        try:
            import navpilot_bridge as NB
        except Exception as e:  # numpy missing in a bare interpreter
            self.skipTest("navpilot imports unavailable: %s" % e)
        st = {"schema": "crane-navpilot-stand/1", "seed": 95000, "split": "heldout", "corridor": {"length_m": 10, "half_width_m": 5,
              "start": [0, 0], "goal": [10, 0], "bounds": [-6, 18, -9, 9]}, "stems": [], "branches": [], "targets": []}
        with tempfile.NamedTemporaryFile("w", suffix=".json", delete=False) as f:
            json.dump(st, f)
        with self.assertRaises(NB.guard_seed.__globals__["HeldoutRefused"]):
            NB.load_stand(f.name)
        os.unlink(f.name)


if __name__ == "__main__":
    unittest.main()
