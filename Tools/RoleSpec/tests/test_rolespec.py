# SPDX-License-Identifier: GPL-3.0-or-later
"""RoleSpec tests: determinism, budget matching, no future information, role-free equivalence, split guard."""
import os
import sys
import unittest

import numpy as np

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), ".."))
from rolespec import policies as P, sensing as S, sim, world as W  # noqa: E402

SC = W.Scenario(n_drones=3, cells_per_drone=16, occlusion="high", trade="strong", comms="outage", deadline_s=600.0,
                battery_wh=80.0)
SC_I = W.with_(SC, comms="ideal")
DROP = ("policy", "trace")


def strip(r):
    return {k: v for k, v in r.items() if k not in DROP}


ALL = [("HOMO_FAST", {}), ("HOMO_THOROUGH", {}), ("HOMO_SHARED_REVISIT", {}), ("SPEC_FIXED", {"r": 0.67}),
       ("SPEC_ADAPTIVE", {}), ("SPEC_ADAPTIVE", {"rule": "marginal"}), ("B3_JIN_CONFIRM", {}),
       ("B6_AUCTION", {}), ("B7_SOLO", {}), ("ORACLE", {})]


class TestRoleSpec(unittest.TestCase):
    def test_guard(self):
        for s in (5000, 5019, 90000):
            with self.assertRaises(W.HeldoutRefused):
                W.World(s, SC)
        W.World(199, SC)

    def test_tables(self):
        w = W.World(1, SC)
        T = w.pub.tables
        self.assertTrue(np.allclose(T.P.sum(-1), 1.0))
        self.assertTrue((T.pd[:, S.FAST] <= T.pd[:, S.SLOW] + 1e-12).all())
        self.assertTrue((T.scan_t[:, S.FAST] < T.scan_t[:, S.SLOW]).all())
        Tn = W.World(1, W.with_(SC, trade="none")).pub.tables
        self.assertTrue(np.allclose(Tn.pd[:, 0], Tn.pd[:, 1]))
        # a high alert is evidence for presence on a fresh view
        self.assertTrue((T.P[:, 1, 1, 2] > T.P[:, 5, 1, 2]).all())

    def test_determinism(self):
        for name, kw in ALL:
            for sc in (SC, SC_I):
                a = sim.run_mission(3, sc, P.make(name, **kw), record=True)
                b = sim.run_mission(3, sc, P.make(name, **kw), record=True)
                self.assertEqual(a, b, name)

    def test_budget_matching(self):
        ref = None
        for name, kw in ALL:
            w = W.World(4, SC)
            sig = (w.pub.tables.scan_t.tobytes(), w.pub.tables.P.tobytes(), w.ge_bad.tobytes(), w.u_obs.tobytes(),
                   tuple(w.survivor_cells))
            ref = ref or sig
            self.assertEqual(sig, ref)
            r = sim.run_mission(4, SC, P.make(name, **kw), world=w)
            self.assertEqual(r["battery_wh_each"], SC.battery_wh)
            self.assertLessEqual(r["energy_wh"], SC.n_drones * SC.battery_wh + 1e-6, name)

    def test_planner_input_has_no_truth(self):
        fields = set(sim.DroneView.__slots__)
        for bad in ("world", "present", "survivor_cells", "view_good", "decoy", "u_obs", "ge_bad"):
            self.assertNotIn(bad, fields)

    def _future_perturbed(self, seed, sc, name, kw, t_star):
        w = W.World(seed, sc)
        a = sim.run_mission(seed, sc, P.make(name, **kw), record=True, world=w)
        # looks made strictly before t_star per (c, v), from the trace (scan decided at t, completed later: only the
        # outcomes of looks that COMPLETE after t_star may change; we perturb everything beyond the looks counted
        # among decisions made before t_star - 200 s, which have all completed by t_star)
        w2 = W.World(seed, sc)
        k_used = np.zeros_like(w2.looks)
        for (t, d, c, v, s, role) in a["trace"]:
            if t < t_star:
                k_used[c, v] += 1
        r = np.random.default_rng(99)
        for c in range(w2.C):
            for v in range(4):
                w2.u_obs[c, v, k_used[c, v]:] = r.random(w2.K_MAX - k_used[c, v])
        k0 = int(t_star // sc.slot_s) + 1
        w2.ge_bad[:, :, k0:] = r.random(w2.ge_bad[:, :, k0:].shape) < 0.5
        w2.ge_bad = np.logical_or(w2.ge_bad, np.transpose(w2.ge_bad, (1, 0, 2)))
        b = sim.run_mission(seed, sc, P.make(name, **kw), record=True, world=w2)
        cut = t_star - 200.0
        ta = [x for x in a["trace"] if x[0] < cut]
        tb = [x for x in b["trace"] if x[0] < cut]
        return ta, tb, a, b

    def test_no_future_information(self):
        for name, kw in ALL:
            if name == "ORACLE":
                continue
            ta, tb, a, b = self._future_perturbed(5, SC, name, kw, 400.0)
            self.assertGreater(len(ta), 3)
            self.assertEqual(ta, tb, name)

    def test_equivalence_all_recon_verify_by_self(self):
        for sc in (SC, SC_I, W.with_(SC, n_drones=5)):
            for seed in (6, 7):
                h = sim.run_mission(seed, sc, P.make("HOMO_SHARED_REVISIT", alpha=0.3, beta=0.6), record=True)
                s = sim.run_mission(seed, sc, P.make("SPEC_FIXED", r=1.0, verify_by_self=True, alpha=0.3, beta=0.6),
                                    record=True)
                self.assertEqual(strip(h), strip(s))
                self.assertEqual([x[:5] for x in h["trace"]], [x[:5] for x in s["trace"]])

    def test_equivalence_marginal_no_hysteresis(self):
        for seed in (6, 8):
            h = sim.run_mission(seed, SC, P.make("HOMO_SHARED_REVISIT"), record=True)
            s = sim.run_mission(seed, SC, P.make("SPEC_ADAPTIVE", rule="marginal", h=0.0, broad_roles=True),
                                record=True)
            self.assertEqual([x[:5] for x in h["trace"]], [x[:5] for x in s["trace"]])

    def test_ideal_comms_report_equals_confirm(self):
        r = sim.run_mission(2, SC_I, P.make("HOMO_SHARED_REVISIT"))
        self.assertEqual(r["recall_rep"], r["recall_conf"])
        self.assertLessEqual(r["t_rep_mean_c"] - r["t_conf_mean_c"], SC.slot_s * 1.0 + 1e-9)

    def test_switch_penalty_costs_time(self):
        a = sim.run_mission(2, SC_I, P.make("HOMO_SHARED_REVISIT"))
        b = sim.run_mission(2, W.with_(SC_I, switch_s=30.0), P.make("HOMO_SHARED_REVISIT"))
        if a["n_switch"] > 0:
            self.assertLessEqual(b["n_passes"], a["n_passes"])


if __name__ == "__main__":
    unittest.main()
