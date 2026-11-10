# SPDX-License-Identifier: GPL-3.0-or-later
"""Determinism, no-future-information (truth hiding), split guards, comms priority, obs-model swap."""
import copy
import hashlib
import json
import os
import sys
import tempfile
import unittest

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.dirname(HERE))
from missionsim import sim, world as W, comms as C, agent as A, policies as P, obs_model as O  # noqa: E402

SC = W.Scenario(density="medium", horizon_s=300.0, deadline_s=280.0)


def _hash(path):
    with open(path, "rb") as f:
        return hashlib.sha256(f.read()).hexdigest()


class Determinism(unittest.TestCase):
    def test_same_seed_same_mission_and_log(self):
        with tempfile.TemporaryDirectory() as d:
            p1, p2 = os.path.join(d, "a.jsonl"), os.path.join(d, "b.jsonl")
            m1 = sim.run_mission(5, 0, "pod_deadzone", SC, log_path=p1)
            m2 = sim.run_mission(5, 0, "pod_deadzone", SC, log_path=p2)
            self.assertEqual(json.dumps(m1, default=float, sort_keys=True), json.dumps(m2, default=float, sort_keys=True))
            self.assertEqual(_hash(p1), _hash(p2))

    def test_world_is_seeded(self):
        a, b = W.World(9, SC), W.World(9, SC)
        np.testing.assert_array_equal(a.rho, b.rho)
        np.testing.assert_array_equal(a.margin_up, b.margin_up)
        self.assertEqual(a.target_cell, b.target_cell)
        c = W.World(10, SC)
        self.assertFalse(np.array_equal(a.rho, c.rho))


class NoFutureInformation(unittest.TestCase):
    def _decisions(self, world, pol):
        with tempfile.TemporaryDirectory() as d:
            p = os.path.join(d, "l.jsonl")
            M = sim.run_mission(world.seed, 0, pol, world.sc, log_path=p, world=world)
            with open(p) as f:
                recs = [json.loads(l) for l in f]
        return M, recs

    def test_hidden_person_location_does_not_leak(self):
        """Two worlds that differ ONLY in the hidden person location must produce identical decisions until the
        first true detection in either run (planners may not condition on truth they have not observed)."""
        for pol in ("pod", "pod_deadzone", "lease", "coordinator", "b45_mule_recheck"):
            w1 = W.World(4, SC)
            w2 = W.World(4, SC)
            free = [c for c in range(W.NC) if not w2.blocked[c] and c != w1.target_cell
                    and abs(c - w1.target_cell) > 15]
            c2 = free[len(free) // 2]
            w2.target_cell = c2
            w2.target_xy = W.CENTERS[c2] + np.array([1.0, -2.0])
            m1, r1 = self._decisions(w1, pol)
            m2, r2 = self._decisions(w2, pol)
            tcut = min(x for x in (m1["t_first_det"], m2["t_first_det"], 1e9) if x is not None)
            k1 = [(r["t"], r["drone"], json.dumps(r["chosen"])) for r in r1 if r["t"] <= tcut]
            k2 = [(r["t"], r["drone"], json.dumps(r["chosen"])) for r in r2 if r["t"] <= tcut]
            self.assertGreater(len(k1), 5)
            self.assertEqual(k1, k2, pol)

    def test_agent_view_holds_no_truth(self):
        w = W.World(3, SC)
        av = A.AgentView(1, w.public, w.obs, P.make("pod"))
        seen, stack = set(), [av]
        while stack:
            o = stack.pop()
            if id(o) in seen:
                continue
            seen.add(id(o))
            self.assertNotIsInstance(o, W.World)
            if isinstance(o, dict):
                stack += list(o.values())
            elif isinstance(o, (list, tuple, set)):
                stack += list(o)
            elif hasattr(o, "__dict__"):
                stack += list(vars(o).values())

    def test_forecast_differs_from_truth_and_planner_sees_only_forecast(self):
        w = W.World(3, SC)
        self.assertGreater(np.abs(w.public.forecast_up - w.margin_up).mean(), 1.0)
        av1 = A.AgentView(1, w.public, w.obs, P.make("pod"))
        w2 = W.World(3, SC)
        w2.margin_up = w2.margin_up - 30.0  # truth changes, forecast does not
        av2 = A.AgentView(1, w2.public, w2.obs, P.make("pod"))
        np.testing.assert_array_equal(av1.k.p_link, av2.k.p_link)

    def test_only_oracle_uses_truth(self):
        for name, cls in P.POLICIES.items():
            self.assertEqual(cls.uses_truth, name == "oracle", name)


class SplitGuards(unittest.TestCase):
    def test_heldout_refused(self):
        with self.assertRaises(W.HeldoutRefused):
            W.World(90001, SC)
        with self.assertRaises(W.HeldoutRefused):
            sim.run_mission(95000, 0, "pod", SC)

    def test_unassigned_refused(self):
        with self.assertRaises(W.HeldoutRefused):
            W.World(150000, SC)

    def test_dev_and_val_allowed(self):
        self.assertEqual(W.World(3, SC).split, "dev")
        self.assertEqual(W.World(70003, SC).split, "val")

    def test_heldout_with_flag(self):
        self.assertEqual(W.World(90001, SC, allow_heldout=True).split, "heldout")

    def test_experiment_runner_refuses_heldout(self):
        import subprocess
        r = subprocess.run([sys.executable, os.path.join(os.path.dirname(HERE), "experiments.py"), "run", "--exp",
                            "E3", "--split", "heldout", "--layouts", "1"], capture_output=True, text=True)
        self.assertNotEqual(r.returncode, 0)
        self.assertIn("HELD-OUT", r.stderr + r.stdout)


class Comms(unittest.TestCase):
    def _net(self, priority):
        w = W.World(3, SC)
        w.margin_up[:] = 3.0  # weak but alive link: low rate
        w.margin_down[:] = 3.0
        return C.Network(w, 3, 0, 200, priority=priority), w

    def test_priority_delivers_detection_before_bulk(self):
        res = {}
        for prio in (True, False):
            net, w = self._net(prio)
            net.send(0, 1, 0, "MAP", {"x": 1})
            net.send(0, 1, 0, "DET", {"x": 2})
            pos = {1: W.CENTERS[w.start_cells[0]], 2: W.CENTERS[w.start_cells[1]]}
            got = {}
            for t in range(120):
                for node, frm, m in net.tick(t, pos, {1: True, 2: True}):
                    got.setdefault(m.kind, t)
            res[prio] = got
        self.assertLess(res[True]["DET"], res[False]["DET"])

    def test_store_and_forward_survives_outage(self):
        w = W.World(3, SC)
        net = C.Network(w, 3, 0, 300, forced=((1, 0, 0, 50), (1, 2, 0, 50)))
        pos = {1: W.CENTERS[w.start_cells[0]], 2: W.CENTERS[w.start_cells[1]]}
        net.send(0, 1, 0, "DET", {"cid": 1})
        t_got = None
        for t in range(200):
            for node, frm, m in net.tick(t, pos, {1: True, 2: True}):
                if m.kind == "DET" and node == 0 and t_got is None:
                    t_got = t
        self.assertIsNotNone(t_got)
        self.assertGreaterEqual(t_got, 50)


class ObsModel(unittest.TestCase):
    def test_swappable(self):
        m = O.load(prefer_vision=True)
        for meth in ("p_view_good", "p_detect_pass", "p_false_alert", "sample_conf", "log_lr", "expected_pd", "loc_sigma"):
            self.assertTrue(hasattr(m, meth))
        w = W.World(3, SC, obs=O.PlaceholderObsModel())
        self.assertIsInstance(w.obs, O.PlaceholderObsModel)

    def test_monotone_in_speed(self):
        m = O.PlaceholderObsModel()
        self.assertGreater(m.p_detect_pass(4.0, 1.5, 0.5, 8.0), m.p_detect_pass(4.0, 6.0, 0.5, 8.0))

    def test_poor_location_label_kept(self):
        c = A.Candidate(1, [0, 0], 0, 3.0, 0.0)
        e = c.drone_ev(1)
        e.update(lo=2.0, n=1, sum_est=np.zeros(2), s_ind=1.0, s_com=400.0, views=[0])
        s = c.summary(10.0)
        self.assertTrue(s["confirmed"])
        self.assertEqual(s["label"], "CONFIDENT_POOR_LOC")


if __name__ == "__main__":
    unittest.main()
