import random
import unittest
import zlib

from crane_cluster import promotion as P

# scipy.stats.wilcoxon docs example (Darwin's plant heights): two-sided p = 0.041259765625
DARWIN = [6, 8, 14, 16, 23, 24, 28, 29, 41, -48, 49, 56, 60, -67, 75]


def rows(values_by_arm, metric="score"):
    return [{"arm": arm, "seed": s, "name": metric, "value": v, "status": "ok"}
            for arm, vals in values_by_arm.items() for s, v in vals.items()]


class Wilcoxon(unittest.TestCase):
    def test_exact_matches_reference(self):
        p, n, w = P.wilcoxon_greater(DARWIN)
        self.assertEqual(n, 15)
        self.assertEqual(w, 96)
        self.assertAlmostEqual(p, 0.041259765625 / 2, places=12)

    def test_all_positive_small_n(self):
        self.assertAlmostEqual(P.wilcoxon_greater([1, 2, 3, 4, 5])[0], 1 / 32)
        self.assertAlmostEqual(P.wilcoxon_greater([-1, -2, -3, -4, -5])[0], 1.0)

    def test_zeros_dropped_and_ties_averaged(self):
        p, n, w = P.wilcoxon_greater([0, 0, 1, 1, -1, 2])
        self.assertEqual(n, 4)
        self.assertEqual(w, 2 + 2 + 4)  # ranks of |1|,|1|,|1| are 2 each, |2| is 4
        self.assertTrue(0 < p < 1)

    def test_normal_approximation_is_close_to_exact(self):
        rnd = random.Random(7)
        d = [rnd.gauss(0.3, 1) for _ in range(30)]
        exact = P.wilcoxon_greater(d)[0]
        old, P.EXACT_MAX_N = P.EXACT_MAX_N, 0
        try:
            approx = P.wilcoxon_greater(d)[0]
        finally:
            P.EXACT_MAX_N = old
        self.assertAlmostEqual(exact, approx, delta=0.01)

    def test_holm(self):
        self.assertEqual(P.holm({"a": 0.01, "b": 0.04, "c": 0.03}), {"a": 0.03, "c": 0.06, "b": 0.06})


class Pairing(unittest.TestCase):
    def test_only_common_seeds_are_paired_and_direction_flips(self):
        r = rows({"baseline": {1: 1.0, 2: 2.0, 3: 3.0}, "a": {1: 1.5, 3: 2.0, 4: 9.0}})
        self.assertEqual(P.paired_diffs(r, "score", "a"), [0.5, -1.0])
        self.assertEqual(P.paired_diffs(r, "score", "a", "decrease"), [-0.5, 1.0])

    def test_failed_runs_are_excluded(self):
        r = rows({"baseline": {1: 1.0, 2: 1.0}, "a": {1: 2.0, 2: 2.0}})
        r[-1]["status"] = "failed"
        self.assertEqual(len(P.paired_diffs(r, "score", "a")), 1)


def hyp(variants=("a",), seeds=8, **promo):
    return {"metric": "score", "direction": "increase", "seeds": list(range(1, seeds + 1)),
            "variants": [{"name": v, "active": True} for v in variants], "budget": {}, "promotion": promo}


def simulate(effects, seeds, sd=0.1):
    out = {"baseline": {}}
    for s in seeds:
        base = random.Random(s).gauss(0, 1)
        out["baseline"][s] = base + random.Random(1000 + s).gauss(0, sd)
        for v, e in effects.items():
            out.setdefault(v, {})[s] = base + e + random.Random(zlib.crc32(f"{v}:{s}".encode())).gauss(0, sd)
    return rows(out)


class Decide(unittest.TestCase):
    def test_wait_while_pending(self):
        self.assertEqual(P.decide({"pending": 2, "rows": []}, hyp()).action, "wait")

    def test_clear_win_promotes(self):
        d = P.decide({"pending": 0, "rows": simulate({"a": 0.5}, range(1, 9))}, hyp())
        self.assertEqual(d.action, "promote")
        self.assertEqual(d.variants, ["a"])
        self.assertLess(d.stats["a"].p_better_adj, 0.05)

    def test_clear_loss_rejects(self):
        self.assertEqual(P.decide({"pending": 0, "rows": simulate({"a": -0.5}, range(1, 9))}, hyp()).action, "reject")

    def test_no_effect_extends_then_rejects_at_cap(self):
        h = hyp(max_seeds=16)

        def null_rows(n):  # differences alternate in sign with growing size: no effect either way
            return rows({"baseline": {s: 0.0 for s in range(1, n + 1)},
                         "a": {s: (s if s % 2 else -s) * 0.01 for s in range(1, n + 1)}})
        d = P.decide({"pending": 0, "rows": null_rows(8)}, h)
        self.assertEqual(d.action, "extend")
        self.assertEqual(d.seeds, list(range(9, 17)))
        d2 = P.decide({"pending": 0, "rows": null_rows(16)}, h)
        self.assertEqual(d2.action, "reject")

    def test_min_effect_blocks_a_tiny_but_significant_gain(self):
        r = simulate({"a": 0.05}, range(1, 13), sd=0.001)
        self.assertEqual(P.decide({"pending": 0, "rows": r}, hyp(seeds=12)).action, "promote")
        self.assertNotEqual(P.decide({"pending": 0, "rows": r}, hyp(seeds=12, min_effect=0.1, max_seeds=12)).action, "promote")

    def test_successive_halving_keeps_the_best_half(self):
        self.assertEqual(P.halving_survivors({"a": 0.1, "b": 0.5, "c": 0.3, "d": -1}), ["b", "c"])
        self.assertEqual(P.halving_survivors({"a": 0.1, "b": 0.5, "c": 0.3}, eta=3), ["b"])
        d = P.decide({"pending": 0, "rows": simulate({"a": 0.1, "b": 0.6, "c": 0.3, "d": -0.2}, range(1, 9))},
                     hyp(variants="abcd"))
        self.assertEqual(d.action, "extend")
        self.assertEqual(d.variants, ["b", "c"])

    def test_budget_limits_extension(self):
        d = P.decide({"pending": 0, "rows": simulate({"a": 0.0}, range(1, 9))},
                     {**hyp(), "budget": {"max_runs": 20}}, runs_used=16)
        self.assertEqual(d.action, "extend")
        self.assertEqual(len(d.seeds), 2)  # (20 - 16) // 2 arms
        d = P.decide({"pending": 0, "rows": simulate({"a": 0.0}, range(1, 9))},
                     {**hyp(), "budget": {"max_runs": 16}}, runs_used=16)
        self.assertEqual(d.action, "reject")


if __name__ == "__main__":
    unittest.main()
