# SPDX-License-Identifier: GPL-3.0-or-later
"""Run: ~/.cache/research/fs-venv/bin/python -m unittest discover -s Tools/Frontier -p 'test_*.py'"""
import hashlib
import json
import multiprocessing as mp
import os
import sys
import shutil
import tempfile
import unittest

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
import cache  # noqa: E402
import costs  # noqa: E402
import jobs  # noqa: E402
import prov  # noqa: E402
import train  # noqa: E402

CALLS = []


def mkd(case):
    d = tempfile.mkdtemp(prefix="frontier_test_", dir=os.path.expanduser("~/.cache"))
    case.addCleanup(shutil.rmtree, d, True)
    return d


def fn(config, seed, fidelity):
    CALLS.append((seed, fidelity))
    return {"x": config["a"] * seed, "f": fidelity}


def _worker(args):
    root, i = args
    for s in range(30):
        cache.get_or_run("conc", lambda c, sd, f: {"v": sd * 2, "w": "x" * 200}, {"a": 1}, s, "L1", [__file__], root=root)
    return i


class CacheTests(unittest.TestCase):
    def setUp(self):
        self.tmp = mkd(self)
        CALLS.clear()
        self.code = os.path.join(self.tmp, "sim.py")
        open(self.code, "w").write("X = 1\n")

    def test_hit_does_not_call_fn(self):
        a = cache.get_or_run("s", fn, {"a": 2}, 3, "L1", [self.code], root=self.tmp)
        b = cache.get_or_run("s", fn, {"a": 2}, 3, "L1", [self.code], root=self.tmp)
        self.assertEqual(a, b)
        self.assertEqual(a, {"x": 6, "f": "L1"})
        self.assertEqual(len(CALLS), 1)
        rec = cache.get_record("s", fn, {"a": 2}, 3, "L1", [self.code], root=self.tmp)
        self.assertTrue(rec["cache_hit"])
        self.assertEqual(len(CALLS), 1)

    def test_key_sensitivity(self):
        k = cache.make_key([self.code], {"a": 1, "b": 2}, 1, "L1")
        self.assertEqual(k, cache.make_key([self.code], {"b": 2, "a": 1}, 1, "L1"))  # canonical JSON
        self.assertNotEqual(k, cache.make_key([self.code], {"a": 1, "b": 2}, 2, "L1"))
        self.assertNotEqual(k, cache.make_key([self.code], {"a": 1, "b": 2}, 1, "L2"))
        self.assertNotEqual(k, cache.make_key([self.code], {"a": 1, "b": 3}, 1, "L1"))

    def test_code_change_invalidates(self):
        cache.get_or_run("s", fn, {"a": 2}, 3, "L1", [self.code], root=self.tmp)
        open(self.code, "w").write("X = 2\n")
        cache.get_or_run("s", fn, {"a": 2}, 3, "L1", [self.code], root=self.tmp)
        self.assertEqual(len(CALLS), 2)
        open(self.code, "w").write("X = 1\n")  # reverting restores the old key: a hit
        cache.get_or_run("s", fn, {"a": 2}, 3, "L1", [self.code], root=self.tmp)
        self.assertEqual(len(CALLS), 2)

    def test_heldout_requires_matching_prereg(self):
        ledger = os.path.join(self.tmp, "SEEDS.md")
        pre = os.path.join(self.tmp, "PREREG_X.md")
        open(pre, "w").write("frozen\n")
        open(ledger, "w").write("| PREREG_X.md | PENDING |\n")
        kw = dict(root=self.tmp, split="heldout", seeds_md=ledger)
        with self.assertRaises(cache.HeldoutRefused):
            cache.get_or_run("h", fn, {"a": 1}, 1, "L1", [self.code], **kw)  # no prereg
        with self.assertRaises(cache.HeldoutRefused):
            cache.get_or_run("h", fn, {"a": 1}, 1, "L1", [self.code], prereg=pre, **kw)  # sha not in ledger
        digest = hashlib.sha256(open(pre, "rb").read()).hexdigest()
        open(ledger, "w").write("| OTHER.md | %s |\n" % digest)
        with self.assertRaises(cache.HeldoutRefused):
            cache.get_or_run("h", fn, {"a": 1}, 1, "L1", [self.code], prereg=pre, **kw)  # sha under the wrong name
        self.assertEqual(CALLS, [])
        open(ledger, "w").write("| PREREG_X.md | %s |\n" % digest)
        cache.get_or_run("h", fn, {"a": 1}, 1, "L1", [self.code], prereg=pre, **kw)
        self.assertEqual(len(CALLS), 1)
        open(pre, "w").write("edited after freeze\n")
        with self.assertRaises(cache.HeldoutRefused):
            cache.get_or_run("h", fn, {"a": 1}, 1, "L1", [self.code], prereg=pre, **kw)

    def test_concurrent_writers(self):
        with mp.get_context("fork").Pool(6) as p:
            p.map(_worker, [(self.tmp, i) for i in range(6)])
        d = os.path.join(self.tmp, "conc")
        n = 0
        keys = []
        for name in os.listdir(d):
            for line in open(os.path.join(d, name), "rb"):
                rec = json.loads(line)  # every line must be valid JSON
                keys.append(rec["key"])
                n += 1
        self.assertEqual(n, 30)                      # one record per (seed): first writer wins, no duplicates
        self.assertEqual(len(set(keys)), 30)
        for s in range(30):
            self.assertEqual(cache.get_or_run("conc", fn, {"a": 1}, s, "L1", [__file__], root=self.tmp), {"v": s * 2, "w": "x" * 200})

    def test_truncated_tail_tolerated(self):
        cache.get_or_run("t", fn, {"a": 1}, 1, "L1", [self.code], root=self.tmp)
        key = cache.make_key([self.code], {"a": 1}, 1, "L1")
        path = cache.shard_path("t", key, self.tmp)
        with open(path, "ab") as f:
            f.write(b'{"key": "deadbeef", "resu')  # crashed writer
        cache.get_or_run("t", fn, {"a": 1}, 2, "L1", [self.code], root=self.tmp)
        self.assertEqual(len(CALLS), 2)
        self.assertEqual(len(list(cache.iter_study("t", self.tmp))), 2)


class ProvTests(unittest.TestCase):
    def test_roundtrip(self):
        d = mkd(self)
        res = os.path.join(d, "x.jsonl")
        open(res, "w").write("{}\n")
        with prov.Timer() as t:
            sum(range(10000))
        rec = prov.collect(config={"a": 1}, seeds=[1, 2, 3], timer=t, code_files=[__file__], extra={"k": "v"})
        p = prov.write_sidecar(res, rec)
        self.assertTrue(p.endswith(".prov.json"))
        back = prov.read_sidecar(res)
        self.assertEqual(back, json.loads(json.dumps(rec)))
        for k in ("git_sha", "diff_sha256", "config_sha256", "seeds", "host", "wall_s", "cpu_s", "max_rss_mb"):
            self.assertIn(k, back)
        self.assertEqual(back["seeds"], [1, 2, 3])
        self.assertEqual(back["config_sha256"], prov.spec_hash({"a": 1}))
        self.assertGreater(back["max_rss_mb"], 1)
        self.assertIn("|", prov.markdown_row(back, "claim", "x.jsonl"))


class CostsAndJobs(unittest.TestCase):
    def test_costs(self):
        p = os.path.join(mkd(self), "c.jsonl")
        costs.seed_audit(p)
        costs.seed_audit(p)  # idempotent
        self.assertEqual(len(costs.load(p)), len(costs.AUDIT))
        self.assertAlmostEqual(costs.cost_per_unit("dotnet_harness", p), 32.0)
        with costs.track("dotnet_harness", "run", n=4, path=p):
            sum(range(1000))
        self.assertLess(costs.cost_per_unit("dotnet_harness", p), 1.0)  # measured overrides audit
        self.assertTrue(os.path.exists(os.path.join(HERE, "costs.jsonl")))

    def test_jobs(self):
        js = jobs.cpu_shard_jobs("d4_test", "Retrospective multi-fidelity selection sweep on dev seeds.", "python x.py", list(range(100)), 30)
        self.assertTrue(all(15 <= len(j["seeds"]) <= 20 for j in js))
        self.assertEqual(sorted(s for j in js for s in j["seeds"]), list(range(100)))
        g = jobs.gpu_job("gputest1", "Train a small decision gate model with checkpoints at each epoch.", "python train.py --model mlp", 600, 4)
        self.assertTrue(g["needs"]["gpu"] and g["needs"]["vram_gb"] <= 8 and g["preemptible"])
        self.assertIn("--resume", g["spec"]["cmd"])
        with self.assertRaises(ValueError):
            jobs.gpu_job("gputest2", "Train a small decision gate model with checkpoints at each epoch.", "x", 60, 12)
        with self.assertRaises(ValueError):
            jobs.cpu_shard_jobs("d4_test", "Retrospective multi-fidelity selection sweep on dev seeds.", "x", [1], 1, split="heldout")


class TrainTests(unittest.TestCase):
    def test_deterministic_and_resume(self):
        X, y = train.synthetic(400, 6, 5)
        a = train.train_logistic(X, y, epochs=8, seed=3)
        b = train.train_logistic(X, y, epochs=8, seed=3)
        self.assertTrue((a.w == b.w).all() and a.b == b.b)
        c = train.train_logistic(X, y, epochs=8, seed=4)
        self.assertFalse((a.w == c.w).all())
        d = mkd(self)
        train.train_logistic(X, y, epochs=4, seed=3, ckpt_dir=d)           # "preempted" after 4 epochs
        r = train.train_logistic(X, y, epochs=8, seed=3, ckpt_dir=d, resume=True)
        self.assertTrue((a.w == r.w).all() and a.b == r.b)                  # bit-identical to the uninterrupted run
        self.assertGreater(((a.predict(X) > .5) == (y > .5)).mean(), 0.7)

    @unittest.skipIf(train.torch is None, "torch not importable (optional)")
    def test_mlp_deterministic(self):
        X, y = train.synthetic(200, 4, 1)
        m1 = train.train_mlp(X, y, epochs=3, seed=2)
        m2 = train.train_mlp(X, y, epochs=3, seed=2)
        for p, q in zip(m1.parameters(), m2.parameters()):
            self.assertTrue((p == q).all())


if __name__ == "__main__":
    unittest.main()
