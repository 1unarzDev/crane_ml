"""Multi-slot workers: per-backend slots, the core/GPU/memory budget, concurrent jobs and drain."""
import tempfile
import threading
import time
import unittest
from pathlib import Path

from crane_cluster import config, worker
from crane_cluster.slots import Slots

BACKENDS = {"fast": {"type": "dummy", "slots": 6, "cpus": 1, "mem_gb": 1}, "task": {"type": "command", "command": "x",
            "slots": 2, "cpus": 8, "mem_gb": 3}, "unity": {"type": "x.unity:ClusterBackend"},
            "theater-render": {"type": "x.theater:ClusterBackend"}, "dummy": {"type": "dummy"}}


class Clock:
    def __init__(self):
        self.t = 0.0

    def __call__(self):
        return self.t


def slots(conf=None, mem=100.0, backends=BACKENDS, clock=None):
    cfg = {"backends": backends, **({"slots": conf} if conf is not None else {})}
    return Slots(cfg, {"cpu": 20, "gpu_count": 1}, mem_gb=lambda: mem, clock=clock or Clock())


class SlotsTest(unittest.TestCase):
    def test_without_a_slots_section_one_job_at_a_time(self):
        s = slots()
        self.assertEqual(set(s.free()), set(BACKENDS))
        self.assertEqual(set(s.free().values()), {1})
        s.acquire("j1", "fast")
        self.assertEqual(s.free(), {})
        s.release("j1")
        self.assertEqual(set(s.free().values()), {1})

    def test_defaults(self):
        s = slots({})
        self.assertEqual(s.cpus, 10)   # half the cores
        self.assertEqual({b: (v["slots"], v["gpu"]) for b, v in s.spec.items()},
                         {"fast": (6, False), "task": (2, False), "unity": (1, True), "theater-render": (1, True),
                          "dummy": (8, False)})

    def test_core_budget_gpu_slots_and_slot_indexes(self):
        s = slots({"cpus": 16})
        self.assertEqual(s.free(), {"fast": 6, "task": 2, "unity": 1, "theater-render": 1, "dummy": 8})
        self.assertEqual([s.acquire(f"t{i}", "task") for i in range(2)], [0, 1])   # 16 of 16 cores
        self.assertEqual(s.free(), {"dummy": 8})   # dummy costs no cores; GPU jobs need 2
        s.release("t0")
        self.assertEqual(s.free()["task"], 1)
        self.assertEqual(s.acquire("u", "unity"), 0)   # the lowest unused index
        self.assertNotIn("theater-render", s.free())   # one GPU: unity holds it
        self.assertNotIn("unity", s.free())

    def test_memory_admits_against_estimates_and_ramp(self):
        clock = Clock()
        s = slots({"cpus": 20, "reserve_gb": 2}, mem=10.0, clock=clock)
        s.acquire("t0", "task")   # 10 GB available, 2 reserved, 3 ramping: 5 left
        self.assertEqual((s.free()["task"], s.free()["fast"]), (1, 5))
        clock.t = 61   # past the ramp: MemAvailable already counts it
        self.assertEqual(s.free()["fast"], 6)
        idle = slots({"cpus": 20}, mem=0.5)
        self.assertEqual(idle.free()["task"], 1)   # an idle worker always admits one job


class Client:
    """A coordinator that hands out queued jobs up to max_jobs among the requested backends."""

    def __init__(self, jobs):
        self.queue, self.calls, self.lock = list(jobs), [], threading.Lock()

    def post(self, path, body):
        with self.lock:
            self.calls.append((path, body))
            if path == "/v1/lease":
                ok = [j for j in self.queue if j["backend"] in body.get("backends", [j["backend"]])][:body["max_jobs"]]
                self.queue = [j for j in self.queue if j not in ok]
                return {"jobs": ok, "repos": {}, "lease_ttl_s": 30}
            if path.endswith("/runs"):
                return {"run_id": f"{path.split('/')[3]}.r{body['seed']}"}
            return {}

    def upload(self, *a):
        pass


class ConcurrentWorker(unittest.TestCase):
    def setUp(self):
        self.tmp = tempfile.TemporaryDirectory()

    def tearDown(self):
        self.tmp.cleanup()

    def worker(self, jobs, **extra):
        cfg = config.load(None)
        cfg.update({"data_dir": Path(self.tmp.name) / "data", "off_file": Path(self.tmp.name) / "off", "poll_s": 0.05,
                    "heartbeat_s": 0.1, "long_poll_s": 0, "policy": {"nice": 0},
                    "backends": {"dummy": {"type": "dummy", "slots": 3}, "gpu": {"type": "dummy", "gpu": True}}, **extra})
        return worker.Worker(cfg, client=Client(jobs))

    def test_jobs_run_side_by_side_with_their_own_slots(self):
        jobs = [{"job_id": f"j{i}", "backend": "dummy", "attempt": 1, "seeds": [1], "spec": {"sleep_s": 0.5}}
                for i in range(3)] + [{"job_id": "g", "backend": "gpu", "attempt": 1, "seeds": [1], "spec": {"sleep_s": 0.5}}]
        w = self.worker(jobs, slots={"cpus": 8})
        isolation = {}
        run = w.run_job

        def run_job(job, ttl, slot=0):
            isolation[job["job_id"]] = slot
            return run(job, ttl, slot=slot)

        w.run_job = run_job
        threading.Timer(0.3, worker.drain_file(w.cfg).touch).start()
        t0 = time.time()
        w.loop()   # leases all four, drains: exits once they are done
        self.assertLess(time.time() - t0, 1.8)   # one after another would take 2 s
        done = [p for p, _ in w.client.calls if p.endswith("/complete")]
        self.assertEqual(sorted(done), [f"/v1/jobs/{j}/complete" for j in ("g", "j0", "j1", "j2")])
        self.assertEqual(sorted(isolation.values()), [0, 1, 2, 3])
        leases = [b for p, b in w.client.calls if p == "/v1/lease"]
        self.assertEqual((leases[0]["backends"], leases[0]["max_jobs"]), (["dummy", "gpu"], 1))
        self.assertFalse(worker.drain_file(w.cfg).exists())

    def test_a_job_runs_on_its_own_backend_instance(self):
        w = self.worker([])
        self.assertIsNot(w.new_backend("dummy"), w.new_backend("dummy"))
        self.assertNotIn("slots", w.new_backend("dummy").options)


if __name__ == "__main__":
    unittest.main()
