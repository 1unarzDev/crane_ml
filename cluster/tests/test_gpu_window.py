"""The GPU window (gpuwindow.py): the schedule, the idle/lock gate, the role split, and a simulated
idle -> active flip that must give a running GPU job back within seconds, with no attempt counted."""
import datetime as dt
import tempfile
import threading
import time
import unittest
from pathlib import Path

from crane_cluster import config, gpuwindow, worker
from tests.test_worker_parts import FakeClient

D = dt.datetime   # 2026-11-02 is a Monday
HOURS = gpuwindow.DEFAULTS["hours"]   # mon-fri 01:00-08:00, fri,sat 23:00-11:00


class Schedule(unittest.TestCase):
    def test_default_hours(self):
        w = lambda *a: gpuwindow.window_at(HOURS, D(*a))   # noqa: E731
        self.assertEqual(w(2026, 11, 2, 0, 59), (False, D(2026, 11, 2, 1, 0)))     # Mon before
        self.assertEqual(w(2026, 11, 2, 1, 0), (True, D(2026, 11, 2, 8, 0)))       # Mon inside
        self.assertEqual(w(2026, 11, 2, 8, 0), (False, D(2026, 11, 3, 1, 0)))      # Mon after: Tue 01:00
        # Friday: 01-08, then 23:00 to Saturday 11:00; Saturday 23:00 to Sunday 11:00; then Monday 01:00.
        self.assertEqual(w(2026, 11, 6, 12, 0), (False, D(2026, 11, 6, 23, 0)))
        self.assertEqual(w(2026, 11, 7, 3, 0), (True, D(2026, 11, 7, 11, 0)))
        self.assertEqual(w(2026, 11, 7, 23, 30), (True, D(2026, 11, 8, 11, 0)))
        self.assertEqual(w(2026, 11, 8, 11, 0), (False, D(2026, 11, 9, 1, 0)))

    def test_entries_merge_and_parse(self):
        hours = ["daily 22:00-24:00", "tue-wed 00:00-06:00", "sun,mon 10:00-12:00"]
        self.assertEqual(gpuwindow.window_at(hours, D(2026, 11, 3, 23, 0)), (True, D(2026, 11, 4, 6, 0)))  # Tue 22 -> Wed 06
        self.assertEqual(gpuwindow.window_at(hours, D(2026, 11, 2, 11, 0)), (True, D(2026, 11, 2, 12, 0)))
        self.assertEqual(gpuwindow.window_at([], D(2026, 11, 2)), (False, None))
        for bad in ["mon 1-2", "funday 01:00-02:00", "mon 25:00-26:00", "01:00-02:00"]:
            with self.assertRaises(ValueError):
                gpuwindow.parse_hours([bad])


class Probe:
    """A fake session: idle seconds that grow with a fake clock until input, and a lock flag."""
    def __init__(self, idle=0.0, locked=False):
        self.t, self.last_input, self.locked = 1000.0, 1000.0 - idle, locked

    def tick(self, s):
        self.t += s

    def touch(self):
        self.last_input = self.t

    def idle(self):
        return self.t - self.last_input


def gate(probe, now=D(2026, 11, 2, 2, 0), **opts):
    return gpuwindow.Gate(opts, idle_fn=probe.idle, locked_fn=lambda: probe.locked, clock=lambda: now, mono=lambda: probe.t)


class GateTest(unittest.TestCase):
    def test_closed_outside_the_window_or_while_active(self):
        p = Probe(idle=3600)
        self.assertIn("next at Mon 01:00", gate(p, now=D(2026, 11, 2, 0, 30)).closed())
        self.assertIsNone(gate(p).closed())
        p.touch()
        p.tick(300)
        self.assertIn("user active", gate(p).closed())                 # 5 min < 10 min
        p.locked = True
        self.assertIsNone(gate(p).closed())                            # locked, quiet for 60 s+
        p.touch()
        self.assertIn("user active", gate(p).closed())                 # a key on the lock screen: wait again
        self.assertEqual(gate(Probe(3600), enabled=False).closed(), "GPU window disabled")

    def test_unknown_activity_never_counts_as_idle(self):
        g = gpuwindow.Gate({}, idle_fn=lambda: None, locked_fn=lambda: None, clock=lambda: D(2026, 11, 2, 2, 0))
        self.assertIn("unknown", g.closed())

    def test_interrupted_by_input_unlock_or_the_window_end(self):
        p = Probe(idle=900)
        g = gate(p)
        g.arm()
        for _ in range(5):
            p.tick(1)
            self.assertIsNone(g.interrupted())
        p.tick(0.4)
        p.touch()
        p.tick(0.6)
        self.assertEqual(g.interrupted(), "user input")

        p = Probe(idle=120, locked=True)
        g = gate(p)
        g.arm()
        p.tick(1)
        self.assertIsNone(g.interrupted())
        p.locked = False
        p.tick(1)
        self.assertEqual(g.interrupted(), "session unlocked")

        g = gate(Probe(idle=900), now=D(2026, 11, 2, 8, 0))
        self.assertEqual(g.interrupted(), "GPU window closed")

    def test_report_for_the_feed(self):
        r = gate(Probe(0), now=D(2026, 11, 2, 9, 0)).report(waiting="outside")
        self.assertFalse(r["open"])
        self.assertTrue(r["next_at"].startswith("2026-11-03T01:00"))
        self.assertTrue(gate(Probe(0)).report()["until"].startswith("2026-11-02T08:00"))


class Lanes(unittest.TestCase):
    def cfg(self, gpu_window=True):
        c = config.load(None)
        c["name"] = "harmony"
        c["backends"] = {"unity": {"type": "dummy"}, "theater-render": {"type": "dummy"}, "fastsim": {"type": "dummy"}}
        c["policy"] = {"nice": 10, "pause_on_activity": True, "idle_seconds": 300, "max_est_minutes": 60}
        if gpu_window:
            c["gpu"] = gpuwindow.options({"enabled": True})
        return c

    def test_split(self):
        self.assertEqual(config.lanes(self.cfg()), {"cpu": ["fastsim"], "gpu": ["theater-render", "unity"]})
        self.assertEqual(config.lanes(self.cfg(gpu_window=False)), {"cpu": ["fastsim", "theater-render", "unity"]})

    def test_policy_reports_both_lanes(self):
        w = worker.Worker(self.cfg(), client=FakeClient([]))
        p = w.lease_policy({"cpu": "user active", "gpu": "outside the GPU window"})
        self.assertEqual(p["window"]["backends"], ["theater-render", "unity"])
        self.assertEqual(p["lanes"], [{"name": "cpu", "backends": ["fastsim"], "paused": "user active"},
                                      {"name": "gpu", "backends": ["theater-render", "unity"], "paused": "outside the GPU window"}])
        self.assertEqual(p["max_est_minutes"], 60)

    def test_cpu_lane_leases_while_the_gpu_lane_is_closed(self):
        with tempfile.TemporaryDirectory() as tmp:
            c = self.cfg()
            c.update({"data_dir": Path(tmp), "off_file": Path(tmp) / "off", "policy": {"nice": 0}})
            w = worker.Worker(c, client=FakeClient([]))
            probe = Probe(idle=0)   # he is at it, outside the hours
            w.gate = gate(probe, now=D(2026, 11, 2, 12, 0))
            w.loop(once=True)
            path, body = w.client.calls[-1]
            self.assertEqual((path, body["backends"]), ("/v1/lease", ["fastsim"]))
            self.assertEqual([l.get("paused") for l in body["policy"]["lanes"]], [None, "outside the GPU window (next at Tue 01:00)"])


class Client(FakeClient):
    def post(self, path, body):
        if path == "/v1/lease" and body.get("max_jobs") == 0:   # a report: leases nothing
            self.calls.append((path, body))
            return {"jobs": [], "repos": {}, "lease_ttl_s": 3}
        return super().post(path, body)


class Flip(unittest.TestCase):
    """The gpu lane leases while the PC is idle, gives the job back at the first input and waits for idle again."""

    def setUp(self):
        self.tmp = tempfile.TemporaryDirectory()

    def tearDown(self):
        self.tmp.cleanup()

    def runner(self, jobs, probe, now):
        cfg = config.load(None)
        cfg.update({"data_dir": Path(self.tmp.name) / "data", "off_file": Path(self.tmp.name) / "off", "poll_s": 0.05,
                    "heartbeat_s": 60, "policy": {"nice": 0}, "backends": {"unity": {"type": "dummy"}},
                    "gpu": gpuwindow.options({"enabled": True, "check_s": 0.2})})
        w = worker.Worker(cfg, client=Client(jobs))
        w.gate = gpuwindow.Gate(cfg["gpu"], idle_fn=probe.idle, locked_fn=lambda: probe.locked, clock=now,
                                mono=lambda: probe.t)
        return w

    def test_idle_active_flip(self):
        probe = Probe(idle=900)
        clock = {"now": D(2026, 11, 2, 0, 50)}
        job = {"job_id": "g1", "backend": "unity", "attempt": 1, "seeds": [1, 2], "spec": {"sleep_s": 30}}
        w = self.runner([job], probe, lambda: clock["now"])

        w.loop(once=True)   # before the window: a lease for no job that reports the next opening
        path, body = w.client.calls[-1]
        self.assertEqual((path, body["max_jobs"], body["state"]), ("/v1/lease", 0, "paused"))
        self.assertTrue(body["policy"]["window"]["next_at"].startswith("2026-11-02T01:00"))

        clock["now"] = D(2026, 11, 2, 1, 5)   # inside, idle 15 min: leases g1; then he moves the mouse
        stop = threading.Event()

        def clock_runs():   # the fake session's clock runs in real time; input after 0.5 s
            t0 = time.monotonic()
            while not stop.wait(0.05):
                probe.t = 1000.0 + (time.monotonic() - t0)
                if probe.t > 1000.5 and probe.last_input < 1000:
                    probe.touch()
        threading.Thread(target=clock_runs, daemon=True).start()
        t0 = time.time()
        w.loop(once=True)
        stop.set()
        self.assertLess(time.time() - t0, 5)   # released within seconds, not at the 60 s heartbeat
        lease = [b for p, b in w.client.calls if p == "/v1/lease"][-1]
        self.assertEqual(lease["max_jobs"], 1)
        self.assertTrue(lease["policy"]["window"]["open"])
        self.assertIn(("/v1/release", {"job_ids": ["g1"]}), w.client.calls)
        self.assertFalse([p for p, _ in w.client.calls if p.endswith("/complete") or p.endswith("/runs")])

        n = len(w.client.calls)
        w.loop(once=True)   # paused until idle again: no job leased
        path, body = w.client.calls[n]
        self.assertEqual((path, body["max_jobs"]), ("/v1/lease", 0))
        self.assertIn("user active", body["policy"]["window"]["waiting"])


if __name__ == "__main__":
    unittest.main()
