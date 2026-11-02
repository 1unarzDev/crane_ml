import datetime as dt
import json
import subprocess
import sys
import tempfile
import textwrap
import threading
import time
import unittest
from pathlib import Path

from crane_cluster import backends, config, hyp, idle, miniyaml, worker
from crane_cluster.backends import JobContext
from crane_cluster.gitcache import GitCache

WORKER_YAML = textwrap.dedent("""\
    coordinator: http://127.0.0.1:8787   # local
    name: "box # one"
    poll_s: 5
    caps:
      tags: [lab, fast]
      vulkan: false
    policy:
      nice: 10
      quiet_hours: ["09:00-17:00", '22:30-01:00']
      pause_on_activity: yes
    backends:
      dummy: {type: dummy, sleep_s: 0.5}
      sim:
        type: command
        command: [python3, -c, "print({seed})"]
        env:
          SLOT: "{isolation_id}"
    stages:
      - name: fast
        backend: dummy
        needs: {min_cpu: 2}
      - name: gpu
        backend: sim
        needs:
          gpu: true
          os: [linux, windows]
    empty:
    neg: -3.5e+2
""")


class MiniYaml(unittest.TestCase):
    def test_subset_parser_matches_expectations(self):
        d = miniyaml.parse(WORKER_YAML)
        self.assertEqual(d["coordinator"], "http://127.0.0.1:8787")
        self.assertEqual(d["name"], "box # one")
        self.assertEqual(d["caps"], {"tags": ["lab", "fast"], "vulkan": False})
        self.assertEqual(d["policy"]["quiet_hours"], ["09:00-17:00", "22:30-01:00"])
        self.assertIs(d["policy"]["pause_on_activity"], True)
        self.assertEqual(d["backends"]["dummy"], {"type": "dummy", "sleep_s": 0.5})
        self.assertEqual(d["backends"]["sim"]["command"], ["python3", "-c", "print({seed})"])
        self.assertEqual(d["stages"][1], {"name": "gpu", "backend": "sim", "needs": {"gpu": True, "os": ["linux", "windows"]}})
        self.assertIsNone(d["empty"])
        self.assertEqual(d["neg"], -350.0)

    def test_agrees_with_pyyaml_when_available(self):
        try:
            import yaml
        except ImportError:
            self.skipTest("PyYAML not installed")
        self.assertEqual(miniyaml.parse(WORKER_YAML), yaml.safe_load(WORKER_YAML))
        example = Path(__file__).resolve().parents[1] / "examples"
        for f in example.glob("*.yaml"):
            self.assertEqual(miniyaml.parse(f.read_text()), yaml.safe_load(f.read_text()), f.name)


class Idle(unittest.TestCase):
    def test_quiet_hours(self):
        p = {"quiet_hours": ["09:00-17:00", "22:30-01:00"]}
        at = lambda h, m=0, day=3: dt.datetime(2026, 11, 2 + day, h, m)  # 2026-11-02 is a Monday
        self.assertTrue(idle.in_quiet_hours(p, at(9)))
        self.assertFalse(idle.in_quiet_hours(p, at(17)))
        self.assertTrue(idle.in_quiet_hours(p, at(23)))
        self.assertTrue(idle.in_quiet_hours(p, at(0, 30)))
        self.assertFalse(idle.in_quiet_hours(p, at(1, 0)))
        weekdays = {**p, "quiet_days": ["mon", "tue", "wed", "thu", "fri"]}
        self.assertFalse(idle.in_quiet_hours(weekdays, at(10, day=5)))  # Saturday
        self.assertTrue(idle.in_quiet_hours(weekdays, at(0, 30, day=5)))  # Friday night's window
        self.assertIsNone(idle.unavailable_reason({}))
        with self.assertRaises(ValueError):
            idle.in_quiet_hours({"quiet_hours": ["9-5"]})

    def test_a_desktop_probe_reports_idle_time_to_a_session_0_worker(self):
        with tempfile.TemporaryDirectory() as t:
            path = Path(t) / "input-idle.json"
            self.assertIsNone(idle.probe_idle_seconds(path))  # nobody logged in
            path.write_text(json.dumps({"idle_s": 30.0, "at": 1000.0}))
            self.assertEqual(idle.probe_idle_seconds(path, now=1010.0), 40.0)
            self.assertIsNone(idle.probe_idle_seconds(path, now=1500.0))  # stale: the probe stopped


def ctx(tmp, **job):
    return JobContext(job={"job_id": "j1", "arm": "a", "spec": {}, "seeds": [1], **job}, work_dir=Path(tmp),
                      isolation={"slot": 0, "port_base": 20000, "isolation_id": 7})


class Backends(unittest.TestCase):
    def test_dummy_is_seeded_and_paired(self):
        b = backends.build({"dummy": {"type": "dummy", "sleep_s": 0}})["dummy"]
        with tempfile.TemporaryDirectory() as t:
            b.prepare(ctx(t, spec={"effect": 1.0}))
            m1 = b.metrics(b.run(3))
            b.prepare(ctx(t, arm="baseline"))
            m0 = b.metrics(b.run(3))
            self.assertAlmostEqual(m1["score"] - m0["score"], 1.0, delta=0.15)
            names = [n for _, n, _ in b.artifacts(Path(t) / "runs" / "seed-3")]
            self.assertEqual(names, ["summary.json", "metrics.json", "artifacts/log.txt"])

    def test_command_backend_on_host(self):
        code = "import json,os,sys; json.dump(dict(x=int(sys.argv[1]) * 2, iso=int(os.environ['SLOT'])), open(sys.argv[2] + '/metrics.json', 'w'))"
        b = backends.build({"sim": {"type": "command", "command": [sys.executable, "-c", code, "{seed}", "{run_dir}"],
                                    "env": {"SLOT": "{isolation_id}"}}})["sim"]
        with tempfile.TemporaryDirectory() as t:
            b.prepare(ctx(t))
            self.assertEqual(b.metrics(b.run(21)), {"x": 42.0, "iso": 7.0})
            bad = backends.build({"sim": {"type": "command", "command": [sys.executable, "-c", "raise SystemExit(3)"]}})["sim"]
            bad.prepare(ctx(t))
            with self.assertRaisesRegex(RuntimeError, "exited 3"):
                bad.run(1)

    def test_command_backend_container_argv(self):
        b = backends.build({"sim": {"type": "command", "image": "img:1", "gpus": True, "command": "sim --seed {seed} --src {repos[org/sim]}"}})["sim"]
        with tempfile.TemporaryDirectory() as t:
            c = ctx(t)
            c.repos = {"org/sim": Path(t) / "src"}
            b.prepare(c)
            argv = b.argv(5, Path(t) / "run")
            self.assertEqual(argv[:4], ["docker", "run", "--rm", "--name"])
            self.assertIn(f"{Path(t) / 'src'}:/crane/src/sim:ro", argv)
            self.assertIn("--gpus", argv)
            self.assertEqual(argv[-3:], ["sh", "-c", "sim --seed 5 --src /crane/src/sim"])

    def test_module_class_backends_load_by_path(self):
        b = backends.build({"x": {"type": "crane_cluster.backends.dummy:DummyBackend"}})
        self.assertEqual(type(b["x"]).__name__, "DummyBackend")

    def test_a_backend_path_makes_a_checkout_importable(self):
        with tempfile.TemporaryDirectory() as t:
            (Path(t) / "outside_pkg.py").write_text(
                "from crane_cluster.backends import Backend\nclass B(Backend):\n    pass\n")
            b = backends.build({"x": {"type": "outside_pkg:B", "path": t, "k": 1}})["x"]
            self.assertEqual((type(b).__name__, b.options), ("B", {"k": 1}))


def git(*a, cwd):
    subprocess.run(["git", "-c", "user.email=t@t", "-c", "user.name=t", *a], cwd=cwd, check=True, capture_output=True)


class GitCacheTest(unittest.TestCase):
    def test_checkout_exact_refs_and_refresh(self):
        with tempfile.TemporaryDirectory() as t:
            src = Path(t) / "src"
            src.mkdir()
            git("init", "-q", "-b", "main", cwd=src)
            (src / "f").write_text("1")
            git("add", "f", cwd=src)
            git("commit", "-qm", "one", cwd=src)
            first = subprocess.run(["git", "rev-parse", "HEAD"], cwd=src, capture_output=True, text=True).stdout.strip()
            cache = GitCache(Path(t) / "cache")
            sha = cache.checkout("org/src", str(src), "main", Path(t) / "co1")
            self.assertEqual(sha, first)
            (src / "f").write_text("2")
            git("commit", "-qam", "two", cwd=src)
            second = subprocess.run(["git", "rev-parse", "HEAD"], cwd=src, capture_output=True, text=True).stdout.strip()
            # an unknown sha triggers one fetch; the old sha still checks out exactly
            self.assertEqual(cache.checkout("org/src", str(src), second, Path(t) / "co2"), second)
            self.assertEqual((Path(t) / "co2" / "f").read_text(), "2")
            self.assertEqual(cache.checkout("org/src", str(src), first, Path(t) / "co1"), first)
            self.assertEqual((Path(t) / "co1" / "f").read_text(), "1")
            with self.assertRaises(RuntimeError):
                cache.checkout("org/src", str(src), "nope", Path(t) / "co3")


class HypSpec(unittest.TestCase):
    def test_normalize(self):
        spec = {"name": "n", "claim": "c", "metric": "m", "direction": "increase", "baseline": "@main",
                "variants": [{"name": "a", "ref": "f" * 40}], "seeds": 3, "stages": [{"backend": "dummy"}]}
        out = hyp.normalize(spec)
        self.assertEqual(out["seeds"], [1, 2, 3])
        self.assertEqual(out["baseline_ref"], "@main")
        self.assertNotIn("baseline", out)
        example = Path(__file__).resolve().parents[1] / "examples" / "hypothesis.yaml"
        body = hyp.normalize(miniyaml.load_file(example), resolve=False)
        json.dumps(body)
        self.assertEqual(body["direction"], "increase")


class FakeClient:
    """Records worker calls; leases ``jobs`` once."""
    def __init__(self, jobs=()):
        self.jobs, self.calls = list(jobs), []

    def post(self, path, body):
        self.calls.append((path, body))
        if path == "/v1/lease":
            jobs, self.jobs = self.jobs, []
            return {"jobs": jobs, "repos": {}, "lease_ttl_s": 3}
        if path.endswith("/runs"):
            return {"run_id": "r1"}
        return {}

    def upload(self, *a):
        pass

    def paths(self):
        return [p for p, _ in self.calls]


class PauseWhen(unittest.TestCase):
    def setUp(self):
        self.tmp = tempfile.TemporaryDirectory()
        self.flag = Path(self.tmp.name) / "live"

    def tearDown(self):
        self.tmp.cleanup()

    def worker(self, jobs=(), **extra):
        cfg = config.load(None)
        cfg.update({"data_dir": Path(self.tmp.name) / "data", "off_file": Path(self.tmp.name) / "off", "poll_s": 0.05,
                    "heartbeat_s": 0.1, "policy": {"nice": 0}, "pause_when": f"test -e '{self.flag}'",
                    "backends": {"dummy": {"type": "dummy"}}, **extra})
        return worker.Worker(cfg, client=FakeClient(jobs))

    def test_command_exit_status(self):
        self.assertFalse(idle.pause_requested(None))
        self.assertTrue(idle.pause_requested("true"))
        self.assertFalse(idle.pause_requested("false"))
        self.assertTrue(idle.pause_requested("sleep 5", timeout_s=0.2))  # a hung check pauses
        self.assertEqual(idle.unavailable_reason({}, cfg={"pause_when": "true"}), "pause_when")
        self.assertIsNone(idle.unavailable_reason({}, cfg={"pause_when": "false"}))

    def test_paused_worker_leases_nothing_then_resumes(self):
        self.flag.touch()
        w = self.worker()
        w.loop(once=True)
        self.assertEqual(w.client.paths(), ["/v1/heartbeat"])
        self.assertEqual(w.client.calls[0][1]["state"], "paused")
        self.flag.unlink()
        w.loop(once=True)
        self.assertEqual(w.client.paths()[-1], "/v1/lease")

    def test_running_job_is_released_on_pause(self):
        job = {"job_id": "j1", "backend": "dummy", "attempt": 1, "seeds": [1, 2], "spec": {"sleep_s": 30}}
        w = self.worker([job])
        threading.Timer(0.3, self.flag.touch).start()
        t0 = time.time()
        w.loop(once=True)
        self.assertLess(time.time() - t0, 10)
        self.assertIn(("/v1/release", {"job_ids": ["j1"]}), w.client.calls)
        self.assertFalse([p for p in w.client.paths() if p.endswith("/complete") or p.endswith("/runs")])


    def test_off_flag_releases_at_once_and_status_reports_it(self):
        job = {"job_id": "j2", "backend": "dummy", "attempt": 1, "seeds": [1], "spec": {"sleep_s": 30}}
        w = self.worker([job], heartbeat_s=60, pause_when=None)
        off = w.cfg["off_file"]
        threading.Timer(0.3, off.touch).start()
        t0 = time.time()
        w.loop(once=True)
        self.assertLess(time.time() - t0, 5)  # well inside one 60 s heartbeat
        self.assertIn(("/v1/release", {"job_ids": ["j2"]}), w.client.calls)
        w.loop(once=True)
        self.assertEqual(w.client.calls[-1][1]["stats"], {"reason": "off"})
        status = lambda: self._status(w.cfg)
        self.assertEqual(status(), "off")
        off.unlink()
        self.assertEqual(status(), "paused (off)")  # the daemon's last word until its next round
        w.note("running", job="j3")
        self.assertEqual(status(), "on, running job j3")

    def _status(self, cfg):
        import contextlib
        import io
        from unittest import mock
        buf = io.StringIO()
        with mock.patch.object(worker, "_systemd_unit", return_value=False), contextlib.redirect_stdout(buf):
            worker.cmd_status(None, cfg)
        return buf.getvalue().strip()


class InputWatch(unittest.TestCase):
    def test_key_events_count_and_sync_events_do_not(self):
        from unittest import mock
        import os
        r, wfd = os.pipe()
        os.set_blocking(r, False)
        watch = idle._InputWatch()
        with mock.patch.object(watch, "_open", side_effect=lambda: {r: "fake"}):
            first = watch.idle_seconds()
            self.assertIsNotNone(first)
            time.sleep(0.3)
            os.write(wfd, watch.EVENT.pack(0, 0, 0, 0, 0))  # EV_SYN only: not user input
            time.sleep(0.2)
            self.assertGreater(watch.idle_seconds(), 0.4)
            os.write(wfd, watch.EVENT.pack(0, 0, 1, 30, 1))  # EV_KEY
            for _ in range(50):
                if watch.idle_seconds() < 0.2:
                    break
                time.sleep(0.05)
            self.assertLess(watch.idle_seconds(), 0.2)

    def test_unknown_without_devices(self):
        from unittest import mock
        watch = idle._InputWatch()
        with mock.patch.object(watch, "_open", return_value={}):
            self.assertIsNone(watch.idle_seconds())


if __name__ == "__main__":
    unittest.main()
