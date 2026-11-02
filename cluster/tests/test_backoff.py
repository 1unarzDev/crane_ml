"""Clients wait out an unavailable coordinator (503 out of quota, 5xx, unreachable): 5 s doubling to 5 min."""
import io
import json
import threading
import unittest
import urllib.error
from contextlib import redirect_stderr
from unittest import mock

from crane_cluster import api, hyp, worker
from crane_cluster.api import ApiError, Backoff, unavailable


def http_error(code, body, retry_after=None):
    hdrs = {"retry-after": str(retry_after)} if retry_after is not None else {}
    msg = mock.Mock()
    msg.get = lambda k, d=None: hdrs.get(k.lower(), d)
    return urllib.error.HTTPError("http://x", code, "err", msg, io.BytesIO(json.dumps(body).encode()))


class BackoffTest(unittest.TestCase):
    def test_doubles_to_the_cap_and_honours_retry_after_up_to_it(self):
        b = Backoff()
        self.assertEqual([b.step() for _ in range(9)], [5, 10, 20, 40, 80, 160, 300, 300, 300])
        b.reset()
        e = ApiError(503, {"error": "D1 daily quota exhausted"}, retry_after=8649)
        self.assertEqual(b.step(e), 300)  # Retry-After lengthens a step, never past 5 min
        b.reset()
        self.assertEqual(b.step(ApiError(503, {}, retry_after=30)), 30)

    def test_classifies_unavailable(self):
        self.assertIn("quota", unavailable(ApiError(503, {"error": "D1 daily quota exhausted"})))
        self.assertIn("HTTP 502", unavailable(ApiError(502, "bad gateway")))
        self.assertIn("unreachable", unavailable(urllib.error.URLError("refused")))
        self.assertIsNone(unavailable(ApiError(401, {"error": "unauthorized"})))
        self.assertIsNone(unavailable(ApiError(409, {"error": "not the lease holder"})))

    def test_client_reads_retry_after(self):
        c = api.Client("http://x", retries=0)
        with mock.patch("urllib.request.urlopen", side_effect=http_error(503, {"error": "quota"}, 120)):
            with self.assertRaises(ApiError) as cm:
                c.get("/v1/ping")
        self.assertEqual((cm.exception.status, cm.exception.retry_after), (503, 120.0))


class HypTest(unittest.TestCase):
    def test_get_waits_through_503_and_unreachable(self):
        c = hyp.WaitingClient("http://x")
        slept = []
        c.sleep = slept.append
        calls = iter([ApiError(503, {"error": "quota"}, 60), OSError("refused"), {"ok": True}])

        def fake(self, path, **q):
            v = next(calls)
            if isinstance(v, Exception):
                raise v
            return v

        err = io.StringIO()
        with mock.patch.object(api.Client, "get", fake), redirect_stderr(err):
            self.assertEqual(c.get("/v1/hyp"), {"ok": True})
        self.assertEqual(slept, [60, 10])
        self.assertIn("coordinator unavailable (HTTP 503: quota)", err.getvalue())

    def test_get_raises_a_4xx_at_once(self):
        c = hyp.WaitingClient("http://x")
        c.sleep = lambda s: self.fail("must not wait on a 4xx")
        with mock.patch.object(api.Client, "get", side_effect=ApiError(404, {"error": "unknown hypothesis"})):
            with self.assertRaises(ApiError):
                c.get("/v1/hyp/h_x")

    def test_wait_until_up(self):
        c = api.Client("http://x")
        seq = iter([ApiError(503, {"error": "quota"}), ApiError(500, "boom"), {"ok": True}])

        def req(method, path, retries=None, **kw):
            v = next(seq)
            if isinstance(v, Exception):
                raise v
            return v

        c.request = req
        lines, slept = [], []
        api.wait_until_up(c, log=lines.append, sleep=slept.append)
        self.assertEqual(slept, [5, 10])
        self.assertEqual(len(lines), 2)


class WorkerLoopTest(unittest.TestCase):
    def make(self, responses):
        w = worker.Worker.__new__(worker.Worker)
        w.cfg = {"poll_s": 30, "heartbeat_s": 60, "long_poll_s": 25, "data_dir": mock.MagicMock()}
        w.policy, w.caps, w.paused = {}, {"backends": ["dummy"]}, None
        w.stop = threading.Event()
        w.backoff = Backoff()
        w.note = lambda *a, **k: None
        w.between_jobs = lambda v: None
        w.run_job = mock.Mock(side_effect=lambda *a: w.stop.set())
        seq = iter(responses)
        bodies = []

        def post(path, body):
            bodies.append(body)
            v = next(seq)
            if isinstance(v, Exception):
                raise v
            return v

        w.client = mock.Mock()
        w.client.post = post
        waits = []

        def wait(s):
            waits.append(round(s))
            if len(bodies) >= len(responses):
                w.stop.set()
            return False

        w.stop.wait = wait
        return w, bodies, waits

    def test_backs_off_on_503_and_unreachable_then_long_polls(self):
        job = {"job_id": "j1"}
        w, bodies, waits = self.make([
            ApiError(503, {"error": "D1 daily quota exhausted"}, 9000),
            OSError("refused"),
            ApiError(502, "bad gateway"),
            {"jobs": [job], "repos": {}, "lease_ttl_s": 300},
        ])
        with mock.patch.object(worker.idle, "unavailable_reason", return_value=None), \
             mock.patch.object(worker.idle, "lower_priority"):
            w.loop()
        self.assertEqual(waits[:3], [300, 10, 20])  # Retry-After capped at 5 min, then doubling from where it was
        w.run_job.assert_called_once_with(job, 300.0)
        self.assertEqual(bodies[0]["wait_s"], 25)
        self.assertEqual(w.backoff.next, 5)  # reset after the coordinator answered

    def test_a_4xx_other_than_426_still_raises(self):
        w, _, _ = self.make([ApiError(401, {"error": "unauthorized"})])
        with mock.patch.object(worker.idle, "unavailable_reason", return_value=None), \
             mock.patch.object(worker.idle, "lower_priority"):
            with self.assertRaises(ApiError):
                w.loop()


if __name__ == "__main__":
    unittest.main()
