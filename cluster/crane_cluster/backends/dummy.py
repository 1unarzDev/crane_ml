"""A backend that simulates nothing: sleeps, then reports a seeded random metric.

Used to test the cluster end to end. The value is ``mean + effect + noise(seed)``
plus a small per-arm jitter, so arms run on the same seed share their noise and a
paired test sees the effect. Job spec keys (overriding backend options):
sleep_s, mean, sd, effect, jitter_sd, fail_seeds, metric.
"""
from __future__ import annotations

import json
import random
import threading
import time
import zlib
from pathlib import Path

from . import Backend, register


@register("dummy")
class DummyBackend(Backend):
    def prepare(self, ctx):
        super().prepare(ctx)
        self._stop = threading.Event()

    def _opt(self, key, default):
        return self.ctx.spec.get(key, self.options.get(key, default))

    def run(self, seed: int) -> Path:
        d = self.run_dir(seed)
        if self._stop.wait(float(self._opt("sleep_s", 1.0))):
            raise RuntimeError("cancelled")
        if seed in self._opt("fail_seeds", []):
            raise RuntimeError(f"seed {seed} fails by request")
        noise = random.Random(seed).gauss(0.0, float(self._opt("sd", 0.1)))
        arm = str(self.ctx.job.get("arm") or "")
        jitter = random.Random(zlib.crc32(f"{arm}:{seed}".encode())).gauss(0.0, float(self._opt("jitter_sd", 0.02)))
        value = float(self._opt("mean", 0.5)) + float(self._opt("effect", 0.0)) + noise + jitter
        metric = str(self._opt("metric", "score"))
        (d / "metrics.json").write_text(json.dumps({metric: value}))
        (d / "summary.json").write_text(json.dumps({"seed": seed, "arm": arm, metric: value}))
        (d / "artifacts").mkdir(exist_ok=True)
        (d / "artifacts" / "log.txt").write_text(f"dummy run seed={seed} arm={arm} at {time.time():.0f}\n")
        return d

    def cancel(self):
        self._stop.set()
