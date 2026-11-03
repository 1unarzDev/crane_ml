"""Job slots: how many jobs of each backend a worker runs at once.

Without a ``slots:`` section a worker runs one job at a time (the old behaviour). With one, each backend
gets concurrent slots from its options in ``backends:``::

    slots:
      cpus: 16          # the core budget jobs share (default: half the cores)
      reserve_gb: 3     # MemAvailable kept free (default 2)
      gpus: 1           # GPU jobs at once (default: the detected GPU count, at least 1)
    backends:
      nlc_task: {type: command, ..., slots: 2, cpus: 8, mem_gb: 3.8}

Per backend: ``slots`` (its own cap), ``cpus`` (cores one job uses), ``mem_gb`` (its memory estimate) and
``gpu`` (it holds a GPU slot; defaults to true for unity and theater backends). Defaults: GPU backends one
per GPU, 2 cores, 4 GB; dummy 8 slots, no cores or memory; any other as many as the core budget allows,
1 core and 1 GB each. A job starts only while the core budget, the GPU slots and MemAvailable (less the
reserve and the estimates of jobs started in the last ``ramp_s``, whose memory may not show yet) admit it;
an idle worker always admits one job.
"""
from __future__ import annotations

import os
import threading
import time
from collections import Counter

from . import config as config_mod

KEYS = ("slots", "cpus", "mem_gb", "gpu")   # slot options, removed before a backend sees its options
GPU_HINTS = ("unity", "theater")


class Slots:
    def __init__(self, cfg: dict, caps: dict, mem_gb=config_mod.mem_available_gb, clock=time.monotonic):
        conf = cfg.get("slots")
        self.multi = isinstance(conf, dict)
        conf = conf if self.multi else {}
        cores = int(caps.get("cpu") or os.cpu_count() or 1)
        self.cpus = float(conf.get("cpus", max(1, cores // 2)))
        self.reserve = float(conf.get("reserve_gb", 2))
        self.ramp_s = float(conf.get("ramp_s", 60))
        self.gpus = max(1, int(conf.get("gpus", caps.get("gpu_count") or 1)))
        self.max_jobs = int(conf.get("max_jobs", 64)) if self.multi else 1
        self.mem_gb, self.clock = mem_gb, clock
        self.spec: dict[str, dict] = {}
        for name, opts in (cfg.get("backends") or {}).items():
            opts = opts or {}
            kind = f"{name} {opts.get('type', name)}".lower()
            gpu = bool(opts.get("gpu", any(h in kind for h in GPU_HINTS)))
            dummy = str(opts.get("type", name)) == "dummy"
            self.spec[name] = {
                "gpu": gpu,
                "slots": int(opts.get("slots", self.gpus if gpu else 8 if dummy else max(1, int(self.cpus)))),
                "cpus": float(opts.get("cpus", 0 if dummy else 2 if gpu else 1)),
                "mem_gb": float(opts.get("mem_gb", 0 if dummy else 4 if gpu else 1)),
            }
        self.running: dict[str, tuple[str, float, int]] = {}   # job id -> (backend, started, slot index)
        self.lock = threading.Lock()

    def free(self) -> dict[str, int]:
        """Backend -> how many more of its jobs could start now (counting each backend alone)."""
        with self.lock:
            n = len(self.running)
            if n >= self.max_jobs:
                return {}
            by = Counter(b for b, _, _ in self.running.values())
            cpu_left = self.cpus - sum(self.spec.get(b, {}).get("cpus", 0) for b in by.elements())
            gpu_left = self.gpus - sum(1 for b in by.elements() if self.spec.get(b, {}).get("gpu"))
            mem = self.mem_gb()
            if mem is not None:
                now = self.clock()
                mem -= self.reserve + sum(self.spec.get(b, {}).get("mem_gb", 0) for b, t, _ in self.running.values()
                                          if now - t < self.ramp_s)
            out = {}
            for b, s in self.spec.items():
                k = min(s["slots"] - by[b], self.max_jobs - n)
                if s["gpu"]:
                    k = min(k, gpu_left)
                fits = k
                if s["cpus"] > 0:
                    fits = min(fits, int(cpu_left // s["cpus"]))
                if mem is not None and s["mem_gb"] > 0:
                    fits = min(fits, int(mem // s["mem_gb"]))
                k = fits if n else min(k, max(1, fits))   # an idle worker always admits one job
                if k > 0:
                    out[b] = k
            return out

    def acquire(self, job_id: str, backend: str) -> int:
        """Registers a started job; returns its slot index (the lowest unused), for its ports and isolation id."""
        with self.lock:
            used = {i for _, _, i in self.running.values()}
            idx = next(i for i in range(len(used) + 1) if i not in used)
            self.running[job_id] = (backend, self.clock(), idx)
            return idx

    def release(self, job_id: str) -> None:
        with self.lock:
            self.running.pop(job_id, None)
