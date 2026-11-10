# SPDX-License-Identifier: GPL-3.0-or-later
"""Cost ledger: wall time, CPU time and peak RSS per episode and per batch, appended to costs.jsonl (stdlib only).

    with costs.track("navpilot", unit="episode", n=20, fidelity="L1"):    # one batch of 20 episodes
        run_batch()
    @costs.timed("mission_sim", unit="mission")                           # one record per call
    def run_mission(...): ...

`costs.jsonl` (next to this file; override with $FRONTIER_COSTS) holds two record kinds:
  kind "audit"    seeded from the 2026-11-10 compute audit: {sim, unit, lo_s, hi_s, fidelity, source}
  kind "measured" appended by track()/timed(): {sim, unit, n, wall_s, cpu_s, max_rss_mb, per_unit_wall_s, ...}
`cost_per_unit(sim)` gives the planning cost in wall seconds per unit: median of measured records if any, else the
geometric mean of the audit range. The D4 selector reads costs only through this function.
Appends take an exclusive flock, so concurrent workers are safe. ru_maxrss is the process peak, not a delta.
"""
import fcntl
import functools
import json
import math
import os
import resource
import time

HERE = os.path.dirname(os.path.abspath(__file__))
DEFAULT_PATH = os.path.join(HERE, "costs.jsonl")


def costs_path():
    return os.environ.get("FRONTIER_COSTS") or DEFAULT_PATH


# (sim, unit, lo_s, hi_s, fidelity, source). Ranges are wall seconds per unit.
AUDIT = [
    ("forestsearch_greedy", "episode", 0.001, 0.008, "L1", "audit 2026-11-10: ForestSearch greedy 0.001-0.008 s/episode"),
    ("forestsearch_lookahead", "episode", 0.55, 1.24, "L1", "audit 2026-11-10: ForestSearch lookahead 0.55-1.24 s/episode"),
    ("nscale", "episode", 0.002, 0.040, "L1", "audit 2026-11-10: nscale 2-40 ms/episode"),
    ("navpilot", "episode", 1 / 0.75, 1 / 0.75, "L1", "audit 2026-11-10: NavPilot 0.75 episodes/s"),
    ("missionsim", "mission", 0.13, 0.28, "L1", "audit 2026-11-10: MissionSim 0.13-0.28 s/mission"),
    ("rolespec", "cell", 12.0, 37.0, "L1", "audit 2026-11-10: RoleSpec 12-37 s/cell"),
    ("dotnet_harness", "run", 32.0, 32.0, "L2", "audit 2026-11-10: .NET harness ~32 s/run"),
    ("unity_batch", "sim_second", 2.8, 2.8, "L3", "audit 2026-11-10: Unity batch 2.8x slower than real time"),
    ("unity_render", "frame", 0.375, 0.375, "L3", "audit 2026-11-10: Unity render 0.375 s/frame"),
    ("rtdetr", "frame", 0.0208, 0.0208, "L1", "audit 2026-11-10: RT-DETR 20.8 ms/frame"),
]


def seed_audit(path=None):
    """Write the audit records if the file does not yet contain them (idempotent)."""
    path = path or costs_path()
    have = set()
    if os.path.exists(path):
        for line in open(path):
            try:
                r = json.loads(line)
            except ValueError:
                continue
            if r.get("kind") == "audit":
                have.add((r["sim"], r["unit"]))
    for sim, unit, lo, hi, fid, src in AUDIT:
        if (sim, unit) not in have:
            _append(path, {"kind": "audit", "sim": sim, "unit": unit, "lo_s": lo, "hi_s": hi, "fidelity": fid, "source": src})


def _append(path, rec):
    line = (json.dumps(rec, sort_keys=True) + "\n").encode()
    fd = os.open(path, os.O_WRONLY | os.O_CREAT | os.O_APPEND, 0o644)
    try:
        fcntl.flock(fd, fcntl.LOCK_EX)
        os.write(fd, line)
    finally:
        os.close(fd)


class track:
    """Context manager: one record per batch of `n` units (per-unit means are derived)."""

    def __init__(self, sim, unit="episode", n=1, fidelity=None, note=None, path=None):
        self.sim, self.unit, self.n, self.fidelity, self.note, self.path = sim, unit, int(n), fidelity, note, path

    def __enter__(self):
        self.t0, self.c0 = time.time(), time.process_time()
        return self

    def __exit__(self, et, ev, tb):
        wall, cpu = time.time() - self.t0, time.process_time() - self.c0
        rss = resource.getrusage(resource.RUSAGE_SELF).ru_maxrss / 1024.0
        rec = {"kind": "measured", "sim": self.sim, "unit": self.unit, "n": self.n, "fidelity": self.fidelity,
               "wall_s": round(wall, 6), "cpu_s": round(cpu, 6), "max_rss_mb": round(rss, 1),
               "per_unit_wall_s": round(wall / max(self.n, 1), 6), "per_unit_cpu_s": round(cpu / max(self.n, 1), 6),
               "ok": et is None, "note": self.note, "ts": time.strftime("%Y-%m-%dT%H:%M:%S%z")}
        _append(self.path or costs_path(), rec)
        return False


def timed(sim, unit="episode", fidelity=None, path=None):
    def deco(fn):
        @functools.wraps(fn)
        def wrap(*a, **k):
            with track(sim, unit, 1, fidelity, path=path):
                return fn(*a, **k)
        return wrap
    return deco


def load(path=None):
    path = path or costs_path()
    out = []
    if os.path.exists(path):
        for line in open(path):
            try:
                out.append(json.loads(line))
            except ValueError:
                continue
    return out


def cost_per_unit(sim, path=None, recs=None):
    """Planning cost, wall seconds per unit: median of measured per-unit costs, else geometric mean of the audit range."""
    recs = recs if recs is not None else load(path)
    meas = sorted(r["per_unit_wall_s"] for r in recs if r.get("kind") == "measured" and r["sim"] == sim and r.get("ok", True))
    if meas:
        return meas[len(meas) // 2]
    for r in recs:
        if r.get("kind") == "audit" and r["sim"] == sim:
            return math.sqrt(r["lo_s"] * r["hi_s"])
    raise KeyError("no cost for sim %r" % sim)


if __name__ == "__main__":
    seed_audit()
    for r in load():
        if r["kind"] == "audit":
            print("%-24s %-10s %8.4g - %-8.4g s  %s" % (r["sim"], r["unit"], r["lo_s"], r["hi_s"], r["fidelity"]))
