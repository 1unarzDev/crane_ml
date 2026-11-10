# SPDX-License-Identifier: GPL-3.0-or-later
"""Provenance sidecar for results files (stdlib only).

Every results file `X` gets `X.prov.json` with: git sha, dirty-diff hash (sha256 of `git diff HEAD` plus the names and
contents of untracked files under the repo's Tools/), config/spec hash, seeds, host, wall and CPU seconds and peak RSS.
Modelled on the claim-to-file table in crane-forest-sar Tools/ForestSearch/results/provenance.md, but machine-readable:
`prov.markdown_row(record, claim)` renders one row of that table.

    with prov.Timer() as t:
        ...run...
    prov.write_sidecar("results/x.jsonl", prov.collect(config=cfg, seeds=[1, 2], timer=t, code_files=[__file__]))
    rec = prov.read_sidecar("results/x.jsonl")
"""
import hashlib
import json
import os
import platform
import resource
import socket
import subprocess
import sys
import time

SCHEMA = "frontier-prov/1"
HERE = os.path.dirname(os.path.abspath(__file__))


class Timer:
    """Wall and process-CPU seconds, plus peak RSS (MB) of this process at exit (ru_maxrss is a process peak)."""

    def __enter__(self):
        self.t0, self.c0 = time.time(), time.process_time()
        return self

    def __exit__(self, *exc):
        self.wall_s = time.time() - self.t0
        self.cpu_s = time.process_time() - self.c0
        self.max_rss_mb = peak_rss_mb()
        return False


def peak_rss_mb():
    return resource.getrusage(resource.RUSAGE_SELF).ru_maxrss / 1024.0  # Linux: KB


def _git(args, cwd):
    try:
        return subprocess.run(["git"] + args, cwd=cwd, capture_output=True, timeout=30, check=True).stdout
    except (OSError, subprocess.SubprocessError):
        return None


def git_state(cwd=None):
    cwd = cwd or HERE
    sha = _git(["rev-parse", "HEAD"], cwd)
    top = _git(["rev-parse", "--show-toplevel"], cwd)
    if sha is None or top is None:
        return {"git_sha": None, "dirty": None, "diff_sha256": None, "branch": None}
    top = top.decode().strip()
    diff = _git(["diff", "HEAD"], top) or b""
    h = hashlib.sha256(diff)
    untracked = (_git(["ls-files", "--others", "--exclude-standard", "--", "Tools"], top) or b"").decode().split("\n")
    keep = [u for u in untracked if u and u.endswith((".py", ".cs", ".json", ".md", ".sh", ".csv"))
            and "scratch" not in u and "/results/" not in u]
    for name in sorted(keep):  # untracked source/spec files only: logs and scratch change constantly and would make the hash useless
        try:
            if os.path.getsize(os.path.join(top, name)) < (1 << 20):
                h.update(name.encode() + b"\0" + _read(os.path.join(top, name)))
        except OSError:
            pass
    branch = (_git(["rev-parse", "--abbrev-ref", "HEAD"], top) or b"").decode().strip()
    dirty = bool(diff) or bool(keep)
    return {"git_sha": sha.decode().strip(), "dirty": dirty, "diff_sha256": h.hexdigest(), "branch": branch}


def _read(p):
    with open(p, "rb") as f:
        return f.read()


def _sha_file(p):
    with open(p, "rb") as f:
        return hashlib.sha256(f.read()).hexdigest()


def spec_hash(obj):
    return hashlib.sha256(json.dumps(obj, sort_keys=True, separators=(",", ":"), default=str).encode()).hexdigest()


def collect(config=None, seeds=None, timer=None, code_files=None, extra=None, cwd=None):
    rec = {"schema": SCHEMA, "created": time.strftime("%Y-%m-%dT%H:%M:%S%z")}
    rec.update(git_state(cwd))
    rec["config_sha256"] = spec_hash(config) if config is not None else None
    rec["seeds"] = list(seeds) if seeds is not None else None
    rec["host"] = {"hostname": socket.gethostname(), "platform": platform.platform(), "python": sys.version.split()[0],
                   "cpus": os.cpu_count()}
    if timer is not None:
        rec["wall_s"], rec["cpu_s"], rec["max_rss_mb"] = round(timer.wall_s, 4), round(timer.cpu_s, 4), round(timer.max_rss_mb, 1)
    if code_files:
        rec["code_files"] = {os.path.basename(p): _sha_file(p) for p in code_files}
    if extra:
        rec["extra"] = extra
    return rec


def sidecar_path(results_path):
    return results_path + ".prov.json"


def write_sidecar(results_path, record):
    p = sidecar_path(results_path)
    tmp = p + ".tmp%d" % os.getpid()
    with open(tmp, "w") as f:
        json.dump(record, f, indent=1, sort_keys=True)
        f.write("\n")
    os.replace(tmp, p)
    return p


def read_sidecar(results_path):
    with open(sidecar_path(results_path)) as f:
        return json.load(f)


def markdown_row(record, claim, results_file, status="exploratory"):
    sha = (record.get("git_sha") or "no-git")[:10] + ("+dirty" if record.get("dirty") else "")
    return "| %s | %s | %s (git %s, host %s) |" % (claim, results_file, status, sha, record["host"]["hostname"])
