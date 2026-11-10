# SPDX-License-Identifier: GPL-3.0-or-later
"""Emit research-cluster job JSON (rc-job/1) for `research submit -p drones JOBS.json`. NEVER submits anything.

Schema: ~/research-cluster/SPEC.md section 2.1 and docs/COORDINATION.md section 5. Output is `{"jobs": [...]}`.

CPU sweeps (`cpu_shard_jobs`): kind `fastsim`, `seeds` split into shards of 15-20 (COMPUTE.md: per-job overhead, not CPU,
is the limit; use 15-20 seeds per job for sweeps of >= 200 runs, still >= 12 jobs so every slot gets one; 5 per job
only for small runs). `spec` is never published, so it carries the command.

GPU jobs (`gpu_job`): SPEC ONLY. No PyTorch training backend exists on the cluster yet (adding one is the owner's
decision), so the kind is the placeholder `command` and the job will not be leased until a backend offers it. Target is
the harmony host (RTX 3060 12 GB): `needs.gpu`, `vram_gb <= 8`, `preemptible: true`. The command must be
checkpoint/resume-safe: it is started with `--resume` and `--checkpoint-dir` (see train.py) so a yield between seeds
or epochs loses at most one checkpoint interval.

    python jobs.py cpu --name d4_recfeas --cmd "python Tools/Frontier/mf/run.py" --seeds 0-63 --est-seconds 600 --out jobs.json
    python jobs.py gpu --name d1_gate_mlp --cmd "python Tools/Frontier/train.py --device cuda ..." --vram-gb 4 --out jobs.json
"""
import argparse
import json
import re
import sys

PROJECT = "drones"
MAX_VRAM_GB = 8.0
SHARD_MIN, SHARD_MAX = 15, 20
ID_RE = re.compile(r"^[A-Za-z0-9][A-Za-z0-9._:-]{2,127}$")


def parse_seeds(s):
    out = []
    for part in str(s).split(","):
        if "-" in part:
            a, b = part.split("-")
            out += list(range(int(a), int(b) + 1))
        elif part:
            out.append(int(part))
    return out


def shard(seeds, size=18):
    """Split into shards of 15-20 seeds, as even as possible; a remainder is spread, not left as a runt."""
    if not SHARD_MIN <= size <= SHARD_MAX:
        raise ValueError("shard size must be %d-%d" % (SHARD_MIN, SHARD_MAX))
    n = len(seeds)
    if n <= SHARD_MAX:
        return [list(seeds)]
    k = -(-n // size)
    base, extra = divmod(n, k)
    out, i = [], 0
    for j in range(k):
        m = base + (1 if j < extra else 0)
        out.append(list(seeds[i:i + m]))
        i += m
    return out


def _check_id(jid):
    if not ID_RE.match(jid):
        raise ValueError("bad job_id %r" % jid)


def _check_purpose(p):
    if not 20 <= len(p) <= 600:
        raise ValueError("purpose must be 20-600 chars (published text; no paths, hosts or prompts)")
    if "/home/" in p or "/tmp/" in p:
        raise ValueError("purpose must not contain local paths")


def cpu_shard_jobs(name, purpose, cmd, seeds, est_seconds_per_seed, shard_size=18, campaign=None, priority=100,
                   priority_class="sim", split="dev", extra_spec=None, submitter="frontier-d4"):
    _check_purpose(purpose)
    if split == "heldout":
        raise ValueError("held-out jobs are never emitted here: they need a frozen prereg and the owner's go-ahead")
    jobs = []
    for i, sh in enumerate(shard(list(seeds), shard_size)):
        jid = "%s.s%03d" % (name, i)
        _check_id(jid)
        jobs.append({"schema": "rc-job/1", "job_id": jid, "kind": "fastsim", "purpose": purpose,
                     "est_seconds": int(est_seconds_per_seed), "submitter": submitter, "campaign": campaign or name,
                     "priority_class": priority_class, "priority": priority,
                     "needs": {"os": ["linux"], "tags": ["nlc", "linux"], "min_cpu": 1},
                     "preemptible": True, "seeds": sh,
                     "spec": dict({"cmd": cmd, "split": split, "seeds": [sh[0], sh[-1]], "n_seeds": len(sh)}, **(extra_spec or {})),
                     "outputs": {"class": "summary"}, "max_attempts": 3})
    return jobs


def gpu_job(name, purpose, cmd, est_seconds, vram_gb=4.0, host="harmony", campaign=None, priority=100,
            priority_class="background", checkpoint_dir="checkpoints", extra_spec=None, submitter="frontier-train"):
    _check_purpose(purpose)
    if vram_gb > MAX_VRAM_GB:
        raise ValueError("vram_gb %.1f exceeds the %.0f GB class limit" % (vram_gb, MAX_VRAM_GB))
    _check_id(name)
    full_cmd = "%s --device cuda --checkpoint-dir %s --resume" % (cmd, checkpoint_dir)
    return {"schema": "rc-job/1", "job_id": name, "kind": "command", "purpose": purpose,
            "est_seconds": int(est_seconds), "submitter": submitter, "campaign": campaign or name,
            "priority_class": priority_class, "priority": priority,
            "needs": {"host": host, "gpu": True, "vram_gb": float(vram_gb), "os": ["linux", "windows"]},
            "preemptible": True, "max_attempts": 5, "lease_ttl_s": 300,
            "spec": dict({"cmd": full_cmd, "checkpoint_dir": checkpoint_dir, "resume": True,
                          "_note": "SPEC ONLY: no PyTorch training backend exists; kind 'command' is a placeholder"},
                         **(extra_spec or {})),
            "outputs": {"class": "summary"}}


def dump(jobs, out=None):
    text = json.dumps({"jobs": jobs}, indent=1, sort_keys=True) + "\n"
    if out:
        with open(out, "w") as f:
            f.write(text)
    else:
        sys.stdout.write(text)


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    sub = ap.add_subparsers(dest="mode", required=True)
    c = sub.add_parser("cpu")
    c.add_argument("--name", required=True)
    c.add_argument("--purpose", default="Retrospective multi-fidelity selection sweep over dev seeds with the frozen frontier harness.")
    c.add_argument("--cmd", required=True)
    c.add_argument("--seeds", required=True)
    c.add_argument("--est-seconds", type=int, default=60, help="per seed")
    c.add_argument("--shard-size", type=int, default=18)
    c.add_argument("--out")
    g = sub.add_parser("gpu")
    g.add_argument("--name", required=True)
    g.add_argument("--purpose", default="Train a small decision-gate model on cached coordination-regret labels, with checkpoints at each epoch.")
    g.add_argument("--cmd", required=True)
    g.add_argument("--est-seconds", type=int, default=1800)
    g.add_argument("--vram-gb", type=float, default=4.0)
    g.add_argument("--host", default="harmony")
    g.add_argument("--out")
    a = ap.parse_args(argv)
    if a.mode == "cpu":
        jobs = cpu_shard_jobs(a.name, a.purpose, a.cmd, parse_seeds(a.seeds), a.est_seconds, a.shard_size)
    else:
        jobs = [gpu_job(a.name, a.purpose, a.cmd, a.est_seconds, a.vram_gb, a.host)]
    dump(jobs, a.out)
    print("emitted %d job(s); NOT submitted" % len(jobs), file=sys.stderr)


if __name__ == "__main__":
    main()
