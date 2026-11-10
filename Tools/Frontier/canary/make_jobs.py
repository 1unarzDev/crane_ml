# SPDX-License-Identifier: GPL-3.0-or-later
"""Emit canary jobs.json (2 rc-job/1 bodies, SPEC.md 2.1) for `research submit -p drones jobs.json`.  NOT SUBMITTED.

Status: BLOCKED on a user decision. No worker backend runs PyTorch training today (kinds offered: fastsim, unity, llm-batch,
ald-shard, command); 'command' is assumed here and may be refused. Target host: harmony (<= 8 GB VRAM).
Job 2 depends on job 1's checkpoint: it uses `needs.after`-free sequencing by submitting job 2 only after `research wait` on job 1
(or with the same anti_affinity host pin so the checkpoint dir is shared). Preemption is exercised deliberately: job 1 stops
at its 30 s checkpoint (--stop-after-ckpt); job 2 resumes and finishes the 60 s budget.
"""
import json, sys

HOST_TAG = "host:harmony"  # assumption: the worker advertises a host tag; replace with the real needs.host key


def job(i, extra):
    return {"schema": "rc-job/1", "job_id": "drones.frontier.gpucanary.%d" % i, "kind": "command",
            "purpose": "GPU canary %d of 2 for the drones project: a tiny MLP trained for 60 s with a checkpoint at 30 s and a resume test; checks the queue, CUDA and checkpoint path before real training." % i,
            "submitter": "drones-frontier-ops", "campaign": "frontier_gpu_canary", "priority_class": "sim", "priority": 100,
            "est_seconds": 90,
            "needs": {"os": ["linux"], "gpu": True, "vram_gb": 1.0, "min_ram_gb": 4, "tags": [HOST_TAG]},
            "preemptible": True, "anti_affinity": {"keys": [], "scope": "host"}, "inputs": [], "items": None, "seeds": None,
            "repos": [{"name": "crane-drones", "url": "<crane-drones remote>", "ref": "research/drone-sim"}],
            "spec": {"argv": ["python", "Tools/Frontier/canary/canary_train.py", "--out", "$JOB_SCRATCH/canary", "--budget-s", "60",
                              "--ckpt-at-s", "30", "--device", "cuda"] + extra},
            "outputs": {"class": "summary"}, "max_attempts": 2, "lease_ttl_s": 300}


if __name__ == "__main__":
    json.dump({"jobs": [job(1, ["--stop-after-ckpt"]), job(2, ["--resume"])]}, open(sys.argv[1] if len(sys.argv) > 1 else "jobs.json", "w"), indent=1)
