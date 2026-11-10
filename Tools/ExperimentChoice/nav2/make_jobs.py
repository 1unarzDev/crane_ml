"""Write the queue job files for campaign expchoice-doorway-v0 (EXPLORATORY). Submits nothing.

  python3 -I Tools/ExperimentChoice/nav2/make_jobs.py <full commit of research/expchoice-doorway>

Reads the frozen design from the modular-study worktree at that commit (git show, no checkout) and writes
~/.cache/expchoice/jobs/{canary,rep0,rep1}.json in the shape the `mrmn_trial` backend takes: one seed is
one trial, seed = run_order. The repository url is the modular study's main checkout; the worktree's
branch lives in the same object store, so nothing is pushed or bundled.
"""
import json
import os
import subprocess
import sys

MRMN = "/home/lunarz/worktrees/mrmn-expchoice"
URL = "/home/lunarz/modular_robot_morphology_aware_navigation"
DESIGN = "studies/expchoice_doorway_v0/design.json"
CAMPAIGN = "expchoice-doorway-v0"
SUBMITTER = "expchoice-nav2"
OUT = os.path.expanduser("~/.cache/expchoice/jobs")
STOP = ("Canary gate: both records valid, arm A completes on the 1.20 m bay, real-time factor at least 0.5, "
        "expected plugins in the launch log. Interim look after replicate 0 (run_order 0-19): stop if more "
        "than 4 of 20 are infrastructure failures, median real-time factor is under 0.5, or the roomy "
        "courses show contact in 3 or more trials. Replicate 1 runs only if replicate 0 passes.")


def job(job_id, seeds, commit, purpose, est):
    return {"schema": "rc-job/1", "job_id": job_id, "kind": "mrmn_trial", "seeds": seeds,
            "repos": [{"name": "mrmn", "url": URL, "ref": commit}],
            "spec": {"design": DESIGN, "campaign": CAMPAIGN}, "campaign": CAMPAIGN,
            "purpose": purpose, "est_seconds": est, "submitter": SUBMITTER,
            "expect": {"owner": SUBMITTER, "metrics": ["record_ok", "real_time_factor"], "stop": STOP}}


def main():
    commit = sys.argv[1]
    assert len(commit) == 40, "pass the full commit id"
    envelope = json.loads(subprocess.check_output(["git", "-C", MRMN, "show", "%s:%s" % (commit, DESIGN)]))
    trials = envelope["design"]["trials"]
    assert [t["run_order"] for t in trials] == list(range(len(trials)))
    short = envelope["design_hash"][:8]
    by = {t["run_order"]: t for t in trials}

    def describe(seeds):
        ds = sorted({by[s]["d_cm"] for s in seeds})
        return "bay depths %s cm" % ", ".join(str(d) for d in ds)

    os.makedirs(OUT, exist_ok=True)
    canary = [job("ecd0-canary-%d" % s, [s], commit,
                  "Doorway study canary %d of 2: arm %s on the roomiest course (bay 1.20 m, offset 0.70 m), where both "
                  "Nav2 configurations are expected to reach the goal. First execution of the doorway runner. Design %s."
                  % (s + 1, by[s]["arm"], short),
                  # The first trial of a commit also builds the workspace and runs the test suite (about 5 min).
                  480 if s == 0 else 180) for s in (0, 1)]
    chunks = {"rep0": [list(range(2, 8)), list(range(8, 14)), list(range(14, 20))],
              "rep1": [list(range(20, 26)), list(range(26, 32)), list(range(32, 38)), [38, 39]]}
    files = {"canary": canary}
    for name, groups in chunks.items():
        files[name] = [job("ecd0-%s-%02d" % (name, i + 1), seeds, commit,
                           "Doorway study %s chunk %d: paired trials of a rotate-only and a reverse-capable Nav2 "
                           "configuration on offset-doorway courses, %s, run_order %d-%d. Tests whether the faster "
                           "configuration changes with bay depth. Design %s."
                           % ("replicate 0" if name == "rep0" else "replicate 1", i + 1, describe(seeds),
                              seeds[0], seeds[-1], short), 240)
                       for i, seeds in enumerate(groups)]
    covered = sorted(s for jobs in files.values() for j in jobs for s in j["seeds"])
    assert covered == list(range(len(trials))), "every run_order exactly once"
    for name, jobs in files.items():
        for j in jobs:
            assert 20 <= len(j["purpose"]) <= 600 and "/" not in j["purpose"]
        with open(os.path.join(OUT, name + ".json"), "w", encoding="utf-8") as f:
            json.dump(jobs, f, indent=1)
        print(name, len(jobs), "jobs", sum(len(j["seeds"]) for j in jobs), "trials",
              "est %.1f h" % (sum(len(j["seeds"]) * j["est_seconds"] for j in jobs) / 3600))


if __name__ == "__main__":
    main()
