#!/usr/bin/env bash
# Campaign expchoice-doorway-v0 — the exact submission commands. NOT EXECUTED by the session that wrote
# this. Do not run while another campaign holds the simulation slot unless its owner agrees: the
# mrmn_trial backend has one slot on innovation and these jobs queue behind whatever holds it.
#
# Project: modular (the mrmn_trial backend is that project's; its token is on this host). The campaign
# name keeps these runs apart from the modular study's in the feed and on disk
# (~/research-data/mrmn/expchoice-doorway-v0/). The project's fair share is shared with that study.
set -euo pipefail
echo "read this file; run its steps by hand" >&2; exit 1

# 0. The commit to run (branch research/expchoice-doorway; lives in the modular repository's object store,
#    which is the job's repository url — nothing is pushed).
COMMIT=$(git -C /home/lunarz/worktrees/mrmn-expchoice rev-parse HEAD)
git -C /home/lunarz/modular_robot_morphology_aware_navigation cat-file -e "$COMMIT^{commit}"
test -z "$(git -C /home/lunarz/worktrees/mrmn-expchoice status --porcelain)"

# 1. Job files (writes ~/.cache/expchoice/jobs/{canary,rep0,rep1}.json; submits nothing).
python3 -I /home/lunarz/worktrees/expchoice/Tools/ExperimentChoice/nav2/make_jobs.py "$COMMIT"

# 2. Canary: 2 jobs, 1 trial each (run_order 0 and 1: arm A and arm B on d = 1.20 m).
#    Estimate: about 8 min for the first (build + tests + trial), about 3 min for the second.
~/research-cluster/bin/research submit -p modular ~/.cache/expchoice/jobs/canary.json --wait --timeout 3600
~/research-cluster/bin/research status -p modular --campaign expchoice-doorway-v0
#    Then work through the canary checklist in BATCH_V0.md before anything else is submitted.

# 3. Replicate 0: 3 jobs, 18 trials (run_order 2-19). Estimate 1.2 h of slot time (range 0.7-1.6 h).
~/research-cluster/bin/research submit -p modular ~/.cache/expchoice/jobs/rep0.json
#    Interim look (BATCH_V0.md, stop rules) when all 20 replicate-0 records exist.

# 4. Replicate 1: 4 jobs, 20 trials (run_order 20-39). Estimate 1.3 h of slot time (range 0.8-1.8 h).
~/research-cluster/bin/research submit -p modular ~/.cache/expchoice/jobs/rep1.json

# Stop or inspect:
#   ~/research-cluster/bin/research guard -p modular
#   ~/research-cluster/bin/research cancel -p modular <job ids>
#   ~/research-cluster/bin/research batch pause <batch key> --why "<what you saw>"
