# SPDX-License-Identifier: GPL-3.0-or-later
"""DESIGN ONLY.  Emits rc-job/1 JSON for the learned-policy jobs of D2 (behaviour cloning from exact solutions, then PPO)
through Tools/Frontier/jobs.py.  NOTHING is submitted and the training scripts named in the commands (bs_learn_bc.py,
bs_learn_ppo.py) are NOT written yet: they are specified in results/DEV.md section 9.  Run:  python bs_learned_jobs.py"""
import json, os, sys
HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(HERE, "..", "Frontier"))
import jobs as J  # noqa: E402


def build():
    out = []
    out += J.cpu_shard_jobs(
        "d2_exact_dataset", "Generate exact-solver (expectimax) state-action pairs on random tiny search-verify-report instances "
        "(<= 6 cells, 2 view classes, horizon <= 6) for behaviour cloning of a belief-space verification policy.",
        "python Tools/BeliefSVR/bs_learn_bc.py gen --instances", list(range(0, 288)), 30, 18, campaign="d2_learned",
        priority_class="sim", split="dev")
    out.append(J.gpu_job(
        "d2-bc-mlp", "Behaviour-clone the exact expectimax action (report / look same view / look independent view / partner / move) "
        "from belief features with a small MLP; DEV instances only, checkpoint every epoch.",
        "python Tools/BeliefSVR/bs_learn_bc.py train --data d2_exact_dataset --hidden 128", 900, vram_gb=2.0, campaign="d2_learned"))
    out.append(J.gpu_job(
        "d2-ppo-belief", "PPO fine-tune of the behaviour-cloned belief-space verification policy in the BeliefSVR mission simulator on DEV "
        "layouts, with an explicit false-report penalty, checkpoint at every update, resume-safe.",
        "python Tools/BeliefSVR/bs_learn_ppo.py --init d2-bc-mlp --layouts 100-219", 4 * 3600, vram_gb=4.0, campaign="d2_learned",
        extra_spec={"depends_on": "d2-bc-mlp"}))
    return out


if __name__ == "__main__":
    jobs = build()
    J.dump(jobs, os.path.join(HERE, "jobs", "d2_learned_jobs.json"))
    print(len(jobs), "jobs written (not submitted)")
