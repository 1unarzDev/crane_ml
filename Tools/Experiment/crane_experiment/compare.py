"""Equal-budget, replicated optimizer comparison over the fake worker. FIXTURE ONLY."""
import json
import os
import tempfile

import numpy as np

from .fake_worker import FakeWorker
from .fixtures import demo_spec
from .runner import run_study
from .stats import bootstrap_ci, mannwhitney_less
from .study import Study

ALL = ["default", "random", "sobol", "cmaes", "tpe", "gp_bo", "contextual_gp", "mfbo"]


def _regret(study, worker):
    """Mean true regret of the recommendation on held-out contexts (noise-free fake truth)."""
    r = []
    for c in study.held:
        cfg = study.recommend_config(c["id"])
        u = study.space.to_unit(cfg)
        r.append(worker.true_regret(u, c["features"][0]))
    return float(np.mean(r))


def compare(optimizers=ALL, replicates=5, n_search=40, seed=20260101, feedback="sanitized",
            worker=None):
    worker = worker or FakeWorker()
    res = {o: {"regret": [], "decision": [], "n_search_trials": [], "loophole_trials": [],
               "invalid_trials": [], "spent": []} for o in optimizers}
    for r in range(replicates):
        spec = demo_spec(seed=seed + r, n_search=n_search)
        for o in optimizers:
            with tempfile.TemporaryDirectory() as d:
                st = Study.create(os.path.join(d, "s"), spec, {"optimizer": o, "feedback": feedback,
                                                               "worker": "fake"}, fsync=False)
                run_study(st, worker)
                st.freeze("human")
                rep = st.report()
                a = res[o]
                a["regret"].append(_regret(st, worker))
                a["decision"].append(rep["decision"]["action"] + (":" + rep["decision"]["reason"] if rep["decision"]["reason"] else ""))
                a["n_search_trials"].append(sum(1 for t in st.trials.values() if t["kind"] == "search"))
                a["loophole_trials"].append(rep["accounting"]["by_status"]["loophole"])
                a["invalid_trials"].append(rep["accounting"]["by_status"]["invalid"] + rep["accounting"]["by_status"]["stale"])
                a["spent"].append(rep["accounting"]["budget_spent"])
    table = {}
    for o, a in res.items():
        q = np.percentile(a["regret"], [25, 50, 75])
        table[o] = {"median_regret": float(q[1]), "q25": float(q[0]), "q75": float(q[2]),
                    "iqr": float(q[2] - q[0]),
                    "median_ci95": list(bootstrap_ci(a["regret"])), "mean_search_trials": float(np.mean(a["n_search_trials"])),
                    "mean_spent": float(np.mean(a["spent"])), **a}
    rnd = table.get("random", {}).get("median_regret")
    p_vs_random = ({o: mannwhitney_less(t["regret"], table["random"]["regret"])
                    for o, t in table.items() if o != "random"} if "random" in table else None)
    return {
        "p_less_than_random_mannwhitney": p_vs_random,
        "label": "FIXTURE_ONLY", "evidence": "NOT_EVIDENCE",
        "note": "Synthetic quadratic fake worker; says nothing about CRANE, Nav2 or any real system.",
        "replicates": replicates, "n_search_trials_nominal": n_search, "seed": seed,
        "budget": "equal total cost per optimizer; common random numbers within a replicate",
        "metric": "mean noise-free regret of the recommendation on held-out contexts (lower is better)",
        "table": table,
        "beats_random": ({o: bool(t["median_regret"] < rnd) for o, t in table.items() if o != "random"}
                         if rnd is not None else None),
    }


def format_table(rep):
    lines = [f"{'optimizer':<15}{'median':>9}{'CI95 lo':>9}{'CI95 hi':>9}{'IQR':>9}"]
    for o, t in rep["table"].items():
        lines.append(f"{o:<15}{t['median_regret']:>9.4f}{t['median_ci95'][0]:>9.4f}{t['median_ci95'][1]:>9.4f}{t['iqr']:>9.4f}")
    return "\n".join(lines)


def save(rep, path):
    os.makedirs(os.path.dirname(path), exist_ok=True)
    with open(path, "w", encoding="utf-8") as fh:
        json.dump(rep, fh, indent=1, sort_keys=True)
