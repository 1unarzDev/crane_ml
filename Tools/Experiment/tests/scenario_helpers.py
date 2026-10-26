"""Shared scenario builders for tests."""
import os

import numpy as np

from crane_experiment.fake_worker import FakeWorker
from crane_experiment.fixtures import demo_spec
from crane_experiment.runner import run_study
from crane_experiment.study import Study


def one_context_spec(seed=3, n_search=40, **kw):
    spec = demo_spec(seed=seed, n_search=n_search, n_train=1, **kw)
    spec["contexts"][0]["features"] = [0.5]
    return spec


def make_study(tmp_path, spec=None, optimizer="random", feedback="sanitized", worker="fake", name="s"):
    spec = spec or demo_spec()
    return Study.create(os.path.join(str(tmp_path), name), spec,
                        {"optimizer": optimizer, "feedback": feedback, "worker": worker}, fsync=False)


def naive_loophole_scenario(tmp_path, seed=3, optimizer="gp_bo"):
    """A reward-trusting optimizer (raw feedback) searches a space that contains the loophole."""
    st = make_study(tmp_path, one_context_spec(seed=seed), optimizer=optimizer, feedback="raw")
    run_study(st, FakeWorker())
    rep = st.report()
    opt_u = FakeWorker.optimum(0.5)
    u = st.space.to_unit(rep["recommended_config"])
    return st, rep, float(np.max(np.abs(u - opt_u)))
