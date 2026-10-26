"""crane-physical-calibration-dataset-v1 (heldout) -> crane.dataset_manifest/v2."""
from __future__ import annotations

import copy

from .. import model
from . import common as c

V1_SCHEMA = "crane-physical-calibration-dataset-v1"
COPY = ("timeBasis", "coordinateFrame", "timing", "frameTransform", "calibrationProfile", "acceptance",
        "heldOutFactors", "simTimestampField", "realTimestampField")


def migrate(v1: dict) -> dict:
    if v1.get("schema") != V1_SCHEMA:
        raise ValueError("not a heldout dataset manifest v1")
    trials = []
    for t in v1["trials"]:
        f = t.get("factors", {})
        nt = copy.deepcopy(t)
        nt["site"] = f.get("site")
        nt["session"] = f.get("session", f.get("day"))
        nt["runId"] = str(f.get("runId", t["id"]))
        trials.append(nt)
    doc = {"schema": model.MANIFEST_SCHEMA, "kind": "dataset_manifest",
           "origin": "synthetic-fixture" if v1["sourceKind"] == "synthetic-fixture" else "physical",
           **{k: copy.deepcopy(v1[k]) for k in COPY if k in v1}, "trials": trials,
           "migratedFrom": {"schema": V1_SCHEMA, "sourceKind": v1["sourceKind"],
                            "mapping": "site<-factors.site; session<-factors.session|factors.day; runId<-factors.runId|trial id"}}
    return c.finish(doc)
