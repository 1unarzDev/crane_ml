"""crane-exp: JSON in / JSON out. Every command takes --role (default llm, the least privileged)."""
import argparse
import json
import os
import sys

from . import claims as claims_mod
from .errors import StudyError
from .fake_worker import make_worker
from .roles import require
from .study import Study


def _json_arg(value):
    if value is None:
        return None
    if value == "-":
        return json.load(sys.stdin)
    if os.path.exists(value):
        with open(value, encoding="utf-8") as fh:
            return json.load(fh)
    return json.loads(value)


def build_parser():
    p = argparse.ArgumentParser(prog="crane-exp")
    p.add_argument("--role", default=os.environ.get("CRANE_EXP_ROLE", "llm"), choices=["llm", "human"])
    sub = p.add_subparsers(dest="cmd", required=True)

    def study_cmd(name, **extra):
        sp = sub.add_parser(name)
        sp.add_argument("--study", required=True)
        for k, kw in extra.items():
            sp.add_argument("--" + k.replace("_", "-"), **kw)
        return sp

    c = study_cmd("create", spec=dict(required=True), optimizer=dict(default="default"),
                  feedback=dict(default="sanitized", choices=["sanitized", "raw"]),
                  worker=dict(default="fake"))
    c.set_defaults(fn=cmd_create)
    study_cmd("ask", kind=dict(default=None), split=dict(default="train"),
              fidelity=dict(default=None)).set_defaults(fn=cmd_ask)
    study_cmd("evaluate", trial=dict(required=True)).set_defaults(fn=cmd_evaluate)
    study_cmd("tell", trial=dict(required=True), result=dict(required=True)).set_defaults(fn=cmd_tell)
    study_cmd("freeze").set_defaults(fn=cmd_freeze)
    study_cmd("status").set_defaults(fn=cmd_status)
    study_cmd("report").set_defaults(fn=cmd_report)
    study_cmd("verify").set_defaults(fn=cmd_verify)
    study_cmd("edit-spec", spec=dict(required=True)).set_defaults(fn=cmd_edit_spec)
    cl = sub.add_parser("claims")
    cl.add_argument("action", choices=["check"])
    cl.add_argument("--study", required=True)
    cl.add_argument("--claims", required=True)
    cl.set_defaults(fn=cmd_claims)
    cp = sub.add_parser("compare")
    cp.add_argument("--replicates", type=int, default=5)
    cp.add_argument("--n-search", type=int, default=40)
    cp.add_argument("--out", default=None)
    cp.set_defaults(fn=cmd_compare)
    return p


def cmd_create(a):
    spec = _json_arg(a.spec)
    st = Study.create(a.study, spec, {"optimizer": a.optimizer, "feedback": a.feedback,
                                      "worker": a.worker}, role=a.role)
    return st.status()


def cmd_ask(a):
    return Study.open(a.study).ask(a.role, kind=a.kind, split=a.split, fidelity=a.fidelity)


def cmd_evaluate(a):
    st = Study.open(a.study)
    return st.evaluate(a.trial, make_worker(st.runtime["worker"]), a.role)


def cmd_tell(a):
    return Study.open(a.study).tell(a.role, a.trial, _json_arg(a.result))


def cmd_freeze(a):
    return Study.open(a.study).freeze(a.role)


def cmd_status(a):
    require(a.role, "status")
    return Study.open(a.study).status()


def cmd_report(a):
    return Study.open(a.study).report(a.role)


def cmd_verify(a):
    require(a.role, "verify")
    st = Study.open(a.study)
    return {"entries_verified": st.ledger.verify(), "head": st.ledger.head}


def cmd_edit_spec(a):
    st = Study.open(a.study)
    st.edit_spec(a.role, _json_arg(a.spec))
    return st.status()


def cmd_claims(a):
    require(a.role, "claims_check")
    st = Study.open(a.study)
    viol = claims_mod.check_claims(_json_arg(a.claims), st.evidence_map(), st.profile_states())
    return {"violations": viol, "ok": not viol}


def cmd_compare(a):
    require(a.role, "compare")
    from . import compare as cmp
    rep = cmp.compare(replicates=a.replicates, n_search=a.n_search)
    if a.out:
        cmp.save(rep, a.out)
    return rep


def main(argv=None):
    a = build_parser().parse_args(argv)
    try:
        res = a.fn(a)
    except StudyError as exc:
        print(json.dumps({"ok": False, "error": exc.code, "message": str(exc)}))
        return 2
    print(json.dumps({"ok": True, "result": res}, sort_keys=True, default=str))
    if a.cmd == "claims" and not res["ok"]:
        return 1
    return 0


if __name__ == "__main__":
    sys.exit(main())
