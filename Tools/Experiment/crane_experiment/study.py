"""Study: the single mutation point. Lifecycle is ask -> evaluate(worker) -> result.

`tell` is internal (called by evaluate); the public `tell` exists for humans reporting a
result they obtained elsewhere and is refused for the LLM role.
"""
import copy
import json
import math
import os

import numpy as np

from . import abstain, accounting
from .budget import Budget
from .claims import PROFILE_CEILING, RANK
from .errors import (BudgetExhausted, DuplicateTell, HeldOutLocked, LedgerCorrupt, SpecFrozen,
                     SpecTampered, StudyError, UnknownTrial)
from .ledger import Ledger, evidence_id
from .loophole import detect_loophole
from .optimizers import make_optimizer
from .roles import require
from .seeds import crn_seed, optimizer_seed
from .spec import CAL_ORDER, Space, spec_sha, validate_spec, with_defaults

DEFAULT_RUNTIME = {"optimizer": "default", "feedback": "sanitized", "worker": "fake"}


def _clean(o):
    if isinstance(o, float):
        return o if math.isfinite(o) else None
    if isinstance(o, dict):
        return {k: _clean(v) for k, v in o.items()}
    if isinstance(o, (list, tuple)):
        return [_clean(v) for v in o]
    if isinstance(o, np.generic):
        return _clean(o.item())
    return o


class Study:
    # ------------------------------------------------------------------ construction
    def __init__(self, path, spec_raw, runtime, ledger):
        self.path = path
        self._raw = copy.deepcopy(spec_raw)
        self._spec_sha = spec_sha(spec_raw)
        self._spec = with_defaults(spec_raw)
        self.runtime = {**DEFAULT_RUNTIME, **(runtime or {})}
        self.ledger = ledger
        self.space = Space(self._spec["space"])
        self.train = [c for c in self._spec["contexts"] if c["split"] == "train"]
        self.held = [c for c in self._spec["contexts"] if c["split"] == "heldout"]
        self.ctx = {c["id"]: c for c in self._spec["contexts"]}
        b = self._spec["budget"]
        self.budget = Budget(b["total_cost"], b.get("reserve_cost", 0.0))
        self.trials = {}
        self.frozen = None
        self.n_asks = 0
        self.n_search_by_ctx = {}
        self.n_baseline = self.n_confirm = self.n_heldout = 0
        self.opt = self._make_opt()

    def _make_opt(self):
        s = self._spec
        feats = len(self.train[0].get("features", []))
        return make_optimizer(
            self.runtime["optimizer"], dim=self.space.dim,
            seed=optimizer_seed(s["seed"], self.runtime["optimizer"]),
            x0=self.space.to_unit(s["default"]), ctx_dim=feats,
            fidelities={k: v["cost"] for k, v in s["fidelities"].items() if k != "real"},
            default_fidelity=s["default_fidelity"])

    @staticmethod
    def _spec_path(path):
        return os.path.join(path, "spec.json")

    @classmethod
    def create(cls, path, spec, runtime=None, role="human", fsync=True):
        require(role, "create")
        validate_spec(spec)
        os.makedirs(path, exist_ok=True)
        lp = os.path.join(path, "ledger.jsonl")
        if os.path.exists(lp):
            raise StudyError("study already exists")
        with open(cls._spec_path(path), "w", encoding="utf-8") as fh:
            json.dump(spec, fh, sort_keys=True, indent=1)
        led = Ledger(lp, fsync=fsync)
        rt = {**DEFAULT_RUNTIME, **(runtime or {})}
        led.append("create", {"spec": spec, "spec_sha": spec_sha(spec), "runtime": rt})
        st = cls(path, spec, rt, led)
        return st

    @classmethod
    def open(cls, path, fsync=True):
        led = Ledger(os.path.join(path, "ledger.jsonl"), fsync=fsync)
        if not led.entries or led.entries[0]["type"] != "create":
            raise LedgerCorrupt("ledger must start with a create event")
        first = led.entries[0]["data"]
        cur = first["spec"]
        for e in led.entries:
            if e["type"] == "spec_edit":
                cur = e["data"]["spec"]
        st = cls(path, cur, first["runtime"], led)
        st._check_disk()
        for e in led.entries[1:]:
            st._replay(e)
        return st

    # ------------------------------------------------------------------ spec integrity
    @property
    def spec(self):
        return copy.deepcopy(self._spec)

    @property
    def spec_sha(self):
        return self._spec_sha

    def _check_disk(self):
        try:
            with open(self._spec_path(self.path), encoding="utf-8") as fh:
                disk = json.load(fh)
        except (OSError, ValueError) as exc:
            raise SpecTampered(f"spec.json unreadable: {exc}")
        if spec_sha(disk) != self._spec_sha:
            raise SpecTampered("spec.json no longer matches the spec hash recorded in the ledger")

    def edit_spec(self, role, new_spec):
        require(role, "edit_spec")
        self._check_disk()
        if self.n_asks > 0:
            raise SpecFrozen("the spec is frozen after the first ask")
        validate_spec(new_spec)
        with open(self._spec_path(self.path), "w", encoding="utf-8") as fh:
            json.dump(new_spec, fh, sort_keys=True, indent=1)
        self.ledger.append("spec_edit", {"spec": new_spec, "spec_sha": spec_sha(new_spec),
                                         "prev_spec_sha": self._spec_sha})
        self.__init__(self.path, new_spec, self.runtime, self.ledger)

    # ------------------------------------------------------------------ helpers
    def _feat(self, ctx_id):
        f = self.ctx[ctx_id].get("features")
        return None if f is None else np.asarray(f, float)

    def _profile_state(self, profile):
        return self._spec["profiles"].get(profile, {}).get("calibrationState", "FIXTURE_ONLY")

    def _pairs(self):
        return [(c, k) for c in self.train for k in range(self._spec["baseline_replicates"])]

    def _record(self, etype, data):
        return self.ledger.append(etype, data)

    def _search_ask(self, ctx_id):
        x, fid = self.opt.ask(self._feat(ctx_id))
        return np.clip(np.asarray(x, float), 0, 1), fid

    # ------------------------------------------------------------------ ask
    def ask(self, role, kind=None, split="train", fidelity=None):
        require(role, "ask_heldout" if split == "heldout" else "ask_train")
        if fidelity == "real":
            require(role, "ask_real")
        self._check_disk()
        s = self._spec
        if split == "heldout":
            if self.frozen is None:
                raise HeldOutLocked("held-out contexts are locked until freeze()")
            plan = [(c, k, kd) for c in self.held for k in range(s["baseline_replicates"])
                    for kd in ("heldout", "heldout_default")]
            if self.n_heldout >= len(plan):
                raise StudyError("held-out plan exhausted")
            c, k, kind = plan[self.n_heldout]
            if kind == "heldout":
                x = self.space.to_unit(self.frozen["recommendations"][c["id"]])
            else:
                x = self.space.to_unit(s["default"])
            fid = fidelity or s["default_fidelity"]
        else:
            if self.frozen is not None:
                raise StudyError("study is frozen; only held-out asks remain")
            pairs = self._pairs()
            if kind is None:
                kind = "baseline" if self.n_baseline < len(pairs) else "search"
            if kind == "baseline":
                if self.n_baseline >= len(pairs):
                    raise StudyError("baseline already complete")
                c, k = pairs[self.n_baseline]
                x, fid = self.space.to_unit(s["default"]), fidelity or s["default_fidelity"]
            elif kind == "confirm":
                if self.n_baseline < len(pairs) or self.n_confirm >= len(pairs):
                    raise StudyError("confirm requires a complete baseline and is limited to one pass")
                c, k = pairs[self.n_confirm]
                x = self.space.to_unit(self.recommend_config(c["id"]))
                fid = fidelity or s["default_fidelity"]
            elif kind == "search":
                c = self.train[sum(self.n_search_by_ctx.values()) % len(self.train)]
                k = self.n_search_by_ctx.get(c["id"], 0) % s["crn_replicates"]
                x = fid = None   # chosen by the optimizer after the budget check
            else:
                raise StudyError(f"unknown kind {kind!r}")
        search = kind == "search"
        tid = f"t{len(self.trials):04d}"
        costs = {k: v["cost"] for k, v in s["fidelities"].items()}
        cheapest = min((k for k in costs if k != "real"), key=costs.get)
        if search:
            # budget is checked BEFORE the optimizer is consulted, so a refused ask has no side effects
            floor = costs[cheapest] if self.runtime["optimizer"] == "mfbo" else costs[s["default_fidelity"]]
            if floor > self.budget.remaining(True) + 1e-12:
                raise BudgetExhausted(f"remaining search budget {self.budget.remaining(True):.3f} < {floor}")
            x, fid = self._search_ask(c["id"])
            if fidelity:
                fid = fidelity
            if costs[fid] > self.budget.remaining(True) + 1e-12:
                fid = cheapest      # optimizer wanted a costlier fidelity than what is left
        est = costs[fid]
        if est > self.budget.remaining(search) + 1e-12:
            raise BudgetExhausted(f"remaining budget {self.budget.remaining(search):.3f} < {est}")
        if fid == "real":
            require(role, "ask_real")
        data = {"trial_id": tid, "kind": kind, "split": split, "context_id": c["id"], "k": k,
                "fidelity": fid, "seed": crn_seed(s["seed"], c["id"], k),
                "x": [float(v) for v in x], "config": self.space.from_unit(x),
                "est_cost": est, "role": role}
        self._record("ask", data)
        self._apply_ask(data)
        return {"trial_id": tid, "kind": kind, "split": split, "fidelity": fid,
                "context": {key: c[key] for key in ("id", "site", "session", "profile", "features") if key in c},
                "config": data["config"], "est_cost": est, "seed": data["seed"]}

    def _apply_ask(self, d):
        self.n_asks += 1
        kind = d["kind"]
        if kind == "search":
            self.n_search_by_ctx[d["context_id"]] = self.n_search_by_ctx.get(d["context_id"], 0) + 1
        elif kind == "baseline":
            self.n_baseline += 1
        elif kind == "confirm":
            self.n_confirm += 1
        else:
            self.n_heldout += 1
        self.budget.reserve_trial(d["trial_id"], d["est_cost"], search=(kind == "search"))
        self.trials[d["trial_id"]] = {**d, "status": "pending", "cost": 0.0, "reward": None,
                                      "violation": False, "flags": []}

    # ------------------------------------------------------------------ evaluate / tell
    def evaluate(self, trial_id, worker, role):
        require(role, "evaluate")
        t = self._trial(trial_id)
        if t["fidelity"] == "real":
            require(role, "ask_real")
        self._check_disk()
        c = self.ctx[t["context_id"]]
        if t["status"] != "pending":
            raise DuplicateTell(f"trial {trial_id} already has a result")
        result = worker.run(t["config"], c, t["fidelity"], t["seed"], self._spec)
        return self._tell(t, result, "worker", role, getattr(worker, "label_ceiling", "IMPLEMENTED"))

    def tell(self, role, trial_id, result):
        """Human-only: report a result obtained outside the worker loop."""
        require(role, "tell")
        t = self._trial(trial_id)
        self._check_disk()
        if t["status"] != "pending":
            raise DuplicateTell(f"trial {trial_id} already has a result")
        return self._tell(t, result, "external", role, "IMPLEMENTED")

    def _trial(self, trial_id):
        if trial_id not in self.trials:
            raise UnknownTrial(f"unknown trial {trial_id}")
        return self.trials[trial_id]

    def _classify(self, result, ctx):
        r = result.get("reward")
        if result.get("error") or not isinstance(r, (int, float)) or not math.isfinite(r):
            return "invalid", [str(result.get("error") or "NO_REWARD")]
        age = result.get("obs_age_s", 0.0)
        if result.get("profile_sha") != ctx["profile"] or (age is not None and age > self._spec["max_obs_age_s"]):
            return "stale", ["PROFILE_OR_OBSERVATION_STALE"]
        flags = detect_loophole(result)
        if flags:
            return "loophole", flags
        return "valid", []

    def _tell(self, t, result, source, role, worker_ceiling):
        s, c = self._spec, self.ctx[t["context_id"]]
        result = _clean(dict(result))
        status, flags = self._classify(result, c)
        cost = result.get("cost")
        if not isinstance(cost, (int, float)) or not math.isfinite(cost) or cost < 0:
            cost, flags = t["est_cost"], flags + ["COST_MISSING_CHARGED_ESTIMATE"]
        viol = bool(result.get("violation", False))
        r = result.get("reward")
        trusted = status == "valid" or (self.runtime["feedback"] == "raw" and status in ("stale", "loophole"))
        y = float(r) if (trusted and r is not None) else None   # failures never become objective values
        margin = result.get("margin") if y is not None else None
        if y is not None and margin is None:
            margin = -1.0 if viol else 1.0
        feasible = y is not None and not viol and (margin is None or margin >= 0)
        ceiling = PROFILE_CEILING[self._profile_state(c["profile"])]
        ceiling = min(ceiling, worker_ceiling if worker_ceiling in RANK else "IMPLEMENTED", key=lambda l: RANK[l])
        data = {"trial_id": t["trial_id"], "source": source, "role": role, "status": status,
                "flags": flags, "reward": r, "y_feedback": y, "feasible": feasible, "margin": margin, "cost": float(cost),
                "violation": viol, "label_ceiling": ceiling, "result": result}
        e = self._record("tell", data)
        self._apply_tell(data)
        return {"trial_id": t["trial_id"], "status": status, "flags": flags, "cost": float(cost),
                "evidence_id": evidence_id(e["hash"])}

    def _apply_tell(self, d):
        t = self.trials[d["trial_id"]]
        self.budget.charge(t["trial_id"], d["cost"])   # every status is charged
        t.update(status=d["status"], flags=d["flags"], reward=d["reward"], cost=d["cost"],
                 violation=d["violation"], y_feedback=d["y_feedback"], feasible=d["feasible"], label_ceiling=d["label_ceiling"])
        if t["kind"] == "search":
            self.opt.tell(np.array(t["x"]), self._feat(t["context_id"]), t["fidelity"], d["y_feedback"],
                         d["feasible"], d["margin"])

    # ------------------------------------------------------------------ freeze
    def freeze(self, role):
        require(role, "freeze")
        if self.frozen is not None:
            raise StudyError("already frozen")
        self._check_disk()
        recs = {c["id"]: self.recommend_config(c["id"]) for c in self.held}
        n_plan = 2 * len(self.held) * self._spec["baseline_replicates"]
        allowance = 1.25 * n_plan * self._spec["fidelities"][self._spec["default_fidelity"]]["cost"]
        data = {"recommendations": recs, "train_head": self.ledger.head, "heldout_allowance": allowance}
        self._record("freeze", data)
        self._apply_freeze(data)
        return data

    def _apply_freeze(self, d):
        self.frozen = d
        self.budget.total += d["heldout_allowance"]   # held-out evaluation is budgeted separately

    # ------------------------------------------------------------------ replay
    def _replay(self, e):
        t, d = e["type"], e["data"]
        if t == "ask":
            if d["kind"] == "search":
                x, fid = self._search_ask(d["context_id"])
                if not np.allclose(x, d["x"], atol=1e-9):
                    raise LedgerCorrupt(f"replay divergence at {d['trial_id']}: optimizer is not deterministic")
            self._apply_ask(d)
        elif t == "tell":
            self._apply_tell(d)
        elif t == "freeze":
            self._apply_freeze(d)
        elif t == "spec_edit":
            raise LedgerCorrupt("spec_edit after create is only valid before the first ask")

    # ------------------------------------------------------------------ recommendation
    def _valid_search(self):
        return [t for t in self.trials.values() if t["kind"] == "search" and t["status"] == "valid"
                and t["fidelity"] == self._spec["default_fidelity"]]

    def contaminated(self):
        return (self.runtime["feedback"] == "raw"
                and any(t["status"] in ("loophole", "stale", "invalid") for t in self.trials.values()))

    def recommend_config(self, ctx_id=None):
        if self.contaminated():
            v = self._valid_search()
            if v:
                return dict(max(v, key=lambda t: t["reward"])["config"])
            return dict(self._spec["default"])
        if ctx_id is None:
            f = np.mean([c["features"] for c in self.train], axis=0) if self.train[0].get("features") else None
        else:
            f = self._feat(ctx_id)
        return self.space.from_unit(self.opt.recommend(f))

    # ------------------------------------------------------------------ evidence map & report
    def evidence_map(self):
        m = {}
        for e in self.ledger.entries:
            m[evidence_id(e["hash"])] = e["data"].get("label_ceiling", "IMPLEMENTED") if e["type"] == "tell" else "IMPLEMENTED"
        return m

    def profile_states(self):
        return {p: v.get("calibrationState") for p, v in self._spec["profiles"].items()}

    def status(self):
        return {"study_id": self._spec["study_id"], "spec_sha": self._spec_sha,
                "ledger_head": self.ledger.head, "ledger_entries": len(self.ledger.entries),
                "n_asks": self.n_asks, "frozen": self.frozen is not None,
                "budget": {"total": self.budget.total, "spent": self.budget.spent,
                           "reserved": self.budget.reserved, "reserve_for_confirm": self.budget.reserve},
                "pending": [k for k, t in self.trials.items() if t["status"] == "pending"],
                "runtime": self.runtime}

    def _paired(self, kind_a, kind_b):
        by = {}
        for t in self.trials.values():
            if t["kind"] in (kind_a, kind_b) and t["status"] != "pending":
                by.setdefault((t["context_id"], t["k"]), {})[t["kind"]] = t
        d, stale = [], 0
        for p in by.values():
            a, b = p.get(kind_a), p.get(kind_b)
            if a and a["status"] == "stale":
                stale += 1
            if a and b and a["status"] == "valid" and b["status"] == "valid":
                d.append(a["reward"] - b["reward"])
        return d, stale

    def report(self, role="human"):
        require(role, "report")
        s = self._spec
        trials = list(self.trials.values())
        acc = accounting.account(trials, self.budget)
        search = [t for t in trials if t["kind"] == "search" and t["status"] != "pending"]
        d, stale = self._paired("confirm", "baseline")
        confirm = [t for t in trials if t["kind"] == "confirm" and t["status"] != "pending"]
        nviol = sum(1 for t in confirm if t["violation"] or t["status"] != "valid")
        risk = (nviol + 0.5) / (len(confirm) + 1) if confirm else 1.0
        req = s["claim_scope"]["requiredCalibrationState"]
        states = [self._profile_state(c["profile"]) for c in self.train]
        calib_ok = all(CAL_ORDER.index(st) >= CAL_ORDER.index(req) for st in states)
        loop = [t for t in trials if t["status"] == "loophole"]
        gates = {
            "loophole_count": len(loop), "n_trials": len(search),
            "n_valid": sum(1 for t in search if t["status"] == "valid"),
            "valid_fraction": (sum(1 for t in search if t["status"] == "valid") / len(search)) if search else 0.0,
            "min_valid_fraction": s["min_valid_fraction"], "has_candidate": bool(self._valid_search()),
            "profile_mismatch": any(t["status"] == "stale" for t in confirm),
            "calibration_ok": calib_ok, "constraint_risk": risk, "alpha": s["alpha"],
            "improvement_lcb": abstain.lcb_mean(d),
        }
        action, reason = abstain.decide(gates)
        ceilings = [t["label_ceiling"] for t in trials if t["status"] != "pending"]
        label = min(ceilings, key=lambda l: RANK[l]) if ceilings else "NOT_RUN"
        rec = self.recommend_config(None)
        rep = {
            "schema": "crane.report/v1", "study_id": s["study_id"], "spec_sha": self._spec_sha,
            "ledger_head": self.ledger.head, "evidence_id": evidence_id(self.ledger.head),
            "label_ceiling": label,
            "decision": {"action": action, "reason": reason},
            "recommended_config": rec,
            "recommended_source": "best_valid_trial" if self.contaminated() else "optimizer",
            "recommendation_endorsed": action == "RECOMMEND",
            "gates": {k: (None if isinstance(v, float) and not math.isfinite(v) else v) for k, v in gates.items()},
            "improvement_over_default": {"n_pairs": len(d), "mean": float(np.mean(d)) if d else None,
                                         "lcb95": gates["improvement_lcb"] if d and len(d) > 1 else None},
            "accounting": acc,
            "loophole": {"flagged_trials": [t["trial_id"] for t in loop],
                         "flags": sorted({f for t in loop for f in t["flags"]})},
            "n_stale": acc["by_status"]["stale"], "n_invalid": acc["by_status"]["invalid"],
        }
        if self.frozen is not None:
            hd, _ = self._paired("heldout", "heldout_default")
            rep["heldout"] = {"n_pairs": len(hd), "mean_improvement": float(np.mean(hd)) if hd else None,
                              "lcb95": abstain.lcb_mean(hd) if len(hd) > 1 else None}
        if label in ("FIXTURE_VALIDATED", "IMPLEMENTED", "NOT_RUN") or self.runtime["worker"] == "fake":
            rep["note"] = "FIXTURE_ONLY: not evidence about CRANE"
        return rep
